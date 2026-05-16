// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// lwo_estimator.cpp — Orchestrator for LiDAR-Wheel Odometry (no IMU).

#include "tof_slam/frontend_w/estimator/lwo_estimator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <unistd.h>

#include <Eigen/Eigenvalues>
#include <spdlog/spdlog.h>

#include "tof_slam/common/lie/so3.hpp"
#include "tof_slam/frontend/estimator/correspondence_finder.hpp"
#include "tof_slam/frontend/filter/range_filter.hpp"
#include "tof_slam/frontend/filter/stride_filter.hpp"
#include "tof_slam/frontend/filter/voxel_grid.hpp"

namespace tof_slam {
namespace lwo {

namespace {

constexpr int kRejectReasonNone = 0;
constexpr int kRejectReasonObjective = 1;
constexpr int kRejectReasonWheel = 2;
constexpr int kRejectReasonGround = 3;
constexpr int kRejectReasonPoseDelta = 4;
constexpr int kRejectReasonYawDelta = 5;
constexpr int kRejectReasonTrust = 6;

constexpr int kAcceptReasonNone = 0;
constexpr int kAcceptReasonBlend = 1;
constexpr int kAcceptReasonWindow = 2;
constexpr int kAcceptReasonBlendDisabled = 3;

constexpr int kCommitReasonNone = 0;
constexpr int kCommitReasonAccepted = 1;
constexpr int kCommitReasonAcceptRequired = 2;
constexpr int kCommitReasonRmsDrop = 3;
constexpr int kCommitReasonTrust = 4;
constexpr int kCommitReasonLidarRms = 5;
constexpr int kCommitReasonPoseDelta = 6;
constexpr int kCommitReasonYawDelta = 7;

}  // namespace

// ---------------------------------------------------------------------------
// Construction helpers
// ---------------------------------------------------------------------------

/// Build an anchor-map SurfelMapConfig: same as main map but with temporal
/// pruning disabled so surfels never expire.
static core::SurfelMapConfig make_anchor_map_config(
    const core::SurfelMapConfig& base) {
  core::SurfelMapConfig cfg = base;
  // Note: v1.0 SurfelMap does not have max_l0_age_frames.
  // Anchor map is frozen by simply not calling update() after build phase.
  return cfg;
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

LwoEstimator::LwoEstimator()
    : LwoEstimator(Config{}) {}

LwoEstimator::LwoEstimator(const Config& config)
    : config_(config),
      T_body_lidar_current_(config.T_body_lidar),
      wheel_propagator_(config.wheel_propagator),
      ground_constraint_(config.ground_constraint),
      wheel_measurement_(config.wheel_measurement),
      iekf_updater_(config.iekf),
      surfel_map_(config.surfel_map),
      pko_(config.pko),
      anchor_map_(make_anchor_map_config(config.surfel_map)) {
  // Propagate wheel measurement switch to IEKF config.
  iekf_updater_.mutable_cfg().enable_wheel_measurement =
      config.enable_wheel_measurement;
}

// ---------------------------------------------------------------------------
// feed_wheel
// ---------------------------------------------------------------------------

void LwoEstimator::feed_wheel(float vx, float omega_z, double timestamp) {
  std::lock_guard<std::mutex> lock(mutex_);

  // First wheel message: immediate initialization (no gravity init needed).
  if (!initialized_) {
    state_.reset();
    initialized_ = true;
    first_lidar_frame_ = true;
    frame_count_ = 0;
    last_wheel_time_ = timestamp;
    last_vx_enc_ = vx;
    last_omega_z_enc_ = omega_z;
    SPDLOG_INFO("LWO initialized from first wheel message at t={:.4f}", timestamp);
    return;
  }

  // Compute dt.
  double dt_d = timestamp - last_wheel_time_;

  if (dt_d <= 0.0) {
    // Backward time: skip entirely.
    return;
  }
  if (dt_d > 1.0) {
    // Gap too large to propagate reliably.  Reset the time reference so the
    // NEXT event can compute a valid dt, preventing permanent deadlock.
    spdlog::warn("[LWO] feed_wheel: dt={:.3f}s gap detected (ts={:.4f}), "
                 "resetting time reference", dt_d, timestamp);
    last_wheel_time_ = timestamp;
    last_vx_enc_ = vx;
    last_omega_z_enc_ = omega_z;
    return;
  }
  last_wheel_time_ = timestamp;
  last_vx_enc_ = vx;
  last_omega_z_enc_ = omega_z;
  float dt = static_cast<float>(dt_d);

  // Propagate state forward using wheel odometry.
  state_ = wheel_propagator_.propagate(state_, vx, omega_z, dt);

  // Safety: detect velocity explosion (should never happen with clamped inputs)
  const float vel_norm = state_.velocity.norm();
  if (vel_norm > 5.0f) {
    spdlog::warn("[LWO] vel_explosion: vel_mag={:.3f} vx_in={:.4f} omega_in={:.5f} "
                 "dt={:.4f} scale={:.4f} bias=[{:.5f},{:.5f}]",
                 static_cast<double>(vel_norm),
                 static_cast<double>(vx),
                 static_cast<double>(omega_z),
                 static_cast<double>(dt),
                 static_cast<double>(state_.wheel_scale),
                 static_cast<double>(state_.wheel_gyro_bias(0)),
                 static_cast<double>(state_.wheel_gyro_bias(1)));
    // Clamp velocity to prevent runaway
    state_.velocity = state_.velocity.normalized() * 3.0f;
  }

  // Debug: wheel propagation (throttled: every 1000th message)
  if (config_.enable_debug_log && (++wheel_debug_counter_ % 1000 == 0)) {
    spdlog::info("[LWO-DBG] wheel: dt={:.4f} vx={:.4f} omega={:.5f} "
                 "pos=[{:.3f},{:.3f},{:.3f}] vel=[{:.4f},{:.4f},{:.4f}]",
                 static_cast<double>(dt),
                 static_cast<double>(vx),
                 static_cast<double>(omega_z),
                 static_cast<double>(state_.position.x()),
                 static_cast<double>(state_.position.y()),
                 static_cast<double>(state_.position.z()),
                 static_cast<double>(state_.velocity.x()),
                 static_cast<double>(state_.velocity.y()),
                 static_cast<double>(state_.velocity.z()));
  }

  // Store state snapshot for undistortion.
  core::StampedPose sp;
  sp.timestamp = timestamp;
  sp.pose = state_.pose();
  state_history_.push_back(sp);

  // Bound history size.
  while (state_history_.size() > 2000) {
    state_history_.pop_front();
  }
}

// ---------------------------------------------------------------------------
// feed_wheel_delta
// ---------------------------------------------------------------------------

void LwoEstimator::feed_wheel_delta(
    float vx, float omega_z, double timestamp,
    const Eigen::Vector3f& delta_pos_body, float delta_yaw) {
  std::lock_guard<std::mutex> lock(mutex_);

  const auto t_wheel_start = config_.check_usage
      ? std::chrono::steady_clock::now()
      : std::chrono::steady_clock::time_point{};

  // First wheel message: initialize identical to feed_wheel.
  if (!initialized_) {
    state_.reset();
    initialized_ = true;
    first_lidar_frame_ = true;
    frame_count_ = 0;
    last_wheel_time_ = timestamp;
    last_vx_enc_ = vx;
    last_omega_z_enc_ = omega_z;
    SPDLOG_INFO("LWO initialized from first wheel_delta message at t={:.4f}", timestamp);
    return;
  }

  // Compute dt (used for covariance propagation only; not for re-integration).
  double dt_d = timestamp - last_wheel_time_;

  if (dt_d <= 0.0) {
    return;
  }
  if (dt_d > 1.0) {
    spdlog::warn("[LWO] feed_wheel_delta: dt={:.3f}s gap (ts={:.4f}), "
                 "resetting time reference", dt_d, timestamp);
    last_wheel_time_ = timestamp;
    last_vx_enc_ = vx;
    last_omega_z_enc_ = omega_z;
    return;
  }
  last_wheel_time_ = timestamp;
  last_vx_enc_ = vx;
  last_omega_z_enc_ = omega_z;
  float dt = static_cast<float>(dt_d);

  // --- Apply delta pose directly to state (no re-integration) ---
  //
  // delta_pos_body is in the *previous* body frame (computed in wheel_callback
  // as R_prev^T * (pos_world_curr - pos_world_prev)).  Apply it via the
  // rotation that was current at the start of this interval (state_.rotation
  // before yaw update), then update rotation.

  // Save rotation before yaw update for position application.
  const Eigen::Matrix3f R_old = state_.rotation;

  // 1. Apply yaw rotation: R_new = R_old * Exp(delta_yaw * z)
  const Eigen::Matrix3f dR =
      Eigen::AngleAxisf(delta_yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  state_.rotation = R_old * dR;

  // 2. Apply position delta: p_new = p_old + R_old * delta_pos_body
  //    Using R_old (not R_new) keeps consistency with the body frame in which
  //    delta_pos_body was expressed.
  state_.position += R_old * delta_pos_body;

  // 3. Update velocity from twist (body-frame forward velocity → world frame).
  //    This keeps the IEKF wheel measurement H-matrix consistent.
  state_.velocity = state_.rotation *
                    Eigen::Vector3f(state_.wheel_scale * vx, 0.0f, 0.0f);

  // Safety: detect velocity explosion.
  const float vel_norm = state_.velocity.norm();
  if (vel_norm > 5.0f) {
    spdlog::warn("[LWO] feed_wheel_delta vel_explosion: vel_mag={:.3f} "
                 "vx_in={:.4f} omega_in={:.5f} dt={:.4f} scale={:.4f}",
                 static_cast<double>(vel_norm),
                 static_cast<double>(vx),
                 static_cast<double>(omega_z),
                 static_cast<double>(dt),
                 static_cast<double>(state_.wheel_scale));
    state_.velocity = state_.velocity.normalized() * 3.0f;
  }

  // --- Covariance propagation (same F/Q as feed_wheel) ---
  //
  // We intentionally reuse the twist-based F and Q matrices.  The delta pose
  // reduces position/heading bias, but the process noise model is unchanged.
  const LwoStateCovariance F =
      wheel_propagator_.build_transition_matrix(state_, vx, omega_z, dt);
  const LwoStateCovariance Q = wheel_propagator_.process_noise();
  // When ext calibration is disabled, use 12x12 block propagation to
  // preserve exact numerical backward compatibility.
  if (!config_.iekf.enable_ext_calibration) {
    constexpr int kBaseDim = 12;
    const Eigen::Matrix<float, kBaseDim, kBaseDim> F12 =
        F.topLeftCorner<kBaseDim, kBaseDim>();
    const Eigen::Matrix<float, kBaseDim, kBaseDim> P12 =
        state_.covariance.topLeftCorner<kBaseDim, kBaseDim>();
    const Eigen::Matrix<float, kBaseDim, kBaseDim> Q12 =
        Q.topLeftCorner<kBaseDim, kBaseDim>();
    state_.covariance.topLeftCorner<kBaseDim, kBaseDim>() =
        F12 * P12 * F12.transpose() + Q12 * dt;
  } else {
    state_.covariance = F * state_.covariance * F.transpose() + Q * dt;
  }

  if (config_.enable_debug_log && (++wheel_debug_counter_ % 1000 == 0)) {
    spdlog::info("[LWO-DBG] wheel_delta: dt={:.4f} dp_b=[{:.4f},{:.4f},{:.4f}] "
                 "dyaw={:.5f} pos=[{:.3f},{:.3f},{:.3f}]",
                 static_cast<double>(dt),
                 static_cast<double>(delta_pos_body.x()),
                 static_cast<double>(delta_pos_body.y()),
                 static_cast<double>(delta_pos_body.z()),
                 static_cast<double>(delta_yaw),
                 static_cast<double>(state_.position.x()),
                 static_cast<double>(state_.position.y()),
                 static_cast<double>(state_.position.z()));
  }

  // Store state snapshot for undistortion.
  core::StampedPose sp;
  sp.timestamp = timestamp;
  sp.pose = state_.pose();
  state_history_.push_back(sp);

  // Bound history size.
  while (state_history_.size() > 2000) {
    state_history_.pop_front();
  }

  if (config_.check_usage) {
    const float ms = std::chrono::duration<float, std::milli>(
        std::chrono::steady_clock::now() - t_wheel_start).count();
    wheel_ms_accumulator_ += ms;
  }
}

// ---------------------------------------------------------------------------
// Preprocessing helpers
// ---------------------------------------------------------------------------

core::PointCloud LwoEstimator::preprocess_scan(const core::PointCloud& scan,
                                                 double timestamp) {
  // 1. Stride filter
  core::PointCloud after_stride = core::stride_filter(scan, config_.stride);

  // 2. Voxel grid filter
  core::VoxelGridConfig vg_cfg;
  vg_cfg.leaf_size = config_.voxel_leaf_size;
  vg_cfg.enable_planarity_filter = false;
  core::PointCloud after_voxel = core::voxel_grid_filter(after_stride, vg_cfg);

  // 3. Undistortion (optional)
  core::PointCloud after_undistort = after_voxel;
  bool undist_applied = false;
  if (config_.enable_undistortion && state_history_.size() >= 2) {
    std::vector<core::StampedPose> traj(state_history_.begin(),
                                         state_history_.end());
    after_undistort =
        core::undistort_scan(after_voxel, traj, timestamp, T_body_lidar_current_);
    undist_applied = true;
  }

  // 4. Range filter
  core::PointCloud after_range =
      core::range_filter(after_undistort, config_.min_range, config_.max_range);

  // Debug: preprocessing stats (A)
  if (config_.enable_debug_log) {
    spdlog::info("[LWO-DBG] frame={} preproc: raw={} stride={} voxel={} "
                 "undist={} range=[{:.1f},{:.1f}] pts_final={}",
                 frame_count_,
                 scan.size(),
                 after_stride.size(),
                 after_voxel.size(),
                 undist_applied ? after_undistort.size() : after_voxel.size(),
                 static_cast<double>(config_.min_range),
                 static_cast<double>(config_.max_range),
                 after_range.size());
  }

  return after_range;
}

core::PointCloud LwoEstimator::transform_to_world(
    const core::PointCloud& scan) const {
  // T_world_lidar = T_world_body * T_body_lidar
  core::Se3 T_world_lidar = state_.pose() * T_body_lidar_current_;
  return scan.transformed_copy(T_world_lidar);
}

float LwoEstimator::compute_lidar_rms(
    const LwoState& state,
    const std::vector<core::Correspondence>& corrs) const {
  if (corrs.empty()) return 0.0f;

  const core::Se3 T_wl = state.pose() * T_body_lidar_current_;
  float sq_sum = 0.0f;
  int n = 0;
  for (const auto& c : corrs) {
    const float r = c.normal.dot(T_wl * c.p_lidar) - c.plane_d;
    sq_sum += r * r;
    ++n;
  }
  return n > 0 ? std::sqrt(sq_sum / static_cast<float>(n)) : 0.0f;
}

float LwoEstimator::compute_wheel_cost(const LwoState& state,
                                       float vx_enc,
                                       float omega_z_enc,
                                       float omega_z_b) const {
  const WheelMeasurementResult wv =
      wheel_measurement_.compute(state, vx_enc, omega_z_enc, omega_z_b);
  const float inv_v = 1.0f / std::max(wv.noise_cov(0, 0), 1e-6f);
  const float inv_w = 1.0f / std::max(wv.noise_cov(1, 1), 1e-6f);
  return wv.residual(0) * wv.residual(0) * inv_v +
         wv.residual(1) * wv.residual(1) * inv_w;
}

float LwoEstimator::compute_ground_cost(const LwoState& state) const {
  const GroundConstraintResult gc = ground_constraint_.compute(state);
  float cost = 0.0f;
  for (int i = 0; i < 3; ++i) {
    cost += gc.residual(i) * gc.residual(i) /
            std::max(gc.noise_cov(i, i), 1e-6f);
  }
  return cost;
}

float LwoEstimator::compute_prior_cost(const LwoState& x_wheel,
                                       const LwoState& x_other,
                                       float* pose_delta_norm,
                                       float* yaw_delta_abs) const {
  const Eigen::Vector3f dp_world = x_other.position - x_wheel.position;
  const Eigen::Vector3f dp_body = x_wheel.rotation.transpose() * dp_world;
  const Eigen::Matrix3f dR = x_wheel.rotation.transpose() * x_other.rotation;
  const Eigen::Vector3f dphi = core::So3(dR).Log();
  const float yaw = std::abs(dphi.z());

  if (pose_delta_norm) *pose_delta_norm = dp_world.norm();
  if (yaw_delta_abs) *yaw_delta_abs = yaw;

  constexpr float kForwardWeight = 1.0f;
  constexpr float kLateralWeight = 4.0f;
  constexpr float kVerticalWeight = 2.0f;
  constexpr float kRollPitchWeight = 2.0f;
  constexpr float kYawWeight = 6.0f;

  return kForwardWeight * dp_body.x() * dp_body.x() +
         kLateralWeight * dp_body.y() * dp_body.y() +
         kVerticalWeight * dp_body.z() * dp_body.z() +
         kRollPitchWeight * (dphi.x() * dphi.x() + dphi.y() * dphi.y()) +
         kYawWeight * yaw * yaw;
}

void LwoEstimator::compute_observability_metrics(
    const LwoState& state,
    const std::vector<core::Correspondence>& corrs,
    float* obs_min_eig,
    float* obs_cond_ratio) const {
  if (obs_min_eig) *obs_min_eig = 0.0f;
  if (obs_cond_ratio) *obs_cond_ratio = 0.0f;
  if (corrs.empty()) return;

  Eigen::Matrix<float, 6, 6> info =
      Eigen::Matrix<float, 6, 6>::Zero();
  const Eigen::Matrix3f R_il = T_body_lidar_current_.rotation().matrix();
  const Eigen::Vector3f t_il = T_body_lidar_current_.translation();
  const Eigen::Matrix3f R_wb = state.rotation;

  for (const auto& c : corrs) {
    const Eigen::Vector3f p_body = R_il * c.p_lidar + t_il;
    const Eigen::Vector3f C = R_wb.transpose() * c.normal;
    Eigen::Matrix<float, 1, 6> h = Eigen::Matrix<float, 1, 6>::Zero();
    h.block<1, 3>(0, 0) = -(core::Hat(p_body) * C).transpose();
    h.block<1, 3>(0, 3) = -c.normal.transpose();

    float sigma = config_.iekf.lidar_noise_std;
    if (c.noise_override > 0.0f) sigma = c.noise_override;
    sigma *= (1.0f + config_.iekf.planarity_noise_scale * c.planarity);
    const float w = 1.0f / std::max(sigma * sigma, 1e-4f);
    info.noalias() += w * h.transpose() * h;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 6, 6>> solver(info);
  if (solver.info() != Eigen::Success) return;
  const auto& eig = solver.eigenvalues();
  const float min_eig = std::max(eig(0), 0.0f);
  const float max_eig = std::max(eig(5), 1e-6f);
  if (obs_min_eig) *obs_min_eig = min_eig;
  if (obs_cond_ratio) *obs_cond_ratio = min_eig / max_eig;
}

LwoEstimator::LwoTrustMetrics LwoEstimator::evaluate_trust_metrics(
    const LwoState& x_wheel,
    const LwoState& x_lidar,
    const std::vector<core::Correspondence>& corrs,
    float vx_enc,
    float omega_z_enc,
    float omega_z_b) const {
  LwoTrustMetrics m;
  m.corr_count = static_cast<int>(corrs.size());
  m.lidar_rms_prior = compute_lidar_rms(x_wheel, corrs);
  m.lidar_rms_post = compute_lidar_rms(x_lidar, corrs);
  m.lidar_rms_ratio = (m.lidar_rms_prior > 1e-6f)
      ? (m.lidar_rms_post / m.lidar_rms_prior)
      : 1.0f;
  m.wheel_cost_prior = compute_wheel_cost(x_wheel, vx_enc, omega_z_enc, omega_z_b);
  m.wheel_cost_post = compute_wheel_cost(x_lidar, vx_enc, omega_z_enc, omega_z_b);
  m.ground_cost_prior = compute_ground_cost(x_wheel);
  m.ground_cost_post = compute_ground_cost(x_lidar);
  m.prior_cost_post = compute_prior_cost(
      x_wheel, x_lidar, &m.pose_delta_norm, &m.yaw_delta_abs);
  compute_observability_metrics(
      x_wheel, corrs, &m.obs_min_eig, &m.obs_cond_ratio);

  const float s_corr = std::clamp(
      static_cast<float>(m.corr_count) /
      std::max(static_cast<float>(config_.min_correspondences), 1.0f),
      0.0f, 1.0f);
  const float s_rms = std::clamp(
      (m.lidar_rms_prior - m.lidar_rms_post) /
      std::max(m.lidar_rms_prior, 1e-3f),
      0.0f, 1.0f);
  const float wheel_ref =
      std::max(m.wheel_cost_prior, config_.trust_wheel_cost_floor);
  const float s_wheel = (m.wheel_cost_post <= wheel_ref)
      ? 1.0f
      : std::clamp(wheel_ref / std::max(m.wheel_cost_post, wheel_ref),
                   0.0f, 1.0f);
  const float ground_ref =
      std::max(m.ground_cost_prior, config_.trust_ground_cost_floor);
  const float s_ground = (m.ground_cost_post <= ground_ref)
      ? 1.0f
      : std::clamp(ground_ref / std::max(m.ground_cost_post, ground_ref),
                   0.0f, 1.0f);
  const float s_prior = 1.0f / (1.0f + 5.0f * m.prior_cost_post);
  const float s_obs = 0.5f * std::clamp(m.obs_min_eig / 10.0f, 0.0f, 1.0f) +
                      0.5f * std::clamp(m.obs_cond_ratio / 0.05f, 0.0f, 1.0f);
  m.trust_score = std::clamp(
      0.15f * s_corr +
      0.25f * s_rms +
      0.20f * s_wheel +
      0.10f * s_ground +
      0.15f * s_prior +
      0.15f * s_obs,
      0.0f, 1.0f);
  return m;
}

LwoEstimator::LwoAcceptanceResult LwoEstimator::accept_or_blend_update(
    const LwoState& x_wheel,
    const LwoState& x_lidar,
    const LwoTrustMetrics& metrics) {
  LwoAcceptanceResult out;
  out.output_state = x_wheel;
  out.metrics = metrics;

  const float base_cost =
      metrics.lidar_rms_prior * metrics.lidar_rms_prior +
      config_.no_harm_lambda_wheel * metrics.wheel_cost_prior +
      config_.no_harm_lambda_ground * metrics.ground_cost_prior;
  const float candidate_cost =
      metrics.lidar_rms_post * metrics.lidar_rms_post +
      config_.no_harm_lambda_wheel * metrics.wheel_cost_post +
      config_.no_harm_lambda_ground * metrics.ground_cost_post +
      config_.no_harm_lambda_prior * metrics.prior_cost_post;

  const bool objective_improved =
      !config_.enable_no_harm_gate ||
      (candidate_cost + config_.no_harm_accept_margin < base_cost);
  const float wheel_ref =
      std::max(metrics.wheel_cost_prior, config_.trust_wheel_cost_floor);
  const bool wheel_safe =
      metrics.wheel_cost_post <=
      std::max(wheel_ref * config_.trust_max_wheel_cost_increase,
               metrics.wheel_cost_prior + config_.trust_wheel_safe_abs_increase);
  const float ground_ref =
      std::max(metrics.ground_cost_prior, config_.trust_ground_cost_floor);
  const bool ground_safe =
      metrics.ground_cost_post <=
      std::max(ground_ref * config_.trust_max_ground_cost_increase,
               metrics.ground_cost_prior + config_.trust_ground_safe_abs_increase);
  const bool rms_safe =
      metrics.lidar_rms_post <=
      metrics.lidar_rms_prior * config_.trust_min_lidar_rms_drop_ratio;
  const bool rms_nonworsening =
      metrics.lidar_rms_ratio <= config_.accept_max_mean_rms_ratio;
  const bool pose_window_safe =
      metrics.pose_delta_norm <= config_.accept_max_mean_pose_delta;
  const bool yaw_window_safe =
      metrics.yaw_delta_abs <= config_.accept_max_mean_yaw_delta;
  const bool current_window_consistent =
      objective_improved && wheel_safe && ground_safe &&
      metrics.trust_score >= config_.trust_min_score_for_accept &&
      rms_nonworsening && pose_window_safe && yaw_window_safe;
  const bool hard_reject =
      !objective_improved || !wheel_safe || !ground_safe ||
      metrics.pose_delta_norm > config_.map_commit_max_pose_delta * 2.0f ||
      metrics.yaw_delta_abs > config_.map_commit_max_yaw_delta * 2.0f ||
      metrics.trust_score < config_.trust_min_score_for_blend;

  out.metrics.objective_improved = objective_improved;
  out.metrics.wheel_safe = wheel_safe;
  out.metrics.ground_safe = ground_safe;
  out.metrics.rms_safe = rms_safe;
  out.metrics.window_consistent = current_window_consistent;
  out.metrics.reject_reason_code = kRejectReasonNone;
  out.metrics.accept_reason_code = kAcceptReasonNone;
  out.metrics.commit_reason_code = kCommitReasonNone;

  int hits = current_window_consistent ? 1 : 0;
  float trust_sum = metrics.trust_score;
  const int window_size = std::max(config_.accept_window_size, 1);
  int samples = 1;
  for (auto it = accept_history_.rbegin();
       it != accept_history_.rend() && samples < window_size;
       ++it, ++samples) {
    trust_sum += it->trust_score;
    if (it->window_consistent) {
      ++hits;
    }
  }
  const float trust_window_mean = trust_sum / static_cast<float>(samples);
  out.metrics.trust_window_mean = trust_window_mean;
  out.metrics.accept_window_hits = hits;

  if (hard_reject) {
    out.metrics.accepted = false;
    out.metrics.blended = false;
    out.metrics.blend_alpha = 0.0f;
    out.metrics.map_commit_allowed = false;
    if (!objective_improved) {
      out.metrics.reject_reason_code = kRejectReasonObjective;
    } else if (!wheel_safe) {
      out.metrics.reject_reason_code = kRejectReasonWheel;
    } else if (!ground_safe) {
      out.metrics.reject_reason_code = kRejectReasonGround;
    } else if (metrics.pose_delta_norm > config_.map_commit_max_pose_delta * 2.0f) {
      out.metrics.reject_reason_code = kRejectReasonPoseDelta;
    } else if (metrics.yaw_delta_abs > config_.map_commit_max_yaw_delta * 2.0f) {
      out.metrics.reject_reason_code = kRejectReasonYawDelta;
    } else if (metrics.trust_score < config_.trust_min_score_for_blend) {
      out.metrics.reject_reason_code = kRejectReasonTrust;
    }
  } else {
    const bool accept_full =
        hits >= std::max(config_.accept_min_consistent_frames, 1) &&
        trust_window_mean >= config_.trust_min_score_for_accept &&
        rms_nonworsening &&
        pose_window_safe &&
        yaw_window_safe &&
        (rms_safe || metrics.lidar_rms_post <= metrics.lidar_rms_prior);

    if (accept_full || !config_.enable_trust_blending) {
      out.output_state = x_lidar;
      out.output_state.covariance = x_lidar.covariance;
      const auto diag =
          x_lidar.covariance.diagonal().cwiseMax(x_wheel.covariance.diagonal());
      out.output_state.covariance.diagonal() = diag;
      out.metrics.accepted = true;
      out.metrics.blended = false;
      out.metrics.blend_alpha = 1.0f;
      out.metrics.accept_reason_code =
          accept_full ? kAcceptReasonWindow : kAcceptReasonBlendDisabled;
    } else {
      LwoStateVector delta = x_lidar - x_wheel;
      delta.segment<3>(kLwoVelIdx).setZero();
      delta(kLwoScaleIdx) = 0.0f;
      delta.segment<2>(kLwoBiasIdx).setZero();
      delta(kLwoExtYawIdx) = 0.0f;
      delta.segment<2>(kLwoExtXyIdx).setZero();

      const float trust_t = std::clamp(
          (metrics.trust_score - config_.trust_min_score_for_blend) /
          std::max(config_.trust_min_score_for_commit -
                   config_.trust_min_score_for_blend, 1e-3f),
          0.0f, 1.0f);
      const float alpha = std::clamp(
          config_.trust_blend_alpha_min +
          trust_t * (config_.trust_blend_alpha_max - config_.trust_blend_alpha_min),
          config_.trust_blend_alpha_min,
          config_.trust_blend_alpha_max);
      float scaled_alpha = alpha;
      if (metrics.corr_count < config_.min_correspondences +
                                config_.low_corr_blend_margin) {
        scaled_alpha *= config_.low_corr_blend_alpha_scale;
      }
      if (!current_window_consistent) {
        scaled_alpha *= config_.inconsistent_blend_alpha_scale;
      }
      scaled_alpha = std::clamp(
          scaled_alpha,
          config_.trust_blend_alpha_min,
          config_.trust_blend_alpha_max);

      out.output_state = x_wheel + scaled_alpha * delta;
      out.output_state.velocity = x_wheel.velocity;
      out.output_state.wheel_scale = x_wheel.wheel_scale;
      out.output_state.wheel_gyro_bias = x_wheel.wheel_gyro_bias;
      out.output_state.ext_delta_yaw = x_wheel.ext_delta_yaw;
      out.output_state.ext_delta_xy = x_wheel.ext_delta_xy;
      out.output_state.covariance = x_wheel.covariance;
      out.metrics.accepted = false;
      out.metrics.blended = true;
      out.metrics.blend_alpha = scaled_alpha;
      out.metrics.accept_reason_code = kAcceptReasonBlend;
    }
  }

  const bool eligible_for_commit =
      (out.metrics.accepted || !config_.map_commit_require_accept) &&
      (!config_.map_commit_require_rms_drop ||
       metrics.lidar_rms_post < metrics.lidar_rms_prior) &&
      metrics.trust_score >= config_.trust_min_score_for_commit &&
      metrics.lidar_rms_post <= config_.map_commit_max_lidar_rms &&
      metrics.pose_delta_norm <= config_.map_commit_max_pose_delta &&
      metrics.yaw_delta_abs <= config_.map_commit_max_yaw_delta;
  out.metrics.map_commit_allowed = eligible_for_commit;
  if (eligible_for_commit) {
    out.metrics.commit_reason_code = kCommitReasonAccepted;
  } else if (config_.map_commit_require_accept && !out.metrics.accepted) {
    out.metrics.commit_reason_code = kCommitReasonAcceptRequired;
  } else if (config_.map_commit_require_rms_drop &&
             !(metrics.lidar_rms_post < metrics.lidar_rms_prior)) {
    out.metrics.commit_reason_code = kCommitReasonRmsDrop;
  } else if (metrics.trust_score < config_.trust_min_score_for_commit) {
    out.metrics.commit_reason_code = kCommitReasonTrust;
  } else if (metrics.lidar_rms_post > config_.map_commit_max_lidar_rms) {
    out.metrics.commit_reason_code = kCommitReasonLidarRms;
  } else if (metrics.pose_delta_norm > config_.map_commit_max_pose_delta) {
    out.metrics.commit_reason_code = kCommitReasonPoseDelta;
  } else if (metrics.yaw_delta_abs > config_.map_commit_max_yaw_delta) {
    out.metrics.commit_reason_code = kCommitReasonYawDelta;
  }

  accept_history_.push_back(AcceptanceHistoryEntry{
      metrics.trust_score,
      metrics.lidar_rms_ratio,
      metrics.pose_delta_norm,
      metrics.yaw_delta_abs,
      objective_improved,
      wheel_safe,
      ground_safe,
      rms_nonworsening,
      current_window_consistent,
  });
  while (static_cast<int>(accept_history_.size()) > window_size) {
    accept_history_.pop_front();
  }

  return out;
}

// ---------------------------------------------------------------------------
// update_extrinsic_from_state
// ---------------------------------------------------------------------------

void LwoEstimator::update_extrinsic_from_state() {
  // Rebuild T_body_lidar_current_ from the initial (config) extrinsic plus
  // the delta corrections stored in the IEKF state.
  //
  // R_corrected = R_il * Rz(ext_delta_yaw)
  // t_corrected = t_il + [ext_delta_x, ext_delta_y, 0]^T
  const Eigen::Matrix3f R_il = config_.T_body_lidar.rotation_matrix();
  const Eigen::Vector3f t_il = config_.T_body_lidar.translation();

  const Eigen::Matrix3f dR =
      Eigen::AngleAxisf(state_.ext_delta_yaw, Eigen::Vector3f::UnitZ())
          .toRotationMatrix();
  const Eigen::Matrix3f R_corrected = R_il * dR;

  Eigen::Vector3f t_corrected = t_il;
  t_corrected.x() += state_.ext_delta_xy(0);
  t_corrected.y() += state_.ext_delta_xy(1);

  T_body_lidar_current_ = core::Se3(core::So3(R_corrected), t_corrected);
}

// ---------------------------------------------------------------------------
// find_correspondences_lwo
// ---------------------------------------------------------------------------

std::vector<core::Correspondence> LwoEstimator::find_correspondences_lwo(
    const core::PointCloud& scan) const {
  std::vector<core::Correspondence> correspondences;
  if (scan.empty()) return correspondences;

  const core::Se3 T_wb = state_.pose();
  const core::Se3 T_wl = T_wb * T_body_lidar_current_;
  ankerl::unordered_dense::map<core::VoxelKey, int, core::VoxelKeyHash> l1_count;

  correspondences.reserve(scan.size());
  for (const auto& pt : scan) {
    const Eigen::Vector3f p_lidar = pt.to_eigen();
    const Eigen::Vector3f p_world = T_wl * p_lidar;

    if (config_.enable_quality_aware_corr && config_.corr_max_per_l1 > 0) {
      const core::VoxelKey k1 = surfel_map_.compute_l1_key(p_world);
      auto [it, inserted] = l1_count.try_emplace(k1, 0);
      if (++(it->second) > config_.corr_max_per_l1) {
        continue;
      }
    }

    core::Surfel surfel;
    const bool found = surfel_map_.get_surfel(p_world, &surfel);
    if (!found) continue;
    if (!surfel.valid) continue;

    const float plane_dist = std::abs(surfel.normal.dot(p_world - surfel.centroid));
    if (config_.enable_quality_aware_corr &&
        config_.corr_max_plane_distance > 0.0f &&
        plane_dist > config_.corr_max_plane_distance) {
      continue;
    }

    core::Correspondence corr;
    corr.p_lidar = p_lidar;
    corr.normal = surfel.normal;
    corr.plane_d = surfel.normal.dot(surfel.centroid);
    corr.planarity = surfel.planarity;
    corr.centroid = surfel.centroid;
    corr.normal_sigma2 = surfel.normal_sigma2;

    // v1.0 adaptive noise fields: range and cos_incidence.
    // ray = p_world - sensor_origin_world (sensor origin in world frame).
    const Eigen::Vector3f sensor_origin = T_wl.translation();
    const Eigen::Vector3f ray = p_world - sensor_origin;
    corr.range = ray.norm();
    if (corr.range > 1e-6f) {
      corr.cos_incidence = std::abs(ray.dot(surfel.normal)) / corr.range;
    } else {
      corr.cos_incidence = 1.0f;
    }

    correspondences.push_back(corr);
  }

  // Sharing count: count how many correspondences share the same L1 voxel.
  if (!correspondences.empty()) {
    ankerl::unordered_dense::map<core::VoxelKey, int, core::VoxelKeyHash>
        share_count;
    const core::Se3 T_wl_share = state_.pose() * T_body_lidar_current_;
    for (const auto& c : correspondences) {
      const Eigen::Vector3f pw = T_wl_share * c.p_lidar;
      const core::VoxelKey k1 = surfel_map_.compute_l1_key(pw);
      share_count[k1]++;
    }
    for (auto& c : correspondences) {
      const Eigen::Vector3f pw = T_wl_share * c.p_lidar;
      const core::VoxelKey k1 = surfel_map_.compute_l1_key(pw);
      auto it = share_count.find(k1);
      if (it != share_count.end()) {
        c.sharing_count = it->second;
      }
    }
  }

  return correspondences;
}

// ---------------------------------------------------------------------------
// find_anchor_correspondences
// ---------------------------------------------------------------------------

std::vector<core::Correspondence> LwoEstimator::find_anchor_correspondences(
    const core::PointCloud& scan) const {
  std::vector<core::Correspondence> correspondences;
  if (!anchor_map_frozen_ || scan.empty()) return correspondences;

  const core::Se3 T_wb = state_.pose();
  const core::Se3 T_wl = T_wb * T_body_lidar_current_;

  correspondences.reserve(scan.size());
  for (const auto& pt : scan) {
    const Eigen::Vector3f p_lidar = pt.to_eigen();
    const Eigen::Vector3f p_world = T_wl * p_lidar;

    core::Surfel surfel;
    if (!anchor_map_.get_surfel(p_world, &surfel)) continue;
    if (!surfel.valid) continue;

    core::Correspondence corr;
    corr.p_lidar = p_lidar;
    corr.normal = surfel.normal;
    corr.plane_d = surfel.normal.dot(surfel.centroid);
    corr.planarity = surfel.planarity;
    corr.normal_sigma2 = surfel.normal_sigma2;
    corr.noise_override = config_.anchor_noise_std;
    correspondences.push_back(corr);
  }

  return correspondences;
}

// ---------------------------------------------------------------------------
// apply_anchor_yaw_correction — post-IEKF yaw-only correction using the
// frozen anchor map.  Runs after the main IEKF update.
//
// Design (docs/15_post_iekf_yaw_anchor_correction.md):
//   For each scan point that hits a frozen anchor surfel, compute:
//     r_i = n^T * (p_world - c_anchor)         (point-to-plane residual)
//     J_i = -n_x * d_y + n_y * d_x             (yaw Jacobian)
//     where d = p_world - robot_pos (world-frame lever arm)
//   Solve: delta_psi = -A/B  where A = sum(w_i * J_i * r_i), B = sum(w_i * J_i^2)
//   Apply: R_corrected = R_z(gain * delta_psi) * R_current  (LEFT multiply)
// ---------------------------------------------------------------------------

void LwoEstimator::apply_anchor_yaw_correction(const core::PointCloud& scan) {
  // Gate 1: feature enabled and anchor map frozen.
  if (!config_.anchor_enable || !anchor_map_frozen_) return;
  // Gate 2: minimum frame guard (don't apply during anchor build).
  if (frame_count_ <= config_.anchor_build_frames) return;
  // Gate 3: wait for min_frame to allow sufficient map buildup.
  if (frame_count_ < config_.anchor_min_frame) return;
  // Gate 4: proximity to anchor origin.
  const float dist_to_anchor =
      (state_.position.head<2>() - anchor_origin_.head<2>()).norm();
  const float eff_radius = config_.anchor_proximity_radius;
  if (dist_to_anchor > eff_radius) {
    was_outside_anchor_ = true;
    return;
  }

  // Leg-aware cumulative yaw reset.
  // When robot returns to anchor zone after being outside, reset the
  // cumulative yaw budget so each visit gets fresh correction capacity.
  if (was_outside_anchor_) {
    spdlog::info("[ANCHOR-YAW] frame={} RE-ENTER anchor zone: resetting "
                 "cumulative_yaw={:.5f}rad, correction_ema={:.6f}",
                 frame_count_,
                 static_cast<double>(cumulative_anchor_yaw_),
                 static_cast<double>(anchor_correction_ema_));
    cumulative_anchor_yaw_ = 0.0f;
    anchor_correction_ema_ = 0.0f;
    anchor_correction_count_ = 0;
    was_outside_anchor_ = false;
  }

  // Dynamic P_yaw inflation — LOCAL only for Kalman gain calc.
  // Does NOT modify state covariance to avoid feedback through main IEKF.
  float local_P_yaw_for_anchor = state_.covariance(kLwoRotIdx + 2, kLwoRotIdx + 2);
  if (config_.anchor_dynamic_p_yaw) {
    const float target_p_yaw = std::max(
        config_.anchor_p_yaw_inflate_min,
        outward_max_p_yaw_ * config_.anchor_p_yaw_inflate_scale);
    local_P_yaw_for_anchor = std::max(local_P_yaw_for_anchor, target_p_yaw);
  }

  // --- Build T_world_lidar from current state ---
  const core::Se3 T_wb = state_.pose();
  const core::Se3 T_wl = T_wb * T_body_lidar_current_;
  const Eigen::Vector3f robot_pos = state_.position;

  // --- First pass: collect (J_i, r_i) pairs for MAD-based sigma estimate ---
  struct JRPair { float J; float r; };
  std::vector<JRPair> pairs;
  pairs.reserve(scan.size());

  for (const auto& pt : scan) {
    const Eigen::Vector3f p_lidar = pt.to_eigen();
    const Eigen::Vector3f p_world = T_wl * p_lidar;

    core::Surfel surfel;
    if (!anchor_map_.get_surfel(p_world, &surfel)) continue;
    if (!surfel.valid) continue;

    // Point-to-plane residual: r = n^T * (p - c)
    const float r = surfel.normal.dot(p_world - surfel.centroid);

    // Yaw Jacobian: J = -n_x * d_y + n_y * d_x
    // where d = p_world - robot_pos is the world-frame lever arm
    const Eigen::Vector3f d = p_world - robot_pos;
    const float J = -surfel.normal.x() * d.y() + surfel.normal.y() * d.x();

    pairs.push_back({J, r});
  }

  // Gate 5: minimum correspondences.
  if (static_cast<int>(pairs.size()) < config_.anchor_yaw_min_corrs) {
    if (config_.enable_debug_log && frame_count_ % 10 == 0) {
      spdlog::info("[ANCHOR-YAW] frame={} skip: pairs={} < min={}",
                   frame_count_, pairs.size(), config_.anchor_yaw_min_corrs);
    }
    return;
  }

  // --- Tukey bisquare outlier rejection (MAD-based sigma estimate) ---
  // Compute MAD of residuals to estimate robust sigma.
  std::vector<float> abs_r;
  abs_r.reserve(pairs.size());
  for (const auto& p : pairs) abs_r.push_back(std::abs(p.r));
  std::nth_element(abs_r.begin(),
                   abs_r.begin() + static_cast<int>(abs_r.size()) / 2,
                   abs_r.end());
  const float mad = abs_r[abs_r.size() / 2];
  const float sigma_robust = std::max(1.4826f * mad, 0.01f);  // MAD -> std
  const float tukey_c = 4.685f * sigma_robust;                 // Tukey constant (95% eff.)

  // --- Weighted least squares: A = sum(w*J*r), B = sum(w*J^2) ---
  float A = 0.0f;  // numerator
  float B = 0.0f;  // information (denominator)
  int yaw_sensitive_count = 0;

  for (const auto& p : pairs) {
    // Tukey bisquare weight: w = (1 - (r/c)^2)^2  if |r| < c, else 0
    if (std::abs(p.r) >= tukey_c) continue;
    const float u = p.r / tukey_c;
    const float tmp = 1.0f - u * u;
    const float w = tmp * tmp;

    // Only use correspondences with non-negligible yaw sensitivity.
    // J is dimensionless (plane normal dot lever arm) — threshold ~0.05 m.
    if (std::abs(p.J) < 0.05f) continue;

    A += w * p.J * p.r;
    B += w * p.J * p.J;
    ++yaw_sensitive_count;
  }

  // Gate 6: information threshold.
  if (B < config_.anchor_yaw_B_min || yaw_sensitive_count < 5) {
    if (config_.enable_debug_log && frame_count_ % 10 == 0) {
      spdlog::info("[ANCHOR-YAW] frame={} skip: B={:.4f} < B_min={:.4f} "
                   "yaw_sens={}",
                   frame_count_,
                   static_cast<double>(B),
                   static_cast<double>(config_.anchor_yaw_B_min),
                   yaw_sensitive_count);
    }
    return;
  }

  // --- Solve for delta_psi ---
  const float delta_psi_raw = -A / B;

  // Gate 7: reject large corrections.
  if (std::abs(delta_psi_raw) > config_.anchor_yaw_max_correction) {
    if (frame_count_ % 10 == 0) {
      spdlog::info("[ANCHOR-YAW] frame={} REJECTED: raw={:.5f}rad > max={:.5f}rad "
                   "dist={:.3f}m pairs={} yaw_sens={}",
                   frame_count_,
                   static_cast<double>(delta_psi_raw),
                   static_cast<double>(config_.anchor_yaw_max_correction),
                   static_cast<double>(dist_to_anchor),
                   pairs.size(),
                   yaw_sensitive_count);
    }
    return;
  }

  // --- Consistency gate: track EMA of raw corrections ---
  // Block corrections when signal oscillates (anchor map bias noise).
  anchor_correction_ema_ = config_.anchor_consistency_alpha * delta_psi_raw +
      (1.0f - config_.anchor_consistency_alpha) * anchor_correction_ema_;
  anchor_correction_count_++;

  // After warmup, only apply when EMA shows consistent direction
  // (disabled when threshold=0.0)
  if (config_.anchor_consistency_threshold > 0.0f && anchor_correction_count_ > 5) {
    const bool ema_strong = std::abs(anchor_correction_ema_) > config_.anchor_consistency_threshold;
    const bool direction_match = (delta_psi_raw * anchor_correction_ema_ > 0.0f);
    if (!ema_strong || !direction_match) {
      spdlog::info("[ANCHOR-YAW] frame={} CONSISTENCY_GATE: ema={:.6f} raw={:.5f} "
                   "strong={} dir_match={} dist={:.3f}m pairs={}",
                   frame_count_,
                   static_cast<double>(anchor_correction_ema_),
                   static_cast<double>(delta_psi_raw),
                   ema_strong, direction_match,
                   static_cast<double>(dist_to_anchor),
                   pairs.size());
      return;
    }
  }

  // --- Residual-validated correction ---
  // Compute pre-correction residual sum, tentatively apply, check improvement.
  float pre_rss = 0.0f;
  for (const auto& p : pairs) {
    pre_rss += p.r * p.r;
  }

  const float delta_psi = config_.anchor_yaw_gain * delta_psi_raw;

  // Tentatively apply rotation
  Eigen::Vector3f delta_rot = Eigen::Vector3f::Zero();
  delta_rot.z() = delta_psi;
  const Eigen::Matrix3f R_correction = core::So3::Exp(delta_rot).matrix();
  const Eigen::Matrix3f R_corrected = R_correction * state_.rotation;

  // Compute post-correction residuals (re-transform points with corrected pose)
  const core::Se3 T_wb_corr = core::Se3(R_corrected, state_.position);
  const core::Se3 T_wl_corr = T_wb_corr * T_body_lidar_current_;
  float post_rss = 0.0f;
  int post_count = 0;
  for (const auto& pt : scan) {
    const Eigen::Vector3f p_lidar = pt.to_eigen();
    const Eigen::Vector3f p_world = T_wl_corr * p_lidar;
    core::Surfel surfel;
    if (!anchor_map_.get_surfel(p_world, &surfel)) continue;
    if (!surfel.valid) continue;
    const float r = surfel.normal.dot(p_world - surfel.centroid);
    post_rss += r * r;
    ++post_count;
  }

  // Only apply if residual decreased (correction is helpful)
  const bool residual_improved = (post_rss < pre_rss * config_.anchor_residual_threshold);
  const bool within_cumulative = (std::abs(cumulative_anchor_yaw_ + delta_psi) < config_.anchor_max_cumulative_yaw);
  float applied_psi = 0.0f;
  if (residual_improved && within_cumulative) {
    state_.rotation = R_corrected;
    applied_psi = delta_psi;
    cumulative_anchor_yaw_ += delta_psi;
    anchor_corrected_this_frame_ = true;
  }

  // --- Update P_yaw with scalar Kalman update ---
  const float P_yaw_actual = state_.covariance(kLwoRotIdx + 2, kLwoRotIdx + 2);
  const float K = local_P_yaw_for_anchor * B /
                  (1.0f + local_P_yaw_for_anchor * B);
  if (residual_improved && within_cumulative) {
    state_.covariance(kLwoRotIdx + 2, kLwoRotIdx + 2) =
        std::max((1.0f - K) * P_yaw_actual, P_yaw_actual * 0.9f);
  }

  // --- Log result ---
  spdlog::info("[ANCHOR-YAW] frame={} dist={:.3f}m pairs={} yaw_sens={} "
               "raw={:.5f}rad applied={:.5f}rad ({:.3f}deg) "
               "B={:.2f} pre_rss={:.5f} post_rss={:.5f} improved={} P_yaw={:.6f}",
               frame_count_,
               static_cast<double>(dist_to_anchor),
               pairs.size(),
               yaw_sensitive_count,
               static_cast<double>(delta_psi_raw),
               static_cast<double>(applied_psi),
               static_cast<double>(applied_psi * 180.0f / 3.14159f),
               static_cast<double>(B),
               static_cast<double>(pre_rss),
               static_cast<double>(post_rss),
               residual_improved ? 1 : 0,
               static_cast<double>(state_.covariance(kLwoRotIdx + 2, kLwoRotIdx + 2)));
}

// ---------------------------------------------------------------------------
// feed_lidar
// ---------------------------------------------------------------------------

bool LwoEstimator::feed_lidar(const core::PointCloud& scan, double timestamp) {
  std::lock_guard<std::mutex> lock(mutex_);

  using Clock = std::chrono::steady_clock;
  const bool do_usage = config_.check_usage;
  const auto t_start = do_usage ? Clock::now() : Clock::time_point{};

  if (!initialized_) {
    return false;
  }

  // Reset usage for this frame
  if (do_usage) {
    last_usage_ = FrameUsage{};
    last_usage_.frame_id   = frame_count_;
    last_usage_.timestamp  = timestamp;
    last_usage_.queue_depth = pending_queue_depth_;
    last_usage_.wheel_ms   = wheel_ms_accumulator_;
    wheel_ms_accumulator_  = 0.0f;
  }

  const auto t_preprocess_start = do_usage ? Clock::now() : Clock::time_point{};

  // Preprocess
  core::PointCloud processed = preprocess_scan(scan, timestamp);

  if (do_usage) {
    last_usage_.preprocess_ms = std::chrono::duration<float, std::milli>(
        Clock::now() - t_preprocess_start).count();
    last_usage_.scan_points = static_cast<int>(processed.size());
  }

  if (processed.empty()) {
    SPDLOG_WARN("LWO feed_lidar: processed scan empty (raw={} pts), skipping frame (frame_count={})",
                scan.size(), frame_count_);
    return false;
  }

  // Store processed scan for external inspection (e.g. raw vs processed diff)
  last_processed_scan_ = processed;

  // Reset diagnostics for this frame
  last_diag_ = FrameDiagnostics{};
  last_diag_.frame_id  = frame_count_;
  last_diag_.timestamp = timestamp;

  // First frame: just add points to the map, no IEKF update.
  if (first_lidar_frame_) {
    core::PointCloud world_cloud = transform_to_world(processed);
    surfel_map_.update(world_cloud, state_.position);

    // ---- Anchor map: seed with frame 0 ----
    if (config_.anchor_enable && frame_count_ == 0) {
      anchor_origin_ = state_.position;
      anchor_map_.update(world_cloud, state_.position);
      SPDLOG_INFO("[ANCHOR] Build started: origin=[{:.3f},{:.3f},{:.3f}] "
                  "build_frames={}",
                  static_cast<double>(anchor_origin_.x()),
                  static_cast<double>(anchor_origin_.y()),
                  static_cast<double>(anchor_origin_.z()),
                  config_.anchor_build_frames);
    }

    first_lidar_frame_ = false;
    last_lidar_time_ = timestamp;
    frame_count_++;
    state_history_.clear();
    accept_history_.clear();
    SPDLOG_INFO("LWO first lidar frame processed, map initialized with {} points",
                world_cloud.size());
    return true;
  }

  // --- Safety: clamp velocity state before IEKF ---------------------------
  // The wheel propagator directly assigns velocity = R * v_b, but the IEKF
  // or trapezoidal position integration can cause velocity to accumulate.
  // Hard-clamp to prevent runaway (max 5 m/s for ground robot).
  {
    const float vmag = state_.velocity.norm();
    if (vmag > 5.0f) {
      SPDLOG_WARN("LWO frame={} vel_clamp: vel_mag={:.2f} → 3.0",
                  frame_count_, static_cast<double>(vmag));
      state_.velocity = state_.velocity.normalized() * 3.0f;
    }
  }

  // --- IEKF outer loop (matching LIO pattern: re-linearize correspondences) --
  const LwoState wheel_state_before_lidar = state_;
  LwoStateCovariance P_wheel = state_.covariance;  // Freeze wheel prior.

  // Body yaw rate for wheel measurement residual.
  const float omega_z_b = last_omega_z_enc_ - state_.wheel_gyro_bias(1);
  LwoIekfResult last_result;
  bool had_iekf = false;
  bool hard_rejected = false;
  std::vector<core::Correspondence> final_corrs;
  LwoTrustMetrics trust_metrics;

  // ========== P_yaw tracking (always, for dynamic inflation) ==============
  if (config_.anchor_enable && config_.anchor_dynamic_p_yaw) {
    const float P_yaw_track =
        state_.covariance(kLwoRotIdx + 2, kLwoRotIdx + 2);
    outward_max_p_yaw_ = std::max(outward_max_p_yaw_, P_yaw_track);
  }

  // ========== Anchor Map Build / Freeze ====================================
  if (config_.anchor_enable && !anchor_map_frozen_) {
    // --- Legacy build: fixed frame count ---
    // Always track P_yaw and max distance for dynamic inflation
    const float P_yaw_cur =
        state_.covariance(kLwoRotIdx + 2, kLwoRotIdx + 2);
    const float dist_from_origin =
        (state_.position.head<2>() - anchor_origin_.head<2>()).norm();
    outward_max_p_yaw_ = std::max(outward_max_p_yaw_, P_yaw_cur);
    if (dist_from_origin > max_dist_from_origin_) {
      max_dist_from_origin_ = dist_from_origin;
    }

    if (frame_count_ < config_.anchor_build_frames) {
      core::PointCloud world_cloud_anchor = transform_to_world(processed);
      anchor_map_.update(world_cloud_anchor, state_.position);
    } else {
      anchor_map_.flush_dirty();
      anchor_map_frozen_ = true;
      SPDLOG_INFO("[ANCHOR] Map frozen at frame={}: l0={} l1={} "
                  "outward_max_P_yaw={:.6f} max_dist={:.2f}m "
                  "origin=[{:.3f},{:.3f},{:.3f}]",
                  frame_count_,
                  anchor_map_.l0_count(),
                  anchor_map_.l1_count(),
                  static_cast<double>(outward_max_p_yaw_),
                  static_cast<double>(max_dist_from_origin_),
                  static_cast<double>(anchor_origin_.x()),
                  static_cast<double>(anchor_origin_.y()),
                  static_cast<double>(anchor_origin_.z()));
    }
  }

  // Flush dirty surfels before correspondence finding (v1.0 pattern).
  // Ensures newly inserted L0 points have been propagated to L1 surfels.
  surfel_map_.flush_dirty();

  for (int outer = 0; outer < config_.max_outer_iters; ++outer) {
    pko_.reset();

    const auto t_corr_start = do_usage ? Clock::now() : Clock::time_point{};
    std::vector<core::Correspondence> corrs = find_correspondences_lwo(processed);
    if (do_usage) {
      last_usage_.corr_find_ms += std::chrono::duration<float, std::milli>(
          Clock::now() - t_corr_start).count();
    }
    last_corr_count_ = static_cast<int>(corrs.size());

    if (outer == 0) {
      last_diag_.corr_count = last_corr_count_;
      if (!corrs.empty()) {
        float plan_sum = 0.0f;
        for (const auto& c : corrs) plan_sum += c.planarity;
        last_diag_.corr_planarity_avg =
            plan_sum / static_cast<float>(corrs.size());
      }
    }

    if (frame_count_ % 100 == 0) {
      const float vel_mag = state_.velocity.norm();
      SPDLOG_INFO("LWO frame={} outer={} corrs={} pos=[{:.2f},{:.2f},{:.2f}] "
                  "vel_mag={:.3f} scale={:.4f} l0={} l1={}",
                  frame_count_, outer, corrs.size(),
                  static_cast<double>(state_.position.x()),
                  static_cast<double>(state_.position.y()),
                  static_cast<double>(state_.position.z()),
                  static_cast<double>(vel_mag),
                  static_cast<double>(state_.wheel_scale),
                  surfel_map_.l0_count(), surfel_map_.l1_count());
    }

    if (config_.enable_debug_log && !corrs.empty()) {
      float plan_sum = 0.0f;
      float plan_min = std::numeric_limits<float>::max();
      for (const auto& c : corrs) {
        plan_sum += c.planarity;
        if (c.planarity < plan_min) plan_min = c.planarity;
      }
      const float plan_avg = plan_sum / static_cast<float>(corrs.size());
      spdlog::info("[LWO-DBG] frame={} corr: total={} planarity_avg={:.4f} "
                   "planarity_min={:.4f}",
                   frame_count_, corrs.size(),
                   static_cast<double>(plan_avg),
                   static_cast<double>(plan_min));
    }

    if (static_cast<int>(corrs.size()) < config_.min_correspondences &&
        frame_count_ > config_.bootstrap_frames) {
      final_corrs = corrs;
      break;
    }

    if (config_.anchor_iekf_blend_enable &&
        config_.anchor_enable &&
        anchor_map_frozen_ &&
        frame_count_ >= config_.anchor_min_frame) {
      const float eff_radius = config_.anchor_proximity_radius;
      const float dist_to_anchor =
          (state_.position.head<2>() - anchor_origin_.head<2>()).norm();

      if (dist_to_anchor < eff_radius) {
        const float proximity_t = 1.0f - dist_to_anchor / eff_radius;
        const float blend_ratio =
            config_.anchor_iekf_blend_ratio * proximity_t;

        const auto t_anchor_corr_start =
            do_usage ? Clock::now() : Clock::time_point{};
        const std::vector<core::Correspondence> anchor_corrs =
            find_anchor_correspondences(processed);
        if (do_usage) {
          last_usage_.anchor_corr_ms += std::chrono::duration<float, std::milli>(
              Clock::now() - t_anchor_corr_start).count();
        }

        if (!anchor_corrs.empty()) {
          const int n_live = static_cast<int>(corrs.size());
          const float ratio_safe = std::min(blend_ratio, 0.99f);
          const int n_target = static_cast<int>(
              ratio_safe / (1.0f - ratio_safe) * static_cast<float>(n_live) + 0.5f);
          const int n_inject = std::min(
              n_target, static_cast<int>(anchor_corrs.size()));

          if (n_inject > 0) {
            corrs.insert(corrs.end(),
                         anchor_corrs.begin(),
                         anchor_corrs.begin() + n_inject);

            if (outer == 0 &&
                (frame_count_ % 10 == 0 || config_.enable_debug_log)) {
              SPDLOG_INFO("[ANCHOR-IEKF] frame={} outer={} dist={:.3f}m "
                          "proximity_t={:.3f} blend_ratio={:.3f} "
                          "n_live={} n_anchor_avail={} n_inject={} total={}",
                          frame_count_, outer,
                          static_cast<double>(dist_to_anchor),
                          static_cast<double>(proximity_t),
                          static_cast<double>(blend_ratio),
                          n_live,
                          static_cast<int>(anchor_corrs.size()),
                          n_inject,
                          static_cast<int>(corrs.size()));
            }
          }
        }
      }
    }

    final_corrs = corrs;

    LwoState prior_for_iekf = state_;
    prior_for_iekf.covariance = P_wheel;

    if (config_.vel_noise_scale > 0.0f) {
      const float vx = std::abs(last_vx_enc_);
      const float speed = std::max(vx, config_.vel_noise_min_speed);
      const float base_noise = config_.iekf.lidar_noise_std;
      iekf_updater_.mutable_cfg().lidar_noise_std =
          base_noise * (1.0f + config_.vel_noise_scale / speed);
    }

    const auto t_iekf_start = do_usage ? Clock::now() : Clock::time_point{};
    LwoIekfResult result = iekf_updater_.update(
        prior_for_iekf, corrs, T_body_lidar_current_,
        ground_constraint_, wheel_measurement_,
        last_vx_enc_, last_omega_z_enc_, omega_z_b, &pko_);
    had_iekf = true;
    last_result = result;
    if (do_usage) {
      last_usage_.iekf_ms += std::chrono::duration<float, std::milli>(
          Clock::now() - t_iekf_start).count();
      last_usage_.iekf_jacobian_ms += result.jacobian_ms;
      last_usage_.iekf_build_ms    += result.build_info_ms;
      last_usage_.iekf_solve_ms    += result.solve_ms;
      last_usage_.outer_iters       = outer + 1;
      last_usage_.total_inner_iters += result.total_iterations;
      last_usage_.corr_count        = static_cast<int>(corrs.size());
    }
    if (frame_count_ % 50 == 0) {
      SPDLOG_INFO("  lwo_iekf outer={} iters={} converged={} "
                  "pos_delta=[{:.4f},{:.4f},{:.4f}]",
                  outer, result.total_iterations, result.converged,
                  static_cast<double>(result.state.position.x() - state_.position.x()),
                  static_cast<double>(result.state.position.y() - state_.position.y()),
                  static_cast<double>(result.state.position.z() - state_.position.z()));
    }

    if (config_.enable_debug_log) {
      const Eigen::Vector3f dp = result.state.position - state_.position;
      const Eigen::Matrix3f dR = state_.rotation.transpose() * result.state.rotation;
      const Eigen::Vector3f drpy = dR.eulerAngles(2, 1, 0);
      const float res_rms = compute_lidar_rms(result.state, corrs);
      spdlog::info("[LWO-DBG] frame={} iekf: outer={} iters={} converged={} "
                   "pos_delta=[{:.4f},{:.4f},{:.4f}] "
                   "rot_delta=[{:.5f},{:.5f},{:.5f}] res_rms={:.5f}",
                   frame_count_, outer, result.total_iterations,
                   result.converged ? 1 : 0,
                   static_cast<double>(dp.x()),
                   static_cast<double>(dp.y()),
                   static_cast<double>(dp.z()),
                   static_cast<double>(drpy(2)),
                   static_cast<double>(drpy(1)),
                   static_cast<double>(drpy(0)),
                   static_cast<double>(res_rms));
    }

    {
      const float total_pos_delta =
          (result.state.position - state_.position).norm();
      const Eigen::Matrix3f dR =
          state_.rotation.transpose() * result.state.rotation;
      const float cos_angle = std::clamp(
          (dR.trace() - 1.0f) * 0.5f, -1.0f, 1.0f);
      const float total_rot_delta = std::acos(cos_angle);

      if (total_pos_delta > config_.iekf_max_total_pos_correction ||
          total_rot_delta > config_.iekf_max_total_rot_correction) {
        spdlog::warn("[LWO] IEKF rejected: pos={:.4f}m rot={:.4f}rad "
                     "(limits: {:.4f}m, {:.4f}rad) frame={}",
                     static_cast<double>(total_pos_delta),
                     static_cast<double>(total_rot_delta),
                     static_cast<double>(config_.iekf_max_total_pos_correction),
                     static_cast<double>(config_.iekf_max_total_rot_correction),
                     frame_count_);
        hard_rejected = true;
        state_ = wheel_state_before_lidar;
        state_.covariance = P_wheel;
        state_.covariance(kLwoRotIdx + 2, kLwoRotIdx + 2) += 0.01f;
        break;
      }
    }

    state_.rotation = result.state.rotation;
    state_.position = result.state.position;
    state_.wheel_scale = result.state.wheel_scale;
    state_.wheel_gyro_bias = result.state.wheel_gyro_bias;
    state_.ext_delta_yaw = result.state.ext_delta_yaw;
    state_.ext_delta_xy  = result.state.ext_delta_xy;

    if (config_.iekf.enable_ext_calibration) {
      update_extrinsic_from_state();
    }

    if (result.converged && outer > 0) {
      state_.covariance = result.state.covariance;
      break;
    }
    if (outer == config_.max_outer_iters - 1) {
      state_.covariance = result.state.covariance;
    }
  }

  if (had_iekf && !hard_rejected && !final_corrs.empty()) {
    trust_metrics = evaluate_trust_metrics(
        wheel_state_before_lidar, state_, final_corrs,
        last_vx_enc_, last_omega_z_enc_, omega_z_b);
    const LwoAcceptanceResult acceptance = accept_or_blend_update(
        wheel_state_before_lidar, state_, trust_metrics);
    state_ = acceptance.output_state;
    trust_metrics = acceptance.metrics;
  } else {
    state_ = wheel_state_before_lidar;
    state_.covariance = P_wheel;
    trust_metrics.corr_count = last_corr_count_;
  }

  if ((trust_metrics.accepted || trust_metrics.blended) &&
      config_.enable_pos_bias_est) {
    const Eigen::Vector3f dx_pos_world =
        state_.position - wheel_state_before_lidar.position;
    const Eigen::Vector3f dx_pos_body =
        state_.rotation.transpose() * dx_pos_world;
    const float dx_pos_body_norm = dx_pos_body.head<2>().norm();
    if (dx_pos_body_norm < config_.pos_bias_outlier_threshold) {
      const float alpha = config_.pos_bias_ema_alpha;
      pos_bias_ema_ = (1.0f - alpha) * pos_bias_ema_ + alpha * dx_pos_body;
      if (!config_.pos_bias_enable_z) {
        pos_bias_ema_(2) = 0.0f;
      }
      pos_bias_sample_count_++;
    }

    if (pos_bias_sample_count_ >= config_.pos_bias_warmup_frames) {
      const Eigen::Vector3f bias_world = state_.rotation * pos_bias_ema_;
      state_.position -= config_.pos_bias_correction_gain * bias_world;
    }

    if (frame_count_ % 50 == 0) {
      SPDLOG_INFO("[POS-BIAS] frame={} bias_body=[{:.5f},{:.5f},{:.5f}] "
                  "norm={:.5f}m samples={} active={}",
                  frame_count_,
                  static_cast<double>(pos_bias_ema_(0)),
                  static_cast<double>(pos_bias_ema_(1)),
                  static_cast<double>(pos_bias_ema_(2)),
                  static_cast<double>(pos_bias_ema_.head<2>().norm()),
                  pos_bias_sample_count_,
                  pos_bias_sample_count_ >= config_.pos_bias_warmup_frames ? 1 : 0);
    }
  }

  if (had_iekf) {
    last_diag_.iekf_converged = last_result.converged && !hard_rejected;
    last_diag_.iekf_iters     = last_result.total_iterations;
    last_diag_.wv_residual_vx    = last_result.wv_residual_vx;
    last_diag_.wv_residual_omega = last_result.wv_residual_omega;
    last_diag_.wv_info_vx        = last_result.wv_info_vx;
    last_diag_.wv_info_omega     = last_result.wv_info_omega;
  }
  last_diag_.iekf_pos_delta = trust_metrics.pose_delta_norm;
  last_diag_.iekf_rot_delta = trust_metrics.yaw_delta_abs;
  last_diag_.lidar_rms_prior = trust_metrics.lidar_rms_prior;
  last_diag_.lidar_rms_post =
      final_corrs.empty() ? 0.0f : compute_lidar_rms(state_, final_corrs);
  last_diag_.corr_res_rms = last_diag_.lidar_rms_post;
  last_diag_.trust_score = trust_metrics.trust_score;
  last_diag_.trust_window_mean = trust_metrics.trust_window_mean;
  last_diag_.accept_window_hits = trust_metrics.accept_window_hits;
  last_diag_.accepted_lidar_update = trust_metrics.accepted;
  last_diag_.blended_lidar_update = trust_metrics.blended;
  last_diag_.blend_alpha = trust_metrics.blend_alpha;
  last_diag_.map_commit_allowed = trust_metrics.map_commit_allowed;
  last_diag_.objective_improved = trust_metrics.objective_improved;
  last_diag_.wheel_safe = trust_metrics.wheel_safe;
  last_diag_.ground_safe = trust_metrics.ground_safe;
  last_diag_.rms_safe = trust_metrics.rms_safe;
  last_diag_.window_consistent = trust_metrics.window_consistent;
  last_diag_.reject_reason_code = trust_metrics.reject_reason_code;
  last_diag_.accept_reason_code = trust_metrics.accept_reason_code;
  last_diag_.commit_reason_code = trust_metrics.commit_reason_code;
  last_diag_.wheel_cost_prior = trust_metrics.wheel_cost_prior;
  last_diag_.wheel_cost_post = trust_metrics.wheel_cost_post;
  last_diag_.ground_cost_prior = trust_metrics.ground_cost_prior;
  last_diag_.ground_cost_post = trust_metrics.ground_cost_post;
  last_diag_.pose_delta_norm = trust_metrics.pose_delta_norm;
  last_diag_.yaw_delta_abs = trust_metrics.yaw_delta_abs;
  last_diag_.obs_min_eig = trust_metrics.obs_min_eig;
  last_diag_.obs_cond_ratio = trust_metrics.obs_cond_ratio;

  if (config_.iekf.enable_ext_calibration) {
    update_extrinsic_from_state();
    if (frame_count_ % 50 == 0) {
      SPDLOG_INFO("[EXT_STATE] frame={} ext_yaw={:.6f}rad ext_xy=[{:.6f},{:.6f}]m "
                  "P_ext_yaw={:.6f} P_ext_x={:.6f} P_ext_y={:.6f}",
                  frame_count_,
                  static_cast<double>(state_.ext_delta_yaw),
                  static_cast<double>(state_.ext_delta_xy(0)),
                  static_cast<double>(state_.ext_delta_xy(1)),
                  static_cast<double>(state_.covariance(kLwoExtYawIdx, kLwoExtYawIdx)),
                  static_cast<double>(state_.covariance(kLwoExtXyIdx, kLwoExtXyIdx)),
                  static_cast<double>(state_.covariance(kLwoExtXyIdx + 1, kLwoExtXyIdx + 1)));
    }
  }

  // Inflate P_yaw when correspondences are few so the next good frame
  // can correct accumulated drift.
  if (last_corr_count_ < config_.min_correspondences) {
    state_.covariance(kLwoRotIdx + 2, kLwoRotIdx + 2) +=
        config_.low_corr_heading_inflation;
    low_corr_frames_++;
    if (frame_count_ % 10 == 0 || low_corr_frames_ <= 3) {
      SPDLOG_INFO("LWO frame={} LOW_CORRS={} P_yaw_inflated={:.6f} "
                  "consecutive={}",
                  frame_count_, last_corr_count_,
                  static_cast<double>(
                      state_.covariance(kLwoRotIdx + 2, kLwoRotIdx + 2)),
                  low_corr_frames_);
    }
    last_diag_.low_corr             = true;
    last_diag_.low_corr_consecutive = low_corr_frames_;
  } else {
    low_corr_frames_ = 0;
    last_diag_.low_corr             = false;
    last_diag_.low_corr_consecutive = 0;
  }

  // ---- Covariance monitoring ------------------------------------------------
  if (frame_count_ % 50 == 0) {
    const auto P = state_.covariance.diagonal();
    spdlog::info("[LWO] frame={} P_diag: rot=[{:.4f},{:.4f},{:.4f}] "
                 "pos=[{:.4f},{:.4f},{:.4f}] vel=[{:.4f},{:.4f},{:.4f}] "
                 "s={:.6f} bias=[{:.6f},{:.6f}]",
                 frame_count_,
                 static_cast<double>(P(0)),  static_cast<double>(P(1)),  static_cast<double>(P(2)),
                 static_cast<double>(P(3)),  static_cast<double>(P(4)),  static_cast<double>(P(5)),
                 static_cast<double>(P(6)),  static_cast<double>(P(7)),  static_cast<double>(P(8)),
                 static_cast<double>(P(9)),
                 static_cast<double>(P(10)), static_cast<double>(P(11)));
  }

  // Debug: covariance diagonal
  if (config_.enable_debug_log) {
    const auto P = state_.covariance.diagonal();
    spdlog::info("[LWO-DBG] frame={} P: rot=[{:.6f},{:.6f},{:.6f}] "
                 "pos=[{:.6f},{:.6f},{:.6f}] vel=[{:.6f},{:.6f},{:.6f}] "
                 "s={:.8f} bias=[{:.8f},{:.8f}]",
                 frame_count_,
                 static_cast<double>(P(0)),  static_cast<double>(P(1)),  static_cast<double>(P(2)),
                 static_cast<double>(P(3)),  static_cast<double>(P(4)),  static_cast<double>(P(5)),
                 static_cast<double>(P(6)),  static_cast<double>(P(7)),  static_cast<double>(P(8)),
                 static_cast<double>(P(9)),
                 static_cast<double>(P(10)), static_cast<double>(P(11)));
  }

  // ---- Path length tracking ------------------------------------------------
  if (frame_count_ > 0) {
    cumulative_path_length_ += (state_.position - prev_position_).norm();
  }
  prev_position_ = state_.position;

  // --- Post-IEKF anchor yaw correction (uses frozen anchor map) ---
  anchor_corrected_this_frame_ = false;
  {
    const auto t_anchor_yaw = do_usage ? Clock::now() : Clock::time_point{};
    apply_anchor_yaw_correction(processed);
    if (do_usage) {
      last_usage_.anchor_yaw_ms = std::chrono::duration<float, std::milli>(
          Clock::now() - t_anchor_yaw).count();
    }
  }

  // --- Map update ---
  // Map commit is stricter than pose publish: only trusted LiDAR updates
  // are allowed to modify the surfel map.
  bool map_updated = false;
  const auto t_map_start = do_usage ? Clock::now() : Clock::time_point{};
  const bool bootstrap_phase = (frame_count_ <= config_.bootstrap_frames);
  if (trust_metrics.map_commit_allowed || bootstrap_phase) {
    core::PointCloud world_cloud = transform_to_world(processed);
    surfel_map_.update(world_cloud, state_.position);
    map_updated = true;
    if (bootstrap_phase && !trust_metrics.map_commit_allowed) {
      SPDLOG_INFO("LWO frame={} BOOTSTRAP_MAP_COMMIT (trust={:.3f} corrs={})",
                  frame_count_,
                  static_cast<double>(trust_metrics.trust_score),
                  last_corr_count_);
    }
  } else if (frame_count_ % 10 == 0) {
    SPDLOG_INFO("LWO frame={} SKIP_MAP_UPDATE trust={:.3f} corrs={}",
                frame_count_,
                static_cast<double>(trust_metrics.trust_score),
                last_corr_count_);
  }
  if (do_usage) {
    last_usage_.map_update_ms = std::chrono::duration<float, std::milli>(
        Clock::now() - t_map_start).count();
  }
  last_diag_.skip_map_update = !map_updated;

  // Debug: map stats
  if (config_.enable_debug_log) {
    spdlog::info("[LWO-DBG] frame={} map: l0={} l1={} map_updated={}",
                 frame_count_,
                 surfel_map_.l0_count(), surfel_map_.l1_count(),
                 map_updated ? 1 : 0);
  }

  last_lidar_time_ = timestamp;

  // Collect final state/covariance/map diagnostics
  {
    last_diag_.vel_mag      = state_.velocity.norm();
    last_diag_.wheel_scale  = state_.wheel_scale;
    last_diag_.gyro_bias_z  = state_.wheel_gyro_bias(1);
    const auto& P = state_.covariance.diagonal();
    last_diag_.P_rot_x  = P(kLwoRotIdx + 0);
    last_diag_.P_rot_y  = P(kLwoRotIdx + 1);
    last_diag_.P_rot_z  = P(kLwoRotIdx + 2);
    last_diag_.P_pos_x  = P(kLwoPosIdx + 0);
    last_diag_.P_pos_y  = P(kLwoPosIdx + 1);
    last_diag_.P_pos_z  = P(kLwoPosIdx + 2);
    last_diag_.P_vel_x  = P(kLwoVelIdx + 0);
    last_diag_.P_vel_y  = P(kLwoVelIdx + 1);
    last_diag_.P_scale  = P(kLwoScaleIdx);
    last_diag_.map_l0   = surfel_map_.l0_count();
    last_diag_.map_l1   = surfel_map_.l1_count();
  }

  // Collect total timing and system resources
  if (do_usage) {
    last_usage_.total_ms = std::chrono::duration<float, std::milli>(
        Clock::now() - t_start).count();
    collect_system_resources();
  }

  frame_count_++;

  // Clear state history after LiDAR processing.
  state_history_.clear();

  return true;
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

LwoState LwoEstimator::current_state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

LwoEstimator::FrameDiagnostics LwoEstimator::last_diagnostics() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_diag_;
}

LwoEstimator::FrameUsage LwoEstimator::last_usage() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_usage_;
}

void LwoEstimator::set_queue_depth(int depth) {
  // No mutex needed: called from processing thread before feed_lidar
  pending_queue_depth_ = depth;
}

void LwoEstimator::accumulate_wheel_ms(float ms) {
  // No mutex needed: called from same processing thread as feed_wheel_delta
  wheel_ms_accumulator_ += ms;
}

void LwoEstimator::collect_system_resources() {
  using Clock = std::chrono::steady_clock;

  // RSS from /proc/self/statm (page units)
  {
    std::ifstream statm("/proc/self/statm");
    if (statm.is_open()) {
      long pages_total = 0, pages_resident = 0;
      statm >> pages_total >> pages_resident;
      last_usage_.rss_mb = static_cast<float>(pages_resident * getpagesize())
                           / (1024.0f * 1024.0f);
    }
  }

  // CPU% from /proc/self/stat
  {
    std::ifstream stat_file("/proc/self/stat");
    if (stat_file.is_open()) {
      std::string line;
      std::getline(stat_file, line);
      // Skip past the comm field (enclosed in parens) to avoid space issues.
      const auto paren_end = line.rfind(')');
      if (paren_end != std::string::npos) {
        std::istringstream iss(line.substr(paren_end + 2));
        std::string field;
        // Fields after comm+state: ppid(2) pgrp(3) session(4) tty_nr(5)
        // tpgid(6) flags(7) minflt(8) cminflt(9) majflt(10) cmajflt(11)
        // utime(12) stime(13) — skip 11 before utime/stime
        for (int i = 0; i < 11; ++i) iss >> field;
        long utime = 0, stime = 0;
        iss >> utime >> stime;
        const long total_ticks = utime + stime;

        const auto now = Clock::now();
        if (cpu_tracking_initialized_) {
          const long dticks = total_ticks - prev_cpu_ticks_;
          const float dwall = std::chrono::duration<float>(
              now - prev_cpu_wall_).count();
          if (dwall > 0.001f) {
            const float dcpu = static_cast<float>(dticks)
                               / static_cast<float>(sysconf(_SC_CLK_TCK));
            last_usage_.cpu_percent = 100.0f * dcpu / dwall;
          }
        }
        prev_cpu_ticks_ = total_ticks;
        prev_cpu_wall_  = now;
        cpu_tracking_initialized_ = true;
      }
    }
  }
}

bool LwoEstimator::initialized() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return initialized_;
}

int LwoEstimator::frame_count() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return frame_count_;
}

std::vector<core::Surfel> LwoEstimator::map_surfels() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return surfel_map_.all_surfels();
}

const core::SurfelMap& LwoEstimator::surfel_map() const {
  return surfel_map_;
}

// ---------------------------------------------------------------------------
// reset
// ---------------------------------------------------------------------------

void LwoEstimator::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  state_.reset();
  // Reset corrected extrinsic to initial (uncorrected) value.
  T_body_lidar_current_ = config_.T_body_lidar;
  initialized_ = false;
  frame_count_ = 0;
  first_lidar_frame_ = true;
  last_wheel_time_ = 0.0;
  last_lidar_time_ = 0.0;
  last_vx_enc_ = 0.0f;
  last_omega_z_enc_ = 0.0f;
  cumulative_path_length_ = 0.0f;
  prev_position_ = Eigen::Vector3f::Zero();
  surfel_map_.reset();
  pko_.reset();
  state_history_.clear();
  accept_history_.clear();
  // Anchor map reset
  anchor_map_.reset();
  anchor_map_frozen_ = false;
  anchor_origin_ = Eigen::Vector3f::Zero();
  // Extended reference map reset
  max_dist_from_origin_ = 0.0f;
  outward_max_p_yaw_ = 0.0f;
  anchor_correction_ema_ = 0.0f;
  anchor_correction_count_ = 0;
  cumulative_anchor_yaw_ = 0.0f;
  was_outside_anchor_ = false;
}

}  // namespace lwo
}  // namespace tof_slam
