// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// lio_estimator.cpp — Thin orchestrator composing all core LIO components.

#include "tof_slam/frontend/estimator/lio_estimator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <random>

#include <Eigen/SVD>

#include <spdlog/spdlog.h>

#include "tof_slam/frontend/diag/boundary_hash.hpp"
#include "tof_slam/frontend/diag/prescan_trace.hpp"
#include "tof_slam/frontend/diag/upstream_trace.hpp"
#include "tof_slam/frontend/estimator/correspondence_finder.hpp"
#include "tof_slam/frontend/estimator/gravity_init.hpp"
#include "tof_slam/frontend/estimator/imu_propagator.hpp"
#include "tof_slam/frontend/filter/range_filter.hpp"
#include "tof_slam/frontend/filter/stride_filter.hpp"
#include "tof_slam/frontend/filter/voxel_grid.hpp"

namespace tof_slam {
namespace core {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

LioEstimator::LioEstimator()
    : LioEstimator(Config{}) {}

LioEstimator::LioEstimator(const Config& config)
    : config_(config),
      process_noise_(build_process_noise()),
      surfel_map_(config.surfel_map),
      point_voxel_map_(config.point_voxel_map),
      pko_(config.pko) {
}

LioEstimator::~LioEstimator() = default;

// ---------------------------------------------------------------------------
// build_process_noise
// ---------------------------------------------------------------------------

StateCovariance LioEstimator::build_process_noise() const {
  // Q layout matches the reference Estimator:
  //   [0:3]  = gyro noise
  //   [3:6]  = acc noise
  //   [6:9]  = acc noise  (velocity block driven by acc noise)
  //   [9:12] = gyro bias random walk
  //  [12:15] = acc bias random walk
  //  [15:18] = gravity noise
  StateCovariance Q = StateCovariance::Identity();
  Q.block<3, 3>(0, 0) *=
      config_.gyro_noise_std * config_.gyro_noise_std;
  Q.block<3, 3>(3, 3) *=
      config_.acc_noise_std * config_.acc_noise_std;
  Q.block<3, 3>(6, 6) *=
      config_.acc_noise_std * config_.acc_noise_std;
  Q.block<3, 3>(9, 9) *=
      config_.gyro_bias_noise_std * config_.gyro_bias_noise_std;
  Q.block<3, 3>(12, 12) *=
      config_.acc_bias_noise_std * config_.acc_bias_noise_std;
  Q.block<3, 3>(15, 15) *=
      config_.gravity_noise_std * config_.gravity_noise_std;
  return Q;
}

// ---------------------------------------------------------------------------
// initialize
// ---------------------------------------------------------------------------

bool LioEstimator::initialize(const std::vector<ImuMeasurement>& imu_buffer) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (initialized_) {
    return false;
  }

  GravityInitResult result = initialize_gravity(imu_buffer);
  if (!result.success) {
    return false;
  }

  state_ = result.initial_state;
  imu_acc_scale_ = result.imu_acc_scale;
  initialized_ = true;
  first_lidar_frame_ = true;
  frame_count_ = 0;

  // Set last IMU time from the buffer so that the first feed_imu computes a
  // valid dt.
  if (!imu_buffer.empty()) {
    last_imu_time_ = imu_buffer.back().timestamp;
  }

  return true;
}

bool LioEstimator::initialize(const GravityInitResult& result) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (initialized_) {
    return false;
  }

  if (!result.success) {
    return false;
  }

  state_ = result.initial_state;
  imu_acc_scale_ = result.imu_acc_scale;
  initialized_ = true;
  first_lidar_frame_ = true;
  frame_count_ = 0;

  // Seed last_imu_time_ from the init buffer's last timestamp so the very
  // first feed_imu() computes a valid dt instead of dt ≈ 1.7e9 (Unix epoch).
  if (result.last_imu_timestamp > 0.0) {
    last_imu_time_ = result.last_imu_timestamp;
  }

  // Open diagnostics CSV for real-time monitoring.
  if (!config_.diagnostics_log_path.empty()) {
    diagnostics_.open_log(config_.diagnostics_log_path);
  }

  return true;
}

// ---------------------------------------------------------------------------
// feed_imu
// ---------------------------------------------------------------------------

void LioEstimator::feed_imu(const ImuMeasurement& imu) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initialized_) {
    return;
  }

  // Compute dt
  double dt_d = imu.timestamp - last_imu_time_;
  if (dt_d <= 0.0 || dt_d > 1.0) {
    // Skip propagation for invalid time steps, but ALWAYS advance
    // last_imu_time_ to allow recovery from timestamp discontinuities.
    // Without this update, a single large jump permanently disables IMU.
    last_imu_time_ = imu.timestamp;
    return;
  }
  last_imu_time_ = imu.timestamp;
  float dt = static_cast<float>(dt_d);

  // Apply accelerometer scale (g-unit correction from gravity init)
  ImuMeasurement scaled_imu = imu;
  scaled_imu.accel = imu.accel * imu_acc_scale_;

  state_ = propagate_imu(state_, scaled_imu, dt, process_noise_);

  // Buffer IMU for Fixed-Lag Smoother preintegration.
  if (config_.enable_fixed_lag_smoother) {
    fls_imu_buffer_.push_back(scaled_imu);
  }

  // Store state snapshot for undistortion
  StampedPose sp;
  sp.timestamp = imu.timestamp;
  sp.pose = state_.pose();
  state_history_.push_back(sp);

  // Bound history size to avoid unbounded memory growth.
  while (state_history_.size() > 2000) {
    state_history_.pop_front();
  }
}

// ---------------------------------------------------------------------------
// Preprocessing helpers
// ---------------------------------------------------------------------------

PointCloud LioEstimator::preprocess_scan(const PointCloud& scan,
                                          double timestamp) {
  using Clock = std::chrono::high_resolution_clock;
  const bool do_timing = config_.enable_debug_timing;
  PreprocessTiming pt;
  pt.n_raw = static_cast<int>(scan.size());

  // Task #70 U1a Phase-1 instrumentation.  Env-gated dump of state_history_
  // size + boundary IMU timestamps + current state snapshot at every LIDAR
  // frame entry.  Goal: diff across 10 determinism-run CSVs to find the
  // first frame where the trajectory fed into undistort_scan diverges.
  if (tof_slam::frontend::diag::PrescanTrace::enabled()) {
    const std::size_t sh_size = state_history_.size();
    const double sh_first_ts =
        sh_size > 0 ? state_history_.front().timestamp : 0.0;
    const double sh_last_ts =
        sh_size > 0 ? state_history_.back().timestamp : 0.0;
    const Eigen::Quaternionf q(state_.rotation);
    tof_slam::frontend::diag::PrescanTrace::instance().log(
        tof_slam::frontend::diag::PrescanTrace::instance_run_id(),
        frame_count_, timestamp, sh_size, sh_first_ts, sh_last_ts,
        state_.position.x(), state_.position.y(), state_.position.z(),
        q.x(), q.y(), q.z(), q.w(),
        state_.gyro_bias.x(), state_.gyro_bias.y(), state_.gyro_bias.z(),
        state_.acc_bias.x(), state_.acc_bias.y(), state_.acc_bias.z(),
        last_imu_time_);
  }

  // B0 — Raw scan bytes entering the estimator (PV-3 attribution).
  if (tof_slam::frontend::diag::BoundaryLogger::enabled()) {
    const auto& pts = scan.points();
    const auto view = tof_slam::frontend::diag::make_byte_view(
        pts.data(), pts.size() * sizeof(Point3D));
    tof_slam::frontend::diag::BoundaryLogger::instance().log(
        tof_slam::frontend::diag::current_frame_idx(),
        tof_slam::frontend::diag::BoundaryId::B0_RawScan,
        view,
        static_cast<double>(scan.size()));
  }

  // 1. Stride filter
  const auto t0 = do_timing ? Clock::now() : Clock::time_point{};
  PointCloud after_stride = stride_filter(scan, config_.stride);
  if (do_timing) {
    pt.stride_ms = std::chrono::duration<float, std::milli>(
        Clock::now() - t0).count();
  }
  pt.n_after_stride = static_cast<int>(after_stride.size());

  // 2. Voxel grid filter
  const auto t1 = do_timing ? Clock::now() : Clock::time_point{};
  VoxelGridConfig vg_cfg;
  vg_cfg.leaf_size = config_.voxel_leaf_size;
  vg_cfg.enable_planarity_filter = false;
  PointCloud after_voxel = voxel_grid_filter(after_stride, vg_cfg);
  if (do_timing) {
    pt.voxel_ms = std::chrono::duration<float, std::milli>(
        Clock::now() - t1).count();
  }
  pt.n_after_voxel = static_cast<int>(after_voxel.size());

  // Log offset_time statistics (first 3 scans only) for undistortion debugging.
  {
    if (scan_diag_count_ < 3 && !after_voxel.empty()) {
      float min_ot = after_voxel[0].offset_time;
      float max_ot = after_voxel[0].offset_time;
      for (const auto& p : after_voxel) {
        if (p.offset_time < min_ot) min_ot = p.offset_time;
        if (p.offset_time > max_ot) max_ot = p.offset_time;
      }
      SPDLOG_INFO("SCAN_DIAG[{}] n_pts={} offset_time=[{:.6f}, {:.6f}] "
                  "scan_ts={:.6f} hist_size={}",
                  scan_diag_count_, after_voxel.size(), min_ot, max_ot,
                  timestamp, state_history_.size());
      scan_diag_count_++;
    }
  }

  // 3a. Range filter
  const auto t3_pre = do_timing ? Clock::now() : Clock::time_point{};
  PointCloud after_range_pre =
      range_filter(after_voxel, config_.min_range, config_.max_range);
  if (do_timing) {
    // Charge range time here temporarily; overwritten after undistort if needed.
    pt.range_ms = std::chrono::duration<float, std::milli>(
        Clock::now() - t3_pre).count();
  }

  // 3b. Undistortion (optional)
  const auto t2 = do_timing ? Clock::now() : Clock::time_point{};
  PointCloud after_undistort = after_range_pre;
  if (config_.enable_undistortion && state_history_.size() >= 2) {
    std::vector<StampedPose> traj(state_history_.begin(),
                                   state_history_.end());
    after_undistort =
        undistort_scan(after_range_pre, traj, timestamp, config_.T_body_lidar);
  }
  if (do_timing) {
    pt.undistort_ms = std::chrono::duration<float, std::milli>(
        Clock::now() - t2).count();
  }

  // 4. (No second range filter needed — range was already applied above.)
  //    Re-measure timing as total of range + undistort passes.
  PointCloud after_range = after_undistort;
  pt.n_processed = static_cast<int>(after_range.size());

  // Store timing for feed_lidar to collect.
  last_preprocess_timing_ = pt;

  // B1 — Post-preprocess scan (deskewed + feature-selected).
  if (tof_slam::frontend::diag::BoundaryLogger::enabled()) {
    const auto& pts = after_range.points();
    const auto view = tof_slam::frontend::diag::make_byte_view(
        pts.data(), pts.size() * sizeof(Point3D));
    tof_slam::frontend::diag::BoundaryLogger::instance().log(
        tof_slam::frontend::diag::current_frame_idx(),
        tof_slam::frontend::diag::BoundaryId::B1_Preprocessed,
        view,
        static_cast<double>(after_range.size()));
  }

  return after_range;
}

PointCloud LioEstimator::transform_to_world(const PointCloud& scan) const {
  // T_world_lidar = T_world_body * T_body_lidar
  Se3 T_world_lidar = state_.pose() * config_.T_body_lidar;
  return scan.transformed_copy(T_world_lidar);
}

// ---------------------------------------------------------------------------
// feed_lidar
// ---------------------------------------------------------------------------

bool LioEstimator::feed_lidar(const PointCloud& scan, double timestamp) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initialized_) {
    return false;
  }

  // Deduplicate: skip if this timestamp is too close to the previous frame.
  // DDS best-effort QoS can deliver the same message multiple times.
  // At 10Hz LiDAR, min expected dt is ~100ms. Use 50ms guard.
  if (frame_count_ > 0 && (timestamp - last_lidar_time_) < 0.05) {
    return false;
  }

  // PV-3 attribution: publish the current frame index for all hooks
  // (B0..B5) to read via the thread_local accessor. Uses frame_count_
  // as the monotonic per-frame id. Placed AFTER the dedup guard so that
  // dropped frames do not pollute the frame_idx stream. Observational only.
  tof_slam::frontend::diag::set_current_frame_idx(
      static_cast<std::uint64_t>(frame_count_));

  using Clock = std::chrono::high_resolution_clock;
  const auto t_start = Clock::now();

  // Preprocess
  PointCloud processed = preprocess_scan(scan, timestamp);

  if (processed.empty()) {
    return false;
  }

  const auto t_preprocess = Clock::now();

  // First frame: just add points to the map, no IEKF update.
  if (first_lidar_frame_) {
    PointCloud world_cloud = transform_to_world(processed);
    surfel_map_.update(world_cloud, state_.position);
    point_voxel_map_.update(world_cloud, state_.position);
    first_lidar_frame_ = false;
    last_lidar_time_ = timestamp;
    last_corrected_position_ = state_.position;
    frame_count_++;
    state_history_.clear();
    return true;  // Map was populated, but no IEKF correction.
  }

  // Pre-flush all dirty surfels so that get_surfel() during IEKF does
  // not trigger lazy PCA recomputation (avoids redundant work across
  // multiple outer iterations and removes the dirty-flag branch).
  surfel_map_.flush_dirty();

  // --- IEKF ---
  IekfLoopResult iekf_result = run_iekf_loop(processed, timestamp);

  const auto t_iekf = Clock::now();

  // --- Map update ---
  PointCloud world_cloud = transform_to_world(processed);

  // Task #70 U1 upstream bisection — env-gated per-point trace of
  // (body_p, world_p, state_pose) at the moment world_cloud is handed
  // to update_map(). Disabled path is a single cached-bool branch per
  // frame (see ringbuf_trace.hpp off-path invariant). Classifies the
  // Dark01 2-class split into:
  //   U1a — bp differs between classes → preprocess / IMU undistortion race
  //   U1b — bp identical, state differs → IEKF inner-iteration race
  //   U1c — bp + state identical, wp differs → IEEE-754 impossible
  //          under single-thread scalar Eigen (tooling bug)
  {
    namespace tsdiag = tof_slam::frontend::diag;
    if (tsdiag::UpstreamTrace::enabled()) {
      const Se3 T_world_lidar = state_.pose() * config_.T_body_lidar;
      const Eigen::Matrix3f R_wl = T_world_lidar.rotation_matrix();
      const Eigen::Quaternionf q_wl(R_wl);
      const Eigen::Vector3f   t_wl = T_world_lidar.translation();
      const std::string& run_id = tsdiag::UpstreamTrace::instance_run_id();
      const size_t n_pts = std::min(processed.size(), world_cloud.size());
      for (size_t i = 0; i < n_pts; ++i) {
        const Eigen::Vector3f bp = processed[i].to_eigen();
        const Eigen::Vector3f wp = world_cloud[i].to_eigen();
        tsdiag::UpstreamTrace::instance().log(
            run_id, static_cast<int>(frame_count_), static_cast<int>(i),
            bp.x(), bp.y(), bp.z(),
            wp.x(), wp.y(), wp.z(),
            t_wl.x(), t_wl.y(), t_wl.z(),
            q_wl.x(), q_wl.y(), q_wl.z(), q_wl.w());
      }
    }
  }

  const auto t_map_insert = Clock::now();
  update_map(world_cloud, iekf_result);

  // --- Fixed-Lag Smoother (Tier 4) ---
  if (config_.enable_fixed_lag_smoother) {
    // Snapshot BEFORE FLS mutates state_.
    PointCloud snapshot_scan = processed;
    const LioState snapshot_state = state_;
    const StateCovariance snapshot_P = state_.covariance;

    // FLS uses correspondences from forward IEKF (for diagnostics only).
    run_fixed_lag_smoother({});

    // Store for next frame's FLS.
    prev_processed_scan_ = std::move(snapshot_scan);
    prev_state_posterior_ = snapshot_state;
    prev_P_posterior_ = snapshot_P;
    prev_frame_valid_ = true;
  }

  const auto t_map = Clock::now();

  last_lidar_time_ = timestamp;

  // --- Diagnostics and timing ---
  log_frame_diagnostics(iekf_result, processed, timestamp,
                        t_start, t_preprocess, t_iekf, t_map_insert, t_map);

  frame_count_++;

  // ---- T5.4-R8 Two-Stage Scene Classifier ----
  // Stage A (frame_count_ == stage_a_frame): instantaneous rho_1 gate splits
  //   INDOOR / CLASS_D / defer. Eliminates VI03 warmup contamination.
  // Stage B (frame_count_ == stage_b_window_end): R7.1 8-way decision tree
  //   over [stage_b_window_start, stage_b_window_end) aggregation, only if
  //   Stage A deferred.
  // Frame-numbering note: classifier hook fires AFTER frame_count_++ (above).
  // CSV frame=N ↔ runtime frame_count_=N+1. Defaults are runtime values.
  if (classifier_cfg_.enable && !classifier_state_.locked) {
    const int fc = static_cast<int>(frame_count_);

    // STAGE A — frame-1 instantaneous hard gate (R8.1 §1).
    if (!classifier_state_.stage_a_complete &&
        fc == classifier_cfg_.stage_a_frame) {
      const auto cls_opt = classify_stage_a(iekf_result.total_corrs,
                                            surfel_map_.l1_count(),
                                            iekf_result.max_degen,
                                            static_cast<float>(surfel_map_.cos2_mean()),
                                            classifier_cfg_);
      classifier_state_.stage_a_complete  = true;
      classifier_state_.stage_a_frame_idx = fc;
      const double rho_1 =
          static_cast<double>(iekf_result.total_corrs) /
          std::max<double>(static_cast<double>(surfel_map_.l1_count()), 1.0);
      if (cls_opt.has_value()) {
        classifier_state_.locked       = true;
        classifier_state_.locked_class = *cls_opt;
        classifier_state_.lock_frame   = fc;
        apply_template_(*cls_opt);
        SPDLOG_INFO("[classifier] STAGE_A LOCK frame={} class={} rho_1={:.4f}",
                    fc, scene_class_name(*cls_opt).data(), rho_1);
      } else {
        SPDLOG_INFO("[classifier] STAGE_A DEFER frame={} rho_1={:.4f}",
                    fc, rho_1);
      }
    }

    // STAGE B — aggregation window + lock at stage_b_window_end (R8.1 §3).
    if (classifier_state_.stage_a_complete && !classifier_state_.locked) {
      if (fc >= classifier_cfg_.stage_b_window_start &&
          fc <  classifier_cfg_.stage_b_window_end) {
        classifier_state_.warmup.update(
            iekf_result.total_corrs,
            surfel_map_.l1_count(),
            static_cast<double>(surfel_map_.cos2_mean()),
            iekf_result.max_degen,
            static_cast<double>(state_.velocity.norm()));
        classifier_state_.stage_b_started = true;
      }
      if (fc == classifier_cfg_.stage_b_window_end) {
        const SceneClass cls = classify_stage_b(classifier_state_.warmup,
                                                classifier_cfg_);
        classifier_state_.locked       = true;
        classifier_state_.locked_class = cls;
        classifier_state_.lock_frame   = fc;
        apply_template_(cls);
        SPDLOG_INFO("[classifier] STAGE_B LOCK frame={} class={} window=[{},{})",
                    fc, scene_class_name(cls).data(),
                    classifier_cfg_.stage_b_window_start,
                    classifier_cfg_.stage_b_window_end);
      }
    }
  }

  // L2 diagnostic log (every 100 frames when enabled).
  if (config_.surfel_map.enable_l2_correspondences && frame_count_ % 100 == 0) {
    SPDLOG_INFO("L2: l2_count={}", surfel_map_.l2_count());
  }

  // Clear state history after LiDAR processing.
  state_history_.clear();

  return true;
}

// ---------------------------------------------------------------------------
// apply_template_  — Scene classifier lock-time dispatch (T5.4-R7 F.4)
// ---------------------------------------------------------------------------

void LioEstimator::apply_template_(SceneClass cls) {
  const ClassTemplate& t = get_template(cls);

  // Phase A — IEKF correspondence params (LioEstimator::Config)
  config_.max_corr_per_l1            = t.max_corr_per_l1;
  config_.max_plane_distance         = t.max_plane_distance;
  config_.pvmap_k_neighbors          = t.pvmap_k_neighbors;
  config_.pvmap_sigma2_scale         = t.pvmap_sigma2_scale;
  // Static flags (mutated only here, never elsewhere)
  config_.enable_degen_pvmap_override = t.enable_degen_pvmap_override;
  config_.enable_cscf                 = t.enable_cscf;
  config_.cscf_kernel_bandwidth       = t.cscf_kernel_bandwidth;
  config_.enable_point_to_distribution = t.enable_geometric_covariance;

  // Phase B — SurfelMap setters (10 from F.1)
  surfel_map_.set_l0_ema_alpha_min(t.l0_ema_alpha_min);
  surfel_map_.set_alpha_degen_floor(t.alpha_degen_floor);
  surfel_map_.set_ema_gate_radius(t.ema_gate_radius);
  surfel_map_.set_l0_centroid_freeze_count(t.l0_centroid_freeze_count);
  surfel_map_.set_sigma2_age_scale(t.sigma2_age_scale);
  surfel_map_.set_pncg_threshold(t.pncg_threshold);
  surfel_map_.set_l2_noise_scale(t.l2_noise_scale);
  // CLASS_D-specific (VI03 P2D / geometric covariance mode)
  surfel_map_.set_enable_geometric_covariance(t.enable_geometric_covariance);
  surfel_map_.set_geometric_cov_min_eigenvalue(t.geometric_cov_min_eigenvalue);
  surfel_map_.set_geometric_cov_min_points(t.geometric_cov_min_points);

  // PVMap geometry: NOTE — T5.4-R7 F.3 accepts construction-time params leakage
  // (pvmap_voxel_size, pvmap_max_points_per_voxel) per architect Q12-NEW.
  // DK01 (voxel_size 0.5 vs per-seq 0.3) ≈ +2% leakage acceptable for EC3.
  // VI03 (max_pts 60 vs per-seq 20): F.5 EC2 will measure.

  // S13-B.A.6 Phase C — Path B: route P1 anisotropic-IEKF tuple per class.
  // Master gate: config_.iekf.anisotropic_iekf_router_enable (B.A.5).
  // When false (default), Phase C is a no-op: P1 fields read YAML defaults
  // (avia_outdoor.yaml leaves absent → all-OFF → legacy bit-identity).
  // When true, the class-conditional P1Tuple overrides whatever YAML loaded.
  // Only kT_CLEAN_DENSE sets non-default P1Tuple (cfg_19); 9 others are OFF
  // → 9 non-CLEAN_DENSE classes preserve legacy scalar IEKF path verbatim,
  // isolating V0 + Path A failure modes (architect §1.3).
  if (config_.iekf.anisotropic_iekf_router_enable) {
    config_.iekf.anisotropic_iekf_enable         = t.p1.anisotropic_iekf_enable;
    config_.iekf.anisotropic_iekf_scalar_shim    = t.p1.anisotropic_iekf_scalar_shim;
    config_.iekf.anisotropic_iekf_epsilon        = t.p1.anisotropic_iekf_epsilon;
    config_.iekf.anisotropic_iekf_rho_ref_avia   = t.p1.anisotropic_iekf_rho_ref_avia;
    config_.iekf.anisotropic_iekf_chi2_threshold = t.p1.anisotropic_iekf_chi2_threshold;
    config_.iekf.anisotropic_iekf_sigma_theta_sq = t.p1.anisotropic_iekf_sigma_theta_sq;
    config_.iekf.enable_range_inverse_weight     = t.p1.enable_range_inverse_weight;
    config_.iekf.range_inverse_ref               = t.p1.range_inverse_ref;
    config_.iekf.range_inverse_power             = t.p1.range_inverse_power;
    config_.iekf.range_inverse_min_ratio         = t.p1.range_inverse_min_ratio;
    SPDLOG_INFO("[P1-router] Phase C dispatch: class={} → P1 enable={} eps={} range_inv={}",
                static_cast<int>(cls),
                t.p1.anisotropic_iekf_enable ? "true" : "false",
                t.p1.anisotropic_iekf_epsilon,
                t.p1.enable_range_inverse_weight ? "ON" : "OFF");
  }

  // ---------------------------------------------------------------------------
  // R0.10 H4 — surgical pre-LOCK rebuild, OUTDOOR_DRIFT only (DK02-class).
  // ---------------------------------------------------------------------------
  // DK01 reaches this function via Stage B with cls=CLEAN_DENSE; the guard
  // ensures DK01 is byte-identical to R0.9 H3b. Mid-360 / NTU / indoor are
  // protected by the H3b is_avia_outdoor sensor guard upstream of Stage A.
  //
  // Why fire AFTER Phase A/B/C: the rebuild's recompute happens in
  // flush_dirty() next frame using the params that we just installed (tighter
  // OUTDOOR_DRIFT template) — not the stale loose unified params that
  // contaminated the L0 EMA centroids in frames [0..frame_count_].
  if (cls == SceneClass::OUTDOOR_DRIFT) {
    surfel_map_.rebuild_pre_lock_surfels(static_cast<int>(frame_count_));
    point_voxel_map_.clear_pre_lock_voxels(static_cast<int>(frame_count_));
    SPDLOG_INFO("[H4] OUTDOOR_DRIFT pre-LOCK rebuild fired at frame={}",
                frame_count_);
  }
}

// ---------------------------------------------------------------------------
// run_iekf_loop
// ---------------------------------------------------------------------------

IekfLoopResult LioEstimator::run_iekf_loop(const PointCloud& processed,
                                            double /*timestamp*/) {
  using Clock = std::chrono::high_resolution_clock;

  IekfLoopResult out;
  const bool do_timing = config_.enable_debug_timing;

  // P_imu (IMU-propagated covariance) is the CONSTANT prior for ALL
  // outer/inner iterations — matches reference exactly.
  const StateCovariance P_imu = state_.covariance;  // Freeze IMU prior.

  const int effective_max_outer = config_.iekf.max_outer_iters;

  // D1: Pre-compute corr_scan ONCE before outer loop (same every iteration
  // since `processed` is constant). Enables stable kNN cache indexing.
  const int max_corrs = config_.max_correspondences;
  PointCloud corr_scan;
  if (max_corrs > 0 && static_cast<int>(processed.size()) > max_corrs * 2) {
    const int target = static_cast<int>(max_corrs * 1.3f);
    const int sub_stride = std::max(1, static_cast<int>(processed.size()) / target);
    corr_scan.reserve(target + 1);
    for (size_t i = 0; i < processed.size(); i += sub_stride) {
      corr_scan.push_back(processed[i]);
    }
  }
  const PointCloud& scan_for_corr = corr_scan.empty() ? processed : corr_scan;

  // D1: kNN cache — allocated once, populated on outer=0, reused on outer>0.
  std::vector<CachedKnnEntry> knn_cache(scan_for_corr.size());
  bool knn_cache_populated = false;

  // Correspondence reuse buffer for cf_reuse_after_iter feature.
  std::vector<Correspondence> prev_corrs;

  // CSCF determinism fix: flush all dirty voxels before the IEKF loop.
  // gather_l1_surfels() reads surfel data under shared_lock WITHOUT checking
  // the dirty flag. If get_surfel() concurrently triggers recompute_surfel()
  // via const_cast (also under shared_lock), torn writes cause non-deterministic
  // kernel interpolation. Pre-flushing ensures no voxel is dirty when the OMP
  // parallel correspondence loop runs.
  if (config_.enable_cscf) {
    surfel_map_.flush_dirty();
  }

  for (int outer = 0; outer < effective_max_outer; ++outer) {
    ++out.total_outer;
    // Reset PKO for each outer iteration.
    pko_.reset();

    // Find correspondences at current state linearization point.
    const auto t_corr = Clock::now();

    // B2 — State prediction + world-frame query points (PV-3 attribution).
    //
    // Deviation vs architect spec: the architect document assumed the state
    // exposed a quaternion (4d) + 5x Vector3d = 19 doubles. The actual
    // LioState uses Matrix3f rotation + 5x Vector3f = 24 floats. We hash
    // the on-wire struct layout of the 24 state floats (96 bytes) followed
    // by the deterministic top-left 6x6 pose sub-covariance (36 doubles =
    // 288 bytes) pulled from state_.covariance. Aux = condition number of
    // the 6x6 pose sub-covariance via Eigen::JacobiSVD. Preview transform
    // is hashed as a second FNV-1a chain into h1 by feeding the transformed
    // point-cloud byte span.
    //
    // This hook fires only on the first outer iteration so that B2 captures
    // the predicted (pre-IEKF-correction) state exactly once per frame.
    if (outer == 0 &&
        tof_slam::frontend::diag::BoundaryLogger::enabled()) {
      // Pack the state members deterministically.
      // Layout: rotation(9 f) | position(3 f) | velocity(3 f) |
      //         gyro_bias(3 f) | acc_bias(3 f) | gravity(3 f) = 24 floats.
      float state_pack[24];
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          state_pack[r * 3 + c] = state_.rotation(r, c);
        }
      }
      for (int i = 0; i < 3; ++i) state_pack[9 + i]  = state_.position(i);
      for (int i = 0; i < 3; ++i) state_pack[12 + i] = state_.velocity(i);
      for (int i = 0; i < 3; ++i) state_pack[15 + i] = state_.gyro_bias(i);
      for (int i = 0; i < 3; ++i) state_pack[18 + i] = state_.acc_bias(i);
      for (int i = 0; i < 3; ++i) state_pack[21 + i] = state_.gravity(i);

      // Top-left 6x6 of the 18x18 covariance, serialized row-major as 36
      // doubles. Materialized into a local buffer so that Eigen's storage
      // order (column-major by default) does not leak into the byte view.
      double cov_pack[36];
      for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
          cov_pack[r * 6 + c] = state_.covariance(r, c);
        }
      }

      // Chain the state span and the cov span by concatenating into one
      // contiguous buffer (96 + 288 = 384 bytes).
      unsigned char chain[sizeof(state_pack) + sizeof(cov_pack)];
      std::memcpy(chain, state_pack, sizeof(state_pack));
      std::memcpy(chain + sizeof(state_pack), cov_pack, sizeof(cov_pack));

      // Preview transform of scan_for_corr — contributes to the hash as a
      // second chained span, NOT consumed downstream (strictly observational).
      PointCloud preview_world = transform_to_world(scan_for_corr);
      const auto& prev_pts = preview_world.points();

      // Compose: first FNV-1a over (state || cov), then continue feeding
      // the preview point bytes into the same accumulator.
      const std::uint64_t h1 = tof_slam::frontend::diag::fnv1a_64(
          tof_slam::frontend::diag::make_byte_view(chain, sizeof(chain)));
      // Absorb the preview bytes into h1 via a simple continuation: rehash
      // concatenation by XOR-feeding h1 as a seed through the aux path.
      // We cannot re-seed constexpr fnv1a_64 easily, so we log the combined
      // hash via a single logger call on a temporary vector. Cheapest:
      // compute a second hash and combine via a 64-bit mixing step.
      const auto prev_view = tof_slam::frontend::diag::make_byte_view(
          prev_pts.data(), prev_pts.size() * sizeof(Point3D));
      const std::uint64_t h2 = tof_slam::frontend::diag::fnv1a_64(prev_view);
      // Classic splitmix-style mix to combine h1 and h2 into one 64-bit hash.
      std::uint64_t h_combined = h1 ^ (h2 + 0x9e3779b97f4a7c15ULL +
                                       (h1 << 6) + (h1 >> 2));

      // Aux scalar: condition number of the 6x6 pose sub-covariance.
      Eigen::Matrix<double, 6, 6> P6 = state_.covariance.topLeftCorner<6, 6>();
      double cond_number = 0.0;
      {
        Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(P6);
        const auto& sv = svd.singularValues();
        const double smin = sv(5);
        const double smax = sv(0);
        cond_number = (smin > 1e-300) ? (smax / smin)
                                       : std::numeric_limits<double>::infinity();
      }

      // Log via the shared logger API by packing the combined hash as a
      // synthetic 8-byte span — this way log() still computes FNV-1a of
      // the packed bytes which equals h_combined by construction is NOT
      // guaranteed. Instead, bypass: log a byte view over a tiny buffer
      // that the logger will hash; then recover the aux channel.
      //
      // Simplest: just feed the combined hash through the logger as the
      // only byte content. The CSV column 3 will then be
      // fnv1a_64(h_combined_bytes) which is deterministic and carries the
      // combined signature.
      const auto combined_view = tof_slam::frontend::diag::make_byte_view(
          &h_combined, sizeof(h_combined));
      tof_slam::frontend::diag::BoundaryLogger::instance().log(
          tof_slam::frontend::diag::current_frame_idx(),
          tof_slam::frontend::diag::BoundaryId::B2_Predicted,
          combined_view,
          cond_number);
    }

    const float effective_max_plane_dist = config_.max_plane_distance;

    // DDPO: pass previous frame's degenerate directions to force PVMap
    // for surfel correspondences aligned with degenerate translation direction.
    const Eigen::Vector3f* ddpo_dirs = nullptr;
    int ddpo_n_dirs = 0;
    if (config_.enable_degen_pvmap_override && prev_frame_num_degen_trans_dirs_ > 0) {
      ddpo_dirs = prev_frame_degen_trans_dirs_;
      ddpo_n_dirs = prev_frame_num_degen_trans_dirs_;
    }

    HybridCorrespondenceParams corr_params;
    corr_params.max_plane_distance = effective_max_plane_dist;
    corr_params.pvmap_k_neighbors = config_.pvmap_k_neighbors;
    corr_params.pvmap_planarity_threshold = config_.pvmap_planarity_threshold;
    corr_params.max_corr_per_l1 = config_.max_corr_per_l1;

    corr_params.pvmap_sigma2_scale = config_.pvmap_sigma2_scale;
    corr_params.degen_trans_dirs = ddpo_dirs;
    corr_params.num_degen_trans_dirs = ddpo_n_dirs;
    corr_params.degen_pvmap_cos_threshold = config_.degen_pvmap_cos_threshold;
    corr_params.enable_point_to_distribution = config_.enable_point_to_distribution;
    corr_params.p2d_chi2_threshold = config_.p2d_chi2_threshold;
    corr_params.p2d_cov_reg_eps = config_.p2d_cov_reg_eps;
    corr_params.enable_cscf = config_.enable_cscf;
    corr_params.cscf_kernel_bandwidth = config_.cscf_kernel_bandwidth;
    corr_params.cf_omp_max_threads = config_.cf_omp_max_threads;
    corr_params.adaptive_threshold_divisor = config_.adaptive_threshold_divisor;
    // D1: kNN cache DISABLED — incompatible with voxel-based PVMap.
    // Delta-gating (0.4 * voxel_size) reduced regression from 34% to 19%
    // but did not eliminate it. Root cause: voxel-grid discrete neighbor
    // lookup is fundamentally sensitive to query-point shifts, unlike
    // FAST-LIO2's continuous ikd-tree (2.24m radius).
    // Tried: unconditional cache (ATE 0.153→0.205), delta-gated (→0.183).
    // Both deterministic (CV=0%) but with unacceptable ATE regression.

    // Correspondence finding: either ikd-tree per-iteration re-query
    // or legacy PVMap/SurfelMap hybrid with optional reuse.
    const int reuse_after = config_.iekf.cf_reuse_after_iter;
    std::vector<Correspondence> corrs;
    if (reuse_after > 0 && outer >= reuse_after && !prev_corrs.empty()) {
      corrs = prev_corrs;  // Reuse previous iteration's correspondences
    } else {
      corrs = find_correspondences_hybrid_select(
          state_, config_.T_body_lidar, scan_for_corr,
          surfel_map_, point_voxel_map_, corr_params);
      prev_corrs = corrs;  // Save for potential reuse
    }

    if (!knn_cache_populated) {
      knn_cache_populated = true;
    }

    // Final cap via Fisher-Yates partial shuffle if still over limit.
    if (max_corrs > 0 && static_cast<int>(corrs.size()) > max_corrs) {
      // corr_shuffle_rng_ is a member (fixed seed 42 for reproducibility)
      const int n = static_cast<int>(corrs.size());
      for (int i = 0; i < max_corrs; ++i) {
        std::uniform_int_distribution<int> dist(i, n - 1);
        std::swap(corrs[i], corrs[dist(corr_shuffle_rng_)]);
      }
      corrs.resize(max_corrs);
    }

    if (do_timing) {
      out.corr_find_ms += std::chrono::duration<float, std::milli>(
          Clock::now() - t_corr).count();
    }
    out.total_corrs = static_cast<int>(corrs.size());

    // Log every 100th frame or when correspondences are low (< 200).
    if (frame_count_ % 100 == 0 || corrs.size() < 200 || outer > 0) {
      SPDLOG_INFO("frame={} outer={} corrs={} mode=HS pos=[{:.2f},{:.2f},{:.2f}] "
                  "vel=[{:.3f},{:.3f},{:.3f}] l0={} l1={}",
                  frame_count_, outer, corrs.size(),
                  static_cast<double>(state_.position.x()),
                  static_cast<double>(state_.position.y()),
                  static_cast<double>(state_.position.z()),
                  static_cast<double>(state_.velocity.x()),
                  static_cast<double>(state_.velocity.y()),
                  static_cast<double>(state_.velocity.z()),
                  surfel_map_.l0_count(), surfel_map_.l1_count());
    }

    // Skip IEKF when too few correspondences.
    // Minimum 6 for the 6-DOF point-to-plane problem (was 20, relaxed for
    // non-repetitive scanning LiDARs like Livox Avia).
    constexpr int kMinCorrespondences = 6;
    if (static_cast<int>(corrs.size()) < kMinCorrespondences) {
      if (corrs.size() > 0) {
        SPDLOG_WARN("frame={} outer={} skipping IEKF: only {} corrs (min={})",
                     frame_count_, outer, corrs.size(), kMinCorrespondences);
      }
      // Aggressive velocity damping when IEKF is skipped to prevent runaway
      // IMU drift.  Non-repetitive LiDARs (Avia) may have periodic zero-corr
      // frames; we must survive these by keeping velocity bounded.
      constexpr float kVelDamping = 0.5f;  // Strong damping per frame
      state_.velocity *= kVelDamping;

      // Also apply hard velocity cap here (not just in post-IEKF section)
      constexpr float kMaxVelSkip = 5.0f;
      const float vel_n = state_.velocity.norm();
      if (vel_n > kMaxVelSkip) {
        state_.velocity *= (kMaxVelSkip / vel_n);
      }

      // Also inflate covariance to signal uncertainty growth.
      state_.covariance *= 1.1f;
      break;
    }

    // Snapshot position before IEKF for outer convergence check.
    const Eigen::Vector3f pos_before_outer = state_.position;

    LioState prior_for_iekf = state_;
    prior_for_iekf.covariance = P_imu;

    const auto t_iekf_inner = Clock::now();

    Pko* pko_ptr = config_.enable_pko ? &pko_ : nullptr;
    IcdrState* icdr_ptr = config_.iekf.enable_icdr ? &icdr_state_ : nullptr;
    IekfResult result = iekf_update(prior_for_iekf, corrs, config_.T_body_lidar,
                                    config_.iekf, pko_ptr, icdr_ptr);

    if (do_timing) {
      out.iekf_inner_ms += std::chrono::duration<float, std::milli>(
          Clock::now() - t_iekf_inner).count();
      out.jacobian_ms += result.jacobian_ms;
      out.huber_pko_ms += result.huber_pko_ms;
      out.build_info_ms += result.build_info_ms;
      out.solve_ms += result.solve_ms;
    }
    out.total_inner += result.total_iterations;
    if (result.num_degenerate_dirs > out.max_degen) {
      out.max_degen = result.num_degenerate_dirs;
    }
    // Keep degenerate translation directions from worst iteration.
    // Pair eigenvalue_ratio with the winning directions so severity
    // scaling in Eq.(4) uses the correct eigendecomp context.
    if (result.num_degen_trans_dirs > out.num_degen_trans_dirs) {
      out.num_degen_trans_dirs = result.num_degen_trans_dirs;
      for (int d = 0; d < result.num_degen_trans_dirs; ++d) {
        out.degen_trans_dirs[d] = result.degen_trans_dirs[d];
      }
      out.eigenvalue_ratio = result.eigenvalue_ratio;
    }
    // S12-B.A.3 DG-A: propagate per-channel signature (last iter overwrites).
    for (int c = 0; c < 3; ++c) {
      out.dg_a_rho[c]       = result.dg_a_rho[c];
      out.dg_a_d_trans[c]   = result.dg_a_d_trans[c];
      out.dg_a_cos_agree[c] = result.dg_a_cos_agree[c];
      out.dg_a_n_corr[c]    = result.dg_a_n_corr[c];
    }
    out.res_mean = result.res_mean;
    out.res_rms = result.res_rms;

    if (do_timing) {
      for (int i = 0; i < 6; ++i) out.eigenvalues[i] = result.eigenvalues[i];
    }

    if (frame_count_ % 100 == 0 || corrs.size() < 200 || outer > 0) {
      SPDLOG_INFO("  iekf iters={} converged={} pos_delta=[{:.4f},{:.4f},{:.4f}]",
                  result.total_iterations, result.converged,
                  static_cast<double>(result.state.position.x() - state_.position.x()),
                  static_cast<double>(result.state.position.y() - state_.position.y()),
                  static_cast<double>(result.state.position.z() - state_.position.z()));
    }

    // Take state mean from IEKF result (always, regardless of convergence).
    state_.rotation  = result.state.rotation;
    state_.position  = result.state.position;
    state_.velocity  = result.state.velocity;
    state_.gyro_bias = result.state.gyro_bias;
    state_.acc_bias  = result.state.acc_bias;
    state_.gravity   = result.state.gravity;

    // --- Adaptive outer convergence: early termination when state change is small ---
    if (outer >= 1 && config_.iekf.outer_convergence_threshold > 0.0f) {
      const float outer_pos_delta = (state_.position - pos_before_outer).norm();
      if (outer_pos_delta < config_.iekf.outer_convergence_threshold) {
        SPDLOG_INFO("  outer convergence: outer={} pos_delta={:.6f} < thresh={:.4f}, breaking",
                    outer, static_cast<double>(outer_pos_delta),
                    static_cast<double>(config_.iekf.outer_convergence_threshold));
        state_.covariance = result.state.covariance;
        out.converged = true;
        break;
      }
    }

    // B5 — Output state, captured before velocity-damping perturbs it
    // (PV-3 attribution).
    //
    // Layout matches B2: 24 floats of state (96 bytes) followed by
    // 36 doubles of top-left 6x6 pose covariance (288 bytes).
    // Aux = Frobenius norm of the 6x6 pose covariance block.
    // Fires only on the last outer iteration (outer == effective_max_outer-1
    // or on convergence — here we log every outer so the analyzer keeps the
    // final row per frame, which carries the true post-update state).
    if (tof_slam::frontend::diag::BoundaryLogger::enabled()) {
      float state_pack[24];
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          state_pack[r * 3 + c] = state_.rotation(r, c);
        }
      }
      for (int i = 0; i < 3; ++i) state_pack[9 + i]  = state_.position(i);
      for (int i = 0; i < 3; ++i) state_pack[12 + i] = state_.velocity(i);
      for (int i = 0; i < 3; ++i) state_pack[15 + i] = state_.gyro_bias(i);
      for (int i = 0; i < 3; ++i) state_pack[18 + i] = state_.acc_bias(i);
      for (int i = 0; i < 3; ++i) state_pack[21 + i] = state_.gravity(i);

      double cov_pack[36];
      for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
          cov_pack[r * 6 + c] = state_.covariance(r, c);
        }
      }

      unsigned char chain[sizeof(state_pack) + sizeof(cov_pack)];
      std::memcpy(chain, state_pack, sizeof(state_pack));
      std::memcpy(chain + sizeof(state_pack), cov_pack, sizeof(cov_pack));

      // Aux: Frobenius norm of the 6x6 pose covariance block.
      const Eigen::Matrix<double, 6, 6> P6 =
          state_.covariance.topLeftCorner<6, 6>();
      const double frob_cov = P6.norm();

      tof_slam::frontend::diag::BoundaryLogger::instance().log(
          tof_slam::frontend::diag::current_frame_idx(),
          tof_slam::frontend::diag::BoundaryId::B5_Output,
          tof_slam::frontend::diag::make_byte_view(chain, sizeof(chain)),
          frob_cov);
    }

    // ------------------------------------------------------------------
    // Velocity sanity: proportional damping based on correspondence quality.
    //
    // For non-repetitive LiDARs (Avia), correspondence count fluctuates
    // wildly.  When the IEKF has few constraints, the velocity estimate
    // is unreliable.  We apply graduated damping:
    //   corrs >= 100 : no damping (well-constrained)
    //   corrs  50-99 : mild (0.95x)
    //   corrs  20-49 : moderate (0.85x)
    //   corrs   6-19 : strong (0.7x)
    //
    // Additionally, cap velocity magnitude at a physically reasonable
    // limit (indoor walking: ~2 m/s, with margin 5 m/s).
    // ------------------------------------------------------------------
    {
      if (config_.enable_velocity_damping) {
        const int nc = out.total_corrs;
        float damp = 1.0f;
        if (nc < 20)       damp = 0.6f;
        else if (nc < 50)  damp = 0.8f;
        else if (nc < 100) damp = 0.9f;

        if (damp < 1.0f) {
          state_.velocity *= damp;
        }
      }

      // Hard velocity cap (prevents runaway during recovery).
      // Always active regardless of enable_velocity_damping.
      constexpr float kMaxVelNorm = 5.0f;  // m/s — generous margin
      const float vel_norm = state_.velocity.norm();
      if (vel_norm > kMaxVelNorm) {
        state_.velocity *= (kMaxVelNorm / vel_norm);
        SPDLOG_WARN("frame={} velocity capped: {:.2f} -> {:.2f} m/s",
                    frame_count_, static_cast<double>(vel_norm),
                    static_cast<double>(kMaxVelNorm));
      }
    }

    // Covariance update: apply whenever inner loop converges.
    // Phase E-3 fix: removed the `outer > 0` requirement.
    // When IEKF converges at outer=0, the posterior covariance is valid —
    // it means the IMU propagation was already close to the optimal state.
    // NOT updating P here causes covariance over-estimation (P_imu retained),
    // making the next frame's IEKF over-trust LiDAR correction.
    if (result.converged) {
      state_.covariance = result.state.covariance;
      out.converged = true;
      break;
    }
  }

  last_corrected_position_ = state_.position;

  // ICDR TIP: update per-frame state (position + frame counter) ONCE per
  // LiDAR frame, not inside the inner IEKF loop.
  if (config_.iekf.enable_icdr && icdr_state_.initialized) {
    icdr_state_.prev_position = state_.position;
    ++icdr_state_.frame_count;
  }

  // DDPO: save this frame's degenerate directions for next frame's use.
  prev_frame_num_degen_trans_dirs_ = out.num_degen_trans_dirs;
  for (int d = 0; d < out.num_degen_trans_dirs; ++d) {
    prev_frame_degen_trans_dirs_[d] = out.degen_trans_dirs[d];
  }

  // DARBF persistence counter: track consecutive degenerate frames.
  // Only activate DARBF freeze when degeneracy persists for min_persist frames.
  if (out.num_degen_trans_dirs > 0) {
    degen_persist_count_++;
  } else {
    if (degen_persist_count_ > 0 && (frame_count_ % 100 == 0 || degen_persist_count_ > 5)) {
      SPDLOG_INFO("DARBF_PERSIST: frame={} reset after {} consecutive degen frames",
                  frame_count_, degen_persist_count_);
    }
    degen_persist_count_ = 0;
  }

  return out;
}

// ---------------------------------------------------------------------------
// update_map
// ---------------------------------------------------------------------------

void LioEstimator::update_map(const PointCloud& world_cloud,
                               const IekfLoopResult& iekf_result) {
  // --- Surfel Keyframe Gate (Task #133 Iter 3) ---
  // Gate surfel map insertion on translation+rotation keyframe predicate.
  // When gate is OFF, returns {true, Off} → bit-identical to pre-gate.
  const SurfelInsertDecision surfel_decision = should_insert_surfel();

  if (surfel_decision.insert) {
    // Degeneracy-adaptive alpha for map insertion.
    // DECREASE alpha during degenerate frames to protect existing centroids.
    // Skip alpha manipulation when ring buffer is active (ring buffer
    // uses FIFO forgetting instead of EMA, so alpha is irrelevant).
    const float saved_alpha_min = surfel_map_.config().l0_ema_alpha_min;
    float alpha_eff = saved_alpha_min;

    // Eq.(4): Direction-selective alpha modulation.
    // When degenerate directions are available, per-voxel directional suppression
    // in add_point() replaces the old global scalar: α_eff = α_min·(1−|n·d|²).
    // The global scalar is only used as fallback when directions are unavailable.
    if (config_.enable_degeneracy_adaptive_alpha && iekf_result.max_degen > 0 &&
        saved_alpha_min > 0.0f) {
      if (iekf_result.num_degen_trans_dirs > 0) {
        // Eq.(4): per-voxel directional modulation handles selective suppression.
        // Pass eigenvalue_ratio for severity scaling (Eq.4 extension).
        surfel_map_.set_degen_state(iekf_result.degen_trans_dirs,
                                    iekf_result.num_degen_trans_dirs,
                                    iekf_result.eigenvalue_ratio);
      } else {
        // Fallback: global scalar when direction vectors unavailable.
        alpha_eff = saved_alpha_min / (1.0f + config_.degeneracy_alpha_scale *
                                    static_cast<float>(iekf_result.max_degen));
        surfel_map_.set_l0_ema_alpha_min(alpha_eff);
        surfel_map_.set_degen_state(nullptr, 0);
      }
    } else {
      surfel_map_.set_degen_state(nullptr, 0);
    }

    surfel_map_.update(world_cloud, state_.position);

    // Restore alpha if it was modified.
    if (std::abs(alpha_eff - saved_alpha_min) > 1e-8f) {
      surfel_map_.set_l0_ema_alpha_min(saved_alpha_min);
    }

    // Advance surfel keyframe anchor (skip when gate is Off to preserve
    // ColdStart semantics if gate is later enabled mid-run).
    if (surfel_decision.reason != SurfelInsertReason::Off) {
      last_surfel_kf_frame_    = frame_count_;
      last_surfel_kf_position_ = state_.position;
      last_surfel_kf_rotation_ = state_.rotation;
    }
  }  // end surfel_decision.insert

  // DARBF: set degeneracy freeze state on PVMap before update.
  // Freezes ring buffer writes for voxels aligned with degenerate directions,
  // preserving early-inserted anchor points for unbiased DDPO correspondences.
  // Persistence filter: only activate after min_persist consecutive degen frames.
  if (config_.point_voxel_map.enable_degen_freeze) {
    const bool persist_met = (degen_persist_count_ >= config_.degen_freeze_min_persist);
    const bool darbf_active = iekf_result.num_degen_trans_dirs > 0 && persist_met;
    point_voxel_map_.set_degen_freeze_state(
        darbf_active ? iekf_result.degen_trans_dirs : nullptr,
        darbf_active ? iekf_result.num_degen_trans_dirs : 0, frame_count_);
    if (iekf_result.num_degen_trans_dirs > 0 && !persist_met &&
        (frame_count_ % 100 == 0 || degen_persist_count_ <= 2)) {
      SPDLOG_INFO("DARBF_PERSIST: frame={} degen detected but persist={}/{} — freeze suppressed",
                  frame_count_, degen_persist_count_, config_.degen_freeze_min_persist);
    }
  }
  point_voxel_map_.set_effective_svd_min_eigenvalue(-1.0f);

  point_voxel_map_.update(world_cloud, state_.position, iekf_result.res_rms, frame_count_);

  // DARBF diagnostic logging.
  if (config_.point_voxel_map.enable_degen_freeze &&
      (frame_count_ % 100 == 0 || frame_count_ < 5)) {
    const int n_frozen = point_voxel_map_.frozen_voxel_count();
    if (n_frozen > 0) {
      SPDLOG_INFO("DARBF: frame={} frozen_voxels={}/{} degen_dirs={}",
                  frame_count_, n_frozen, point_voxel_map_.voxel_count(),
                  iekf_result.num_degen_trans_dirs);
    }
  }

}


// ---------------------------------------------------------------------------
// run_fixed_lag_smoother  (Tier 4: velocity-coupled 2-frame GN + IMU preint)
// ---------------------------------------------------------------------------
//
// After IEKF converges and map is updated for frame k, jointly optimize
// frames k-1 and k with IMU preintegration cross-coupling.
//
// The IMU preintegration factor provides the sole genuine Hessian cross-term
// between the two frames — without it, the 18×18 system decomposes into
// two independent 9×9 blocks (proven by adversarial review).

void LioEstimator::run_fixed_lag_smoother(
    const std::vector<Correspondence>& /* curr_corrs */) {
  if (!config_.enable_fixed_lag_smoother || !prev_frame_valid_) {
    return;
  }

  if (prev_processed_scan_.empty()) {
    SPDLOG_WARN("FLS: frame={} skip — prev scan is empty", frame_count_);
    fls_imu_buffer_.clear();
    prev_frame_valid_ = false;
    return;
  }

  // 1. Preintegrate buffered IMU measurements.
  if (fls_imu_buffer_.size() < 3) {
    if (frame_count_ % 100 == 0) {
      SPDLOG_INFO("FLS: frame={} skip — only {} IMU samples (need ≥3)",
                  frame_count_, fls_imu_buffer_.size());
    }
    fls_imu_buffer_.clear();
    return;
  }

  const float noise_scale = config_.fls.imu_noise_scale;
  const ImuPreintegration preint = preintegrate_imu(
      fls_imu_buffer_,
      prev_state_posterior_.gyro_bias,
      prev_state_posterior_.acc_bias,
      config_.gyro_noise_std * noise_scale,
      config_.acc_noise_std * noise_scale);

  fls_imu_buffer_.clear();

  if (!preint.valid()) {
    SPDLOG_WARN("FLS: frame={} skip — preintegration invalid", frame_count_);
    return;
  }

  // 2. Re-query correspondences for frame k-1 against CURRENT (updated) map.
  surfel_map_.flush_dirty();

  HybridCorrespondenceParams corr_params;
  corr_params.max_plane_distance = config_.max_plane_distance;
  corr_params.pvmap_k_neighbors = config_.pvmap_k_neighbors;
  corr_params.pvmap_planarity_threshold = config_.pvmap_planarity_threshold;
  corr_params.max_corr_per_l1 = config_.max_corr_per_l1;
  corr_params.pvmap_sigma2_scale = config_.pvmap_sigma2_scale;
  corr_params.enable_cscf = config_.enable_cscf;
  corr_params.cscf_kernel_bandwidth = config_.cscf_kernel_bandwidth;

  std::vector<Correspondence> corrs_prev =
      find_correspondences_hybrid_select(
          prev_state_posterior_, config_.T_body_lidar, prev_processed_scan_,
          surfel_map_, point_voxel_map_, corr_params);

  if (static_cast<int>(corrs_prev.size()) < config_.fls_min_correspondences) {
    if (frame_count_ % 100 == 0) {
      SPDLOG_INFO("FLS: frame={} skip — only {} prev corrs (min={})",
                  frame_count_, corrs_prev.size(),
                  config_.fls_min_correspondences);
    }
    return;
  }

  // 3. Run the 2-frame Gauss-Newton smoother.
  const FlsResult fls = fixed_lag_smooth(
      prev_state_posterior_,
      state_,
      prev_P_posterior_,
      state_.covariance,
      corrs_prev,
      preint,
      config_.T_body_lidar,
      config_.iekf,
      config_.fls);

  // 4. Apply correction to state_ (rotation and position only).
  // Velocity is a nuisance variable needed for IMU coupling but must NOT
  // be applied: the FLS doesn't optimize bias, so bias errors create
  // persistent velocity residuals that compound each frame.
  if (fls.applied) {
    state_.rotation = fls.state_curr.rotation;
    state_.position = fls.state_curr.position;
    // state_.velocity intentionally NOT updated.
  }

  // 5. Log diagnostic info.
  constexpr float kRadToDeg = 180.0f / 3.14159265358979f;
  if (frame_count_ % 50 == 0 || fls.pos_correction_curr > 0.005f) {
    SPDLOG_INFO("FLS: frame={} corrs={} dp={:.6f}m dR={:.4f}deg dv={:.6f}m/s "
                "iters={} H01={:.2f} {}",
                frame_count_, corrs_prev.size(),
                fls.pos_correction_curr,
                fls.rot_correction_curr * kRadToDeg,
                fls.vel_correction_curr,
                fls.iterations,
                fls.cross_block_norm,
                fls.applied ? "APPLIED" : "REJECTED");
  }
}

// ---------------------------------------------------------------------------
// log_frame_diagnostics
// ---------------------------------------------------------------------------

void LioEstimator::log_frame_diagnostics(
    const IekfLoopResult& iekf_result,
    const PointCloud& processed,
    double timestamp,
    std::chrono::high_resolution_clock::time_point t_start,
    std::chrono::high_resolution_clock::time_point t_preprocess,
    std::chrono::high_resolution_clock::time_point t_iekf,
    std::chrono::high_resolution_clock::time_point t_map_insert,
    std::chrono::high_resolution_clock::time_point t_map) {
  using Clock = std::chrono::high_resolution_clock;
  const bool do_timing = config_.enable_debug_timing;

  // --- Real-time diagnostics (requirements 3-2 through 3-4) ---
  {
    FrameDiagnostics diag;
    diag.frame = frame_count_;
    diag.timestamp = timestamp;
    diag.res_rms = iekf_result.res_rms;
    diag.res_mean = iekf_result.res_mean;
    diag.converged = iekf_result.converged;
    diag.iekf_iters = iekf_result.total_inner;
    // pos_delta: movement since last frame (for velocity sanity check)
    const Eigen::Vector3f pos_delta = state_.position - last_corrected_position_;
    diag.pos_delta_norm = pos_delta.norm();
    diag.num_degenerate_dirs = iekf_result.max_degen;
    diag.num_correspondences = iekf_result.total_corrs;
    diag.l0_count = surfel_map_.l0_count();
    diag.l1_count = surfel_map_.l1_count();
    diag.gravity_norm = state_.gravity.norm();
    diag.gyro_bias_norm = state_.gyro_bias.norm();
    diag.acc_bias_norm = state_.acc_bias.norm();
    diag.velocity_norm = state_.velocity.norm();
    diag.trace_P_pos = static_cast<float>(
        state_.covariance(3, 3) + state_.covariance(4, 4) +
        state_.covariance(5, 5));
    diag.total_ms = std::chrono::duration<float, std::milli>(
        t_map - t_start).count();
    // Eq.(4) instrumentation: d_deg direction + cos² distribution
    if (iekf_result.num_degen_trans_dirs > 0) {
      const auto& d = iekf_result.degen_trans_dirs[0];
      diag.d_deg_x = d.x();
      diag.d_deg_y = d.y();
      diag.d_deg_z = d.z();
    }
    diag.mean_cos2 = surfel_map_.cos2_mean();
    diag.cos2_count = surfel_map_.cos2_count();
    // S6-R1.2 ρ_λ measurement campaign instrumentation (no algorithm change).
    diag.eigenvalue_ratio = iekf_result.eigenvalue_ratio;
    // S12-B.A.3 DG-A per-channel signature (zero when dg_a_enable=false).
    diag.dg_a_rho_l1     = iekf_result.dg_a_rho[0];
    diag.dg_a_rho_l2     = iekf_result.dg_a_rho[1];
    diag.dg_a_rho_full   = iekf_result.dg_a_rho[2];
    diag.dg_a_cos_l1_l2  = iekf_result.dg_a_cos_agree[0];
    diag.dg_a_cos_l1_full = iekf_result.dg_a_cos_agree[1];
    diag.dg_a_cos_l2_full = iekf_result.dg_a_cos_agree[2];
    diag.dg_a_n_l1       = iekf_result.dg_a_n_corr[0];
    diag.dg_a_n_l2       = iekf_result.dg_a_n_corr[1];
    diagnostics_.update(diag);
  }

  // --- Timing log (CSV) ---
  {
    if (!timing_csv_initialized_ && !config_.timing_log_path.empty()) {
      timing_csv_.open(config_.timing_log_path,
                      std::ios::out | std::ios::trunc);
      if (timing_csv_.is_open()) {
        if (do_timing) {
          // Detailed debug CSV header
          timing_csv_ << "frame,timestamp,"
                         "stride_ms,voxel_ms,undistort_ms,range_ms,preprocess_ms,"
                         "corr_find_ms,jacobian_ms,huber_pko_ms,build_info_ms,"
                         "solve_ms,iekf_inner_ms,iekf_total_ms,"
                         "map_transform_ms,map_update_ms,map_ms,total_ms,"
                         "n_raw,n_stride,n_voxel,n_processed,n_corrs,"
                         "n_outer,n_inner,converged,n_degen,"
                         "res_mean,res_rms,"
                         "eig0,eig1,eig2,eig3,eig4,eig5,"
                         "pos_x,pos_y,pos_z,vel_norm\n";
        } else {
          timing_csv_ << "frame,timestamp,preprocess_ms,iekf_ms,map_ms,total_ms,"
                         "n_points,n_corrs,n_outer_iters\n";
        }
      }
      timing_csv_initialized_ = true;
    }
    if (timing_csv_.is_open()) {
      auto ms = [](auto a, auto b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
      };

      const double preprocess_ms = ms(t_start, t_preprocess);
      const double iekf_ms = ms(t_preprocess, t_iekf);
      const double map_ms_total = ms(t_iekf, t_map);
      const double total_ms = ms(t_start, t_map);

      if (do_timing) {
        const double map_xform_ms = ms(t_iekf, t_map_insert);
        const double map_upd_ms = ms(t_map_insert, t_map);
        const PreprocessTiming& pt = last_preprocess_timing_;
        const float vel_norm = state_.velocity.norm();

        timing_csv_ << std::fixed
          << (frame_count_) << ","
          << std::setprecision(3) << timestamp << ","
          << std::setprecision(2)
          << pt.stride_ms << "," << pt.voxel_ms << ","
          << pt.undistort_ms << "," << pt.range_ms << ","
          << preprocess_ms << ","
          << iekf_result.corr_find_ms << ","
          << iekf_result.jacobian_ms << "," << iekf_result.huber_pko_ms << ","
          << iekf_result.build_info_ms << "," << iekf_result.solve_ms << ","
          << iekf_result.iekf_inner_ms << "," << iekf_ms << ","
          << map_xform_ms << "," << map_upd_ms << ","
          << map_ms_total << "," << total_ms << ","
          << pt.n_raw << "," << pt.n_after_stride << ","
          << pt.n_after_voxel << "," << pt.n_processed << ","
          << iekf_result.total_corrs << ","
          << iekf_result.total_outer << "," << iekf_result.total_inner << ","
          << (iekf_result.converged ? 1 : 0) << "," << iekf_result.max_degen << ","
          << std::setprecision(5)
          << iekf_result.res_mean << "," << iekf_result.res_rms << ","
          << std::setprecision(1)
          << iekf_result.eigenvalues[0] << "," << iekf_result.eigenvalues[1] << ","
          << iekf_result.eigenvalues[2] << "," << iekf_result.eigenvalues[3] << ","
          << iekf_result.eigenvalues[4] << "," << iekf_result.eigenvalues[5] << ","
          << std::setprecision(3)
          << static_cast<double>(state_.position.x()) << ","
          << static_cast<double>(state_.position.y()) << ","
          << static_cast<double>(state_.position.z()) << ","
          << std::setprecision(3) << static_cast<double>(vel_norm)
          << "\n";
      } else {
        timing_csv_ << (frame_count_) << ","
                   << std::fixed << std::setprecision(3) << timestamp << ","
                   << std::setprecision(2)
                   << preprocess_ms << "," << iekf_ms << ","
                   << map_ms_total << "," << total_ms << ","
                   << processed.size() << ","
                   << iekf_result.total_corrs << "," << iekf_result.total_outer << "\n";
      }

      if (frame_count_ % 50 == 0) {
        timing_csv_.flush();
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

LioState LioEstimator::current_state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

bool LioEstimator::initialized() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return initialized_;
}

int LioEstimator::frame_count() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return frame_count_;
}

std::vector<Surfel> LioEstimator::map_surfels() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return surfel_map_.all_surfels();
}

const SurfelMap& LioEstimator::surfel_map() const {
  return surfel_map_;
}

// ---------------------------------------------------------------------------
// reset
// ---------------------------------------------------------------------------

void LioEstimator::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  state_.reset();
  initialized_ = false;
  frame_count_ = 0;
  first_lidar_frame_ = true;
  last_imu_time_ = 0.0;
  last_lidar_time_ = 0.0;
  imu_acc_scale_ = 1.0f;
  last_corrected_position_ = Eigen::Vector3f::Zero();
  surfel_map_.reset();
  point_voxel_map_.reset();
  pko_.reset();
  icdr_state_ = IcdrState{};  // Reset ICDR persistent state
  state_history_.clear();

  // Reset promoted-from-static members
  scan_diag_count_ = 0;
  corr_shuffle_rng_.seed(42);
  if (timing_csv_.is_open()) timing_csv_.close();
  timing_csv_initialized_ = false;

  // T5.4-R8: classifier two-stage state
  classifier_state_.reset();
}

// ---------------------------------------------------------------------------
// should_insert_surfel (Task #133 Iter 3 — surfel keyframe gate)
// ---------------------------------------------------------------------------

LioEstimator::SurfelInsertDecision
LioEstimator::should_insert_surfel() const noexcept {
  // 1. Off-path: gate disabled → bit-identical to pre-gate (every-frame insert).
  if (!config_.enable_surfel_keyframe_gate) {
    return {true, SurfelInsertReason::Off};
  }

  // 2. Cold start: no anchor recorded yet.
  if (last_surfel_kf_frame_ < 0) {
    return {true, SurfelInsertReason::ColdStart};
  }

  // 3. Warmup: always insert during first N frames for map bootstrap.
  if (frame_count_ < config_.surfel_kf_warmup_frames) {
    return {true, SurfelInsertReason::Warmup};
  }

  // 4. Translation predicate: ||p_now - p_last_kf|| > tau_t.
  const float dx_norm =
      (state_.position - last_surfel_kf_position_).norm();
  if (dx_norm > static_cast<float>(config_.surfel_kf_trans_thresh_m)) {
    return {true, SurfelInsertReason::Keyframe};
  }

  // 5. Rotation predicate: angle(R_now * R_last^T) > tau_r.
  const Eigen::Matrix3f dR =
      state_.rotation * last_surfel_kf_rotation_.transpose();
  const float cos_theta =
      std::min(1.0f, std::max(-1.0f, 0.5f * (dR.trace() - 1.0f)));
  const float theta = std::acos(cos_theta);
  if (theta > static_cast<float>(config_.surfel_kf_rot_thresh_rad)) {
    return {true, SurfelInsertReason::Keyframe};
  }

  // 6. Near-stationary: skip surfel insertion this frame.
  return {false, SurfelInsertReason::Skip};
}

}  // namespace core
}  // namespace tof_slam
