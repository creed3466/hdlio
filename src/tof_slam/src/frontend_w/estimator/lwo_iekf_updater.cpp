// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// lwo_iekf_updater.cpp — Fused single-pass IEKF update for 15-D LWO state.
//
// v1.0 upgrade: replaces explicit Nx15 H matrix construction with a fused
// single-pass 6x6 accumulator for the lidar factor (rot+pos), matching the
// LIO iekf_updater pattern.  GC, NH, WV factors contribute directly as
// rank-1/rank-2/rank-3 updates to the 15x15 information matrix.

#include "tof_slam/frontend_w/estimator/lwo_iekf_updater.hpp"

#include <chrono>
#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <spdlog/spdlog.h>

#include "tof_slam/common/lie/so3.hpp"

namespace tof_slam {
namespace lwo {

LwoIekfUpdater::LwoIekfUpdater(const LwoIekfConfig& cfg) : cfg_(cfg) {}

// ---------------------------------------------------------------------------
// update — fused single-pass IEKF
// ---------------------------------------------------------------------------

LwoIekfResult LwoIekfUpdater::update(
    const LwoState& prior,
    const std::vector<core::Correspondence>& correspondences,
    const core::Se3& T_body_lidar,
    const GroundConstraint& gc,
    const WheelMeasurement& wv,
    float vx_enc,
    float omega_z_enc,
    float omega_z_b,
    core::Pko* pko) const {

  using Clock = std::chrono::steady_clock;
  const bool do_timing = cfg_.check_usage;

  LwoIekfResult result_init;  // Accumulate timing across iters

  LwoState state = prior;
  bool converged  = false;
  int  total_iters = 0;

  const int n_corr = static_cast<int>(correspondences.size());

  // Pre-compute prior information (float64 for stability).
  const LwoStateCovariance& P_prior = prior.covariance;
  LwoStateCovariance P_inv = LwoStateCovariance::Zero();
  if (cfg_.enable_ext_calibration) {
    P_inv = P_prior.cast<double>().inverse().cast<float>();
  }

  // Pre-compute extrinsic constants (state-independent).
  const Eigen::Matrix3f R_il = T_body_lidar.rotation().matrix();
  const Eigen::Vector3f t_il = T_body_lidar.translation();

  // Pre-compute p_imu = R_il * p_lidar + t_il (constant across iterations).
  std::vector<Eigen::Vector3f> p_imu_cache(n_corr);
  for (int i = 0; i < n_corr; ++i) {
    p_imu_cache[i] = R_il * correspondences[i].p_lidar + t_il;
  }

  // Adaptive noise pre-computed constants.
  const float sigma2_base = cfg_.lidar_noise_std * cfg_.lidar_noise_std;
  const float r_ref_sq = cfg_.adaptive_range_ref * cfg_.adaptive_range_ref;

  // PKO scale — computed once at iter==0 from a residual-only pass.
  float pko_norm_scale = 1.0f;
  float pko_delta = 1.0f;
  bool  pko_initialized = false;

  // G matrix from last iteration (for posterior covariance).
  LwoStateCovariance G_final = LwoStateCovariance::Zero();

  for (int iter = 0; iter < cfg_.max_inner_iters; ++iter) {
    ++total_iters;

    const auto t_jac = do_timing ? Clock::now() : Clock::time_point{};

    if (n_corr == 0) {
      // No correspondences: skip IEKF entirely.
      break;
    }

    // ------------------------------------------------------------------
    // 1. PKO initialization (iter==0 only): residual-only pass.
    // ------------------------------------------------------------------
    if (iter == 0 && pko != nullptr && !pko_initialized) {
      const Eigen::Matrix3f R_wb = state.rotation;
      const Eigen::Vector3f t_wb = state.position;
      Eigen::VectorXf res_init(n_corr);
      for (int i = 0; i < n_corr; ++i) {
        const Eigen::Vector3f p_world = R_wb * p_imu_cache[i] + t_wb;
        res_init(i) = -(correspondences[i].normal.dot(p_world) -
                        correspondences[i].plane_d);
      }
      const float res_mean = res_init.mean();
      const float res_var = (res_init.array() - res_mean).square().mean();
      pko_norm_scale = std::max(std::sqrt(res_var) / 3.0f, 1e-6f);

      std::vector<double> res_d(n_corr);
      Eigen::VectorXf huber_res = res_init / pko_norm_scale;
      for (int i = 0; i < n_corr; ++i) {
        res_d[i] = static_cast<double>(huber_res(i));
      }
      pko_delta = static_cast<float>(pko->compute_scale_factor(res_d));
      pko_initialized = true;
    }

    const float delta = (pko != nullptr) ? pko_delta : 1.0f;

    // ------------------------------------------------------------------
    // 2. FUSED LIDAR FACTOR (v1.0 pattern): single-pass 6x6 accumulator.
    //
    // For each correspondence:
    //   1. Transform to world
    //   2. Compute residual
    //   3. Huber weight from cached delta
    //   4. Adaptive sigma2 (or planarity-only fallback)
    //   5. Jacobian row h (6x1)
    //   6. Accumulate HTRinvH_66 += w * h * h^T, HTRinvz_6 += w*r * h
    //
    // Ext calibration: if enabled, also accumulate into ext columns.
    // ------------------------------------------------------------------

    const Eigen::Matrix3f R_wb = state.rotation;
    const Eigen::Vector3f t_wb = state.position;
    const Eigen::Matrix3f R_wb_t = R_wb.transpose();

    Eigen::Matrix<float, 6, 6> HTRinvH_66 = Eigen::Matrix<float, 6, 6>::Zero();
    Eigen::Matrix<float, 6, 1> HTRinvz_6 = Eigen::Matrix<float, 6, 1>::Zero();

    // Ext calibration accumulators (only used when enabled).
    // ext_yaw(1) + ext_xy(2) = 3 extra DOF beyond the 6-DOF pose.
    // We accumulate cross-terms between pose(6) and ext(3).
    Eigen::Matrix<float, 3, 3> HTRinvH_ext = Eigen::Matrix<float, 3, 3>::Zero();
    Eigen::Matrix<float, 6, 3> HTRinvH_pose_ext = Eigen::Matrix<float, 6, 3>::Zero();
    Eigen::Matrix<float, 3, 1> HTRinvz_ext = Eigen::Matrix<float, 3, 1>::Zero();

    // Also keep the 6x6 LiDAR-only info for degeneracy detection.
    Eigen::Matrix<float, 6, 6> info_lidar_pose_only =
        Eigen::Matrix<float, 6, 6>::Zero();

    for (int i = 0; i < n_corr; ++i) {
      const auto& corr = correspondences[i];
      const Eigen::Vector3f& p_imu = p_imu_cache[i];
      const Eigen::Vector3f p_world = R_wb * p_imu + t_wb;

      // Point-to-plane residual.
      const float r = -(corr.normal.dot(p_world) - corr.plane_d);

      // Huber weight.
      const float abs_r_norm = std::abs(r / pko_norm_scale);
      const float huber_w = (abs_r_norm <= delta) ? 1.0f : (delta / abs_r_norm);

      // Adaptive sigma2.
      float sigma2_i;
      if (cfg_.enable_adaptive_noise) {
        const Eigen::Vector3f dp = p_world - corr.centroid;
        const float dist_sq = dp.squaredNorm();
        const float range_pen = cfg_.adaptive_range_scale *
                                (corr.range * corr.range) / r_ref_sq;
        const float one_m_cos = 1.0f - corr.cos_incidence;
        const float incidence_pen = cfg_.adaptive_incidence_scale *
                                    one_m_cos * one_m_cos;
        const float planarity_pen = cfg_.adaptive_planarity_scale *
                                    corr.planarity * corr.planarity;
        sigma2_i = sigma2_base * (1.0f + range_pen + incidence_pen +
                                  planarity_pen) +
                   dist_sq * corr.normal_sigma2;
      } else {
        // Fallback: planarity-only or noise_override.
        const float eff_sigma = (corr.noise_override > 0.0f)
            ? corr.noise_override
            : cfg_.lidar_noise_std *
                  (1.0f + cfg_.planarity_noise_scale * corr.planarity);
        sigma2_i = eff_sigma * eff_sigma;
      }

      float w = huber_w / (0.001f + sigma2_i);
      if (corr.sharing_count > 1) {
        w /= static_cast<float>(corr.sharing_count);
      }

      // Jacobian: h = [-A, -n] (6x1).
      const Eigen::Vector3f C = R_wb_t * corr.normal;
      const Eigen::Vector3f A = p_imu.cross(C);
      Eigen::Matrix<float, 6, 1> h;
      h << -A, -corr.normal;

      // Weighted rank-1 accumulation.
      HTRinvH_66.noalias() += w * h * h.transpose();
      HTRinvz_6.noalias() += (w * r) * h;

      // Track LiDAR-only info for degeneracy detection (before ext).
      if (cfg_.enable_degeneracy_projection) {
        info_lidar_pose_only.noalias() += w * h * h.transpose();
      }

      // Ext calibration Jacobian columns (if enabled).
      if (cfg_.enable_ext_calibration) {
        const Eigen::Vector3f& p_lidar = corr.p_lidar;
        // ext_yaw: -(C^T * R_il * [-py, px, 0])
        const Eigen::Vector3f p_rot(-p_lidar.y(), p_lidar.x(), 0.0f);
        const float h_yaw = -(C.transpose() * R_il * p_rot)(0);
        // ext_xy: -normal^T * R_wb * [e1, e2]
        const float h_x = -corr.normal.dot(R_wb.col(0));
        const float h_y = -corr.normal.dot(R_wb.col(1));
        Eigen::Matrix<float, 3, 1> h_ext;
        h_ext << h_yaw, h_x, h_y;

        HTRinvH_ext.noalias() += w * h_ext * h_ext.transpose();
        HTRinvH_pose_ext.noalias() += w * h * h_ext.transpose();
        HTRinvz_ext.noalias() += (w * r) * h_ext;
      }

      // L2 multi-scale: additional rank-1 update from coarser surfel.
      if (cfg_.enable_l2_correspondences && corr.has_l2) {
        const float r_L2 = -(corr.l2_normal.dot(p_world) - corr.l2_plane_d);
        const Eigen::Vector3f C_L2 = R_wb_t * corr.l2_normal;
        const Eigen::Vector3f A_L2 = p_imu.cross(C_L2);
        Eigen::Matrix<float, 6, 1> h_L2;
        h_L2 << -A_L2, -corr.l2_normal;

        // L2 noise model.
        const Eigen::Vector3f dp_L2 = p_world - corr.l2_centroid;
        const float dist_sq_L2 = dp_L2.squaredNorm();
        float sigma2_L2;
        if (cfg_.enable_adaptive_noise) {
          const float range_pen = cfg_.adaptive_range_scale *
                                  (corr.range * corr.range) / r_ref_sq;
          const float one_m_cos_L2 = 1.0f -
              std::abs(corr.l2_normal.dot(dp_L2) /
                       (std::sqrt(dist_sq_L2) + 1e-6f));
          const float incidence_pen_L2 = cfg_.adaptive_incidence_scale *
                                         one_m_cos_L2 * one_m_cos_L2;
          const float planarity_pen_L2 = cfg_.adaptive_planarity_scale *
                                         corr.l2_planarity * corr.l2_planarity;
          sigma2_L2 = cfg_.l2_noise_scale * sigma2_base *
                          (1.0f + range_pen + incidence_pen_L2 +
                           planarity_pen_L2) +
                      dist_sq_L2 * corr.l2_normal_sigma2;
        } else {
          sigma2_L2 = cfg_.l2_noise_scale * sigma2_base +
                      dist_sq_L2 * corr.l2_normal_sigma2;
        }

        const float abs_r_L2_norm = std::abs(r_L2 / pko_norm_scale);
        const float huber_w_L2 =
            (abs_r_L2_norm <= delta) ? 1.0f : (delta / abs_r_L2_norm);
        const float w_L2 = huber_w_L2 / (0.001f + sigma2_L2);

        HTRinvH_66.noalias() += w_L2 * h_L2 * h_L2.transpose();
        HTRinvz_6.noalias() += (w_L2 * r_L2) * h_L2;
      }
    }

    // ------------------------------------------------------------------
    // 3. Ground constraint (3 rows: roll, pitch, z).
    // ------------------------------------------------------------------
    const GroundConstraintResult gc_res = gc.compute(state);

    // ------------------------------------------------------------------
    // 4. Non-holonomic constraint (1 row: vy_body = 0).
    // ------------------------------------------------------------------
    const Eigen::Vector3f Rtv = state.rotation.transpose() * state.velocity;
    const Eigen::Matrix3f skew_Rtv = core::Hat(Rtv);
    Eigen::Matrix<float, 1, kLwoStateDim> H_nh =
        Eigen::Matrix<float, 1, kLwoStateDim>::Zero();
    H_nh.block<1, 3>(0, kLwoRotIdx) = -skew_Rtv.row(1);
    H_nh.block<1, 3>(0, kLwoVelIdx) =
        -state.rotation.col(1).transpose();
    const float vy_body = Rtv(1);
    const float r_nh = -vy_body;

    // ------------------------------------------------------------------
    // 5. Wheel velocity measurement (2 rows: vx, omega_z, optional).
    // ------------------------------------------------------------------
    const int n_wv = cfg_.enable_wheel_measurement ? 2 : 0;
    WheelMeasurementResult wv_res;
    if (n_wv > 0) {
      wv_res = wv.compute(state, vx_enc, omega_z_enc, omega_z_b);
    }

    if (do_timing) result_init.jacobian_ms +=
        std::chrono::duration<float, std::milli>(Clock::now() - t_jac).count();
    const auto t_build_solve = do_timing ? Clock::now() : Clock::time_point{};

    // ------------------------------------------------------------------
    // 6. Assemble 15x15 information matrix directly (no N-row H_full).
    //
    // Lidar factor: embed 6x6 at [0:6, 0:6]
    // GC: rank-3 update at scattered indices
    // NH: rank-1 update at scattered indices
    // WV: rank-2 update at scattered indices (optional)
    // ------------------------------------------------------------------

    LwoStateCovariance HTRinvH = LwoStateCovariance::Zero();
    LwoStateVector HTRinvr = LwoStateVector::Zero();

    // Lidar factor: embed 6x6 accumulator.
    HTRinvH.block<6, 6>(0, 0) = HTRinvH_66;
    HTRinvr.head<6>() = HTRinvz_6;

    // Ext calibration: embed ext blocks.
    if (cfg_.enable_ext_calibration) {
      // ext block [12:15, 12:15]
      HTRinvH.block<3, 3>(kLwoExtYawIdx, kLwoExtYawIdx) = HTRinvH_ext;
      // cross-terms [0:6, 12:15] and [12:15, 0:6]
      HTRinvH.block<6, 3>(0, kLwoExtYawIdx) = HTRinvH_pose_ext;
      HTRinvH.block<3, 6>(kLwoExtYawIdx, 0) = HTRinvH_pose_ext.transpose();
      // ext residual contribution
      HTRinvr.segment<3>(kLwoExtYawIdx) = HTRinvz_ext;
    }

    // Ground constraint: H_gc (3 x 15) with noise.
    // Sign: negate H_gc (dh/dx -> -dh/dx convention).
    {
      const Eigen::Matrix<float, 3, kLwoStateDim> H_gc_neg = -gc_res.H;
      for (int j = 0; j < 3; ++j) {
        const float w_gc = 1.0f / gc_res.noise_cov(j, j);
        const Eigen::Matrix<float, kLwoStateDim, 1> hj =
            H_gc_neg.row(j).transpose();
        HTRinvH.noalias() += w_gc * hj * hj.transpose();
        HTRinvr.noalias() += (w_gc * gc_res.residual(j)) * hj;
      }
    }

    // Non-holonomic constraint: rank-1.
    {
      constexpr float kNhNoiseStd = 0.05f;
      const float w_nh = 1.0f / (kNhNoiseStd * kNhNoiseStd);
      // Negate H_nh (same convention).
      const Eigen::Matrix<float, kLwoStateDim, 1> h_nh_neg =
          (-H_nh).transpose();
      HTRinvH.noalias() += w_nh * h_nh_neg * h_nh_neg.transpose();
      HTRinvr.noalias() += (w_nh * r_nh) * h_nh_neg;
    }

    // Wheel velocity measurement: rank-2.
    if (n_wv > 0) {
      const Eigen::Matrix<float, 2, kLwoStateDim> H_wv_neg = -wv_res.H;
      for (int j = 0; j < 2; ++j) {
        const float w_wv = 1.0f / wv_res.noise_cov(j, j);
        const Eigen::Matrix<float, kLwoStateDim, 1> hj =
            H_wv_neg.row(j).transpose();
        HTRinvH.noalias() += w_wv * hj * hj.transpose();
        HTRinvr.noalias() += (w_wv * wv_res.residual(j)) * hj;
      }
    }

    // ------------------------------------------------------------------
    // 7. Solve: K1 = (HTRinvH + P_inv)^{-1}, dx = K1 * (-HTRinvr)
    // ------------------------------------------------------------------

    LwoStateCovariance K1 = LwoStateCovariance::Zero();
    LwoStateVector dx = LwoStateVector::Zero();

    if (!cfg_.enable_ext_calibration) {
      // 12-DOF path: use first 12 rows/cols only for backward compat.
      constexpr int kBaseDim = 12;
      const Eigen::Matrix<float, kBaseDim, kBaseDim> HTRinvH12 =
          HTRinvH.topLeftCorner<kBaseDim, kBaseDim>();
      const Eigen::Matrix<float, kBaseDim, kBaseDim> P12_inv =
          P_prior.topLeftCorner<kBaseDim, kBaseDim>().inverse();
      const Eigen::Matrix<float, kBaseDim, kBaseDim> info12 =
          HTRinvH12 + P12_inv;

      const Eigen::Matrix<double, kBaseDim, kBaseDim> info12_d =
          info12.cast<double>();
      const Eigen::Matrix<float, kBaseDim, kBaseDim> K1_12 =
          info12_d.inverse().cast<float>();

      const Eigen::Matrix<float, kBaseDim, 1> HTRinvr12 =
          HTRinvr.head<kBaseDim>();
      dx.head<kBaseDim>() = K1_12 * (-HTRinvr12);

      K1.topLeftCorner<kBaseDim, kBaseDim>() = K1_12;
      // Keep full HTRinvH for G computation.
    } else {
      // 15-DOF path.
      const LwoStateCovariance info_matrix = HTRinvH + P_inv;
      const Eigen::Matrix<double, kLwoStateDim, kLwoStateDim> info_d =
          info_matrix.cast<double>();
      K1 = info_d.inverse().cast<float>();
      dx = K1 * (-HTRinvr);
    }

    if (do_timing) {
      result_init.build_info_ms +=
          std::chrono::duration<float, std::milli>(
              Clock::now() - t_build_solve).count() * 0.5f;
      result_init.solve_ms +=
          std::chrono::duration<float, std::milli>(
              Clock::now() - t_build_solve).count() * 0.5f;
    }

    // Zero velocity correction — velocity is deterministic from wheel.
    dx.segment<3>(kLwoVelIdx).setZero();

    // Observability-gated scale/bias locking.
    const float abs_omega = std::abs(omega_z_enc);
    if (abs_omega > 0.15f || std::abs(vx_enc) < 0.1f) {
      dx(kLwoScaleIdx) = 0.0f;
    } else {
      dx(kLwoScaleIdx) = std::clamp(dx(kLwoScaleIdx), -0.01f, 0.01f);
    }
    dx(kLwoBiasIdx)     = 0.0f;
    dx(kLwoBiasIdx + 1) = 0.0f;

    // Extrinsic calibration observability gate.
    {
      const bool ext_observable =
          cfg_.enable_ext_calibration &&
          (std::abs(omega_z_enc) > cfg_.ext_obs_min_omega) &&
          (n_corr >= cfg_.ext_obs_min_correspondences);

      if (!ext_observable) {
        dx(kLwoExtYawIdx) = 0.0f;
        dx.segment<2>(kLwoExtXyIdx).setZero();
      } else {
        static int ext_log_counter = 0;
        if (iter == 0 && (++ext_log_counter % 10 == 0)) {
          SPDLOG_INFO("[EXT_CALIB] dx_yaw={:.6f}rad dx_xy=[{:.6f},{:.6f}]m "
                      "omega={:.3f} corrs={} state_yaw={:.6f} state_xy=[{:.6f},{:.6f}]",
                      static_cast<double>(dx(kLwoExtYawIdx)),
                      static_cast<double>(dx(kLwoExtXyIdx)),
                      static_cast<double>(dx(kLwoExtXyIdx + 1)),
                      static_cast<double>(omega_z_enc),
                      n_corr,
                      static_cast<double>(state.ext_delta_yaw),
                      static_cast<double>(state.ext_delta_xy(0)),
                      static_cast<double>(state.ext_delta_xy(1)));
        }
        dx(kLwoExtYawIdx) = std::clamp(dx(kLwoExtYawIdx),
                                        -cfg_.ext_max_delta_yaw,
                                         cfg_.ext_max_delta_yaw);
        dx(kLwoExtXyIdx)     = std::clamp(dx(kLwoExtXyIdx),
                                           -cfg_.ext_max_delta_xy,
                                            cfg_.ext_max_delta_xy);
        dx(kLwoExtXyIdx + 1) = std::clamp(dx(kLwoExtXyIdx + 1),
                                           -cfg_.ext_max_delta_xy,
                                            cfg_.ext_max_delta_xy);
      }
    }

    // ------------------------------------------------------------------
    // 8. Degeneracy-aware selective update (Zhang & Singh ICRA 2016).
    //
    // Uses the LiDAR-only 6x6 information (pre-accumulated above).
    // ------------------------------------------------------------------
    if (cfg_.enable_degeneracy_projection) {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 6, 6>> solver(
          info_lidar_pose_only);
      const auto& eigenvalues = solver.eigenvalues();
      const auto& eigenvectors = solver.eigenvectors();

      Eigen::Matrix<float, 6, 6> V_f =
          Eigen::Matrix<float, 6, 6>::Zero();
      int n_constrained = 0;

      const float eff_threshold = cfg_.degeneracy_eigenvalue_threshold;

      if (cfg_.enable_soft_degeneracy) {
        for (int j = 0; j < 6; ++j) {
          const float ratio = eigenvalues(j) /
              std::max(eff_threshold, 1e-6f);
          const float w = std::min(1.0f,
              std::pow(std::max(ratio, 0.0f), cfg_.degeneracy_soft_power));
          if (w > 1e-4f) {
            V_f += w * eigenvectors.col(j) * eigenvectors.col(j).transpose();
          }
          if (eigenvalues(j) >= cfg_.degeneracy_eigenvalue_threshold)
            ++n_constrained;
        }
      } else {
        for (int j = 0; j < 6; ++j) {
          if (eigenvalues(j) >= eff_threshold) {
            V_f += eigenvectors.col(j) * eigenvectors.col(j).transpose();
            ++n_constrained;
          }
        }
      }

      dx.head<6>() = V_f * dx.head<6>();

      static int degen_log_counter = 0;
      if (iter == 0 && (++degen_log_counter % 100 == 0)) {
        if (cfg_.enable_soft_degeneracy) {
          float w0 = std::min(1.0f, std::pow(std::max(eigenvalues(0) /
              std::max(cfg_.degeneracy_eigenvalue_threshold, 1e-6f), 0.0f),
              cfg_.degeneracy_soft_power));
          float w3 = std::min(1.0f, std::pow(std::max(eigenvalues(3) /
              std::max(cfg_.degeneracy_eigenvalue_threshold, 1e-6f), 0.0f),
              cfg_.degeneracy_soft_power));
          float w5 = std::min(1.0f, std::pow(std::max(eigenvalues(5) /
              std::max(cfg_.degeneracy_eigenvalue_threshold, 1e-6f), 0.0f),
              cfg_.degeneracy_soft_power));
          SPDLOG_INFO("[DEGEN-SOFT] hard_constrained={}/6 eig=[{:.1f},{:.1f},{:.1f},"
                      "{:.1f},{:.1f},{:.1f}] w=[{:.3f},...,{:.3f},...,{:.3f}]",
                      n_constrained,
                      static_cast<double>(eigenvalues(0)),
                      static_cast<double>(eigenvalues(1)),
                      static_cast<double>(eigenvalues(2)),
                      static_cast<double>(eigenvalues(3)),
                      static_cast<double>(eigenvalues(4)),
                      static_cast<double>(eigenvalues(5)),
                      static_cast<double>(w0),
                      static_cast<double>(w3),
                      static_cast<double>(w5));
        } else {
          SPDLOG_INFO("[DEGEN] constrained={}/6 th={:.2f} eig=[{:.1f},{:.1f},{:.1f},"
                      "{:.1f},{:.1f},{:.1f}]",
                      n_constrained,
                      static_cast<double>(eff_threshold),
                      static_cast<double>(eigenvalues(0)),
                      static_cast<double>(eigenvalues(1)),
                      static_cast<double>(eigenvalues(2)),
                      static_cast<double>(eigenvalues(3)),
                      static_cast<double>(eigenvalues(4)),
                      static_cast<double>(eigenvalues(5)));
        }
      }
    }

    // NaN check on dx.
    if (!std::isfinite(dx.sum())) {
      spdlog::warn("[LWO IEKF] dx NaN at iter={}, falling back to prior", iter);
      state = prior;
      converged = false;
      break;
    }

    // Clamp position and rotation correction magnitudes.
    {
      const float pos_norm = dx.segment<3>(kLwoPosIdx).norm();
      if (pos_norm > cfg_.max_pos_correction) {
        dx.segment<3>(kLwoPosIdx) *= cfg_.max_pos_correction / pos_norm;
        spdlog::warn("[LWO IEKF] pos correction clamped: {:.4f}m -> {:.4f}m iter={}",
                     static_cast<double>(pos_norm),
                     static_cast<double>(cfg_.max_pos_correction), iter);
      }
      const float rot_norm = dx.segment<3>(kLwoRotIdx).norm();
      if (rot_norm > cfg_.max_rot_correction) {
        dx.segment<3>(kLwoRotIdx) *= cfg_.max_rot_correction / rot_norm;
        spdlog::warn("[LWO IEKF] rot correction clamped: {:.5f}rad -> {:.5f}rad iter={}",
                     static_cast<double>(rot_norm),
                     static_cast<double>(cfg_.max_rot_correction), iter);
      }
    }

    // Apply state correction on manifold.
    state = state + dx;

    // Hard clamp scale to physically plausible range [0.9, 1.1].
    state.wheel_scale = std::clamp(state.wheel_scale, 0.9f, 1.1f);

    // Hard clamp ext deltas.
    if (cfg_.enable_ext_calibration) {
      state.ext_delta_yaw = std::clamp(state.ext_delta_yaw,
                                        -cfg_.ext_max_delta_yaw,
                                         cfg_.ext_max_delta_yaw);
      state.ext_delta_xy(0) = std::clamp(state.ext_delta_xy(0),
                                          -cfg_.ext_max_delta_xy,
                                           cfg_.ext_max_delta_xy);
      state.ext_delta_xy(1) = std::clamp(state.ext_delta_xy(1),
                                          -cfg_.ext_max_delta_xy,
                                           cfg_.ext_max_delta_xy);
    }

    // G matrix.
    G_final = K1 * HTRinvH;

    // Convergence check.
    const float rot_norm = dx.segment<3>(kLwoRotIdx).norm();
    const float pos_norm = dx.segment<3>(kLwoPosIdx).norm();
    if (rot_norm < cfg_.convergence_threshold &&
        pos_norm < cfg_.convergence_threshold) {
      converged = true;
      break;
    }
  }

  // Posterior covariance: P_post = (I - G) * P_prior.
  LwoStateCovariance P_post =
      (LwoStateCovariance::Identity() - G_final) * P_prior;
  state.covariance = (P_post + P_post.transpose()) * 0.5f;

  // P floor.
  constexpr float kMinPRotRP  = 1e-4f;
  const float kMinPRotYaw = cfg_.p_yaw_floor;
  constexpr float kMinPPos    = 1e-3f;
  constexpr float kMinPVel    = 1e-4f;
  state.covariance(kLwoRotIdx + 0, kLwoRotIdx + 0) =
      std::max(state.covariance(kLwoRotIdx + 0, kLwoRotIdx + 0), kMinPRotRP);
  state.covariance(kLwoRotIdx + 1, kLwoRotIdx + 1) =
      std::max(state.covariance(kLwoRotIdx + 1, kLwoRotIdx + 1), kMinPRotRP);
  state.covariance(kLwoRotIdx + 2, kLwoRotIdx + 2) =
      std::max(state.covariance(kLwoRotIdx + 2, kLwoRotIdx + 2), kMinPRotYaw);
  for (int i = 0; i < 3; ++i) {
    state.covariance(kLwoPosIdx + i, kLwoPosIdx + i) =
        std::max(state.covariance(kLwoPosIdx + i, kLwoPosIdx + i), kMinPPos);
    state.covariance(kLwoVelIdx + i, kLwoVelIdx + i) =
        std::max(state.covariance(kLwoVelIdx + i, kLwoVelIdx + i), kMinPVel);
  }

  // NaN/Inf safety.
  const float trace_post = state.covariance.diagonal().sum();
  const bool state_bad = !std::isfinite(state.position.sum()) ||
                          !std::isfinite(state.rotation.sum()) ||
                          !std::isfinite(state.velocity.sum()) ||
                          !std::isfinite(trace_post);
  if (state_bad) {
    state = prior;
    converged = false;
  }

  LwoIekfResult result;
  result.state              = state;
  result.converged          = converged;
  result.total_iterations   = total_iters;
  result.num_correspondences = n_corr;
  result.jacobian_ms        = result_init.jacobian_ms;
  result.build_info_ms      = result_init.build_info_ms;
  result.solve_ms           = result_init.solve_ms;

  // Wheel measurement diagnostics.
  if (cfg_.enable_wheel_measurement) {
    const WheelMeasurementResult wv_post = wv.compute(state, vx_enc,
                                                       omega_z_enc, omega_z_b);
    result.wv_residual_vx    = wv_post.residual(0);
    result.wv_residual_omega = wv_post.residual(1);
    result.wv_info_vx        = 1.0f / wv_post.noise_cov(0, 0);
    result.wv_info_omega     = 1.0f / wv_post.noise_cov(1, 1);
  }

  return result;
}

}  // namespace lwo
}  // namespace tof_slam
