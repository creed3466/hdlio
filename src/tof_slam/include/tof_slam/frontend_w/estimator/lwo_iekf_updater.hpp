// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// lwo_iekf_updater.hpp — Information-form IEKF update for the 15-D LWO state.
//
// Extends the LIO IEKF (iekf_updater.hpp) to the 15-D LwoState with three
// measurement sources combined in a single update step:
//
//   H_full = [H_lidar (N×15)]
//             [H_gc    (3×15)]
//             [H_wv    (2×15)]
//
//   K1 = (H_full^T R_full^{-1} H_full + P^{-1})^{-1}
//   dx = K1 * H_full^T R_full^{-1} r_full
//   P_post = (I - G) * P_prior,  G = K1 * H_full^T R_full^{-1} H_full

#ifndef TOF_SLAM_FRONTEND_W_ESTIMATOR_LWO_IEKF_UPDATER_HPP_
#define TOF_SLAM_FRONTEND_W_ESTIMATOR_LWO_IEKF_UPDATER_HPP_

#include <vector>

#include "tof_slam/frontend_w/estimator/lwo_state.hpp"
#include "tof_slam/frontend_w/measurement/ground_constraint.hpp"
#include "tof_slam/frontend_w/measurement/wheel_measurement.hpp"
#include "tof_slam/frontend/estimator/correspondence.hpp"
#include "tof_slam/common/lie/se3.hpp"
#include "tof_slam/frontend/robust/pko.hpp"

namespace tof_slam {
namespace lwo {

// ---------------------------------------------------------------------------
// LwoIekfConfig
// ---------------------------------------------------------------------------

struct LwoIekfConfig {
  int   max_inner_iters      = 4;
  float convergence_threshold = 1e-3f;
  float lidar_noise_std       = 0.05f;

  // Covariance floor for yaw (P_yaw_min).
  // Higher → LiDAR always has authority to correct heading.
  // Lower → heading locked to wheel gyro more tightly.
  float p_yaw_floor           = 1e-4f;

  // C2: Adaptive LiDAR noise by surfel planarity (fallback when adaptive off).
  // sigma_i = lidar_noise_std * (1 + planarity_noise_scale * planarity)
  // 0 = disabled, typical value = 2.0-5.0
  float planarity_noise_scale = 0.0f;

  // v1.0 adaptive noise model (range/incidence/planarity/normal_sigma2).
  // When enabled, overrides the simple planarity-only noise model above.
  bool  enable_adaptive_noise     = false;
  float adaptive_range_ref        = 5.0f;   // reference range (m) for range penalty
  float adaptive_range_scale      = 1.0f;   // weight of range penalty term
  float adaptive_incidence_scale  = 10.0f;  // weight of incidence penalty term
  float adaptive_planarity_scale  = 2.0f;   // weight of planarity penalty term

  // L2 multi-scale correspondences (coarser surfel layer).
  bool  enable_l2_correspondences = false;
  float l2_noise_scale            = 4.0f;   // L2 noise multiplier vs L1

  // Degeneracy-aware selective update (Zhang & Singh ICRA 2016).
  // Projects dx onto well-constrained subspace of HTRinvH using eigenvalue
  // decomposition.  Directions with LiDAR eigenvalue below threshold are
  // suppressed, preventing corrections in degenerate directions (e.g.,
  // heading with sparse ToF, or position along corridor axis).
  bool  enable_degeneracy_projection = false;
  float degeneracy_eigenvalue_threshold = 10.0f;  // min LiDAR eigenvalue to trust

  // Soft degeneracy: smooth tapering instead of hard threshold.
  // weight_i = clamp((eigenvalue_i / threshold)^power, 0, 1)
  // When disabled, uses hard on/off selection.
  bool  enable_soft_degeneracy = false;
  float degeneracy_soft_power  = 2.0f;  // tapering exponent (1=linear, 2=quadratic)

  // Per-iteration correction magnitude clamp.
  // Prevents catastrophic divergence from outlier correspondences or
  // ill-conditioned information matrices.  Direction is preserved.
  float max_pos_correction = 0.05f;   // meters per inner iter (5cm)
  float max_rot_correction = 0.035f;  // radians per inner iter (~2 deg)

  // Sub-module timing (populated when check_usage is true)
  bool check_usage = false;

  // Wheel velocity/omega measurement update (completes the H_wv rows
  // described in the header).  When enabled, encoder vx and omega_z are
  // fused as IEKF measurements alongside LiDAR and ground constraint,
  // acting as a regulariser that prevents LiDAR-induced drift.
  bool  enable_wheel_measurement    = false;   // Master switch (default off for backward compat)

  // Online extrinsic calibration
  bool  enable_ext_calibration      = false;   // Master switch (disabled by default)
  float ext_obs_min_omega            = 0.1f;   // Min |omega_z| for yaw observability (rad/s)
  int   ext_obs_min_correspondences  = 100;    // Min correspondences for ext update
  float ext_max_delta_yaw            = 0.01f;  // Max allowed cumulative yaw correction (rad)
  float ext_max_delta_xy             = 0.01f;  // Max allowed cumulative XY correction (m)
};

// ---------------------------------------------------------------------------
// LwoIekfResult
// ---------------------------------------------------------------------------

struct LwoIekfResult {
  LwoState state;
  bool converged          = false;
  int  total_iterations   = 0;
  int  num_correspondences = 0;
  float dx_rot_z          = 0.0f;  // Raw yaw correction before gain/zeroing

  // Wheel measurement diagnostics (populated when enable_wheel_measurement)
  float wv_residual_vx    = 0.0f;  // vx_enc - vx_body/s (before IEKF)
  float wv_residual_omega = 0.0f;  // omega_enc - omega_b - b_oz
  float wv_info_vx        = 0.0f;  // 1/sigma_vx^2 (information weight)
  float wv_info_omega     = 0.0f;  // 1/sigma_omega^2

  // Sub-module timing (populated when LwoIekfConfig::check_usage is true)
  float jacobian_ms   = 0.0f;
  float build_info_ms = 0.0f;
  float solve_ms      = 0.0f;
};

// ---------------------------------------------------------------------------
// LwoIekfUpdater
// ---------------------------------------------------------------------------

class LwoIekfUpdater {
 public:
  explicit LwoIekfUpdater(const LwoIekfConfig& cfg = {});

  /// Run the information-form IEKF inner loop with combined measurements.
  ///
  /// @param prior            Prior LwoState (from WheelPropagator).
  /// @param correspondences  Point-to-plane correspondences (world-frame).
  /// @param T_body_lidar     LiDAR-to-body extrinsic transform.
  /// @param gc               GroundConstraint measurement source.
  /// @param wv               WheelMeasurement measurement source.
  /// @param vx_enc           Encoder forward velocity (m/s).
  /// @param omega_z_enc      Encoder yaw rate (rad/s).
  /// @param omega_z_b        Body yaw rate for WheelMeasurement residual row-1.
  /// @param pko              Optional PKO for adaptive Huber (nullptr = unit).
  LwoIekfResult update(
      const LwoState& prior,
      const std::vector<core::Correspondence>& correspondences,
      const core::Se3& T_body_lidar,
      const GroundConstraint& gc,
      const WheelMeasurement& wv,
      float vx_enc,
      float omega_z_enc,
      float omega_z_b,
      core::Pko* pko = nullptr) const;

  /// Mutable access to config for per-frame parameter adjustment (e.g. velocity-adaptive noise).
  LwoIekfConfig& mutable_cfg() { return cfg_; }

 private:
  LwoIekfConfig cfg_;
};

}  // namespace lwo
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_W_ESTIMATOR_LWO_IEKF_UPDATER_HPP_
