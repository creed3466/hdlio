// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// fixed_lag_smoother.hpp — 2-frame velocity-coupled Gauss-Newton smoother.
//
// Jointly optimizes frames k-1 and k using:
//   - Prior factors from IEKF posteriors
//   - LiDAR point-to-plane factors (frame k-1 only — frame k's info is
//     already absorbed into P_k prior, avoiding double-counting)
//   - IMU preintegration factor (sole genuine cross-frame coupling)
//
// This replaces backward_refine() which was shown to be mathematically
// equivalent to re-running IEKF independently (frozen-map LiDAR-only
// Hessian decomposes into independent blocks without IMU cross-term).

#ifndef TOF_SLAM_FRONTEND_ESTIMATOR_FIXED_LAG_SMOOTHER_HPP_
#define TOF_SLAM_FRONTEND_ESTIMATOR_FIXED_LAG_SMOOTHER_HPP_

#include <vector>

#include "tof_slam/common/lie/se3.hpp"
#include "tof_slam/common/types/state.hpp"
#include "tof_slam/frontend/estimator/correspondence.hpp"
#include "tof_slam/frontend/estimator/iekf_updater.hpp"
#include "tof_slam/frontend/estimator/imu_preintegration.hpp"

namespace tof_slam {
namespace core {

// ---------------------------------------------------------------------------
// FlsConfig
// ---------------------------------------------------------------------------

struct FlsConfig {
  int max_iterations = 3;
  float convergence_threshold = 1e-4f;
  float max_pos_correction = 0.05f;   // 5cm cap
  float max_rot_correction = 0.087f;  // 5° cap in radians
  float max_vel_correction = 0.5f;    // 0.5 m/s cap
  // IMU noise scale: the IEKF process noise parameters (gyro/acc_noise_std)
  // are often tuned very tight for propagation.  For preintegration the
  // covariance must represent actual sensor uncertainty, otherwise Q_inv
  // overwhelms the IEKF prior.  This multiplier scales both gyro and accel
  // noise densities before preintegration.  Default 30 empirically balances
  // IMU information with IEKF prior for VN100 @ 400Hz.
  float imu_noise_scale = 500.0f;
};

// ---------------------------------------------------------------------------
// FlsResult
// ---------------------------------------------------------------------------

struct FlsResult {
  LioState state_curr;          // corrected state for frame k
  bool converged = false;
  bool applied = false;         // true if correction was applied (not rejected)
  int iterations = 0;
  float dx_norm_final = 0.0f;
  float pos_correction_curr = 0.0f;
  float rot_correction_curr = 0.0f;
  float vel_correction_curr = 0.0f;
  float cross_block_norm = 0.0f;  // Frobenius norm of H_01 (coupling proof)
};

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// 2-frame Gauss-Newton fixed-lag smoother.
///
/// Jointly optimizes frames k-1 and k with 18-dim state
/// (per frame: delta_theta 3, delta_p 3, delta_v 3).
///
/// @param state_prev     IEKF posterior for frame k-1 (warm start).
/// @param state_curr     IEKF posterior for frame k (warm start).
/// @param P_prev         IEKF posterior covariance for frame k-1.
/// @param P_curr         IEKF posterior covariance for frame k.
/// @param corrs_prev     Correspondences for frame k-1 (re-queried against
///                       current map — genuinely new information).
/// @param preint         IMU preintegration between frames k-1 and k.
/// @param T_body_lidar   LiDAR-body extrinsic.
/// @param iekf_config    IEKF config (for noise params).
/// @param fls_config     FLS-specific config.
/// @return               Smoothed state for frame k and diagnostics.
FlsResult fixed_lag_smooth(
    const LioState& state_prev,
    const LioState& state_curr,
    const StateCovariance& P_prev,
    const StateCovariance& P_curr,
    const std::vector<Correspondence>& corrs_prev,
    const ImuPreintegration& preint,
    const Se3& T_body_lidar,
    const IekfConfig& iekf_config,
    const FlsConfig& fls_config);

}  // namespace core
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_ESTIMATOR_FIXED_LAG_SMOOTHER_HPP_
