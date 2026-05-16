// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// imu_preintegration.hpp — Forster et al. 2015 on-manifold IMU preintegration.
//
// Accumulates IMU measurements between two LiDAR keyframes into a compact
// preintegrated measurement (delta_R, delta_v, delta_p) with covariance
// and bias-correction Jacobians.  Used by the Fixed-Lag Smoother as the
// sole cross-frame coupling factor.

#ifndef TOF_SLAM_FRONTEND_ESTIMATOR_IMU_PREINTEGRATION_HPP_
#define TOF_SLAM_FRONTEND_ESTIMATOR_IMU_PREINTEGRATION_HPP_

#include <vector>

#include <Eigen/Dense>

#include "tof_slam/common/types/imu_types.hpp"

namespace tof_slam {
namespace core {

// ---------------------------------------------------------------------------
// ImuPreintegration — preintegrated IMU measurement
// ---------------------------------------------------------------------------

/// Preintegrated IMU measurement between two LiDAR keyframes.
///
/// State ordering in the 9×9 covariance:
///   [0:3] delta_rotation (tangent space)
///   [3:6] delta_velocity
///   [6:9] delta_position
struct ImuPreintegration {
  // --- Preintegrated deltas (bias-corrected at construction time) ---
  Eigen::Matrix3f delta_R = Eigen::Matrix3f::Identity();
  Eigen::Vector3f delta_v = Eigen::Vector3f::Zero();
  Eigen::Vector3f delta_p = Eigen::Vector3f::Zero();
  double delta_t = 0.0;

  // 9×9 covariance: [delta_rot(3), delta_vel(3), delta_pos(3)]
  Eigen::Matrix<double, 9, 9> covariance =
      Eigen::Matrix<double, 9, 9>::Zero();

  // --- Bias-correction Jacobians (3×3 each) ---
  // First-order correction when bias estimate changes without full
  // re-preintegration:
  //   delta_R_corrected = delta_R * Exp(J_bg_rot * delta_bg)
  //   delta_v_corrected = delta_v + J_bg_vel * delta_bg + J_ba_vel * delta_ba
  //   delta_p_corrected = delta_p + J_bg_pos * delta_bg + J_ba_pos * delta_ba
  Eigen::Matrix3f J_bg_rot = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f J_bg_vel = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f J_ba_vel = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f J_bg_pos = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f J_ba_pos = Eigen::Matrix3f::Zero();

  // --- Bias linearization point ---
  Eigen::Vector3f bg_lin = Eigen::Vector3f::Zero();
  Eigen::Vector3f ba_lin = Eigen::Vector3f::Zero();

  int num_measurements = 0;

  /// Check if preintegration contains valid data.
  bool valid() const { return num_measurements > 0 && delta_t > 0.0; }
};

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// Accumulate IMU measurements between two LiDAR frames into a preintegrated
/// measurement.
///
/// Uses midpoint integration (consistent with propagate_imu in
/// imu_propagator.cpp) for delta accumulation and covariance propagation.
///
/// @param imu_buffer          IMU measurements in [t_{k-1}, t_k], sorted.
/// @param bg                  Gyro bias estimate at linearization point.
/// @param ba                  Accel bias estimate at linearization point.
/// @param gyro_noise_density  Gyro noise density [rad/s/sqrt(Hz)].
/// @param acc_noise_density   Accel noise density [m/s^2/sqrt(Hz)].
/// @return                    Preintegrated measurement.
ImuPreintegration preintegrate_imu(
    const std::vector<ImuMeasurement>& imu_buffer,
    const Eigen::Vector3f& bg,
    const Eigen::Vector3f& ba,
    float gyro_noise_density,
    float acc_noise_density);

}  // namespace core
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_ESTIMATOR_IMU_PREINTEGRATION_HPP_
