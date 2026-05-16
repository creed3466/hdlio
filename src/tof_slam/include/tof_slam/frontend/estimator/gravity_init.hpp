// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// gravity_init.hpp — Stationary gravity alignment and bias estimation.
//
// Merges logic from Estimator::GravityInitialization() and the inline
// g-unit detection in iglio_slam_node.cpp into one pure function.

#ifndef TOF_SLAM_FRONTEND_ESTIMATOR_GRAVITY_INIT_HPP_
#define TOF_SLAM_FRONTEND_ESTIMATOR_GRAVITY_INIT_HPP_

#include <string>
#include <vector>

#include "tof_slam/common/types/imu_types.hpp"
#include "tof_slam/common/types/state.hpp"

namespace tof_slam {
namespace core {

/// Result of gravity initialization.
struct GravityInitResult {
  bool success = false;
  LioState initial_state;
  /// Scale to apply to all future accelerometer readings (>1 when sensor
  /// reports in g-units).
  float imu_acc_scale = 1.0f;
  std::string message;
  /// Timestamp of the last IMU sample used during initialization.
  /// Set so that the first feed_imu() after initialize() computes a valid dt.
  double last_imu_timestamp = 0.0;
};

/// Initialize the LIO state from a buffer of stationary IMU samples.
///
/// 1. Checks minimum sample count (default 20).
/// 2. Auto-detects g-units (|mean_acc| < 2.0 → scale = gravity_norm/|mean_acc|).
/// 3. Aligns the world frame so that gravity matches `gravity_prior`.
/// 4. Estimates initial gyro and accelerometer biases.
///
/// @param buffer         Stationary IMU samples (already in m/s^2 OR g-units).
/// @param gravity_prior  Desired gravity vector in world frame
///                       (default: [0, 0, -9.81]).
/// @param min_samples    Minimum number of samples required.
GravityInitResult initialize_gravity(
    const std::vector<ImuMeasurement>& buffer,
    const Eigen::Vector3f& gravity_prior =
        Eigen::Vector3f(0.0f, 0.0f, -kGravityNorm),
    int min_samples = 20);

}  // namespace core
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_ESTIMATOR_GRAVITY_INIT_HPP_
