// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// imu_types.hpp — IMU measurement type (no ROS dependency).

#ifndef TOF_SLAM_COMMON_TYPES_IMU_TYPES_HPP_
#define TOF_SLAM_COMMON_TYPES_IMU_TYPES_HPP_

#include <Eigen/Dense>

namespace tof_slam {
namespace core {

/// Single IMU sample.  Timestamps are in seconds (double precision for
/// nanosecond-level timing; measurement data is float32 to match the EKF
/// state precision).
struct ImuMeasurement {
  double timestamp = 0.0;        // Seconds (e.g., ROS header stamp).
  Eigen::Vector3f gyro   = Eigen::Vector3f::Zero();  // rad/s
  Eigen::Vector3f accel  = Eigen::Vector3f::Zero();  // m/s^2
};

}  // namespace core
}  // namespace tof_slam

#endif  // TOF_SLAM_COMMON_TYPES_IMU_TYPES_HPP_
