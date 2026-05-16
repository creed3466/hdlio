// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// imu_adapter.hpp — Converts ROS IMU messages and manages gravity
// initialization buffer.

#ifndef TOF_SLAM_ROS1_IMU_ADAPTER_HPP_
#define TOF_SLAM_ROS1_IMU_ADAPTER_HPP_

#include <sensor_msgs/Imu.h>
#include <optional>
#include <vector>

#include "tof_slam/common/types/imu_types.hpp"
#include "tof_slam/frontend/estimator/gravity_init.hpp"

namespace tof_slam {
namespace ros_adapter {

/// Converts ROS IMU messages and manages gravity initialization buffer.
///
/// Usage:
/// 1. Call process() for each incoming IMU message.
/// 2. Before gravity init: buffers samples, returns nullopt.
/// 3. When buffer reaches init_sample_count: calls initialize_gravity(),
///    returns nullopt for that batch (samples consumed by init).
/// 4. After init: returns converted ImuMeasurement immediately.
class ImuAdapter {
 public:
  struct Config {
    /// Number of stationary IMU samples to collect before gravity init.
    int init_sample_count = 100;
    /// Expected gravity direction in world frame (m/s^2).
    Eigen::Vector3f gravity_prior = Eigen::Vector3f(0.0f, 0.0f, -9.81f);
  };

  /// Construct with default configuration.
  ImuAdapter();
  /// Construct with custom configuration.
  explicit ImuAdapter(const Config& config);

  /// Process an IMU message.
  ///
  /// @param msg  Incoming ROS IMU message (non-null).
  /// @return     ImuMeasurement after gravity init completes; nullopt while
  ///             buffering or at the exact sample that triggers init.
  std::optional<core::ImuMeasurement> process(
      const sensor_msgs::Imu::ConstPtr& msg);

  /// Returns true once gravity initialization has succeeded.
  bool initialized() const { return initialized_; }

  /// Returns true if initialization was attempted (may have failed).
  bool init_attempted() const { return init_attempted_; }

  /// Gravity init result — only meaningful after initialized() == true.
  const core::GravityInitResult& init_result() const { return init_result_; }

  /// Reset to pre-initialization state (clears buffer and result).
  void reset();

 private:
  /// Convert a ROS IMU message to a core ImuMeasurement (no scaling applied).
  static core::ImuMeasurement convert(
      const sensor_msgs::Imu::ConstPtr& msg);

  Config config_;
  bool initialized_    = false;
  bool init_attempted_ = false;
  core::GravityInitResult init_result_;
  std::vector<core::ImuMeasurement> init_buffer_;
};

}  // namespace ros_adapter
}  // namespace tof_slam

#endif  // TOF_SLAM_ROS1_IMU_ADAPTER_HPP_
