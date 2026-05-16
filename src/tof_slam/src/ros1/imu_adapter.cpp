// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "tof_slam/ros1/imu_adapter.hpp"

namespace tof_slam {
namespace ros_adapter {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

ImuAdapter::ImuAdapter() : ImuAdapter(Config{}) {}

ImuAdapter::ImuAdapter(const Config& config) : config_(config) {
  init_buffer_.reserve(static_cast<size_t>(config_.init_sample_count));
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

core::ImuMeasurement ImuAdapter::convert(
    const sensor_msgs::Imu::ConstPtr& msg) {
  core::ImuMeasurement m;
  m.timestamp = msg->header.stamp.toSec();
  m.gyro  = Eigen::Vector3f(static_cast<float>(msg->angular_velocity.x),
                             static_cast<float>(msg->angular_velocity.y),
                             static_cast<float>(msg->angular_velocity.z));
  m.accel = Eigen::Vector3f(
      static_cast<float>(msg->linear_acceleration.x),
      static_cast<float>(msg->linear_acceleration.y),
      static_cast<float>(msg->linear_acceleration.z));
  return m;
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

std::optional<core::ImuMeasurement> ImuAdapter::process(
    const sensor_msgs::Imu::ConstPtr& msg) {
  auto measurement = convert(msg);

  if (!initialized_) {
    init_buffer_.push_back(measurement);

    // Sliding window: keep only the last init_sample_count samples.
    while (static_cast<int>(init_buffer_.size()) > config_.init_sample_count) {
      init_buffer_.erase(init_buffer_.begin());
    }

    if (static_cast<int>(init_buffer_.size()) >= config_.init_sample_count) {
      // Attempt gravity initialization with the sliding window.
      init_result_    = core::initialize_gravity(init_buffer_,
                                                  config_.gravity_prior);
      init_attempted_ = true;

      if (init_result_.success) {
        initialized_ = true;
        init_buffer_.clear();  // Free memory after successful init.
      }
      // On failure, keep the sliding window and retry on next sample.
    }

    // Always return nullopt while in buffering / init phase.
    return std::nullopt;
  }

  // Post-initialization: pass measurement through immediately.
  return measurement;
}

void ImuAdapter::reset() {
  initialized_    = false;
  init_attempted_ = false;
  init_result_    = core::GravityInitResult{};
  init_buffer_.clear();
}

}  // namespace ros_adapter
}  // namespace tof_slam
