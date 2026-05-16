#pragma once

#include <deque>
#include <mutex>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>

#include "tof_slam/common/types.hpp"
#include "tof_slam/common/config.hpp"

namespace tof_slam {

/// IMU preprocessor: buffer management, gravity alignment, bias estimation
/// Follows Super-LIO's initialization procedure (kf_init)
class ImuPreprocessor {
public:
  explicit ImuPreprocessor(const TofSlamConfig& config);

  /// Feed a new IMU message into the buffer
  void feed(const sensor_msgs::Imu::ConstPtr& msg);

  /// Whether gravity alignment initialization is complete
  /// (requires imu_init_samples worth of static data)
  bool isGravityAligned() const { return gravity_aligned_; }
  bool isInitialized() const { return gravity_aligned_; }

  /// Get initial rotation from gravity alignment
  /// (aligns measured gravity to [0,0,-g_norm], removes yaw)
  Eigen::Quaterniond getInitialRotation() const { return initial_rotation_; }

  /// Get initial gyro bias (mean of static gyro readings)
  void getInitialBiases(Eigen::Vector3d& bg, Eigen::Vector3d& ba) const;

  /// Get IMU scale factor: g_norm / ||mean_accel||
  double getImuScale() const { return imu_scale_; }

  /// Extract IMU samples in time range [t0, t1] and remove consumed data
  std::vector<ImuSample> consumeSamples(const ros::Time& t0, const ros::Time& t1);

  /// Get timestamp of latest buffered sample
  ros::Time getLatestTimestamp() const;

private:
  const TofSlamConfig& config_;

  mutable std::mutex mutex_;
  std::deque<ImuSample> buffer_;

  // Initialization state (Super-LIO kf_init equivalent)
  bool gravity_aligned_ = false;
  int init_sample_count_ = 0;
  Eigen::Vector3d mean_gyro_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d mean_accel_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond initial_rotation_{Eigen::Quaterniond::Identity()};
  double imu_scale_ = 1.0;
};

}  // namespace tof_slam
