#pragma once

#include <deque>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include "tof_slam/common/types.hpp"
#include "tof_slam/common/config.hpp"

namespace tof_slam {

/// Builds relative SE(3) wheel prior from consecutive odometry messages
/// [Algorithm.md §4, Implement_Plan.md §3.4]
class WheelPriorBuilder {
 public:
  explicit WheelPriorBuilder(const TofSlamConfig& config);

  /// Feed an odometry message. Returns true if a WheelPrior is available.
  bool feed(const nav_msgs::msg::Odometry::SharedPtr& msg);

  /// Get the accumulated wheel prior since last call and reset accumulator.
  /// This returns the full motion between pointcloud arrivals.
  WheelPrior consumePrior();

  /// Get accumulated prior from t0 to current time (for scan alignment)
  WheelPrior getAccumulatedPrior(const rclcpp::Time& t0, const rclcpp::Time& t1) const;

  /// Interpolate wheel odometry pose at given timestamp (for motion distortion correction)
  /// Returns false if timestamp is outside the buffered odom range
  bool interpolatePoseAt(const rclcpp::Time& t,
                         Eigen::Quaterniond& q_out,
                         Eigen::Vector3d& p_out) const;

  /// Reset state
  void reset();

  bool isInitialized() const { return initialized_; }

 private:
  /// Single odom sample for ring buffer
  struct OdomSample {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  };
  static constexpr int kOdomBufferSize = 200;
  /// Compute anisotropic motion-proportional covariance
  Eigen::Matrix<double, 6, 6> computeCovariance(
      const Eigen::Vector3d& delta_translation,
      const Eigen::Vector3d& delta_rotation) const;

  /// Compute simple slip score from motion characteristics
  double computeSlipScore(const Eigen::Vector3d& delta_trans,
                          const Eigen::Vector3d& delta_rot,
                          const nav_msgs::msg::Odometry::SharedPtr& msg) const;

  const TofSlamConfig& config_;
  bool initialized_{false};

  // Previous odometry state (per-message tracking)
  Eigen::Quaterniond prev_q_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d prev_p_{Eigen::Vector3d::Zero()};
  rclcpp::Time prev_stamp_{0, 0, RCL_ROS_TIME};

  // Accumulated relative transform since last getLatestPrior() call
  // This accumulates all odom increments between pointcloud arrivals
  Eigen::Matrix4d accumulated_T_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix<double, 6, 6> accumulated_Q_{Eigen::Matrix<double, 6, 6>::Zero()};
  double accumulated_slip_{0.0};
  rclcpp::Time accum_start_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time accum_end_stamp_{0, 0, RCL_ROS_TIME};
  int accum_count_{0};

  WheelPrior latest_prior_;

  // Ring buffer for motion distortion correction (interpolatePoseAt)
  std::deque<OdomSample> odom_buffer_;
};

}  // namespace tof_slam
