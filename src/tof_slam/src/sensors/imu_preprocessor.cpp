#include "tof_slam/sensors/imu_preprocessor.hpp"

#include <cmath>
#include <rclcpp/time.hpp>

namespace tof_slam {

ImuPreprocessor::ImuPreprocessor(const TofSlamConfig& config) : config_(config) {}

void ImuPreprocessor::feed(const sensor_msgs::msg::Imu::SharedPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);

  ImuSample sample;
  sample.stamp = rclcpp::Time(msg->header.stamp).seconds();
  sample.gyro = Eigen::Vector3d(
    msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  sample.accel = Eigen::Vector3d(
    msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

  buffer_.push_back(sample);

  // Gravity alignment: incremental mean (Super-LIO kf_init)
  if (!gravity_aligned_) {
    init_sample_count_++;
    mean_gyro_ += (sample.gyro - mean_gyro_) / init_sample_count_;
    mean_accel_ += (sample.accel - mean_accel_) / init_sample_count_;

    if (init_sample_count_ >= config_.imu_init_samples) {
      // Compute gravity direction from mean accelerometer
      double g_norm = std::abs(config_.imu_gravity_z);  // typically 9.81
      Eigen::Vector3d gravity = -mean_accel_ * g_norm / mean_accel_.norm();
      Eigen::Vector3d ref_gravity(0, 0, -g_norm);

      // Rotation aligning measured gravity to reference
      Eigen::Matrix3d init_rot =
        Eigen::Quaterniond::FromTwoVectors(gravity, ref_gravity).toRotationMatrix();

      // Remove yaw component (make initial heading zero)
      Eigen::Vector3d n = init_rot.col(0);
      double yaw = std::atan2(n(1), n(0));
      Eigen::Matrix3d R_yaw_inv =
        Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      Eigen::Matrix3d rot = R_yaw_inv * init_rot;

      initial_rotation_ = Eigen::Quaterniond(rot);
      initial_rotation_.normalize();
      imu_scale_ = g_norm / mean_accel_.norm();
      gravity_aligned_ = true;
    }
  }
}

void ImuPreprocessor::getInitialBiases(Eigen::Vector3d& bg, Eigen::Vector3d& ba) const {
  bg = mean_gyro_;               // Static gyro mean = bias estimate
  ba = Eigen::Vector3d::Zero();  // Super-LIO sets ba=0 initially
}

std::vector<ImuSample> ImuPreprocessor::consumeSamples(
    const rclcpp::Time& t0, const rclcpp::Time& t1) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<ImuSample> result;

  // Super-LIO approach: pop ALL samples with stamp <= t1 and return them.
  // The ESKF's predict() handles time boundary logic internally.
  (void)t0;  // unused — no interpolation boundary retained
  const double t1_sec = t1.seconds();
  while (!buffer_.empty() && buffer_.front().stamp <= t1_sec) {
    result.push_back(buffer_.front());
    buffer_.pop_front();
  }

  return result;
}

rclcpp::Time ImuPreprocessor::getLatestTimestamp() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (buffer_.empty()) return rclcpp::Time(0, 0, RCL_ROS_TIME);
  return rclcpp::Time(
      static_cast<int64_t>(buffer_.back().stamp * 1e9), RCL_ROS_TIME);
}

}  // namespace tof_slam
