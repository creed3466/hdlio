#include "tof_slam/sensors/wheel_prior_builder.hpp"
#include "tof_slam/common/se3.hpp"

#include <cmath>
#include <algorithm>

namespace tof_slam {

WheelPriorBuilder::WheelPriorBuilder(const TofSlamConfig& config)
    : config_(config) {}

bool WheelPriorBuilder::feed(const nav_msgs::msg::Odometry::SharedPtr& msg) {
  // Extract pose from odometry message
  const auto& pos = msg->pose.pose.position;
  const auto& ori = msg->pose.pose.orientation;

  Eigen::Quaterniond curr_q(ori.w, ori.x, ori.y, ori.z);
  curr_q.normalize();
  Eigen::Vector3d curr_p(pos.x, pos.y, pos.z);

  rclcpp::Time curr_stamp(msg->header.stamp);

  // First message: store as previous and return false
  if (!initialized_) {
    prev_q_ = curr_q;
    prev_p_ = curr_p;
    prev_stamp_ = curr_stamp;
    accum_start_stamp_ = curr_stamp;
    initialized_ = true;
    // Store first sample in ring buffer
    odom_buffer_.push_back({curr_stamp, curr_q, curr_p});
    return false;
  }

  // Store current sample in ring buffer for motion distortion interpolation
  odom_buffer_.push_back({curr_stamp, curr_q, curr_p});
  if (static_cast<int>(odom_buffer_.size()) > kOdomBufferSize) {
    odom_buffer_.pop_front();
  }

  // Compute incremental relative transform: DeltaT = T_prev^{-1} * T_curr
  Eigen::Matrix4d T_prev = se3::toTransform(prev_q_, prev_p_);
  Eigen::Matrix4d T_curr = se3::toTransform(curr_q, curr_p);
  Eigen::Matrix4d T_prev_inv = se3::inverseSE3(T_prev);
  Eigen::Matrix4d delta_T = T_prev_inv * T_curr;

  // Extract relative rotation and translation from delta_T
  Eigen::Quaterniond delta_q;
  Eigen::Vector3d delta_p;
  se3::fromTransform(delta_T, delta_q, delta_p);

  // Compute rotation vector via LogSO3
  Eigen::Vector3d delta_phi = se3::LogSO3(delta_q.toRotationMatrix());

  // Build anisotropic covariance for this increment
  Eigen::Matrix<double, 6, 6> Q = computeCovariance(delta_p, delta_phi);

  // Compute slip score
  double slip = computeSlipScore(delta_p, delta_phi, msg);

  // Inflate covariance by slip score
  if (slip > 0.0) {
    double inflation = 1.0 + config_.slip_inflation_gain * slip;
    Q *= inflation;
  }

  // Accumulate: compose transforms and sum covariances
  accumulated_T_ = accumulated_T_ * delta_T;
  accumulated_Q_ += Q;
  accumulated_slip_ = std::max(accumulated_slip_, slip);
  accum_end_stamp_ = curr_stamp;
  ++accum_count_;

  // Extract accumulated relative motion for latest_prior_
  Eigen::Quaterniond accum_q;
  Eigen::Vector3d accum_p;
  se3::fromTransform(accumulated_T_, accum_q, accum_p);

  latest_prior_.q_bbnext = accum_q;
  latest_prior_.p_bbnext = accum_p;
  latest_prior_.Q = accumulated_Q_;
  latest_prior_.slip_score = accumulated_slip_;
  latest_prior_.t0 = accum_start_stamp_.seconds();
  latest_prior_.t1 = curr_stamp.seconds();

  // Update previous state for next increment
  prev_q_ = curr_q;
  prev_p_ = curr_p;
  prev_stamp_ = curr_stamp;

  return true;
}

Eigen::Matrix<double, 6, 6> WheelPriorBuilder::computeCovariance(
    const Eigen::Vector3d& delta_translation,
    const Eigen::Vector3d& delta_rotation) const {
  // delta_rotation = [roll_delta, pitch_delta, yaw_delta]
  // delta_translation = [dx, dy, dz]
  const double abs_dx = std::abs(delta_translation.x());
  const double abs_dyaw = std::abs(delta_rotation.z());

  // Observed axes: motion-proportional with baseline
  const double var_x = config_.wheel_cov_x * (1.0 + abs_dx + 0.5 * abs_dyaw);
  const double var_y = config_.wheel_cov_y * (1.0 + abs_dx + abs_dyaw);
  const double var_yaw = config_.wheel_cov_yaw * (1.0 + 0.5 * abs_dx + abs_dyaw);

  // Unobserved axes: constant large uncertainty
  const double var_roll = config_.wheel_cov_roll;
  const double var_pitch = config_.wheel_cov_pitch;
  const double var_z = config_.wheel_cov_z;

  // Convention: xi = [phi(rotation); rho(translation)], rotation FIRST
  // diag(sigma^2_roll, sigma^2_pitch, sigma^2_yaw, sigma^2_x, sigma^2_y, sigma^2_z)
  Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
  Q(0, 0) = var_roll;
  Q(1, 1) = var_pitch;
  Q(2, 2) = var_yaw;
  Q(3, 3) = var_x;
  Q(4, 4) = var_y;
  Q(5, 5) = var_z;

  return Q;
}

double WheelPriorBuilder::computeSlipScore(
    const Eigen::Vector3d& delta_trans,
    const Eigen::Vector3d& delta_rot,
    const nav_msgs::msg::Odometry::SharedPtr& msg) const {
  // Simple heuristic: check if lateral velocity is high relative to forward
  // Use twist from odometry if available
  const double vx = msg->twist.twist.linear.x;
  const double vy = msg->twist.twist.linear.y;

  // If forward velocity is negligible, no meaningful slip estimate
  const double forward_speed = std::abs(vx);
  if (forward_speed < 1e-3) {
    return 0.0;
  }

  // Slip score = ratio of lateral to forward velocity, clamped to [0, 1]
  double lateral_ratio = std::abs(vy) / forward_speed;
  return std::clamp(lateral_ratio, 0.0, 1.0);
}

WheelPrior WheelPriorBuilder::consumePrior() {
  WheelPrior result = latest_prior_;

  // Reset accumulator for next interval
  accumulated_T_ = Eigen::Matrix4d::Identity();
  accumulated_Q_ = Eigen::Matrix<double, 6, 6>::Zero();
  accumulated_slip_ = 0.0;
  accum_start_stamp_ = accum_end_stamp_;
  accum_count_ = 0;

  return result;
}

WheelPrior WheelPriorBuilder::getAccumulatedPrior(
    const rclcpp::Time& /*t0*/, const rclcpp::Time& /*t1*/) const {
  return latest_prior_;
}

void WheelPriorBuilder::reset() {
  initialized_ = false;
  prev_q_ = Eigen::Quaterniond::Identity();
  prev_p_ = Eigen::Vector3d::Zero();
  prev_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  latest_prior_ = WheelPrior{};
  odom_buffer_.clear();
}

bool WheelPriorBuilder::interpolatePoseAt(const rclcpp::Time& t,
                                           Eigen::Quaterniond& q_out,
                                           Eigen::Vector3d& p_out) const {
  if (odom_buffer_.size() < 2) return false;

  // Check range
  if (t < odom_buffer_.front().stamp || t > odom_buffer_.back().stamp) {
    return false;
  }

  // Binary search: find first sample with stamp >= t
  size_t lo = 0;
  size_t hi = odom_buffer_.size() - 1;
  while (lo + 1 < hi) {
    const size_t mid = (lo + hi) / 2;
    if (odom_buffer_[mid].stamp <= t) {
      lo = mid;
    } else {
      hi = mid;
    }
  }

  const OdomSample& s0 = odom_buffer_[lo];
  const OdomSample& s1 = odom_buffer_[hi];

  const double dt = (s1.stamp - s0.stamp).seconds();
  if (dt < 1e-9) {
    q_out = s0.q;
    p_out = s0.p;
    return true;
  }

  const double alpha = std::clamp((t - s0.stamp).seconds() / dt, 0.0, 1.0);
  q_out = s0.q.slerp(alpha, s1.q);
  p_out = s0.p + alpha * (s1.p - s0.p);
  return true;
}

}  // namespace tof_slam
