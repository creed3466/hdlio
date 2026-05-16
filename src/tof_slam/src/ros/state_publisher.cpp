// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// state_publisher.cpp — ROS 2 state publishing implementation.

#include "tof_slam/ros/state_publisher.hpp"

#include <cstring>
#include <Eigen/Dense>

namespace tof_slam {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
StatePublisher::StatePublisher(const Config& config,
                               const Publishers& pubs,
                               rclcpp::Logger logger,
                               rclcpp::Clock::SharedPtr clock,
                               std::function<Eigen::Matrix4d()> map_correction_provider)
    : config_(config),
      pubs_(pubs),
      logger_(logger),
      clock_(std::move(clock)),
      map_correction_provider_(std::move(map_correction_provider)) {
  path_msg_.header.frame_id = config_.map_frame;
}

// ---------------------------------------------------------------------------
// update_odom_pose — thread-safe, called from wheel_callback
// ---------------------------------------------------------------------------
void StatePublisher::update_odom_pose(const Eigen::Vector3f& position,
                                      const Eigen::Quaternionf& orientation) {
  std::lock_guard<std::mutex> lk(odom_pose_mutex_);
  odom_position_      = position;
  odom_rotation_      = orientation.toRotationMatrix();
  odom_pose_received_ = true;

  if (!odom_origin_set_) {
    odom_origin_pos_     = position;
    odom_origin_rot_inv_ = odom_rotation_.transpose();
    odom_origin_set_     = true;
  }
}

// ---------------------------------------------------------------------------
// get_relative_odom — relative pose from origin (for CSV)
// ---------------------------------------------------------------------------
std::pair<Eigen::Vector3f, Eigen::Quaternionf>
StatePublisher::get_relative_odom() const {
  std::lock_guard<std::mutex> lk(odom_pose_mutex_);
  if (!odom_pose_received_ || !odom_origin_set_) {
    return {Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity()};
  }
  const Eigen::Vector3f rel_pos =
      odom_origin_rot_inv_ * (odom_position_ - odom_origin_pos_);
  const Eigen::Matrix3f rel_rot = odom_origin_rot_inv_ * odom_rotation_;
  Eigen::Quaternionf rel_q(rel_rot);
  rel_q.normalize();
  return {rel_pos, rel_q};
}

// ---------------------------------------------------------------------------
// publish_lio_state — LIO odometry, path, TF (map->base_link)
// ---------------------------------------------------------------------------
void StatePublisher::publish_lio_state(const core::LioState& state,
                                       double timestamp) {
  const rclcpp::Time ros_time(static_cast<int64_t>(timestamp * 1e9));
  Eigen::Quaternionf q(state.rotation);
  q.normalize();

  // Odometry
  nav_msgs::msg::Odometry odom;
  odom.header.stamp    = ros_time;
  odom.header.frame_id = config_.map_frame;
  odom.child_frame_id  = config_.base_frame;
  odom.pose.pose.position.x    = state.position.x();
  odom.pose.pose.position.y    = state.position.y();
  odom.pose.pose.position.z    = state.position.z();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  odom.twist.twist.linear.x    = state.velocity.x();
  odom.twist.twist.linear.y    = state.velocity.y();
  odom.twist.twist.linear.z    = state.velocity.z();
  pubs_.odom_pub->publish(odom);

  // Path
  geometry_msgs::msg::PoseStamped ps;
  ps.header = odom.header;
  ps.pose   = odom.pose.pose;
  path_msg_.header.stamp = ros_time;
  path_msg_.poses.push_back(ps);
  pubs_.path_pub->publish(path_msg_);

  // TF: map -> base_link
  geometry_msgs::msg::TransformStamped tf;
  tf.header         = odom.header;
  tf.child_frame_id = config_.base_frame;
  tf.transform.translation.x = state.position.x();
  tf.transform.translation.y = state.position.y();
  tf.transform.translation.z = state.position.z();
  tf.transform.rotation.x    = q.x();
  tf.transform.rotation.y    = q.y();
  tf.transform.rotation.z    = q.z();
  tf.transform.rotation.w    = q.w();
  pubs_.tf_broadcaster->sendTransform(tf);
}

Eigen::Matrix4d StatePublisher::current_map_correction() const {
  if (map_correction_provider_) {
    return map_correction_provider_();
  }
  return Eigen::Matrix4d::Identity();
}

void StatePublisher::apply_map_correction(
    const Eigen::Vector3f& nominal_position,
    const Eigen::Quaternionf& nominal_orientation,
    Eigen::Vector3f& corrected_position,
    Eigen::Quaternionf& corrected_orientation) const {
  Eigen::Matrix4d T_nominal = Eigen::Matrix4d::Identity();
  T_nominal.block<3, 3>(0, 0) = nominal_orientation.toRotationMatrix().cast<double>();
  T_nominal.block<3, 1>(0, 3) = nominal_position.cast<double>();

  const Eigen::Matrix4d T_map = current_map_correction() * T_nominal;
  corrected_position = T_map.block<3, 1>(0, 3).cast<float>();
  corrected_orientation = Eigen::Quaternionf(T_map.block<3, 3>(0, 0).cast<float>());
  corrected_orientation.normalize();
}

void StatePublisher::rebuild_corrected_path(const rclcpp::Time& ros_time) {
  path_msg_.header.stamp = ros_time;
  path_msg_.header.frame_id = config_.map_frame;
  path_msg_.poses.clear();
  path_msg_.poses.reserve(path_samples_.size());

  for (const auto& sample : path_samples_) {
    Eigen::Vector3f corrected_position;
    Eigen::Quaternionf corrected_orientation;
    apply_map_correction(sample.position, sample.orientation,
                         corrected_position, corrected_orientation);

    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = sample.stamp;
    ps.header.frame_id = config_.map_frame;
    ps.pose.position.x = corrected_position.x();
    ps.pose.position.y = corrected_position.y();
    ps.pose.position.z = corrected_position.z();
    ps.pose.orientation.x = corrected_orientation.x();
    ps.pose.orientation.y = corrected_orientation.y();
    ps.pose.orientation.z = corrected_orientation.z();
    ps.pose.orientation.w = corrected_orientation.w();
    path_msg_.poses.push_back(ps);
  }
}

// ---------------------------------------------------------------------------
// publish_lwo_state — orchestrate all LWO publishing
// ---------------------------------------------------------------------------
void StatePublisher::publish_lwo_state(
    const lwo::LwoState& state, double timestamp,
    const core::PointCloud& raw_scan,
    const builtin_interfaces::msg::Time& lidar_stamp,
    const core::Se3& T_body_lidar,
    const core::PointCloud& processed_scan) {
  const rclcpp::Time ros_time(static_cast<int64_t>(timestamp * 1e9));
  Eigen::Quaternionf q(state.rotation);
  q.normalize();

  publish_odometry(state, ros_time, q);
  publish_path(state, ros_time, q);
  publish_tf_lwo(state, ros_time, q);
  log_drift(state, timestamp);
  publish_clouds(raw_scan, lidar_stamp, state, T_body_lidar, processed_scan);
}

// ---------------------------------------------------------------------------
// publish_odometry
// ---------------------------------------------------------------------------
void StatePublisher::publish_odometry(const lwo::LwoState& state,
                                       const rclcpp::Time& ros_time,
                                       const Eigen::Quaternionf& q) {
  Eigen::Vector3f corrected_position;
  Eigen::Quaternionf corrected_orientation;
  apply_map_correction(state.position, q, corrected_position, corrected_orientation);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp    = ros_time;
  odom.header.frame_id = config_.map_frame;
  odom.child_frame_id  = config_.base_frame;
  odom.pose.pose.position.x    = corrected_position.x();
  odom.pose.pose.position.y    = corrected_position.y();
  odom.pose.pose.position.z    = corrected_position.z();
  odom.pose.pose.orientation.x = corrected_orientation.x();
  odom.pose.pose.orientation.y = corrected_orientation.y();
  odom.pose.pose.orientation.z = corrected_orientation.z();
  odom.pose.pose.orientation.w = corrected_orientation.w();
  odom.twist.twist.linear.x    = state.velocity.x();
  odom.twist.twist.linear.y    = state.velocity.y();
  odom.twist.twist.linear.z    = state.velocity.z();
  pubs_.odom_pub->publish(odom);
}

// ---------------------------------------------------------------------------
// publish_path
// ---------------------------------------------------------------------------
void StatePublisher::publish_path(const lwo::LwoState& state,
                                   const rclcpp::Time& ros_time,
                                   const Eigen::Quaternionf& q) {
  PathSample sample;
  sample.stamp = ros_time;
  sample.position = state.position;
  sample.orientation = q;
  path_samples_.push_back(sample);

  rebuild_corrected_path(ros_time);
  pubs_.path_pub->publish(path_msg_);
}

// ---------------------------------------------------------------------------
// publish_tf_lwo — map->base_link (direct) + map->odom (REP-105)
// ---------------------------------------------------------------------------
void StatePublisher::publish_tf_lwo(const lwo::LwoState& state,
                                     const rclcpp::Time& ros_time,
                                     const Eigen::Quaternionf& q) {
  Eigen::Vector3f corrected_position;
  Eigen::Quaternionf corrected_orientation;
  apply_map_correction(state.position, q, corrected_position, corrected_orientation);

  geometry_msgs::msg::TransformStamped tf_header;
  tf_header.header.stamp    = ros_time;
  tf_header.header.frame_id = config_.map_frame;

  // map -> base_link (LC-corrected SLAM output)
  geometry_msgs::msg::TransformStamped tf_map_base = tf_header;
  tf_map_base.child_frame_id = config_.base_frame;
  tf_map_base.transform.translation.x = corrected_position.x();
  tf_map_base.transform.translation.y = corrected_position.y();
  tf_map_base.transform.translation.z = corrected_position.z();
  tf_map_base.transform.rotation.x    = corrected_orientation.x();
  tf_map_base.transform.rotation.y    = corrected_orientation.y();
  tf_map_base.transform.rotation.z    = corrected_orientation.z();
  tf_map_base.transform.rotation.w    = corrected_orientation.w();
  pubs_.tf_broadcaster->sendTransform(tf_map_base);

  // map -> odom (REP-105: T_map_odom = T_map_base * inv(T_odom_base))
  Eigen::Matrix3f R_odom_base = Eigen::Matrix3f::Identity();
  Eigen::Vector3f t_odom_base = Eigen::Vector3f::Zero();
  {
    std::lock_guard<std::mutex> lk(odom_pose_mutex_);
    if (odom_pose_received_) {
      R_odom_base = odom_rotation_;
      t_odom_base = odom_position_;
    }
  }
  const Eigen::Matrix3f R_odom_base_inv = R_odom_base.transpose();
  const Eigen::Vector3f t_odom_base_inv = -R_odom_base_inv * t_odom_base;
  const Eigen::Matrix3f R_map_odom = corrected_orientation.toRotationMatrix() * R_odom_base_inv;
  const Eigen::Vector3f t_map_odom = corrected_orientation.toRotationMatrix() * t_odom_base_inv + corrected_position;
  const Eigen::Quaternionf q_map_odom(R_map_odom);

  geometry_msgs::msg::TransformStamped tf_map_odom = tf_header;
  tf_map_odom.child_frame_id = config_.odom_frame;
  tf_map_odom.transform.translation.x = t_map_odom.x();
  tf_map_odom.transform.translation.y = t_map_odom.y();
  tf_map_odom.transform.translation.z = t_map_odom.z();
  tf_map_odom.transform.rotation.x    = q_map_odom.x();
  tf_map_odom.transform.rotation.y    = q_map_odom.y();
  tf_map_odom.transform.rotation.z    = q_map_odom.z();
  tf_map_odom.transform.rotation.w    = q_map_odom.w();
  pubs_.tf_broadcaster->sendTransform(tf_map_odom);

  // NOTE: base_link -> lidar_frame is a static extrinsic published via /tf_static.
}

// ---------------------------------------------------------------------------
// log_drift — throttled LWO-vs-odom comparison log
// ---------------------------------------------------------------------------
void StatePublisher::log_drift(const lwo::LwoState& state, double timestamp) {
  Eigen::Vector3f odom_pos = Eigen::Vector3f::Zero();
  bool have_odom = false;
  {
    std::lock_guard<std::mutex> lk(odom_pose_mutex_);
    if (odom_pose_received_) {
      odom_pos  = odom_position_;
      have_odom = true;
    }
  }
  if (!have_odom) return;

  const float drift = (state.position - odom_pos).head<2>().norm();
  RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000,
      "[LWO-vs-ODOM] t=%.1f lwo=[%.3f,%.3f,%.3f] odom=[%.3f,%.3f,%.3f] "
      "drift_2d=%.3f vel=[%.3f,%.3f,%.3f]",
      timestamp,
      state.position.x(), state.position.y(), state.position.z(),
      odom_pos.x(), odom_pos.y(), odom_pos.z(),
      drift,
      state.velocity.x(), state.velocity.y(), state.velocity.z());
}

// ---------------------------------------------------------------------------
// publish_clouds — raw, processed, and world-frame point clouds
// ---------------------------------------------------------------------------
void StatePublisher::publish_clouds(const core::PointCloud& raw_scan,
                                     const builtin_interfaces::msg::Time& lidar_stamp,
                                     const lwo::LwoState& state,
                                     const core::Se3& T_body_lidar,
                                     const core::PointCloud& processed_scan) {
  // 1. Raw input cloud (before preprocessing)
  if (!raw_scan.empty()) {
    pubs_.raw_cloud_pub->publish(
        to_cloud_msg(raw_scan, config_.lidar_frame, lidar_stamp));
  }

  // 2. Processed cloud (stride + voxel + undistort — what IEKF actually sees)
  if (!processed_scan.empty()) {
    pubs_.processed_cloud_pub->publish(
        to_cloud_msg(processed_scan, config_.lidar_frame, lidar_stamp));
    pubs_.wlo_cloud_pub->publish(
        to_cloud_msg(processed_scan, config_.lidar_frame, lidar_stamp));
  } else if (!raw_scan.empty()) {
    pubs_.wlo_cloud_pub->publish(
        to_cloud_msg(raw_scan, config_.lidar_frame, lidar_stamp));
  }

  // 3. World-frame cloud (processed, transformed to corrected map frame)
  if (!processed_scan.empty()) {
    Eigen::Vector3f corrected_position;
    Eigen::Quaternionf corrected_orientation;
    apply_map_correction(state.position, Eigen::Quaternionf(state.rotation),
                         corrected_position, corrected_orientation);

    const Eigen::Matrix3f R_bl = T_body_lidar.rotation_matrix();
    const Eigen::Vector3f t_bl = T_body_lidar.translation();
    const Eigen::Matrix3f R_mb = corrected_orientation.toRotationMatrix();
    const Eigen::Vector3f t_mb = corrected_position;
    const Eigen::Matrix3f R_ml = R_mb * R_bl;
    const Eigen::Vector3f t_ml = R_mb * t_bl + t_mb;
    pubs_.world_cloud_pub->publish(
        to_cloud_msg(processed_scan, config_.map_frame, lidar_stamp, R_ml, t_ml));
  }
}

// ---------------------------------------------------------------------------
// to_cloud_msg — PointCloud2 serialization (no transform)
// ---------------------------------------------------------------------------
sensor_msgs::msg::PointCloud2 StatePublisher::to_cloud_msg(
    const core::PointCloud& cloud,
    const std::string& frame_id,
    const builtin_interfaces::msg::Time& stamp) {
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp    = stamp;
  msg.header.frame_id = frame_id;
  msg.height = 1;
  msg.width  = static_cast<uint32_t>(cloud.size());
  msg.is_dense = false; msg.is_bigendian = false;

  auto add_field = [&](const std::string& name, uint32_t off) {
    sensor_msgs::msg::PointField f;
    f.name = name; f.offset = off;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32; f.count = 1;
    msg.fields.push_back(f);
  };
  add_field("x", 0); add_field("y", 4); add_field("z", 8);
  msg.point_step = 12;
  msg.row_step   = 12 * msg.width;
  msg.data.resize(msg.row_step);

  uint8_t* ptr = msg.data.data();
  for (const auto& pt : cloud.points()) {
    float fx = pt.x, fy = pt.y, fz = pt.z;
    std::memcpy(ptr,     &fx, 4);
    std::memcpy(ptr + 4, &fy, 4);
    std::memcpy(ptr + 8, &fz, 4);
    ptr += 12;
  }
  return msg;
}

// ---------------------------------------------------------------------------
// to_cloud_msg — PointCloud2 serialization (with world transform)
// ---------------------------------------------------------------------------
sensor_msgs::msg::PointCloud2 StatePublisher::to_cloud_msg(
    const core::PointCloud& cloud,
    const std::string& frame_id,
    const builtin_interfaces::msg::Time& stamp,
    const Eigen::Matrix3f& R_world_lidar,
    const Eigen::Vector3f& t_world_lidar) {
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp    = stamp;
  msg.header.frame_id = frame_id;
  msg.height = 1;
  msg.width  = static_cast<uint32_t>(cloud.size());
  msg.is_dense = false; msg.is_bigendian = false;

  auto add_field = [&](const std::string& name, uint32_t off) {
    sensor_msgs::msg::PointField f;
    f.name = name; f.offset = off;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32; f.count = 1;
    msg.fields.push_back(f);
  };
  add_field("x", 0); add_field("y", 4); add_field("z", 8);
  msg.point_step = 12;
  msg.row_step   = 12 * msg.width;
  msg.data.resize(msg.row_step);

  uint8_t* ptr = msg.data.data();
  for (const auto& pt : cloud.points()) {
    const Eigen::Vector3f p_w =
        R_world_lidar * Eigen::Vector3f(pt.x, pt.y, pt.z) + t_world_lidar;
    float fx = p_w.x(), fy = p_w.y(), fz = p_w.z();
    std::memcpy(ptr,     &fx, 4);
    std::memcpy(ptr + 4, &fy, 4);
    std::memcpy(ptr + 8, &fz, 4);
    ptr += 12;
  }
  return msg;
}

}  // namespace tof_slam
