// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// state_publisher.hpp — ROS 2 publisher for LIO and LWO SLAM states.
//
// Encapsulates all odometry/path/TF/cloud publishing logic, keeping
// SlamNode focused on event routing and estimator management.
//
// Odom pose ownership: update_odom_pose() is called from wheel_callback;
// publish_tf_lwo() and log_drift() read the pose internally under mutex.
// CSV callers use get_relative_odom() to retrieve the relative pose.

#ifndef TOF_SLAM_ROS_STATE_PUBLISHER_HPP_
#define TOF_SLAM_ROS_STATE_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "tof_slam/common/types/state.hpp"
#include "tof_slam/common/lie/se3.hpp"
#include "tof_slam/frontend_w/estimator/lwo_state.hpp"
#include "tof_slam/common/types/point_types.hpp"

namespace tof_slam {

class StatePublisher {
 public:
  // ---- Configuration -------------------------------------------------------
  struct Config {
    std::string map_frame   = "map";
    std::string odom_frame  = "odom";
    std::string base_frame  = "base_link";
    std::string lidar_frame = "livox_frame";
  };

  // ---- Non-owning publisher handles ----------------------------------------
  // All publishers are owned by SlamNode (rclcpp::Publisher SharedPtrs).
  // tf_broadcaster is a raw non-owning pointer — SlamNode owns the unique_ptr.
  struct Publishers {
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr wlo_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr world_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_cloud_pub;
    tf2_ros::TransformBroadcaster* tf_broadcaster = nullptr;  // non-owning
  };

  // ---- Constructor ---------------------------------------------------------
  StatePublisher(const Config& config,
                 const Publishers& pubs,
                 rclcpp::Logger logger,
                 rclcpp::Clock::SharedPtr clock,
                 std::function<Eigen::Matrix4d()> map_correction_provider = {});

  // Non-copyable, non-movable (owns mutex)
  StatePublisher(const StatePublisher&) = delete;
  StatePublisher& operator=(const StatePublisher&) = delete;

  // ---- Public API ----------------------------------------------------------

  // LIO mode: publish odometry + path + TF (map->base_link). No cloud.
  void publish_lio_state(const core::LioState& state, double timestamp);

  // LWO mode: orchestrate odometry, path, TF, drift log, clouds.
  // processed_scan: result of lwo_estimator->last_processed_scan() (caller obtains).
  // T_body_lidar: extrinsic used for world-frame cloud transform.
  void publish_lwo_state(const lwo::LwoState& state, double timestamp,
                         const core::PointCloud& raw_scan,
                         const builtin_interfaces::msg::Time& lidar_stamp,
                         const core::Se3& T_body_lidar,
                         const core::PointCloud& processed_scan);

  // Thread-safe odom pose update. Called from wheel_callback.
  // Sets the internal origin on the first call.
  void update_odom_pose(const Eigen::Vector3f& position,
                        const Eigen::Quaternionf& orientation);

  // Retrieve relative odom pose (relative to first received pose).
  // Returns {rel_position, rel_quaternion}.
  // Returns {Zero, Identity} if no odom received yet.
  std::pair<Eigen::Vector3f, Eigen::Quaternionf> get_relative_odom() const;

 private:
  // ---- LWO publish sub-methods ---------------------------------------------
  void publish_odometry(const lwo::LwoState& state,
                        const rclcpp::Time& ros_time,
                        const Eigen::Quaternionf& q);
  void publish_path(const lwo::LwoState& state,
                    const rclcpp::Time& ros_time,
                    const Eigen::Quaternionf& q);
  void publish_tf_lwo(const lwo::LwoState& state,
                      const rclcpp::Time& ros_time,
                      const Eigen::Quaternionf& q);
  [[nodiscard]] Eigen::Matrix4d current_map_correction() const;
  void apply_map_correction(const Eigen::Vector3f& nominal_position,
                            const Eigen::Quaternionf& nominal_orientation,
                            Eigen::Vector3f& corrected_position,
                            Eigen::Quaternionf& corrected_orientation) const;
  void rebuild_corrected_path(const rclcpp::Time& ros_time);
  void log_drift(const lwo::LwoState& state, double timestamp);
  void publish_clouds(const core::PointCloud& raw_scan,
                      const builtin_interfaces::msg::Time& lidar_stamp,
                      const lwo::LwoState& state,
                      const core::Se3& T_body_lidar,
                      const core::PointCloud& processed_scan);

  // ---- PointCloud2 serialization (no member state) -------------------------
  [[nodiscard]] static sensor_msgs::msg::PointCloud2 to_cloud_msg(
      const core::PointCloud& cloud, const std::string& frame_id,
      const builtin_interfaces::msg::Time& stamp);
  [[nodiscard]] static sensor_msgs::msg::PointCloud2 to_cloud_msg(
      const core::PointCloud& cloud, const std::string& frame_id,
      const builtin_interfaces::msg::Time& stamp,
      const Eigen::Matrix3f& R_world_lidar,
      const Eigen::Vector3f& t_world_lidar);

  // ---- Members -------------------------------------------------------------
  Config config_;
  Publishers pubs_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::function<Eigen::Matrix4d()> map_correction_provider_;

  struct PathSample {
    rclcpp::Time stamp;
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
  };

  // Accumulated path for path topic
  nav_msgs::msg::Path path_msg_;
  std::vector<PathSample> path_samples_;

  // Wheel odom pose tracking (odom -> base_link from /odom topic)
  mutable std::mutex odom_pose_mutex_;
  Eigen::Matrix3f odom_rotation_   = Eigen::Matrix3f::Identity();
  Eigen::Vector3f odom_position_   = Eigen::Vector3f::Zero();
  bool odom_pose_received_         = false;

  // Odom origin for relative pose computation (first received = origin)
  Eigen::Vector3f odom_origin_pos_      = Eigen::Vector3f::Zero();
  Eigen::Matrix3f odom_origin_rot_inv_  = Eigen::Matrix3f::Identity();
  bool odom_origin_set_                 = false;
};

}  // namespace tof_slam

#endif  // TOF_SLAM_ROS_STATE_PUBLISHER_HPP_
