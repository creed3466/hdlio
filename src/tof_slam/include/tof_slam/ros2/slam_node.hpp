// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// slam_node.hpp — ROS 2 node wrapping core::LioEstimator for TofSLAM.
//
// Timestamp-batched sync model (FAST-LIO2 pattern): both IMU and LiDAR
// callbacks push to a unified queue.  The processing thread waits for a
// LiDAR event, drains and sorts by sensor timestamp, then processes the
// batch.  This guarantees deterministic IMU-LiDAR grouping.

#ifndef TOF_SLAM_ROS2_SLAM_NODE_HPP_
#define TOF_SLAM_ROS2_SLAM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <thread>

#include "tof_slam/frontend/estimator/lio_estimator.hpp"
#include "tof_slam/ros2/point_cloud_adapter.hpp"
#include "tof_slam/ros2/imu_adapter.hpp"

namespace tof_slam {

class SlamNode : public rclcpp::Node {
 public:
  explicit SlamNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~SlamNode() override;

 private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void processing_loop();
  void publish_state(const core::LioState& state, double timestamp);

  // Core components
  std::unique_ptr<core::LioEstimator> estimator_;
  ros_adapter::PointCloudAdapter pc_adapter_;
  ros_adapter::ImuAdapter imu_adapter_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // ---- Unified event queue (single-threaded processing) --------------------
  struct SensorEvent {
    enum Type { IMU, LIDAR };
    Type type;
    // IMU payload
    sensor_msgs::msg::Imu::SharedPtr imu_msg;
    // LiDAR payload
    core::PointCloud cloud;
    double timestamp;  // used for LiDAR events
  };
  std::deque<SensorEvent> event_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::thread processing_thread_;
  std::atomic<bool> running_{true};

  // LiDAR dedup timestamp (DDS best-effort can re-deliver messages)
  double last_lidar_queue_ts_{0.0};

  // Accumulated path
  nav_msgs::msg::Path path_msg_;

  // Frame IDs
  std::string map_frame_{"map"};
  std::string base_frame_{"base_link"};

  // CSV trajectory logging
  std::ofstream csv_file_;
  std::mutex csv_mutex_;
};

}  // namespace tof_slam

#endif  // TOF_SLAM_ROS2_SLAM_NODE_HPP_
