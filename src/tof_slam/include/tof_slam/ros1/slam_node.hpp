// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// slam_node.hpp — ROS 1 (Noetic) node wrapping core::LioEstimator for TofSLAM.
//
// Single-threaded event queue model: both IMU and LiDAR callbacks push to a
// unified queue.  When deterministic_queue is enabled, events are inserted in
// timestamp order with a small buffer delay, eliminating OS thread scheduler
// non-determinism.  Otherwise, events are processed in arrival order (FIFO).

#ifndef TOF_SLAM_ROS1_SLAM_NODE_HPP_
#define TOF_SLAM_ROS1_SLAM_NODE_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <thread>

#include "tof_slam/frontend/estimator/lio_estimator.hpp"
#include "tof_slam/ros1/point_cloud_adapter.hpp"
#include "tof_slam/ros1/imu_adapter.hpp"
#ifdef HAS_LIVOX_ROS_DRIVER2
#include "tof_slam/ros1/livox_custom_msg_adapter.hpp"
#endif

namespace tof_slam {

class SlamNode {
 public:
  SlamNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~SlamNode();

 private:
  // ---- Unified event queue (single-threaded processing) --------------------
  struct SensorEvent {
    enum Type { IMU, LIDAR };
    Type type;
    // IMU payload
    sensor_msgs::Imu::ConstPtr imu_msg;
    // LiDAR payload
    core::PointCloud cloud;
    double timestamp;  // sensor timestamp from message header
  };

  void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
  void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
#ifdef HAS_LIVOX_ROS_DRIVER2
  void livox_callback(const livox_ros_driver2::CustomMsg::ConstPtr& msg);
#endif
  void enqueue_event(SensorEvent ev);   // sorted or FIFO insert
  void processing_loop();
  void publish_state(const core::LioState& state, double timestamp);

  // ROS node handles
  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;

  // Core components
  std::unique_ptr<core::LioEstimator> estimator_;
  ros_adapter::PointCloudAdapter pc_adapter_;
#ifdef HAS_LIVOX_ROS_DRIVER2
  ros_adapter::LivoxCustomMsgAdapter livox_adapter_;
  bool use_livox_custom_msg_{false};
#endif
  ros_adapter::ImuAdapter imu_adapter_;

  // ROS interfaces
  ros::Subscriber imu_sub_;
  ros::Subscriber lidar_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher path_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // Event queue state
  std::deque<SensorEvent> event_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::thread processing_thread_;
  std::atomic<bool> running_{true};

  // LiDAR dedup timestamp (transport can re-deliver messages)
  double last_lidar_queue_ts_{0.0};

  // Deterministic queue: timestamp-sorted insertion + buffer delay
  // Eliminates ROS1 callback scheduling non-determinism by ensuring events
  // are always processed in sensor timestamp order regardless of arrival order.
  bool deterministic_queue_{false};
  double queue_buffer_delay_{0.005};  // seconds (default 5ms)
  double queue_newest_ts_{0.0};       // newest timestamp seen in queue
  // R2' LiDAR-anchor: newest LiDAR ts (post-dedup) seen in queue.  Used by
  // processing_loop readiness predicate: an IMU at t is released iff a LiDAR
  // at t' > t has already been enqueued, making window assignment a pure
  // function of sensor timestamps (independent of wall-clock arrival).
  double queue_last_lidar_ts_{0.0};
  bool deterministic_queue_lidar_anchor_{true};

  // Accumulated path
  nav_msgs::Path path_msg_;

  // Frame IDs
  std::string map_frame_{"map"};
  std::string base_frame_{"base_link"};

  // CSV trajectory logging
  std::ofstream csv_file_;
  std::mutex csv_mutex_;

  // ---- Determinism debug (env TOFSLAM_DEBUG_DETERMINISM=1) ----
  bool debug_determinism_{false};
  std::ofstream debug_imu_file_;
  std::ofstream debug_state_file_;
  int debug_imu_logged_{0};
  int debug_state_logged_{0};
  static constexpr int kDebugImuMax   = 200;
  static constexpr int kDebugStateMax = 6;  // frames 0..5
};

}  // namespace tof_slam

#endif  // TOF_SLAM_ROS1_SLAM_NODE_HPP_
