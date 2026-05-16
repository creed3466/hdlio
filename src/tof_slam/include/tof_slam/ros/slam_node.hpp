// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// slam_node.hpp — ROS 2 node wrapping LioEstimator or LwoEstimator.
//
// Single-threaded event queue model: IMU/Wheel and LiDAR callbacks push to
// one queue; a single processing thread pops events in arrival order.
//
// Mode selection via config:
//   use_imu=true  → LIO mode (LioEstimator + IMU propagation)
//   use_imu=false, use_wheel_odometry=true → LWO mode (LwoEstimator + Wheel)

#ifndef TOF_SLAM_ROS_SLAM_NODE_HPP_
#define TOF_SLAM_ROS_SLAM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>

#include "tof_slam/frontend/estimator/lio_estimator.hpp"
#include "tof_slam/frontend_w/estimator/lwo_estimator.hpp"
#include "tof_slam/ros/csv_logger.hpp"
#include "tof_slam/ros/param_loader.hpp"
#include "tof_slam/ros/point_cloud_adapter.hpp"
#include "tof_slam/ros/imu_adapter.hpp"
#include "tof_slam/ros/state_publisher.hpp"
#include "tof_slam/ros/backend_runner.hpp"
#include "tof_slam/common/lie/se3.hpp"
#include "tof_slam/mapping/occupancy_grid.hpp"

namespace tof_slam {

class SlamNode : public rclcpp::Node {
 public:
  explicit SlamNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~SlamNode() override;

 private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void wheel_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void processing_loop();

  // ---- Constructor helpers (called once, in order) ------------------------
  void setup_csv_logging();
  void setup_pub_sub();

  // ---- wheel_callback helper ----------------------------------------------
  // Returns false on first call (no prev state); updates prev_odom_* on success.
  [[nodiscard]] bool compute_twist_from_pose_diff(
      double current_ts, const Eigen::Vector3f& cur_pos,
      float cur_yaw, float& vx, float& omega_z);

  // Mode flag
  bool use_imu_ = true;
  bool use_wheel_ = false;

  // Core components — only one is active at a time
  std::unique_ptr<core::LioEstimator> lio_estimator_;
  std::unique_ptr<lwo::LwoEstimator>  lwo_estimator_;

  ros_adapter::PointCloudAdapter pc_adapter_;
  ros_adapter::ImuAdapter imu_adapter_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr wlo_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr world_cloud_pub_;
  // Debug: raw input cloud (before preprocessing) and processed vs raw diff
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mapper_keyframe_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr latest_submap_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr loop_source_submap_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr loop_target_submap_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // ---- Unified event queue (single-threaded processing) --------------------
  struct SensorEvent {
    enum Type { IMU, LIDAR, WHEEL };
    Type type;
    // IMU payload
    sensor_msgs::msg::Imu::SharedPtr imu_msg;
    // WHEEL payload
    float wheel_vx = 0.0f;
    float wheel_omega_z = 0.0f;
    // Delta-pose from consecutive odom messages (in body frame).
    // Using firmware's integrated pose diff avoids float32 re-integration drift.
    Eigen::Vector3f wheel_delta_pos = Eigen::Vector3f::Zero();
    float wheel_delta_yaw = 0.0f;
    bool wheel_use_delta_pose = false;  // true: use delta_pos/delta_yaw instead of vx/omega_z
    // LiDAR payload
    core::PointCloud cloud;
    double timestamp = 0.0;
    // 원본 msg->header.stamp (sec/nanosec 그대로 보존 — double 변환 오차 없음)
    builtin_interfaces::msg::Time lidar_stamp{};
  };
  std::deque<SensorEvent> event_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::thread processing_thread_;
  std::atomic<bool> running_{true};

  // LiDAR dedup timestamp (DDS best-effort can re-deliver messages)
  double last_lidar_queue_ts_{0.0};

  // Frame IDs (REP-105: map -> odom -> base_link -> lidar)
  std::string map_frame_{"map"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  std::string lidar_frame_{"livox_frame"};

  // Topic names (stored for use in setup helpers)
  std::string imu_topic_;
  std::string lidar_topic_;
  std::string odom_topic_;

  // LWO extrinsic (body <- lidar), stored for cloud publish
  core::Se3 T_body_lidar_lwo_;

  // State publisher (owns path, odom pose tracking, cloud serialization)
  std::unique_ptr<StatePublisher> state_publisher_;

  // CSV trajectory and diagnostics logging
  CsvLogger csv_logger_;

  // Resource profiling flag (mirrors LwoEstimator::Config::check_usage)
  bool check_usage_ = false;

  // LiDAR timestamp monotonicity filter
  double last_lidar_header_ts_ = 0.0;

  // Pose-diff velocity computation (for odom topics without twist)
  double prev_odom_ts_ = 0.0;
  Eigen::Vector3f prev_odom_pos_ = Eigen::Vector3f::Zero();
  float prev_odom_yaw_ = 0.0f;
  bool prev_odom_valid_ = false;

  // ---- Loop Closure Backend ------------------------------------------------
  bool enable_loop_closure_ = false;
  std::unique_ptr<BackendRunner> backend_runner_;
  Eigen::Matrix4d T_map_odom_correction_{Eigen::Matrix4d::Identity()};
  mutable std::mutex correction_mutex_;
  size_t next_backend_keyframe_id_{0};      // Assigned only to accepted keyframes
  double lc_keyframe_trans_thresh_ = 0.3;   // m
  double lc_keyframe_rot_thresh_ = 0.15;    // rad
  Eigen::Vector3f last_keyframe_pos_ = Eigen::Vector3f::Zero();
  float last_keyframe_yaw_ = 0.0f;
  bool first_keyframe_ = true;

  struct MappingUsageSnapshot {
    float ogm_input_transform_ms{0.0f};
    float ogm_local_grid_ms{0.0f};
    float ogm_global_assemble_ms{0.0f};
  };

  /// Try to create a keyframe from the current LWO state and submit to backend.
  MappingUsageSnapshot trySubmitKeyframe(const lwo::LwoState& state,
                                         const core::PointCloud& processed_scan,
                                         const core::PointCloud& raw_cloud,
                                         const core::Se3& T_body_lidar);

  /// Publish loop closure link markers and backend debug views to RViz2.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lc_marker_pub_;
  void publishLoopClosureMarkers(const PoseGraphSnapshot& pose_graph);
  void publishBackendDebugVisualization();
  void publishMapperKeyframeMarkers(const std::vector<KeyframePoseSnapshot>& keyframes);
  void publishLatestSubmapCloud(const std::vector<VisualSubmapSnapshot>& submaps);
  void publishLoopClosureSubmapClouds(const std::vector<VisualSubmapSnapshot>& submaps,
                                      const std::vector<GicpMatchResult>& results);

  // ---- Map Save Service -------------------------------------------------------
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;
  void handleSaveMap(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // ---- OGM (Occupancy Grid Map for Nav2) -----------------------------------
  struct OgmKeyframeAsset {
    size_t backend_keyframe_id{0};
    LocalGrid local_grid;
    Eigen::Matrix4d T_odom_body_nominal{Eigen::Matrix4d::Identity()};
  };

  struct OgmAssemblySnapshot {
    std::vector<LocalGrid> local_grids;
    std::vector<Eigen::Matrix4d> poses;
  };

  bool enable_ogm_ = false;
  std::unique_ptr<OccupancyGridGenerator> ogm_generator_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr ogm_pub_;
  TofSlamConfig ogm_slam_config_;
  std::vector<OgmKeyframeAsset> ogm_keyframe_assets_;
  std::vector<Eigen::Matrix4d> ogm_corrected_poses_;
  int ogm_publish_counter_{0};
  int ogm_publish_stride_{5};
  float last_ogm_pgo_rebuild_ms_{0.0f};
  mutable std::mutex ogm_mutex_;
  mutable std::mutex ogm_usage_mutex_;
  float publishOccupancyGrid();
  void rebuildOgmAfterPGO();
  OgmAssemblySnapshot snapshotOgmAssemblyInputs() const;
};

}  // namespace tof_slam

#endif  // TOF_SLAM_ROS_SLAM_NODE_HPP_
