// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// slam_node.cpp — ROS 2 node wrapping LioEstimator (LIO) or LwoEstimator (LWO).
//
// Single-threaded event queue model: IMU/Wheel and LiDAR callbacks push to one
// queue; a single processing thread pops events in arrival order.  This
// guarantees all measurements preceding a LiDAR frame are propagated
// before IEKF runs.

#include "tof_slam/ros/slam_node.hpp"
#include "tof_slam/backend/map_saver.hpp"
#include "tof_slam/mapping/map_aligner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <unordered_map>
#include <cstring>

#include <sensor_msgs/msg/point_field.hpp>
#include <pcl/common/transforms.h>

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace tof_slam {

namespace {

sensor_msgs::msg::PointCloud2 makeColoredCloudMsg(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    uint8_t r, uint8_t g, uint8_t b) {
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(cloud.size());
  msg.is_dense = false;
  msg.is_bigendian = false;

  auto add_field = [&](const std::string& name, uint32_t off, uint8_t datatype) {
    sensor_msgs::msg::PointField f;
    f.name = name;
    f.offset = off;
    f.datatype = datatype;
    f.count = 1;
    msg.fields.push_back(f);
  };

  add_field("x", 0, sensor_msgs::msg::PointField::FLOAT32);
  add_field("y", 4, sensor_msgs::msg::PointField::FLOAT32);
  add_field("z", 8, sensor_msgs::msg::PointField::FLOAT32);
  add_field("rgb", 12, sensor_msgs::msg::PointField::UINT32);
  msg.point_step = 16;
  msg.row_step = msg.point_step * msg.width;
  msg.data.resize(msg.row_step);

  const uint32_t rgb =
      (static_cast<uint32_t>(r) << 16) |
      (static_cast<uint32_t>(g) << 8) |
      static_cast<uint32_t>(b);
  uint8_t* ptr = msg.data.data();
  for (const auto& pt : cloud.points) {
    std::memcpy(ptr + 0, &pt.x, sizeof(float));
    std::memcpy(ptr + 4, &pt.y, sizeof(float));
    std::memcpy(ptr + 8, &pt.z, sizeof(float));
    std::memcpy(ptr + 12, &rgb, sizeof(uint32_t));
    ptr += msg.point_step;
  }

  return msg;
}

}  // namespace

// ---------------------------------------------------------------------------
// Construction / Destruction
// ---------------------------------------------------------------------------

SlamNode::SlamNode(const rclcpp::NodeOptions& options)
    : Node("tofslam_node", options) {
  param_loader::declare_all_params(*this);

  use_imu_   = get_parameter("use_imu").as_bool();
  use_wheel_ = get_parameter("use_wheel_odometry").as_bool();

  map_frame_   = get_parameter("map_frame").as_string();
  odom_frame_  = get_parameter("odom_frame").as_string();
  base_frame_  = get_parameter("base_frame").as_string();
  lidar_frame_ = get_parameter("lidar_frame").as_string();
  imu_topic_   = get_parameter("imu_topic").as_string();
  lidar_topic_ = get_parameter("lidar_topic").as_string();
  odom_topic_  = get_parameter("odom_topic").as_string();

  const double ext_x = get_parameter("extrinsic_x").as_double();
  const double ext_y = get_parameter("extrinsic_y").as_double();
  const double ext_z = get_parameter("extrinsic_z").as_double();
  const Eigen::Matrix3f ext_rot = param_loader::build_extrinsic_rot(*this);

  if (use_imu_) {
    auto result = param_loader::build_lio_config(*this, ext_rot, ext_x, ext_y, ext_z);
    lio_estimator_ = std::make_unique<core::LioEstimator>(result.estimator_config);
    imu_adapter_ = ros_adapter::ImuAdapter(result.imu_adapter_config);
    RCLCPP_INFO(get_logger(), "LIO mode: imu=%s lidar=%s init_samples=%d undistort=%s",
                imu_topic_.c_str(), lidar_topic_.c_str(),
                result.imu_adapter_config.init_sample_count,
                result.estimator_config.enable_undistortion ? "ON" : "OFF");
  } else if (use_wheel_) {
    auto cfg = param_loader::build_lwo_config(*this, ext_rot, ext_x, ext_y, ext_z);
    T_body_lidar_lwo_ = cfg.T_body_lidar;
    lwo_estimator_ = std::make_unique<lwo::LwoEstimator>(cfg);
    RCLCPP_INFO(get_logger(), "LWO mode: odom=%s lidar=%s undistort=%s",
                odom_topic_.c_str(), lidar_topic_.c_str(),
                cfg.enable_undistortion ? "ON" : "OFF");
  } else {
    RCLCPP_ERROR(get_logger(),
                 "No valid mode: use_imu=false and use_wheel_odometry=false. "
                 "Defaulting to LIO mode with IMU.");
  }

  setup_csv_logging();
  setup_pub_sub();

  // ---- Loop Closure Backend ------------------------------------------------
  enable_loop_closure_ = get_parameter("enable_loop_closure").as_bool();
  if (enable_loop_closure_ && use_wheel_) {
    auto lc_config = param_loader::build_loop_closure_config(*this);

    // Build a TofSlamConfig with loop closure specific keyframe thresholds
    TofSlamConfig slam_config;
    lc_keyframe_trans_thresh_ = get_parameter("lc_keyframe_trans_thresh").as_double();
    lc_keyframe_rot_thresh_ = get_parameter("lc_keyframe_rot_thresh").as_double();
    slam_config.keyframe_trans_thresh = lc_keyframe_trans_thresh_;
    slam_config.keyframe_rot_thresh = lc_keyframe_rot_thresh_;
    slam_config.huber_delta = get_parameter("lc_pgo_huber_delta").as_double();
    slam_config.pgo_numeric_diff_eps = get_parameter("lc_pgo_numeric_diff_eps").as_double();
    slam_config.pgo_use_loop_huber = get_parameter("lc_pgo_use_loop_huber").as_bool();

    backend_runner_ = std::make_unique<BackendRunner>(slam_config, lc_config);

    // Open GICP debug CSV
    const auto dump_path = get_parameter("dump_path").as_string();
    backend_runner_->manager().openDebugCsv(dump_path + "/gicp_debug.csv");
    backend_runner_->openPgoDebugCsvs(dump_path);

    // Loop-closure debug visualization callback
    backend_runner_->setStatusCallback([this]() {
      publishBackendDebugVisualization();
    });

    // Set correction callback
    backend_runner_->setCorrectionCallback(
        [this](const Eigen::Matrix4d& correction) {
          {
            std::lock_guard<std::mutex> lk(correction_mutex_);
            T_map_odom_correction_ = correction;
          }
          RCLCPP_INFO(get_logger(), "[LC] T_map_odom updated");

          // Rebuild OGM with PGO-corrected poses
          if (enable_ogm_ && ogm_generator_) {
            rebuildOgmAfterPGO();
          }

        });

    // LC visualization markers (transient_local so RViz sees them even if started late)
    lc_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/tofslam/loop_closure_markers",
        rclcpp::QoS(1).transient_local().reliable());
    mapper_keyframe_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "/tofslam/mapper_keyframes",
        rclcpp::QoS(1).transient_local().reliable());
    latest_submap_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/tofslam/latest_submap",
        rclcpp::QoS(1).transient_local().reliable());
    loop_source_submap_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/tofslam/loop_source_submap",
        rclcpp::QoS(1).transient_local().reliable());
    loop_target_submap_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/tofslam/loop_target_submap",
        rclcpp::QoS(1).transient_local().reliable());

    backend_runner_->start();

    RCLCPP_INFO(get_logger(),
                "Loop closure backend enabled: submap_kf=%d search_r=%.1f gicp_fitness=%.2f",
                lc_config.submap_keyframe_count,
                lc_config.loop_search_radius,
                lc_config.gicp_fitness_threshold);
  }

  // ---- OGM (Occupancy Grid Map) --------------------------------------------
  enable_ogm_ = enable_loop_closure_;  // OGM requires keyframe infrastructure
  if (enable_ogm_ && use_wheel_) {
    // Build TofSlamConfig with OGM parameters
    ogm_slam_config_.ogm_resolution = get_parameter("ogm_resolution").as_double();
    ogm_slam_config_.ogm_local_range = get_parameter("ogm_local_range").as_double();
    ogm_slam_config_.ogm_log_odds_hit = get_parameter("ogm_log_odds_hit").as_double();
    ogm_slam_config_.ogm_log_odds_miss = get_parameter("ogm_log_odds_miss").as_double();
    ogm_slam_config_.ogm_log_odds_max = get_parameter("ogm_log_odds_max").as_double();
    ogm_slam_config_.ogm_log_odds_min = get_parameter("ogm_log_odds_min").as_double();
    ogm_slam_config_.ogm_height_min = get_parameter("ogm_height_min").as_double();
    ogm_slam_config_.ogm_height_max = get_parameter("ogm_height_max").as_double();
    ogm_slam_config_.ogm_endpoint_dilation_radius = get_parameter("ogm_endpoint_dilation_radius").as_int();
    ogm_slam_config_.ogm_global_dilation_radius = get_parameter("ogm_global_dilation_radius").as_int();
    ogm_publish_stride_ = get_parameter("ogm_publish_stride").as_int();

    ogm_generator_ = std::make_unique<OccupancyGridGenerator>(ogm_slam_config_);
    std::string ogm_topic = get_parameter("ogm_topic").as_string();
    ogm_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(ogm_topic,
        rclcpp::QoS(1).transient_local().reliable());

    RCLCPP_INFO(get_logger(), "OGM enabled: res=%.2f range=%.1f stride=%d topic=%s",
                ogm_slam_config_.ogm_resolution, ogm_slam_config_.ogm_local_range,
                ogm_publish_stride_, ogm_topic.c_str());
  }

  // ---- Map Save Service -------------------------------------------------------
  save_map_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/save_map",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        handleSaveMap(req, res);
      });
  RCLCPP_INFO(get_logger(), "Map save service: %s/save_map",
              get_fully_qualified_name());

  RCLCPP_INFO(get_logger(), "TofSLAM node ready (unified queue).");
}

SlamNode::~SlamNode() {
  running_ = false;
  queue_cv_.notify_all();
  if (processing_thread_.joinable()) processing_thread_.join();

  // Stop backend thread
  if (backend_runner_) {
    // Log final loop closure stats
    auto stats = backend_runner_->getStats();
    RCLCPP_INFO(get_logger(),
                "[LC] Final stats: submaps=%d tested=%d converged=%d "
                "accepted=%d rejected=%d optimizations=%d",
                stats.total_submaps, stats.total_candidates_tested,
                stats.total_gicp_converged, stats.total_loops_accepted,
                stats.total_loops_rejected, stats.total_optimizations);
    backend_runner_->stop();
  }
  // csv_logger_ destructor handles file closing automatically
}

// ---------------------------------------------------------------------------
// setup_csv_logging — open trajectory + diagnostics CSV files
// ---------------------------------------------------------------------------
void SlamNode::setup_csv_logging() {
  const auto csv_path  = get_parameter("trajectory_csv_path").as_string();
  const auto dump_path = get_parameter("dump_path").as_string();

  check_usage_ = get_parameter("check_usage").as_bool();

  csv_logger_.open(csv_path, dump_path, use_wheel_);

  RCLCPP_INFO(get_logger(), "Trajectory CSV: %s", csv_path.c_str());
  if (use_wheel_) {
    RCLCPP_INFO(get_logger(), "Diagnostics CSV enabled (dump_path=%s)", dump_path.c_str());
  }

  if (use_wheel_ && check_usage_) {
    // Derive stem from trajectory_csv_path
    std::string stem = csv_path;
    const auto slash = stem.rfind('/');
    if (slash != std::string::npos) stem = stem.substr(slash + 1);
    const auto dot = stem.rfind('.');
    if (dot != std::string::npos) stem = stem.substr(0, dot);
    csv_logger_.open_usage(dump_path, stem);
    RCLCPP_INFO(get_logger(), "Usage CSV enabled (dump_path=%s stem=%s)",
                dump_path.c_str(), stem.c_str());
  }
}

// ---------------------------------------------------------------------------
// setup_pub_sub — create publishers, subscribers, TF broadcaster, processing thread
// ---------------------------------------------------------------------------
void SlamNode::setup_pub_sub() {
  odom_pub_  = create_publisher<nav_msgs::msg::Odometry>("/tofslam/odometry", 10);
  path_pub_  = create_publisher<nav_msgs::msg::Path>("/tofslam/path", 10);
  wlo_cloud_pub_       = create_publisher<sensor_msgs::msg::PointCloud2>("/tofslam/wlo/points", 10);
  world_cloud_pub_     = create_publisher<sensor_msgs::msg::PointCloud2>("/tofslam/wlo/world_points", 10);
  raw_cloud_pub_       = create_publisher<sensor_msgs::msg::PointCloud2>("/tofslam/wlo/raw_points", 10);
  processed_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/tofslam/wlo/processed_points", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  if (use_imu_) {
    // LIO: IMU best-effort with large queue; LiDAR reliable
    auto imu_qos = rclcpp::QoS(
        rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data))
        .best_effort().keep_last(2000);
    auto lidar_qos = rclcpp::QoS(
        rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
        .reliable().keep_last(2000);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, imu_qos,
        [this](sensor_msgs::msg::Imu::SharedPtr msg) { imu_callback(msg); });
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic_, lidar_qos,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { lidar_callback(msg); });
  } else if (use_wheel_) {
    // LWO: best_effort QoS (matches bag publisher) with large queue to prevent drops
    auto lidar_qos = rclcpp::QoS(
        rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
        .best_effort().keep_last(2000);
    auto odom_qos = rclcpp::QoS(
        rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
        .best_effort().keep_last(2000);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, odom_qos,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { wheel_callback(msg); });
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic_, lidar_qos,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { lidar_callback(msg); });
  }

  // StatePublisher: constructed after publishers and TF broadcaster are ready.
  StatePublisher::Config sp_config{map_frame_, odom_frame_, base_frame_, lidar_frame_};
  StatePublisher::Publishers sp_pubs{
      odom_pub_, path_pub_, wlo_cloud_pub_, world_cloud_pub_,
      raw_cloud_pub_, processed_cloud_pub_, tf_broadcaster_.get()};
  state_publisher_ = std::make_unique<StatePublisher>(
      sp_config, sp_pubs, get_logger(), get_clock(),
      [this]() {
        std::lock_guard<std::mutex> lk(correction_mutex_);
        return T_map_odom_correction_;
      });

  processing_thread_ = std::thread(&SlamNode::processing_loop, this);
}

// ---------------------------------------------------------------------------
// IMU callback — pushes to unified event queue (lightweight)
// ---------------------------------------------------------------------------
void SlamNode::imu_callback(
    const sensor_msgs::msg::Imu::SharedPtr msg) {
  SensorEvent ev;
  ev.type = SensorEvent::IMU;
  ev.imu_msg = msg;
  ev.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  {
    std::lock_guard<std::mutex> lk(queue_mutex_);
    event_queue_.push_back(std::move(ev));
  }
  queue_cv_.notify_one();
}

// ---------------------------------------------------------------------------
// compute_twist_from_pose_diff — derive vx/omega_z from consecutive odom poses
// ---------------------------------------------------------------------------
bool SlamNode::compute_twist_from_pose_diff(
    double current_ts, const Eigen::Vector3f& cur_pos,
    float cur_yaw, float& vx, float& omega_z) {
  if (!prev_odom_valid_) {
    prev_odom_ts_    = current_ts;
    prev_odom_pos_   = cur_pos;
    prev_odom_yaw_   = cur_yaw;
    prev_odom_valid_ = true;
    return false;  // no previous state yet
  }

  const double dt = current_ts - prev_odom_ts_;

  // Skip messages arriving too fast (< 10ms) to avoid velocity spikes.
  if (dt < 0.01) return false;

  if (dt <= 1.0) {
    const Eigen::Vector3f delta_world = cur_pos - prev_odom_pos_;

    // Rotate world delta into body frame using previous heading
    const float cos_yaw = std::cos(prev_odom_yaw_);
    const float sin_yaw = std::sin(prev_odom_yaw_);
    vx = (cos_yaw * delta_world.x() + sin_yaw * delta_world.y())
         / static_cast<float>(dt);

    // Delta yaw normalised to [-pi, pi]
    float delta_yaw = cur_yaw - prev_odom_yaw_;
    if (delta_yaw >  static_cast<float>(M_PI)) delta_yaw -= 2.0f * static_cast<float>(M_PI);
    if (delta_yaw < -static_cast<float>(M_PI)) delta_yaw += 2.0f * static_cast<float>(M_PI);
    omega_z = delta_yaw / static_cast<float>(dt);

    // Clamp to physically plausible range for ground robot
    constexpr float kMaxVx    = 3.0f;   // m/s
    constexpr float kMaxOmega = 3.0f;   // rad/s
    vx      = std::clamp(vx,      -kMaxVx,    kMaxVx);
    omega_z = std::clamp(omega_z, -kMaxOmega, kMaxOmega);
  }

  prev_odom_ts_  = current_ts;
  prev_odom_pos_ = cur_pos;
  prev_odom_yaw_ = cur_yaw;
  return true;
}

// ---------------------------------------------------------------------------
// Wheel callback — pushes to unified event queue (lightweight)
// ---------------------------------------------------------------------------
void SlamNode::wheel_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const double current_ts =
      msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

  // Extract current pose
  const auto& p = msg->pose.pose.position;
  const auto& o = msg->pose.pose.orientation;
  const Eigen::Vector3f current_pos(
      static_cast<float>(p.x), static_cast<float>(p.y),
      static_cast<float>(p.z));
  const float qw = static_cast<float>(o.w), qx = static_cast<float>(o.x);
  const float qy = static_cast<float>(o.y), qz = static_cast<float>(o.z);
  const float current_yaw = std::atan2(2.0f * (qw * qz + qx * qy),
                                       1.0f - 2.0f * (qy * qy + qz * qz));

  // Store odom->base_link pose for REP-105 TF and CSV (via StatePublisher)
  {
    Eigen::Quaternionf q_odom(qw, qx, qy, qz);
    q_odom.normalize();
    state_publisher_->update_odom_pose(current_pos, q_odom);
  }

  // Snapshot previous odom state before any update, for delta pose computation.
  // Both the twist-fallback path (compute_twist_from_pose_diff) and the direct
  // delta path below must see the same prev_ values.
  const bool had_prev_odom    = prev_odom_valid_;
  const double snap_prev_ts   = prev_odom_ts_;
  const Eigen::Vector3f snap_prev_pos = prev_odom_pos_;
  const float snap_prev_yaw   = prev_odom_yaw_;

  // Get velocity: prefer twist field; fall back to pose-diff
  float vx      = static_cast<float>(msg->twist.twist.linear.x);
  float omega_z = static_cast<float>(msg->twist.twist.angular.z);
  const bool twist_is_zero = (std::abs(vx) < 1e-6f) && (std::abs(omega_z) < 1e-6f);

  if (twist_is_zero) {
    RCLCPP_INFO_ONCE(get_logger(),
                     "wheel_callback: using pose-diff velocity (twist is zero)");
    // Note: compute_twist_from_pose_diff also updates prev_odom_* internally.
    // We use snapshots above to decouple delta pose computation from this path.
    if (!compute_twist_from_pose_diff(current_ts, current_pos, current_yaw,
                                      vx, omega_z)) {
      return;  // first call — no event yet
    }
  } else {
    RCLCPP_INFO_ONCE(get_logger(),
                     "wheel_callback FIRST CALL: vx=%.3f omega=%.3f ts=%.3f",
                     vx, omega_z, current_ts);
    // Update prev_odom_* ourselves (compute_twist_from_pose_diff not called).
    prev_odom_ts_    = current_ts;
    prev_odom_pos_   = current_pos;
    prev_odom_yaw_   = current_yaw;
    prev_odom_valid_ = true;
  }

  SensorEvent ev;
  ev.type          = SensorEvent::WHEEL;
  ev.wheel_vx      = vx;
  ev.wheel_omega_z = omega_z;
  ev.timestamp     = current_ts;

  // Compute delta pose from consecutive odom poses using the snapshotted prev
  // state.  This uses the firmware's double64 integrated pose diff directly,
  // avoiding our own float32 re-integration and its ~1cm drift.
  if (had_prev_odom) {
    const double dt = current_ts - snap_prev_ts;
    if (dt > 0.01 && dt < 1.0) {
      const Eigen::Vector3f delta_world = current_pos - snap_prev_pos;

      // Rotate world delta into previous body frame
      const float cos_yaw = std::cos(snap_prev_yaw);
      const float sin_yaw = std::sin(snap_prev_yaw);
      ev.wheel_delta_pos.x() =  cos_yaw * delta_world.x() + sin_yaw * delta_world.y();
      ev.wheel_delta_pos.y() = -sin_yaw * delta_world.x() + cos_yaw * delta_world.y();
      ev.wheel_delta_pos.z() = delta_world.z();

      float delta_yaw = current_yaw - snap_prev_yaw;
      // Normalize to [-pi, pi]
      if (delta_yaw >  static_cast<float>(M_PI)) delta_yaw -= 2.0f * static_cast<float>(M_PI);
      if (delta_yaw < -static_cast<float>(M_PI)) delta_yaw += 2.0f * static_cast<float>(M_PI);
      ev.wheel_delta_yaw      = delta_yaw;
      ev.wheel_use_delta_pose = true;
    }
  }

  {
    std::lock_guard<std::mutex> lk(queue_mutex_);
    event_queue_.push_back(std::move(ev));
  }
  queue_cv_.notify_one();
}

// ---------------------------------------------------------------------------
// LiDAR callback — pushes to unified event queue (lightweight)
// ---------------------------------------------------------------------------
void SlamNode::lidar_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  static size_t lidar_cb_total_ = 0;
  static size_t lidar_cb_oos_drop_ = 0;
  static size_t lidar_cb_dedup_drop_ = 0;
  static size_t lidar_cb_queued_ = 0;
  ++lidar_cb_total_;

  const double ts =
      msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

  // Reject out-of-sequence LiDAR messages (duplicate topic in bag DB).
  if (last_lidar_header_ts_ > 0.0) {
    const double dt = ts - last_lidar_header_ts_;
    if (dt < -0.01 || dt > 2.0) {
      ++lidar_cb_oos_drop_;
      RCLCPP_WARN(get_logger(),
        "[LIDAR-CB] OOS drop #%zu: ts=%.3f last=%.3f dt=%.3f",
        lidar_cb_oos_drop_, ts, last_lidar_header_ts_, dt);
      return;
    }
  }
  last_lidar_header_ts_ = ts;

  // Dedup: guard at 50ms (LiDAR is 10Hz → min dt ~100ms)
  {
    std::lock_guard<std::mutex> lk(queue_mutex_);
    if (ts - last_lidar_queue_ts_ < 0.05 && last_lidar_queue_ts_ > 0.0) {
      ++lidar_cb_dedup_drop_;
      return;
    }
    last_lidar_queue_ts_ = ts;
  }

  SensorEvent ev;
  ev.type        = SensorEvent::LIDAR;
  ev.cloud       = pc_adapter_.convert(msg);
  ev.timestamp   = ts;
  ev.lidar_stamp = msg->header.stamp;  // original sec/nanosec preserved
  {
    std::lock_guard<std::mutex> lk(queue_mutex_);
    event_queue_.push_back(std::move(ev));
    ++lidar_cb_queued_;
  }
  queue_cv_.notify_one();

  // Log every 100 callbacks
  if (lidar_cb_total_ % 100 == 0) {
    RCLCPP_INFO(get_logger(),
      "[LIDAR-CB] total=%zu queued=%zu oos_drop=%zu dedup_drop=%zu ts=%.3f",
      lidar_cb_total_, lidar_cb_queued_, lidar_cb_oos_drop_,
      lidar_cb_dedup_drop_, ts);
  }
}

// ---------------------------------------------------------------------------
// Processing thread — FIFO event processing
// ---------------------------------------------------------------------------
void SlamNode::processing_loop() {
  RCLCPP_INFO(get_logger(), "Processing thread started (FIFO queue).");
  size_t n_imu = 0, n_lidar = 0, n_wheel = 0;

  while (true) {  // Drain remaining queue events after running_ becomes false
    SensorEvent ev;
    size_t queue_depth_snapshot = 0;
    {
      std::unique_lock<std::mutex> lk(queue_mutex_);
      queue_cv_.wait(lk,
                     [this] { return !event_queue_.empty() || !running_; });
      if (!running_ && event_queue_.empty()) break;
      if (event_queue_.empty()) continue;
      ev = std::move(event_queue_.front());
      event_queue_.pop_front();
      queue_depth_snapshot = event_queue_.size();
    }

    try {
      if (ev.type == SensorEvent::IMU) {
        // ---- IMU event (LIO mode only) ----
        auto result = imu_adapter_.process(ev.imu_msg);

        if (!lio_estimator_->initialized() && imu_adapter_.initialized()) {
          const auto& ir = imu_adapter_.init_result();
          if (ir.success) {
            if (lio_estimator_->initialize(ir)) {
              RCLCPP_INFO(get_logger(), "Gravity init OK: %s", ir.message.c_str());
              const auto& s = ir.initial_state;
              RCLCPP_INFO(get_logger(),
                "INIT_STATE: pos=[%.4f,%.4f,%.4f] vel=[%.4f,%.4f,%.4f] "
                "gravity=[%.4f,%.4f,%.4f] scale=%.6f",
                s.position.x(), s.position.y(), s.position.z(),
                s.velocity.x(), s.velocity.y(), s.velocity.z(),
                s.gravity.x(), s.gravity.y(), s.gravity.z(),
                ir.imu_acc_scale);
            }
          }
        }
        if (result.has_value() && lio_estimator_->initialized()) {
          lio_estimator_->feed_imu(*result);
          ++n_imu;
        }
      } else if (ev.type == SensorEvent::WHEEL) {
        // ---- Wheel event (LWO mode only) ----
        if (use_wheel_ && lwo_estimator_) {
          if (ev.wheel_use_delta_pose) {
            // Use firmware's integrated delta pose directly to avoid float32
            // re-integration drift (~1cm error vs double64 firmware integration).
            lwo_estimator_->feed_wheel_delta(
                ev.wheel_vx, ev.wheel_omega_z, ev.timestamp,
                ev.wheel_delta_pos, ev.wheel_delta_yaw);
          } else {
            lwo_estimator_->feed_wheel(ev.wheel_vx, ev.wheel_omega_z, ev.timestamp);
          }
          ++n_wheel;
        }
      } else {
        // ---- LiDAR event ----
        if (use_imu_ && lio_estimator_) {
          if (!lio_estimator_->initialized()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Waiting for gravity initialization...");
          } else if (lio_estimator_->feed_lidar(ev.cloud, ev.timestamp)) {
            ++n_lidar;
            const auto& state = lio_estimator_->current_state();
            state_publisher_->publish_lio_state(state, ev.timestamp);
            Eigen::Quaternionf q(state.rotation);
            q.normalize();
            csv_logger_.write_trajectory_lio(state.position, q, ev.timestamp);
          }
        } else if (use_wheel_ && lwo_estimator_) {
          if (!lwo_estimator_->initialized()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Waiting for first wheel odometry message...");
          } else {
            lwo_estimator_->set_queue_depth(
                static_cast<int>(queue_depth_snapshot));
            if (lwo_estimator_->feed_lidar(ev.cloud, ev.timestamp)) {
              ++n_lidar;
              const auto& state = lwo_estimator_->current_state();
              const core::PointCloud processed = lwo_estimator_->last_processed_scan();
              const core::Se3 T_body_lidar =
                  lwo_estimator_->current_body_lidar_extrinsic();
              state_publisher_->publish_lwo_state(
                  state, ev.timestamp, ev.cloud, ev.lidar_stamp,
                  T_body_lidar, processed);
              // CSV: apply LC correction (T_map_odom) to get map-frame pose
              Eigen::Vector3f csv_pos = state.position;
              Eigen::Quaternionf csv_q(state.rotation);
              csv_q.normalize();
              if (enable_loop_closure_) {
                std::lock_guard<std::mutex> lk(correction_mutex_);
                Eigen::Matrix4d T_odom = Eigen::Matrix4d::Identity();
                T_odom.block<3,3>(0,0) = state.rotation.cast<double>();
                T_odom.block<3,1>(0,3) = state.position.cast<double>();
                Eigen::Matrix4d T_map = T_map_odom_correction_ * T_odom;
                csv_pos = T_map.block<3,1>(0,3).cast<float>();
                Eigen::Quaterniond q_map(T_map.block<3,3>(0,0));
                q_map.normalize();
                csv_q = q_map.cast<float>();
              }
              auto [rel_pos, rel_q] = state_publisher_->get_relative_odom();
              csv_logger_.write_trajectory(csv_pos, csv_q, ev.timestamp,
                                           rel_pos, rel_q);
              csv_logger_.write_diagnostics(lwo_estimator_->last_diagnostics());

              MappingUsageSnapshot mapping_usage;
              if (enable_loop_closure_ && backend_runner_) {
                mapping_usage = trySubmitKeyframe(
                    state, processed, ev.cloud, T_body_lidar);
              }

              if (check_usage_) {
                auto usage = lwo_estimator_->last_usage();
                usage.ogm_input_transform_ms = mapping_usage.ogm_input_transform_ms;
                usage.ogm_local_grid_ms = mapping_usage.ogm_local_grid_ms;
                usage.ogm_global_assemble_ms = mapping_usage.ogm_global_assemble_ms;
                {
                  std::lock_guard<std::mutex> lk(ogm_usage_mutex_);
                  usage.ogm_pgo_rebuild_ms = last_ogm_pgo_rebuild_ms_;
                }
                if (backend_runner_) {
                  const auto backend_usage = backend_runner_->latestUsage();
                  usage.backend_add_keyframe_ms = backend_usage.add_keyframe_ms;
                  usage.backend_loop_detect_ms = backend_usage.loop_detect_ms;
                  usage.backend_gicp_ms = backend_usage.gicp_ms;
                  usage.backend_pgo_ms = backend_usage.pgo_ms;
                  usage.backend_total_ms = backend_usage.total_ms;
                  usage.backend_queue_depth = backend_usage.queue_depth;
                  usage.backend_candidates_tested = backend_usage.candidates_tested;
                  usage.backend_loops_accepted = backend_usage.loops_accepted;
                }
                csv_logger_.write_usage(usage);
              }
            }
          }
        }
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Exception in processing loop: %s", e.what());
    }
  }

  {
    std::lock_guard<std::mutex> lk(queue_mutex_);
    RCLCPP_INFO(get_logger(),
                "Processing thread stopped. lidar=%zu imu=%zu wheel=%zu "
                "remaining_queue=%zu",
                n_lidar, n_imu, n_wheel, event_queue_.size());
  }
}

// ---------------------------------------------------------------------------
// trySubmitKeyframe — keyframe selection + backend submission
// ---------------------------------------------------------------------------
SlamNode::MappingUsageSnapshot SlamNode::trySubmitKeyframe(const lwo::LwoState& state,
                                 const core::PointCloud& processed_scan,
                                 const core::PointCloud& raw_cloud,
                                 const core::Se3& T_body_lidar) {
  using Clock = std::chrono::steady_clock;
  MappingUsageSnapshot usage;

  const Eigen::Vector3f pos = state.position;
  // Extract yaw from rotation matrix
  const float yaw = std::atan2(state.rotation(1, 0), state.rotation(0, 0));

  if (first_keyframe_) {
    first_keyframe_ = false;
    last_keyframe_pos_ = pos;
    last_keyframe_yaw_ = yaw;
  }

  // Check keyframe criteria: translation or rotation threshold
  const float trans_delta = (pos - last_keyframe_pos_).head<2>().norm();
  float yaw_delta = std::abs(yaw - last_keyframe_yaw_);
  if (yaw_delta > static_cast<float>(M_PI)) {
    yaw_delta = 2.0f * static_cast<float>(M_PI) - yaw_delta;
  }

  if (trans_delta < static_cast<float>(lc_keyframe_trans_thresh_) &&
      yaw_delta < static_cast<float>(lc_keyframe_rot_thresh_)) {
    return usage;  // Not enough motion for a new keyframe
  }

  last_keyframe_pos_ = pos;
  last_keyframe_yaw_ = yaw;
  const size_t backend_keyframe_id = next_backend_keyframe_id_++;

  // Build PoseState
  PoseState pose;
  Eigen::Matrix3d R = state.rotation.cast<double>();
  pose.q_wb = Eigen::Quaterniond(R).normalized();
  pose.p_wb = pos.cast<double>();
  pose.P = Eigen::Matrix<double, 6, 6>::Identity() * 1e-4;

  // The frontend exposes processed_scan in lidar frame. Convert it to body
  // frame before giving it to the backend, which assumes body-frame clouds.
  auto pcl_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  const Eigen::Matrix3f R_body_lidar = T_body_lidar.rotation_matrix();
  const Eigen::Vector3f t_body_lidar = T_body_lidar.translation();
  pcl_cloud->reserve(processed_scan.size());
  for (const auto& pt : processed_scan) {
    const Eigen::Vector3f p_lidar(pt.x, pt.y, pt.z);
    const Eigen::Vector3f p_body = R_body_lidar * p_lidar + t_body_lidar;
    pcl_cloud->emplace_back(p_body.x(), p_body.y(), p_body.z());
  }

  // === OGM: Generate local grid for this keyframe (using raw cloud for density) ===
  if (ogm_generator_ && enable_ogm_) {
    const auto t_transform_start = Clock::now();
    // Transform raw cloud from lidar frame to body frame
    auto ogm_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    const Eigen::Matrix3f R_body_lidar = T_body_lidar.rotation_matrix();
    const Eigen::Vector3f t_body_lidar = T_body_lidar.translation();
    ogm_cloud->reserve(raw_cloud.size());
    for (const auto& pt : raw_cloud) {
      Eigen::Vector3f p_lidar(pt.x, pt.y, pt.z);
      Eigen::Vector3f p_body = R_body_lidar * p_lidar + t_body_lidar;
      ogm_cloud->emplace_back(p_body.x(), p_body.y(), p_body.z());
    }
    usage.ogm_input_transform_ms = std::chrono::duration<float, std::milli>(
        Clock::now() - t_transform_start).count();

    const auto t_local_grid_start = Clock::now();
    LocalGrid lg = ogm_generator_->generateLocalGrid(ogm_cloud,
        backend_keyframe_id);
    usage.ogm_local_grid_ms = std::chrono::duration<float, std::milli>(
        Clock::now() - t_local_grid_start).count();

    // World pose for this keyframe
    Eigen::Matrix4d T_world = Eigen::Matrix4d::Identity();
    T_world.block<3,3>(0,0) = state.rotation.cast<double>();
    T_world.block<3,1>(0,3) = state.position.cast<double>();

    OgmKeyframeAsset asset;
    asset.backend_keyframe_id = backend_keyframe_id;
    asset.local_grid = std::move(lg);
    asset.T_odom_body_nominal = T_world;

    {
      std::lock_guard<std::mutex> lk(ogm_mutex_);
      ogm_keyframe_assets_.push_back(std::move(asset));
    }

    ++ogm_publish_counter_;
    if (ogm_publish_counter_ % ogm_publish_stride_ == 0) {
      usage.ogm_global_assemble_ms = publishOccupancyGrid();
    }
  }

  backend_runner_->submitKeyframe(backend_keyframe_id, pose, pcl_cloud);
  return usage;
}

// ---------------------------------------------------------------------------
// handleSaveMap — service callback for ~/save_map
// ---------------------------------------------------------------------------
void SlamNode::handleSaveMap(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  RCLCPP_INFO(get_logger(), "[MapSave] Save map request received.");

  if (!enable_loop_closure_ || !backend_runner_) {
    response->success = false;
    response->message = "Loop closure backend not enabled. Cannot save map.";
    RCLCPP_WARN(get_logger(), "[MapSave] %s", response->message.c_str());
    return;
  }

  // Build MapSaveConfig from current parameters
  MapSaveConfig save_config;
  save_config.save_voxel_size = 0.05;
  save_config.save_submaps = true;
  save_config.save_pose_graph = true;
  save_config.map_frame = map_frame_;
  save_config.base_frame = base_frame_;
  save_config.lidar_frame = lidar_frame_;
  save_config.extrinsic_x = get_parameter("extrinsic_x").as_double();
  save_config.extrinsic_y = get_parameter("extrinsic_y").as_double();
  save_config.extrinsic_z = get_parameter("extrinsic_z").as_double();
  save_config.extrinsic_roll = get_parameter("extrinsic_roll").as_double();
  save_config.extrinsic_pitch = get_parameter("extrinsic_pitch").as_double();
  save_config.extrinsic_yaw = get_parameter("extrinsic_yaw").as_double();
  save_config.surfel_l0_voxel_size =
      get_parameter("frontend_map_l0_voxel_size").as_double();
  save_config.surfel_l1_hierarchy_factor =
      get_parameter("frontend_voxel_hierarchy_factor").as_int();
  save_config.surfel_planarity_threshold =
      get_parameter("frontend_map_planarity_threshold").as_double();
  save_config.surfel_min_l0_for_surfel =
      get_parameter("frontend_min_l0_for_surfel").as_int();

  // Save path: dump_path/saved_map
  const auto dump_path = get_parameter("dump_path").as_string();
  const std::string save_dir = dump_path + "/saved_map";

  auto result = backend_runner_->saveMap(save_dir, save_config);

  response->success = result.success;
  if (result.success) {
    // Save OGM as Nav2-compatible PNG + YAML (with axis alignment)
    bool ogm_saved = false;
    AlignmentResult alignment;
    auto ogm_snapshot = snapshotOgmAssemblyInputs();
    if (enable_ogm_ && ogm_generator_ && !ogm_snapshot.local_grids.empty()) {
      // 1. Assemble initial OGM for alignment detection
      auto initial_grid = ogm_generator_->assembleGlobalGrid(
          ogm_snapshot.local_grids, ogm_snapshot.poses);

      // 2. Detect dominant wall direction
      alignment = MapAligner::detectAlignment(initial_grid);

      if (alignment.aligned) {
        // 3. Rotate all poses by detected angle
        auto aligned_poses = ogm_snapshot.poses;
        MapAligner::rotatePoses(aligned_poses, alignment.angle_rad);

        // 4. Reassemble OGM with rotated poses → axis-aligned
        auto aligned_grid = ogm_generator_->assembleGlobalGrid(
            ogm_snapshot.local_grids, aligned_poses);

        MapSaver saver(save_config);
        ogm_saved = saver.saveOccupancyGridMap(save_dir, aligned_grid);

        RCLCPP_INFO(get_logger(),
            "[MapSave] OGM axis-aligned: angle=%.2f° confidence=%.3f lines=%d",
            alignment.angle_rad * 180.0 / M_PI,
            alignment.confidence, alignment.num_lines_detected);
      } else {
        // No alignment needed — save as-is
        MapSaver saver(save_config);
        ogm_saved = saver.saveOccupancyGridMap(save_dir, initial_grid);
      }

      // 5. Save alignment metadata
      if (alignment.aligned || alignment.num_lines_detected > 0) {
        const std::string align_path = save_dir + "/alignment.yaml";
        std::ofstream af(align_path);
        if (af.is_open()) {
          af << std::fixed << std::setprecision(6);
          af << "# Map axis-alignment metadata\n";
          af << "alignment:\n";
          af << "  aligned: " << (alignment.aligned ? "true" : "false") << "\n";
          af << "  angle_rad: " << alignment.angle_rad << "\n";
          af << "  angle_deg: " << (alignment.angle_rad * 180.0 / M_PI) << "\n";
          af << "  confidence: " << alignment.confidence << "\n";
          af << "  num_lines: " << alignment.num_lines_detected << "\n";
          af << "  method: hough_transform\n";
        }
      }
    }

    std::ostringstream oss;
    oss << "Map saved to " << save_dir
        << " (" << result.global_map_points << " points, "
        << result.num_keyframes << " keyframes, "
        << result.num_submaps << " submaps, "
        << result.num_loop_closures << " loops";
    if (ogm_saved) {
      oss << ", OGM: map.png+map.yaml";
      if (alignment.aligned) {
        oss << " [aligned " << std::fixed << std::setprecision(1)
            << (alignment.angle_rad * 180.0 / M_PI) << "°]";
      }
    }
    oss << ")";
    response->message = oss.str();
    RCLCPP_INFO(get_logger(), "[MapSave] %s", response->message.c_str());
  } else {
    response->message = result.message;
    RCLCPP_ERROR(get_logger(), "[MapSave] Failed: %s", response->message.c_str());
  }
}

// ---------------------------------------------------------------------------
// publishOccupancyGrid — assemble and publish nav_msgs::OccupancyGrid
// ---------------------------------------------------------------------------
SlamNode::OgmAssemblySnapshot SlamNode::snapshotOgmAssemblyInputs() const {
  OgmAssemblySnapshot snapshot;

  std::lock_guard<std::mutex> lk(ogm_mutex_);
  snapshot.local_grids.reserve(ogm_keyframe_assets_.size());
  snapshot.poses.reserve(ogm_keyframe_assets_.size());
  for (const auto& asset : ogm_keyframe_assets_) {
    snapshot.local_grids.push_back(asset.local_grid);
    snapshot.poses.push_back(asset.T_odom_body_nominal);
  }

  const size_t corrected_count =
      std::min(ogm_corrected_poses_.size(), snapshot.poses.size());
  for (size_t i = 0; i < corrected_count; ++i) {
    snapshot.poses[i] = ogm_corrected_poses_[i];
  }

  return snapshot;
}

float SlamNode::publishOccupancyGrid() {
  using Clock = std::chrono::steady_clock;
  if (!ogm_pub_) return 0.0f;

  auto snapshot = snapshotOgmAssemblyInputs();
  if (snapshot.local_grids.empty()) return 0.0f;

  const auto t_start = Clock::now();
  auto global = ogm_generator_->assembleGlobalGrid(snapshot.local_grids,
                                                   snapshot.poses);
  if (global.width <= 0 || global.height <= 0) return 0.0f;

  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = now();
  msg.header.frame_id = map_frame_;
  msg.info.resolution = static_cast<float>(global.resolution);
  msg.info.width = static_cast<uint32_t>(global.width);
  msg.info.height = static_cast<uint32_t>(global.height);
  msg.info.origin.position.x = global.origin_x;
  msg.info.origin.position.y = global.origin_y;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(global.data.begin(), global.data.end());

  ogm_pub_->publish(msg);
  RCLCPP_DEBUG(get_logger(), "[OGM] Published %dx%d (grids=%zu)",
               global.width, global.height, snapshot.local_grids.size());
  return std::chrono::duration<float, std::milli>(Clock::now() - t_start).count();
}

// ---------------------------------------------------------------------------
// rebuildOgmAfterPGO — reassemble OGM with PGO-corrected keyframe poses
// ---------------------------------------------------------------------------
void SlamNode::rebuildOgmAfterPGO() {
  using Clock = std::chrono::steady_clock;
  if (!ogm_generator_ || !backend_runner_ || !ogm_pub_) return;

  std::vector<OgmKeyframeAsset> assets;
  {
    std::lock_guard<std::mutex> lk(ogm_mutex_);
    assets = ogm_keyframe_assets_;
  }
  if (assets.empty()) return;

  const auto t_start = Clock::now();
  const auto pg_snapshot = backend_runner_->manager().getPoseGraphSnapshot();

  std::unordered_map<size_t, Eigen::Matrix4d> optimized_poses;
  optimized_poses.reserve(pg_snapshot.nodes.size());
  size_t max_node_id = 0;
  for (const auto& node : pg_snapshot.nodes) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = node.q.toRotationMatrix();
    T.block<3,1>(0,3) = node.p;
    optimized_poses.emplace(node.id, T);
    max_node_id = std::max(max_node_id, node.id);
  }

  Eigen::Matrix4d correction = Eigen::Matrix4d::Identity();
  {
    std::lock_guard<std::mutex> lk(correction_mutex_);
    correction = T_map_odom_correction_;
  }

  std::vector<LocalGrid> local_grids;
  std::vector<Eigen::Matrix4d> corrected_poses;
  local_grids.reserve(assets.size());
  corrected_poses.reserve(assets.size());

  size_t historical_missing = 0;
  for (const auto& asset : assets) {
    local_grids.push_back(asset.local_grid);

    auto it = optimized_poses.find(asset.backend_keyframe_id);
    if (it != optimized_poses.end()) {
      corrected_poses.push_back(it->second);
      continue;
    }

    corrected_poses.push_back(correction * asset.T_odom_body_nominal);
    if (!pg_snapshot.nodes.empty() && asset.backend_keyframe_id <= max_node_id) {
      ++historical_missing;
      RCLCPP_ERROR(
          get_logger(),
          "[OGM] Missing optimized pose for historical keyframe %zu during rebuild",
          asset.backend_keyframe_id);
    }
  }

  auto global = ogm_generator_->reassembleAfterPGO(local_grids, corrected_poses);
  if (global.width <= 0 || global.height <= 0) return;

  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = now();
  msg.header.frame_id = map_frame_;
  msg.info.resolution = static_cast<float>(global.resolution);
  msg.info.width = static_cast<uint32_t>(global.width);
  msg.info.height = static_cast<uint32_t>(global.height);
  msg.info.origin.position.x = global.origin_x;
  msg.info.origin.position.y = global.origin_y;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(global.data.begin(), global.data.end());

  ogm_pub_->publish(msg);

  {
    std::lock_guard<std::mutex> lk(ogm_mutex_);
    ogm_corrected_poses_ = corrected_poses;
  }

  const float elapsed_ms = std::chrono::duration<float, std::milli>(
      Clock::now() - t_start).count();
  {
    std::lock_guard<std::mutex> lk(ogm_usage_mutex_);
    last_ogm_pgo_rebuild_ms_ = elapsed_ms;
  }

  if (historical_missing > 0) {
    RCLCPP_ERROR(get_logger(),
                 "[OGM] Rebuild used fallback poses for %zu historical keyframes",
                 historical_missing);
  }

  RCLCPP_INFO(get_logger(), "[OGM] Reassembled after PGO: %dx%d (grids=%zu) in %.1f ms",
              global.width, global.height, assets.size(), elapsed_ms);
}

// ---------------------------------------------------------------------------
// publishLoopClosureMarkers — visualize LC links and keyframes in RViz2
// ---------------------------------------------------------------------------
void SlamNode::publishLoopClosureMarkers(const PoseGraphSnapshot& pose_graph) {
  if (!lc_marker_pub_) return;

  const auto& nodes = pose_graph.nodes;
  const auto& edges = pose_graph.edges;

  visualization_msgs::msg::MarkerArray ma;

  // -- LINE_LIST: loop closure edges (cyan) --
  visualization_msgs::msg::Marker lines;
  lines.header.frame_id = map_frame_;
  lines.header.stamp = now();
  lines.ns = "loop_closure";
  lines.id = 0;
  lines.type = visualization_msgs::msg::Marker::LINE_LIST;
  lines.action = visualization_msgs::msg::Marker::ADD;
  lines.scale.x = 0.03;  // line width
  lines.color.r = 0.0f;
  lines.color.g = 1.0f;
  lines.color.b = 1.0f;
  lines.color.a = 0.8f;
  lines.pose.orientation.w = 1.0;

  // -- SPHERE_LIST: LC keyframe positions (magenta) --
  visualization_msgs::msg::Marker spheres;
  spheres.header = lines.header;
  spheres.ns = "loop_closure";
  spheres.id = 1;
  spheres.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  spheres.action = visualization_msgs::msg::Marker::ADD;
  spheres.scale.x = 0.08;
  spheres.scale.y = 0.08;
  spheres.scale.z = 0.08;
  spheres.color.r = 1.0f;
  spheres.color.g = 0.0f;
  spheres.color.b = 1.0f;
  spheres.color.a = 0.8f;
  spheres.pose.orientation.w = 1.0;

  // Helper: find node position by id
  auto findPos = [&](size_t id) -> geometry_msgs::msg::Point {
    geometry_msgs::msg::Point pt;
    for (const auto& n : nodes) {
      if (n.id == id) {
        pt.x = n.p.x();
        pt.y = n.p.y();
        pt.z = n.p.z();
        return pt;
      }
    }
    pt.x = pt.y = pt.z = 0.0;
    return pt;
  };

  std::set<size_t> lc_node_ids;
  int lc_count = 0;

  for (const auto& e : edges) {
    if (e.type != PoseGraphEdge::LOOP) continue;

    auto p1 = findPos(e.from_id);
    auto p2 = findPos(e.to_id);
    lines.points.push_back(p1);
    lines.points.push_back(p2);
    lc_node_ids.insert(e.from_id);
    lc_node_ids.insert(e.to_id);
    ++lc_count;
  }

  for (size_t nid : lc_node_ids) {
    spheres.points.push_back(findPos(nid));
  }

  if (lines.points.empty()) {
    // No LC edges — publish DELETEALL to clear any stale markers
    lines.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(lines);
  } else {
    ma.markers.push_back(lines);
    ma.markers.push_back(spheres);
  }

  lc_marker_pub_->publish(ma);
  RCLCPP_INFO(get_logger(), "[LC-VIZ] Published %d loop closure links, %zu keyframe markers",
              lc_count, lc_node_ids.size());
}

void SlamNode::publishBackendDebugVisualization() {
  if (!backend_runner_ || !enable_loop_closure_) return;

  const auto snapshot = backend_runner_->manager().getDebugSnapshot();
  publishMapperKeyframeMarkers(snapshot.keyframes);
  publishLatestSubmapCloud(snapshot.visual_submaps);
  publishLoopClosureMarkers(snapshot.pose_graph);
  publishLoopClosureSubmapClouds(snapshot.visual_submaps, snapshot.last_results);
}

void SlamNode::publishMapperKeyframeMarkers(
    const std::vector<KeyframePoseSnapshot>& keyframes) {
  if (!mapper_keyframe_pub_) return;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = map_frame_;
  marker.header.stamp = now();
  marker.ns = "mapper_keyframes";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.07;
  marker.scale.y = 0.07;
  marker.scale.z = 0.07;
  marker.color.r = 0.2f;
  marker.color.g = 1.0f;
  marker.color.b = 0.2f;
  marker.color.a = 0.9f;
  marker.pose.orientation.w = 1.0;
  marker.points.reserve(keyframes.size());

  for (const auto& keyframe : keyframes) {
    geometry_msgs::msg::Point pt;
    pt.x = keyframe.p.x();
    pt.y = keyframe.p.y();
    pt.z = keyframe.p.z();
    marker.points.push_back(pt);
  }

  mapper_keyframe_pub_->publish(marker);
}

void SlamNode::publishLatestSubmapCloud(const std::vector<VisualSubmapSnapshot>& submaps) {
  if (!latest_submap_pub_) return;

  const auto stamp = now();
  if (submaps.empty()) {
    latest_submap_pub_->publish(makeColoredCloudMsg(
        pcl::PointCloud<pcl::PointXYZ>{}, map_frame_, stamp, 80, 220, 120));
    return;
  }

  latest_submap_pub_->publish(makeColoredCloudMsg(
      *submaps.back().world_cloud, map_frame_, stamp, 80, 220, 120));
}

void SlamNode::publishLoopClosureSubmapClouds(
    const std::vector<VisualSubmapSnapshot>& submaps,
    const std::vector<GicpMatchResult>& results) {
  if (!loop_source_submap_pub_ || !loop_target_submap_pub_) return;

  const auto stamp = now();
  const GicpMatchResult* accepted = nullptr;
  for (auto it = results.rbegin(); it != results.rend(); ++it) {
    if (it->accepted) {
      accepted = &(*it);
      break;
    }
  }

  if (!accepted) {
    loop_source_submap_pub_->publish(makeColoredCloudMsg(
        pcl::PointCloud<pcl::PointXYZ>{}, map_frame_, stamp, 255, 140, 0));
    loop_target_submap_pub_->publish(makeColoredCloudMsg(
        pcl::PointCloud<pcl::PointXYZ>{}, map_frame_, stamp, 0, 200, 255));
    return;
  }

  const VisualSubmapSnapshot* source = nullptr;
  const VisualSubmapSnapshot* target = nullptr;
  for (const auto& submap : submaps) {
    if (submap.id == accepted->query_submap_id) source = &submap;
    if (submap.id == accepted->match_submap_id) target = &submap;
  }

  loop_source_submap_pub_->publish(makeColoredCloudMsg(
      source && source->world_cloud ? *source->world_cloud : pcl::PointCloud<pcl::PointXYZ>{},
      map_frame_, stamp, 255, 140, 0));
  loop_target_submap_pub_->publish(makeColoredCloudMsg(
      target && target->world_cloud ? *target->world_cloud : pcl::PointCloud<pcl::PointXYZ>{},
      map_frame_, stamp, 0, 200, 255));
}

}  // namespace tof_slam
