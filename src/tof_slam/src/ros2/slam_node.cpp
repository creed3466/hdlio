// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// slam_node.cpp — ROS 2 node wrapping core::LioEstimator.
//
// Timestamp-batched sync model (FAST-LIO2 pattern): IMU and LiDAR callbacks
// push to a unified queue.  The processing thread waits until a LiDAR event
// arrives, drains all events up to it, sorts by sensor timestamp, then
// processes the batch.  This guarantees deterministic IMU-LiDAR grouping
// regardless of DDS transport jitter or OS thread scheduling.

#include "tof_slam/ros2/slam_node.hpp"

#include <algorithm>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iomanip>

namespace tof_slam {

SlamNode::SlamNode(const rclcpp::NodeOptions& options)
    : Node("tofslam_node", options) {
  // ---- Declare parameters --------------------------------------------------
  declare_parameter<std::string>("map_frame", "map");
  declare_parameter<std::string>("base_frame", "base_link");
  declare_parameter<std::string>("imu_topic", "/livox/imu_192_168_0_65");
  declare_parameter<std::string>("lidar_topic", "/livox/lidar_192_168_0_65");

  declare_parameter<double>("imu_gyro_noise", 0.1);
  declare_parameter<double>("imu_accel_noise", 0.1);
  declare_parameter<double>("imu_gyro_bias_noise", 0.0001);
  declare_parameter<double>("imu_accel_bias_noise", 0.0001);
  declare_parameter<double>("imu_gravity_z", -9.7946);
  declare_parameter<int>("imu_init_samples", 100);

  declare_parameter<double>("extrinsic_x", 0.0);
  declare_parameter<double>("extrinsic_y", 0.0);
  declare_parameter<double>("extrinsic_z", 0.0);

  declare_parameter<double>("frontend_voxel_size", 0.1);
  declare_parameter<int>("frontend_max_iterations", 4);
  declare_parameter<double>("frontend_convergence_threshold", 0.001);
  declare_parameter<bool>("frontend_enable_undistortion", false);
  declare_parameter<double>("frontend_min_distance", 0.5);
  declare_parameter<double>("frontend_max_distance", 100.0);
  declare_parameter<double>("frontend_map_planarity_threshold", 0.1);
  declare_parameter<double>("frontend_scan_duration", 0.1);
  declare_parameter<int>("frontend_init_imu_samples", 100);
  declare_parameter<int>("frontend_voxel_hierarchy_factor", 3);
  declare_parameter<double>("frontend_map_box_multiplier", 2.0);
  declare_parameter<int>("frontend_stride", 4);
  declare_parameter<bool>("frontend_stride_then_voxel", true);
  declare_parameter<double>("frontend_lidar_noise_std", 0.05);
  declare_parameter<double>("frontend_map_l0_voxel_size", -1.0);  // -1 = use frontend_voxel_size
  declare_parameter<int>("frontend_max_correspondences", 0);     // 0 = no limit
  declare_parameter<int>("frontend_min_l0_for_surfel", 3);
  declare_parameter<double>("frontend_l0_ema_alpha_min", 0.0);    // 0 = original running mean

  declare_parameter<int>("frontend_max_inner_iterations", 4);
  declare_parameter<double>("frontend_max_plane_distance", 0.0);  // 0 = no limit

  // Degeneracy-Aware IEKF
  declare_parameter<bool>("frontend_enable_degeneracy_detection", true);
  declare_parameter<double>("frontend_degeneracy_threshold", 50.0);

  // Covariance floor
  declare_parameter<double>("frontend_p_floor_rot", 1e-6);
  declare_parameter<double>("frontend_p_floor_pos", 1e-6);
  declare_parameter<double>("frontend_p_floor_vel", 1e-4);
  declare_parameter<double>("frontend_p_floor_bias", 1e-8);
  declare_parameter<double>("frontend_p_floor_grav", 1e-8);

  // IMU Bias Pseudo-Observation
  declare_parameter<bool>("frontend_enable_bias_pseudo_obs", false);
  declare_parameter<double>("frontend_bias_bg_sigma", 0.01);
  declare_parameter<double>("frontend_bias_ba_sigma", 0.1);

  // Velocity Pseudo-Observation
  declare_parameter<bool>("frontend_enable_velocity_pseudo_obs", false);
  declare_parameter<double>("frontend_velocity_sigma", 1.0);

  // Gravity norm constraint
  declare_parameter<bool>("frontend_enable_gravity_norm_constraint", true);
  declare_parameter<double>("frontend_gravity_norm_sigma", 0.01);

  // Degeneracy-Adaptive EMA Alpha
  declare_parameter<bool>("frontend_enable_degeneracy_adaptive_alpha", false);
  declare_parameter<double>("frontend_degeneracy_alpha_scale", 1.0);

  // PointVoxelMap: per-query kNN plane fitting (Faster-LIO style)
  declare_parameter<double>("frontend_pvmap_voxel_size", 0.5);
  declare_parameter<int>("frontend_pvmap_max_points_per_voxel", 20);
  declare_parameter<int>("frontend_pvmap_k_neighbors", 5);
  declare_parameter<double>("frontend_pvmap_planarity_threshold", 0.15);

  // L2 Multi-Scale Correspondence
  declare_parameter<bool>("frontend_enable_l2_correspondences", false);
  declare_parameter<double>("frontend_l2_planarity_threshold", 0.15);
  declare_parameter<int>("frontend_min_l1_for_l2_surfel", 4);
  declare_parameter<double>("frontend_l2_noise_scale", 9.0);

  // Debug timing
  declare_parameter<bool>("frontend_enable_debug_timing", false);

  declare_parameter<std::string>("point_time_field", "timestamp");
  declare_parameter<std::string>("point_time_unit", "sec");
  declare_parameter<std::string>("point_time_reference", "absolute");

  // ---- Read parameters -----------------------------------------------------
  map_frame_  = get_parameter("map_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  const auto imu_topic   = get_parameter("imu_topic").as_string();
  const auto lidar_topic = get_parameter("lidar_topic").as_string();

  const double gyr_noise  = get_parameter("imu_gyro_noise").as_double();
  const double acc_noise  = get_parameter("imu_accel_noise").as_double();
  const double bgyr_noise = get_parameter("imu_gyro_bias_noise").as_double();
  const double bacc_noise = get_parameter("imu_accel_bias_noise").as_double();
  const double gravity_z  = get_parameter("imu_gravity_z").as_double();

  int init_samples = get_parameter("imu_init_samples").as_int();
  const int frontend_init = get_parameter("frontend_init_imu_samples").as_int();
  if (frontend_init > 0) init_samples = frontend_init;

  const double ext_x = get_parameter("extrinsic_x").as_double();
  const double ext_y = get_parameter("extrinsic_y").as_double();
  const double ext_z = get_parameter("extrinsic_z").as_double();

  // ---- Build LioEstimator::Config ------------------------------------------
  // YAML values (imu_gyro_noise etc.) are VARIANCES (σ²).
  // Q diagonal = noise_std² = variance.  So noise_std = sqrt(variance).
  core::LioEstimator::Config cfg;
  cfg.gyro_noise_std      = static_cast<float>(std::sqrt(gyr_noise));
  cfg.acc_noise_std       = static_cast<float>(std::sqrt(acc_noise));
  cfg.gyro_bias_noise_std = static_cast<float>(std::sqrt(bgyr_noise));
  cfg.acc_bias_noise_std  = static_cast<float>(std::sqrt(bacc_noise));

  RCLCPP_INFO(get_logger(),
    "Q noise std: gyr=%.6f acc=%.6f bgyr=%.6f bacc=%.6f grav=%.6f",
    cfg.gyro_noise_std, cfg.acc_noise_std,
    cfg.gyro_bias_noise_std, cfg.acc_bias_noise_std,
    cfg.gravity_noise_std);

  cfg.stride          = get_parameter("frontend_stride").as_int();
  cfg.voxel_leaf_size = static_cast<float>(
      get_parameter("frontend_voxel_size").as_double());
  cfg.min_range = static_cast<float>(
      get_parameter("frontend_min_distance").as_double());
  cfg.max_range = static_cast<float>(
      get_parameter("frontend_max_distance").as_double());

  cfg.iekf.max_outer_iters        = get_parameter("frontend_max_iterations").as_int();
  cfg.iekf.max_inner_iters        = get_parameter("frontend_max_inner_iterations").as_int();
  cfg.iekf.convergence_threshold  = static_cast<float>(
      get_parameter("frontend_convergence_threshold").as_double());
  cfg.iekf.lidar_noise_std = static_cast<float>(
      get_parameter("frontend_lidar_noise_std").as_double());

  // Degeneracy-Aware IEKF
  cfg.iekf.enable_degeneracy_detection =
      get_parameter("frontend_enable_degeneracy_detection").as_bool();
  cfg.iekf.degeneracy_threshold = static_cast<float>(
      get_parameter("frontend_degeneracy_threshold").as_double());

  // Covariance floor
  cfg.iekf.p_floor_rot = static_cast<float>(
      get_parameter("frontend_p_floor_rot").as_double());
  cfg.iekf.p_floor_pos = static_cast<float>(
      get_parameter("frontend_p_floor_pos").as_double());
  cfg.iekf.p_floor_vel = static_cast<float>(
      get_parameter("frontend_p_floor_vel").as_double());
  cfg.iekf.p_floor_bias = static_cast<float>(
      get_parameter("frontend_p_floor_bias").as_double());
  cfg.iekf.p_floor_grav = static_cast<float>(
      get_parameter("frontend_p_floor_grav").as_double());

  // IMU Bias Pseudo-Observation
  cfg.iekf.enable_bias_pseudo_obs =
      get_parameter("frontend_enable_bias_pseudo_obs").as_bool();
  cfg.iekf.bias_bg_sigma = static_cast<float>(
      get_parameter("frontend_bias_bg_sigma").as_double());
  cfg.iekf.bias_ba_sigma = static_cast<float>(
      get_parameter("frontend_bias_ba_sigma").as_double());

  // Velocity Pseudo-Observation
  cfg.iekf.enable_velocity_pseudo_obs =
      get_parameter("frontend_enable_velocity_pseudo_obs").as_bool();
  cfg.iekf.velocity_sigma = static_cast<float>(
      get_parameter("frontend_velocity_sigma").as_double());

  // Gravity norm constraint
  cfg.iekf.enable_gravity_norm_constraint =
      get_parameter("frontend_enable_gravity_norm_constraint").as_bool();
  cfg.iekf.gravity_norm_sigma = static_cast<float>(
      get_parameter("frontend_gravity_norm_sigma").as_double());

  // PointVoxelMap
  cfg.point_voxel_map.voxel_size = static_cast<float>(
      get_parameter("frontend_pvmap_voxel_size").as_double());
  cfg.point_voxel_map.max_points_per_voxel =
      get_parameter("frontend_pvmap_max_points_per_voxel").as_int();
  cfg.point_voxel_map.max_distance = cfg.max_range;
  cfg.pvmap_k_neighbors =
      get_parameter("frontend_pvmap_k_neighbors").as_int();
  cfg.pvmap_planarity_threshold = static_cast<float>(
      get_parameter("frontend_pvmap_planarity_threshold").as_double());

  // Debug timing
  cfg.enable_debug_timing =
      get_parameter("frontend_enable_debug_timing").as_bool();
  cfg.iekf.enable_debug_timing = cfg.enable_debug_timing;

  RCLCPP_INFO(get_logger(),
    "IEKF: degeneracy=%s (thresh=%.1f) P_floor=%.1e debug_timing=%s",
    cfg.iekf.enable_degeneracy_detection ? "ON" : "OFF",
    cfg.iekf.degeneracy_threshold,
    cfg.iekf.p_floor_pos,
    cfg.enable_debug_timing ? "ON" : "OFF");

  // Map L0 voxel size: use dedicated param if set, else fall back to scan voxel.
  const double map_l0 = get_parameter("frontend_map_l0_voxel_size").as_double();
  cfg.surfel_map.l0_voxel_size = (map_l0 > 0.0)
      ? static_cast<float>(map_l0) : cfg.voxel_leaf_size;
  cfg.surfel_map.l1_hierarchy_factor =
      get_parameter("frontend_voxel_hierarchy_factor").as_int();
  cfg.surfel_map.min_l0_for_surfel =
      get_parameter("frontend_min_l0_for_surfel").as_int();
  cfg.surfel_map.max_distance        = cfg.max_range;
  cfg.surfel_map.planarity_threshold = static_cast<float>(
      get_parameter("frontend_map_planarity_threshold").as_double());
  cfg.surfel_map.distance_multiplier = static_cast<float>(
      get_parameter("frontend_map_box_multiplier").as_double());
  cfg.surfel_map.l0_ema_alpha_min = static_cast<float>(
      get_parameter("frontend_l0_ema_alpha_min").as_double());

  // L2 Multi-Scale Correspondence
  cfg.surfel_map.enable_l2_correspondences =
      get_parameter("frontend_enable_l2_correspondences").as_bool();
  cfg.surfel_map.l2_planarity_threshold = static_cast<float>(
      get_parameter("frontend_l2_planarity_threshold").as_double());
  cfg.surfel_map.min_l1_for_l2_surfel =
      get_parameter("frontend_min_l1_for_l2_surfel").as_int();
  cfg.surfel_map.l2_noise_scale = static_cast<float>(
      get_parameter("frontend_l2_noise_scale").as_double());
  cfg.iekf.enable_l2_correspondences = cfg.surfel_map.enable_l2_correspondences;
  cfg.iekf.l2_noise_scale            = cfg.surfel_map.l2_noise_scale;
  if (cfg.surfel_map.enable_l2_correspondences) {
    RCLCPP_INFO(get_logger(),
                "L2 Multi-Scale: ENABLED (planarity=%.2f min_l1=%d noise_scale=%.1f)",
                cfg.surfel_map.l2_planarity_threshold,
                cfg.surfel_map.min_l1_for_l2_surfel,
                cfg.surfel_map.l2_noise_scale);
  }

  cfg.max_correspondences =
      get_parameter("frontend_max_correspondences").as_int();
  cfg.max_plane_distance = static_cast<float>(
      get_parameter("frontend_max_plane_distance").as_double());

  cfg.enable_undistortion =
      get_parameter("frontend_enable_undistortion").as_bool();
  cfg.scan_duration = static_cast<float>(
      get_parameter("frontend_scan_duration").as_double());
  cfg.enable_degeneracy_adaptive_alpha =
      get_parameter("frontend_enable_degeneracy_adaptive_alpha").as_bool();
  cfg.degeneracy_alpha_scale = static_cast<float>(
      get_parameter("frontend_degeneracy_alpha_scale").as_double());

  // Extrinsics: translation only (R = Identity for MID-360)
  cfg.T_body_lidar = core::Se3(
      Eigen::Matrix3f::Identity(),
      Eigen::Vector3f(static_cast<float>(ext_x),
                      static_cast<float>(ext_y),
                      static_cast<float>(ext_z)));

  estimator_ = std::make_unique<core::LioEstimator>(cfg);

  // ---- ImuAdapter ----------------------------------------------------------
  ros_adapter::ImuAdapter::Config imu_cfg;
  imu_cfg.init_sample_count = init_samples;
  imu_cfg.gravity_prior     = Eigen::Vector3f(0.0f, 0.0f,
                                              static_cast<float>(gravity_z));
  imu_adapter_ = ros_adapter::ImuAdapter(imu_cfg);

  // ---- CSV logging ---------------------------------------------------------
  {
    std::lock_guard<std::mutex> lk(csv_mutex_);
    csv_file_.open("/root/tofslam_traj.csv", std::ios::out | std::ios::trunc);
    if (csv_file_.is_open()) {
      csv_file_ << "t_sec,tx,ty,tz,qx,qy,qz,qw\n";
      csv_file_.flush();
      RCLCPP_INFO(get_logger(), "Trajectory CSV: /root/tofslam_traj.csv");
    } else {
      RCLCPP_WARN(get_logger(),
                  "Cannot open /root/tofslam_traj.csv for writing");
    }
  }

  // ---- Publishers ----------------------------------------------------------
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/tofslam/odometry", 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("/tofslam/path", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  path_msg_.header.frame_id = map_frame_;

  // ---- Subscribers ---------------------------------------------------------
  // IMU: best_effort with large queue (high rate, lightweight processing).
  // LiDAR: reliable with large queue to prevent frame drops during
  //   slow processing.  Combined with ros2 bag play's default reliable
  //   publisher, this guarantees ALL frames are delivered to the callback.
  //   The internal event queue (unbounded deque) buffers them until the
  //   processing thread catches up.
  auto imu_qos = rclcpp::QoS(
      rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data))
      .best_effort()
      .keep_last(2000);

  auto lidar_qos = rclcpp::QoS(
      rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
      .reliable()
      .keep_last(2000);

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, imu_qos,
      [this](sensor_msgs::msg::Imu::SharedPtr msg) { imu_callback(msg); });

  lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic, lidar_qos,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        lidar_callback(msg);
      });

  // ---- Processing thread ---------------------------------------------------
  processing_thread_ = std::thread(&SlamNode::processing_loop, this);

  RCLCPP_INFO(get_logger(),
              "TofSLAM node ready (unified queue). imu=%s lidar=%s "
              "init_samples=%d undistort=%s",
              imu_topic.c_str(), lidar_topic.c_str(), init_samples,
              cfg.enable_undistortion ? "ON" : "OFF");
}

SlamNode::~SlamNode() {
  running_ = false;
  queue_cv_.notify_all();
  if (processing_thread_.joinable()) processing_thread_.join();
  std::lock_guard<std::mutex> lk(csv_mutex_);
  if (csv_file_.is_open()) csv_file_.close();
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
// LiDAR callback — pushes to unified event queue (lightweight)
// ---------------------------------------------------------------------------
void SlamNode::lidar_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  const double ts =
      msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

  // Dedup: skip if timestamp is too close to last LiDAR (DDS can re-deliver).
  // LiDAR is 10Hz → min dt ~ 100ms. Guard at 50ms.
  {
    std::lock_guard<std::mutex> lk(queue_mutex_);
    if (ts - last_lidar_queue_ts_ < 0.05 && last_lidar_queue_ts_ > 0.0) return;
    last_lidar_queue_ts_ = ts;
  }

  SensorEvent ev;
  ev.type = SensorEvent::LIDAR;
  ev.cloud = pc_adapter_.convert(msg);
  ev.timestamp = ts;

  {
    std::lock_guard<std::mutex> lk(queue_mutex_);
    event_queue_.push_back(std::move(ev));
  }
  queue_cv_.notify_one();
}

// ---------------------------------------------------------------------------
// Processing thread — timestamp-batched sync (FAST-LIO2 pattern)
//
// Wait until at least one LiDAR event is queued, then drain all events up to
// (and including) the first LiDAR plus any late-arriving IMU with timestamp
// <= lidar_ts.  Sort the batch by sensor timestamp, then process in order.
// This ensures deterministic IMU-LiDAR grouping regardless of DDS/transport
// jitter or OS thread scheduling.
// ---------------------------------------------------------------------------
void SlamNode::processing_loop() {
  RCLCPP_INFO(get_logger(), "Processing thread started (timestamp-batched sync).");
  size_t n_imu = 0;
  size_t n_lidar = 0;

  while (running_) {
    // ---- Step 1: Wait until we have at least one LiDAR event ----
    std::vector<SensorEvent> batch;
    {
      std::unique_lock<std::mutex> lk(queue_mutex_);
      queue_cv_.wait(lk, [this] {
        if (!running_) return true;
        for (const auto& ev : event_queue_) {
          if (ev.type == SensorEvent::LIDAR) return true;
        }
        return false;
      });

      if (!running_ && event_queue_.empty()) break;

      // ---- Step 2: Find the first LiDAR and drain all events up to it ----
      double lidar_ts = 0.0;
      size_t lidar_idx = 0;
      for (size_t i = 0; i < event_queue_.size(); ++i) {
        if (event_queue_[i].type == SensorEvent::LIDAR) {
          lidar_ts = event_queue_[i].timestamp;
          lidar_idx = i;
          break;
        }
      }

      // Drain: take all events from front up to and including the LiDAR
      for (size_t i = 0; i <= lidar_idx; ++i) {
        batch.push_back(std::move(event_queue_.front()));
        event_queue_.pop_front();
      }
      // Also grab any IMU events right after the LiDAR that have
      // timestamp <= lidar_ts (arrived late due to transport jitter).
      while (!event_queue_.empty() &&
             event_queue_.front().type == SensorEvent::IMU &&
             event_queue_.front().timestamp <= lidar_ts) {
        batch.push_back(std::move(event_queue_.front()));
        event_queue_.pop_front();
      }
    }

    if (batch.empty()) continue;

    // ---- Step 3: Sort batch by sensor timestamp ----
    std::sort(batch.begin(), batch.end(),
              [](const SensorEvent& a, const SensorEvent& b) {
                if (a.timestamp != b.timestamp) return a.timestamp < b.timestamp;
                // For equal timestamps: IMU before LiDAR (IMU must propagate first)
                return a.type == SensorEvent::IMU && b.type == SensorEvent::LIDAR;
              });

    // ---- Step 4: Process the sorted batch ----
    try {
      for (auto& ev : batch) {
        if (ev.type == SensorEvent::IMU) {
          auto result = imu_adapter_.process(ev.imu_msg);

          if (!estimator_->initialized() && imu_adapter_.initialized()) {
            const auto& ir = imu_adapter_.init_result();
            if (ir.success) {
              if (estimator_->initialize(ir)) {
                RCLCPP_INFO(get_logger(), "Gravity init OK: %s",
                            ir.message.c_str());
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

          if (result.has_value() && estimator_->initialized()) {
            estimator_->feed_imu(*result);
            ++n_imu;
          }
        } else {
          if (!estimator_->initialized()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Waiting for gravity initialization...");
          } else if (estimator_->feed_lidar(ev.cloud, ev.timestamp)) {
            ++n_lidar;
            publish_state(estimator_->current_state(), ev.timestamp);
          }
        }
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Exception in processing loop: %s",
                   e.what());
    }
  }

  RCLCPP_INFO(get_logger(),
              "Processing thread stopped. lidar=%zu imu=%zu", n_lidar, n_imu);
}

// ---------------------------------------------------------------------------
// Publish odometry, path, TF and CSV from a LioState snapshot
// ---------------------------------------------------------------------------
void SlamNode::publish_state(const core::LioState& state,
                                  double timestamp) {
  const rclcpp::Time ros_time(static_cast<int64_t>(timestamp * 1e9));
  Eigen::Quaternionf q(state.rotation);
  q.normalize();

  // Odometry
  nav_msgs::msg::Odometry odom;
  odom.header.stamp    = ros_time;
  odom.header.frame_id = map_frame_;
  odom.child_frame_id  = base_frame_;
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
  odom_pub_->publish(odom);

  // Path
  geometry_msgs::msg::PoseStamped ps;
  ps.header          = odom.header;
  ps.pose            = odom.pose.pose;
  path_msg_.header.stamp = ros_time;
  path_msg_.poses.push_back(ps);
  path_pub_->publish(path_msg_);

  // TF
  geometry_msgs::msg::TransformStamped tf;
  tf.header           = odom.header;
  tf.child_frame_id   = base_frame_;
  tf.transform.translation.x = state.position.x();
  tf.transform.translation.y = state.position.y();
  tf.transform.translation.z = state.position.z();
  tf.transform.rotation.x    = q.x();
  tf.transform.rotation.y    = q.y();
  tf.transform.rotation.z    = q.z();
  tf.transform.rotation.w    = q.w();
  tf_broadcaster_->sendTransform(tf);

  // CSV
  std::lock_guard<std::mutex> lk(csv_mutex_);
  if (csv_file_.is_open()) {
    csv_file_ << std::fixed << std::setprecision(9)
              << timestamp       << ","
              << state.position.x() << "," << state.position.y() << ","
              << state.position.z() << ","
              << q.x() << "," << q.y() << "," << q.z() << "," << q.w()
              << "\n";
    csv_file_.flush();
  }
}

}  // namespace tof_slam
