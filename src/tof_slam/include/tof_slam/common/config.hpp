#pragma once

#include <string>

namespace tof_slam {

/// Runtime configuration for the TofSLAM pipeline
struct TofSlamConfig {
  // Sensor (3D ToF / LiDAR)
  double tof_range_min = 0.1;    // m
  double tof_range_max = 4.0;    // m
  double tof_voxel_size = 0.05;  // m
  double statistical_outlier_mean_k = 10;   // neighbors for SOR
  double statistical_outlier_stddev = 1.5;  // stddev threshold
  int preprocess_sor_point_threshold = 4000;  // enable pre-voxel path above this size
  double preprocess_presample_voxel_size = 0.10;  // m, coarse voxel before SOR

  // Point timing (for Point-LIO parity deskew/scheduling)
  // Per-point time is used as "relative time from frame start" in seconds.
  // Many sensors encode it as:
  // - absolute time (sec) in a float/double field, or
  // - relative time in ns/us/ms stored as integer.
  // These parameters allow runtime adaptation per dataset/bag.
  std::string point_time_field = "timestamp";   // PointCloud2 field name (e.g. "timestamp", "time", "t")
  std::string point_time_unit = "sec";          // "sec" | "ms" | "us" | "ns"
  std::string point_time_reference = "absolute"; // "absolute" | "relative"

  // Registration
  int max_iterations = 5;
  double convergence_trans = 0.001;  // m
  double convergence_rot = 0.0003;   // rad (~0.02 deg)
  double huber_delta = 0.1;          // loop-edge robust loss delta
  double pgo_numeric_diff_eps = 1e-6;  // finite-difference epsilon for backend SE3 factor
  bool pgo_use_loop_huber = true;      // apply robust loss on loop edges

  // Wheel odometry (optional)
  bool use_wheel_odometry = true;    // set false if no wheel odom available
  double wheel_cov_x = 0.01;
  double wheel_cov_y = 0.05;
  double wheel_cov_z = 0.001;      // Ground robot: z barely changes per step
  double wheel_cov_roll = 0.001;   // Ground robot: roll ≈ 0
  double wheel_cov_pitch = 0.001;  // Ground robot: pitch ≈ 0
  double wheel_cov_yaw = 0.02;
  double slip_inflation_gain = 5.0;

  // Default prior covariance when no wheel odometry
  double no_odom_cov_rot = 0.1;    // rad^2 per step
  double no_odom_cov_trans = 0.05; // m^2 per step

  // Mapping (iVox — Faster-LIO style)
  double map_voxel_size = 0.1;     // m
  double map_radius = 20.0;        // m
  int min_support_points = 3;
  int ivox_nearby_type = 18;       // 0=CENTER, 6=NEARBY6, 18=NEARBY18, 26=NEARBY26
  size_t ivox_capacity = 1000000;  // LRU cache max voxels (Faster-LIO default)

  // Degeneracy
  double degeneracy_threshold = 100.0;

  // Loop Closure
  bool enable_loop_closure = true;
  int loop_min_keyframe_gap = 20;
  double loop_score_threshold = 0.3;
  double loop_ring_key_threshold = 0.5;
  double loop_icp_fitness_threshold = 0.3;
  int loop_icp_max_iterations = 50;
  double loop_icp_max_correspondence_distance = 2.0;

  // Sensor extrinsic: base_frame → sensor_frame (T_bs)
  bool extrinsic_est_en = false;  // enable online extrinsic estimation in EKF
  double extrinsic_x = 0.0;
  double extrinsic_y = 0.0;
  double extrinsic_z = 0.15;
  double extrinsic_roll = 0.0;
  double extrinsic_pitch = 0.0;
  double extrinsic_yaw = 0.0;

  // Keyframe
  double keyframe_trans_thresh = 0.5;  // m
  double keyframe_rot_thresh = 0.1;    // rad

  // Frame IDs
  std::string map_frame = "map";
  std::string odom_frame = "odom";
  std::string base_frame = "base_link";
  std::string sensor_frame = "camera";

  // Input topics
  std::string odom_topic = "/odom";
  std::string pointcloud_topic = "/camera/depth/points";
  std::string imu_topic = "/imu/data";

  // OGM (Occupancy Grid Map) — 2D projection from 3D point cloud
  double ogm_resolution = 0.05;       // m per cell
  double ogm_local_range = 5.0;       // m, local grid range
  double ogm_log_odds_hit = 0.85;     // log-odds for occupied
  double ogm_log_odds_miss = -0.4;    // log-odds for free
  double ogm_log_odds_max = 3.5;      // clamping max
  double ogm_log_odds_min = -2.0;     // clamping min
  double ogm_height_min = -0.1;       // m, min z for OGM projection
  double ogm_height_max = 1.5;        // m, max z for OGM projection
  int ogm_endpoint_dilation_radius = 1;  // occupied endpoint thickening in cells
  int ogm_global_dilation_radius = 0;    // post-assembly occupied dilation in cells
  int ogm_global_padding_cells = 0;      // padding around canonical global canvas in cells
  bool ogm_crop_to_observed_bounds = false;  // crop global canvas to observed support instead of pose envelope
  double ogm_origin_bias_x = 0.0;        // m, shifts map content left/right in image
  double ogm_origin_bias_y = 0.0;        // m, shifts map content up/down in image
  bool ogm_anchor_first_pose = false;    // keep global min corner anchored to the first keyframe pose
  int ogm_publish_stride = 5;         // publish local/global OGM every N keyframes
  int ogm_publish_cloud_gap = 20;     // min cloud gap between OGM publishes
  int backend_opt_keyframe_stride = 10;  // run backend every N keyframes
  int backend_opt_cloud_gap = 20;        // min cloud gap between backend runs

  // Per-Point EKF (Point-LIO style; single update path)
  bool use_per_point_ekf = true;
  double ekf_measurement_noise = 0.005;    // R per point (m^2)
  int ekf_max_points = 2000;               // max points for EKF update (downsample if needed)
  double ekf_velocity_noise = 0.1;         // process noise for velocity state
  double ekf_chi2_threshold = 6.635;       // 99% chi-squared 1-DOF
  double ekf_rot_lost_thresh = 1.0;        // rad^2
  double ekf_trans_lost_thresh = 5.0;      // m^2
  double ekf_rot_degraded_thresh = 0.1;    // rad^2
  double ekf_trans_degraded_thresh = 0.5;  // m^2
  int iekf_max_iterations = 2;  // Batch IEKF iterations (Faster-LIO style)
  double iekf_convergence_trans = 0.001;
  double iekf_convergence_rot = 0.0005;
  double ekf_noise_dist_alpha = 0.5;
  double ekf_noise_min_cos_incidence = 0.1;
  double ekf_subsample_voxel_size = 0.3;
  double ekf_match_threshold = 0.5;  // max point-to-surfel distance (m) for correspondence
  double match_s = 81.0;              // Point-LIO: reject if p_norm > match_s * pd2^2 (adaptive)
  double plane_thr = 0.1;             // Point-LIO: plane flatness (surfel: reject if lambda_min > plane_thr*lambda_mid)

  // Per-Point EKF map insertion hardening
  int map_insert_interval = 2;
  double map_insert_cov_threshold = 1.0;
  int map_insert_min_updated = 10;
  int point_lio_num_segments = 4;
  bool point_lio_require_timestamps = true;
  bool point_lio_disable_fallback_icp = true;

  // [M7-T2] Ground constraint noise (configurable, replaces hardcoded values)
  // For a ground robot without IMU, these pseudo-measurements constrain roll/pitch/z.
  // Values relaxed from 0.0001 (too tight -> P exhaustion) to realistic floor noise.
  double ground_z_noise = 0.001;    // m  -- z-position constraint (was 0.0001)
  double ground_rp_noise = 0.01;    // rad -- roll/pitch constraint (was 0.0001, 100x relaxed)
  double ground_vz_noise = 0.01;    // m/s -- z-velocity constraint (was 0.001)

  // IMU integration (Phase 5) [Architecture.md SS13.6]
  bool use_imu = false;
  double imu_gyro_noise = 0.01;           // rad/s/sqrt(Hz)
  double imu_accel_noise = 0.1;           // m/s^2/sqrt(Hz)
  double imu_gyro_bias_noise = 0.0001;    // rad/s^2/sqrt(Hz)
  double imu_accel_bias_noise = 0.001;    // m/s^3/sqrt(Hz)
  double imu_gravity_x = 0.0;
  double imu_gravity_y = 0.0;
  double imu_gravity_z = -9.81;
  int    imu_init_samples = 100;
  double imu_extrinsic_x = 0.0;
  double imu_extrinsic_y = 0.0;
  double imu_extrinsic_z = 0.0;
  double imu_extrinsic_roll = 0.0;
  double imu_extrinsic_pitch = 0.0;
  double imu_extrinsic_yaw = 0.0;
  double imu_wheel_velocity_noise = 0.05; // m/s

  // Point-LIO state_output process noise parameters
  double plio_vel_cov = 20.0;              // velocity process noise
  double plio_gyr_cov_output = 0.1;        // angular velocity process noise
  double plio_acc_cov_output = 0.1;        // acceleration process noise
  double plio_b_gyr_cov = 0.0001;          // gyro bias random walk
  double plio_b_acc_cov = 0.0001;          // accel bias random walk
  double plio_laser_point_cov = 0.01;      // LiDAR measurement noise
  double plio_imu_meas_omg_cov = 0.1;      // IMU angular velocity measurement noise
  double plio_imu_meas_acc_cov = 0.1;      // IMU acceleration measurement noise
  double plio_satu_gyro = 35.0;            // gyro saturation limit (rad/s)
  double plio_satu_acc = 30.0;             // accel saturation limit (m/s^2)
  bool   plio_check_satu = true;           // enable IMU saturation check
  double plio_G_m_s2 = 9.81;              // gravity magnitude for accel normalization

  // Output topics
  std::string output_odom_topic = "/tof_slam/odom";
  std::string output_path_topic = "/tof_slam/path";
  std::string output_debug_topic = "/tof_slam/debug";
  std::string output_cloud_topic = "/tof_slam/cloud";
  std::string output_ogm_topic = "/map";

  // ---- IG-LIO frontend parameters ----
  double iglio_voxel_size = 0.1;
  int    iglio_stride = 4;
  bool   iglio_stride_then_voxel = true;
  int    iglio_max_iterations = 4;
  double iglio_convergence_threshold = 0.001;
  bool   iglio_enable_undistortion = false;
  double iglio_min_distance = 3.0;
  double iglio_max_distance = 100.0;
  double iglio_map_planarity_threshold = 0.1;
  double iglio_scan_duration = 0.1;
  int    iglio_init_imu_samples = 100;
  int    iglio_voxel_hierarchy_factor = 3;
  double iglio_map_box_multiplier = 2.0;
  double iglio_scan_planarity_threshold = 0.1;
  double iglio_point_to_surfel_threshold = 0.1;
  int    iglio_min_surfel_inliers = 5;
  double iglio_min_linearity_ratio = 0.3;
  double iglio_lidar_noise_std = 0.05;
};

}  // namespace tof_slam
