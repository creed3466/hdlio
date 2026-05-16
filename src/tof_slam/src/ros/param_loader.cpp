// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// param_loader.cpp — Stateless free functions for ROS parameter loading.

#include "tof_slam/ros/param_loader.hpp"

#include <cmath>

namespace tof_slam {
namespace param_loader {

// ---------------------------------------------------------------------------
// declare_all_params — all ROS parameter declarations
// ---------------------------------------------------------------------------
void declare_all_params(rclcpp::Node& node) {
  node.declare_parameter<std::string>("map_frame", "map");
  node.declare_parameter<std::string>("odom_frame", "odom");
  node.declare_parameter<std::string>("base_frame", "base_link");
  node.declare_parameter<std::string>("lidar_frame", "livox_frame");
  node.declare_parameter<std::string>("imu_topic", "/livox/imu_192_168_0_65");
  node.declare_parameter<std::string>("lidar_topic", "/livox/lidar_192_168_0_65");

  // Mode selection
  node.declare_parameter<bool>("use_imu", true);
  node.declare_parameter<bool>("use_wheel_odometry", false);
  node.declare_parameter<std::string>("odom_topic", "/odom");

  // IMU noise
  node.declare_parameter<double>("imu_gyro_noise", 0.1);
  node.declare_parameter<double>("imu_accel_noise", 0.1);
  node.declare_parameter<double>("imu_gyro_bias_noise", 0.0001);
  node.declare_parameter<double>("imu_accel_bias_noise", 0.0001);
  node.declare_parameter<double>("imu_gravity_z", -9.7946);
  node.declare_parameter<int>("imu_init_samples", 100);

  // Extrinsics
  node.declare_parameter<double>("extrinsic_x", 0.0);
  node.declare_parameter<double>("extrinsic_y", 0.0);
  node.declare_parameter<double>("extrinsic_z", 0.0);
  node.declare_parameter<double>("extrinsic_roll",  0.0);
  node.declare_parameter<double>("extrinsic_pitch", 0.0);
  node.declare_parameter<double>("extrinsic_yaw",   0.0);
  node.declare_parameter<std::vector<double>>("extrinsic_rotation",
      std::vector<double>{1,0,0, 0,1,0, 0,0,1});

  // Scan preprocessing
  node.declare_parameter<double>("frontend_voxel_size", 0.1);
  node.declare_parameter<int>("frontend_max_iterations", 4);
  node.declare_parameter<double>("frontend_convergence_threshold", 0.001);
  node.declare_parameter<bool>("frontend_enable_undistortion", false);
  node.declare_parameter<double>("frontend_min_distance", 0.5);
  node.declare_parameter<double>("frontend_max_distance", 100.0);
  node.declare_parameter<double>("frontend_map_planarity_threshold", 0.1);
  node.declare_parameter<double>("frontend_scan_duration", 0.1);
  node.declare_parameter<int>("frontend_init_imu_samples", 100);
  node.declare_parameter<int>("frontend_voxel_hierarchy_factor", 3);
  node.declare_parameter<double>("frontend_map_box_multiplier", 2.0);
  node.declare_parameter<int>("frontend_stride", 4);
  node.declare_parameter<double>("frontend_lidar_noise_std", 0.05);
  node.declare_parameter<int>("frontend_map_max_age_frames", 0);
  node.declare_parameter<double>("frontend_map_l0_voxel_size", -1.0);
  node.declare_parameter<int>("frontend_max_correspondences", 0);
  node.declare_parameter<int>("frontend_min_l0_for_surfel", 3);
  // NOTE: frontend_enable_neighbor_search removed (not in v1.0 SurfelMapConfig)
  // NOTE: frontend_neighbor_max_plane_dist removed (not in v1.0 SurfelMapConfig)
  node.declare_parameter<double>("frontend_l0_ema_alpha_min", 0.0);
  node.declare_parameter<int>("frontend_l0_centroid_freeze_count", 0);
  node.declare_parameter<double>("frontend_ema_gate_radius", 0.0);
  node.declare_parameter<bool>("frontend_enable_degen_pvmap_override", false);
  node.declare_parameter<double>("frontend_degen_pvmap_cos_threshold", 0.5);
  node.declare_parameter<int>("frontend_degen_freeze_min_persist", 0);
  node.declare_parameter<double>("frontend_pvmap_sigma2_scale", 2.0);

  // Surfel Keyframe Gate (Task #133 Iter 3)
  node.declare_parameter<bool>("frontend_enable_surfel_keyframe_gate", false);
  node.declare_parameter<double>("frontend_surfel_kf_trans_thresh_m", 0.30);
  node.declare_parameter<double>("frontend_surfel_kf_rot_thresh_rad", 0.10);
  node.declare_parameter<int>("frontend_surfel_kf_warmup_frames", 20);

  // NOTE: RANSAC surfel params removed (not in v1.0 SurfelMapConfig)
  node.declare_parameter<int>("frontend_max_inner_iterations", 4);
  node.declare_parameter<double>("frontend_max_plane_distance", 0.0);

  // Degeneracy-aware IEKF
  node.declare_parameter<bool>("frontend_enable_degeneracy_detection", true);
  node.declare_parameter<double>("frontend_degeneracy_threshold", 50.0);
  node.declare_parameter<double>("frontend_degeneracy_ratio_threshold", 0.0);
  node.declare_parameter<double>("frontend_degeneracy_soft_floor", 0.0);
  node.declare_parameter<double>("frontend_map_degen_ratio_threshold", 0.01);
  // NOTE: frontend_degeneracy_beta removed (not in v1.0 IekfConfig)

  // ICDR: Information-theoretic Continuous Degeneracy Regularization
  node.declare_parameter<bool>("frontend_enable_icdr", false);
  node.declare_parameter<double>("frontend_icdr_rho_thresh", 0.3);
  node.declare_parameter<double>("frontend_icdr_tau", 0.05);
  node.declare_parameter<double>("frontend_icdr_w_min", 0.01);
  node.declare_parameter<bool>("frontend_enable_icdr_tip", true);
  node.declare_parameter<double>("frontend_icdr_tip_alpha", 0.3);
  node.declare_parameter<double>("frontend_icdr_tip_beta", 0.5);
  node.declare_parameter<double>("frontend_icdr_tip_d_decay", 2.0);

  // Covariance floor
  node.declare_parameter<double>("frontend_p_floor_rot", 1e-6);
  node.declare_parameter<double>("frontend_p_floor_pos", 1e-6);
  node.declare_parameter<double>("frontend_p_floor_vel", 1e-4);
  node.declare_parameter<double>("frontend_p_floor_bias", 1e-8);
  node.declare_parameter<double>("frontend_p_floor_grav", 1e-8);

  // IMU bias pseudo-observation
  node.declare_parameter<bool>("frontend_enable_bias_pseudo_obs", false);
  node.declare_parameter<double>("frontend_bias_bg_sigma", 0.01);
  node.declare_parameter<double>("frontend_bias_ba_sigma", 0.1);

  // Velocity pseudo-observation
  node.declare_parameter<bool>("frontend_enable_velocity_pseudo_obs", false);
  node.declare_parameter<double>("frontend_velocity_sigma", 1.0);

  // Gravity norm constraint
  node.declare_parameter<bool>("frontend_enable_gravity_norm_constraint", true);
  node.declare_parameter<double>("frontend_gravity_norm_sigma", 0.01);

  // NOTE: frontend_enable_iterative_undistortion removed (v1.0 uses enable_undistortion)

  // Degeneracy-adaptive EMA alpha
  node.declare_parameter<bool>("frontend_enable_degeneracy_adaptive_alpha", false);
  node.declare_parameter<double>("frontend_degeneracy_alpha_scale", 1.0);
  node.declare_parameter<bool>("frontend_enable_velocity_damping", true);

  // NOTE: genz dual metric params removed (not in v1.0 IekfConfig)

  // NOTE: frontend_corr_mode / frontend_l0_min_centroids removed (not in v1.0)
  // frontend_l0_planarity_threshold renamed to frontend_pvmap_planarity_threshold (loaded below)
  node.declare_parameter<double>("frontend_pvmap_voxel_size", 0.5);
  node.declare_parameter<int>("frontend_pvmap_max_points_per_voxel", 20);
  node.declare_parameter<int>("frontend_pvmap_k_neighbors", 5);
  node.declare_parameter<double>("frontend_pvmap_planarity_threshold", 0.15);
  node.declare_parameter<bool>("frontend_pvmap_enable_degen_freeze", false);
  node.declare_parameter<double>("frontend_pvmap_degen_freeze_cos_threshold", 0.7);
  node.declare_parameter<int>("frontend_pvmap_degen_freeze_max_frames", 200);

  // Debug timing
  node.declare_parameter<bool>("frontend_enable_debug_timing", false);

  // Point cloud time field
  node.declare_parameter<std::string>("point_time_field", "timestamp");
  node.declare_parameter<std::string>("point_time_unit", "sec");
  node.declare_parameter<std::string>("point_time_reference", "absolute");

  // LWO-specific noise parameters
  node.declare_parameter<double>("wheel_velocity_noise", 0.05);
  node.declare_parameter<double>("wheel_omega_noise", 0.05);
  // Wheel measurement update noise (separate from process noise)
  // Defaults to -1.0 meaning "use wheel_velocity_noise / wheel_omega_noise"
  node.declare_parameter<double>("wm_noise_vx", -1.0);
  node.declare_parameter<double>("wm_noise_omega", -1.0);
  node.declare_parameter<double>("wheel_scale_noise", 0.001);
  node.declare_parameter<double>("wheel_gyro_bias_noise", 0.001);
  node.declare_parameter<double>("ground_z_noise", 0.001);
  node.declare_parameter<double>("ground_rp_noise", 0.01);

  // Wheel propagator process noise
  node.declare_parameter<double>("wheel_sigma_rot", 0.05);
  node.declare_parameter<double>("wheel_sigma_pos", 0.01);
  node.declare_parameter<double>("wheel_sigma_vel", 0.05);

  // Bootstrap map commit (sparse ToF)
  node.declare_parameter<int>("bootstrap_frames", 30);

  // Low correspondence guard
  node.declare_parameter<int>("min_correspondences", 50);
  node.declare_parameter<double>("low_corr_heading_inflation", 0.001);
  node.declare_parameter<double>("p_yaw_floor", 0.0001);

  // Adaptive LiDAR noise by surfel planarity
  node.declare_parameter<double>("planarity_noise_scale", 0.0);

  // Degeneracy-aware selective update
  node.declare_parameter<bool>("enable_degeneracy_projection", false);
  node.declare_parameter<double>("degeneracy_eigenvalue_threshold", 10.0);
  node.declare_parameter<bool>("enable_soft_degeneracy", false);
  node.declare_parameter<double>("degeneracy_soft_power", 2.0);

  // Position correction bias estimation
  node.declare_parameter<bool>("enable_pos_bias_est", false);
  node.declare_parameter<double>("pos_bias_ema_alpha", 0.02);
  node.declare_parameter<int>("pos_bias_warmup_frames", 50);
  node.declare_parameter<double>("pos_bias_outlier_threshold", 0.01);
  node.declare_parameter<double>("pos_bias_correction_gain", 1.0);
  node.declare_parameter<bool>("pos_bias_enable_z", false);

  // Frozen Anchor Map (heading drift correction on return leg)
  node.declare_parameter<bool>("anchor_enable", false);
  node.declare_parameter<int>("anchor_build_frames", 30);
  node.declare_parameter<double>("anchor_proximity_radius", 2.0);
  node.declare_parameter<int>("anchor_min_frame", 100);
  node.declare_parameter<int>("anchor_min_overlap_corrs", 15);
  node.declare_parameter<double>("anchor_noise_std", 0.5);
  node.declare_parameter<double>("anchor_p_yaw_inflate", 0.01);
  node.declare_parameter<double>("anchor_max_residual_rms", 0.15);
  node.declare_parameter<double>("anchor_residual_threshold", 1.0);
  node.declare_parameter<double>("anchor_max_cumulative_yaw", 0.03);

  // Dynamic P_yaw inflation on return leg
  node.declare_parameter<bool>("anchor_dynamic_p_yaw", false);
  node.declare_parameter<double>("anchor_p_yaw_inflate_min", 0.001);
  node.declare_parameter<double>("anchor_p_yaw_inflate_scale", 1.0);

  // Anchor consistency gate
  node.declare_parameter<double>("anchor_consistency_alpha", 0.3);
  node.declare_parameter<double>("anchor_consistency_threshold", 0.003);

  // Post-IEKF anchor yaw-only correction
  node.declare_parameter<double>("anchor_yaw_gain", 0.5);
  node.declare_parameter<double>("anchor_yaw_max_correction", 0.0524);
  node.declare_parameter<double>("anchor_yaw_B_min", 1.0);
  node.declare_parameter<int>("anchor_yaw_min_corrs", 10);

  // Anchor-hybrid IEKF blend
  node.declare_parameter<bool>("anchor_iekf_blend_enable", false);
  node.declare_parameter<double>("anchor_iekf_blend_ratio", 0.5);

  // Debug logging
  node.declare_parameter<bool>("lwo_enable_debug_log", false);

  // Wheel measurement in IEKF update
  node.declare_parameter<bool>("enable_wheel_measurement", false);

  // No-harm gate / trust blending / quality-aware correspondences
  node.declare_parameter<bool>("enable_no_harm_gate", true);
  node.declare_parameter<bool>("enable_trust_blending", true);
  node.declare_parameter<bool>("enable_quality_aware_corr", true);
  node.declare_parameter<double>("no_harm_lambda_wheel", 2.0);
  node.declare_parameter<double>("no_harm_lambda_ground", 1.0);
  node.declare_parameter<double>("no_harm_lambda_prior", 4.0);
  node.declare_parameter<double>("no_harm_accept_margin", 0.0);
  node.declare_parameter<double>("trust_min_lidar_rms_drop_ratio", 0.90);
  node.declare_parameter<double>("trust_max_wheel_cost_increase", 1.5);
  node.declare_parameter<double>("trust_max_ground_cost_increase", 1.5);
  node.declare_parameter<double>("trust_wheel_cost_floor", 1e-3);
  node.declare_parameter<double>("trust_ground_cost_floor", 1.0);
  node.declare_parameter<double>("trust_wheel_safe_abs_increase", 1e-3);
  node.declare_parameter<double>("trust_ground_safe_abs_increase", 10.0);
  node.declare_parameter<double>("trust_min_score_for_accept", 0.62);
  node.declare_parameter<double>("trust_min_score_for_commit", 0.65);
  node.declare_parameter<double>("trust_min_score_for_blend", 0.35);
  node.declare_parameter<double>("trust_blend_alpha_min", 0.0);
  node.declare_parameter<double>("trust_blend_alpha_max", 0.8);
  node.declare_parameter<int>("accept_window_size", 5);
  node.declare_parameter<int>("accept_min_consistent_frames", 3);
  node.declare_parameter<double>("accept_max_mean_rms_ratio", 0.98);
  node.declare_parameter<double>("accept_max_mean_pose_delta", 0.03);
  node.declare_parameter<double>("accept_max_mean_yaw_delta", 0.02);
  node.declare_parameter<int>("low_corr_blend_margin", 2);
  node.declare_parameter<double>("low_corr_blend_alpha_scale", 0.25);
  node.declare_parameter<double>("inconsistent_blend_alpha_scale", 0.5);
  node.declare_parameter<double>("corr_max_plane_distance", 0.20);
  node.declare_parameter<int>("corr_max_per_l1", 2);
  node.declare_parameter<int>("corr_min_centroids", 4);
  node.declare_parameter<double>("corr_hybrid_planarity_threshold", 0.12);
  node.declare_parameter<bool>("map_commit_require_accept", true);
  node.declare_parameter<bool>("map_commit_require_rms_drop", true);
  node.declare_parameter<double>("map_commit_max_lidar_rms", 0.10);
  node.declare_parameter<double>("map_commit_max_pose_delta", 0.05);
  node.declare_parameter<double>("map_commit_max_yaw_delta", 0.03);

  // Pose graph backend solver
  node.declare_parameter<double>("lc_pgo_huber_delta", 5.0);
  node.declare_parameter<double>("lc_pgo_numeric_diff_eps", 1e-6);
  node.declare_parameter<bool>("lc_pgo_use_loop_huber", false);

  // IEKF correction magnitude clamps
  node.declare_parameter<double>("iekf_max_pos_correction", 0.05);    // per inner iter
  node.declare_parameter<double>("iekf_max_rot_correction", 0.035);   // per inner iter
  node.declare_parameter<double>("iekf_max_total_pos_correction", 0.10);  // total per frame
  node.declare_parameter<double>("iekf_max_total_rot_correction", 0.087); // total per frame

  // Velocity-adaptive LiDAR noise
  node.declare_parameter<double>("vel_noise_scale", 0.0);
  node.declare_parameter<double>("vel_noise_min_speed", 0.1);

  // Resource profiling
  node.declare_parameter<bool>("check_usage", false);

  // Online extrinsic calibration
  node.declare_parameter<bool>("enable_ext_calibration", false);
  node.declare_parameter<double>("ext_obs_min_omega", 0.1);
  node.declare_parameter<int>("ext_obs_min_correspondences", 100);
  node.declare_parameter<double>("ext_convergence_threshold", 0.0001);
  node.declare_parameter<int>("ext_convergence_frames", 50);
  node.declare_parameter<double>("ext_max_delta_yaw", 0.01);
  node.declare_parameter<double>("ext_max_delta_xy", 0.01);

  // CSV logging paths
  node.declare_parameter<std::string>("trajectory_csv_path", "/root/tofslam_traj.csv");
  node.declare_parameter<std::string>("dump_path", "/workspace/dump");

  // ── Occupancy Grid Map (Nav2 Integration) ──────────────────────────
  node.declare_parameter<bool>("ogm_enable", false);
  node.declare_parameter<double>("ogm_resolution", 0.05);
  node.declare_parameter<double>("ogm_local_range", 5.0);
  node.declare_parameter<double>("ogm_log_odds_hit", 0.85);
  node.declare_parameter<double>("ogm_log_odds_miss", -0.4);
  node.declare_parameter<double>("ogm_log_odds_max", 3.5);
  node.declare_parameter<double>("ogm_log_odds_min", -2.0);
  node.declare_parameter<double>("ogm_height_min", 0.1);
  node.declare_parameter<double>("ogm_height_max", 1.2);
  node.declare_parameter<int>("ogm_endpoint_dilation_radius", 1);
  node.declare_parameter<int>("ogm_global_dilation_radius", 1);
  node.declare_parameter<int>("ogm_publish_stride", 5);
  node.declare_parameter<std::string>("ogm_topic", "/map");

  // ── Loop Closure Backend ──────────────────────────────────────────
  node.declare_parameter<bool>("enable_loop_closure", false);
  node.declare_parameter<int>("lc_submap_keyframe_count", 30);
  node.declare_parameter<double>("lc_submap_voxel_size", 0.1);
  node.declare_parameter<double>("lc_loop_search_radius", 5.0);
  node.declare_parameter<int>("lc_loop_min_keyframe_gap", 30);
  node.declare_parameter<int>("lc_loop_min_submap_gap", 2);
  node.declare_parameter<int>("lc_max_candidates_per_query", 3);
  node.declare_parameter<int>("lc_gicp_max_iterations", 50);
  node.declare_parameter<double>("lc_gicp_max_correspondence_distance", 1.0);
  node.declare_parameter<double>("lc_gicp_fitness_threshold", 0.5);
  node.declare_parameter<double>("lc_gicp_min_fitness_threshold", 0.01);
  node.declare_parameter<int>("lc_gicp_min_correspondences", 50);
  node.declare_parameter<double>("lc_gicp_max_translation", 2.0);
  node.declare_parameter<double>("lc_gicp_max_rotation", 0.5);
  node.declare_parameter<double>("lc_overlap_max_distance", 0.35);
  node.declare_parameter<int>("lc_min_initial_inliers", 40);
  node.declare_parameter<double>("lc_min_initial_overlap_ratio", 0.10);
  node.declare_parameter<double>("lc_min_initial_target_overlap_ratio", 0.15);
  node.declare_parameter<double>("lc_min_final_overlap_ratio", 0.20);
  node.declare_parameter<double>("lc_min_final_target_overlap_ratio", 0.05);
  node.declare_parameter<double>("lc_min_match_score", 2.0);
  node.declare_parameter<double>("lc_heading_max_delta", 2.09);
  node.declare_parameter<double>("lc_loop_information_weight", 100.0);
  node.declare_parameter<int>("lc_pgo_max_iterations", 20);
  node.declare_parameter<bool>("lc_project_to_2d", true);
  node.declare_parameter<double>("lc_height_filter_min", -0.1);
  node.declare_parameter<double>("lc_height_filter_max", 1.5);
  node.declare_parameter<bool>("lc_enable_debug_log", true);
  node.declare_parameter<double>("lc_keyframe_trans_thresh", 0.3);
  node.declare_parameter<double>("lc_keyframe_rot_thresh", 0.15);
}

// ---------------------------------------------------------------------------
// build_extrinsic_rot — RPY (ZYX) takes priority; falls back to 3x3 matrix
// ---------------------------------------------------------------------------
Eigen::Matrix3f build_extrinsic_rot(const rclcpp::Node& node) {
  const double ext_roll  = node.get_parameter("extrinsic_roll").as_double();
  const double ext_pitch = node.get_parameter("extrinsic_pitch").as_double();
  const double ext_yaw   = node.get_parameter("extrinsic_yaw").as_double();
  const bool rpy_nonzero = (std::abs(ext_roll)  > 1e-9 ||
                            std::abs(ext_pitch) > 1e-9 ||
                            std::abs(ext_yaw)   > 1e-9);
  const auto ext_rot_vec = node.get_parameter("extrinsic_rotation").as_double_array();
  const bool matrix_provided = (ext_rot_vec.size() == 9);

  Eigen::Matrix3f ext_rot = Eigen::Matrix3f::Identity();
  if (rpy_nonzero || !matrix_provided) {
    // ZYX: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    const float cr = std::cos(static_cast<float>(ext_roll));
    const float sr = std::sin(static_cast<float>(ext_roll));
    const float cp = std::cos(static_cast<float>(ext_pitch));
    const float sp = std::sin(static_cast<float>(ext_pitch));
    const float cy = std::cos(static_cast<float>(ext_yaw));
    const float sy = std::sin(static_cast<float>(ext_yaw));
    Eigen::Matrix3f Rz, Ry, Rx;
    Rz << cy, -sy, 0, sy, cy, 0, 0, 0, 1;
    Ry << cp, 0, sp, 0, 1, 0, -sp, 0, cp;
    Rx << 1, 0, 0, 0, cr, -sr, 0, sr, cr;
    ext_rot = Rz * Ry * Rx;
    RCLCPP_INFO(node.get_logger(),
      "Extrinsic RPY: roll=%.4f pitch=%.4f yaw=%.4f rad",
      ext_roll, ext_pitch, ext_yaw);
  } else {
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        ext_rot(i, j) = static_cast<float>(ext_rot_vec[i * 3 + j]);
    RCLCPP_INFO(node.get_logger(), "Extrinsic rotation from 3x3 matrix param");
  }
  RCLCPP_INFO(node.get_logger(),
    "Extrinsic R:\n  [%.3f %.3f %.3f]\n  [%.3f %.3f %.3f]\n  [%.3f %.3f %.3f]",
    ext_rot(0,0),ext_rot(0,1),ext_rot(0,2),
    ext_rot(1,0),ext_rot(1,1),ext_rot(1,2),
    ext_rot(2,0),ext_rot(2,1),ext_rot(2,2));
  return ext_rot;
}

// ---------------------------------------------------------------------------
// build_lio_config — build LioEstimator config from parameters
// ---------------------------------------------------------------------------
LioSetupResult build_lio_config(const rclcpp::Node& node,
                                const Eigen::Matrix3f& ext_rot,
                                double ext_x, double ext_y, double ext_z) {
  const double gyr_noise  = node.get_parameter("imu_gyro_noise").as_double();
  const double acc_noise  = node.get_parameter("imu_accel_noise").as_double();
  const double bgyr_noise = node.get_parameter("imu_gyro_bias_noise").as_double();
  const double bacc_noise = node.get_parameter("imu_accel_bias_noise").as_double();
  const double gravity_z  = node.get_parameter("imu_gravity_z").as_double();

  int init_samples = node.get_parameter("imu_init_samples").as_int();
  const int frontend_init = node.get_parameter("frontend_init_imu_samples").as_int();
  if (frontend_init > 0) init_samples = frontend_init;

  // YAML values are VARIANCES (sig^2); noise_std = sqrt(variance).
  core::LioEstimator::Config cfg;
  cfg.gyro_noise_std      = static_cast<float>(std::sqrt(gyr_noise));
  cfg.acc_noise_std       = static_cast<float>(std::sqrt(acc_noise));
  cfg.gyro_bias_noise_std = static_cast<float>(std::sqrt(bgyr_noise));
  cfg.acc_bias_noise_std  = static_cast<float>(std::sqrt(bacc_noise));

  RCLCPP_INFO(node.get_logger(),
    "Q noise std: gyr=%.6f acc=%.6f bgyr=%.6f bacc=%.6f grav=%.6f",
    cfg.gyro_noise_std, cfg.acc_noise_std,
    cfg.gyro_bias_noise_std, cfg.acc_bias_noise_std,
    cfg.gravity_noise_std);

  cfg.stride          = node.get_parameter("frontend_stride").as_int();
  cfg.voxel_leaf_size = static_cast<float>(
      node.get_parameter("frontend_voxel_size").as_double());
  cfg.min_range = static_cast<float>(
      node.get_parameter("frontend_min_distance").as_double());
  cfg.max_range = static_cast<float>(
      node.get_parameter("frontend_max_distance").as_double());

  cfg.iekf.max_outer_iters       = node.get_parameter("frontend_max_iterations").as_int();
  cfg.iekf.max_inner_iters       = node.get_parameter("frontend_max_inner_iterations").as_int();
  cfg.iekf.convergence_threshold = static_cast<float>(
      node.get_parameter("frontend_convergence_threshold").as_double());
  cfg.iekf.lidar_noise_std = static_cast<float>(
      node.get_parameter("frontend_lidar_noise_std").as_double());

  // Degeneracy-aware IEKF
  cfg.iekf.enable_degeneracy_detection =
      node.get_parameter("frontend_enable_degeneracy_detection").as_bool();
  cfg.iekf.degeneracy_threshold = static_cast<float>(
      node.get_parameter("frontend_degeneracy_threshold").as_double());
  cfg.iekf.degeneracy_ratio_threshold = static_cast<float>(
      node.get_parameter("frontend_degeneracy_ratio_threshold").as_double());
  cfg.iekf.degeneracy_soft_floor = static_cast<float>(
      node.get_parameter("frontend_degeneracy_soft_floor").as_double());
  cfg.iekf.map_degen_ratio_threshold = static_cast<float>(
      node.get_parameter("frontend_map_degen_ratio_threshold").as_double());
  // NOTE: degeneracy_beta removed (not in v1.0 IekfConfig)

  // ICDR
  cfg.iekf.enable_icdr =
      node.get_parameter("frontend_enable_icdr").as_bool();
  cfg.iekf.icdr_rho_thresh = static_cast<float>(
      node.get_parameter("frontend_icdr_rho_thresh").as_double());
  cfg.iekf.icdr_tau = static_cast<float>(
      node.get_parameter("frontend_icdr_tau").as_double());
  cfg.iekf.icdr_w_min = static_cast<float>(
      node.get_parameter("frontend_icdr_w_min").as_double());
  cfg.iekf.enable_icdr_tip =
      node.get_parameter("frontend_enable_icdr_tip").as_bool();
  cfg.iekf.icdr_tip_alpha = static_cast<float>(
      node.get_parameter("frontend_icdr_tip_alpha").as_double());
  cfg.iekf.icdr_tip_beta = static_cast<float>(
      node.get_parameter("frontend_icdr_tip_beta").as_double());
  cfg.iekf.icdr_tip_d_decay = static_cast<float>(
      node.get_parameter("frontend_icdr_tip_d_decay").as_double());

  // Covariance floor
  cfg.iekf.p_floor_rot  = static_cast<float>(node.get_parameter("frontend_p_floor_rot").as_double());
  cfg.iekf.p_floor_pos  = static_cast<float>(node.get_parameter("frontend_p_floor_pos").as_double());
  cfg.iekf.p_floor_vel  = static_cast<float>(node.get_parameter("frontend_p_floor_vel").as_double());
  cfg.iekf.p_floor_bias = static_cast<float>(node.get_parameter("frontend_p_floor_bias").as_double());
  cfg.iekf.p_floor_grav = static_cast<float>(node.get_parameter("frontend_p_floor_grav").as_double());

  // IMU bias pseudo-observation
  cfg.iekf.enable_bias_pseudo_obs =
      node.get_parameter("frontend_enable_bias_pseudo_obs").as_bool();
  cfg.iekf.bias_bg_sigma = static_cast<float>(
      node.get_parameter("frontend_bias_bg_sigma").as_double());
  cfg.iekf.bias_ba_sigma = static_cast<float>(
      node.get_parameter("frontend_bias_ba_sigma").as_double());

  // Velocity pseudo-observation
  cfg.iekf.enable_velocity_pseudo_obs =
      node.get_parameter("frontend_enable_velocity_pseudo_obs").as_bool();
  cfg.iekf.velocity_sigma = static_cast<float>(
      node.get_parameter("frontend_velocity_sigma").as_double());

  // Gravity norm constraint
  cfg.iekf.enable_gravity_norm_constraint =
      node.get_parameter("frontend_enable_gravity_norm_constraint").as_bool();
  cfg.iekf.gravity_norm_sigma = static_cast<float>(
      node.get_parameter("frontend_gravity_norm_sigma").as_double());

  // NOTE: genz dual metric removed (not in v1.0 IekfConfig)

  // Correspondence finder mode
  // NOTE: corr_mode / CorrMode enum removed (not in v1.0 LioEstimator::Config)
  // NOTE: l0_min_centroids removed (not in v1.0 LioEstimator::Config)
  cfg.point_voxel_map.voxel_size = static_cast<float>(
      node.get_parameter("frontend_pvmap_voxel_size").as_double());
  cfg.point_voxel_map.max_points_per_voxel =
      node.get_parameter("frontend_pvmap_max_points_per_voxel").as_int();
  cfg.point_voxel_map.max_distance = cfg.max_range;
  cfg.pvmap_k_neighbors = node.get_parameter("frontend_pvmap_k_neighbors").as_int();
  cfg.pvmap_planarity_threshold = static_cast<float>(
      node.get_parameter("frontend_pvmap_planarity_threshold").as_double());
  cfg.point_voxel_map.enable_degen_freeze =
      node.get_parameter("frontend_pvmap_enable_degen_freeze").as_bool();
  cfg.point_voxel_map.degen_freeze_cos_threshold = static_cast<float>(
      node.get_parameter("frontend_pvmap_degen_freeze_cos_threshold").as_double());
  cfg.point_voxel_map.degen_freeze_max_frames =
      node.get_parameter("frontend_pvmap_degen_freeze_max_frames").as_int();

  // Debug timing
  cfg.enable_debug_timing = node.get_parameter("frontend_enable_debug_timing").as_bool();
  cfg.iekf.enable_debug_timing = cfg.enable_debug_timing;

  RCLCPP_INFO(node.get_logger(),
    "degeneracy=%s (thresh=%.1f) P_floor=%.1e debug_timing=%s",
    cfg.iekf.enable_degeneracy_detection ? "ON" : "OFF",
    cfg.iekf.degeneracy_threshold,
    cfg.iekf.p_floor_pos,
    cfg.enable_debug_timing ? "ON" : "OFF");

  // Surfel map
  const double map_l0 = node.get_parameter("frontend_map_l0_voxel_size").as_double();
  cfg.surfel_map.l0_voxel_size = (map_l0 > 0.0)
      ? static_cast<float>(map_l0) : cfg.voxel_leaf_size;
  cfg.surfel_map.l1_hierarchy_factor =
      node.get_parameter("frontend_voxel_hierarchy_factor").as_int();
  cfg.surfel_map.min_l0_for_surfel =
      node.get_parameter("frontend_min_l0_for_surfel").as_int();
  // NOTE: enable_neighbor_search / neighbor_max_plane_dist removed (not in v1.0 SurfelMapConfig)
  cfg.surfel_map.max_distance        = cfg.max_range;
  cfg.surfel_map.planarity_threshold = static_cast<float>(
      node.get_parameter("frontend_map_planarity_threshold").as_double());
  cfg.surfel_map.distance_multiplier = static_cast<float>(
      node.get_parameter("frontend_map_box_multiplier").as_double());
  cfg.surfel_map.l0_ema_alpha_min = static_cast<float>(
      node.get_parameter("frontend_l0_ema_alpha_min").as_double());
  cfg.surfel_map.l0_centroid_freeze_count =
      node.get_parameter("frontend_l0_centroid_freeze_count").as_int();
  cfg.surfel_map.ema_gate_radius = static_cast<float>(
      node.get_parameter("frontend_ema_gate_radius").as_double());
  cfg.enable_degen_pvmap_override =
      node.get_parameter("frontend_enable_degen_pvmap_override").as_bool();
  cfg.degen_pvmap_cos_threshold = static_cast<float>(
      node.get_parameter("frontend_degen_pvmap_cos_threshold").as_double());
  cfg.degen_freeze_min_persist =
      node.get_parameter("frontend_degen_freeze_min_persist").as_int();
  cfg.pvmap_sigma2_scale = static_cast<float>(
      node.get_parameter("frontend_pvmap_sigma2_scale").as_double());

  // Surfel Keyframe Gate (Task #133 Iter 3)
  cfg.enable_surfel_keyframe_gate =
      node.get_parameter("frontend_enable_surfel_keyframe_gate").as_bool();
  cfg.surfel_kf_trans_thresh_m =
      node.get_parameter("frontend_surfel_kf_trans_thresh_m").as_double();
  cfg.surfel_kf_rot_thresh_rad =
      node.get_parameter("frontend_surfel_kf_rot_thresh_rad").as_double();
  cfg.surfel_kf_warmup_frames =
      node.get_parameter("frontend_surfel_kf_warmup_frames").as_int();

  // NOTE: RANSAC surfel params removed (not in v1.0 SurfelMapConfig)

  cfg.max_correspondences = node.get_parameter("frontend_max_correspondences").as_int();
  cfg.max_plane_distance  = static_cast<float>(
      node.get_parameter("frontend_max_plane_distance").as_double());

  cfg.enable_undistortion = node.get_parameter("frontend_enable_undistortion").as_bool();
  cfg.scan_duration = static_cast<float>(
      node.get_parameter("frontend_scan_duration").as_double());
  // NOTE: enable_iterative_undistortion removed (v1.0 uses enable_undistortion)
  cfg.enable_degeneracy_adaptive_alpha =
      node.get_parameter("frontend_enable_degeneracy_adaptive_alpha").as_bool();
  cfg.degeneracy_alpha_scale = static_cast<float>(
      node.get_parameter("frontend_degeneracy_alpha_scale").as_double());
  cfg.enable_velocity_damping =
      node.get_parameter("frontend_enable_velocity_damping").as_bool();

  cfg.T_body_lidar = core::Se3(
      ext_rot,
      Eigen::Vector3f(static_cast<float>(ext_x),
                      static_cast<float>(ext_y),
                      static_cast<float>(ext_z)));

  // ImuAdapter config
  ros_adapter::ImuAdapter::Config imu_cfg;
  imu_cfg.init_sample_count = init_samples;
  imu_cfg.gravity_prior     = Eigen::Vector3f(0.0f, 0.0f,
                                              static_cast<float>(gravity_z));

  return LioSetupResult{cfg, imu_cfg};
}

// ---------------------------------------------------------------------------
// build_lwo_config — build LwoEstimator config from parameters
// ---------------------------------------------------------------------------
lwo::LwoEstimator::Config build_lwo_config(const rclcpp::Node& node,
                                           const Eigen::Matrix3f& ext_rot,
                                           double ext_x, double ext_y,
                                           double ext_z) {
  lwo::LwoEstimator::Config cfg;

  cfg.stride = node.get_parameter("frontend_stride").as_int();
  cfg.voxel_leaf_size = static_cast<float>(
      node.get_parameter("frontend_voxel_size").as_double());
  cfg.min_range = static_cast<float>(
      node.get_parameter("frontend_min_distance").as_double());
  cfg.max_range = static_cast<float>(
      node.get_parameter("frontend_max_distance").as_double());
  cfg.iekf.max_inner_iters = node.get_parameter("frontend_max_iterations").as_int();
  cfg.iekf.convergence_threshold = static_cast<float>(
      node.get_parameter("frontend_convergence_threshold").as_double());
  cfg.iekf.lidar_noise_std = static_cast<float>(
      node.get_parameter("frontend_lidar_noise_std").as_double());

  // Surfel map
  cfg.surfel_map.l0_voxel_size = cfg.voxel_leaf_size;
  cfg.surfel_map.l1_hierarchy_factor =
      node.get_parameter("frontend_voxel_hierarchy_factor").as_int();
  cfg.surfel_map.max_distance = cfg.max_range;
  cfg.surfel_map.planarity_threshold = static_cast<float>(
      node.get_parameter("frontend_map_planarity_threshold").as_double());
  cfg.surfel_map.distance_multiplier = static_cast<float>(
      node.get_parameter("frontend_map_box_multiplier").as_double());
  // Note: v1.0 SurfelMap does not have max_l0_age_frames.
  // cfg.surfel_map.max_l0_age_frames =
  //     node.get_parameter("frontend_map_max_age_frames").as_int();

  cfg.enable_undistortion = node.get_parameter("frontend_enable_undistortion").as_bool();
  cfg.scan_duration = static_cast<float>(
      node.get_parameter("frontend_scan_duration").as_double());
  cfg.T_body_lidar = core::Se3(
      ext_rot,
      Eigen::Vector3f(static_cast<float>(ext_x),
                      static_cast<float>(ext_y),
                      static_cast<float>(ext_z)));

  // Wheel / ground noise
  {
    const double proc_vx    = node.get_parameter("wheel_velocity_noise").as_double();
    const double proc_omega = node.get_parameter("wheel_omega_noise").as_double();
    const double wm_vx      = node.get_parameter("wm_noise_vx").as_double();
    const double wm_omega   = node.get_parameter("wm_noise_omega").as_double();
    // Measurement noise: use dedicated param if > 0, else fall back to process noise
    cfg.wheel_measurement.noise_vx = static_cast<float>(
        wm_vx > 0.0 ? wm_vx : proc_vx);
    cfg.wheel_measurement.noise_omega_z = static_cast<float>(
        wm_omega > 0.0 ? wm_omega : proc_omega);
  }
  cfg.wheel_propagator.sigma_scale = static_cast<float>(
      node.get_parameter("wheel_scale_noise").as_double());
  cfg.wheel_propagator.sigma_bias = static_cast<float>(
      node.get_parameter("wheel_gyro_bias_noise").as_double());
  cfg.wheel_propagator.sigma_rot = static_cast<float>(
      node.get_parameter("wheel_sigma_rot").as_double());
  cfg.wheel_propagator.sigma_pos = static_cast<float>(
      node.get_parameter("wheel_sigma_pos").as_double());
  cfg.wheel_propagator.sigma_vel = static_cast<float>(
      node.get_parameter("wheel_sigma_vel").as_double());
  cfg.ground_constraint.noise_z = static_cast<float>(
      node.get_parameter("ground_z_noise").as_double());
  cfg.ground_constraint.noise_roll_pitch = static_cast<float>(
      node.get_parameter("ground_rp_noise").as_double());

  // Bootstrap map commit (sparse ToF)
  cfg.bootstrap_frames = node.get_parameter("bootstrap_frames").as_int();

  // Low correspondence guard
  cfg.min_correspondences = node.get_parameter("min_correspondences").as_int();
  cfg.low_corr_heading_inflation = static_cast<float>(
      node.get_parameter("low_corr_heading_inflation").as_double());
  cfg.iekf.p_yaw_floor = static_cast<float>(
      node.get_parameter("p_yaw_floor").as_double());

  // Adaptive LiDAR noise by planarity
  cfg.iekf.planarity_noise_scale = static_cast<float>(
      node.get_parameter("planarity_noise_scale").as_double());

  // Degeneracy-aware selective update
  cfg.iekf.enable_degeneracy_projection =
      node.get_parameter("enable_degeneracy_projection").as_bool();
  cfg.iekf.degeneracy_eigenvalue_threshold = static_cast<float>(
      node.get_parameter("degeneracy_eigenvalue_threshold").as_double());
  cfg.iekf.enable_soft_degeneracy =
      node.get_parameter("enable_soft_degeneracy").as_bool();
  cfg.iekf.degeneracy_soft_power = static_cast<float>(
      node.get_parameter("degeneracy_soft_power").as_double());

  // Position correction bias estimation
  cfg.enable_pos_bias_est = node.get_parameter("enable_pos_bias_est").as_bool();
  cfg.pos_bias_ema_alpha = static_cast<float>(
      node.get_parameter("pos_bias_ema_alpha").as_double());
  cfg.pos_bias_warmup_frames = node.get_parameter("pos_bias_warmup_frames").as_int();
  cfg.pos_bias_outlier_threshold = static_cast<float>(
      node.get_parameter("pos_bias_outlier_threshold").as_double());
  cfg.pos_bias_correction_gain = static_cast<float>(
      node.get_parameter("pos_bias_correction_gain").as_double());
  cfg.pos_bias_enable_z = node.get_parameter("pos_bias_enable_z").as_bool();

  cfg.enable_debug_log = node.get_parameter("lwo_enable_debug_log").as_bool();

  // Wheel measurement in IEKF update
  cfg.enable_wheel_measurement =
      node.get_parameter("enable_wheel_measurement").as_bool();

  cfg.enable_no_harm_gate =
      node.get_parameter("enable_no_harm_gate").as_bool();
  cfg.enable_trust_blending =
      node.get_parameter("enable_trust_blending").as_bool();
  cfg.enable_quality_aware_corr =
      node.get_parameter("enable_quality_aware_corr").as_bool();
  cfg.no_harm_lambda_wheel = static_cast<float>(
      node.get_parameter("no_harm_lambda_wheel").as_double());
  cfg.no_harm_lambda_ground = static_cast<float>(
      node.get_parameter("no_harm_lambda_ground").as_double());
  cfg.no_harm_lambda_prior = static_cast<float>(
      node.get_parameter("no_harm_lambda_prior").as_double());
  cfg.no_harm_accept_margin = static_cast<float>(
      node.get_parameter("no_harm_accept_margin").as_double());
  cfg.trust_min_lidar_rms_drop_ratio = static_cast<float>(
      node.get_parameter("trust_min_lidar_rms_drop_ratio").as_double());
  cfg.trust_max_wheel_cost_increase = static_cast<float>(
      node.get_parameter("trust_max_wheel_cost_increase").as_double());
  cfg.trust_max_ground_cost_increase = static_cast<float>(
      node.get_parameter("trust_max_ground_cost_increase").as_double());
  cfg.trust_wheel_cost_floor = static_cast<float>(
      node.get_parameter("trust_wheel_cost_floor").as_double());
  cfg.trust_ground_cost_floor = static_cast<float>(
      node.get_parameter("trust_ground_cost_floor").as_double());
  cfg.trust_wheel_safe_abs_increase = static_cast<float>(
      node.get_parameter("trust_wheel_safe_abs_increase").as_double());
  cfg.trust_ground_safe_abs_increase = static_cast<float>(
      node.get_parameter("trust_ground_safe_abs_increase").as_double());
  cfg.trust_min_score_for_accept = static_cast<float>(
      node.get_parameter("trust_min_score_for_accept").as_double());
  cfg.trust_min_score_for_commit = static_cast<float>(
      node.get_parameter("trust_min_score_for_commit").as_double());
  cfg.trust_min_score_for_blend = static_cast<float>(
      node.get_parameter("trust_min_score_for_blend").as_double());
  cfg.trust_blend_alpha_min = static_cast<float>(
      node.get_parameter("trust_blend_alpha_min").as_double());
  cfg.trust_blend_alpha_max = static_cast<float>(
      node.get_parameter("trust_blend_alpha_max").as_double());
  cfg.accept_window_size = node.get_parameter("accept_window_size").as_int();
  cfg.accept_min_consistent_frames =
      node.get_parameter("accept_min_consistent_frames").as_int();
  cfg.accept_max_mean_rms_ratio = static_cast<float>(
      node.get_parameter("accept_max_mean_rms_ratio").as_double());
  cfg.accept_max_mean_pose_delta = static_cast<float>(
      node.get_parameter("accept_max_mean_pose_delta").as_double());
  cfg.accept_max_mean_yaw_delta = static_cast<float>(
      node.get_parameter("accept_max_mean_yaw_delta").as_double());
  cfg.low_corr_blend_margin =
      node.get_parameter("low_corr_blend_margin").as_int();
  cfg.low_corr_blend_alpha_scale = static_cast<float>(
      node.get_parameter("low_corr_blend_alpha_scale").as_double());
  cfg.inconsistent_blend_alpha_scale = static_cast<float>(
      node.get_parameter("inconsistent_blend_alpha_scale").as_double());
  cfg.corr_max_plane_distance = static_cast<float>(
      node.get_parameter("corr_max_plane_distance").as_double());
  cfg.corr_max_per_l1 = node.get_parameter("corr_max_per_l1").as_int();
  cfg.corr_min_centroids = node.get_parameter("corr_min_centroids").as_int();
  cfg.corr_hybrid_planarity_threshold = static_cast<float>(
      node.get_parameter("corr_hybrid_planarity_threshold").as_double());
  cfg.map_commit_require_accept =
      node.get_parameter("map_commit_require_accept").as_bool();
  cfg.map_commit_require_rms_drop =
      node.get_parameter("map_commit_require_rms_drop").as_bool();
  cfg.map_commit_max_lidar_rms = static_cast<float>(
      node.get_parameter("map_commit_max_lidar_rms").as_double());
  cfg.map_commit_max_pose_delta = static_cast<float>(
      node.get_parameter("map_commit_max_pose_delta").as_double());
  cfg.map_commit_max_yaw_delta = static_cast<float>(
      node.get_parameter("map_commit_max_yaw_delta").as_double());

  // IEKF correction magnitude clamps
  cfg.iekf.max_pos_correction = static_cast<float>(
      node.get_parameter("iekf_max_pos_correction").as_double());
  cfg.iekf.max_rot_correction = static_cast<float>(
      node.get_parameter("iekf_max_rot_correction").as_double());
  cfg.iekf_max_total_pos_correction = static_cast<float>(
      node.get_parameter("iekf_max_total_pos_correction").as_double());
  cfg.iekf_max_total_rot_correction = static_cast<float>(
      node.get_parameter("iekf_max_total_rot_correction").as_double());

  // Resource profiling
  cfg.check_usage = node.get_parameter("check_usage").as_bool();
  cfg.iekf.check_usage = cfg.check_usage;

  // Frozen Anchor Map
  cfg.anchor_enable = node.get_parameter("anchor_enable").as_bool();
  cfg.anchor_build_frames = node.get_parameter("anchor_build_frames").as_int();
  cfg.anchor_proximity_radius = static_cast<float>(
      node.get_parameter("anchor_proximity_radius").as_double());
  cfg.anchor_min_frame = node.get_parameter("anchor_min_frame").as_int();
  cfg.anchor_min_overlap_corrs = node.get_parameter("anchor_min_overlap_corrs").as_int();
  cfg.anchor_noise_std = static_cast<float>(
      node.get_parameter("anchor_noise_std").as_double());
  cfg.anchor_p_yaw_inflate = static_cast<float>(
      node.get_parameter("anchor_p_yaw_inflate").as_double());
  cfg.anchor_max_residual_rms = static_cast<float>(
      node.get_parameter("anchor_max_residual_rms").as_double());
  cfg.anchor_residual_threshold = static_cast<float>(
      node.get_parameter("anchor_residual_threshold").as_double());
  cfg.anchor_max_cumulative_yaw = static_cast<float>(
      node.get_parameter("anchor_max_cumulative_yaw").as_double());

  // Dynamic P_yaw inflation
  cfg.anchor_dynamic_p_yaw = node.get_parameter("anchor_dynamic_p_yaw").as_bool();
  cfg.anchor_p_yaw_inflate_min = static_cast<float>(
      node.get_parameter("anchor_p_yaw_inflate_min").as_double());
  cfg.anchor_p_yaw_inflate_scale = static_cast<float>(
      node.get_parameter("anchor_p_yaw_inflate_scale").as_double());

  // Anchor consistency gate
  cfg.anchor_consistency_alpha = static_cast<float>(
      node.get_parameter("anchor_consistency_alpha").as_double());
  cfg.anchor_consistency_threshold = static_cast<float>(
      node.get_parameter("anchor_consistency_threshold").as_double());

  // Post-IEKF anchor yaw-only correction
  cfg.anchor_yaw_gain = static_cast<float>(
      node.get_parameter("anchor_yaw_gain").as_double());
  cfg.anchor_yaw_max_correction = static_cast<float>(
      node.get_parameter("anchor_yaw_max_correction").as_double());
  cfg.anchor_yaw_B_min = static_cast<float>(
      node.get_parameter("anchor_yaw_B_min").as_double());
  cfg.anchor_yaw_min_corrs = node.get_parameter("anchor_yaw_min_corrs").as_int();

  // Anchor-hybrid IEKF blend
  cfg.anchor_iekf_blend_enable = node.get_parameter("anchor_iekf_blend_enable").as_bool();
  cfg.anchor_iekf_blend_ratio = static_cast<float>(
      node.get_parameter("anchor_iekf_blend_ratio").as_double());

  // Velocity-adaptive LiDAR noise
  cfg.vel_noise_scale = static_cast<float>(
      node.get_parameter("vel_noise_scale").as_double());
  cfg.vel_noise_min_speed = static_cast<float>(
      node.get_parameter("vel_noise_min_speed").as_double());

  // Online extrinsic calibration
  cfg.iekf.enable_ext_calibration =
      node.get_parameter("enable_ext_calibration").as_bool();
  cfg.iekf.ext_obs_min_omega = static_cast<float>(
      node.get_parameter("ext_obs_min_omega").as_double());
  cfg.iekf.ext_obs_min_correspondences =
      node.get_parameter("ext_obs_min_correspondences").as_int();
  cfg.iekf.ext_max_delta_yaw = static_cast<float>(
      node.get_parameter("ext_max_delta_yaw").as_double());
  cfg.iekf.ext_max_delta_xy = static_cast<float>(
      node.get_parameter("ext_max_delta_xy").as_double());

  return cfg;
}

// ---------------------------------------------------------------------------
// build_loop_closure_config — loop closure backend config
// ---------------------------------------------------------------------------
LoopClosureConfig build_loop_closure_config(const rclcpp::Node& node) {
  LoopClosureConfig lc;
  lc.submap_keyframe_count = node.get_parameter("lc_submap_keyframe_count").as_int();
  lc.submap_voxel_size = node.get_parameter("lc_submap_voxel_size").as_double();
  lc.loop_search_radius = node.get_parameter("lc_loop_search_radius").as_double();
  lc.loop_min_keyframe_gap = node.get_parameter("lc_loop_min_keyframe_gap").as_int();
  lc.loop_min_submap_gap = node.get_parameter("lc_loop_min_submap_gap").as_int();
  lc.max_candidates_per_query = node.get_parameter("lc_max_candidates_per_query").as_int();
  lc.gicp_max_iterations = node.get_parameter("lc_gicp_max_iterations").as_int();
  lc.gicp_max_correspondence_distance = node.get_parameter("lc_gicp_max_correspondence_distance").as_double();
  lc.gicp_fitness_threshold = node.get_parameter("lc_gicp_fitness_threshold").as_double();
  lc.gicp_min_fitness_threshold = node.get_parameter("lc_gicp_min_fitness_threshold").as_double();
  lc.gicp_min_correspondences = node.get_parameter("lc_gicp_min_correspondences").as_int();
  lc.gicp_max_translation = node.get_parameter("lc_gicp_max_translation").as_double();
  lc.gicp_max_rotation = node.get_parameter("lc_gicp_max_rotation").as_double();
  lc.overlap_max_distance = node.get_parameter("lc_overlap_max_distance").as_double();
  lc.min_initial_inliers = node.get_parameter("lc_min_initial_inliers").as_int();
  lc.min_initial_overlap_ratio = node.get_parameter("lc_min_initial_overlap_ratio").as_double();
  lc.min_initial_target_overlap_ratio = node.get_parameter("lc_min_initial_target_overlap_ratio").as_double();
  lc.min_final_overlap_ratio = node.get_parameter("lc_min_final_overlap_ratio").as_double();
  lc.min_final_target_overlap_ratio = node.get_parameter("lc_min_final_target_overlap_ratio").as_double();
  lc.min_match_score = node.get_parameter("lc_min_match_score").as_double();
  lc.heading_max_delta = node.get_parameter("lc_heading_max_delta").as_double();
  lc.loop_information_weight = node.get_parameter("lc_loop_information_weight").as_double();
  lc.pgo_max_iterations = node.get_parameter("lc_pgo_max_iterations").as_int();
  lc.project_to_2d = node.get_parameter("lc_project_to_2d").as_bool();
  lc.height_filter_min = node.get_parameter("lc_height_filter_min").as_double();
  lc.height_filter_max = node.get_parameter("lc_height_filter_max").as_double();
  lc.enable_debug_log = node.get_parameter("lc_enable_debug_log").as_bool();
  return lc;
}

}  // namespace param_loader
}  // namespace tof_slam
