// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// slam_node.cpp — ROS 1 (Noetic) node wrapping core::LioEstimator.
//
// Single-threaded event queue model: IMU and LiDAR callbacks push to one
// queue.  When deterministic_queue is enabled, events are inserted in sensor
// timestamp order with a configurable buffer delay, ensuring identical
// processing order across runs regardless of OS thread scheduling.
// Otherwise, events are processed in arrival order (FIFO).

#include "tof_slam/ros1/slam_node.hpp"

#include <algorithm>
#include <cstdlib>
#include <string>

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <iomanip>

namespace tof_slam {

SlamNode::SlamNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) {
  // ---- Read parameters ------------------------------------------------------
  pnh_.param<std::string>("map_frame", map_frame_, "map");
  pnh_.param<std::string>("base_frame", base_frame_, "base_link");

  std::string imu_topic, lidar_topic;
  pnh_.param<std::string>("imu_topic", imu_topic, "/livox/imu_192_168_0_65");
  pnh_.param<std::string>("lidar_topic", lidar_topic, "/livox/lidar_192_168_0_65");

  double gyr_noise, acc_noise, bgyr_noise, bacc_noise, gravity_z;
  pnh_.param<double>("imu_gyro_noise", gyr_noise, 0.1);
  pnh_.param<double>("imu_accel_noise", acc_noise, 0.1);
  pnh_.param<double>("imu_gyro_bias_noise", bgyr_noise, 0.0001);
  pnh_.param<double>("imu_accel_bias_noise", bacc_noise, 0.0001);
  pnh_.param<double>("imu_gravity_z", gravity_z, -9.7946);
  double frontend_gravity_noise_std;
  pnh_.param<double>("frontend_gravity_noise_std", frontend_gravity_noise_std, 0.001);

  int imu_init_samples_param;
  pnh_.param<int>("imu_init_samples", imu_init_samples_param, 100);
  int init_samples = imu_init_samples_param;

  double ext_x, ext_y, ext_z;
  pnh_.param<double>("extrinsic_x", ext_x, 0.0);
  pnh_.param<double>("extrinsic_y", ext_y, 0.0);
  pnh_.param<double>("extrinsic_z", ext_z, 0.0);

  // Extrinsic rotation (RPY in degrees, ZYX convention)
  double ext_roll_deg, ext_pitch_deg, ext_yaw_deg;
  pnh_.param<double>("extrinsic_roll",  ext_roll_deg,  0.0);
  pnh_.param<double>("extrinsic_pitch", ext_pitch_deg, 0.0);
  pnh_.param<double>("extrinsic_yaw",   ext_yaw_deg,   0.0);

  // Frontend parameters
  double frontend_voxel_size;
  pnh_.param<double>("frontend_voxel_size", frontend_voxel_size, 0.1);
  int frontend_max_iterations;
  pnh_.param<int>("frontend_max_iterations", frontend_max_iterations, 4);
  double frontend_convergence_threshold;
  pnh_.param<double>("frontend_convergence_threshold", frontend_convergence_threshold, 0.001);
  bool frontend_enable_undistortion;
  pnh_.param<bool>("frontend_enable_undistortion", frontend_enable_undistortion, false);
  double frontend_min_distance;
  pnh_.param<double>("frontend_min_distance", frontend_min_distance, 0.5);
  double frontend_max_distance;
  pnh_.param<double>("frontend_max_distance", frontend_max_distance, 100.0);
  double frontend_map_planarity_threshold;
  pnh_.param<double>("frontend_map_planarity_threshold", frontend_map_planarity_threshold, 0.1);
  double frontend_scan_duration;
  pnh_.param<double>("frontend_scan_duration", frontend_scan_duration, 0.1);
  int frontend_init_imu_samples;
  pnh_.param<int>("frontend_init_imu_samples", frontend_init_imu_samples, 100);
  if (frontend_init_imu_samples > 0) init_samples = frontend_init_imu_samples;
  int frontend_voxel_hierarchy_factor;
  pnh_.param<int>("frontend_voxel_hierarchy_factor", frontend_voxel_hierarchy_factor, 3);
  double frontend_map_box_multiplier;
  pnh_.param<double>("frontend_map_box_multiplier", frontend_map_box_multiplier, 2.0);
  int frontend_stride;
  pnh_.param<int>("frontend_stride", frontend_stride, 4);
  double frontend_lidar_noise_std;
  pnh_.param<double>("frontend_lidar_noise_std", frontend_lidar_noise_std, 0.05);
  double frontend_map_l0_voxel_size;
  pnh_.param<double>("frontend_map_l0_voxel_size", frontend_map_l0_voxel_size, -1.0);
  int frontend_max_correspondences;
  pnh_.param<int>("frontend_max_correspondences", frontend_max_correspondences, 0);
  int frontend_min_l0_for_surfel;
  pnh_.param<int>("frontend_min_l0_for_surfel", frontend_min_l0_for_surfel, 3);
  double frontend_l0_ema_alpha_min;
  pnh_.param<double>("frontend_l0_ema_alpha_min", frontend_l0_ema_alpha_min, 0.0);
  int frontend_surfel_lock_frames;
  pnh_.param<int>("frontend_surfel_lock_frames", frontend_surfel_lock_frames, 50);
  int frontend_l0_centroid_freeze_count;
  pnh_.param<int>("frontend_l0_centroid_freeze_count", frontend_l0_centroid_freeze_count, 0);
  double frontend_ema_gate_radius;
  pnh_.param<double>("frontend_ema_gate_radius", frontend_ema_gate_radius, 0.0);
  double frontend_sigma2_age_scale;
  pnh_.param<double>("frontend_sigma2_age_scale", frontend_sigma2_age_scale, 0.0);
  double frontend_pncg_threshold;
  pnh_.param<double>("frontend_pncg_threshold", frontend_pncg_threshold, 0.0);
  double frontend_alpha_degen_floor;
  pnh_.param<double>("frontend_alpha_degen_floor", frontend_alpha_degen_floor, 0.0);
  double frontend_degen_severity_ratio_ref;
  pnh_.param<double>("frontend_degen_severity_ratio_ref", frontend_degen_severity_ratio_ref, 0.0);
  // Geometric Covariance (iG-LIO-style observation covariance)
  bool frontend_enable_geometric_covariance;
  pnh_.param<bool>("frontend_enable_geometric_covariance", frontend_enable_geometric_covariance, false);
  double frontend_geometric_cov_min_eigenvalue;
  pnh_.param<double>("frontend_geometric_cov_min_eigenvalue", frontend_geometric_cov_min_eigenvalue, 4e-4);
  int frontend_geometric_cov_min_points;
  pnh_.param<int>("frontend_geometric_cov_min_points", frontend_geometric_cov_min_points, 6);

  // S12-B.B.1 HS-A: Cross-Level Rank-3 Information Filter (sensor-global).
  bool frontend_hs_a_enable_rank3;
  pnh_.param<bool>("frontend_hs_a_enable_rank3", frontend_hs_a_enable_rank3, false);
  double frontend_hs_a_l1_sigma_floor;
  pnh_.param<double>("frontend_hs_a_l1_sigma_floor", frontend_hs_a_l1_sigma_floor, 1.0e-6);
  double frontend_hs_a_l2_spd_eps;
  pnh_.param<double>("frontend_hs_a_l2_spd_eps", frontend_hs_a_l2_spd_eps, 1.0e-9);
  bool frontend_enable_degen_pvmap_override;
  pnh_.param<bool>("frontend_enable_degen_pvmap_override", frontend_enable_degen_pvmap_override, false);
  double frontend_degen_pvmap_cos_threshold;
  pnh_.param<double>("frontend_degen_pvmap_cos_threshold", frontend_degen_pvmap_cos_threshold, 0.5);
  int frontend_degen_freeze_min_persist;
  pnh_.param<int>("frontend_degen_freeze_min_persist", frontend_degen_freeze_min_persist, 0);
  double frontend_pvmap_sigma2_scale;
  pnh_.param<double>("frontend_pvmap_sigma2_scale", frontend_pvmap_sigma2_scale, 2.0);

  int frontend_max_inner_iterations;
  pnh_.param<int>("frontend_max_inner_iterations", frontend_max_inner_iterations, 4);
  double frontend_max_plane_distance;
  pnh_.param<double>("frontend_max_plane_distance", frontend_max_plane_distance, 0.0);
  double frontend_adaptive_threshold_divisor;
  pnh_.param<double>("frontend_adaptive_threshold_divisor",
                      frontend_adaptive_threshold_divisor, 9.0);

  // Degeneracy-Aware IEKF
  bool frontend_enable_degeneracy_detection;
  pnh_.param<bool>("frontend_enable_degeneracy_detection", frontend_enable_degeneracy_detection, true);
  double frontend_degeneracy_threshold;
  pnh_.param<double>("frontend_degeneracy_threshold", frontend_degeneracy_threshold, 50.0);
  double frontend_degeneracy_ratio_threshold;
  pnh_.param<double>("frontend_degeneracy_ratio_threshold", frontend_degeneracy_ratio_threshold, 0.0);
  double frontend_degeneracy_soft_floor;
  pnh_.param<double>("frontend_degeneracy_soft_floor", frontend_degeneracy_soft_floor, 0.0);
  double frontend_map_degen_ratio_threshold;
  pnh_.param<double>("frontend_map_degen_ratio_threshold", frontend_map_degen_ratio_threshold, 0.01);

  // ICDR: Information-theoretic Continuous Degeneracy Regularization
  bool frontend_enable_icdr;
  pnh_.param<bool>("frontend_enable_icdr", frontend_enable_icdr, false);
  double frontend_icdr_rho_thresh;
  pnh_.param<double>("frontend_icdr_rho_thresh", frontend_icdr_rho_thresh, 0.3);
  double frontend_icdr_tau;
  pnh_.param<double>("frontend_icdr_tau", frontend_icdr_tau, 0.05);
  double frontend_icdr_w_min;
  pnh_.param<double>("frontend_icdr_w_min", frontend_icdr_w_min, 0.01);
  bool frontend_enable_icdr_tip;
  pnh_.param<bool>("frontend_enable_icdr_tip", frontend_enable_icdr_tip, true);
  double frontend_icdr_tip_alpha;
  pnh_.param<double>("frontend_icdr_tip_alpha", frontend_icdr_tip_alpha, 0.3);
  double frontend_icdr_tip_beta;
  pnh_.param<double>("frontend_icdr_tip_beta", frontend_icdr_tip_beta, 0.5);
  double frontend_icdr_tip_d_decay;
  pnh_.param<double>("frontend_icdr_tip_d_decay", frontend_icdr_tip_d_decay, 2.0);

  // Covariance floor
  double frontend_p_floor_rot;
  pnh_.param<double>("frontend_p_floor_rot", frontend_p_floor_rot, 1e-6);
  double frontend_p_floor_pos;
  pnh_.param<double>("frontend_p_floor_pos", frontend_p_floor_pos, 1e-6);
  double frontend_p_floor_vel;
  pnh_.param<double>("frontend_p_floor_vel", frontend_p_floor_vel, 1e-4);
  double frontend_p_floor_bias;
  pnh_.param<double>("frontend_p_floor_bias", frontend_p_floor_bias, 1e-8);
  double frontend_p_floor_grav;
  pnh_.param<double>("frontend_p_floor_grav", frontend_p_floor_grav, 1e-8);

  // IMU Bias Pseudo-Observation
  bool frontend_enable_bias_pseudo_obs;
  pnh_.param<bool>("frontend_enable_bias_pseudo_obs", frontend_enable_bias_pseudo_obs, false);
  double frontend_bias_bg_sigma;
  pnh_.param<double>("frontend_bias_bg_sigma", frontend_bias_bg_sigma, 0.01);
  double frontend_bias_ba_sigma;
  pnh_.param<double>("frontend_bias_ba_sigma", frontend_bias_ba_sigma, 0.1);

  // Velocity Pseudo-Observation
  bool frontend_enable_velocity_pseudo_obs;
  pnh_.param<bool>("frontend_enable_velocity_pseudo_obs", frontend_enable_velocity_pseudo_obs, false);
  double frontend_velocity_sigma;
  pnh_.param<double>("frontend_velocity_sigma", frontend_velocity_sigma, 1.0);
  double frontend_velocity_degen_sigma;
  pnh_.param<double>("frontend_velocity_degen_sigma", frontend_velocity_degen_sigma, 0.0);

  // Gravity norm constraint
  bool frontend_enable_gravity_norm_constraint;
  pnh_.param<bool>("frontend_enable_gravity_norm_constraint", frontend_enable_gravity_norm_constraint, true);
  double frontend_gravity_norm_sigma;
  pnh_.param<double>("frontend_gravity_norm_sigma", frontend_gravity_norm_sigma, 0.01);

  // A-matrix (SO(3) Left Jacobian) manifold correction
  bool frontend_enable_a_matrix_correction;
  pnh_.param<bool>("frontend_enable_a_matrix_correction", frontend_enable_a_matrix_correction, true);

  // Degeneracy-Adaptive EMA Alpha
  bool frontend_enable_degeneracy_adaptive_alpha;
  pnh_.param<bool>("frontend_enable_degeneracy_adaptive_alpha", frontend_enable_degeneracy_adaptive_alpha, false);
  double frontend_degeneracy_alpha_scale;
  pnh_.param<double>("frontend_degeneracy_alpha_scale", frontend_degeneracy_alpha_scale, 1.0);
  bool frontend_enable_velocity_damping;
  pnh_.param<bool>("frontend_enable_velocity_damping", frontend_enable_velocity_damping, true);

  // PointVoxelMap
  double frontend_pvmap_voxel_size;
  pnh_.param<double>("frontend_pvmap_voxel_size", frontend_pvmap_voxel_size, 0.5);
  int frontend_pvmap_max_points_per_voxel;
  pnh_.param<int>("frontend_pvmap_max_points_per_voxel", frontend_pvmap_max_points_per_voxel, 20);
  int frontend_pvmap_k_neighbors;
  pnh_.param<int>("frontend_pvmap_k_neighbors", frontend_pvmap_k_neighbors, 5);
  double frontend_pvmap_planarity_threshold;
  pnh_.param<double>("frontend_pvmap_planarity_threshold", frontend_pvmap_planarity_threshold, 0.15);
  int frontend_pvmap_knn_search_half;
  pnh_.param<int>("frontend_pvmap_knn_search_half", frontend_pvmap_knn_search_half, 1);
  bool frontend_pvmap_enable_degen_freeze;
  pnh_.param<bool>("frontend_pvmap_enable_degen_freeze", frontend_pvmap_enable_degen_freeze, false);

  // Phase 2: PVMap map pruning — independent from max_range for bounded map size
  double frontend_pvmap_max_distance;
  pnh_.param<double>("frontend_pvmap_max_distance", frontend_pvmap_max_distance, -1.0);
  double frontend_pvmap_distance_multiplier;
  pnh_.param<double>("frontend_pvmap_distance_multiplier", frontend_pvmap_distance_multiplier, 1.5);
  double frontend_pvmap_degen_freeze_cos_threshold;
  pnh_.param<double>("frontend_pvmap_degen_freeze_cos_threshold", frontend_pvmap_degen_freeze_cos_threshold, 0.7);
  int frontend_pvmap_degen_freeze_max_frames;
  pnh_.param<int>("frontend_pvmap_degen_freeze_max_frames", frontend_pvmap_degen_freeze_max_frames, 200);
  bool frontend_pvmap_use_qr_plane_fit;
  pnh_.param<bool>("frontend_pvmap_use_qr_plane_fit", frontend_pvmap_use_qr_plane_fit, false);
  bool frontend_pvmap_use_proximity_insertion;
  pnh_.param<bool>("frontend_pvmap_use_proximity_insertion", frontend_pvmap_use_proximity_insertion, false);
  bool frontend_pvmap_use_voxel_plane_cache;
  pnh_.param<bool>("frontend_pvmap_use_voxel_plane_cache", frontend_pvmap_use_voxel_plane_cache, false);
  int frontend_pvmap_voxel_plane_min_points;
  pnh_.param<int>("frontend_pvmap_voxel_plane_min_points", frontend_pvmap_voxel_plane_min_points, 5);
  double frontend_pvmap_svd_min_eigenvalue;
  pnh_.param<double>("frontend_pvmap_svd_min_eigenvalue", frontend_pvmap_svd_min_eigenvalue, 0.001);

  // Point-to-Distribution (P2D) — iG-LIO style voxel Gaussian residuals
  bool frontend_enable_point_to_distribution;
  pnh_.param<bool>("frontend_enable_point_to_distribution", frontend_enable_point_to_distribution, false);
  double frontend_p2d_chi2_threshold;
  pnh_.param<double>("frontend_p2d_chi2_threshold", frontend_p2d_chi2_threshold, 7.815);
  double frontend_p2d_cov_reg_eps;
  pnh_.param<double>("frontend_p2d_cov_reg_eps", frontend_p2d_cov_reg_eps, 0.001);
  double frontend_p2d_iekf_chi2_threshold;
  pnh_.param<double>("frontend_p2d_iekf_chi2_threshold", frontend_p2d_iekf_chi2_threshold, 11.345);
  bool frontend_enable_p2d_adaptive_noise;
  pnh_.param<bool>("frontend_enable_p2d_adaptive_noise", frontend_enable_p2d_adaptive_noise, true);
  double frontend_p2d_omega_scale;
  pnh_.param<double>("frontend_p2d_omega_scale", frontend_p2d_omega_scale, 1.0);

  // CSCF (Continuous Surfel Correspondence Field)
  bool frontend_enable_cscf;
  pnh_.param<bool>("frontend_enable_cscf", frontend_enable_cscf, false);
  double frontend_cscf_kernel_bandwidth;
  pnh_.param<double>("frontend_cscf_kernel_bandwidth",
                      frontend_cscf_kernel_bandwidth, 0.25);

  // Surfel Keyframe Gate (Task #133 Iter 3): gate surfel map insertion on
  // translation+rotation keyframe predicate. Mirrors iG-LIO's 0.5m gate.
  bool frontend_enable_surfel_keyframe_gate;
  pnh_.param<bool>("frontend_enable_surfel_keyframe_gate",
                   frontend_enable_surfel_keyframe_gate, false);
  double frontend_surfel_kf_trans_thresh_m;
  pnh_.param<double>("frontend_surfel_kf_trans_thresh_m",
                     frontend_surfel_kf_trans_thresh_m, 0.30);
  double frontend_surfel_kf_rot_thresh_rad;
  pnh_.param<double>("frontend_surfel_kf_rot_thresh_rad",
                     frontend_surfel_kf_rot_thresh_rad, 0.10);
  int frontend_surfel_kf_warmup_frames;
  pnh_.param<int>("frontend_surfel_kf_warmup_frames",
                  frontend_surfel_kf_warmup_frames, 20);

  // L2 Multi-Scale Correspondence
  bool frontend_enable_l2_correspondences;
  pnh_.param<bool>("frontend_enable_l2_correspondences", frontend_enable_l2_correspondences, false);
  double frontend_l2_planarity_threshold;
  pnh_.param<double>("frontend_l2_planarity_threshold", frontend_l2_planarity_threshold, 0.15);
  int frontend_min_l1_for_l2_surfel;
  pnh_.param<int>("frontend_min_l1_for_l2_surfel", frontend_min_l1_for_l2_surfel, 4);
  double frontend_l2_noise_scale;
  pnh_.param<double>("frontend_l2_noise_scale", frontend_l2_noise_scale, 9.0);

  // Per-L1 Adaptive Surfel Lock
  double frontend_adaptive_lock_cos_thresh;
  pnh_.param<double>("frontend_adaptive_lock_cos_thresh", frontend_adaptive_lock_cos_thresh, 0.85);

  // Adaptive noise
  bool frontend_enable_adaptive_noise;
  pnh_.param<bool>("frontend_enable_adaptive_noise", frontend_enable_adaptive_noise, false);
  double frontend_adaptive_range_scale, frontend_adaptive_range_ref;
  double frontend_adaptive_incidence_scale, frontend_adaptive_planarity_scale;
  pnh_.param<double>("frontend_adaptive_range_scale", frontend_adaptive_range_scale, 0.3);
  pnh_.param<double>("frontend_adaptive_range_ref", frontend_adaptive_range_ref, 25.0);
  pnh_.param<double>("frontend_adaptive_incidence_scale", frontend_adaptive_incidence_scale, 1.5);
  pnh_.param<double>("frontend_adaptive_planarity_scale", frontend_adaptive_planarity_scale, 2.0);

  // Range-inverse weight (indoor close-range boost)
  bool frontend_enable_range_inverse_weight;
  pnh_.param<bool>("frontend_enable_range_inverse_weight", frontend_enable_range_inverse_weight, false);
  double frontend_range_inverse_ref, frontend_range_inverse_power, frontend_range_inverse_min_ratio;
  pnh_.param<double>("frontend_range_inverse_ref", frontend_range_inverse_ref, 10.0);
  pnh_.param<double>("frontend_range_inverse_power", frontend_range_inverse_power, 1.0);
  pnh_.param<double>("frontend_range_inverse_min_ratio", frontend_range_inverse_min_ratio, 0.1);

  // max_corr_per_l1
  int frontend_max_corr_per_l1;
  pnh_.param<int>("frontend_max_corr_per_l1", frontend_max_corr_per_l1, 0);

  // PKO Huber robust weighting
  bool frontend_enable_pko;
  pnh_.param<bool>("frontend_enable_pko", frontend_enable_pko, false);

  // Sigma2 normal and sharing weight parameterization
  bool frontend_enable_sigma2_normal;
  pnh_.param<bool>("frontend_enable_sigma2_normal", frontend_enable_sigma2_normal, true);

  // S12-B.A.3 DG-A: Per-Channel Anisotropic Degeneracy Signature.
  // Sensor-global only (sprint12_architecture I-3 — NOT per-seq).
  bool frontend_dg_a_enable;
  pnh_.param<bool>("frontend_dg_a_enable", frontend_dg_a_enable, false);
  double frontend_dg_a_schur_eps;
  pnh_.param<double>("frontend_dg_a_schur_eps", frontend_dg_a_schur_eps, 1.0e-6);
  bool frontend_dg_a_log_per_channel;
  pnh_.param<bool>("frontend_dg_a_log_per_channel", frontend_dg_a_log_per_channel, false);

  // S13-B.A.1 P1: Anisotropic Hierarchical Information Filter.
  // Sensor-global only (sprint13_architecture §3.6 / §5 — NOT per-seq).
  // All default-OFF; flag-absent ≡ flag=false ≡ legacy scalar path.
  // CI grep hook (scripts/check_p1_flag_scope.sh) forbids these keys in
  // **/avia_v6_seq/*.yaml, **/avia_indoor_seq/*.yaml, **/mid360_seq/*.yaml,
  // **/ntu*.yaml — sensor-global only allowed.
  bool frontend_anisotropic_iekf_enable;
  pnh_.param<bool>("frontend_anisotropic_iekf_enable",
                   frontend_anisotropic_iekf_enable, false);
  bool frontend_anisotropic_iekf_scalar_shim;
  pnh_.param<bool>("frontend_anisotropic_iekf_scalar_shim",
                   frontend_anisotropic_iekf_scalar_shim, false);
  double frontend_anisotropic_iekf_epsilon;
  pnh_.param<double>("frontend_anisotropic_iekf_epsilon",
                     frontend_anisotropic_iekf_epsilon, 1.0e-3);
  double frontend_anisotropic_iekf_rho_ref_avia;
  pnh_.param<double>("frontend_anisotropic_iekf_rho_ref_avia",
                     frontend_anisotropic_iekf_rho_ref_avia, 0.0);
  double frontend_anisotropic_iekf_chi2_threshold;
  pnh_.param<double>("frontend_anisotropic_iekf_chi2_threshold",
                     frontend_anisotropic_iekf_chi2_threshold, 3.841);
  double frontend_anisotropic_iekf_sigma_theta_sq;
  pnh_.param<double>("frontend_anisotropic_iekf_sigma_theta_sq",
                     frontend_anisotropic_iekf_sigma_theta_sq, 9.0e-6);

  // S13-B.A.5 Path B: master gate routing P1 via scene_classifier
  // apply_template_() Phase C. When true, per-class kT_*::p1 overrides
  // YAML at LOCK. Sensor-global; CI hook forbids on Mid-360/NTU/per-seq.
  bool frontend_anisotropic_iekf_router_enable;
  pnh_.param<bool>("frontend_anisotropic_iekf_router_enable",
                   frontend_anisotropic_iekf_router_enable, false);

  int frontend_sharing_weight_threshold;
  pnh_.param<int>("frontend_sharing_weight_threshold", frontend_sharing_weight_threshold, 1);
  double frontend_sharing_weight_ref;
  pnh_.param<double>("frontend_sharing_weight_ref", frontend_sharing_weight_ref, 1.0);

  // Fixed-Lag Smoother (Tier 4 — IMU-coupled 2-frame GN)
  bool frontend_enable_fixed_lag_smoother;
  pnh_.param<bool>("frontend_enable_fixed_lag_smoother", frontend_enable_fixed_lag_smoother, false);
  int frontend_fls_max_iterations;
  pnh_.param<int>("frontend_fls_max_iterations", frontend_fls_max_iterations, 3);
  double frontend_fls_convergence_threshold;
  pnh_.param<double>("frontend_fls_convergence_threshold", frontend_fls_convergence_threshold, 1e-4);
  double frontend_fls_max_pos_correction;
  pnh_.param<double>("frontend_fls_max_pos_correction", frontend_fls_max_pos_correction, 0.05);
  double frontend_fls_max_rot_correction;
  pnh_.param<double>("frontend_fls_max_rot_correction", frontend_fls_max_rot_correction, 0.087);
  double frontend_fls_max_vel_correction;
  pnh_.param<double>("frontend_fls_max_vel_correction", frontend_fls_max_vel_correction, 0.5);
  int frontend_fls_min_correspondences;
  pnh_.param<int>("frontend_fls_min_correspondences", frontend_fls_min_correspondences, 50);
  double frontend_fls_imu_noise_scale;
  pnh_.param<double>("frontend_fls_imu_noise_scale", frontend_fls_imu_noise_scale, 30.0);

  // Debug timing
  bool frontend_enable_debug_timing;
  pnh_.param<bool>("frontend_enable_debug_timing", frontend_enable_debug_timing, false);

  std::string point_time_field, point_time_unit, point_time_reference;
  pnh_.param<std::string>("point_time_field", point_time_field, "timestamp");
  pnh_.param<std::string>("point_time_unit", point_time_unit, "sec");
  pnh_.param<std::string>("point_time_reference", point_time_reference, "absolute");

  // Deterministic queue: timestamp-sorted insertion eliminates ROS1 callback
  // scheduling non-determinism.  Events are inserted in sensor timestamp order
  // rather than arrival order, ensuring identical processing order across runs.
  // Buffer delay (optional, default 0) adds a small wait before processing to
  // allow concurrent callbacks to arrive and be sorted.
  pnh_.param<bool>("deterministic_queue", deterministic_queue_, false);
  double dq_delay_ms;
  pnh_.param<double>("deterministic_queue_delay_ms", dq_delay_ms, 0.0);
  queue_buffer_delay_ = dq_delay_ms * 0.001;  // ms → sec
  // R2' LiDAR-anchor predicate (default true).  When false, fall back to the
  // legacy wall-clock watermark (queue_newest_ts_ − queue_buffer_delay_) so
  // rollback remains a one-line yaml flip.
  pnh_.param<bool>("deterministic_queue_lidar_anchor",
                   deterministic_queue_lidar_anchor_, true);

  // ---- Determinism debug (env-gated) ---------------------------------------
  if (const char* p = std::getenv("TOFSLAM_DEBUG_DETERMINISM")) {
    if (std::string(p) == "1") {
      debug_determinism_ = true;
      const char* imu_path = std::getenv("TOFSLAM_DEBUG_IMU_PATH");
      const char* state_path = std::getenv("TOFSLAM_DEBUG_STATE_PATH");
      debug_imu_file_.open(imu_path ? imu_path : "/root/tofslam_debug_imu.csv");
      debug_state_file_.open(state_path ? state_path
                                        : "/root/tofslam_debug_state.csv");
      if (debug_imu_file_.is_open()) {
        debug_imu_file_ << std::setprecision(17)
                        << "idx,phase,timestamp,gx,gy,gz,ax,ay,az\n";
      }
      if (debug_state_file_.is_open()) {
        debug_state_file_ << std::setprecision(17)
                          << "frame,phase,timestamp,px,py,pz,"
                             "qw,qx,qy,qz,vx,vy,vz,"
                             "gx,gy,gz,bgx,bgy,bgz,bax,bay,baz\n";
      }
      ROS_INFO("Determinism debug: ENABLED (imu=%s state=%s)",
               imu_path ? imu_path : "/root/tofslam_debug_imu.csv",
               state_path ? state_path : "/root/tofslam_debug_state.csv");
    }
  }

  // ---- Build LioEstimator::Config -------------------------------------------
  // YAML values (imu_gyro_noise etc.) are VARIANCES (sigma^2).
  // Q diagonal = noise_std^2 = variance.  So noise_std = sqrt(variance).
  core::LioEstimator::Config cfg;
  cfg.gyro_noise_std      = static_cast<float>(std::sqrt(gyr_noise));
  cfg.acc_noise_std       = static_cast<float>(std::sqrt(acc_noise));
  cfg.gyro_bias_noise_std = static_cast<float>(std::sqrt(bgyr_noise));
  cfg.acc_bias_noise_std  = static_cast<float>(std::sqrt(bacc_noise));

  cfg.gravity_noise_std = static_cast<float>(frontend_gravity_noise_std);
  ROS_INFO("Q noise std: gyr=%.6f acc=%.6f bgyr=%.6f bacc=%.6f grav=%.6f",
    cfg.gyro_noise_std, cfg.acc_noise_std,
    cfg.gyro_bias_noise_std, cfg.acc_bias_noise_std,
    cfg.gravity_noise_std);

  cfg.stride          = frontend_stride;
  cfg.voxel_leaf_size = static_cast<float>(frontend_voxel_size);
  cfg.min_range       = static_cast<float>(frontend_min_distance);
  cfg.max_range       = static_cast<float>(frontend_max_distance);

  cfg.iekf.max_outer_iters       = frontend_max_iterations;
  cfg.iekf.max_inner_iters       = frontend_max_inner_iterations;
  cfg.iekf.convergence_threshold = static_cast<float>(frontend_convergence_threshold);
  cfg.iekf.lidar_noise_std       = static_cast<float>(frontend_lidar_noise_std);

  int frontend_cf_reuse_after_iter;
  pnh_.param<int>("frontend_cf_reuse_after_iter", frontend_cf_reuse_after_iter, 0);
  cfg.iekf.cf_reuse_after_iter   = frontend_cf_reuse_after_iter;

  float frontend_outer_convergence_threshold;
  pnh_.param<float>("frontend_outer_convergence_threshold", frontend_outer_convergence_threshold, 0.0f);
  cfg.iekf.outer_convergence_threshold = frontend_outer_convergence_threshold;

  int frontend_cf_omp_max_threads;
  pnh_.param<int>("frontend_cf_omp_max_threads", frontend_cf_omp_max_threads, 4);
  cfg.cf_omp_max_threads = frontend_cf_omp_max_threads;

  int frontend_iekf_omp_threads;
  pnh_.param<int>("frontend_iekf_omp_threads", frontend_iekf_omp_threads, 0);
  cfg.iekf.iekf_omp_threads = frontend_iekf_omp_threads;

  // Degeneracy-Aware IEKF
  cfg.iekf.enable_degeneracy_detection = frontend_enable_degeneracy_detection;
  cfg.iekf.degeneracy_threshold        = static_cast<float>(frontend_degeneracy_threshold);
  cfg.iekf.degeneracy_ratio_threshold  = static_cast<float>(frontend_degeneracy_ratio_threshold);
  cfg.iekf.degeneracy_soft_floor       = static_cast<float>(frontend_degeneracy_soft_floor);
  cfg.iekf.map_degen_ratio_threshold   = static_cast<float>(frontend_map_degen_ratio_threshold);

  // ICDR
  cfg.iekf.enable_icdr        = frontend_enable_icdr;
  cfg.iekf.icdr_rho_thresh    = static_cast<float>(frontend_icdr_rho_thresh);
  cfg.iekf.icdr_tau           = static_cast<float>(frontend_icdr_tau);
  cfg.iekf.icdr_w_min         = static_cast<float>(frontend_icdr_w_min);
  cfg.iekf.enable_icdr_tip    = frontend_enable_icdr_tip;
  cfg.iekf.icdr_tip_alpha     = static_cast<float>(frontend_icdr_tip_alpha);
  cfg.iekf.icdr_tip_beta      = static_cast<float>(frontend_icdr_tip_beta);
  cfg.iekf.icdr_tip_d_decay   = static_cast<float>(frontend_icdr_tip_d_decay);

  // Adaptive noise
  cfg.iekf.enable_adaptive_noise    = frontend_enable_adaptive_noise;
  cfg.iekf.adaptive_range_scale     = static_cast<float>(frontend_adaptive_range_scale);
  cfg.iekf.adaptive_range_ref       = static_cast<float>(frontend_adaptive_range_ref);
  cfg.iekf.adaptive_incidence_scale = static_cast<float>(frontend_adaptive_incidence_scale);
  cfg.iekf.adaptive_planarity_scale = static_cast<float>(frontend_adaptive_planarity_scale);
  // Range-inverse weight
  cfg.iekf.enable_range_inverse_weight = frontend_enable_range_inverse_weight;
  cfg.iekf.range_inverse_ref           = static_cast<float>(frontend_range_inverse_ref);
  cfg.iekf.range_inverse_power         = static_cast<float>(frontend_range_inverse_power);
  cfg.iekf.range_inverse_min_ratio     = static_cast<float>(frontend_range_inverse_min_ratio);
  // P2D IEKF-specific parameters
  cfg.iekf.p2d_chi2_threshold          = static_cast<float>(frontend_p2d_iekf_chi2_threshold);
  cfg.iekf.enable_p2d_adaptive_noise   = frontend_enable_p2d_adaptive_noise;
  cfg.iekf.p2d_omega_scale             = static_cast<float>(frontend_p2d_omega_scale);
  cfg.max_corr_per_l1               = frontend_max_corr_per_l1;

  // Covariance floor
  cfg.iekf.p_floor_rot  = static_cast<float>(frontend_p_floor_rot);
  cfg.iekf.p_floor_pos  = static_cast<float>(frontend_p_floor_pos);
  cfg.iekf.p_floor_vel  = static_cast<float>(frontend_p_floor_vel);
  cfg.iekf.p_floor_bias = static_cast<float>(frontend_p_floor_bias);
  cfg.iekf.p_floor_grav = static_cast<float>(frontend_p_floor_grav);

  // IMU Bias Pseudo-Observation
  cfg.iekf.enable_bias_pseudo_obs = frontend_enable_bias_pseudo_obs;
  cfg.iekf.bias_bg_sigma          = static_cast<float>(frontend_bias_bg_sigma);
  cfg.iekf.bias_ba_sigma          = static_cast<float>(frontend_bias_ba_sigma);

  // Velocity Pseudo-Observation
  cfg.iekf.enable_velocity_pseudo_obs = frontend_enable_velocity_pseudo_obs;
  cfg.iekf.velocity_sigma             = static_cast<float>(frontend_velocity_sigma);
  cfg.iekf.velocity_degen_sigma       = static_cast<float>(frontend_velocity_degen_sigma);

  // Gravity norm constraint
  cfg.iekf.enable_gravity_norm_constraint = frontend_enable_gravity_norm_constraint;
  cfg.iekf.gravity_norm_sigma             = static_cast<float>(frontend_gravity_norm_sigma);

  // A-matrix manifold correction
  cfg.iekf.enable_a_matrix_correction = frontend_enable_a_matrix_correction;

  // Sigma2 normal and sharing weight
  cfg.iekf.enable_sigma2_normal       = frontend_enable_sigma2_normal;
  cfg.iekf.sharing_weight_threshold   = frontend_sharing_weight_threshold;
  cfg.iekf.sharing_weight_ref         = static_cast<float>(frontend_sharing_weight_ref);

  // S12-B.A.3 DG-A plumbing
  cfg.iekf.dg_a_enable                = frontend_dg_a_enable;
  cfg.iekf.dg_a_schur_eps             = static_cast<float>(frontend_dg_a_schur_eps);
  cfg.iekf.dg_a_log_per_channel       = frontend_dg_a_log_per_channel;
  if (frontend_dg_a_enable) {
    ROS_INFO("[DG-A] enable=true schur_eps=%.1e log_per_channel=%d (s12 instrumentation)",
             frontend_dg_a_schur_eps, frontend_dg_a_log_per_channel ? 1 : 0);
  }

  // S13-B.A.1 P1 anisotropic-IEKF plumbing (sprint13_architecture §3.4).
  // All fields default-OFF; values populated regardless of master flag so
  // shim/byte-compare modes can be exercised independently.
  cfg.iekf.anisotropic_iekf_enable        = frontend_anisotropic_iekf_enable;
  cfg.iekf.anisotropic_iekf_scalar_shim   = frontend_anisotropic_iekf_scalar_shim;
  cfg.iekf.anisotropic_iekf_epsilon       = static_cast<float>(frontend_anisotropic_iekf_epsilon);
  cfg.iekf.anisotropic_iekf_rho_ref_avia  = frontend_anisotropic_iekf_rho_ref_avia;
  cfg.iekf.anisotropic_iekf_chi2_threshold =
      static_cast<float>(frontend_anisotropic_iekf_chi2_threshold);
  cfg.iekf.anisotropic_iekf_sigma_theta_sq =
      static_cast<float>(frontend_anisotropic_iekf_sigma_theta_sq);
  // S13-B.A.5 Path B router master gate.
  cfg.iekf.anisotropic_iekf_router_enable = frontend_anisotropic_iekf_router_enable;
  if (frontend_anisotropic_iekf_router_enable) {
    ROS_INFO("[P1-router] scene-class adaptive P1 router ENABLED "
             "(per-class P1Tuple via apply_template_ Phase C at LOCK frame)");
  }
  if (frontend_anisotropic_iekf_enable) {
    ROS_INFO("[P1] anisotropic_iekf enable=true shim=%d eps=%.1e rho_ref_avia=%.6f "
             "chi2_thr=%.3f sigma_theta_sq=%.1e (S13 unified-config research)",
             frontend_anisotropic_iekf_scalar_shim ? 1 : 0,
             frontend_anisotropic_iekf_epsilon,
             frontend_anisotropic_iekf_rho_ref_avia,
             frontend_anisotropic_iekf_chi2_threshold,
             frontend_anisotropic_iekf_sigma_theta_sq);
  }

  // PointVoxelMap
  cfg.point_voxel_map.voxel_size           = static_cast<float>(frontend_pvmap_voxel_size);
  cfg.point_voxel_map.max_points_per_voxel = frontend_pvmap_max_points_per_voxel;
  // Use explicit pvmap_max_distance if set (> 0), otherwise fall back to max_range.
  // Bug fix: previously always overwrote with max_range, making the config param useless.
  cfg.point_voxel_map.max_distance         = (frontend_pvmap_max_distance > 0)
      ? static_cast<float>(frontend_pvmap_max_distance)
      : cfg.max_range;
  cfg.point_voxel_map.distance_multiplier  =
      static_cast<float>(frontend_pvmap_distance_multiplier);
  cfg.pvmap_k_neighbors                    = frontend_pvmap_k_neighbors;
  cfg.pvmap_planarity_threshold            = static_cast<float>(frontend_pvmap_planarity_threshold);
  cfg.point_voxel_map.knn_search_half      = frontend_pvmap_knn_search_half;
  cfg.point_voxel_map.enable_degen_freeze  = frontend_pvmap_enable_degen_freeze;
  cfg.point_voxel_map.degen_freeze_cos_threshold = static_cast<float>(frontend_pvmap_degen_freeze_cos_threshold);
  cfg.point_voxel_map.degen_freeze_max_frames = frontend_pvmap_degen_freeze_max_frames;
  cfg.point_voxel_map.use_qr_plane_fit    = frontend_pvmap_use_qr_plane_fit;
  cfg.point_voxel_map.use_proximity_insertion = frontend_pvmap_use_proximity_insertion;
  cfg.point_voxel_map.use_voxel_plane_cache  = frontend_pvmap_use_voxel_plane_cache;
  cfg.point_voxel_map.voxel_plane_min_points = frontend_pvmap_voxel_plane_min_points;
  cfg.point_voxel_map.svd_min_eigenvalue = static_cast<float>(frontend_pvmap_svd_min_eigenvalue);

  cfg.enable_point_to_distribution = frontend_enable_point_to_distribution;
  cfg.p2d_chi2_threshold = static_cast<float>(frontend_p2d_chi2_threshold);
  cfg.p2d_cov_reg_eps = static_cast<float>(frontend_p2d_cov_reg_eps);
  cfg.enable_cscf = frontend_enable_cscf;
  cfg.cscf_kernel_bandwidth =
      static_cast<float>(frontend_cscf_kernel_bandwidth);

  // Surfel Keyframe Gate (Task #133 Iter 3)
  cfg.enable_surfel_keyframe_gate          = frontend_enable_surfel_keyframe_gate;
  cfg.surfel_kf_trans_thresh_m             = frontend_surfel_kf_trans_thresh_m;
  cfg.surfel_kf_rot_thresh_rad             = frontend_surfel_kf_rot_thresh_rad;
  cfg.surfel_kf_warmup_frames              = frontend_surfel_kf_warmup_frames;

  // PKO Huber robust weighting
  cfg.enable_pko = frontend_enable_pko;

  // Fixed-Lag Smoother (Tier 4)
  cfg.enable_fixed_lag_smoother   = frontend_enable_fixed_lag_smoother;
  cfg.fls_min_correspondences     = frontend_fls_min_correspondences;
  cfg.fls.max_iterations          = frontend_fls_max_iterations;
  cfg.fls.convergence_threshold   = static_cast<float>(frontend_fls_convergence_threshold);
  cfg.fls.max_pos_correction      = static_cast<float>(frontend_fls_max_pos_correction);
  cfg.fls.max_rot_correction      = static_cast<float>(frontend_fls_max_rot_correction);
  cfg.fls.max_vel_correction      = static_cast<float>(frontend_fls_max_vel_correction);
  cfg.fls.imu_noise_scale         = static_cast<float>(frontend_fls_imu_noise_scale);

  // Debug timing
  cfg.enable_debug_timing       = frontend_enable_debug_timing;
  cfg.iekf.enable_debug_timing  = cfg.enable_debug_timing;

  // Output paths (empty = disabled)
  std::string diag_path, timing_path, traj_path;
  pnh_.param<std::string>("diagnostics_log_path", diag_path, "/root/tofslam_diagnostics.csv");
  pnh_.param<std::string>("timing_log_path", timing_path, "/root/tofslam_timing.csv");
  pnh_.param<std::string>("traj_csv_path", traj_path, "/root/tofslam_traj.csv");
  cfg.diagnostics_log_path = diag_path;
  cfg.timing_log_path = timing_path;
  cfg.traj_csv_path = traj_path;

  ROS_INFO("IEKF: degeneracy=%s (thresh=%.1f) P_floor=%.1e debug_timing=%s",
    cfg.iekf.enable_degeneracy_detection ? "ON" : "OFF",
    cfg.iekf.degeneracy_threshold,
    cfg.iekf.p_floor_pos,
    cfg.enable_debug_timing ? "ON" : "OFF");
  if (cfg.iekf.enable_icdr) {
    ROS_INFO("ICDR: rho_thresh=%.3f tau=%.3f w_min=%.3f TIP=%s (alpha=%.2f beta=%.2f d_decay=%.1f)",
      cfg.iekf.icdr_rho_thresh, cfg.iekf.icdr_tau, cfg.iekf.icdr_w_min,
      cfg.iekf.enable_icdr_tip ? "ON" : "OFF",
      cfg.iekf.icdr_tip_alpha, cfg.iekf.icdr_tip_beta, cfg.iekf.icdr_tip_d_decay);
  }

  // Map L0 voxel size: use dedicated param if set, else fall back to scan voxel.
  cfg.surfel_map.l0_voxel_size = (frontend_map_l0_voxel_size > 0.0)
      ? static_cast<float>(frontend_map_l0_voxel_size) : cfg.voxel_leaf_size;
  cfg.surfel_map.l1_hierarchy_factor = frontend_voxel_hierarchy_factor;
  cfg.surfel_map.min_l0_for_surfel   = frontend_min_l0_for_surfel;
  cfg.surfel_map.max_distance        = cfg.max_range;
  cfg.surfel_map.planarity_threshold = static_cast<float>(frontend_map_planarity_threshold);
  cfg.surfel_map.distance_multiplier = static_cast<float>(frontend_map_box_multiplier);
  cfg.surfel_map.l0_ema_alpha_min    = static_cast<float>(frontend_l0_ema_alpha_min);
  cfg.surfel_map.surfel_lock_frames  = frontend_surfel_lock_frames;
  cfg.surfel_map.l0_centroid_freeze_count = frontend_l0_centroid_freeze_count;
  cfg.surfel_map.ema_gate_radius     = static_cast<float>(frontend_ema_gate_radius);
  cfg.surfel_map.sigma2_age_scale    = static_cast<float>(frontend_sigma2_age_scale);
  cfg.surfel_map.pncg_threshold    = static_cast<float>(frontend_pncg_threshold);
  cfg.surfel_map.alpha_degen_floor = static_cast<float>(frontend_alpha_degen_floor);
  cfg.surfel_map.degen_severity_ratio_ref = static_cast<float>(frontend_degen_severity_ratio_ref);
  cfg.surfel_map.enable_geometric_covariance  = frontend_enable_geometric_covariance;
  cfg.surfel_map.geometric_cov_min_eigenvalue = static_cast<float>(frontend_geometric_cov_min_eigenvalue);
  cfg.surfel_map.geometric_cov_min_points     = frontend_geometric_cov_min_points;
  // S12-B.B.1 + B.B.2 HS-A plumbing (sensor-global).
  cfg.surfel_map.hs_a_enable_rank3            = frontend_hs_a_enable_rank3;
  cfg.surfel_map.hs_a_l1_sigma_floor          = static_cast<float>(frontend_hs_a_l1_sigma_floor);
  cfg.surfel_map.hs_a_l2_spd_eps              = static_cast<float>(frontend_hs_a_l2_spd_eps);
  if (frontend_hs_a_enable_rank3) {
    ROS_INFO("[HS-A] enable_rank3=true l1_sigma_floor=%.1e l2_spd_eps=%.1e (s12 rank-3 pull-back)",
             frontend_hs_a_l1_sigma_floor, frontend_hs_a_l2_spd_eps);
  }
  if (frontend_enable_geometric_covariance) {
    ROS_INFO("Geometric Covariance: ENABLED (min_ev=%.1e min_pts=%d)",
             frontend_geometric_cov_min_eigenvalue, frontend_geometric_cov_min_points);
  }

  cfg.enable_degen_pvmap_override    = frontend_enable_degen_pvmap_override;
  cfg.degen_pvmap_cos_threshold      = static_cast<float>(frontend_degen_pvmap_cos_threshold);
  cfg.degen_freeze_min_persist       = frontend_degen_freeze_min_persist;
  cfg.pvmap_sigma2_scale             = static_cast<float>(frontend_pvmap_sigma2_scale);
  cfg.surfel_map.adaptive_lock_cos_thresh = static_cast<float>(frontend_adaptive_lock_cos_thresh);

  // L2 Multi-Scale Correspondence
  cfg.surfel_map.enable_l2_correspondences = frontend_enable_l2_correspondences;
  cfg.surfel_map.l2_planarity_threshold    = static_cast<float>(frontend_l2_planarity_threshold);
  cfg.surfel_map.min_l1_for_l2_surfel      = frontend_min_l1_for_l2_surfel;
  cfg.surfel_map.l2_noise_scale            = static_cast<float>(frontend_l2_noise_scale);
  cfg.iekf.enable_l2_correspondences       = cfg.surfel_map.enable_l2_correspondences;
  cfg.iekf.l2_noise_scale                  = cfg.surfel_map.l2_noise_scale;
  if (cfg.surfel_map.enable_l2_correspondences) {
    ROS_INFO("L2 Multi-Scale: ENABLED (planarity=%.2f min_l1=%d noise_scale=%.1f)",
             cfg.surfel_map.l2_planarity_threshold,
             cfg.surfel_map.min_l1_for_l2_surfel,
             cfg.surfel_map.l2_noise_scale);
  }

  cfg.max_correspondences = frontend_max_correspondences;
  cfg.max_plane_distance  = static_cast<float>(frontend_max_plane_distance);
  cfg.adaptive_threshold_divisor = static_cast<float>(frontend_adaptive_threshold_divisor);

  cfg.enable_undistortion = frontend_enable_undistortion;
  cfg.scan_duration       = static_cast<float>(frontend_scan_duration);
  cfg.enable_degeneracy_adaptive_alpha = frontend_enable_degeneracy_adaptive_alpha;
  cfg.degeneracy_alpha_scale           = static_cast<float>(frontend_degeneracy_alpha_scale);
  cfg.enable_velocity_damping          = frontend_enable_velocity_damping;

  // Extrinsics: rotation (RPY ZYX) + translation
  {
    Eigen::Matrix3f ext_rot = Eigen::Matrix3f::Identity();
    const bool has_rpy = (ext_roll_deg != 0.0 || ext_pitch_deg != 0.0 || ext_yaw_deg != 0.0);
    if (has_rpy) {
      const float r = static_cast<float>(ext_roll_deg  * M_PI / 180.0);
      const float p = static_cast<float>(ext_pitch_deg * M_PI / 180.0);
      const float y = static_cast<float>(ext_yaw_deg   * M_PI / 180.0);
      Eigen::AngleAxisf Rz(y, Eigen::Vector3f::UnitZ());
      Eigen::AngleAxisf Ry(p, Eigen::Vector3f::UnitY());
      Eigen::AngleAxisf Rx(r, Eigen::Vector3f::UnitX());
      ext_rot = (Rz * Ry * Rx).matrix();
      ROS_INFO("Extrinsic rotation (RPY deg): roll=%.3f pitch=%.3f yaw=%.3f",
               ext_roll_deg, ext_pitch_deg, ext_yaw_deg);
      ROS_INFO("Extrinsic R:\n  [%.6f %.6f %.6f]\n  [%.6f %.6f %.6f]\n  [%.6f %.6f %.6f]",
               ext_rot(0,0), ext_rot(0,1), ext_rot(0,2),
               ext_rot(1,0), ext_rot(1,1), ext_rot(1,2),
               ext_rot(2,0), ext_rot(2,1), ext_rot(2,2));
    }
    cfg.T_body_lidar = core::Se3(
        ext_rot,
        Eigen::Vector3f(static_cast<float>(ext_x),
                        static_cast<float>(ext_y),
                        static_cast<float>(ext_z)));
  }

  estimator_ = std::make_unique<core::LioEstimator>(cfg);

  // R9 C2': classifier enable flag (default true; per-seq YAMLs set false).
  bool frontend_enable_classifier;
  pnh_.param<bool>("frontend_enable_classifier", frontend_enable_classifier, true);
  estimator_->set_classifier_enable(frontend_enable_classifier);
  ROS_INFO("LioEstimator: scene classifier %s",
           frontend_enable_classifier ? "ENABLED (R-A two-stage)" : "DISABLED");

  // R0.9 H3b: Avia outdoor sensor-domain GUARD for OUTDOOR_DRIFT Stage A
  // discriminator. Default false (Mid-360/indoor/NTU/Avia-indoor leave default).
  // Activated ONLY by avia_outdoor.yaml's `frontend_is_avia_outdoor: true`.
  bool frontend_is_avia_outdoor;
  pnh_.param<bool>("frontend_is_avia_outdoor", frontend_is_avia_outdoor, false);
  estimator_->set_is_avia_outdoor(frontend_is_avia_outdoor);
  ROS_INFO("LioEstimator: is_avia_outdoor guard %s",
           frontend_is_avia_outdoor ? "TRUE (OUTDOOR_DRIFT discriminator armed)" : "false");

  // ---- Livox CustomMsg adapter -----------------------------------------------
  std::string lidar_msg_type;
  pnh_.param<std::string>("lidar_msg_type", lidar_msg_type, "auto");

#ifdef HAS_LIVOX_ROS_DRIVER2
  // Auto-detect: if topic contains "livox" and not "pointcloud", assume CustomMsg
  if (lidar_msg_type == "auto") {
    if (lidar_topic.find("livox") != std::string::npos &&
        lidar_topic.find("pointcloud") == std::string::npos) {
      lidar_msg_type = "livox_custom";
    } else {
      lidar_msg_type = "pointcloud2";
    }
  }
  use_livox_custom_msg_ = (lidar_msg_type == "livox_custom");

  if (use_livox_custom_msg_) {
    livox_adapter_ = ros_adapter::LivoxCustomMsgAdapter();
    ROS_INFO("LiDAR input: Livox CustomMsg (direct C++ adapter)");
  } else {
    ROS_INFO("LiDAR input: sensor_msgs/PointCloud2");
  }
#else
  (void)lidar_msg_type;
  ROS_INFO("LiDAR input: sensor_msgs/PointCloud2 (Livox support disabled)");
#endif

  // ---- ImuAdapter -----------------------------------------------------------
  ros_adapter::ImuAdapter::Config imu_cfg;
  imu_cfg.init_sample_count = init_samples;
  imu_cfg.gravity_prior     = Eigen::Vector3f(0.0f, 0.0f,
                                              static_cast<float>(gravity_z));
  imu_adapter_ = ros_adapter::ImuAdapter(imu_cfg);

  // ---- CSV logging ----------------------------------------------------------
  if (!cfg.traj_csv_path.empty()) {
    std::lock_guard<std::mutex> lk(csv_mutex_);
    csv_file_.open(cfg.traj_csv_path, std::ios::out | std::ios::trunc);
    if (csv_file_.is_open()) {
      csv_file_ << "t_sec,tx,ty,tz,qx,qy,qz,qw\n";
      csv_file_.flush();
      ROS_INFO("Trajectory CSV: %s", cfg.traj_csv_path.c_str());
    } else {
      ROS_WARN("Cannot open trajectory CSV: %s", cfg.traj_csv_path.c_str());
    }
  }

  // ---- Publishers -----------------------------------------------------------
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/tofslam/odometry", 10);
  path_pub_ = nh_.advertise<nav_msgs::Path>("/tofslam/path", 10);
  path_msg_.header.frame_id = map_frame_;

  // ---- Subscribers ----------------------------------------------------------
  imu_sub_ = nh_.subscribe(imu_topic, 2000,
      &SlamNode::imu_callback, this,
      ros::TransportHints().tcpNoDelay());

#ifdef HAS_LIVOX_ROS_DRIVER2
  if (use_livox_custom_msg_) {
    lidar_sub_ = nh_.subscribe(lidar_topic, 2000,
        &SlamNode::livox_callback, this);
  } else
#endif
  {
    lidar_sub_ = nh_.subscribe(lidar_topic, 2000,
        &SlamNode::lidar_callback, this);
  }

  // ---- Processing thread ----------------------------------------------------
  processing_thread_ = std::thread(&SlamNode::processing_loop, this);

  if (deterministic_queue_) {
    ROS_INFO("Deterministic queue: ENABLED (mode=%s, buffer_delay=%.1fms)",
             deterministic_queue_lidar_anchor_ ? "LIDAR_ANCHOR(R2')" : "WATERMARK",
             queue_buffer_delay_ * 1000.0);
  } else {
    ROS_INFO("Deterministic queue: DISABLED (FIFO arrival order)");
  }

  ROS_INFO("TofSLAM node ready (unified queue). imu=%s lidar=%s "
           "msg_type=%s init_samples=%d undistort=%s",
           imu_topic.c_str(), lidar_topic.c_str(),
#ifdef HAS_LIVOX_ROS_DRIVER2
           use_livox_custom_msg_ ? "livox_custom" : "pointcloud2",
#else
           "pointcloud2",
#endif
           init_samples,
           cfg.enable_undistortion ? "ON" : "OFF");
}

SlamNode::~SlamNode() {
  running_ = false;
  queue_cv_.notify_all();
  if (processing_thread_.joinable()) processing_thread_.join();
  std::lock_guard<std::mutex> lk(csv_mutex_);
  if (csv_file_.is_open()) csv_file_.close();
  if (debug_imu_file_.is_open()) debug_imu_file_.close();
  if (debug_state_file_.is_open()) debug_state_file_.close();
}

// ---------------------------------------------------------------------------
// enqueue_event — sorted insertion (deterministic) or FIFO (legacy)
// ---------------------------------------------------------------------------
void SlamNode::enqueue_event(SensorEvent ev) {
  std::lock_guard<std::mutex> lk(queue_mutex_);

  if (ev.timestamp > queue_newest_ts_) {
    queue_newest_ts_ = ev.timestamp;
  }

  // R2' LiDAR-anchor: track the newest LiDAR timestamp that has been
  // inserted into the queue.  This anchors the readiness predicate in
  // processing_loop(): IMUs are released only when a strictly-later LiDAR
  // has been enqueued.  Post-dedup (dedup already ran in the LiDAR callback
  // before this function is called), so the max() is over post-dedup ts.
  if (ev.type == SensorEvent::LIDAR && ev.timestamp > queue_last_lidar_ts_) {
    queue_last_lidar_ts_ = ev.timestamp;
  }

  if (deterministic_queue_) {
    // Ordering predicate: strict "before" on (timestamp, type) with IMU
    // winning the tiebreak when timestamps are equal.  Rationale: at the
    // start of a LiDAR scan, the matching IMU sample must propagate the
    // state first so that undistortion uses the freshest bias/gravity
    // estimate.
    auto strictly_before = [](const SensorEvent& a, const SensorEvent& b) {
      if (a.timestamp < b.timestamp) return true;
      if (a.timestamp > b.timestamp) return false;
      return (a.type == SensorEvent::IMU) && (b.type == SensorEvent::LIDAR);
    };

    // Insert in timestamp order (IMU-before-LiDAR tiebreak).  Scan from the
    // back: find the first element that must precede |ev|, and insert after
    // it.  This yields identical processing order regardless of callback
    // arrival order (OS thread scheduler).
    auto it = event_queue_.end();
    while (it != event_queue_.begin()) {
      auto prev = std::prev(it);
      if (!strictly_before(ev, *prev)) break;
      it = prev;
    }
    event_queue_.insert(it, std::move(ev));
  } else {
    // Legacy FIFO: arrival order
    event_queue_.push_back(std::move(ev));
  }

  queue_cv_.notify_one();
}

// ---------------------------------------------------------------------------
// IMU callback — pushes to unified event queue (lightweight)
// ---------------------------------------------------------------------------
void SlamNode::imu_callback(
    const sensor_msgs::Imu::ConstPtr& msg) {
  SensorEvent ev;
  ev.type = SensorEvent::IMU;
  ev.imu_msg = msg;
  ev.timestamp = msg->header.stamp.toSec();
  enqueue_event(std::move(ev));
}

// ---------------------------------------------------------------------------
// LiDAR callback — pushes to unified event queue (lightweight)
// ---------------------------------------------------------------------------
void SlamNode::lidar_callback(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  const double ts = msg->header.stamp.toSec();

  // Dedup: skip if timestamp is too close to last LiDAR (transport can re-deliver).
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
  enqueue_event(std::move(ev));
}

#ifdef HAS_LIVOX_ROS_DRIVER2
// ---------------------------------------------------------------------------
// Livox CustomMsg callback — direct conversion without PointCloud2 overhead
// Reference: FAST-LIO2 livox_pcl_cbk() + avia_handler()
// ---------------------------------------------------------------------------
void SlamNode::livox_callback(
    const livox_ros_driver2::CustomMsg::ConstPtr& msg) {
  const double ts = msg->header.stamp.toSec();

  // Dedup: same logic as lidar_callback
  {
    std::lock_guard<std::mutex> lk(queue_mutex_);
    if (ts - last_lidar_queue_ts_ < 0.05 && last_lidar_queue_ts_ > 0.0) return;
    last_lidar_queue_ts_ = ts;
  }

  SensorEvent ev;
  ev.type = SensorEvent::LIDAR;
  ev.cloud = livox_adapter_.convert(msg);
  ev.timestamp = ts;
  enqueue_event(std::move(ev));
}
#endif  // HAS_LIVOX_ROS_DRIVER2

// ---------------------------------------------------------------------------
// Processing thread — deterministic (timestamp-sorted) or FIFO event processing
//
// When deterministic_queue_ is enabled, events are only popped once they fall
// below the watermark (queue_newest_ts_ - queue_buffer_delay_).  This creates
// a small reordering buffer that absorbs OS-thread-scheduling skew between
// the IMU and LiDAR callbacks — an out-of-order event that arrives within the
// buffer window will be sort-inserted into its correct position before it is
// ever consumed.  On shutdown (running_ == false) the remaining queue is
// drained without regard to the watermark so no tail events are lost.
// ---------------------------------------------------------------------------
void SlamNode::processing_loop() {
  ROS_INFO("Processing thread started (%s).",
           deterministic_queue_ ? "deterministic queue" : "FIFO queue");
  size_t n_imu = 0;
  size_t n_lidar = 0;

  while (running_) {
    SensorEvent ev;
    bool have_event = false;

    // Wait for an event that is ready to be consumed.
    //
    // R2'' LiDAR-anchored predicate (deterministic_queue_lidar_anchor_=true):
    //   Task #70 U1a Fix C — both LiDAR and IMU require a STRICTLY-later
    //   LiDAR to have been enqueued before release.  This guarantees that
    //   by the time a LiDAR reaches the queue front, every IMU sample
    //   inside its scan window has already been enqueued and will drain
    //   first (strictly_before tiebreak IMU < LiDAR at equal ts), so
    //   state_history_ is fully populated before preprocess_scan reads
    //   it.  Pre-init fallback: release whenever queue_last_lidar_ts_ ==
    //   0.0 so gravity-init IMU windows are not starved before the first
    //   LiDAR arrives.  Aligns with FAST-LIO2 / iG-LIO sync_packages
    //   discipline (imu_buffer fully drained up to lidar_end_time before
    //   UndistortPcl runs).
    //
    //   PRIOR (R2'): LiDAR released on `front.ts <= queue_last_lidar_ts_`
    //   (i.e. on its own arrival), which releases LiDAR before tail-end
    //   in-window IMUs are enqueued → state_history_ race → 1e-6 bp
    //   divergence → 2-class ATE on Dark01.  Phase-1 Heisenbug collapse
    //   confirmed this mechanism.
    //
    // Legacy wall-clock watermark (deterministic_queue_lidar_anchor_=false):
    //   kept intact for rollback.  front is ready iff ts ≤ newest − buffer.
    auto is_ready = [this]() -> bool {
      if (!running_) return true;
      if (event_queue_.empty()) return false;
      if (!deterministic_queue_) return true;  // FIFO: always ready
      const auto& front = event_queue_.front();
      if (deterministic_queue_lidar_anchor_) {
        if (queue_last_lidar_ts_ == 0.0) return true;  // pre-init fallback
        // Both LiDAR and IMU: strictly-later LiDAR required to anchor the
        // scan window.  See header comment above for rationale.
        return front.timestamp < queue_last_lidar_ts_;
      }
      // Legacy: watermark-based release.
      const double watermark = queue_newest_ts_ - queue_buffer_delay_;
      return front.timestamp <= watermark;
    };

    {
      std::unique_lock<std::mutex> lk(queue_mutex_);
      queue_cv_.wait(lk, [&is_ready]() { return is_ready(); });
      if (!running_ && event_queue_.empty()) break;
      if (event_queue_.empty()) continue;

      // Recheck readiness under the lock before popping.  On shutdown we
      // drain everything regardless of watermark / anchor.
      const bool ready =
          !deterministic_queue_ || !running_ || is_ready();
      if (!ready) continue;

      ev = std::move(event_queue_.front());
      event_queue_.pop_front();
      have_event = true;
    }

    if (!have_event) continue;

    try {
      if (ev.type == SensorEvent::IMU) {
        // ---------- IMU event ----------
        // DEBUG: log IMU prefix (phase = pre_init / post_init).
        if (debug_determinism_ && debug_imu_logged_ < kDebugImuMax &&
            debug_imu_file_.is_open()) {
          const char* phase =
              estimator_->initialized() ? "post_init" : "pre_init";
          const auto& im = *ev.imu_msg;
          debug_imu_file_ << debug_imu_logged_ << "," << phase << ","
                          << ev.timestamp << ","
                          << im.angular_velocity.x << ","
                          << im.angular_velocity.y << ","
                          << im.angular_velocity.z << ","
                          << im.linear_acceleration.x << ","
                          << im.linear_acceleration.y << ","
                          << im.linear_acceleration.z << "\n";
          ++debug_imu_logged_;
          if (debug_imu_logged_ == kDebugImuMax) debug_imu_file_.flush();
        }

        auto result = imu_adapter_.process(ev.imu_msg);

        // On the frame that completes gravity init, initialize the estimator
        if (!estimator_->initialized() && imu_adapter_.initialized()) {
          const auto& ir = imu_adapter_.init_result();
          if (ir.success) {
            if (estimator_->initialize(ir)) {
              ROS_INFO("Gravity init OK: %s", ir.message.c_str());
              const auto& s = ir.initial_state;
              ROS_INFO(
                "INIT_STATE: pos=[%.4f,%.4f,%.4f] vel=[%.4f,%.4f,%.4f] "
                "gravity=[%.4f,%.4f,%.4f] scale=%.6f",
                s.position.x(), s.position.y(), s.position.z(),
                s.velocity.x(), s.velocity.y(), s.velocity.z(),
                s.gravity.x(), s.gravity.y(), s.gravity.z(),
                ir.imu_acc_scale);
              // DEBUG: log state right after gravity init ("frame 0").
              if (debug_determinism_ && debug_state_file_.is_open()) {
                Eigen::Quaternionf q(s.rotation);
                q.normalize();
                debug_state_file_ << 0 << ",init,"
                  << ev.timestamp << ","
                  << s.position.x() << "," << s.position.y() << "," << s.position.z() << ","
                  << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << ","
                  << s.velocity.x() << "," << s.velocity.y() << "," << s.velocity.z() << ","
                  << s.gravity.x() << "," << s.gravity.y() << "," << s.gravity.z() << ","
                  << s.gyro_bias.x() << "," << s.gyro_bias.y() << "," << s.gyro_bias.z() << ","
                  << s.acc_bias.x()  << "," << s.acc_bias.y()  << "," << s.acc_bias.z() << "\n";
                debug_state_file_.flush();
              }
            }
          }
        }

        if (result.has_value() && estimator_->initialized()) {
          estimator_->feed_imu(*result);
          ++n_imu;
        }
      } else {
        // ---------- LiDAR event ----------
        if (!estimator_->initialized()) {
          ROS_WARN_THROTTLE(2.0, "Waiting for gravity initialization...");
        } else {
          // DEBUG: snapshot state BEFORE feed_lidar for first few frames.
          const bool log_state = debug_determinism_ &&
                                 debug_state_logged_ < kDebugStateMax &&
                                 debug_state_file_.is_open();
          if (log_state) {
            const auto& s = estimator_->current_state();
            Eigen::Quaternionf q(s.rotation);
            q.normalize();
            debug_state_file_ << debug_state_logged_ + 1 << ",before,"
              << ev.timestamp << ","
              << s.position.x() << "," << s.position.y() << "," << s.position.z() << ","
              << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << ","
              << s.velocity.x() << "," << s.velocity.y() << "," << s.velocity.z() << ","
              << s.gravity.x() << "," << s.gravity.y() << "," << s.gravity.z() << ","
              << s.gyro_bias.x() << "," << s.gyro_bias.y() << "," << s.gyro_bias.z() << ","
              << s.acc_bias.x()  << "," << s.acc_bias.y()  << "," << s.acc_bias.z() << "\n";
          }
          if (estimator_->feed_lidar(ev.cloud, ev.timestamp)) {
            ++n_lidar;
            publish_state(estimator_->current_state(), ev.timestamp);
            if (log_state) {
              const auto& s = estimator_->current_state();
              Eigen::Quaternionf q(s.rotation);
              q.normalize();
              debug_state_file_ << debug_state_logged_ + 1 << ",after,"
                << ev.timestamp << ","
                << s.position.x() << "," << s.position.y() << "," << s.position.z() << ","
                << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << ","
                << s.velocity.x() << "," << s.velocity.y() << "," << s.velocity.z() << ","
                << s.gravity.x() << "," << s.gravity.y() << "," << s.gravity.z() << ","
                << s.gyro_bias.x() << "," << s.gyro_bias.y() << "," << s.gyro_bias.z() << ","
                << s.acc_bias.x()  << "," << s.acc_bias.y()  << "," << s.acc_bias.z() << "\n";
              debug_state_file_.flush();
              ++debug_state_logged_;
            }
          }
        }
      }
    } catch (const std::exception& e) {
      ROS_ERROR("Exception in processing loop: %s", e.what());
    }
  }

  ROS_INFO("Processing thread stopped. lidar=%zu imu=%zu", n_lidar, n_imu);
}

// ---------------------------------------------------------------------------
// Publish odometry, path, TF and CSV from a LioState snapshot
// ---------------------------------------------------------------------------
void SlamNode::publish_state(const core::LioState& state,
                                  double timestamp) {
  const ros::Time ros_time = ros::Time().fromSec(timestamp);
  Eigen::Quaternionf q(state.rotation);
  q.normalize();

  // Odometry
  nav_msgs::Odometry odom;
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
  odom_pub_.publish(odom);

  // Path
  geometry_msgs::PoseStamped ps;
  ps.header          = odom.header;
  ps.pose            = odom.pose.pose;
  path_msg_.header.stamp = ros_time;
  path_msg_.poses.push_back(ps);
  path_pub_.publish(path_msg_);

  // TF
  geometry_msgs::TransformStamped tf;
  tf.header           = odom.header;
  tf.child_frame_id   = base_frame_;
  tf.transform.translation.x = state.position.x();
  tf.transform.translation.y = state.position.y();
  tf.transform.translation.z = state.position.z();
  tf.transform.rotation.x    = q.x();
  tf.transform.rotation.y    = q.y();
  tf.transform.rotation.z    = q.z();
  tf.transform.rotation.w    = q.w();
  tf_broadcaster_.sendTransform(tf);

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
