// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// lwo_estimator.hpp — Thin orchestrator for LiDAR-Wheel Odometry (no IMU).
//
// Mirrors LioEstimator's role but replaces IMU propagation with wheel
// odometry propagation and adds ground + wheel velocity constraints.
//
// No ROS dependency.  Thread-safe state access via mutex.

#ifndef TOF_SLAM_FRONTEND_W_ESTIMATOR_LWO_ESTIMATOR_HPP_
#define TOF_SLAM_FRONTEND_W_ESTIMATOR_LWO_ESTIMATOR_HPP_

#include <chrono>
#include <deque>
#include <mutex>
#include <vector>

#include "tof_slam/frontend_w/estimator/lwo_iekf_updater.hpp"
#include "tof_slam/frontend_w/estimator/lwo_state.hpp"
#include "tof_slam/frontend_w/estimator/wheel_propagator.hpp"
#include "tof_slam/frontend_w/measurement/ground_constraint.hpp"
#include "tof_slam/frontend_w/measurement/wheel_measurement.hpp"
#include "tof_slam/frontend/estimator/motion_undistort.hpp"
#include "tof_slam/frontend/map/surfel_map.hpp"
#include "tof_slam/frontend/robust/pko.hpp"
#include "tof_slam/common/lie/se3.hpp"
#include "tof_slam/common/types/point_types.hpp"

namespace tof_slam {
namespace lwo {

// ---------------------------------------------------------------------------
// LwoEstimator
// ---------------------------------------------------------------------------

class LwoEstimator {
 public:
  struct LwoTrustMetrics {
    int corr_count = 0;
    float lidar_rms_prior = 0.0f;
    float lidar_rms_post = 0.0f;
    float lidar_rms_ratio = 1.0f;
    float wheel_cost_prior = 0.0f;
    float wheel_cost_post = 0.0f;
    float ground_cost_prior = 0.0f;
    float ground_cost_post = 0.0f;
    float prior_cost_post = 0.0f;
    float pose_delta_norm = 0.0f;
    float yaw_delta_abs = 0.0f;
    float obs_min_eig = 0.0f;
    float obs_cond_ratio = 0.0f;
    float trust_score = 0.0f;
    float trust_window_mean = 0.0f;
    int accept_window_hits = 0;
    bool objective_improved = false;
    bool wheel_safe = false;
    bool ground_safe = false;
    bool rms_safe = false;
    bool window_consistent = false;
    bool accepted = false;
    bool blended = false;
    float blend_alpha = 0.0f;
    bool map_commit_allowed = false;
    int reject_reason_code = 0;
    int accept_reason_code = 0;
    int commit_reason_code = 0;
  };

  struct LwoAcceptanceResult {
    LwoState output_state;
    LwoTrustMetrics metrics;
  };

  struct Config {
    // Wheel propagator
    WheelPropagatorConfig wheel_propagator;

    // Ground constraint
    GroundConstraintConfig ground_constraint;

    // Wheel measurement
    WheelMeasurementConfig wheel_measurement;

    // IEKF
    LwoIekfConfig iekf;

    // Scan preprocessing
    int stride = 4;
    float voxel_leaf_size = 0.5f;
    float min_range = 3.0f;
    float max_range = 100.0f;

    // Surfel map (long-term, all states)
    core::SurfelMapConfig surfel_map;

    // Outer loop
    int max_outer_iters = 4;

    // Minimum correspondences to perform IEKF update.
    // Below this, skip IEKF and inflate heading covariance to enable recovery.
    int min_correspondences = 50;
    // Heading covariance inflation rate during low-correspondence periods.
    // P_yaw += inflation_rate * dt  (rad^2/frame)
    float low_corr_heading_inflation = 0.001f;

    // IEKF result rejection: if total correction across all inner iterations
    // exceeds these limits, reject IEKF and keep wheel-only state.
    float iekf_max_total_pos_correction = 0.10f;   // meters (10cm)
    float iekf_max_total_rot_correction = 0.087f;   // radians (~5 deg)

    // PKO
    core::PkoConfig pko;

    // Extrinsics (LiDAR -> body frame)
    core::Se3 T_body_lidar;  // default identity

    // Undistortion
    bool enable_undistortion = true;
    float scan_duration = 0.1f;

    // Trajectory history limit
    size_t max_trajectory_size = 10000;

    // Position correction bias estimation (compensates extrinsic rotation error).
    // Tracks EMA of IEKF position corrections in body frame and subtracts
    // the estimated systematic bias.
    bool   enable_pos_bias_est         = false;
    float  pos_bias_ema_alpha          = 0.02f;   // EMA smoothing (50-frame time constant)
    int    pos_bias_warmup_frames      = 50;       // Accumulate without subtraction
    float  pos_bias_outlier_threshold  = 0.01f;   // Skip corrections > 1cm/frame
    float  pos_bias_correction_gain    = 1.0f;    // 0-1, how aggressively to subtract
    bool   pos_bias_enable_z           = false;    // z-axis (usually handled by ground)

    // Frozen Anchor Map for heading drift correction on return leg.
    // Builds a separate surfel map from the first N frames, freezes it,
    // and matches against it when the robot returns near the starting position.
    bool   anchor_enable              = false;
    int    anchor_build_frames        = 30;       // Frames to build anchor map (default: 30 = 3s)
    float  anchor_proximity_radius    = 2.0f;     // Distance threshold to origin (m)
    int    anchor_min_frame           = 100;      // Don't trigger until this frame
    int    anchor_min_overlap_corrs   = 15;       // Min anchor correspondences for validity
    float  anchor_noise_std           = 0.5f;     // Anchor correspondence noise sigma (m)
    float  anchor_p_yaw_inflate       = 0.01f;   // Inflate P_yaw when anchor active (rad^2)
    float  anchor_max_residual_rms    = 0.15f;   // Reject if anchor residual RMS exceeds this (m)
    float  anchor_residual_threshold  = 1.0f;    // Residual validation: post_rss < pre_rss * threshold (1.0=any improvement)
    float  anchor_max_cumulative_yaw  = 0.03f;   // Max |cumulative yaw correction| (rad, ~1.7°)

    // Aggressive P_yaw inflation on return leg.
    // When return leg detected, inflate P_yaw to tracked outward maximum
    // so anchor yaw correction gets meaningful Kalman gain.
    bool   anchor_dynamic_p_yaw      = false;     // Enable dynamic P_yaw inflation
    float  anchor_p_yaw_inflate_min   = 0.001f;   // Floor for dynamic inflation (rad^2)
    float  anchor_p_yaw_inflate_scale = 1.0f;     // Scale factor for tracked max P_yaw

    // Anchor consistency gate: track EMA of signed corrections.
    // Only apply when EMA magnitude exceeds threshold (consistent drift signal).
    // Protects against noise injection when heading is already accurate.
    float  anchor_consistency_alpha    = 0.3f;    // EMA smoothing for signed corrections
    float  anchor_consistency_threshold = 0.003f; // Min EMA magnitude to apply (rad, ~0.17 deg)

    // Post-IEKF yaw-only correction using anchor map.
    // Replaces the IEKF augmentation approach (which was harmful).
    // Runs after the main IEKF update; applies a weighted-least-squares
    // yaw correction derived from anchor surfel plane constraints.
    float  anchor_yaw_gain            = 0.5f;     // [0,1] correction gain
    float  anchor_yaw_max_correction  = 0.0524f;  // max correction in rad (~3 deg)
    float  anchor_yaw_B_min           = 1.0f;     // min information threshold
    int    anchor_yaw_min_corrs       = 10;        // min anchor correspondences

    // Anchor-hybrid IEKF: blend frozen anchor map correspondences into the
    // IEKF outer loop alongside live map correspondences.
    // When the robot is within anchor_proximity_radius, anchor correspondences
    // are merged with live correspondences at a distance-weighted ratio:
    //   blend = anchor_iekf_blend_ratio * (1 - dist/proximity_radius)
    // At the origin: blend = anchor_iekf_blend_ratio (e.g. 50% anchor).
    // At the boundary: blend = 0% anchor (full live map).
    // This gives the IEKF a stable heading reference from the frozen map
    // without replacing live correspondences, preventing the self-confirming
    // correction loop caused by the live map co-drifting with the state.
    bool   anchor_iekf_blend_enable   = false;    // Enable anchor-IEKF hybrid mode
    float  anchor_iekf_blend_ratio    = 0.5f;     // Max fraction of anchor corrs at center

    // Velocity-adaptive LiDAR noise scaling.
    // sigma_lidar_eff = lidar_noise_std * (1 + vel_noise_scale / max(|vx|, vel_noise_min_speed))
    // Higher scale → more deference to wheel odom at low speeds.
    // 0.0 = disabled.
    float vel_noise_scale     = 0.0f;   // Typical: 0.05-0.2
    float vel_noise_min_speed = 0.1f;   // m/s, clamp floor to prevent division explosion

    // Wheel velocity/omega measurement in IEKF update.
    // When enabled, encoder readings act as soft constraints in the IEKF,
    // preventing LiDAR-induced scan-to-map drift from corrupting the state.
    bool enable_wheel_measurement = false;

    // No-harm acceptance gate and conservative blending.
    bool enable_no_harm_gate = true;
    bool enable_trust_blending = true;
    bool enable_quality_aware_corr = true;
    float no_harm_lambda_wheel = 2.0f;
    float no_harm_lambda_ground = 1.0f;
    float no_harm_lambda_prior = 4.0f;
    float no_harm_accept_margin = 0.0f;
    float trust_min_lidar_rms_drop_ratio = 0.90f;
    float trust_max_wheel_cost_increase = 1.5f;
    float trust_max_ground_cost_increase = 1.5f;
    float trust_wheel_cost_floor = 1e-3f;
    float trust_ground_cost_floor = 1.0f;
    float trust_wheel_safe_abs_increase = 1e-3f;
    float trust_ground_safe_abs_increase = 10.0f;
    float trust_min_score_for_accept = 0.62f;
    float trust_min_score_for_commit = 0.65f;
    float trust_min_score_for_blend = 0.35f;
    float trust_blend_alpha_min = 0.0f;
    float trust_blend_alpha_max = 0.8f;
    int accept_window_size = 5;
    int accept_min_consistent_frames = 3;
    float accept_max_mean_rms_ratio = 0.98f;
    float accept_max_mean_pose_delta = 0.03f;
    float accept_max_mean_yaw_delta = 0.02f;
    int low_corr_blend_margin = 2;
    float low_corr_blend_alpha_scale = 0.25f;
    float inconsistent_blend_alpha_scale = 0.5f;
    float corr_max_plane_distance = 0.20f;
    int corr_max_per_l1 = 2;
    int corr_min_centroids = 4;
    float corr_hybrid_planarity_threshold = 0.12f;
    bool map_commit_require_accept = true;
    bool map_commit_require_rms_drop = true;
    float map_commit_max_lidar_rms = 0.10f;
    float map_commit_max_pose_delta = 0.05f;
    float map_commit_max_yaw_delta = 0.03f;

    // Bootstrap: unconditionally commit map for the first N frames so the
    // surfel map can grow from zero even when trust metrics are undefined.
    int bootstrap_frames = 30;

    // Debug logging for degradation monitoring
    bool enable_debug_log = false;

    // System resource profiling (timing + CPU/RSS per lidar frame)
    bool check_usage = false;
  };

  LwoEstimator();
  explicit LwoEstimator(const Config& config);

  /// Feed a wheel odometry measurement (propagation step).
  ///
  /// @param vx       Forward velocity from encoder (m/s), body frame.
  /// @param omega_z  Yaw rate from encoder (rad/s), body frame.
  /// @param timestamp  Absolute timestamp (seconds).
  void feed_wheel(float vx, float omega_z, double timestamp);

  /// Feed a wheel odometry measurement using delta pose from consecutive odom
  /// messages.  Bypasses re-integration by applying the firmware's double64
  /// integrated pose diff directly to state_, eliminating ~1cm drift.
  ///
  /// @param vx            Forward velocity from encoder (m/s), body frame.
  ///                      Used only for IEKF wheel measurement and velocity
  ///                      state update — NOT re-integrated for position.
  /// @param omega_z       Yaw rate from encoder (rad/s), body frame.
  ///                      Used only for IEKF wheel measurement.
  /// @param timestamp     Absolute timestamp (seconds).
  /// @param delta_pos_body  Position delta in previous body frame (meters).
  ///                        Derived from consecutive firmware odom poses.
  /// @param delta_yaw     Yaw delta (radians), normalized to [-pi, pi].
  void feed_wheel_delta(float vx, float omega_z, double timestamp,
                        const Eigen::Vector3f& delta_pos_body, float delta_yaw);

  /// Feed a LiDAR scan.  Returns true if the IEKF update was performed.
  bool feed_lidar(const core::PointCloud& scan, double timestamp);

  // ---------------------------------------------------------------------------
  // Per-frame diagnostics snapshot (returned by feed_lidar)
  // ---------------------------------------------------------------------------
  struct FrameDiagnostics {
    int    frame_id          = 0;
    double timestamp         = 0.0;

    // --- Correspondence quality ---
    int    corr_count        = 0;    // # correspondences found (outer=0)
    float  corr_planarity_avg = 0.0f; // mean planarity of correspondences
    float  corr_res_rms      = 0.0f; // point-to-plane residual RMS after IEKF
    float  lidar_rms_prior   = 0.0f;
    float  lidar_rms_post    = 0.0f;
    float  trust_score       = 0.0f;
    float  trust_window_mean = 0.0f;
    int    accept_window_hits = 0;

    // --- IEKF convergence ---
    bool   iekf_converged    = false;
    int    iekf_iters        = 0;
    float  iekf_pos_delta    = 0.0f; // ||delta_pos|| after IEKF
    float  iekf_rot_delta    = 0.0f; // ||delta_rot|| (rad) after IEKF

    // --- Degradation flags ---
    bool   low_corr          = false; // corr_count < min_correspondences
    bool   skip_map_update   = false; // map was NOT updated this frame
    int    low_corr_consecutive = 0;  // consecutive low-corr frames
    bool   accepted_lidar_update = false;
    bool   blended_lidar_update = false;
    float  blend_alpha = 0.0f;
    bool   map_commit_allowed = false;
    bool   objective_improved = false;
    bool   wheel_safe = false;
    bool   ground_safe = false;
    bool   rms_safe = false;
    bool   window_consistent = false;
    int    reject_reason_code = 0;
    int    accept_reason_code = 0;
    int    commit_reason_code = 0;

    // --- State ---
    float  vel_mag           = 0.0f;  // ||velocity||
    float  wheel_scale       = 1.0f;
    float  gyro_bias_z       = 0.0f;

    // --- Covariance diagonal (key elements) ---
    float  P_rot_x           = 0.0f;
    float  P_rot_y           = 0.0f;
    float  P_rot_z           = 0.0f;  // yaw uncertainty
    float  P_pos_x           = 0.0f;
    float  P_pos_y           = 0.0f;
    float  P_pos_z           = 0.0f;
    float  P_vel_x           = 0.0f;
    float  P_vel_y           = 0.0f;
    float  P_scale           = 0.0f;

    // --- Wheel measurement diagnostics ---
    float  wv_residual_vx    = 0.0f;  // post-IEKF vx residual (m/s)
    float  wv_residual_omega = 0.0f;  // post-IEKF omega residual (rad/s)
    float  wv_info_vx        = 0.0f;  // information weight for vx
    float  wv_info_omega     = 0.0f;  // information weight for omega
    float  wheel_cost_prior  = 0.0f;
    float  wheel_cost_post   = 0.0f;
    float  ground_cost_prior = 0.0f;
    float  ground_cost_post  = 0.0f;
    float  pose_delta_norm   = 0.0f;
    float  yaw_delta_abs     = 0.0f;
    float  obs_min_eig       = 0.0f;
    float  obs_cond_ratio    = 0.0f;

    // --- Map stats ---
    int    map_l0            = 0;
    int    map_l1            = 0;
  };

  // ---------------------------------------------------------------------------
  // Per-frame resource usage snapshot (populated when check_usage=true)
  // ---------------------------------------------------------------------------
  struct FrameUsage {
    int    frame_id          = 0;
    double timestamp         = 0.0;

    // Per-module timing (ms)
    float  total_ms          = 0.0f;
    float  preprocess_ms     = 0.0f;
    float  corr_find_ms      = 0.0f;  // all outer iters summed
    float  anchor_corr_ms    = 0.0f;
    float  iekf_ms           = 0.0f;  // all outer iters summed
    float  anchor_yaw_ms     = 0.0f;
    float  map_update_ms     = 0.0f;

    // IEKF sub-module timing (ms, all outer iters summed)
    float  iekf_jacobian_ms  = 0.0f;
    float  iekf_build_ms     = 0.0f;
    float  iekf_solve_ms     = 0.0f;

    // Wheel propagator timing (accumulated between lidar frames)
    float  wheel_ms          = 0.0f;

    // System resources
    float  cpu_percent       = 0.0f;
    float  rss_mb            = 0.0f;
    int    queue_depth       = 0;

    // OGM timing (filled by SlamNode)
    float  ogm_input_transform_ms  = 0.0f;
    float  ogm_local_grid_ms       = 0.0f;
    float  ogm_global_assemble_ms  = 0.0f;
    float  ogm_pgo_rebuild_ms      = 0.0f;

    // Backend timing (filled by SlamNode from BackendUsageSnapshot)
    float  backend_add_keyframe_ms = 0.0f;
    float  backend_loop_detect_ms  = 0.0f;
    float  backend_gicp_ms         = 0.0f;
    float  backend_pgo_ms          = 0.0f;
    float  backend_total_ms        = 0.0f;
    int    backend_queue_depth     = 0;
    int    backend_candidates_tested = 0;
    int    backend_loops_accepted  = 0;

    // Iteration / size stats
    int    outer_iters       = 0;
    int    total_inner_iters = 0;
    int    corr_count        = 0;
    int    scan_points       = 0;
  };

  /// Thread-safe snapshot of the current state.
  LwoState current_state() const;

  /// Get the current body-lidar extrinsic transform (thread-safe).
  core::Se3 current_body_lidar_extrinsic() const { return T_body_lidar_current_; }

  /// Diagnostics from the most recent feed_lidar() call (thread-safe).
  FrameDiagnostics last_diagnostics() const;

  /// Usage from the most recent feed_lidar() call (thread-safe).
  FrameUsage last_usage() const;

  /// Set queue depth (called by SlamNode before feed_lidar).
  void set_queue_depth(int depth);

  /// Accumulate wheel propagation time (called inside feed_wheel_delta/feed_wheel).
  void accumulate_wheel_ms(float ms);

  /// Whether the estimator has been initialized (first wheel message received).
  bool initialized() const;

  /// Number of LiDAR frames processed so far.
  int frame_count() const;

  /// Snapshot of all currently valid surfels (thread-safe).
  std::vector<core::Surfel> map_surfels() const;

  /// Direct access to the surfel map (NOT thread-safe).
  const core::SurfelMap& surfel_map() const;

  /// Processed scan from the most recent feed_lidar() call (lidar frame, after
  /// stride/voxel/undistortion). Useful for comparing against raw input cloud.
  core::PointCloud last_processed_scan() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return last_processed_scan_;
  }

  /// Reset all internal state.
  void reset();

 private:
  /// Preprocess a raw scan: stride -> voxel grid -> undistort -> range filter.
  core::PointCloud preprocess_scan(const core::PointCloud& scan, double timestamp);

  /// Rebuild T_body_lidar_current_ from config_.T_body_lidar plus the
  /// ext_delta_yaw and ext_delta_xy fields from the current state.
  /// Called after each IEKF update when enable_ext_calibration is true.
  void update_extrinsic_from_state();

  /// Collect /proc-based CPU% and RSS into last_usage_ (check_usage only).
  void collect_system_resources();

  /// Transform a scan from LiDAR frame to world frame using current state.
  core::PointCloud transform_to_world(const core::PointCloud& scan) const;

  /// Find correspondences using current LWO state (adapter for LIO's finder).
  std::vector<core::Correspondence> find_correspondences_lwo(
      const core::PointCloud& scan) const;

  float compute_lidar_rms(const LwoState& state,
                          const std::vector<core::Correspondence>& corrs) const;
  float compute_wheel_cost(const LwoState& state,
                           float vx_enc,
                           float omega_z_enc,
                           float omega_z_b) const;
  float compute_ground_cost(const LwoState& state) const;
  float compute_prior_cost(const LwoState& x_wheel,
                           const LwoState& x_other,
                           float* pose_delta_norm = nullptr,
                           float* yaw_delta_abs = nullptr) const;
  void compute_observability_metrics(const LwoState& state,
                                     const std::vector<core::Correspondence>& corrs,
                                     float* obs_min_eig,
                                     float* obs_cond_ratio) const;
  LwoTrustMetrics evaluate_trust_metrics(const LwoState& x_wheel,
                                         const LwoState& x_lidar,
                                         const std::vector<core::Correspondence>& corrs,
                                         float vx_enc,
                                         float omega_z_enc,
                                         float omega_z_b) const;
  LwoAcceptanceResult accept_or_blend_update(
      const LwoState& x_wheel,
      const LwoState& x_lidar,
      const LwoTrustMetrics& metrics);

  /// Find correspondences against the frozen anchor map.
  /// Returns empty if anchor map is not frozen or robot is not in proximity.
  std::vector<core::Correspondence> find_anchor_correspondences(
      const core::PointCloud& scan) const;

  /// Post-IEKF yaw-only correction using the frozen anchor map.
  /// Runs after the main IEKF update; applies a WLS yaw delta derived
  /// from anchor surfel plane residuals with Tukey bisquare weighting.
  void apply_anchor_yaw_correction(const core::PointCloud& scan);

  // --- Configuration -------------------------------------------------------
  Config config_;

  // --- Online extrinsic calibration ----------------------------------------
  /// Corrected extrinsic: config_.T_body_lidar + current state ext_delta fields.
  /// Initialized to config_.T_body_lidar; updated after each IEKF update when
  /// enable_ext_calibration is true.  Used everywhere instead of config_.T_body_lidar.
  core::Se3 T_body_lidar_current_;

  // --- Core state ----------------------------------------------------------
  mutable std::mutex mutex_;
  LwoState state_;
  bool initialized_ = false;
  int frame_count_ = 0;
  bool first_lidar_frame_ = true;
  double last_wheel_time_ = 0.0;
  double last_lidar_time_ = 0.0;

  // Latest wheel encoder readings (cached for IEKF wheel measurement).
  float last_vx_enc_ = 0.0f;
  float last_omega_z_enc_ = 0.0f;

  // --- Sub-components ------------------------------------------------------
  WheelPropagator wheel_propagator_;
  GroundConstraint ground_constraint_;
  WheelMeasurement wheel_measurement_;
  LwoIekfUpdater iekf_updater_;
  core::SurfelMap surfel_map_;
  core::Pko pko_;

  // --- Undistortion state history ------------------------------------------
  std::deque<core::StampedPose> state_history_;

  struct AcceptanceHistoryEntry {
    float trust_score = 0.0f;
    float lidar_rms_ratio = 1.0f;
    float pose_delta_norm = 0.0f;
    float yaw_delta_abs = 0.0f;
    bool objective_improved = false;
    bool wheel_safe = false;
    bool ground_safe = false;
    bool rms_nonworsening = false;
    bool window_consistent = false;
  };
  std::deque<AcceptanceHistoryEntry> accept_history_;

  // --- Path length tracking (for path_length/gt ratio monitoring) ----------
  float cumulative_path_length_ = 0.0f;
  Eigen::Vector3f prev_position_ = Eigen::Vector3f::Zero();

  // --- Low correspondence tracking ----------------------------------------
  int low_corr_frames_ = 0;  // Consecutive frames with too few correspondences
  int last_corr_count_ = 0;  // Correspondence count from last IEKF outer loop

  // --- Per-frame diagnostics -----------------------------------------------
  FrameDiagnostics last_diag_;  // Written by feed_lidar, read by slam_node
  core::PointCloud  last_processed_scan_;  // Processed scan (after stride/voxel/undistort)

  // --- Per-frame resource usage (check_usage=true) -------------------------
  FrameUsage last_usage_;
  int pending_queue_depth_ = 0;
  float wheel_ms_accumulator_ = 0.0f;

  // CPU tracking (for delta computation)
  long prev_cpu_ticks_ = 0;
  std::chrono::steady_clock::time_point prev_cpu_wall_;
  bool cpu_tracking_initialized_ = false;

  // --- Position correction bias estimation --------------------------------
  Eigen::Vector3f pos_bias_ema_ = Eigen::Vector3f::Zero();  // Body-frame bias EMA
  int pos_bias_sample_count_ = 0;  // Number of valid samples accumulated

  // --- Debug throttle counters --------------------------------------------
  int wheel_debug_counter_ = 0;      // Throttle wheel debug log (every 1000th)

  // --- Frozen Anchor Map (heading drift correction on return leg) ---------
  core::SurfelMap anchor_map_;
  bool anchor_map_frozen_ = false;
  Eigen::Vector3f anchor_origin_ = Eigen::Vector3f::Zero();

  // --- Extended Reference Map tracking ------------------------------------
  float max_dist_from_origin_ = 0.0f;     // Max 2D distance from anchor origin
  float outward_max_p_yaw_ = 0.0f;         // Max P_yaw during outward leg

  // --- Anchor correction consistency tracking ---
  float anchor_correction_ema_ = 0.0f;     // EMA of signed raw yaw corrections
  int   anchor_correction_count_ = 0;       // Number of valid corrections seen
  float cumulative_anchor_yaw_ = 0.0f;     // Running sum of applied anchor yaw corrections (rad)
  bool  anchor_corrected_this_frame_ = false;  // True if anchor yaw correction was applied this frame
  bool  was_outside_anchor_ = false;           // Track if robot left anchor zone (for per-visit yaw reset)
};

}  // namespace lwo
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_W_ESTIMATOR_LWO_ESTIMATOR_HPP_
