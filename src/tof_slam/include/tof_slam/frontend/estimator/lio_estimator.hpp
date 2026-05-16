// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// lio_estimator.hpp — Thin orchestrator composing all core LIO components.
//
// Replaces the 1314-line Estimator "God class" with a coordinator that
// owns and delegates to: IMU propagator, gravity initializer, surfel map,
// correspondence finder, IEKF updater, motion undistorter, PKO, and filters.
//
// No ROS dependency.  Thread-safe state access via mutex.

#ifndef TOF_SLAM_FRONTEND_ESTIMATOR_LIO_ESTIMATOR_HPP_
#define TOF_SLAM_FRONTEND_ESTIMATOR_LIO_ESTIMATOR_HPP_

#include <chrono>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include "tof_slam/frontend/estimator/diagnostics_monitor.hpp"
#include "tof_slam/frontend/estimator/gravity_init.hpp"
#include "tof_slam/frontend/estimator/iekf_updater.hpp"
#include "tof_slam/frontend/estimator/imu_preintegration.hpp"
#include "tof_slam/frontend/estimator/fixed_lag_smoother.hpp"
#include "tof_slam/frontend/estimator/motion_undistort.hpp"
#include "tof_slam/common/lie/se3.hpp"
#include "tof_slam/frontend/map/surfel_map.hpp"
#include "tof_slam/frontend/map/point_voxel_map.hpp"
#include "tof_slam/frontend/policy/scene_classifier.hpp"
#include <pcl/point_types.h>
#include "tof_slam/frontend/robust/pko.hpp"
#include "tof_slam/common/types/imu_types.hpp"
#include "tof_slam/common/types/point_types.hpp"
#include "tof_slam/common/types/state.hpp"

namespace tof_slam {
namespace core {

// ---------------------------------------------------------------------------
// PreprocessTiming — sub-module timing for preprocess_scan
// ---------------------------------------------------------------------------
struct PreprocessTiming {
  float stride_ms = 0.0f;
  float voxel_ms = 0.0f;
  float undistort_ms = 0.0f;
  float range_ms = 0.0f;
  int n_raw = 0;
  int n_after_stride = 0;
  int n_after_voxel = 0;
  int n_processed = 0;
};

// ---------------------------------------------------------------------------
// IekfLoopResult — output of run_iekf_loop, consumed by update_map / logging
// ---------------------------------------------------------------------------
struct IekfLoopResult {
  bool converged = false;
  int max_degen = 0;
  int total_corrs = 0;
  int total_outer = 0;
  int total_inner = 0;
  float res_mean = 0.0f;
  float res_rms = 0.0f;
  float eigenvalues[6] = {};
  int num_degen_trans_dirs = 0;
  Eigen::Vector3f degen_trans_dirs[3] = {
      Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()};
  /// Eigenvalue ratio λ₀/λ₅ for Eq.(4) severity scaling.
  float eigenvalue_ratio = 0.0f;

  // S12-B.A.3 DG-A: per-channel Schur signature (terminal IEKF iter).
  // Index: 0=L1, 1=L2, 2=full(joint). Zero when IekfConfig::dg_a_enable=false.
  float dg_a_rho[3] = {0.0f, 0.0f, 0.0f};
  Eigen::Vector3f dg_a_d_trans[3] = {
      Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()};
  float dg_a_cos_agree[3] = {0.0f, 0.0f, 0.0f};
  int dg_a_n_corr[3] = {0, 0, 0};
  // Timing accumulators (only meaningful when enable_debug_timing=true)
  float corr_find_ms = 0.0f;
  float iekf_inner_ms = 0.0f;
  float jacobian_ms = 0.0f;
  float huber_pko_ms = 0.0f;
  float build_info_ms = 0.0f;
  float solve_ms = 0.0f;
};

// ---------------------------------------------------------------------------
// LioEstimator
// ---------------------------------------------------------------------------

class LioEstimator {
 public:
  struct Config {
    // Process noise standard deviations
    float gyro_noise_std = 0.01f;
    float acc_noise_std = 0.1f;
    float gyro_bias_noise_std = 0.0001f;
    float acc_bias_noise_std = 0.001f;
    float gravity_noise_std = 0.001f;

    // Scan preprocessing
    int stride = 4;
    float voxel_leaf_size = 0.4f;
    float min_range = 0.5f;
    float max_range = 50.0f;

    // IEKF
    IekfConfig iekf;

    // Surfel map
    SurfelMapConfig surfel_map;

    // Correspondence finder mode: hybrid_select is the only production mode.
    // Queries both surfel and PVMap per point, selects better plane by
    // normal_sigma2 quality metric.

    // PointVoxelMap (alternative to surfel map for correspondence finding)
    bool use_point_voxel_map = false;  ///< Switch to PointVoxelMap mode
    PointVoxelMapConfig point_voxel_map;
    int pvmap_k_neighbors = 5;           ///< kNN count for plane fitting
    float pvmap_planarity_threshold = 0.15f;  ///< Max planarity for valid planes

    /// PVMap sigma2 scale in hybrid correspondence selection.
    /// Default 2.0 = surfel-primary (PVMap gets 2x penalty, good for 360° FOV).
    /// 1.0 = fair competition (recommended for sparse 70° FOV like Avia).
    float pvmap_sigma2_scale = 2.0f;

    /// Point-to-Distribution (P2D): iG-LIO style voxel Gaussian residuals.
    bool enable_point_to_distribution = false;
    float p2d_chi2_threshold = 7.815f;
    float p2d_cov_reg_eps = 1e-3f;

    /// CSCF (Continuous Surfel Correspondence Field): kernel-weighted L1
    /// surfel interpolation for smooth correspondence fields.
    bool enable_cscf = false;
    float cscf_kernel_bandwidth = 0.25f;

    // PKO
    bool enable_pko = false;
    PkoConfig pko;

    // Extrinsics (LiDAR -> body/IMU frame)
    Se3 T_body_lidar;  // default identity

    // Undistortion
    bool enable_undistortion = true;
    float scan_duration = 0.1f;

    // Trajectory history limit
    size_t max_trajectory_size = 10000;

    // Max correspondences for IEKF (0 = no limit, use all).
    // Subsamples correspondences to cap IEKF cost while keeping full scan for map.
    int max_correspondences = 0;

    // Max point-to-plane distance for correspondence rejection (0 = no limit).
    float max_plane_distance = 0.0f;

    /// Configurable divisor for the adaptive correspondence threshold formula:
    ///   adaptive_thresh = min(max_plane_distance, sqrt(range) / divisor)
    /// Default 9.0 reproduces FAST-LIO2-era outdoor behavior.
    /// Indoor: 16-20 tightens the gate for short-range (2-8m) scenes.
    float adaptive_threshold_divisor = 9.0f;

    // Max correspondences per L1 voxel (0 = no limit).
    int max_corr_per_l1 = 0;

    /// Maximum OpenMP threads for correspondence finding.
    /// 0 = use OMP_NUM_THREADS (default), >0 = cap at this value.
    int cf_omp_max_threads = 4;

    // Debug: enable detailed sub-module timing to CSV
    bool enable_debug_timing = false;

    /// Decrease L0 EMA alpha during degenerate frames (map protection).
    /// When degeneracy is detected, the pose estimate is unreliable in
    /// degenerate directions. Lower alpha preserves existing (presumably
    /// correct) centroids by giving less weight to new (potentially wrong)
    /// points. This prevents cascading map contamination.
    /// alpha_eff = alpha_min / (1 + degeneracy_alpha_scale * n_degen_dirs)
    bool enable_degeneracy_adaptive_alpha = false;
    float degeneracy_alpha_scale = 1.0f;

    // --- Velocity Damping ---
    /// Post-IEKF graduated velocity damping based on correspondence count.
    /// When correspondences are sparse, velocity may be poorly constrained;
    /// damping prevents runaway drift.  However, this operates outside the
    /// IEKF probabilistic framework (modifies state mean without updating
    /// covariance), which can cause filter inconsistency during fast motion.
    /// Disable (false) to rely solely on IEKF velocity pseudo-obs for
    /// velocity regulation.  Default true for backward compatibility.
    bool enable_velocity_damping = true;

    // --- DDPO (Degeneracy-Directed PVMap Override) ---
    /// When degeneracy is detected in a translation direction, force PVMap
    /// for correspondences whose surfel normal aligns with that direction.
    /// Breaks the EMA centroid feedback loop (b∞ = δ/α) that causes
    /// systematic drift in corridors.
    bool enable_degen_pvmap_override = false;
    float degen_pvmap_cos_threshold = 0.5f;  ///< Hard override: |cos| >= this → force PVMap

    // --- DARBF Persistence Filter ---
    /// Minimum consecutive degenerate frames before DARBF activates.
    /// Prevents brief (1-5 frame) degeneracy episodes from triggering freeze
    /// in transient-degen sequences like Dark01. 0 = immediate (no filter).
    int degen_freeze_min_persist = 0;

    // --- Surfel Keyframe Gate (Task #133 Iter 3) ---
    /// Gate surfel map insertion on translation+rotation keyframe predicate.
    /// When false, codepath is bit-identical to pre-gate (every-frame insertion).
    /// When true, suppresses surfel_map_.update() on near-stationary frames,
    /// reducing EMA centroid contamination from closely-spaced drifted poses.
    /// Mirrors iG-LIO's 0.5m keyframe gate architecture.
    bool enable_surfel_keyframe_gate = false;
    /// Translation threshold (m). iG-LIO uses 0.5m; start conservative at 0.30.
    double surfel_kf_trans_thresh_m = 0.30;
    /// Rotation threshold (rad). Triggers new geometry observation on rotation.
    double surfel_kf_rot_thresh_rad = 0.10;
    /// Always insert during first N frames for map bootstrap.
    int surfel_kf_warmup_frames = 20;

    // --- Fixed-Lag Smoother (Tier 4 NTU VIRAL) ---
    /// 2-frame velocity-coupled Gauss-Newton smoother with IMU preintegration
    /// cross-coupling.  The IMU preintegration factor provides the sole genuine
    /// cross-frame coupling missing from the single-scan IEKF.
    bool enable_fixed_lag_smoother = false;
    /// Minimum correspondences for FLS (same semantics as backward_min_corrs).
    int fls_min_correspondences = 50;
    FlsConfig fls;

    // --- Output paths (empty = disabled) ---
    std::string diagnostics_log_path;  ///< Path for diagnostics CSV. Empty = disabled.
    std::string timing_log_path;       ///< Path for timing CSV. Empty = disabled.
    std::string traj_csv_path;         ///< Path for trajectory CSV. Empty = disabled.

  };

  LioEstimator();
  explicit LioEstimator(const Config& config);
  ~LioEstimator();

  /// Initialize with gravity from stationary IMU buffer.
  /// Returns true on success.
  bool initialize(const std::vector<ImuMeasurement>& imu_buffer);

  /// Initialize from a pre-computed GravityInitResult (e.g., from ImuAdapter).
  /// Returns true on success.
  bool initialize(const GravityInitResult& result);

  /// Feed a single IMU measurement (propagation step).
  void feed_imu(const ImuMeasurement& imu);

  /// Feed a LiDAR scan.  Returns true if the IEKF update was performed
  /// (false on first frame or if not initialized).
  bool feed_lidar(const PointCloud& scan, double timestamp);

  /// Thread-safe snapshot of the current state.
  LioState current_state() const;

  /// Whether gravity initialization has been completed.
  bool initialized() const;

  /// Number of LiDAR frames processed so far.
  int frame_count() const;

  /// Snapshot of all currently valid surfels (thread-safe).
  std::vector<Surfel> map_surfels() const;

  /// Direct access to the surfel map (NOT thread-safe -- caller must
  /// guarantee exclusive access when using this).
  const SurfelMap& surfel_map() const;

  /// Reset all internal state (map, covariance, counters).
  void reset();

  /// R9 C2': enable/disable R-A two-stage scene classifier at runtime.
  /// Per-seq YAMLs set this false (their tuned params shouldn't be overridden
  /// by lock-time templates). Unified avia_outdoor.yaml leaves default true.
  void set_classifier_enable(bool enable) noexcept {
    classifier_cfg_.enable = enable;
  }

  /// R0.9 H3b: arm OUTDOOR_DRIFT Stage A discriminator (Avia outdoor only).
  /// Mirrors set_classifier_enable pattern. Per-class GUARD; default false.
  /// Set true ONLY by avia_outdoor.yaml's `frontend_is_avia_outdoor: true`.
  void set_is_avia_outdoor(bool flag) noexcept {
    classifier_cfg_.is_avia_outdoor = flag;
  }

 private:
  /// Build the 18x18 process noise matrix from config noise stds.
  StateCovariance build_process_noise() const;

  /// Preprocess a raw scan: stride -> voxel grid -> undistort -> range filter.
  PointCloud preprocess_scan(const PointCloud& scan, double timestamp);

  /// Transform a scan from LiDAR frame to world frame using current state.
  PointCloud transform_to_world(const PointCloud& scan) const;

  /// Run IEKF outer loop: correspondence finding, IEKF update, velocity
  /// damping, convergence check.  Modifies state_ in place.
  IekfLoopResult run_iekf_loop(const PointCloud& processed, double timestamp);

  /// Update surfel map and PVMap after IEKF.  Handles alpha modulation
  /// (degeneracy-adaptive), surfel update,
  /// DARBF freeze, and PVMap update.
  void update_map(const PointCloud& world_cloud,
                  const IekfLoopResult& iekf_result);

  /// Fixed-Lag Smoother (Tier 4): 2-frame velocity-coupled Gauss-Newton
  /// with IMU preintegration cross-coupling.
  /// @param curr_corrs Correspondences from the forward IEKF (used for
  ///                   diagnostic comparison only; not added to FLS Hessian
  ///                   to avoid double-counting with P_k prior).
  void run_fixed_lag_smoother(const std::vector<Correspondence>& curr_corrs);

  /// Surfel keyframe gate decision categories (Task #133 Iter 3).
  enum class SurfelInsertReason {
    Off,        ///< gate disabled (bit-identical to pre-gate)
    ColdStart,  ///< first surfel insertion (no anchor recorded yet)
    Warmup,     ///< frame_count_ < surfel_kf_warmup_frames
    Keyframe,   ///< translation or rotation threshold exceeded
    Skip,       ///< stationary frame, gate suppresses insertion
  };

  struct SurfelInsertDecision {
    bool insert;
    SurfelInsertReason reason;
  };

  /// Surfel keyframe gate predicate (Task #133 Iter 3).
  /// Pure / no side effects — caller advances anchor state.
  /// When config_.enable_surfel_keyframe_gate=false, returns
  /// {true, Off} so behavior is bit-identical to pre-gate.
  SurfelInsertDecision should_insert_surfel() const noexcept;

  /// Feed diagnostics monitor and write timing CSV.
  void log_frame_diagnostics(
      const IekfLoopResult& iekf_result,
      const PointCloud& processed,
      double timestamp,
      std::chrono::high_resolution_clock::time_point t_start,
      std::chrono::high_resolution_clock::time_point t_preprocess,
      std::chrono::high_resolution_clock::time_point t_iekf,
      std::chrono::high_resolution_clock::time_point t_map_insert,
      std::chrono::high_resolution_clock::time_point t_map);

  // --- Configuration -------------------------------------------------------
  Config config_;

  // --- Core state ----------------------------------------------------------
  mutable std::mutex mutex_;
  LioState state_;
  bool initialized_ = false;
  int frame_count_ = 0;
  bool first_lidar_frame_ = true;
  double last_imu_time_ = 0.0;
  double last_lidar_time_ = 0.0;
  float imu_acc_scale_ = 1.0f;
  Eigen::Vector3f last_corrected_position_ = Eigen::Vector3f::Zero();  ///< For inter-frame pos_delta diagnostics

  // --- T5.4-R7 Scene Classifier ---
  // Owns warmup-accumulator + locked class label. update() runs at end of
  // feed_lidar() (post-init frames only). apply_template_() runs once at lock.
  ClassifierState  classifier_state_;
  ClassifierConfig classifier_cfg_;
  /// Apply class template values via SurfelMap setters + Config mutation.
  void apply_template_(SceneClass cls);

  // DDPO: previous frame's degenerate translation directions (one-frame lag).
  int prev_frame_num_degen_trans_dirs_ = 0;
  Eigen::Vector3f prev_frame_degen_trans_dirs_[3] = {
      Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()
  };

  // Promoted from static locals for reset-safety and multi-instance support.
  int scan_diag_count_ = 0;            ///< Undistortion diag log counter (was static in preprocess_scan)
  std::mt19937 corr_shuffle_rng_{42};   ///< Correspondence shuffle RNG (was static in feed_lidar)
  std::ofstream timing_csv_;            ///< Timing CSV stream (was static in feed_lidar)
  bool timing_csv_initialized_ = false; ///< Whether timing CSV header has been written

  // DARBF persistence counter: consecutive frames with degeneracy detected.
  // DARBF freeze only activates when count >= degen_freeze_min_persist.
  int degen_persist_count_ = 0;

  // --- Surfel Keyframe Gate state (Task #133 Iter 3) -----------------------
  // Frame index of last surfel map insertion. -1 = none.
  int last_surfel_kf_frame_ = -1;
  // Pose at last surfel insertion (translation+rotation predicate).
  Eigen::Vector3f last_surfel_kf_position_ = Eigen::Vector3f::Zero();
  Eigen::Matrix3f last_surfel_kf_rotation_ = Eigen::Matrix3f::Identity();

  // --- Process noise (constant after construction) -------------------------
  StateCovariance process_noise_;

  // --- Sub-components ------------------------------------------------------
  SurfelMap surfel_map_;
  PointVoxelMap point_voxel_map_;
  Pko pko_;
  IcdrState icdr_state_;  ///< ICDR persistent state (TIP eigenvalue EMA)

  // --- Undistortion state history ------------------------------------------
  std::deque<StampedPose> state_history_;

  // --- Fixed-Lag Smoother (Tier 4) state -----------------------------------
  /// Previous frame's preprocessed+undistorted scan (body frame).
  PointCloud prev_processed_scan_;
  /// Previous frame's IEKF posterior state (after IEKF, before map update).
  LioState prev_state_posterior_;
  /// Previous frame's IEKF posterior covariance.
  StateCovariance prev_P_posterior_;
  /// Whether previous frame data is valid for FLS.
  bool prev_frame_valid_ = false;
  /// IMU measurements buffered between LiDAR frames for preintegration.
  /// Cleared after each FLS run.
  std::vector<ImuMeasurement> fls_imu_buffer_;

  // --- Debug timing (transient, set by preprocess_scan for feed_lidar) -----
  PreprocessTiming last_preprocess_timing_;

  // --- Real-time diagnostics monitor (requirements 3-2 through 3-4) --------
  DiagnosticsMonitor diagnostics_;

};

}  // namespace core
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_ESTIMATOR_LIO_ESTIMATOR_HPP_
