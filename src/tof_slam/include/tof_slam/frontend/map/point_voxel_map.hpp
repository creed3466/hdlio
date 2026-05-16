// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// point_voxel_map.hpp — Raw-point voxel map for per-query kNN plane fitting.
//
// Stores raw points (Eigen::Vector3f) in per-voxel ring buffers.
// Enables Faster-LIO / KISS-ICP style per-query kNN plane fitting,
// eliminating the surfel sharing problem of L0/L1 hierarchical maps.
//
// No ROS dependencies.  Uses tof_slam::core types throughout.

#ifndef TOF_SLAM_FRONTEND_MAP_POINT_VOXEL_MAP_HPP_
#define TOF_SLAM_FRONTEND_MAP_POINT_VOXEL_MAP_HPP_

#include <mutex>
#include <shared_mutex>
#include <vector>

#include <Eigen/Dense>

#include "tof_slam/frontend/map/voxel_key.hpp"
#include "tof_slam/common/types/point_types.hpp"
#include "unordered_dense.h"

namespace tof_slam {
namespace core {

// ---------------------------------------------------------------------------
// PointVoxelMapConfig
// ---------------------------------------------------------------------------

struct PointVoxelMapConfig {
  float voxel_size = 0.5f;          ///< Voxel edge length (metres)
  int   max_points_per_voxel = 20;  ///< Ring buffer capacity per voxel
  float max_distance = 100.0f;      ///< Pruning radius from sensor
  float distance_multiplier = 1.5f; ///< Recentering trigger ratio
  int   knn_search_half = 1;        ///< Search half-extent in voxels (1=3x3x3, 2=5x5x5)

  // --- DARBF: Degeneracy-Aware Ring Buffer Freeze ---
  /// When degeneracy is active, freeze ring buffer writes for voxels whose
  /// stored plane normal aligns with the degenerate direction. This preserves
  /// early-inserted (unbiased) anchor points for DDPO correspondences.
  bool  enable_degen_freeze = false;
  float degen_freeze_cos_threshold = 0.7f;  ///< |cos(voxel_normal, degen_dir)| threshold
  int   degen_freeze_max_frames = 200;      ///< Auto-unfreeze after this many frames

  // --- Option A: QR plane fitting (FAST-LIO2 esti_plane parity) ---
  /// Replace PCA eigendecomposition with QR linear solve for plane fitting.
  /// QR is well-determined at k=5, while PCA needs k>10 for stable normals.
  /// Includes per-neighbor 0.1m inlier check (integral to QR approach).
  bool  use_qr_plane_fit = false;

  // --- Option B: Proximity-based insertion (FAST-LIO2 map_incremental parity) ---
  /// Replace FIFO ring buffer with proximity-to-voxel-center insertion.
  /// New point replaces the farthest-from-center existing point only if
  /// it is closer to center. Preserves geometrically representative points.
  bool  use_proximity_insertion = false;

  // --- Option C: Per-voxel cached plane (iG-LIO architecture parity) ---
  /// Cache plane normal+centroid per voxel using incremental covariance
  /// (Welford). Eliminates per-query gather_knn+PCA. Correspondence finder
  /// reads cached planes from multiple neighbor voxels, providing normal
  /// diversity that constrains degenerate environments (tunnels).
  bool  use_voxel_plane_cache = false;

  /// Minimum point count in a voxel before its cached plane is considered
  /// valid for correspondence generation.
  int   voxel_plane_min_points = 5;

  /// SVD regularization: minimum eigenvalue for the smallest (normal) direction.
  /// Default 1e-3 creates 1000:1 ratio (iG-LIO style disk covariance).
  /// Larger values (e.g. 0.01, 0.1) relax regularization, preserving more
  /// axial constraint information in degenerate environments (tunnels).
  /// Set to 0.0 to disable SVD regularization (use raw covariance + epsilon).
  float svd_min_eigenvalue = 1e-3f;
};

// ---------------------------------------------------------------------------
// PlaneResult — output of per-query plane fitting
// ---------------------------------------------------------------------------

struct PlaneResult {
  Eigen::Vector3f centroid  = Eigen::Vector3f::Zero();
  Eigen::Vector3f normal    = Eigen::Vector3f::Zero();
  float           planarity = 1.0f;   ///< σ_min/σ_max [0=planar, 1=isotropic]
  float           normal_sigma2 = 0.0f;
  float           sigma_max = 0.0f;
  float           sigma_min = 0.0f;
  int             n_points  = 0;      ///< Number of kNN points used for PCA
  bool            valid     = false;
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();  ///< 3×3 covariance (for P2D)
};

// ---------------------------------------------------------------------------
// PointVoxelMap
// ---------------------------------------------------------------------------

class PointVoxelMap {
 public:
  explicit PointVoxelMap(const PointVoxelMapConfig& config = {});

  // ---- DARBF: Degeneracy freeze control ------------------------------------

  /// Set degeneracy state before calling update(). Called each frame.
  /// Pass nullptr/0 when no degeneracy is detected (unfreezes all voxels).
  void set_degen_freeze_state(const Eigen::Vector3f* degen_dirs,
                               int num_dirs, int current_frame);

  /// Adaptive SVD: set effective min eigenvalue before update().
  /// -1.0 means use config default.
  void set_effective_svd_min_eigenvalue(float val) {
    effective_svd_min_eigenvalue_ = val;
  }

  /// Number of currently frozen voxels (diagnostic).
  int frozen_voxel_count() const;

  // ---- Main mutation -------------------------------------------------------

  /// Add all points in @p cloud (in world frame).
  /// @p sensor_pos is used for map recentering / pruning.
  void update(const PointCloud& cloud, const Eigen::Vector3f& sensor_pos,
              float frame_res_rms = -1.0f, int frame_id = 0);

  // ---- Queries -------------------------------------------------------------

  /// Gather the k nearest raw points to @p query from the voxel neighborhood.
  /// Returns the number of points found (written to @p out_points).
  /// @p out_points must point to an array of at least @p max_points elements.
  /// @p out_dists2 (optional) receives squared distances (same length as out_points).
  int gather_knn(const Eigen::Vector3f& query,
                 int k,
                 Eigen::Vector3f* out_points,
                 float* out_dists2 = nullptr) const;

  /// Fit a plane from the k nearest raw points to @p query.
  /// Returns a PlaneResult with valid=true if a good plane was found.
  /// @p max_dist limits how far neighbors can be from the query (metres).
  ///   0 = no distance limit (use all k nearest regardless of distance).
  PlaneResult fit_plane_knn(const Eigen::Vector3f& query,
                             int k_neighbors,
                             float planarity_threshold,
                             float max_dist = 0.0f) const;

  /// D1: Fit a plane from pre-gathered neighbor points (skip kNN re-search).
  /// Used for IEKF iteration > 0 where neighbors are cached from iteration 0.
  /// No mutex needed since it doesn't access map_.
  PlaneResult fit_plane_from_neighbors(
      const Eigen::Vector3f& query,
      const Eigen::Vector3f* neighbors,
      int n_neighbors,
      float planarity_threshold,
      float max_dist = 0.0f) const;

  /// Option C: Get cached plane results from neighbor voxels.
  /// Returns up to (2H+1)^3 PlaneResults from voxels around @p query.
  /// Each result has the voxel's pre-computed plane (centroid+normal).
  /// @p out_planes must point to an array of at least max_results elements.
  /// @p max_plane_dist: max distance from query to accepted plane.
  /// Returns the number of valid plane results written.
  int get_voxel_planes(const Eigen::Vector3f& query,
                       PlaneResult* out_planes,
                       int max_results,
                       float planarity_threshold,
                       float max_plane_dist = 0.3f,
                       int search_half_override = -1) const;

  // ---- Statistics ----------------------------------------------------------

  size_t voxel_count() const;
  size_t point_count() const;

  // ---- Lifecycle -----------------------------------------------------------

  const PointVoxelMapConfig& config() const { return config_; }
  void reset();

  /// R0.10 H4: prune voxels with last_obs_frame <= frame_threshold.
  /// Used by SurfelMap H4 surgical rebuild at LOCK to remove pre-LOCK
  /// PVMap contamination (path a-restricted, R0.10.1).
  void clear_pre_lock_voxels(int frame_threshold);

 private:
  // ---- Internal voxel type -------------------------------------------------

  struct Voxel {
    /// Ring buffer of raw points.  Fixed-capacity, wraps via head_ index.
    std::vector<Eigen::Vector3f> points;
    int head = 0;   ///< Next write position (wraps around)
    int count = 0;  ///< Number of valid points (<= capacity)

    // R0.10 H4: frame of latest observation; pre-LOCK pruning predicate.
    int  last_obs_frame = -1;

    // DARBF: freeze state for degeneracy-aware ring buffer preservation.
    bool frozen = false;              ///< Write-suppressed when true
    int  freeze_frame_start = -1;     ///< Frame when freeze began
    Eigen::Vector3f cached_normal = Eigen::Vector3f::Zero();
    bool has_cached_normal = false;

    // Option C: Per-voxel plane cache (Welford incremental covariance).
    // Updated after each point insertion batch in update().
    Eigen::Vector3f plane_centroid = Eigen::Vector3f::Zero();
    Eigen::Matrix3f plane_cov     = Eigen::Matrix3f::Zero();
    Eigen::Vector3f plane_normal  = Eigen::Vector3f::Zero();
    float           plane_planarity = 1.0f;    ///< λ_min/λ_max
    float           plane_normal_sigma2 = 0.0f;
    int             plane_cache_count = 0;     ///< count at last recompute
    bool            plane_cache_valid = false;

    void init(int capacity) {
      points.resize(capacity);
      head = 0;
      count = 0;
      frozen = false;
      freeze_frame_start = -1;
      has_cached_normal = false;
      plane_centroid.setZero();
      plane_cov.setZero();
      plane_normal.setZero();
      plane_planarity = 1.0f;
      plane_normal_sigma2 = 0.0f;
      plane_cache_count = 0;
      plane_cache_valid = false;
      last_obs_frame = -1;
    }

    void push(const Eigen::Vector3f& p) {
      if (frozen) return;  // DARBF: skip write to preserve anchor points.
      points[head] = p;
      head = (head + 1) % static_cast<int>(points.size());
      if (count < static_cast<int>(points.size())) ++count;
    }
  };

  // ---- Internal helpers ----------------------------------------------------

  VoxelKey to_key(const Eigen::Vector3f& p) const;

  /// Remove voxels outside the keep-box.
  void prune(const Eigen::Vector3f& center, float box_half);

  /// Option C: Recompute plane cache for all dirty voxels.
  /// Called at end of update() when use_voxel_plane_cache is true.
  void recompute_voxel_plane_caches();

  // ---- Data ----------------------------------------------------------------

  PointVoxelMapConfig config_;
  float inv_voxel_size_ = 2.0f;  ///< 1 / voxel_size

  ankerl::unordered_dense::map<VoxelKey, Voxel, VoxelKeyHash> map_;

  Eigen::Vector3f map_center_      = Eigen::Vector3f::Zero();
  bool            map_initialized_ = false;

  // Option C: dirty voxel keys for incremental plane cache update.
  std::vector<VoxelKey> dirty_voxel_keys_;

  // DARBF: transient degeneracy state (set per frame via set_degen_freeze_state).
  Eigen::Vector3f degen_dirs_[3];
  int num_degen_dirs_ = 0;
  int current_frame_ = 0;

  // Adaptive SVD: per-frame effective min eigenvalue (-1 = use config default).
  float effective_svd_min_eigenvalue_ = -1.0f;

  mutable std::shared_mutex mutex_;
};

}  // namespace core
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_MAP_POINT_VOXEL_MAP_HPP_
