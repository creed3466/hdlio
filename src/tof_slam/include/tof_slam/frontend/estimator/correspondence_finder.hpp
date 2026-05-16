// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// correspondence_finder.hpp — Pure function to find point-to-plane
// correspondences between a LiDAR scan and a SurfelMap.

#ifndef TOF_SLAM_FRONTEND_ESTIMATOR_CORRESPONDENCE_FINDER_HPP_
#define TOF_SLAM_FRONTEND_ESTIMATOR_CORRESPONDENCE_FINDER_HPP_

#include <vector>

#include <pcl/point_types.h>

#include "tof_slam/frontend/estimator/correspondence.hpp"
#include "tof_slam/common/lie/se3.hpp"
#include "tof_slam/frontend/map/surfel_map.hpp"
#include "tof_slam/frontend/map/point_voxel_map.hpp"
#include "tof_slam/common/types/point_types.hpp"
#include "tof_slam/common/types/state.hpp"

namespace tof_slam {
namespace core {

// ---------------------------------------------------------------------------
// HybridStats — aggregate surfel-PVMap agreement statistics per frame.
// Used by adaptive pvmap_sigma2_scale mechanism to detect surfel contamination.
// ---------------------------------------------------------------------------
struct HybridStats {
  int n_dual = 0;           ///< Points where both surfel and PVMap were valid
  float sum_cos_nn = 0.0f;  ///< Sum of |surfel.normal · pvmap.normal| for dual points
  int n_surfel_only = 0;    ///< Points where only surfel was valid
  int n_pvmap_only = 0;     ///< Points where only PVMap was valid
  int n_degen_override = 0; ///< Points where DDPO forced PVMap over surfel

  /// Mean normal agreement ratio [0,1]. Returns 0 if no dual points.
  float mean_agreement() const {
    return (n_dual > 0) ? (sum_cos_nn / static_cast<float>(n_dual)) : 0.0f;
  }
};

// ---------------------------------------------------------------------------
// D1: CachedKnnEntry — cached kNN neighbors from IEKF iteration 0.
// ---------------------------------------------------------------------------

/// Cached kNN neighbors from IEKF iteration 0 for reuse on subsequent iterations.
/// Includes the query point at cache time for position-delta gating.
struct CachedKnnEntry {
  Eigen::Vector3f neighbors[30];
  int n_gathered = 0;
  Eigen::Vector3f p_world_cached = Eigen::Vector3f::Zero();  ///< Query point at cache time
};

// ---------------------------------------------------------------------------
// HybridCorrespondenceParams — bundled parameters for hybrid correspondence.
// Replaces 10 individual function arguments for cleaner API.
// ---------------------------------------------------------------------------
struct HybridCorrespondenceParams {
  float max_plane_distance = 0.15f;
  int pvmap_k_neighbors = 5;
  float pvmap_planarity_threshold = 0.15f;
  int max_corr_per_l1 = 0;
  float pvmap_sigma2_scale = 2.0f;
  HybridStats* out_stats = nullptr;
  const Eigen::Vector3f* degen_trans_dirs = nullptr;
  int num_degen_trans_dirs = 0;
  float degen_pvmap_cos_threshold = 0.5f;

  // --- D1: kNN cache with position-delta gating ---
  std::vector<CachedKnnEntry>* knn_cache = nullptr;  ///< Mutable kNN cache (delta-gated)
  bool knn_cache_is_populated = false;                ///< Cache populated from prior iteration

  // --- Spinning LiDAR mode (S1-S4) ---
  // When true, bypass surfel map (S1), use tighter residual threshold (S4),
  // and reject ring-plane artifacts (S3). Designed for spinning LiDARs
  // (e.g., Velodyne VLP-16, Ouster OS1-64) where surfel PCA fails on
  // ring patterns.
  /// Maximum OpenMP threads for correspondence finding (0 = OMP default).
  int cf_omp_max_threads = 4;

  /// Point-to-Distribution (P2D): use voxel Gaussian (mean+covariance) instead
  /// of point-to-plane. Each correspondence carries Ω = Σ^{-1} for 3×1 residual.
  /// Requires use_voxel_plane_cache=true in PVMap config.
  bool enable_point_to_distribution = false;
  float p2d_chi2_threshold = 7.815f;  ///< Chi² outlier gate (df=3, p=0.05)
  float p2d_cov_reg_eps = 1e-3f;      ///< Regularization for Σ inversion

  /// CSCF (Continuous Surfel Correspondence Field): kernel-weighted L1 surfel
  /// interpolation.  When enabled, gathers 3x3x3 L1 neighborhood surfels and
  /// computes a Gaussian-weighted interpolated surfel (normal, centroid, sigma2).
  /// Falls back to discrete get_surfel when interpolation fails (e.g., no valid
  /// neighbors or opposing normals cancel out).  Default false (bit-identical).
  bool enable_cscf = false;
  float cscf_kernel_bandwidth = 0.25f;  ///< Gaussian kernel sigma for L1 surfel weighting (m)

  /// Configurable divisor for the adaptive correspondence threshold formula:
  ///   adaptive_thresh = min(max_plane_distance, sqrt(range) / divisor)
  /// Default 9.0 reproduces FAST-LIO2-era outdoor behavior.
  /// Indoor: 16-20 tightens the gate for short-range (2-8m) scenes.
  /// Theoretical basis: at range R, the 3σ acceptance for σ_lidar~0.02m is
  ///   ~0.10m, which corresponds to sqrt(4)/20 at 4m range (divisor=20).
  float adaptive_threshold_divisor = 9.0f;
};

/// Per-Correspondence Hybrid: queries BOTH surfel and PVMap, selects better plane.
/// Selection criterion: lower normal_sigma2 (more reliable normal).
/// Falls back to surfel when PVMap fails, or PVMap when surfel fails.
/// Both modes use normal-direction point-to-plane only → E[r(x_true)] = 0.
std::vector<Correspondence> find_correspondences_hybrid_select(
    const LioState& state,
    const Se3& T_body_lidar,
    const PointCloud& scan,
    const SurfelMap& surfel_map,
    const PointVoxelMap& pvmap,
    const HybridCorrespondenceParams& params);

/// @deprecated Use HybridCorrespondenceParams overload instead.
inline std::vector<Correspondence> find_correspondences_hybrid_select(
    const LioState& state,
    const Se3& T_body_lidar,
    const PointCloud& scan,
    const SurfelMap& surfel_map,
    const PointVoxelMap& pvmap,
    float max_plane_distance,
    int pvmap_k_neighbors,
    float pvmap_planarity_threshold,
    int max_corr_per_l1,
    float pvmap_sigma2_scale,
    HybridStats* out_stats,
    const Eigen::Vector3f* degen_trans_dirs,
    int num_degen_trans_dirs,
    float degen_pvmap_cos_threshold) {
  HybridCorrespondenceParams p;
  p.max_plane_distance = max_plane_distance;
  p.pvmap_k_neighbors = pvmap_k_neighbors;
  p.pvmap_planarity_threshold = pvmap_planarity_threshold;
  p.max_corr_per_l1 = max_corr_per_l1;
  p.pvmap_sigma2_scale = pvmap_sigma2_scale;
  p.out_stats = out_stats;
  p.degen_trans_dirs = degen_trans_dirs;
  p.num_degen_trans_dirs = num_degen_trans_dirs;
  p.degen_pvmap_cos_threshold = degen_pvmap_cos_threshold;
  return find_correspondences_hybrid_select(
      state, T_body_lidar, scan, surfel_map, pvmap, p);
}

}  // namespace core
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_ESTIMATOR_CORRESPONDENCE_FINDER_HPP_
