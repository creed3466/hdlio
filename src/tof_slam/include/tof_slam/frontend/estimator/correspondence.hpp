// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// correspondence.hpp — Point-to-plane correspondence for LiDAR observations.

#ifndef TOF_SLAM_FRONTEND_ESTIMATOR_CORRESPONDENCE_HPP_
#define TOF_SLAM_FRONTEND_ESTIMATOR_CORRESPONDENCE_HPP_

#include <Eigen/Dense>
#include <vector>

#include "tof_slam/frontend/map/voxel_key.hpp"

namespace tof_slam {
namespace core {

/// Residual mode for IEKF update.
enum class ResidualMode : uint8_t {
  kPointToPlane = 0,         ///< Standard 1×1 scalar residual: n^T(p-c)
  kPointToDistribution = 1,  ///< iG-LIO style 3×1 Mahalanobis: Σ^{-1}(p-μ)
};

/// A single point-to-plane correspondence.
struct Correspondence {
  Eigen::Vector3f p_lidar;   // Point in LiDAR sensor frame
  Eigen::Vector3f normal;    // Surfel normal in world frame
  float plane_d = 0.0f;      // Plane offset: n^T * centroid

  // --- Per-correspondence quality metrics (for adaptive R) ---
  float range = 0.0f;          // Distance from sensor to point [m]
  float cos_incidence = 1.0f;  // |cos(angle between ray and normal)| [0,1]
  float planarity = 0.0f;      // Surfel planarity [0=planar, 1=isotropic]

  // --- Phase E-1: Surfel normal uncertainty for measurement noise model ---
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();  // Surfel centroid (world)
  float normal_sigma2 = 0.0f;  // Normal angular variance: λ₃/(N×λ₁)

  // --- Per-correspondence noise override (Frozen Anchor Map) ---
  // When nonzero, the IEKF uses this sigma instead of the global lidar_noise_std.
  // Used for anchor map correspondences which have different noise characteristics.
  float noise_override = 0.0f;  // 0 = use global lidar_noise_std

  // --- Surfel sharing weight ---
  // Number of correspondences sharing the same L1 voxel in this frame.
  // Used to down-weight shared surfels: w_sharing = 1 / sharing_count.
  int sharing_count = 1;

  // --- Normal Agreement Weight (NAW, Task #133 Iter 5) ---
  // |surfel.normal · pvmap.normal| for dual-source correspondences.
  // 1.0 for single-source (no second opinion available).
  // Used in IEKF to downweight correspondences where normals disagree.
  float normal_agreement = 1.0f;

  // --- D4: Cached L1 voxel key (eliminates redundant recomputation in share_count) ---
  VoxelKey l1_key;

  // --- Point-to-Distribution (Voxel-Gaussian) residual ---
  ResidualMode residual_mode = ResidualMode::kPointToPlane;
  Eigen::Matrix3f voxel_cov_inv = Eigen::Matrix3f::Zero();  ///< Ω_i = Σ^{-1}

  // --- L2 multi-scale correspondence ---
  // Supplementary coarser-scale surfel (2.7m voxels built from L1 centroids).
  // When has_l2 is true, the IEKF accumulates an additional rank-1 update
  // from this L2 surfel on top of the L1 update (pure information addition).
  bool            has_l2 = false;
  Eigen::Vector3f l2_normal = Eigen::Vector3f::Zero();
  float           l2_plane_d = 0.0f;
  Eigen::Vector3f l2_centroid = Eigen::Vector3f::Zero();
  float           l2_normal_sigma2 = 0.0f;
  float           l2_planarity = 1.0f;

  // --- S13-B.A.2 P1: Anisotropic surfel covariance per level ---
  // Per sprint13_architecture §3.1: populated by surfel_map.cpp builder
  // when frontend_anisotropic_iekf_enable=true. Σ_{L1} is consumed from
  // S12-B.B.1 storage (surfel_map.hpp:368-375); Σ_{L2} is computed by
  // direct L2 PCA (NOT the deactivated B.B.2 pull-back at surfel_map.cpp:178-205).
  // When flag=false: both has_surfel_cov_* default-false and Σ fields stay zero;
  // iekf_updater takes the legacy scalar path and never reads these fields.
  bool            has_surfel_cov_L1 = false;
  Eigen::Matrix3f surfel_cov_L1 = Eigen::Matrix3f::Zero();
  bool            has_surfel_cov_L2 = false;
  Eigen::Matrix3f surfel_cov_L2 = Eigen::Matrix3f::Zero();
};

}  // namespace core
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_ESTIMATOR_CORRESPONDENCE_HPP_
