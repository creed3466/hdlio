// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// voxel_grid.hpp — Scan-level voxel downsampling with optional planarity
//                  filtering (PCA-based).

#ifndef TOF_SLAM_FRONTEND_FILTER_VOXEL_GRID_HPP_
#define TOF_SLAM_FRONTEND_FILTER_VOXEL_GRID_HPP_

#include "tof_slam/frontend/map/voxel_key.hpp"
#include "tof_slam/common/types/point_types.hpp"

namespace tof_slam {
namespace core {

/// Configuration for voxel-grid downsampling.
struct VoxelGridConfig {
  float leaf_size = 0.4f;
  bool enable_planarity_filter = false;
  /// Planarity threshold:  sigma_min / sigma_max.  Lower = more planar.
  /// Default 0.1 = relaxed (scan-level).  Map-level uses 0.01.
  float planarity_threshold = 0.1f;
};

/// Voxel-grid downsample `input` using centroid per voxel.
///
/// If `cfg.enable_planarity_filter` is true, voxels whose planarity score
/// exceeds the threshold are rejected (keeping only planar patches).
PointCloud voxel_grid_filter(const PointCloud& input,
                             const VoxelGridConfig& cfg);

}  // namespace core
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_FILTER_VOXEL_GRID_HPP_
