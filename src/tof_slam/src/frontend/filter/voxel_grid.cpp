// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "tof_slam/frontend/filter/voxel_grid.hpp"

#include <Eigen/Dense>
#include <vector>

#include "unordered_dense.h"

namespace tof_slam {
namespace core {

namespace {

/// Accumulated statistics for one voxel cell.
struct VoxelBucket {
  Eigen::Vector3f sum = Eigen::Vector3f::Zero();
  float sum_intensity = 0.0f;
  float sum_offset_time = 0.0f;
  std::vector<Eigen::Vector3f> positions;  // For planarity (lazy allocation).
  int count = 0;

  void add(const Point3D& p) {
    sum += p.to_eigen();
    sum_intensity += p.intensity;
    sum_offset_time += p.offset_time;
    ++count;
  }

  void add_with_planarity(const Point3D& p) {
    add(p);
    positions.push_back(p.to_eigen());
  }

  Point3D centroid() const {
    const float inv = 1.0f / static_cast<float>(count);
    Point3D c;
    c.x = sum.x() * inv;
    c.y = sum.y() * inv;
    c.z = sum.z() * inv;
    c.intensity = sum_intensity * inv;
    c.offset_time = sum_offset_time * inv;
    return c;
  }

  /// sigma_min / sigma_max  (lower = more planar).  Returns 0 if < 3 points.
  float planarity() const {
    if (static_cast<int>(positions.size()) < 3) return 0.0f;

    const Eigen::Vector3f mean = sum / static_cast<float>(count);
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (const auto& p : positions) {
      const Eigen::Vector3f d = p - mean;
      cov += d * d.transpose();
    }
    cov /= static_cast<float>(count);

    Eigen::JacobiSVD<Eigen::Matrix3f> svd(cov);
    const auto& sv = svd.singularValues();
    return sv(2) / (sv(0) + 1e-6f);
  }
};

}  // namespace

PointCloud voxel_grid_filter(const PointCloud& input,
                             const VoxelGridConfig& cfg) {
  PointCloud output;
  if (input.empty() || cfg.leaf_size <= 0.0f) return output;

  const float inv_leaf = 1.0f / cfg.leaf_size;

  ankerl::unordered_dense::map<VoxelKey, VoxelBucket, VoxelKeyHash> buckets;
  buckets.reserve(input.size() / 4);  // heuristic

  for (const auto& p : input) {
    VoxelKey key = point_to_voxel_key(p.x, p.y, p.z, inv_leaf);
    auto& bucket = buckets[key];
    if (cfg.enable_planarity_filter) {
      bucket.add_with_planarity(p);
    } else {
      bucket.add(p);
    }
  }

  // Sort voxel keys for deterministic output order.
  // unordered_dense::map iteration order is non-deterministic; sorting by
  // VoxelKey ensures identical point ordering across runs, which is critical
  // for deterministic correspondence finding and IEKF accumulation.
  std::vector<VoxelKey> sorted_keys;
  sorted_keys.reserve(buckets.size());
  for (const auto& [key, bucket] : buckets) {
    sorted_keys.push_back(key);
  }
  std::sort(sorted_keys.begin(), sorted_keys.end());

  output.reserve(sorted_keys.size());
  for (const VoxelKey& key : sorted_keys) {
    const auto& bucket = buckets[key];
    if (cfg.enable_planarity_filter) {
      const float plan = bucket.planarity();
      if (plan > cfg.planarity_threshold) continue;  // Not planar enough.
    }
    output.push_back(bucket.centroid());
  }

  return output;
}

}  // namespace core
}  // namespace tof_slam
