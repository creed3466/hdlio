#pragma once

/// center_voxel_filter.hpp
///
/// CenterVoxelFilter — voxel-grid downsampling that retains the point
/// geometrically closest to the voxel centre (not the centroid).
///
/// Faithful port of:
///   Super-LIO/src/super_lio/include/OctVoxMap/VoxelGridFilter.h
/// adapted to the TofSLAM namespace and pcl::PointXYZ without the
/// robin_hood dependency.
///
/// Usage:
///   CenterVoxelFilter f;
///   f.setLeafSize(0.3);
///   f.setInputCloud(cloud_in);
///   f.filter(cloud_out);

#include <cstddef>
#include <cmath>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace tof_slam {

class CenterVoxelFilter {
public:
  CenterVoxelFilter() = default;

  void setLeafSize(double size) {
    leaf_size_ = size;
    inv_leaf_  = 1.0 / size;
  }

  void setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    input_ = cloud;
  }

  /// Filter: for each occupied voxel keep the point closest to its centre.
  /// Output cloud is written into `output` (existing contents are discarded).
  void filter(pcl::PointCloud<pcl::PointXYZ>::Ptr& output);

private:
  double leaf_size_ = 0.5;
  double inv_leaf_  = 2.0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_;

  // -------------------------------------------------------------------------
  // Voxel key and hash matching Super-LIO's bit-packed key approach.
  // An offset of 1000 is added to each dimension before packing to avoid
  // problems with negative coordinates (mirrors Super-LIO's offset_ trick).
  // -------------------------------------------------------------------------
  static constexpr int KEY_OFFSET = 1000;

  /// Pack the three integer voxel coordinates into a single size_t key using
  /// 15-bit fields per axis (range [-1000, 32767] with offset applied).
  static size_t makeKey(int ix, int iy, int iz) {
    const size_t x = static_cast<size_t>(ix + KEY_OFFSET);
    const size_t y = static_cast<size_t>(iy + KEY_OFFSET);
    const size_t z = static_cast<size_t>(iz + KEY_OFFSET);
    return (z << 30) | (y << 15) | x;
  }

  struct VoxelEntry {
    std::size_t point_idx;
    double      dist_sq;
  };
};

// ---------------------------------------------------------------------------
// Inline implementation
// ---------------------------------------------------------------------------
inline void CenterVoxelFilter::filter(pcl::PointCloud<pcl::PointXYZ>::Ptr& output) {
  if (!output) {
    output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  }

  if (!input_ || input_->empty()) {
    output->clear();
    return;
  }

  std::unordered_map<size_t, VoxelEntry> grid;
  grid.reserve(input_->size());

  for (size_t i = 0; i < input_->size(); ++i) {
    const pcl::PointXYZ& pt = input_->points[i];
    const float px = pt.x, py = pt.y, pz = pt.z;

    // Voxel index: round to nearest integer multiple of leaf_size_.
    const int ix = static_cast<int>(std::round(static_cast<double>(px) * inv_leaf_));
    const int iy = static_cast<int>(std::round(static_cast<double>(py) * inv_leaf_));
    const int iz = static_cast<int>(std::round(static_cast<double>(pz) * inv_leaf_));

    // Voxel centre.
    const double cx = ix * leaf_size_;
    const double cy = iy * leaf_size_;
    const double cz = iz * leaf_size_;

    const double dx     = static_cast<double>(px) - cx;
    const double dy     = static_cast<double>(py) - cy;
    const double dz     = static_cast<double>(pz) - cz;
    const double dist_sq = dx * dx + dy * dy + dz * dz;

    const size_t key = makeKey(ix, iy, iz);

    auto it = grid.find(key);
    if (it == grid.end()) {
      grid.emplace(key, VoxelEntry{i, dist_sq});
    } else if (dist_sq < it->second.dist_sq) {
      it->second = {i, dist_sq};
    }
  }

  // Collect surviving points into the output cloud.
  output->points.clear();
  output->points.reserve(grid.size());
  for (const auto& kv : grid) {
    output->points.push_back(input_->points[kv.second.point_idx]);
  }
  output->width    = static_cast<uint32_t>(output->points.size());
  output->height   = 1;
  output->is_dense = true;
  if (input_->header.stamp || !input_->header.frame_id.empty()) {
    output->header = input_->header;
  }
}

}  // namespace tof_slam
