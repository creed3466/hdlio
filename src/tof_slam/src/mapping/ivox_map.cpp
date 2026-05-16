#include "tof_slam/mapping/ivox_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tof_slam {

// ---------------------------------------------------------------------------
// VoxelData (original TofSLAM, unchanged)
// ---------------------------------------------------------------------------

void VoxelData::addPoint(const Eigen::Vector3d& p, double timestamp) {
  sum += p;
  sum_sq += p * p.transpose();
  count++;
  last_update = timestamp;
  surfel_valid = false;

  if (static_cast<int>(points.size()) < kMaxPointsPerVoxel) {
    points.push_back(p);
  } else {
    const int slot = (count - 1) % kMaxPointsPerVoxel;
    points[slot] = p;
  }
}

void VoxelData::recomputeSurfel() {
  if (count == 0) {
    surfel_valid = false;
    return;
  }

  const Eigen::Vector3d centroid = sum / static_cast<double>(count);
  const Eigen::Matrix3d covariance =
      sum_sq / static_cast<double>(count) - centroid * centroid.transpose();

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  const Eigen::Vector3d eigenvalues = solver.eigenvalues();
  const Eigen::Matrix3d eigenvectors = solver.eigenvectors();

  Eigen::Vector3d normal = eigenvectors.col(0);
  if (normal.z() < 0.0) {
    normal = -normal;
  }

  surfel.centroid = centroid;
  surfel.covariance = covariance;
  surfel.normal = normal;
  surfel.support_count = count;
  surfel.timestamp = last_update;
  surfel_valid = true;
}

// ---------------------------------------------------------------------------
// IVoxMap
// ---------------------------------------------------------------------------

IVoxMap::IVoxMap(const TofSlamConfig& config)
    : config_(config),
      voxel_size_(config.map_voxel_size),
      inv_voxel_size_(1.0 / config.map_voxel_size),
      nearby_type_(nearbyTypeFromInt(config.ivox_nearby_type)),
      capacity_(config.ivox_capacity) {
  generateNearbyOffsets();
}

void IVoxMap::generateNearbyOffsets() {
  nearby_offsets_.clear();
  nearby_offsets_.emplace_back(0, 0, 0);

  if (nearby_type_ >= NearbyType::kNearby6) {
    nearby_offsets_.emplace_back(-1, 0, 0);
    nearby_offsets_.emplace_back(1, 0, 0);
    nearby_offsets_.emplace_back(0, 1, 0);
    nearby_offsets_.emplace_back(0, -1, 0);
    nearby_offsets_.emplace_back(0, 0, -1);
    nearby_offsets_.emplace_back(0, 0, 1);
  }

  if (nearby_type_ >= NearbyType::kNearby18) {
    nearby_offsets_.emplace_back(1, 1, 0);
    nearby_offsets_.emplace_back(-1, 1, 0);
    nearby_offsets_.emplace_back(1, -1, 0);
    nearby_offsets_.emplace_back(-1, -1, 0);
    nearby_offsets_.emplace_back(1, 0, 1);
    nearby_offsets_.emplace_back(-1, 0, 1);
    nearby_offsets_.emplace_back(1, 0, -1);
    nearby_offsets_.emplace_back(-1, 0, -1);
    nearby_offsets_.emplace_back(0, 1, 1);
    nearby_offsets_.emplace_back(0, -1, 1);
    nearby_offsets_.emplace_back(0, 1, -1);
    nearby_offsets_.emplace_back(0, -1, -1);
  }

  if (nearby_type_ >= NearbyType::kNearby26) {
    nearby_offsets_.emplace_back(1, 1, 1);
    nearby_offsets_.emplace_back(-1, 1, 1);
    nearby_offsets_.emplace_back(1, -1, 1);
    nearby_offsets_.emplace_back(-1, -1, 1);
    nearby_offsets_.emplace_back(1, 1, -1);
    nearby_offsets_.emplace_back(-1, 1, -1);
    nearby_offsets_.emplace_back(1, -1, -1);
    nearby_offsets_.emplace_back(-1, -1, -1);
  }
}

Eigen::Vector3i IVoxMap::pointToVoxelIndex(const Eigen::Vector3d& p) const {
  return Eigen::Vector3i(static_cast<int>(std::floor(p.x() * inv_voxel_size_)),
                         static_cast<int>(std::floor(p.y() * inv_voxel_size_)),
                         static_cast<int>(std::floor(p.z() * inv_voxel_size_)));
}

Eigen::Vector3d IVoxMap::voxelCenter(const Eigen::Vector3i& idx) const {
  return (idx.cast<double>() + Eigen::Vector3d::Constant(0.5)) * voxel_size_;
}

// ---------------------------------------------------------------------------
// Insertion with LRU tracking
// ---------------------------------------------------------------------------

void IVoxMap::insertCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_world, double timestamp) {
  insertCloud(cloud_world, timestamp, 0);
}

void IVoxMap::insertCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_world, double timestamp,
    int max_points) {
  if (!cloud_world) return;
  const int N = static_cast<int>(cloud_world->points.size());
  const int stride = (max_points > 0 && N > max_points) ? (N / max_points) : 1;

  for (int i = 0; i < N; i += stride) {
    const auto& pt = cloud_world->points[i];
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }

    const Eigen::Vector3d p(pt.x, pt.y, pt.z);
    const Eigen::Vector3i idx = pointToVoxelIndex(p);

    // Insert point into voxel data (same as original)
    voxels_[idx].addPoint(p, timestamp);

    // LRU tracking: promote to front
    auto lru_it = lru_map_.find(idx);
    if (lru_it != lru_map_.end()) {
      lru_order_.splice(lru_order_.begin(), lru_order_, lru_it->second);
    } else {
      lru_order_.push_front(idx);
      lru_map_[idx] = lru_order_.begin();
    }

    // LRU eviction
    if (lru_map_.size() > capacity_) {
      const auto& back_key = lru_order_.back();
      voxels_.erase(back_key);
      lru_map_.erase(back_key);
      lru_order_.pop_back();
    }
  }
}

// ---------------------------------------------------------------------------
// Queries (use configurable nearby_offsets_)
// ---------------------------------------------------------------------------

bool IVoxMap::findNearestSurfel(const Eigen::Vector3d& query_world,
                                Surfel& surfel_out) const {
  const Eigen::Vector3i center_idx = pointToVoxelIndex(query_world);

  double best_dist_sq = std::numeric_limits<double>::max();
  bool found = false;

  for (const auto& offset : nearby_offsets_) {
    const Eigen::Vector3i neighbor_idx = center_idx + offset;

    auto it = voxels_.find(neighbor_idx);
    if (it == voxels_.end()) {
      continue;
    }

    VoxelData& voxel = const_cast<VoxelData&>(it->second);
    if (!voxel.surfel_valid) {
      voxel.recomputeSurfel();
    }

    if (voxel.surfel.support_count < config_.min_support_points) {
      continue;
    }

    const double dist_sq =
        (voxel.surfel.centroid - query_world).squaredNorm();
    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      surfel_out = voxel.surfel;
      found = true;
    }
  }

  return found;
}

bool IVoxMap::findNearestPoints(const Eigen::Vector3d& query_world,
                                std::vector<Eigen::Vector3d>& points_out,
                                int max_num,
                                double max_range) const {
  const Eigen::Vector3i center_idx = pointToVoxelIndex(query_world);
  const double max_range_sq = max_range * max_range;

  std::vector<std::pair<double, Eigen::Vector3d>> candidates;

  for (const auto& offset : nearby_offsets_) {
    const Eigen::Vector3i neighbor_idx = center_idx + offset;

    auto it = voxels_.find(neighbor_idx);
    if (it == voxels_.end()) {
      continue;
    }

    const VoxelData& voxel = it->second;
    for (const auto& pt : voxel.points) {
      const double dist_sq = (pt - query_world).squaredNorm();
      if (dist_sq < max_range_sq) {
        candidates.emplace_back(dist_sq, pt);
      }
    }
  }

  if (static_cast<int>(candidates.size()) > max_num) {
    std::nth_element(candidates.begin(),
                     candidates.begin() + max_num,
                     candidates.end(),
                     [](const auto& a, const auto& b) { return a.first < b.first; });
  }

  const int n = std::min(static_cast<int>(candidates.size()), max_num);
  points_out.clear();
  points_out.reserve(n);
  for (int i = 0; i < n; ++i) {
    points_out.push_back(candidates[i].second);
  }

  return static_cast<int>(points_out.size()) >= max_num;
}

// ---------------------------------------------------------------------------
// Map maintenance
// ---------------------------------------------------------------------------

void IVoxMap::pruneByRadius(const Eigen::Vector3d& center) {
  static constexpr size_t kMinVoxelsAfterPrune = 100;
  if (voxels_.size() <= kMinVoxelsAfterPrune) return;

  const double radius_sq = config_.map_radius * config_.map_radius;

  size_t survive_count = 0;
  for (const auto& [idx, voxel] : voxels_) {
    Eigen::Vector3d centroid;
    if (voxel.count > 0) {
      centroid = voxel.sum / static_cast<double>(voxel.count);
    } else {
      centroid = voxelCenter(idx);
    }
    if ((centroid - center).squaredNorm() <= radius_sq) {
      ++survive_count;
    }
  }

  if (survive_count < kMinVoxelsAfterPrune) return;

  for (auto it = voxels_.begin(); it != voxels_.end();) {
    Eigen::Vector3d centroid;
    if (it->second.count > 0) {
      centroid = it->second.sum / static_cast<double>(it->second.count);
    } else {
      centroid = voxelCenter(it->first);
    }

    if ((centroid - center).squaredNorm() > radius_sq) {
      // Remove from LRU tracking too
      auto lru_it = lru_map_.find(it->first);
      if (lru_it != lru_map_.end()) {
        lru_order_.erase(lru_it->second);
        lru_map_.erase(lru_it);
      }
      it = voxels_.erase(it);
    } else {
      ++it;
    }
  }
}

void IVoxMap::pruneByTime(double current_time, double max_age) {
  const double cutoff = current_time - max_age;

  for (auto it = voxels_.begin(); it != voxels_.end();) {
    if (it->second.last_update < cutoff) {
      auto lru_it = lru_map_.find(it->first);
      if (lru_it != lru_map_.end()) {
        lru_order_.erase(lru_it->second);
        lru_map_.erase(lru_it);
      }
      it = voxels_.erase(it);
    } else {
      ++it;
    }
  }
}

std::vector<Surfel> IVoxMap::getAllSurfels() const {
  std::vector<Surfel> result;
  result.reserve(voxels_.size());

  for (auto& [idx, voxel] : voxels_) {
    if (!voxel.surfel_valid) {
      const_cast<VoxelData&>(voxel).recomputeSurfel();
    }
    if (voxel.surfel_valid &&
        voxel.surfel.support_count >= config_.min_support_points) {
      result.push_back(voxel.surfel);
    }
  }

  return result;
}

void IVoxMap::clear() {
  voxels_.clear();
  lru_order_.clear();
  lru_map_.clear();
}

}  // namespace tof_slam
