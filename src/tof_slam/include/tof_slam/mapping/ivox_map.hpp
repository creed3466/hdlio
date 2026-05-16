#pragma once

/// iVox hash-voxel map for scan-to-map registration.
///
/// Combines the proven TofSLAM VoxelData design (ring buffer + surfel cache)
/// with Faster-LIO features:
///   - LRU eviction for bounded memory (replaces manual pruneByRadius)
///   - Configurable NearbyType (CENTER/6/18/26)
///   - Original TofSLAM hash and floor() voxelization preserved.

#include <algorithm>
#include <cmath>
#include <limits>
#include <list>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "tof_slam/common/config.hpp"
#include "tof_slam/common/types.hpp"

namespace tof_slam {

/// Configurable search neighborhood (Faster-LIO).
enum class NearbyType : int {
  kCenter = 0,
  kNearby6 = 6,
  kNearby18 = 18,
  kNearby26 = 26
};

inline NearbyType nearbyTypeFromInt(int v) {
  switch (v) {
    case 0:  return NearbyType::kCenter;
    case 6:  return NearbyType::kNearby6;
    case 18: return NearbyType::kNearby18;
    case 26: return NearbyType::kNearby26;
    default: return NearbyType::kNearby26;
  }
}

/// Spatial hashing for voxel indices.
struct VoxelHash {
  size_t operator()(const Eigen::Vector3i& v) const {
    return static_cast<size_t>(
        v.x() * 73856093L ^ v.y() * 19349669L ^ v.z() * 83492791L);
  }
};

struct VoxelEqual {
  bool operator()(const Eigen::Vector3i& a, const Eigen::Vector3i& b) const {
    return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
  }
};

/// Voxel data: ring buffer + surfel cache.
struct VoxelData {
  static constexpr int kMaxPointsPerVoxel = 20;

  Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d sum_sq{Eigen::Matrix3d::Zero()};
  int count{0};
  double last_update{0.0};

  Surfel surfel;
  bool surfel_valid{false};

  std::vector<Eigen::Vector3d> points;

  void addPoint(const Eigen::Vector3d& p, double timestamp);
  void recomputeSurfel();
};

/// Hash-voxel map with LRU eviction and configurable NearbyType.
class IVoxMap {
 public:
  explicit IVoxMap(const TofSlamConfig& config);

  void insertCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_world,
                   double timestamp);
  void insertCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_world,
                   double timestamp, int max_points);

  bool findNearestSurfel(const Eigen::Vector3d& query_world,
                         Surfel& surfel_out) const;

  bool findNearestPoints(const Eigen::Vector3d& query_world,
                         std::vector<Eigen::Vector3d>& points_out,
                         int max_num = 5,
                         double max_range = 5.0) const;

  void pruneByRadius(const Eigen::Vector3d& center);
  void pruneByTime(double current_time, double max_age);

  size_t size() const { return voxels_.size(); }
  std::vector<Surfel> getAllSurfels() const;
  void clear();

 private:
  Eigen::Vector3i pointToVoxelIndex(const Eigen::Vector3d& p) const;
  Eigen::Vector3d voxelCenter(const Eigen::Vector3i& idx) const;
  void generateNearbyOffsets();

  const TofSlamConfig& config_;
  double voxel_size_;
  double inv_voxel_size_;
  NearbyType nearby_type_;
  size_t capacity_;

  std::unordered_map<Eigen::Vector3i, VoxelData, VoxelHash, VoxelEqual> voxels_;

  // LRU tracking: ordered list of voxel keys (front = most recent)
  std::list<Eigen::Vector3i> lru_order_;
  std::unordered_map<Eigen::Vector3i,
                     std::list<Eigen::Vector3i>::iterator,
                     VoxelHash, VoxelEqual> lru_map_;

  std::vector<Eigen::Vector3i> nearby_offsets_;
};

}  // namespace tof_slam
