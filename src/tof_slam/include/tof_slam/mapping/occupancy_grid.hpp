#pragma once

#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "tof_slam/common/types.hpp"
#include "tof_slam/common/config.hpp"

namespace tof_slam {

/// Local occupancy grid for a single keyframe (XY projection from 3D point cloud)
struct LocalGrid {
  std::vector<double> log_odds;    // log-odds values, row-major
  int width{0};                     // cells in x
  int height{0};                    // cells in y
  double resolution{0.05};         // m per cell
  double origin_x{0.0};            // world x of grid origin (bottom-left)
  double origin_y{0.0};            // world y of grid origin
  size_t keyframe_id{0};

  /// Get cell index, returns -1 if out of bounds
  int cellIndex(int gx, int gy) const {
    if (gx < 0 || gx >= width || gy < 0 || gy >= height) return -1;
    return gy * width + gx;
  }

  /// Convert world coordinates to grid coordinates
  bool worldToGrid(double wx, double wy, int& gx, int& gy) const {
    gx = static_cast<int>(std::floor((wx - origin_x) / resolution));
    gy = static_cast<int>(std::floor((wy - origin_y) / resolution));
    return gx >= 0 && gx < width && gy >= 0 && gy < height;
  }
};

/// Occupancy Grid Map generator.
/// Generates per-keyframe local grids and assembles global OGM.
class OccupancyGridGenerator {
 public:
  explicit OccupancyGridGenerator(const TofSlamConfig& config);

  /// Generate a local occupancy grid from a point cloud (body frame)
  /// The grid is centered on the robot position (0,0 in body frame)
  LocalGrid generateLocalGrid(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_body,
      size_t keyframe_id) const;

  /// Assemble global OGM from multiple local grids with their world poses
  /// Returns occupancy values in [-1, 100] (ROS convention: -1=unknown, 0=free, 100=occupied)
  struct GlobalGrid {
    std::vector<int8_t> data;   // occupancy values
    int width{0};
    int height{0};
    double resolution{0.05};
    double origin_x{0.0};
    double origin_y{0.0};
    double raw_min_x{0.0};
    double raw_min_y{0.0};
    double raw_max_x{0.0};
    double raw_max_y{0.0};
    double canvas_min_x{0.0};
    double canvas_min_y{0.0};
    double canvas_max_x{0.0};
    double canvas_max_y{0.0};
    int padding_cells{0};
    bool origin_snapped{false};
  };

  GlobalGrid assembleGlobalGrid(
      const std::vector<LocalGrid>& local_grids,
      const std::vector<Eigen::Matrix4d>& world_poses) const;

  /// Reassemble global grid after PGO (reuse local grids, new poses)
  GlobalGrid reassembleAfterPGO(
      const std::vector<LocalGrid>& local_grids,
      const std::vector<Eigen::Matrix4d>& corrected_poses) const;

 private:
  /// Apply occupied-cell dilation after global assembly
  void dilateGlobalOccupied(std::vector<double>& global_log_odds,
                            int width, int height) const;

  /// Morphological cleaning: remove isolated occupied cells (noise)
  void morphologicalClean(std::vector<double>& log_odds,
                          int width, int height) const;

  /// Mark endpoint cell and nearby cells as occupied to compensate sparse sampling
  void markOccupied(LocalGrid& grid, int gx, int gy,
                    double range_scale = 1.0) const;

  /// Ray casting: mark cells along ray from origin to endpoint
  void rayCast(LocalGrid& grid, double x0, double y0,
               double x1, double y1) const;

  /// Clamp log-odds value
  double clampLogOdds(double val) const;

  /// Convert log-odds to probability [0, 1]
  static double logOddsToProbability(double log_odds);

  const TofSlamConfig& config_;
};

}  // namespace tof_slam
