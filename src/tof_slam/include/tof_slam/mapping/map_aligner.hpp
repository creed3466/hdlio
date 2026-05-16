#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "tof_slam/mapping/occupancy_grid.hpp"

namespace tof_slam {

/// Result of axis-alignment detection
struct AlignmentResult {
  double angle_rad = 0.0;      ///< Z-axis rotation to align walls to XY axes
  bool aligned = false;         ///< true if alignment was performed
  int num_lines_detected = 0;   ///< Number of line segments detected
  double confidence = 0.0;      ///< Peak votes / total votes (0..1)
};

/// Detect dominant wall direction in OGM and rotate map data for axis alignment.
///
/// Uses Hough Transform to find dominant line direction, then provides
/// coordinate-level rotation utilities to align walls with X/Y axes.
class MapAligner {
 public:
  /// Detect dominant wall direction from assembled OGM.
  /// Returns angle in radians that should be applied to rotate walls
  /// to be axis-aligned. Range: (-pi/4, pi/4].
  /// @param grid Assembled global grid (with occupancy data)
  /// @param occupied_thresh Occupancy value threshold (0-100) for occupied cells
  /// @param min_line_length Minimum line segment length in pixels
  /// @param min_lines Minimum lines required for valid detection
  /// @param min_confidence Minimum confidence for valid alignment
  static AlignmentResult detectAlignment(
      const OccupancyGridGenerator::GlobalGrid& grid,
      int occupied_thresh = 65,
      int min_line_length = 10,
      int min_lines = 10,
      double min_confidence = 0.15);

  /// Rotate all 4x4 SE(3) poses around Z axis by angle_rad.
  /// p' = Rz(-angle) * p,  R' = Rz(-angle) * R
  static void rotatePoses(
      std::vector<Eigen::Matrix4d>& poses,
      double angle_rad);

  /// Rotate a single 4x4 SE(3) pose around Z axis.
  static Eigen::Matrix4d rotatePose(
      const Eigen::Matrix4d& pose,
      double angle_rad);
};

}  // namespace tof_slam
