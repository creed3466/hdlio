#pragma once

#include <array>
#include <Eigen/Core>
#include <Eigen/QR>

namespace tof_slam {

/// Plane coefficients: normal (unit) and signed distance d
/// Plane equation: n^T * p + d = 0
struct PlaneCoeff {
  double a, b, c, d;  // abcd format matching Super-LIO
  bool valid = false;
};

/// Fit a plane to N points (4 or 5) using QR decomposition
/// Returns abcd where [a,b,c] is the unit normal and d = 1/||raw_normal||
/// Validity: all points within 0.1m of fitted plane
/// This is EXACTLY Super-LIO's calc_plane_coeff
PlaneCoeff fitPlane(const std::array<Eigen::Vector3d, 5>& points, int num_points);

/// Compute point-to-plane error and check range-dependent inlier condition
/// Returns true if point_range^2 > 81 * error^2 (Super-LIO outlier rejection)
/// error_out is set to the signed point-to-plane distance
bool computePlaneError(const PlaneCoeff& plane, const Eigen::Vector3d& point_world,
                       double point_body_range, double& error_out);

}  // namespace tof_slam
