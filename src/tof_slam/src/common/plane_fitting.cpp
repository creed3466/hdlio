#include "tof_slam/common/plane_fitting.hpp"
#include <cmath>

namespace tof_slam {

PlaneCoeff fitPlane(const std::array<Eigen::Vector3d, 5>& points, int N) {
  PlaneCoeff result;
  Eigen::Vector3d normvec;

  if (N == 5) {
    Eigen::Matrix<double, 5, 3> A;
    Eigen::Matrix<double, 5, 1> b;
    for (int j = 0; j < 5; j++) {
      A.row(j) = points[j].transpose();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  } else {
    Eigen::Matrix<double, 4, 3> A;
    Eigen::Matrix<double, 4, 1> b;
    for (int j = 0; j < N; j++) {
      A.row(j) = points[j].transpose();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  }

  double n = normvec.norm();
  if (n < 1e-6) { result.valid = false; return result; }

  result.d = 1.0 / n;
  normvec *= result.d;  // normalize
  result.a = normvec[0];
  result.b = normvec[1];
  result.c = normvec[2];

  // Validity check: all points within 0.1m
  for (int i = 0; i < N; i++) {
    double dist = result.a * points[i](0) + result.b * points[i](1) + result.c * points[i](2) + result.d;
    if (std::abs(dist) > 0.1) { result.valid = false; return result; }
  }
  result.valid = true;
  return result;
}

bool computePlaneError(const PlaneCoeff& plane, const Eigen::Vector3d& pw,
                       double body_range, double& error_out) {
  error_out = plane.a * pw[0] + plane.b * pw[1] + plane.c * pw[2] + plane.d;
  return body_range > 81.0 * error_out * error_out;
}

}  // namespace tof_slam
