#pragma once

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Cholesky>

namespace tof_slam {
namespace math {

/// Normalize angle to [-pi, pi]
inline double normalizeAngle(double angle) {
  // Use remainder for robust wrapping
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0.0) {
    angle += 2.0 * M_PI;
  }
  return angle - M_PI;
}

/// Clamp value to range [min_val, max_val]
inline double clamp(double val, double min_val, double max_val) {
  if (val < min_val) return min_val;
  if (val > max_val) return max_val;
  return val;
}

/// Check if matrix is positive definite (via Cholesky decomposition)
inline bool isPositiveDefinite(const Eigen::MatrixXd& M) {
  if (M.rows() != M.cols()) return false;
  Eigen::LLT<Eigen::MatrixXd> llt(M);
  return llt.info() == Eigen::Success;
}

/// Force matrix to be symmetric: M = (M + M^T) / 2
inline Eigen::MatrixXd symmetrize(const Eigen::MatrixXd& M) {
  return (M + M.transpose()) * 0.5;
}

}  // namespace math
}  // namespace tof_slam
