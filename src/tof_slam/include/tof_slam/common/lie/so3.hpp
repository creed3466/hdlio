// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// so3.hpp — SO(3) Lie group operations (float32).
//
// Provides the exponential/logarithmic maps, group composition, and
// skew-symmetric utilities needed by the LIO estimator.  All operations
// are in single-precision to match the 18-D EKF state.

#ifndef TOF_SLAM_COMMON_LIE_SO3_HPP_
#define TOF_SLAM_COMMON_LIE_SO3_HPP_

#include <Eigen/Dense>

namespace tof_slam {
namespace core {

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
inline constexpr float kEpsilon = 1e-6f;
inline constexpr float kPi = 3.14159265358979323846f;

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// Skew-symmetric (hat) operator:  R^3 -> so(3).
Eigen::Matrix3f Hat(const Eigen::Vector3f& v);

/// Inverse of Hat:  so(3) -> R^3.
Eigen::Vector3f Vee(const Eigen::Matrix3f& S);


// ---------------------------------------------------------------------------
// So3 — 3-D rotation as a 3x3 matrix
// ---------------------------------------------------------------------------
class So3 {
 public:
  // -- Constructors ---------------------------------------------------------

  /// Default: identity rotation.
  So3() = default;

  /// Construct from a rotation matrix.  The matrix is projected onto SO(3)
  /// via SVD to correct numerical drift.
  explicit So3(const Eigen::Matrix3f& R);

  /// Named constructor — identity element.
  static So3 Identity();

  // -- Exponential / logarithmic maps --------------------------------------

  /// Axis-angle -> rotation matrix  (Rodrigues formula).
  static So3 Exp(const Eigen::Vector3f& omega);

  /// Rotation matrix -> axis-angle vector.
  Eigen::Vector3f Log() const;

  // -- Group operations ----------------------------------------------------

  /// Composition  R_this * R_rhs.
  So3 operator*(const So3& rhs) const;

  /// Rotate a vector  R * v.
  Eigen::Vector3f operator*(const Eigen::Vector3f& v) const;

  /// Group inverse  R^T.
  So3 inverse() const;

  // -- Accessors -----------------------------------------------------------

  /// Underlying 3x3 matrix (const).
  const Eigen::Matrix3f& matrix() const { return matrix_; }

  /// Underlying 3x3 matrix (mutable — use with caution, call normalize()
  /// after manual edits).
  Eigen::Matrix3f& matrix() { return matrix_; }

  // -- Maintenance ---------------------------------------------------------

  /// Project back onto SO(3) via SVD (corrects drift from repeated
  /// multiplications).
  void normalize();

 private:
  Eigen::Matrix3f matrix_ = Eigen::Matrix3f::Identity();
};

}  // namespace core
}  // namespace tof_slam

#endif  // TOF_SLAM_COMMON_LIE_SO3_HPP_
