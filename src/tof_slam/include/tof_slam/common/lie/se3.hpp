// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// se3.hpp — SE(3) Lie group operations (float32).
//
// Represents rigid-body transformations as (So3, translation).  Provides
// exponential/logarithmic maps, composition, inverse, and Jacobians
// needed by the LIO estimator and pose-graph backend.

#ifndef TOF_SLAM_COMMON_LIE_SE3_HPP_
#define TOF_SLAM_COMMON_LIE_SE3_HPP_

#include <Eigen/Core>

#include "tof_slam/common/lie/so3.hpp"

namespace tof_slam {
namespace core {

// ---------------------------------------------------------------------------
// Jacobians  (needed by ESKF covariance propagation)
// ---------------------------------------------------------------------------

/// Right Jacobian of SO(3):
///   J_r(phi) = I - (1-cos||phi||)/||phi||^2 * [phi]_x
///              + (||phi||-sin||phi||)/||phi||^3 * [phi]_x^2
Eigen::Matrix3f RightJacobian(const Eigen::Vector3f& phi);

/// Left Jacobian of SO(3):  J_l(phi) = J_r(-phi).
Eigen::Matrix3f LeftJacobian(const Eigen::Vector3f& phi);

// ---------------------------------------------------------------------------
// Se3 — rigid-body transformation
// ---------------------------------------------------------------------------
class Se3 {
 public:
  // -- Constructors ---------------------------------------------------------

  /// Default: identity transform.
  Se3() = default;

  /// From rotation + translation.
  Se3(const So3& rotation, const Eigen::Vector3f& translation);

  /// From raw matrices.
  Se3(const Eigen::Matrix3f& R, const Eigen::Vector3f& t);

  /// From 4x4 homogeneous matrix (extracts R and t; R is SVD-projected).
  explicit Se3(const Eigen::Matrix4f& T);

  /// Named constructor — identity element.
  static Se3 Identity();

  // -- Exponential / logarithmic maps --------------------------------------

  /// 6-D twist [rho; phi] -> SE(3).
  /// Convention: rho = translational part, phi = rotational part.
  static Se3 Exp(const Eigen::Matrix<float, 6, 1>& xi);

  /// SE(3) -> 6-D twist [rho; phi].
  Eigen::Matrix<float, 6, 1> Log() const;

  // -- Group operations ----------------------------------------------------

  /// Composition  T_this * T_rhs.
  Se3 operator*(const Se3& rhs) const;

  /// Transform a point  R * p + t.
  Eigen::Vector3f operator*(const Eigen::Vector3f& p) const;

  /// Group inverse.
  Se3 inverse() const;

  // -- Accessors -----------------------------------------------------------

  const So3& rotation() const { return rotation_; }
  So3& rotation() { return rotation_; }

  Eigen::Matrix3f rotation_matrix() const { return rotation_.matrix(); }

  const Eigen::Vector3f& translation() const { return translation_; }
  Eigen::Vector3f& translation() { return translation_; }

  /// Build 4x4 homogeneous matrix.
  Eigen::Matrix4f matrix() const;

 private:
  So3 rotation_;
  Eigen::Vector3f translation_ = Eigen::Vector3f::Zero();
};

}  // namespace core
}  // namespace tof_slam

#endif  // TOF_SLAM_COMMON_LIE_SE3_HPP_
