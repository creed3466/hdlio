// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "tof_slam/common/lie/se3.hpp"

#include <cmath>

namespace tof_slam {
namespace core {

// ---------------------------------------------------------------------------
// Jacobians
// ---------------------------------------------------------------------------

Eigen::Matrix3f RightJacobian(const Eigen::Vector3f& phi) {
  const float theta = phi.norm();
  if (theta < kEpsilon) {
    return Eigen::Matrix3f::Identity();
  }

  const Eigen::Matrix3f K = Hat(phi / theta);

  // Using unit-axis form K = [phi/theta]_x:
  //   J_r = I - (1 - cos(theta)) / theta * K
  //           + (theta - sin(theta)) / theta * K^2
  const float a = (1.0f - std::cos(theta)) / theta;
  const float b = (theta - std::sin(theta)) / theta;
  return Eigen::Matrix3f::Identity() - a * K + b * (K * K);
}

Eigen::Matrix3f LeftJacobian(const Eigen::Vector3f& phi) {
  // J_l(phi) = J_r(-phi)
  return RightJacobian(-phi);
}

// ---------------------------------------------------------------------------
// Se3 — constructors
// ---------------------------------------------------------------------------

Se3::Se3(const So3& rotation, const Eigen::Vector3f& translation)
    : rotation_(rotation), translation_(translation) {}

Se3::Se3(const Eigen::Matrix3f& R, const Eigen::Vector3f& t)
    : rotation_(R), translation_(t) {}

Se3::Se3(const Eigen::Matrix4f& T)
    : rotation_(T.block<3, 3>(0, 0)), translation_(T.block<3, 1>(0, 3)) {}

Se3 Se3::Identity() { return Se3(); }

// ---------------------------------------------------------------------------
// Se3 — exponential map
// ---------------------------------------------------------------------------

Se3 Se3::Exp(const Eigen::Matrix<float, 6, 1>& xi) {
  const Eigen::Vector3f rho = xi.head<3>();  // translation part
  const Eigen::Vector3f phi = xi.tail<3>();  // rotation part

  const So3 R = So3::Exp(phi);
  const float theta = phi.norm();

  Eigen::Vector3f t;
  if (theta < kEpsilon) {
    // V ≈ I for small rotations.
    t = rho;
  } else {
    // V = I + (1 - cos(theta)) / theta^2 * [phi]_x
    //       + (theta - sin(theta)) / theta^3 * [phi]_x^2
    const Eigen::Matrix3f phi_hat = Hat(phi);
    const float theta2 = theta * theta;
    const float theta3 = theta2 * theta;

    const Eigen::Matrix3f V =
        Eigen::Matrix3f::Identity() +
        ((1.0f - std::cos(theta)) / theta2) * phi_hat +
        ((theta - std::sin(theta)) / theta3) * (phi_hat * phi_hat);

    t = V * rho;
  }

  return Se3(R, t);
}

// ---------------------------------------------------------------------------
// Se3 — logarithmic map
// ---------------------------------------------------------------------------

Eigen::Matrix<float, 6, 1> Se3::Log() const {
  Eigen::Matrix<float, 6, 1> xi;

  const Eigen::Vector3f phi = rotation_.Log();
  const float theta = phi.norm();

  if (theta < kEpsilon) {
    // V^{-1} ≈ I for small rotations.
    xi.head<3>() = translation_;
  } else {
    // V^{-1} = I - 0.5 * [phi]_x + c * [phi]_x^2
    //   where c = (1/theta^2)(1 - theta*cos(theta/2)/(2*sin(theta/2)))
    //           = (2*sin(theta) - theta*(1+cos(theta))) / (2*theta^2*sin(theta))
    const Eigen::Matrix3f phi_hat = Hat(phi);
    const float theta2 = theta * theta;
    const float sin_theta = std::sin(theta);
    const float cos_theta = std::cos(theta);

    const float c = (2.0f * sin_theta - theta * (1.0f + cos_theta)) /
                    (2.0f * theta2 * sin_theta);

    const Eigen::Matrix3f V_inv = Eigen::Matrix3f::Identity() -
                                  0.5f * phi_hat +
                                  c * (phi_hat * phi_hat);

    xi.head<3>() = V_inv * translation_;
  }

  xi.tail<3>() = phi;
  return xi;
}

// ---------------------------------------------------------------------------
// Se3 — group operations
// ---------------------------------------------------------------------------

Se3 Se3::operator*(const Se3& rhs) const {
  return Se3(rotation_ * rhs.rotation_,
             translation_ + rotation_ * rhs.translation_);
}

Eigen::Vector3f Se3::operator*(const Eigen::Vector3f& p) const {
  return rotation_ * p + translation_;
}

Se3 Se3::inverse() const {
  So3 R_inv = rotation_.inverse();
  return Se3(R_inv, R_inv * (-translation_));
}

Eigen::Matrix4f Se3::matrix() const {
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T.block<3, 3>(0, 0) = rotation_.matrix();
  T.block<3, 1>(0, 3) = translation_;
  return T;
}

}  // namespace core
}  // namespace tof_slam
