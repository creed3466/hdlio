// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "tof_slam/common/lie/so3.hpp"

#include <Eigen/SVD>
#include <cmath>

namespace tof_slam {
namespace core {

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

Eigen::Matrix3f Hat(const Eigen::Vector3f& v) {
  Eigen::Matrix3f S;
  // clang-format off
  S <<    0.0f, -v(2),  v(1),
         v(2),   0.0f, -v(0),
        -v(1),  v(0),   0.0f;
  // clang-format on
  return S;
}

Eigen::Vector3f Vee(const Eigen::Matrix3f& S) {
  return Eigen::Vector3f(S(2, 1), S(0, 2), S(1, 0));
}

// ---------------------------------------------------------------------------
// So3 — constructors
// ---------------------------------------------------------------------------

So3::So3(const Eigen::Matrix3f& R) {
  // Project onto SO(3) via SVD:  R_proj = U * V^T  with det = +1.
  Eigen::JacobiSVD<Eigen::Matrix3f> svd(
      R, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3f U = svd.matrixU();
  const Eigen::Matrix3f& V = svd.matrixV();

  matrix_ = U * V.transpose();

  // Ensure proper rotation (det = +1, not reflection).
  if (matrix_.determinant() < 0.0f) {
    U.col(2) *= -1.0f;
    matrix_ = U * V.transpose();
  }
}

So3 So3::Identity() { return So3(); }

// ---------------------------------------------------------------------------
// So3 — exponential map  (Rodrigues formula)
// ---------------------------------------------------------------------------

So3 So3::Exp(const Eigen::Vector3f& omega) {
  const float theta = omega.norm();

  if (theta < kEpsilon) {
    // First-order approximation:  R ≈ I + [omega]_x
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity() + Hat(omega);
    return So3(R);  // SVD-project to guarantee valid SO(3)
  }

  const float inv_theta = 1.0f / theta;
  const Eigen::Vector3f k = omega * inv_theta;  // unit rotation axis
  const Eigen::Matrix3f K = Hat(k);

  // R = I + sin(theta) * K + (1 - cos(theta)) * K^2
  Eigen::Matrix3f R = Eigen::Matrix3f::Identity() +
                      std::sin(theta) * K +
                      (1.0f - std::cos(theta)) * (K * K);
  // Direct construction — Rodrigues already produces a valid rotation.
  So3 result;
  result.matrix_ = R;
  return result;
}

// ---------------------------------------------------------------------------
// So3 — logarithmic map
// ---------------------------------------------------------------------------

Eigen::Vector3f So3::Log() const {
  const float cos_theta =
      std::clamp((matrix_.trace() - 1.0f) * 0.5f, -1.0f, 1.0f);
  const float theta = std::acos(cos_theta);

  // Case 1: theta ≈ 0  —  small-angle approximation.
  if (theta < kEpsilon) {
    return Vee(matrix_ - Eigen::Matrix3f::Identity());
  }

  const float sin_theta = std::sin(theta);

  // Case 2: theta near pi  —  sin(theta) is small, the general formula
  // theta/(2*sin_theta) amplifies numerical noise.  Switch to eigenvector
  // extraction from (R + I) = 2 * n * n^T when |sin_theta| < threshold.
  // Using 1e-2f instead of kEpsilon because float32 Rodrigues accumulates
  // errors proportional to the amplification factor theta/sin_theta.
  if (std::abs(sin_theta) < 1e-2f) {
    // (R + I)/2 = n * n^T.  Extract axis from the largest diagonal entry.
    const Eigen::Matrix3f RpI = matrix_ + Eigen::Matrix3f::Identity();
    Eigen::Vector3f axis;
    int max_idx = 0;
    if (RpI(1, 1) > RpI(0, 0)) max_idx = 1;
    if (RpI(2, 2) > RpI(max_idx, max_idx)) max_idx = 2;

    axis[max_idx] = std::sqrt(std::max(0.0f, RpI(max_idx, max_idx) * 0.5f));

    if (axis[max_idx] > kEpsilon) {
      for (int i = 0; i < 3; ++i) {
        if (i != max_idx) {
          // Use average of symmetric elements for better precision.
          axis[i] = (RpI(max_idx, i) + RpI(i, max_idx)) * 0.25f *
                    (1.0f / axis[max_idx]);
        }
      }
    }

    // Disambiguate sign: axis should be consistent with Vee(R - R^T).
    const Eigen::Vector3f skew = Vee(matrix_ - matrix_.transpose());
    if (skew.dot(axis) < 0.0f) {
      axis = -axis;
    }

    return axis.normalized() * theta;
  }

  // Case 3: general case.
  const float factor = theta / (2.0f * sin_theta);
  return factor * Vee(matrix_ - matrix_.transpose());
}

// ---------------------------------------------------------------------------
// So3 — group operations
// ---------------------------------------------------------------------------

So3 So3::operator*(const So3& rhs) const {
  So3 result;
  result.matrix_ = matrix_ * rhs.matrix_;
  return result;
}

Eigen::Vector3f So3::operator*(const Eigen::Vector3f& v) const {
  return matrix_ * v;
}

So3 So3::inverse() const {
  So3 result;
  result.matrix_ = matrix_.transpose();
  return result;
}

void So3::normalize() { *this = So3(matrix_); }

}  // namespace core
}  // namespace tof_slam
