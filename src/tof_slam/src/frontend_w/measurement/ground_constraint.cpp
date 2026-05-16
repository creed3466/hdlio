// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "tof_slam/frontend_w/measurement/ground_constraint.hpp"

#include "tof_slam/common/lie/so3.hpp"

namespace tof_slam {
namespace lwo {

GroundConstraint::GroundConstraint(const GroundConstraintConfig& cfg)
    : cfg_(cfg) {}

// ---------------------------------------------------------------------------
// jacobian
// ---------------------------------------------------------------------------

Eigen::Matrix<float, 3, kLwoStateDim> GroundConstraint::jacobian() const {
  Eigen::Matrix<float, 3, kLwoStateDim> H =
      Eigen::Matrix<float, 3, kLwoStateDim>::Zero();

  // Row 0: d(phi_roll)/d(phi) = [1, 0, 0]  -> column 0
  H(0, kLwoRotIdx + 0) = 1.0f;
  // Row 1: d(phi_pitch)/d(phi) = [0, 1, 0]  -> column 1
  H(1, kLwoRotIdx + 1) = 1.0f;
  // Row 2: d(p_z)/d(p) = [0, 0, 1]  -> column 5
  H(2, kLwoPosIdx + 2) = 1.0f;

  return H;
}

// ---------------------------------------------------------------------------
// noise
// ---------------------------------------------------------------------------

Eigen::Matrix3f GroundConstraint::noise() const {
  const float sr = cfg_.noise_roll_pitch;
  const float sz = cfg_.noise_z;
  return Eigen::DiagonalMatrix<float, 3>(sr * sr, sr * sr, sz * sz).toDenseMatrix();
}

// ---------------------------------------------------------------------------
// compute
// ---------------------------------------------------------------------------

GroundConstraintResult GroundConstraint::compute(const LwoState& state) const {
  GroundConstraintResult result;
  result.noise_cov = noise();

  // Extract roll/pitch/yaw via SO(3)::Log.
  const Eigen::Vector3f phi = core::So3(state.rotation).Log();

  // Compute inverse right Jacobian J_r^{-1}(phi) for the rotation block.
  // First-order: J_r^{-1} ≈ I + 0.5 * Hat(phi), exact for small angles.
  // This accounts for the nonlinear coupling between roll, pitch, yaw
  // components of Log(R) when R is non-trivial.
  const float theta2 = phi.squaredNorm();
  Eigen::Matrix3f J_r_inv;
  if (theta2 < 1e-6f) {
    J_r_inv = Eigen::Matrix3f::Identity();
  } else {
    J_r_inv = Eigen::Matrix3f::Identity() + 0.5f * core::Hat(phi);
  }

  // Build state-dependent Jacobian H (3 x kLwoStateDim).
  result.H = Eigen::Matrix<float, 3, kLwoStateDim>::Zero();
  // Roll and pitch rows: first two rows of J_r_inv over rotation columns.
  result.H.block<2, 3>(0, kLwoRotIdx) = J_r_inv.topRows<2>();
  // Z-position row: direct.
  result.H(2, kLwoPosIdx + 2) = 1.0f;

  // Residuals: target = 0 for roll, pitch, z.
  result.residual(0) = -phi(0);
  result.residual(1) = -phi(1);
  result.residual(2) = -state.position(2);

  return result;
}

}  // namespace lwo
}  // namespace tof_slam
