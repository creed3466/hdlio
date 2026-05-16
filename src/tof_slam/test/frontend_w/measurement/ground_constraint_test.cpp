// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "tof_slam/frontend_w/estimator/lwo_state.hpp"
#include "tof_slam/frontend_w/measurement/ground_constraint.hpp"

namespace tof_slam {
namespace lwo {
namespace {

// ---------------------------------------------------------------------------
// Identity state -> zero residual
// ---------------------------------------------------------------------------

TEST(GroundConstraint, IdentityStateZeroResidual) {
  GroundConstraint gc;
  LwoState s;
  const auto res = gc.compute(s);

  EXPECT_NEAR(res.residual(0), 0.0f, 1e-5f) << "roll residual";
  EXPECT_NEAR(res.residual(1), 0.0f, 1e-5f) << "pitch residual";
  EXPECT_NEAR(res.residual(2), 0.0f, 1e-5f) << "z residual";
}

// ---------------------------------------------------------------------------
// Non-zero z position -> non-zero z residual
// ---------------------------------------------------------------------------

TEST(GroundConstraint, ZPositionResidual) {
  GroundConstraint gc;
  LwoState s;
  s.position(2) = 0.5f;  // 50 cm above ground
  const auto res = gc.compute(s);

  EXPECT_NEAR(res.residual(2), -0.5f, 1e-5f) << "z residual = -p_z";
}

// ---------------------------------------------------------------------------
// Jacobian structure: only cols 0, 1, 5 are non-zero
// ---------------------------------------------------------------------------

TEST(GroundConstraint, JacobianStructure) {
  GroundConstraint gc;
  const auto H = gc.jacobian();

  // Row 0 (roll): only col 0 nonzero
  EXPECT_FLOAT_EQ(H(0, 0), 1.0f);
  for (int j = 1; j < kLwoStateDim; ++j) {
    EXPECT_FLOAT_EQ(H(0, j), 0.0f) << "Row 0, col " << j;
  }

  // Row 1 (pitch): only col 1 nonzero
  EXPECT_FLOAT_EQ(H(1, 1), 1.0f);
  for (int j = 0; j < kLwoStateDim; ++j) {
    if (j != 1) EXPECT_FLOAT_EQ(H(1, j), 0.0f) << "Row 1, col " << j;
  }

  // Row 2 (z): only col 5 nonzero
  EXPECT_FLOAT_EQ(H(2, 5), 1.0f);
  for (int j = 0; j < kLwoStateDim; ++j) {
    if (j != 5) EXPECT_FLOAT_EQ(H(2, j), 0.0f) << "Row 2, col " << j;
  }
}

// ---------------------------------------------------------------------------
// Noise covariance is diagonal and positive
// ---------------------------------------------------------------------------

TEST(GroundConstraint, NoiseDiagonalPositive) {
  GroundConstraintConfig cfg;
  cfg.noise_roll_pitch = 0.01f;
  cfg.noise_z          = 0.001f;
  GroundConstraint gc(cfg);

  const Eigen::Matrix3f R = gc.noise();
  EXPECT_NEAR(R(0, 0), 0.01f * 0.01f, 1e-10f);
  EXPECT_NEAR(R(1, 1), 0.01f * 0.01f, 1e-10f);
  EXPECT_NEAR(R(2, 2), 0.001f * 0.001f, 1e-10f);
  // Off-diagonal should be zero.
  EXPECT_FLOAT_EQ(R(0, 1), 0.0f);
  EXPECT_FLOAT_EQ(R(0, 2), 0.0f);
}

// ---------------------------------------------------------------------------
// Finite-difference check on Jacobian (roll/pitch rows via rotation)
// ---------------------------------------------------------------------------

TEST(GroundConstraint, JacobianFiniteDifference) {
  GroundConstraint gc;
  LwoState s;
  // Small roll rotation to make residual non-trivial.
  s.rotation = core::So3::Exp(Eigen::Vector3f(0.05f, 0.03f, 0.0f)).matrix();

  const auto res_0 = gc.compute(s);
  const float eps = 1e-4f;

  // Check rotation columns (0, 1, 2).
  for (int col = 0; col < 3; ++col) {
    LwoStateVector perturb = LwoStateVector::Zero();
    perturb(col) = eps;
    LwoState s_p = s + perturb;
    const auto res_p = gc.compute(s_p);

    for (int row = 0; row < 3; ++row) {
      const float fd = (res_p.residual(row) - res_0.residual(row)) / eps;
      // H = dr/dx, but our H is d(state_component)/d(state),
      // so compare directly.
      EXPECT_NEAR(fd, -res_0.H(row, col), 0.01f)
          << "H[" << row << "," << col << "] finite-diff mismatch";
    }
  }
}

}  // namespace
}  // namespace lwo
}  // namespace tof_slam
