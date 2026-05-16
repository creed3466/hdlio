// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// lidar_jacobian_test.cpp — Unit tests for compute_lidar_jacobians().

#include "tof_slam/frontend/estimator/lidar_jacobian.hpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <vector>

#include "tof_slam/frontend/estimator/correspondence.hpp"
#include "tof_slam/common/lie/se3.hpp"
#include "tof_slam/common/lie/so3.hpp"
#include "tof_slam/common/types/state.hpp"

namespace tof_slam {
namespace core {
namespace {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Build a default identity state (R=I, p=0).
LioState identity_state() {
  LioState s;
  s.rotation = Eigen::Matrix3f::Identity();
  s.position = Eigen::Vector3f::Zero();
  return s;
}

/// Build a trivial extrinsic (identity).
Se3 identity_extrinsic() { return Se3::Identity(); }

/// Build one horizontal-plane correspondence: normal = (0,0,1), plane_d = 0.
/// A point at height h has residual -h.
Correspondence horizontal_plane_corr(const Eigen::Vector3f& p_lidar) {
  Correspondence c;
  c.p_lidar = p_lidar;
  c.normal  = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
  c.plane_d = 0.0f;
  return c;
}

// ---------------------------------------------------------------------------
// TEST: OutputDimensions — H must be N×18 and residuals must be length N.
// ---------------------------------------------------------------------------
TEST(LidarJacobianTest, OutputDimensions) {
  LioState state = identity_state();
  Se3 extrinsic  = identity_extrinsic();

  const int N = 5;
  std::vector<Correspondence> corrs(N, horizontal_plane_corr({1.f, 0.f, 2.f}));

  Eigen::MatrixXf H;
  Eigen::VectorXf r;
  compute_lidar_jacobians(state, extrinsic, corrs, H, r);

  EXPECT_EQ(H.rows(), N);
  EXPECT_EQ(H.cols(), kStateDim);
  EXPECT_EQ(r.size(), N);
}

// ---------------------------------------------------------------------------
// TEST: EmptyCorrespondences — H is 0×18, residuals is empty.
// ---------------------------------------------------------------------------
TEST(LidarJacobianTest, EmptyCorrespondences) {
  LioState state = identity_state();
  Se3 extrinsic  = identity_extrinsic();

  std::vector<Correspondence> corrs;

  Eigen::MatrixXf H;
  Eigen::VectorXf r;
  compute_lidar_jacobians(state, extrinsic, corrs, H, r);

  EXPECT_EQ(H.rows(), 0);
  EXPECT_EQ(H.cols(), kStateDim);
  EXPECT_EQ(r.size(), 0);
}

// ---------------------------------------------------------------------------
// TEST: ResidualSign — point above the horizontal plane (z>0) should give
//       a negative residual (residual = -(n^T*p_world + d) = -z).
// ---------------------------------------------------------------------------
TEST(LidarJacobianTest, ResidualSign) {
  LioState state = identity_state();
  Se3 extrinsic  = identity_extrinsic();

  // Point at (0, 0, 3) in world frame (identity extrinsic+state).
  Correspondence c = horizontal_plane_corr({0.f, 0.f, 3.f});
  std::vector<Correspondence> corrs{c};

  Eigen::MatrixXf H;
  Eigen::VectorXf r;
  compute_lidar_jacobians(state, extrinsic, corrs, H, r);

  // residual = -(n^T * p_world - 0) = -(0*0 + 0*0 + 1*3) = -3
  EXPECT_NEAR(r(0), -3.0f, 1e-5f);
}

// ---------------------------------------------------------------------------
// TEST: PointOnPlane — point exactly on the plane gives residual ≈ 0.
// ---------------------------------------------------------------------------
TEST(LidarJacobianTest, PointOnPlane) {
  LioState state = identity_state();
  Se3 extrinsic  = identity_extrinsic();

  // plane z = 0, point at z = 0 → residual = 0
  Correspondence c = horizontal_plane_corr({1.0f, 2.0f, 0.0f});
  std::vector<Correspondence> corrs{c};

  Eigen::MatrixXf H;
  Eigen::VectorXf r;
  compute_lidar_jacobians(state, extrinsic, corrs, H, r);

  EXPECT_NEAR(r(0), 0.0f, 1e-5f);
}

// ---------------------------------------------------------------------------
// TEST: IdentityPose — with identity state and extrinsic, p_world = p_lidar.
// Use a slanted plane (normal not axis-aligned) to exercise full math.
// ---------------------------------------------------------------------------
TEST(LidarJacobianTest, IdentityPose) {
  LioState state = identity_state();
  Se3 extrinsic  = identity_extrinsic();

  // Plane normal = (1/√2, 1/√2, 0), plane_d = -2 (plane passes through (2,0,0))
  const float inv_sqrt2 = 1.0f / std::sqrt(2.0f);
  Correspondence c;
  c.p_lidar = Eigen::Vector3f(3.0f, 1.0f, 0.0f);
  c.normal  = Eigen::Vector3f(inv_sqrt2, inv_sqrt2, 0.0f);
  c.plane_d = -2.0f;

  // Expected: p_world = p_lidar = (3,1,0)
  // residual = -(n^T * p_world - plane_d) = -(4/√2 - (-2)) = -(4/√2 + 2)
  const float expected_r = -(c.normal.dot(c.p_lidar) - c.plane_d);

  std::vector<Correspondence> corrs{c};
  Eigen::MatrixXf H;
  Eigen::VectorXf r;
  compute_lidar_jacobians(state, extrinsic, corrs, H, r);

  EXPECT_NEAR(r(0), expected_r, 1e-4f);
}

// ---------------------------------------------------------------------------
// TEST: PositionOnlyNonzero — columns 6–17 must be all zero.
// ---------------------------------------------------------------------------
TEST(LidarJacobianTest, PositionOnlyNonzero) {
  LioState state = identity_state();
  Se3 extrinsic  = identity_extrinsic();

  Correspondence c;
  c.p_lidar = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
  c.normal  = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
  c.plane_d = 0.0f;
  std::vector<Correspondence> corrs{c};

  Eigen::MatrixXf H;
  Eigen::VectorXf r;
  compute_lidar_jacobians(state, extrinsic, corrs, H, r);

  // Columns 6–17 must be zero.
  const float tail_norm = H.block<1, 12>(0, 6).norm();
  EXPECT_NEAR(tail_norm, 0.0f, 1e-7f);

  // Columns 0–5 must not all be zero (rotation + position blocks).
  const float active_norm = H.block<1, 6>(0, 0).norm();
  EXPECT_GT(active_norm, 1e-4f);
}

// ---------------------------------------------------------------------------
// TEST: FiniteDifferenceRotation — numerical gradient (rotation) vs. H[:,0:3].
// ---------------------------------------------------------------------------
TEST(LidarJacobianTest, FiniteDifferenceRotation) {
  // Non-trivial state: 10° rotation around z-axis, position offset.
  LioState state;
  const float theta = 0.1745f;  // ~10°
  state.rotation = So3::Exp(Eigen::Vector3f(0.f, 0.f, theta)).matrix();
  state.position = Eigen::Vector3f(0.5f, -0.3f, 0.1f);

  // Non-trivial extrinsic.
  Se3 extrinsic(So3::Exp(Eigen::Vector3f(0.01f, 0.02f, 0.03f)),
                Eigen::Vector3f(0.05f, 0.0f, -0.02f));

  Correspondence c;
  c.p_lidar = Eigen::Vector3f(2.0f, 1.0f, 0.5f);
  c.normal  = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
  c.plane_d = -1.0f;
  std::vector<Correspondence> corrs{c};

  Eigen::MatrixXf H;
  Eigen::VectorXf r0;
  compute_lidar_jacobians(state, extrinsic, corrs, H, r0);

  // Numerical finite difference along each rotation axis.
  // tol is 2e-3 (not 1e-3) because float32 FD with step 1e-4 has ~1e-3
  // truncation error near this operating point.
  const float delta = 1e-4f;
  const float tol   = 2e-3f;

  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3f perturb = Eigen::Vector3f::Zero();
    perturb(axis) = delta;

    // Perturb rotation: R_new = R * Exp(perturb)
    LioState state_p = state;
    state_p.rotation = state.rotation * So3::Exp(perturb).matrix();

    Eigen::MatrixXf H_p;
    Eigen::VectorXf r_p;
    compute_lidar_jacobians(state_p, extrinsic, corrs, H_p, r_p);

    const float fd_col = (r_p(0) - r0(0)) / delta;
    EXPECT_NEAR(fd_col, H(0, kRotIdx + axis), tol)
        << "FD vs analytical Jacobian mismatch at rotation axis " << axis;
  }
}

// ---------------------------------------------------------------------------
// TEST: FiniteDifferencePosition — numerical gradient (position) vs. H[:,3:6].
// ---------------------------------------------------------------------------
TEST(LidarJacobianTest, FiniteDifferencePosition) {
  LioState state;
  state.rotation = So3::Exp(Eigen::Vector3f(0.05f, -0.1f, 0.2f)).matrix();
  state.position = Eigen::Vector3f(1.0f, 0.0f, 0.0f);

  Se3 extrinsic = identity_extrinsic();

  Correspondence c;
  c.p_lidar = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
  c.normal  = Eigen::Vector3f(0.0f, 1.0f, 0.0f);  // y-normal plane
  c.plane_d = -0.5f;
  std::vector<Correspondence> corrs{c};

  Eigen::MatrixXf H;
  Eigen::VectorXf r0;
  compute_lidar_jacobians(state, extrinsic, corrs, H, r0);

  const float delta = 1e-4f;
  const float tol   = 1e-3f;

  for (int axis = 0; axis < 3; ++axis) {
    LioState state_p = state;
    state_p.position(axis) += delta;

    Eigen::MatrixXf H_p;
    Eigen::VectorXf r_p;
    compute_lidar_jacobians(state_p, extrinsic, corrs, H_p, r_p);

    const float fd_col = (r_p(0) - r0(0)) / delta;
    EXPECT_NEAR(fd_col, H(0, kPosIdx + axis), tol)
        << "FD vs analytical Jacobian mismatch at position axis " << axis;
  }
}

}  // namespace
}  // namespace core
}  // namespace tof_slam
