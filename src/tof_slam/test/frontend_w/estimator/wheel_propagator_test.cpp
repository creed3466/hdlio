// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include "tof_slam/frontend_w/estimator/lwo_state.hpp"
#include "tof_slam/frontend_w/estimator/wheel_propagator.hpp"

namespace tof_slam {
namespace lwo {
namespace {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

LwoState identity_state() {
  LwoState s;
  s.reset();
  return s;
}

// ---------------------------------------------------------------------------
// Stationary: no motion -> state unchanged (except covariance grows)
// ---------------------------------------------------------------------------

TEST(WheelPropagator, StationaryNoMovement) {
  WheelPropagator prop;
  LwoState s = identity_state();
  const LwoState result = prop.propagate(s, 0.0f, 0.0f, 0.1f);

  EXPECT_TRUE(result.rotation.isApprox(Eigen::Matrix3f::Identity(), 1e-5f));
  EXPECT_TRUE(result.position.isZero(1e-5f));
  EXPECT_TRUE(result.velocity.isZero(1e-5f));
  EXPECT_FLOAT_EQ(result.wheel_scale, 1.0f);
}

// ---------------------------------------------------------------------------
// Straight line: pure forward motion accumulates position
// ---------------------------------------------------------------------------

TEST(WheelPropagator, StraightLineForward) {
  WheelPropagator prop;
  LwoState s = identity_state();

  const float vx  = 1.0f;   // 1 m/s
  const float wz  = 0.0f;
  const float dt  = 0.1f;
  const int   N   = 10;

  for (int i = 0; i < N; ++i) {
    s = prop.propagate(s, vx, wz, dt);
  }

  // Trapezoidal: step 0 contributes 0.5*(0+1)*0.1=0.05m, steps 1-9 each 0.1m -> 0.95m
  EXPECT_NEAR(s.position(0), 0.95f, 0.02f)
      << "x position after 1s straight-line: midpoint integration gives 0.95m";
  EXPECT_NEAR(s.position(1), 0.0f, 1e-4f);
  EXPECT_NEAR(s.position(2), 0.0f, 1e-4f);
}

// ---------------------------------------------------------------------------
// Pure rotation: forward=0, omega_z != 0 -> rotation accumulates, pos ~0
// ---------------------------------------------------------------------------

TEST(WheelPropagator, PureRotation) {
  WheelPropagator prop;
  LwoState s = identity_state();

  const float omega = static_cast<float>(M_PI / 2.0);  // 90 deg/s
  const float dt    = 0.01f;
  const int   N     = 100;  // 1 second total -> 90 deg rotation

  for (int i = 0; i < N; ++i) {
    s = prop.propagate(s, 0.0f, omega, dt);
  }

  // After 1s at pi/2 rad/s around Z: yaw should be ~pi/2
  const core::So3 R(s.rotation);
  const Eigen::Vector3f rpy = R.Log();  // approximate roll/pitch/yaw
  EXPECT_NEAR(rpy(2), static_cast<float>(M_PI / 2.0), 0.05f)
      << "Yaw should be ~90 deg after 1s pure rotation";
  EXPECT_NEAR(s.position.norm(), 0.0f, 0.01f)
      << "Position should be near zero during pure rotation";
}

// ---------------------------------------------------------------------------
// Covariance grows with time (no observations)
// ---------------------------------------------------------------------------

TEST(WheelPropagator, CovarianceGrowsOverTime) {
  WheelPropagator prop;
  LwoState s = identity_state();

  const float rot_before = s.covariance.block<3, 3>(kLwoRotIdx, kLwoRotIdx).trace();
  const float pos_before = s.covariance.block<3, 3>(kLwoPosIdx, kLwoPosIdx).trace();
  s = prop.propagate(s, 0.5f, 0.1f, 0.1f);
  const float rot_after = s.covariance.block<3, 3>(kLwoRotIdx, kLwoRotIdx).trace();
  const float pos_after = s.covariance.block<3, 3>(kLwoPosIdx, kLwoPosIdx).trace();

  // With F[vel,vel]=0, velocity covariance correctly SHRINKS (velocity is
  // directly assigned from encoder, not accumulated), so total trace may
  // decrease.  Rotation and position covariances grow due to process noise.
  EXPECT_GT(rot_after, rot_before)
      << "Rotation covariance should grow due to process noise";
  EXPECT_GT(pos_after, pos_before)
      << "Position covariance should grow due to process noise";
}

// ---------------------------------------------------------------------------
// Finite-difference check on F_w (rotation block)
// ---------------------------------------------------------------------------

TEST(WheelPropagator, TransitionMatrixFiniteDifference) {
  WheelPropagator prop;
  LwoState s = identity_state();
  // Set a non-trivial rotation.
  s.rotation = core::So3::Exp(Eigen::Vector3f(0.1f, 0.05f, 0.3f)).matrix();
  s.wheel_scale = 1.05f;

  const float vx   = 0.5f;
  const float wz   = 0.2f;
  const float dt   = 0.05f;
  const float eps  = 1e-4f;

  const LwoStateCovariance F =
      prop.build_transition_matrix(s, vx, wz, dt);
  const LwoState s_nominal = prop.propagate(s, vx, wz, dt);

  // Check selected columns via finite differences.
  for (int col : {0, 1, 2, 3, 4, 5, 9, 10, 11}) {
    LwoStateVector perturb = LwoStateVector::Zero();
    perturb(col) = eps;

    LwoState s_plus = s + perturb;
    s_plus.covariance = s.covariance;  // keep same P for propagation
    const LwoState x_plus = prop.propagate(s_plus, vx, wz, dt);

    const LwoStateVector dx_fd = (x_plus - s_nominal) / eps;
    const LwoStateVector dx_an = F.col(col);

    for (int row = 0; row < kLwoStateDim; ++row) {
      EXPECT_NEAR(dx_fd(row), dx_an(row), 5e-3f)
          << "F[" << row << "," << col << "] mismatch";
    }
  }
}

}  // namespace
}  // namespace lwo
}  // namespace tof_slam
