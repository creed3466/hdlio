// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include "tof_slam/frontend_w/estimator/lwo_state.hpp"

namespace tof_slam {
namespace lwo {
namespace {

// ---------------------------------------------------------------------------
// LwoState: identity state sanity
// ---------------------------------------------------------------------------

TEST(LwoState, DefaultIsIdentity) {
  LwoState s;
  EXPECT_TRUE(s.rotation.isApprox(Eigen::Matrix3f::Identity()));
  EXPECT_TRUE(s.position.isZero());
  EXPECT_TRUE(s.velocity.isZero());
  EXPECT_FLOAT_EQ(s.wheel_scale, 1.0f);
  EXPECT_TRUE(s.wheel_gyro_bias.isZero());
}

// ---------------------------------------------------------------------------
// LwoState: operator+ / operator- round-trip
// ---------------------------------------------------------------------------

TEST(LwoState, PlusMinusRoundTrip) {
  LwoState s;
  s.wheel_scale = 1.0f;

  LwoStateVector delta = LwoStateVector::Zero();
  delta(kLwoRotIdx)      = 0.1f;   // small rotation
  delta(kLwoRotIdx + 1)  = 0.05f;
  delta(kLwoPosIdx)      = 1.0f;
  delta(kLwoPosIdx + 1)  = 2.0f;
  delta(kLwoVelIdx)      = 0.5f;
  delta(kLwoScaleIdx)    = 0.01f;
  delta(kLwoBiasIdx)     = 0.001f;
  delta(kLwoBiasIdx + 1) = 0.002f;

  const LwoState s2 = s + delta;
  const LwoStateVector recovered = s2 - s;

  for (int i = 0; i < kLwoStateDim; ++i) {
    EXPECT_NEAR(recovered(i), delta(i), 1e-5f)
        << "Mismatch at index " << i;
  }
}

// ---------------------------------------------------------------------------
// LwoState: operator+ preserves covariance (copy-through)
// ---------------------------------------------------------------------------

TEST(LwoState, PlusPreservesCovariance) {
  LwoState s;
  s.covariance *= 2.0f;
  const LwoStateVector delta = LwoStateVector::Zero();
  const LwoState s2 = s + delta;
  EXPECT_TRUE(s2.covariance.isApprox(s.covariance));
}

// ---------------------------------------------------------------------------
// LwoState: reset returns to canonical identity
// ---------------------------------------------------------------------------

TEST(LwoState, ResetRestoresDefault) {
  LwoState s;
  s.position  = Eigen::Vector3f(1, 2, 3);
  s.wheel_scale = 1.5f;
  s.reset();
  EXPECT_TRUE(s.rotation.isApprox(Eigen::Matrix3f::Identity()));
  EXPECT_TRUE(s.position.isZero());
  EXPECT_FLOAT_EQ(s.wheel_scale, 1.0f);
}

// ---------------------------------------------------------------------------
// LwoState: pose() / set_pose() round-trip
// ---------------------------------------------------------------------------

TEST(LwoState, PoseRoundTrip) {
  LwoState s;
  s.position = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
  // 30-degree rotation around Z
  const float angle = 30.0f * static_cast<float>(M_PI) / 180.0f;
  s.rotation = core::So3::Exp(Eigen::Vector3f(0, 0, angle)).matrix();

  const core::Se3 pose = s.pose();
  LwoState s2;
  s2.set_pose(pose);

  EXPECT_TRUE(s2.rotation.isApprox(s.rotation, 1e-5f));
  EXPECT_TRUE(s2.position.isApprox(s.position, 1e-5f));
}

}  // namespace
}  // namespace lwo
}  // namespace tof_slam
