// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "tof_slam/common/types/state.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <sstream>

namespace tof_slam {
namespace core {
namespace {

constexpr float kTol = 1e-4f;

// ===========================================================================
// LioState — manifold round trips
// ===========================================================================

TEST(LioStateTest, PlusZeroDeltaIsIdentity) {
  LioState state;
  state.position = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
  state.velocity = Eigen::Vector3f(0.1f, 0.2f, 0.3f);
  state.rotation = So3::Exp(Eigen::Vector3f(0.1f, -0.2f, 0.3f)).matrix();

  LioState result = state + StateVector::Zero();
  EXPECT_LT((result.rotation - state.rotation).norm(), kTol);
  EXPECT_LT((result.position - state.position).norm(), kTol);
  EXPECT_LT((result.velocity - state.velocity).norm(), kTol);
}

TEST(LioStateTest, PlusMinusRoundTrip) {
  LioState state;
  state.position = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
  state.velocity = Eigen::Vector3f(0.5f, -0.3f, 0.1f);
  state.rotation = So3::Exp(Eigen::Vector3f(0.2f, -0.1f, 0.4f)).matrix();
  state.gyro_bias = Eigen::Vector3f(0.01f, -0.02f, 0.03f);
  state.acc_bias = Eigen::Vector3f(0.1f, 0.2f, -0.1f);

  StateVector delta = StateVector::Zero();
  delta.segment<3>(kRotIdx) = Eigen::Vector3f(0.05f, -0.03f, 0.02f);
  delta.segment<3>(kPosIdx) = Eigen::Vector3f(0.5f, -0.2f, 0.1f);
  delta.segment<3>(kVelIdx) = Eigen::Vector3f(0.1f, 0.0f, -0.05f);
  delta.segment<3>(kGyrBiasIdx) = Eigen::Vector3f(0.001f, -0.001f, 0.0f);
  delta.segment<3>(kAccBiasIdx) = Eigen::Vector3f(0.01f, 0.0f, -0.01f);
  delta.segment<3>(kGravIdx) = Eigen::Vector3f(0.0f, 0.0f, 0.01f);

  LioState updated = state + delta;
  StateVector recovered = updated - state;
  EXPECT_LT((recovered - delta).norm(), kTol)
      << "Recovered:\n" << recovered.transpose()
      << "\nExpected:\n" << delta.transpose();
}

TEST(LioStateTest, PlusMinusRoundTripRotationOnly) {
  LioState state;
  state.rotation = So3::Exp(Eigen::Vector3f(0.3f, -0.5f, 0.2f)).matrix();

  StateVector delta = StateVector::Zero();
  delta.segment<3>(kRotIdx) = Eigen::Vector3f(0.1f, 0.1f, -0.1f);

  LioState updated = state + delta;
  StateVector recovered = updated - state;
  EXPECT_LT((recovered - delta).norm(), kTol);
}

TEST(LioStateTest, PlusMinusRoundTripTranslationOnly) {
  LioState state;
  state.position = Eigen::Vector3f(10.0f, 20.0f, 30.0f);

  StateVector delta = StateVector::Zero();
  delta.segment<3>(kPosIdx) = Eigen::Vector3f(1.0f, -2.0f, 3.0f);

  LioState updated = state + delta;
  StateVector recovered = updated - state;
  EXPECT_LT((recovered - delta).norm(), kTol);
}

TEST(LioStateTest, MinusSelfIsZero) {
  LioState state;
  state.position = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
  state.rotation = So3::Exp(Eigen::Vector3f(0.1f, 0.2f, 0.3f)).matrix();
  state.velocity = Eigen::Vector3f(-0.5f, 0.3f, 0.0f);

  StateVector diff = state - state;
  EXPECT_LT(diff.norm(), kTol);
}

// ===========================================================================
// LioState — pose accessor
// ===========================================================================

TEST(LioStateTest, PoseConsistency) {
  LioState state;
  state.rotation = So3::Exp(Eigen::Vector3f(0.1f, -0.2f, 0.3f)).matrix();
  state.position = Eigen::Vector3f(5.0f, 6.0f, 7.0f);

  Se3 T = state.pose();
  EXPECT_LT((T.rotation_matrix() - state.rotation).norm(), kTol);
  EXPECT_LT((T.translation() - state.position).norm(), kTol);
}

TEST(LioStateTest, PoseInverseTimesOriginalIsIdentity) {
  LioState state;
  state.rotation = So3::Exp(Eigen::Vector3f(0.5f, -0.3f, 0.1f)).matrix();
  state.position = Eigen::Vector3f(1.0f, 2.0f, 3.0f);

  Se3 T = state.pose();
  Se3 I = T.inverse() * T;
  EXPECT_LT((I.matrix() - Eigen::Matrix4f::Identity()).norm(), kTol);
}

TEST(LioStateTest, SetPose) {
  LioState state;
  Se3 T(So3::Exp(Eigen::Vector3f(0.2f, 0.3f, -0.1f)),
        Eigen::Vector3f(10.0f, 20.0f, 30.0f));

  state.set_pose(T);
  EXPECT_LT((state.rotation - T.rotation_matrix()).norm(), kTol);
  EXPECT_LT((state.position - T.translation()).norm(), kTol);
}

// ===========================================================================
// LioState — reset
// ===========================================================================

TEST(LioStateTest, ResetToDefault) {
  LioState state;
  state.position = Eigen::Vector3f(100, 200, 300);
  state.velocity = Eigen::Vector3f(1, 2, 3);

  state.reset();
  EXPECT_LT(state.position.norm(), kTol);
  EXPECT_LT(state.velocity.norm(), kTol);
  EXPECT_NEAR(state.gravity.z(), -kGravityNorm, kTol);
  EXPECT_LT((state.rotation - Eigen::Matrix3f::Identity()).norm(), kTol);
}

// ===========================================================================
// LioState — operator+=
// ===========================================================================

TEST(LioStateTest, PlusEquals) {
  LioState state;
  state.position = Eigen::Vector3f(1, 2, 3);

  StateVector delta = StateVector::Zero();
  delta.segment<3>(kPosIdx) = Eigen::Vector3f(10, 20, 30);
  state += delta;

  EXPECT_NEAR(state.position.x(), 11.0f, kTol);
  EXPECT_NEAR(state.position.y(), 22.0f, kTol);
  EXPECT_NEAR(state.position.z(), 33.0f, kTol);
}

// ===========================================================================
// LioState — ostream operator
// ===========================================================================

TEST(LioStateTest, OstreamDoesNotCrash) {
  LioState state;
  std::ostringstream oss;
  oss << state;
  EXPECT_FALSE(oss.str().empty());
}

}  // namespace
}  // namespace core
}  // namespace tof_slam
