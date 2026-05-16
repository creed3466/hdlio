// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "tof_slam/frontend/estimator/imu_propagator.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace tof_slam {
namespace core {
namespace {

constexpr float kTol = 1e-3f;

/// Build a default diagonal process noise matrix.
StateCovariance default_Q() {
  StateCovariance Q = StateCovariance::Zero();
  Q.block<3, 3>(kRotIdx, kRotIdx)     = Eigen::Matrix3d::Identity() * 1e-4;
  Q.block<3, 3>(kPosIdx, kPosIdx)     = Eigen::Matrix3d::Identity() * 1e-6;
  Q.block<3, 3>(kVelIdx, kVelIdx)     = Eigen::Matrix3d::Identity() * 1e-4;
  Q.block<3, 3>(kGyrBiasIdx, kGyrBiasIdx) = Eigen::Matrix3d::Identity() * 1e-6;
  Q.block<3, 3>(kAccBiasIdx, kAccBiasIdx) = Eigen::Matrix3d::Identity() * 1e-6;
  Q.block<3, 3>(kGravIdx, kGravIdx)   = Eigen::Matrix3d::Identity() * 1e-8;
  return Q;
}

// ===========================================================================
// Safety guards
// ===========================================================================

TEST(ImuPropagatorTest, NegativeDtReturnsUnchanged) {
  LioState prior;
  prior.position = Eigen::Vector3f(1, 2, 3);
  ImuMeasurement imu;
  LioState result = propagate_imu(prior, imu, -0.01f, default_Q());
  EXPECT_EQ(result.position, prior.position);
}

TEST(ImuPropagatorTest, LargeDtReturnsUnchanged) {
  LioState prior;
  prior.position = Eigen::Vector3f(1, 2, 3);
  ImuMeasurement imu;
  LioState result = propagate_imu(prior, imu, 1.5f, default_Q());
  EXPECT_EQ(result.position, prior.position);
}

// ===========================================================================
// Zero-motion test
// ===========================================================================

TEST(ImuPropagatorTest, StationaryRobotStaysStill) {
  LioState state;
  state.reset();

  // Stationary robot: acc reading = -gravity (in body frame).
  // gravity = [0, 0, -9.81], rotation = I  =>  acc_body = [0, 0, 9.81]
  ImuMeasurement imu;
  imu.gyro = Eigen::Vector3f::Zero();
  imu.accel = Eigen::Vector3f(0, 0, kGravityNorm);  // acc = -R^T * g

  const StateCovariance Q = default_Q();
  const float dt = 0.01f;

  for (int i = 0; i < 100; ++i) {
    state = propagate_imu(state, imu, dt, Q);
  }

  // After 1 second of stationary, position/velocity should remain near zero.
  EXPECT_LT(state.position.norm(), 0.1f);
  EXPECT_LT(state.velocity.norm(), 0.1f);
}

// ===========================================================================
// Constant acceleration test
// ===========================================================================

TEST(ImuPropagatorTest, ConstantAccelerationOnZ) {
  LioState state;
  state.reset();

  // Extra +1 m/s^2 on z above gravity compensation.
  ImuMeasurement imu;
  imu.gyro = Eigen::Vector3f::Zero();
  // acc_body = -R^T * g + extra = [0, 0, 9.81] + [0, 0, 1.0]
  imu.accel = Eigen::Vector3f(0, 0, kGravityNorm + 1.0f);

  const StateCovariance Q = default_Q();
  const float dt = 0.01f;

  for (int i = 0; i < 100; ++i) {
    state = propagate_imu(state, imu, dt, Q);
  }

  // After 1 second at +1 m/s^2: v_z ≈ 1 m/s, p_z ≈ 0.5 m
  EXPECT_NEAR(state.velocity.z(), 1.0f, 0.05f);
  EXPECT_NEAR(state.position.z(), 0.5f, 0.05f);
}

// ===========================================================================
// Constant rotation test
// ===========================================================================

TEST(ImuPropagatorTest, ConstantRotationAroundZ) {
  LioState state;
  state.reset();

  ImuMeasurement imu;
  imu.gyro = Eigen::Vector3f(0, 0, 0.1f);  // 0.1 rad/s around z
  imu.accel = Eigen::Vector3f(0, 0, kGravityNorm);

  const StateCovariance Q = default_Q();
  const float dt = 0.01f;

  for (int i = 0; i < 100; ++i) {
    state = propagate_imu(state, imu, dt, Q);
  }

  // After 1 second at 0.1 rad/s: total rotation ≈ 0.1 rad
  So3 expected_R = So3::Exp(Eigen::Vector3f(0, 0, 0.1f));
  EXPECT_LT((state.rotation - expected_R.matrix()).norm(), 0.02f);
}

// ===========================================================================
// Covariance positive-definite
// ===========================================================================

TEST(ImuPropagatorTest, CovarianceRemainsPD) {
  LioState state;
  state.reset();

  ImuMeasurement imu;
  imu.gyro = Eigen::Vector3f(0.01f, -0.02f, 0.03f);
  imu.accel = Eigen::Vector3f(0.1f, -0.2f, kGravityNorm);

  const StateCovariance Q = default_Q();
  const float dt = 0.01f;

  for (int i = 0; i < 100; ++i) {
    state = propagate_imu(state, imu, dt, Q);
  }

  // Check symmetry.
  EXPECT_LT((state.covariance - state.covariance.transpose()).norm(), 1e-6f);

  // Check positive-definite via Cholesky.
  Eigen::LLT<StateCovariance> llt(state.covariance);
  EXPECT_EQ(llt.info(), Eigen::Success);
}

// ===========================================================================
// Transition matrix dimensions
// ===========================================================================

TEST(ImuPropagatorTest, TransitionMatrixIsCorrectSize) {
  LioState state;
  ImuMeasurement imu;
  auto F = build_transition_matrix(state, imu, 0.01f);
  EXPECT_EQ(F.rows(), kStateDim);
  EXPECT_EQ(F.cols(), kStateDim);
}

}  // namespace
}  // namespace core
}  // namespace tof_slam
