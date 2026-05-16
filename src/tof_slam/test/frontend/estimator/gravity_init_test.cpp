// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "tof_slam/frontend/estimator/gravity_init.hpp"

#include <gtest/gtest.h>

namespace tof_slam {
namespace core {
namespace {

constexpr float kTol = 0.1f;

/// Create N synthetic stationary IMU samples with a given acceleration vector.
std::vector<ImuMeasurement> make_stationary(int n,
                                            const Eigen::Vector3f& acc) {
  std::vector<ImuMeasurement> buf(n);
  for (int i = 0; i < n; ++i) {
    buf[i].timestamp = 0.01 * i;
    buf[i].accel = acc;
    buf[i].gyro = Eigen::Vector3f::Zero();
  }
  return buf;
}

// ===========================================================================
// Success cases
// ===========================================================================

TEST(GravityInitTest, StandardMssSamples) {
  // Stationary with acc = [0, 0, 9.81] (sensor reports in m/s^2)
  auto buf = make_stationary(100, Eigen::Vector3f(0, 0, 9.81f));
  auto result = initialize_gravity(buf);

  EXPECT_TRUE(result.success) << result.message;
  EXPECT_NEAR(result.imu_acc_scale, 1.0f, 0.1f);
  // Gravity should be aligned to [0, 0, -9.81].
  EXPECT_NEAR(result.initial_state.gravity.z(), -kGravityNorm, kTol);
}

TEST(GravityInitTest, GUnitAutoDetection) {
  // MID-360 style: acc ≈ [0, 0, 1.0] (g-units)
  auto buf = make_stationary(100, Eigen::Vector3f(0, 0, 1.0f));
  auto result = initialize_gravity(buf);

  EXPECT_TRUE(result.success) << result.message;
  EXPECT_NEAR(result.imu_acc_scale, kGravityNorm, 0.5f);
  EXPECT_NEAR(result.initial_state.gravity.z(), -kGravityNorm, kTol);
}

TEST(GravityInitTest, TiltedSensor) {
  // Sensor tilted 45 degrees: acc = [0, g*sin(45), g*cos(45)]
  const float s = kGravityNorm * 0.70710678f;  // sin(45) = cos(45)
  auto buf = make_stationary(100, Eigen::Vector3f(0, s, s));
  auto result = initialize_gravity(buf);

  EXPECT_TRUE(result.success) << result.message;
  // Gravity should still end up near [0, 0, -9.81] after alignment.
  EXPECT_NEAR(result.initial_state.gravity.z(), -kGravityNorm, kTol);
}

// ===========================================================================
// Failure cases
// ===========================================================================

TEST(GravityInitTest, TooFewSamplesReturnsFalse) {
  auto buf = make_stationary(5, Eigen::Vector3f(0, 0, 9.81f));
  auto result = initialize_gravity(buf);
  EXPECT_FALSE(result.success);
}

TEST(GravityInitTest, EmptyBufferReturnsFalse) {
  auto result = initialize_gravity({});
  EXPECT_FALSE(result.success);
}

TEST(GravityInitTest, WildAccNormReturnsFalse) {
  // Acc norm way off from gravity
  auto buf = make_stationary(100, Eigen::Vector3f(50, 0, 0));
  auto result = initialize_gravity(buf);
  EXPECT_FALSE(result.success);
}

// ===========================================================================
// Bias estimation
// ===========================================================================

TEST(GravityInitTest, GyroBiasEstimated) {
  std::vector<ImuMeasurement> buf(100);
  for (int i = 0; i < 100; ++i) {
    buf[i].timestamp = 0.01 * i;
    buf[i].accel = Eigen::Vector3f(0, 0, 9.81f);
    buf[i].gyro = Eigen::Vector3f(0.01f, -0.02f, 0.005f);  // Constant bias
  }
  auto result = initialize_gravity(buf);
  EXPECT_TRUE(result.success);
  EXPECT_NEAR(result.initial_state.gyro_bias.x(), 0.01f, 1e-4f);
  EXPECT_NEAR(result.initial_state.gyro_bias.y(), -0.02f, 1e-4f);
}

// ===========================================================================
// Stationarity / covariance
// ===========================================================================

TEST(GravityInitTest, CovarianceIsSymmetricPD) {
  auto buf = make_stationary(100, Eigen::Vector3f(0, 0, 9.81f));
  auto result = initialize_gravity(buf);
  EXPECT_TRUE(result.success);

  const auto& P = result.initial_state.covariance;
  EXPECT_LT((P - P.transpose()).norm(), 1e-6f);

  Eigen::LLT<StateCovariance> llt(P);
  EXPECT_EQ(llt.info(), Eigen::Success);
}

}  // namespace
}  // namespace core
}  // namespace tof_slam
