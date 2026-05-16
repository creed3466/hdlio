// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "tof_slam/common/lie/so3.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>

namespace tof_slam {
namespace core {
namespace {

constexpr float kTol = 1e-5f;

// ===========================================================================
// Hat / Vee
// ===========================================================================

TEST(HatTest, ProducesSkewSymmetric) {
  const Eigen::Vector3f v(1.0f, 2.0f, 3.0f);
  const Eigen::Matrix3f S = Hat(v);
  EXPECT_NEAR((S + S.transpose()).norm(), 0.0f, kTol);
}

TEST(HatTest, TraceIsZero) {
  const Eigen::Vector3f v(0.5f, -1.2f, 3.7f);
  EXPECT_NEAR(Hat(v).trace(), 0.0f, kTol);
}

TEST(HatVeeTest, RoundTrip) {
  const Eigen::Vector3f v(1.0f, -2.0f, 0.3f);
  EXPECT_LT((Vee(Hat(v)) - v).norm(), kTol);
}

TEST(HatVeeTest, ZeroVector) {
  const Eigen::Vector3f v = Eigen::Vector3f::Zero();
  EXPECT_LT(Vee(Hat(v)).norm(), kTol);
}

// ===========================================================================
// So3::Exp / Log  — round trips
// ===========================================================================

TEST(So3Test, ExpProducesValidRotation) {
  const Eigen::Vector3f omega(0.1f, -0.2f, 0.3f);
  const So3 R = So3::Exp(omega);
  const Eigen::Matrix3f RtR = R.matrix().transpose() * R.matrix();
  EXPECT_LT((RtR - Eigen::Matrix3f::Identity()).norm(), kTol);
  EXPECT_NEAR(R.matrix().determinant(), 1.0f, kTol);
}

TEST(So3Test, ExpLogRoundTripSmallAngle) {
  const Eigen::Vector3f omega(1e-8f, -2e-8f, 5e-9f);
  const So3 R = So3::Exp(omega);
  const Eigen::Vector3f recovered = R.Log();
  EXPECT_LT((recovered - omega).norm(), kTol);
}

TEST(So3Test, ExpLogRoundTripModerateAngle) {
  const Eigen::Vector3f omega(0.5f, -0.3f, 0.7f);
  const So3 R = So3::Exp(omega);
  const Eigen::Vector3f recovered = R.Log();
  EXPECT_LT((recovered - omega).norm(), kTol);
}

TEST(So3Test, ExpLogRoundTripLargeAngle) {
  // theta ≈ 2.8 rad (close to pi but not exactly)
  const Eigen::Vector3f omega(1.5f, -1.8f, 1.2f);
  const So3 R = So3::Exp(omega);
  const Eigen::Vector3f recovered = R.Log();
  EXPECT_LT((recovered - omega).norm(), 1e-4f);
}

TEST(So3Test, ExpLogRoundTripNearPi) {
  // theta = pi - epsilon
  const float theta = kPi - 0.001f;
  const Eigen::Vector3f axis =
      Eigen::Vector3f(1.0f, 1.0f, 1.0f).normalized();
  const Eigen::Vector3f omega = axis * theta;
  const So3 R = So3::Exp(omega);
  const Eigen::Vector3f recovered = R.Log();
  // Near pi, the axis direction may flip — check that the rotation is the same.
  const So3 R2 = So3::Exp(recovered);
  EXPECT_LT((R.matrix() - R2.matrix()).norm(), 1e-3f);
}

TEST(So3Test, ExpZeroIsIdentity) {
  const So3 R = So3::Exp(Eigen::Vector3f::Zero());
  EXPECT_LT((R.matrix() - Eigen::Matrix3f::Identity()).norm(), kTol);
}

TEST(So3Test, LogIdentityIsZero) {
  const So3 R = So3::Identity();
  EXPECT_LT(R.Log().norm(), kTol);
}

// ===========================================================================
// So3 — group operations
// ===========================================================================

TEST(So3Test, CompositionMatchesMatrixProduct) {
  const So3 A = So3::Exp(Eigen::Vector3f(0.1f, 0.2f, -0.3f));
  const So3 B = So3::Exp(Eigen::Vector3f(-0.4f, 0.1f, 0.5f));
  const So3 AB = A * B;
  EXPECT_LT((AB.matrix() - A.matrix() * B.matrix()).norm(), kTol);
}

TEST(So3Test, InverseTimesOriginalIsIdentity) {
  const So3 R = So3::Exp(Eigen::Vector3f(0.3f, -0.5f, 0.2f));
  const So3 I = R.inverse() * R;
  EXPECT_LT((I.matrix() - Eigen::Matrix3f::Identity()).norm(), kTol);
}

TEST(So3Test, RotateVector) {
  // 90-degree rotation around z-axis: [1,0,0] -> [0,1,0]
  const Eigen::Vector3f omega(0.0f, 0.0f, kPi / 2.0f);
  const So3 R = So3::Exp(omega);
  const Eigen::Vector3f v(1.0f, 0.0f, 0.0f);
  const Eigen::Vector3f rotated = R * v;
  EXPECT_NEAR(rotated.x(), 0.0f, kTol);
  EXPECT_NEAR(rotated.y(), 1.0f, kTol);
  EXPECT_NEAR(rotated.z(), 0.0f, kTol);
}

// ===========================================================================
// So3 — normalize
// ===========================================================================

TEST(So3Test, NormalizeCorrectionsDrift) {
  So3 R = So3::Exp(Eigen::Vector3f(0.1f, 0.2f, 0.3f));
  // Introduce drift.
  R.matrix()(0, 0) += 0.01f;
  R.matrix()(1, 1) += 0.01f;
  R.normalize();
  const Eigen::Matrix3f RtR = R.matrix().transpose() * R.matrix();
  EXPECT_LT((RtR - Eigen::Matrix3f::Identity()).norm(), kTol);
  EXPECT_NEAR(R.matrix().determinant(), 1.0f, kTol);
}

// ===========================================================================
// Random round-trip stress test
// ===========================================================================

TEST(So3Test, RandomExpLogRoundTrip) {
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> dist(-2.5f, 2.5f);

  for (int i = 0; i < 100; ++i) {
    const Eigen::Vector3f omega(dist(rng), dist(rng), dist(rng));
    const So3 R = So3::Exp(omega);

    // Verify R is valid.
    const Eigen::Matrix3f RtR = R.matrix().transpose() * R.matrix();
    EXPECT_LT((RtR - Eigen::Matrix3f::Identity()).norm(), kTol)
        << "Failed at iteration " << i;
    EXPECT_NEAR(R.matrix().determinant(), 1.0f, kTol);

    // Verify round trip (rotation-level, not necessarily omega-level for
    // |omega| > pi).  Tolerance is relaxed for float32 near-pi cases.
    const Eigen::Vector3f recovered = R.Log();
    const So3 R2 = So3::Exp(recovered);
    // float32 near-pi: Rodrigues + Log amplification can produce ~1e-3 error.
    EXPECT_LT((R.matrix() - R2.matrix()).norm(), 5e-3f)
        << "Round-trip failed at iteration " << i
        << " with |omega|=" << omega.norm();
  }
}

}  // namespace
}  // namespace core
}  // namespace tof_slam
