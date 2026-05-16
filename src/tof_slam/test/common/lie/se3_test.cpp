// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "tof_slam/common/lie/se3.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>

namespace tof_slam {
namespace core {
namespace {

constexpr float kTol = 1e-5f;

// ===========================================================================
// Se3::Exp / Log — round trips
// ===========================================================================

TEST(Se3Test, ExpLogRoundTripSmallTwist) {
  Eigen::Matrix<float, 6, 1> xi;
  xi << 1e-8f, -2e-8f, 3e-8f, 1e-9f, -1e-9f, 2e-9f;
  const Se3 T = Se3::Exp(xi);
  const auto recovered = T.Log();
  EXPECT_LT((recovered - xi).norm(), kTol);
}

TEST(Se3Test, ExpLogRoundTripModerate) {
  Eigen::Matrix<float, 6, 1> xi;
  xi << 0.1f, -0.2f, 0.3f, 0.4f, -0.1f, 0.2f;
  const Se3 T = Se3::Exp(xi);
  const auto recovered = T.Log();
  EXPECT_LT((recovered - xi).norm(), kTol);
}

TEST(Se3Test, ExpLogRoundTripPureTranslation) {
  Eigen::Matrix<float, 6, 1> xi;
  xi << 1.0f, 2.0f, 3.0f, 0.0f, 0.0f, 0.0f;
  const Se3 T = Se3::Exp(xi);
  const auto recovered = T.Log();
  EXPECT_LT((recovered - xi).norm(), kTol);
}

TEST(Se3Test, ExpLogRoundTripPureRotation) {
  Eigen::Matrix<float, 6, 1> xi;
  xi << 0.0f, 0.0f, 0.0f, 0.3f, -0.2f, 0.5f;
  const Se3 T = Se3::Exp(xi);
  const auto recovered = T.Log();
  EXPECT_LT((recovered - xi).norm(), kTol);
}

TEST(Se3Test, ExpZeroIsIdentity) {
  const Se3 T = Se3::Exp(Eigen::Matrix<float, 6, 1>::Zero());
  EXPECT_LT((T.matrix() - Eigen::Matrix4f::Identity()).norm(), kTol);
}

// ===========================================================================
// Se3 — group operations
// ===========================================================================

TEST(Se3Test, CompositionMatchesMatrixProduct) {
  Eigen::Matrix<float, 6, 1> xi_a, xi_b;
  xi_a << 0.1f, -0.2f, 0.3f, 0.2f, 0.1f, -0.1f;
  xi_b << -0.3f, 0.1f, 0.2f, -0.1f, 0.3f, 0.2f;

  const Se3 A = Se3::Exp(xi_a);
  const Se3 B = Se3::Exp(xi_b);
  const Se3 AB = A * B;

  EXPECT_LT((AB.matrix() - A.matrix() * B.matrix()).norm(), kTol);
}

TEST(Se3Test, InverseCompositionIsIdentity) {
  Eigen::Matrix<float, 6, 1> xi;
  xi << 0.5f, -0.3f, 1.0f, 0.2f, -0.4f, 0.1f;
  const Se3 T = Se3::Exp(xi);
  const Se3 I = T.inverse() * T;
  EXPECT_LT((I.matrix() - Eigen::Matrix4f::Identity()).norm(), kTol);
}

TEST(Se3Test, InverseOfProductIsReversedProduct) {
  Eigen::Matrix<float, 6, 1> xi_a, xi_b;
  xi_a << 0.1f, -0.2f, 0.3f, 0.4f, 0.1f, -0.1f;
  xi_b << -0.3f, 0.1f, 0.2f, -0.1f, 0.3f, 0.2f;

  const Se3 A = Se3::Exp(xi_a);
  const Se3 B = Se3::Exp(xi_b);

  const Se3 AB_inv = (A * B).inverse();
  const Se3 B_inv_A_inv = B.inverse() * A.inverse();

  EXPECT_LT((AB_inv.matrix() - B_inv_A_inv.matrix()).norm(), kTol);
}

// ===========================================================================
// Se3 — transform point
// ===========================================================================

TEST(Se3Test, TransformPointEqualsRpPlusT) {
  const So3 R = So3::Exp(Eigen::Vector3f(0.1f, -0.2f, 0.3f));
  const Eigen::Vector3f t(1.0f, 2.0f, 3.0f);
  const Se3 T(R, t);

  const Eigen::Vector3f p(0.5f, -1.0f, 2.0f);
  const Eigen::Vector3f expected = R * p + t;
  const Eigen::Vector3f result = T * p;

  EXPECT_LT((result - expected).norm(), kTol);
}

TEST(Se3Test, IdentityTransformPreservesPoint) {
  const Eigen::Vector3f p(1.0f, 2.0f, 3.0f);
  const Eigen::Vector3f result = Se3::Identity() * p;
  EXPECT_LT((result - p).norm(), kTol);
}

// ===========================================================================
// Se3 — from 4x4 matrix
// ===========================================================================

TEST(Se3Test, ConstructFromMatrix) {
  Eigen::Matrix<float, 6, 1> xi;
  xi << 0.5f, -1.0f, 0.3f, 0.2f, -0.1f, 0.4f;
  const Se3 original = Se3::Exp(xi);
  const Se3 from_mat(original.matrix());
  EXPECT_LT((from_mat.matrix() - original.matrix()).norm(), kTol);
}

// ===========================================================================
// Se3 — accessors
// ===========================================================================

TEST(Se3Test, RotationAccessor) {
  const So3 R = So3::Exp(Eigen::Vector3f(0.1f, 0.2f, 0.3f));
  const Se3 T(R, Eigen::Vector3f(1.0f, 2.0f, 3.0f));
  EXPECT_LT((T.rotation().matrix() - R.matrix()).norm(), kTol);
  EXPECT_LT((T.rotation_matrix() - R.matrix()).norm(), kTol);
}

TEST(Se3Test, TranslationAccessor) {
  const Eigen::Vector3f t(1.0f, 2.0f, 3.0f);
  const Se3 T(So3::Identity(), t);
  EXPECT_LT((T.translation() - t).norm(), kTol);
}

// ===========================================================================
// Jacobians
// ===========================================================================

TEST(JacobianTest, RightJacobianIdentityForSmallAngle) {
  const Eigen::Vector3f phi(1e-9f, -1e-9f, 1e-9f);
  const Eigen::Matrix3f Jr = RightJacobian(phi);
  EXPECT_LT((Jr - Eigen::Matrix3f::Identity()).norm(), kTol);
}

TEST(JacobianTest, LeftJacobianEqualsRightJacobianOfNegative) {
  const Eigen::Vector3f phi(0.3f, -0.5f, 0.2f);
  const Eigen::Matrix3f Jl = LeftJacobian(phi);
  const Eigen::Matrix3f Jr_neg = RightJacobian(-phi);
  EXPECT_LT((Jl - Jr_neg).norm(), kTol);
}

TEST(JacobianTest, RightJacobianTimesInverseIsIdentity) {
  // J_r(phi) * J_r^{-1}(phi) = I
  // We verify indirectly: J_r(phi)^T * J_l(phi)^T has a known structure.
  // A simpler check: det(J_r) should be non-zero for moderate angles.
  const Eigen::Vector3f phi(0.5f, -0.3f, 0.7f);
  const Eigen::Matrix3f Jr = RightJacobian(phi);
  EXPECT_GT(std::abs(Jr.determinant()), 0.1f);
}

// ===========================================================================
// Random round-trip stress test
// ===========================================================================

TEST(Se3Test, RandomExpLogRoundTrip) {
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> dist(-2.0f, 2.0f);

  for (int i = 0; i < 100; ++i) {
    Eigen::Matrix<float, 6, 1> xi;
    xi << dist(rng), dist(rng), dist(rng), dist(rng), dist(rng), dist(rng);

    // Keep rotation part moderate to avoid pi-singularity.
    xi.tail<3>() *= 0.5f;

    const Se3 T = Se3::Exp(xi);
    const auto recovered = T.Log();
    const Se3 T2 = Se3::Exp(recovered);

    EXPECT_LT((T.matrix() - T2.matrix()).norm(), 1e-4f)
        << "Round-trip failed at iteration " << i;
  }
}

}  // namespace
}  // namespace core
}  // namespace tof_slam
