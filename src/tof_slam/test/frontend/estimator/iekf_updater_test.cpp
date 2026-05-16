// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// iekf_updater_test.cpp — Unit tests for the IEKF inner-loop updater.

#include "tof_slam/frontend/estimator/iekf_updater.hpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <vector>

#include "tof_slam/frontend/estimator/correspondence.hpp"
#include "tof_slam/common/lie/se3.hpp"
#include "tof_slam/frontend/robust/pko.hpp"
#include "tof_slam/common/types/state.hpp"

namespace tof_slam {
namespace core {
namespace {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Build a default prior: identity rotation, zero position, diagonal P.
LioState make_prior(float cov_diag = 1.0f) {
  LioState s;
  s.rotation = Eigen::Matrix3f::Identity();
  s.position = Eigen::Vector3f::Zero();
  s.velocity = Eigen::Vector3f::Zero();
  s.gyro_bias = Eigen::Vector3f::Zero();
  s.acc_bias = Eigen::Vector3f::Zero();
  s.gravity = Eigen::Vector3f(0.0f, 0.0f, -9.81f);
  s.covariance = StateCovariance::Identity() * cov_diag;
  return s;
}

/// Build correspondences for a horizontal plane at z = 0.
/// Points are at world-frame position (x, y, h).
/// normal = (0,0,1), plane_d = 0 → residual = -h.
std::vector<Correspondence> horizontal_plane_corrs(
    int n, float height, float noise_offset = 0.0f) {
  std::vector<Correspondence> corrs(n);
  for (int i = 0; i < n; ++i) {
    Correspondence c;
    c.p_lidar = Eigen::Vector3f(static_cast<float>(i), 0.0f, height + noise_offset);
    c.normal = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    c.plane_d = 0.0f;
    corrs[i] = c;
  }
  return corrs;
}

/// Default identity extrinsic (LiDAR coincides with body frame).
Se3 identity_extrinsic() { return Se3::Identity(); }

// ---------------------------------------------------------------------------
// TEST: EmptyCorrespondencesReturnsUnchanged
// ---------------------------------------------------------------------------
TEST(IekfUpdaterTest, EmptyCorrespondencesReturnsUnchanged) {
  const LioState prior = make_prior();
  const IekfConfig cfg;
  const std::vector<Correspondence> no_corrs;

  const IekfResult result =
      iekf_update(prior, no_corrs, identity_extrinsic(), cfg);

  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.total_iterations, 0);
  EXPECT_EQ(result.num_correspondences, 0);

  // State must be identical to prior.
  EXPECT_TRUE(result.state.rotation.isApprox(prior.rotation, 1e-6f));
  EXPECT_TRUE(result.state.position.isApprox(prior.position, 1e-6f));
}

// ---------------------------------------------------------------------------
// TEST: HuberWeightsBelowDelta — all residuals within delta → weights = 1.
// ---------------------------------------------------------------------------
TEST(IekfUpdaterTest, HuberWeightsBelowDelta) {
  const float delta = 2.0f;
  Eigen::VectorXf residuals(4);
  residuals << 0.5f, -1.0f, 1.9f, 0.0f;

  const Eigen::VectorXf w = compute_huber_weights(residuals, delta);

  ASSERT_EQ(w.size(), 4);
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(w(i), 1.0f, 1e-6f) << "weight at index " << i;
  }
}

// ---------------------------------------------------------------------------
// TEST: HuberWeightsAboveDelta — |r| = 2*delta → weight = 0.5.
// ---------------------------------------------------------------------------
TEST(IekfUpdaterTest, HuberWeightsAboveDelta) {
  const float delta = 1.0f;
  Eigen::VectorXf residuals(1);
  residuals << 2.0f * delta;

  const Eigen::VectorXf w = compute_huber_weights(residuals, delta);

  ASSERT_EQ(w.size(), 1);
  EXPECT_NEAR(w(0), 0.5f, 1e-6f);
}

// ---------------------------------------------------------------------------
// TEST: HuberWeightsMixed — some above, some below delta.
// ---------------------------------------------------------------------------
TEST(IekfUpdaterTest, HuberWeightsMixed) {
  const float delta = 1.0f;
  Eigen::VectorXf residuals(3);
  residuals << 0.5f, 2.0f, -4.0f;

  const Eigen::VectorXf w = compute_huber_weights(residuals, delta);

  ASSERT_EQ(w.size(), 3);
  EXPECT_NEAR(w(0), 1.0f, 1e-6f);        // |0.5| < 1
  EXPECT_NEAR(w(1), 0.5f, 1e-6f);        // delta/|2| = 0.5
  EXPECT_NEAR(w(2), 0.25f, 1e-6f);       // delta/|-4| = 0.25
}

// ---------------------------------------------------------------------------
// TEST: PositionCorrectionFlatPlane
// Prior state: position 0.5 m above the z=0 plane.
// Many correspondences → update should pull position.z toward 0.
// ---------------------------------------------------------------------------
TEST(IekfUpdaterTest, PositionCorrectionFlatPlane) {
  LioState prior = make_prior(1.0f);
  prior.position = Eigen::Vector3f(0.0f, 0.0f, 0.5f);

  // 50 points all at height 0.5 m in the LiDAR frame (same as body frame —
  // identity extrinsic).  The plane is at z=0, so residual = -0.5 for each.
  const auto corrs = horizontal_plane_corrs(50, 0.5f);

  IekfConfig cfg;
  cfg.max_inner_iters = 10;
  cfg.convergence_threshold = 1e-5f;
  cfg.lidar_noise_std = 0.01f;

  const IekfResult result =
      iekf_update(prior, corrs, identity_extrinsic(), cfg);

  // With incremental update (no prior correction), the IEKF converges to the
  // ML estimate: points at lidar z=0.5 plus robot at world z=0.5 put world
  // points at z=1.0, and the plane is at z=0, so the ML solution is z=-0.5.
  // Verify that position.z decreased (moved in the correct direction).
  EXPECT_LT(result.state.position.z(), prior.position.z())
      << "Expected position.z to decrease after IEKF update";
}

// ---------------------------------------------------------------------------
// TEST: ConvergenceDetected — well-conditioned problem should converge.
// ---------------------------------------------------------------------------
TEST(IekfUpdaterTest, ConvergenceDetected) {
  LioState prior = make_prior(1.0f);
  prior.position = Eigen::Vector3f(0.0f, 0.0f, 0.1f);

  const auto corrs = horizontal_plane_corrs(100, 0.1f);

  IekfConfig cfg;
  cfg.max_inner_iters = 20;
  cfg.convergence_threshold = 1e-4f;
  cfg.lidar_noise_std = 0.01f;

  const IekfResult result =
      iekf_update(prior, corrs, identity_extrinsic(), cfg);

  EXPECT_TRUE(result.converged);
  EXPECT_GT(result.total_iterations, 0);
  EXPECT_LE(result.total_iterations, cfg.max_inner_iters);
}

// ---------------------------------------------------------------------------
// TEST: CovarianceShrinks — posterior trace < prior trace.
// ---------------------------------------------------------------------------
TEST(IekfUpdaterTest, CovarianceShrinks) {
  LioState prior = make_prior(1.0f);
  prior.position = Eigen::Vector3f(0.0f, 0.0f, 0.2f);

  const auto corrs = horizontal_plane_corrs(50, 0.2f);

  IekfConfig cfg;
  cfg.max_inner_iters = 10;
  cfg.convergence_threshold = 1e-5f;
  cfg.lidar_noise_std = 0.01f;

  const IekfResult result =
      iekf_update(prior, corrs, identity_extrinsic(), cfg);

  const float prior_trace = prior.covariance.trace();
  const float posterior_trace = result.state.covariance.trace();

  EXPECT_LT(posterior_trace, prior_trace)
      << "Posterior covariance trace should be smaller than prior";
}

// ---------------------------------------------------------------------------
// TEST: CovarianceRemainsPD — posterior covariance must be symmetric PD.
// ---------------------------------------------------------------------------
TEST(IekfUpdaterTest, CovarianceRemainsPD) {
  LioState prior = make_prior(1.0f);
  prior.position = Eigen::Vector3f(0.0f, 0.0f, 0.3f);

  const auto corrs = horizontal_plane_corrs(30, 0.3f);

  IekfConfig cfg;
  cfg.max_inner_iters = 5;
  cfg.lidar_noise_std = 0.01f;

  const IekfResult result =
      iekf_update(prior, corrs, identity_extrinsic(), cfg);

  const StateCovariance& P = result.state.covariance;

  // Symmetry: ||P - P^T||_F < tolerance.
  const float sym_err = (P - P.transpose()).norm();
  EXPECT_LT(sym_err, 1e-4f) << "Covariance is not symmetric";

  // Positive definiteness: all eigenvalues > 0.
  Eigen::SelfAdjointEigenSolver<StateCovariance> eig(P);
  ASSERT_EQ(eig.info(), Eigen::Success);
  const float min_eigen = eig.eigenvalues().minCoeff();
  EXPECT_GT(min_eigen, 0.0f)
      << "Covariance has non-positive eigenvalue: " << min_eigen;
}

// ---------------------------------------------------------------------------
// TEST: WithNullPko — pko=nullptr uses unit Huber delta (no crash, correct).
// ---------------------------------------------------------------------------
TEST(IekfUpdaterTest, WithNullPko) {
  LioState prior = make_prior(1.0f);
  prior.position = Eigen::Vector3f(0.0f, 0.0f, 0.1f);

  const auto corrs = horizontal_plane_corrs(20, 0.1f);

  IekfConfig cfg;
  cfg.max_inner_iters = 5;
  cfg.lidar_noise_std = 0.01f;

  // Explicitly pass nullptr PKO.
  EXPECT_NO_THROW({
    const IekfResult result =
        iekf_update(prior, corrs, identity_extrinsic(), cfg, nullptr);
    EXPECT_GT(result.num_correspondences, 0);
  });
}

// ---------------------------------------------------------------------------
// TEST: IterationCountReported — total_iterations in [1, max_inner_iters].
// ---------------------------------------------------------------------------
TEST(IekfUpdaterTest, IterationCountReported) {
  LioState prior = make_prior(1.0f);
  prior.position = Eigen::Vector3f(0.0f, 0.0f, 0.5f);

  const auto corrs = horizontal_plane_corrs(20, 0.5f);

  IekfConfig cfg;
  cfg.max_inner_iters = 5;
  cfg.lidar_noise_std = 0.01f;

  const IekfResult result =
      iekf_update(prior, corrs, identity_extrinsic(), cfg);

  EXPECT_GE(result.total_iterations, 1);
  EXPECT_LE(result.total_iterations, cfg.max_inner_iters);
}

// ---------------------------------------------------------------------------
// TEST: NumCorrespondencesReported — matches input size.
// ---------------------------------------------------------------------------
TEST(IekfUpdaterTest, NumCorrespondencesReported) {
  LioState prior = make_prior(1.0f);
  const int n_corrs = 42;
  const auto corrs = horizontal_plane_corrs(n_corrs, 0.1f);

  IekfConfig cfg;
  const IekfResult result =
      iekf_update(prior, corrs, identity_extrinsic(), cfg);

  EXPECT_EQ(result.num_correspondences, n_corrs);
}

// ---------------------------------------------------------------------------
// TEST: WithPkoAdaptive — using Pko (non-null) does not crash and returns
//       a valid result.
// ---------------------------------------------------------------------------
TEST(IekfUpdaterTest, WithPkoAdaptive) {
  LioState prior = make_prior(1.0f);
  prior.position = Eigen::Vector3f(0.0f, 0.0f, 0.2f);

  // Mix: many inliers at height 0.2 m, a few outliers at height 2.0 m.
  auto corrs = horizontal_plane_corrs(40, 0.2f);
  auto outliers = horizontal_plane_corrs(5, 2.0f);
  corrs.insert(corrs.end(), outliers.begin(), outliers.end());

  IekfConfig cfg;
  cfg.max_inner_iters = 5;
  cfg.lidar_noise_std = 0.01f;

  Pko pko;
  const IekfResult result =
      iekf_update(prior, corrs, identity_extrinsic(), cfg, &pko);

  EXPECT_GT(result.num_correspondences, 0);
  EXPECT_GE(result.total_iterations, 1);

  // Result covariance must be finite (no NaN/Inf).
  EXPECT_TRUE(result.state.covariance.allFinite());
}

// ---------------------------------------------------------------------------
// TEST: HuberWeightsExactDelta — |r| == delta → weight = 1.
// ---------------------------------------------------------------------------
TEST(IekfUpdaterTest, HuberWeightsExactDelta) {
  const float delta = 1.5f;
  Eigen::VectorXf residuals(1);
  residuals << delta;  // exactly at threshold

  const Eigen::VectorXf w = compute_huber_weights(residuals, delta);

  // |r| <= delta → weight = 1.
  EXPECT_NEAR(w(0), 1.0f, 1e-6f);
}

}  // namespace
}  // namespace core
}  // namespace tof_slam
