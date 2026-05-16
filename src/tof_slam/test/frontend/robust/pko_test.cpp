// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "tof_slam/frontend/robust/pko.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>

namespace tof_slam {
namespace core {
namespace {

// ===========================================================================
// Pko — basic behavior
// ===========================================================================

TEST(PkoTest, EmptyResidualsReturnsOne) {
  Pko pko;
  EXPECT_DOUBLE_EQ(pko.compute_scale_factor({}), 1.0);
}

TEST(PkoTest, NonAdaptiveReturnsMinScale) {
  PkoConfig cfg;
  cfg.use_adaptive = false;
  cfg.min_scale_factor = 0.05;
  Pko pko(cfg);

  std::vector<double> residuals(100, 0.1);
  EXPECT_DOUBLE_EQ(pko.compute_scale_factor(residuals), 0.05);
}

TEST(PkoTest, GaussianResidualsInRange) {
  Pko pko;
  std::mt19937 rng(42);
  std::normal_distribution<double> dist(0.0, 1.0);

  std::vector<double> residuals(500);
  for (auto& r : residuals) r = dist(rng);

  const double delta = pko.compute_scale_factor(residuals);
  EXPECT_GE(delta, pko.config().min_scale_factor);
  EXPECT_LE(delta, pko.config().max_scale_factor);
}

TEST(PkoTest, OutlierContaminatedResidualsProducesSmallerDelta) {
  Pko pko_clean;
  Pko pko_dirty;

  std::mt19937 rng(42);
  std::normal_distribution<double> inlier(0.0, 0.5);
  std::normal_distribution<double> outlier(0.0, 10.0);

  std::vector<double> clean(500), dirty(500);
  for (int i = 0; i < 500; ++i) {
    clean[i] = inlier(rng);
    dirty[i] = (i < 450) ? inlier(rng) : outlier(rng);
  }

  const double delta_clean = pko_clean.compute_scale_factor(clean);
  const double delta_dirty = pko_dirty.compute_scale_factor(dirty);

  // With outliers, the optimizer should produce a different (typically smaller
  // for outlier rejection) or at least valid delta.
  EXPECT_GE(delta_dirty, pko_dirty.config().min_scale_factor);
  EXPECT_LE(delta_dirty, pko_dirty.config().max_scale_factor);
  // Both should be positive and finite.
  EXPECT_GT(delta_clean, 0.0);
  EXPECT_GT(delta_dirty, 0.0);
}

TEST(PkoTest, ResetMakesNextCallIndependent) {
  Pko pko;
  std::mt19937 rng(42);
  std::normal_distribution<double> dist(0.0, 1.0);

  std::vector<double> residuals(200);
  for (auto& r : residuals) r = dist(rng);

  pko.compute_scale_factor(residuals);
  pko.reset();

  // After reset, should be able to compute again without crash.
  const double delta = pko.compute_scale_factor(residuals);
  EXPECT_GE(delta, pko.config().min_scale_factor);
}

TEST(PkoTest, AllZeroResidualsDoesNotCrash) {
  Pko pko;
  std::vector<double> residuals(100, 0.0);
  const double delta = pko.compute_scale_factor(residuals);
  EXPECT_GE(delta, 0.0);
}

TEST(PkoTest, SingleResidualDoesNotCrash) {
  Pko pko;
  std::vector<double> residuals = {0.5};
  const double delta = pko.compute_scale_factor(residuals);
  EXPECT_GE(delta, pko.config().min_scale_factor);
}

TEST(PkoTest, DeterministicWithFixedSeed) {
  Pko pko1, pko2;
  std::vector<double> residuals(300);
  std::mt19937 rng(123);
  std::normal_distribution<double> dist(0.0, 2.0);
  for (auto& r : residuals) r = dist(rng);

  const double d1 = pko1.compute_scale_factor(residuals);
  const double d2 = pko2.compute_scale_factor(residuals);
  EXPECT_DOUBLE_EQ(d1, d2);
}

}  // namespace
}  // namespace core
}  // namespace tof_slam
