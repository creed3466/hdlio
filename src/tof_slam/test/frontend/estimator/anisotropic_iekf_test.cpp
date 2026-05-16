// Copyright 2026 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// anisotropic_iekf_test.cpp — Sprint 13 V1 unit tests.
//
// Per sprint13_architecture §9 V1: verify
//   (i)  Ω_eff(Σ, R, ε) > 0 for all rank-{1,2,3} PSD Σ
//   (ii) Ω_eff monotone-decreasing in ε
//   (iii) Ω_eff(ε=0.001f, Σ=σ²I, σ_p=0) ≡ 1/(0.001f + σ²) (G2 shim equivalence)
//   plus Σ_p positive definiteness and bearing-range structure.

#include "tof_slam/frontend/estimator/anisotropic_iekf.hpp"

#include <cmath>
#include <limits>

#include <Eigen/Dense>
#include <gtest/gtest.h>

namespace tof_slam {
namespace core {
namespace {

// Tolerance for fp64 algebraic equality (pure double-path operations).
constexpr double kTolFp64 = 1.0e-12;
// Tolerance for operations that involve float32 → double casts (Σ_level,
// Σ_p, ε all start as float). Architect R0.2 §2.5 derives per-w_i
// tolerance `max(1e-6, 1e-5·|w|)` for exactly this case.
constexpr double kTolMixed_abs = 1.0e-6;
constexpr double kTolMixed_rel = 1.0e-5;

// Tolerance helper matching architect R0.2 §2.5.
inline double mixed_tol(double w) {
  return std::max(kTolMixed_abs, kTolMixed_rel * std::abs(w));
}

// ---------------------------------------------------------------------------
// build_sigma_p — bearing-range point-noise model
// ---------------------------------------------------------------------------

TEST(BuildSigmaP, DegenerateZeroPoint_FallsBackToRadialIdentity) {
  // p ≈ 0 → fallback path: σ²_r · I (avoids 0/0 in r_hat).
  const Eigen::Vector3f p = Eigen::Vector3f::Zero();
  const auto S = build_sigma_p(p, 1.0e-4f, 9.0e-6f);
  EXPECT_NEAR(S(0, 0), 1.0e-4f, 1.0e-9);
  EXPECT_NEAR(S(1, 1), 1.0e-4f, 1.0e-9);
  EXPECT_NEAR(S(2, 2), 1.0e-4f, 1.0e-9);
  EXPECT_NEAR(S(0, 1), 0.0f, 1.0e-9);
}

TEST(BuildSigmaP, SymmetricPSD_ForTypicalAvia) {
  // Typical Avia point: 10 m forward, slight elevation.
  const Eigen::Vector3f p(10.0f, 1.0f, 0.5f);
  const auto S = build_sigma_p(p, 1.0e-4f /* (1cm)² */, 9.0e-6f /* (0.3°)² */);
  // Symmetry
  EXPECT_NEAR(S(0, 1), S(1, 0), 1.0e-12);
  EXPECT_NEAR(S(0, 2), S(2, 0), 1.0e-12);
  EXPECT_NEAR(S(1, 2), S(2, 1), 1.0e-12);
  // PSD via Cholesky
  Eigen::LLT<Eigen::Matrix3f> llt(S);
  EXPECT_EQ(llt.info(), Eigen::Success) << "Σ_p not PSD";
  // Trace > 0
  EXPECT_GT(S.trace(), 0.0f);
}

TEST(BuildSigmaP, RadialDirectionCarriesRadialVariance) {
  // Project Σ_p onto r̂: should equal σ²_r.
  const Eigen::Vector3f p(7.0f, 0.0f, 0.0f);
  const float sigma_r_sq = 2.5e-4f;
  const float sigma_theta_sq = 4.0e-6f;
  const auto S = build_sigma_p(p, sigma_r_sq, sigma_theta_sq);
  const Eigen::Vector3f r_hat = p / p.norm();
  const float proj = r_hat.transpose() * S * r_hat;
  EXPECT_NEAR(proj, sigma_r_sq, 1.0e-9);
}

TEST(BuildSigmaP, TangentialDirectionScalesAsR2_SigmaTheta2) {
  // Project Σ_p onto a unit vector orthogonal to r̂: should equal r²·σ²_θ.
  const Eigen::Vector3f p(10.0f, 0.0f, 0.0f);
  const float sigma_r_sq = 1.0e-4f;
  const float sigma_theta_sq = 9.0e-6f;
  const auto S = build_sigma_p(p, sigma_r_sq, sigma_theta_sq);
  const Eigen::Vector3f tang(0.0f, 1.0f, 0.0f);
  const float proj = tang.transpose() * S * tang;
  const float expected = 100.0f * sigma_theta_sq;  // r² = 100
  EXPECT_NEAR(proj, expected, 1.0e-7);
}

// ---------------------------------------------------------------------------
// compute_omega_eff — anisotropic projected information
// ---------------------------------------------------------------------------

TEST(ComputeOmegaEff, PositivityForRank3Sigma) {
  // Σ_level full-rank diagonal anisotropic
  Eigen::Matrix3f Sigma_level = Eigen::Matrix3f::Zero();
  Sigma_level(0, 0) = 1.0e-3f;
  Sigma_level(1, 1) = 5.0e-4f;
  Sigma_level(2, 2) = 2.0e-4f;
  const Eigen::Vector3f n(0.0f, 0.0f, 1.0f);
  const Eigen::Vector3f p(8.0f, 2.0f, 0.5f);
  const Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
  const double w = compute_omega_eff(n, Sigma_level, R, p, 1.0e-4f, 9.0e-6f, 1.0e-3f);
  EXPECT_GT(w, 0.0);
  EXPECT_TRUE(std::isfinite(w));
}

TEST(ComputeOmegaEff, PositivityForRank2Sigma) {
  // Σ_level rank-2 (in-tangent only — typical planar surfel after S12 B.B.1)
  Eigen::Vector3f n(0.0f, 0.0f, 1.0f);
  const Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
  const Eigen::Matrix3f tangent_proj = I - n * n.transpose();
  const Eigen::Matrix3f Sigma_level = 8.0e-4f * tangent_proj;  // rank-2 PSD
  const Eigen::Vector3f p(12.0f, -1.0f, 0.8f);
  const double w =
      compute_omega_eff(n, Sigma_level, Eigen::Matrix3f::Identity(), p, 1.0e-4f, 9.0e-6f, 1.0e-3f);
  EXPECT_GT(w, 0.0);
  EXPECT_TRUE(std::isfinite(w));
}

TEST(ComputeOmegaEff, PositivityForRank1Sigma) {
  // Σ_level rank-1 (HC1 paper-only — pure in-normal)
  Eigen::Vector3f n(1.0f, 0.0f, 0.0f);
  const Eigen::Matrix3f Sigma_level = 1.0e-3f * n * n.transpose();
  const Eigen::Vector3f p(5.5f, 0.3f, 0.2f);
  const double w =
      compute_omega_eff(n, Sigma_level, Eigen::Matrix3f::Identity(), p, 1.0e-4f, 9.0e-6f, 1.0e-3f);
  EXPECT_GT(w, 0.0);
  EXPECT_TRUE(std::isfinite(w));
}

TEST(ComputeOmegaEff, PositivityForRank0Sigma_Pure_EpsilonRidge) {
  // Σ_level = 0 — degenerate but ε·I makes inverse exist.
  const Eigen::Matrix3f Sigma_level = Eigen::Matrix3f::Zero();
  const Eigen::Vector3f n(0.0f, 0.0f, 1.0f);
  const Eigen::Vector3f p(3.0f, 0.0f, 0.0f);
  const double w =
      compute_omega_eff(n, Sigma_level, Eigen::Matrix3f::Identity(), p, 0.0f, 0.0f, 1.0e-3f);
  EXPECT_GT(w, 0.0);
  // With Σ_level=0, Σ_p=0, ε=1e-3 → Ω_eff = nᵀ(εI)⁻¹n = 1/ε ≈ 1000.
  // R0.2 §2.5 tolerance: float32 ε cast introduces 1e-5 relative drift.
  EXPECT_NEAR(w, 1000.0, mixed_tol(1000.0));
}

TEST(ComputeOmegaEff, MonotoneDecreasingInEpsilon) {
  // Architect §9 V1 (ii): Ω_eff monotone-decreasing in ε.
  Eigen::Matrix3f Sigma_level = Eigen::Matrix3f::Zero();
  Sigma_level(0, 0) = 5.0e-4f;
  Sigma_level(1, 1) = 3.0e-4f;
  Sigma_level(2, 2) = 1.0e-4f;
  const Eigen::Vector3f n = Eigen::Vector3f(1.0f, 1.0f, 1.0f).normalized();
  const Eigen::Vector3f p(7.0f, 0.0f, 0.0f);
  const Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
  const double w_small_eps =
      compute_omega_eff(n, Sigma_level, R, p, 1.0e-4f, 9.0e-6f, 1.0e-5f);
  const double w_default =
      compute_omega_eff(n, Sigma_level, R, p, 1.0e-4f, 9.0e-6f, 1.0e-3f);
  const double w_large_eps =
      compute_omega_eff(n, Sigma_level, R, p, 1.0e-4f, 9.0e-6f, 1.0e-1f);
  // Strict monotone decreasing
  EXPECT_GT(w_small_eps, w_default);
  EXPECT_GT(w_default, w_large_eps);
  EXPECT_GT(w_small_eps, w_large_eps);
}

TEST(ComputeOmegaEff, LimitEpsilonToInfinity_ApproachesZero) {
  // Architect §9 V1: Ω_eff(ε→∞) → 0.
  const Eigen::Matrix3f Sigma_level = 1.0e-3f * Eigen::Matrix3f::Identity();
  const Eigen::Vector3f n(1.0f, 0.0f, 0.0f);
  const Eigen::Vector3f p(5.0f, 0.0f, 0.0f);
  const double w_huge =
      compute_omega_eff(n, Sigma_level, Eigen::Matrix3f::Identity(), p, 0.0f, 0.0f, 1.0e6f);
  // 1/(1e6 + 1e-3 + 0) ≈ 1e-6, projected on unit n → ≈ 1e-6
  EXPECT_LT(w_huge, 1.0e-5);
  EXPECT_GT(w_huge, 0.0);
}

TEST(ComputeOmegaEff, ShimEquivalence_MatchesLegacyScalar) {
  // Architect §4 I-2 (G2 shim equivalence). When Σ_level = σ²·I,
  // σ_p = 0, ε = 0.001f, Ω_eff = 1/(0.001 + σ²).
  // Test multiple σ² values matching realistic HDLIO sigma2_i magnitudes.
  for (double sigma2_legacy : {0.0, 1.0e-8, 1.0e-7, 1.0e-6, 1.0e-5, 1.0e-3}) {
    const Eigen::Matrix3f Sigma_level =
        static_cast<float>(sigma2_legacy) * Eigen::Matrix3f::Identity();
    const Eigen::Vector3f n(0.0f, 0.0f, 1.0f);
    const Eigen::Vector3f p(10.0f, 1.0f, 0.0f);  // any non-zero point
    const double w_aniso = compute_omega_eff(
        n, Sigma_level, Eigen::Matrix3f::Identity(),
        p, 0.0f /* σ_r²=0 */, 0.0f /* σ_θ²=0 */, 0.001f /* ε=legacy floor */);
    const double w_legacy = compute_omega_eff_shim_legacy(sigma2_legacy);
    // Architect R0.2 §2.5 per-w_i tolerance: max(1e-6, 1e-5·|w|).
    // Float32 → double cast of (Σ_level=σ²I) + (ε=0.001f) introduces
    // O(1e-5) relative drift, which is the documented shim envelope.
    EXPECT_NEAR(w_aniso, w_legacy, mixed_tol(w_legacy))
        << "sigma2_legacy = " << sigma2_legacy;
  }
}

TEST(ComputeOmegaEff, UnitNormalAlongLargestPrincipalDirection_GivesSmallestWeight) {
  // Anisotropy intuition: when n aligns with Σ_level's largest eigendirection
  // (most uncertain), Ω_eff should be smaller than when n aligns with the
  // smallest eigendirection (most certain).
  Eigen::Matrix3f Sigma_level = Eigen::Matrix3f::Zero();
  Sigma_level(0, 0) = 1.0e-2f;  // largest variance → largest uncertainty
  Sigma_level(1, 1) = 1.0e-4f;
  Sigma_level(2, 2) = 1.0e-6f;  // smallest variance → smallest uncertainty
  const Eigen::Vector3f p(3.0f, 0.0f, 0.0f);
  const Eigen::Vector3f n_uncertain(1.0f, 0.0f, 0.0f);  // along axis 0
  const Eigen::Vector3f n_certain(0.0f, 0.0f, 1.0f);    // along axis 2

  const double w_uncertain = compute_omega_eff(
      n_uncertain, Sigma_level, Eigen::Matrix3f::Identity(), p, 0.0f, 0.0f, 1.0e-6f);
  const double w_certain = compute_omega_eff(
      n_certain, Sigma_level, Eigen::Matrix3f::Identity(), p, 0.0f, 0.0f, 1.0e-6f);
  // Certain direction should yield MUCH larger information weight.
  EXPECT_GT(w_certain, w_uncertain * 10.0)
      << "Anisotropy not reflected: w_uncertain=" << w_uncertain
      << " w_certain=" << w_certain;
}

TEST(ComputeOmegaEff, RotationCovariance_RWorldAppliesToSigmaP) {
  // Two equivalent setups should give the same Ω_eff:
  //  Setup A: identity rotation, point in world frame already.
  //  Setup B: 90° rotation about z, point pre-rotated in lidar frame.
  Eigen::Matrix3f Sigma_level = Eigen::Matrix3f::Zero();
  Sigma_level(0, 0) = 1.0e-4f;
  Sigma_level(1, 1) = 1.0e-4f;
  Sigma_level(2, 2) = 1.0e-4f;
  const Eigen::Vector3f n(0.0f, 0.0f, 1.0f);

  const Eigen::Vector3f p_a(5.0f, 0.0f, 0.5f);
  const Eigen::Matrix3f R_a = Eigen::Matrix3f::Identity();
  const double w_a =
      compute_omega_eff(n, Sigma_level, R_a, p_a, 1.0e-4f, 9.0e-6f, 1.0e-3f);

  // Rotate p by -90° about z and rotate R by +90° about z — equivalent point.
  Eigen::AngleAxisf rot_z(static_cast<float>(M_PI / 2.0), Eigen::Vector3f::UnitZ());
  const Eigen::Matrix3f R_b = rot_z.toRotationMatrix();
  const Eigen::Vector3f p_b = R_b.transpose() * p_a;
  const double w_b =
      compute_omega_eff(n, Sigma_level, R_b, p_b, 1.0e-4f, 9.0e-6f, 1.0e-3f);

  // Rotation algebra is exact in fp64, but float32 inputs introduce
  // ~1e-5 rel drift in the LDLT inverse. Use R0.2 §2.5 envelope.
  EXPECT_NEAR(w_a, w_b, mixed_tol(w_a));
}

TEST(ComputeOmegaEff, MonotonicityInPointRange_TangentialGrowsWithRange) {
  // Σ_p tangential = r² σ²_θ — bigger r means more tangential noise.
  // For a normal orthogonal to r̂, larger r should yield smaller Ω_eff.
  const Eigen::Matrix3f Sigma_level = Eigen::Matrix3f::Zero();
  const Eigen::Vector3f n(0.0f, 0.0f, 1.0f);  // orthogonal to forward (x)
  const Eigen::Matrix3f R = Eigen::Matrix3f::Identity();

  const double w_near = compute_omega_eff(
      n, Sigma_level, R, Eigen::Vector3f(2.0f, 0.0f, 0.0f), 0.0f, 1.0e-4f, 1.0e-6f);
  const double w_far = compute_omega_eff(
      n, Sigma_level, R, Eigen::Vector3f(20.0f, 0.0f, 0.0f), 0.0f, 1.0e-4f, 1.0e-6f);
  EXPECT_GT(w_near, w_far) << "near=" << w_near << " far=" << w_far;
}

// ---------------------------------------------------------------------------
// compute_omega_eff_shim_legacy — direct legacy formula
// ---------------------------------------------------------------------------

TEST(ComputeOmegaEffShimLegacy, MatchesLegacyDivision) {
  for (double s2 : {0.0, 1.0e-8, 1.0e-6, 1.0e-3}) {
    const double w = compute_omega_eff_shim_legacy(s2);
    const double w_expected = 1.0 / (0.001 + s2);
    EXPECT_NEAR(w, w_expected, kTolFp64);
  }
}

// ---------------------------------------------------------------------------
// S13-B.B.1 V2: DG-A γ control gate regime table (architect §3.2)
// ---------------------------------------------------------------------------

/// Standalone γ computation mirroring iekf_updater.cpp logic for unit-test
/// verification. Identical algebra; no surrounding IEKF state.
inline double compute_gamma(double rho_L2, double cos_12, double rho_ref_avia) {
  if (rho_ref_avia <= 0.0 || rho_L2 <= 0.0) {
    return 1.0;  // disabled / fallback: keep γ at prior value (init 1.0)
  }
  const double cos2 = cos_12 * cos_12;
  double beta = 1.0 - rho_L2 / rho_ref_avia;
  if (beta < 0.0) beta = 0.0;
  if (beta > 1.0) beta = 1.0;
  double gamma = rho_L2 * (1.0 - beta * cos2);
  if (gamma < 0.0) gamma = 0.0;
  return gamma;
}

TEST(DgaGate, Disabled_RhoRefZero_GammaIs1) {
  // ρ_ref_avia = 0 (default pre-G5) → fallback path, γ stays at init.
  EXPECT_NEAR(compute_gamma(0.001, 0.96, 0.0), 1.0, kTolFp64);
  EXPECT_NEAR(compute_gamma(0.5,   0.5,  0.0), 1.0, kTolFp64);
  EXPECT_NEAR(compute_gamma(1.0,   1.0,  0.0), 1.0, kTolFp64);
}

TEST(DgaGate, Disabled_RhoL2Zero_GammaIs1) {
  // ρ_L2 = 0 (degenerate L2) → fallback, γ = 1.
  EXPECT_NEAR(compute_gamma(0.0, 0.96, 0.01), 1.0, kTolFp64);
}

TEST(DgaGate, Regime_i_L1Strong_Agreement_Suppresses) {
  // Avia empirical (S12-B.A.3): ρ_L2 ≈ 0.0016, cos²θ_12 ≈ 0.93 (cos≈0.96).
  // With ρ_ref calibrated to median, ρ_L2 ≪ ρ_ref → β ≈ 1 → γ → 0.
  // (i) L1 strong, channels agree → γ near 0 (suppress).
  const double rho_L2 = 0.0016;
  const double cos_12 = 0.96;  // cos²=0.9216
  const double rho_ref = 0.05;  // hypothetical G5-baked, ≫ rho_L2
  const double gamma = compute_gamma(rho_L2, cos_12, rho_ref);
  // β = clip(1 − 0.0016/0.05, 0, 1) = clip(0.968, 0, 1) = 0.968
  // γ = 0.0016 · (1 − 0.968 · 0.9216) = 0.0016 · 0.1077 ≈ 1.72e-4
  EXPECT_LT(gamma, 0.001) << "Regime (i): γ should approach 0 when channels agree and L2 is weak";
  EXPECT_GT(gamma, 0.0);
}

TEST(DgaGate, Regime_ii_L2Competitive_KeepAtRhoL2) {
  // (ii) ρ_L2 ≈ ρ_ref → β ≈ 0 → γ = ρ_L2 · 1 = ρ_L2 (no agreement penalty).
  const double rho_L2 = 0.05;
  const double rho_ref = 0.05;  // β = 0
  const double cos_12 = 0.96;
  const double gamma = compute_gamma(rho_L2, cos_12, rho_ref);
  // β = clip(1 − 1.0, 0, 1) = 0 → γ = 0.05 · (1 − 0) = 0.05
  EXPECT_NEAR(gamma, 0.05, kTolFp64);
}

TEST(DgaGate, Regime_iii_L1Strong_Disagreement_KeepIndependentInfo) {
  // (iii) ρ_L2 small, but channels disagree (cos² near 0) → γ → ρ_L2.
  const double rho_L2 = 0.0016;
  const double cos_12 = 0.1;  // cos² = 0.01
  const double rho_ref = 0.05;
  const double gamma = compute_gamma(rho_L2, cos_12, rho_ref);
  // β = 0.968, γ = 0.0016 · (1 − 0.968 · 0.01) = 0.0016 · 0.99032 ≈ 1.585e-3
  EXPECT_NEAR(gamma, rho_L2 * (1.0 - 0.968 * 0.01), 1e-6);
  EXPECT_GT(gamma, 0.95 * rho_L2)
      << "Regime (iii): γ should be near ρ_L2 when channels disagree";
}

TEST(DgaGate, Boundary_BetaClampedToOne_WhenRhoL2NegativeRelativeTo_RhoRef) {
  // ρ_L2 close to 0 → 1 − 0/ρ_ref = 1 → β = 1 (max penalty active).
  const double gamma = compute_gamma(1e-9, 1.0, 0.01);
  // γ = 1e-9 · (1 − 1·1) = 0 (clamped)
  EXPECT_NEAR(gamma, 0.0, 1e-12);
}

TEST(DgaGate, Boundary_BetaClampedToZero_WhenRhoL2GreaterThan_RhoRef) {
  // ρ_L2 > ρ_ref → 1 − ρ_L2/ρ_ref < 0 → β clamped to 0.
  // γ = ρ_L2 · (1 − 0·cos²) = ρ_L2 regardless of cos².
  const double rho_L2 = 0.1;
  const double rho_ref = 0.05;
  EXPECT_NEAR(compute_gamma(rho_L2, 0.0, rho_ref), rho_L2, kTolFp64);
  EXPECT_NEAR(compute_gamma(rho_L2, 1.0, rho_ref), rho_L2, kTolFp64);
}

TEST(DgaGate, RhoL2_MonotoneSuppression_WithAgreement) {
  // Hold cos²=1 (agree), vary ρ_L2 from 0 → ρ_ref. As ρ_L2 grows
  // toward ρ_ref, β drops toward 0, so γ increases toward ρ_L2.
  const double rho_ref = 0.05;
  const double cos_12 = 1.0;
  const double g_small = compute_gamma(0.001, cos_12, rho_ref);
  const double g_med   = compute_gamma(0.025, cos_12, rho_ref);
  const double g_large = compute_gamma(0.05,  cos_12, rho_ref);
  EXPECT_LT(g_small, g_med);
  EXPECT_LT(g_med,   g_large);
  EXPECT_NEAR(g_large, 0.05, kTolFp64);
}

TEST(DgaGate, Determinism_StaticFormula_NoState) {
  // γ is a pure function of (ρ_L2, cos_12, ρ_ref). No EMA, no runtime
  // state → I-3 determinism preserved. Same inputs → same outputs across
  // any number of calls.
  const double rho_L2 = 0.003;
  const double cos_12 = 0.85;
  const double rho_ref = 0.02;
  double prev = 0.0;
  for (int i = 0; i < 10; ++i) {
    const double cur = compute_gamma(rho_L2, cos_12, rho_ref);
    if (i > 0) EXPECT_EQ(cur, prev);
    prev = cur;
  }
}

}  // namespace
}  // namespace core
}  // namespace tof_slam

// catkin_add_gtest under this docker image's catkin/gtest configuration does
// not auto-link gtest_main. Provide an explicit main() — standard GoogleTest
// pattern (https://google.github.io/googletest/primer.html).
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
