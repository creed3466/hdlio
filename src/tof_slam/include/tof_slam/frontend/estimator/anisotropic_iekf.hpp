// Copyright 2026 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// anisotropic_iekf.hpp — Sprint 13 P1 helpers (header-only).
//
// Per sprint13_architecture §3.1 + §3.3: builds the per-correspondence
// anisotropic information `Ω_eff = nᵀ(Σ_level + R·Σ_p·Rᵀ + ε·I)⁻¹ n`,
// where Σ_p is the bearing-range point-noise model
//   Σ_p = σ²_r·r̂r̂ᵀ + r²·σ²_θ·(I − r̂r̂ᵀ).
//
// This file is header-only inline so unit tests (anisotropic_iekf_test.cpp)
// can exercise it without pulling in tof_slam_frontend.a. Both L1 and L2
// IEKF weight sites in iekf_updater.cpp consume the same primitives.
//
// Invariants (sprint13_architecture §4):
//   I-1: Ω_eff preserves scalar point-to-plane residual semantics. The
//        Hessian add at L1 (:887) / L2 (:953) remains rank-1 in the
//        6-vector h_i.
//   I-2: When the caller passes Σ_level = σ²·I, the bearing-range Σ_p
//        contribution can be neutralized (σ²_r = σ²_θ = 0), and ε = 0.001f,
//        Ω_eff = 1/(0.001f + σ²). This is the G2 shim path.
//   I-3: ρ_ref_avia is NOT consumed here — γ gating lives in iekf_updater.cpp.
//        These helpers compute the anisotropic weight only.

#ifndef TOF_SLAM_FRONTEND_ESTIMATOR_ANISOTROPIC_IEKF_HPP_
#define TOF_SLAM_FRONTEND_ESTIMATOR_ANISOTROPIC_IEKF_HPP_

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

namespace tof_slam {
namespace core {

/// Builds Σ_p = σ²_r·r̂r̂ᵀ + r²·σ²_θ·(I − r̂r̂ᵀ) for one LiDAR point.
///
/// Inputs:
///   p_lidar       — point in sensor frame [m]
///   sigma_r_sq    — radial variance [m²] (typically lidar_noise_std²)
///   sigma_theta_sq — bearing variance [rad²] (sensor-global YAML)
///
/// Returns a 3×3 SPD matrix (symmetric, all eigenvalues > 0 unless both
/// variances are zero). Caller may add to other SPD matrices freely.
inline Eigen::Matrix3f build_sigma_p(const Eigen::Vector3f& p_lidar,
                                     float sigma_r_sq,
                                     float sigma_theta_sq) {
  const float r = p_lidar.norm();
  // Degenerate case (r → 0): radial direction undefined. Default to
  // identity-scaled bearing term + radial floor; harmless because near-
  // zero-range points are typically rejected upstream.
  if (r < 1.0e-6f) {
    return sigma_r_sq * Eigen::Matrix3f::Identity();
  }
  const Eigen::Vector3f r_hat = p_lidar / r;
  const Eigen::Matrix3f rrt = r_hat * r_hat.transpose();
  const Eigen::Matrix3f I_minus_rrt = Eigen::Matrix3f::Identity() - rrt;
  return sigma_r_sq * rrt + (r * r) * sigma_theta_sq * I_minus_rrt;
}

/// Computes Ω_eff = nᵀ (Σ_level + R·Σ_p·Rᵀ + ε·I)⁻¹ n.
///
/// Returns a scalar in 1/m² (information density along normal direction).
/// SPD-safe: the ε·I ridge guarantees the inverse exists for any rank
/// 0…3 PSD `Σ_level` and any non-zero ε.
///
/// Inputs:
///   normal_world   — surfel normal in world frame, unit length expected
///   Sigma_level    — Σ_{L1 or L2} surfel covariance in world frame (3×3 PSD)
///   R_world_from_lidar — rotation rotating LiDAR-frame Σ_p into world
///   p_lidar        — point in LiDAR sensor frame (used to build Σ_p)
///   sigma_r_sq, sigma_theta_sq — point-noise variances (see build_sigma_p)
///   epsilon        — ridge scalar (architect §3.4 default 1.0e-3)
///
/// Caller responsibility: pass a unit-length normal; ε > 0.
inline double compute_omega_eff(const Eigen::Vector3f& normal_world,
                                const Eigen::Matrix3f& Sigma_level,
                                const Eigen::Matrix3f& R_world_from_lidar,
                                const Eigen::Vector3f& p_lidar,
                                float sigma_r_sq,
                                float sigma_theta_sq,
                                float epsilon) {
  const Eigen::Matrix3f Sigma_p_lidar =
      build_sigma_p(p_lidar, sigma_r_sq, sigma_theta_sq);
  const Eigen::Matrix3f Sigma_p_world =
      R_world_from_lidar * Sigma_p_lidar * R_world_from_lidar.transpose();
  // Promote to double for the LDLT inverse — float HTRinvH accumulator
  // is double per iekf_updater.cpp:887, so callers expect double output.
  const Eigen::Matrix3d M =
      (Sigma_level.cast<double>() + Sigma_p_world.cast<double>() +
       static_cast<double>(epsilon) * Eigen::Matrix3d::Identity());
  // LDLT is SPD-aware and deterministic (no SVD branching). Project the
  // normal through the inverse: Ω_eff = nᵀ M⁻¹ n via `solve` to avoid
  // explicit inverse instability.
  const Eigen::Vector3d n_d = normal_world.cast<double>();
  const Eigen::Vector3d M_inv_n = M.ldlt().solve(n_d);
  return n_d.dot(M_inv_n);
}

/// UNIT-TEST ONLY helper. NOT for runtime use.
///
/// Originally intended as the G2 byte-compare reference. Runtime usage was
/// removed in S13-B.A.3 after empirical discovery (smoke A.3 fix-1 → -3):
/// the LIO pipeline's V3 SOP CV=0 % determinism (S12-B.V3) requires that
/// the shim path's w_i value bit-match the legacy fp32 expression
/// `robust_w / (0.001f + sigma2_i)`. This double-precision form differs by
/// 1-2 ulp per correspondence due to fp32 division vs fp64 reciprocal +
/// fp32 cast rounding; the ulp drift accumulates through HTRinvH and the
/// IEKF inner loop into multi-cm ATE divergence over ~2000 frames
/// (Dark01 0.114 m legacy vs 0.139 m via this helper at runtime).
///
/// Runtime shim path lives directly in iekf_updater.cpp:889-892 as the
/// inline fp32 expression. This helper remains for unit-test verification
/// of the Ω_eff mathematical form against the legacy formula, NOT to be
/// called from the hot loop.
inline double compute_omega_eff_shim_legacy(double normal_sigma2_legacy) {
  // 1.0 / (0.001 + normal_sigma2_legacy) — pure fp64 reference.
  // Runtime path in iekf_updater.cpp uses fp32 expression for byte-id.
  return 1.0 / (0.001 + normal_sigma2_legacy);
}

}  // namespace core
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_ESTIMATOR_ANISOTROPIC_IEKF_HPP_
