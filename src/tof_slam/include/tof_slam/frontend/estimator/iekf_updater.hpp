// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// iekf_updater.hpp — Information-form Iterated Extended Kalman Filter update.
//
// Implements the inner-loop IEKF update step used by the IG-LIO estimator:
//
//   K1 = (H^T R^{-1} H + P^{-1})^{-1}
//   dx = K1 * (H^T R^{-1} z - P^{-1} * (x_k - x_prior))
//   x  = x_prior + dx
//   P_new = K1   (posterior covariance in information form)
//
// The outer re-linearisation loop (map re-query) lives in LioEstimator.
// This module handles the inner loop only (fixed correspondences).

#ifndef TOF_SLAM_FRONTEND_ESTIMATOR_IEKF_UPDATER_HPP_
#define TOF_SLAM_FRONTEND_ESTIMATOR_IEKF_UPDATER_HPP_

#include <vector>

#include "tof_slam/frontend/estimator/correspondence.hpp"
#include "tof_slam/frontend/robust/pko.hpp"
#include "tof_slam/common/lie/se3.hpp"
#include "tof_slam/common/types/state.hpp"

namespace tof_slam {
namespace core {

// ---------------------------------------------------------------------------
// IekfConfig
// ---------------------------------------------------------------------------

struct IekfConfig {
  int max_outer_iters = 4;          // reference: 4 (from config)
  int max_inner_iters = 4;          // reference: 4 (hardcoded)
  float convergence_threshold = 1e-3f;  // reference: 0.001
  float lidar_noise_std = 0.05f;    // reference default: 0.05

  /// Correspondence reuse: skip find_correspondences for outer iterations
  /// >= this value and reuse the previous iteration's correspondence list.
  /// 0 = disabled (recompute every iteration, default).
  /// 2 = recompute on iter 0,1; reuse iter 1's corrs for iter 2,3.
  /// Reduces cf cost by (max_outer - cf_reuse_after) / max_outer.
  /// Later IEKF iterations have small state corrections, so correspondence
  /// associations change minimally — reuse is safe.
  int cf_reuse_after_iter = 0;

  /// Outer-loop early termination: if the position delta between consecutive
  /// outer iterations drops below this threshold (meters), break the outer loop.
  /// 0 = disabled (run all max_outer_iters, default).
  /// Recommended: 0.01 — 1cm position change is negligible for map accuracy.
  /// Only checked for outer >= 1 (need at least 2 outer iterations).
  /// When triggered, covariance is updated from the last inner-loop result.
  float outer_convergence_threshold = 0.0f;

  /// Maximum OpenMP threads for IEKF build_info accumulation.
  /// 0 = use OMP_NUM_THREADS (default). Parallelizes the per-correspondence
  /// HTRinvH rank-1 accumulation loop (dominant cost at ~40ms with 400 corrs).
  int iekf_omp_threads = 0;

  // --- Degeneracy-Aware IEKF (LODESTAR/SelectiveKF-inspired) ---
  bool enable_degeneracy_detection = true;
  /// Absolute eigenvalue threshold for H^T R^{-1} H (6×6).
  /// Directions with λ < threshold are gated.  This is a hard floor
  /// only useful for very sparse correspondence sets (N < 50).
  float degeneracy_threshold = 10.0f;

  /// Relative eigenvalue ratio threshold (λ_i / λ_max).
  /// Directions with ratio below this are considered degenerate.
  /// 0 = disabled (use only absolute threshold).
  /// Recommended: 0.01 (directions below 1% of max are degenerate).
  /// This is sensor-independent and works for any correspondence count.
  float degeneracy_ratio_threshold = 0.0f;

  /// Soft floor for degenerate directions (0-1).
  /// 0 = hard gating (completely zero out degenerate directions).
  /// >0 = soft gating (scale contribution by floor + (1-floor) * ratio/threshold).
  /// Soft gating maintains a weak measurement prior in degenerate directions,
  /// preventing complete reliance on IMU.
  float degeneracy_soft_floor = 0.0f;

  /// Eigenvalue ratio threshold for map-insertion direction extraction.
  /// Controls whether Eq.(4) direction-selective alpha modulation fires.
  /// A direction is extracted for map protection when λ₀/λ_max < this value
  /// AND the eigenvector has a significant translation component (pos_norm>0.1).
  /// Separate from degeneracy_ratio_threshold to allow independent tuning:
  ///   - degeneracy_ratio_threshold: gates IEKF + overall modulation decision
  ///   - map_degen_ratio_threshold: selects Eq.(4) vs global scalar fallback
  /// Default: 0.01 (direction has < 1% of max constraint → extract for map).
  float map_degen_ratio_threshold = 0.01f;

  // --- Covariance floor (prevents over-confidence) ---
  float p_floor_rot = 1e-6f;    ///< Min diagonal for rotation block
  float p_floor_pos = 1e-6f;    ///< Min diagonal for position block
  float p_floor_vel = 1e-4f;    ///< Min diagonal for velocity block
  float p_floor_bias = 1e-8f;   ///< Min diagonal for bias blocks
  float p_floor_grav = 1e-8f;   ///< Min diagonal for gravity block

  // --- IMU Bias Pseudo-Observation (Point-LIO inspired) ---
  /// Add zero-mean Gaussian prior on IMU biases to make them directly
  /// observable in the IEKF.  Without this, biases are only indirectly
  /// observable through pos-vel cross-covariance, which decays in float32.
  bool enable_bias_pseudo_obs = false;
  float bias_bg_sigma = 0.01f;   ///< Gyro bias pseudo-obs noise std [rad/s]
  float bias_ba_sigma = 0.1f;    ///< Acc bias pseudo-obs noise std [m/s²]

  // --- Velocity Pseudo-Observation ---
  /// Add zero-mean prior on velocity to counteract systematic velocity
  /// overestimation caused by sparse correspondences (Avia non-repetitive).
  /// This directly constrains velocity in the IEKF information matrix,
  /// preventing path ratio blow-up (observed: 3.09× without this).
  bool enable_velocity_pseudo_obs = false;
  float velocity_sigma = 1.0f;   ///< Velocity pseudo-obs noise std [m/s]

  /// Direction-selective velocity constraint: when degeneracy is detected,
  /// apply a tighter velocity prior specifically in the degenerate direction.
  /// This prevents axial drift in tunnels where LiDAR has no constraint
  /// along the tunnel axis, while keeping loose constraints elsewhere so
  /// LiDAR dominates in well-observed directions.
  float velocity_degen_sigma = 0.0f;  ///< 0 = disabled; >0 = tight sigma in degen dir

  // --- Gravity norm pseudo-observation ---
  /// Enforce ||g|| ≈ 9.81 via soft constraint in information matrix.
  /// Residual: r_g = ||g|| - 9.81, Jacobian: g^T / ||g|| (1×3).
  /// Prevents gravity magnitude drift without changing state dimension.
  bool enable_gravity_norm_constraint = true;
  float gravity_norm_sigma = 0.01f;  ///< Gravity norm constraint noise std [m/s²]


  // --- L2 multi-scale correspondence weighting ---
  /// When enabled, each correspondence with has_l2=true contributes an
  /// additional rank-1 update to H^T R^{-1} H from the coarser L2 surfel.
  /// This is a PSD addition — posterior covariance can only improve.
  bool  enable_l2_correspondences = false;
  float l2_noise_scale = 9.0f;  ///< sigma^2_L2 = l2_noise_scale x sigma^2_base

  // --- Adaptive Measurement Noise ---
  bool enable_adaptive_noise = false;     // OFF by default (NTU VIRAL compatible)
  float adaptive_range_scale = 0.3f;
  float adaptive_range_ref = 25.0f;
  float adaptive_incidence_scale = 1.5f;
  float adaptive_planarity_scale = 2.0f;

  // --- Range-Inverse Weight (indoor close-range boost) ---
  /// When enabled, sigma2 is scaled by (range/range_ref)^power, giving
  /// close-range points lower noise (higher weight) and far-range points
  /// higher noise (lower weight).  Independent of enable_adaptive_noise.
  /// When both are active, this REPLACES the additive range_penalty term
  /// to avoid double-counting range effects.
  bool  enable_range_inverse_weight = false;
  float range_inverse_ref = 10.0f;      ///< Reference range [m]
  float range_inverse_power = 1.0f;     ///< Exponent (1=linear, 2=quadratic)
  float range_inverse_min_ratio = 0.1f; ///< Floor clamp to avoid instability

  // --- Sigma2 Normal (surfel normal uncertainty) ---
  /// When enabled, adds distance-dependent normal uncertainty to measurement noise:
  ///   sigma2_i = sigma2_base + dist_sq * normal_sigma2
  /// Beneficial for dense sensors (Mid-360). Harmful for sparse 16ch (NTU VIRAL)
  /// where distant correspondences are the only measurements available.
  bool enable_sigma2_normal = true;  // default true for backward compat

  // --- Sharing Weight (surfel sharing down-weighting) ---
  /// Down-weight correspondences that share the same surfel.
  /// threshold: only apply when sharing_count > threshold (v12 NTU: 3, current: 1)
  /// ref: divide by sharing_count/ref instead of sharing_count (v12 NTU: 3.0)
  int sharing_weight_threshold = 1;    // default 1 for backward compat
  float sharing_weight_ref = 1.0f;     // default 1.0 for backward compat

  // --- A-matrix (SO(3) Left Jacobian) manifold correction ---
  /// When enabled, applies the SO(3) left Jacobian correction to the rotation
  /// block of P at each IEKF iteration and to the posterior covariance after
  /// convergence (FAST-LIO2 style).  Set to false to match v5.0-H1 behavior,
  /// which achieved better performance on M3DGR (0.327m avg ATE vs 1.012m).
  bool enable_a_matrix_correction = true;

  // --- P2D (Point-to-Distribution) IEKF gate ---
  float p2d_chi2_threshold = 11.345f;  ///< Chi² outlier gate in IEKF (df=3, p=0.99)
  bool enable_p2d_adaptive_noise = true;  ///< P2D-specific adaptive noise (separate from P2P)
  float p2d_omega_scale = 1.0f;  ///< Global P2D precision scaling: Omega *= scale.
                                  ///< Accounts for centroid uncertainty not captured in
                                  ///< geometric covariance (shape only). <1.0 reduces P2D
                                  ///< influence relative to P2P.

  // --- Debug timing ---
  bool enable_debug_timing = false;  ///< Enable sub-module timing

  // --- ICDR: Information-theoretic Continuous Degeneracy Regularization ---
  // Novel method replacing binary eigenvalue gating with three components:
  // 1. Information Ratio (IR): per-direction LiDAR/prior information ratio
  // 2. Continuous Sigmoid Transition (CST): smooth weighting, no discontinuity
  // 3. Temporal Information Persistence (TIP): geometric memory across frames
  bool enable_icdr = false;               ///< Master toggle (replaces apply_degeneracy_remap)
  float icdr_rho_thresh = 0.3f;           ///< IR threshold ρ ∈ [0,1] — below = degeneracy region
  float icdr_tau = 0.05f;                 ///< Sigmoid sharpness (smaller = sharper transition)
  float icdr_w_min = 0.01f;              ///< Minimum weight floor (avoids total info loss)
  bool enable_icdr_tip = true;            ///< TIP sub-toggle (can use IR+CST without TIP)
  float icdr_tip_alpha = 0.3f;           ///< EMA smoothing factor (higher = more responsive)
  float icdr_tip_beta = 0.5f;            ///< Persistence strength scaling
  float icdr_tip_d_decay = 2.0f;         ///< Pose displacement decay constant [m]

  // --- DG-A: Per-Channel Anisotropic Degeneracy Signature (Sprint 12, S12-B.A.2/A.3) ---
  /// Master toggle. When false (default): per-level accumulators are computed
  /// (mirror rank-1 adds, cheap) but Schur signature is NOT computed and no
  /// diagnostics emitted. Preserves Sprint 5 R9-C2++ bit-identical.
  bool dg_a_enable = false;
  /// Ridge regularizer ε·I added to H_RR,ℓ before LDLT inverse in Schur.
  float dg_a_schur_eps = 1.0e-6f;
  /// When true, emit per-channel signature to diagnostics CSV via B5 boundary.
  bool dg_a_log_per_channel = false;

  // --- S13-B.A.1: P1 Anisotropic Hierarchical Information Filter ----------
  // Sensor-global toggles ONLY (sprint13_architecture I-3 — NOT per-seq).
  // All default-OFF; flag=false preserves S5 R9-C2++ + S12 V5 bit-identical.
  //
  // Mechanism: replace scalar w_i = robust_w/(0.001f + sigma2_i) at L1
  // (:869) and L2 (:949) with scalar projection of rank-3 anisotropic
  // information Ω_eff = nᵀ(Σ_{level,c_i} + R·Σ_{p,i}·Rᵀ + ε·I)⁻¹ n, plus
  // DG-A control gate γ = ρ_L2·(1 − β·cos²θ_12) on L2 channel.
  /// Master toggle. When false (default): legacy scalar path verbatim,
  /// rank-3 code paths are dead. NTU + Mid-360 configs MUST stay OFF.
  bool anisotropic_iekf_enable = false;
  /// Shim sub-flag (G2/G2b byte-compare). When true AND enable=true:
  /// Ω_eff forced to legacy form 1/(0.001f + sigma2_i_legacy) so byte-
  /// compare reproduces scalar path exactly. Architect §4 I-2.
  bool anisotropic_iekf_scalar_shim = false;
  /// ε ridge in (Σ + RΣ_pRᵀ + ε·I)⁻¹. G5 grid search tunes ∈ {1e-2, 1e-3,
  /// 1e-4, 1e-5}; shim mode forces 0.001f. Default 1.0e-3.
  float anisotropic_iekf_epsilon = 1.0e-3f;
  /// ρ_ref_avia for β=clip(1 − ρ_L2/ρ_ref, 0, 1). G5 bakes per Avia
  /// calibration; Mid-360/NTU leave at 0 (flag-OFF semantics). Static
  /// YAML constant (NOT runtime EMA) preserves I-3 determinism.
  double anisotropic_iekf_rho_ref_avia = 0.0;
  /// χ²₁ outlier-rejection threshold (95% default 3.841). G4 empirical
  /// refit may adjust ε; threshold itself stays at 3.841.
  float anisotropic_iekf_chi2_threshold = 3.841f;
  /// Bearing variance (rad²) for Σ_p,i = σ²_r·r̂r̂ᵀ + r²·σ²_θ·(I−r̂r̂ᵀ).
  /// Sensor-global; default (0.3°)² = 9e-6. σ²_r reuses existing
  /// lidar_noise_std² from base IekfConfig.
  float anisotropic_iekf_sigma_theta_sq = 9.0e-6f;

  /// S13-B.A.5 Path B: master gate routing P1 fields via scene
  /// classifier apply_template_() Phase C. When false (default),
  /// YAML P1 keys flow through unchanged (B.A.1 sensor-global behavior
  /// for shim/byte-compare debug). When true, per-class kT_*::p1
  /// overrides YAML at LOCK frame. Sensor-global; Mid-360 + NTU configs
  /// MUST leave default-false (I-4 + I-5 preserved).
  bool anisotropic_iekf_router_enable = false;
};

// ---------------------------------------------------------------------------
// IcdrState — Persistent state for ICDR temporal information persistence
// ---------------------------------------------------------------------------

struct IcdrState {
  /// TIP: EMA eigenvalue spectrum (6 values, ascending order).
  double lambda_ema[6] = {0, 0, 0, 0, 0, 0};
  /// TIP: cumulative pose displacement since last full-info frame [m].
  double cumulative_delta_pose = 0.0;
  /// Frame counter for EMA initialization.
  int frame_count = 0;
  bool initialized = false;
  /// Previous position for displacement tracking.
  Eigen::Vector3f prev_position = Eigen::Vector3f::Zero();
};

// ---------------------------------------------------------------------------
// IekfResult
// ---------------------------------------------------------------------------

struct IekfResult {
  LioState state;
  bool converged = false;
  int total_iterations = 0;
  int num_correspondences = 0;
  int num_degenerate_dirs = 0;    ///< Max degenerate directions across iters
  float res_mean = 0.0f;         ///< Mean residual (last iteration)
  float res_rms = 0.0f;          ///< RMS residual (last iteration)

  // --- IEKF correction norms (diagnostics) ---
  float rot_correction_norm = 0.0f;  ///< ||dx[0:3]|| at final iteration
  float pos_correction_norm = 0.0f;  ///< ||dx[3:6]|| at final iteration

  // --- Eigenvalue diagnostics (populated when IekfConfig::enable_debug_timing) ---
  float eigenvalues[6] = {0,0,0,0,0,0};  ///< HTRinvH eigenvalues (ascending)

  /// Eigenvalue ratio λ₀/λ₅ from the pre-gating map-direction eigendecomp.
  /// Always populated (no debug_timing dependency).  Used by Eq.(4) severity
  /// scaling: severity = 1 / (1 + ratio / ratio_ref).
  float eigenvalue_ratio = 0.0f;

  /// Degenerate translation directions (extracted from 6×6 eigenvectors).
  /// Only the position block (rows 3:6) of degenerate eigenvectors is stored,
  /// renormalized to unit length. Used for direction-selective map insertion.
  int num_degen_trans_dirs = 0;
  Eigen::Vector3f degen_trans_dirs[3] = {
      Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()
  };

  // --- DG-A diagnostics (populated when IekfConfig::dg_a_enable) ---
  /// Per-level Schur translation-block eigenvalue ratios ρ_ℓ = λ_min/λ_max.
  /// Index: 0=L1, 1=L2, 2=full (joint). Zero when channel is empty or disabled.
  float dg_a_rho[3] = {0.0f, 0.0f, 0.0f};
  /// Per-level weakest translation directions (3-vector, unit-norm).
  /// Index: 0=L1, 1=L2, 2=full.
  Eigen::Vector3f dg_a_d_trans[3] = {
      Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()
  };
  /// Cross-channel agreement cosines (|dot|, in [0,1]).
  /// 0=cos(d_L1,d_L2), 1=cos(d_L1,d_full), 2=cos(d_L2,d_full).
  float dg_a_cos_agree[3] = {0.0f, 0.0f, 0.0f};
  /// Per-level correspondence counts at IEKF terminal iteration.
  int dg_a_n_corr[3] = {0, 0, 0};

  // --- ICDR diagnostics (populated when IekfConfig::enable_icdr) ---
  float icdr_rho[6] = {0,0,0,0,0,0};     ///< Information ratios per direction
  float icdr_weights[6] = {0,0,0,0,0,0}; ///< Sigmoid weights per direction

  // --- Sub-module timing (populated when IekfConfig::enable_debug_timing) ---
  float jacobian_ms = 0.0f;      ///< Jacobian computation total
  float huber_pko_ms = 0.0f;     ///< Huber weight + PKO total
  float build_info_ms = 0.0f;    ///< Build R_inv + HTRinvH + degeneracy
  float solve_ms = 0.0f;         ///< info^{-1} + dx computation
};

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// Compute Huber weights for a vector of residuals.
///
///   weight_i = 1.0           if |r_i| <= delta
///   weight_i = delta / |r_i| if |r_i| >  delta
///
/// @param residuals  Input residual vector (any length).
/// @param delta      Huber threshold (> 0).
/// @return           Weight vector, same length as residuals.
Eigen::VectorXf compute_huber_weights(const Eigen::VectorXf& residuals,
                                      float delta);

/// Run one IEKF inner-loop update (fixed correspondences).
///
/// Performs at most config.max_inner_iters iterations of the
/// information-form Kalman update, starting from @p prior and
/// terminating early when ||dx|| < config.convergence_threshold.
///
/// The posterior covariance is set to
///   K1 = (H^T R^{-1} H + P_prior^{-1})^{-1}
/// which is both the inverse information matrix and the gain used for dx.
///
/// @param prior            Prior LioState (from IMU propagation / last outer
///                         iteration).
/// @param correspondences  Point-to-plane correspondences.
/// @param T_body_lidar     LiDAR → body/IMU extrinsic transform.
/// @param config           IEKF hyper-parameters.
/// @return                 Updated state + convergence diagnostics.
IekfResult iekf_update(const LioState& prior,
                       const std::vector<Correspondence>& correspondences,
                       const Se3& T_body_lidar,
                       const IekfConfig& config,
                       Pko* pko = nullptr,
                       IcdrState* icdr_state = nullptr);

}  // namespace core
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_ESTIMATOR_IEKF_UPDATER_HPP_
