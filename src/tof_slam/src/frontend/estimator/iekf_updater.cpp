// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// iekf_updater.cpp — Information-form Iterated Extended Kalman Filter update.
//
// Core features:
//   1A. Degeneracy-Aware IEKF: binary eigenvalue gating of H^T R^{-1} H (6×6)
//       to prevent measurement update in poorly-constrained directions.
//   1B. Adaptive Measurement Noise: per-correspondence σ² based on
//       range, incidence angle, and surfel planarity.
//   P floor: prevents covariance collapse that causes over-confidence.

#include "tof_slam/frontend/estimator/iekf_updater.hpp"
#include "tof_slam/frontend/estimator/anisotropic_iekf.hpp"
#include "tof_slam/frontend/estimator/correspondence.hpp"

#include <chrono>
#include <cmath>
#include <vector>

#include <omp.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <spdlog/spdlog.h>

#include "tof_slam/common/lie/se3.hpp"
#include "tof_slam/common/lie/so3.hpp"
#include "tof_slam/frontend/diag/boundary_hash.hpp"
#include "tof_slam/frontend/estimator/lidar_jacobian.hpp"
#include "tof_slam/frontend/robust/pko.hpp"

namespace tof_slam {
namespace core {

namespace {
// S12-B.A.2 DG-A: Schur translation-block helper.
//
// Given HTRinvH ∈ R^{6×6} (rotation/translation block ordering: top-left=R,
// bottom-right=t), compute the translation-only Schur complement
//   S_t = H_tt − H_tR · (H_RR + ε·I)^{-1} · H_Rt
// and extract:
//   - ρ_trans = λ_min(S_t) / λ_max(S_t) (translation conditioning)
//   - d_trans = unit eigenvector of S_t at λ_min (weakest direction)
//
// Determinism: LDLT with explicit ε·I ridge is deterministic. Matrix3d
// SelfAdjointEigenSolver is deterministic per Eigen documentation.
//
// Returns (rho_trans, d_trans). When H is effectively zero (all eigenvalues
// below 1e-12), returns (0, Zero) — caller should treat as "no info".
struct SchurTransResult {
  float rho_trans = 0.0f;
  Eigen::Vector3f d_trans = Eigen::Vector3f::Zero();
};

inline SchurTransResult compute_schur_translation(
    const Eigen::Matrix<double, 6, 6>& H, double eps) {
  SchurTransResult out;
  // Extract blocks. State order: [rotation(0:3), translation(3:6)].
  const Eigen::Matrix3d H_RR = H.topLeftCorner<3, 3>()
      + eps * Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d H_tt = H.bottomRightCorner<3, 3>();
  const Eigen::Matrix3d H_Rt = H.topRightCorner<3, 3>();
  // Closed-form 3×3 LDLT inverse (deterministic).
  const Eigen::LDLT<Eigen::Matrix3d> ldlt(H_RR);
  if (ldlt.info() != Eigen::Success) return out;
  const Eigen::Matrix3d H_RR_inv = ldlt.solve(Eigen::Matrix3d::Identity());
  const Eigen::Matrix3d S_t =
      H_tt - H_Rt.transpose() * H_RR_inv * H_Rt;
  // Symmetrize for SelfAdjointEigenSolver numerical robustness.
  const Eigen::Matrix3d S_sym = 0.5 * (S_t + S_t.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(S_sym);
  if (eig.info() != Eigen::Success) return out;
  const auto& vals = eig.eigenvalues();    // ascending
  const auto& vecs = eig.eigenvectors();
  const double lam_max = vals(2);
  const double lam_min = vals(0);
  if (lam_max < 1e-12) return out;
  out.rho_trans = static_cast<float>(lam_min / lam_max);
  out.d_trans = vecs.col(0).cast<float>();
  // Normalize (numerical safety; eigenvectors are unit but cast may drift).
  const float n = out.d_trans.norm();
  if (n > 1e-6f) out.d_trans /= n;
  return out;
}
}  // namespace

// ---------------------------------------------------------------------------
// compute_huber_weights
// ---------------------------------------------------------------------------

Eigen::VectorXf compute_huber_weights(const Eigen::VectorXf& residuals,
                                      float delta) {
  const int n = residuals.size();
  Eigen::VectorXf weights = Eigen::VectorXf::Ones(n);
  for (int i = 0; i < n; ++i) {
    const float abs_r = std::abs(residuals(i));
    if (abs_r > delta) {
      weights(i) = delta / abs_r;
    }
  }
  return weights;
}

// ---------------------------------------------------------------------------
// apply_covariance_floor
// ---------------------------------------------------------------------------

static void apply_covariance_floor(StateCovariance& P,
                                   const IekfConfig& config) {
  // Enforce minimum diagonal values to prevent covariance collapse.
  // This ensures the filter never becomes overconfident in any direction,
  // which is critical when degeneracy detection is active — degenerate
  // directions need sufficient P so that IMU propagation is trusted.
  auto clamp_block = [&P](int start, int size, float floor) {
    for (int i = start; i < start + size; ++i) {
      if (P(i, i) < floor) {
        P(i, i) = floor;
      }
    }
  };
  clamp_block(0, 3, config.p_floor_rot);    // rotation
  clamp_block(3, 3, config.p_floor_pos);    // position
  clamp_block(6, 3, config.p_floor_vel);    // velocity
  clamp_block(9, 3, config.p_floor_bias);   // gyro bias
  clamp_block(12, 3, config.p_floor_bias);  // acc bias
  clamp_block(15, 3, config.p_floor_grav);  // gravity
}

// Note: compute_adaptive_R_inv has been removed (Phase E refactor).
// Adaptive noise computation is now inlined in the IEKF fused loop,
// combined with surfel normal uncertainty.

// ---------------------------------------------------------------------------
// apply_degeneracy_remap (binary eigenvalue gating)
// ---------------------------------------------------------------------------
// Eigendecompose H^T R^{-1} H (6×6) and zero out eigenvalues below the
// threshold.  Degenerate directions are completely suppressed, making the
// filter trust IMU propagation in those directions.
//
// Returns the number of degenerate directions detected (0-6).

static int apply_degeneracy_remap(
    Eigen::Matrix<double, 6, 6>& HTRinvH_66,
    Eigen::Matrix<double, 6, 1>& HTRinvz_6,
    const IekfConfig& config,
    int frame_count,
    float* out_eigenvalues = nullptr,
    int* out_num_degen_trans_dirs = nullptr,
    Eigen::Vector3f* out_degen_trans_dirs = nullptr) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig(HTRinvH_66);
  const auto& eigenvalues = eig.eigenvalues();
  const auto& V = eig.eigenvectors();

  int n_degenerate = 0;

  // Project HTRinvz to eigen-basis for consistent gating.
  Eigen::Matrix<double, 6, 1> z_eigen = V.transpose() * HTRinvz_6;

  const double threshold = static_cast<double>(config.degeneracy_threshold);
  const double ratio_threshold = static_cast<double>(config.degeneracy_ratio_threshold);
  const double soft_floor = static_cast<double>(config.degeneracy_soft_floor);
  const double lambda_max = eigenvalues(5);  // Largest eigenvalue (sorted ascending).

  // Hybrid gating: combines absolute threshold AND relative ratio.
  // A direction is degenerate if:
  //   (1) λ < absolute threshold (for very sparse correspondence sets), OR
  //   (2) λ/λ_max < ratio_threshold (for dense sets where absolute is useless)
  //
  // Soft gating: instead of binary zero, scale the contribution:
  //   scale = soft_floor + (1-soft_floor) * min(1, ratio_i / ratio_threshold)
  Eigen::Matrix<double, 6, 1> remapped_eig;
  for (int i = 0; i < 6; ++i) {
    const double lam = eigenvalues(i);
    const double ratio_i = (lambda_max > 1e-6) ? (lam / lambda_max) : 0.0;

    const bool abs_degen = (lam <= threshold);
    const bool ratio_degen = (ratio_threshold > 0.0 && ratio_i < ratio_threshold);

    if (abs_degen || ratio_degen) {
      ++n_degenerate;
      if (soft_floor > 0.0 && ratio_threshold > 0.0) {
        // Soft gating: proportional scaling.
        const double scale = soft_floor +
            (1.0 - soft_floor) * std::min(1.0, ratio_i / ratio_threshold);
        remapped_eig(i) = lam * scale;
        z_eigen(i) *= scale;
      } else {
        // Hard gating: zero out completely.
        remapped_eig(i) = 0.0;
        z_eigen(i) = 0.0;
      }
    } else {
      remapped_eig(i) = lam;
    }
  }

  // Extract degenerate translation directions for direction-selective map insertion.
  if (out_num_degen_trans_dirs && out_degen_trans_dirs) {
    int n_dirs = 0;
    for (int i = 0; i < 6 && n_dirs < 3; ++i) {
      const double lam = eigenvalues(i);
      const double ratio_i = (lambda_max > 1e-6) ? (lam / lambda_max) : 0.0;
      const bool abs_degen = (lam <= threshold);
      const bool ratio_degen = (ratio_threshold > 0.0 && ratio_i < ratio_threshold);

      if (abs_degen || ratio_degen) {
        // Extract position block (rows 3:6) of the 6D eigenvector.
        Eigen::Vector3f pos_block = V.col(i).tail<3>().cast<float>();
        const float pos_norm = pos_block.norm();
        if (pos_norm > 0.1f) {  // Only store if translation component is significant
          out_degen_trans_dirs[n_dirs] = pos_block / pos_norm;
          ++n_dirs;
        }
      }
    }
    *out_num_degen_trans_dirs = n_dirs;
  }

  // Reconstruct in original basis.
  HTRinvH_66 = V * remapped_eig.asDiagonal() * V.transpose();
  HTRinvz_6 = V * z_eigen;

  // Log degeneracy events.
  if (n_degenerate > 0) {
    static int degen_log_count = 0;
    if (degen_log_count < 20 || frame_count % 100 == 0) {
      const double ratio = (lambda_max > 1e-6)
                              ? eigenvalues(0) / lambda_max : 0.0;
      SPDLOG_WARN("DEGENERACY: {}/6 dirs (eigenvalues: {:.1f} {:.1f} {:.1f} "
                  "{:.1f} {:.1f} {:.1f}, ratio={:.6f} tau={:.1f} "
                  "ratio_tau={:.4f} soft={:.2f})",
                  n_degenerate,
                  eigenvalues(0), eigenvalues(1), eigenvalues(2),
                  eigenvalues(3), eigenvalues(4), eigenvalues(5),
                  ratio, threshold, ratio_threshold, soft_floor);
      ++degen_log_count;
    }
  }

  // Periodic eigenvalue logging (every frame where frame_count%100==0,
  // or always when frame_count==0 which is the case in current call site).
  if (frame_count % 100 == 0) {
    const double ratio = (lambda_max > 1e-6)
                            ? eigenvalues(0) / lambda_max : 0.0;
    SPDLOG_INFO("EIGENVALS frame={}: {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} "
                "ratio={:.6f} tau={:.1f} ratio_tau={:.4f} n_degen={}",
                frame_count,
                eigenvalues(0), eigenvalues(1), eigenvalues(2),
                eigenvalues(3), eigenvalues(4), eigenvalues(5),
                ratio, threshold, ratio_threshold, n_degenerate);
  }

  // Output raw eigenvalues (before gating) for diagnostics.
  if (out_eigenvalues) {
    for (int i = 0; i < 6; ++i) out_eigenvalues[i] = static_cast<float>(eigenvalues(i));
  }

  return n_degenerate;
}

// ---------------------------------------------------------------------------
// apply_icdr_remap — Information-theoretic Continuous Degeneracy Regularization
// ---------------------------------------------------------------------------
// Novel degeneracy method with three components:
//   1. Information Ratio (IR): ρ_i = λ_lidar_i / (λ_lidar_i + λ_prior_i)
//      Measures LiDAR information RELATIVE to IMU prior per direction.
//   2. Continuous Sigmoid Transition (CST): w_i = sigmoid((ρ - ρ_thresh) / τ)
//      Smooth weighting avoiding Jacobian discontinuities.
//   3. Temporal Information Persistence (TIP): EMA of eigenvalues with
//      pose-displacement-based decay for geometric memory.
//
// Unlike Zhang & Singh (2016) binary gating, this:
//   - Adapts automatically to sensor/environment (no absolute threshold)
//   - Provides smooth transitions (no Jacobian jumps across IEKF iterations)
//   - Remembers recent geometric structure during transient degeneracy
//
// Returns the number of degenerate directions (ρ < ρ_thresh).

static int apply_icdr_remap(
    Eigen::Matrix<double, 6, 6>& HTRinvH_66,
    Eigen::Matrix<double, 6, 1>& HTRinvz_6,
    const Eigen::Matrix<double, 6, 6>& P_pose_inv,
    const IekfConfig& config,
    IcdrState* icdr_state,
    const Eigen::Vector3f& current_position,
    int frame_count,
    float* out_eigenvalues = nullptr,
    int* out_num_degen_trans_dirs = nullptr,
    Eigen::Vector3f* out_degen_trans_dirs = nullptr,
    float* out_icdr_rho = nullptr,
    float* out_icdr_weights = nullptr) {

  // Step 1: Eigendecompose H^T R^{-1} H (6×6).
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig(HTRinvH_66);
  const auto& eigenvalues = eig.eigenvalues();
  const auto& V = eig.eigenvectors();

  // Project HTRinvz to eigen-basis for consistent gating.
  Eigen::Matrix<double, 6, 1> z_eigen = V.transpose() * HTRinvz_6;

  // Step 2: Compute Information Ratio (IR) per direction.
  // ρ_i = λ_lidar_i / (λ_lidar_i + v_i^T P_inv v_i)
  // This is the Kalman gain fraction: ρ≈1 → LiDAR dominates, ρ≈0 → prior dominates.
  double rho[6];
  double lambda_effective[6];
  for (int i = 0; i < 6; ++i) {
    lambda_effective[i] = eigenvalues(i);
  }

  // Step 2a: Temporal Information Persistence (TIP).
  // Maintain EMA of eigenvalues; when current drops below EMA,
  // use decayed historical value as information floor.
  //
  // IMPORTANT: TIP state (EMA, displacement) is updated ONLY on the first
  // inner IEKF iteration (frame_count == 1, i.e. total_iters==1 at call site).
  // Subsequent inner iterations within the same frame reuse the EMA values
  // but do NOT re-accumulate displacement or re-initialize.  The actual
  // prev_position/frame_count update happens once per LiDAR frame in
  // LioEstimator, not here.
  if (config.enable_icdr_tip && icdr_state != nullptr) {
    const bool is_first_inner_iter = (frame_count <= 1);

    if (!icdr_state->initialized) {
      // First-ever frame: initialize EMA with current eigenvalues.
      for (int i = 0; i < 6; ++i) {
        icdr_state->lambda_ema[i] = eigenvalues(i);
      }
      icdr_state->initialized = true;
      icdr_state->cumulative_delta_pose = 0.0;
      icdr_state->prev_position = current_position;  // seed for next frame delta
    } else if (is_first_inner_iter) {
      // First inner iteration of a new frame: update EMA + displacement.
      const float delta_pos =
          (current_position - icdr_state->prev_position).norm();
      icdr_state->cumulative_delta_pose += static_cast<double>(delta_pos);

      const double alpha = static_cast<double>(config.icdr_tip_alpha);
      const double beta = static_cast<double>(config.icdr_tip_beta);
      const double d_decay = static_cast<double>(config.icdr_tip_d_decay);

      for (int i = 0; i < 6; ++i) {
        // Update EMA.
        icdr_state->lambda_ema[i] =
            alpha * eigenvalues(i) + (1.0 - alpha) * icdr_state->lambda_ema[i];

        // If current eigenvalue dropped below EMA, inject persistent info.
        // Decay with pose displacement (not time), so it works when robot stops.
        if (eigenvalues(i) < icdr_state->lambda_ema[i]) {
          const double decay = std::exp(
              -icdr_state->cumulative_delta_pose / std::max(d_decay, 0.01));
          const double lambda_floor =
              beta * icdr_state->lambda_ema[i] * decay;
          lambda_effective[i] = std::max(eigenvalues(i), lambda_floor);
        }
      }

      // Reset displacement accumulator when all directions are non-degenerate.
      // (Checked after IR computation below.)
    } else {
      // Later inner iterations: apply TIP floor using cached EMA, no EMA update.
      const double beta = static_cast<double>(config.icdr_tip_beta);
      const double d_decay = static_cast<double>(config.icdr_tip_d_decay);

      for (int i = 0; i < 6; ++i) {
        if (eigenvalues(i) < icdr_state->lambda_ema[i]) {
          const double decay = std::exp(
              -icdr_state->cumulative_delta_pose / std::max(d_decay, 0.01));
          const double lambda_floor =
              beta * icdr_state->lambda_ema[i] * decay;
          lambda_effective[i] = std::max(eigenvalues(i), lambda_floor);
        }
      }
    }
  }

  // Step 2b: Compute ρ using lambda_effective and prior precision.
  int n_degenerate = 0;
  for (int i = 0; i < 6; ++i) {
    // Prior information in this eigendirection.
    const Eigen::Matrix<double, 6, 1>& vi = V.col(i);
    const double lambda_prior = vi.transpose() * P_pose_inv * vi;

    // Information ratio.
    const double lambda_total = lambda_effective[i] + lambda_prior;
    rho[i] = (lambda_total > 1e-12)
        ? (lambda_effective[i] / lambda_total) : 0.0;

    if (rho[i] < static_cast<double>(config.icdr_rho_thresh)) {
      ++n_degenerate;
    }
  }

  // Step 3: Continuous Sigmoid Transition (CST).
  // w_i = sigmoid((ρ_i - ρ_thresh) / τ), clamped to [w_min, 1].
  const double rho_thresh = static_cast<double>(config.icdr_rho_thresh);
  const double tau = static_cast<double>(config.icdr_tau);
  const double w_min = static_cast<double>(config.icdr_w_min);

  Eigen::Matrix<double, 6, 1> remapped_eig;
  for (int i = 0; i < 6; ++i) {
    const double exponent = -(rho[i] - rho_thresh) / std::max(tau, 1e-6);
    double w = 1.0 / (1.0 + std::exp(exponent));
    w = std::max(w, w_min);

    remapped_eig(i) = w * eigenvalues(i);  // Use raw eigenvalue with sigmoid weight
    z_eigen(i) *= w;

    if (out_icdr_rho) out_icdr_rho[i] = static_cast<float>(rho[i]);
    if (out_icdr_weights) out_icdr_weights[i] = static_cast<float>(w);
  }

  // Reset TIP displacement accumulator when all directions non-degenerate.
  if (config.enable_icdr_tip && icdr_state != nullptr && n_degenerate == 0) {
    icdr_state->cumulative_delta_pose = 0.0;
  }

  // Step 4: Extract degenerate translation directions for DDPO/DARBF.
  // Uses ρ < ρ_thresh instead of eigenvalue threshold.
  if (out_num_degen_trans_dirs && out_degen_trans_dirs) {
    int n_dirs = 0;
    for (int i = 0; i < 6 && n_dirs < 3; ++i) {
      if (rho[i] < rho_thresh) {
        Eigen::Vector3f pos_block = V.col(i).tail<3>().cast<float>();
        const float pos_norm = pos_block.norm();
        if (pos_norm > 0.1f) {
          out_degen_trans_dirs[n_dirs] = pos_block / pos_norm;
          ++n_dirs;
        }
      }
    }
    *out_num_degen_trans_dirs = n_dirs;
  }

  // Step 5: Reconstruct in original basis.
  HTRinvH_66 = V * remapped_eig.asDiagonal() * V.transpose();
  HTRinvz_6 = V * z_eigen;

  // Log ICDR events.
  if (n_degenerate > 0) {
    static int icdr_log_count = 0;
    if (icdr_log_count < 20 || frame_count % 100 == 0) {
      SPDLOG_WARN("ICDR: {}/6 degenerate (ρ: {:.4f} {:.4f} {:.4f} {:.4f} "
                  "{:.4f} {:.4f}, w: {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f})",
                  n_degenerate,
                  rho[0], rho[1], rho[2], rho[3], rho[4], rho[5],
                  remapped_eig(0) / std::max(eigenvalues(0), 1e-12),
                  remapped_eig(1) / std::max(eigenvalues(1), 1e-12),
                  remapped_eig(2) / std::max(eigenvalues(2), 1e-12),
                  remapped_eig(3) / std::max(eigenvalues(3), 1e-12),
                  remapped_eig(4) / std::max(eigenvalues(4), 1e-12),
                  remapped_eig(5) / std::max(eigenvalues(5), 1e-12));
      ++icdr_log_count;
    }
  }

  // Periodic eigenvalue logging.
  if (frame_count % 100 == 0) {
    SPDLOG_INFO("ICDR frame={}: λ=[{:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f}] "
                "ρ=[{:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f}] n_degen={}",
                frame_count,
                eigenvalues(0), eigenvalues(1), eigenvalues(2),
                eigenvalues(3), eigenvalues(4), eigenvalues(5),
                rho[0], rho[1], rho[2], rho[3], rho[4], rho[5],
                n_degenerate);
  }

  // Output raw eigenvalues (before gating) for diagnostics.
  if (out_eigenvalues) {
    for (int i = 0; i < 6; ++i) out_eigenvalues[i] = static_cast<float>(eigenvalues(i));
  }

  return n_degenerate;
}

// ---------------------------------------------------------------------------
// iekf_update
// ---------------------------------------------------------------------------

IekfResult iekf_update(const LioState& prior,
                       const std::vector<Correspondence>& correspondences,
                       const Se3& T_body_lidar,
                       const IekfConfig& config,
                       Pko* pko,
                       IcdrState* icdr_state) {
  // Early-out: no correspondences — nothing to update.
  if (correspondences.empty()) {
    IekfResult result;
    result.state = prior;
    result.converged = false;
    result.total_iterations = 0;
    result.num_correspondences = 0;
    return result;
  }

  // Working state — updated iteratively.
  LioState state = prior;

  bool converged = false;
  int total_iters = 0;

  // Covariance handling: P_prior is the IMU-propagated covariance.
  //
  // FAST-LIO2 resets P_ = P_propagated at each iteration and applies
  // A-matrix (SO(3) left Jacobian) correction to account for manifold
  // curvature.  We do the same: reset P_iter = P_prior, apply A^T to
  // the rotation block, then compute P_inv from the corrected P.
  //
  // CRITICAL: Use float64 for covariance inversions to prevent numerical
  // drift on long sequences.  The 18×18 P matrix has diagonal elements
  // spanning 10+ orders of magnitude (rotation ~1e-6, bias ~1e-8, vel ~1e-1),
  // giving condition numbers >1e8.
  using CovD = StateCovariance;  // already double
  const StateCovariance& P_prior = prior.covariance;

  // Pre-compute p_imu = R_il * p_lidar + t_il for all correspondences.
  // This is extrinsic-only and state-independent, so it's constant across
  // all inner iterations.
  const Eigen::Matrix3f R_il = T_body_lidar.rotation().matrix();
  const Eigen::Vector3f t_il = T_body_lidar.translation();
  const int n_corrs = static_cast<int>(correspondences.size());

  // Residual vector (reused each iteration). No H matrix — Jacobian rows
  // are computed on-the-fly in the fused HTRinvH accumulation loop.
  Eigen::VectorXf residuals(n_corrs);
  std::vector<Eigen::Vector3f> p_imu_cache(n_corrs);
  for (int i = 0; i < n_corrs; ++i) {
    p_imu_cache[i] = R_il * correspondences[i].p_lidar + t_il;
  }

  // G matrix from last inner iteration (for covariance update after loop).
  StateCovariance G_final = StateCovariance::Zero();

  // Track degeneracy for reporting.
  int max_degenerate_dirs = 0;

  // Eigenvalue diagnostics — stores values from the LAST iteration.
  float frame_eigenvalues[6] = {0,0,0,0,0,0};

  // ICDR diagnostics — stores values from the LAST iteration.
  float frame_icdr_rho[6] = {0,0,0,0,0,0};
  float frame_icdr_weights[6] = {0,0,0,0,0,0};

  // Degenerate translation directions (from last iteration with degeneracy).
  float frame_eigenvalue_ratio = 0.0f;
  int frame_num_degen_trans_dirs = 0;
  Eigen::Vector3f frame_degen_trans_dirs[3] = {
      Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()
  };

  // S12-B.A.2 DG-A: per-level Schur signature (last iteration).
  // Index: 0=L1, 1=L2, 2=full(joint). Populated only when config.dg_a_enable.
  float frame_dg_a_rho[3] = {0.0f, 0.0f, 0.0f};
  Eigen::Vector3f frame_dg_a_d_trans[3] = {
      Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()
  };
  float frame_dg_a_cos_agree[3] = {0.0f, 0.0f, 0.0f};
  int frame_dg_a_n_corr[3] = {0, 0, 0};

  // Debug timing accumulators.
  using Clock = std::chrono::high_resolution_clock;
  const bool do_timing = config.enable_debug_timing;
  float acc_jacobian_ms = 0.0f;
  float acc_huber_pko_ms = 0.0f;
  float acc_build_info_ms = 0.0f;
  float acc_solve_ms = 0.0f;

  // Last-iteration residual statistics.
  float last_res_mean = 0.0f;
  float last_res_rms = 0.0f;

  const int n = static_cast<int>(correspondences.size());

  // Pre-compute base noise variance.
  const float sigma2_base = config.lidar_noise_std * config.lidar_noise_std;
  const float r_ref_sq = config.adaptive_range_ref * config.adaptive_range_ref;

  // PKO Huber: compute scale and delta once from initial residuals.
  float pko_normalization_scale = 1.0f;
  float pko_delta_cached = 1.0f;
  bool pko_scale_initialized = false;

  // Track last dx norms for diagnostics.
  float last_rot_dx_norm = 0.0f;
  float last_pos_dx_norm = 0.0f;

  // S13-B.B.1 DG-A γ control gate state (per architect §3.2):
  //   γ = ρ_L2 · (1 − β · cos²θ_12), β = clip(1 − ρ_L2/ρ_ref_avia, 0, 1)
  // γ_cached holds the PRIOR iteration's γ so the current iteration's L2
  // weight site reads a value not affected by its own update (avoids per-
  // iter feedback loop violating I-3 determinism). First iter: γ_cached = 1.0.
  // γ is computed AFTER the DG-A signature block at iter end and stored
  // for next iter consumption. When anisotropic_iekf_enable=false OR
  // dg_a_enable=false OR ρ_ref_avia ≤ 0, γ stays 1.0 (no modulation).
  double gamma_cached = 1.0;

  for (int iter = 0; iter < config.max_inner_iters; ++iter) {
    ++total_iters;

    const auto t_jac_start = do_timing ? Clock::now() : Clock::time_point{};

    // ------------------------------------------------------------------
    // 0. A-matrix manifold correction (FAST-LIO2 style).
    //
    // Reset P to P_propagated each iteration, then apply the SO(3) left
    // Jacobian A^T to the rotation block rows/columns of P.  This accounts
    // for the curvature of SO(3) when the current iterate differs from the
    // propagated state.
    //
    // phi = Log(R_prior^T * R_current)  (rotation deviation from prior)
    // A = J_l(phi)  (SO(3) left Jacobian)
    // P_corrected[rot_rows, :] = A^T * P_prior[rot_rows, :]
    // P_corrected[:, rot_cols] = P_prior[:, rot_cols] * A
    //
    // On iteration 0: phi = 0, A = I, no correction (state == prior).
    // ------------------------------------------------------------------
    CovD P_iter = P_prior;  // reset to propagated each iteration
    {
      // Compute rotation deviation from prior.
      const Eigen::Matrix3f R_diff = prior.rotation.transpose() * state.rotation;
      const Eigen::Vector3f phi = So3(R_diff).Log();

      if (config.enable_a_matrix_correction && phi.norm() > kEpsilon) {
        const Eigen::Matrix3f A = LeftJacobian(phi);
        const Eigen::Matrix3d At = A.transpose().cast<double>();
        const Eigen::Matrix3d Ad = A.cast<double>();

        // Apply A^T to rotation rows (0:3) of P
        P_iter.topRows<3>() = At * P_prior.topRows<3>();
        // Apply A to rotation columns (0:3) of P
        P_iter.leftCols<3>() = P_iter.leftCols<3>() * Ad;
      }
    }
    const CovD P_inv_d = P_iter.inverse();

    if (do_timing) {
      auto dt = std::chrono::duration<float, std::milli>(Clock::now() - t_jac_start).count();
      acc_huber_pko_ms += dt;
    }

    // ------------------------------------------------------------------
    // 2. FULLY FUSED single-pass: residual + weight + Jacobian + accumulate.
    //
    // Proposal 3 (paper analysis): Fuse ALL per-correspondence operations
    // into a single pass through the correspondence array.  Each iteration:
    //   1. Compute residual r_i (transform + dot product)
    //   2. Compute Huber weight from cached delta (1 branch)
    //   3. Compute adaptive σ² (if enabled) and combined weight W_i
    //   4. Compute Jacobian row h_i (cross product)
    //   5. Accumulate HTRinvH += W_i * h_i * h_i^T (6×6 rank-1)
    //   6. Accumulate HTRinvz += W_i * h_i * r_i   (6×1)
    //
    // Memory traffic: ~1KB (6×6 + 6×1 accumulators in registers) vs
    //                 ~160KB/iter (old 5-pass approach with Nx6 H matrix).
    //
    // The correspondence array is traversed ONCE per inner iteration.
    // ------------------------------------------------------------------

    const auto t_build_start = do_timing ? Clock::now() : Clock::time_point{};

    const Eigen::Matrix3f R_wb = state.rotation;
    const Eigen::Vector3f t_wb = state.position;
    const Eigen::Matrix3f R_wb_t = R_wb.transpose();

    // PKO initialization: compute scale and delta once at iter=0.
    {
      const bool do_pko_init = (pko != nullptr) && !pko_scale_initialized;
      if (do_pko_init) {
        Eigen::VectorXf init_residuals(n);
        for (int i = 0; i < n; ++i) {
          const Eigen::Vector3f p_w = R_wb * p_imu_cache[i] + t_wb;
          init_residuals(i) = -(correspondences[i].normal.dot(p_w) - correspondences[i].plane_d);
        }
        const float res_mean = init_residuals.mean();
        const float res_var = (init_residuals.array() - res_mean).square().mean();
        pko_normalization_scale = std::max(std::sqrt(res_var) / 3.0f, 1e-6f);

        Eigen::VectorXf norm_res = init_residuals / pko_normalization_scale;
        std::vector<double> res_d(n);
        for (int i = 0; i < n; ++i) res_d[i] = static_cast<double>(norm_res(i));
        pko_delta_cached = static_cast<float>(pko->compute_scale_factor(res_d));
        pko_scale_initialized = true;
      }
    }
    const float huber_delta = (pko != nullptr) ? pko_delta_cached : 1e30f;

    // Float64 accumulation — every reference LIO (FAST-LIO2, iG-LIO, Point-LIO)
    // uses double for H^T R^{-1} H.  With N~500 rank-1 updates of magnitude ~400,
    // float32 accumulated diagonal (~2e5) has only 4 significant digits, losing
    // precision in weakly-constrained directions (condition number ~350).
    Eigen::Matrix<double, 6, 6> HTRinvH_66 = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> HTRinvz_6 = Eigen::Matrix<double, 6, 1>::Zero();
    float sum_res = 0.0f;
    float sum_res_sq = 0.0f;
    // S12-B.A.1 DG-A: per-level HTRinvH accumulators (mirror rank-1 adds).
    // L1 includes both P2D path (line 717) and P2P L1 path (line 801).
    // L2 includes P2P L2 path (line 863) when corr.has_l2.
    // Invariant I-2: HTRinvH_66 ≡ HTRinvH_L1 + HTRinvH_L2 (bit-identical).
    Eigen::Matrix<double, 6, 6> HTRinvH_L1 = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> HTRinvH_L2 = Eigen::Matrix<double, 6, 6>::Zero();

    // OpenMP parallel reduction over correspondences.
    // Each thread accumulates into a pre-allocated slot; sequential reduction
    // after the parallel region ensures deterministic floating-point ordering.
    // Previous #pragma omp critical merge was non-deterministic: thread entry
    // order varies with CSCF's 27× cache-contention-heavy gather, causing
    // FP non-associativity → bit-level divergence across runs (CV=12.6%).
    const int iekf_omp_threads = std::min(
        omp_get_max_threads(),
        (config.iekf_omp_threads > 0) ? config.iekf_omp_threads : omp_get_max_threads());

    // Per-thread accumulators — indexed by omp_get_thread_num().
    std::vector<Eigen::Matrix<double, 6, 6>> thr_H(iekf_omp_threads, Eigen::Matrix<double, 6, 6>::Zero());
    std::vector<Eigen::Matrix<double, 6, 1>> thr_z(iekf_omp_threads, Eigen::Matrix<double, 6, 1>::Zero());
    std::vector<float> thr_res(iekf_omp_threads, 0.0f);
    std::vector<float> thr_res2(iekf_omp_threads, 0.0f);
    // S12-B.A.1 DG-A: per-thread per-level accumulators.
    // Mirror the existing thr_H pattern; reduction loop below extends in same order.
    std::vector<Eigen::Matrix<double, 6, 6>> thr_H_L1(iekf_omp_threads, Eigen::Matrix<double, 6, 6>::Zero());
    std::vector<Eigen::Matrix<double, 6, 6>> thr_H_L2(iekf_omp_threads, Eigen::Matrix<double, 6, 6>::Zero());
    // S12-B.A.2 DG-A: per-thread correspondence counts (L1/L2/full).
    std::vector<int> thr_n_L1(iekf_omp_threads, 0);
    std::vector<int> thr_n_L2(iekf_omp_threads, 0);
    std::vector<int> thr_n_full(iekf_omp_threads, 0);

    #pragma omp parallel num_threads(iekf_omp_threads) if (n >= 64)
    {
    const int tid = omp_get_thread_num();
    auto& local_HTRinvH = thr_H[tid];
    auto& local_HTRinvz = thr_z[tid];
    auto& local_sum_res = thr_res[tid];
    auto& local_sum_res_sq = thr_res2[tid];
    // S12-B.A.1 DG-A: per-level local aliases.
    auto& local_HTRinvH_L1 = thr_H_L1[tid];
    auto& local_HTRinvH_L2 = thr_H_L2[tid];
    // S12-B.A.2 DG-A: per-thread correspondence counters.
    int& local_n_L1 = thr_n_L1[tid];
    int& local_n_L2 = thr_n_L2[tid];
    int& local_n_full = thr_n_full[tid];

    #pragma omp for schedule(static) nowait
    for (int i = 0; i < n; ++i) {
      const Eigen::Vector3f& p_imu  = p_imu_cache[i];
      const auto& corr = correspondences[i];

      // 1. Transform point to world frame.
      const Eigen::Vector3f p_world = R_wb * p_imu + t_wb;

      // ================================================================
      // P2D: Point-to-Distribution (Voxel-Gaussian) residual — 3×1
      // ================================================================
      if (corr.residual_mode == ResidualMode::kPointToDistribution) {
        const Eigen::Vector3f r3 = p_world - corr.centroid;  // 3×1

        // Diagnostics: store normal-projected residual for PKO-compatible scale.
        // r3.dot(normal) is the P2P-equivalent residual (meters, normal direction).
        const float r_proj = std::abs(r3.dot(corr.normal));
        residuals(i) = r_proj;
        local_sum_res += r_proj;
        local_sum_res_sq += r_proj * r_proj;

        // Chi² Mahalanobis outlier gate (3-DOF, configurable threshold).
        // Replaces PKO/Huber which is calibrated for P2P meter-scale residuals
        // and cannot share the same MAD normalization with P2D.
        // iG-LIO uses 7.815 (p=0.05); default 11.345 (p=0.99).
        const float maha_sq = r3.transpose() * corr.voxel_cov_inv * r3;
        if (maha_sq > config.p2d_chi2_threshold) {
          continue;  // Hard reject: skip this P2D correspondence
        }

        // No Huber/PKO for P2D — Omega already encodes per-correspondence trust.
        const float robust_w = 1.0f;

        // 3×6 Jacobian: H_i = [-R_wb * [p_imu]× | I₃]
        // dh/d(δrot) = -R_wb * skew(p_imu)
        // dh/d(δpos) = I₃
        const Eigen::Vector3d p_imu_d = p_imu.cast<double>();
        const Eigen::Matrix3d R_wb_d = R_wb.cast<double>();
        Eigen::Matrix3d skew;
        skew << 0.0, -p_imu_d(2), p_imu_d(1),
                p_imu_d(2), 0.0, -p_imu_d(0),
                -p_imu_d(1), p_imu_d(0), 0.0;

        Eigen::Matrix<double, 3, 6> H_i;
        H_i.leftCols<3>() = -R_wb_d * skew;
        H_i.rightCols<3>() = Eigen::Matrix3d::Identity();

        // Adaptive noise scaling for P2D: apply incidence/planarity penalties
        // to reduce trust for grazing-angle / non-planar correspondences,
        // matching the production P2P adaptive noise behavior.
        float p2d_adaptive_scale = 1.0f;
        if (config.enable_p2d_adaptive_noise && config.enable_adaptive_noise) {
          const float one_minus_cos = 1.0f - corr.cos_incidence;
          const float incidence_pen =
              config.adaptive_incidence_scale * one_minus_cos * one_minus_cos;
          const float planarity_pen =
              config.adaptive_planarity_scale * corr.planarity * corr.planarity;
          p2d_adaptive_scale = 1.0f / (1.0f + incidence_pen + planarity_pen);
        }

        // Weighted information: H_i^T * (robust_w * adaptive * scale * Ω) * H_i
        // p2d_omega_scale accounts for centroid uncertainty not captured in the
        // geometric (shape-only) covariance. <1.0 reduces P2D influence.
        const Eigen::Matrix3d Omega_d =
            static_cast<double>(robust_w * p2d_adaptive_scale *
                                config.p2d_omega_scale) *
            corr.voxel_cov_inv.cast<double>();
        const Eigen::Matrix<double, 6, 3> HtO = H_i.transpose() * Omega_d;

        local_HTRinvH.noalias() += HtO * H_i;
        local_HTRinvz.noalias()  += HtO * r3.cast<double>();
        // S12-B.A.1 DG-A: P2D path classified as L1 (PVMap voxel-Gaussian).
        local_HTRinvH_L1.noalias() += HtO * H_i;
        ++local_n_L1;
        ++local_n_full;

        continue;  // Skip P2P path below
      }

      // ================================================================
      // P2P: Point-to-Plane residual — 1×1 (standard path)
      // ================================================================

      // Point-to-plane residual (always computed for diagnostics/PKO).
      const float r_plane = -(corr.normal.dot(p_world) - corr.plane_d);

      // 2. Compute per-correspondence adaptive noise sigma2_i.
      //    MUST precede the chi2 gate so that the gate uses per-correspondence
      //    adaptive sigma rather than the fixed sigma2_base floor.
      //    Normal uncertainty (sigma2_normal) is config-gated (harmful for sparse 16ch).
      //    Adaptive penalties (range/incidence/planarity) are config-gated.
      const Eigen::Vector3f dp = p_world - corr.centroid;
      const float sigma2_normal = config.enable_sigma2_normal
          ? dp.squaredNorm() * corr.normal_sigma2
          : 0.0f;
      float sigma2_i;
      if (config.enable_adaptive_noise) {
        // Range penalty: skip additive term if range_inverse_weight handles range.
        const float range_penalty = config.enable_range_inverse_weight
            ? 0.0f
            : config.adaptive_range_scale * (corr.range * corr.range) / r_ref_sq;
        const float one_minus_cos = 1.0f - corr.cos_incidence;
        const float incidence_penalty =
            config.adaptive_incidence_scale * one_minus_cos * one_minus_cos;
        const float planarity_penalty =
            config.adaptive_planarity_scale * corr.planarity * corr.planarity;
        sigma2_i =
            sigma2_base * (1.0f + range_penalty + incidence_penalty +
                           planarity_penalty) + sigma2_normal;
        // Range-inverse scaling: close-range → lower sigma2, far-range → higher.
        if (config.enable_range_inverse_weight) {
          const float ratio = std::max(
              config.range_inverse_min_ratio,
              corr.range / config.range_inverse_ref);
          sigma2_i *= std::pow(ratio, config.range_inverse_power);
        }
      } else if (config.enable_range_inverse_weight) {
        const float ratio = std::max(
            config.range_inverse_min_ratio,
            corr.range / config.range_inverse_ref);
        sigma2_i =
            (sigma2_base + sigma2_normal) *
            std::pow(ratio, config.range_inverse_power);
      } else {
        sigma2_i = sigma2_base + sigma2_normal;
      }

      residuals(i) = r_plane;
      local_sum_res += r_plane;
      local_sum_res_sq += r_plane * r_plane;

      // Robust weight: Huber/PKO weighting.
      float robust_w = 1.0f;
      if (pko != nullptr) {
        const float abs_r_norm = std::abs(r_plane / pko_normalization_scale);
        robust_w = (abs_r_norm <= huber_delta) ? 1.0f : (huber_delta / abs_r_norm);
      }

      // ----------------------------------------------------------------
      // S13-B.A.3 P1: Per-correspondence L1 weight.
      //
      // Two paths:
      //   (a) anisotropic_iekf_enable=false  → legacy scalar form verbatim
      //                                        (S5 R9-C2++ + S12 V5 bit-id).
      //   (b) anisotropic_iekf_enable=true   → P1 active:
      //         · scalar_shim=true     → byte-identical legacy fp32 expr
      //           (G2 verifies new-flag plumbing does not perturb math).
      //         · has_surfel_cov_L1    → full Ω_eff = nᵀ(Σ_L1+RΣ_pRᵀ+εI)⁻¹n.
      //         · neither              → fall back to scalar with cfg ε
      //           (path identifiable, NOT a hidden legacy leak).
      //
      // CRITICAL (S13-B.A.3 lesson): the shim path MUST reuse the exact
      // legacy fp32 expression `robust_w / (0.001f + sigma2_i)`. Even
      // mathematically-equivalent fp64 reciprocal + cast yields 1-2 ulp
      // per-w_i drift that the LIO accumulator + IEKF inner loop turn
      // into multi-cm ATE divergence over 2000+ frames. V3 SOP CV=0%
      // determinism (Sprint 12) demands the bit-identical expression,
      // not just "byte-compare within tolerance".
      //
      // The header helper compute_omega_eff_shim_legacy() exists for
      // unit-test verification only; runtime uses the inline expression.
      // ----------------------------------------------------------------
      float w_i;
      double omega_eff_double = 0.0;  // for χ² gate (computed below if needed)
      if (!config.anisotropic_iekf_enable) {
        // Path (a): legacy verbatim. Preserves I-2 (S5 bit-identical when OFF).
        w_i = robust_w / (0.001f + sigma2_i);
      } else {
        if (config.anisotropic_iekf_scalar_shim) {
          // Shim: reuse legacy fp32 expression exactly. Architect §4 I-2
          // requires byte-identity, not "close-to". R0.2 §2.5 tolerances
          // are unit-test envelopes, not runtime tolerances.
          w_i = robust_w / (0.001f + sigma2_i);
          // omega_eff_double left at 0.0 — χ² gate is skipped under shim.
        } else if (corr.has_surfel_cov_L1) {
          // Full Ω_eff = nᵀ(Σ_L1 + R·Σ_p·Rᵀ + ε·I)⁻¹ n.
          // R_world_from_lidar ≈ R_wb (body→lidar extrinsic small).
          omega_eff_double = compute_omega_eff(
              corr.normal,
              corr.surfel_cov_L1,
              R_wb,
              corr.p_lidar,
              sigma2_base,
              config.anisotropic_iekf_sigma_theta_sq,
              config.anisotropic_iekf_epsilon);
          w_i = robust_w * static_cast<float>(omega_eff_double);
        } else {
          // Σ_L1 storage not populated for this corr (e.g. B.A.2's
          // surfel_cov fill skipped a voxel). Fall back to scalar form
          // with the CONFIGURED ε ridge (NOT 0.001f) — keeps the path
          // identifiable as P1-active without secretly leaking legacy
          // floor into the calibration grid.
          omega_eff_double = 1.0 /
              (static_cast<double>(config.anisotropic_iekf_epsilon) +
               static_cast<double>(sigma2_i));
          w_i = robust_w * static_cast<float>(omega_eff_double);
        }

        // χ²₁ outlier gate — only when shim is OFF (architect §3.1).
        // Setting w_i = 0 preserves the rank-1 add structure but
        // contributes nothing to HTRinvH/HTRinvz (information add = 0).
        if (!config.anisotropic_iekf_scalar_shim) {
          const double chi2_i =
              static_cast<double>(r_plane) * static_cast<double>(r_plane) *
              omega_eff_double;
          if (chi2_i > static_cast<double>(config.anisotropic_iekf_chi2_threshold)) {
            w_i = 0.0f;
          }
        }
      }
      (void)omega_eff_double;  // suppress unused-when-OFF warning

      // Sharing weight — down-weight correspondences sharing same surfel.
      if (corr.sharing_count > config.sharing_weight_threshold) {
        w_i /= static_cast<float>(corr.sharing_count) / config.sharing_weight_ref;
      }

        // 4. Jacobian row: h = [-A^T, -n^T]  (1×6)
        const Eigen::Vector3f C = R_wb_t * corr.normal;
        const Eigen::Vector3f A = p_imu.cross(C);

        Eigen::Matrix<double, 6, 1> h;
        h(0) = -A(0);  h(1) = -A(1);  h(2) = -A(2);
        h(3) = -corr.normal(0);  h(4) = -corr.normal(1);  h(5) = -corr.normal(2);

        // 5-6. Weighted rank-1 accumulation (double precision).
        const double w_d = static_cast<double>(w_i);
        const double r_d = static_cast<double>(r_plane);
        local_HTRinvH.noalias() += w_d * h * h.transpose();
        local_HTRinvz.noalias()  += (w_d * r_d) * h;
        // S12-B.A.1 DG-A: P2P L1 surfel contribution.
        local_HTRinvH_L1.noalias() += w_d * h * h.transpose();
        ++local_n_L1;
        ++local_n_full;

        // L2 multi-scale: additional rank-1 update from coarser surfel.
        // By the matrix information inequality, adding this PSD term can only
        // improve (or maintain) the posterior covariance — never degrade it.
        if (config.enable_l2_correspondences && corr.has_l2) {
          // L2 residual.
          const float r_L2 = -(corr.l2_normal.dot(p_world) - corr.l2_plane_d);

          // L2 Jacobian (same structure as L1, different normal).
          const Eigen::Vector3f C_L2 = R_wb_t * corr.l2_normal;
          const Eigen::Vector3f A_L2 = p_imu.cross(C_L2);
          Eigen::Matrix<double, 6, 1> h_L2;
          h_L2(0) = -A_L2(0);  h_L2(1) = -A_L2(1);  h_L2(2) = -A_L2(2);
          h_L2(3) = -corr.l2_normal(0);
          h_L2(4) = -corr.l2_normal(1);
          h_L2(5) = -corr.l2_normal(2);

          // L2 noise model: coarser scale = higher noise (v5.0-H1 parity).
          const Eigen::Vector3f dp_L2 = p_world - corr.l2_centroid;
          const float dist_sq_L2 = dp_L2.squaredNorm();
          float sigma2_L2;
          if (config.enable_adaptive_noise) {
            const float range_penalty = config.enable_range_inverse_weight
                ? 0.0f
                : config.adaptive_range_scale * (corr.range * corr.range) / r_ref_sq;
            const float one_minus_cos_L2 =
                1.0f - std::abs(corr.l2_normal.dot(dp_L2) /
                                (std::sqrt(dist_sq_L2) + 1e-6f));
            const float incidence_penalty_L2 =
                config.adaptive_incidence_scale * one_minus_cos_L2 * one_minus_cos_L2;
            const float planarity_penalty_L2 =
                config.adaptive_planarity_scale * corr.l2_planarity * corr.l2_planarity;
            sigma2_L2 = config.l2_noise_scale * sigma2_base *
                            (1.0f + range_penalty + incidence_penalty_L2 +
                             planarity_penalty_L2) +
                        (config.enable_sigma2_normal ? dist_sq_L2 * corr.l2_normal_sigma2 : 0.0f);
          } else {
            sigma2_L2 = config.l2_noise_scale * sigma2_base +
                        (config.enable_sigma2_normal ? dist_sq_L2 * corr.l2_normal_sigma2 : 0.0f);
          }
          // Range-inverse scaling for L2 correspondences.
          if (config.enable_range_inverse_weight) {
            const float ratio = std::max(
                config.range_inverse_min_ratio,
                corr.range / config.range_inverse_ref);
            sigma2_L2 *= std::pow(ratio, config.range_inverse_power);
          }

          // Huber weight for L2 (active when PKO enabled, matching L1).
          float huber_w_L2 = 1.0f;
          if (pko != nullptr) {
            const float abs_r_L2_norm =
                std::abs(r_L2 / pko_normalization_scale);
            huber_w_L2 =
                (abs_r_L2_norm <= huber_delta) ? 1.0f : (huber_delta / abs_r_L2_norm);
          }
          // S13-B.A.4 P1: L2 weight — mirrors L1 path at :869-947.
          // Four sub-paths (same structure as L1, see sprint13_architecture
          // §3.1). γ DG-A control gate is NOT applied here (B.B.1 lands that);
          // γ_cached starts at 1.0 by architect §3.2 design.
          float w_L2;
          double omega_eff_L2_double = 0.0;
          if (!config.anisotropic_iekf_enable) {
            // Path (a): legacy verbatim.
            w_L2 = huber_w_L2 / (0.001f + sigma2_L2);
          } else {
            if (config.anisotropic_iekf_scalar_shim) {
              // Path (b) shim: byte-identical legacy fp32 (S13-B.A.3 lesson).
              w_L2 = huber_w_L2 / (0.001f + sigma2_L2);
            } else if (corr.has_surfel_cov_L2) {
              // Path (c) full Ω_eff_L2 = l2_nᵀ(Σ_L2 + R·Σ_p·Rᵀ + ε·I)⁻¹ l2_n.
              omega_eff_L2_double = compute_omega_eff(
                  corr.l2_normal,
                  corr.surfel_cov_L2,
                  R_wb,
                  corr.p_lidar,
                  sigma2_base,
                  config.anisotropic_iekf_sigma_theta_sq,
                  config.anisotropic_iekf_epsilon);
              // S13-B.B.1: γ_cached from PRIOR iter's DG-A signature
              // (architect §3.2). First iter / OFF → γ_cached = 1.0.
              w_L2 = huber_w_L2 *
                     static_cast<float>(gamma_cached * omega_eff_L2_double);
            } else {
              // Path (d) Σ_L2 not populated → scalar with cfg ε.
              omega_eff_L2_double = 1.0 /
                  (static_cast<double>(config.anisotropic_iekf_epsilon) +
                   static_cast<double>(sigma2_L2));
              w_L2 = huber_w_L2 * static_cast<float>(omega_eff_L2_double);
            }

            // χ²₁ outlier gate (skip under shim, mirrors L1 path).
            if (!config.anisotropic_iekf_scalar_shim) {
              const double chi2_L2 =
                  static_cast<double>(r_L2) * static_cast<double>(r_L2) *
                  omega_eff_L2_double;
              if (chi2_L2 > static_cast<double>(config.anisotropic_iekf_chi2_threshold)) {
                w_L2 = 0.0f;
              }
            }
          }
          (void)omega_eff_L2_double;  // reserved for B.B.1 DG-A γ feedback

          const double w_L2_d = static_cast<double>(w_L2);
          const double r_L2_d = static_cast<double>(r_L2);
          local_HTRinvH.noalias() += w_L2_d * h_L2 * h_L2.transpose();
          local_HTRinvz.noalias()  += (w_L2_d * r_L2_d) * h_L2;
          // S12-B.A.1 DG-A: P2P L2 surfel contribution (gated on corr.has_l2).
          local_HTRinvH_L2.noalias() += w_L2_d * h_L2 * h_L2.transpose();
          ++local_n_L2;
          // Note: L2 add is supplementary to the L1 add at :801 within the
          // SAME iteration of the same correspondence — do NOT increment
          // local_n_full here (already incremented at L1 path).
        }

    }

    }  // end omp parallel

    // Deterministic sequential reduction: always thread 0, 1, 2, ...
    // Guarantees identical FP accumulation order across runs.
    for (int t = 0; t < iekf_omp_threads; ++t) {
      HTRinvH_66 += thr_H[t];
      HTRinvz_6  += thr_z[t];
      sum_res    += thr_res[t];
      sum_res_sq += thr_res2[t];
      // S12-B.A.1 DG-A: extend reduction in same deterministic order.
      HTRinvH_L1 += thr_H_L1[t];
      HTRinvH_L2 += thr_H_L2[t];
    }

    // S12-B.A.1 DG-A Invariant I-2 (debug-mode assertion):
    // HTRinvH_66 ≡ HTRinvH_L1 + HTRinvH_L2 bit-identically.
    // Per-iteration writes occur in fixed order (L1 add → joint add, or
    // L2 add → joint add) within the same thread; reduction sums per-thread
    // partial sums in the same order. Holds with FP non-associativity because
    // the same operands in the same order yield the same result.
#ifndef NDEBUG
    {
      const double inv_norm =
          (HTRinvH_66 - HTRinvH_L1 - HTRinvH_L2).norm();
      assert(inv_norm < 1e-9 && "DG-A I-2 invariant violated");
    }
#endif

    // S12-B.A.2 DG-A: per-level Schur translation-block signature.
    // Runs at every outer iteration; the LAST iteration's values land in
    // frame_dg_a_* (consumed by result population at end of iekf_update).
    if (config.dg_a_enable) {
      // Reduce per-thread correspondence counts.
      int n_corr_L1_sum = 0, n_corr_L2_sum = 0, n_corr_full_sum = 0;
      for (int t = 0; t < iekf_omp_threads; ++t) {
        n_corr_L1_sum   += thr_n_L1[t];
        n_corr_L2_sum   += thr_n_L2[t];
        n_corr_full_sum += thr_n_full[t];
      }
      // Per-channel Schur — skip empty channels (avoid LDLT on zero matrix).
      SchurTransResult sr_L1, sr_L2, sr_full;
      if (n_corr_L1_sum > 0)
        sr_L1 = compute_schur_translation(HTRinvH_L1, config.dg_a_schur_eps);
      if (n_corr_L2_sum > 0)
        sr_L2 = compute_schur_translation(HTRinvH_L2, config.dg_a_schur_eps);
      if (n_corr_full_sum > 0)
        sr_full = compute_schur_translation(HTRinvH_66, config.dg_a_schur_eps);
      // Store per-iteration (overwritten until terminal iteration).
      frame_dg_a_rho[0] = sr_L1.rho_trans;
      frame_dg_a_rho[1] = sr_L2.rho_trans;
      frame_dg_a_rho[2] = sr_full.rho_trans;
      frame_dg_a_d_trans[0] = sr_L1.d_trans;
      frame_dg_a_d_trans[1] = sr_L2.d_trans;
      frame_dg_a_d_trans[2] = sr_full.d_trans;
      // Cross-channel agreement cosines |dot|, sign-invariant.
      frame_dg_a_cos_agree[0] =
          std::abs(sr_L1.d_trans.dot(sr_L2.d_trans));
      frame_dg_a_cos_agree[1] =
          std::abs(sr_L1.d_trans.dot(sr_full.d_trans));
      frame_dg_a_cos_agree[2] =
          std::abs(sr_L2.d_trans.dot(sr_full.d_trans));
      frame_dg_a_n_corr[0] = n_corr_L1_sum;
      frame_dg_a_n_corr[1] = n_corr_L2_sum;
      frame_dg_a_n_corr[2] = n_corr_full_sum;

      // S13-B.B.1 DG-A γ control gate (architect §3.2 + R0.1 §2.4 + R0.2 §2):
      //   γ = ρ_L2 · (1 − β · cos²θ_12)
      //   β = clip(1 − ρ_L2 / ρ_ref_avia, 0, 1)
      // γ updates AT END OF this iter; consumed at NEXT iter's L2 weight
      // site via gamma_cached (lives at function-scope outside loop).
      // Requires anisotropic_iekf_enable=true (gate is the *control*; without
      // P1 weight path active, γ has no consumer). ρ_ref_avia ≤ 0 (default
      // pre-G5 calibration) disables modulation — γ stays at the prior
      // iter's value (initially 1.0, B.A.4 behavior preserved).
      //
      // Regime table (R0.1 §2.4 / R0.2 §2.4):
      //   (i)   L1 strong + agree (cos²=1, ρ_L2 ≪ ρ_ref → β→1) → γ=0      (suppress)
      //   (ii)  L1 weak / ρ_L2 ≈ ρ_ref (β→0) → γ=ρ_L2                     (keep at ρ_L2)
      //   (iii) L1 strong + disagree (cos²=0, β→1) → γ=ρ_L2 ·1 = ρ_L2     (keep)
      if (config.anisotropic_iekf_enable &&
          config.anisotropic_iekf_rho_ref_avia > 0.0 &&
          sr_L2.rho_trans > 0.0 && n_corr_L2_sum > 0) {
        const double rho_L2 = sr_L2.rho_trans;
        const double cos_12 = frame_dg_a_cos_agree[0];  // |dot(d_L1, d_L2)|
        const double cos2_12 = cos_12 * cos_12;
        const double rho_ref = config.anisotropic_iekf_rho_ref_avia;
        double beta = 1.0 - rho_L2 / rho_ref;
        if (beta < 0.0) beta = 0.0;
        if (beta > 1.0) beta = 1.0;
        gamma_cached = rho_L2 * (1.0 - beta * cos2_12);
        // γ ∈ [0, ρ_L2] in normal conditions; clamp non-negative for safety.
        if (gamma_cached < 0.0) gamma_cached = 0.0;
      }
      // else: γ_cached stays at the prior iter's value (initially 1.0).
    }

    // Track residual statistics.
    last_res_mean = sum_res / n;
    last_res_rms = std::sqrt(sum_res_sq / n);

    // ------------------------------------------------------------------
    // Degeneracy detection — eigenvalue gating of HTRinvH_66.
    // Directions with insufficient LiDAR constraint are zeroed out,
    // making the filter trust IMU propagation in those directions.
    // ------------------------------------------------------------------
    // Direction-selective map insertion: extract the WEAKEST translation
    // direction from eigenvalue analysis BEFORE IEKF gating modifies HTRinvH_66.
    // Only extract one direction (the weakest) to avoid over-suppression.
    // Criterion: eigenvalue ratio < 0.01 (direction has < 1% of max constraint)
    // AND the eigenvector has a significant translation component.
    {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig_map(HTRinvH_66);
      const auto& ev = eig_map.eigenvalues();
      const auto& V_map = eig_map.eigenvectors();
      const double lam_max = ev(5);
      const double kMapRatioThresh = static_cast<double>(config.map_degen_ratio_threshold);

      if (lam_max > 1e-6) {
        const double ratio = ev(0) / lam_max;
        frame_eigenvalue_ratio = static_cast<float>(ratio);

        if (ratio < kMapRatioThresh) {
          // Extract translation block of the weakest eigenvector only.
          Eigen::Vector3f pos_block = V_map.col(0).tail<3>().cast<float>();
          const float pos_norm = pos_block.norm();

          if (pos_norm > 0.1f) {
            frame_degen_trans_dirs[0] = pos_block / pos_norm;
            frame_num_degen_trans_dirs = 1;
          }
        }
      }

      // B4 — IEKF iteration signature (PV-3 attribution).
      //
      // Reuses the already-computed 6-eigenvalue spectrum (no extra SVD).
      // Fires on every inner iteration; the analyzer keeps only the final
      // row per (frame_idx, B4) pair, which corresponds to the last
      // iteration (converged or max-iter) — matching the architect spec.
      //
      // Layout: 8 doubles = (λ0, λ1, λ2, λ3, λ4, λ5, residual_rms_sq, iter).
      // Aux: condition number = λ5 / max(λ0, 1e-12).
      if (tof_slam::frontend::diag::BoundaryLogger::enabled()) {
        double pack[8];
        for (int i = 0; i < 6; ++i) {
          pack[i] = static_cast<double>(ev(i));
        }
        pack[6] = static_cast<double>(last_res_rms) *
                  static_cast<double>(last_res_rms);
        pack[7] = static_cast<double>(total_iters);
        const double denom = std::max(static_cast<double>(ev(0)), 1e-12);
        const double cond_num = static_cast<double>(ev(5)) / denom;
        tof_slam::frontend::diag::BoundaryLogger::instance().log(
            tof_slam::frontend::diag::current_frame_idx(),
            tof_slam::frontend::diag::BoundaryId::B4_IekfIter,
            tof_slam::frontend::diag::make_byte_view(pack, sizeof(pack)),
            cond_num);
      }
    }

    if (config.enable_icdr && icdr_state != nullptr) {
      // ICDR: Information-theoretic Continuous Degeneracy Regularization.
      // BUG FIX: Use inverse of the 6×6 rot-pos subblock of P_iter, NOT the
      // top-left 6×6 of the full 18×18 inverse.  The latter includes Schur
      // complement contributions from vel/bias/gravity cross-terms, which
      // inflates lambda_prior and drives all ρ→0 (everything looks degenerate).
      const Eigen::Matrix<double, 6, 6> P_pose_inv_correct =
          P_iter.topLeftCorner<6, 6>().inverse();

      // Use separate degen-dir output so we don't overwrite the pre-ICDR
      // map-insertion direction computed above.
      int icdr_num_degen_dirs = 0;
      Eigen::Vector3f icdr_degen_dirs[3];

      int n_degen = apply_icdr_remap(
          HTRinvH_66, HTRinvz_6,
          P_pose_inv_correct,
          config, icdr_state,
          state.position, total_iters,
          frame_eigenvalues,
          &icdr_num_degen_dirs,
          icdr_degen_dirs,
          frame_icdr_rho,
          frame_icdr_weights);

      // Merge: use whichever found more degenerate translation directions.
      // When ICDR directions win, reset eigenvalue_ratio to 0 so severity
      // falls back to 1.0 — the ICDR eigendecomp is a different matrix and
      // the map-direction ratio is not physically meaningful for ICDR dirs.
      if (icdr_num_degen_dirs > frame_num_degen_trans_dirs) {
        frame_num_degen_trans_dirs = icdr_num_degen_dirs;
        for (int d = 0; d < icdr_num_degen_dirs; ++d) {
          frame_degen_trans_dirs[d] = icdr_degen_dirs[d];
        }
        frame_eigenvalue_ratio = 0.0f;  // invalidate map-direction ratio
      }
      if (n_degen > max_degenerate_dirs) {
        max_degenerate_dirs = n_degen;
      }
    } else if (config.enable_degeneracy_detection) {
      int n_degen = apply_degeneracy_remap(
          HTRinvH_66, HTRinvz_6,
          config, 0,
          do_timing ? frame_eigenvalues : nullptr);
      if (n_degen > max_degenerate_dirs) {
        max_degenerate_dirs = n_degen;
      }
    }

    // Embed into full 18x18.
    StateCovariance HTRinvH_full = StateCovariance::Zero();
    HTRinvH_full.topLeftCorner<6, 6>() = HTRinvH_66;

    const float sigma_vel  = config.velocity_sigma;
    const float sigma_bg   = config.bias_bg_sigma;
    const float sigma_ba   = config.bias_ba_sigma;

    // ------------------------------------------------------------------
    // Phase C: IMU Bias Pseudo-Observation (Point-LIO inspired).
    //
    // Add zero-mean Gaussian prior on gyro/acc biases:
    //   z_bg = 0 - bg,  R_bg = σ_bg² I₃   →  info += (1/σ_bg²) I₃ at [9:12]
    //   z_ba = 0 - ba,  R_ba = σ_ba² I₃   →  info += (1/σ_ba²) I₃ at [12:15]
    //
    // This makes biases directly observable in the information matrix,
    // preventing unconstrained bias drift (especially acc z-bias → z-drift).
    // ------------------------------------------------------------------
    if (config.enable_bias_pseudo_obs) {
      const float bg_info = 1.0f / (sigma_bg * sigma_bg);
      const float ba_info = 1.0f / (sigma_ba * sigma_ba);
      for (int i = 9; i < 12; ++i)  HTRinvH_full(i, i) += bg_info;
      for (int i = 12; i < 15; ++i) HTRinvH_full(i, i) += ba_info;
    }

    // Phase E: Velocity pseudo-observation — zero-mean prior on velocity.
    // Adds (1/σ_v²) to diagonal at indices [6:9] (velocity block).
    // This constrains velocity to stay near zero, preventing systematic
    // overestimation when correspondences are sparse.
    if (config.enable_velocity_pseudo_obs) {
      const float vel_info = 1.0f / (sigma_vel * sigma_vel);
      for (int i = 6; i < 9; ++i)  HTRinvH_full(i, i) += vel_info;
    }

    // Direction-selective velocity constraint in degenerate directions.
    // When degeneracy is detected (e.g., tunnel axis), LiDAR cannot
    // constrain axial velocity → drift. Add extra information (tighter
    // sigma) specifically in the degenerate direction to prevent this.
    // Uses a rank-1 update: d * d^T * (1/σ_degen² - 1/σ_vel²) where d
    // is the degenerate direction unit vector.
    if (config.velocity_degen_sigma > 0.0f &&
        frame_num_degen_trans_dirs > 0) {
      const double degen_info = 1.0 / (config.velocity_degen_sigma *
                                        config.velocity_degen_sigma);
      const double base_info = config.enable_velocity_pseudo_obs
          ? (1.0 / (static_cast<double>(sigma_vel) * sigma_vel)) : 0.0;
      const double extra_info = degen_info - base_info;
      if (extra_info > 0.0) {
        for (int dd = 0; dd < frame_num_degen_trans_dirs; ++dd) {
          const Eigen::Vector3d d =
              frame_degen_trans_dirs[dd].cast<double>();
          // Rank-1 update to velocity block [6:9, 6:9].
          HTRinvH_full.block<3, 3>(6, 6) +=
              extra_info * (d * d.transpose());
        }
      }
    }

    // Gravity norm constraint — enforce ||g|| ≈ 9.81.
    // Residual: r_g = ||g|| - 9.81
    // Jacobian: h_g = g^T / ||g||  (1×3 at indices [15:18])
    // Information: h_g^T * (1/σ²) * h_g  (3×3 rank-1 update)
    if (config.enable_gravity_norm_constraint) {
      const Eigen::Vector3f g = state.gravity;
      const float g_norm = g.norm();
      if (g_norm > 1e-6f) {
        const Eigen::Vector3d h_g = (g / g_norm).cast<double>();
        const double g_info = 1.0 / (config.gravity_norm_sigma * config.gravity_norm_sigma);
        // Rank-1 update to information matrix at gravity block [15:18]
        HTRinvH_full.block<3, 3>(kGravIdx, kGravIdx).noalias() +=
            g_info * h_g * h_g.transpose();
      }
    }

    // Full information matrix — computed in float64 for numerical stability.
    const CovD info_matrix_d = HTRinvH_full + P_inv_d;

    if (do_timing) {
      auto dt = std::chrono::duration<float, std::milli>(Clock::now() - t_build_start).count();
      acc_build_info_ms += dt;
    }

    // K1 = info^{-1} in float64 — matches reference: direct .inverse().
    const auto t_solve_start = do_timing ? Clock::now() : Clock::time_point{};
    const CovD K1_d = info_matrix_d.inverse();
    const StateCovariance K1 = K1_d;

    // ------------------------------------------------------------------
    // 5. Compute state correction dx.
    //
    // HTRinvz_6 was already computed in the fused loop above.
    // Build full 18×1 information-weighted residual vector.
    // LiDAR contribution at indices 0-5, bias pseudo-obs at 9-14.
    // ------------------------------------------------------------------

    // Build full 18×1 information-weighted residual vector (double precision).
    Eigen::Matrix<double, kStateDim, 1> HTRinvz_full_d =
        Eigen::Matrix<double, kStateDim, 1>::Zero();
    HTRinvz_full_d.head<6>() = HTRinvz_6;

    // Phase C: add bias pseudo-observation residuals.
    // Residual = predicted - observed = bias - 0 = bias.
    // Same sign convention as LiDAR (predicted - observed).
    if (config.enable_bias_pseudo_obs) {
      const double bg_info = 1.0 / (sigma_bg * sigma_bg);
      const double ba_info = 1.0 / (sigma_ba * sigma_ba);
      HTRinvz_full_d.segment<3>(9) = bg_info * state.gyro_bias.cast<double>();
      HTRinvz_full_d.segment<3>(12) = ba_info * state.acc_bias.cast<double>();
    }

    // Phase E: add velocity pseudo-observation residual.
    // Pushes velocity toward zero (indoor walking assumption).
    if (config.enable_velocity_pseudo_obs) {
      const double vel_info = 1.0 / (sigma_vel * sigma_vel);
      HTRinvz_full_d.segment<3>(6) = vel_info * state.velocity.cast<double>();
    }

    // Direction-selective velocity residual in degenerate directions.
    if (config.velocity_degen_sigma > 0.0f &&
        frame_num_degen_trans_dirs > 0) {
      const double degen_info = 1.0 / (config.velocity_degen_sigma *
                                        config.velocity_degen_sigma);
      const double base_info = config.enable_velocity_pseudo_obs
          ? (1.0 / (sigma_vel * sigma_vel)) : 0.0;
      const double extra_info = degen_info - base_info;
      if (extra_info > 0.0) {
        const Eigen::Vector3d vel = state.velocity.cast<double>();
        for (int dd = 0; dd < frame_num_degen_trans_dirs; ++dd) {
          const Eigen::Vector3d d =
              frame_degen_trans_dirs[dd].cast<double>();
          // Project velocity onto degenerate direction, add extra residual.
          const double v_proj = d.dot(vel);
          HTRinvz_full_d.segment<3>(6) += extra_info * v_proj * d;
        }
      }
    }

    // Gravity norm constraint residual.
    // r_g = ||g|| - 9.81, h_g = g/||g||
    // HTRinvz contribution: h_g * (1/σ²) * r_g
    if (config.enable_gravity_norm_constraint) {
      const Eigen::Vector3d g = state.gravity.cast<double>();
      const double g_norm = g.norm();
      if (g_norm > 1e-6) {
        const Eigen::Vector3d h_g = g / g_norm;
        const double g_info = 1.0 / (config.gravity_norm_sigma * config.gravity_norm_sigma);
        const double r_g = g_norm - static_cast<double>(kGravityNorm);
        HTRinvz_full_d.segment<3>(kGravIdx) += g_info * r_g * h_g;
      }
    }

    // ------------------------------------------------------------------
    // State correction: dx = K1 * (-H^T R^{-1} z)
    //
    // This is the MAP estimate for the state perturbation given measurements
    // and prior.  The prior information enters through P^{-1} in K1.
    // ------------------------------------------------------------------
    const StateVector dx =
        (K1 * (-HTRinvz_full_d)).cast<float>();

    // ------------------------------------------------------------------
    // Safety: cap dx norm to prevent catastrophic corrections.
    // Phase B testing showed that removing/relaxing this cap causes
    // cascading divergence: large dx → correspondence loss → velocity
    // runaway → NaN.  Keep original thresholds.
    // ------------------------------------------------------------------
    const float rot_dx_norm = dx.segment<3>(0).norm();
    const float pos_dx_norm = dx.segment<3>(3).norm();
    last_rot_dx_norm = rot_dx_norm;
    last_pos_dx_norm = pos_dx_norm;
    if (rot_dx_norm > 0.5f || pos_dx_norm > 1.0f) {
      SPDLOG_WARN("IEKF dx too large: rot={:.4f} pos={:.4f}, skipping",
                  rot_dx_norm, pos_dx_norm);
      break;
    }

    // ------------------------------------------------------------------
    // 6. Apply state correction on the manifold.
    // ------------------------------------------------------------------
    state = state + dx;

    // NOTE: Gravity re-normalization DISABLED inside IEKF inner loop.
    // Applying it here breaks the IEKF linearization consistency — the next
    // iteration sees a different state than what the Jacobian was computed for.
    // If needed, do it AFTER the IEKF outer loop in lio_estimator.cpp.

    // ------------------------------------------------------------------
    // 7. Compute G matrix and posterior covariance via (I-G)*P.
    //
    //    G = K1 * H^T * R^{-1} * H     (18x18 gain matrix)
    //    P_posterior = (I - G) * P_prior
    //
    //    This form preserves the cross-covariance structure (pos-vel,
    //    rot-vel, etc.) much better than K1 = info^{-1} in float32,
    //    because it avoids inverting a matrix with extreme condition
    //    number (position block >> velocity block).
    // ------------------------------------------------------------------
    // G = K1 * H^T * R^{-1} * H (full 18×18).
    // When bias pseudo-obs is enabled, HTRinvH_full has non-zero entries
    // at bias blocks too, so we use the full matrix product.
    StateCovariance G = K1 * HTRinvH_full.cast<double>();
    G_final = G;

    if (do_timing) {
      auto dt = std::chrono::duration<float, std::milli>(Clock::now() - t_solve_start).count();
      acc_solve_ms += dt;
    }

    // ------------------------------------------------------------------
    // 8. Convergence check.
    // ------------------------------------------------------------------
    const float rot_norm = dx.segment<3>(0).norm();
    const float pos_norm = dx.segment<3>(3).norm();
    if (rot_norm < config.convergence_threshold &&
        pos_norm < config.convergence_threshold) {
      converged = true;
      break;
    }
  }

  // Apply posterior covariance with A-matrix manifold correction.
  //
  // FAST-LIO2 (esekfom.hpp:1834-1924):
  //   L = A^T * P_propagated * A  (A-corrected covariance)
  //   K_x_corrected = A^T * K_x  (A-corrected gain)
  //   P_posterior = L - K_x_corrected * P_propagated
  //
  // Our equivalent: use last G_final (= K_x = K1 * HTRinvH at convergence),
  // apply A^T to rotation block, then compute P_posterior = (I - G_corr) * L.
  {
    // A-matrix for the final state's rotation deviation from prior.
    const Eigen::Matrix3f R_diff_final =
        prior.rotation.transpose() * state.rotation;
    const Eigen::Vector3f phi_final = So3(R_diff_final).Log();

    CovD L = P_prior;  // start with propagated
    CovD G_corr = G_final;

    if (config.enable_a_matrix_correction && phi_final.norm() > kEpsilon) {
      const Eigen::Matrix3f A = LeftJacobian(phi_final);
      const Eigen::Matrix3d At = A.transpose().cast<double>();
      const Eigen::Matrix3d Ad = A.cast<double>();

      // L = A^T * P * A for rotation block
      L.topRows<3>() = At * L.topRows<3>();
      L.leftCols<3>() = L.leftCols<3>() * Ad;

      // G_corrected: apply A^T to rotation rows of K_x
      G_corr.topRows<3>() = At * G_corr.topRows<3>();
    }

    const CovD P_post_d = L - G_corr * P_prior;
    // Symmetrise to counteract accumulation of rounding.
    const CovD P_sym_d = (P_post_d + P_post_d.transpose()) * 0.5;
    state.covariance = P_sym_d;
  }

  // Apply covariance floor to prevent collapse.
  apply_covariance_floor(state.covariance, config);

  IekfResult result;
  result.state = state;
  result.converged = converged;
  result.total_iterations = total_iters;
  result.num_correspondences = static_cast<int>(correspondences.size());
  result.num_degenerate_dirs = max_degenerate_dirs;
  result.res_mean = last_res_mean;
  result.res_rms = last_res_rms;

  // Store final correction norms for diagnostics
  result.rot_correction_norm = last_rot_dx_norm;
  result.pos_correction_norm = last_pos_dx_norm;

  // Debug timing
  result.jacobian_ms = acc_jacobian_ms;
  result.huber_pko_ms = acc_huber_pko_ms;
  result.build_info_ms = acc_build_info_ms;
  result.solve_ms = acc_solve_ms;

  // Eigenvalue diagnostics (from last iteration)
  for (int i = 0; i < 6; ++i) result.eigenvalues[i] = frame_eigenvalues[i];

  // ICDR diagnostics
  if (config.enable_icdr) {
    for (int i = 0; i < 6; ++i) {
      result.icdr_rho[i] = frame_icdr_rho[i];
      result.icdr_weights[i] = frame_icdr_weights[i];
    }
  }

  // Degenerate translation directions for direction-selective map insertion
  result.num_degen_trans_dirs = frame_num_degen_trans_dirs;
  result.eigenvalue_ratio = frame_eigenvalue_ratio;
  for (int d = 0; d < frame_num_degen_trans_dirs; ++d) {
    result.degen_trans_dirs[d] = frame_degen_trans_dirs[d];
  }

  // S12-B.A.2 DG-A: per-channel signature output (terminal IEKF iteration).
  // Zero when config.dg_a_enable is false (frame_dg_a_* default-zero).
  for (int c = 0; c < 3; ++c) {
    result.dg_a_rho[c]        = frame_dg_a_rho[c];
    result.dg_a_d_trans[c]    = frame_dg_a_d_trans[c];
    result.dg_a_cos_agree[c]  = frame_dg_a_cos_agree[c];
    result.dg_a_n_corr[c]     = frame_dg_a_n_corr[c];
  }

  return result;
}

}  // namespace core
}  // namespace tof_slam
