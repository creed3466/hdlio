// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// diagnostics_monitor.hpp — Real-time performance monitoring and degradation
// detection for the LIO pipeline.
//
// Tracks per-frame diagnostics and detects anomalies:
//   - Residual RMS spike (3x moving average → drift warning)
//   - IEKF convergence failure streak (5+ consecutive → convergence warning)
//   - Low correspondence count (<100 → map sparsity warning)
//   - Gravity norm deviation (|g| outside 9.81 ± 0.1 → gravity drift warning)
//
// Outputs a CSV line per frame for offline analysis and issues spdlog warnings
// for real-time degradation detection.

#ifndef TOF_SLAM_FRONTEND_ESTIMATOR_DIAGNOSTICS_MONITOR_HPP_
#define TOF_SLAM_FRONTEND_ESTIMATOR_DIAGNOSTICS_MONITOR_HPP_

#include <cmath>
#include <fstream>
#include <iomanip>
#include <string>

#include <spdlog/spdlog.h>

namespace tof_slam {
namespace core {

/// Per-frame diagnostics snapshot.
struct FrameDiagnostics {
  int frame = 0;
  double timestamp = 0.0;

  // IEKF
  float res_rms = 0.0f;
  float res_mean = 0.0f;
  bool converged = true;
  int iekf_iters = 0;
  float pos_delta_norm = 0.0f;
  int num_degenerate_dirs = 0;

  // Correspondence
  int num_correspondences = 0;
  int num_l0_ok = 0;      // hybrid mode: L0 PCA successes
  int num_surfel_ok = 0;  // hybrid mode: surfel fallback successes

  // Map
  int l0_count = 0;
  int l1_count = 0;

  // IMU / State
  float gravity_norm = 9.81f;
  float gyro_bias_norm = 0.0f;
  float acc_bias_norm = 0.0f;
  float velocity_norm = 0.0f;

  // Covariance
  float trace_P_pos = 0.0f;  // trace of 3x3 position covariance block

  // Timing
  float total_ms = 0.0f;

  // Eq.(4) degeneracy direction instrumentation
  float d_deg_x = 0.0f;   ///< degeneracy translation direction x
  float d_deg_y = 0.0f;   ///< degeneracy translation direction y
  float d_deg_z = 0.0f;   ///< degeneracy translation direction z
  float mean_cos2 = 0.0f; ///< mean |n·d_deg|² over all modulated voxels this frame
  int   cos2_count = 0;   ///< number of voxels that computed cos² this frame

  // S6-R1.2 ρ_λ measurement campaign instrumentation (no algorithm change).
  /// IEKF measurement Jacobian eigenvalue ratio ρ_λ = λ_min/λ_max as computed
  /// by iekf_updater. Sourced from iekf_result.eigenvalue_ratio (see
  /// iekf_updater.cpp:1285 and lio_estimator.cpp:856).
  float eigenvalue_ratio = 0.0f;

  // S12-B.A.3 DG-A: per-channel Schur signature (instrumentation-only).
  // Populated when IekfConfig::dg_a_enable AND dg_a_log_per_channel.
  // Index: 0=L1, 1=L2, 2=full(joint).
  float dg_a_rho_l1 = 0.0f;
  float dg_a_rho_l2 = 0.0f;
  float dg_a_rho_full = 0.0f;
  float dg_a_cos_l1_l2 = 0.0f;     ///< |dot(d_L1, d_L2)|
  float dg_a_cos_l1_full = 0.0f;   ///< |dot(d_L1, d_full)|
  float dg_a_cos_l2_full = 0.0f;   ///< |dot(d_L2, d_full)|
  int   dg_a_n_l1 = 0;
  int   dg_a_n_l2 = 0;
};

/// Degradation alert types.
enum class AlertType {
  kNone = 0,
  kResidualSpike,       // res_rms > 3x moving average
  kConvergenceFailure,  // 5+ consecutive non-convergence
  kLowCorrespondences,  // < 100 correspondences
  kGravityDrift,        // |g| outside [9.71, 9.91]
};

/// Real-time diagnostics monitor with EMA tracking and alert detection.
class DiagnosticsMonitor {
 public:
  DiagnosticsMonitor() = default;

  /// Open CSV log file. Call once at initialization.
  void open_log(const std::string& path) {
    csv_.open(path, std::ios::out | std::ios::trunc);
    if (csv_.is_open()) {
      csv_ << "frame,timestamp,res_rms,res_mean,converged,iekf_iters,"
           << "pos_delta,degen_dirs,corrs,l0_ok,surfel_ok,"
           << "l0_count,l1_count,gravity_norm,gyro_bias_norm,"
           << "acc_bias_norm,vel_norm,trace_P_pos,total_ms,alert,"
           << "d_deg_x,d_deg_y,d_deg_z,mean_cos2,cos2_count,"
           << "eigenvalue_ratio,"
           // S12-B.A.3 DG-A per-channel signature columns:
           << "dg_a_rho_l1,dg_a_rho_l2,dg_a_rho_full,"
           << "dg_a_cos_l1_l2,dg_a_cos_l1_full,dg_a_cos_l2_full,"
           << "dg_a_n_l1,dg_a_n_l2\n";
      csv_.flush();
      SPDLOG_INFO("Diagnostics CSV: {}", path);
    }
  }

  /// Update with a new frame's diagnostics. Returns alert type (if any).
  AlertType update(const FrameDiagnostics& d) {
    AlertType alert = AlertType::kNone;

    // --- Residual RMS EMA and spike detection ---
    if (frame_count_ > 10) {
      if (res_rms_ema_ > 1e-6f && d.res_rms > 3.0f * res_rms_ema_) {
        alert = AlertType::kResidualSpike;
        SPDLOG_WARN("[DIAG] frame={} RESIDUAL_SPIKE: res_rms={:.4f} "
                    "(3x avg={:.4f})",
                    d.frame, d.res_rms, res_rms_ema_);
      }
    }
    res_rms_ema_ = kAlpha * d.res_rms + (1.0f - kAlpha) * res_rms_ema_;

    // --- Convergence failure streak ---
    if (!d.converged) {
      nonconv_streak_++;
      if (nonconv_streak_ >= 5 && alert == AlertType::kNone) {
        alert = AlertType::kConvergenceFailure;
        SPDLOG_WARN("[DIAG] frame={} CONVERGENCE_FAILURE: {} consecutive "
                    "non-convergent frames",
                    d.frame, nonconv_streak_);
      }
    } else {
      nonconv_streak_ = 0;
    }

    // --- Low correspondences ---
    if (d.num_correspondences < 100 && alert == AlertType::kNone) {
      alert = AlertType::kLowCorrespondences;
      SPDLOG_WARN("[DIAG] frame={} LOW_CORRESPONDENCES: {} (min=100)",
                  d.frame, d.num_correspondences);
    }

    // --- Gravity norm drift ---
    if (std::abs(d.gravity_norm - 9.81f) > 0.1f &&
        alert == AlertType::kNone) {
      alert = AlertType::kGravityDrift;
      SPDLOG_WARN("[DIAG] frame={} GRAVITY_DRIFT: |g|={:.4f} "
                  "(expected 9.81 ± 0.1)",
                  d.frame, d.gravity_norm);
    }

    // --- CSV output ---
    if (csv_.is_open()) {
      csv_ << d.frame << "," << std::fixed << std::setprecision(6)
           << d.timestamp << "," << std::setprecision(4)
           << d.res_rms << "," << d.res_mean << ","
           << (d.converged ? 1 : 0) << "," << d.iekf_iters << ","
           << d.pos_delta_norm << "," << d.num_degenerate_dirs << ","
           << d.num_correspondences << "," << d.num_l0_ok << ","
           << d.num_surfel_ok << ","
           << d.l0_count << "," << d.l1_count << ","
           << d.gravity_norm << "," << d.gyro_bias_norm << ","
           << d.acc_bias_norm << "," << d.velocity_norm << ","
           << d.trace_P_pos << ","
           << d.total_ms << "," << static_cast<int>(alert) << ","
           << d.d_deg_x << "," << d.d_deg_y << "," << d.d_deg_z << ","
           << d.mean_cos2 << "," << d.cos2_count << ","
           << d.eigenvalue_ratio << ","
           // S12-B.A.3 DG-A per-channel signature row:
           << d.dg_a_rho_l1 << "," << d.dg_a_rho_l2 << "," << d.dg_a_rho_full << ","
           << d.dg_a_cos_l1_l2 << "," << d.dg_a_cos_l1_full << "," << d.dg_a_cos_l2_full << ","
           << d.dg_a_n_l1 << "," << d.dg_a_n_l2 << "\n";
      // Flush every 100 frames for real-time monitoring
      if (d.frame % 100 == 0) csv_.flush();
    }

    frame_count_++;
    return alert;
  }

  /// Close CSV file.
  void close() {
    if (csv_.is_open()) csv_.close();
  }

  /// Get current residual RMS EMA.
  float res_rms_ema() const { return res_rms_ema_; }

  /// Get consecutive non-convergence count.
  int nonconv_streak() const { return nonconv_streak_; }

 private:
  static constexpr float kAlpha = 0.01f;  // EMA ~100-frame window

  std::ofstream csv_;
  int frame_count_ = 0;
  float res_rms_ema_ = 0.0f;
  int nonconv_streak_ = 0;
};

}  // namespace core
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_ESTIMATOR_DIAGNOSTICS_MONITOR_HPP_
