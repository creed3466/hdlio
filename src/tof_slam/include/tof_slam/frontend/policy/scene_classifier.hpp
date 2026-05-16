// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 TofSLAM Authors
//
// scene_classifier.hpp — Sprint 5 R7 scene-classifier paradigm.
//
// One-shot sequence-level classifier. After N_warmup post-init frames, the
// classifier emits a single SceneClass label that locks for the remainder
// of the sequence. The corresponding ClassTemplate is applied via SurfelMap
// setters + Config mutation + lazy PointVoxelMap build.
//
// Replaces R3-R6 frame-level predicate dispatch (proven architectural floor
// 0.388m vs target 0.314m). EC1 offline gate already PASSED (17/17 strict).

#ifndef TOF_SLAM_FRONTEND_POLICY_SCENE_CLASSIFIER_HPP_
#define TOF_SLAM_FRONTEND_POLICY_SCENE_CLASSIFIER_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

namespace tof_slam {
namespace core {

/// One enum entry per per-seq template + IN_DEFAULT for indoor.
/// Order is stable for serialization (written to diagnostics CSV).
enum class SceneClass : std::uint8_t {
  CLEAN_DENSE     = 0,  ///< DK01 template (R8 rule)
  OUTDOOR_DRIFT   = 1,  ///< DK02 template (R7 rule)
  CLEAN_OUT       = 2,  ///< DY03 template + R10 default (R2 rule)
  DYNAMIC         = 3,  ///< DY04 template (R4 rule)
  OCCLUSION_PNCG  = 4,  ///< OC03 template (R3 rule)
  OCCLUSION       = 5,  ///< OC04 template (R5 rule)
  CLASS_D         = 6,  ///< VI03 template (R1 rule)
  HIGH_COS2       = 7,  ///< VI04 template (R6 rule)
  CORRIDOR        = 8,  ///< VI05 template (R9 rule)
  IN_DEFAULT      = 9,  ///< Indoor: maps to avia_indoor.yaml-equivalent
};

std::string_view scene_class_name(SceneClass c) noexcept;

/// Aggregated stats over frames [0, N_warmup), where frame 0 is the first
/// frame *after* first_lidar_frame_ (gravity-init frames excluded).
struct WarmupAccumulator {
  std::int64_t sum_corrs       = 0;
  std::int64_t sum_l1          = 0;
  double       sum_cos2        = 0.0;
  double       sum_vel         = 0.0;
  double       sum_ndeg        = 0.0;
  int          count_ndeg_ge_3 = 0;
  int          count_cos2_ge_02 = 0;
  int          count_cos2_ge_03 = 0;
  int          frames_collected = 0;

  void update(int corrs, std::size_t l1, double mean_cos2,
              int n_degen_dirs, double vel_norm) noexcept;
  void reset() noexcept;
};

/// Classifier configuration. Default values match R8.1 hybrid two-stage paradigm.
/// Stage A (frame_count_==stage_a_frame): instantaneous rho_1 gate splits
///   - INDOOR (rho_1 >= rho_indoor_min)  → IN_DEFAULT
///   - CLASS_D (rho_class_d_min <= rho_1 < rho_class_d_max)  → CLASS_D (VI03)
///   - OUTDOOR_OTHER (rho_1 < rho_class_d_min)  → defer to Stage B
/// Stage B (frame_count_==stage_b_window_end): aggregated [stage_b_window_start,
/// stage_b_window_end) over (corrs, l1_count, cos2, max_degen, vel) drives the
/// R7.1 decision tree (R2..R10; R1 CLASS_D removed because Stage A handles it).
///
/// Frame-numbering note: CSV `frame=N` ↔ runtime `frame_count_=N+1` because
/// the classifier hook fires AFTER `frame_count_++` (lio_estimator.cpp:444).
/// R8.1 derived all numbers from CSV; defaults below are runtime values (+1).
struct ClassifierConfig {
  bool enable    = true;
  bool is_indoor = false;     ///< ROS launch arg.

  // R8: two-stage trigger configuration (runtime frame_count_ values).
  int  stage_a_frame         = 2;     ///< CSV frame=1 ↔ runtime frame_count_=2.
  int  stage_b_window_start  = 51;    ///< CSV frame [50,80) ↔ runtime [51,81).
  int  stage_b_window_end    = 81;
  // R8 R-A: joint (degen, rho_1) predicate. R8.1's rho-alone gate failed at
  // runtime (VI03 runtime rho_1=0.629 vs CSV 0.914 — config-dependent surfel
  // construction shifts rho distribution). degen_dirs (IEKF info-matrix
  // eigenstructure) is config-invariant; VI03 is uniquely (degen=1, rho>=0.50)
  // among all 17 seqs. Verified by 9-seq runtime campaign (dump/R8_F5_RA_measure).
  int   degen_class_d_max    = 1;     ///< Stage A CLASS_D: degen <= this (VI03=1).
  float rho_class_d_min      = 0.50f; ///< Stage A CLASS_D: rho >= this AND degen<=1.
                                       ///< VI03=0.629, VI04=0.410, OC04=0.332.
  float rho_indoor_min       = 1.00f; ///< Stage A INDOOR safety net (indoors >=1.21).

  // R0.9 H3b: Stage A OUTDOOR_DRIFT discriminator (4-clause joint predicate).
  // Signal thresholds derived as maximum-margin separators on Avia outdoor 9-seq
  // [verified: dump/S13_R0_8_telemetry_20260513_2326/, researcher R0.9.1 §3.1.1].
  // is_avia_outdoor is a sensor-domain GUARD (mirrors is_indoor at line 74) —
  // defense-in-depth even with 0/9 Mid-360 signal matches per researcher §3.1.1.C.
  bool        is_avia_outdoor        = false;  ///< Set true ONLY in avia_outdoor.yaml.
  int         degen_outdoor_drift    = 2;      ///< DK02 max_degen exact match.
  float       cos2_outdoor_drift_max = 0.04f;  ///< DK02 cos2_mean_inst=0.039 (margin -2.6%).
  std::size_t l1_outdoor_drift_min   = 900;    ///< DK02 l1_count=944 (margin +4.7%).

  // Stage B decision-tree thresholds (R7.1 §A.2 verbatim — R1 CLASS_D dead).
  // R1 fields retained for fixture binary-compat but unused in classify_stage_b.
  float pct_d3_class_d         = 0.30f;
  float rho_class_d            = 0.50f;
  // R2 (CLEAN_OUT / DY03):
  float pct_c03_clean_out      = 0.60f;
  float vel_clean_out          = 0.15f;
  float cos2_clean_out         = 0.30f;
  // R3 (OCCLUSION_PNCG / OC03):
  // S13-R0.8 H2: raise pct_d3 max (0.95→1.01) to admit OC03 (pct_d3=1.000)
  //              raise pct_c03 min (0.50→0.85) to separate OC03 (pct_c03=1.000)
  //                from DY04 (pct_c03=0.767) — DY04 now correctly falls to R4.
  // Evidence: Step 0 telemetry dump/S13_R0_8_telemetry_20260513_2326/*.
  float pct_d3_occ_pncg_max    = 1.01f;
  float pct_c03_occ_pncg_min   = 0.85f;
  float vel_occ_pncg_min       = 0.15f;
  float cos2_occ_pncg_min      = 0.27f;
  // R4 (DYNAMIC / DY04):
  float pct_d3_dynamic_min     = 0.95f;
  // R5 (OCCLUSION / OC04):
  float pct_c02_occ_min        = 0.60f;
  float pct_c03_occ_max        = 0.20f;
  float corrs_occ_min          = 800.0f;
  float ndeg_occ_min           = 2.9f;
  // R6 (HIGH_COS2 / VI04): added pct_d3 discriminator to escape DK02 misfire
  // (DK02 pct_d3=0.78 vs VI04 pct_d3=0.12 — clean separation).
  // S13-R0.8 H2: raise pct_d3 max (0.50→0.85) — under unified config Step 0
  //              telemetry VI04 pct_d3=0.700 (was 0.12 in v6_seq), DK02
  //              pct_d3=1.000 (was 0.78). DK02 still excluded via R7
  //              OUTDOOR_DRIFT which fires earlier (rho<0.32, vel<0.10).
  float cos2_high_max          = 0.11f;
  float pct_d3_high_max        = 0.85f;  ///< S13-R0.8 H2: VI04 unified pct_d3=0.700
  // R7 (OUTDOOR_DRIFT / DK02):
  float rho_drift_max          = 0.32f;
  float pct_d3_drift_min       = 0.95f;
  float vel_drift_max          = 0.10f;
  // R8 (CLEAN_DENSE / DK01):
  float rho_clean_dense_max    = 0.40f;
  float cos2_clean_dense_max   = 0.20f;
  // R9 (CORRIDOR / VI05):
  // S13-R0.8 H2: lower pct_c02 min (0.80→0.70) — VI05 unified pct_c02=0.733.
  //              raise pct_c03 max (0.20→0.40) — VI05 unified pct_c03=0.367.
  //              DY03 (pct_c02=1.0, pct_c03=1.0) already caught by R2
  //              (pct_c03≥0.60, vel<0.15) before reaching R9. Side-effect safe.
  float pct_c02_corridor_min   = 0.70f;
  float pct_c03_corridor_max   = 0.40f;
  float vel_corridor_max       = 0.10f;
  float cos2_corridor_max      = 0.30f;
};

/// Stage A — frame_count_==stage_a_frame instantaneous hard gate.
/// R8 R-A joint predicate: uses max_degen (config-invariant IEKF info-matrix
/// eigenstructure) in addition to rho_1 to robustly identify CLASS_D under
/// unified config.
/// R0.9 H3b: 4-way return — IN_DEFAULT / CLASS_D / OUTDOOR_DRIFT / nullopt.
/// OUTDOOR_DRIFT routes DK02 to early LOCK via 4-clause AND predicate
/// (cfg.is_avia_outdoor GUARD first; researcher §3.1.1, architect §3.4).
/// Pure function. No side effects.
std::optional<SceneClass> classify_stage_a(int corrs_f1, std::size_t l1_f1,
                                            int max_degen_f1,
                                            float cos2_mean_inst_f1,
                                            const ClassifierConfig& cfg) noexcept;

/// Stage B — [stage_b_window_start, stage_b_window_end) aggregation,
/// 8-way (R7.1 decision tree R2..R10). Never returns CLASS_D or IN_DEFAULT.
/// Pure function. No side effects, no I/O, no globals. Deterministic.
SceneClass classify_stage_b(const WarmupAccumulator& acc,
                            const ClassifierConfig& cfg) noexcept;

/// LioEstimator-owned classifier state.
struct ClassifierState {
  WarmupAccumulator warmup{};                         ///< Stage B accumulator [51,81).
  bool              locked            = false;        ///< Terminal lock (A or B).
  SceneClass        locked_class      = SceneClass::CLEAN_DENSE;
  int               lock_frame        = -1;
  // R8 two-stage tracking:
  bool              stage_a_complete  = false;       ///< Stage A hook fired.
  bool              stage_b_started   = false;       ///< Stage B accumulator started.
  int               stage_a_frame_idx = -1;          ///< Diagnostics only.

  void reset() noexcept {
    warmup.reset();
    locked = false;
    locked_class = SceneClass::CLEAN_DENSE;
    lock_frame = -1;
    stage_a_complete = false;
    stage_b_started = false;
    stage_a_frame_idx = -1;
  }
};

/// S13-B.A.5 Path B: per-class P1 anisotropic-IEKF tuple.
/// Default = all-OFF = legacy scalar IEKF path (preserves I-2 bit-identity).
/// Only kT_CLEAN_DENSE is set to cfg_19 (Dark01 V4 grid winner) in
/// scene_classifier.cpp; 9 other kT_* entries leave defaults.
/// See docs/research/sprint13_architecture_pathB_20260513.md §3.1.
struct P1Tuple {
  bool   anisotropic_iekf_enable      = false;
  bool   anisotropic_iekf_scalar_shim = false;
  float  anisotropic_iekf_epsilon     = 1.0e-3f;
  double anisotropic_iekf_rho_ref_avia = 0.0;
  float  anisotropic_iekf_chi2_threshold = 3.841f;
  float  anisotropic_iekf_sigma_theta_sq = 9.0e-6f;
  bool   enable_range_inverse_weight  = false;
  float  range_inverse_ref            = 10.0f;
  float  range_inverse_power          = 1.0f;
  float  range_inverse_min_ratio      = 0.1f;
};

/// Class-conditional config template. Verbatim per-class config values
/// from avia_v6_seq/*.yaml (outdoor) and avia_indoor.yaml (IN_DEFAULT).
struct ClassTemplate {
  // Phase A (correspondence + IEKF, applied to LioEstimator::Config)
  float l2_noise_scale;
  int   max_corr_per_l1;
  float max_plane_distance;
  int   pvmap_k_neighbors;
  float pvmap_sigma2_scale;

  // Phase B (SurfelMap setters)
  float l0_ema_alpha_min;
  float alpha_degen_floor;
  float ema_gate_radius;
  int   l0_centroid_freeze_count;
  float sigma2_age_scale;
  float pncg_threshold;

  // Static flags (LioEstimator::Config + SurfelMap)
  bool enable_degen_pvmap_override;     // DDPO
  bool enable_cscf;
  bool enable_geometric_covariance;     // also wires P2D mode

  // PVMap construction-time (lazy build at lock)
  float pvmap_voxel_size;
  int   pvmap_max_points_per_voxel;
  float pvmap_planarity_threshold;

  // CLASS_D-specific extensions
  float cscf_kernel_bandwidth;
  float geometric_cov_min_eigenvalue;
  int   geometric_cov_min_points;
  float p2d_omega_scale;

  // S13-B.A.5 Path B: per-class P1 router payload.
  // Defaults to all-OFF (= legacy scalar IEKF) so existing kT_*
  // initializers continue to compile via aggregate trailing default-init.
  P1Tuple p1{};
};

/// Returns const reference to the verbatim template for @p cls.
/// Linker-resolved static table; no heap, no I/O.
const ClassTemplate& get_template(SceneClass cls) noexcept;

}  // namespace core
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_POLICY_SCENE_CLASSIFIER_HPP_
