// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 TofSLAM Authors

#include "tof_slam/frontend/policy/scene_classifier.hpp"

#include <algorithm>

namespace tof_slam {
namespace core {

// ---------------------------------------------------------------------------
// SceneClass name strings (stable for diagnostics)
// ---------------------------------------------------------------------------

std::string_view scene_class_name(SceneClass c) noexcept {
  switch (c) {
    case SceneClass::CLEAN_DENSE:    return "CLEAN_DENSE";
    case SceneClass::OUTDOOR_DRIFT:  return "OUTDOOR_DRIFT";
    case SceneClass::CLEAN_OUT:      return "CLEAN_OUT";
    case SceneClass::DYNAMIC:        return "DYNAMIC";
    case SceneClass::OCCLUSION_PNCG: return "OCCLUSION_PNCG";
    case SceneClass::OCCLUSION:      return "OCCLUSION";
    case SceneClass::CLASS_D:        return "CLASS_D";
    case SceneClass::HIGH_COS2:      return "HIGH_COS2";
    case SceneClass::CORRIDOR:       return "CORRIDOR";
    case SceneClass::IN_DEFAULT:     return "IN_DEFAULT";
  }
  return "UNKNOWN";
}

// ---------------------------------------------------------------------------
// WarmupAccumulator
// ---------------------------------------------------------------------------

void WarmupAccumulator::update(int corrs, std::size_t l1, double mean_cos2,
                                int n_degen_dirs, double vel_norm) noexcept {
  sum_corrs += static_cast<std::int64_t>(corrs);
  sum_l1    += static_cast<std::int64_t>(l1);
  sum_cos2  += mean_cos2;
  sum_vel   += vel_norm;
  sum_ndeg  += static_cast<double>(n_degen_dirs);
  if (n_degen_dirs >= 3) ++count_ndeg_ge_3;
  if (mean_cos2 >= 0.20) ++count_cos2_ge_02;
  if (mean_cos2 >= 0.30) ++count_cos2_ge_03;
  ++frames_collected;
}

void WarmupAccumulator::reset() noexcept {
  *this = WarmupAccumulator{};
}

// ---------------------------------------------------------------------------
// classify_stage_a() — R8.1 §1 frame-1 hard gate.
// Splits {INDOOR / CLASS_D / OUTDOOR_OTHER-defer} from a single-frame
// (corrs, l1_count) snapshot at runtime frame_count_==cfg.stage_a_frame.
// Returns nullopt to defer to Stage B.
// ---------------------------------------------------------------------------

std::optional<SceneClass> classify_stage_a(int corrs_f1, std::size_t l1_f1,
                                            int max_degen_f1,
                                            float cos2_mean_inst_f1,
                                            const ClassifierConfig& cfg) noexcept {
  // is_indoor short-circuit (ROS launch arg). Matches R7 INDOOR semantics.
  if (cfg.is_indoor) return SceneClass::IN_DEFAULT;

  // rho_1 = corrs / max(l1, 1). Integer division wrapped in float; denominator
  // guarded so l1_count==0 (theoretical edge) defers safely.
  const float rho_1 = static_cast<float>(corrs_f1) /
                      std::max<float>(static_cast<float>(l1_f1), 1.0f);

  // INDOOR safety net (rho >= 1.00) — fires if is_indoor was not set but the
  // frame-1 signal is unambiguously indoor (rho_1 ≥ 1.21 for all 8 indoor seqs
  // per 9-seq runtime campaign, margin 0.21 to threshold).
  if (rho_1 >= cfg.rho_indoor_min) return SceneClass::IN_DEFAULT;

  // R-A: CLASS_D joint predicate. Runtime 9-seq campaign showed VI03 is the
  // unique (degen=1, rho>=0.50) sequence: VI03 (1, 0.629), VI04 (1, 0.410),
  // OC04 (1, 0.332), others (degen≥2). degen_dirs is config-invariant
  // (IEKF info-matrix structure); rho is config-shifted but VI03 stays highest
  // among degen=1 outdoor seqs.
  if (max_degen_f1 <= cfg.degen_class_d_max &&
      rho_1 >= cfg.rho_class_d_min)
    return SceneClass::CLASS_D;

  // R0.9 H3b: OUTDOOR_DRIFT joint predicate (DK02 early LOCK).
  // Clause order chosen for short-circuit cost (cheapest first) per
  // architect §3.4 ruling:
  //   (1) is_avia_outdoor — bool, sensor-domain GUARD; eliminates
  //       Mid-360/indoor/NTU at zero arithmetic cost (mirrors is_indoor pattern).
  //   (2) max_degen == 2 — int compare, categorical separator (DK02 unique).
  //   (3) cos2_mean_inst < 0.04 — float compare.
  //   (4) l1_count > 900 — size_t compare; uses already-computed l1_count
  //       at the call site in lio_estimator.cpp (no extra map read).
  // [verified: researcher §3.1.1 9-seq Avia + §3.1.1.C 9-seq Mid-360 telemetry —
  //  DK02 is the unique global match; 0/9 Mid-360 false positives even pre-guard.]
  if (cfg.is_avia_outdoor &&
      max_degen_f1 == cfg.degen_outdoor_drift &&
      cos2_mean_inst_f1 < cfg.cos2_outdoor_drift_max &&
      l1_f1 > cfg.l1_outdoor_drift_min) {
    return SceneClass::OUTDOOR_DRIFT;
  }

  // OUTDOOR_OTHER: defer aggregation to Stage B.
  return std::nullopt;
}

// ---------------------------------------------------------------------------
// classify_stage_b() — R7.1 decision tree minus R1 (CLASS_D handled by A).
// 8-way over {CLEAN_DENSE, OUTDOOR_DRIFT, CLEAN_OUT, DYNAMIC, OCCLUSION_PNCG,
// OCCLUSION, HIGH_COS2, CORRIDOR}. Never returns CLASS_D or IN_DEFAULT.
// Port of /tmp/decision_tree_v2.py with R8.1 window [50,80) semantics.
// ---------------------------------------------------------------------------

SceneClass classify_stage_b(const WarmupAccumulator& a,
                            const ClassifierConfig& cfg) noexcept {
  const int n = std::max(1, a.frames_collected);
  const double inv_n = 1.0 / static_cast<double>(n);

  // rho_warmup = sum(corrs) / max(sum(l1), 1)  // option (b)
  const double rho        = static_cast<double>(a.sum_corrs) /
                            static_cast<double>(std::max<std::int64_t>(a.sum_l1, 1));
  const double cos2_mean  = a.sum_cos2 * inv_n;
  const double vel_mean   = a.sum_vel  * inv_n;
  const double ndeg_mean  = a.sum_ndeg * inv_n;
  const double corrs_mean = static_cast<double>(a.sum_corrs) * inv_n;
  const double pct_d3     = static_cast<double>(a.count_ndeg_ge_3)  * inv_n;
  const double pct_c02    = static_cast<double>(a.count_cos2_ge_02) * inv_n;
  const double pct_c03    = static_cast<double>(a.count_cos2_ge_03) * inv_n;

  // R1 (CLASS_D) deleted: Stage A handles VI03.

  // R2: DY03 — high motion-cos2 plateau, low vel
  if (pct_c03 >= cfg.pct_c03_clean_out &&
      vel_mean < cfg.vel_clean_out &&
      cos2_mean >= cfg.cos2_clean_out)
    return SceneClass::CLEAN_OUT;

  // R3: OC03 — occluders cause pct_d3 drop, high cos2, motion present
  if (pct_d3 < cfg.pct_d3_occ_pncg_max &&
      pct_c03 >= cfg.pct_c03_occ_pncg_min &&
      vel_mean >= cfg.vel_occ_pncg_min &&
      cos2_mean >= cfg.cos2_occ_pncg_min)
    return SceneClass::OCCLUSION_PNCG;

  // R4: DY04 — full degeneracy, sustained motion-cos2 plateau
  if (pct_d3 >= cfg.pct_d3_dynamic_min &&
      pct_c03 >= cfg.pct_c03_occ_pncg_min &&
      vel_mean >= cfg.vel_occ_pncg_min)
    return SceneClass::DYNAMIC;

  // R5: OC04 — mid-cos2 plateau, high corrs
  if (pct_c02 >= cfg.pct_c02_occ_min &&
      pct_c03 < cfg.pct_c03_occ_max &&
      corrs_mean >= cfg.corrs_occ_min &&
      ndeg_mean >= cfg.ndeg_occ_min)
    return SceneClass::OCCLUSION;

  // R6: VI04 — very low cos2_mean (R7.2: pct_d3<0.5 escapes DK02 which has
  // sustained ndeg>=3 from non-repeating Avia scan pattern).
  if (cos2_mean < cfg.cos2_high_max && pct_c03 == 0.0 &&
      pct_d3 < cfg.pct_d3_high_max)
    return SceneClass::HIGH_COS2;

  // R7: DK02 — outdoor heading-drift
  if (rho < cfg.rho_drift_max &&
      pct_d3 >= cfg.pct_d3_drift_min &&
      vel_mean < cfg.vel_drift_max)
    return SceneClass::OUTDOOR_DRIFT;

  // R8: DK01 — clean-dense
  if (rho < cfg.rho_clean_dense_max &&
      pct_d3 >= cfg.pct_d3_drift_min &&
      cos2_mean < cfg.cos2_clean_dense_max)
    return SceneClass::CLEAN_DENSE;

  // R9: VI05 — corridor (cos2 in [0.20, 0.30) plateau, low vel)
  if (pct_c02 >= cfg.pct_c02_corridor_min &&
      pct_c03 < cfg.pct_c03_corridor_max &&
      vel_mean < cfg.vel_corridor_max &&
      cos2_mean < cfg.cos2_corridor_max)
    return SceneClass::CORRIDOR;

  // R10: fallback
  return SceneClass::CLEAN_OUT;
}

// ---------------------------------------------------------------------------
// 10 Verbatim ClassTemplates from avia_v6_seq/*.yaml + avia_indoor.yaml
// ---------------------------------------------------------------------------

namespace {

// CLEAN_DENSE — DK01 (avia_v6_seq/dark01.yaml) + S13-B.A.6 Path B: cfg_19 P1Tuple
// cfg_19 = V4 grid winner on Dark01 (ATE 0.0961m): σ²_base=0.02²(YAML), ref=15, ε=1e-3, range_inv=ON
constexpr ClassTemplate kT_CLEAN_DENSE = {
  /*l2_noise_scale*/9.0f, /*max_corr_per_l1*/0, /*max_plane_distance*/0.3f,
  /*pvmap_k_neighbors*/5, /*pvmap_sigma2_scale*/0.5f,
  /*l0_ema_alpha_min*/0.10f, /*alpha_degen_floor*/0.3f, /*ema_gate_radius*/0.0f,
  /*l0_centroid_freeze_count*/50, /*sigma2_age_scale*/0.0f, /*pncg_threshold*/0.0f,
  /*DDPO*/true, /*CSCF*/false, /*geom_cov*/false,
  /*pvmap_voxel_size*/0.3f, /*pvmap_max_pts*/20 /* R9: M permanent */, /*pvmap_planarity_threshold*/0.10f,
  /*cscf_bw*/0.25f, /*geom_cov_min_eig*/4e-4f, /*geom_cov_min_pts*/6, /*p2d_omega*/0.1f,
  /*p1*/{
    /*anisotropic_iekf_enable*/      true,
    /*anisotropic_iekf_scalar_shim*/ false,
    /*anisotropic_iekf_epsilon*/     1.0e-3f,    // cfg_19 ε
    /*anisotropic_iekf_rho_ref_avia*/0.0,        // γ inactive (Path A falsified)
    /*anisotropic_iekf_chi2_threshold*/3.841f,   // χ²₁ 95%
    /*anisotropic_iekf_sigma_theta_sq*/9.0e-6f,
    /*enable_range_inverse_weight*/  true,       // cfg_19 range_inv ON
    /*range_inverse_ref*/            15.0f,      // cfg_19 ref
    /*range_inverse_power*/          1.0f,
    /*range_inverse_min_ratio*/      0.1f,
  }
};

// OUTDOOR_DRIFT — DK02 (avia_v6_seq/dark02.yaml)
constexpr ClassTemplate kT_OUTDOOR_DRIFT = {
  3.0f, 0, 0.15f, 9, 1.0f,
  0.10f, 0.3f, 0.4f, 50, 0.01f, 0.0f,
  true, false, false,
  0.5f, 20 /* R9: M permanent */, 0.10f,
  0.25f, 4e-4f, 6, 0.1f,
  /*p1*/{}  // S13-B.A.6: P1 OFF → legacy scalar IEKF (I-5 preserved)
};

// CLEAN_OUT — DY03 (avia_v6_seq/dynamic03.yaml) + R10 default
constexpr ClassTemplate kT_CLEAN_OUT = {
  9.0f, 3, 0.3f, 9, 1.0f,
  0.10f, 0.3f, 0.2f, 50, 0.0f, 0.0f,
  true, false, false,
  0.5f, 20 /* R9: M permanent */, 0.10f,
  0.25f, 4e-4f, 6, 0.1f,
  /*p1*/{}  // S13-B.A.6: P1 OFF (Dyn03 V0 catastrophe isolation)
};

// DYNAMIC — DY04 (avia_v6_seq/dynamic04.yaml)
constexpr ClassTemplate kT_DYNAMIC = {
  9.0f, 3, 0.3f, 9, 1.0f,
  0.10f, 0.0f /* floor=0.0 */, 0.3f, 50, 0.0f, 0.0f,
  true, false, false,
  0.5f, 20 /* R9: M permanent */, 0.10f,
  0.25f, 4e-4f, 6, 0.1f,
  /*p1*/{}  // S13-B.A.6: P1 OFF (Dyn04 Path A 8900% catastrophe isolation)
};

// OCCLUSION_PNCG — OC03 (avia_v6_seq/occlusion03.yaml)
// R9 §3.2: keeps pvmap_k_neighbors=9 — lock-time k-recovery active (k IS mutable
// per lio_estimator.cpp:501). M permanent at 20 (no recovery possible).
constexpr ClassTemplate kT_OCCLUSION_PNCG = {
  9.0f, 3, 0.3f, 9 /* R9 lock→k=9 */, 0.7f /* pvmap_sigma2=0.7 */,
  0.145f, 0.3f, 0.2f, 50, 0.0f, 0.10f /* pncg=0.10 */,
  true, false, false,
  0.5f, 20 /* R9: M permanent */, 0.10f,
  0.25f, 4e-4f, 6, 0.1f,
  /*p1*/{}  // S13-B.A.6: P1 OFF
};

// OCCLUSION — OC04 (avia_v6_seq/occlusion04.yaml)
constexpr ClassTemplate kT_OCCLUSION = {
  9.0f, 0, 0.3f, 9, 1.0f,
  0.145f, 0.3f, 0.0f, 50, 0.0f, 0.0f,
  true, false, false,
  0.5f, 20 /* R9: M permanent */, 0.10f,
  0.25f, 4e-4f, 6, 0.1f,
  /*p1*/{}  // S13-B.A.6: P1 OFF
};

// CLASS_D — VI03 (avia_v6_seq/varying_illu03.yaml) — DDPO+CSCF+geom_cov ON, α=0.02
// S13-R0.11.1 H1 REVERTED: p1.{} (P1 OFF) — R0.10 Smoke B empirically falsified
// R0.8 H1 anisotropic-only at VI03 = 2.569m × 3 byte-id (4× HARD-ABORT). With
// p1.{}, when avia_outdoor.yaml router master gate is ON, Phase C overrides
// CLASS_D's IEKF P1 fields with default-OFF (anisotropic=false, range_inv=false)
// → restores V3 Path B P1-OFF baseline (VI03 ≈ 1.230m). H3b OUTDOOR_DRIFT
// discriminator and R0.10 H4 surfel rebuild are preserved (orthogonal to CLASS_D).
// See docs/results/s13_r0.10_smoke_B_20260514.md (Option B surgical revert) and
// docs/research/sprint13_research_R0_8_20260513.md §3.1 (now empirically refuted).
constexpr ClassTemplate kT_CLASS_D = {
  9.0f, 0, 0.3f, 5 /* k=5 dense */, 1.5f /* pvmap_sigma2=1.5 */,
  0.02f /* α=0.02 */, 0.3f, 0.0f, 50, 0.0f, 0.0f,
  true /* DDPO */, true /* CSCF */, true /* geom_cov */,
  0.5f, 20 /* pvmap_max_pts=20 */, 0.10f,
  0.25f, 4e-4f, 6, 0.1f,
  /*p1*/{}  // S13-R0.11.1: P1 OFF (H1 reverted, Smoke B Option B)
};

// HIGH_COS2 — VI04 (avia_v6_seq/varying_illu04.yaml)
constexpr ClassTemplate kT_HIGH_COS2 = {
  3.0f, 3, 0.3f, 9, 1.0f,
  0.145f, 0.3f, 0.2f, 0 /* freeze=0 */, 0.0f, 0.0f,
  true, false, false,
  0.5f, 20 /* R9: M permanent */, 0.10f,
  0.25f, 4e-4f, 6, 0.1f,
  /*p1*/{}  // S13-B.A.6: P1 OFF
};

// CORRIDOR — VI05 (avia_v6_seq/varying_illu05.yaml)
constexpr ClassTemplate kT_CORRIDOR = {
  3.0f, 0, 0.3f, 9, 1.0f,
  0.10f, 0.3f, 0.0f, 50, 0.0f, 0.0f,
  true, false, false,
  0.5f, 20 /* R9: M permanent */, 0.10f,
  0.25f, 4e-4f, 6, 0.1f,
  /*p1*/{}  // S13-B.A.6: P1 OFF
};

// IN_DEFAULT — avia_indoor.yaml unified consensus
// (l2=9, max_corr=3, plane=0.3, k=5, sigma2=2.0, alpha=0.10, floor=0.3, gate=0.2, freeze=50)
constexpr ClassTemplate kT_IN_DEFAULT = {
  9.0f, 3, 0.3f, 5, 2.0f,
  0.10f, 0.3f, 0.2f, 50, 0.0f, 0.0f,
  false /* DDPO off indoor */, false, false,
  0.5f, 20 /* R9: M permanent */, 0.10f,
  0.25f, 4e-4f, 6, 0.1f,
  /*p1*/{}  // S13-B.A.6: P1 OFF (indoor, avia_indoor.yaml doesn't set router_enable)
};

}  // namespace

const ClassTemplate& get_template(SceneClass cls) noexcept {
  switch (cls) {
    case SceneClass::CLEAN_DENSE:    return kT_CLEAN_DENSE;
    case SceneClass::OUTDOOR_DRIFT:  return kT_OUTDOOR_DRIFT;
    case SceneClass::CLEAN_OUT:      return kT_CLEAN_OUT;
    case SceneClass::DYNAMIC:        return kT_DYNAMIC;
    case SceneClass::OCCLUSION_PNCG: return kT_OCCLUSION_PNCG;
    case SceneClass::OCCLUSION:      return kT_OCCLUSION;
    case SceneClass::CLASS_D:        return kT_CLASS_D;
    case SceneClass::HIGH_COS2:      return kT_HIGH_COS2;
    case SceneClass::CORRIDOR:       return kT_CORRIDOR;
    case SceneClass::IN_DEFAULT:     return kT_IN_DEFAULT;
  }
  return kT_CLEAN_OUT;  // R10 fallback
}

}  // namespace core
}  // namespace tof_slam
