// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// surfel_map.cpp — Implementation of the hierarchical L0/L1 surfel map.

#include "tof_slam/frontend/map/surfel_map.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Eigenvalues>
#include <spdlog/spdlog.h>

#include "tof_slam/frontend/diag/boundary_hash.hpp"

namespace tof_slam {
namespace core {

// ---------------------------------------------------------------------------
// Constructor / lifecycle
// ---------------------------------------------------------------------------

SurfelMap::SurfelMap(const SurfelMapConfig& config)
    : config_(config),
      inv_l0_size_(1.0f / config.l0_voxel_size) {}

void SurfelMap::reset() {
  std::unique_lock<std::shared_mutex> lock(mutex_);
  l0_map_.clear();
  l1_map_.clear();
  l2_map_.clear();
  hit_set_.clear();
  map_center_      = Eigen::Vector3f::Zero();
  map_initialized_ = false;
}

// ---------------------------------------------------------------------------
// Key helpers
// ---------------------------------------------------------------------------

VoxelKey SurfelMap::l0_key(const Eigen::Vector3f& p) const {
  return point_to_voxel_key(p.x(), p.y(), p.z(), inv_l0_size_);
}

VoxelKey SurfelMap::l1_key(const Eigen::Vector3f& p) const {
  const float inv = inv_l0_size_ / static_cast<float>(config_.l1_hierarchy_factor);
  return point_to_voxel_key(p.x(), p.y(), p.z(), inv);
}

VoxelKey SurfelMap::l2_key(const Eigen::Vector3f& p) const {
  const float inv = inv_l0_size_ /
      static_cast<float>(config_.l1_hierarchy_factor * config_.l2_hierarchy_factor);
  return point_to_voxel_key(p.x(), p.y(), p.z(), inv);
}

VoxelKey SurfelMap::parent_key(const VoxelKey& k0) const {
  // Floor-division by hierarchy factor preserves negative-coordinate semantics.
  const int f = config_.l1_hierarchy_factor;
  auto floor_div = [&](int v) -> int {
    return (v >= 0) ? (v / f) : ((v - (f - 1)) / f);
  };
  return {floor_div(k0.x), floor_div(k0.y), floor_div(k0.z)};
}

VoxelKey SurfelMap::l1_to_l2_parent(const VoxelKey& k1) const {
  const int f = config_.l2_hierarchy_factor;
  auto floor_div = [&](int v) -> int {
    return (v >= 0) ? (v / f) : ((v - (f - 1)) / f);
  };
  return {floor_div(k1.x), floor_div(k1.y), floor_div(k1.z)};
}

// ---------------------------------------------------------------------------
// Hierarchy registration helpers
// ---------------------------------------------------------------------------

void SurfelMap::register_to_parent(const VoxelKey& k0) {
  // Called while mutex_ is already held by the caller (add_point).
  const VoxelKey kp = parent_key(k0);
  l1_map_[kp].children.insert(k0);
}

void SurfelMap::unregister_from_parent(const VoxelKey& k0) {
  // Called while mutex_ is already held.
  const VoxelKey kp = parent_key(k0);
  auto it = l1_map_.find(kp);
  if (it == l1_map_.end()) return;

  it->second.children.erase(k0);

  // Invalidate surfel when below minimum threshold.
  if (static_cast<int>(it->second.children.size()) < config_.min_l0_for_surfel) {
    it->second.has_surfel = false;
  }

  // Remove empty L1 voxel and unregister from L2.
  if (it->second.children.empty()) {
    if (config_.enable_l2_correspondences) {
      unregister_l1_from_l2(kp);
    }
    l1_map_.erase(it);
  }
}

// ---------------------------------------------------------------------------
// L2 hierarchy registration helpers
// ---------------------------------------------------------------------------

void SurfelMap::register_l1_to_l2(const VoxelKey& k1) {
  // Called while mutex_ is already held.
  // Only register if L2 is enabled and the L1 voxel has a valid surfel.
  if (!config_.enable_l2_correspondences) return;

  const VoxelKey k2 = l1_to_l2_parent(k1);
  L2Voxel& vx2 = l2_map_[k2];
  const bool was_empty = vx2.children.empty();
  vx2.children.insert(k1);
  if (!was_empty || vx2.children.size() == 1) {
    vx2.dirty = true;
  }
}

void SurfelMap::unregister_l1_from_l2(const VoxelKey& k1) {
  // Called while mutex_ is already held.
  if (!config_.enable_l2_correspondences) return;

  const VoxelKey k2 = l1_to_l2_parent(k1);
  auto it = l2_map_.find(k2);
  if (it == l2_map_.end()) return;

  it->second.children.erase(k1);

  if (static_cast<int>(it->second.children.size()) < config_.min_l1_for_l2_surfel) {
    it->second.has_surfel = false;
  }

  if (it->second.children.empty()) {
    l2_map_.erase(it);
  }
}

bool SurfelMap::recompute_l2_surfel(const VoxelKey& k2) {
  // Caller holds mutex_.
  auto it = l2_map_.find(k2);
  if (it == l2_map_.end()) return false;

  L2Voxel& vx2 = it->second;
  const int child_count = static_cast<int>(vx2.children.size());

  if (child_count < config_.min_l1_for_l2_surfel) {
    vx2.has_surfel = false;
    vx2.dirty      = false;
    return false;
  }

  // Collect L1 centroids in deterministic order (sorted by VoxelKey).
  std::vector<VoxelKey> sorted_children(vx2.children.begin(),
                                        vx2.children.end());
  std::sort(sorted_children.begin(), sorted_children.end());

  std::vector<Eigen::Vector3f> pts;
  pts.reserve(child_count);
  for (const VoxelKey& ck : sorted_children) {
    auto it1 = l1_map_.find(ck);
    if (it1 != l1_map_.end() && it1->second.has_surfel) {
      pts.push_back(it1->second.surfel_centroid);
    }
  }

  if (static_cast<int>(pts.size()) < config_.min_l1_for_l2_surfel) {
    vx2.has_surfel = false;
    vx2.dirty      = false;
    return false;
  }

  // PCA on L1 centroids.
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  for (const auto& p : pts) centroid += p;
  centroid /= static_cast<float>(pts.size());

  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  for (const auto& p : pts) {
    const Eigen::Vector3f d = p - centroid;
    cov += d * d.transpose();
  }
  cov /= static_cast<float>(pts.size());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov, Eigen::ComputeEigenvectors);
  const Eigen::Vector3f ev = eig.eigenvalues();  // ascending: ev(0) <= ev(1) <= ev(2)

  const float linearity = (ev(2) - ev(1)) / (ev(2) + 1e-6f);
  const float planarity = ev(0) / (ev(2) + 1e-6f);

  // L2 uses a more permissive planarity threshold (coarser scale).
  if (planarity > config_.l2_planarity_threshold || linearity > 0.95f) {
    vx2.has_surfel = false;
    vx2.dirty      = false;
    vx2.last_child_count = static_cast<int>(pts.size());
    return false;
  }

  const Eigen::Vector3f normal = eig.eigenvectors().col(0);
  const int n_pts = static_cast<int>(pts.size());
  const float normal_sigma2 =
      ev(0) / (static_cast<float>(n_pts) * ev(2) + 1e-8f);

  // S12-B.B.2 HS-A rank-3 pull-back DEACTIVATED per Rule 16 R2 fallback.
  //
  // Three implementation attempts (2026-05-12):
  //   v1 covariance pull-back (J·Σ·J^T):       Dark01 ATE +47% regression
  //   v2 info pull-back + scatter seed:        Dark01 ATE +9% regression
  //   v3 info pull-back pure (no scatter):     Dark01 ATE +19% regression
  //
  // All variants exceed architect R2 gate (>5% regression on HS-A-only),
  // confirming the predicted R2 risk: rank-3 propagation amplifies L1
  // covariance into L2 normal direction, distorting IEKF measurement weight
  // calibrated to scalar σ². Same lesson as Sprint 9 (VoxelMap σ²_normal):
  // math-correct → not tuning-compatible without re-calibrating IEKF.
  //
  // Architect §8 R2 mitigation: fall back to rank-1 form (existing scalar).
  // B.B.1 L1 surfel_cov storage retained (B.B.1 commit 306b248) — available
  // for future re-tuning or paper-only theoretical claim.
  //
  // To re-enable in future: revisit IEKF sigma2_normal calibration jointly
  // with HS-A pull-back (Sprint 13+ candidate).
  vx2.dirty            = false;
  vx2.has_surfel       = true;
  vx2.surfel_normal    = normal;
  vx2.surfel_centroid  = centroid;
  vx2.planarity_score  = planarity;
  vx2.normal_sigma2    = normal_sigma2;
  vx2.last_child_count = n_pts;

  // S13-B.A.4 P1: rank-3 L2 surfel covariance for anisotropic Ω_eff_L2.
  // Direct L2 PCA form (NOT the deactivated B.B.2 pull-back from L1):
  //   Σ_L2 = σ²_n_eff_L2 · (n·nᵀ) + σ²_t_eff_L2 · (I − n·nᵀ)
  // where σ²_n_eff_L2 = max(normal_sigma2, floor), σ²_t_eff_L2 = max(ev(0)/N, floor).
  // Storage is always-on (~36 bytes per L2 voxel, negligible cache impact);
  // use is gated downstream at iekf_updater.cpp:1005 by anisotropic_iekf_enable.
  // Mirrors the B.B.1 L1 pattern but storage is unconditional here because
  // the SurfelMapConfig does not carry the anisotropic_iekf_enable flag
  // (it lives on IekfConfig). Default-Zero downstream `has_surfel_cov_L2`
  // on Correspondence preserves S5/V5 bit-identical when flag=OFF.
  {
    const float floor_v = config_.hs_a_l1_sigma_floor;
    const float sigma2_n_eff_L2 = std::max(normal_sigma2, floor_v);
    const float sigma2_t_eff_L2 = std::max(
        ev(0) / static_cast<float>(n_pts), floor_v);
    const Eigen::Matrix3f I3 = Eigen::Matrix3f::Identity();
    const Eigen::Matrix3f n_outer = normal * normal.transpose();
    vx2.surfel_cov = sigma2_n_eff_L2 * n_outer
                   + sigma2_t_eff_L2 * (I3 - n_outer);
    vx2.has_surfel_cov = true;
  }
  return true;
}


// ---------------------------------------------------------------------------
// Point addition: TWO-FUNCTION DISPATCH
// ---------------------------------------------------------------------------
// CRITICAL INVARIANT: add_point() (non-degenerate path) must produce
// bit-identical codegen to the pre-Eq.(4) version. Any Eq.(4) logic MUST
// go exclusively in add_point_degen(). The two functions are compiled
// independently, so changes to one cannot disrupt the other.
//
// Background: VI03 has a two-attractor trajectory (0.6m good vs 0.99m bad).
// ANY perturbation to add_point()'s compiled code — even dead branches
// that are never taken — disrupts GCC's register allocation and pushes VI03
// into the bad attractor. VI04 requires inline Eq.(4) computation to avoid
// function-call-boundary FP changes that cause catastrophic divergence.
// The two-function approach satisfies both constraints simultaneously.
// ---------------------------------------------------------------------------

void SurfelMap::add_point(const Eigen::Vector3f& p) {
  const VoxelKey k0 = l0_key(p);
  const bool is_new = (l0_map_.find(k0) == l0_map_.end());

  L0Voxel& vx = l0_map_[k0];
  const int n  = vx.point_count;

  if (n == 0) {
    vx.centroid    = p;
    vx.point_count = 1;
    vx.last_obs_frame = current_frame_;
    if (config_.enable_geometric_covariance) {
      vx.points_sum = p;
      vx.cov_sum    = p * p.transpose();
    }
  } else {
    if (config_.ema_gate_radius > 0.0f) {
      const float dist = (p - vx.centroid).norm();
      if (dist > config_.ema_gate_radius) {
        return;
      }
    }

    const float alpha_running = 1.0f / static_cast<float>(n + 1);
    const float alpha = (config_.l0_ema_alpha_min > 0.0f)
                        ? std::max(alpha_running, config_.l0_ema_alpha_min)
                        : alpha_running;

    if (config_.l0_centroid_freeze_count > 0 &&
        n >= config_.l0_centroid_freeze_count) {
      vx.point_count = n + 1;
      vx.last_obs_frame = current_frame_;
    } else {
      vx.centroid    = vx.centroid * (1.0f - alpha) + p * alpha;
      vx.point_count = n + 1;
      vx.last_obs_frame = current_frame_;
      if (config_.enable_geometric_covariance) {
        vx.points_sum += p;
        vx.cov_sum.noalias() += p * p.transpose();
      }
    }
  }

  if (is_new) {
    register_to_parent(k0);
  }
}

// ---------------------------------------------------------------------------
// Eq.(4) path: direction-selective alpha modulation (degenerate frames only).
// This function replicates add_point()'s logic but computes per-voxel
// alpha_min using the parent L1 surfel normal and degeneracy directions.
// Compiled independently from add_point() — changes here do NOT affect
// add_point()'s codegen.
// ---------------------------------------------------------------------------

void SurfelMap::add_point_degen(const Eigen::Vector3f& p) {
  const VoxelKey k0 = l0_key(p);
  const bool is_new = (l0_map_.find(k0) == l0_map_.end());

  L0Voxel& vx = l0_map_[k0];
  const int n  = vx.point_count;

  if (n == 0) {
    vx.centroid    = p;
    vx.point_count = 1;
    vx.last_obs_frame = current_frame_;
    if (config_.enable_geometric_covariance) {
      vx.points_sum = p;
      vx.cov_sum    = p * p.transpose();
    }
  } else {
    if (config_.ema_gate_radius > 0.0f) {
      const float dist = (p - vx.centroid).norm();
      if (dist > config_.ema_gate_radius) {
        return;
      }
    }

    const float alpha_running = 1.0f / static_cast<float>(n + 1);

    // Eq.(4): direction-selective alpha modulation with severity scaling.
    // α_eff = α_min · max(floor, 1 − severity × max_d |n · d_deg|²)
    // severity = 1 / (1 + eigenvalue_ratio / ratio_ref)
    //   → outdoor (ratio~2e-4): severity ≈ 0.83 (near-current behaviour)
    //   → indoor  (ratio~2.6e-3): severity ≈ 0.28 (gentler modulation)
    float alpha_min_eff = config_.l0_ema_alpha_min;
    if (alpha_min_eff > 0.0f) {
      const VoxelKey k1 = parent_key(k0);
      const auto it = l1_map_.find(k1);
      if (it != l1_map_.end() && it->second.has_surfel) {
        const Eigen::Vector3f& n_surfel = it->second.surfel_normal;
        float max_cos2 = 0.0f;
        for (int d = 0; d < num_degen_dirs_; ++d) {
          const float dot = n_surfel.dot(degen_dirs_[d]);
          max_cos2 = std::max(max_cos2, dot * dot);
        }
        const float scaled_cos2 = degen_severity_ * max_cos2;
        const float mod_factor = (config_.alpha_degen_floor > 0.0f)
            ? std::max(config_.alpha_degen_floor, 1.0f - scaled_cos2)
            : (1.0f - scaled_cos2);
        alpha_min_eff = alpha_min_eff * mod_factor;
        // Accumulate cos² for Eq.(4) instrumentation (no behavioral change)
        cos2_sum_ += max_cos2;
        ++cos2_count_;
      }
    }

    const float alpha = (alpha_min_eff > 0.0f)
                        ? std::max(alpha_running, alpha_min_eff)
                        : alpha_running;

    if (config_.l0_centroid_freeze_count > 0 &&
        n >= config_.l0_centroid_freeze_count) {
      vx.point_count = n + 1;
      vx.last_obs_frame = current_frame_;
    } else {
      vx.centroid    = vx.centroid * (1.0f - alpha) + p * alpha;
      vx.point_count = n + 1;
      vx.last_obs_frame = current_frame_;
      if (config_.enable_geometric_covariance) {
        vx.points_sum += p;
        vx.cov_sum.noalias() += p * p.transpose();
      }
    }
  }

  if (is_new) {
    register_to_parent(k0);
  }
}

// ---------------------------------------------------------------------------
// Pruning
// ---------------------------------------------------------------------------

void SurfelMap::prune(const Eigen::Vector3f& center, float box_half) {
  // Collect L0 keys to remove (spatial pruning only).
  std::vector<VoxelKey> to_remove;
  to_remove.reserve(l0_map_.size() / 4);

  for (const auto& [k0, vx] : l0_map_) {
    // Spatial pruning: remove voxels outside the keep-box.
    const float cx = (static_cast<float>(k0.x) + 0.5f) * config_.l0_voxel_size;
    const float cy = (static_cast<float>(k0.y) + 0.5f) * config_.l0_voxel_size;
    const float cz = (static_cast<float>(k0.z) + 0.5f) * config_.l0_voxel_size;

    if (std::abs(cx - center.x()) > box_half ||
        std::abs(cy - center.y()) > box_half ||
        std::abs(cz - center.z()) > box_half) {
      to_remove.push_back(k0);
    }
  }

  for (const auto& k0 : to_remove) {
    unregister_from_parent(k0);
    l0_map_.erase(k0);
  }

  // Remove orphaned L1 voxels (unregister_from_parent already handles most,
  // but a direct erase may leave empty entries if children was already empty).
  std::vector<VoxelKey> l1_empty;
  for (const auto& [k1, vx1] : l1_map_) {
    if (vx1.children.empty()) l1_empty.push_back(k1);
  }
  for (const auto& k1 : l1_empty) l1_map_.erase(k1);

  // Remove orphaned L2 voxels (when L2 enabled).
  if (config_.enable_l2_correspondences) {
    std::vector<VoxelKey> l2_empty;
    for (const auto& [k2, vx2] : l2_map_) {
      if (vx2.children.empty()) l2_empty.push_back(k2);
    }
    for (const auto& k2 : l2_empty) l2_map_.erase(k2);
  }
}

// ---------------------------------------------------------------------------
// Surfel PCA computation
// ---------------------------------------------------------------------------

bool SurfelMap::recompute_surfel(const VoxelKey& k1) {
  // Caller holds mutex_.
  auto it = l1_map_.find(k1);
  if (it == l1_map_.end()) return false;

  L1Voxel& vx1 = it->second;
  const int child_count = static_cast<int>(vx1.children.size());

  if (child_count < config_.min_l0_for_surfel) {
    vx1.has_surfel = false;
    return false;
  }

  // Surfel Locking with Periodic Refresh + Per-L1 Adaptive Lock Override.
  //
  // When L0 topology (child count) is unchanged, freeze the surfel for
  // surfel_lock_frames to prevent short-term contamination cascades.
  // After the lock period expires, allow periodic refresh so that long
  // sequences can adapt to genuine geometry changes.
  //
  // Lock < 0: DISABLED (baseline behavior — always recompute when dirty).
  // Lock = 0: permanent lock (never refresh unless topology changes).
  // Lock > 0: lock for N frames, then periodic refresh.
  //
  // Per-L1 Adaptive Lock Override (lock_override):
  // Set during a periodic refresh when the new normal deviates from the
  // locked normal by more than adaptive_lock_cos_thresh.  While set, this
  // voxel always recomputes PCA every frame (topology-unchanged notwithstanding).
  // Cleared automatically when geometry stabilises at the next refresh.
  //
  // Avia (sparse 70° FOV): lock=50 recommended (prevents contamination).
  // Mid-360 (dense 360° FOV): lock=-1 recommended (frequent updates needed).
  if (config_.surfel_lock_frames >= 0 &&
      vx1.has_surfel && vx1.last_child_count == child_count &&
      !vx1.lock_override) {
    if (config_.surfel_lock_frames == 0 ||
        (current_frame_ - vx1.last_recompute_frame) < config_.surfel_lock_frames) {
      vx1.dirty = false;
      return true;  // Surfel is locked, no recomputation needed
    }
    // Lock expired: allow periodic refresh (fall through to PCA recomputation)
  }

  // Collect L0 centroids in deterministic order (sorted by VoxelKey).
  // unordered_dense::set iteration order is non-deterministic; sorting
  // ensures identical floating-point accumulation order across runs.
  std::vector<VoxelKey> sorted_children(vx1.children.begin(),
                                        vx1.children.end());
  std::sort(sorted_children.begin(), sorted_children.end());

  // Always use centroid-based PCA, even in ring buffer mode.
  // Ring buffer mode only changes HOW centroids are computed (finite-window
  // mean instead of EMA), not what PCA receives (still one centroid per L0).
  // This preserves PCA stability (~5 centroids) while bounding contamination.
  std::vector<Eigen::Vector3f> pts;
  pts.reserve(child_count);

  // Geometric covariance: aggregate raw-point running stats across L0 children.
  Eigen::Vector3f agg_points_sum = Eigen::Vector3f::Zero();
  Eigen::Matrix3f agg_cov_sum    = Eigen::Matrix3f::Zero();
  int             agg_total_pts  = 0;

  for (const VoxelKey& ck : sorted_children) {
    auto it0 = l0_map_.find(ck);
    if (it0 != l0_map_.end()) {
      pts.push_back(it0->second.centroid);
      // Accumulate raw-point statistics for geometric covariance.
      if (config_.enable_geometric_covariance && it0->second.point_count > 0) {
        agg_points_sum += it0->second.points_sum;
        agg_cov_sum.noalias() += it0->second.cov_sum;
        agg_total_pts += it0->second.point_count;
      }
    }
  }

  if (static_cast<int>(pts.size()) < config_.min_l0_for_surfel) {
    vx1.has_surfel = false;
    return false;
  }

  // Save previous surfel state before PCA (used for both scene transition
  // detection and per-L1 adaptive lock override).
  const bool had_surfel_before = vx1.has_surfel;
  Eigen::Vector3f old_normal = vx1.surfel_normal;

  // Standard PCA path.
  const bool result = compute_surfel_from_centroids(vx1, pts);

  // Geometric covariance: compute 3×3 observation covariance from aggregated
  // raw-point statistics (iG-LIO style). SVD-regularise and store inverse.
  if (config_.enable_geometric_covariance && result &&
      agg_total_pts >= config_.geometric_cov_min_points) {
    const Eigen::Vector3f agg_centroid =
        agg_points_sum / static_cast<float>(agg_total_pts);
    // Unbiased sample covariance: (cov_sum - N * mu * mu^T) / (N - 1)
    Eigen::Matrix3f agg_cov =
        (agg_cov_sum -
         static_cast<float>(agg_total_pts) * agg_centroid * agg_centroid.transpose()) /
        static_cast<float>(agg_total_pts - 1);

    // Eigenvalue decomposition for regularisation.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig_geo(
        agg_cov, Eigen::ComputeEigenvectors);
    Eigen::Vector3f ev_geo = eig_geo.eigenvalues();  // ascending
    const Eigen::Matrix3f& U = eig_geo.eigenvectors();

    // Data-driven: soft clamp eigenvalues to floor.
    const float ev_floor = config_.geometric_cov_min_eigenvalue;
    ev_geo(0) = std::max(ev_geo(0), ev_floor);
    ev_geo(1) = std::max(ev_geo(1), ev_floor);
    ev_geo(2) = std::max(ev_geo(2), ev_floor);

    // Reconstruct regularised covariance and precision.
    const Eigen::Matrix3f cov_reg = U * ev_geo.asDiagonal() * U.transpose();

    vx1.has_geometric_cov = true;
    vx1.geometric_cov     = cov_reg;
    vx1.geometric_cov_inv =
        U * ev_geo.cwiseInverse().asDiagonal() * U.transpose();
  } else {
    vx1.has_geometric_cov = false;
  }

  // Per-L1 Adaptive Surfel Lock: after every PCA recomputation (periodic
  // refresh or lock_override path), compare the new normal against the
  // previously locked normal.  If they differ by more than
  // adaptive_lock_cos_thresh, keep lock_override=true so this voxel
  // recomputes every frame until geometry stabilises.  Once stable,
  // clear lock_override so the normal lock re-engages.
  if (config_.surfel_lock_frames > 0 && result && had_surfel_before) {
    const float cos_angle = std::abs(vx1.surfel_normal.dot(old_normal));
    if (cos_angle < config_.adaptive_lock_cos_thresh) {
      vx1.lock_override = true;   // Normal shifted — keep recomputing
    } else {
      vx1.lock_override = false;  // Normal stable — re-engage lock
    }
  }

  // L2 dirty propagation: if L1 now has a valid surfel, ensure it is
  // registered with its L2 parent and mark L2 dirty.
  if (config_.enable_l2_correspondences) {
    if (vx1.has_surfel) {
      // Register L1 with its L2 parent (no-op if already registered).
      register_l1_to_l2(k1);
      // Mark L2 parent dirty for lazy recomputation.
      const VoxelKey k2 = l1_to_l2_parent(k1);
      auto it2 = l2_map_.find(k2);
      if (it2 != l2_map_.end()) {
        it2->second.dirty = true;
      }
    }
  }

  return result;
}

// ---------------------------------------------------------------------------
// Shared PCA surfel computation from a set of centroids
// ---------------------------------------------------------------------------

bool SurfelMap::compute_surfel_from_centroids(
    L1Voxel& vx1, const std::vector<Eigen::Vector3f>& pts) {
  // Compute centroid.
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  for (const auto& p : pts) centroid += p;
  centroid /= static_cast<float>(pts.size());

  // Compute 3×3 covariance.
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  for (const auto& p : pts) {
    const Eigen::Vector3f d = p - centroid;
    cov += d * d.transpose();
  }
  cov /= static_cast<float>(pts.size());

  // SelfAdjointEigenSolver — optimized for symmetric 3x3 (~2x faster than SVD).
  // Eigenvalues in ascending order: λ0 ≤ λ1 ≤ λ2.
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov,
      Eigen::ComputeEigenvectors);
  const Eigen::Vector3f ev = eig.eigenvalues();  // ascending: ev(0) ≤ ev(1) ≤ ev(2)

  // Linearity check: (λ_max - λ_mid) / (λ_max + ε).
  const float linearity = (ev(2) - ev(1)) / (ev(2) + 1e-6f);

  // Planarity ratio (0 = planar, 1 = isotropic): λ_min / λ_max
  const float planarity = ev(0) / (ev(2) + 1e-6f);

  // Reject surfel if not a valid plane or too linear.
  if (planarity > config_.planarity_threshold || linearity > 0.95f) {
    vx1.has_surfel = false;
    vx1.dirty = false;
    vx1.last_child_count = static_cast<int>(pts.size());
    return false;
  }

  // Normal = eigenvector for smallest eigenvalue (column 0, ascending order).
  const Eigen::Vector3f normal = eig.eigenvectors().col(0);

  // Phase E-1: Normal uncertainty from eigenvalue perturbation theory.
  //
  // Normal uncertainty: simplified from Anderson perturbation theory.
  // normal_sigma2 = λ_0 / (k · λ_2) — approximation that avoids the
  // 1/(λ_j - λ_0)² singularity when eigenvalues are close.
  // The full 2-term formula diverges near degenerate planes (λ_1 ≈ λ_0).
  const int n_pts = static_cast<int>(pts.size());
  const float normal_sigma2 =
      ev(0) / (static_cast<float>(n_pts) * ev(2) + 1e-8f);

  vx1.dirty                = false;
  vx1.has_surfel           = true;
  vx1.surfel_normal        = normal;
  vx1.surfel_centroid      = centroid;
  vx1.planarity_score      = planarity;
  vx1.normal_sigma2        = normal_sigma2;
  vx1.last_child_count     = static_cast<int>(pts.size());

  // S12-B.B.1 HS-A: rank-3 surfel covariance for cross-level info-filter pull-back.
  // PCA-derived anisotropic form:
  //   Σ_c = σ²_n_eff · (n·nᵀ) + σ²_t_eff · (I − n·nᵀ)
  // Verified PSD: both σ²_eff are non-negative; rank ≤ 3 by construction.
  // Strict rank-3 when σ²_t_eff > 0 (always true with σ²_floor > 0).
  // Default-OFF preserves S5 R9-C2++ bit-identical (vx1.has_surfel_cov stays false).
  if (config_.hs_a_enable_rank3) {
    const float floor_v = config_.hs_a_l1_sigma_floor;
    // Normal-direction variance (rank-1, along n).
    // Same form as normal_sigma2 ABOVE: λ₀/(N·λ₂).
    const float sigma2_n_eff = std::max(normal_sigma2, floor_v);
    // Tangent-plane variance (rank-2, on (I − n·nᵀ)).
    // λ₀/N is the in-plane scatter of points around centroid.
    const float sigma2_t_eff = std::max(
        ev(0) / static_cast<float>(n_pts), floor_v);
    const Eigen::Matrix3f I3 = Eigen::Matrix3f::Identity();
    const Eigen::Matrix3f n_outer = normal * normal.transpose();
    vx1.surfel_cov = sigma2_n_eff * n_outer
                   + sigma2_t_eff * (I3 - n_outer);
    vx1.has_surfel_cov = true;
  } else {
    vx1.has_surfel_cov = false;
  }
  vx1.last_recompute_frame = current_frame_;
  return true;
}

// ---------------------------------------------------------------------------
// Public: update
// ---------------------------------------------------------------------------

void SurfelMap::update(const PointCloud& cloud,
                       const Eigen::Vector3f& sensor_pos) {
  std::unique_lock<std::shared_mutex> lock(mutex_);

  if (cloud.empty()) return;

  ++current_frame_;

  // ------------------------------------------------------------------
  // 1. Initialise map centre on first call.
  // ------------------------------------------------------------------
  if (!map_initialized_) {
    map_center_      = sensor_pos;
    map_initialized_ = true;
  }

  // ------------------------------------------------------------------
  // 2. Re-centre if the sensor has moved too far from the box centre.
  // ------------------------------------------------------------------
  const float box_half        = config_.max_distance * config_.distance_multiplier;
  const float recenter_thresh = config_.max_distance / config_.distance_multiplier;

  if ((sensor_pos - map_center_).norm() > recenter_thresh) {
    map_center_ = sensor_pos;
    prune(map_center_, box_half);
  }

  // ------------------------------------------------------------------
  // 3. Insert all points (EMA centroid path).
  //    Two-function dispatch: add_point() for non-degenerate frames,
  //    add_point_degen() for degenerate frames (includes inline Eq.(4)).
  //    See function definitions above for rationale.
  // ------------------------------------------------------------------
  const bool do_pncg = (config_.pncg_threshold > 0.0f);
  const bool has_degen = (num_degen_dirs_ > 0);

  if (has_degen) {
    // Degenerate path: per-voxel Eq.(4) alpha modulation inside add_point_degen.
    for (const Point3D& pt : cloud) {
      const Eigen::Vector3f p = pt.to_eigen();
      if (do_pncg) {
        const VoxelKey k1 = l1_key(p);
        const auto it = l1_map_.find(k1);
        if (it != l1_map_.end() && it->second.has_surfel) {
          const float p2p = std::abs(it->second.surfel_normal.dot(
                                p - it->second.surfel_centroid));
          if (p2p > config_.pncg_threshold) {
            continue;
          }
        }
      }
      add_point_degen(p);
    }
  } else {
    // Non-degenerate path: clean add_point() with pre-Eq.(4) codegen.
    for (const Point3D& pt : cloud) {
      const Eigen::Vector3f p = pt.to_eigen();
      if (do_pncg) {
        const VoxelKey k1 = l1_key(p);
        const auto it = l1_map_.find(k1);
        if (it != l1_map_.end() && it->second.has_surfel) {
          const float p2p = std::abs(it->second.surfel_normal.dot(
                                p - it->second.surfel_centroid));
          if (p2p > config_.pncg_threshold) {
            continue;
          }
        }
      }
      add_point(p);
    }
  }

  // ------------------------------------------------------------------
  // 4. Mark affected L1 voxels as dirty (lazy surfel update).
  //    SVD recomputation is deferred to get_surfel() query time.
  //    This avoids computing surfels for voxels that are never queried.
  // ------------------------------------------------------------------
  ankerl::unordered_dense::set<VoxelKey, VoxelKeyHash> affected;
  affected.reserve(static_cast<size_t>(cloud.size()));
  for (const Point3D& pt : cloud) {
    affected.insert(l1_key(pt.to_eigen()));
  }

  // B9 — affected set iteration-order signature (Task #36 PV-4 localizer).
  // The writes (`dirty = true`) that follow are idempotent, so even an
  // order-varying iteration here produces the same end-state. This hash
  // is strictly observational — it detects whether iteration order over
  // `affected` is constant across runs.
  if (tof_slam::frontend::diag::BoundaryLogger::enabled()) {
    namespace diag = tof_slam::frontend::diag;
    std::uint64_t h = diag::kFnv1a64OffsetBasis;
    for (const VoxelKey& k1 : affected) {
      const int pack[3] = {k1.x, k1.y, k1.z};
      h = diag::fnv1a_64_update(
          h, diag::make_byte_view(pack, sizeof(pack)));
    }
    diag::BoundaryLogger::instance().log_precomputed(
        diag::current_frame_idx(),
        diag::BoundaryId::B9_AffectedOrder,
        h,
        static_cast<double>(affected.size()));
  }

  for (const VoxelKey& k1 : affected) {
    auto it = l1_map_.find(k1);
    if (it != l1_map_.end()) {
      it->second.dirty = true;
    }
  }

  // L2: Mark affected L2 parents as dirty when L2 is enabled.
  if (config_.enable_l2_correspondences) {
    for (const VoxelKey& k1 : affected) {
      const VoxelKey k2 = l1_to_l2_parent(k1);
      auto it2 = l2_map_.find(k2);
      if (it2 != l2_map_.end()) {
        it2->second.dirty = true;
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Public: flush_dirty
// ---------------------------------------------------------------------------

void SurfelMap::flush_dirty() {
  std::unique_lock<std::shared_mutex> lock(mutex_);

  // Flush L1 dirty voxels first (L2 depends on L1 centroids).
  // Sort dirty keys for deterministic recomputation order.
  std::vector<VoxelKey> dirty_keys;
  for (const auto& [k, vx1] : l1_map_) {
    if (vx1.dirty) dirty_keys.push_back(k);
  }
  std::sort(dirty_keys.begin(), dirty_keys.end());

  // B10 — dirty_keys order signature (Task #36 PV-4 localizer).
  // After std::sort, the order MUST be identical across runs assuming the
  // set of dirty keys is identical. A divergence here implicates the
  // upstream collection step (affected-set iteration) or the underlying
  // l1_map_ set of keys.
  if (tof_slam::frontend::diag::BoundaryLogger::enabled()) {
    namespace diag = tof_slam::frontend::diag;
    std::uint64_t h = diag::kFnv1a64OffsetBasis;
    for (const VoxelKey& k : dirty_keys) {
      const int pack[3] = {k.x, k.y, k.z};
      h = diag::fnv1a_64_update(
          h, diag::make_byte_view(pack, sizeof(pack)));
    }
    diag::BoundaryLogger::instance().log_precomputed(
        diag::current_frame_idx(),
        diag::BoundaryId::B10_SurfelDirtyOrder,
        h,
        static_cast<double>(dirty_keys.size()));
  }

  for (const VoxelKey& k : dirty_keys) {
    recompute_surfel(k);
  }

  // Flush L2 dirty voxels (only when enabled).
  if (config_.enable_l2_correspondences) {
    std::vector<VoxelKey> dirty_l2_keys;
    for (const auto& [k, vx2] : l2_map_) {
      if (vx2.dirty) dirty_l2_keys.push_back(k);
    }
    std::sort(dirty_l2_keys.begin(), dirty_l2_keys.end());
    for (const VoxelKey& k : dirty_l2_keys) {
      recompute_l2_surfel(k);
    }
  }
}

// ---------------------------------------------------------------------------
// R0.10 H4: rebuild_pre_lock_surfels — surgical pre-LOCK L0+L1/L2 re-derivation
// ---------------------------------------------------------------------------
// Prunes every L0Voxel observed at or before frame_threshold (i.e. the
// pre-LOCK [0..frame_threshold] window), then forces every surviving L1 (and
// L2 when enabled) parent of a pruned L0 to recompute its PCA-cached surfel
// next time flush_dirty() runs (option α dirty cascade).
//
// Order of operations (researcher §3.3.3 + Codex Round 2 §5.2):
//   1. Collect L0 kill keys, sort (determinism input).
//   2. Collect L1 (and L2) parent keys BEFORE Step 3 mutation, because
//      unregister_from_parent may erase the L1 voxel entirely when its
//      children become empty; we still need to mark surviving L1s dirty.
//   3. Prune L0 + clean L1 children-set via unregister_from_parent.
//   4. Mark surviving L1s dirty (idempotent; guard with find()).
//   5. Mark surviving L2s dirty (when enable_l2_correspondences).
//
// Determinism (I-2 / I-8): all unordered_map iterations are followed by a
// sort before mutation, so the eventual write order to dirty=true is
// deterministic across runs.  The dirty=true write itself is idempotent.
//
// Locking: takes unique_lock<mutex_> at entry. Internal helper
// unregister_from_parent is documented "Called while mutex_ is already held"
// — do NOT add internal locking inside the helper.
// ---------------------------------------------------------------------------
void SurfelMap::rebuild_pre_lock_surfels(int frame_threshold) {
  std::unique_lock<std::shared_mutex> lock(mutex_);

  // ---- Step 1: collect L0 kill set deterministically -----------------------
  std::vector<VoxelKey> kill_keys;
  kill_keys.reserve(l0_map_.size());
  for (const auto& [k, vx] : l0_map_) {
    if (vx.last_obs_frame >= 0 && vx.last_obs_frame <= frame_threshold) {
      kill_keys.push_back(k);
    }
  }
  std::sort(kill_keys.begin(), kill_keys.end());

  // ---- Step 2: collect L1 (and L2) parent keys BEFORE mutation -------------
  // We must collect parents BEFORE Step 3 because unregister_from_parent may
  // erase the L1 voxel entirely when its children become empty; we still want
  // to mark surviving L1s dirty (option α).
  ankerl::unordered_dense::set<VoxelKey, VoxelKeyHash> dirty_l1_set;
  ankerl::unordered_dense::set<VoxelKey, VoxelKeyHash> dirty_l2_set;
  for (const VoxelKey& k0 : kill_keys) {
    const VoxelKey kp = parent_key(k0);
    dirty_l1_set.insert(kp);
    if (config_.enable_l2_correspondences) {
      dirty_l2_set.insert(l1_to_l2_parent(kp));
    }
  }

  // ---- Step 3: prune L0 voxels (children-set cleanup happens in helper) ----
  for (const VoxelKey& k0 : kill_keys) {
    unregister_from_parent(k0);   // cleans L1 children; erases empty L1
    l0_map_.erase(k0);
  }

  // ---- Step 4: option α — mark surviving L1s dirty -------------------------
  // Surviving L1s with children.size() >= min_l0_for_surfel kept their cached
  // surfel from pre-LOCK PCA. Force flush_dirty() to recompute next frame.
  std::vector<VoxelKey> dirty_l1_sorted(dirty_l1_set.begin(), dirty_l1_set.end());
  std::sort(dirty_l1_sorted.begin(), dirty_l1_sorted.end());
  for (const VoxelKey& k1 : dirty_l1_sorted) {
    auto it = l1_map_.find(k1);
    if (it != l1_map_.end()) {
      it->second.dirty = true;
    }
  }

  // ---- Step 5: option α cascade — mark surviving L2s dirty -----------------
  if (config_.enable_l2_correspondences) {
    std::vector<VoxelKey> dirty_l2_sorted(dirty_l2_set.begin(), dirty_l2_set.end());
    std::sort(dirty_l2_sorted.begin(), dirty_l2_sorted.end());
    for (const VoxelKey& k2 : dirty_l2_sorted) {
      auto it = l2_map_.find(k2);
      if (it != l2_map_.end()) {
        it->second.dirty = true;
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Public: get_surfel
// ---------------------------------------------------------------------------

bool SurfelMap::get_surfel(const Eigen::Vector3f& point, Surfel* surfel) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);

  const VoxelKey k1 = l1_key(point);
  auto it = l1_map_.find(k1);
  if (it == l1_map_.end()) return false;

  // Lazy surfel recomputation: if dirty, recompute SVD on demand.
  // const_cast is safe here: mutex_ is already held, and the surfel cache
  // is logically const (same result, just deferred computation).
  if (it->second.dirty) {
    auto& self = const_cast<SurfelMap&>(*this);
    if (!self.recompute_surfel(k1)) {
      // recompute_surfel may erase the entry; re-check.
      return false;
    }
    // Re-find: recompute_surfel may have invalidated the iterator.
    it = l1_map_.find(k1);
    if (it == l1_map_.end()) return false;
  }

  const L1Voxel& vx1 = it->second;
  if (!vx1.has_surfel) return false;

  if (surfel) {
    surfel->centroid       = vx1.surfel_centroid;
    surfel->normal         = vx1.surfel_normal;
    surfel->planarity      = vx1.planarity_score;
    surfel->normal_sigma2  = vx1.normal_sigma2;
    surfel->valid          = true;

    // Surfel age penalty: inflate normal_sigma2 based on how many frames
    // since last PCA recomputation.  Stale surfels get higher uncertainty,
    // making PVMap more competitive in hybrid correspondence selection.
    if (config_.sigma2_age_scale > 0.0f) [[unlikely]] {
      const int age = current_frame_ - vx1.last_recompute_frame;
      surfel->normal_sigma2 *= (1.0f + config_.sigma2_age_scale *
                                static_cast<float>(age));
    }

    // Geometric covariance: copy precision matrix to output surfel.
    if (vx1.has_geometric_cov) {
      surfel->has_geometric_cov = true;
      surfel->geometric_cov_inv = vx1.geometric_cov_inv;
    }

    // S13-B.A.4: P1 anisotropic Σ_L1 export (from B.B.1 storage, when
    // hs_a_enable_rank3=true at recompute_surfel() time). Consumer in
    // iekf_updater gates use by anisotropic_iekf_enable.
    surfel->has_surfel_cov = vx1.has_surfel_cov;
    surfel->surfel_cov     = vx1.surfel_cov;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Public: get_l2_surfel
// ---------------------------------------------------------------------------

bool SurfelMap::get_l2_surfel(const Eigen::Vector3f& point, Surfel* surfel) const {
  if (!config_.enable_l2_correspondences) return false;

  std::shared_lock<std::shared_mutex> lock(mutex_);

  const VoxelKey k2 = l2_key(point);
  auto it = l2_map_.find(k2);
  if (it == l2_map_.end()) return false;

  // Lazy recomputation if dirty.
  if (it->second.dirty) {
    auto& self = const_cast<SurfelMap&>(*this);
    if (!self.recompute_l2_surfel(k2)) {
      return false;
    }
    it = l2_map_.find(k2);
    if (it == l2_map_.end()) return false;
  }

  const L2Voxel& vx2 = it->second;
  if (!vx2.has_surfel) return false;

  if (surfel) {
    surfel->centroid      = vx2.surfel_centroid;
    surfel->normal        = vx2.surfel_normal;
    surfel->planarity     = vx2.planarity_score;
    surfel->normal_sigma2 = vx2.normal_sigma2;
    surfel->valid         = true;
    // S13-B.A.4: P1 anisotropic Σ_L2 export. Always-on when populated
    // by recompute_l2_surfel(); consumer iekf_updater gates by its own
    // anisotropic_iekf_enable flag.
    surfel->has_surfel_cov = vx2.has_surfel_cov;
    surfel->surfel_cov     = vx2.surfel_cov;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Public: statistics
// ---------------------------------------------------------------------------

size_t SurfelMap::l0_count() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return l0_map_.size();
}

size_t SurfelMap::l1_count() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return l1_map_.size();
}

size_t SurfelMap::l2_count() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return l2_map_.size();
}

size_t SurfelMap::point_count() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  size_t total = 0;
  for (const auto& [k, vx] : l0_map_) total += static_cast<size_t>(vx.point_count);
  return total;
}

// ---------------------------------------------------------------------------
// Public: visualisation
// ---------------------------------------------------------------------------

std::vector<Surfel> SurfelMap::all_surfels() const {
  std::unique_lock<std::shared_mutex> lock(mutex_);

  // Flush all dirty voxels before collecting surfels.
  // const_cast is safe: mutex_ is held, and this is logically const.
  auto& self = const_cast<SurfelMap&>(*this);
  std::vector<VoxelKey> dirty_keys;
  for (const auto& [k, vx1] : l1_map_) {
    if (vx1.dirty) dirty_keys.push_back(k);
  }
  for (const VoxelKey& k : dirty_keys) {
    self.recompute_surfel(k);
  }

  std::vector<Surfel> out;
  out.reserve(l1_map_.size());
  for (const auto& [k, vx1] : l1_map_) {
    if (vx1.has_surfel) {
      Surfel s;
      s.centroid       = vx1.surfel_centroid;
      s.normal         = vx1.surfel_normal;
      s.planarity      = vx1.planarity_score;
      s.normal_sigma2  = vx1.normal_sigma2;
      s.valid          = true;
      out.push_back(s);
    }
  }
  return out;
}

// ---------------------------------------------------------------------------
// Public: hit tracking
// ---------------------------------------------------------------------------

void SurfelMap::clear_hits() {
  std::unique_lock<std::shared_mutex> lock(mutex_);
  hit_set_.clear();
}

void SurfelMap::mark_hit(const VoxelKey& l1_key) {
  std::unique_lock<std::shared_mutex> lock(mutex_);
  hit_set_[l1_key] = true;
}

bool SurfelMap::is_hit(const VoxelKey& l1_key) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return hit_set_.find(l1_key) != hit_set_.end();
}

// ---------------------------------------------------------------------------
// Public: gather_l0_neighbors (for L0-based per-query plane fitting)
// ---------------------------------------------------------------------------

int SurfelMap::gather_l0_neighbors(const Eigen::Vector3f& point,
                                    Eigen::Vector3f* out_centroids) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);

  const VoxelKey k0_center = l0_key(point);
  int count = 0;

  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        const VoxelKey k0(k0_center.x + dx, k0_center.y + dy,
                          k0_center.z + dz);
        auto it = l0_map_.find(k0);
        if (it != l0_map_.end() && it->second.point_count > 0) {
          out_centroids[count++] = it->second.centroid;
        }
      }
    }
  }
  return count;
}

// ---------------------------------------------------------------------------
// Public: gather_l0_neighbors_fast (center + 6 face-adjacent only)
// ---------------------------------------------------------------------------

int SurfelMap::gather_l0_neighbors_fast(const Eigen::Vector3f& point,
                                         Eigen::Vector3f* out_centroids) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);

  const VoxelKey k0_center = l0_key(point);
  int count = 0;

  // Center voxel
  {
    auto it = l0_map_.find(k0_center);
    if (it != l0_map_.end() && it->second.point_count > 0) {
      out_centroids[count++] = it->second.centroid;
    }
  }

  // 6 face-adjacent voxels (±x, ±y, ±z)
  static constexpr int dx[6] = {-1, 1, 0, 0, 0, 0};
  static constexpr int dy[6] = {0, 0, -1, 1, 0, 0};
  static constexpr int dz[6] = {0, 0, 0, 0, -1, 1};
  for (int i = 0; i < 6; ++i) {
    const VoxelKey k0(k0_center.x + dx[i], k0_center.y + dy[i],
                      k0_center.z + dz[i]);
    auto it = l0_map_.find(k0);
    if (it != l0_map_.end() && it->second.point_count > 0) {
      out_centroids[count++] = it->second.centroid;
    }
  }

  return count;
}

// ---------------------------------------------------------------------------
// Public: fit_plane_from_l0 (per-query L0 PCA for indoor accuracy)
// ---------------------------------------------------------------------------

Surfel SurfelMap::fit_plane_from_l0(const Eigen::Vector3f& query,
                                     float planarity_threshold) const {
  Surfel result;  // valid = false by default

  Eigen::Vector3f pts[7];
  const int n = gather_l0_neighbors_fast(query, pts);
  if (n < 3) return result;

  // Compute centroid.
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  for (int i = 0; i < n; ++i) {
    centroid += pts[i];
  }
  centroid /= static_cast<float>(n);

  // Compute 3x3 covariance matrix.
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  for (int i = 0; i < n; ++i) {
    const Eigen::Vector3f d = pts[i] - centroid;
    cov.noalias() += d * d.transpose();
  }
  cov /= static_cast<float>(n);

  // Eigendecomposition (eigenvalues ascending: λ₀ ≤ λ₁ ≤ λ₂).
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(
      cov, Eigen::ComputeEigenvectors);
  const float lambda_min = eig.eigenvalues()(0);
  const float lambda_max = eig.eigenvalues()(2);

  if (lambda_max < 1e-8f) return result;

  const float planarity = lambda_min / lambda_max;
  if (planarity > planarity_threshold) return result;

  // Normal = eigenvector of smallest eigenvalue.
  Eigen::Vector3f normal = eig.eigenvectors().col(0);

  // Orient normal toward the query point.
  if (normal.dot(query - centroid) < 0.0f) {
    normal = -normal;
  }

  result.centroid      = centroid;
  result.normal        = normal;
  result.planarity     = planarity;
  result.normal_sigma2 = lambda_min /
      (static_cast<float>(n) * lambda_max + 1e-10f);
  result.valid         = true;
  return result;
}

// ---------------------------------------------------------------------------
// Public: gather_l1_surfels (for CSCF kernel-weighted interpolation)
// ---------------------------------------------------------------------------

int SurfelMap::gather_l1_surfels(const Eigen::Vector3f& point,
                                  Surfel out_surfels[27],
                                  Eigen::Vector3f out_centroids[27]) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);

  const VoxelKey k1_center = l1_key(point);
  int count = 0;

  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        const int idx = (dx + 1) * 9 + (dy + 1) * 3 + (dz + 1);
        // Initialize as invalid.
        out_surfels[idx].valid = false;

        const VoxelKey k1(k1_center.x + dx, k1_center.y + dy,
                          k1_center.z + dz);
        auto it = l1_map_.find(k1);
        if (it == l1_map_.end()) continue;

        const L1Voxel& vx1 = it->second;
        if (!vx1.has_surfel) continue;

        out_surfels[idx].centroid      = vx1.surfel_centroid;
        out_surfels[idx].normal        = vx1.surfel_normal;
        out_surfels[idx].planarity     = vx1.planarity_score;
        out_surfels[idx].normal_sigma2 = vx1.normal_sigma2;
        out_surfels[idx].has_geometric_cov = vx1.has_geometric_cov;
        if (vx1.has_geometric_cov) {
          out_surfels[idx].geometric_cov_inv = vx1.geometric_cov_inv;
        }
        out_surfels[idx].valid         = true;
        out_centroids[idx]             = vx1.surfel_centroid;
        ++count;
      }
    }
  }
  return count;
}

}  // namespace core
}  // namespace tof_slam
