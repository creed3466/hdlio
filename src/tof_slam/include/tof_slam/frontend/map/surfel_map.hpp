// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// surfel_map.hpp — Hierarchical L0/L1 surfel map for IEKF correspondences.
//
// Clean-room redesign of iglio/VoxelMap into the core/ architecture.
// No ROS dependencies. Uses tof_slam::core types throughout.

#ifndef TOF_SLAM_FRONTEND_MAP_SURFEL_MAP_HPP_
#define TOF_SLAM_FRONTEND_MAP_SURFEL_MAP_HPP_

#include <mutex>
#include <shared_mutex>
#include <vector>

#include <Eigen/Dense>

#include "tof_slam/frontend/map/voxel_key.hpp"
#include "tof_slam/common/types/point_types.hpp"
#include "unordered_dense.h"

namespace tof_slam {
namespace core {

// ---------------------------------------------------------------------------
// SurfelMapConfig
// ---------------------------------------------------------------------------

/// Plain configuration struct — replaces the seven separate Set* calls in
/// the original VoxelMap API.
struct SurfelMapConfig {
  float l0_voxel_size = 0.1f;       ///< L0 leaf voxel size (metres)
  int   l1_hierarchy_factor = 3;    ///< L1 size = l0_voxel_size * factor
  float max_distance = 100.0f;      ///< Radius used for voxel pruning
  float planarity_threshold = 0.05f; ///< Max σ_min/σ_max for a valid surfel
  int   min_l0_for_surfel = 5;      ///< Min occupied L0 children for a valid surfel
  float distance_multiplier = 1.5f; ///< Recentering trigger ratio

  /// L0 centroid EMA minimum alpha.  Prevents ancient observations from
  /// being permanently locked in.  Effective window ≈ 1/alpha_min frames.
  /// 0 = disable EMA (use original running mean).  0.02 = ~50 frame window.
  float l0_ema_alpha_min = 0.0f;

  /// Surfel lock duration (frames). When topology (L0 child count) is
  /// unchanged, the L1 surfel is frozen for this many frames before allowing
  /// periodic refresh. This prevents short-term contamination cascades while
  /// allowing long-term geometry adaptation.
  /// <0 = DISABLED (baseline behavior — always recompute when dirty).
  ///  0 = permanent lock (never refresh unless topology changes).
  /// >0 = lock for N frames, then periodic refresh.
  /// Avia (sparse 70° FOV): 50 recommended. Mid-360 (dense 360°): -1.
  int surfel_lock_frames = 50;

  /// L2 multi-scale correspondence: enable 3rd hierarchy level.
  /// L2 voxel size = L1 size * l2_hierarchy_factor = 0.9m * 3 = 2.7m.
  /// L2 surfel is computed from L1 centroids (not L0).
  bool  enable_l2_correspondences = false;
  int   l2_hierarchy_factor = 3;
  float l2_planarity_threshold = 0.15f;  ///< More permissive than L1 (0.05)
  int   min_l1_for_l2_surfel = 4;         ///< Min L1 children for valid L2 surfel
  float l2_noise_scale = 9.0f;            ///< sigma^2_L2 = l2_noise_scale * sigma^2_L1


  /// Eq.(4) alpha modulation floor.  When Eq.(4) fires, per-voxel alpha is
  /// modulated by (1 − severity × |n · d_deg|²).  Without a floor, voxels
  /// whose normals align with d_deg can have alpha driven to near zero,
  /// effectively freezing their centroids.  The floor limits the minimum
  /// modulation factor:
  ///   alpha_eff = alpha_min * max(floor, 1 − severity × |n · d_deg|²)
  /// 0 = no floor (original behaviour, can freeze aligned voxels).
  /// Recommended: 0.3 (no voxel loses >70% of its alpha).
  float alpha_degen_floor = 0.0f;

  /// Eq.(4) severity scaling reference ratio.
  /// Severity = 1 / (1 + eigenvalue_ratio / ratio_ref).
  /// Controls automatic indoor/outdoor adaptation:
  ///   - Low ratio (outdoor ~2e-4): severity → 1 (full modulation, like current)
  ///   - High ratio (indoor ~2.6e-3): severity → 0.28 (gentler modulation)
  /// 0 = disabled (severity fixed at 1.0, legacy behaviour).
  /// Recommended: 1e-3 (sits between outdoor and indoor ratio distributions).
  float degen_severity_ratio_ref = 0.0f;

  // L0 Centroid Freeze: stop EMA centroid updates after a voxel has
  // accumulated this many observations.  After freeze, the centroid is
  // locked as a fixed reference — new points in the same L0 cell are
  // counted (point_count increments) but do NOT modify the centroid.
  // This breaks the EMA drift contamination feedback loop: once a
  // surface is well-established, the IEKF sees stable residuals that
  // reflect true drift rather than absorbed drift.
  // 0 = disabled (current behavior). Typical: 30-80 (iG-LIO uses 50).
  int l0_centroid_freeze_count = 0;

  // Per-L1 Adaptive Surfel Lock: during periodic refresh, compare new normal
  // with the previously locked normal.  If they differ by more than this
  // cosine threshold, disable locking for that voxel until geometry stabilises.
  // Default 0.85 ≈ 32°.  Only evaluated when surfel_lock_frames > 0.
  float adaptive_lock_cos_thresh = 0.85f;  ///< cos(angle) below which lock_override is set

  /// Spatial gate on L0 centroid EMA update.
  /// Rejects point insertion when ||p_new - centroid|| > ema_gate_radius.
  /// Prevents contamination from gross pose errors (e.g., corridor drift,
  /// initialization transient). Recommended: 2.0 * l0_voxel_size.
  /// 0 = disabled (current behavior, unconditional EMA update).
  float ema_gate_radius = 0.0f;

  /// Surfel normal_sigma2 age penalty: inflate normal uncertainty based on
  /// how many frames have elapsed since the surfel's last PCA recomputation.
  /// Applied at query time (get_surfel) to avoid modifying cached state.
  ///   sigma2_eff = sigma2 * (1 + sigma2_age_scale * age_frames)
  /// 0 = disabled (no age penalty). Canonical Dark02: 0.01.
  float sigma2_age_scale = 0.0f;

  /// Per-Voxel Normal Consistency Gate (PNCG): reject incoming points whose
  /// point-to-plane residual against the parent L1 surfel exceeds this
  /// threshold (metres). Prevents "wrong-side" points from contaminating
  /// L0 centroids when the incoming point is inconsistent with the
  /// established local plane geometry.
  ///   residual = |n · (p - c)|  where n = L1 surfel normal, c = L1 centroid
  /// Applied in update() before add_point(), only when the parent L1 has a
  /// valid surfel (has_surfel=true). New voxels with no surfel are unaffected.
  /// 0 = disabled (current behavior). Recommended: 0.10 (10cm).
  float pncg_threshold = 0.0f;

  /// Geometric Covariance (iG-LIO-style): accumulate per-L0 running sums
  /// (points_sum, cov_sum) and aggregate at L1 to compute a 3×3 observation
  /// covariance. SVD-regularised and inverted to give a per-surfel precision
  /// matrix Ω = Σ⁻¹ for Mahalanobis-weighted IEKF measurement updates.
  /// The covariance captures the true point-scatter distribution, including
  /// drift-induced spread, without depending on IEKF P (no circular dependency).
  /// When enabled, correspondences use ResidualMode::kPointToDistribution.
  /// false = disabled (current P2P behaviour).
  bool enable_geometric_covariance = false;

  /// Minimum eigenvalue for SVD regularisation of the geometric covariance.
  /// Clamps the smallest eigenvalue to this floor, preventing singular
  /// inversions and enforcing a minimum normal-direction noise floor.
  /// Recommended: lidar_noise_std² (e.g. 0.02² = 4e-4).
  float geometric_cov_min_eigenvalue = 4e-4f;

  /// Minimum total raw points across L0 children for valid geometric
  /// covariance. Below this, fall back to standard P2P with centroid PCA.
  int geometric_cov_min_points = 6;

  // S12-B.B.1 HS-A: Cross-Level Rank-3 Information Filter.
  /// Master toggle. When false (default): L1 surfel_cov NOT populated;
  /// recompute_l2_surfel() uses existing scalar normal_sigma2 path.
  /// When true: L1 surfel_cov populated rank-3 per PCA eigenstructure;
  /// B.B.2 recompute_l2_surfel pulls back via Jacobian and projects to scalar.
  bool hs_a_enable_rank3 = false;
  /// L1 PCA-derived variance floor (clamp σ²_n_eff and σ²_t_eff at L1 site).
  float hs_a_l1_sigma_floor = 1.0e-6f;
  /// S12-B.B.2 L2 SPD floor — ridge added to Σ_L2 before projection.
  /// Guards against rank-deficient pull-back when L1 children are coplanar.
  float hs_a_l2_spd_eps = 1.0e-9f;
};

// ---------------------------------------------------------------------------
// Surfel — output type returned by get_surfel()
// ---------------------------------------------------------------------------

struct Surfel {
  Eigen::Vector3f centroid  = Eigen::Vector3f::Zero();
  Eigen::Vector3f normal    = Eigen::Vector3f::Zero();
  float           planarity = 1.0f;   ///< 0 = perfectly planar, 1 = isotropic
  bool            valid     = false;

  /// Normal uncertainty from PCA eigenvalue perturbation theory.
  /// normal_sigma2 = λ₃ / (N * λ₁), where λ₁≥λ₂≥λ₃ are PCA eigenvalues
  /// and N is the number of L0 centroids used for PCA.
  /// Used to propagate surfel quality into IEKF measurement noise:
  ///   σ²_meas = σ²_lidar + ‖p_world - centroid‖² × normal_sigma2
  float           normal_sigma2 = 0.0f;

  /// Geometric covariance precision matrix Ω = Σ⁻¹ (SVD-regularised).
  /// When has_geometric_cov = true, correspondence should use P2D mode
  /// with this precision matrix for Mahalanobis-weighted IEKF updates.
  bool            has_geometric_cov = false;
  Eigen::Matrix3f geometric_cov_inv = Eigen::Matrix3f::Zero();

  /// S13-B.A.4: Rank-3 surfel covariance Σ (architect §3.1 P1 path).
  /// Populated by get_surfel() (L1) and get_l2_surfel() (L2) from the
  /// underlying L1Voxel.surfel_cov / L2Voxel.surfel_cov when those are
  /// populated. Consumed at iekf_updater.cpp L1 (:869) and L2 (:1005)
  /// weight sites when anisotropic_iekf_enable=true.
  bool            has_surfel_cov = false;
  Eigen::Matrix3f surfel_cov     = Eigen::Matrix3f::Zero();
};

// ---------------------------------------------------------------------------
// L2Voxel — region-level surfel (built from L1 centroids)
// ---------------------------------------------------------------------------

struct L2Voxel {
  ankerl::unordered_dense::set<VoxelKey, VoxelKeyHash> children;  ///< L1 keys
  bool            has_surfel        = false;
  bool            dirty             = false;
  Eigen::Vector3f surfel_normal     = Eigen::Vector3f::Zero();
  Eigen::Vector3f surfel_centroid   = Eigen::Vector3f::Zero();
  float           planarity_score   = 1.0f;
  float           normal_sigma2     = 0.0f;
  int             last_child_count  = 0;

  // --- S13-B.A.4 P1: L2 surfel covariance (rank-3 PCA-derived) ---
  // Populated by recompute_l2_surfel() when config.anisotropic_iekf_enable
  // is true. NOT the B.B.2 pull-back path (Σ_L1 → Σ_L2 via Jacobian) which
  // remains deactivated (architect §3.5 dead-code clause). Instead we use
  // direct L2 PCA eigenvalues, mirroring B.B.1's L1 formula:
  //   Σ_L2 = σ²_n_eff_L2·(n·nᵀ) + σ²_t_eff_L2·(I − n·nᵀ)
  // Default-Zero preserves S5/V5 bit-identical when flag=OFF.
  bool            has_surfel_cov    = false;
  Eigen::Matrix3f surfel_cov        = Eigen::Matrix3f::Zero();
};

// ---------------------------------------------------------------------------
// SurfelMap
// ---------------------------------------------------------------------------

class SurfelMap {
 public:
  explicit SurfelMap(const SurfelMapConfig& config = {});

  // ---- Main mutation -------------------------------------------------------

  /// Add all points in @p cloud and update affected L1 surfels.
  /// @p sensor_pos is used for map recentering / pruning.
  void update(const PointCloud& cloud, const Eigen::Vector3f& sensor_pos);

  // ---- Queries -------------------------------------------------------------

  /// Return the surfel for the L1 voxel that contains @p point.
  /// Returns false (and leaves *surfel unchanged) if no valid surfel exists.
  bool get_surfel(const Eigen::Vector3f& point, Surfel* surfel) const;

  /// Gather L0 centroids in a 3x3x3 neighborhood around @p point.
  /// Returns the number of centroids found (written to @p out_centroids).
  /// @p out_centroids must point to an array of at least 27 elements.
  /// This is the core primitive for L0-based per-query plane fitting.
  int gather_l0_neighbors(const Eigen::Vector3f& point,
                          Eigen::Vector3f* out_centroids) const;

  /// Fast variant: gather L0 centroids from center + 6 face-adjacent voxels.
  /// Only 7 hash lookups instead of 27. Returns 0-7 centroids.
  /// @p out_centroids must point to an array of at least 7 elements.
  int gather_l0_neighbors_fast(const Eigen::Vector3f& point,
                               Eigen::Vector3f* out_centroids) const;

  /// Per-query plane fitting from raw L0 centroids (fresh PCA, no EMA smoothing).
  /// Uses gather_l0_neighbors_fast (7 hash lookups). Returns Surfel with valid=false
  /// when fewer than 3 centroids found or planarity > planarity_threshold.
  /// Thread-safe: acquires shared_lock internally via gather_l0_neighbors_fast.
  Surfel fit_plane_from_l0(const Eigen::Vector3f& query,
                           float planarity_threshold = 0.3f) const;

  /// Gather L1 surfels from the 3x3x3 neighborhood of the query point.
  /// Returns the number of valid surfels found (up to 27).
  /// Used by CSCF for kernel-weighted correspondence interpolation.
  /// @p out_surfels and @p out_centroids must point to arrays of at least 27 elements.
  int gather_l1_surfels(const Eigen::Vector3f& point,
                        Surfel out_surfels[27],
                        Eigen::Vector3f out_centroids[27]) const;

  /// Return the L2 surfel for the region containing @p point.
  /// Returns false (and leaves *surfel unchanged) if no valid L2 surfel exists.
  /// Only active when config_.enable_l2_correspondences is true.
  bool get_l2_surfel(const Eigen::Vector3f& point, Surfel* surfel) const;

  // ---- Pre-flush for IEKF --------------------------------------------------

  /// Recompute all dirty L1 surfels eagerly (before IEKF correspondence
  /// queries).  After this call, get_surfel() never triggers lazy
  /// recomputation and can run without internal mutation, enabling faster
  /// sequential queries without unnecessary dirty-flag checks.
  void flush_dirty();

  // ---- Statistics ----------------------------------------------------------

  size_t l0_count()    const;
  size_t l1_count()    const;
  size_t l2_count()    const;
  size_t point_count() const;

  // ---- Visualisation -------------------------------------------------------

  /// Snapshot of all currently valid surfels (thread-safe copy).
  std::vector<Surfel> all_surfels() const;

  // ---- Hit tracking (used by IEKF to record which L1 voxels were queried) --

  void clear_hits();
  void mark_hit(const VoxelKey& l1_key);
  bool is_hit(const VoxelKey& l1_key) const;

  // ---- Lifecycle -----------------------------------------------------------

  const SurfelMapConfig& config() const { return config_; }
  int current_frame() const { return current_frame_; }

  /// Temporarily override L0 EMA alpha_min (for degeneracy-adaptive map).
  void set_l0_ema_alpha_min(float alpha) { config_.l0_ema_alpha_min = alpha; }

  // -------- R7 Scene-Classifier Setters (T5.4-R7 F.1, lock-time mutation) --------
  // All inline, mutate config_ in-place. Called from LioEstimator::feed_lidar
  // under its mutex lock once at classifier-lock frame. No mid-frame race.
  void set_alpha_degen_floor(float v)             { config_.alpha_degen_floor = v; }
  void set_ema_gate_radius(float v)               { config_.ema_gate_radius = v; }
  void set_l0_centroid_freeze_count(int v)        { config_.l0_centroid_freeze_count = v; }
  void set_sigma2_age_scale(float v)              { config_.sigma2_age_scale = v; }
  void set_pncg_threshold(float v)                { config_.pncg_threshold = v; }
  void set_l2_noise_scale(float v)                { config_.l2_noise_scale = v; }
  // CLASS_D-specific (VI03 template wires P2D / geometric covariance mode):
  void set_enable_geometric_covariance(bool v)    { config_.enable_geometric_covariance = v; }
  void set_geometric_cov_min_eigenvalue(float v)  { config_.geometric_cov_min_eigenvalue = v; }
  void set_geometric_cov_min_points(int v)        { config_.geometric_cov_min_points = v; }

  /// Set degeneracy state for direction-selective alpha modulation (Eq.4).
  /// Pass nullptr/0 when no degeneracy is detected.
  /// @param ev_ratio  eigenvalue ratio λ₀/λ₅ for severity scaling (0 = use 1.0)
  void set_degen_state(const Eigen::Vector3f* dirs, int num_dirs,
                       float ev_ratio = 0.0f) {
    num_degen_dirs_ = (dirs && num_dirs > 0) ? std::min(num_dirs, 3) : 0;
    for (int i = 0; i < num_degen_dirs_; ++i)
      degen_dirs_[i] = dirs[i];
    // Compute severity from eigenvalue ratio and configured reference.
    // severity = 1 / (1 + ratio / ratio_ref)
    // Correct for all ev_ratio >= 0: ratio=0 → severity=1.0 (maximally degenerate).
    // When ratio_ref = 0 (legacy/disabled): severity fixed at 1.0.
    if (config_.degen_severity_ratio_ref > 0.0f) {
      degen_severity_ = 1.0f / (1.0f + ev_ratio / config_.degen_severity_ratio_ref);
    } else {
      degen_severity_ = 1.0f;
    }
    // Reset per-frame cos² accumulator for Eq.(4) instrumentation
    cos2_sum_ = 0.0f;
    cos2_count_ = 0;
  }

  void reset();

  /// R0.10 H4: surgical pre-LOCK L0 + L1/L2 dirty re-derivation.
  /// Prunes L0 voxels with last_obs_frame <= frame_threshold, cascades the
  /// dirty flag onto surviving L1/L2 parents (option α, R0.10.1) so PCA
  /// recomputes against the post-prune children-set in flush_dirty().
  /// Caller must NOT hold mutex_; this method takes a unique_lock internally.
  void rebuild_pre_lock_surfels(int frame_threshold);

  /// Compute L1 voxel key for a world-frame point (public for deduplication).
  VoxelKey compute_l1_key(const Eigen::Vector3f& p) const { return l1_key(p); }

 private:
  // ---- Internal node types -------------------------------------------------

  /// L0 leaf: centroid accumulated via EMA.
  struct L0Voxel {
    Eigen::Vector3f centroid     = Eigen::Vector3f::Zero();
    int             point_count  = 0;
    int             last_obs_frame = 0;  ///< Frame counter for observation tracking

    // Geometric covariance running statistics (iG-LIO style).
    // Accumulated only when enable_geometric_covariance = true.
    // cov = (cov_sum - points_sum * centroid^T) / (point_count - 1)
    Eigen::Vector3f points_sum   = Eigen::Vector3f::Zero();
    Eigen::Matrix3f cov_sum      = Eigen::Matrix3f::Zero();  ///< Σ(p * p^T)
  };

  /// L1 parent: aggregates L0 children and caches PCA surfel.
  struct L1Voxel {
    ankerl::unordered_dense::set<VoxelKey, VoxelKeyHash> children;  ///< L0 keys

    // Cached surfel (recomputed when dirty)
    bool            has_surfel        = false;
    bool            dirty             = false;  ///< Surfel needs lazy recomputation
    Eigen::Vector3f surfel_normal     = Eigen::Vector3f::Zero();
    Eigen::Vector3f surfel_centroid   = Eigen::Vector3f::Zero();
    float           planarity_score   = 1.0f;
    float           normal_sigma2     = 0.0f;   ///< Phase E-1: normal uncertainty
    int             last_child_count  = 0;
    int             last_recompute_frame = 0;  ///< H1: frame at which surfel was last recomputed

    // Per-L1 Adaptive Surfel Lock: set true when periodic refresh detects
    // that the normal has rotated beyond adaptive_lock_cos_thresh.  While
    // true, this voxel always recomputes PCA (as if lock=-1) regardless of
    // the global surfel_lock_frames setting.  Cleared automatically once the
    // normal stabilises (cos angle >= threshold on the next refresh cycle).
    bool            lock_override        = false;  ///< Per-voxel lock bypass

    // Geometric covariance (computed from aggregated L0 running stats).
    // SVD-regularised 3×3 covariance and its inverse (precision matrix).
    // Only valid when has_geometric_cov = true.
    bool            has_geometric_cov    = false;
    Eigen::Matrix3f geometric_cov        = Eigen::Matrix3f::Zero();
    Eigen::Matrix3f geometric_cov_inv    = Eigen::Matrix3f::Zero();

    // S12-B.B.1 HS-A: Rank-3 surfel covariance for cross-level info-filter pull-back.
    // Built from PCA: Σ_c = σ²_n_eff·(n·nᵀ) + σ²_t_eff·(I − n·nᵀ)
    //   σ²_n_eff = max(λ₀/(N·λ₂), σ²_floor)  // normal-direction
    //   σ²_t_eff = max(λ₀/N, σ²_floor)        // tangent-plane direction
    // Populated only when SurfelMapConfig::hs_a_enable_rank3 = true.
    // Consumed by recompute_l2_surfel() (B.B.2) for rank-3 Jacobian pull-back.
    bool            has_surfel_cov       = false;
    Eigen::Matrix3f surfel_cov           = Eigen::Matrix3f::Zero();
  };

  // ---- Internal helpers ----------------------------------------------------

  VoxelKey l0_key(const Eigen::Vector3f& p) const;
  VoxelKey l1_key(const Eigen::Vector3f& p) const;
  VoxelKey l2_key(const Eigen::Vector3f& p) const;

  /// Compute L1 parent key from an L0 key using floor-division.
  VoxelKey parent_key(const VoxelKey& k0) const;

  /// Compute L2 parent key from an L1 key using floor-division.
  VoxelKey l1_to_l2_parent(const VoxelKey& k1) const;

  /// Insert a single point (non-degenerate path). This function's codegen
  /// MUST remain identical to the pre-Eq.(4) version — do NOT modify.
  void add_point(const Eigen::Vector3f& p);

  /// Insert a single point with inline Eq.(4) direction-selective alpha
  /// modulation (degenerate frames only). Compiled independently from
  /// add_point() so changes here cannot disrupt the non-degenerate path.
  void add_point_degen(const Eigen::Vector3f& p);

  void register_to_parent(const VoxelKey& k0);
  void unregister_from_parent(const VoxelKey& k0);

  /// Register an L1 key with its L2 parent (called when L1 first gets a surfel).
  void register_l1_to_l2(const VoxelKey& k1);
  /// Unregister an L1 key from its L2 parent (called when L1 becomes empty).
  void unregister_l1_from_l2(const VoxelKey& k1);
  /// Recompute the L2 surfel for a single L2 voxel from its L1 children.
  bool recompute_l2_surfel(const VoxelKey& k2);

  /// Remove L0 voxels whose centres lie outside the axis-aligned keep-box.
  void prune(const Eigen::Vector3f& center, float box_half);

  /// Recompute the PCA surfel for a single L1 voxel.  Returns true if a
  /// valid surfel was created, false if the voxel was discarded.
  bool recompute_surfel(const VoxelKey& k1);

  /// PCA surfel computation from a given set of centroids.
  bool compute_surfel_from_centroids(L1Voxel& vx1,
                                      const std::vector<Eigen::Vector3f>& pts);

  // ---- Data ----------------------------------------------------------------

  SurfelMapConfig config_;
  float inv_l0_size_ = 10.0f;  ///< 1 / l0_voxel_size (cached)

  ankerl::unordered_dense::map<VoxelKey, L0Voxel,  VoxelKeyHash> l0_map_;
  ankerl::unordered_dense::map<VoxelKey, L1Voxel,  VoxelKeyHash> l1_map_;
  ankerl::unordered_dense::map<VoxelKey, L2Voxel,  VoxelKeyHash> l2_map_;
  ankerl::unordered_dense::map<VoxelKey, bool,     VoxelKeyHash> hit_set_;

  Eigen::Vector3f map_center_       = Eigen::Vector3f::Zero();
  bool            map_initialized_  = false;
  int             current_frame_    = 0;       ///< Frame counter for L0 aging

  // Direction-selective alpha modulation (Eq.4): per-frame degeneracy state.
  Eigen::Vector3f degen_dirs_[3];
  int             num_degen_dirs_   = 0;
  float           degen_severity_   = 1.0f;  ///< Eq.(4) severity from ratio scaling

  // Eq.(4) per-frame cos² instrumentation (no behavioral change)
  float cos2_sum_  = 0.0f;
  int   cos2_count_ = 0;
 public:
  float cos2_mean() const { return cos2_count_ > 0 ? cos2_sum_ / cos2_count_ : 0.0f; }
  int   cos2_count() const { return cos2_count_; }
 private:

  mutable std::shared_mutex mutex_;
};

}  // namespace core
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_MAP_SURFEL_MAP_HPP_
