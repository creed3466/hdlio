// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// correspondence_finder.cpp — Find point-to-plane correspondences.

#include "tof_slam/frontend/estimator/correspondence_finder.hpp"

#include <cmath>
#include <limits>

#include <Eigen/Eigenvalues>
#include <omp.h>
#include <spdlog/spdlog.h>

#include "tof_slam/frontend/diag/boundary_hash.hpp"
#include "tof_slam/frontend/map/voxel_key.hpp"
#include "unordered_dense.h"

namespace tof_slam {
namespace core {

// ---------------------------------------------------------------------------
// Per-Correspondence Hybrid Surfel-PVMap (H-Corr / kHybridSelect)
// ---------------------------------------------------------------------------

std::vector<Correspondence> find_correspondences_hybrid_select(
    const LioState& state,
    const Se3& T_body_lidar,
    const PointCloud& scan,
    const SurfelMap& surfel_map,
    const PointVoxelMap& pvmap,
    const HybridCorrespondenceParams& params) {
  // Unpack params for backward compatibility with existing body code.
  const float max_plane_distance = params.max_plane_distance;
  const int pvmap_k_neighbors = params.pvmap_k_neighbors;
  const float pvmap_planarity_threshold = params.pvmap_planarity_threshold;
  const int max_corr_per_l1 = params.max_corr_per_l1;
  const float pvmap_sigma2_scale = params.pvmap_sigma2_scale;
  HybridStats* out_stats = params.out_stats;
  const Eigen::Vector3f* degen_trans_dirs = params.degen_trans_dirs;
  const int num_degen_trans_dirs = params.num_degen_trans_dirs;
  const float degen_pvmap_cos_threshold = params.degen_pvmap_cos_threshold;
  std::vector<Correspondence> correspondences;

  if (scan.empty()) {
    return correspondences;
  }

  // Precompute combined transform: T_wl = T_wb * T_bl
  const Se3 T_wb = state.pose();
  const Se3 T_wl = T_wb * T_body_lidar;
  const Eigen::Matrix3f R_wl = T_wl.rotation().matrix();
  const Eigen::Vector3f t_wl = T_wl.translation();
  const Eigen::Vector3f sensor_origin_world = t_wl;

  correspondences.reserve(scan.size());

  // B6 — Scan input signature (Task #36 PV-4 ordering localizer).
  if (tof_slam::frontend::diag::BoundaryLogger::enabled()) {
    namespace diag = tof_slam::frontend::diag;
    std::uint64_t h = diag::kFnv1a64OffsetBasis;
    for (const auto& pt : scan) {
      const float p[3] = {pt.x, pt.y, pt.z};
      h = diag::fnv1a_64_update(
          h, diag::make_byte_view(p, sizeof(p)));
    }
    diag::BoundaryLogger::instance().log_precomputed(
        diag::current_frame_idx(),
        diag::BoundaryId::B6_ScanForCorr,
        h,
        static_cast<double>(scan.size()));
  }

  // kNN search radius: (2H+1) x pvmap voxel size, matching the search neighborhood.
  const float knn_max_dist =
      pvmap.config().voxel_size * static_cast<float>(2 * pvmap.config().knn_search_half + 1);

  const int scan_size = static_cast<int>(scan.size());

  // Minimum scan size threshold for OpenMP parallelization.
  // Below this, fork/join overhead exceeds the benefit.
  const bool use_parallel = (scan_size >= 200);

  // =======================================================================
  // Option C: Voxel Plane Cache path — multi-voxel correspondences.
  // When use_voxel_plane_cache is active, each scan point queries up to
  // (2H+1)^3 neighbor voxels and gets one correspondence per voxel.
  // This provides normal diversity critical for degenerate environments.
  // =======================================================================
  const bool use_voxel_plane_cache = pvmap.config().use_voxel_plane_cache;

  if (use_voxel_plane_cache) {
    // GICP-style voxel Gaussian correspondence: NEARBY_7 search
    // (center voxel + 6 face-adjacent). Take the first valid voxel
    // whose chi² < threshold (iG-LIO approach). Falls back to closest
    // plane if P2D is disabled.
    //
    // NEARBY_7 offsets: center, ±x, ±y, ±z
    static constexpr int nearby7[7][3] = {
        {0,0,0}, {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}
    };

    PlaneResult vp_buf[27];  // max for H=1
    ankerl::unordered_dense::map<VoxelKey, int, VoxelKeyHash> l1_count;

    for (const auto& pt : scan) {
      const Eigen::Vector3f p_lidar = pt.to_eigen();
      const Eigen::Vector3f p_world = R_wl * p_lidar + t_wl;
      const Eigen::Vector3f ray = p_world - sensor_origin_world;
      const float range = p_lidar.norm();

      const VoxelKey k1 = surfel_map.compute_l1_key(p_world);

      if (max_corr_per_l1 > 0) {
        auto [it, inserted] = l1_count.try_emplace(k1, 0);
        if (++(it->second) > max_corr_per_l1) {
          continue;
        }
      }

      // Adaptive residual threshold.
      const float adaptive_thresh = (max_plane_distance > 0.0f)
          ? std::min(max_plane_distance,
                     std::sqrt(std::max(range, 0.1f)) / params.adaptive_threshold_divisor)
          : 0.3f;

      // NEARBY_7: probe center + 6 face-adjacent voxels (iG-LIO strategy).
      // Pass search_half_override=1 to force H=1 (3×3×3=27 voxel search)
      // regardless of config_.knn_search_half. max_results=7 caps output
      // to the most relevant voxels.
      const int n_planes = pvmap.get_voxel_planes(
          p_world, vp_buf, 7,
          pvmap_planarity_threshold, adaptive_thresh,
          /*search_half_override=*/1);

      if (n_planes == 0) continue;

      // Select the best plane: for P2D, prefer lowest Mahalanobis distance.
      // For P2P, prefer lowest point-to-plane distance.
      int best_idx = 0;
      float best_dist = std::numeric_limits<float>::max();
      for (int vi = 0; vi < n_planes; ++vi) {
        float d;
        if (params.enable_point_to_distribution && vp_buf[vi].n_points >= 5) {
          // Mahalanobis distance using SVD-regularized covariance
          const Eigen::Vector3f r3 = p_world - vp_buf[vi].centroid;
          const Eigen::Matrix3f cov_reg =
              vp_buf[vi].cov + params.p2d_cov_reg_eps * Eigen::Matrix3f::Identity();
          d = r3.transpose() * cov_reg.inverse() * r3;  // chi² score
        } else {
          d = std::abs(
              vp_buf[vi].normal.dot(p_world - vp_buf[vi].centroid));
        }
        if (d < best_dist) {
          best_dist = d;
          best_idx = vi;
        }
      }
      const PlaneResult& plane = vp_buf[best_idx];

      Correspondence corr;
      corr.p_lidar       = p_lidar;
      corr.normal        = plane.normal;
      corr.plane_d       = plane.normal.dot(plane.centroid);
      corr.planarity     = plane.planarity;
      corr.centroid      = plane.centroid;
      corr.normal_sigma2 = plane.normal_sigma2;
      corr.range         = range;
      corr.l1_key        = k1;

      if (range > 1e-6f) {
        corr.cos_incidence = std::abs(ray.dot(corr.normal)) / range;
      } else {
        corr.cos_incidence = 1.0f;
      }

      // P2D: set residual mode and compute Ω = (Σ + εI)^{-1}
      if (params.enable_point_to_distribution && plane.n_points >= 5) {
        // plane.cov stores the 3×3 covariance from PCA.
        // Regularize and invert.
        const Eigen::Matrix3f cov_reg =
            plane.cov + params.p2d_cov_reg_eps * Eigen::Matrix3f::Identity();
        const Eigen::Matrix3f omega = cov_reg.inverse();

        // Chi² outlier gate: r^T Ω r > threshold → skip
        const Eigen::Vector3f r3 = p_world - plane.centroid;
        const float maha_sq = r3.transpose() * omega * r3;
        if (maha_sq > params.p2d_chi2_threshold) {
          continue;  // outlier
        }

        corr.residual_mode = ResidualMode::kPointToDistribution;
        corr.voxel_cov_inv = omega;
      }

      correspondences.push_back(corr);
    }

    if (out_stats) {
      out_stats->n_pvmap_only = static_cast<int>(correspondences.size());
    }
    return correspondences;
  }

  if (!use_parallel) {
    // =====================================================================
    // Sequential path (small scans or single-thread fallback)
    // =====================================================================
    ankerl::unordered_dense::map<VoxelKey, int, VoxelKeyHash> l1_count;
    int point_idx = 0;

    for (const auto& pt : scan) {
      const Eigen::Vector3f p_lidar = pt.to_eigen();
      const Eigen::Vector3f p_world = R_wl * p_lidar + t_wl;

      const VoxelKey k1 = surfel_map.compute_l1_key(p_world);

      if (max_corr_per_l1 > 0) {
        auto [it, inserted] = l1_count.try_emplace(k1, 0);
        if (++(it->second) > max_corr_per_l1) {
          ++point_idx;
          continue;
        }
      }

      Surfel surfel;
      bool surfel_valid = false;
      if (params.enable_cscf) {
        // CSCF: Kernel-weighted L1 surfel interpolation.
        Surfel l1_surfels[27];
        Eigen::Vector3f l1_centroids[27];
        const int n_surfels = surfel_map.gather_l1_surfels(p_world, l1_surfels, l1_centroids);

        if (n_surfels >= 1) {
          const float sigma2 = params.cscf_kernel_bandwidth * params.cscf_kernel_bandwidth;
          Eigen::Vector3f n_weighted = Eigen::Vector3f::Zero();
          Eigen::Vector3f c_weighted = Eigen::Vector3f::Zero();
          float w_total = 0.0f;
          float sigma2_weighted = 0.0f;
          float planarity_min = 1.0f;
          // Track geometric covariance from highest-weight neighbor.
          // Use Zero() default — Identity() would be a wrong precision matrix
          // if it ever leaked through the has_geometric_cov guard.
          bool best_geo_cov_valid = false;
          Eigen::Matrix3f best_geo_cov_inv = Eigen::Matrix3f::Zero();
          float best_geo_w = 0.0f;

          for (int si = 0; si < 27; ++si) {
            if (!l1_surfels[si].valid) continue;
            const float dist_sq = (p_world - l1_centroids[si]).squaredNorm();
            const float w = std::exp(-dist_sq / (2.0f * sigma2));
            if (w < 1e-6f) continue;  // Skip negligible weights

            n_weighted += w * l1_surfels[si].normal;
            c_weighted += w * l1_surfels[si].centroid;
            w_total += w;
            sigma2_weighted += w * l1_surfels[si].normal_sigma2;
            planarity_min = std::min(planarity_min, l1_surfels[si].planarity);
            if (l1_surfels[si].has_geometric_cov && w > best_geo_w) {
              best_geo_cov_valid = true;
              best_geo_cov_inv = l1_surfels[si].geometric_cov_inv;
              best_geo_w = w;
            }
          }

          if (w_total > 1e-8f) {
            const float inv_w = 1.0f / w_total;
            Eigen::Vector3f n_interp = n_weighted * inv_w;
            const float n_norm = n_interp.norm();

            // Gate: reject if normals cancel out (opposing normals at thin walls)
            if (n_norm > 0.3f) {
              surfel.normal = n_interp / n_norm;  // Normalize
              surfel.centroid = c_weighted * inv_w;
              surfel.normal_sigma2 = sigma2_weighted * inv_w;
              surfel.planarity = planarity_min;
              surfel.has_geometric_cov = best_geo_cov_valid;
              if (best_geo_cov_valid) {
                surfel.geometric_cov_inv = best_geo_cov_inv;
              }
              surfel.valid = true;
              surfel_valid = true;
            }
          }
        }
        if (!surfel_valid) {
          // Fallback to discrete lookup
          surfel_valid = surfel_map.get_surfel(p_world, &surfel) &&
                         surfel.valid;
        }
      } else {
        surfel_valid = surfel_map.get_surfel(p_world, &surfel) &&
                       surfel.valid;
      }

      PlaneResult plane;
      if (params.knn_cache) {
        auto& entry = (*params.knn_cache)[point_idx];

        if (params.knn_cache_is_populated) {
          const float delta2 = (p_world - entry.p_world_cached).squaredNorm();
          const float threshold = pvmap.config().voxel_size * 0.4f;

          if (delta2 < threshold * threshold && entry.n_gathered > 0) {
            plane = pvmap.fit_plane_from_neighbors(
                p_world, entry.neighbors, entry.n_gathered,
                pvmap_planarity_threshold, knn_max_dist);
          } else {
            entry.n_gathered = pvmap.gather_knn(
                p_world, std::min(pvmap_k_neighbors, 30), entry.neighbors, nullptr);
            entry.p_world_cached = p_world;
            if (entry.n_gathered > 0) {
              plane = pvmap.fit_plane_from_neighbors(
                  p_world, entry.neighbors, entry.n_gathered,
                  pvmap_planarity_threshold, knn_max_dist);
            }
          }
        } else {
          entry.n_gathered = pvmap.gather_knn(
              p_world, std::min(pvmap_k_neighbors, 30), entry.neighbors, nullptr);
          entry.p_world_cached = p_world;
          if (entry.n_gathered > 0) {
            plane = pvmap.fit_plane_from_neighbors(
                p_world, entry.neighbors, entry.n_gathered,
                pvmap_planarity_threshold, knn_max_dist);
          }
        }
      } else {
        plane = pvmap.fit_plane_knn(p_world, pvmap_k_neighbors,
                                    pvmap_planarity_threshold, knn_max_dist);
      }
      bool pvmap_valid = plane.valid;

      if (pvmap_valid) {
        const Eigen::Vector3f ray_orient = p_world - sensor_origin_world;
        if (plane.normal.dot(ray_orient) > 0.0f) {
          plane.normal = -plane.normal;
        }
        const float pvmap_plane_dist =
            std::abs(plane.normal.dot(p_world - plane.centroid));

        const float range = p_lidar.norm();
        const float adaptive_thresh = std::min(
            max_plane_distance,
            std::sqrt(std::max(range, 0.1f)) / params.adaptive_threshold_divisor);
        if (max_plane_distance > 0.0f && pvmap_plane_dist > adaptive_thresh) {
          pvmap_valid = false;
        }
      }

      bool surfel_dist_ok = true;
      if (surfel_valid) {
        const float surfel_plane_dist =
            std::abs(surfel.normal.dot(p_world - surfel.centroid));
        const float range_s = p_lidar.norm();
        const float adaptive_thresh_s = std::min(
            max_plane_distance,
            std::sqrt(std::max(range_s, 0.1f)) / params.adaptive_threshold_divisor);
        if (max_plane_distance > 0.0f && surfel_plane_dist > adaptive_thresh_s) {
          surfel_dist_ok = false;
        }
      }
      const bool use_surfel_src = surfel_valid && surfel_dist_ok;

      if (!use_surfel_src && !pvmap_valid) {
        ++point_idx;
        continue;
      }

      bool use_surfel;
      float cos_nn = 1.0f;
      if (use_surfel_src && !pvmap_valid) {
        use_surfel = true;
        if (out_stats) { out_stats->n_surfel_only++; }
      } else if (!use_surfel_src && pvmap_valid) {
        use_surfel = false;
        if (out_stats) { out_stats->n_pvmap_only++; }
      } else {
        cos_nn = std::abs(surfel.normal.dot(plane.normal));

        if (out_stats) {
          out_stats->n_dual++;
          out_stats->sum_cos_nn += cos_nn;
        }

        bool degen_forced_pvmap = false;
        if (num_degen_trans_dirs > 0 && degen_trans_dirs != nullptr) {
          float max_cos_align = 0.0f;
          for (int d = 0; d < num_degen_trans_dirs; ++d) {
            const float cos_align = std::abs(surfel.normal.dot(degen_trans_dirs[d]));
            max_cos_align = std::max(max_cos_align, cos_align);
          }
          if (max_cos_align >= degen_pvmap_cos_threshold) {
            degen_forced_pvmap = true;
            if (out_stats) { out_stats->n_degen_override++; }
          }
        }

        if (degen_forced_pvmap) {
          use_surfel = false;
        } else {
          const float surfel_eff_sigma2 = surfel.normal_sigma2;
          if (cos_nn >= 0.7f) {
            const float pvmap_adj_sigma2 = plane.normal_sigma2 * pvmap_sigma2_scale;
            use_surfel = (surfel_eff_sigma2 <= pvmap_adj_sigma2);
          } else {
            // Disagree branch: normals differ significantly → always prefer surfel.
            use_surfel = true;
          }
        }
      }

      Correspondence corr;
      corr.p_lidar = p_lidar;

      const Eigen::Vector3f ray = p_world - sensor_origin_world;
      corr.range = ray.norm();

      if (use_surfel) {
        corr.normal        = surfel.normal;
        corr.plane_d       = surfel.normal.dot(surfel.centroid);
        corr.planarity     = surfel.planarity;
        corr.centroid      = surfel.centroid;
        corr.normal_sigma2 = surfel.normal_sigma2;
        // S13-B.A.4: copy Σ_L1 from B.B.1 storage (populated only when
        // hs_a_enable_rank3=true; default false → fields stay zero).
        corr.has_surfel_cov_L1 = surfel.has_surfel_cov;
        corr.surfel_cov_L1     = surfel.surfel_cov;
      } else {
        corr.normal        = plane.normal;
        corr.plane_d       = plane.normal.dot(plane.centroid);
        corr.planarity     = plane.planarity;
        corr.centroid      = plane.centroid;
        corr.normal_sigma2 = plane.normal_sigma2;
        // Plane (PVMap) source: no surfel-cov tag.
        corr.has_surfel_cov_L1 = false;
      }

      if (use_surfel_src && pvmap_valid) {
        corr.normal_agreement = cos_nn;
      }

      // Geometric covariance → P2D mode: when surfel has a valid precision
      // matrix, switch this correspondence to Mahalanobis-weighted P2D.
      if (use_surfel && surfel.has_geometric_cov) {
        corr.residual_mode = ResidualMode::kPointToDistribution;
        corr.voxel_cov_inv = surfel.geometric_cov_inv;
      }

      if (corr.range > 1e-6f) {
        corr.cos_incidence = std::abs(ray.dot(corr.normal)) / corr.range;
      } else {
        corr.cos_incidence = 1.0f;
      }

      // L2 supplement.
      if (surfel_map.config().enable_l2_correspondences) {
        Surfel l2_surfel;
        if (surfel_map.get_l2_surfel(p_world, &l2_surfel)) {
          const float l2_plane_dist =
              std::abs(l2_surfel.normal.dot(p_world - l2_surfel.centroid));
          if (max_plane_distance <= 0.0f ||
              l2_plane_dist < max_plane_distance * 3.0f) {
            Eigen::Vector3f l2_n = l2_surfel.normal;
            if (l2_n.dot(p_world - sensor_origin_world) > 0.0f) {
              l2_n = -l2_n;
            }
            const float l1_l2_cos = use_surfel_src
                ? std::abs(surfel.normal.dot(l2_n))
                : std::abs(corr.normal.dot(l2_n));
            const bool gate1_pass = (l1_l2_cos >= 0.866f);
            const bool gate2_pass = (l2_plane_dist < 0.3f);
            if (gate1_pass && gate2_pass) {
              corr.has_l2           = true;
              corr.l2_normal        = l2_n;
              corr.l2_plane_d       = l2_n.dot(l2_surfel.centroid);
              corr.l2_centroid      = l2_surfel.centroid;
              corr.l2_normal_sigma2 = l2_surfel.normal_sigma2;
              corr.l2_planarity     = l2_surfel.planarity;
              // S13-B.A.4: copy Σ_L2 rank-3 covariance (always populated by
              // recompute_l2_surfel; consumer gated by anisotropic_iekf_enable).
              corr.has_surfel_cov_L2 = l2_surfel.has_surfel_cov;
              corr.surfel_cov_L2     = l2_surfel.surfel_cov;
            }
          }
        }
      }

      corr.l1_key = k1;
      correspondences.push_back(corr);
      ++point_idx;
    }

    // B7 — l1_count iteration-order signature (Task #36 PV-4 ordering localizer).
    if (tof_slam::frontend::diag::BoundaryLogger::enabled()) {
      namespace diag = tof_slam::frontend::diag;
      std::uint64_t h = diag::kFnv1a64OffsetBasis;
      for (const auto& [k, cnt] : l1_count) {
        const int pack[4] = {k.x, k.y, k.z, cnt};
        h = diag::fnv1a_64_update(
            h, diag::make_byte_view(pack, sizeof(pack)));
      }
      diag::BoundaryLogger::instance().log_precomputed(
          diag::current_frame_idx(),
          diag::BoundaryId::B7_L1CountOrder,
          h,
          static_cast<double>(l1_count.size()));
    }

  } else {
    // =====================================================================
    // Parallel path (OpenMP two-pass)
    // =====================================================================

    // Determine thread count (configurable cap, default 4).
    const int omp_cap = (params.cf_omp_max_threads > 0) ? params.cf_omp_max_threads : omp_get_max_threads();
    const int num_threads = std::min(omp_get_max_threads(), omp_cap);

    // Per-thread storage for Pass 1.
    std::vector<std::vector<Correspondence>> thread_corrs(num_threads);
    std::vector<ankerl::unordered_dense::map<VoxelKey, int, VoxelKeyHash>> thread_l1_counts(num_threads);
    std::vector<HybridStats> thread_stats(num_threads);

    // Pre-reserve per-thread vectors.
    for (int t = 0; t < num_threads; ++t) {
      thread_corrs[t].reserve(scan_size / num_threads + 1);
    }

    // --- Pass 1: Parallel per-point processing ---
    // Each thread processes a contiguous chunk of the scan (schedule(static)).
    // Per-thread l1_count tracks locally; global cap is enforced in Pass 2.

    #pragma omp parallel num_threads(num_threads)
    {
      const int tid = omp_get_thread_num();
      auto& local_corrs = thread_corrs[tid];
      auto& local_l1_count = thread_l1_counts[tid];
      auto& local_stats = thread_stats[tid];

      #pragma omp for schedule(static)
      for (int i = 0; i < scan_size; ++i) {
        const auto& pt = scan[i];

        // 1. Transform point from LiDAR frame to world frame.
        const Eigen::Vector3f p_lidar = pt.to_eigen();
        const Eigen::Vector3f p_world = R_wl * p_lidar + t_wl;

        // D4: Always compute L1 key.
        const VoxelKey k1 = surfel_map.compute_l1_key(p_world);

        // Per-L1 correspondence cap: track in local map (no global enforcement here).
        if (max_corr_per_l1 > 0) {
          auto [it, inserted] = local_l1_count.try_emplace(k1, 0);
          ++(it->second);
          // NOTE: Do NOT enforce the cap here. Defer to merge pass for
          // global scan-order-consistent cap enforcement.
        }

        // 2. Query surfel map (L1 cached PCA plane).
        Surfel surfel;
        bool surfel_valid = false;
        if (params.enable_cscf) {
          // CSCF: Kernel-weighted L1 surfel interpolation (parallel path).
          Surfel l1_surfels[27];
          Eigen::Vector3f l1_centroids[27];
          const int n_surfels = surfel_map.gather_l1_surfels(p_world, l1_surfels, l1_centroids);

          if (n_surfels >= 1) {
            const float sigma2 = params.cscf_kernel_bandwidth * params.cscf_kernel_bandwidth;
            Eigen::Vector3f n_weighted = Eigen::Vector3f::Zero();
            Eigen::Vector3f c_weighted = Eigen::Vector3f::Zero();
            float w_total = 0.0f;
            float sigma2_weighted = 0.0f;
            float planarity_min = 1.0f;
            // Track geometric covariance from highest-weight neighbor.
            // Use Zero() default — Identity() would be a wrong precision matrix
            // if it ever leaked through the has_geometric_cov guard.
            bool best_geo_cov_valid = false;
            Eigen::Matrix3f best_geo_cov_inv = Eigen::Matrix3f::Zero();
            float best_geo_w = 0.0f;

            for (int si = 0; si < 27; ++si) {
              if (!l1_surfels[si].valid) continue;
              const float dist_sq = (p_world - l1_centroids[si]).squaredNorm();
              const float w = std::exp(-dist_sq / (2.0f * sigma2));
              if (w < 1e-6f) continue;

              n_weighted += w * l1_surfels[si].normal;
              c_weighted += w * l1_surfels[si].centroid;
              w_total += w;
              sigma2_weighted += w * l1_surfels[si].normal_sigma2;
              planarity_min = std::min(planarity_min, l1_surfels[si].planarity);
              if (l1_surfels[si].has_geometric_cov && w > best_geo_w) {
                best_geo_cov_valid = true;
                best_geo_cov_inv = l1_surfels[si].geometric_cov_inv;
                best_geo_w = w;
              }
            }

            if (w_total > 1e-8f) {
              const float inv_w = 1.0f / w_total;
              Eigen::Vector3f n_interp = n_weighted * inv_w;
              const float n_norm = n_interp.norm();

              if (n_norm > 0.3f) {
                surfel.normal = n_interp / n_norm;
                surfel.centroid = c_weighted * inv_w;
                surfel.normal_sigma2 = sigma2_weighted * inv_w;
                surfel.planarity = planarity_min;
                surfel.has_geometric_cov = best_geo_cov_valid;
                if (best_geo_cov_valid) {
                  surfel.geometric_cov_inv = best_geo_cov_inv;
                }
                surfel.valid = true;
                surfel_valid = true;
              }
            }
          }
          if (!surfel_valid) {
            surfel_valid = surfel_map.get_surfel(p_world, &surfel) &&
                           surfel.valid;
          }
        } else {
          surfel_valid = surfel_map.get_surfel(p_world, &surfel) &&
                         surfel.valid;
        }

        // 3. Query PVMap — D1: kNN cache with position-delta gating.
        PlaneResult plane;
        if (params.knn_cache) {
          auto& entry = (*params.knn_cache)[i];

          if (params.knn_cache_is_populated) {
            const float delta2 = (p_world - entry.p_world_cached).squaredNorm();
            const float threshold = pvmap.config().voxel_size * 0.4f;

            if (delta2 < threshold * threshold && entry.n_gathered > 0) {
              plane = pvmap.fit_plane_from_neighbors(
                  p_world, entry.neighbors, entry.n_gathered,
                  pvmap_planarity_threshold, knn_max_dist);
            } else {
              entry.n_gathered = pvmap.gather_knn(
                  p_world, std::min(pvmap_k_neighbors, 30), entry.neighbors, nullptr);
              entry.p_world_cached = p_world;
              if (entry.n_gathered > 0) {
                plane = pvmap.fit_plane_from_neighbors(
                    p_world, entry.neighbors, entry.n_gathered,
                    pvmap_planarity_threshold, knn_max_dist);
              }
            }
          } else {
            entry.n_gathered = pvmap.gather_knn(
                p_world, std::min(pvmap_k_neighbors, 30), entry.neighbors, nullptr);
            entry.p_world_cached = p_world;
            if (entry.n_gathered > 0) {
              plane = pvmap.fit_plane_from_neighbors(
                  p_world, entry.neighbors, entry.n_gathered,
                  pvmap_planarity_threshold, knn_max_dist);
            }
          }
        } else {
          plane = pvmap.fit_plane_knn(p_world, pvmap_k_neighbors,
                                      pvmap_planarity_threshold, knn_max_dist);
        }
        bool pvmap_valid = plane.valid;

        // Orient PVMap normal toward sensor when valid.
        if (pvmap_valid) {
          const Eigen::Vector3f ray_orient = p_world - sensor_origin_world;
          if (plane.normal.dot(ray_orient) > 0.0f) {
            plane.normal = -plane.normal;
          }
          const float pvmap_plane_dist =
              std::abs(plane.normal.dot(p_world - plane.centroid));

          const float range = p_lidar.norm();
          const float adaptive_thresh = std::min(
              max_plane_distance,
              std::sqrt(std::max(range, 0.1f)) / params.adaptive_threshold_divisor);
          if (max_plane_distance > 0.0f && pvmap_plane_dist > adaptive_thresh) {
            pvmap_valid = false;
          }
        }

        // Validate surfel plane distance.
        bool surfel_dist_ok = true;
        if (surfel_valid) {
          const float surfel_plane_dist =
              std::abs(surfel.normal.dot(p_world - surfel.centroid));
          const float range_s = p_lidar.norm();
          const float adaptive_thresh_s = std::min(
              max_plane_distance,
              std::sqrt(std::max(range_s, 0.1f)) / params.adaptive_threshold_divisor);
          if (max_plane_distance > 0.0f && surfel_plane_dist > adaptive_thresh_s) {
            surfel_dist_ok = false;
          }
        }
        const bool use_surfel_src = surfel_valid && surfel_dist_ok;

        if (!use_surfel_src && !pvmap_valid) {
          continue;
        }

        // 4. Selection logic: Surfel-Primary hybrid.
        bool use_surfel;
        float cos_nn = 1.0f;
        if (use_surfel_src && !pvmap_valid) {
          use_surfel = true;
          local_stats.n_surfel_only++;
        } else if (!use_surfel_src && pvmap_valid) {
          use_surfel = false;
          local_stats.n_pvmap_only++;
        } else {
          // Both sources available.
          cos_nn = std::abs(surfel.normal.dot(plane.normal));

          local_stats.n_dual++;
          local_stats.sum_cos_nn += cos_nn;

          // --- DDPO: Degeneracy-Directed PVMap Override ---
          bool degen_forced_pvmap = false;
          if (num_degen_trans_dirs > 0 && degen_trans_dirs != nullptr) {
            float max_cos_align = 0.0f;
            for (int d = 0; d < num_degen_trans_dirs; ++d) {
              const float cos_align = std::abs(surfel.normal.dot(degen_trans_dirs[d]));
              max_cos_align = std::max(max_cos_align, cos_align);
            }
            if (max_cos_align >= degen_pvmap_cos_threshold) {
              degen_forced_pvmap = true;
              local_stats.n_degen_override++;
            }
          }

          if (degen_forced_pvmap) {
            use_surfel = false;
          } else {
            const float surfel_eff_sigma2 = surfel.normal_sigma2;
            if (cos_nn >= 0.7f) {
              const float pvmap_adj_sigma2 = plane.normal_sigma2 * pvmap_sigma2_scale;
              use_surfel = (surfel_eff_sigma2 <= pvmap_adj_sigma2);
            } else {
              // Disagree branch: normals differ significantly → always prefer surfel.
              use_surfel = true;
            }
          }
        }

        // 5. Build Correspondence from the selected source.
        Correspondence corr;
        corr.p_lidar = p_lidar;

        const Eigen::Vector3f ray = p_world - sensor_origin_world;
        corr.range = ray.norm();

        if (use_surfel) {
          corr.normal        = surfel.normal;
          corr.plane_d       = surfel.normal.dot(surfel.centroid);
          corr.planarity     = surfel.planarity;
          corr.centroid      = surfel.centroid;
          corr.normal_sigma2 = surfel.normal_sigma2;
          // S13-B.A.4: copy Σ_L1 from B.B.1 storage (mirrors first path at :419).
          corr.has_surfel_cov_L1 = surfel.has_surfel_cov;
          corr.surfel_cov_L1     = surfel.surfel_cov;
        } else {
          corr.normal        = plane.normal;
          corr.plane_d       = plane.normal.dot(plane.centroid);
          corr.planarity     = plane.planarity;
          corr.centroid      = plane.centroid;
          corr.normal_sigma2 = plane.normal_sigma2;
          corr.has_surfel_cov_L1 = false;
        }

        // NAW: store normal agreement for dual-source points.
        if (use_surfel_src && pvmap_valid) {
          corr.normal_agreement = cos_nn;
        }

        // Geometric covariance → P2D mode (parallel path).
        if (use_surfel && surfel.has_geometric_cov) {
          corr.residual_mode = ResidualMode::kPointToDistribution;
          corr.voxel_cov_inv = surfel.geometric_cov_inv;
        }

        // Incidence angle.
        if (corr.range > 1e-6f) {
          corr.cos_incidence = std::abs(ray.dot(corr.normal)) / corr.range;
        } else {
          corr.cos_incidence = 1.0f;
        }

        // 6. L2 supplement.
        if (surfel_map.config().enable_l2_correspondences) {
          Surfel l2_surfel;
          if (surfel_map.get_l2_surfel(p_world, &l2_surfel)) {
            const float l2_plane_dist =
                std::abs(l2_surfel.normal.dot(p_world - l2_surfel.centroid));
            if (max_plane_distance <= 0.0f ||
                l2_plane_dist < max_plane_distance * 3.0f) {
              Eigen::Vector3f l2_n = l2_surfel.normal;
              if (l2_n.dot(p_world - sensor_origin_world) > 0.0f) {
                l2_n = -l2_n;
              }
              const float l1_l2_cos = use_surfel_src
                  ? std::abs(surfel.normal.dot(l2_n))
                  : std::abs(corr.normal.dot(l2_n));
              const bool gate1_pass = (l1_l2_cos >= 0.866f);
              const bool gate2_pass = (l2_plane_dist < 0.3f);
              if (gate1_pass && gate2_pass) {
                corr.has_l2           = true;
                corr.l2_normal        = l2_n;
                corr.l2_plane_d       = l2_n.dot(l2_surfel.centroid);
                corr.l2_centroid      = l2_surfel.centroid;
                corr.l2_normal_sigma2 = l2_surfel.normal_sigma2;
                corr.l2_planarity     = l2_surfel.planarity;
                // S13-B.A.4: copy Σ_L2 (always-on from recompute_l2_surfel).
                corr.has_surfel_cov_L2 = l2_surfel.has_surfel_cov;
                corr.surfel_cov_L2     = l2_surfel.surfel_cov;
              }
            }
          }
        }

        corr.l1_key = k1;
        local_corrs.push_back(corr);
      }
    }  // end omp parallel

    // --- Pass 2: Sequential merge ---
    // With schedule(static), thread 0 has the earliest scan indices, thread 1
    // the next chunk, etc. Concatenating in thread order preserves scan order.

    // Count total correspondences.
    size_t total_corrs = 0;
    for (int t = 0; t < num_threads; ++t) {
      total_corrs += thread_corrs[t].size();
    }
    correspondences.reserve(total_corrs);

    // Merge with global l1_count cap enforcement.
    ankerl::unordered_dense::map<VoxelKey, int, VoxelKeyHash> l1_count;
    for (int t = 0; t < num_threads; ++t) {
      for (auto& corr : thread_corrs[t]) {
        if (max_corr_per_l1 > 0) {
          auto [it, inserted] = l1_count.try_emplace(corr.l1_key, 0);
          if (++(it->second) > max_corr_per_l1) {
            continue;  // Skip -- global cap exceeded
          }
        }
        correspondences.push_back(std::move(corr));
      }
    }

    // Merge stats.
    if (out_stats) {
      for (int t = 0; t < num_threads; ++t) {
        out_stats->n_dual += thread_stats[t].n_dual;
        out_stats->sum_cos_nn += thread_stats[t].sum_cos_nn;
        out_stats->n_surfel_only += thread_stats[t].n_surfel_only;
        out_stats->n_pvmap_only += thread_stats[t].n_pvmap_only;
        out_stats->n_degen_override += thread_stats[t].n_degen_override;
      }
    }

    // B7 — l1_count iteration-order signature (Task #36 PV-4 ordering localizer).
    if (tof_slam::frontend::diag::BoundaryLogger::enabled()) {
      namespace diag = tof_slam::frontend::diag;
      std::uint64_t h = diag::kFnv1a64OffsetBasis;
      for (const auto& [k, cnt] : l1_count) {
        const int pack[4] = {k.x, k.y, k.z, cnt};
        h = diag::fnv1a_64_update(
            h, diag::make_byte_view(pack, sizeof(pack)));
      }
      diag::BoundaryLogger::instance().log_precomputed(
          diag::current_frame_idx(),
          diag::BoundaryId::B7_L1CountOrder,
          h,
          static_cast<double>(l1_count.size()));
    }
  }  // end parallel path

  // --- Surfel sharing count propagation (D4: uses cached l1_key) ---
  // Count how many correspondences share each L1 voxel, then assign.
  {
    ankerl::unordered_dense::map<VoxelKey, int, VoxelKeyHash> share_count;
    for (const auto& c : correspondences) {
      share_count[c.l1_key]++;
    }
    for (auto& c : correspondences) {
      c.sharing_count = share_count[c.l1_key];
    }

    // B8 — share_count iteration-order signature (Task #36 PV-4 localizer).
    if (tof_slam::frontend::diag::BoundaryLogger::enabled()) {
      namespace diag = tof_slam::frontend::diag;
      std::uint64_t h = diag::kFnv1a64OffsetBasis;
      for (const auto& [k, cnt] : share_count) {
        const int pack[4] = {k.x, k.y, k.z, cnt};
        h = diag::fnv1a_64_update(
            h, diag::make_byte_view(pack, sizeof(pack)));
      }
      diag::BoundaryLogger::instance().log_precomputed(
          diag::current_frame_idx(),
          diag::BoundaryId::B8_ShareCountOrder,
          h,
          static_cast<double>(share_count.size()));
    }
  }

  // B3 — Correspondence census (post-assembly) — PV-3 attribution hook.
  if (tof_slam::frontend::diag::BoundaryLogger::enabled()) {
    namespace diag = tof_slam::frontend::diag;
    std::uint64_t h = diag::kFnv1a64OffsetBasis;
    for (const auto& c : correspondences) {
      h = diag::fnv1a_64_update(
          h, diag::make_byte_view(c.p_lidar.data(), sizeof(float) * 3));
      h = diag::fnv1a_64_update(
          h, diag::make_byte_view(c.normal.data(), sizeof(float) * 3));
      h = diag::fnv1a_64_update(
          h, diag::make_byte_view(&c.plane_d, sizeof(float)));
      h = diag::fnv1a_64_update(
          h, diag::make_byte_view(&c.range, sizeof(float)));
      h = diag::fnv1a_64_update(
          h, diag::make_byte_view(&c.cos_incidence, sizeof(float)));
      h = diag::fnv1a_64_update(
          h, diag::make_byte_view(&c.planarity, sizeof(float)));
      h = diag::fnv1a_64_update(
          h, diag::make_byte_view(c.centroid.data(), sizeof(float) * 3));
      h = diag::fnv1a_64_update(
          h, diag::make_byte_view(&c.normal_sigma2, sizeof(float)));
      h = diag::fnv1a_64_update(
          h, diag::make_byte_view(&c.noise_override, sizeof(float)));
      h = diag::fnv1a_64_update(
          h, diag::make_byte_view(&c.sharing_count, sizeof(int)));
      // Fold has_l2 as a single byte, then skip the 3 padding bytes.
      const unsigned char has_l2_byte = c.has_l2 ? 1u : 0u;
      h = diag::fnv1a_64_update(
          h, diag::make_byte_view(&has_l2_byte, sizeof(unsigned char)));
      h = diag::fnv1a_64_update(
          h, diag::make_byte_view(c.l2_normal.data(), sizeof(float) * 3));
      h = diag::fnv1a_64_update(
          h, diag::make_byte_view(&c.l2_plane_d, sizeof(float)));
      h = diag::fnv1a_64_update(
          h, diag::make_byte_view(c.l2_centroid.data(), sizeof(float) * 3));
      h = diag::fnv1a_64_update(
          h, diag::make_byte_view(&c.l2_normal_sigma2, sizeof(float)));
      h = diag::fnv1a_64_update(
          h, diag::make_byte_view(&c.l2_planarity, sizeof(float)));
    }
    diag::BoundaryLogger::instance().log_precomputed(
        diag::current_frame_idx(),
        diag::BoundaryId::B3_Corrs,
        h,
        static_cast<double>(correspondences.size()));
  }

  return correspondences;
}

}  // namespace core
}  // namespace tof_slam
