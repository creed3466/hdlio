// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// point_voxel_map.cpp — Raw-point voxel map for per-query kNN plane fitting.

#include "tof_slam/frontend/map/point_voxel_map.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <spdlog/spdlog.h>

namespace tof_slam {
namespace core {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

PointVoxelMap::PointVoxelMap(const PointVoxelMapConfig& config)
    : config_(config),
      inv_voxel_size_(1.0f / config.voxel_size) {}

// ---------------------------------------------------------------------------
// DARBF: Degeneracy freeze control
// ---------------------------------------------------------------------------

void PointVoxelMap::set_degen_freeze_state(const Eigen::Vector3f* degen_dirs,
                                            int num_dirs, int current_frame) {
  std::unique_lock<std::shared_mutex> lock(mutex_);
  num_degen_dirs_ = (degen_dirs && num_dirs > 0) ? std::min(num_dirs, 3) : 0;
  for (int d = 0; d < num_degen_dirs_; ++d) {
    degen_dirs_[d] = degen_dirs[d];
  }
  current_frame_ = current_frame;
}

int PointVoxelMap::frozen_voxel_count() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  int count = 0;
  for (const auto& [key, vx] : map_) {
    if (vx.frozen) ++count;
  }
  return count;
}

// ---------------------------------------------------------------------------
// update
// ---------------------------------------------------------------------------

void PointVoxelMap::update(const PointCloud& cloud,
                            const Eigen::Vector3f& sensor_pos,
                            float /*frame_res_rms*/, int frame_id) {
  std::unique_lock<std::shared_mutex> lock(mutex_);

  // D2 (Task #71 Iter 2d): advance the frame clock on every update.
  // Previously only set by set_degen_freeze_state(), which is not called
  // when enable_degen_freeze=false. Freshness gate requires a monotone clock
  // regardless of DARBF state. Safe because set_degen_freeze_state(), when
  // called, passes frame_count_ as well — both paths now write the same value.
  current_frame_ = frame_id;

  // Initialize map center on first update.
  if (!map_initialized_) {
    map_center_ = sensor_pos;
    map_initialized_ = true;
  }

  // Recentering: if sensor has moved too far from map center, recenter & prune.
  const float dist_from_center = (sensor_pos - map_center_).norm();
  if (dist_from_center > config_.max_distance * config_.distance_multiplier) {
    map_center_ = sensor_pos;
    prune(map_center_, config_.max_distance);
  }

  // --- DARBF: freeze/unfreeze logic ---
  const bool do_darbf = config_.enable_degen_freeze;
  const bool degen_active = (num_degen_dirs_ > 0);

  if (do_darbf) {
    if (!degen_active) {
      // No degeneracy: unfreeze all frozen voxels.
      for (auto& [key, vx] : map_) {
        if (vx.frozen) {
          vx.frozen = false;
          vx.has_cached_normal = false;  // Invalidate for future recomputation.
        }
      }
    } else {
      // Degeneracy active: check max freeze duration, unfreeze expired.
      for (auto& [key, vx] : map_) {
        if (vx.frozen &&
            (current_frame_ - vx.freeze_frame_start) > config_.degen_freeze_max_frames) {
          vx.frozen = false;
          vx.has_cached_normal = false;
        }
      }
    }
  }

  // Insert all points.
  const int capacity = config_.max_points_per_voxel;
  const float cos_thresh = config_.degen_freeze_cos_threshold;
  const bool track_dirty = config_.use_voxel_plane_cache;
  if (track_dirty) dirty_voxel_keys_.clear();

  for (size_t i = 0; i < cloud.size(); ++i) {
    const Eigen::Vector3f p = cloud[i].to_eigen();
    const VoxelKey key = to_key(p);

    auto it = map_.find(key);
    if (it == map_.end()) {
      // New voxel — always insert (never freeze on first point).
      Voxel vx;
      vx.init(capacity);
      vx.push(p);
      vx.last_obs_frame = current_frame_;  // R0.10 H4: record observation frame
      map_.emplace(key, std::move(vx));
      if (track_dirty) dirty_voxel_keys_.push_back(key);
    } else {
      Voxel& vx = it->second;

      // DARBF: check if this voxel should be frozen.
      if (do_darbf && degen_active && !vx.frozen && vx.count >= 5) {
        // Compute voxel normal if not cached.
        if (!vx.has_cached_normal) {
          // Quick plane fit from stored points.
          Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
          for (int j = 0; j < vx.count; ++j) centroid += vx.points[j];
          centroid /= static_cast<float>(vx.count);

          Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
          for (int j = 0; j < vx.count; ++j) {
            const Eigen::Vector3f diff = vx.points[j] - centroid;
            cov.noalias() += diff * diff.transpose();
          }
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
          const float lambda_min = eig.eigenvalues()(0);
          const float lambda_max = eig.eigenvalues()(2);
          // Only cache normal if voxel is reasonably planar.
          if (lambda_max > 1e-8f && (lambda_min / lambda_max) < 0.3f) {
            vx.cached_normal = eig.eigenvectors().col(0);
            vx.has_cached_normal = true;
          }
        }

        // Check alignment with degenerate directions.
        if (vx.has_cached_normal) {
          for (int d = 0; d < num_degen_dirs_; ++d) {
            const float cos_align = std::abs(vx.cached_normal.dot(degen_dirs_[d]));
            if (cos_align >= cos_thresh) {
              vx.frozen = true;
              vx.freeze_frame_start = current_frame_;
              break;
            }
          }
        }
      }

      const bool was_frozen = vx.frozen;
      const int cap_ring = static_cast<int>(vx.points.size());

      // Option B: Proximity-based insertion (FAST-LIO2 map_incremental parity).
      // Instead of FIFO, keep the point closest to the voxel center.
      // New point replaces the farthest-from-center existing point only if
      // the new point is closer to center. Preserves geometric diversity.
      if (config_.use_proximity_insertion && !was_frozen && vx.count >= cap_ring) {
        // Voxel center = key midpoint
        const Eigen::Vector3f voxel_center(
            (static_cast<float>(key.x) + 0.5f) * config_.voxel_size,
            (static_cast<float>(key.y) + 0.5f) * config_.voxel_size,
            (static_cast<float>(key.z) + 0.5f) * config_.voxel_size);
        const float new_dist2 = (p - voxel_center).squaredNorm();

        // Find the farthest existing point from center
        int farthest_idx = 0;
        float farthest_dist2 = 0.0f;
        for (int j = 0; j < vx.count; ++j) {
          const float d2 = (vx.points[j] - voxel_center).squaredNorm();
          if (d2 > farthest_dist2) {
            farthest_dist2 = d2;
            farthest_idx = j;
          }
        }

        // Replace only if new point is closer to center
        if (new_dist2 < farthest_dist2) {
          vx.points[farthest_idx] = p;
          vx.has_cached_normal = false;  // Invalidate cached normal
          if (track_dirty) dirty_voxel_keys_.push_back(key);
        }
        vx.last_obs_frame = current_frame_;  // R0.10 H4: voxel observed this frame
        // Skip normal FIFO push — proximity insertion handled above
      } else {
        if (!vx.frozen && track_dirty) dirty_voxel_keys_.push_back(key);
        vx.push(p);  // Standard FIFO push (or frozen skip)
        vx.last_obs_frame = current_frame_;  // R0.10 H4: voxel observed this frame
      }

    }
  }

  // Option C: Recompute plane caches for voxels whose point count changed.
  if (config_.use_voxel_plane_cache) {
    recompute_voxel_plane_caches();
  }
}

// ---------------------------------------------------------------------------
// gather_knn
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// gather_knn — Branch-and-bound optimized (ikd-tree principle)
// ---------------------------------------------------------------------------
// Traverses neighbor voxels in shell order (center → face → edge → corner).
// Maintains a fixed-size max-heap of k best candidates. For each voxel,
// computes a lower-bound squared distance; if it exceeds the current k-th
// nearest, the entire voxel is skipped (branch-and-bound pruning).
//
// Complexity:
//   Best case (dense map): ~k distance computations (center voxel fills heap,
//   remaining 26 voxels pruned). Worst case: same as brute-force (270 comps).
//   Eliminates nth_element/sort overhead and heap allocation path entirely.
// ---------------------------------------------------------------------------

namespace {

// Shell-ordered neighbor offsets for H=1 (27 voxels).
// Shell 0: center (1), Shell 1: face-adjacent (6),
// Shell 2: edge-adjacent (12), Shell 3: corners (8).
// Sorted by L∞ distance, then lexicographic within shell.
static constexpr int kNeighborOffsets27[27][3] = {
  // Shell 0: center
  { 0, 0, 0},
  // Shell 1: 6 face-adjacent (L∞ = 1, exactly 1 nonzero axis)
  {-1, 0, 0}, { 0,-1, 0}, { 0, 0,-1},
  { 0, 0, 1}, { 0, 1, 0}, { 1, 0, 0},
  // Shell 2: 12 edge-adjacent (L∞ = 1, exactly 2 nonzero axes)
  {-1,-1, 0}, {-1, 0,-1}, {-1, 0, 1}, {-1, 1, 0},
  { 0,-1,-1}, { 0,-1, 1}, { 0, 1,-1}, { 0, 1, 1},
  { 1,-1, 0}, { 1, 0,-1}, { 1, 0, 1}, { 1, 1, 0},
  // Shell 3: 8 corners (L∞ = 1, all 3 axes nonzero)
  {-1,-1,-1}, {-1,-1, 1}, {-1, 1,-1}, {-1, 1, 1},
  { 1,-1,-1}, { 1,-1, 1}, { 1, 1,-1}, { 1, 1, 1},
};

// Lower-bound squared distance from point q to axis-aligned voxel [lo, hi).
inline float voxel_lower_bound_d2(const Eigen::Vector3f& q,
                                   float vx_lo_x, float vx_lo_y, float vx_lo_z,
                                   float vs) {
  float d2 = 0.0f;
  // X axis
  float lo = vx_lo_x, hi = vx_lo_x + vs;
  if (q.x() < lo) { float d = lo - q.x(); d2 += d * d; }
  else if (q.x() > hi) { float d = q.x() - hi; d2 += d * d; }
  // Y axis
  lo = vx_lo_y; hi = vx_lo_y + vs;
  if (q.y() < lo) { float d = lo - q.y(); d2 += d * d; }
  else if (q.y() > hi) { float d = q.y() - hi; d2 += d * d; }
  // Z axis
  lo = vx_lo_z; hi = vx_lo_z + vs;
  if (q.z() < lo) { float d = lo - q.z(); d2 += d * d; }
  else if (q.z() > hi) { float d = q.z() - hi; d2 += d * d; }
  return d2;
}

// Max-heap sift-down for fixed-size k-heap (largest at root).
inline void sift_down(float* d2_heap, Eigen::Vector3f* pt_heap, int n, int i) {
  while (true) {
    int largest = i;
    int left = 2 * i + 1;
    int right = 2 * i + 2;
    if (left < n && d2_heap[left] > d2_heap[largest]) largest = left;
    if (right < n && d2_heap[right] > d2_heap[largest]) largest = right;
    if (largest == i) break;
    std::swap(d2_heap[i], d2_heap[largest]);
    std::swap(pt_heap[i], pt_heap[largest]);
    i = largest;
  }
}

inline void sift_up(float* d2_heap, Eigen::Vector3f* pt_heap, int i) {
  while (i > 0) {
    int parent = (i - 1) / 2;
    if (d2_heap[i] <= d2_heap[parent]) break;
    std::swap(d2_heap[i], d2_heap[parent]);
    std::swap(pt_heap[i], pt_heap[parent]);
    i = parent;
  }
}

}  // namespace

int PointVoxelMap::gather_knn(const Eigen::Vector3f& query,
                               int k,
                               Eigen::Vector3f* out_points,
                               float* out_dists2) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);

  const VoxelKey center_key = to_key(query);
  const int H = config_.knn_search_half;
  const float vs = config_.voxel_size;

  // k-max-heap on stack (k is typically 10, max ~20).
  constexpr int kMaxK = 64;
  const int kk = std::min(k, kMaxK);
  float heap_d2[kMaxK];
  Eigen::Vector3f heap_pts[kMaxK];
  int heap_size = 0;
  float worst_d2 = std::numeric_limits<float>::max();

  // Helper: try to insert a point into the k-heap.
  auto try_insert = [&](const Eigen::Vector3f& p, float d2) {
    if (heap_size < kk) {
      heap_d2[heap_size] = d2;
      heap_pts[heap_size] = p;
      ++heap_size;
      sift_up(heap_d2, heap_pts, heap_size - 1);
      if (heap_size == kk) worst_d2 = heap_d2[0];  // root = max
    } else if (d2 < worst_d2) {
      // Replace root (worst) with new point
      heap_d2[0] = d2;
      heap_pts[0] = p;
      sift_down(heap_d2, heap_pts, kk, 0);
      worst_d2 = heap_d2[0];
    }
  };

  if (H == 1) {
    // Adaptive neighbor search: center(1) → face(6) → edge+corner(20)
    // After each shell, check if heap is full AND worst_d2 is tight enough
    // to prune remaining shells. For dense maps, often only center+face
    // are needed (7 lookups instead of 27 = ~3.8× speedup).
    //
    // Shell boundaries in kNeighborOffsets27:
    //   [0,1)   = center (1 voxel)
    //   [1,7)   = face-adjacent (6 voxels)
    //   [7,27)  = edge + corner (20 voxels)
    static constexpr int kShellEnd[3] = {1, 7, 27};

    // Precompute query's distance to each face of the home voxel.
    // d_face[i] = distance from query to nearest boundary along axis i.
    // Used for analytical shell minimum lower-bound computation.
    const float home_lo_x = static_cast<float>(center_key.x) * vs;
    const float home_lo_y = static_cast<float>(center_key.y) * vs;
    const float home_lo_z = static_cast<float>(center_key.z) * vs;
    float d_face[3] = {
      std::min(query.x() - home_lo_x, home_lo_x + vs - query.x()),
      std::min(query.y() - home_lo_y, home_lo_y + vs - query.y()),
      std::min(query.z() - home_lo_z, home_lo_z + vs - query.z()),
    };
    // Ensure non-negative (query should be inside home voxel)
    for (int i = 0; i < 3; ++i) d_face[i] = std::max(d_face[i], 0.0f);
    // Sort d_face ascending for shell lb computation
    float d_sorted[3] = {d_face[0], d_face[1], d_face[2]};
    if (d_sorted[0] > d_sorted[1]) std::swap(d_sorted[0], d_sorted[1]);
    if (d_sorted[1] > d_sorted[2]) std::swap(d_sorted[1], d_sorted[2]);
    if (d_sorted[0] > d_sorted[1]) std::swap(d_sorted[0], d_sorted[1]);
    // d_sorted[0] ≤ d_sorted[1] ≤ d_sorted[2]

    for (int shell = 0; shell < 3; ++shell) {
      // Analytical shell minimum lower-bound (correct, not approximation).
      // Shell 1 (6 face-adjacent, 1 nonzero offset):
      //   min lb = min(d_face)² = d_sorted[0]²
      //   The closest face voxel is along the axis where query is nearest to boundary.
      // Shell 2 (12 edge + 8 corner, 2-3 nonzero offsets):
      //   min lb = d_sorted[0]² + d_sorted[1]²
      //   The closest edge voxel uses the two axes nearest to boundaries.
      //   Corners always have lb ≥ edges, so edge minimum suffices.
      if (shell > 0 && heap_size == kk) {
        float shell_min_lb;
        if (shell == 1) {
          shell_min_lb = d_sorted[0] * d_sorted[0];
        } else {
          // shell == 2: edge + corner
          shell_min_lb = d_sorted[0] * d_sorted[0] + d_sorted[1] * d_sorted[1];
        }
        if (shell_min_lb >= worst_d2) break;  // Prune entire remaining shells
      }

      for (int vi = (shell == 0 ? 0 : kShellEnd[shell - 1]); vi < kShellEnd[shell]; ++vi) {
        const int dx = kNeighborOffsets27[vi][0];
        const int dy = kNeighborOffsets27[vi][1];
        const int dz = kNeighborOffsets27[vi][2];

        // Per-voxel branch-and-bound pruning
        if (heap_size == kk) {
          const float vx_lo_x = static_cast<float>(center_key.x + dx) * vs;
          const float vx_lo_y = static_cast<float>(center_key.y + dy) * vs;
          const float vx_lo_z = static_cast<float>(center_key.z + dz) * vs;
          const float lb = voxel_lower_bound_d2(query, vx_lo_x, vx_lo_y, vx_lo_z, vs);
          if (lb >= worst_d2) continue;
        }

        const VoxelKey nkey(center_key.x + dx, center_key.y + dy,
                            center_key.z + dz);
        auto it = map_.find(nkey);
        if (it == map_.end()) continue;

        const Voxel& vx = it->second;

        for (int i = 0; i < vx.count; ++i) {
          const Eigen::Vector3f& p = vx.points[i];
          const float d2 = (p - query).squaredNorm();
          try_insert(p, d2);
        }
      }  // end vi loop
    }  // end shell loop
  } else {
    // General path for H > 1: same logic with dynamic offsets
    // Generate offsets sorted by L∞ shell distance
    const int side = 2 * H + 1;
    const int n_voxels = side * side * side;
    struct VoxelOffset { int dx, dy, dz; int shell; };
    VoxelOffset offsets[343];  // max: H=3 → 7^3=343
    int n_off = 0;
    for (int dx = -H; dx <= H; ++dx) {
      for (int dy = -H; dy <= H; ++dy) {
        for (int dz = -H; dz <= H; ++dz) {
          int shell = std::max({std::abs(dx), std::abs(dy), std::abs(dz)});
          offsets[n_off++] = {dx, dy, dz, shell};
        }
      }
    }
    // Sort by shell (center first), then lexicographic
    std::sort(offsets, offsets + n_off, [](const VoxelOffset& a, const VoxelOffset& b) {
      if (a.shell != b.shell) return a.shell < b.shell;
      if (a.dx != b.dx) return a.dx < b.dx;
      if (a.dy != b.dy) return a.dy < b.dy;
      return a.dz < b.dz;
    });

    for (int vi = 0; vi < n_off; ++vi) {
      const int dx = offsets[vi].dx;
      const int dy = offsets[vi].dy;
      const int dz = offsets[vi].dz;

      if (heap_size == kk) {
        const float vx_lo_x = static_cast<float>(center_key.x + dx) * vs;
        const float vx_lo_y = static_cast<float>(center_key.y + dy) * vs;
        const float vx_lo_z = static_cast<float>(center_key.z + dz) * vs;
        const float lb = voxel_lower_bound_d2(query, vx_lo_x, vx_lo_y, vx_lo_z, vs);
        if (lb >= worst_d2) continue;
      }

      const VoxelKey nkey(center_key.x + dx, center_key.y + dy,
                          center_key.z + dz);
      auto it = map_.find(nkey);
      if (it == map_.end()) continue;

      const Voxel& vx = it->second;

      for (int i = 0; i < vx.count; ++i) {
        const Eigen::Vector3f& p = vx.points[i];
        const float d2 = (p - query).squaredNorm();
        try_insert(p, d2);
      }
    }
  }

  if (heap_size == 0) return 0;

  // Extract heap into output in sorted order (ascending distance).
  // Heap-sort: repeatedly extract max and place at end.
  const int n_out = heap_size;
  for (int i = n_out - 1; i > 0; --i) {
    std::swap(heap_d2[0], heap_d2[i]);
    std::swap(heap_pts[0], heap_pts[i]);
    sift_down(heap_d2, heap_pts, i, 0);
  }

  // Copy to output
  for (int i = 0; i < n_out; ++i) {
    out_points[i] = heap_pts[i];
    if (out_dists2) out_dists2[i] = heap_d2[i];
  }

  return n_out;
}

// ---------------------------------------------------------------------------
// fit_plane_qr — QR linear solve (FAST-LIO2 esti_plane parity)
// ---------------------------------------------------------------------------
// Solves the plane equation: ax + by + cz = -1 via column-pivoting
// Householder QR. Then validates ALL neighbors within 0.1m of the plane.
// This is well-determined even at k=5 (3 unknowns, k equations).

static PlaneResult fit_plane_qr_impl(const Eigen::Vector3f& query,
                                      const Eigen::Vector3f* neighbors,
                                      int n_found) {
  PlaneResult result;
  if (n_found < 3) return result;

  // Build A*x = b, where b = [-1,...,-1]^T
  Eigen::MatrixXf A(n_found, 3);
  Eigen::VectorXf b = Eigen::VectorXf::Constant(n_found, -1.0f);
  for (int i = 0; i < n_found; ++i) {
    A(i, 0) = neighbors[i].x();
    A(i, 1) = neighbors[i].y();
    A(i, 2) = neighbors[i].z();
  }

  // QR solve: normvec = (a, b, c) such that ax + by + cz + 1 = 0
  const Eigen::Vector3f normvec = A.colPivHouseholderQr().solve(b);
  const float n_len = normvec.norm();
  if (n_len < 1e-8f) return result;

  // Normalized normal and distance from origin
  Eigen::Vector3f normal = normvec / n_len;
  const float d = 1.0f / n_len;

  // Per-neighbor inlier check: ALL must be within 0.1m (FAST-LIO2 parity)
  constexpr float kInlierThreshold = 0.1f;
  for (int i = 0; i < n_found; ++i) {
    const float dist_to_plane =
        std::abs(normal.dot(neighbors[i]) + d);
    if (dist_to_plane > kInlierThreshold) {
      return result;  // Outlier neighbor — reject entire plane
    }
  }

  // Compute centroid for residual computation
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  for (int i = 0; i < n_found; ++i) {
    centroid += neighbors[i];
  }
  centroid /= static_cast<float>(n_found);

  // Orient normal toward query
  const Eigen::Vector3f to_query = query - centroid;
  if (normal.dot(to_query) < 0.0f) {
    normal = -normal;
  }

  // Build result
  result.centroid = centroid;
  result.normal = normal;
  result.planarity = 0.0f;  // QR does not compute eigenvalues
  result.sigma_max = 0.0f;
  result.sigma_min = 0.0f;
  result.normal_sigma2 = 0.001f;  // Match FAST-LIO2 LASER_POINT_COV
  result.n_points = n_found;
  result.valid = true;

  return result;
}

// ---------------------------------------------------------------------------
// fit_plane_knn
// ---------------------------------------------------------------------------

PlaneResult PointVoxelMap::fit_plane_knn(const Eigen::Vector3f& query,
                                          int k_neighbors,
                                          float planarity_threshold,
                                          float max_dist) const {
  PlaneResult result;

  // Gather k nearest neighbors.
  Eigen::Vector3f neighbors[30];  // k is typically 5-20
  float dists2[30];
  const int k = std::min(k_neighbors, 30);
  const int n_gathered = gather_knn(query, k, neighbors, dists2);

  // Apply distance filter if max_dist is set.
  int n_found = n_gathered;
  if (max_dist > 0.0f) {
    const float max_dist2 = max_dist * max_dist;
    n_found = 0;
    for (int i = 0; i < n_gathered; ++i) {
      if (dists2[i] <= max_dist2) {
        neighbors[n_found] = neighbors[i];
        ++n_found;
      }
    }
  }

  // Option A: QR plane fitting (FAST-LIO2 esti_plane parity)
  if (config_.use_qr_plane_fit) {
    return fit_plane_qr_impl(query, neighbors, n_found);
  }

  // --- Default PCA path ---
  if (n_found < 3) {
    return result;  // Not enough points for plane fitting
  }

  // Compute centroid.
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  for (int i = 0; i < n_found; ++i) {
    centroid += neighbors[i];
  }
  centroid /= static_cast<float>(n_found);

  // Compute 3x3 covariance matrix.
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  for (int i = 0; i < n_found; ++i) {
    const Eigen::Vector3f diff = neighbors[i] - centroid;
    cov.noalias() += diff * diff.transpose();
  }
  cov /= static_cast<float>(n_found);

  // Eigendecomposition (eigenvalues ascending: λ₀ ≤ λ₁ ≤ λ₂).
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(
      cov, Eigen::ComputeEigenvectors);
  const Eigen::Vector3f& eigenvalues = eig.eigenvalues();

  const float lambda_min = eigenvalues(0);
  const float lambda_max = eigenvalues(2);

  // Degenerate check.
  if (lambda_max < 1e-8f) {
    return result;
  }

  // Planarity: λ_min / λ_max.  0 = perfectly planar, 1 = isotropic.
  const float planarity = lambda_min / lambda_max;
  if (planarity > planarity_threshold) {
    return result;  // Not planar enough
  }

  // Normal = eigenvector of smallest eigenvalue.
  Eigen::Vector3f normal = eig.eigenvectors().col(0);

  // Orient normal toward the query point (away from surface interior).
  // This ensures consistent orientation for point-to-plane residuals.
  const Eigen::Vector3f to_query = query - centroid;
  if (normal.dot(to_query) < 0.0f) {
    normal = -normal;
  }

  // Build result.
  result.centroid = centroid;
  result.normal = normal;
  result.planarity = planarity;
  result.sigma_max = lambda_max;
  result.sigma_min = lambda_min;
  result.normal_sigma2 = lambda_min / (static_cast<float>(n_found) * lambda_max + 1e-10f);
  result.n_points = n_found;
  result.valid = true;

  return result;
}

// ---------------------------------------------------------------------------
// fit_plane_from_neighbors (D1: skip kNN re-search, reuse cached neighbors)
// ---------------------------------------------------------------------------

PlaneResult PointVoxelMap::fit_plane_from_neighbors(
    const Eigen::Vector3f& query,
    const Eigen::Vector3f* neighbors_in,
    int n_neighbors,
    float planarity_threshold,
    float max_dist) const {
  PlaneResult result;

  // Apply distance filter if max_dist is set.
  Eigen::Vector3f neighbors[30];
  int n_found = 0;
  if (config_.use_qr_plane_fit) {
    // QR path: gather filtered neighbors, then delegate
    if (max_dist > 0.0f) {
      const float max_dist2 = max_dist * max_dist;
      for (int i = 0; i < n_neighbors && n_found < 30; ++i) {
        const float d2 = (neighbors_in[i] - query).squaredNorm();
        if (d2 <= max_dist2) {
          neighbors[n_found++] = neighbors_in[i];
        }
      }
    } else {
      n_found = std::min(n_neighbors, 30);
      for (int i = 0; i < n_found; ++i) neighbors[i] = neighbors_in[i];
    }
    return fit_plane_qr_impl(query, neighbors, n_found);
  }

  if (max_dist > 0.0f) {
    const float max_dist2 = max_dist * max_dist;
    for (int i = 0; i < n_neighbors && n_found < 30; ++i) {
      const float d2 = (neighbors_in[i] - query).squaredNorm();
      if (d2 <= max_dist2) {
        neighbors[n_found] = neighbors_in[i];
        ++n_found;
      }
    }
  } else {
    n_found = std::min(n_neighbors, 30);
    for (int i = 0; i < n_found; ++i) {
      neighbors[i] = neighbors_in[i];
    }
  }

  if (n_found < 3) {
    return result;  // Not enough points for plane fitting
  }

  // Compute centroid.
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  for (int i = 0; i < n_found; ++i) {
    centroid += neighbors[i];
  }
  centroid /= static_cast<float>(n_found);

  // Compute 3x3 covariance matrix.
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  for (int i = 0; i < n_found; ++i) {
    const Eigen::Vector3f diff = neighbors[i] - centroid;
    cov.noalias() += diff * diff.transpose();
  }
  cov /= static_cast<float>(n_found);

  // Eigendecomposition (eigenvalues ascending: lambda_0 <= lambda_1 <= lambda_2).
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(
      cov, Eigen::ComputeEigenvectors);
  const Eigen::Vector3f& eigenvalues = eig.eigenvalues();

  const float lambda_min = eigenvalues(0);
  const float lambda_max = eigenvalues(2);

  // Degenerate check.
  if (lambda_max < 1e-8f) {
    return result;
  }

  // Planarity: lambda_min / lambda_max.  0 = perfectly planar, 1 = isotropic.
  const float planarity = lambda_min / lambda_max;
  if (planarity > planarity_threshold) {
    return result;  // Not planar enough
  }

  // Normal = eigenvector of smallest eigenvalue.
  Eigen::Vector3f normal = eig.eigenvectors().col(0);

  // Orient normal toward the query point (away from surface interior).
  const Eigen::Vector3f to_query = query - centroid;
  if (normal.dot(to_query) < 0.0f) {
    normal = -normal;
  }

  // Build result.
  result.centroid = centroid;
  result.normal = normal;
  result.planarity = planarity;
  result.sigma_max = lambda_max;
  result.sigma_min = lambda_min;
  result.normal_sigma2 = lambda_min / (static_cast<float>(n_found) * lambda_max + 1e-10f);
  result.n_points = n_found;
  result.valid = true;

  return result;
}

// ---------------------------------------------------------------------------
// Option C: recompute_voxel_plane_caches
// ---------------------------------------------------------------------------
// Recompute centroid + covariance + normal for voxels whose point count
// changed since last recompute.  Called at end of update() under the
// exclusive lock.  Cost: one PCA per dirty voxel per frame — amortised
// because most voxels are unchanged between frames.

void PointVoxelMap::recompute_voxel_plane_caches() {
  const int min_pts = config_.voxel_plane_min_points;

  // Only process voxels that were modified this frame.
  for (const auto& key : dirty_voxel_keys_) {
    auto it = map_.find(key);
    if (it == map_.end()) continue;
    Voxel& vx = it->second;

    // Skip if already up to date (duplicate key in dirty list).
    if (vx.plane_cache_count == vx.count) continue;
    vx.plane_cache_count = vx.count;

    if (vx.count < min_pts) {
      vx.plane_cache_valid = false;
      continue;
    }

    // Recompute centroid from all valid points.
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (int i = 0; i < vx.count; ++i) {
      centroid += vx.points[i];
    }
    centroid /= static_cast<float>(vx.count);

    // Recompute 3×3 covariance.
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (int i = 0; i < vx.count; ++i) {
      const Eigen::Vector3f diff = vx.points[i] - centroid;
      cov.noalias() += diff * diff.transpose();
    }
    cov /= static_cast<float>(vx.count);

    // Eigendecomposition (ascending: λ₀ ≤ λ₁ ≤ λ₂).
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(
        cov, Eigen::ComputeEigenvectors);
    const Eigen::Vector3f& ev = eig.eigenvalues();
    const float lambda_min = ev(0);
    const float lambda_max = ev(2);

    if (lambda_max < 1e-8f) {
      vx.plane_cache_valid = false;
      continue;
    }

    vx.plane_centroid = centroid;
    vx.plane_normal   = eig.eigenvectors().col(0);
    vx.plane_planarity = lambda_min / lambda_max;
    vx.plane_normal_sigma2 =
        lambda_min / (static_cast<float>(vx.count) * lambda_max + 1e-10f);

    // SVD-regularized covariance: clamp smallest eigenvalue.
    // Adaptive SVD: use per-frame effective value if set, else config default.
    const float svd_min = (effective_svd_min_eigenvalue_ >= 0.0f)
        ? effective_svd_min_eigenvalue_
        : config_.svd_min_eigenvalue;
    const Eigen::Matrix3f U = eig.eigenvectors();  // columns = eigenvectors
    if (svd_min > 0.0f) {
      const Eigen::Vector3f reg_ev(svd_min, 1.0f, 1.0f);
      vx.plane_cov = U * reg_ev.asDiagonal() * U.transpose();
    } else {
      // Raw covariance with small epsilon for numerical stability
      const Eigen::Vector3f raw_ev(
          std::max(lambda_min, 1e-6f) / lambda_max,
          ev(1) / lambda_max,
          1.0f);
      vx.plane_cov = U * raw_ev.asDiagonal() * U.transpose();
    }

    vx.plane_cache_valid = true;
  }
}

// ---------------------------------------------------------------------------
// Option C: get_voxel_planes — multi-voxel cached plane lookup
// ---------------------------------------------------------------------------
// For each voxel in the (2H+1)^3 neighborhood of @p query, return the
// cached plane if valid.  This replaces the gather_knn+PCA path with O(27)
// hash lookups + distance checks.

int PointVoxelMap::get_voxel_planes(const Eigen::Vector3f& query,
                                     PlaneResult* out_planes,
                                     int max_results,
                                     float planarity_threshold,
                                     float max_plane_dist,
                                     int search_half_override) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);

  const VoxelKey center_key = to_key(query);
  const int H = (search_half_override >= 0) ? search_half_override
                                             : config_.knn_search_half;
  int n_out = 0;

  for (int dx = -H; dx <= H && n_out < max_results; ++dx) {
    for (int dy = -H; dy <= H && n_out < max_results; ++dy) {
      for (int dz = -H; dz <= H && n_out < max_results; ++dz) {
        const VoxelKey nkey(center_key.x + dx, center_key.y + dy,
                            center_key.z + dz);
        auto it = map_.find(nkey);
        if (it == map_.end()) continue;

        const Voxel& vx = it->second;
        if (!vx.plane_cache_valid) continue;
        if (vx.plane_planarity > planarity_threshold) continue;

        // Distance from query to cached plane.
        const float dist_to_plane =
            std::abs(vx.plane_normal.dot(query - vx.plane_centroid));
        if (max_plane_dist > 0.0f && dist_to_plane > max_plane_dist) {
          continue;
        }

        // Orient normal toward query (consistent with fit_plane_knn).
        Eigen::Vector3f normal = vx.plane_normal;
        if (normal.dot(query - vx.plane_centroid) < 0.0f) {
          normal = -normal;
        }

        PlaneResult& pr = out_planes[n_out];
        pr.centroid       = vx.plane_centroid;
        pr.normal         = normal;
        pr.planarity      = vx.plane_planarity;
        pr.normal_sigma2  = vx.plane_normal_sigma2;
        pr.sigma_max      = 0.0f;
        pr.sigma_min      = 0.0f;
        pr.n_points       = vx.count;
        pr.valid          = true;
        pr.cov            = vx.plane_cov;  // P2D: pass covariance through
        ++n_out;
      }
    }
  }

  return n_out;
}

// ---------------------------------------------------------------------------
// Statistics
// ---------------------------------------------------------------------------

size_t PointVoxelMap::voxel_count() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return map_.size();
}

size_t PointVoxelMap::point_count() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  size_t total = 0;
  for (const auto& [key, vx] : map_) {
    total += vx.count;
  }
  return total;
}

// ---------------------------------------------------------------------------
// reset
// ---------------------------------------------------------------------------

void PointVoxelMap::reset() {
  std::unique_lock<std::shared_mutex> lock(mutex_);
  map_.clear();
  map_center_ = Eigen::Vector3f::Zero();
  map_initialized_ = false;
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

VoxelKey PointVoxelMap::to_key(const Eigen::Vector3f& p) const {
  return point_to_voxel_key(p.x(), p.y(), p.z(), inv_voxel_size_);
}

void PointVoxelMap::prune(const Eigen::Vector3f& center, float box_half) {
  // Remove voxels whose center is outside the axis-aligned keep box.
  // Voxel center = (key + 0.5) * voxel_size.
  const float vs = config_.voxel_size;
  auto it = map_.begin();
  while (it != map_.end()) {
    const float cx = (static_cast<float>(it->first.x) + 0.5f) * vs;
    const float cy = (static_cast<float>(it->first.y) + 0.5f) * vs;
    const float cz = (static_cast<float>(it->first.z) + 0.5f) * vs;
    if (std::abs(cx - center.x()) > box_half ||
        std::abs(cy - center.y()) > box_half ||
        std::abs(cz - center.z()) > box_half) {
      it = map_.erase(it);
    } else {
      ++it;
    }
  }
}

// ---------------------------------------------------------------------------
// R0.10 H4: clear_pre_lock_voxels — surgical pre-LOCK pruning
// ---------------------------------------------------------------------------
// Prunes every PVMap Voxel observed at or before frame_threshold (i.e. the
// pre-LOCK [0..frame_threshold] window). Called by lio_estimator at Stage A
// LOCK when cls == OUTDOOR_DRIFT (see lio_estimator.cpp apply_template_).
//
// Determinism (I-8): kill list is sorted before erase to make the eventual
// rehash chain order independent of hash-table iteration order.
// ---------------------------------------------------------------------------
void PointVoxelMap::clear_pre_lock_voxels(int frame_threshold) {
  std::unique_lock<std::shared_mutex> lock(mutex_);

  // Collect kill set; sort to make erase order deterministic
  // (ankerl::unordered_dense::map iteration order is implementation-defined).
  std::vector<VoxelKey> kill_keys;
  kill_keys.reserve(map_.size());  // Upper bound; safe for the LOCK-frame call.
  for (const auto& [k, vx] : map_) {
    if (vx.last_obs_frame >= 0 && vx.last_obs_frame <= frame_threshold) {
      kill_keys.push_back(k);
    }
  }
  std::sort(kill_keys.begin(), kill_keys.end());

  for (const VoxelKey& k : kill_keys) {
    map_.erase(k);
  }
}

}  // namespace core
}  // namespace tof_slam
