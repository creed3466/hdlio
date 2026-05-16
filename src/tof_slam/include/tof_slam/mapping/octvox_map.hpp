#pragma once

/// octvox_map.hpp
///
/// OctVoxMap — a voxel map with 2x2x2 octant sub-structure and a hierarchical
/// K-nearest-neighbour (HKNN) search.  Faithful port of
///   Super-LIO/src/super_lio/include/OctVoxMap/OctVoxMap.hpp
/// adapted to the TofSLAM conventions (Eigen::Vector3d, tof_slam namespace,
/// std::unordered_map instead of tsl::robin_map).

#include <array>
#include <cstring>
#include <iostream>
#include <list>
#include <memory>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "tof_slam/mapping/hknn_tables.hpp"

namespace tof_slam {

// ---------------------------------------------------------------------------
// KNNHeap<K>
//
// Fixed-capacity max-heap that keeps the K nearest neighbours seen so far.
// Internally the array is unsorted; only the index of the current worst entry
// (highest squared distance) is tracked so that replacements are O(K).
// ---------------------------------------------------------------------------
template <int K>
class KNNHeap {
public:
  KNNHeap() : count(0), worst_(0), max_dist2_(0.0) {
    std::memset(dist2_, 0, sizeof(dist2_));
  }

  void reset() {
    count     = 0;
    worst_    = 0;
    max_dist2_ = 0.0;
    std::memset(dist2_, 0, sizeof(dist2_));
  }

  /// Try to insert a candidate point.  Accepted when the heap is not yet full
  /// OR when the new distance is strictly smaller than the current worst.
  inline void tryInsert(double dist2, const Eigen::Vector3d& pt) {
    const bool not_full      = (count < K);
    const bool should_insert = not_full || (dist2 < max_dist2_);

    if (!should_insert) return;

    const uint8_t idx = not_full ? count : worst_;
    dist2_[idx]  = dist2;
    points_[idx] = pt;

    if (not_full) {
      ++count;
      if (dist2 > max_dist2_) {
        max_dist2_ = dist2;
        worst_     = idx;
      }
    } else {
      updateWorst();
    }
  }

  /// Worst (largest) squared distance currently in the heap.
  inline double worstDist() const { return max_dist2_; }

  uint8_t count;
  uint8_t worst_;
  double  max_dist2_;
  double  dist2_[K];
  std::array<Eigen::Vector3d, K> points_;

private:
  /// Branchless linear scan to find new worst entry after a replacement.
  /// Unrolled for K==5 (the primary use case); falls back to a loop otherwise.
  inline void updateWorst() {
    double  best_d   = dist2_[0];
    uint8_t best_idx = 0;
    for (int i = 1; i < K; ++i) {
      if (dist2_[i] > best_d) {
        best_d   = dist2_[i];
        best_idx = static_cast<uint8_t>(i);
      }
    }
    worst_     = best_idx;
    max_dist2_ = best_d;
  }
};

// ---------------------------------------------------------------------------
// OctVox
//
// A single voxel split into a 2x2x2 grid of sub-voxels (octants).  Each
// sub-voxel stores one representative point computed as an incremental mean,
// and a count (capped at MAX_POINTS_PER_SUBVOXEL).
//
// Sub-voxel index encoding: local_idx = (dz<<2)|(dy<<1)|dx
//   where dx/dy/dz are the least-significant bits of the fine-grid key.
// ---------------------------------------------------------------------------
class OctVox {
public:
  static constexpr uint8_t UNINIT_MASK              = 0x00;
  static constexpr uint8_t MAX_POINTS_PER_SUBVOXEL  = 20;
  static constexpr double  DISTANCE_THRESHOLD_SQ    = 0.1 * 0.1;  // metres²

  OctVox(const Eigen::Vector3d& pt, uint8_t local_idx) {
    counts_.fill(UNINIT_MASK);
    points_[local_idx] = pt;
    counts_[local_idx] = 1;
  }

  void addPoint(const Eigen::Vector3d& pt, uint8_t local_idx) {
    uint8_t&       cnt   = counts_[local_idx];
    Eigen::Vector3d& rep = points_[local_idx];

    if (cnt == UNINIT_MASK) {
      rep = pt;
      cnt = 1;
      return;
    }
    if (cnt >= MAX_POINTS_PER_SUBVOXEL) return;
    if ((pt - rep).squaredNorm() > DISTANCE_THRESHOLD_SQ) return;

    // Incremental mean update.
    rep = (rep * cnt + pt) / (cnt + 1);
    ++cnt;
  }

  bool getPoint(uint8_t local_idx, Eigen::Vector3d& pt) const {
    if (counts_[local_idx] == UNINIT_MASK) return false;
    pt = points_[local_idx];
    return true;
  }

  std::array<uint8_t,       8> counts_;
  std::array<Eigen::Vector3d, 8> points_;
};

// ---------------------------------------------------------------------------
// OctVoxMap
//
// Maintains an LRU-ordered collection of OctVox voxels indexed by a 3-D
// integer key.  Supports bulk point insertion and HKNN queries using the
// precomputed traversal tables in hknn_tables.hpp.
// ---------------------------------------------------------------------------
class OctVoxMap {
public:
  using Ptr = std::shared_ptr<OctVoxMap>;
  using Key = Eigen::Vector3i;

  struct Options {
    double  resolution = 0.5;
    size_t  capacity   = 2000000;

    Options() = default;
    Options(double res, size_t cap) : resolution(res), capacity(cap) {}
  };

  // -------------------------------------------------------------------------
  // Construction
  // -------------------------------------------------------------------------
  OctVoxMap() {
    initSearchPtrs();
  }

  explicit OctVoxMap(const Options& opts) {
    setOptions(opts);
    std::cout << " ---> OctVoxMap init. Resolution: " << resolution_
              << " Capacity: " << capacity_ << '\n';
    initSearchPtrs();
  }

  // -------------------------------------------------------------------------
  // Configuration
  // -------------------------------------------------------------------------
  void setOptions(const Options& opts) {
    resolution_      = opts.resolution;
    capacity_        = opts.capacity;
    inv_resolution_  = 1.0 / resolution_;
    sub_resolution_  = resolution_ / 2.0;
    sub_inv_res_     = 1.0 / sub_resolution_;
  }

  // -------------------------------------------------------------------------
  // Mutation
  // -------------------------------------------------------------------------

  /// Bulk-insert a set of world-frame points.
  void insert(const std::vector<Eigen::Vector3d>& points) {
    for (const auto& pt : points) {
      // Fine-grid key (one sub-voxel = resolution/2 in each dimension).
      Key fine_key = (pt * sub_inv_res_).array().floor().cast<int>();

      // Voxel key (one voxel = resolution in each dimension).
      Key key;
      key[0] = fine_key[0] >> 1;
      key[1] = fine_key[1] >> 1;
      key[2] = fine_key[2] >> 1;

      // Which of the 8 sub-voxels inside this voxel.
      const uint8_t dx        = static_cast<uint8_t>(fine_key[0] & 1);
      const uint8_t dy        = static_cast<uint8_t>(fine_key[1] & 1);
      const uint8_t dz        = static_cast<uint8_t>(fine_key[2] & 1);
      const uint8_t local_idx = static_cast<uint8_t>((dz << 2) | (dy << 1) | dx);

      auto it = grids_.find(key);
      if (it == grids_.end()) {
        // New voxel — prepend to LRU list and register in hash map.
        data_.emplace_front(std::piecewise_construct,
                            std::forward_as_tuple(key),
                            std::forward_as_tuple(pt, local_idx));
        grids_.emplace(key, data_.begin());

        // Evict LRU tail when over capacity.
        if (data_.size() >= capacity_) {
          grids_.erase(data_.back().first);
          data_.pop_back();
        }
      } else {
        // Existing voxel — update sub-voxel and move to front (LRU).
        it->second->second.addPoint(pt, local_idx);
        data_.splice(data_.begin(), data_, it->second);
      }
    }
  }

  // -------------------------------------------------------------------------
  // Query
  // -------------------------------------------------------------------------

  /// HKNN search: fill `heap` with (up to K=5) nearest points to `query`.
  void getTopK(const Eigen::Vector3d& query, KNNHeap<5>& heap) const {
    // Determine the voxel key and intra-voxel octant of the query point.
    const Key fine_key = (query * sub_inv_res_).array().floor().cast<int>();

    Key key;
    key[0] = fine_key[0] >> 1;
    key[1] = fine_key[1] >> 1;
    key[2] = fine_key[2] >> 1;

    const int dx        = fine_key[0] & 1;
    const int dy        = fine_key[1] & 1;
    const int dz        = fine_key[2] & 1;
    const int local_idx = (dz << 2) | (dy << 1) | dx;

    // Mirror axis maps table offsets to the actual neighbour directions so
    // that the same traversal tables work for all 8 octants.
    const Key mirror = Key(1 - (dx << 1), 1 - (dy << 1), 1 - (dz << 1));

    // Pre-cache pointers to the 8 closest voxels (group 0 of the traversal).
    static constexpr int PRE_CACHE = 8;
    const OctVox* cached[PRE_CACHE];
    std::fill_n(cached, PRE_CACHE, nullptr);

    for (int i = 0; i < PRE_CACHE; ++i) {
      const Key nkey = key + mirror.cwiseProduct(HKNN_neighbor_voxel[i]);
      auto it = grids_.find(nkey);
      if (it != grids_.end()) {
        cached[i] = &it->second->second;
      }
    }

    Eigen::Vector3d sub_pt;

    // Iterate over distance groups; apply early-exit when heap is full and
    // the group minimum distance exceeds the current worst heap distance.
    for (int g = 0; g < group_idx_max_; ++g) {
      const uint8_t* ptr = search_ptrs_[g];
      const uint8_t* end = search_ptrs_[g + 1];

      while (ptr < end) {
        const uint8_t neighbor_idx = *ptr++;
        uint8_t       data_size    = *ptr++;

        if (neighbor_idx < PRE_CACHE) {
          // Fast path: voxel pointer already cached.
          const OctVox* vox = cached[neighbor_idx];
          if (vox) {
            while (data_size--) {
              const uint8_t sidx = (*ptr++) ^ static_cast<uint8_t>(local_idx);
              if (vox->getPoint(sidx, sub_pt)) {
                heap.tryInsert((sub_pt - query).squaredNorm(), sub_pt);
              }
            }
          } else {
            ptr += data_size;
          }
        } else {
          // General path: look up voxel in the hash map.
          const Key nkey = key + mirror.cwiseProduct(HKNN_neighbor_voxel[neighbor_idx]);
          auto it = grids_.find(nkey);
          if (it != grids_.end()) {
            const OctVox* vox = &it->second->second;
            while (data_size--) {
              const uint8_t sidx = (*ptr++) ^ static_cast<uint8_t>(local_idx);
              if (vox->getPoint(sidx, sub_pt)) {
                heap.tryInsert((sub_pt - query).squaredNorm(), sub_pt);
              }
            }
          } else {
            ptr += data_size;
          }
        }
      }

      // Early exit: heap full and all remaining points are farther away.
      if (heap.count == 5 && heap.max_dist2_ < static_cast<double>(orders_min_dis2[g])) {
        break;
      }
    }
  }

  // -------------------------------------------------------------------------
  // Adaptive group search control
  // -------------------------------------------------------------------------

  /// Reset the maximum group index to the full search range.
  void resetMaxGroup() {
    group_idx_max_ = static_cast<int>(flat_search_order_offsets.size()) - 1;
  }

  /// Shrink the maximum group index by one (down to a minimum of 4).
  void decreaseMaxGroup() {
    if (group_idx_max_ > 4) --group_idx_max_;
  }

  // -------------------------------------------------------------------------
  // Misc
  // -------------------------------------------------------------------------
  void clear() {
    grids_.clear();
    data_.clear();
  }

  void printInfo() const {
    std::cout << " ---> OctVoxMap info. Size: " << data_.size()
              << " Capacity: " << capacity_ << '\n';
  }

  /// Extract all stored points into a flat float buffer (x,y,z triples).
  void getMap(std::vector<float>& output) const {
    output.clear();
    Eigen::Vector3d pt;
    for (const auto& vp : data_) {
      for (uint8_t i = 0; i < 8; ++i) {
        if (!vp.second.getPoint(i, pt)) continue;
        output.push_back(static_cast<float>(pt.x()));
        output.push_back(static_cast<float>(pt.y()));
        output.push_back(static_cast<float>(pt.z()));
      }
    }
  }

private:
  // -------------------------------------------------------------------------
  // Internal types
  // -------------------------------------------------------------------------

  /// HashShiftMix: matches Super-LIO's HASH_VEC exactly.
  struct KeyHash {
    size_t operator()(const Key& v) const {
      size_t h = static_cast<size_t>(v[0]);
      h ^= static_cast<size_t>(v[1]) * 0x9e3779b9UL + (h << 6) + (h >> 2);
      h ^= static_cast<size_t>(v[2]) * 0x85ebca6bUL + (h << 6) + (h >> 2);
      return h;
    }
  };

  using DataList = std::list<std::pair<Key, OctVox>>;
  using DataIter = typename DataList::iterator;

  // -------------------------------------------------------------------------
  // Data members
  // -------------------------------------------------------------------------
  double resolution_     = 0.5;
  double inv_resolution_ = 2.0;
  double sub_resolution_ = 0.25;
  double sub_inv_res_    = 4.0;
  size_t capacity_       = 2000000;

  DataList data_;
  std::unordered_map<Key, DataIter, KeyHash> grids_;

  /// Pre-computed raw pointers into flat_search_order, one per group boundary.
  std::vector<const uint8_t*> search_ptrs_;
  int group_idx_max_ = 0;

  // -------------------------------------------------------------------------
  // Initialisation helpers
  // -------------------------------------------------------------------------
  void initSearchPtrs() {
    search_ptrs_.reserve(flat_search_order_offsets.size());
    for (size_t i = 0; i < flat_search_order_offsets.size(); ++i) {
      search_ptrs_.push_back(flat_search_order.data() + flat_search_order_offsets[i]);
    }
    group_idx_max_ = static_cast<int>(flat_search_order_offsets.size()) - 1;
  }
};

}  // namespace tof_slam
