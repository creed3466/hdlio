// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// point_types.hpp — Lightweight 3-D point and point-cloud container.
//
// No ROS or PCL dependency.  Used throughout the core algorithm pipeline.

#ifndef TOF_SLAM_COMMON_TYPES_POINT_TYPES_HPP_
#define TOF_SLAM_COMMON_TYPES_POINT_TYPES_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <vector>

#include "tof_slam/common/lie/se3.hpp"

namespace tof_slam {
namespace core {

// ---------------------------------------------------------------------------
// Point3D
// ---------------------------------------------------------------------------

struct Point3D {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float intensity = 0.0f;
  float offset_time = 0.0f;  // Seconds from scan start.

  // -- Constructors ---------------------------------------------------------

  Point3D() = default;

  Point3D(float x, float y, float z)
      : x(x), y(y), z(z) {}

  Point3D(float x, float y, float z, float intensity)
      : x(x), y(y), z(z), intensity(intensity) {}

  Point3D(float x, float y, float z, float intensity, float offset_time)
      : x(x), y(y), z(z), intensity(intensity), offset_time(offset_time) {}

  // -- Arithmetic -----------------------------------------------------------

  Point3D operator+(const Point3D& rhs) const {
    return {x + rhs.x, y + rhs.y, z + rhs.z};
  }

  Point3D operator-(const Point3D& rhs) const {
    return {x - rhs.x, y - rhs.y, z - rhs.z};
  }

  Point3D operator*(float s) const { return {x * s, y * s, z * s}; }

  // -- Distance / norm ------------------------------------------------------

  float norm() const { return std::sqrt(x * x + y * y + z * z); }

  float squared_norm() const { return x * x + y * y + z * z; }

  float distance_to(const Point3D& other) const {
    return (*this - other).norm();
  }

  // -- Eigen conversion -----------------------------------------------------

  Eigen::Vector3f to_eigen() const { return {x, y, z}; }

  static Point3D from_eigen(const Eigen::Vector3f& v) {
    return {v.x(), v.y(), v.z()};
  }
};

// ---------------------------------------------------------------------------
// PointCloud — std::vector<Point3D> wrapper with transform helper.
// ---------------------------------------------------------------------------

class PointCloud {
 public:
  using Ptr = std::shared_ptr<PointCloud>;
  using ConstPtr = std::shared_ptr<const PointCloud>;

  // -- STL-like interface ---------------------------------------------------

  void push_back(const Point3D& p) { points_.push_back(p); }
  void reserve(size_t n) { points_.reserve(n); }
  void clear() { points_.clear(); }
  bool empty() const { return points_.empty(); }
  size_t size() const { return points_.size(); }

  Point3D& operator[](size_t i) { return points_[i]; }
  const Point3D& operator[](size_t i) const { return points_[i]; }

  auto begin() { return points_.begin(); }
  auto end() { return points_.end(); }
  auto begin() const { return points_.cbegin(); }
  auto end() const { return points_.cend(); }

  // -- Accessors ------------------------------------------------------------

  std::vector<Point3D>& points() { return points_; }
  const std::vector<Point3D>& points() const { return points_; }

  // -- Geometric operations -------------------------------------------------

  /// Transform all points in-place by an SE(3) transform.
  void transform(const Se3& T) {
    for (auto& p : points_) {
      const Eigen::Vector3f v = T * p.to_eigen();
      p.x = v.x();
      p.y = v.y();
      p.z = v.z();
    }
  }

  /// Return a transformed copy (original unchanged).
  PointCloud transformed_copy(const Se3& T) const {
    PointCloud out;
    out.reserve(size());
    for (const auto& p : points_) {
      const Eigen::Vector3f v = T * p.to_eigen();
      Point3D q = p;  // Preserve intensity / offset_time.
      q.x = v.x();
      q.y = v.y();
      q.z = v.z();
      out.push_back(q);
    }
    return out;
  }

  /// Compute axis-aligned bounding box.
  void bounding_box(Eigen::Vector3f& min_pt, Eigen::Vector3f& max_pt) const {
    if (points_.empty()) {
      min_pt = max_pt = Eigen::Vector3f::Zero();
      return;
    }
    min_pt = max_pt = points_[0].to_eigen();
    for (size_t i = 1; i < points_.size(); ++i) {
      const auto e = points_[i].to_eigen();
      min_pt = min_pt.cwiseMin(e);
      max_pt = max_pt.cwiseMax(e);
    }
  }

  /// Centroid of all points.
  Eigen::Vector3f centroid() const {
    Eigen::Vector3f sum = Eigen::Vector3f::Zero();
    for (const auto& p : points_) {
      sum += p.to_eigen();
    }
    return points_.empty() ? sum : sum / static_cast<float>(points_.size());
  }

 private:
  std::vector<Point3D> points_;
};

using PointCloudPtr = std::shared_ptr<PointCloud>;
using PointCloudConstPtr = std::shared_ptr<const PointCloud>;

}  // namespace core
}  // namespace tof_slam

#endif  // TOF_SLAM_COMMON_TYPES_POINT_TYPES_HPP_
