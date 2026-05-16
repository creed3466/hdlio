// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// point_cloud_adapter.cpp — Converts ROS PointCloud2 to core::PointCloud.
//
// Uses raw byte-buffer access (memcpy) rather than PointCloud2ConstIterator
// to avoid type-punning UB and to keep field access explicit.

#include "tof_slam/ros/point_cloud_adapter.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>

#include <spdlog/spdlog.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace tof_slam {
namespace ros_adapter {

namespace {

// ---------------------------------------------------------------------------
// Field-lookup helpers
// ---------------------------------------------------------------------------

/// Returns the byte offset of a named field, or -1 if absent.
int32_t field_offset(
    const std::vector<sensor_msgs::msg::PointField>& fields,
    const std::string& name) {
  for (const auto& f : fields) {
    if (f.name == name) {
      return static_cast<int32_t>(f.offset);
    }
  }
  return -1;
}

/// Returns the datatype code of a named field, or 0 if absent.
uint8_t field_datatype(
    const std::vector<sensor_msgs::msg::PointField>& fields,
    const std::string& name) {
  for (const auto& f : fields) {
    if (f.name == name) {
      return f.datatype;
    }
  }
  return 0;
}

// ---------------------------------------------------------------------------
// Typed read from raw byte buffer (safe, no UB)
// ---------------------------------------------------------------------------

template <typename T>
inline T read_field(const uint8_t* row, int32_t offset) {
  T value;
  std::memcpy(&value, row + offset, sizeof(T));
  return value;
}

// ---------------------------------------------------------------------------
// NaN / Inf guard
// ---------------------------------------------------------------------------

inline bool is_finite_point(float x, float y, float z) {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

}  // namespace

// ---------------------------------------------------------------------------
// PointCloudAdapter — public API
// ---------------------------------------------------------------------------

core::PointCloud PointCloudAdapter::convert(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg) const {
  if (!msg) {
    return core::PointCloud{};
  }
  const double scan_ts =
      static_cast<double>(msg->header.stamp.sec) +
      static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
  return convert(msg, scan_ts);
}

core::PointCloud PointCloudAdapter::convert(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
    double scan_timestamp) const {
  if (!msg) {
    return core::PointCloud{};
  }

  const auto& fields = msg->fields;
  const uint32_t point_step = msg->point_step;
  const uint32_t n_points   = msg->width * msg->height;

  // Locate mandatory x/y/z fields.
  const int32_t off_x = field_offset(fields, "x");
  const int32_t off_y = field_offset(fields, "y");
  const int32_t off_z = field_offset(fields, "z");

  if (off_x < 0 || off_y < 0 || off_z < 0) {
    // Malformed message — return empty cloud rather than throwing.
    return core::PointCloud{};
  }

  // Locate optional fields.
  const int32_t off_intensity   = field_offset(fields, "intensity");
  const int32_t off_timestamp   = field_offset(fields, "timestamp");
  const int32_t off_offset_time = field_offset(fields, "offset_time");
  const int32_t off_t           = field_offset(fields, "t");  // Ouster

  const bool has_intensity   = (off_intensity   >= 0);
  const bool has_timestamp   = (off_timestamp   >= 0);
  const bool has_offset_time = (off_offset_time >= 0);
  const bool has_t           = (off_t           >= 0);

  // Verify the "timestamp" field is actually FLOAT64 (8 bytes).
  // livox_ros_driver2 stores absolute seconds as a double.
  const uint8_t ts_datatype = field_datatype(fields, "timestamp");
  const bool timestamp_is_double =
      has_timestamp && (ts_datatype == sensor_msgs::msg::PointField::FLOAT64);

  // "offset_time" is conventionally UINT32 (nanoseconds).
  const uint8_t ot_datatype = field_datatype(fields, "offset_time");
  const bool offset_time_is_uint32 =
      has_offset_time && (ot_datatype == sensor_msgs::msg::PointField::UINT32);

  // Ouster "t" field — UINT32 (nanoseconds offset from scan start).
  const uint8_t t_datatype = field_datatype(fields, "t");
  const bool t_is_uint32 =
      has_t && (t_datatype == sensor_msgs::msg::PointField::UINT32);

  // Velodyne "time" field — FLOAT32 (seconds, offset from scan END, negative).
  const int32_t off_time = field_offset(fields, "time");
  const bool has_time = (off_time >= 0);
  const uint8_t time_datatype = field_datatype(fields, "time");
  const bool time_is_float32 =
      has_time && (time_datatype == sensor_msgs::msg::PointField::FLOAT32);

  core::PointCloud cloud;
  cloud.reserve(static_cast<size_t>(n_points));

  const uint8_t* data = msg->data.data();

  for (uint32_t i = 0; i < n_points; ++i) {
    const uint8_t* row = data + static_cast<size_t>(i) * point_step;

    const float x = read_field<float>(row, off_x);
    const float y = read_field<float>(row, off_y);
    const float z = read_field<float>(row, off_z);

    if (!is_finite_point(x, y, z)) {
      continue;
    }

    core::Point3D pt;
    pt.x = x;
    pt.y = y;
    pt.z = z;

    // -- Intensity ------------------------------------------------------------
    if (has_intensity) {
      pt.intensity = read_field<float>(row, off_intensity);
    } else {
      pt.intensity = 0.0f;
    }

    // -- Timing ---------------------------------------------------------------
    if (has_timestamp && timestamp_is_double) {
      // Absolute time -> offset from scan start.
      // Livox MID-360 stores nanoseconds as FLOAT64; detect and convert.
      const double abs_ts_raw = read_field<double>(row, off_timestamp);
      const double abs_ts = (abs_ts_raw > 1e15) ? abs_ts_raw * 1e-9 : abs_ts_raw;
      pt.offset_time = static_cast<float>(abs_ts - scan_timestamp);
    } else if (has_offset_time && offset_time_is_uint32) {
      // Relative time (uint32 nanoseconds) -> seconds.
      const uint32_t ot_ns = read_field<uint32_t>(row, off_offset_time);
      pt.offset_time =
          static_cast<float>(static_cast<double>(ot_ns) * 1e-9);
    } else if (has_t && t_is_uint32) {
      // Ouster "t" field: UINT32 nanoseconds offset from scan start.
      const uint32_t t_ns = read_field<uint32_t>(row, off_t);
      pt.offset_time =
          static_cast<float>(static_cast<double>(t_ns) * 1e-9);
    } else if (has_time && time_is_float32) {
      // Velodyne "time" field: FLOAT32 seconds, typically negative
      // (offset from scan END, range ~[-0.1, 0]).
      // Store raw value; will shift to [0, scan_duration] after the loop.
      pt.offset_time = read_field<float>(row, off_time);
    } else {
      pt.offset_time = 0.0f;
    }

    cloud.push_back(pt);
  }

  // Post-process: Velodyne "time" field has negative offsets (from scan END).
  // Shift so that the earliest point has offset_time ~ 0.
  if (has_time && time_is_float32 && !cloud.empty()) {
    float min_t = cloud[0].offset_time;
    for (const auto& p : cloud) {
      min_t = std::min(min_t, p.offset_time);
    }
    if (min_t < 0.0f) {
      for (auto& p : cloud) {
        p.offset_time -= min_t;  // shift: [-0.1, 0] -> [0, 0.1]
      }
    }
  }

  // Debug: log timestamp diagnostics for first 3 scans.
  {
    static int debug_scan_count = 0;
    if (debug_scan_count < 3 && !cloud.empty()) {
      // Sample offset_time statistics.
      float min_ot = cloud[0].offset_time;
      float max_ot = cloud[0].offset_time;
      int zero_count = 0;
      for (const auto& p : cloud) {
        min_ot = std::min(min_ot, p.offset_time);
        max_ot = std::max(max_ot, p.offset_time);
        if (std::abs(p.offset_time) < 1e-9f) ++zero_count;
      }
      // Also read raw timestamp of first point for diagnosis.
      double raw_ts_first = 0.0;
      if (has_timestamp && timestamp_is_double && n_points > 0) {
        raw_ts_first = read_field<double>(data, off_timestamp);
      }
      SPDLOG_INFO("[PointCloudAdapter] scan={} pts={} scan_ts={:.9f} "
                  "raw_ts[0]={:.9f} offset_time: min={:.6f} max={:.6f} "
                  "zero={}/{} has_ts={} has_ot={} has_velo_time={}",
                  debug_scan_count, cloud.size(), scan_timestamp,
                  raw_ts_first, min_ot, max_ot,
                  zero_count, cloud.size(),
                  has_timestamp && timestamp_is_double,
                  has_offset_time && offset_time_is_uint32,
                  has_time && time_is_float32);
      ++debug_scan_count;
    }
  }

  return cloud;
}

}  // namespace ros_adapter
}  // namespace tof_slam
