// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// map_saver.cpp -- Save loop-closure-optimized map for localization mode.

#include "tof_slam/backend/map_saver.hpp"
#include "tof_slam/backend/loop_closure_manager.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace tof_slam {

namespace {

// Magic bytes for pose graph binary format
constexpr uint32_t kPoseGraphMagic   = 0x54535047;  // "TSPG"
constexpr uint32_t kPoseGraphVersion = 1;

/// Write a plain-old-data value to a binary stream.
template <typename T>
void writePod(std::ofstream& ofs, const T& val) {
  ofs.write(reinterpret_cast<const char*>(&val), sizeof(T));
}

/// Current UTC timestamp as ISO-8601 string (best-effort).
std::string utcTimestamp() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf{};
#ifdef _WIN32
  gmtime_s(&tm_buf, &t);
#else
  gmtime_r(&t, &tm_buf);
#endif
  std::ostringstream oss;
  oss << std::put_time(&tm_buf, "%Y-%m-%dT%H:%M:%SZ");
  return oss.str();
}

}  // namespace

MapSaver::MapSaver(const MapSaveConfig& config) : config_(config) {}

MapSaveResult MapSaver::save(const std::string& save_dir,
                             const LoopClosureManager& manager) {
  spdlog::info("[MapSaver] Starting map save to '{}'", save_dir);

  // Create output directory (and parents) if they do not exist.
  std::error_code ec;
  std::filesystem::create_directories(save_dir, ec);
  if (ec) {
    MapSaveResult err;
    err.success = false;
    err.message = "Failed to create directory '" + save_dir + "': " + ec.message();
    spdlog::error("[MapSaver] {}", err.message);
    return err;
  }

  // 1. Build and save global map (mandatory).
  MapSaveResult result = buildAndSaveGlobalMap(save_dir, manager);
  if (!result.success) {
    return result;
  }

  // 2. Save pose graph (optional, but strongly recommended).
  if (config_.save_pose_graph) {
    const std::string pg_path = save_dir + "/pose_graph.bin";
    if (!savePoseGraph(pg_path, manager)) {
      spdlog::warn("[MapSaver] Pose graph save failed; continuing.");
    }
  }

  // 3. Save individual submaps (optional).
  if (config_.save_submaps) {
    const std::string submap_dir = save_dir + "/submaps";
    if (!saveSubmaps(submap_dir, manager)) {
      spdlog::warn("[MapSaver] Submap save had issues; continuing.");
    }
  }

  // 4. Write metadata YAML.
  const std::string meta_path = save_dir + "/metadata.yaml";
  if (!saveMetadata(meta_path, result)) {
    spdlog::warn("[MapSaver] Metadata write failed; continuing.");
  }

  result.save_path = save_dir;
  spdlog::info("[MapSaver] Map saved successfully. "
               "keyframes={} submaps={} loops={} raw_pts={} map_pts={}",
               result.num_keyframes, result.num_submaps,
               result.num_loop_closures,
               result.total_raw_points, result.global_map_points);
  return result;
}

MapSaveResult MapSaver::buildAndSaveGlobalMap(const std::string& save_dir,
                                              const LoopClosureManager& manager) {
  MapSaveResult result;
  result.success = false;

  const auto& keyframes = manager.keyframeManager().getAllKeyframes();
  if (keyframes.empty()) {
    result.message = "No keyframes available; nothing to save.";
    spdlog::warn("[MapSaver] {}", result.message);
    return result;
  }

  result.num_keyframes = static_cast<int>(keyframes.size());
  result.num_submaps   = static_cast<int>(manager.numSubmaps());

  // Count loop closure edges in the pose graph.
  for (const auto& edge : manager.poseGraph().getEdges()) {
    if (edge.type == PoseGraphEdge::LOOP) {
      ++result.num_loop_closures;
    }
  }

  spdlog::info("[MapSaver] Building global map from {} keyframes ...",
               result.num_keyframes);

  pcl::PointCloud<pcl::PointXYZ>::Ptr merged(
      new pcl::PointCloud<pcl::PointXYZ>);

  // Compute bounds and accumulate world-frame points.
  Eigen::Vector3d bounds_min = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::max());
  Eigen::Vector3d bounds_max = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::lowest());

  for (const auto& kf : keyframes) {
    if (!kf.cloud || kf.cloud->empty()) {
      continue;
    }

    // Prefer PGO-optimised pose; fall back to IEKF pose.
    Eigen::Quaterniond q;
    Eigen::Vector3d    p;
    if (!manager.poseGraph().getOptimizedPose(kf.id, q, p)) {
      q = kf.state.q_wb;
      p = kf.state.p_wb;
    }

    // Build 4x4 rigid transform: T_world_body.
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = q.toRotationMatrix().cast<float>();
    T.block<3, 1>(0, 3) = p.cast<float>();

    // Transform cloud from body frame to world frame and append.
    pcl::PointCloud<pcl::PointXYZ> cloud_world;
    pcl::transformPointCloud(*kf.cloud, cloud_world, T);

    result.total_raw_points += static_cast<int>(cloud_world.size());

    for (const auto& pt : cloud_world) {
      merged->push_back(pt);
      bounds_min.x() = std::min(bounds_min.x(), static_cast<double>(pt.x));
      bounds_min.y() = std::min(bounds_min.y(), static_cast<double>(pt.y));
      bounds_min.z() = std::min(bounds_min.z(), static_cast<double>(pt.z));
      bounds_max.x() = std::max(bounds_max.x(), static_cast<double>(pt.x));
      bounds_max.y() = std::max(bounds_max.y(), static_cast<double>(pt.y));
      bounds_max.z() = std::max(bounds_max.z(), static_cast<double>(pt.z));
    }
  }

  if (merged->empty()) {
    result.message = "All keyframe clouds are empty; nothing to save.";
    spdlog::error("[MapSaver] {}", result.message);
    return result;
  }

  result.bounds_min = bounds_min;
  result.bounds_max = bounds_max;

  spdlog::info("[MapSaver] Merged {} raw points; applying voxel filter "
               "(leaf={:.3f}m) ...",
               merged->size(), config_.save_voxel_size);

  // Voxel downsample.
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(
      new pcl::PointCloud<pcl::PointXYZ>);
  if (config_.save_voxel_size > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(merged);
    vg.setLeafSize(static_cast<float>(config_.save_voxel_size),
                   static_cast<float>(config_.save_voxel_size),
                   static_cast<float>(config_.save_voxel_size));
    vg.filter(*filtered);
  } else {
    filtered = merged;
  }

  result.global_map_points = static_cast<int>(filtered->size());

  spdlog::info("[MapSaver] After voxel filter: {} points. Saving PCD ...",
               result.global_map_points);

  // Save as binary PCD.
  const std::string pcd_path = save_dir + "/global_map.pcd";
  if (pcl::io::savePCDFileBinary(pcd_path, *filtered) != 0) {
    result.message = "pcl::io::savePCDFileBinary failed for '" + pcd_path + "'";
    spdlog::error("[MapSaver] {}", result.message);
    return result;
  }

  spdlog::info("[MapSaver] global_map.pcd saved: {}", pcd_path);
  result.success = true;
  result.message = "OK";
  return result;
}

// Binary layout: [Header: magic|version|num_nodes|num_edges]
//   [Node: id(u64), qwxyz(f64×4), pxyz(f64×3)] × num_nodes
//   [Edge: from|to(u64), T_rel(f64×16), info(f64×36), type(u8)] × num_edges
bool MapSaver::savePoseGraph(const std::string& filepath,
                             const LoopClosureManager& manager) {
  std::ofstream ofs(filepath, std::ios::binary);
  if (!ofs.is_open()) {
    spdlog::error("[MapSaver] Cannot open '{}' for writing", filepath);
    return false;
  }

  const auto& nodes = manager.poseGraph().getNodes();
  const auto& edges = manager.poseGraph().getEdges();

  const auto num_nodes = static_cast<uint32_t>(nodes.size());
  const auto num_edges = static_cast<uint32_t>(edges.size());

  // Header
  writePod(ofs, kPoseGraphMagic);
  writePod(ofs, kPoseGraphVersion);
  writePod(ofs, num_nodes);
  writePod(ofs, num_edges);

  // Nodes
  for (const auto& node : nodes) {
    writePod(ofs, static_cast<uint64_t>(node.id));
    writePod(ofs, node.q.w());
    writePod(ofs, node.q.x());
    writePod(ofs, node.q.y());
    writePod(ofs, node.q.z());
    writePod(ofs, node.p.x());
    writePod(ofs, node.p.y());
    writePod(ofs, node.p.z());
  }

  // Edges
  for (const auto& edge : edges) {
    writePod(ofs, static_cast<uint64_t>(edge.from_id));
    writePod(ofs, static_cast<uint64_t>(edge.to_id));

    // T_relative as row-major 4×4 doubles
    Eigen::Matrix4d T_row = edge.T_relative;
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        writePod(ofs, T_row(r, c));
      }
    }

    // information as row-major 6×6 doubles
    for (int r = 0; r < 6; ++r) {
      for (int c = 0; c < 6; ++c) {
        writePod(ofs, edge.information(r, c));
      }
    }

    // Edge type as single byte (0=WHEEL, 1=SCAN, 2=LOOP)
    const uint8_t type_byte = static_cast<uint8_t>(edge.type);
    writePod(ofs, type_byte);
  }

  if (!ofs.good()) {
    spdlog::error("[MapSaver] Stream error while writing '{}'", filepath);
    return false;
  }

  spdlog::info("[MapSaver] pose_graph.bin saved: {} nodes, {} edges -> '{}'",
               num_nodes, num_edges, filepath);
  return true;
}

bool MapSaver::saveSubmaps(const std::string& submap_dir,
                           const LoopClosureManager& manager) {
  const auto submaps = manager.getVisualSubmapSnapshots();
  if (submaps.empty()) {
    spdlog::info("[MapSaver] No submaps to save.");
    return true;
  }

  std::error_code ec;
  std::filesystem::create_directories(submap_dir, ec);
  if (ec) {
    spdlog::error("[MapSaver] Cannot create submap dir '{}': {}",
                  submap_dir, ec.message());
    return false;
  }

  int saved = 0;
  for (const auto& sm : submaps) {
    if (!sm.world_cloud || sm.world_cloud->empty()) continue;

    std::ostringstream fname;
    fname << submap_dir << "/submap_"
          << std::setfill('0') << std::setw(3) << sm.id << ".pcd";
    if (pcl::io::savePCDFileBinary(fname.str(), *sm.world_cloud) == 0) {
      ++saved;
    } else {
      spdlog::warn("[MapSaver] Failed to save submap {}", sm.id);
    }
  }

  spdlog::info("[MapSaver] Saved {}/{} submaps to '{}'",
               saved, submaps.size(), submap_dir);
  return saved > 0;
}

bool MapSaver::saveMetadata(const std::string& filepath,
                            const MapSaveResult& result) {
  std::ofstream ofs(filepath);
  if (!ofs.is_open()) {
    spdlog::error("[MapSaver] Cannot open '{}' for writing", filepath);
    return false;
  }

  ofs << "# TofSLAM saved-map metadata\n";
  ofs << "version: 1\n";
  ofs << "timestamp: \"" << utcTimestamp() << "\"\n";
  ofs << "\n";

  ofs << "frames:\n";
  ofs << "  map:   \"" << config_.map_frame   << "\"\n";
  ofs << "  base:  \"" << config_.base_frame  << "\"\n";
  ofs << "  lidar: \"" << config_.lidar_frame << "\"\n";
  ofs << "\n";

  ofs << "extrinsic:\n";
  ofs << std::fixed << std::setprecision(6);
  ofs << "  x:     " << config_.extrinsic_x     << "\n";
  ofs << "  y:     " << config_.extrinsic_y     << "\n";
  ofs << "  z:     " << config_.extrinsic_z     << "\n";
  ofs << "  roll:  " << config_.extrinsic_roll  << "\n";
  ofs << "  pitch: " << config_.extrinsic_pitch << "\n";
  ofs << "  yaw:   " << config_.extrinsic_yaw   << "\n";
  ofs << "\n";

  ofs << "surfel_map:\n";
  ofs << "  l0_voxel_size:        " << config_.surfel_l0_voxel_size      << "\n";
  ofs << "  l1_hierarchy_factor:  " << config_.surfel_l1_hierarchy_factor << "\n";
  ofs << "  planarity_threshold:  " << config_.surfel_planarity_threshold << "\n";
  ofs << "  min_l0_for_surfel:    " << config_.surfel_min_l0_for_surfel   << "\n";
  ofs << "\n";

  ofs << "map_save:\n";
  ofs << "  voxel_size: " << config_.save_voxel_size << "\n";
  ofs << "\n";

  ofs << "statistics:\n";
  ofs << "  num_keyframes:     " << result.num_keyframes     << "\n";
  ofs << "  num_submaps:       " << result.num_submaps       << "\n";
  ofs << "  num_loop_closures: " << result.num_loop_closures << "\n";
  ofs << "  total_raw_points:  " << result.total_raw_points  << "\n";
  ofs << "  global_map_points: " << result.global_map_points << "\n";
  ofs << "\n";

  ofs << "bounds:\n";
  ofs << "  min: [" << result.bounds_min.x() << ", "
                    << result.bounds_min.y() << ", "
                    << result.bounds_min.z() << "]\n";
  ofs << "  max: [" << result.bounds_max.x() << ", "
                    << result.bounds_max.y() << ", "
                    << result.bounds_max.z() << "]\n";

  if (!ofs.good()) {
    spdlog::error("[MapSaver] Stream error while writing '{}'", filepath);
    return false;
  }

  spdlog::info("[MapSaver] metadata.yaml saved: '{}'", filepath);
  return true;
}

// Nav2 map_server format: map.png (8-bit grayscale) + map.yaml
// Pixels: 0=occupied, 254=free, 205=unknown
bool MapSaver::saveOccupancyGridMap(const std::string& save_dir,
                                    const OccupancyGridGenerator::GlobalGrid& grid) {
  if (grid.width <= 0 || grid.height <= 0 || grid.data.empty()) {
    spdlog::warn("[MapSaver] OGM grid is empty; skipping.");
    return false;
  }

  std::error_code ec;
  std::filesystem::create_directories(save_dir, ec);
  if (ec) {
    spdlog::error("[MapSaver] Cannot create OGM dir '{}': {}", save_dir, ec.message());
    return false;
  }

  // Build 8-bit grayscale image
  // OccupancyGrid: row-major, bottom-left origin
  // PNG/OpenCV: row-major, top-left origin → flip Y
  cv::Mat image(grid.height, grid.width, CV_8UC1);

  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      int ogm_idx = y * grid.width + x;
      int img_row = grid.height - 1 - y;  // Y-flip for image coordinates

      int8_t val = grid.data[ogm_idx];
      uint8_t pixel;
      if (val < 0) {
        pixel = 205;  // unknown (nav2 convention)
      } else {
        // Map [0, 100] → [254, 0]  (0=free→white, 100=occupied→black)
        pixel = static_cast<uint8_t>(
            std::clamp(254 - static_cast<int>(val) * 254 / 100, 0, 254));
      }
      image.at<uint8_t>(img_row, x) = pixel;
    }
  }

  // Save PNG
  const std::string png_path = save_dir + "/map.png";
  if (!cv::imwrite(png_path, image)) {
    spdlog::error("[MapSaver] Failed to write OGM PNG: '{}'", png_path);
    return false;
  }

  // Save YAML (Nav2 map_server format)
  const std::string yaml_path = save_dir + "/map.yaml";
  std::ofstream yaml(yaml_path);
  if (!yaml.is_open()) {
    spdlog::error("[MapSaver] Cannot open OGM YAML: '{}'", yaml_path);
    return false;
  }

  yaml << std::fixed << std::setprecision(6);
  yaml << "image: map.png\n";
  yaml << "mode: trinary\n";
  yaml << "resolution: " << grid.resolution << "\n";
  yaml << "origin: [" << grid.origin_x << ", " << grid.origin_y << ", 0.000000]\n";
  yaml << "negate: 0\n";
  yaml << "occupied_thresh: 0.65\n";
  yaml << "free_thresh: 0.25\n";

  if (!yaml.good()) {
    spdlog::error("[MapSaver] Stream error writing OGM YAML: '{}'", yaml_path);
    return false;
  }

  spdlog::info("[MapSaver] OGM saved: {}x{} -> '{}' + '{}'",
               grid.width, grid.height, png_path, yaml_path);
  return true;
}

}  // namespace tof_slam
