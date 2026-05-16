#include "tof_slam/mapping/occupancy_grid.hpp"

#include <cmath>
#include <algorithm>
#include <limits>

namespace tof_slam {

namespace {

double snapDownToResolution(double value, double resolution) {
  return std::floor(value / resolution) * resolution;
}

double snapUpToResolution(double value, double resolution) {
  return std::ceil(value / resolution) * resolution;
}

}  // namespace

OccupancyGridGenerator::OccupancyGridGenerator(const TofSlamConfig& config)
    : config_(config) {}

LocalGrid OccupancyGridGenerator::generateLocalGrid(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_body,
    size_t keyframe_id) const {
  LocalGrid grid;
  grid.keyframe_id = keyframe_id;
  grid.resolution = config_.ogm_resolution;

  double range = config_.ogm_local_range;
  grid.width = static_cast<int>(std::ceil(2.0 * range / grid.resolution));
  grid.height = static_cast<int>(std::ceil(2.0 * range / grid.resolution));

  // Grid origin at bottom-left corner (centered on robot)
  grid.origin_x = -range;
  grid.origin_y = -range;

  // Initialize all cells to 0 (unknown in log-odds)
  grid.log_odds.assign(grid.width * grid.height, 0.0);

  if (!cloud_body || cloud_body->empty()) {
    return grid;
  }

  // Ray casting from robot origin (0,0) to each point (XY projection)
  // Height filter: only use points within [ogm_height_min, ogm_height_max]
  for (const auto& pt : cloud_body->points) {
    double pz = static_cast<double>(pt.z);
    if (pz < config_.ogm_height_min || pz > config_.ogm_height_max) {
      continue;
    }

    double px = static_cast<double>(pt.x);
    double py = static_cast<double>(pt.y);
    double dist = std::sqrt(px * px + py * py);

    if (dist < 0.2 || dist > range) {
      continue;
    }

    // Cast ray from origin to endpoint
    rayCast(grid, 0.0, 0.0, px, py);

    // Mark endpoint as occupied (range-dependent confidence)
    double range_scale = std::max(0.3, 1.0 - 0.1 * (dist - 1.0));
    int gx, gy;
    if (grid.worldToGrid(px, py, gx, gy)) {
      markOccupied(grid, gx, gy, range_scale);
    }
  }

  return grid;
}

void OccupancyGridGenerator::markOccupied(LocalGrid& grid, int gx, int gy,
                                           double range_scale) const {
  const int radius = std::max(0, config_.ogm_endpoint_dilation_radius);
  const double hit = config_.ogm_log_odds_hit * range_scale;
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      // Circular dilation: skip corners outside radius
      if (dx * dx + dy * dy > radius * radius) continue;
      const int idx = grid.cellIndex(gx + dx, gy + dy);
      if (idx >= 0) {
        grid.log_odds[idx] = clampLogOdds(grid.log_odds[idx] + hit);
      }
    }
  }
}

void OccupancyGridGenerator::rayCast(LocalGrid& grid, double x0, double y0,
                                      double x1, double y1) const {
  // Bresenham-style ray casting in grid coordinates
  int gx0, gy0, gx1, gy1;
  grid.worldToGrid(x0, y0, gx0, gy0);
  grid.worldToGrid(x1, y1, gx1, gy1);

  // Clamp to grid bounds
  gx0 = std::clamp(gx0, 0, grid.width - 1);
  gy0 = std::clamp(gy0, 0, grid.height - 1);
  gx1 = std::clamp(gx1, 0, grid.width - 1);
  gy1 = std::clamp(gy1, 0, grid.height - 1);

  int dx = std::abs(gx1 - gx0);
  int dy = std::abs(gy1 - gy0);
  int sx = (gx0 < gx1) ? 1 : -1;
  int sy = (gy0 < gy1) ? 1 : -1;
  int err = dx - dy;

  int cx = gx0, cy = gy0;

  while (true) {
    // Mark intermediate cells as free (but not the endpoint)
    if (cx == gx1 && cy == gy1) {
      break;
    }

    int idx = grid.cellIndex(cx, cy);
    if (idx >= 0) {
      grid.log_odds[idx] = clampLogOdds(
          grid.log_odds[idx] + config_.ogm_log_odds_miss);
    }

    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      cx += sx;
    }
    if (e2 < dx) {
      err += dx;
      cy += sy;
    }
  }
}

double OccupancyGridGenerator::clampLogOdds(double val) const {
  return std::clamp(val, config_.ogm_log_odds_min, config_.ogm_log_odds_max);
}

double OccupancyGridGenerator::logOddsToProbability(double log_odds) {
  return 1.0 / (1.0 + std::exp(-log_odds));
}

void OccupancyGridGenerator::dilateGlobalOccupied(
    std::vector<double>& global_log_odds, int width, int height) const {
  const int radius = std::max(0, config_.ogm_global_dilation_radius);
  if (radius <= 0 || width <= 0 || height <= 0) {
    return;
  }

  std::vector<double> dilated = global_log_odds;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int idx = y * width + x;
      if (global_log_odds[idx] <= 0.0) {
        continue;
      }
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
            continue;
          }
          const int nidx = ny * width + nx;
          dilated[nidx] = std::max(dilated[nidx], config_.ogm_log_odds_hit);
        }
      }
    }
  }
  global_log_odds.swap(dilated);
}

OccupancyGridGenerator::GlobalGrid OccupancyGridGenerator::assembleGlobalGrid(
    const std::vector<LocalGrid>& local_grids,
    const std::vector<Eigen::Matrix4d>& world_poses) const {
  if (local_grids.empty() || local_grids.size() != world_poses.size()) {
    return GlobalGrid{};
  }

  // Find global bounding box. Optionally crop to observed local-grid support,
  // otherwise keep the broader pose-envelope behavior.
  double global_min_x = std::numeric_limits<double>::max();
  double global_min_y = std::numeric_limits<double>::max();
  double global_max_x = std::numeric_limits<double>::lowest();
  double global_max_y = std::numeric_limits<double>::lowest();
  bool has_observed_bounds = false;

  double range = config_.ogm_local_range;

  if (config_.ogm_crop_to_observed_bounds) {
    for (size_t k = 0; k < local_grids.size(); ++k) {
      const auto& lg = local_grids[k];
      const auto& pose = world_poses[k];
      const double cos_yaw = pose(0, 0);
      const double sin_yaw = pose(1, 0);
      const double tx = pose(0, 3);
      const double ty = pose(1, 3);

      for (int ly = 0; ly < lg.height; ++ly) {
        for (int lx = 0; lx < lg.width; ++lx) {
          const int local_idx = ly * lg.width + lx;
          if (std::abs(lg.log_odds[local_idx]) < 1e-6) {
            continue;
          }

          const double lwx = lg.origin_x + (static_cast<double>(lx) + 0.5) * lg.resolution;
          const double lwy = lg.origin_y + (static_cast<double>(ly) + 0.5) * lg.resolution;
          const double gwx = cos_yaw * lwx - sin_yaw * lwy + tx;
          const double gwy = sin_yaw * lwx + cos_yaw * lwy + ty;
          const double half_res = 0.5 * lg.resolution;

          global_min_x = std::min(global_min_x, gwx - half_res);
          global_min_y = std::min(global_min_y, gwy - half_res);
          global_max_x = std::max(global_max_x, gwx + half_res);
          global_max_y = std::max(global_max_y, gwy + half_res);
          has_observed_bounds = true;
        }
      }
    }
  }

  if (!config_.ogm_crop_to_observed_bounds || !has_observed_bounds) {
    for (const auto& pose : world_poses) {
      const double tx = pose(0, 3);
      const double ty = pose(1, 3);
      global_min_x = std::min(global_min_x, tx - range);
      global_min_y = std::min(global_min_y, ty - range);
      global_max_x = std::max(global_max_x, tx + range);
      global_max_y = std::max(global_max_y, ty + range);
    }
  }

  if (config_.ogm_anchor_first_pose && !world_poses.empty()) {
    global_min_x = world_poses.front()(0, 3) - range;
    global_min_y = world_poses.front()(1, 3) - range;
  }

  double res = config_.ogm_resolution;
  GlobalGrid global;
  global.resolution = res;
  global.raw_min_x = global_min_x;
  global.raw_min_y = global_min_y;
  global.raw_max_x = global_max_x;
  global.raw_max_y = global_max_y;
  global.padding_cells = std::max(0, config_.ogm_global_padding_cells);

  const double padding_m = static_cast<double>(global.padding_cells) * res;
  const double biased_min_x = global_min_x + config_.ogm_origin_bias_x;
  const double biased_min_y = global_min_y + config_.ogm_origin_bias_y;
  const double biased_max_x = global_max_x + config_.ogm_origin_bias_x;
  const double biased_max_y = global_max_y + config_.ogm_origin_bias_y;

  global.canvas_min_x = snapDownToResolution(biased_min_x, res) - padding_m;
  global.canvas_min_y = snapDownToResolution(biased_min_y, res) - padding_m;
  global.canvas_max_x = snapUpToResolution(biased_max_x, res) + padding_m;
  global.canvas_max_y = snapUpToResolution(biased_max_y, res) + padding_m;
  global.origin_x = global.canvas_min_x;
  global.origin_y = global.canvas_min_y;
  global.origin_snapped = true;
  global.width = std::max(1, static_cast<int>(std::llround(
      (global.canvas_max_x - global.canvas_min_x) / res)));
  global.height = std::max(1, static_cast<int>(std::llround(
      (global.canvas_max_y - global.canvas_min_y) / res)));

  // Limit max size to prevent memory issues
  if (global.width > 4000 || global.height > 4000) {
    global.width = std::min(global.width, 4000);
    global.height = std::min(global.height, 4000);
    global.canvas_max_x = global.origin_x + static_cast<double>(global.width) * res;
    global.canvas_max_y = global.origin_y + static_cast<double>(global.height) * res;
  }

  // Initialize with log-odds 0 (unknown)
  std::vector<double> global_log_odds(global.width * global.height, 0.0);

  // Overlay each local grid
  for (size_t k = 0; k < local_grids.size(); ++k) {
    const auto& lg = local_grids[k];
    const auto& pose = world_poses[k];
    double cos_yaw = pose(0, 0);
    double sin_yaw = pose(1, 0);
    double tx = pose(0, 3);
    double ty = pose(1, 3);

    for (int ly = 0; ly < lg.height; ++ly) {
      for (int lx = 0; lx < lg.width; ++lx) {
        int local_idx = ly * lg.width + lx;
        double lo = lg.log_odds[local_idx];
        if (std::abs(lo) < 1e-6) continue;  // skip unknown cells

        // Local grid coordinates to local world
        double lwx = lg.origin_x + (lx + 0.5) * lg.resolution;
        double lwy = lg.origin_y + (ly + 0.5) * lg.resolution;

        // Transform to global frame
        double gwx = cos_yaw * lwx - sin_yaw * lwy + tx;
        double gwy = sin_yaw * lwx + cos_yaw * lwy + ty;

        // Global grid coordinates
        int ggx = static_cast<int>(std::floor((gwx - global.origin_x) / res));
        int ggy = static_cast<int>(std::floor((gwy - global.origin_y) / res));

        if (ggx >= 0 && ggx < global.width && ggy >= 0 && ggy < global.height) {
          int gidx = ggy * global.width + ggx;
          global_log_odds[gidx] = clampLogOdds(global_log_odds[gidx] + lo);
        }
      }
    }
  }

  dilateGlobalOccupied(global_log_odds, global.width, global.height);
  morphologicalClean(global_log_odds, global.width, global.height);

  // Convert log-odds to ROS OccupancyGrid values: -1=unknown, 0-100=probability
  global.data.resize(global.width * global.height);
  for (size_t i = 0; i < global_log_odds.size(); ++i) {
    double lo = global_log_odds[i];
    if (std::abs(lo) < 1e-6) {
      global.data[i] = -1;  // unknown
    } else {
      double prob = logOddsToProbability(lo);
      global.data[i] = static_cast<int8_t>(std::clamp(
          static_cast<int>(prob * 100.0), 0, 100));
    }
  }

  return global;
}

OccupancyGridGenerator::GlobalGrid OccupancyGridGenerator::reassembleAfterPGO(
    const std::vector<LocalGrid>& local_grids,
    const std::vector<Eigen::Matrix4d>& corrected_poses) const {
  // Reassembly after PGO is identical to initial assembly but with corrected poses
  return assembleGlobalGrid(local_grids, corrected_poses);
}

void OccupancyGridGenerator::morphologicalClean(
    std::vector<double>& log_odds, int width, int height) const {
  if (width <= 2 || height <= 2) return;

  // Erosion: remove occupied cells with fewer than 2 occupied neighbors
  std::vector<bool> survive(width * height, false);
  for (int y = 1; y < height - 1; ++y) {
    for (int x = 1; x < width - 1; ++x) {
      const int idx = y * width + x;
      if (log_odds[idx] <= 0.0) continue;

      int neighbors = 0;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) continue;
          if (log_odds[(y + dy) * width + (x + dx)] > 0.0) {
            ++neighbors;
          }
        }
      }
      survive[idx] = (neighbors >= 2);
    }
  }

  // Zero out non-surviving occupied cells
  for (int i = 0; i < width * height; ++i) {
    if (log_odds[i] > 0.0 && !survive[i]) {
      log_odds[i] = 0.0;
    }
  }
}

}  // namespace tof_slam
