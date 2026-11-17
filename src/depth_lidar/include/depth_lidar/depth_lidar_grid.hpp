#pragma once
#include "depth_lidar/depth_lidar_geometry.hpp"
#include <limits>

namespace depth_lidar {
struct GridConfig {
  double resolution_m{0.05};
  double x_min_m{0.0}, x_max_m{3.0}, y_min_m{-3.0}, y_max_m{3.0};
  int min_returns_per_cell{1};
};
struct ClusterConfig {
  int min_cells{2}, min_returns{2};
  int connectivity{8}; // 4: shared sides only; 8: shared sides or corners.
};
struct OccupiedCell {
  std::uint32_t returns{0};
  double sum_x{0}, sum_y{0};
  double nearest_range_m{std::numeric_limits<double>::infinity()};
  double age_sec{0}; // Newest supporting return in this cell.
};
struct GridCluster {
  std::vector<std::size_t> cells;
  std::size_t returns{0};
  double center_x{0}, center_y{0}; // Mean of supporting scan returns, not object volume center.
  double min_x{0}, max_x{0}, min_y{0}, max_y{0}; // Cell envelope; not used for collision inflation.
  double nearest_range_m{std::numeric_limits<double>::infinity()};
  double age_sec{0}; // Maximum age among occupied cells.
};
struct BoundaryEdge {
  double x1, y1, x2, y2, age_sec;
  std::size_t cluster_id; // Frame-local, no object tracking.
};
struct GridResult {
  GridConfig config;
  int width{0}, height{0}; // row-major: left row * forward width + forward column.
  std::vector<OccupiedCell> cells;
  std::vector<GridCluster> clusters;
  std::vector<BoundaryEdge> boundary;
  std::size_t occupied_cells{0};
};
bool validateGridConfig(const GridConfig &, const ClusterConfig &, std::string & reason);
// Confirmed angular returns only. Missing cells remain UNKNOWN; no free-space inference.
GridResult clusterScan(const ScanResult &, const ProjectionConfig &, const GridConfig &, const ClusterConfig &);
} // namespace depth_lidar
