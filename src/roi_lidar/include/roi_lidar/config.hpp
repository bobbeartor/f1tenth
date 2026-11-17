#pragma once
#include "roi_lidar/clusters.hpp"
#include "roi_lidar/view.hpp"
#include "roi_lidar/terrain.hpp"
#include <point_cloud/depth_source.hpp>
namespace roi_lidar
{
struct Config
{
  point_cloud::Config camera;
  Options scan;
  ClusterOptions cluster;
  ViewOptions view;
  TerrainOptions terrain;
  bool show_terrain{true}, show_projected_points{true};
  double terrain_alpha{0.80};
  int point_radius{2};
  bool publish_lane_bev{false},publish_imu{false};
  bool show_bev_window{true};
  bool gui{true}, publish_preview{true}, camera_image{true};
  double preview_fps{15}, rgb_fps{15}, sync_sec{0.08}, max_age{0.25};
  int preview_size{800};
  bool show_raw_points{false};
  std::string interpolation{"linear"};
  Config() {camera.fps=60; camera.subpixel=true; camera.dot_intensity=0.7; cluster.enabled=false; scan.use_startup_plane=true; scan.ransac.enabled=false;}
};
inline void validate(const Config & c)
{
  point_cloud::validate(c.camera); validate(c.scan); validateView(c.view); validateClusters(c.cluster);
  validateTerrain(c.terrain,c.view);
  if (!std::isfinite(c.terrain_alpha) || c.terrain_alpha<0 || c.terrain_alpha>1 ||
    c.point_radius<1 || c.point_radius>4) {
    throw std::invalid_argument("preview.terrain_alpha=0..1, preview.point_radius=1..4");
  }
  if (c.cluster.enabled && c.cluster.min_height_m>c.scan.max_height) {
    throw std::invalid_argument("cluster.min_height_m must not exceed height.max_m");
  }
  if (c.interpolation!="nearest" && c.interpolation!="linear" && c.interpolation!="cubic" && c.interpolation!="lanczos4") {
    throw std::invalid_argument("preview.interpolation: nearest / linear / cubic / lanczos4");
  }
  if (!std::isfinite(c.preview_fps) || c.preview_fps<1 || c.preview_fps>60 ||
    !std::isfinite(c.rgb_fps) || c.rgb_fps<1 || c.rgb_fps>120 ||
    !std::isfinite(c.sync_sec) || c.sync_sec<0.005 || c.sync_sec>0.25 ||
    !std::isfinite(c.max_age) || c.max_age<0.05 || c.max_age>2 ||
    c.preview_size<240 || c.preview_size>1600) {
    throw std::invalid_argument("preview FPS=1..60, rgb FPS=1..120, sync=0.005..0.25s, age=0.05..2s, size=240..1600");
  }
}
}
// Shared declaration/snapshot/validation table; startup and mount are read-only.
#define ROI_PARAMETERS(X) \
  X("preview.show_bev_window",show_bev_window,as_bool) \
  X("motion.publish_imu",publish_imu,as_bool) \
  X("motion.imu_rate_hz",camera.imu_rate_hz,as_int) \
  X("motion.imu_queue_size",camera.imu_queue_size,as_int) \
  X("motion.imu_max_batch_reports",camera.imu_max_batch_reports,as_int) \
  X("preview.rgb_width_px",camera.rgb_width,as_int) \
  X("preview.rgb_height_px",camera.rgb_height,as_int) \
  X("preview.publish_lane_bev",publish_lane_bev,as_bool) \
  X("terrain.fill_enabled",terrain.fill_enabled,as_bool) \
  X("terrain.fill_max_cells",terrain.fill_max_cells,as_int) \
  X("terrain.fill_clearance_cells",terrain.fill_clearance_cells,as_int) \
  X("terrain.cell_m",terrain.cell_m,as_double) \
  X("terrain.below_ground_m",terrain.below_ground_m,as_double) \
  X("terrain.min_ground_samples",terrain.min_ground_samples,as_int) \
  X("terrain.min_obstacle_samples",terrain.min_obstacle_samples,as_int) \
  X("terrain.vehicle_width_m",terrain.vehicle_width_m,as_double) \
  X("preview.show_terrain",show_terrain,as_bool) \
  X("preview.show_projected_points",show_projected_points,as_bool) \
  X("preview.point_radius",point_radius,as_int) \
  X("preview.terrain_alpha",terrain_alpha,as_double) \
  X("camera.resolution",camera.resolution,as_string) \
  X("camera.fps",camera.fps,as_double) \
  X("depth.mode",camera.mode,as_string) \
  X("depth.ir_dot_projector_intensity",camera.dot_intensity,as_double) \
  X("depth.ir_flood_light_intensity",camera.flood_intensity,as_double) \
  X("depth.confidence_threshold",camera.confidence,as_int) \
  X("depth.left_right_check",camera.lr_check,as_bool) \
  X("depth.left_right_threshold",camera.lr_threshold,as_int) \
  X("depth.subpixel",camera.subpixel,as_bool) \
  X("depth.subpixel_fractional_bits",camera.subpixel_bits,as_int) \
  X("depth.extended_disparity",camera.extended,as_bool) \
  X("depth.median_filter",camera.median,as_string) \
  X("roi.x",scan.roi_x,as_double) \
  X("roi.y",scan.roi_y,as_double) \
  X("roi.width",scan.roi_width,as_double) \
  X("roi.height",scan.roi_height,as_double) \
  X("points.pixel_stride",scan.pixel_stride,as_int) \
  X("points.min_depth_m",scan.min_depth,as_double) \
  X("points.max_depth_m",scan.max_depth,as_double) \
  X("obstacle.min_height_m",scan.obstacle_min_height,as_double) \
  X("ground.use_startup_plane",scan.use_startup_plane,as_bool) \
  X("ground.ransac_enabled",scan.ransac.enabled,as_bool) \
  X("ground.ransac_distance_m",scan.ransac.distance_m,as_double) \
  X("ground.ransac_max_range_m",scan.ransac.max_range_m,as_double) \
  X("ground.ransac_max_tilt_deg",scan.ransac.max_tilt_deg,as_double) \
  X("ground.ransac_max_offset_m",scan.ransac.max_offset_m,as_double) \
  X("ground.ransac_min_inlier_ratio",scan.ransac.min_inlier_ratio,as_double) \
  X("ground.ransac_max_iterations",scan.ransac.max_iterations,as_int) \
  X("ground.ransac_max_samples",scan.ransac.max_samples,as_int) \
  X("outlier.enabled",scan.outlier.enabled,as_bool) \
  X("outlier.radius_m",scan.outlier.radius_m,as_double) \
  X("outlier.min_neighbors",scan.outlier.min_neighbors,as_int) \
  X("ground.enabled",scan.ground_enabled,as_bool) \
  X("ground.distance_m",scan.ground_distance,as_double) \
  X("ground.distance_per_meter",scan.ground_distance_per_meter,as_double) \
  X("ground.max_distance_m",scan.ground_max_distance,as_double) \
  X("height.max_m",scan.max_height,as_double) \
  X("range.min_m",scan.min_range,as_double) \
  X("range.max_m",scan.max_range,as_double) \
  X("scan.angle_min_deg",scan.angle_min,as_double) \
  X("scan.angle_max_deg",scan.angle_max,as_double) \
  X("scan.bins",scan.bins,as_int) \
  X("scan.min_samples",scan.min_samples,as_int) \
  X("scan.support_distance_m",scan.support_distance,as_double) \
  X("cluster.enabled",cluster.enabled,as_bool) \
  X("cluster.tolerance_m",cluster.tolerance_m,as_double) \
  X("cluster.min_bins",cluster.min_bins,as_int) \
  X("cluster.max_gap_bins",cluster.max_gap_bins,as_int) \
  X("cluster.min_height_m",cluster.min_height_m,as_double) \
  X("preview.show_raw_points",show_raw_points,as_bool) \
  X("preview.interpolation",interpolation,as_string) \
  X("preview.gui",gui,as_bool) \
  X("preview.publish",publish_preview,as_bool) \
  X("preview.camera_image",camera_image,as_bool) \
  X("preview.fps",preview_fps,as_double) \
  X("preview.rgb_fps",rgb_fps,as_double) \
  X("preview.max_sync_sec",sync_sec,as_double) \
  X("preview.width_m",view.width_m,as_double) \
  X("preview.forward_m",view.forward_m,as_double) \
  X("preview.size_px",preview_size,as_int) \
  X("input.max_age_sec",max_age,as_double)
