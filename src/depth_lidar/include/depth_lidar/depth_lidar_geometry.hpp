#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>
#include <string>

namespace depth_lidar
{
struct RoiRect { int x{0}, y{0}, width{0}, height{0}; };
struct CameraGeometry {
  int width{0}, height{0};
  double fx{0}, fy{0}, cx{0}, cy{0};
};
struct ProjectionConfig {
  double roi_width_ratio{1.0}, roi_height_ratio{1.0}, roi_bottom_offset_ratio{0.0};
  int pixel_stride{2};
  double min_depth_m{0.10}, max_depth_m{4.0};
  double min_height_m{0.05}, max_height_m{0.40};
  double min_range_m{0.10}, max_range_m{3.0}, range_offset_m{0.0};
  double angle_min_deg{-70.0}, angle_max_deg{70.0};
  int bins{141}, min_points_per_bin{2};
  int min_neighbors{1};
  double neighbor_delta_m{0.08};
};
// Rigid transform: p_target = rotation * p_source + translation.
// cameraMount() maps CAM_A optical RDF into vehicle forward/left/up.
struct FixedTransform {
  std::array<double, 9> rotation{{0,0,1, -1,0,0, 0,-1,0}};
  std::array<double, 3> translation{{0,0,0}};
};
FixedTransform cameraMount(double roll_deg, double pitch_down_deg, double yaw_deg,
  double x_m, double y_m, double height_m);
FixedTransform compose(const FixedTransform & target_from_middle, const FixedTransform & middle_from_source);

struct ScanResult {
  std::vector<float> ranges; // NaN = unknown/unobserved, never inferred free space.
  std::vector<float> ages;   // Host observation age, excluding USB transport latency.
  std::size_t valid_bins{0}, accepted_points{0};
};
ScanResult emptyScan(const ProjectionConfig & config);
bool validateProjectionConfig(const ProjectionConfig & config, std::string & reason);
RoiRect computeRoi(int width, int height, double width_ratio, double height_ratio, double bottom_offset_ratio);

class ScanProjector {
public:
  void configure(const CameraGeometry & camera, const FixedTransform & transform, const ProjectionConfig & config);
  ScanResult project(const std::uint16_t * depth, std::size_t stride_elements) const;
  std::size_t rayCount() const { return rays_.size(); }
  RoiRect roi() const { return roi_; }
private:
  struct Ray { int u, v; double x, y; std::uint16_t min_depth, max_depth; };
  CameraGeometry camera_;
  FixedTransform transform_;
  ProjectionConfig config_;
  RoiRect roi_;
  std::vector<Ray> rays_;
  bool configured_{false};
};
} // namespace depth_lidar
