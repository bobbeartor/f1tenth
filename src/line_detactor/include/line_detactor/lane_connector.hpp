#ifndef LINE_DETACTOR__LANE_CONNECTOR_HPP_
#define LINE_DETACTOR__LANE_CONNECTOR_HPP_

#include <array>
#include <cstdint>
#include <limits>
#include <vector>

#include <opencv2/core.hpp>
#include "line_detactor/centerline.hpp"

namespace line_detactor
{

struct LaneConnectionConfig
{
  bool enabled{true};
  bool ccl_serial_enabled{true};  // Serial 16-bit CCL for small masks; no global thread setting.
  int padding_px{30};  // Per side; does not change model input or BEV scale.
  int min_component_area_px{8};
  int skeleton_downsample_factor{1};  // 1 = original resolution; valid range 1..4.
  double min_fragment_length_px{8.0};
  int max_fragments{24};
  double tangent_window_px{8.0};
  double max_gap_px{80.0};
  double corridor_half_width_px{4.0};
  double direction_tolerance_deg{20.0};
  double max_turn_deg{180.0};
  double max_curvature_per_px{0.12};
  double max_arc_ratio{1.8};
  double border_endpoint_distance_px{6.0};
  int line_width_px{2};
};

struct LaneConnectionResult
{
  // Retained model-supported component skeletons, extracted once and reused
  // by centerline generation. Coordinates include result padding.
  ObservedLanePaths observed_paths;
  cv::Mat labels;  // mono8: 0=background, 1/2=left/right model, 3/4=left/right bridge.
  cv::Mat image;   // bgr8: lanes/bridges; the node adds green stop lines and yellow centerline afterward.
  CenterlineResult centerline;
  cv::Mat stop_line_mask;  // Independent mono8 0/255; original ROI only, no lane cleanup/bridges.
  bool stop_line_present{false};
  // Front-axle origin to the vehicle-near edge of the stop line, measured along
  // the generated center path. NaN means that no reliable path/line intersection exists.
  double stop_line_distance_m{std::numeric_limits<double>::quiet_NaN()};
  std::uint8_t state{0U};  // 0=NONE, 1=LEFT_ONLY, 2=RIGHT_ONLY, 3=BOTH.
};

void validate_lane_connection(const LaneConnectionConfig & config);
// render_image preserves the existing bridge policy. render_bgr independently
// suppresses only the display image, leaving labels and control geometry intact.
LaneConnectionResult connect_lane_fragments(
  const cv::Mat & labels, const LaneConnectionConfig & config,
  bool render_image = true, ProcessingProfile * profile = nullptr,
  bool render_bgr = true);

}  // namespace line_detactor

#endif
