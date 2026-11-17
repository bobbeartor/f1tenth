#ifndef LINE_DETACTOR__CENTERLINE_HPP_
#define LINE_DETACTOR__CENTERLINE_HPP_
#include <array>
#include <cstdint>
#include <vector>
#include <opencv2/core.hpp>
#include "line_detactor/processing_profile.hpp"
namespace line_detactor
{
using ObservedLanePaths =
  std::array<std::vector<std::vector<cv::Point2f>>, 2>;

struct CenterlineConfig
{
  bool enabled{true};
  double lane_width_m{0.65};
  double bev_width_m{1.2};
  double bev_height_m{3.0};
  double sample_spacing_m{0.015};
  double output_spacing_m{0.01};  // Final resampling, before smoothing/publication.
  double clearance_check_spacing_m{0.005};  // Maximum step for segment clearance checks.
  double min_fragment_length_m{0.08};
  double tangent_window_m{0.06};
  double width_tolerance_m{0.12};
  double pair_along_tolerance_m{0.055};
  double pair_heading_tolerance_deg{40.0};
  double max_gap_m{0.12};
  double max_start_distance_m{0.65};
  double min_clearance_m{0.16};
  double outside_margin_m{0.12};
  int max_samples{2000};
  int line_width_px{2};
  bool corner_outer_enabled{true};
  double corner_outer_weight{0.85};
  double corner_outward_offset_m{0.05};
  double corner_entry_distance_m{0.40};
  double corner_entry_full_offset_distance_m{0.0};  // No distance attenuation inside this range.
  double corner_outer_window_m{0.60};
  double corner_outer_tangent_window_m{0.15};
  double corner_outer_min_length_m{0.30};
  double corner_outer_min_turn_deg{8.0};
  double corner_outer_full_turn_deg{25.0};
  bool smoothing_enabled{true};
  double smoothing_sigma_m{0.04};
  double smoothing_window_m{0.65};
  double smoothing_max_shift_m{0.03};
  double smoothing_strength{1.0};
  double straight_turn_deg{12.0};
  double corner_turn_deg{35.0};
  double turn_window_m{0.30};
};
struct CenterlineResult
{
  // Ordered extended-image pixels (x right, y down). No TF/vehicle frame implied.
  std::vector<cv::Point2f> points;
  // 1=single-side offset, 2=paired midpoint, 3=short gap, 4=outer-reference blend/shift; before smoothing.
  std::vector<std::uint8_t> support;
  cv::Mat mask;
  bool sample_limit_reached{false};
};
void validate_centerline(const CenterlineConfig & config);
CenterlineResult generate_centerline(
  const cv::Mat & labels, const ObservedLanePaths & observed_paths,
  int source_width, int padding, const CenterlineConfig & config,
  bool render_mask = true, ProcessingProfile * profile = nullptr);
}  // namespace line_detactor
#endif
