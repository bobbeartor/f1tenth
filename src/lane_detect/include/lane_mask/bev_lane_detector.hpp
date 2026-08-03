#ifndef LANE_MASK__BEV_LANE_DETECTOR_HPP_
#define LANE_MASK__BEV_LANE_DETECTOR_HPP_

#include <vector>

#include <opencv2/core.hpp>

namespace lane_mask
{

struct BevLaneDetectorConfig
{
  int white_threshold{160};
  int vertical_close_px{5};
  int minimum_run_width_px{1};
  int maximum_run_width_px{18};
  int row_step_px{2};
  double expected_lane_width_px{62.5};
  double lane_width_tolerance_px{12.5};
  double initial_center_tolerance_px{45.0};
  double single_lane_initial_tolerance_px{20.0};
  double maximum_lateral_step_px{8.0};
  int maximum_tracking_gap_rows{20};
  int minimum_points_per_lane{15};
  double maximum_fit_residual_px{6.0};
  bool allow_single_lane{true};
};

struct LanePolynomial
{
  bool valid{false};
  cv::Vec3d coefficients{0.0, 0.0, 0.0};
  int point_count{0};
  int minimum_row{0};
  int maximum_row{-1};
  double rms_error_px{0.0};

  double columnAt(double row) const;
  double derivativeAt(double row) const;
};

struct BevLaneDetection
{
  cv::Mat binary_mask;
  cv::Mat run_filtered_mask;
  std::vector<cv::Point2d> left_points;
  std::vector<cv::Point2d> right_points;
  std::vector<cv::Point2d> center_points;
  LanePolynomial left;
  LanePolynomial right;
  LanePolynomial center;
  bool center_valid{false};
  bool both_lanes_valid{false};
  double measured_lane_width_px{0.0};
  int minimum_center_row{0};
  int maximum_center_row{-1};

  double centerColumnAt(double row, double expected_lane_width_px) const;
  double centerDerivativeAt(double row) const;
};

class BevLaneDetector
{
public:
  explicit BevLaneDetector(BevLaneDetectorConfig config);

  BevLaneDetection detect(const cv::Mat & image) const;

private:
  BevLaneDetectorConfig config_;
};

}  // namespace lane_mask

#endif  // LANE_MASK__BEV_LANE_DETECTOR_HPP_
