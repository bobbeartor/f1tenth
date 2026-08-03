#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "lane_mask/bev_lane_detector.hpp"

namespace
{

void require(const bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

double expectedCenter(const int row)
{
  const double distance = 180.0 - static_cast<double>(row);
  return 100.0 + 0.0012 * distance * distance;
}

std::vector<cv::Point> makeLane(const double lateral_offset)
{
  std::vector<cv::Point> points;
  for (int row = 185; row >= 20; --row) {
    points.emplace_back(
      static_cast<int>(std::lround(expectedCenter(row) + lateral_offset)),
      row);
  }
  return points;
}

lane_mask::BevLaneDetector makeDetector()
{
  lane_mask::BevLaneDetectorConfig config;
  config.white_threshold = 160;
  config.vertical_close_px = 5;
  config.minimum_run_width_px = 1;
  config.maximum_run_width_px = 18;
  config.row_step_px = 2;
  config.expected_lane_width_px = 62.5;
  config.lane_width_tolerance_px = 20.0;
  config.initial_center_tolerance_px = 45.0;
  config.single_lane_initial_tolerance_px = 20.0;
  config.maximum_lateral_step_px = 8.0;
  config.maximum_tracking_gap_rows = 20;
  config.minimum_points_per_lane = 18;
  config.maximum_fit_residual_px = 5.0;
  config.allow_single_lane = true;
  return lane_mask::BevLaneDetector(config);
}

void testPairRejectsHorizontalEdge()
{
  cv::Mat image = cv::Mat::zeros(190, 200, CV_8UC3);
  const std::vector<std::vector<cv::Point>> left{makeLane(-31.25)};
  const std::vector<std::vector<cv::Point>> right{makeLane(31.25)};
  cv::polylines(image, left, false, cv::Scalar(255, 255, 255), 5);
  cv::polylines(image, right, false, cv::Scalar(255, 255, 255), 5);

  // Real-car residual: a bright mat boundary crosses almost the whole BEV.
  // Its per-row run is much wider than tape and must not enter either fit.
  cv::line(
    image,
    cv::Point(10, 100),
    cv::Point(190, 100),
    cv::Scalar(255, 255, 255),
    5);
  cv::rectangle(
    image,
    cv::Rect(8, 45, 35, 8),
    cv::Scalar(255, 255, 255),
    cv::FILLED);

  const auto detection = makeDetector().detect(image);
  require(detection.left.valid, "left curved lane must be fitted");
  require(detection.right.valid, "right curved lane must be fitted");
  require(detection.both_lanes_valid, "paired detection must report BOTH");
  require(
    std::abs(detection.measured_lane_width_px - 62.5) < 3.0,
    "fitted lane width must remain near 62.5 px");

  for (const int row : {170, 120, 60}) {
    require(
      std::abs(
        detection.centerColumnAt(row, 62.5) -
        expectedCenter(row)) < 3.0,
      "center polynomial must follow the synthetic curve");
  }
}

void testSingleLaneFallback()
{
  cv::Mat image = cv::Mat::zeros(190, 200, CV_8UC3);
  const std::vector<std::vector<cv::Point>> left{makeLane(-31.25)};
  cv::polylines(image, left, false, cv::Scalar(255, 255, 255), 5);

  const auto detection = makeDetector().detect(image);
  require(detection.left.valid, "single left lane must be fitted");
  require(!detection.right.valid, "missing right lane must stay invalid");
  require(detection.center_valid, "single lane must yield inferred center");
  require(!detection.both_lanes_valid, "single lane must not report BOTH");
  require(
    std::abs(
      detection.centerColumnAt(120, 62.5) -
      expectedCenter(120)) < 3.0,
    "single-lane center must use half expected lane width");
}

void testHorizontalOnlyIsRejected()
{
  cv::Mat image = cv::Mat::zeros(190, 200, CV_8UC3);
  cv::line(
    image,
    cv::Point(5, 100),
    cv::Point(194, 100),
    cv::Scalar(255, 255, 255),
    7);

  const auto detection = makeDetector().detect(image);
  require(!detection.center_valid, "horizontal mat edge must not make a path");
  require(
    cv::countNonZero(detection.run_filtered_mask) == 0,
    "over-wide horizontal runs must be removed");
}

}  // namespace

int main()
{
  testPairRejectsHorizontalEdge();
  testSingleLaneFallback();
  testHorizontalOnlyIsRejected();
  std::cout << "BEV lane detector tests passed\n";
  return EXIT_SUCCESS;
}
