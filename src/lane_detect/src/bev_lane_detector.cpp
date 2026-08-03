#include "lane_mask/bev_lane_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace lane_mask
{
namespace
{

struct Run
{
  int first{0};
  int last{0};

  int width() const
  {
    return last - first + 1;
  }

  double center() const
  {
    return 0.5 * static_cast<double>(first + last);
  }
};

struct TrackSegment
{
  std::vector<cv::Point2d> left_points;
  std::vector<cv::Point2d> right_points;
  std::vector<cv::Point2d> center_points;
  std::vector<double> pair_widths;
  bool initialized{false};
  double tracked_left{0.0};
  double tracked_right{0.0};
  int last_observed_row{0};
  int missing_rows{0};

  int score() const
  {
    const int left_count = static_cast<int>(left_points.size());
    const int right_count = static_cast<int>(right_points.size());
    return left_count + right_count +
           2 * std::min(left_count, right_count);
  }
};

int makeOdd(const int value)
{
  const int clamped = std::max(1, value);
  return (clamped % 2 == 0) ? clamped + 1 : clamped;
}

std::vector<Run> findRuns(
  const cv::Mat & binary,
  const int row,
  const int minimum_width,
  const int maximum_width,
  cv::Mat * filtered)
{
  std::vector<Run> runs;
  const auto * pixels = binary.ptr<std::uint8_t>(row);
  int column = 0;
  while (column < binary.cols) {
    while (column < binary.cols && pixels[column] == 0U) {
      ++column;
    }
    if (column >= binary.cols) {
      break;
    }

    const int first = column;
    while (column < binary.cols && pixels[column] != 0U) {
      ++column;
    }
    const Run run{first, column - 1};
    if (run.width() < minimum_width || run.width() > maximum_width) {
      continue;
    }

    runs.push_back(run);
    if (filtered != nullptr) {
      filtered->row(row).colRange(run.first, run.last + 1).setTo(255);
    }
  }
  return runs;
}

double median(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  const auto middle = values.begin() +
    static_cast<std::ptrdiff_t>(values.size() / 2U);
  std::nth_element(values.begin(), middle, values.end());
  double value = *middle;
  if (values.size() % 2U == 0U) {
    const auto lower = std::max_element(values.begin(), middle);
    value = 0.5 * (value + *lower);
  }
  return value;
}

LanePolynomial fitPolynomial(
  const std::vector<cv::Point2d> & input_points,
  const int minimum_points,
  const double maximum_residual)
{
  LanePolynomial result;
  if (static_cast<int>(input_points.size()) < minimum_points) {
    return result;
  }

  std::vector<cv::Point2d> points = input_points;
  cv::Vec3d coefficients(0.0, 0.0, 0.0);

  for (int iteration = 0; iteration < 3; ++iteration) {
    if (static_cast<int>(points.size()) < minimum_points) {
      return result;
    }

    cv::Mat design(
      static_cast<int>(points.size()), 3, CV_64F);
    cv::Mat observations(
      static_cast<int>(points.size()), 1, CV_64F);
    for (std::size_t index = 0; index < points.size(); ++index) {
      const double row = points[index].y;
      design.at<double>(static_cast<int>(index), 0) = row * row;
      design.at<double>(static_cast<int>(index), 1) = row;
      design.at<double>(static_cast<int>(index), 2) = 1.0;
      observations.at<double>(static_cast<int>(index), 0) = points[index].x;
    }

    cv::Mat solved;
    if (!cv::solve(design, observations, solved, cv::DECOMP_SVD)) {
      return result;
    }
    coefficients = cv::Vec3d(
      solved.at<double>(0, 0),
      solved.at<double>(1, 0),
      solved.at<double>(2, 0));

    if (iteration == 2) {
      break;
    }

    std::vector<double> residuals;
    residuals.reserve(points.size());
    for (const auto & point : points) {
      const double predicted =
        coefficients[0] * point.y * point.y +
        coefficients[1] * point.y +
        coefficients[2];
      residuals.push_back(std::abs(point.x - predicted));
    }

    const double robust_scale = std::max(1.0, 1.4826 * median(residuals));
    const double cutoff = std::min(
      maximum_residual * 2.0,
      std::max(maximum_residual, 2.5 * robust_scale));
    std::vector<cv::Point2d> inliers;
    inliers.reserve(points.size());
    for (std::size_t index = 0; index < points.size(); ++index) {
      if (residuals[index] <= cutoff) {
        inliers.push_back(points[index]);
      }
    }
    if (inliers.size() == points.size()) {
      break;
    }
    points = std::move(inliers);
  }

  double squared_error = 0.0;
  int minimum_row = std::numeric_limits<int>::max();
  int maximum_row = std::numeric_limits<int>::min();
  for (const auto & point : points) {
    const double predicted =
      coefficients[0] * point.y * point.y +
      coefficients[1] * point.y +
      coefficients[2];
    const double error = point.x - predicted;
    squared_error += error * error;
    minimum_row = std::min(
      minimum_row, static_cast<int>(std::lround(point.y)));
    maximum_row = std::max(
      maximum_row, static_cast<int>(std::lround(point.y)));
  }
  const double rms = std::sqrt(
    squared_error / static_cast<double>(points.size()));
  if (!std::isfinite(rms) || rms > maximum_residual) {
    return result;
  }

  result.valid = true;
  result.coefficients = coefficients;
  result.point_count = static_cast<int>(points.size());
  result.minimum_row = minimum_row;
  result.maximum_row = maximum_row;
  result.rms_error_px = rms;
  return result;
}

void keepBetterSegment(TrackSegment segment, TrackSegment * best)
{
  if (
    segment.score() > best->score() ||
    (segment.score() == best->score() &&
    segment.pair_widths.size() > best->pair_widths.size()))
  {
    *best = std::move(segment);
  }
}

bool initializeTrack(
  const std::vector<Run> & runs,
  const int image_width,
  const BevLaneDetectorConfig & config,
  TrackSegment * segment)
{
  const double image_center =
    0.5 * static_cast<double>(image_width - 1);
  const double minimum_separation =
    config.expected_lane_width_px - config.lane_width_tolerance_px;
  const double maximum_separation =
    config.expected_lane_width_px + config.lane_width_tolerance_px;

  double best_score = std::numeric_limits<double>::infinity();
  bool found_pair = false;
  double best_left = 0.0;
  double best_right = 0.0;
  for (std::size_t left_index = 0; left_index < runs.size(); ++left_index) {
    for (
      std::size_t right_index = left_index + 1U;
      right_index < runs.size();
      ++right_index)
    {
      const double left = runs[left_index].center();
      const double right = runs[right_index].center();
      const double separation = right - left;
      if (
        separation < minimum_separation ||
        separation > maximum_separation)
      {
        continue;
      }
      const double pair_center = 0.5 * (left + right);
      const double center_error = std::abs(pair_center - image_center);
      if (center_error > config.initial_center_tolerance_px) {
        continue;
      }
      const double score =
        center_error +
        0.5 * std::abs(separation - config.expected_lane_width_px);
      if (score < best_score) {
        best_score = score;
        best_left = left;
        best_right = right;
        found_pair = true;
      }
    }
  }

  if (found_pair) {
    segment->initialized = true;
    segment->tracked_left = best_left;
    segment->tracked_right = best_right;
    return true;
  }

  if (!config.allow_single_lane) {
    return false;
  }

  const double expected_left =
    image_center - 0.5 * config.expected_lane_width_px;
  const double expected_right =
    image_center + 0.5 * config.expected_lane_width_px;
  double best_single_error = std::numeric_limits<double>::infinity();
  bool single_is_left = false;
  double best_single = 0.0;
  for (const auto & run : runs) {
    const double center = run.center();
    const double left_error = std::abs(center - expected_left);
    const double right_error = std::abs(center - expected_right);
    if (left_error < best_single_error) {
      best_single_error = left_error;
      best_single = center;
      single_is_left = true;
    }
    if (right_error < best_single_error) {
      best_single_error = right_error;
      best_single = center;
      single_is_left = false;
    }
  }
  if (best_single_error > config.single_lane_initial_tolerance_px) {
    return false;
  }

  segment->initialized = true;
  if (single_is_left) {
    segment->tracked_left = best_single;
    segment->tracked_right =
      best_single + config.expected_lane_width_px;
  } else {
    segment->tracked_right = best_single;
    segment->tracked_left =
      best_single - config.expected_lane_width_px;
  }
  return true;
}

bool updateWithPair(
  const std::vector<Run> & runs,
  const int row,
  const BevLaneDetectorConfig & config,
  TrackSegment * segment)
{
  const double minimum_separation =
    config.expected_lane_width_px - config.lane_width_tolerance_px;
  const double maximum_separation =
    config.expected_lane_width_px + config.lane_width_tolerance_px;
  const double row_factor = std::max(
    1.0,
    static_cast<double>(std::abs(segment->last_observed_row - row)) /
    static_cast<double>(std::max(1, config.row_step_px)));
  const double movement_limit =
    config.maximum_lateral_step_px * row_factor;

  double best_score = std::numeric_limits<double>::infinity();
  bool found = false;
  double best_left = 0.0;
  double best_right = 0.0;
  for (std::size_t left_index = 0; left_index < runs.size(); ++left_index) {
    for (
      std::size_t right_index = left_index + 1U;
      right_index < runs.size();
      ++right_index)
    {
      const double left = runs[left_index].center();
      const double right = runs[right_index].center();
      const double separation = right - left;
      if (
        separation < minimum_separation ||
        separation > maximum_separation ||
        std::abs(left - segment->tracked_left) > movement_limit ||
        std::abs(right - segment->tracked_right) > movement_limit)
      {
        continue;
      }
      const double score =
        std::abs(left - segment->tracked_left) +
        std::abs(right - segment->tracked_right) +
        0.5 * std::abs(separation - config.expected_lane_width_px);
      if (score < best_score) {
        best_score = score;
        best_left = left;
        best_right = right;
        found = true;
      }
    }
  }

  if (!found) {
    return false;
  }

  segment->left_points.emplace_back(best_left, row);
  segment->right_points.emplace_back(best_right, row);
  segment->center_points.emplace_back(0.5 * (best_left + best_right), row);
  segment->pair_widths.push_back(best_right - best_left);
  segment->tracked_left = best_left;
  segment->tracked_right = best_right;
  segment->last_observed_row = row;
  segment->missing_rows = 0;
  return true;
}

bool updateWithSingle(
  const std::vector<Run> & runs,
  const int row,
  const BevLaneDetectorConfig & config,
  TrackSegment * segment)
{
  if (!config.allow_single_lane || runs.empty()) {
    return false;
  }

  const double row_factor = std::max(
    1.0,
    static_cast<double>(std::abs(segment->last_observed_row - row)) /
    static_cast<double>(std::max(1, config.row_step_px)));
  const double movement_limit =
    config.maximum_lateral_step_px * row_factor;
  double best_error = std::numeric_limits<double>::infinity();
  bool found = false;
  bool is_left = false;
  double best_center = 0.0;
  for (const auto & run : runs) {
    const double center = run.center();
    const double left_error = std::abs(center - segment->tracked_left);
    const double right_error = std::abs(center - segment->tracked_right);
    if (left_error <= movement_limit && left_error < best_error) {
      best_error = left_error;
      best_center = center;
      is_left = true;
      found = true;
    }
    if (right_error <= movement_limit && right_error < best_error) {
      best_error = right_error;
      best_center = center;
      is_left = false;
      found = true;
    }
  }
  if (!found) {
    return false;
  }

  if (is_left) {
    const double shift = best_center - segment->tracked_left;
    segment->tracked_left = best_center;
    segment->tracked_right += shift;
    segment->left_points.emplace_back(best_center, row);
    segment->center_points.emplace_back(
      best_center + 0.5 * config.expected_lane_width_px, row);
  } else {
    const double shift = best_center - segment->tracked_right;
    segment->tracked_right = best_center;
    segment->tracked_left += shift;
    segment->right_points.emplace_back(best_center, row);
    segment->center_points.emplace_back(
      best_center - 0.5 * config.expected_lane_width_px, row);
  }
  segment->last_observed_row = row;
  segment->missing_rows = 0;
  return true;
}

}  // namespace

double LanePolynomial::columnAt(const double row) const
{
  return coefficients[0] * row * row +
         coefficients[1] * row +
         coefficients[2];
}

double LanePolynomial::derivativeAt(const double row) const
{
  return 2.0 * coefficients[0] * row + coefficients[1];
}

double BevLaneDetection::centerColumnAt(
  const double row,
  const double expected_lane_width_px) const
{
  if (center.valid) {
    return center.columnAt(row);
  }
  if (left.valid && right.valid) {
    return 0.5 * (left.columnAt(row) + right.columnAt(row));
  }
  if (left.valid) {
    return left.columnAt(row) + 0.5 * expected_lane_width_px;
  }
  if (right.valid) {
    return right.columnAt(row) - 0.5 * expected_lane_width_px;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

double BevLaneDetection::centerDerivativeAt(const double row) const
{
  if (center.valid) {
    return center.derivativeAt(row);
  }
  if (left.valid && right.valid) {
    return 0.5 * (left.derivativeAt(row) + right.derivativeAt(row));
  }
  if (left.valid) {
    return left.derivativeAt(row);
  }
  if (right.valid) {
    return right.derivativeAt(row);
  }
  return 0.0;
}

BevLaneDetector::BevLaneDetector(BevLaneDetectorConfig config)
: config_(std::move(config))
{
  if (
    config_.white_threshold < 0 ||
    config_.white_threshold > 255 ||
    config_.vertical_close_px <= 0 ||
    config_.minimum_run_width_px <= 0 ||
    config_.maximum_run_width_px < config_.minimum_run_width_px ||
    config_.row_step_px <= 0 ||
    config_.expected_lane_width_px <= 0.0 ||
    config_.lane_width_tolerance_px <= 0.0 ||
    config_.lane_width_tolerance_px >= config_.expected_lane_width_px ||
    config_.initial_center_tolerance_px <= 0.0 ||
    config_.single_lane_initial_tolerance_px <= 0.0 ||
    config_.maximum_lateral_step_px <= 0.0 ||
    config_.maximum_tracking_gap_rows < config_.row_step_px ||
    config_.minimum_points_per_lane < 3 ||
    config_.maximum_fit_residual_px <= 0.0)
  {
    throw std::invalid_argument("invalid BEV lane detector configuration");
  }
}

BevLaneDetection BevLaneDetector::detect(const cv::Mat & image) const
{
  if (image.empty()) {
    throw std::invalid_argument("BEV lane detector input is empty");
  }

  cv::Mat gray;
  if (image.type() == CV_8UC3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else if (image.type() == CV_8UC1) {
    gray = image;
  } else {
    throw std::invalid_argument(
            "BEV lane detector expects 8-bit mono or BGR input");
  }

  BevLaneDetection result;
  cv::threshold(
    gray,
    result.binary_mask,
    config_.white_threshold,
    255,
    cv::THRESH_BINARY);
  if (config_.vertical_close_px > 1) {
    const cv::Mat close_kernel = cv::getStructuringElement(
      cv::MORPH_RECT, {1, makeOdd(config_.vertical_close_px)});
    cv::morphologyEx(
      result.binary_mask,
      result.binary_mask,
      cv::MORPH_CLOSE,
      close_kernel);
  }

  result.run_filtered_mask =
    cv::Mat::zeros(result.binary_mask.size(), CV_8UC1);
  std::vector<std::vector<Run>> rows(
    static_cast<std::size_t>(image.rows));
  for (int row = 0; row < image.rows; ++row) {
    rows[static_cast<std::size_t>(row)] = findRuns(
      result.binary_mask,
      row,
      config_.minimum_run_width_px,
      config_.maximum_run_width_px,
      &result.run_filtered_mask);
  }

  TrackSegment current;
  TrackSegment best;
  for (
    int row = image.rows - 1;
    row >= 0;
    row -= config_.row_step_px)
  {
    const auto & runs = rows[static_cast<std::size_t>(row)];
    if (!current.initialized) {
      if (!initializeTrack(runs, image.cols, config_, &current)) {
        continue;
      }
      current.last_observed_row = row;
    }

    if (
      updateWithPair(runs, row, config_, &current) ||
      updateWithSingle(runs, row, config_, &current))
    {
      continue;
    }

    current.missing_rows += config_.row_step_px;
    if (current.missing_rows > config_.maximum_tracking_gap_rows) {
      keepBetterSegment(std::move(current), &best);
      current = TrackSegment{};
    }
  }
  keepBetterSegment(std::move(current), &best);

  result.left_points = best.left_points;
  result.right_points = best.right_points;
  result.center_points = best.center_points;
  result.left = fitPolynomial(
    best.left_points,
    config_.minimum_points_per_lane,
    config_.maximum_fit_residual_px);
  result.right = fitPolynomial(
    best.right_points,
    config_.minimum_points_per_lane,
    config_.maximum_fit_residual_px);
  result.center = fitPolynomial(
    best.center_points,
    config_.minimum_points_per_lane,
    config_.maximum_fit_residual_px);

  if (!best.pair_widths.empty()) {
    const double observed_width = median(best.pair_widths);
    const double minimum_width =
      config_.expected_lane_width_px - config_.lane_width_tolerance_px;
    const double maximum_width =
      config_.expected_lane_width_px + config_.lane_width_tolerance_px;
    if (
      observed_width < minimum_width ||
      observed_width > maximum_width)
    {
      const bool discard_left =
        result.left.point_count < result.right.point_count ||
        (
          result.left.point_count == result.right.point_count &&
          result.left.rms_error_px > result.right.rms_error_px);
      if (discard_left) {
        result.left = LanePolynomial{};
      } else {
        result.right = LanePolynomial{};
      }
    } else {
      result.measured_lane_width_px = observed_width;
    }
  }

  result.both_lanes_valid =
    result.center.valid &&
    result.left.valid &&
    result.right.valid &&
    static_cast<int>(best.pair_widths.size()) >=
    std::max(3, config_.minimum_points_per_lane / 3);
  result.center_valid = result.center.valid;
  if (result.center_valid) {
    result.minimum_center_row = result.center.minimum_row;
    result.maximum_center_row = result.center.maximum_row;
    if (result.measured_lane_width_px <= 0.0) {
      result.measured_lane_width_px =
        best.pair_widths.empty() ?
        config_.expected_lane_width_px :
        median(best.pair_widths);
    }
  }

  return result;
}

}  // namespace lane_mask
