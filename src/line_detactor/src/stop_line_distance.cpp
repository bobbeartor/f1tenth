#include "line_detactor/stop_line_distance.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace line_detactor
{
namespace
{

double cross_2d(const cv::Point2d & a, const cv::Point2d & b)
{
  return a.x * b.y - a.y * b.x;
}

double sorted_quantile(const std::vector<double> & sorted, const double quantile)
{
  if (sorted.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const auto index = static_cast<std::size_t>(std::clamp(
    quantile * static_cast<double>(sorted.size() - 1U), 0.0,
    static_cast<double>(sorted.size() - 1U)));
  return sorted[index];
}

}  // namespace

void validate_stop_line_distance(const StopLineDistanceConfig & c)
{
  const auto quantile = [](double v) {return std::isfinite(v) && v >= 0.0 && v < 0.5;};
  if (c.min_pixels < 2 || c.max_fit_samples < 2 ||
    !quantile(c.support_trim_quantile) || !quantile(c.near_edge_quantile) ||
    !std::isfinite(c.support_margin_px) || c.support_margin_px < 0.0 ||
    !std::isfinite(c.min_crossing_alignment) || c.min_crossing_alignment <= 0.0 ||
    c.min_crossing_alignment > 1.0 ||
    !std::isfinite(c.fit_distance_tolerance_px) || c.fit_distance_tolerance_px <= 0.0 ||
    !std::isfinite(c.fit_angle_tolerance_rad) || c.fit_angle_tolerance_rad <= 0.0)
  {throw std::invalid_argument("Invalid stop-line distance parameters");}
}

double estimate_stop_line_distance_m(
  const cv::Mat & stop_mask,
  const std::vector<cv::Point2f> & centerline,
  const int padding_left,
  const double bev_width_m,
  const double bev_height_m,
  const StopLineDistanceConfig & config)
{
  if (!config.enabled || stop_mask.empty() || stop_mask.type() != CV_8UC1 || centerline.size() < 2U ||
    bev_width_m <= 0.0 || bev_height_m <= 0.0)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  std::vector<cv::Point> pixels;
  cv::findNonZero(stop_mask, pixels);
  if (pixels.size() < static_cast<std::size_t>(config.min_pixels)) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  // Welsch fitting limits the influence of isolated segmentation pixels while
  // retaining an incomplete stop-line segment. Cap fit samples because the
  // 120x300 mask contains redundant adjacent stripe pixels.
  const auto maximum_fit_samples = static_cast<std::size_t>(config.max_fit_samples);
  std::vector<cv::Point> fit_pixels;
  const std::vector<cv::Point> * fit_input = &pixels;
  if (pixels.size() > maximum_fit_samples) {
    fit_pixels.reserve(maximum_fit_samples);
    for (std::size_t index = 0; index < maximum_fit_samples; ++index) {
      fit_pixels.push_back(pixels[
        index * (pixels.size() - 1U) / (maximum_fit_samples - 1U)]);
    }
    fit_input = &fit_pixels;
  }
  cv::Vec4f fitted;
  cv::fitLine(*fit_input, fitted, cv::DIST_WELSCH, 0.0,
    config.fit_distance_tolerance_px, config.fit_angle_tolerance_rad);
  const cv::Point2d line_direction(fitted[0], fitted[1]);
  const cv::Point2d line_point(fitted[2], fitted[3]);
  const cv::Point2d line_normal(-line_direction.y, line_direction.x);

  std::vector<double> along_line;
  std::vector<double> across_line;
  along_line.reserve(pixels.size());
  across_line.reserve(pixels.size());
  for (const auto & pixel : pixels) {
    const cv::Point2d delta(
      static_cast<double>(pixel.x) - line_point.x,
      static_cast<double>(pixel.y) - line_point.y);
    along_line.push_back(delta.dot(line_direction));
    across_line.push_back(delta.dot(line_normal));
  }
  std::sort(along_line.begin(), along_line.end());
  std::sort(across_line.begin(), across_line.end());
  const double supported_min =
    sorted_quantile(along_line, config.support_trim_quantile) - config.support_margin_px;
  const double supported_max =
    sorted_quantile(along_line, 1.0 - config.support_trim_quantile) + config.support_margin_px;

  const double scale_x = bev_width_m / static_cast<double>(stop_mask.cols);
  const double scale_y = bev_height_m / static_cast<double>(stop_mask.rows);
  const double mean_scale = 0.5 * (scale_x + scale_y);
  const auto metric_point = [&](const cv::Point2d & point) {
      return cv::Point2d(
        bev_height_m - (point.y + 0.5) * scale_y,
        0.5 * bev_width_m - (point.x + 0.5) * scale_x);
    };

  cv::Point2d previous(
    static_cast<double>(centerline.front().x) - padding_left,
    static_cast<double>(centerline.front().y));
  const cv::Point2d first_metric = metric_point(previous);
  double walked_m = std::hypot(first_metric.x, first_metric.y);

  for (std::size_t index = 1; index < centerline.size(); ++index) {
    const cv::Point2d current(
      static_cast<double>(centerline[index].x) - padding_left,
      static_cast<double>(centerline[index].y));
    const cv::Point2d segment = current - previous;
    const double denominator = cross_2d(segment, line_direction);
    const cv::Point2d previous_metric = metric_point(previous);
    const cv::Point2d current_metric = metric_point(current);
    const double segment_m = cv::norm(current_metric - previous_metric);
    if (std::abs(denominator) > 1.0e-6) {
      const double fraction = cross_2d(line_point - previous, line_direction) / denominator;
      if (fraction >= 0.0 && fraction <= 1.0) {
        const cv::Point2d intersection = previous + segment * fraction;
        const double support_position = (intersection - line_point).dot(line_direction);
        const double segment_pixels = cv::norm(segment);
        const cv::Point2d path_direction = segment_pixels > 1.0e-6 ?
          segment * (1.0 / segment_pixels) : cv::Point2d();
        const double forward_normal = path_direction.dot(line_normal);
        const double crossing_alignment = std::abs(forward_normal);
        if (support_position >= supported_min && support_position <= supported_max &&
          crossing_alignment >= config.min_crossing_alignment)
        {
          // Convert the fitted stripe center to its robust vehicle-near edge.
          const double near_edge_pixels = forward_normal > 0.0 ?
            std::max(0.0, -sorted_quantile(across_line, config.near_edge_quantile)) / crossing_alignment :
            std::max(0.0, sorted_quantile(across_line, 1.0 - config.near_edge_quantile)) / crossing_alignment;
          return std::max(
            0.0, walked_m + fraction * segment_m - near_edge_pixels * mean_scale);
        }
      }
    }
    walked_m += segment_m;
    previous = current;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

}  // namespace line_detactor
