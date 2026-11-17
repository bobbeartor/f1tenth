#include "depth_lidar/depth_lidar_preview.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace depth_lidar
{

cv::Mat makeStereoPreview(const cv::Mat & left, const cv::Mat & right,
  const RoiRect & depth_roi, const int depth_width, const int depth_height)
{
  if (left.empty() || right.empty() || left.type() != CV_8UC1 || right.type() != CV_8UC1
      || left.size() != right.size() || depth_width <= 0 || depth_height <= 0
      || depth_roi.x < 0 || depth_roi.y < 0 || depth_roi.width <= 0 || depth_roi.height <= 0
      || depth_roi.x + depth_roi.width > depth_width
      || depth_roi.y + depth_roi.height > depth_height)
  {
    throw std::invalid_argument("stereo preview requires matching grayscale frames and a valid depth ROI");
  }
  constexpr int header = 54;
  constexpr int footer = 28;
  cv::Mat image(left.rows + header + footer, left.cols * 2, CV_8UC3, cv::Scalar(255, 255, 255));
  const double sx = static_cast<double>(left.cols) / depth_width;
  const double sy = static_cast<double>(left.rows) / depth_height;
  const cv::Point roi_start(static_cast<int>(std::floor(depth_roi.x * sx)),
    static_cast<int>(std::floor(depth_roi.y * sy)));
  const cv::Point roi_end(static_cast<int>(std::ceil((depth_roi.x + depth_roi.width) * sx)) - 1,
    static_cast<int>(std::ceil((depth_roi.y + depth_roi.height) * sy)) - 1);
  for (int side = 0; side < 2; ++side) {
    cv::Mat panel = image(cv::Rect(side * left.cols, header, left.cols, left.rows));
    cv::cvtColor(side == 0 ? left : right, panel, cv::COLOR_GRAY2BGR);
    cv::rectangle(panel, roi_start, roi_end, cv::Scalar(0, 220, 0), 2, cv::LINE_8);
    const cv::Point label(side * left.cols + 8, 20);
    cv::putText(image, side == 0 ? "LEFT / ROI GUIDE" : "RIGHT / DEPTH ROI",
      label, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(45, 45, 45), 1, cv::LINE_AA);
    std::ostringstream bounds;
    bounds << "Depth ROI x=" << depth_roi.x << " y=" << depth_roi.y
           << " w=" << depth_roi.width << " h=" << depth_roi.height;
    cv::putText(image, bounds.str(), label + cv::Point(0, 22),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(45, 45, 45), 1, cv::LINE_AA);
  }
  cv::putText(image, "C: camera ON/OFF | Green: sampling ROI (height filter applied separately) | Right: depth reference",
    cv::Point(8, image.rows - 9), cv::FONT_HERSHEY_SIMPLEX, 0.4,
    cv::Scalar(45, 45, 45), 1, cv::LINE_AA);
  return image;
}

cv::Mat makeRadarPreview(const ScanResult & scan, const GridResult & detection, const ProjectionConfig & config,
  const int width_px, const bool ready, const std::string & mode)
{
  if (mode != "points" && mode != "clusters" && mode != "both") {
    throw std::invalid_argument("radar preview mode must be points, clusters or both");
  }
  const bool show_points = mode != "clusters";
  const bool show_clusters = mode != "points";
  const double max_range_m = config.max_range_m;
  if (width_px < 240 || !std::isfinite(max_range_m) || max_range_m <= 0.0) {
    throw std::invalid_argument("radar preview requires width >= 240 and positive finite range");
  }
  constexpr double pi = 3.14159265358979323846;
  const int radius_px = width_px / 2 - 38;
  const cv::Point origin(width_px / 2, radius_px + 96);
  cv::Mat image(origin.y + 68, width_px, CV_8UC3, cv::Scalar(255, 255, 255));
  const double pixels_per_meter = radius_px / max_range_m;
  const cv::Scalar grid(210, 210, 210);
  const cv::Scalar ink(65, 65, 65);
  const auto pointAt = [&](double range_m, double angle) {
    return cv::Point(origin.x - static_cast<int>(std::lround(
                       range_m * std::sin(angle) * pixels_per_meter)),
      origin.y - static_cast<int>(std::lround(range_m * std::cos(angle) * pixels_per_meter)));
  };
  const auto labelAt = [&](const std::string & label, cv::Point point, double scale) {
    int baseline = 0;
    const auto size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, scale, 1, &baseline);
    point.x = std::clamp(point.x, 2, std::max(2, width_px - size.width - 2));
    cv::putText(image, label, point, cv::FONT_HERSHEY_SIMPLEX, scale, ink, 1, cv::LINE_AA);
  };

  for (int ring = 1; ring <= 5; ++ring) {
    const double range_m = max_range_m * ring / 5.0;
    const int radius = static_cast<int>(std::lround(range_m * pixels_per_meter));
    cv::ellipse(image, origin, cv::Size(radius, radius), 0.0, 180.0, 360.0,
      grid, 1, cv::LINE_AA);
    std::ostringstream label;
    label << std::fixed << std::setprecision(max_range_m < 1.0 ? 2 : 1) << range_m << "m";
    if (width_px >= 600 || ring == 5 || (width_px >= 400 && ring % 2 == 0)) {
      labelAt(label.str(), cv::Point(origin.x + radius + 3, origin.y - 4), 0.32);
    }
  }
  for (int degrees = -90; degrees <= 90; degrees += 30) {
    const double angle = degrees * pi / 180.0;
    cv::line(image, origin, pointAt(max_range_m, angle), grid, 1, cv::LINE_AA);
    if (std::abs(degrees) != 90) {
      const cv::Point end = pointAt(max_range_m, angle);
      const std::string label = degrees == 0 ? "0" :
        (degrees > 0 ? "+" : "") + std::to_string(degrees);
      labelAt(label, end + cv::Point(-8, -7), 0.35);
    }
  }
  const auto gridPoint = [&](double x, double y) {
    return cv::Point(origin.x-static_cast<int>(std::lround(y*pixels_per_meter)),
      origin.y-static_cast<int>(std::lround(x*pixels_per_meter)));
  };
  if (show_clusters) {
    for (const auto & edge : detection.boundary) {
      cv::line(image, gridPoint(edge.x1,edge.y1), gridPoint(edge.x2,edge.y2),
        edge.age_sec > 0.0 ? cv::Scalar(150,150,150) : cv::Scalar(0,150,230),2,cv::LINE_AA);
    }
  }
  std::size_t drawn_points = 0;
  // Match buildOccupancyGrid's bin geometry, but retain points rejected by grid
  // bounds, per-cell support or cluster-size thresholds for diagnosis.
  if (show_points && config.bins > 1) {
    const auto count = std::min({scan.ranges.size(), scan.ages.size(),
      static_cast<std::size_t>(config.bins)});
    const double angle_step = (config.angle_max_deg-config.angle_min_deg)/(config.bins-1);
    for (std::size_t i = 0; i < count; ++i) {
      const double range = scan.ranges[i], age = scan.ages[i];
      if (!std::isfinite(range) || range < config.min_range_m || range > max_range_m
          || !std::isfinite(age) || age < 0.0) { continue; }
      const auto point = pointAt(range, (config.angle_min_deg+i*angle_step)*pi/180.0);
      cv::circle(image, point, 3, age > 0.0 ? cv::Scalar(110,110,110) : cv::Scalar(210,100,0),
        age > 0.0 ? 1 : cv::FILLED, cv::LINE_AA);
      ++drawn_points;
    }
  }
  cv::drawMarker(image, origin, ink, cv::MARKER_CROSS, 10, 2, cv::LINE_AA);
  labelAt(width_px < 400 ? "0m" : "VEHICLE / 0m",
    origin + cv::Point(width_px < 400 ? -8 : -43, 17), 0.35);
  labelAt(width_px < 400 ? "L (+)" : "LEFT (+)", cv::Point(8, origin.y + 17), 0.35);
  labelAt(width_px < 400 ? "R (-)" : "RIGHT (-)",
    cv::Point(width_px - (width_px < 400 ? 40 : 75), origin.y + 17), 0.35);
  const std::string title = mode == "both" ? "POINTS + CLUSTERS" :
    (show_points ? "SCAN POINTS" : "GRID CLUSTERS");
  labelAt(title + " / VEHICLE FRAME", cv::Point(10, 20), width_px < 400 ? 0.36 : 0.5);
  labelAt("RETURNS " + std::to_string(scan.valid_bins) + " | CELLS " + std::to_string(detection.occupied_cells) + " | CLUSTERS " + std::to_string(detection.clusters.size()),
    cv::Point(10, 38), 0.35);
  if (!ready || ((!show_points || drawn_points == 0) &&
      (!show_clusters || detection.clusters.empty()))) {
    const std::string empty_label = show_points ? "NO CONFIRMED POINTS" : "NO CONFIRMED CLUSTERS";
    labelAt(ready ? empty_label : "SCAN NOT READY", cv::Point(origin.x - 60, origin.y - radius_px / 2), 0.4);
  }
  return image;
}

} // namespace depth_lidar
