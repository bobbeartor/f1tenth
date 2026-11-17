#pragma once
#include "line_detactor/centerline.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace line_detactor {
// Longest near-vehicle model-supported component arc, over either side.
// Never count connector bridges, centerline offsets or disconnected far islands.
inline double observed_lane_length(const ObservedLanePaths & paths,
  int width, int height, int padding, double width_m, double height_m,
  double max_start_m)
{
  const auto invalid = std::numeric_limits<double>::quiet_NaN();
  if (width <= 0 || height <= 0 || padding < 0 || !std::isfinite(width_m) ||
      !std::isfinite(height_m) || !std::isfinite(max_start_m) ||
      width_m <= 0 || height_m <= 0 || max_start_m <= 0) return invalid;
  double longest = 0;
  for (const auto & side : paths) for (const auto & path : side) {
    if (path.size() < 2) continue;
    double length = 0, nearest = std::numeric_limits<double>::infinity();
    cv::Point2d previous;
    for (std::size_t i = 0; i < path.size(); ++i) {
      const auto & p = path[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
          p.x < padding || p.x >= padding + width || p.y < 0 || p.y >= height)
        return invalid;
      cv::Point2d metric(height_m - (p.y + .5) * height_m / height,
        width_m / 2 - (p.x - padding + .5) * width_m / width);
      nearest = std::min(nearest, std::hypot(metric.x, metric.y));
      if (i) length += cv::norm(metric - previous);
      previous = metric;
    }
    if (nearest <= max_start_m) longest = std::max(longest, length);
  }
  return longest;
}
}  // namespace line_detactor
