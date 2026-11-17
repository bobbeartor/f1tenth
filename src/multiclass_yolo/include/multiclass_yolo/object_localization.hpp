#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

namespace multiclass_yolo {

// Vehicle frame on the ground plane: x forward, y left, metres.
struct ObjectPoint {
  double x, y;
};

// Top-view class size. A zero length marks a class with no footprint (traffic light).
struct Footprint {
  double length{0}, width{0};
};

struct LocalizationOptions {
  // Stereo depth error grows with range, so the range window widens with it.
  double window_base_m{.05};
  double window_per_m{.03};
  std::size_t min_points{6};
  double min_fraction{.10};
  // A low percentile instead of the minimum: mixed-depth edge pixels sit in front.
  double near_percentile{.10};
};

struct ObjectEstimate {
  bool valid{false};
  double range{0}, bearing{0};       // front face, from the camera's ground position
  double near_range{0};              // low percentile of the selected points
  double front_x{0}, front_y{0};     // front face in the vehicle frame
  double center_x{0}, center_y{0};   // footprint centre in the vehicle frame
  double cluster_min{0}, cluster_max{0};  // range interval accepted as the object
  std::size_t cluster_points{0};
  Footprint footprint;
  std::array<ObjectPoint, 4> corners{};  // footprint, aligned with the viewing ray
};

// Picks the nearest well-populated range peak among one detection's points and
// places the class footprint behind it along the viewing ray.
// `bearing` is the ray through the box centre; without it the points decide.
inline ObjectEstimate localize(
  const std::vector<ObjectPoint> & points, double origin_x, double origin_y,
  std::optional<double> bearing, const Footprint & footprint, const LocalizationOptions & options = {})
{
  ObjectEstimate out;
  out.footprint = footprint;
  const std::size_t total = points.size();
  if (total < options.min_points || options.min_points == 0) return out;

  std::vector<std::pair<double, std::size_t>> ranges;
  ranges.reserve(total);
  for (std::size_t i = 0; i < total; ++i) {
    const double range = std::hypot(points[i].x - origin_x, points[i].y - origin_y);
    if (std::isfinite(range)) ranges.emplace_back(range, i);
  }
  std::sort(ranges.begin(), ranges.end());
  const std::size_t count = ranges.size();
  const std::size_t needed = std::max(options.min_points,
    static_cast<std::size_t>(std::ceil(options.min_fraction * static_cast<double>(count))));

  // Nearest window holding enough points. A few stray near returns never qualify.
  std::size_t first = count, last = 0;
  double window = 0;
  for (std::size_t i = 0, j = 0; i < count; ++i) {
    window = options.window_base_m + options.window_per_m * ranges[i].first;
    if (j < i) j = i;
    while (j < count && ranges[j].first <= ranges[i].first + window) ++j;
    if (j - i >= needed) {
      first = i;
      last = j;
      break;
    }
  }
  if (first == count) return out;

  // The object cannot be deeper than its class length, so later points are background.
  const double depth = std::max(0.0, footprint.length) + window;
  std::size_t end = last;
  while (end < count && ranges[end].first <= ranges[first].first + depth) ++end;

  out.range = ranges[first + (last - first) / 2].first;
  out.near_range = ranges[first + static_cast<std::size_t>(
    std::clamp(options.near_percentile, 0.0, 1.0) * static_cast<double>(end - first - 1))].first;
  out.cluster_min = ranges[first].first;
  out.cluster_max = ranges[end - 1].first;
  out.cluster_points = end - first;

  if (bearing && std::isfinite(*bearing)) {
    out.bearing = *bearing;
  } else {
    std::vector<double> angles;
    angles.reserve(last - first);
    for (std::size_t i = first; i < last; ++i) {
      const auto & p = points[ranges[i].second];
      angles.push_back(std::atan2(p.y - origin_y, p.x - origin_x));
    }
    std::nth_element(angles.begin(), angles.begin() + angles.size() / 2, angles.end());
    out.bearing = angles[angles.size() / 2];
  }

  const double ux = std::cos(out.bearing), uy = std::sin(out.bearing);
  out.front_x = origin_x + out.range * ux;
  out.front_y = origin_y + out.range * uy;
  const double half_length = footprint.length / 2, half_width = footprint.width / 2;
  out.center_x = out.front_x + half_length * ux;
  out.center_y = out.front_y + half_length * uy;
  // Near-left, near-right, far-right, far-left as seen from the camera.
  const double signs[4][2] = {{-1, 1}, {-1, -1}, {1, -1}, {1, 1}};
  for (int i = 0; i < 4; ++i) {
    const double along = signs[i][0] * half_length, across = signs[i][1] * half_width;
    out.corners[i] = {out.center_x + along * ux - across * uy, out.center_y + along * uy + across * ux};
  }
  out.valid = true;
  return out;
}

}  // namespace multiclass_yolo
