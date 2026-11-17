#pragma once

#include "point_cloud/depth_projection.hpp"

#include <algorithm>
#include <array>
#include <random>

namespace point_cloud
{
struct GroundOptions
{
  bool enabled{true};
  double distance_m{0.03};
  double max_depth_m{3.0};
  double min_height_m{0.08};
  double max_height_m{0.50};
  double max_tilt_deg{45.0};
  double min_inlier_ratio{0.35};
};

inline void validateGround(const GroundOptions & o)
{
  if (!std::isfinite(o.distance_m) || o.distance_m <= 0 || o.distance_m > 0.10 ||
    !std::isfinite(o.max_depth_m) || o.max_depth_m <= 0 ||
    !std::isfinite(o.min_height_m) || !std::isfinite(o.max_height_m) ||
    o.min_height_m <= 0 || o.max_height_m <= o.min_height_m ||
    !std::isfinite(o.max_tilt_deg) || o.max_tilt_deg <= 0 || o.max_tilt_deg > 60 ||
    !std::isfinite(o.min_inlier_ratio) || o.min_inlier_ratio < 0.1 || o.min_inlier_ratio > 1)
  {
    throw std::invalid_argument("ground: distance_m must be (0,0.10], max_depth_m>0, "
      "0<min_height_m<max_height_m, max_tilt_deg in (0,60], min_inlier_ratio in [0.1,1]");
  }
}

struct GroundResult
{
  bool detected{false};
  std::size_t removed{0};
};

// Per-frame RANSAC in rectified optical coordinates. A floor normal points
// roughly along +Y (down), with a positive camera-to-plane distance. Restrict
// fitting to the lower image and nearby points; never select arbitrary walls.
// This is a visualization filter, not a calibrated vehicle-frame transform.
inline GroundResult removeGround(Cloud & cloud, const GroundOptions & o)
{
  validateGround(o);
  if (!o.enabled || cloud.width == 0 || cloud.height == 0) {return {};}
  using Point = std::array<double, 3>;
  std::vector<Point> samples;
  constexpr std::size_t max_samples = 2000;
  std::mt19937 rng(42);
  std::size_t seen = 0;
  // Reservoir sampling avoids stride-dependent aliasing on organized clouds.
  for (std::size_t i = static_cast<std::size_t>(cloud.height / 2) * cloud.width;
    i < static_cast<std::size_t>(cloud.width) * cloud.height; ++i)
  {
    const auto j = i * 3;
    const Point p{cloud.xyz[j], cloud.xyz[j + 1], cloud.xyz[j + 2]};
    if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2]) ||
      p[2] <= 0 || p[2] > o.max_depth_m) {continue;}
    ++seen;
    if (samples.size() < max_samples) {samples.push_back(p);}
    else {
      const auto slot = std::uniform_int_distribution<std::size_t>(0, seen - 1)(rng);
      if (slot < max_samples) {samples[slot] = p;}
    }
  }
  if (samples.size() < 30) {return {};}
  const double min_normal_y = std::cos(o.max_tilt_deg * 3.14159265358979323846 / 180.0);
  std::array<double, 4> best{};
  std::size_t best_count = 0;
  double best_error = std::numeric_limits<double>::infinity();
  auto distance = [](const std::array<double, 4> & plane, const Point & p) {
      return plane[0] * p[0] + plane[1] * p[1] + plane[2] * p[2] - plane[3];
    };
  std::uniform_int_distribution<std::size_t> pick(0, samples.size() - 1);
  for (int iteration = 0; iteration < 160; ++iteration) {
    const auto & a = samples[pick(rng)];
    const auto & b = samples[pick(rng)];
    const auto & c = samples[pick(rng)];
    Point u{}, v{};
    for (int k = 0; k < 3; ++k) {u[k] = b[k] - a[k]; v[k] = c[k] - a[k];}
    std::array<double, 4> plane{
      u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
      u[0] * v[1] - u[1] * v[0], 0};
    double length = std::sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
    if (length < 1e-8) {continue;}
    if (plane[1] < 0) {length = -length;}
    for (int k = 0; k < 3; ++k) {plane[k] /= length; plane[3] += plane[k] * a[k];}
    if (plane[1] < min_normal_y || plane[3] < o.min_height_m || plane[3] > o.max_height_m) {continue;}
    std::size_t count = 0;
    double error = 0;
    double min_x = std::numeric_limits<double>::infinity(), max_x = -min_x;
    double min_z = min_x, max_z = -min_x;
    for (const auto & p : samples) {
      const double d = std::abs(distance(plane, p));
      if (d <= o.distance_m) {
        ++count; error += d;
        min_x = std::min(min_x, p[0]); max_x = std::max(max_x, p[0]);
        min_z = std::min(min_z, p[2]); max_z = std::max(max_z, p[2]);
      }
    }
    // Require a surface with spatial support, not an edge or tiny patch.
    if (max_x - min_x < 0.20 || max_z - min_z < 0.10) {continue;}
    if (count > best_count || (count == best_count && error < best_error)) {
      best = plane; best_count = count; best_error = error;
    }
  }
  if (best_count < 30 || best_count < o.min_inlier_ratio * samples.size()) {return {};}
  GroundResult result{true, 0};
  for (std::size_t j = 0; j < cloud.xyz.size(); j += 3) {
    const Point p{cloud.xyz[j], cloud.xyz[j + 1], cloud.xyz[j + 2]};
    // Do not extrapolate a locally fitted floor beyond the fitting range.
    if (p[2] > 0 && p[2] <= o.max_depth_m && std::abs(distance(best, p)) <= o.distance_m) {
      for (int k = 0; k < 3; ++k) {cloud.xyz[j + k] = std::numeric_limits<float>::quiet_NaN();}
      ++result.removed;
    }
  }
  cloud.valid_points -= result.removed;
  return result;
}
}  // namespace point_cloud
