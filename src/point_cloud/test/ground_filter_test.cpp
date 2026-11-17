#include "point_cloud/ground_filter.hpp"

#include <iostream>
#include <string>

namespace
{
void check(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}
point_cloud::Cloud scene(double tilt, bool wall = false)
{
  point_cloud::Cloud cloud;
  cloud.width = 160; cloud.height = 100;
  for (unsigned v = 0; v < cloud.height; ++v) {
    for (unsigned u = 0; u < cloud.width; ++u) {
      const double x = (static_cast<double>(u) - 80) / 130;
      const double y = (static_cast<double>(v) - 35) / 130;
      const double denominator = std::cos(tilt) * y + std::sin(tilt);
      double z = wall ? 0.8 : 0.20 / denominator;
      if (z <= 0 || z > 5) {z = std::numeric_limits<double>::quiet_NaN();}
      // A 10cm-high object patch in front of the floor.
      if (!wall && u >= 65 && u < 95 && v >= 55 && v < 80) {z = 0.10 / denominator;}
      // Millimeter-scale depth noise.
      if (!wall) {z += 0.001 * (static_cast<int>((u * 17 + v * 7) % 5) - 2);}
      cloud.xyz.push_back(x * z); cloud.xyz.push_back(y * z); cloud.xyz.push_back(z);
      if (std::isfinite(z)) {++cloud.valid_points;}
    }
  }
  return cloud;
}
void equalCloud(const point_cloud::Cloud & a, const point_cloud::Cloud & b)
{
  check(a.width == b.width && a.height == b.height && a.valid_points == b.valid_points,
    "Cloud layout/count changed");
  check(a.xyz.size() == b.xyz.size(), "Cloud size changed");
  for (std::size_t i = 0; i < a.xyz.size(); ++i) {
    check(a.xyz[i] == b.xyz[i] || (std::isnan(a.xyz[i]) && std::isnan(b.xyz[i])), "Cloud data changed");
  }
}
}

int main()
{
  using namespace point_cloud;
  for (double tilt : {0.0, 0.26, 0.52}) {
    const auto original = scene(tilt);
    auto filtered = original;
    const auto result = removeGround(filtered, {});
    check(result.detected && result.removed > 6000, "Noisy tilted floor was not removed");
    check(filtered.width == original.width && filtered.height == original.height,
      "Organized layout lost");
    check(filtered.valid_points + result.removed == original.valid_points, "Invalid valid count");
    for (unsigned v = 55; v < 80; ++v) {
      for (unsigned u = 65; u < 95; ++u) {
        const auto j = (v * filtered.width + u) * 3;
        check(filtered.xyz[j + 2] == original.xyz[j + 2], "Above-floor obstacle removed");
      }
    }
    for (std::size_t j = 0; j < filtered.xyz.size(); j += 3) {
      if (std::isnan(filtered.xyz[j])) {
        check(std::isnan(filtered.xyz[j + 1]) && std::isnan(filtered.xyz[j + 2]), "Partial NaN");
      }
      if (original.xyz[j + 2] > 3.0f) {
        check(filtered.xyz[j + 2] == original.xyz[j + 2], "Filter extrapolated past range");
      }
    }
    auto disabled = original;
    GroundOptions options; options.enabled = false;
    check(!removeGround(disabled, options).detected, "Disabled filter ran");
    equalCloud(disabled, original);
    options.enabled = true;
    check(removeGround(disabled, options).removed == result.removed, "Re-enabling changed detection");
  }
  const auto wall = scene(0, true);
  auto filtered_wall = wall;
  check(!removeGround(filtered_wall, {}).detected, "Wall mistaken for floor");
  equalCloud(filtered_wall, wall);
  Cloud empty;
  check(!removeGround(empty, {}).detected, "Empty input detected");
  auto sparse = scene(0.26);
  std::fill(sparse.xyz.begin(), sparse.xyz.end(), std::numeric_limits<float>::quiet_NaN());
  sparse.valid_points = 0;
  check(!removeGround(sparse, {}).detected, "All-NaN input detected");
  auto degenerate = scene(0.26);
  for (std::size_t j = 0; j < degenerate.xyz.size(); j += 3) {
    degenerate.xyz[j] = 0; degenerate.xyz[j + 1] = 0.2; degenerate.xyz[j + 2] = 1;
  }
  const auto same_points = degenerate;
  check(!removeGround(degenerate, {}).detected, "Degenerate plane accepted");
  equalCloud(degenerate, same_points);
  auto wrong_height = scene(0.26);
  GroundOptions height_options; height_options.min_height_m = 0.3;
  check(!removeGround(wrong_height, height_options).detected, "Wrong camera height accepted");
  for (int field = 0; field < 6; ++field) {
    GroundOptions bad;
    switch (field) {
      case 0: bad.distance_m = 0; break;
      case 1: bad.max_depth_m = -1; break;
      case 2: bad.min_height_m = bad.max_height_m; break;
      case 3: bad.max_height_m = std::numeric_limits<double>::quiet_NaN(); break;
      case 4: bad.max_tilt_deg = 90; break;
      case 5: bad.min_inlier_ratio = 0; break;
    }
    bool rejected = false;
    try {removeGround(empty, bad);} catch (const std::invalid_argument &) {rejected = true;}
    check(rejected, "Invalid ground options accepted");
  }
  std::cout << "Ground tests passed: tilted/noisy floor, obstacles, range, toggle, walls, NaNs, degeneracy, validation\n";
}
