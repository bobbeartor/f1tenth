#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

namespace point_cloud
{
struct Intrinsics
{
  double fx, fy, cx, cy;
};

struct ProjectionOptions
{
  int pixel_stride{1};
  double min_depth_m{0.0};
  double max_depth_m{0.0};  // Zero disables the upper limit.
};

struct Cloud
{
  std::uint32_t width{0}, height{0};
  std::size_t valid_points{0};
  std::vector<float> xyz;
};

// RAW16 depth is millimeters, with row padding allowed. Preserve image layout:
// invalid/filtered pixels become NaN, never spurious points at the camera origin.
inline Cloud projectDepth(
  const std::uint8_t * data, std::size_t size, int width, int height,
  std::size_t row_stride, const Intrinsics & k, const ProjectionOptions & options)
{
  if (!data || width <= 0 || height <= 0 || options.pixel_stride < 1 ||
    !std::isfinite(k.fx) || !std::isfinite(k.fy) ||
    !std::isfinite(k.cx) || !std::isfinite(k.cy) || k.fx <= 0 || k.fy <= 0 ||
    !std::isfinite(options.min_depth_m) || !std::isfinite(options.max_depth_m) ||
    options.min_depth_m < 0 || options.max_depth_m < 0 ||
    (options.max_depth_m > 0 && options.max_depth_m <= options.min_depth_m))
  {
    throw std::invalid_argument("Invalid depth geometry or projection options");
  }
  const auto packed = static_cast<std::size_t>(width) * sizeof(std::uint16_t);
  if (row_stride < packed || size < packed ||
    static_cast<std::size_t>(height - 1) > (size - packed) / row_stride)
  {
    throw std::invalid_argument("Truncated depth frame or invalid row stride");
  }
  Cloud cloud;
  cloud.width = (width - 1) / options.pixel_stride + 1;
  cloud.height = (height - 1) / options.pixel_stride + 1;
  cloud.xyz.resize(static_cast<std::size_t>(cloud.width) * cloud.height * 3,
    std::numeric_limits<float>::quiet_NaN());
  std::size_t index = 0;
  for (int v = 0; v < height; v += options.pixel_stride) {
    for (int u = 0; u < width; u += options.pixel_stride, index += 3) {
      std::uint16_t mm;
      std::memcpy(&mm, data + static_cast<std::size_t>(v) * row_stride + 2 * u, sizeof(mm));
      const double z = mm * 0.001;
      if (mm == 0 || z < options.min_depth_m ||
        (options.max_depth_m > 0 && z > options.max_depth_m)) {continue;}
      cloud.xyz[index] = static_cast<float>((u - k.cx) * z / k.fx);
      cloud.xyz[index + 1] = static_cast<float>((v - k.cy) * z / k.fy);
      cloud.xyz[index + 2] = static_cast<float>(z);
      ++cloud.valid_points;
    }
  }
  return cloud;
}
}  // namespace point_cloud
