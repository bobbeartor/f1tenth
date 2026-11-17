#include "point_cloud/depth_projection.hpp"

#include <iostream>
#include <string>

namespace
{
void check(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}
template<typename F> void rejects(F action)
{
  bool rejected = false;
  try {action();} catch (const std::invalid_argument &) {rejected = true;}
  check(rejected, "Expected invalid input to be rejected");
}
}  // namespace

int main()
{
  using namespace point_cloud;
  // 3x2 depth, including row padding, deliberately unaligned byte storage.
  std::vector<std::uint8_t> storage(17, 0xff);
  auto * data = storage.data() + 1;
  const std::uint16_t depths[6] = {1000, 2000, 0, 3000, 4000, 5000};
  for (int v = 0; v < 2; ++v) {
    for (int u = 0; u < 3; ++u) {std::memcpy(data + v * 8 + u * 2, &depths[v * 3 + u], 2);}
  }
  const Intrinsics k{2, 4, 1, 0};
  const auto cloud = projectDepth(data, 14, 3, 2, 8, k, {});
  check(cloud.width == 3 && cloud.height == 2 && cloud.valid_points == 5, "Pixel layout/count lost");
  check(cloud.xyz[0] == -0.5f && cloud.xyz[1] == 0 && cloud.xyz[2] == 1, "Wrong meters or optical X");
  check(cloud.xyz[3] == 0 && cloud.xyz[5] == 2, "Wrong principal point");
  check(std::isnan(cloud.xyz[6]) && std::isnan(cloud.xyz[7]) && std::isnan(cloud.xyz[8]), "Zero depth must be NaN");
  check(cloud.xyz[9] == -1.5f && cloud.xyz[10] == 0.75f && cloud.xyz[11] == 3, "Padding or optical Y incorrect");
  const auto filtered = projectDepth(data, 14, 3, 2, 8, k, {1, 2.0, 4.0});
  check(filtered.valid_points == 3 && std::isnan(filtered.xyz[0]) && std::isnan(filtered.xyz[15]), "Range endpoints incorrect");
  const auto sampled = projectDepth(data, 14, 3, 2, 8, k, {2, 0, 0});
  check(sampled.width == 2 && sampled.height == 1 && sampled.valid_points == 1, "Stride/odd dimensions incorrect");
  check(sampled.xyz[0] == -0.5f && std::isnan(sampled.xyz[3]), "Stride changed original pixel coordinates");
  const auto far_only = projectDepth(data, 14, 3, 2, 8, k, {1, 4.5, 0});
  check(far_only.valid_points == 1 && far_only.xyz[17] == 5, "max=0 must disable upper limit");
  // A second frame with another resolution/K must not reuse the previous rays.
  const std::uint16_t second[2] = {1000, 1000};
  const auto changed = projectDepth(reinterpret_cast<const std::uint8_t *>(second), 4, 2, 1, 4, {4, 4, 0, 0}, {});
  check(changed.width == 2 && changed.xyz[3] == 0.25f, "Resolution/intrinsics change not applied");
  rejects([&]() {projectDepth(data, 13, 3, 2, 8, k, {});});
  rejects([&]() {projectDepth(data, 14, 3, 2, 5, k, {});});
  rejects([&]() {projectDepth(data, 14, 3, 2, 8, {0, 4, 1, 0}, {});});
  rejects([&]() {projectDepth(data, 14, 3, 2, 8, k, {0, 0, 0});});
  rejects([&]() {projectDepth(data, 14, 3, 2, 8, k, {1, 4, 2});});
  rejects([&]() {projectDepth(nullptr, 14, 3, 2, 8, k, {});});
  std::cout << "Projection tests passed: units, axes, NaNs, padding, stride, ranges, intrinsics, malformed frames\n";
}
