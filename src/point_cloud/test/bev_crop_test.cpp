#include "point_cloud/bev_crop.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace
{
void check(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}
void near(double actual, double expected)
{
  check(std::abs(actual - expected) < 1e-6, "Unexpected coordinate");
}
template<typename F> void rejects(F action)
{
  bool rejected = false;
  try {action();} catch (const std::exception &) {rejected = true;}
  check(rejected, "Invalid input accepted");
}
}
int main()
{
  using namespace point_cloud;
  // Zero-angle camera: forward Z -> vehicle X, right X -> vehicle -Y,
  // down Y -> vehicle -Z. Ground is 20cm below the camera.
  Cloud cloud{4, 1, 4, {0.1f,0.05f,1.0f, 0,0.2f,4, 0.8f,0,1, 0,0.2f,0.1f}};
  const auto original = cloud;
  transformAndCrop(cloud, vehicleFromRgb(-0.16,0,0.2,0,0,0), {});
  check(cloud.valid_points == 1 && cloud.width == 4 && cloud.height == 1, "Crop/count/layout incorrect");
  near(cloud.xyz[0], 0.84); near(cloud.xyz[1], -0.1); near(cloud.xyz[2], 0.15);
  for (std::size_t j = 3; j < cloud.xyz.size(); ++j) {check(std::isnan(cloud.xyz[j]), "Excluded point not NaN");}
  BevOptions off; off.enabled = false;
  cloud = original;
  transformAndCrop(cloud, vehicleFromRgb(-0.16,0,0.2,0,0,0), off);
  check(cloud.valid_points == 4, "Disabled crop discarded points");
  near(cloud.xyz[5], 0);  // floor stays at zero, obstacle keeps height

  Cloud boundary{6, 1, 6, {0,-0.5f,1, 3,0,2, 1,0.5f,3, 2.99f,0.49f,4, -0.01f,0,0, 1,-0.51f,0}};
  BevOptions box; box.y_min_m = -0.5; box.y_max_m = 0.5;
  transformAndCrop(boundary, {}, box);
  check(boundary.valid_points == 2, "Boundary inclusion incorrect");
  near(boundary.xyz[2],1); near(boundary.xyz[11],4); // no Z flattening/cutoff

  // Compose a nonzero rectification rotation and a measured stereo lever arm.
  const std::vector<std::vector<double>> reference_from_frame{
    {0,-1,0,0.01}, {1,0,0,0.02}, {0,0,1,0.03}, {0,0,0,1}};
  const std::vector<std::vector<double>> rgb_from_reference{
    {1,0,0,4}, {0,1,0,-1}, {0,0,1,2}, {0,0,0,1}};
  const auto rectified = calibratedTransform(reference_from_frame, 1.0);
  const auto camera_offset = calibratedTransform(rgb_from_reference, 0.01);
  const auto chain = compose(vehicleFromRgb(-0.16,0,0.2,0,0,0), compose(camera_offset, rectified));
  Cloud one{1,1,1,{0.1f,0.2f,1}};
  transformAndCrop(one, chain, off);
  near(one.xyz[0],0.89); near(one.xyz[1],0.15); near(one.xyz[2],0.09);

  // Pitch changes forward distance: crop must happen after mount correction.
  Cloud pitched{1,1,1,{0,0,1}};
  box.x_max_m = 0.8;
  transformAndCrop(pitched, vehicleFromRgb(-0.16,0,0.2,0,30,0), box);
  check(pitched.valid_points == 1, "Cropped in optical rather than vehicle coordinates");
  near(pitched.xyz[0], std::cos(3.14159265358979323846/6)-0.16);
  near(pitched.xyz[2], -0.3);

  Cloud empty;
  transformAndCrop(empty, {}, {});
  Cloud invalid{1,1,0,{0,std::numeric_limits<float>::quiet_NaN(),1}};
  transformAndCrop(invalid, {}, {});
  check(invalid.valid_points == 0 && std::isnan(invalid.xyz[0]) && std::isnan(invalid.xyz[2]), "NaN leaked");
  box.x_min_m = box.x_max_m;
  rejects([&](){transformAndCrop(empty, {}, box);});
  rejects([](){vehicleFromRgb(0,0,0,0,0,0);});
  auto malformed = rgb_from_reference; malformed[1].pop_back();
  rejects([&](){calibratedTransform(malformed,0.01);});
  malformed = rgb_from_reference; malformed[0][0] = -1;
  rejects([&](){calibratedTransform(malformed,0.01);});
  malformed[0][0] = std::numeric_limits<double>::infinity();
  rejects([&](){calibratedTransform(malformed,0.01);});
  std::cout << "BEV tests passed: axes, crop boundaries, 3D height, toggle, transform order, calibration units, NaNs\n";
}
