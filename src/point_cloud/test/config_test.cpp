#include "point_cloud/depth_source.hpp"

#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>

int main()
{
  using namespace point_cloud;
  validate(Config{});
  if (Config{}.fps != 50.0 || Config{}.ground.distance_m != 0.03) {
    throw std::runtime_error("Requested FPS/ground defaults lost");
  }
  Config high_rate; high_rate.fps = 120; validate(high_rate);
  for (const auto & name : {"400p", "480p", "720p", "800p"}) {
    Config c;
    c.resolution = name;
    validate(c);
  }
  const auto rejects = [](const std::function<void(Config &)> & change) {
      Config c;
      change(c);
      try {validate(c);} catch (const std::invalid_argument &) {return;}
      throw std::runtime_error("Invalid settings unexpectedly accepted");
    };
  rejects([](Config & c) {c.resolution = "1080p";});
  rejects([](Config & c) {c.dot_intensity = 1.1;});
  rejects([](Config & c) {c.flood_intensity = -0.1;});
  rejects([](Config & c) {c.fps = std::numeric_limits<double>::quiet_NaN();});
  rejects([](Config & c) {c.fps = 121;});
  rejects([](Config & c) {c.confidence = 256;});
  rejects([](Config & c) {c.subpixel = true; c.extended = true;});
  rejects([](Config & c) {c.subpixel = true; c.subpixel_bits = 5; c.median = "3x3";});
  rejects([](Config & c) {c.projection.max_depth_m = c.projection.min_depth_m = 2;});
  rejects([](Config & c) {c.projection.pixel_stride = 0;});
  rejects([](Config & c) {c.max_age_sec = 0;});
  rejects([](Config & c) {c.mode = "unknown";});
  rejects([](Config & c) {c.median = "9x9";});
  std::cout << "Configuration tests passed\n";
}
