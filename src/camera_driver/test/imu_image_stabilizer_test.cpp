#include <cmath>
#include <cstdlib>
#include <iostream>

#include <opencv2/core.hpp>

#include "camera_driver/imu_image_stabilizer.hpp"

namespace
{

constexpr double kDegreesToRadians =
  3.141592653589793238462643383279502884 / 180.0;

void require(const bool condition, const char * message)
{
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

}  // namespace

int main()
{
  camera_driver::ImuImageStabilizerConfig config;
  config.warmup_samples = 4U;
  config.trajectory_smoothing_time_constant_sec = 0.25;
  camera_driver::ImuImageStabilizer stabilizer(config);

  for (int index = 0; index < 4; ++index) {
    stabilizer.update(
      cv::Vec3d(0.0, -9.81, 0.0),
      cv::Vec3d(0.0, 0.0, 0.0),
      0.0025 * index);
  }
  require(stabilizer.initialized(), "warmup did not initialize stabilizer");

  stabilizer.update(
    cv::Vec3d(0.0, -9.81, 0.0),
    cv::Vec3d(0.0, 0.0, 1.0),
    0.0125);
  const auto correction = stabilizer.correctionAt(0.0125);
  require(correction.has_value(), "correction lookup failed");
  require(
    std::abs(correction->roll_deg) > 0.1,
    "gyroscope rotation did not produce stabilization correction");

  const auto homography = camera_driver::makeImageStabilizationHomography(
    500.0, 500.0, 640.0, 360.0, *correction);
  require(
    std::isfinite(homography(0, 0)) &&
    std::isfinite(homography(2, 2)),
    "stabilization homography is not finite");

  camera_driver::ImuImageStabilizer pitch_stabilizer(config);
  for (int index = 0; index < 4; ++index) {
    pitch_stabilizer.update(
      cv::Vec3d(0.0, -9.81, 0.0),
      cv::Vec3d(0.0, 0.0, 0.0),
      0.0025 * index);
  }
  pitch_stabilizer.update(
    cv::Vec3d(0.0, -9.81, 0.0),
    cv::Vec3d(-1.0, 0.0, 0.0),
    0.0125);
  const auto pitch_correction = pitch_stabilizer.correctionAt(0.0125);
  require(pitch_correction.has_value(), "pitch correction lookup failed");
  require(
    pitch_correction->pitch_deg > 0.1,
    "negative camera-X gyro must produce positive downward pitch");

  const auto pitch_homography =
    camera_driver::makeImageStabilizationHomography(
    500.0, 500.0, 640.0, 360.0, *pitch_correction);
  const cv::Vec3d current_horizon_ray(
    0.0,
    -std::sin(pitch_correction->pitch_deg * kDegreesToRadians),
    std::cos(pitch_correction->pitch_deg * kDegreesToRadians));
  const cv::Vec3d current_horizon_pixel(
    500.0 * current_horizon_ray[0] + 640.0 * current_horizon_ray[2],
    500.0 * current_horizon_ray[1] + 360.0 * current_horizon_ray[2],
    current_horizon_ray[2]);
  const cv::Vec3d stabilized_horizon_pixel =
    pitch_homography * current_horizon_pixel;
  require(
    std::abs(
      stabilized_horizon_pixel[1] / stabilized_horizon_pixel[2] - 360.0) <
    1.0e-6,
    "positive downward pitch must warp the horizon back to the principal point");

  std::cout << "imu_image_stabilizer_test passed\n";
  return 0;
}
