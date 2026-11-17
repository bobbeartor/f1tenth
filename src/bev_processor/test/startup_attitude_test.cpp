#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include "bev_processor/oak_startup_measurement.hpp"
#include "bev_processor/startup_attitude.hpp"

namespace
{

void require(const bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

bool near(const double first, const double second)
{
  return std::abs(first - second) < 1.0e-9;
}

}  // namespace

int main()
{
  const bev_processor::OakStartupMeasurementConfig default_config;
  require(
    default_config.attitude_source ==
    bev_processor::StartupAttitudeSource::kDepth,
    "startup attitude did not default to the depth plane");

  const auto raw_imu_up = bev_processor::attitudeUpVector(4.0, 18.0);
  const auto depth_up = bev_processor::attitudeUpVector(1.0, 12.0);

  const auto depth_result = bev_processor::selectStartupAttitude(
    raw_imu_up,
    depth_up,
    bev_processor::parseStartupAttitudeSource("depth"),
    2.0,
    5.0);
  require(near(depth_result.roll_deg, 1.0), "depth roll was not selected");
  require(
    near(depth_result.pitch_down_deg, 12.0),
    "depth pitch was not selected");

  const auto imu_result = bev_processor::selectStartupAttitude(
    raw_imu_up,
    depth_up,
    bev_processor::parseStartupAttitudeSource("imu"),
    2.0,
    5.0);
  require(near(imu_result.roll_deg, 2.0), "corrected IMU roll was not selected");
  require(
    near(imu_result.pitch_down_deg, 13.0),
    "corrected IMU pitch was not selected");
  require(
    imu_result.imu_depth_difference_deg > 0.0,
    "IMU/depth difference was not reported");

  bool invalid_source_rejected = false;
  try {
    (void)bev_processor::parseStartupAttitudeSource("fusion");
  } catch (const std::invalid_argument &) {
    invalid_source_rejected = true;
  }
  require(invalid_source_rejected, "invalid attitude source was accepted");

  std::cout << "startup_attitude_test passed\n";
  return 0;
}
