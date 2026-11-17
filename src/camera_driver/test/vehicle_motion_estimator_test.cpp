#include <cmath>
#include <cstdlib>
#include <iostream>

#include "camera_driver/vehicle_motion_estimator.hpp"

namespace
{

void require(const bool condition, const char * message)
{
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

camera_driver::VehicleMotionEstimatorConfig fastConfig()
{
  camera_driver::VehicleMotionEstimatorConfig config;
  config.speed_filter_time_constant_sec = 0.0001;
  config.acceleration_filter_time_constant_sec = 0.0001;
  config.maximum_longitudinal_acceleration_mps2 = 1000.0;
  return config;
}

}  // namespace

int main()
{
  const auto config = fastConfig();
  camera_driver::VehicleMotionEstimator estimator(config);

  const double expected_ratio =
    (54.0 / 13.0) * (37.0 / 13.0);
  require(
    std::abs(estimator.totalGearRatio() - expected_ratio) < 1.0e-12,
    "Traxxas total gear ratio is incorrect");
  const double expected_scale =
    3.141592653589793238462643383279502884 * 0.1095 /
    (60.0 * 2.0 * expected_ratio);
  require(
    std::abs(estimator.metersPerSecondPerErpm() - expected_scale) < 1.0e-12,
    "ERPM-to-m/s scale is incorrect");

  const auto stopped = estimator.update(0, 1.0000);
  require(stopped.speed_mps == 0.0, "zero ERPM did not produce zero speed");
  const auto moving = estimator.update(20620, 1.0125);
  require(
    moving.speed_mps > 4.9 && moving.speed_mps < 5.1,
    "positive ERPM did not produce the expected forward speed");
  require(
    moving.longitudinal_acceleration_mps2 > 0.0,
    "increasing speed did not produce positive acceleration");

  const auto interpolated = estimator.estimateAt(1.00625);
  require(interpolated.has_value(), "bracketed motion lookup failed");
  require(
    interpolated->speed_mps > 2.4 && interpolated->speed_mps < 2.6,
    "motion history interpolation is incorrect");
  require(
    estimator.estimateAt(1.0500).has_value(),
    "fresh motion prediction was rejected");
  require(
    !estimator.estimateAt(1.2000).has_value(),
    "stale motion prediction was accepted");

  const auto reversing = estimator.update(-20620, 2.0000);
  require(
    reversing.speed_mps < -4.9 && reversing.speed_mps > -5.1,
    "negative ERPM did not produce reverse speed");

  require(
    std::abs(camera_driver::lateralAccelerationMps2(4.0, 0.5, 15.0) - 2.0) <
    1.0e-12,
    "v times yaw-rate lateral acceleration is incorrect");
  require(
    std::abs(camera_driver::lateralAccelerationMps2(-4.0, 0.5, 15.0) + 2.0) <
    1.0e-12,
    "reverse lateral acceleration sign is incorrect");
  require(
    camera_driver::lateralAccelerationMps2(6.0, 4.0, 15.0) == 15.0,
    "lateral acceleration safety clamp failed");

  std::cout << "vehicle_motion_estimator_test passed\n";
  return 0;
}
