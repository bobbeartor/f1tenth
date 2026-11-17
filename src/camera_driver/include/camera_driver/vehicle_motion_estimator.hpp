#ifndef CAMERA_DRIVER__VEHICLE_MOTION_ESTIMATOR_HPP_
#define CAMERA_DRIVER__VEHICLE_MOTION_ESTIMATOR_HPP_

#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>

namespace camera_driver
{

struct VehicleMotionEstimatorConfig
{
  double wheel_diameter_m{0.1095};
  int motor_pole_pairs{2};
  int motor_pinion_teeth{13};
  int spur_gear_teeth{54};
  int differential_pinion_teeth{13};
  int differential_ring_teeth{37};
  double erpm_direction_sign{1.0};
  double speed_scale_correction{1.0};
  double speed_deadband_mps{0.03};
  double speed_filter_time_constant_sec{0.05};
  double acceleration_filter_time_constant_sec{0.12};
  double maximum_speed_mps{6.0};
  double maximum_longitudinal_acceleration_mps2{15.0};
  double maximum_sample_interval_sec{0.10};
  double maximum_sample_age_sec{0.10};
  double history_duration_sec{2.0};
};

struct VehicleMotionSample
{
  double timestamp_sec{0.0};
  double raw_speed_mps{0.0};
  double speed_mps{0.0};
  double longitudinal_acceleration_mps2{0.0};
};

double lateralAccelerationMps2(
  double speed_mps,
  double yaw_rate_radps,
  double maximum_absolute_acceleration_mps2);

class VehicleMotionEstimator
{
public:
  explicit VehicleMotionEstimator(
    const VehicleMotionEstimatorConfig & config = {});

  VehicleMotionSample update(std::int32_t measured_erpm, double timestamp_sec);
  std::optional<VehicleMotionSample> estimateAt(double timestamp_sec) const;

  double totalGearRatio() const;
  double metersPerSecondPerErpm() const;

private:
  double speedFromErpm(std::int32_t measured_erpm) const;

  VehicleMotionEstimatorConfig config_;
  double total_gear_ratio_{1.0};
  double meters_per_second_per_erpm_{0.0};

  mutable std::mutex mutex_;
  std::deque<VehicleMotionSample> history_;
};

}  // namespace camera_driver

#endif  // CAMERA_DRIVER__VEHICLE_MOTION_ESTIMATOR_HPP_
