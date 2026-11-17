#include "camera_driver/vehicle_motion_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>

namespace camera_driver
{

namespace
{

constexpr double kPi = 3.141592653589793238462643383279502884;

bool positiveFinite(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

double filterGain(const double dt_sec, const double time_constant_sec)
{
  return -std::expm1(-dt_sec / time_constant_sec);
}

VehicleMotionSample interpolateSample(
  const VehicleMotionSample & first,
  const VehicleMotionSample & second,
  const double timestamp_sec)
{
  const double interval_sec = second.timestamp_sec - first.timestamp_sec;
  if (interval_sec <= 0.0) {
    return second;
  }
  const double amount = std::clamp(
    (timestamp_sec - first.timestamp_sec) / interval_sec, 0.0, 1.0);
  const auto interpolate = [amount](const double a, const double b) {
      return a + amount * (b - a);
    };
  return VehicleMotionSample{
    timestamp_sec,
    interpolate(first.raw_speed_mps, second.raw_speed_mps),
    interpolate(first.speed_mps, second.speed_mps),
    interpolate(
      first.longitudinal_acceleration_mps2,
      second.longitudinal_acceleration_mps2)};
}

}  // namespace

double lateralAccelerationMps2(
  const double speed_mps,
  const double yaw_rate_radps,
  const double maximum_absolute_acceleration_mps2)
{
  if (
    !std::isfinite(speed_mps) ||
    !std::isfinite(yaw_rate_radps) ||
    !positiveFinite(maximum_absolute_acceleration_mps2))
  {
    throw std::invalid_argument("invalid lateral acceleration input");
  }
  return std::clamp(
    speed_mps * yaw_rate_radps,
    -maximum_absolute_acceleration_mps2,
    maximum_absolute_acceleration_mps2);
}

VehicleMotionEstimator::VehicleMotionEstimator(
  const VehicleMotionEstimatorConfig & config)
: config_(config)
{
  if (
    !positiveFinite(config_.wheel_diameter_m) ||
    config_.motor_pole_pairs <= 0 ||
    config_.motor_pinion_teeth <= 0 ||
    config_.spur_gear_teeth <= 0 ||
    config_.differential_pinion_teeth <= 0 ||
    config_.differential_ring_teeth <= 0 ||
    !std::isfinite(config_.erpm_direction_sign) ||
    std::abs(config_.erpm_direction_sign) < 1.0e-9 ||
    !positiveFinite(config_.speed_scale_correction) ||
    !std::isfinite(config_.speed_deadband_mps) ||
    config_.speed_deadband_mps < 0.0 ||
    !positiveFinite(config_.speed_filter_time_constant_sec) ||
    !positiveFinite(config_.acceleration_filter_time_constant_sec) ||
    !positiveFinite(config_.maximum_speed_mps) ||
    !positiveFinite(config_.maximum_longitudinal_acceleration_mps2) ||
    !positiveFinite(config_.maximum_sample_interval_sec) ||
    !positiveFinite(config_.maximum_sample_age_sec) ||
    !positiveFinite(config_.history_duration_sec))
  {
    throw std::invalid_argument("invalid vehicle motion estimator configuration");
  }

  config_.erpm_direction_sign = config_.erpm_direction_sign > 0.0 ? 1.0 : -1.0;
  total_gear_ratio_ =
    static_cast<double>(config_.spur_gear_teeth) /
    static_cast<double>(config_.motor_pinion_teeth) *
    static_cast<double>(config_.differential_ring_teeth) /
    static_cast<double>(config_.differential_pinion_teeth);
  meters_per_second_per_erpm_ =
    config_.erpm_direction_sign * config_.speed_scale_correction *
    (kPi * config_.wheel_diameter_m) /
    (60.0 * static_cast<double>(config_.motor_pole_pairs) *
    total_gear_ratio_);
}

VehicleMotionSample VehicleMotionEstimator::update(
  const std::int32_t measured_erpm,
  const double timestamp_sec)
{
  if (!std::isfinite(timestamp_sec)) {
    throw std::invalid_argument("vehicle motion timestamp must be finite");
  }

  const double raw_speed_mps = speedFromErpm(measured_erpm);
  std::lock_guard<std::mutex> lock(mutex_);
  if (history_.empty()) {
    const VehicleMotionSample first{
      timestamp_sec, raw_speed_mps, raw_speed_mps, 0.0};
    history_.push_back(first);
    return first;
  }

  const VehicleMotionSample & previous = history_.back();
  const double dt_sec = timestamp_sec - previous.timestamp_sec;
  if (!std::isfinite(dt_sec) || dt_sec <= 0.0) {
    return previous;
  }
  if (dt_sec > config_.maximum_sample_interval_sec) {
    history_.clear();
    const VehicleMotionSample reset{
      timestamp_sec, raw_speed_mps, raw_speed_mps, 0.0};
    history_.push_back(reset);
    return reset;
  }

  const double speed_gain = filterGain(
    dt_sec, config_.speed_filter_time_constant_sec);
  const double speed_mps = previous.speed_mps +
    speed_gain * (raw_speed_mps - previous.speed_mps);
  const double raw_acceleration_mps2 = std::clamp(
    (speed_mps - previous.speed_mps) / dt_sec,
    -config_.maximum_longitudinal_acceleration_mps2,
    config_.maximum_longitudinal_acceleration_mps2);
  const double acceleration_gain = filterGain(
    dt_sec, config_.acceleration_filter_time_constant_sec);
  const double acceleration_mps2 =
    previous.longitudinal_acceleration_mps2 + acceleration_gain *
    (raw_acceleration_mps2 - previous.longitudinal_acceleration_mps2);

  const VehicleMotionSample sample{
    timestamp_sec, raw_speed_mps, speed_mps, acceleration_mps2};
  history_.push_back(sample);
  while (
    history_.size() > 2U &&
    timestamp_sec - history_.front().timestamp_sec >
    config_.history_duration_sec)
  {
    history_.pop_front();
  }
  return sample;
}

std::optional<VehicleMotionSample> VehicleMotionEstimator::estimateAt(
  const double timestamp_sec) const
{
  if (!std::isfinite(timestamp_sec)) {
    return std::nullopt;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (history_.empty() || timestamp_sec < history_.front().timestamp_sec) {
    return std::nullopt;
  }

  const auto next = std::lower_bound(
    history_.begin(), history_.end(), timestamp_sec,
    [](const VehicleMotionSample & sample, const double time) {
      return sample.timestamp_sec < time;
    });
  if (next != history_.end()) {
    if (next == history_.begin() || next->timestamp_sec == timestamp_sec) {
      return *next;
    }
    return interpolateSample(*std::prev(next), *next, timestamp_sec);
  }

  const VehicleMotionSample & latest = history_.back();
  const double age_sec = timestamp_sec - latest.timestamp_sec;
  if (age_sec < 0.0 || age_sec > config_.maximum_sample_age_sec) {
    return std::nullopt;
  }
  VehicleMotionSample predicted = latest;
  predicted.timestamp_sec = timestamp_sec;
  predicted.speed_mps = std::clamp(
    latest.speed_mps + latest.longitudinal_acceleration_mps2 * age_sec,
    -config_.maximum_speed_mps,
    config_.maximum_speed_mps);
  return predicted;
}

double VehicleMotionEstimator::totalGearRatio() const
{
  return total_gear_ratio_;
}

double VehicleMotionEstimator::metersPerSecondPerErpm() const
{
  return meters_per_second_per_erpm_;
}

double VehicleMotionEstimator::speedFromErpm(
  const std::int32_t measured_erpm) const
{
  double speed_mps =
    static_cast<double>(measured_erpm) * meters_per_second_per_erpm_;
  if (std::abs(speed_mps) < config_.speed_deadband_mps) {
    speed_mps = 0.0;
  }
  return std::clamp(
    speed_mps, -config_.maximum_speed_mps, config_.maximum_speed_mps);
}

}  // namespace camera_driver
