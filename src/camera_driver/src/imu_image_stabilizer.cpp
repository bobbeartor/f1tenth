#include "camera_driver/imu_image_stabilizer.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>

namespace camera_driver
{

namespace
{

constexpr double kDegreesToRadians =
  3.141592653589793238462643383279502884 / 180.0;
constexpr double kRadiansToDegrees = 1.0 / kDegreesToRadians;

bool finiteVector(const cv::Vec3d & value)
{
  return
    std::isfinite(value[0]) &&
    std::isfinite(value[1]) &&
    std::isfinite(value[2]);
}

cv::Vec3d normalized(const cv::Vec3d & value)
{
  const double norm = cv::norm(value);
  if (!finiteVector(value) || !std::isfinite(norm) || norm <= 1.0e-12) {
    throw std::invalid_argument("stabilizer vector must be finite/non-zero");
  }
  return value / norm;
}

cv::Vec3d integrateGyroscope(
  const cv::Vec3d & up_camera,
  const cv::Vec3d & angular_velocity_camera_radps,
  const double dt_sec)
{
  const double angular_speed = cv::norm(angular_velocity_camera_radps);
  if (angular_speed <= 1.0e-12) {
    return up_camera;
  }

  const cv::Vec3d axis = angular_velocity_camera_radps / angular_speed;
  const double angle = angular_speed * dt_sec;
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  return normalized(
    cosine * up_camera -
    sine * axis.cross(up_camera) +
    (1.0 - cosine) * axis.dot(up_camera) * axis);
}

double rollDegrees(const cv::Vec3d & up_camera)
{
  return std::atan2(-up_camera[0], -up_camera[1]) * kRadiansToDegrees;
}

double pitchDegrees(const cv::Vec3d & up_camera)
{
  return std::atan2(
    -up_camera[2],
    std::hypot(up_camera[0], up_camera[1])) * kRadiansToDegrees;
}

double angleDegrees(const cv::Vec3d & first, const cv::Vec3d & second)
{
  return std::acos(std::clamp(first.dot(second), -1.0, 1.0)) *
         kRadiansToDegrees;
}

double lerp(const double first, const double second, const double amount)
{
  return first + (second - first) * amount;
}

bool validConfig(const ImuImageStabilizerConfig & config)
{
  return
    config.warmup_samples > 0U &&
    std::isfinite(config.minimum_acceleration_mps2) &&
    config.minimum_acceleration_mps2 > 0.0 &&
    std::isfinite(config.maximum_acceleration_mps2) &&
    config.maximum_acceleration_mps2 >
    config.minimum_acceleration_mps2 &&
    std::isfinite(config.acceleration_correction_time_constant_sec) &&
    config.acceleration_correction_time_constant_sec > 0.0 &&
    std::isfinite(config.acceleration_correction_gate_deg) &&
    config.acceleration_correction_gate_deg > 0.0 &&
    config.acceleration_correction_gate_deg < 90.0 &&
    std::isfinite(config.trajectory_smoothing_time_constant_sec) &&
    config.trajectory_smoothing_time_constant_sec > 0.0 &&
    std::isfinite(config.maximum_correction_deg) &&
    config.maximum_correction_deg > 0.0 &&
    config.maximum_correction_deg < 45.0 &&
    std::isfinite(config.maximum_sample_interval_sec) &&
    config.maximum_sample_interval_sec > 0.0 &&
    std::isfinite(config.maximum_history_sec) &&
    config.maximum_history_sec > 0.0 &&
    std::isfinite(config.maximum_frame_imu_age_sec) &&
    config.maximum_frame_imu_age_sec > 0.0;
}

}  // namespace

ImuImageStabilizer::ImuImageStabilizer(
  const ImuImageStabilizerConfig & config)
: config_(config)
{
  if (!validConfig(config_)) {
    throw std::invalid_argument("invalid IMU image stabilizer configuration");
  }
}

void ImuImageStabilizer::update(
  const cv::Vec3d & acceleration_camera_mps2,
  const cv::Vec3d & angular_velocity_camera_radps,
  const double timestamp_sec)
{
  if (
    !finiteVector(acceleration_camera_mps2) ||
    !finiteVector(angular_velocity_camera_radps) ||
    !std::isfinite(timestamp_sec))
  {
    return;
  }

  const double acceleration_magnitude = cv::norm(
    acceleration_camera_mps2);
  const bool acceleration_valid =
    std::isfinite(acceleration_magnitude) &&
    acceleration_magnitude >= config_.minimum_acceleration_mps2 &&
    acceleration_magnitude <= config_.maximum_acceleration_mps2;

  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_) {
    if (!acceleration_valid) {
      return;
    }
    warmup_acceleration_sum_ += acceleration_camera_mps2;
    warmup_gyroscope_sum_ += angular_velocity_camera_radps;
    ++warmup_count_;
    if (warmup_count_ < config_.warmup_samples) {
      return;
    }

    up_camera_ = normalized(warmup_acceleration_sum_);
    gyroscope_bias_radps_ =
      warmup_gyroscope_sum_ / static_cast<double>(warmup_count_);
    smoothed_roll_deg_ = rollDegrees(up_camera_);
    smoothed_pitch_deg_ = pitchDegrees(up_camera_);
    last_timestamp_sec_ = timestamp_sec;
    initialized_ = true;
    history_.push_back(
      TimedCorrection{
        timestamp_sec,
        ImageStabilizationCorrection{}});
    return;
  }

  const double dt_sec = timestamp_sec - last_timestamp_sec_;
  if (!std::isfinite(dt_sec) || dt_sec <= 0.0) {
    return;
  }
  last_timestamp_sec_ = timestamp_sec;

  if (dt_sec > config_.maximum_sample_interval_sec) {
    if (acceleration_valid) {
      up_camera_ = acceleration_camera_mps2 / acceleration_magnitude;
      smoothed_roll_deg_ = rollDegrees(up_camera_);
      smoothed_pitch_deg_ = pitchDegrees(up_camera_);
      history_.clear();
      history_.push_back(
        TimedCorrection{
          timestamp_sec,
          ImageStabilizationCorrection{}});
    }
    return;
  }

  up_camera_ = integrateGyroscope(
    up_camera_,
    angular_velocity_camera_radps - gyroscope_bias_radps_,
    dt_sec);
  if (acceleration_valid) {
    const cv::Vec3d measured_up =
      acceleration_camera_mps2 / acceleration_magnitude;
    if (
      angleDegrees(up_camera_, measured_up) <=
      config_.acceleration_correction_gate_deg)
    {
      const double correction_gain =
        1.0 - std::exp(
        -dt_sec / config_.acceleration_correction_time_constant_sec);
      up_camera_ = normalized(
        (1.0 - correction_gain) * up_camera_ +
        correction_gain * measured_up);
    }
  }

  const double roll_deg = rollDegrees(up_camera_);
  const double pitch_deg = pitchDegrees(up_camera_);
  const double smoothing_gain =
    1.0 - std::exp(
    -dt_sec / config_.trajectory_smoothing_time_constant_sec);
  smoothed_roll_deg_ = lerp(
    smoothed_roll_deg_, roll_deg, smoothing_gain);
  smoothed_pitch_deg_ = lerp(
    smoothed_pitch_deg_, pitch_deg, smoothing_gain);

  const ImageStabilizationCorrection correction{
    std::clamp(
      roll_deg - smoothed_roll_deg_,
      -config_.maximum_correction_deg,
      config_.maximum_correction_deg),
    std::clamp(
      pitch_deg - smoothed_pitch_deg_,
      -config_.maximum_correction_deg,
      config_.maximum_correction_deg)};
  history_.push_back(TimedCorrection{timestamp_sec, correction});
  while (
    history_.size() > 2U &&
    timestamp_sec - history_.front().timestamp_sec >
    config_.maximum_history_sec)
  {
    history_.pop_front();
  }
}

std::optional<ImageStabilizationCorrection>
ImuImageStabilizer::correctionAt(const double timestamp_sec) const
{
  if (!std::isfinite(timestamp_sec)) {
    return std::nullopt;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_ || history_.empty()) {
    return std::nullopt;
  }
  if (
    timestamp_sec < history_.front().timestamp_sec ||
    timestamp_sec - history_.back().timestamp_sec >
    config_.maximum_frame_imu_age_sec)
  {
    return std::nullopt;
  }
  if (timestamp_sec >= history_.back().timestamp_sec) {
    return history_.back().correction;
  }

  const auto next = std::lower_bound(
    history_.begin(),
    history_.end(),
    timestamp_sec,
    [](const TimedCorrection & sample, const double time) {
      return sample.timestamp_sec < time;
    });
  if (next == history_.begin()) {
    return next->correction;
  }
  const auto previous = std::prev(next);
  const double interval_sec =
    next->timestamp_sec - previous->timestamp_sec;
  if (interval_sec <= 0.0) {
    return next->correction;
  }
  const double amount = std::clamp(
    (timestamp_sec - previous->timestamp_sec) / interval_sec,
    0.0,
    1.0);
  return ImageStabilizationCorrection{
    lerp(
      previous->correction.roll_deg,
      next->correction.roll_deg,
      amount),
    lerp(
      previous->correction.pitch_deg,
      next->correction.pitch_deg,
      amount)};
}

bool ImuImageStabilizer::initialized() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return initialized_;
}

void ImuImageStabilizer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  warmup_count_ = 0U;
  warmup_acceleration_sum_ = cv::Vec3d(0.0, 0.0, 0.0);
  warmup_gyroscope_sum_ = cv::Vec3d(0.0, 0.0, 0.0);
  gyroscope_bias_radps_ = cv::Vec3d(0.0, 0.0, 0.0);
  up_camera_ = cv::Vec3d(0.0, -1.0, 0.0);
  smoothed_roll_deg_ = 0.0;
  smoothed_pitch_deg_ = 0.0;
  last_timestamp_sec_ = 0.0;
  initialized_ = false;
  history_.clear();
}

cv::Matx33d makeImageStabilizationHomography(
  const double fx,
  const double fy,
  const double cx,
  const double cy,
  const ImageStabilizationCorrection & correction)
{
  if (
    !std::isfinite(fx) || !std::isfinite(fy) ||
    !std::isfinite(cx) || !std::isfinite(cy) ||
    fx <= 0.0 || fy <= 0.0 ||
    !std::isfinite(correction.roll_deg) ||
    !std::isfinite(correction.pitch_deg))
  {
    throw std::invalid_argument(
            "invalid camera intrinsics or stabilization correction");
  }

  const double roll = correction.roll_deg * kDegreesToRadians;
  const double pitch = correction.pitch_deg * kDegreesToRadians;
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const cv::Matx33d rotation_z(
    cr, -sr, 0.0,
    sr, cr, 0.0,
    0.0, 0.0, 1.0);
  const cv::Matx33d rotation_x(
    1.0, 0.0, 0.0,
    0.0, cp, sp,
    0.0, -sp, cp);
  const cv::Matx33d camera_matrix(
    fx, 0.0, cx,
    0.0, fy, cy,
    0.0, 0.0, 1.0);
  return
    camera_matrix *
    (rotation_z * rotation_x) *
    camera_matrix.inv();
}

}  // namespace camera_driver
