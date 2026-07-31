#include "bev_processor/startup_attitude.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bev_processor
{

namespace
{

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kRadiansToDegrees = 180.0 / kPi;
constexpr double kDegreesToRadians = kPi / 180.0;

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
    throw std::invalid_argument("attitude normal must be finite/non-zero");
  }
  return value / norm;
}

double rollDegrees(const cv::Vec3d & up)
{
  return std::atan2(-up[0], -up[1]) * kRadiansToDegrees;
}

double pitchDownDegrees(const cv::Vec3d & up)
{
  return std::atan2(
    -up[2], std::hypot(up[0], up[1])) * kRadiansToDegrees;
}

double angleDegrees(const cv::Vec3d & first, const cv::Vec3d & second)
{
  return std::acos(std::clamp(first.dot(second), -1.0, 1.0)) *
         kRadiansToDegrees;
}

}  // namespace

StartupAttitudeSource parseStartupAttitudeSource(const std::string & value)
{
  if (value == "depth") {
    return StartupAttitudeSource::kDepth;
  }
  if (value == "imu") {
    return StartupAttitudeSource::kImu;
  }
  throw std::invalid_argument(
          "measurement_attitude_source must be 'depth' or 'imu'");
}

const char * startupAttitudeSourceName(const StartupAttitudeSource source)
{
  switch (source) {
    case StartupAttitudeSource::kDepth:
      return "depth";
    case StartupAttitudeSource::kImu:
      return "imu";
  }
  return "unknown";
}

cv::Vec3d attitudeUpVector(
  const double roll_deg,
  const double pitch_down_deg)
{
  if (!std::isfinite(roll_deg) || !std::isfinite(pitch_down_deg)) {
    throw std::invalid_argument("attitude angles must be finite");
  }
  const double roll_rad = roll_deg * kDegreesToRadians;
  const double pitch_rad = pitch_down_deg * kDegreesToRadians;
  return cv::Vec3d(
    -std::sin(roll_rad) * std::cos(pitch_rad),
    -std::cos(roll_rad) * std::cos(pitch_rad),
    -std::sin(pitch_rad));
}

StartupAttitudeSelection selectStartupAttitude(
  const cv::Vec3d & raw_imu_up_camera,
  const cv::Vec3d & depth_up_camera,
  const StartupAttitudeSource source,
  const double imu_roll_bias_deg,
  const double imu_pitch_bias_deg)
{
  if (
    !std::isfinite(imu_roll_bias_deg) ||
    !std::isfinite(imu_pitch_bias_deg))
  {
    throw std::invalid_argument("IMU attitude biases must be finite");
  }

  const cv::Vec3d raw_imu_up = normalized(raw_imu_up_camera);
  const cv::Vec3d depth_up = normalized(depth_up_camera);
  const double corrected_imu_roll_deg =
    rollDegrees(raw_imu_up) - imu_roll_bias_deg;
  const double corrected_imu_pitch_down_deg =
    pitchDownDegrees(raw_imu_up) - imu_pitch_bias_deg;
  const cv::Vec3d corrected_imu_up = attitudeUpVector(
    corrected_imu_roll_deg, corrected_imu_pitch_down_deg);
  const double depth_roll_deg = rollDegrees(depth_up);
  const double depth_pitch_down_deg = pitchDownDegrees(depth_up);

  StartupAttitudeSelection result;
  result.source = source;
  result.corrected_imu_roll_deg = corrected_imu_roll_deg;
  result.corrected_imu_pitch_down_deg = corrected_imu_pitch_down_deg;
  result.depth_roll_deg = depth_roll_deg;
  result.depth_pitch_down_deg = depth_pitch_down_deg;
  result.imu_depth_difference_deg =
    angleDegrees(corrected_imu_up, depth_up);
  if (source == StartupAttitudeSource::kImu) {
    result.roll_deg = corrected_imu_roll_deg;
    result.pitch_down_deg = corrected_imu_pitch_down_deg;
  } else {
    result.roll_deg = depth_roll_deg;
    result.pitch_down_deg = depth_pitch_down_deg;
  }
  return result;
}

}  // namespace bev_processor
