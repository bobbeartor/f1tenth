#include "camera_driver/imu_image_stabilizer.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <iterator>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <utility>

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
    throw std::runtime_error("stabilizer vector must be finite and non-zero");
  }
  return value / norm;
}

struct Quaternion
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

bool finiteQuaternion(const Quaternion & value)
{
  return
    std::isfinite(value.w) && std::isfinite(value.x) &&
    std::isfinite(value.y) && std::isfinite(value.z);
}

Quaternion normalized(const Quaternion & value)
{
  const double norm = std::sqrt(
    value.w * value.w + value.x * value.x +
    value.y * value.y + value.z * value.z);
  if (!finiteQuaternion(value) || !std::isfinite(norm) || norm <= 1.0e-12) {
    throw std::runtime_error(
            "stabilizer quaternion must be finite and non-zero");
  }
  return Quaternion{
    value.w / norm, value.x / norm, value.y / norm, value.z / norm};
}

Quaternion conjugate(const Quaternion & value)
{
  return Quaternion{value.w, -value.x, -value.y, -value.z};
}

Quaternion multiply(const Quaternion & first, const Quaternion & second)
{
  return Quaternion{
    first.w * second.w - first.x * second.x -
    first.y * second.y - first.z * second.z,
    first.w * second.x + first.x * second.w +
    first.y * second.z - first.z * second.y,
    first.w * second.y - first.x * second.z +
    first.y * second.w + first.z * second.x,
    first.w * second.z + first.x * second.y -
    first.y * second.x + first.z * second.w};
}

Quaternion negated(const Quaternion & value)
{
  return Quaternion{-value.w, -value.x, -value.y, -value.z};
}

Quaternion quaternionFromRotationVector(const cv::Vec3d & rotation_rad)
{
  const double angle = cv::norm(rotation_rad);
  if (!finiteVector(rotation_rad) || !std::isfinite(angle)) {
    throw std::runtime_error("rotation vector must be finite");
  }
  if (angle <= 1.0e-12) {
    return normalized(Quaternion{
      1.0, 0.5 * rotation_rad[0],
      0.5 * rotation_rad[1], 0.5 * rotation_rad[2]});
  }
  const double half_angle = 0.5 * angle;
  const double scale = std::sin(half_angle) / angle;
  return Quaternion{
    std::cos(half_angle),
    scale * rotation_rad[0],
    scale * rotation_rad[1],
    scale * rotation_rad[2]};
}

cv::Vec3d rotationVector(const Quaternion & input)
{
  Quaternion value = normalized(input);
  if (value.w < 0.0) {
    value = negated(value);
  }
  const double vector_norm = std::sqrt(
    value.x * value.x + value.y * value.y + value.z * value.z);
  if (vector_norm <= 1.0e-12) {
    return cv::Vec3d(2.0 * value.x, 2.0 * value.y, 2.0 * value.z);
  }
  const double angle = 2.0 * std::atan2(vector_norm, value.w);
  return cv::Vec3d(value.x, value.y, value.z) * (angle / vector_norm);
}

cv::Vec3d rotateVector(
  const Quaternion & rotation,
  const cv::Vec3d & value)
{
  const Quaternion unit = normalized(rotation);
  const Quaternion vector{0.0, value[0], value[1], value[2]};
  const Quaternion rotated = multiply(multiply(unit, vector), conjugate(unit));
  return cv::Vec3d(rotated.x, rotated.y, rotated.z);
}

Quaternion quaternionFromTwoVectors(
  const cv::Vec3d & source_input,
  const cv::Vec3d & target_input)
{
  const cv::Vec3d source = normalized(source_input);
  const cv::Vec3d target = normalized(target_input);
  const double dot = std::clamp(source.dot(target), -1.0, 1.0);
  if (dot > 1.0 - 1.0e-12) {
    return Quaternion{};
  }
  if (dot < -1.0 + 1.0e-12) {
    cv::Vec3d axis = source.cross(cv::Vec3d(1.0, 0.0, 0.0));
    if (cv::norm(axis) <= 1.0e-6) {
      axis = source.cross(cv::Vec3d(0.0, 1.0, 0.0));
    }
    return quaternionFromRotationVector(
      normalized(axis) * 3.141592653589793238462643383279502884);
  }
  const cv::Vec3d cross = source.cross(target);
  return normalized(Quaternion{1.0 + dot, cross[0], cross[1], cross[2]});
}

cv::Matx33d quaternionRotationMatrix(const Quaternion & input)
{
  const Quaternion value = normalized(input);
  const double xx = value.x * value.x;
  const double yy = value.y * value.y;
  const double zz = value.z * value.z;
  const double xy = value.x * value.y;
  const double xz = value.x * value.z;
  const double yz = value.y * value.z;
  const double wx = value.w * value.x;
  const double wy = value.w * value.y;
  const double wz = value.w * value.z;
  return cv::Matx33d(
    1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy),
    2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
    2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy));
}

double angleDegrees(const cv::Vec3d & first, const cv::Vec3d & second)
{
  return std::acos(std::clamp(first.dot(second), -1.0, 1.0)) *
         kRadiansToDegrees;
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

cv::Vec3d upVectorFromRollPitchDegrees(
  const double roll_deg,
  const double pitch_deg)
{
  const double roll_rad = roll_deg * kDegreesToRadians;
  const double pitch_rad = pitch_deg * kDegreesToRadians;
  return cv::Vec3d(
    -std::sin(roll_rad) * std::cos(pitch_rad),
    -std::cos(roll_rad) * std::cos(pitch_rad),
    -std::sin(pitch_rad));
}

cv::Vec3d interpolateDirection(
  const cv::Vec3d & first,
  const cv::Vec3d & second,
  const double amount)
{
  const Quaternion first_to_second = quaternionFromTwoVectors(first, second);
  const Quaternion partial = quaternionFromRotationVector(
    rotationVector(first_to_second) * std::clamp(amount, 0.0, 1.0));
  return normalized(rotateVector(partial, first));
}

double wrapDegrees(double value)
{
  while (value > 180.0) {
    value -= 360.0;
  }
  while (value < -180.0) {
    value += 360.0;
  }
  return value;
}

bool validHomography(const cv::Matx33d & homography)
{
  const double determinant = cv::determinant(cv::Mat(homography));
  return
    cv::checkRange(cv::Mat(homography)) &&
    std::isfinite(determinant) &&
    std::abs(determinant) > 1.0e-12;
}

bool validConfig(const ImuImageStabilizerConfig & config)
{
  const auto positive_finite = [](const double value) {
      return std::isfinite(value) && value > 0.0;
    };
  return
    std::isfinite(config.startup_discard_duration_sec) &&
    config.startup_discard_duration_sec >= 0.0 &&
    config.startup_discard_duration_sec <= 10.0 &&
    positive_finite(config.reference_calibration_duration_sec) &&
    positive_finite(config.calibration_maximum_angular_speed_degps) &&
    positive_finite(config.gravity_mps2) &&
    config.accelerometer_full_trust_deviation_mps2 >= 0.0 &&
    positive_finite(config.accelerometer_zero_trust_deviation_mps2) &&
    config.accelerometer_zero_trust_deviation_mps2 >
    config.accelerometer_full_trust_deviation_mps2 &&
    positive_finite(config.acceleration_correction_time_constant_sec) &&
    positive_finite(config.acceleration_correction_gate_deg) &&
    config.acceleration_correction_gate_deg < 90.0 &&
    positive_finite(config.roll_acceleration_correction_time_constant_sec) &&
    positive_finite(config.roll_acceleration_direction_gate_deg) &&
    config.roll_acceleration_direction_gate_deg < 90.0 &&
    positive_finite(config.moving_accelerometer_nudge_time_constant_sec) &&
    std::isfinite(config.moving_accelerometer_nudge_strength) &&
    config.moving_accelerometer_nudge_strength >= 0.0 &&
    config.moving_accelerometer_nudge_strength <= 1.0 &&
    positive_finite(config.moving_accelerometer_pitch_nudge_maximum_deg) &&
    config.moving_accelerometer_pitch_nudge_maximum_deg < 5.0 &&
    positive_finite(config.moving_accelerometer_roll_nudge_maximum_deg) &&
    config.moving_accelerometer_roll_nudge_maximum_deg < 5.0 &&
    positive_finite(
      config.moving_gravity_anchor_maximum_correction_rate_degps) &&
    config.moving_gravity_anchor_maximum_correction_rate_degps <= 10.0 &&
    positive_finite(config.reference_tilt_leak_time_constant_sec) &&
    positive_finite(config.stationary_tilt_recovery_time_constant_sec) &&
    positive_finite(config.online_gyroscope_tilt_bias_time_constant_sec) &&
    positive_finite(config.stationary_detection_window_sec) &&
    config.stationary_detection_window_sec <= 10.0 &&
    positive_finite(config.stationary_accelerometer_norm_tolerance_mps2) &&
    config.stationary_accelerometer_norm_tolerance_mps2 <
    config.accelerometer_zero_trust_deviation_mps2 &&
    positive_finite(config.stationary_accelerometer_norm_stddev_mps2) &&
    config.stationary_accelerometer_norm_stddev_mps2 <
    config.stationary_accelerometer_norm_tolerance_mps2 &&
    positive_finite(config.stationary_accelerometer_direction_error_deg) &&
    config.stationary_accelerometer_direction_error_deg < 45.0 &&
    positive_finite(config.stationary_accelerometer_direction_change_deg) &&
    config.stationary_accelerometer_direction_change_deg <
    config.stationary_accelerometer_direction_error_deg &&
    positive_finite(config.stationary_gyroscope_mean_maximum_degps) &&
    config.stationary_gyroscope_mean_maximum_degps < 5.0 &&
    positive_finite(config.stationary_gyroscope_stddev_maximum_degps) &&
    config.stationary_gyroscope_stddev_maximum_degps < 5.0 &&
    (config.pitch_correction_enabled || config.roll_correction_enabled) &&
    std::isfinite(config.gyroscope_correction_gain) &&
    config.gyroscope_correction_gain >= 0.0 &&
    config.gyroscope_correction_gain <= 1.0 &&
    positive_finite(config.maximum_correction_deg) &&
    config.maximum_correction_deg < 45.0 &&
    positive_finite(config.maximum_sample_interval_sec) &&
    positive_finite(config.maximum_history_sec) &&
    positive_finite(config.maximum_frame_imu_wait_sec) &&
    positive_finite(config.maximum_frame_imu_age_sec) &&
    positive_finite(config.maximum_frame_imu_prediction_sec) &&
    config.maximum_frame_imu_prediction_sec <= 0.050;
}

}  // namespace

class ImuImageStabilizer::Impl
{
public:
  explicit Impl(const ImuImageStabilizerConfig & config)
  : config_(config),
    startup_discard_completed_(config.startup_discard_duration_sec <= 0.0)
  {
    if (!validConfig(config_)) {
      throw std::invalid_argument("invalid IMU image stabilizer configuration");
    }
  }

  void update(
    const std::optional<cv::Vec3d> & acceleration_camera_mps2,
    const cv::Vec3d & angular_velocity_camera_radps,
    const double timestamp_sec,
    const std::optional<bool> vehicle_stationary)
  {
    if (
      !finiteVector(angular_velocity_camera_radps) ||
      !std::isfinite(timestamp_sec))
    {
      return;
    }

    const bool acceleration_available =
      acceleration_camera_mps2.has_value() &&
      finiteVector(*acceleration_camera_mps2);
    const double acceleration_magnitude = acceleration_available ?
      cv::norm(*acceleration_camera_mps2) :
      std::numeric_limits<double>::quiet_NaN();
    const double acceleration_confidence = acceleration_available ?
      accelerometerConfidence(acceleration_magnitude) : 0.0;
    const double angular_speed_degps =
      cv::norm(angular_velocity_camera_radps) * kRadiansToDegrees;

    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
      calibration_last_accelerometer_confidence_ = acceleration_confidence;
      calibration_last_angular_speed_degps_ = angular_speed_degps;
      if (!startup_discard_completed_) {
        if (!startup_discard_started_) {
          startup_discard_started_ = true;
          startup_discard_started_at_sec_ = timestamp_sec;
        }
        startup_discard_last_timestamp_sec_ = timestamp_sec;
        if (
          timestamp_sec - startup_discard_started_at_sec_ <
          config_.startup_discard_duration_sec)
        {
          return;
        }
        startup_discard_completed_ = true;
        calibration_last_rejection_reason_.clear();
        return;
      }

      if (!acceleration_available) {
        return;
      }
      if (acceleration_confidence <= 0.0) {
        resetCalibrationLocked("acceleration magnitude outside usable range");
        return;
      }
      if (calibration_count_ == 0U) {
        calibration_started_at_sec_ = timestamp_sec;
      }
      warmup_acceleration_sum_ += *acceleration_camera_mps2;
      warmup_gyroscope_sum_ += angular_velocity_camera_radps;
      calibration_samples_.push_back(StationarySample{
        timestamp_sec, *acceleration_camera_mps2,
        angular_velocity_camera_radps});
      for (int axis = 0; axis < 3; ++axis) {
        warmup_gyroscope_squared_sum_[axis] +=
          angular_velocity_camera_radps[axis] *
          angular_velocity_camera_radps[axis];
      }
      ++calibration_count_;
      calibration_last_timestamp_sec_ = timestamp_sec;
      if (
        timestamp_sec - calibration_started_at_sec_ <
        config_.reference_calibration_duration_sec)
      {
        return;
      }

      const std::string rejection_reason =
        calibrationWindowRejectionReasonLocked();
      if (!rejection_reason.empty()) {
        resetCalibrationLocked(rejection_reason);
        return;
      }

      if (
        config_.external_reference_required &&
        !external_reference_up_camera_.has_value())
      {
        resetCalibrationLocked("waiting for BEV startup ground reference");
        return;
      }

      // The accelerometer baseline and the geometric reference deliberately
      // remain separate. The former identifies the physical startup pose for
      // stationary detection; the latter is the depth-plane normal used by
      // the fixed BEV LUT. This prevents an IMU/depth mounting offset from
      // blocking drift recovery.
      stationary_acceleration_reference_up_camera_ =
        normalized(warmup_acceleration_sum_);
      reference_up_camera_ = external_reference_up_camera_.value_or(
        stationary_acceleration_reference_up_camera_);
      // Depth supplies the absolute BEV attitude, while the accelerometer may
      // have a repeatable mounting/calibration offset from that ground-plane
      // normal. Preserve the depth reference and use the accelerometer only
      // for changes relative to its own startup direction.
      accelerometer_to_reference_rotation_ = quaternionFromTwoVectors(
        stationary_acceleration_reference_up_camera_, reference_up_camera_);
      double acceleration_norm_sum = 0.0;
      for (const StationarySample & sample : calibration_samples_) {
        acceleration_norm_sum += cv::norm(sample.acceleration_camera_mps2);
      }
      gravity_magnitude_reference_mps2_ = acceleration_norm_sum /
        static_cast<double>(calibration_samples_.size());
      gyroscope_bias_radps_ = config_.gyroscope_bias_enabled ?
        warmup_gyroscope_sum_ / static_cast<double>(calibration_count_) :
        cv::Vec3d(0.0, 0.0, 0.0);
      current_up_camera_ = reference_up_camera_;
      moving_accelerometer_nudge_deg_ = cv::Vec2d(0.0, 0.0);
      last_timestamp_sec_ = timestamp_sec;
      calibration_samples_.clear();
      stationary_samples_.clear();
      stationary_confirmed_ = false;
      initialized_ = true;
      history_.push_back(TimedTilt{
        timestamp_sec, current_up_camera_,
        cv::Vec3d(0.0, 0.0, 0.0)});
      history_condition_.notify_all();
      return;
    }

    const double dt_sec = timestamp_sec - last_timestamp_sec_;
    if (!std::isfinite(dt_sec) || dt_sec <= 0.0) {
      return;
    }
    last_timestamp_sec_ = timestamp_sec;
    if (dt_sec > config_.maximum_sample_interval_sec) {
      throw std::runtime_error(
              "IMU timestamp discontinuity exceeded maximum_sample_interval");
    }

    stationary_confirmed_ = false;
    cv::Vec3d stationary_gyroscope_mean = angular_velocity_camera_radps;
    if (vehicle_stationary.has_value()) {
      // Runtime vehicle motion is supplied by measured ERPM. Startup
      // calibration above intentionally remains IMU-based.
      stationary_samples_.clear();
      stationary_confirmed_ = *vehicle_stationary;
    } else if (acceleration_available) {
      stationary_samples_.push_back(StationarySample{
        timestamp_sec, *acceleration_camera_mps2,
        angular_velocity_camera_radps});
      while (
        stationary_samples_.size() > 2U &&
        timestamp_sec - stationary_samples_[1].timestamp_sec >=
        config_.stationary_detection_window_sec)
      {
        stationary_samples_.pop_front();
      }
      const StationaryWindowAssessment stationary =
        assessStationaryWindowLocked();
      stationary_confirmed_ = stationary.confirmed;
      stationary_gyroscope_mean = stationary.mean_gyroscope_radps;
    } else {
      stationary_samples_.clear();
    }

    if (
      config_.online_gyroscope_tilt_bias_enabled &&
      stationary_confirmed_)
    {
      const double bias_gain = 1.0 - std::exp(
        -dt_sec / config_.online_gyroscope_tilt_bias_time_constant_sec);
      // Once the vehicle is confirmed stationary, every camera-frame gyro
      // axis measures bias. Updating all three axes prevents a residual
      // camera-Y bias from leaking into tilt on a downward-mounted camera.
      for (int axis = 0; axis < 3; ++axis) {
        gyroscope_bias_radps_[axis] += bias_gain *
          (stationary_gyroscope_mean[axis] - gyroscope_bias_radps_[axis]);
      }
      ++online_tilt_bias_update_count_;
    }

    const cv::Vec3d corrected_angular_velocity =
      angular_velocity_camera_radps - gyroscope_bias_radps_;
    // Track only the reference-plane normal in camera coordinates. A pure
    // yaw/U-turn rate is parallel to this vector and therefore cannot leak
    // into roll or pitch through a drifting yaw state.
    current_up_camera_ = normalized(rotateVector(
      conjugate(quaternionFromRotationVector(
        corrected_angular_velocity * dt_sec)),
      current_up_camera_));
    // The flat-road suspension model says long-term tilt returns to the
    // startup pose. A deliberately weak leak supplies that low-frequency
    // prior without trusting vehicle acceleration as gravity while driving.
    const double reference_leak_gain = 1.0 - std::exp(
      -dt_sec / config_.reference_tilt_leak_time_constant_sec);
    current_up_camera_ = interpolateDirection(
      current_up_camera_, reference_up_camera_, reference_leak_gain);

    double roll_acceleration_confidence = 0.0;
    double pitch_acceleration_confidence = 0.0;
    const bool use_stationary_reference = stationary_confirmed_;
    const bool use_moving_accelerometer =
      !use_stationary_reference && acceleration_available &&
      acceleration_confidence > 0.0 &&
      !config_.acceleration_correction_stationary_only;
    if (use_stationary_reference || use_moving_accelerometer) {
      const cv::Vec3d predicted_up = current_up_camera_;
      const double predicted_roll_deg = rollDegrees(predicted_up);
      const double predicted_pitch_deg = pitchDegrees(predicted_up);
      cv::Vec3d correction_target = reference_up_camera_;
      if (use_stationary_reference) {
        roll_acceleration_confidence = 1.0;
        pitch_acceleration_confidence = 1.0;
      } else {
        const cv::Vec3d measured_accelerometer_up =
          *acceleration_camera_mps2 / acceleration_magnitude;
        const cv::Vec3d measured_up = normalized(rotateVector(
          accelerometer_to_reference_rotation_,
          measured_accelerometer_up));
        correction_target = measured_up;
        const double direction_error_deg = angleDegrees(
          predicted_up, measured_up);
        const double pitch_direction_confidence = std::clamp(
          1.0 - direction_error_deg /
          config_.acceleration_correction_gate_deg,
          0.0, 1.0);
        const double roll_error_deg = wrapDegrees(
          rollDegrees(measured_up) - predicted_roll_deg);
        const double roll_direction_confidence = std::clamp(
          1.0 - std::abs(roll_error_deg) /
          config_.roll_acceleration_direction_gate_deg,
          0.0, 1.0);
        roll_acceleration_confidence =
          acceleration_confidence * roll_direction_confidence;
        pitch_acceleration_confidence =
          acceleration_confidence * pitch_direction_confidence;
      }
      if (
        roll_acceleration_confidence > 0.0 ||
        pitch_acceleration_confidence > 0.0)
      {
        const double roll_time_constant = use_stationary_reference ?
          config_.stationary_tilt_recovery_time_constant_sec :
          config_.roll_acceleration_correction_time_constant_sec;
        const double pitch_time_constant = use_stationary_reference ?
          config_.stationary_tilt_recovery_time_constant_sec :
          config_.acceleration_correction_time_constant_sec;
        const double roll_gain =
          roll_acceleration_confidence * (1.0 - std::exp(
          -dt_sec / roll_time_constant));
        const double pitch_gain =
          pitch_acceleration_confidence * (1.0 - std::exp(
          -dt_sec / pitch_time_constant));
        // ERPM-confirmed stationarity recovers to the immutable startup view
        // without allowing accelerometer noise to toggle this correction.
        const double target_roll_deg = rollDegrees(correction_target);
        const double target_pitch_deg = pitchDegrees(correction_target);
        double roll_step_deg = roll_gain * wrapDegrees(
          target_roll_deg - predicted_roll_deg);
        double pitch_step_deg = pitch_gain * wrapDegrees(
          target_pitch_deg - predicted_pitch_deg);
        if (!use_stationary_reference) {
          const double maximum_step_deg =
            config_.moving_gravity_anchor_maximum_correction_rate_degps *
            dt_sec;
          const double step_magnitude_deg = std::hypot(
            roll_step_deg, pitch_step_deg);
          if (step_magnitude_deg > maximum_step_deg) {
            const double step_scale = maximum_step_deg / step_magnitude_deg;
            roll_step_deg *= step_scale;
            pitch_step_deg *= step_scale;
          }
          ++moving_gravity_anchor_update_count_;
        }
        const cv::Vec3d corrected_up = upVectorFromRollPitchDegrees(
          predicted_roll_deg + roll_step_deg,
          predicted_pitch_deg + pitch_step_deg);
        current_up_camera_ = normalized(corrected_up);
      }
    }

    // Keep the persistent gyro/gravity-anchor attitude separate from the
    // supplementary fast nudge below. The nudge is never fed back into
    // current_up_camera_, so its estimation error cannot accumulate over time.
    cv::Vec2d target_moving_nudge_deg(0.0, 0.0);
    if (
      config_.moving_accelerometer_nudge_enabled &&
      !stationary_confirmed_ && acceleration_available &&
      acceleration_confidence > 0.0)
    {
      const cv::Vec3d measured_accelerometer_up =
        *acceleration_camera_mps2 / acceleration_magnitude;
      const cv::Vec3d measured_up = normalized(rotateVector(
        accelerometer_to_reference_rotation_, measured_accelerometer_up));
      const double base_roll_deg = rollDegrees(current_up_camera_);
      const double base_pitch_deg = pitchDegrees(current_up_camera_);
      const double roll_error_deg = wrapDegrees(
        rollDegrees(measured_up) - base_roll_deg);
      const double pitch_error_deg = wrapDegrees(
        pitchDegrees(measured_up) - base_pitch_deg);
      const double direction_error_deg = angleDegrees(
        current_up_camera_, measured_up);
      const double roll_direction_confidence = std::clamp(
        1.0 - std::abs(roll_error_deg) /
        config_.roll_acceleration_direction_gate_deg,
        0.0, 1.0);
      const double pitch_direction_confidence = std::clamp(
        1.0 - direction_error_deg /
        config_.acceleration_correction_gate_deg,
        0.0, 1.0);
      target_moving_nudge_deg[0] = std::clamp(
        config_.moving_accelerometer_nudge_strength *
        acceleration_confidence * roll_direction_confidence *
        roll_error_deg,
        -config_.moving_accelerometer_roll_nudge_maximum_deg,
        config_.moving_accelerometer_roll_nudge_maximum_deg);
      target_moving_nudge_deg[1] = std::clamp(
        config_.moving_accelerometer_nudge_strength *
        acceleration_confidence * pitch_direction_confidence *
        pitch_error_deg,
        -config_.moving_accelerometer_pitch_nudge_maximum_deg,
        config_.moving_accelerometer_pitch_nudge_maximum_deg);
    }
    if (config_.moving_accelerometer_nudge_enabled) {
      const double nudge_gain = 1.0 - std::exp(
        -dt_sec / config_.moving_accelerometer_nudge_time_constant_sec);
      moving_accelerometer_nudge_deg_ += nudge_gain *
        (target_moving_nudge_deg - moving_accelerometer_nudge_deg_);
    } else {
      moving_accelerometer_nudge_deg_ = cv::Vec2d(0.0, 0.0);
    }

    const cv::Vec3d output_up_camera = upVectorFromRollPitchDegrees(
      rollDegrees(current_up_camera_) + moving_accelerometer_nudge_deg_[0],
      pitchDegrees(current_up_camera_) + moving_accelerometer_nudge_deg_[1]);
    history_.push_back(TimedTilt{
      timestamp_sec, output_up_camera,
      corrected_angular_velocity});
    while (
      history_.size() > 2U &&
      timestamp_sec - history_.front().timestamp_sec >
      config_.maximum_history_sec)
    {
      history_.pop_front();
    }
    history_condition_.notify_all();
  }

  bool setExternalReferenceUpCamera(const cv::Vec3d & up_camera)
  {
    if (!finiteVector(up_camera) || cv::norm(up_camera) <= 1.0e-9) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialized_) {
      return false;
    }
    external_reference_up_camera_ = normalized(up_camera);
    if (
      calibration_last_rejection_reason_ ==
      "waiting for BEV startup ground reference")
    {
      calibration_last_rejection_reason_.clear();
    }
    return true;
  }

  std::optional<ImageStabilizationCorrection> correctionAt(
    const double timestamp_sec) const
  {
    return correctionAt(timestamp_sec,config_.maximum_frame_imu_age_sec,
      config_.maximum_frame_imu_wait_sec);
  }

  std::optional<ImageStabilizationCorrection> correctionAt(
    const double timestamp_sec, const double maximum_frame_imu_age_sec,
    const double maximum_wait_sec) const
  {
    if (!std::isfinite(maximum_frame_imu_age_sec) || maximum_frame_imu_age_sec<=0 ||
      !std::isfinite(maximum_wait_sec) || maximum_wait_sec<0) {
      throw std::invalid_argument("invalid per-consumer IMU lookup bounds");
    }
    if (!std::isfinite(timestamp_sec)) {
      return std::nullopt;
    }

    std::unique_lock<std::mutex> lock(mutex_);
    if (!initialized_ || history_.empty()) {
      return std::nullopt;
    }
    if (timestamp_sec < history_.front().timestamp_sec) {
      return std::nullopt;
    }

    const auto target_is_bracketed = [this, timestamp_sec]() {
        return
          !history_.empty() &&
          history_.back().timestamp_sec >= timestamp_sec;
      };
    if (!target_is_bracketed()) {
      history_condition_.wait_for(
        lock,
        std::chrono::duration<double>(maximum_wait_sec),
        target_is_bracketed);
    }

    cv::Vec3d current_up_camera;
    double nearest_timestamp = 0.0;
    bool predicted = false;
    double prediction_horizon_sec = 0.0;
    if (target_is_bracketed()) {
      const auto next = std::lower_bound(
        history_.begin(), history_.end(), timestamp_sec,
        [](const TimedTilt & sample, const double time) {
          return sample.timestamp_sec < time;
        });
      if (next == history_.end()) {
        return std::nullopt;
      }
      auto previous = next;
      if (std::abs(next->timestamp_sec - timestamp_sec) > 1.0e-9) {
        if (next == history_.begin()) {
          return std::nullopt;
        }
        previous = std::prev(next);
      }
      const double previous_age_sec =
        timestamp_sec - previous->timestamp_sec;
      const double next_age_sec = next->timestamp_sec - timestamp_sec;
      if (
        previous_age_sec < 0.0 || next_age_sec < 0.0 ||
        previous_age_sec > maximum_frame_imu_age_sec ||
        next_age_sec > maximum_frame_imu_age_sec)
      {
        return std::nullopt;
      }

      current_up_camera = next->up_camera;
      if (previous != next) {
        const double interval_sec =
          next->timestamp_sec - previous->timestamp_sec;
        if (interval_sec <= 0.0) {
          return std::nullopt;
        }
        const double amount = std::clamp(
          (timestamp_sec - previous->timestamp_sec) / interval_sec,
          0.0, 1.0);
        current_up_camera = interpolateDirection(
          previous->up_camera, next->up_camera, amount);
      }
      nearest_timestamp =
        std::abs(previous_age_sec) <= std::abs(next_age_sec) ?
        previous->timestamp_sec : next->timestamp_sec;
    } else {
      const TimedTilt & latest = history_.back();
      prediction_horizon_sec = timestamp_sec - latest.timestamp_sec;
      if (
        !std::isfinite(prediction_horizon_sec) ||
        prediction_horizon_sec < 0.0 ||
        prediction_horizon_sec > config_.maximum_frame_imu_prediction_sec)
      {
        return std::nullopt;
      }
      current_up_camera = normalized(rotateVector(
        conjugate(quaternionFromRotationVector(
          latest.angular_velocity_camera_radps * prediction_horizon_sec)),
        latest.up_camera));
      nearest_timestamp = latest.timestamp_sec;
      predicted = true;
    }

    Quaternion correction = quaternionFromTwoVectors(
      current_up_camera, reference_up_camera_);
    if (
      !config_.pitch_correction_enabled ||
      !config_.roll_correction_enabled)
    {
      cv::Vec3d enabled_rotation = rotationVector(correction);
      if (!config_.pitch_correction_enabled) {
        enabled_rotation[0] = 0.0;
      }
      if (!config_.roll_correction_enabled) {
        enabled_rotation[2] = 0.0;
      }
      enabled_rotation[1] = 0.0;
      correction = quaternionFromRotationVector(enabled_rotation);
    }

    const double roll_error_deg = wrapDegrees(
      rollDegrees(current_up_camera) - rollDegrees(reference_up_camera_));
    const double pitch_error_deg = wrapDegrees(
      pitchDegrees(current_up_camera) - pitchDegrees(reference_up_camera_));
    const cv::Vec3d applied_rotation =
      rotationVector(correction) * config_.gyroscope_correction_gain;
    correction = quaternionFromRotationVector(applied_rotation);
    const double applied_roll_error_deg =
      roll_error_deg * config_.gyroscope_correction_gain;
    const double applied_pitch_error_deg =
      pitch_error_deg * config_.gyroscope_correction_gain;
    const double correction_angle_deg =
      cv::norm(applied_rotation) * kRadiansToDegrees;
    const bool within_limit =
      (!config_.roll_correction_enabled ||
      std::abs(applied_roll_error_deg) <= config_.maximum_correction_deg) &&
      (!config_.pitch_correction_enabled ||
      std::abs(applied_pitch_error_deg) <= config_.maximum_correction_deg);
    return ImageStabilizationCorrection{
      quaternionRotationMatrix(correction),
      timestamp_sec,
      nearest_timestamp,
      nearest_timestamp - timestamp_sec,
      applied_roll_error_deg,
      applied_pitch_error_deg,
      correction_angle_deg,
      within_limit,
      predicted,
      prediction_horizon_sec};
  }

  bool initialized() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return initialized_;
  }

  ImageStabilizerCalibrationProgress calibrationProgress() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double discard_elapsed = startup_discard_completed_ ?
      config_.startup_discard_duration_sec :
      (startup_discard_started_ ? std::clamp(
        startup_discard_last_timestamp_sec_ - startup_discard_started_at_sec_,
        0.0, config_.startup_discard_duration_sec) : 0.0);
    const double calibration_elapsed = calibration_count_ == 0U ? 0.0 :
      std::max(
      0.0, calibration_last_timestamp_sec_ - calibration_started_at_sec_);
    return ImageStabilizerCalibrationProgress{
      discard_elapsed,
      config_.startup_discard_duration_sec,
      !startup_discard_completed_,
      calibration_elapsed,
      config_.reference_calibration_duration_sec,
      calibration_count_,
      calibration_reset_count_,
      calibration_last_accelerometer_confidence_,
      calibration_last_angular_speed_degps_,
      calibration_last_rejection_reason_};
  }

  cv::Vec3d gyroscopeBiasRadps() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return gyroscope_bias_radps_;
  }

  cv::Vec2d movingAccelerometerNudgeDegrees() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return moving_accelerometer_nudge_deg_;
  }

  std::uint64_t movingGravityAnchorUpdateCount() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return moving_gravity_anchor_update_count_;
  }

  std::optional<cv::Vec3d> referenceUpCamera() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
      return std::nullopt;
    }
    return reference_up_camera_;
  }

  bool stationaryConfirmed() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return stationary_confirmed_;
  }

  bool externalReferenceReceived() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return external_reference_up_camera_.has_value();
  }

  std::uint64_t onlineTiltBiasUpdateCount() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return online_tilt_bias_update_count_;
  }

private:
  struct TimedTilt
  {
    double timestamp_sec;
    cv::Vec3d up_camera;
    cv::Vec3d angular_velocity_camera_radps;
  };

  struct StationarySample
  {
    double timestamp_sec;
    cv::Vec3d acceleration_camera_mps2;
    cv::Vec3d angular_velocity_camera_radps;
  };

  struct StationaryWindowAssessment
  {
    bool confirmed{false};
    cv::Vec3d mean_gyroscope_radps{0.0, 0.0, 0.0};
  };

  std::string calibrationWindowRejectionReasonLocked() const
  {
    if (calibration_samples_.size() < 4U) {
      return "insufficient synchronized accel/gyro samples";
    }

    const double count = static_cast<double>(calibration_samples_.size());
    const std::size_t half = calibration_samples_.size() / 2U;
    cv::Vec3d gyroscope_sum(0.0, 0.0, 0.0);
    cv::Vec3d first_half_acceleration_sum(0.0, 0.0, 0.0);
    cv::Vec3d second_half_acceleration_sum(0.0, 0.0, 0.0);
    double acceleration_norm_sum = 0.0;
    double acceleration_norm_squared_sum = 0.0;
    for (std::size_t index = 0U; index < calibration_samples_.size(); ++index) {
      const StationarySample & sample = calibration_samples_[index];
      const double acceleration_norm = cv::norm(
        sample.acceleration_camera_mps2);
      acceleration_norm_sum += acceleration_norm;
      acceleration_norm_squared_sum += acceleration_norm * acceleration_norm;
      gyroscope_sum += sample.angular_velocity_camera_radps;
      if (index < half) {
        first_half_acceleration_sum += sample.acceleration_camera_mps2;
      } else {
        second_half_acceleration_sum += sample.acceleration_camera_mps2;
      }
    }

    const cv::Vec3d mean_gyroscope = gyroscope_sum / count;
    if (
      cv::norm(mean_gyroscope) * kRadiansToDegrees >
      config_.calibration_maximum_angular_speed_degps)
    {
      return "mean gyro indicates camera motion";
    }

    cv::Vec3d gyroscope_squared_error_sum(0.0, 0.0, 0.0);
    for (const StationarySample & sample : calibration_samples_) {
      const cv::Vec3d error =
        sample.angular_velocity_camera_radps - mean_gyroscope;
      for (int axis = 0; axis < 3; ++axis) {
        gyroscope_squared_error_sum[axis] += error[axis] * error[axis];
      }
    }
    for (int axis = 0; axis < 3; ++axis) {
      if (
        std::sqrt(gyroscope_squared_error_sum[axis] / count) *
        kRadiansToDegrees >
        config_.stationary_gyroscope_stddev_maximum_degps)
      {
        return "gyro variation indicates vibration or camera motion";
      }
    }

    const double mean_acceleration_norm = acceleration_norm_sum / count;
    const double acceleration_norm_variance = std::max(
      0.0, acceleration_norm_squared_sum / count -
      mean_acceleration_norm * mean_acceleration_norm);
    if (
      std::sqrt(acceleration_norm_variance) >
      config_.stationary_accelerometer_norm_stddev_mps2)
    {
      return "acceleration variation indicates vibration or camera motion";
    }
    if (
      cv::norm(first_half_acceleration_sum) <= 1.0e-9 ||
      cv::norm(second_half_acceleration_sum) <= 1.0e-9 ||
      angleDegrees(
        normalized(first_half_acceleration_sum),
        normalized(second_half_acceleration_sum)) >
      config_.stationary_accelerometer_direction_change_deg)
    {
      return "gravity direction changed during calibration";
    }
    return {};
  }

  StationaryWindowAssessment assessStationaryWindowLocked() const
  {
    StationaryWindowAssessment result;
    if (
      stationary_samples_.size() < 8U ||
      stationary_samples_.back().timestamp_sec -
      stationary_samples_.front().timestamp_sec + 1.0e-9 <
      config_.stationary_detection_window_sec)
    {
      return result;
    }

    cv::Vec3d acceleration_sum(0.0, 0.0, 0.0);
    cv::Vec3d first_half_acceleration_sum(0.0, 0.0, 0.0);
    cv::Vec3d second_half_acceleration_sum(0.0, 0.0, 0.0);
    cv::Vec3d gyroscope_sum(0.0, 0.0, 0.0);
    double acceleration_norm_sum = 0.0;
    double acceleration_norm_squared_sum = 0.0;
    const std::size_t half = stationary_samples_.size() / 2U;
    for (std::size_t index = 0U; index < stationary_samples_.size(); ++index) {
      const StationarySample & sample = stationary_samples_[index];
      const double acceleration_norm = cv::norm(
        sample.acceleration_camera_mps2);
      acceleration_sum += sample.acceleration_camera_mps2;
      acceleration_norm_sum += acceleration_norm;
      acceleration_norm_squared_sum += acceleration_norm * acceleration_norm;
      gyroscope_sum += sample.angular_velocity_camera_radps;
      if (index < half) {
        first_half_acceleration_sum += sample.acceleration_camera_mps2;
      } else {
        second_half_acceleration_sum += sample.acceleration_camera_mps2;
      }
    }

    const double count = static_cast<double>(stationary_samples_.size());
    const double mean_acceleration_norm = acceleration_norm_sum / count;
    const double acceleration_norm_variance = std::max(
      0.0, acceleration_norm_squared_sum / count -
      mean_acceleration_norm * mean_acceleration_norm);
    if (
      std::abs(mean_acceleration_norm - gravity_magnitude_reference_mps2_) >
      config_.stationary_accelerometer_norm_tolerance_mps2 ||
      std::sqrt(acceleration_norm_variance) >
      config_.stationary_accelerometer_norm_stddev_mps2 ||
      cv::norm(acceleration_sum) <= 1.0e-9 ||
      cv::norm(first_half_acceleration_sum) <= 1.0e-9 ||
      cv::norm(second_half_acceleration_sum) <= 1.0e-9)
    {
      return result;
    }

    const cv::Vec3d mean_acceleration_direction = normalized(acceleration_sum);
    if (
      // Stationarity is judged against the immutable startup pose, not the
      // possibly drifted estimator state. This lets a stopped vehicle recover
      // from a large tilt error while still rejecting constant lateral or
      // longitudinal acceleration as motion.
      angleDegrees(
        mean_acceleration_direction,
        stationary_acceleration_reference_up_camera_) >
      config_.stationary_accelerometer_direction_error_deg ||
      angleDegrees(
        normalized(first_half_acceleration_sum),
        normalized(second_half_acceleration_sum)) >
      config_.stationary_accelerometer_direction_change_deg)
    {
      return result;
    }

    result.mean_gyroscope_radps = gyroscope_sum / count;
    const cv::Vec3d mean_residual =
      result.mean_gyroscope_radps - gyroscope_bias_radps_;
    if (
      cv::norm(mean_residual) * kRadiansToDegrees >
      config_.stationary_gyroscope_mean_maximum_degps)
    {
      return result;
    }

    cv::Vec3d gyroscope_squared_error_sum(0.0, 0.0, 0.0);
    for (const StationarySample & sample : stationary_samples_) {
      const cv::Vec3d error =
        sample.angular_velocity_camera_radps - result.mean_gyroscope_radps;
      for (int axis = 0; axis < 3; ++axis) {
        gyroscope_squared_error_sum[axis] += error[axis] * error[axis];
      }
    }
    for (int axis = 0; axis < 3; ++axis) {
      if (
        std::sqrt(gyroscope_squared_error_sum[axis] / count) *
        kRadiansToDegrees >
        config_.stationary_gyroscope_stddev_maximum_degps)
      {
        return result;
      }
    }

    result.confirmed = true;
    return result;
  }

  double accelerometerConfidence(const double acceleration_magnitude) const
  {
    if (!std::isfinite(acceleration_magnitude)) {
      return 0.0;
    }
    const double deviation = std::abs(
      acceleration_magnitude - config_.gravity_mps2);
    if (deviation <= config_.accelerometer_full_trust_deviation_mps2) {
      return 1.0;
    }
    if (deviation >= config_.accelerometer_zero_trust_deviation_mps2) {
      return 0.0;
    }
    return 1.0 -
      (deviation - config_.accelerometer_full_trust_deviation_mps2) /
      (config_.accelerometer_zero_trust_deviation_mps2 -
      config_.accelerometer_full_trust_deviation_mps2);
  }

  void resetCalibrationLocked(const std::string & reason)
  {
    if (calibration_count_ > 0U) {
      ++calibration_reset_count_;
    }
    calibration_last_rejection_reason_ = reason;
    calibration_count_ = 0U;
    calibration_started_at_sec_ = 0.0;
    calibration_last_timestamp_sec_ = 0.0;
    warmup_acceleration_sum_ = cv::Vec3d(0.0, 0.0, 0.0);
    warmup_gyroscope_sum_ = cv::Vec3d(0.0, 0.0, 0.0);
    warmup_gyroscope_squared_sum_ = cv::Vec3d(0.0, 0.0, 0.0);
    calibration_samples_.clear();
  }

  ImuImageStabilizerConfig config_;
  mutable std::mutex mutex_;
  mutable std::condition_variable history_condition_;
  std::size_t calibration_count_{0U};
  std::size_t calibration_reset_count_{0U};
  bool startup_discard_started_{false};
  bool startup_discard_completed_{false};
  double startup_discard_started_at_sec_{0.0};
  double startup_discard_last_timestamp_sec_{0.0};
  double calibration_started_at_sec_{0.0};
  double calibration_last_timestamp_sec_{0.0};
  double calibration_last_accelerometer_confidence_{0.0};
  double calibration_last_angular_speed_degps_{0.0};
  std::string calibration_last_rejection_reason_;
  cv::Vec3d warmup_acceleration_sum_{0.0, 0.0, 0.0};
  cv::Vec3d warmup_gyroscope_sum_{0.0, 0.0, 0.0};
  cv::Vec3d warmup_gyroscope_squared_sum_{0.0, 0.0, 0.0};
  cv::Vec3d gyroscope_bias_radps_{0.0, 0.0, 0.0};
  double gravity_magnitude_reference_mps2_{9.80665};
  std::uint64_t online_tilt_bias_update_count_{0U};
  std::uint64_t moving_gravity_anchor_update_count_{0U};
  bool stationary_confirmed_{false};
  std::optional<cv::Vec3d> external_reference_up_camera_;
  cv::Vec3d stationary_acceleration_reference_up_camera_{
    0.0, -1.0, 0.0};
  Quaternion accelerometer_to_reference_rotation_{};
  cv::Vec3d reference_up_camera_{0.0, -1.0, 0.0};
  cv::Vec3d current_up_camera_{0.0, -1.0, 0.0};
  cv::Vec2d moving_accelerometer_nudge_deg_{0.0, 0.0};
  double last_timestamp_sec_{0.0};
  bool initialized_{false};
  std::deque<StationarySample> calibration_samples_;
  std::deque<StationarySample> stationary_samples_;
  std::deque<TimedTilt> history_;
};

ImuImageStabilizer::ImuImageStabilizer(
  const ImuImageStabilizerConfig & config)
: impl_(std::make_unique<Impl>(config))
{
}

ImuImageStabilizer::~ImuImageStabilizer() = default;

void ImuImageStabilizer::update(
  const std::optional<cv::Vec3d> & acceleration_camera_mps2,
  const cv::Vec3d & angular_velocity_camera_radps,
  const double timestamp_sec,
  const std::optional<bool> vehicle_stationary)
{
  impl_->update(
    acceleration_camera_mps2, angular_velocity_camera_radps, timestamp_sec,
    vehicle_stationary);
}

bool ImuImageStabilizer::setExternalReferenceUpCamera(
  const cv::Vec3d & up_camera)
{
  return impl_->setExternalReferenceUpCamera(up_camera);
}

std::optional<ImageStabilizationCorrection>
ImuImageStabilizer::correctionAt(const double timestamp_sec) const
{
  return impl_->correctionAt(timestamp_sec);
}

std::optional<ImageStabilizationCorrection>
ImuImageStabilizer::correctionAt(
  const double timestamp_sec, const double maximum_frame_imu_age_sec,
  const double maximum_wait_sec) const
{
  return impl_->correctionAt(timestamp_sec,maximum_frame_imu_age_sec,maximum_wait_sec);
}

bool ImuImageStabilizer::initialized() const
{
  return impl_->initialized();
}

ImageStabilizerCalibrationProgress
ImuImageStabilizer::calibrationProgress() const
{
  return impl_->calibrationProgress();
}

cv::Vec3d ImuImageStabilizer::gyroscopeBiasRadps() const
{
  return impl_->gyroscopeBiasRadps();
}

cv::Vec2d ImuImageStabilizer::movingAccelerometerNudgeDegrees() const
{
  return impl_->movingAccelerometerNudgeDegrees();
}

std::uint64_t ImuImageStabilizer::movingGravityAnchorUpdateCount() const
{
  return impl_->movingGravityAnchorUpdateCount();
}

std::optional<cv::Vec3d> ImuImageStabilizer::referenceUpCamera() const
{
  return impl_->referenceUpCamera();
}

bool ImuImageStabilizer::stationaryConfirmed() const
{
  return impl_->stationaryConfirmed();
}

bool ImuImageStabilizer::externalReferenceReceived() const
{
  return impl_->externalReferenceReceived();
}

std::uint64_t ImuImageStabilizer::onlineTiltBiasUpdateCount() const
{
  return impl_->onlineTiltBiasUpdateCount();
}

LastValidStabilizationHomography::LastValidStabilizationHomography(
  const std::size_t maximum_hold_frames)
: maximum_hold_frames_(maximum_hold_frames)
{
}

void LastValidStabilizationHomography::setMaximumHoldFrames(
  const std::size_t maximum_hold_frames)
{
  std::lock_guard<std::mutex> lock(mutex_);
  maximum_hold_frames_ = maximum_hold_frames;
  homography_.reset();
  last_reused_frame_token_.reset();
  remaining_hold_frames_ = 0U;
}

void LastValidStabilizationHomography::remember(
  const cv::Matx33d & homography)
{
  if (!validHomography(homography)) {
    throw std::invalid_argument("cannot remember an invalid homography");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  homography_ = homography;
  last_reused_frame_token_.reset();
  remaining_hold_frames_ = maximum_hold_frames_;
}

std::optional<cv::Matx33d>
LastValidStabilizationHomography::reuseForFrame(
  const std::uint64_t frame_token)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!homography_.has_value()) {
    return std::nullopt;
  }
  if (
    last_reused_frame_token_.has_value() &&
    *last_reused_frame_token_ == frame_token)
  {
    return homography_;
  }
  if (remaining_hold_frames_ == 0U) {
    return std::nullopt;
  }
  --remaining_hold_frames_;
  last_reused_frame_token_ = frame_token;
  return homography_;
}

void LastValidStabilizationHomography::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  homography_.reset();
  last_reused_frame_token_.reset();
  remaining_hold_frames_ = 0U;
}

std::size_t LastValidStabilizationHomography::remainingHoldFrames() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return remaining_hold_frames_;
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
    !cv::checkRange(cv::Mat(correction.camera_to_reference_tilt)))
  {
    throw std::invalid_argument(
            "invalid camera intrinsics or stabilization correction");
  }

  const cv::Matx33d camera_matrix(
    fx, 0.0, cx,
    0.0, fy, cy,
    0.0, 0.0, 1.0);
  return
    camera_matrix *
    correction.camera_to_reference_tilt *
    camera_matrix.inv();
}

cv::Matx33d makeFixedViewZoomHomography(
  const cv::Matx33d & camera_matrix,
  const double zoom)
{
  if (
    !cv::checkRange(cv::Mat(camera_matrix)) ||
    !std::isfinite(zoom) || zoom < 1.0)
  {
    throw std::invalid_argument("invalid fixed-view zoom geometry");
  }
  const double cx = camera_matrix(0, 2);
  const double cy = camera_matrix(1, 2);
  return cv::Matx33d(
    zoom, 0.0, (1.0 - zoom) * cx,
    0.0, zoom, (1.0 - zoom) * cy,
    0.0, 0.0, 1.0);
}

bool outputIsCoveredBySource(
  const cv::Matx33d & source_to_output,
  const cv::Size & source_size,
  const cv::Size & output_size,
  const double source_border_margin_px)
{
  if (
    !validHomography(source_to_output) ||
    source_size.width <= 1 || source_size.height <= 1 ||
    output_size.width <= 1 || output_size.height <= 1 ||
    !std::isfinite(source_border_margin_px) ||
    source_border_margin_px < 0.0)
  {
    return false;
  }

  const cv::Matx33d output_to_source = source_to_output.inv();
  const std::array<cv::Vec3d, 4> output_corners{
    cv::Vec3d(0.0, 0.0, 1.0),
    cv::Vec3d(static_cast<double>(output_size.width - 1), 0.0, 1.0),
    cv::Vec3d(0.0, static_cast<double>(output_size.height - 1), 1.0),
    cv::Vec3d(
      static_cast<double>(output_size.width - 1),
      static_cast<double>(output_size.height - 1), 1.0)};
  const double maximum_x =
    static_cast<double>(source_size.width - 1) - source_border_margin_px;
  const double maximum_y =
    static_cast<double>(source_size.height - 1) - source_border_margin_px;
  for (const auto & corner : output_corners) {
    cv::Vec3d source = output_to_source * corner;
    if (!finiteVector(source) || source[2] <= 1.0e-9) {
      return false;
    }
    source *= 1.0 / source[2];
    if (
      source[0] < source_border_margin_px || source[0] > maximum_x ||
      source[1] < source_border_margin_px || source[1] > maximum_y)
    {
      return false;
    }
  }
  return true;
}

}  // namespace camera_driver
