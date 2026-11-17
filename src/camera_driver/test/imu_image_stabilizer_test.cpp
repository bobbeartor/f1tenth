#include <cmath>
#include <cstdlib>
#include <iostream>
#include <optional>

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

camera_driver::ImuImageStabilizerConfig fastConfig()
{
  camera_driver::ImuImageStabilizerConfig config;
  config.startup_discard_duration_sec = 0.0;
  config.reference_calibration_duration_sec = 0.01;
  config.stationary_detection_window_sec = 0.01;
  config.maximum_frame_imu_wait_sec = 0.0001;
  config.maximum_frame_imu_age_sec = 0.006;
  config.maximum_frame_imu_prediction_sec = 0.015;
  return config;
}

void calibrate(camera_driver::ImuImageStabilizer & stabilizer)
{
  require(
    !stabilizer.referenceUpCamera().has_value(),
    "reference up was exposed before calibration");
  const cv::Vec3d startup_acceleration(0.0, -9.80665, 0.0);
  for (int index = 0; index <= 4; ++index) {
    stabilizer.update(
      startup_acceleration,
      cv::Vec3d(0.0, 0.0, 0.0),
      0.0025 * static_cast<double>(index));
  }
  require(stabilizer.initialized(), "stationary calibration did not finish");
  const auto reference_up = stabilizer.referenceUpCamera();
  require(
    reference_up.has_value() &&
    cv::norm(*reference_up - cv::Vec3d(0.0, -1.0, 0.0)) < 1.0e-12,
    "IMU fallback reference up is unavailable or incorrect");
}

void calibrateAtTilt(
  camera_driver::ImuImageStabilizer & stabilizer,
  const cv::Vec3d & startup_acceleration)
{
  for (int index = 0; index <= 4; ++index) {
    stabilizer.update(
      startup_acceleration,
      cv::Vec3d(0.0, 0.0, 0.0),
      0.0025 * static_cast<double>(index));
  }
  require(stabilizer.initialized(), "tilted calibration did not finish");
}

void verifyStartupSamplesAreDiscarded()
{
  auto config = fastConfig();
  config.startup_discard_duration_sec = 0.01;
  camera_driver::ImuImageStabilizer stabilizer(config);

  for (int index = 0; index <= 4; ++index) {
    stabilizer.update(
      cv::Vec3d(0.0, -9.80665, 0.0),
      cv::Vec3d(0.0, 0.0, 1.0),
      0.0025 * static_cast<double>(index));
  }
  const auto discarded = stabilizer.calibrationProgress();
  require(
    !discarded.discarding_startup_samples,
    "startup discard did not finish at its configured boundary");
  require(
    discarded.accepted_samples == 0U,
    "a startup-discard gyro sample leaked into calibration");

  for (int index = 1; index <= 6; ++index) {
    stabilizer.update(
      cv::Vec3d(0.0, -9.80665, 0.0),
      cv::Vec3d(0.0, 0.0, 0.0),
      0.01 + 0.0025 * static_cast<double>(index));
  }
  require(stabilizer.initialized(), "post-discard calibration did not finish");
  require(
    cv::norm(stabilizer.gyroscopeBiasRadps()) < 1.0e-12,
    "discarded startup gyro contaminated the learned bias");
}

void verifyLateralAccelerationDoesNotCreateRoll()
{
  const auto config = fastConfig();
  camera_driver::ImuImageStabilizer stabilizer(config);
  calibrate(stabilizer);

  double timestamp_sec = 0.01;
  for (int index = 1; index <= 800; ++index) {
    timestamp_sec = 0.01 + 0.0025 * static_cast<double>(index);
    stabilizer.update(
      cv::Vec3d(1.0, -9.80665, 0.0),
      cv::Vec3d(0.0, 0.0, 0.0),
      timestamp_sec);
  }
  const auto correction = stabilizer.correctionAt(timestamp_sec);
  require(correction.has_value(), "lateral-acceleration lookup failed");
  require(
    std::abs(correction->roll_error_deg) < 1.0e-9,
    "lateral acceleration was incorrectly accumulated as roll");
}

void verifyPitchedUturnDoesNotCreateTilt()
{
  const auto config = fastConfig();
  camera_driver::ImuImageStabilizer stabilizer(config);
  constexpr double pitch_rad = 13.0 * kDegreesToRadians;
  const cv::Vec3d startup_acceleration(
    0.0,
    -9.80665 * std::cos(pitch_rad),
    -9.80665 * std::sin(pitch_rad));
  calibrateAtTilt(stabilizer, startup_acceleration);

  const cv::Vec3d yaw_axis = startup_acceleration / 9.80665;
  double timestamp_sec = 0.01;
  for (int index = 1; index <= 1260; ++index) {
    timestamp_sec = 0.01 + 0.0025 * static_cast<double>(index);
    stabilizer.update(std::nullopt, yaw_axis, timestamp_sec);
  }
  const auto correction = stabilizer.correctionAt(timestamp_sec);
  require(correction.has_value(), "pitched U-turn lookup failed");
  require(
    correction->correction_angle_deg < 1.0e-6,
    "gravity-axis U-turn leaked yaw into tilt");
}

void verifyStationaryRecoveryAndThreeAxisBiasUpdate()
{
  auto config = fastConfig();
  config.stationary_tilt_recovery_time_constant_sec = 0.02;
  config.online_gyroscope_tilt_bias_time_constant_sec = 0.02;
  camera_driver::ImuImageStabilizer stabilizer(config);
  calibrate(stabilizer);

  double timestamp_sec = 0.01;
  for (int index = 1; index <= 40; ++index) {
    timestamp_sec = 0.01 + 0.0025 * static_cast<double>(index);
    stabilizer.update(
      std::nullopt,
      cv::Vec3d(0.0, 0.0, 1.0),
      timestamp_sec);
  }
  const auto drifted = stabilizer.correctionAt(timestamp_sec);
  require(drifted.has_value(), "drifted tilt lookup failed");
  require(
    std::abs(drifted->roll_error_deg) > 3.0,
    "synthetic gyro drift did not create the expected tilt error");

  const cv::Vec3d stationary_gyro_bias(0.0, 0.001, 0.0);
  for (int index = 1; index <= 200; ++index) {
    timestamp_sec += 0.0025;
    stabilizer.update(
      cv::Vec3d(0.0, -9.80665, 0.0),
      stationary_gyro_bias,
      timestamp_sec,
      true);
  }
  const auto recovered = stabilizer.correctionAt(timestamp_sec);
  require(recovered.has_value(), "stationary recovery lookup failed");
  require(
    std::abs(recovered->roll_error_deg) < 0.1,
    "stationary recovery did not restore the startup view");
  require(
    stabilizer.gyroscopeBiasRadps()[1] > 0.0005,
    "stationary online bias update did not include camera Y");
  require(
    stabilizer.onlineTiltBiasUpdateCount() > 0U,
    "stationary online bias update was never activated");
}

void verifyReferenceLeakBoundsGyroDrift()
{
  auto config = fastConfig();
  config.reference_tilt_leak_time_constant_sec = 0.02;
  camera_driver::ImuImageStabilizer stabilizer(config);
  calibrate(stabilizer);

  double timestamp_sec = 0.01;
  for (int index = 1; index <= 800; ++index) {
    timestamp_sec = 0.01 + 0.0025 * static_cast<double>(index);
    stabilizer.update(
      std::nullopt,
      cv::Vec3d(0.0, 0.0, 0.01),
      timestamp_sec);
  }
  const auto correction = stabilizer.correctionAt(timestamp_sec);
  require(correction.has_value(), "reference-leak lookup failed");
  require(
    std::abs(correction->roll_error_deg) < 0.05,
    "startup-reference leak did not bound accumulated gyro drift");
}

void verifyMotionCompensatedGravityCorrectsMovingGyroDrift()
{
  auto config = fastConfig();
  config.acceleration_correction_stationary_only = false;
  config.moving_gravity_anchor_maximum_correction_rate_degps = 10.0;
  config.roll_acceleration_correction_time_constant_sec = 0.02;
  config.roll_acceleration_direction_gate_deg = 10.0;
  config.reference_tilt_leak_time_constant_sec = 1000.0;
  camera_driver::ImuImageStabilizer stabilizer(config);
  calibrate(stabilizer);

  double timestamp_sec = 0.01;
  for (int index = 1; index <= 800; ++index) {
    timestamp_sec += 0.0025;
    stabilizer.update(
      cv::Vec3d(0.0, -9.80665, 0.0),
      cv::Vec3d(0.0, 0.0, 0.01),
      timestamp_sec,
      false);
  }
  const auto correction = stabilizer.correctionAt(timestamp_sec);
  require(correction.has_value(), "moving gravity correction lookup failed");
  require(
    std::abs(correction->roll_error_deg) < 0.1,
    "motion-compensated gravity did not bound moving gyro drift");
}

void verifyMovingAccelerometerStaysAlignedWithExternalBevReference()
{
  auto config = fastConfig();
  config.external_reference_required = true;
  config.acceleration_correction_stationary_only = false;
  config.acceleration_correction_time_constant_sec = 0.02;
  config.roll_acceleration_correction_time_constant_sec = 0.02;
  config.acceleration_correction_gate_deg = 10.0;
  config.roll_acceleration_direction_gate_deg = 10.0;
  config.reference_tilt_leak_time_constant_sec = 1000.0;
  camera_driver::ImuImageStabilizer stabilizer(config);

  constexpr double depth_roll_rad = 1.0 * kDegreesToRadians;
  constexpr double depth_pitch_rad = 3.0 * kDegreesToRadians;
  const cv::Vec3d depth_up_camera(
    -std::sin(depth_roll_rad) * std::cos(depth_pitch_rad),
    -std::cos(depth_roll_rad) * std::cos(depth_pitch_rad),
    -std::sin(depth_pitch_rad));
  require(
    stabilizer.setExternalReferenceUpCamera(depth_up_camera),
    "external BEV reference was rejected for moving alignment test");

  const cv::Vec3d startup_acceleration(0.0, -9.80665, 0.0);
  calibrateAtTilt(stabilizer, startup_acceleration);

  double timestamp_sec = 0.01;
  for (int index = 1; index <= 800; ++index) {
    timestamp_sec += 0.0025;
    stabilizer.update(
      startup_acceleration,
      cv::Vec3d(0.0, 0.0, 0.0),
      timestamp_sec,
      false);
  }
  const auto aligned = stabilizer.correctionAt(timestamp_sec);
  require(aligned.has_value(), "moving BEV alignment lookup failed");
  require(
    aligned->correction_angle_deg < 0.01,
    "moving accelerometer replaced the external depth reference");

  constexpr double relative_roll_rad = 1.0 * kDegreesToRadians;
  const cv::Vec3d changed_acceleration(
    -9.80665 * std::sin(relative_roll_rad),
    -9.80665 * std::cos(relative_roll_rad),
    0.0);
  for (int index = 1; index <= 800; ++index) {
    timestamp_sec += 0.0025;
    stabilizer.update(
      changed_acceleration,
      cv::Vec3d(0.0, 0.0, 0.0),
      timestamp_sec,
      false);
  }
  const auto relative_change = stabilizer.correctionAt(timestamp_sec);
  require(relative_change.has_value(), "relative tilt lookup failed");
  require(
    relative_change->correction_angle_deg > 0.5,
    "external alignment suppressed a real relative accelerometer change");
}

void verifyMovingAccelerometerNudgeIsFastBoundedAndNonAccumulating()
{
  auto config = fastConfig();
  config.moving_accelerometer_nudge_enabled = true;
  config.moving_accelerometer_nudge_time_constant_sec = 0.15;
  config.moving_accelerometer_nudge_strength = 1.0;
  config.moving_accelerometer_roll_nudge_maximum_deg = 0.15;
  config.moving_accelerometer_pitch_nudge_maximum_deg = 0.20;
  config.reference_tilt_leak_time_constant_sec = 1000.0;
  camera_driver::ImuImageStabilizer stabilizer(config);
  calibrate(stabilizer);

  constexpr double measured_roll_rad = 3.0 * kDegreesToRadians;
  const cv::Vec3d tilted_acceleration(
    -9.80665 * std::sin(measured_roll_rad),
    -9.80665 * std::cos(measured_roll_rad),
    0.0);
  double timestamp_sec = 0.01;
  for (int index = 0; index < 60; ++index) {
    timestamp_sec += 0.0025;
    stabilizer.update(
      tilted_acceleration,
      cv::Vec3d(0.0, 0.0, 0.0),
      timestamp_sec,
      false);
  }
  require(
    std::abs(stabilizer.movingAccelerometerNudgeDegrees()[0]) > 0.08,
    "moving accelerometer nudge did not react within one time constant");
  for (int index = 60; index < 400; ++index) {
    timestamp_sec += 0.0025;
    stabilizer.update(
      tilted_acceleration,
      cv::Vec3d(0.0, 0.0, 0.0),
      timestamp_sec,
      false);
  }

  const cv::Vec2d active_nudge =
    stabilizer.movingAccelerometerNudgeDegrees();
  const auto bounded = stabilizer.correctionAt(timestamp_sec);
  require(bounded.has_value(), "bounded nudge lookup failed");
  require(
    std::abs(active_nudge[0]) > 0.13 &&
    std::abs(active_nudge[0]) <= 0.150001,
    "moving accelerometer roll nudge did not reach its configured cap");
  require(
    std::abs(bounded->roll_error_deg) <= 0.151,
    "moving accelerometer nudge accumulated beyond its configured cap");

  for (int index = 0; index < 400; ++index) {
    timestamp_sec += 0.0025;
    stabilizer.update(
      std::nullopt,
      cv::Vec3d(0.0, 0.0, 0.0),
      timestamp_sec,
      false);
  }
  const cv::Vec2d cleared_nudge =
    stabilizer.movingAccelerometerNudgeDegrees();
  const auto cleared = stabilizer.correctionAt(timestamp_sec);
  require(cleared.has_value(), "cleared nudge lookup failed");
  require(
    std::abs(cleared_nudge[0]) < 0.001 &&
    std::abs(cleared->roll_error_deg) < 0.001,
    "moving accelerometer nudge contaminated the persistent gyro attitude");

  constexpr double measured_pitch_rad = 3.0 * kDegreesToRadians;
  const cv::Vec3d pitched_acceleration(
    0.0,
    -9.80665 * std::cos(measured_pitch_rad),
    -9.80665 * std::sin(measured_pitch_rad));
  for (int index = 0; index < 400; ++index) {
    timestamp_sec += 0.0025;
    stabilizer.update(
      pitched_acceleration,
      cv::Vec3d(0.0, 0.0, 0.0),
      timestamp_sec,
      false);
  }
  const cv::Vec2d pitch_nudge =
    stabilizer.movingAccelerometerNudgeDegrees();
  const auto pitch_bounded = stabilizer.correctionAt(timestamp_sec);
  require(pitch_bounded.has_value(), "bounded pitch nudge lookup failed");
  require(
    std::abs(pitch_nudge[1]) > 0.18 &&
    std::abs(pitch_nudge[1]) <= 0.200001 &&
    std::abs(pitch_bounded->pitch_error_deg) <= 0.201,
    "moving accelerometer pitch nudge exceeded its configured cap");
}

void verifyPersistentGravityAnchorAndBoundedNudgeCanRunTogether()
{
  auto config = fastConfig();
  config.acceleration_correction_stationary_only = false;
  config.moving_accelerometer_nudge_enabled = true;
  config.acceleration_correction_time_constant_sec = 0.02;
  config.roll_acceleration_correction_time_constant_sec = 0.02;
  config.acceleration_correction_gate_deg = 10.0;
  config.roll_acceleration_direction_gate_deg = 10.0;
  config.moving_gravity_anchor_maximum_correction_rate_degps = 10.0;
  config.reference_tilt_leak_time_constant_sec = 1000.0;
  camera_driver::ImuImageStabilizer stabilizer(config);
  calibrate(stabilizer);

  double timestamp_sec = 0.01;
  for (int index = 0; index < 800; ++index) {
    timestamp_sec += 0.0025;
    stabilizer.update(
      cv::Vec3d(0.0, -9.80665, 0.0),
      cv::Vec3d(0.0, 0.0, 0.01),
      timestamp_sec,
      false);
  }

  const auto correction = stabilizer.correctionAt(timestamp_sec);
  require(correction.has_value(), "combined anchor/nudge lookup failed");
  require(
    std::abs(correction->roll_error_deg) < 0.1,
    "persistent gravity anchor did not bound gyro drift with nudge enabled");
  require(
    stabilizer.movingGravityAnchorUpdateCount() > 0U,
    "persistent gravity anchor was not activated with bounded nudge");
}

void verifyPersistentGravityAnchorRateIsLimited()
{
  auto config = fastConfig();
  config.acceleration_correction_stationary_only = false;
  config.acceleration_correction_time_constant_sec = 0.001;
  config.roll_acceleration_correction_time_constant_sec = 0.001;
  config.acceleration_correction_gate_deg = 10.0;
  config.roll_acceleration_direction_gate_deg = 10.0;
  config.moving_gravity_anchor_maximum_correction_rate_degps = 0.5;
  config.reference_tilt_leak_time_constant_sec = 1000.0;
  camera_driver::ImuImageStabilizer stabilizer(config);
  calibrate(stabilizer);

  double timestamp_sec = 0.01;
  for (int index = 0; index < 80; ++index) {
    timestamp_sec += 0.0025;
    stabilizer.update(
      std::nullopt,
      cv::Vec3d(0.0, 0.0, 10.0 * kDegreesToRadians),
      timestamp_sec,
      false);
  }
  const auto before = stabilizer.correctionAt(timestamp_sec);
  require(before.has_value(), "pre-anchor drift lookup failed");

  for (int index = 0; index < 400; ++index) {
    timestamp_sec += 0.0025;
    stabilizer.update(
      cv::Vec3d(0.0, -9.80665, 0.0),
      cv::Vec3d(0.0, 0.0, 0.0),
      timestamp_sec,
      false);
  }
  const auto after = stabilizer.correctionAt(timestamp_sec);
  require(after.has_value(), "rate-limited anchor lookup failed");
  const double correction_change_deg =
    std::abs(before->roll_error_deg) - std::abs(after->roll_error_deg);
  require(
    correction_change_deg > 0.40 && correction_change_deg <= 0.51,
    "persistent gravity anchor exceeded its configured correction rate");
}

void verifyLastValidHomographyIsHeldForExactlyTwoMisses()
{
  camera_driver::LastValidStabilizationHomography hold(2U);
  require(
    !hold.reuseForFrame(10U).has_value(),
    "empty hold returned a homography");

  const cv::Matx33d homography(
    1.0, 0.0, 3.0,
    0.0, 1.0, -2.0,
    0.0, 0.0, 1.0);
  hold.remember(homography);
  require(hold.remainingHoldFrames() == 2U, "hold count was not armed");
  const auto first = hold.reuseForFrame(10U);
  const auto same_frame = hold.reuseForFrame(10U);
  const auto second = hold.reuseForFrame(11U);
  require(
    first.has_value() && same_frame.has_value() && second.has_value() &&
    cv::norm(cv::Mat(*first - homography)) < 1.0e-12 &&
    cv::norm(cv::Mat(*same_frame - homography)) < 1.0e-12 &&
    cv::norm(cv::Mat(*second - homography)) < 1.0e-12,
    "last valid homography was not reused unchanged");
  require(
    !hold.reuseForFrame(12U).has_value() &&
    hold.remainingHoldFrames() == 0U,
    "last valid homography was reused beyond two misses");
}

void verifyExternalVehicleStationaryControlsRecovery()
{
  auto config = fastConfig();
  config.stationary_tilt_recovery_time_constant_sec = 0.02;
  camera_driver::ImuImageStabilizer stabilizer(config);
  calibrate(stabilizer);

  double timestamp_sec = 0.01;
  for (int index = 1; index <= 40; ++index) {
    timestamp_sec += 0.0025;
    stabilizer.update(
      std::nullopt,
      cv::Vec3d(0.0, 0.0, 1.0),
      timestamp_sec,
      false);
  }
  const auto drifted = stabilizer.correctionAt(timestamp_sec);
  require(drifted.has_value(), "ERPM-moving tilt lookup failed");
  require(
    std::abs(drifted->roll_error_deg) > 3.0,
    "vehicle moving state unexpectedly activated stationary recovery");

  for (int index = 1; index <= 80; ++index) {
    timestamp_sec += 0.0025;
    stabilizer.update(
      std::nullopt,
      cv::Vec3d(0.0, 0.0, 0.0),
      timestamp_sec,
      true);
  }
  const auto recovered = stabilizer.correctionAt(timestamp_sec);
  require(recovered.has_value(), "ERPM-stationary recovery lookup failed");
  require(
    std::abs(recovered->roll_error_deg) < 0.1,
    "ERPM stationary state did not restore the startup view");
  require(
    stabilizer.stationaryConfirmed(),
    "external vehicle stationary state was not reported");
}

void verifyExternalBevReferenceIsSharedWithErpmStationaryRecovery()
{
  auto config = fastConfig();
  config.external_reference_required = true;
  config.stationary_tilt_recovery_time_constant_sec = 0.02;
  camera_driver::ImuImageStabilizer stabilizer(config);
  const cv::Vec3d startup_acceleration(0.0, -9.80665, 0.0);

  for (int index = 0; index <= 4; ++index) {
    stabilizer.update(
      startup_acceleration,
      cv::Vec3d(0.0, 0.0, 0.0),
      0.0025 * static_cast<double>(index));
  }
  require(
    !stabilizer.initialized(),
    "required BEV reference was silently replaced by the IMU reference");

  constexpr double depth_roll_rad = 5.0 * kDegreesToRadians;
  const cv::Vec3d depth_up_camera(
    -std::sin(depth_roll_rad),
    -std::cos(depth_roll_rad),
    0.0);
  require(
    stabilizer.setExternalReferenceUpCamera(depth_up_camera),
    "valid BEV ground reference was rejected");
  require(
    stabilizer.externalReferenceReceived(),
    "BEV ground reference receipt was not recorded");

  for (int index = 1; index <= 6; ++index) {
    stabilizer.update(
      startup_acceleration,
      cv::Vec3d(0.0, 0.0, 0.0),
      0.01 + 0.0025 * static_cast<double>(index));
  }
  require(
    stabilizer.initialized(),
    "calibration did not finish after the BEV reference arrived");
  const auto shared_reference = stabilizer.referenceUpCamera();
  require(
    shared_reference.has_value() &&
    cv::norm(*shared_reference - depth_up_camera) < 1.0e-12,
    "exposed reference up does not match the BEV reference");

  double timestamp_sec = 0.025;
  const auto startup_correction = stabilizer.correctionAt(timestamp_sec);
  require(
    startup_correction.has_value() &&
    startup_correction->correction_angle_deg < 1.0e-9,
    "shared BEV startup reference produced a non-identity startup warp");

  for (int index = 0; index < 40; ++index) {
    timestamp_sec += 0.0025;
    stabilizer.update(
      std::nullopt, cv::Vec3d(0.0, 0.0, 1.0), timestamp_sec);
  }
  for (int index = 0; index < 200; ++index) {
    timestamp_sec += 0.0025;
    stabilizer.update(
      startup_acceleration,
      cv::Vec3d(0.0, 0.0, 0.0),
      timestamp_sec,
      true);
  }
  const auto recovered = stabilizer.correctionAt(timestamp_sec);
  require(
    stabilizer.stationaryConfirmed(),
    "ERPM stationary state was not accepted with a different depth reference");
  require(
    recovered.has_value() && std::abs(recovered->roll_error_deg) < 0.1,
    "stationary recovery did not return to the shared BEV reference");
}

}  // namespace

int main()
{
  verifyStartupSamplesAreDiscarded();
  verifyLateralAccelerationDoesNotCreateRoll();
  verifyPitchedUturnDoesNotCreateTilt();
  verifyStationaryRecoveryAndThreeAxisBiasUpdate();
  verifyExternalVehicleStationaryControlsRecovery();
  verifyReferenceLeakBoundsGyroDrift();
  verifyMotionCompensatedGravityCorrectsMovingGyroDrift();
  verifyMovingAccelerometerStaysAlignedWithExternalBevReference();
  verifyMovingAccelerometerNudgeIsFastBoundedAndNonAccumulating();
  verifyPersistentGravityAnchorAndBoundedNudgeCanRunTogether();
  verifyPersistentGravityAnchorRateIsLimited();
  verifyLastValidHomographyIsHeldForExactlyTwoMisses();
  verifyExternalBevReferenceIsSharedWithErpmStationaryRecovery();

  const auto config = fastConfig();
  camera_driver::ImuImageStabilizer stabilizer(config);
  calibrate(stabilizer);

  stabilizer.update(
    cv::Vec3d(0.0, -9.80665, 0.0),
    cv::Vec3d(0.0, 0.0, 1.0),
    0.0125);
  const auto roll_correction = stabilizer.correctionAt(0.0125);
  require(roll_correction.has_value(), "roll correction lookup failed");
  require(
    std::abs(roll_correction->roll_error_deg) > 0.1,
    "camera-Z rotation did not move the fixed roll reference");
  require(
    !roll_correction->predicted,
    "exact timestamp was incorrectly marked as predicted");

  const auto predicted = stabilizer.correctionAt(0.0200);
  require(predicted.has_value(), "short gyro prediction failed");
  require(predicted->predicted, "future frame did not use gyro prediction");
  require(
    std::abs(predicted->prediction_horizon_sec - 0.0075) < 1.0e-9,
    "prediction horizon is incorrect");
  require(
    !stabilizer.correctionAt(0.0300).has_value(),
    "prediction exceeded its configured time limit");

  const auto homography = camera_driver::makeImageStabilizationHomography(
    500.0, 500.0, 640.0, 360.0, *roll_correction);
  require(
    cv::checkRange(cv::Mat(homography)),
    "fixed-reference homography is not finite");

  camera_driver::ImuImageStabilizer pitch_stabilizer(config);
  calibrate(pitch_stabilizer);
  pitch_stabilizer.update(
    cv::Vec3d(0.0, -9.80665, 0.0),
    cv::Vec3d(-1.0, 0.0, 0.0),
    0.0125);
  const auto pitch_correction = pitch_stabilizer.correctionAt(0.0125);
  require(pitch_correction.has_value(), "pitch correction lookup failed");
  require(
    pitch_correction->pitch_error_deg > 0.1,
    "negative camera-X gyro must produce positive downward pitch");

  const auto pitch_homography =
    camera_driver::makeImageStabilizationHomography(
    500.0, 500.0, 640.0, 360.0, *pitch_correction);
  const cv::Vec3d current_horizon_ray(
    0.0,
    -std::sin(pitch_correction->pitch_error_deg * kDegreesToRadians),
    std::cos(pitch_correction->pitch_error_deg * kDegreesToRadians));
  const cv::Vec3d current_horizon_pixel(
    500.0 * current_horizon_ray[0] + 640.0 * current_horizon_ray[2],
    500.0 * current_horizon_ray[1] + 360.0 * current_horizon_ray[2],
    current_horizon_ray[2]);
  const cv::Vec3d stabilized_horizon_pixel =
    pitch_homography * current_horizon_pixel;
  require(
    std::abs(
      stabilized_horizon_pixel[1] / stabilized_horizon_pixel[2] - 360.0) <
    1.0e-5,
    "pitch homography did not restore the startup horizon");

  camera_driver::ImuImageStabilizer yaw_stabilizer(config);
  calibrate(yaw_stabilizer);
  yaw_stabilizer.update(
    cv::Vec3d(0.0, -9.80665, 0.0),
    cv::Vec3d(0.0, 1.0, 0.0),
    0.0125);
  const auto yaw_correction = yaw_stabilizer.correctionAt(0.0125);
  require(yaw_correction.has_value(), "yaw attitude lookup failed");
  require(
    yaw_correction->correction_angle_deg < 1.0e-6,
    "gravity-axis yaw must not be stabilized");

  const cv::Matx33d camera_matrix(
    500.0, 0.0, 640.0,
    0.0, 500.0, 360.0,
    0.0, 0.0, 1.0);
  const auto zoom = camera_driver::makeFixedViewZoomHomography(
    camera_matrix, 1.25);
  require(
    camera_driver::outputIsCoveredBySource(
      zoom, cv::Size(1280, 720), cv::Size(1280, 720), 1.5),
    "fixed 1.25x view is not covered by the source image");

  auto moving_config = fastConfig();
  moving_config.calibration_maximum_angular_speed_degps = 0.5;
  camera_driver::ImuImageStabilizer moving_stabilizer(moving_config);
  for (int index = 0; index <= 8; ++index) {
    moving_stabilizer.update(
      cv::Vec3d(0.0, -9.80665, 0.0),
      cv::Vec3d(0.0, 0.0, 0.1),
      0.0025 * static_cast<double>(index));
  }
  require(
    !moving_stabilizer.initialized(),
    "moving camera was accepted as the fixed startup reference");
  require(
    moving_stabilizer.calibrationProgress().reset_count > 0U,
    "moving calibration did not report a reset");

  std::cout << "imu_image_stabilizer_test passed\n";
  return 0;
}
