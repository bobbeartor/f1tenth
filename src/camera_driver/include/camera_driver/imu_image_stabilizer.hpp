#ifndef CAMERA_DRIVER__IMU_IMAGE_STABILIZER_HPP_
#define CAMERA_DRIVER__IMU_IMAGE_STABILIZER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <opencv2/core.hpp>

namespace camera_driver
{

struct ImuImageStabilizerConfig
{
  double startup_discard_duration_sec{1.0};
  double reference_calibration_duration_sec{4.0};
  double calibration_maximum_angular_speed_degps{0.5};
  bool gyroscope_bias_enabled{true};
  double gravity_mps2{9.80665};
  double accelerometer_full_trust_deviation_mps2{0.15};
  double accelerometer_zero_trust_deviation_mps2{1.50};
  double acceleration_correction_time_constant_sec{1.5};
  double acceleration_correction_gate_deg{4.3};
  double roll_acceleration_correction_time_constant_sec{2.0};
  double roll_acceleration_direction_gate_deg{4.3};
  bool acceleration_correction_stationary_only{true};
  bool moving_accelerometer_nudge_enabled{false};
  double moving_accelerometer_nudge_time_constant_sec{0.15};
  double moving_accelerometer_nudge_strength{0.15};
  double moving_accelerometer_pitch_nudge_maximum_deg{0.20};
  double moving_accelerometer_roll_nudge_maximum_deg{0.15};
  double moving_gravity_anchor_maximum_correction_rate_degps{0.50};
  bool external_reference_required{false};
  double reference_tilt_leak_time_constant_sec{4.0};
  double stationary_tilt_recovery_time_constant_sec{0.35};
  bool online_gyroscope_tilt_bias_enabled{true};
  double online_gyroscope_tilt_bias_time_constant_sec{10.0};
  double stationary_detection_window_sec{1.0};
  double stationary_accelerometer_norm_tolerance_mps2{0.20};
  double stationary_accelerometer_norm_stddev_mps2{0.08};
  double stationary_accelerometer_direction_error_deg{1.5};
  double stationary_accelerometer_direction_change_deg{0.15};
  double stationary_gyroscope_mean_maximum_degps{0.5};
  double stationary_gyroscope_stddev_maximum_degps{0.8};
  bool pitch_correction_enabled{true};
  bool roll_correction_enabled{true};
  double gyroscope_correction_gain{1.0};
  double maximum_correction_deg{3.0};
  double maximum_sample_interval_sec{0.1};
  double maximum_history_sec{2.0};
  double maximum_frame_imu_wait_sec{0.001};
  double maximum_frame_imu_age_sec{0.006};
  double maximum_frame_imu_prediction_sec{0.015};
};

struct ImageStabilizationCorrection
{
  cv::Matx33d camera_to_reference_tilt{cv::Matx33d::eye()};
  double frame_timestamp_sec{0.0};
  double nearest_imu_timestamp_sec{0.0};
  double nearest_imu_delta_sec{0.0};
  double roll_error_deg{0.0};
  double pitch_error_deg{0.0};
  double correction_angle_deg{0.0};
  bool within_correction_limit{true};
  bool predicted{false};
  double prediction_horizon_sec{0.0};
};

struct ImageStabilizerCalibrationProgress
{
  double discard_elapsed_sec{0.0};
  double discard_target_sec{0.0};
  bool discarding_startup_samples{false};
  double calibration_elapsed_sec{0.0};
  double calibration_target_sec{0.0};
  std::size_t accepted_samples{0U};
  std::size_t reset_count{0U};
  double last_accelerometer_confidence{0.0};
  double last_angular_speed_degps{0.0};
  std::string last_rejection_reason;
};

class ImuImageStabilizer
{
public:
  explicit ImuImageStabilizer(
    const ImuImageStabilizerConfig & config = {});
  ~ImuImageStabilizer();

  ImuImageStabilizer(const ImuImageStabilizer &) = delete;
  ImuImageStabilizer & operator=(const ImuImageStabilizer &) = delete;

  void update(
    const std::optional<cv::Vec3d> & acceleration_camera_mps2,
    const cv::Vec3d & angular_velocity_camera_radps,
    double timestamp_sec,
    std::optional<bool> vehicle_stationary = std::nullopt);

  // Supplies the immutable ground-plane normal used by the static BEV LUT.
  // It must be expressed in camera optical coordinates and is accepted only
  // before the startup stationary calibration has completed.
  bool setExternalReferenceUpCamera(const cv::Vec3d & up_camera);

  std::optional<ImageStabilizationCorrection> correctionAt(
    double timestamp_sec) const;

  // Per-consumer lookup bounds; leaves the RGB estimator configuration intact.
  // max_age bounds BOTH bracketing sample distances; wait may be zero for polling.
  std::optional<ImageStabilizationCorrection> correctionAt(
    double timestamp_sec, double maximum_frame_imu_age_sec, double maximum_wait_sec) const;

  bool initialized() const;
  ImageStabilizerCalibrationProgress calibrationProgress() const;
  cv::Vec3d gyroscopeBiasRadps() const;
  cv::Vec2d movingAccelerometerNudgeDegrees() const;
  std::uint64_t movingGravityAnchorUpdateCount() const;
  std::optional<cv::Vec3d> referenceUpCamera() const;
  bool stationaryConfirmed() const;
  bool externalReferenceReceived() const;
  std::uint64_t onlineTiltBiasUpdateCount() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

class LastValidStabilizationHomography
{
public:
  explicit LastValidStabilizationHomography(
    std::size_t maximum_hold_frames = 2U);

  void setMaximumHoldFrames(std::size_t maximum_hold_frames);
  void remember(const cv::Matx33d & homography);
  std::optional<cv::Matx33d> reuseForFrame(std::uint64_t frame_token);
  void clear();
  std::size_t remainingHoldFrames() const;

private:
  mutable std::mutex mutex_;
  std::size_t maximum_hold_frames_{2U};
  std::size_t remaining_hold_frames_{0U};
  std::optional<cv::Matx33d> homography_;
  std::optional<std::uint64_t> last_reused_frame_token_;
};

cv::Matx33d makeImageStabilizationHomography(
  double fx,
  double fy,
  double cx,
  double cy,
  const ImageStabilizationCorrection & correction);

cv::Matx33d makeFixedViewZoomHomography(
  const cv::Matx33d & camera_matrix,
  double zoom);

bool outputIsCoveredBySource(
  const cv::Matx33d & source_to_output,
  const cv::Size & source_size,
  const cv::Size & output_size,
  double source_border_margin_px);

}  // namespace camera_driver

#endif  // CAMERA_DRIVER__IMU_IMAGE_STABILIZER_HPP_
