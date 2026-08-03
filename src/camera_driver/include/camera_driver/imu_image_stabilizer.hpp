#ifndef CAMERA_DRIVER__IMU_IMAGE_STABILIZER_HPP_
#define CAMERA_DRIVER__IMU_IMAGE_STABILIZER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
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
  double acceleration_correction_time_constant_sec{4.3};
  double acceleration_correction_gate_deg{4.3};
  double roll_acceleration_correction_time_constant_sec{6.0};
  double roll_acceleration_direction_gate_deg{4.3};
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
  double maximum_correction_deg{12.0};
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
    double timestamp_sec);

  std::optional<ImageStabilizationCorrection> correctionAt(
    double timestamp_sec) const;

  bool initialized() const;
  ImageStabilizerCalibrationProgress calibrationProgress() const;
  cv::Vec3d gyroscopeBiasRadps() const;
  bool stationaryConfirmed() const;
  std::uint64_t onlineTiltBiasUpdateCount() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
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
