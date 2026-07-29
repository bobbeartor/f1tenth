#ifndef CAMERA_DRIVER__IMU_IMAGE_STABILIZER_HPP_
#define CAMERA_DRIVER__IMU_IMAGE_STABILIZER_HPP_

#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>

#include <opencv2/core.hpp>

namespace camera_driver
{

struct ImuImageStabilizerConfig
{
  std::size_t warmup_samples{200U};
  double minimum_acceleration_mps2{7.5};
  double maximum_acceleration_mps2{12.0};
  double acceleration_correction_time_constant_sec{1.5};
  double acceleration_correction_gate_deg{8.0};
  double trajectory_smoothing_time_constant_sec{0.25};
  double maximum_correction_deg{4.0};
  double maximum_sample_interval_sec{0.1};
  double maximum_history_sec{1.0};
  double maximum_frame_imu_age_sec{0.05};
};

struct ImageStabilizationCorrection
{
  double roll_deg{0.0};
  double pitch_deg{0.0};
};

class ImuImageStabilizer
{
public:
  explicit ImuImageStabilizer(
    const ImuImageStabilizerConfig & config = {});

  void update(
    const cv::Vec3d & acceleration_camera_mps2,
    const cv::Vec3d & angular_velocity_camera_radps,
    double timestamp_sec);

  std::optional<ImageStabilizationCorrection> correctionAt(
    double timestamp_sec) const;

  bool initialized() const;
  void reset();

private:
  struct TimedCorrection
  {
    double timestamp_sec;
    ImageStabilizationCorrection correction;
  };

  ImuImageStabilizerConfig config_;
  mutable std::mutex mutex_;
  std::size_t warmup_count_{0U};
  cv::Vec3d warmup_acceleration_sum_{0.0, 0.0, 0.0};
  cv::Vec3d warmup_gyroscope_sum_{0.0, 0.0, 0.0};
  cv::Vec3d gyroscope_bias_radps_{0.0, 0.0, 0.0};
  cv::Vec3d up_camera_{0.0, -1.0, 0.0};
  double smoothed_roll_deg_{0.0};
  double smoothed_pitch_deg_{0.0};
  double last_timestamp_sec_{0.0};
  bool initialized_{false};
  std::deque<TimedCorrection> history_;
};

cv::Matx33d makeImageStabilizationHomography(
  double fx,
  double fy,
  double cx,
  double cy,
  const ImageStabilizationCorrection & correction);

}  // namespace camera_driver

#endif  // CAMERA_DRIVER__IMU_IMAGE_STABILIZER_HPP_
