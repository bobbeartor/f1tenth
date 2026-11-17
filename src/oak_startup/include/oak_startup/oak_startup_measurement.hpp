#ifndef OAK_STARTUP__OAK_STARTUP_MEASUREMENT_HPP_
#define OAK_STARTUP__OAK_STARTUP_MEASUREMENT_HPP_

#include <cstddef>
#include <functional>
#include <string>

#include "oak_startup/startup_attitude.hpp"
namespace cv {class Mat;}

namespace oak_startup
{

struct OakStartupMeasurementConfig
{
  // Optional observer only: does not change measurement or device calibration.
  std::function<void(const cv::Mat &)> preview_callback;
  std::string device_id; // Empty selects the first available OAK.
  double stereo_fps{30.0};
  int stereo_width{1280};
  int stereo_height{800};
  int depth_queue_size{2};
  int stereo_subpixel_fractional_bits{5};
  int stereo_left_right_check_threshold{5};
  int stereo_confidence_threshold{55};
  int stereo_disparity_shift{0};
  double imu_rate_hz{400.0};
  int imu_queue_size{200};
  int imu_max_batch_reports{5};
  double maximum_imu_pair_skew_sec{0.003};
  double warmup_sec{2.0};
  double ir_dot_projector_intensity{1.0};
  bool manual_camera_height_enabled{false};
  double manual_camera_height_m{0.20};

  int roi_width{456};
  int roi_height{228};
  int roi_vertical_offset_px{80};
  bool roi_preview_enabled{true};
  int point_sample_step{2};
  int minimum_valid_points{5080};
  double minimum_depth_m{0.30};
  double maximum_depth_m{3.00};
  double minimum_height_m{0.10};
  double maximum_height_m{0.40};

  int plane_ransac_iterations{200};
  double plane_inlier_threshold_m{0.008};
  int plane_minimum_inliers{3656};
  double plane_minimum_inlier_ratio{0.70};
  double plane_maximum_residual_mad_m{0.005};
  double plane_maximum_imu_difference_deg{5.0};

  StartupAttitudeSource attitude_source{StartupAttitudeSource::kDepth};
  double imu_roll_bias_deg{0.0};
  double imu_pitch_bias_deg{0.0};

  int imu_sample_count{1200};
  double imu_max_direction_rms_deg{0.50};
  double imu_accel_min_mps2{8.30};
  double imu_accel_max_mps2{11.30};
  double imu_gyroscope_mean_maximum_degps{0.80};
  double imu_gyroscope_stddev_maximum_degps{1.40};

  int stable_plane_frame_count{45};
  double maximum_height_stddev_m{0.003};
  double maximum_plane_normal_rms_deg{0.25};
  double timeout_sec{45.0};
};

struct OakStartupMeasurement
{
  std::string device_id;
  double height_m{0.0};
  std::string height_source{"unknown"};
  double roll_deg{0.0};
  double pitch_down_deg{0.0};
  std::string attitude_source{"unknown"};
  double imu_roll_deg{0.0};
  double imu_pitch_down_deg{0.0};
  double corrected_imu_roll_deg{0.0};
  double corrected_imu_pitch_down_deg{0.0};
  double depth_roll_deg{0.0};
  double depth_pitch_down_deg{0.0};
  double imu_direction_rms_deg{0.0};
  double imu_gyroscope_mean_degps{0.0};
  double imu_gyroscope_stddev_degps{0.0};
  double height_stddev_m{0.0};
  double plane_normal_rms_deg{0.0};
  double median_depth_m{0.0};
  double plane_residual_mad_m{0.0};
  double plane_inlier_ratio{0.0};
  double plane_imu_difference_deg{0.0};
  std::size_t valid_point_count{0U};
  std::size_t plane_inlier_count{0U};
};

OakStartupMeasurement measureOakStartupExtrinsics(
  const OakStartupMeasurementConfig & config,
  const std::function<bool()> & should_stop = {});

}  // namespace oak_startup

#endif  // OAK_STARTUP__OAK_STARTUP_MEASUREMENT_HPP_
