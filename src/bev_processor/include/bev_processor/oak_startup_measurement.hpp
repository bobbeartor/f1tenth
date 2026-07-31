#ifndef BEV_PROCESSOR__OAK_STARTUP_MEASUREMENT_HPP_
#define BEV_PROCESSOR__OAK_STARTUP_MEASUREMENT_HPP_

#include <cstddef>
#include <string>

#include "bev_processor/startup_attitude.hpp"

namespace bev_processor
{

struct OakStartupMeasurementConfig
{
  double stereo_fps{30.0};
  int stereo_width{640};
  int stereo_height{400};
  int depth_queue_size{2};
  double imu_rate_hz{400.0};
  int imu_queue_size{200};
  double warmup_sec{1.0};
  double ir_dot_projector_intensity{1.0};

  int roi_width{228};
  int roi_height{114};
  int point_sample_step{2};
  int minimum_valid_points{1270};
  double minimum_depth_m{0.30};
  double maximum_depth_m{3.00};
  double minimum_height_m{0.10};
  double maximum_height_m{1.00};

  int plane_ransac_iterations{200};
  double plane_inlier_threshold_m{0.008};
  int plane_minimum_inliers{914};
  double plane_minimum_inlier_ratio{0.70};
  double plane_maximum_residual_mad_m{0.005};
  double plane_maximum_imu_difference_deg{15.0};

  StartupAttitudeSource attitude_source{StartupAttitudeSource::kDepth};
  double imu_roll_bias_deg{0.0};
  double imu_pitch_bias_deg{0.0};

  int imu_sample_count{800};
  double imu_max_direction_rms_deg{0.25};
  double imu_accel_min_mps2{7.50};
  double imu_accel_max_mps2{12.00};

  int stable_plane_frame_count{30};
  double maximum_height_stddev_m{0.004};
  double maximum_plane_normal_rms_deg{0.25};
  double timeout_sec{30.0};
};

struct OakStartupMeasurement
{
  double height_m{0.0};
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
  const OakStartupMeasurementConfig & config);

}  // namespace bev_processor

#endif  // BEV_PROCESSOR__OAK_STARTUP_MEASUREMENT_HPP_
