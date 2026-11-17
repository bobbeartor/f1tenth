#pragma once

#include <oak_startup/oak_startup_measurement.hpp>
#include <rclcpp/rclcpp.hpp>

namespace point_cloud
{
inline oak_startup::OakStartupMeasurementConfig startupParameters(rclcpp::Node & node)
{
  oak_startup::OakStartupMeasurementConfig config;
  rcl_interfaces::msg::ParameterDescriptor read_only;
  read_only.read_only = true;
  config.stereo_fps = node.declare_parameter<double>(
    "measurement_stereo_fps", config.stereo_fps, read_only);
  config.stereo_width = node.declare_parameter<int>(
    "measurement_stereo_width", config.stereo_width, read_only);
  config.stereo_height = node.declare_parameter<int>(
    "measurement_stereo_height", config.stereo_height, read_only);
  config.depth_queue_size = node.declare_parameter<int>(
    "measurement_depth_queue_size", config.depth_queue_size, read_only);
  config.stereo_subpixel_fractional_bits = node.declare_parameter<int>(
    "measurement_stereo_subpixel_fractional_bits", config.stereo_subpixel_fractional_bits, read_only);
  config.stereo_left_right_check_threshold = node.declare_parameter<int>(
    "measurement_stereo_left_right_check_threshold", config.stereo_left_right_check_threshold, read_only);
  config.stereo_confidence_threshold = node.declare_parameter<int>(
    "measurement_stereo_confidence_threshold", config.stereo_confidence_threshold, read_only);
  config.stereo_disparity_shift = node.declare_parameter<int>(
    "measurement_stereo_disparity_shift", config.stereo_disparity_shift, read_only);
  config.imu_rate_hz = node.declare_parameter<double>(
    "measurement_imu_rate_hz", config.imu_rate_hz, read_only);
  config.imu_queue_size = node.declare_parameter<int>(
    "measurement_imu_queue_size", config.imu_queue_size, read_only);
  config.imu_max_batch_reports = node.declare_parameter<int>(
    "measurement_imu_max_batch_reports", config.imu_max_batch_reports, read_only);
  config.maximum_imu_pair_skew_sec = node.declare_parameter<double>(
    "measurement_maximum_imu_pair_skew_sec", config.maximum_imu_pair_skew_sec, read_only);
  config.warmup_sec = node.declare_parameter<double>(
    "measurement_warmup_sec", config.warmup_sec, read_only);
  config.ir_dot_projector_intensity = node.declare_parameter<double>(
    "measurement_ir_dot_projector_intensity", config.ir_dot_projector_intensity, read_only);
  config.manual_camera_height_enabled = node.declare_parameter<bool>(
    "manual_camera_height_enabled", config.manual_camera_height_enabled, read_only);
  config.manual_camera_height_m = node.declare_parameter<double>(
    "manual_camera_height_m", config.manual_camera_height_m, read_only);
  config.roi_width = node.declare_parameter<int>(
    "measurement_roi_width", config.roi_width, read_only);
  config.roi_height = node.declare_parameter<int>(
    "measurement_roi_height", config.roi_height, read_only);
  config.roi_vertical_offset_px = node.declare_parameter<int>(
    "measurement_roi_vertical_offset_px", config.roi_vertical_offset_px, read_only);
  config.roi_preview_enabled = node.declare_parameter<bool>(
    "measurement_roi_preview_enabled", config.roi_preview_enabled, read_only);
  config.point_sample_step = node.declare_parameter<int>(
    "measurement_point_sample_step", config.point_sample_step, read_only);
  config.minimum_valid_points = node.declare_parameter<int>(
    "measurement_minimum_valid_points", config.minimum_valid_points, read_only);
  config.minimum_depth_m = node.declare_parameter<double>(
    "measurement_minimum_depth_m", config.minimum_depth_m, read_only);
  config.maximum_depth_m = node.declare_parameter<double>(
    "measurement_maximum_depth_m", config.maximum_depth_m, read_only);
  config.minimum_height_m = node.declare_parameter<double>(
    "measurement_minimum_height_m", config.minimum_height_m, read_only);
  config.maximum_height_m = node.declare_parameter<double>(
    "measurement_maximum_height_m", config.maximum_height_m, read_only);
  config.plane_ransac_iterations = node.declare_parameter<int>(
    "measurement_plane_ransac_iterations", config.plane_ransac_iterations, read_only);
  config.plane_inlier_threshold_m = node.declare_parameter<double>(
    "measurement_plane_inlier_threshold_m", config.plane_inlier_threshold_m, read_only);
  config.plane_minimum_inliers = node.declare_parameter<int>(
    "measurement_plane_minimum_inliers", config.plane_minimum_inliers, read_only);
  config.plane_minimum_inlier_ratio = node.declare_parameter<double>(
    "measurement_plane_minimum_inlier_ratio", config.plane_minimum_inlier_ratio, read_only);
  config.plane_maximum_residual_mad_m = node.declare_parameter<double>(
    "measurement_plane_maximum_residual_mad_m", config.plane_maximum_residual_mad_m, read_only);
  config.plane_maximum_imu_difference_deg = node.declare_parameter<double>(
    "measurement_plane_maximum_imu_difference_deg", config.plane_maximum_imu_difference_deg, read_only);
  config.imu_roll_bias_deg = node.declare_parameter<double>(
    "measurement_imu_roll_bias_deg", config.imu_roll_bias_deg, read_only);
  config.imu_pitch_bias_deg = node.declare_parameter<double>(
    "measurement_imu_pitch_bias_deg", config.imu_pitch_bias_deg, read_only);
  config.imu_sample_count = node.declare_parameter<int>(
    "measurement_imu_sample_count", config.imu_sample_count, read_only);
  config.imu_max_direction_rms_deg = node.declare_parameter<double>(
    "measurement_imu_max_direction_rms_deg", config.imu_max_direction_rms_deg, read_only);
  config.imu_accel_min_mps2 = node.declare_parameter<double>(
    "measurement_imu_accel_min_mps2", config.imu_accel_min_mps2, read_only);
  config.imu_accel_max_mps2 = node.declare_parameter<double>(
    "measurement_imu_accel_max_mps2", config.imu_accel_max_mps2, read_only);
  config.imu_gyroscope_mean_maximum_degps = node.declare_parameter<double>(
    "measurement_imu_gyroscope_mean_maximum_degps", config.imu_gyroscope_mean_maximum_degps, read_only);
  config.imu_gyroscope_stddev_maximum_degps = node.declare_parameter<double>(
    "measurement_imu_gyroscope_stddev_maximum_degps", config.imu_gyroscope_stddev_maximum_degps, read_only);
  config.stable_plane_frame_count = node.declare_parameter<int>(
    "measurement_stable_plane_frame_count", config.stable_plane_frame_count, read_only);
  config.maximum_height_stddev_m = node.declare_parameter<double>(
    "measurement_maximum_height_stddev_m", config.maximum_height_stddev_m, read_only);
  config.maximum_plane_normal_rms_deg = node.declare_parameter<double>(
    "measurement_maximum_plane_normal_rms_deg", config.maximum_plane_normal_rms_deg, read_only);
  config.timeout_sec = node.declare_parameter<double>(
    "measurement_timeout_sec", config.timeout_sec, read_only);
  config.attitude_source = oak_startup::parseStartupAttitudeSource(
    node.declare_parameter<std::string>("measurement_attitude_source", "depth", read_only));
  return config;
}
}  // namespace point_cloud
