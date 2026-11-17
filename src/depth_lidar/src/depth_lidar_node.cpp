#include "depth_lidar/depth_lidar_geometry.hpp"
#include "depth_lidar/depth_lidar_grid.hpp"
#include "depth_lidar/depth_lidar_preview.hpp"
#include "depth_lidar/depth_lidar_stabilization.hpp"

#include <depthai/depthai.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include "oak_startup/oak_startup_measurement.hpp"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <algorithm>
#include <atomic>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace depth_lidar
{

namespace
{
using namespace std::chrono_literals;
constexpr std::uint32_t kColorSensorWidth = 1280U;
constexpr std::uint32_t kColorSensorHeight = 800U;

struct NodeConfig
{
  ProjectionConfig projection;
  GridConfig grid;
  ClusterConfig cluster;
  double camera_fps{30.0};
  std::string camera_resolution{"400p"};
  std::string depth_mode{"high_density"};
  int confidence_threshold{200};
  double ir_dot_projector_intensity{0.5};
  bool left_right_check{true};
  bool subpixel{false};
  bool extended_disparity{false};
  std::string median_filter{"3x3"};
  int confirm_hits{3};
  int confirm_window_frames{4};
  double confirm_distance_m{0.10};
  double hold_sec{0.08};
  double max_age_sec{0.20};
  bool nv12_enabled{false};
  double nv12_fps{30.0};
  int nv12_width{1280};
  int nv12_height{800};
  bool preview_enabled{true};
  bool preview_gui{false};
  double preview_fps{15.0};
  int preview_size_px{700};
  std::string preview_mode{"both"};
  bool stereo_preview_enabled{false};
  bool stereo_preview_gui{false};
  double stereo_preview_fps{10.0};
  double sensor_x_m{-0.16};
  double sensor_y_m{0.0};
  double sensor_yaw_deg{0.0};
  double metrics_interval_sec{1.0};
  std::string frame_id{"front_axle"};
};

bool isOneOf(const std::string & value, const std::vector<std::string> & choices)
{
  return std::find(choices.begin(), choices.end(), value) != choices.end();
}

bool validateNodeConfig(const NodeConfig & config, std::string & reason)
{
  if (!std::isfinite(config.camera_fps) || config.camera_fps < 1.0 || config.camera_fps > 120.0) {
    reason = "camera.fps must be in [1.0, 120.0]";
    return false;
  }
  if (!isOneOf(config.camera_resolution, {"400p", "480p", "720p", "800p"})) {
    reason = "camera.resolution must be one of: 400p, 480p, 720p, 800p";
    return false;
  }
  if (!isOneOf(config.depth_mode, {"default", "high_density", "high_accuracy"})) {
    reason = "depth.mode must be one of: default, high_density, high_accuracy";
    return false;
  }
  if (!std::isfinite(config.ir_dot_projector_intensity)
      || config.ir_dot_projector_intensity < 0.0 || config.ir_dot_projector_intensity > 1.0) {
    reason = "depth.ir_dot_projector_intensity must be finite and in [0.0, 1.0]";
    return false;
  }
  if (config.confidence_threshold < 0 || config.confidence_threshold > 255) {
    reason = "depth.confidence_threshold must be in [0, 255]";
    return false;
  }
  if (config.subpixel && config.extended_disparity) {
    reason = "depth.subpixel and depth.extended_disparity cannot both be enabled";
    return false;
  }
  if (!isOneOf(config.median_filter, {"off", "3x3", "5x5", "7x7"})) {
    reason = "depth.median_filter must be one of: off, 3x3, 5x5, 7x7";
    return false;
  }
  if (!std::isfinite(config.preview_fps) || config.preview_fps <= 0.0 || config.preview_fps > 120.0) {
    reason = "preview.fps must be in (0.0, 120.0]";
    return false;
  }
  if (!std::isfinite(config.stereo_preview_fps)
      || config.stereo_preview_fps <= 0.0 || config.stereo_preview_fps > 120.0) {
    reason = "stereo_preview.fps must be in (0.0, 120.0]";
    return false;
  }
  if (!isOneOf(config.preview_mode, {"points", "clusters", "both"})) {
    reason = "preview.mode must be one of: points, clusters, both";
    return false;
  }
  if (config.preview_size_px < 240 || config.preview_size_px > 2000) {
    reason = "preview.size_px must be in [240, 2000]";
    return false;
  }
  if (!std::isfinite(config.metrics_interval_sec) || config.metrics_interval_sec < 0.1 || config.metrics_interval_sec > 60.0) {
    reason = "metrics.print_interval_sec must be in [0.1, 60.0]";
    return false;
  }
  if (config.frame_id.empty()) {
    reason = "frame_id cannot be empty";
    return false;
  }
  if (!std::isfinite(config.nv12_fps) || config.nv12_fps < 1.0 || config.nv12_fps > 120.0) {
    reason = "nv12.fps must be in [1.0, 120.0]";
    return false;
  }
  if (config.nv12_width < 2 || config.nv12_height < 2
      || config.nv12_width > static_cast<int>(kColorSensorWidth)
      || config.nv12_height > static_cast<int>(kColorSensorHeight) || config.nv12_width % 2 != 0
      || config.nv12_height % 2 != 0)
  {
    reason = "nv12 width and height must be even and no larger than 1280x800";
    return false;
  }
  if (!std::isfinite(config.sensor_x_m) || !std::isfinite(config.sensor_y_m)
      || !std::isfinite(config.sensor_yaw_deg))
  {
    reason = "sensor pose must be finite";
    return false;
  }
  if (!validateGridConfig(config.grid, config.cluster, reason)) { return false; }
  if (!validateProjectionConfig(config.projection, reason)) {
    return false;
  }
  if (config.confirm_window_frames < 1 || config.confirm_window_frames > 30
      || config.confirm_hits < 1 || config.confirm_hits > config.confirm_window_frames
      || !std::isfinite(config.confirm_distance_m) || config.confirm_distance_m <= 0.0) {
    reason = "scan.confirm_hits must be 1..confirm_window_frames (1..30); confirm_distance_m must be finite and positive";
    return false;
  }
  if (!std::isfinite(config.hold_sec) || !std::isfinite(config.max_age_sec)
      || config.hold_sec < 0 || config.hold_sec > 0.5 || config.max_age_sec < 0.02
      || config.max_age_sec > 2.0 || config.hold_sec > config.max_age_sec) {
    reason = "scan.hold_sec must be 0..0.5 and <= input.max_age_sec (0.02..2.0)";
    return false;
  }
  return true;
}

std::pair<std::uint32_t, std::uint32_t> parseResolution(const std::string & value)
{
  if (value == "400p") {
    return {640U, 400U};
  }
  if (value == "480p") {
    return {640U, 480U};
  }
  if (value == "720p") {
    return {1280U, 720U};
  }
  if (value == "800p") {
    return {1280U, 800U};
  }
  throw std::invalid_argument("unsupported camera resolution: " + value);
}

dai::node::StereoDepth::PresetMode parseDepthMode(const std::string & value)
{
  if (value == "default") {
    return dai::node::StereoDepth::PresetMode::DEFAULT;
  }
  if (value == "high_accuracy") {
    return dai::node::StereoDepth::PresetMode::FAST_ACCURACY;
  }
  if (value == "high_density") {
    return dai::node::StereoDepth::PresetMode::FAST_DENSITY;
  }
  throw std::invalid_argument("unsupported depth mode: " + value);
}

dai::StereoDepthConfig::MedianFilter parseMedianFilter(const std::string & value)
{
  if (value == "off") {
    return dai::StereoDepthConfig::MedianFilter::MEDIAN_OFF;
  }
  if (value == "3x3") {
    return dai::StereoDepthConfig::MedianFilter::KERNEL_3x3;
  }
  if (value == "5x5") {
    return dai::StereoDepthConfig::MedianFilter::KERNEL_5x5;
  }
  if (value == "7x7") {
    return dai::StereoDepthConfig::MedianFilter::KERNEL_7x7;
  }
  throw std::invalid_argument("unsupported median filter: " + value);
}

// View only; the owning ImgFrame stays alive until rendering completes.
cv::Mat stereoGrayFrame(dai::ImgFrame & frame)
{
  const int width = static_cast<int>(frame.getWidth());
  const int height = static_cast<int>(frame.getHeight());
  const auto type = frame.getType();
  const std::size_t stride = frame.getStride() == 0U
    ? static_cast<std::size_t>(std::max(0, width)) : frame.getStride();
  auto && bytes = frame.getData();
  if (width <= 0 || height <= 0 || stride < static_cast<std::size_t>(width)
      || (type != dai::ImgFrame::Type::RAW8 && type != dai::ImgFrame::Type::GRAY8
          && type != dai::ImgFrame::Type::YUV400p)
      || bytes.size() < stride * static_cast<std::size_t>(height - 1) + width)
  {
    throw std::runtime_error("invalid rectified grayscale frame");
  }
  return cv::Mat(height, width, CV_8UC1, bytes.data(), stride);
}

sensor_msgs::msg::Image matToImageMessage(const cv::Mat & image,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id)
{
  sensor_msgs::msg::Image message;
  message.header.stamp = stamp;
  message.header.frame_id = frame_id;
  message.height = static_cast<std::uint32_t>(image.rows);
  message.width = static_cast<std::uint32_t>(image.cols);
  message.encoding = sensor_msgs::image_encodings::BGR8;
  message.is_bigendian = false;
  message.step = static_cast<sensor_msgs::msg::Image::_step_type>(image.cols * image.elemSize());
  const std::size_t bytes = message.step * message.height;
  message.data.resize(bytes);
  std::memcpy(message.data.data(), image.data, bytes);
  return message;
}

// The runtime image is RECTIFIED_RIGHT (CAM_C). Startup attitude is raw CAM_A.
FixedTransform rgbFromRectifiedRight(dai::CalibrationHandler & calibration)
{
  using Socket = dai::CameraBoardSocket;
  if (calibration.getStereoRightCameraId() != Socket::CAM_C
      || calibration.getStereoLeftCameraId() != Socket::CAM_B) {
    throw std::runtime_error("Expected calibrated CAM_B/C stereo pair");
  }
  const auto extrinsics = calibration.getCameraExtrinsics(Socket::CAM_C, Socket::CAM_A, false);
  const auto rectification = calibration.getStereoRightRectificationRotation();
  if (extrinsics.size() != 4 || rectification.size() != 3) {
    throw std::runtime_error("Missing stereo-to-RGB calibration");
  }
  for (const auto & row : extrinsics) {
    if (row.size() != 4) { throw std::runtime_error("Invalid extrinsic matrix"); }
  }
  for (const auto & row : rectification) {
    if (row.size() != 3) { throw std::runtime_error("Invalid rectification matrix"); }
  }
  FixedTransform rgb_from_right, right_from_rectified;
  for (int r = 0; r < 3; ++r) {
    rgb_from_right.translation[r] = extrinsics[r][3] * 0.01; // EEPROM centimeters -> meters.
    for (int c = 0; c < 3; ++c) {
      rgb_from_right.rotation[3*r+c] = extrinsics[r][c];
      right_from_rectified.rotation[3*r+c] = rectification[c][r];
    }
  }
  return compose(rgb_from_right, right_from_rectified);
}

double hostSeconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
} // namespace

class DepthLidarNode : public rclcpp::Node
{
public:
  DepthLidarNode() : Node("depth_lidar")
  {
    config_.camera_fps = declare_parameter<double>("camera.fps", config_.camera_fps);
    config_.camera_resolution = declare_parameter<std::string>("camera.resolution", config_.camera_resolution);
    config_.depth_mode = declare_parameter<std::string>("depth.mode", config_.depth_mode);
    config_.ir_dot_projector_intensity = declare_parameter<double>(
      "depth.ir_dot_projector_intensity", config_.ir_dot_projector_intensity);
    config_.confidence_threshold = declare_parameter<int>("depth.confidence_threshold", config_.confidence_threshold);
    config_.left_right_check = declare_parameter<bool>("depth.left_right_check", config_.left_right_check);
    config_.subpixel = declare_parameter<bool>("depth.subpixel", config_.subpixel);
    config_.extended_disparity = declare_parameter<bool>("depth.extended_disparity", config_.extended_disparity);
    config_.median_filter = declare_parameter<std::string>("depth.median_filter", config_.median_filter);
    config_.projection.roi_width_ratio = declare_parameter<double>("roi.width_ratio", config_.projection.roi_width_ratio);
    config_.projection.roi_height_ratio = declare_parameter<double>("roi.height_ratio", config_.projection.roi_height_ratio);
    config_.projection.roi_bottom_offset_ratio = declare_parameter<double>("roi.bottom_offset_ratio", config_.projection.roi_bottom_offset_ratio);
    config_.projection.min_range_m = declare_parameter<double>("range.min_m", config_.projection.min_range_m);
    config_.projection.max_range_m = declare_parameter<double>("range.max_m", config_.projection.max_range_m);
    config_.projection.range_offset_m = declare_parameter<double>("range.offset_m", config_.projection.range_offset_m);
    config_.projection.pixel_stride = declare_parameter<int>("points.pixel_stride", config_.projection.pixel_stride);
    config_.projection.min_depth_m = declare_parameter<double>("points.min_depth_m", config_.projection.min_depth_m);
    config_.projection.max_depth_m = declare_parameter<double>("points.max_depth_m", config_.projection.max_depth_m);
    config_.projection.min_height_m = declare_parameter<double>("height.min_m", config_.projection.min_height_m);
    config_.projection.max_height_m = declare_parameter<double>("height.max_m", config_.projection.max_height_m);
    config_.projection.angle_min_deg = declare_parameter<double>("scan.angle_min_deg", config_.projection.angle_min_deg);
    config_.projection.angle_max_deg = declare_parameter<double>("scan.angle_max_deg", config_.projection.angle_max_deg);
    config_.projection.bins = declare_parameter<int>("scan.bins", config_.projection.bins);
    config_.projection.min_points_per_bin = declare_parameter<int>("scan.min_points_per_bin", config_.projection.min_points_per_bin);
    config_.projection.min_neighbors = declare_parameter<int>("filter.min_neighbors", config_.projection.min_neighbors);
    config_.projection.neighbor_delta_m = declare_parameter<double>("filter.neighbor_delta_m", config_.projection.neighbor_delta_m);
    config_.confirm_hits = declare_parameter<int>("scan.confirm_hits", config_.confirm_hits);
    config_.confirm_window_frames = declare_parameter<int>("scan.confirm_window_frames", config_.confirm_window_frames);
    config_.confirm_distance_m = declare_parameter<double>("scan.confirm_distance_m", config_.confirm_distance_m);
    config_.grid.resolution_m = declare_parameter<double>("grid.resolution_m", config_.grid.resolution_m);
    config_.grid.x_min_m = declare_parameter<double>("grid.x_min_m", config_.grid.x_min_m);
    config_.grid.x_max_m = declare_parameter<double>("grid.x_max_m", config_.grid.x_max_m);
    config_.grid.y_min_m = declare_parameter<double>("grid.y_min_m", config_.grid.y_min_m);
    config_.grid.y_max_m = declare_parameter<double>("grid.y_max_m", config_.grid.y_max_m);
    config_.grid.min_returns_per_cell = declare_parameter<int>("grid.min_returns_per_cell", config_.grid.min_returns_per_cell);
    config_.cluster.connectivity = declare_parameter<int>("cluster.connectivity", config_.cluster.connectivity);
    config_.cluster.min_cells = declare_parameter<int>("cluster.min_cells", config_.cluster.min_cells);
    config_.cluster.min_returns = declare_parameter<int>("cluster.min_returns", config_.cluster.min_returns);
    config_.hold_sec = declare_parameter<double>("scan.hold_sec", config_.hold_sec);
    config_.max_age_sec = declare_parameter<double>("input.max_age_sec", config_.max_age_sec);
    config_.nv12_enabled = declare_parameter<bool>("nv12.enabled", config_.nv12_enabled);
    config_.nv12_fps = declare_parameter<double>("nv12.fps", config_.nv12_fps);
    config_.nv12_width = declare_parameter<int>("nv12.width", config_.nv12_width);
    config_.nv12_height = declare_parameter<int>("nv12.height", config_.nv12_height);
    config_.preview_enabled = declare_parameter<bool>("preview.enabled", config_.preview_enabled);
    config_.preview_gui = declare_parameter<bool>("preview.gui", config_.preview_gui);
    config_.preview_fps = declare_parameter<double>("preview.fps", config_.preview_fps);
    config_.preview_size_px = declare_parameter<int>("preview.size_px", config_.preview_size_px);
    config_.preview_mode = declare_parameter<std::string>("preview.mode", config_.preview_mode);
    config_.stereo_preview_enabled = declare_parameter<bool>("stereo_preview.enabled", config_.stereo_preview_enabled);
    config_.stereo_preview_gui = declare_parameter<bool>("stereo_preview.gui", config_.stereo_preview_gui);
    config_.stereo_preview_fps = declare_parameter<double>("stereo_preview.fps", config_.stereo_preview_fps);
    config_.sensor_x_m = declare_parameter<double>("sensor.x_m", config_.sensor_x_m);
    config_.sensor_y_m = declare_parameter<double>("sensor.y_m", config_.sensor_y_m);
    config_.sensor_yaw_deg = declare_parameter<double>("sensor.yaw_deg", config_.sensor_yaw_deg);
    config_.metrics_interval_sec = declare_parameter<double>("metrics.print_interval_sec", config_.metrics_interval_sec);
    config_.frame_id = declare_parameter<std::string>("frame_id", config_.frame_id);
    startup_.ir_dot_projector_intensity = 0.5;
    startup_.roi_preview_enabled = false;
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;
    startup_.device_id = declare_parameter<std::string>("startup.device_id", startup_.device_id, read_only);
    startup_.roi_width = declare_parameter<int>("startup.roi_width", startup_.roi_width, read_only);
    startup_.roi_height = declare_parameter<int>("startup.roi_height", startup_.roi_height, read_only);
    startup_.roi_vertical_offset_px = declare_parameter<int>("startup.roi_vertical_offset_px", startup_.roi_vertical_offset_px, read_only);
    startup_.roi_preview_enabled = declare_parameter<bool>("startup.roi_preview_enabled", startup_.roi_preview_enabled, read_only);
    startup_.point_sample_step = declare_parameter<int>("startup.point_sample_step", startup_.point_sample_step, read_only);
    startup_.minimum_valid_points = declare_parameter<int>("startup.minimum_valid_points", startup_.minimum_valid_points, read_only);
    startup_.minimum_depth_m = declare_parameter<double>("startup.minimum_depth_m", startup_.minimum_depth_m, read_only);
    startup_.maximum_depth_m = declare_parameter<double>("startup.maximum_depth_m", startup_.maximum_depth_m, read_only);
    startup_.minimum_height_m = declare_parameter<double>("startup.minimum_height_m", startup_.minimum_height_m, read_only);
    startup_.maximum_height_m = declare_parameter<double>("startup.maximum_height_m", startup_.maximum_height_m, read_only);
    startup_.plane_ransac_iterations = declare_parameter<int>("startup.plane_ransac_iterations", startup_.plane_ransac_iterations, read_only);
    startup_.plane_inlier_threshold_m = declare_parameter<double>("startup.plane_inlier_threshold_m", startup_.plane_inlier_threshold_m, read_only);
    startup_.plane_minimum_inliers = declare_parameter<int>("startup.plane_minimum_inliers", startup_.plane_minimum_inliers, read_only);
    startup_.plane_minimum_inlier_ratio = declare_parameter<double>("startup.plane_minimum_inlier_ratio", startup_.plane_minimum_inlier_ratio, read_only);
    startup_.plane_maximum_residual_mad_m = declare_parameter<double>("startup.plane_maximum_residual_mad_m", startup_.plane_maximum_residual_mad_m, read_only);
    startup_.plane_maximum_imu_difference_deg = declare_parameter<double>("startup.plane_maximum_imu_difference_deg", startup_.plane_maximum_imu_difference_deg, read_only);
    startup_.stable_plane_frame_count = declare_parameter<int>("startup.stable_plane_frame_count", startup_.stable_plane_frame_count, read_only);
    startup_.maximum_height_stddev_m = declare_parameter<double>("startup.maximum_height_stddev_m", startup_.maximum_height_stddev_m, read_only);
    startup_.maximum_plane_normal_rms_deg = declare_parameter<double>("startup.maximum_plane_normal_rms_deg", startup_.maximum_plane_normal_rms_deg, read_only);
    startup_.timeout_sec = declare_parameter<double>("startup.timeout_sec", startup_.timeout_sec, read_only);
    startup_.warmup_sec = declare_parameter<double>("startup.warmup_sec", startup_.warmup_sec, read_only);
    startup_.ir_dot_projector_intensity = declare_parameter<double>("startup.ir_dot_projector_intensity", startup_.ir_dot_projector_intensity, read_only);
    startup_.imu_sample_count = declare_parameter<int>("startup.imu_sample_count", startup_.imu_sample_count, read_only);
    startup_.imu_max_direction_rms_deg = declare_parameter<double>("startup.imu_max_direction_rms_deg", startup_.imu_max_direction_rms_deg, read_only);
    startup_.imu_gyroscope_mean_maximum_degps = declare_parameter<double>("startup.imu_gyroscope_mean_maximum_degps", startup_.imu_gyroscope_mean_maximum_degps, read_only);
    startup_.imu_gyroscope_stddev_maximum_degps = declare_parameter<double>("startup.imu_gyroscope_stddev_maximum_degps", startup_.imu_gyroscope_stddev_maximum_degps, read_only);
    startup_.imu_roll_bias_deg = declare_parameter<double>("startup.imu_roll_bias_deg", startup_.imu_roll_bias_deg, read_only);
    startup_.imu_pitch_bias_deg = declare_parameter<double>("startup.imu_pitch_bias_deg", startup_.imu_pitch_bias_deg, read_only);
    startup_.stereo_confidence_threshold = declare_parameter<int>("startup.stereo_confidence_threshold", startup_.stereo_confidence_threshold, read_only);
    startup_.attitude_source = oak_startup::parseStartupAttitudeSource(
      declare_parameter<std::string>("startup.attitude_source", "depth", read_only));
    std::string reason;
    if (!validateNodeConfig(config_, reason)) { throw std::invalid_argument(reason); }
    // Reject obsolete profiles instead of silently using defaults for removed stages.
    for (const auto & entry : get_node_parameters_interface()->get_parameter_overrides()) {
      const auto & name = entry.first;
      if (name.rfind("ground.", 0) == 0 || name.rfind("floor.", 0) == 0
          || name == "grid.min_points_per_cell" || name == "cluster.min_points"
          || name == "cluster.radius_margin_m" || name == "cluster.min_radius_m"
          || name == "cluster.max_radius_m" || name == "cluster.neighbor_distance_m"
          || name.rfind("stabilization.", 0) == 0 || name.rfind("bev.", 0) == 0
          || name == "scan.range_selection" || name == "preview.scale") {
        throw std::invalid_argument("Obsolete depth_lidar parameter: " + name + "; use the fixed-pose scan YAML");
      }
    }
    occupancy_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("~/occupancy", rclcpp::SensorDataQoS());
    contours_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/contours", rclcpp::SensorDataQoS());
    clusters_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/clusters", rclcpp::SensorDataQoS());
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("~/scan", rclcpp::SensorDataQoS());
    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/scan_points", rclcpp::SensorDataQoS());
    status_pub_ = create_publisher<std_msgs::msg::String>("~/status", rclcpp::QoS(1).transient_local());
    pose_pub_ = create_publisher<std_msgs::msg::String>("~/startup_pose", rclcpp::QoS(1).transient_local());
    ready_pub_ = create_publisher<std_msgs::msg::Bool>("~/ready", rclcpp::QoS(1).transient_local());
    preview_pub_ = create_publisher<sensor_msgs::msg::Image>("~/preview", rclcpp::SensorDataQoS());
    stereo_pub_ = create_publisher<sensor_msgs::msg::Image>("~/stereo_preview", rclcpp::SensorDataQoS());
    parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        std::lock_guard<std::mutex> lock(config_mutex_);
        auto next = config_;
        bool restart_runtime = false;
        try {
          for (const auto & p : parameters) {
            if (p.get_name() != "preview.mode") {
              restart_runtime = true;
            }
            if (p.get_name() == "camera.fps") { next.camera_fps = p.as_double(); }
            else if (p.get_name() == "camera.resolution") { next.camera_resolution = p.as_string(); }
            else if (p.get_name() == "depth.mode") { next.depth_mode = p.as_string(); }
            else if (p.get_name() == "depth.ir_dot_projector_intensity") { next.ir_dot_projector_intensity = p.as_double(); }
            else if (p.get_name() == "depth.confidence_threshold") { next.confidence_threshold = p.as_int(); }
            else if (p.get_name() == "depth.left_right_check") { next.left_right_check = p.as_bool(); }
            else if (p.get_name() == "depth.subpixel") { next.subpixel = p.as_bool(); }
            else if (p.get_name() == "depth.extended_disparity") { next.extended_disparity = p.as_bool(); }
            else if (p.get_name() == "depth.median_filter") { next.median_filter = p.as_string(); }
            else if (p.get_name() == "roi.width_ratio") { next.projection.roi_width_ratio = p.as_double(); }
            else if (p.get_name() == "roi.height_ratio") { next.projection.roi_height_ratio = p.as_double(); }
            else if (p.get_name() == "roi.bottom_offset_ratio") { next.projection.roi_bottom_offset_ratio = p.as_double(); }
            else if (p.get_name() == "range.min_m") { next.projection.min_range_m = p.as_double(); }
            else if (p.get_name() == "range.max_m") { next.projection.max_range_m = p.as_double(); }
            else if (p.get_name() == "range.offset_m") { next.projection.range_offset_m = p.as_double(); }
            else if (p.get_name() == "points.pixel_stride") { next.projection.pixel_stride = p.as_int(); }
            else if (p.get_name() == "points.min_depth_m") { next.projection.min_depth_m = p.as_double(); }
            else if (p.get_name() == "points.max_depth_m") { next.projection.max_depth_m = p.as_double(); }
            else if (p.get_name() == "height.min_m") { next.projection.min_height_m = p.as_double(); }
            else if (p.get_name() == "height.max_m") { next.projection.max_height_m = p.as_double(); }
            else if (p.get_name() == "scan.angle_min_deg") { next.projection.angle_min_deg = p.as_double(); }
            else if (p.get_name() == "scan.angle_max_deg") { next.projection.angle_max_deg = p.as_double(); }
            else if (p.get_name() == "scan.bins") { next.projection.bins = p.as_int(); }
            else if (p.get_name() == "scan.min_points_per_bin") { next.projection.min_points_per_bin = p.as_int(); }
            else if (p.get_name() == "filter.min_neighbors") { next.projection.min_neighbors = p.as_int(); }
            else if (p.get_name() == "filter.neighbor_delta_m") { next.projection.neighbor_delta_m = p.as_double(); }
            else if (p.get_name() == "scan.confirm_hits") { next.confirm_hits = p.as_int(); }
            else if (p.get_name() == "scan.confirm_window_frames") { next.confirm_window_frames = p.as_int(); }
            else if (p.get_name() == "scan.confirm_distance_m") { next.confirm_distance_m = p.as_double(); }
            else if (p.get_name() == "grid.resolution_m") { next.grid.resolution_m = p.as_double(); }
            else if (p.get_name() == "grid.x_min_m") { next.grid.x_min_m = p.as_double(); }
            else if (p.get_name() == "grid.x_max_m") { next.grid.x_max_m = p.as_double(); }
            else if (p.get_name() == "grid.y_min_m") { next.grid.y_min_m = p.as_double(); }
            else if (p.get_name() == "grid.y_max_m") { next.grid.y_max_m = p.as_double(); }
            else if (p.get_name() == "grid.min_returns_per_cell") { next.grid.min_returns_per_cell = p.as_int(); }
            else if (p.get_name() == "cluster.connectivity") { next.cluster.connectivity = p.as_int(); }
            else if (p.get_name() == "cluster.min_cells") { next.cluster.min_cells = p.as_int(); }
            else if (p.get_name() == "cluster.min_returns") { next.cluster.min_returns = p.as_int(); }
            else if (p.get_name() == "scan.hold_sec") { next.hold_sec = p.as_double(); }
            else if (p.get_name() == "input.max_age_sec") { next.max_age_sec = p.as_double(); }
            else if (p.get_name() == "nv12.enabled") { next.nv12_enabled = p.as_bool(); }
            else if (p.get_name() == "nv12.fps") { next.nv12_fps = p.as_double(); }
            else if (p.get_name() == "nv12.width") { next.nv12_width = p.as_int(); }
            else if (p.get_name() == "nv12.height") { next.nv12_height = p.as_int(); }
            else if (p.get_name() == "preview.enabled") { next.preview_enabled = p.as_bool(); }
            else if (p.get_name() == "preview.gui") { next.preview_gui = p.as_bool(); }
            else if (p.get_name() == "preview.fps") { next.preview_fps = p.as_double(); }
            else if (p.get_name() == "preview.size_px") { next.preview_size_px = p.as_int(); }
            else if (p.get_name() == "preview.mode") { next.preview_mode = p.as_string(); }
            else if (p.get_name() == "stereo_preview.enabled") { next.stereo_preview_enabled = p.as_bool(); }
            else if (p.get_name() == "stereo_preview.gui") { next.stereo_preview_gui = p.as_bool(); }
            else if (p.get_name() == "stereo_preview.fps") { next.stereo_preview_fps = p.as_double(); }
            else if (p.get_name() == "sensor.x_m") { next.sensor_x_m = p.as_double(); }
            else if (p.get_name() == "sensor.y_m") { next.sensor_y_m = p.as_double(); }
            else if (p.get_name() == "sensor.yaw_deg") { next.sensor_yaw_deg = p.as_double(); }
            else if (p.get_name() == "metrics.print_interval_sec") { next.metrics_interval_sec = p.as_double(); }
            else if (p.get_name() == "frame_id") { next.frame_id = p.as_string(); }
            else if (p.get_name() != "use_sim_time") { throw std::invalid_argument("Unknown/read-only parameter: " + p.get_name()); }
          }
          if (!validateNodeConfig(next, result.reason)) { return result; }
          config_ = next;
          if (restart_runtime) { restart_requested_.store(true); }
          result.successful = true;
        } catch (const std::exception & e) { result.reason = e.what(); }
        return result;
      });
    worker_ = std::thread([this]() { cameraLoop(); });
  }
  ~DepthLidarNode() override
  {
    stop_requested_.store(true);
    if (worker_.joinable()) { worker_.join(); }
  }
private:
  bool stopping() const { return stop_requested_.load() || !rclcpp::ok(); }
  void status(const std::string & text, bool ready)
  {
    std_msgs::msg::String message; message.data = text; status_pub_->publish(message);
    std_msgs::msg::Bool flag; flag.data = ready; ready_pub_->publish(flag);
  }
  void publishGrid(const GridResult & grid, const NodeConfig & c,
    const builtin_interfaces::msg::Time & stamp)
  {
    nav_msgs::msg::OccupancyGrid occupancy;
    occupancy.header.stamp=stamp; occupancy.header.frame_id=c.frame_id;
    occupancy.info.resolution=grid.config.resolution_m;
    occupancy.info.width=grid.width; occupancy.info.height=grid.height;
    occupancy.info.origin.position.x=grid.config.x_min_m;
    occupancy.info.origin.position.y=grid.config.y_min_m;
    occupancy.info.origin.orientation.w=1.0;
    occupancy.data.resize(grid.cells.size(),-1);
    for (std::size_t i=0; i<grid.cells.size(); ++i) {
      if (grid.cells[i].returns) { occupancy.data[i]=100; }
    }
    occupancy_pub_->publish(occupancy);
    if (contours_pub_->get_subscription_count()) {
      visualization_msgs::msg::MarkerArray message;
      // IDs are frame-local. Clear old groups even when this frame has no clusters.
      message.markers.resize(grid.clusters.size()+1);
      message.markers[0].header=occupancy.header;
      message.markers[0].action=visualization_msgs::msg::Marker::DELETEALL;
      for (std::size_t i=0; i<grid.clusters.size(); ++i) {
        auto & marker=message.markers[i+1];
        marker.header=occupancy.header; marker.ns="depth_grid_clusters"; marker.id=static_cast<int>(i);
        marker.type=visualization_msgs::msg::Marker::LINE_LIST;
        marker.action=visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w=1.0; marker.scale.x=0.01; marker.color.a=1.0F;
        marker.lifetime.sec=1;
      }
      for (const auto & edge:grid.boundary) {
        auto & marker=message.markers[edge.cluster_id+1];
        geometry_msgs::msg::Point a,b;
        a.x=edge.x1; a.y=edge.y1; b.x=edge.x2; b.y=edge.y2;
        marker.points.push_back(a); marker.points.push_back(b);
        std_msgs::msg::ColorRGBA color;
        color.r=edge.age_sec>0 ? .59F : .90F; color.g=.59F;
        color.b=edge.age_sec>0 ? .59F : 0.0F; color.a=1.0F;
        marker.colors.push_back(color); marker.colors.push_back(color);
      }
      contours_pub_->publish(message);
    }
    if (!clusters_pub_->get_subscription_count()) { return; }
    sensor_msgs::msg::PointCloud2 message; message.header=occupancy.header;
    sensor_msgs::PointCloud2Modifier modifier(message);
    using F=sensor_msgs::msg::PointField;
    modifier.setPointCloud2Fields(12,
      "x",1,F::FLOAT32,"y",1,F::FLOAT32,"z",1,F::FLOAT32,
      "min_x",1,F::FLOAT32,"max_x",1,F::FLOAT32,"min_y",1,F::FLOAT32,"max_y",1,F::FLOAT32,
      "nearest_range_m",1,F::FLOAT32,"observation_age_sec",1,F::FLOAT32,
      "cluster_id",1,F::UINT32,"cell_count",1,F::UINT32,"return_count",1,F::UINT32);
    modifier.resize(grid.clusters.size()); message.is_dense=true;
    if (!grid.clusters.empty()) {
      sensor_msgs::PointCloud2Iterator<float> x(message,"x"),y(message,"y"),z(message,"z"),
        min_x(message,"min_x"),max_x(message,"max_x"),min_y(message,"min_y"),max_y(message,"max_y"),
        range(message,"nearest_range_m"),age(message,"observation_age_sec");
      sensor_msgs::PointCloud2Iterator<std::uint32_t> id(message,"cluster_id"),cells(message,"cell_count"),returns(message,"return_count");
      for (std::size_t i=0; i<grid.clusters.size(); ++i) {
        const auto & cluster=grid.clusters[i];
        *x=cluster.center_x; *y=cluster.center_y; *z=0.0F;
        *min_x=cluster.min_x; *max_x=cluster.max_x; *min_y=cluster.min_y; *max_y=cluster.max_y;
        *range=cluster.nearest_range_m; *age=cluster.age_sec;
        *id=static_cast<std::uint32_t>(i); *cells=static_cast<std::uint32_t>(cluster.cells.size());
        *returns=static_cast<std::uint32_t>(cluster.returns);
        ++x; ++y; ++z; ++min_x; ++max_x; ++min_y; ++max_y; ++range; ++age; ++id; ++cells; ++returns;
      }
    }
    clusters_pub_->publish(message);
  }
  GridResult publishScan(const ScanResult & scan, const NodeConfig & c,
    const builtin_interfaces::msg::Time & stamp, double scan_time)
  {
    const double grid_start=hostSeconds();
    auto grid=clusterScan(scan,c.projection,c.grid,c.cluster);
    last_grid_ms_=(hostSeconds()-grid_start)*1000.0;
    publishGrid(grid,c,stamp);
    constexpr double rad = 3.14159265358979323846 / 180.0;
    sensor_msgs::msg::LaserScan message;
    message.header.stamp = stamp; message.header.frame_id = c.frame_id;
    message.angle_min = c.projection.angle_min_deg * rad;
    message.angle_max = c.projection.angle_max_deg * rad;
    message.angle_increment = (message.angle_max - message.angle_min) / (c.projection.bins - 1);
    message.time_increment = 0.0F; message.scan_time = scan_time;
    message.range_min = c.projection.min_range_m; message.range_max = c.projection.max_range_m;
    message.ranges = scan.ranges; // NaN means unobserved; never claim free space.
    scan_pub_->publish(message);
    if (!points_pub_->get_subscription_count()) { return grid; }
    sensor_msgs::msg::PointCloud2 points;
    points.header = message.header;
    sensor_msgs::PointCloud2Modifier modifier(points);
    using F = sensor_msgs::msg::PointField;
    modifier.setPointCloud2Fields(4, "x", 1, F::FLOAT32, "y", 1, F::FLOAT32,
      "z", 1, F::FLOAT32, "observation_age_sec", 1, F::FLOAT32);
    modifier.resize(scan.valid_bins); points.is_dense = true;
    if (scan.valid_bins) {
      sensor_msgs::PointCloud2Iterator<float> x(points,"x"), y(points,"y"), z(points,"z"), age(points,"observation_age_sec");
      for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
        if (!std::isfinite(scan.ranges[i])) { continue; }
        const double angle = message.angle_min + i * message.angle_increment;
        *x = scan.ranges[i] * std::cos(angle); *y = scan.ranges[i] * std::sin(angle);
        *z = 0.0F; *age = scan.ages[i]; ++x; ++y; ++z; ++age;
      }
    }
    points_pub_->publish(points);
    return grid;
  }
  void configurePipeline(dai::Pipeline & pipeline,
    const NodeConfig & config,
    std::shared_ptr<dai::node::Camera> & left,
    std::shared_ptr<dai::node::Camera> & right,
    std::shared_ptr<dai::node::StereoDepth> & stereo,
    dai::Node::Output *& nv12_output)
  {
    const auto resolution = parseResolution(config.camera_resolution);
    const float camera_fps = static_cast<float>(config.camera_fps);
    left = pipeline.create<dai::node::Camera>();
    right = pipeline.create<dai::node::Camera>();
    left->build(dai::CameraBoardSocket::CAM_B, resolution, camera_fps);
    right->build(dai::CameraBoardSocket::CAM_C, resolution, camera_fps);
    auto * left_output = left->requestOutput(resolution);
    auto * right_output = right->requestOutput(resolution);
    stereo = pipeline.create<dai::node::StereoDepth>();
    stereo->build(*left_output, *right_output, parseDepthMode(config.depth_mode));
    stereo->initialConfig->setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::MILLIMETER);
    stereo->initialConfig->setConfidenceThreshold(config.confidence_threshold);
    stereo->initialConfig->setMedianFilter(parseMedianFilter(config.median_filter));
    stereo->setLeftRightCheck(config.left_right_check);
    stereo->setSubpixel(config.subpixel);
    stereo->setExtendedDisparity(config.extended_disparity);
    // Runtime depth and ROI use the right rectified optical frame.
    stereo->setDepthAlign(dai::StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_RIGHT);

    nv12_output = nullptr;
    if (config.nv12_enabled) {
      auto color = pipeline.create<dai::node::Camera>();
      color->build(dai::CameraBoardSocket::CAM_A,
        std::make_pair(kColorSensorWidth, kColorSensorHeight),
        static_cast<float>(config.nv12_fps));
      nv12_output =
        color->requestOutput(std::make_pair(static_cast<std::uint32_t>(config.nv12_width),
                               static_cast<std::uint32_t>(config.nv12_height)),
          dai::ImgFrame::Type::NV12,
          dai::ImgResizeMode::CROP,
          static_cast<float>(config.nv12_fps));
    }
  }

  void cameraLoop()
  {
    oak_startup::OakStartupMeasurement pose;
    status("MEASURING STARTUP POSE: keep vehicle stationary on level ground", false);
    try {
      RCLCPP_INFO(get_logger(), "Startup IR dot projector intensity: %.2f",
        startup_.ir_dot_projector_intensity);
      pose = oak_startup::measureOakStartupExtrinsics(startup_, [this]() { return stopping(); });
      std::ostringstream description;
      description << std::setprecision(9) << "CAM_A fixed pose: device=" << pose.device_id
        << " roll_deg=" << pose.roll_deg << " pitch_down_deg=" << pose.pitch_down_deg
        << " height_m=" << pose.height_m << " source=" << pose.attitude_source;
      std_msgs::msg::String message; message.data = description.str(); pose_pub_->publish(message);
      RCLCPP_INFO(get_logger(), "%s", message.data.c_str());
    } catch (const std::exception & e) {
      if (!stopping()) {
        status(std::string("STARTUP FAILED: ") + e.what() + "; restart while stationary", false);
        RCLCPP_ERROR(get_logger(), "Startup measurement failed: %s", e.what());
      }
      return; // Never silently calibrate later while the car might be moving.
    }
    while (!stopping()) {
      NodeConfig c;
      {
        std::lock_guard<std::mutex> lock(config_mutex_);
        restart_requested_.store(false);
        c = config_;
      }
      try {
        status("OPENING DEPTH PIPELINE / FIXED POSE", false);
        publishScan(emptyScan(c.projection), c, now(), 1.0/c.camera_fps);
        auto device = std::make_shared<dai::Device>(dai::DeviceInfo(pose.device_id), dai::UsbSpeed::SUPER);
        auto calibration = device->getCalibration();
        const auto vehicle_from_rgb = cameraMount(pose.roll_deg, pose.pitch_down_deg,
          c.sensor_yaw_deg, c.sensor_x_m, c.sensor_y_m, pose.height_m);
        const auto transform = compose(vehicle_from_rgb, rgbFromRectifiedRight(calibration));
        dai::Pipeline pipeline(device);
        pipeline.setAutoCalibrationMode(dai::Pipeline::AutoCalibrationMode::OFF);
        pipeline.setXLinkChunkSize(0);
        std::shared_ptr<dai::node::Camera> left, right;
        std::shared_ptr<dai::node::StereoDepth> stereo;
        dai::Node::Output * nv12_output = nullptr;
        configurePipeline(pipeline, c, left, right, stereo, nv12_output);
        auto depth_queue = stereo->depth.createOutputQueue(1, false);
        std::shared_ptr<dai::MessageQueue> left_queue, right_queue, nv12_queue;
        if (c.stereo_preview_enabled) {
          left_queue = stereo->rectifiedLeft.createOutputQueue(1, false);
          right_queue = stereo->rectifiedRight.createOutputQueue(1, false);
        }
        if (nv12_output) { nv12_queue = nv12_output->createOutputQueue(1, false); }
        pipeline.build();
        const auto bound = [](dai::Node::Output & output) {
          const auto bridge = output.getXLinkBridge();
          if (!bridge || !bridge->xLinkOut) { throw std::runtime_error("Missing XLink bridge"); }
          bridge->xLinkOut->input.setMaxSize(1); bridge->xLinkOut->input.setBlocking(false);
        };
        bound(stereo->depth);
        if (left_queue) { bound(stereo->rectifiedLeft); bound(stereo->rectifiedRight); }
        if (nv12_output) { bound(*nv12_output); }
        pipeline.start();
        // Apply on every open/restart, including zero: startup and runtime are independent.
        const bool ir_applied = device->setIrLaserDotProjectorIntensity(
          static_cast<float>(c.ir_dot_projector_intensity));
        if (!ir_applied && c.ir_dot_projector_intensity > 0.0) {
          throw std::runtime_error(
            "Failed to enable runtime IR dot projector; depth.ir_dot_projector_intensity > 0 requires a supported OAK Pro device");
        }
        RCLCPP_INFO(get_logger(), "Runtime IR dot projector intensity: %.2f (applied=%s)",
          c.ir_dot_projector_intensity, ir_applied ? "true" : "false");
        std::atomic_bool nv12_stop{false}, nv12_failed{false};
        std::atomic<std::uint64_t> nv12_count{0};
        std::thread receiver;
        if (nv12_queue) {
          receiver = std::thread([&]() {
            try {
              while (!stopping() && !nv12_stop.load() && !restart_requested_.load()) {
                if (auto frame = nv12_queue->tryGet<dai::ImgFrame>()) {
                  if (frame->getType() != dai::ImgFrame::Type::NV12) { throw std::runtime_error("Invalid NV12 frame"); }
                  ++nv12_count;
                } else { std::this_thread::sleep_for(250us); }
              }
            } catch (...) { nv12_failed.store(true); }
          });
        }
        std::exception_ptr error;
        try { processFrames(c, transform, depth_queue, left_queue, right_queue, nv12_count, nv12_failed); }
        catch (...) { error = std::current_exception(); }
        nv12_stop.store(true);
        if (receiver.joinable()) { receiver.join(); }
        pipeline.stop();
        if (error) { std::rethrow_exception(error); }
      } catch (const std::exception & e) {
        if (!stopping()) {
          status(std::string("DEPTH UNAVAILABLE: ") + e.what(), false);
          publishScan(emptyScan(c.projection), c, now(), 1.0/c.camera_fps);
          RCLCPP_ERROR(get_logger(), "Depth pipeline: %s", e.what());
          for (int i=0; i<10 && !stopping(); ++i) { std::this_thread::sleep_for(100ms); }
        }
      }
    }
  }

  void processFrames(const NodeConfig & c, const FixedTransform & transform,
    const std::shared_ptr<dai::MessageQueue> & depth_queue,
    const std::shared_ptr<dai::MessageQueue> & left_queue,
    const std::shared_ptr<dai::MessageQueue> & right_queue,
    const std::atomic<std::uint64_t> & nv12_count, const std::atomic_bool & nv12_failed)
  {
    ScanProjector projector;
    ScanHold hold(c.confirm_hits, c.confirm_window_frames, c.confirm_distance_m);
    CameraGeometry camera;
    auto displayed = emptyScan(c.projection);
    auto displayed_grid = clusterScan(displayed,c.projection,c.grid,c.cluster);
    bool configured = false, ready = false, radar_open = false, stereo_open = false;
    double last_rx = hostSeconds(), previous_frame = -1.0;
    double last_preview = 0.0, last_stereo = 0.0, metric_start = last_rx;
    double processing_sum = 0.0, delay_sum = 0.0, fps = 0.0, ms = 0.0, nv12_fps = 0.0;
    std::size_t frames = 0;
    std::uint64_t previous_nv12 = 0;
    std::shared_ptr<dai::ImgFrame> left_frame, right_frame;
    const auto close_windows = [&]() {
      if (radar_open) { cv::destroyWindow("depth_lidar radar preview"); }
      if (stereo_open) { cv::destroyWindow("depth_lidar stereo ROI"); }
    };
    try {
      while (!stopping() && !restart_requested_.load()) {
        if (nv12_failed.load()) { throw std::runtime_error("NV12 receiver failed"); }
        const double clock = hostSeconds();
        if (radar_open || stereo_open) {
          const int key = cv::waitKey(1) & 0xff;
          if (key >= '1' && key <= '3') {
            const std::string mode = key == '1' ? "points" : (key == '2' ? "clusters" : "both");
            const auto result = set_parameters_atomically({rclcpp::Parameter("preview.mode", mode)});
            if (!result.successful) { RCLCPP_WARN(get_logger(), "%s", result.reason.c_str()); }
            last_preview = 0.0;
          }
          if (key == 'c' || key == 'C') {
            const auto result = set_parameters_atomically({rclcpp::Parameter("stereo_preview.enabled", !c.stereo_preview_enabled)});
            if (!result.successful) { RCLCPP_WARN(get_logger(), "%s", result.reason.c_str()); }
            continue;
          }
        }
        if (ready && clock - last_rx > c.max_age_sec) {
          hold.clear(); displayed = emptyScan(c.projection); ready = false;
          status("DEPTH STALE / FIXED POSE RETAINED", false);
          displayed_grid = publishScan(displayed, c, now(), 1.0/c.camera_fps);
        } else if (clock > hold.nextExpiryTime()) {
          displayed = hold.snapshot(clock, c.hold_sec);
          displayed_grid = publishScan(displayed, c, now(), 1.0/c.camera_fps);
        }
        auto frame = depth_queue->tryGet<dai::ImgFrame>();
        if (frame) {
          const double start = hostSeconds();
          const double capture = std::chrono::duration<double>(frame->getTimestamp().time_since_epoch()).count();
          const double age = start - capture;
          const int width = frame->getWidth(), height = frame->getHeight();
          const auto & bytes = frame->getData();
          const std::size_t packed = static_cast<std::size_t>(std::max(0,width))*sizeof(std::uint16_t);
          const std::size_t stride = frame->getStride() ? frame->getStride() : packed;
          if (width <= 0 || height <= 0 || stride < packed || stride % 2
              || frame->getType() != dai::ImgFrame::Type::RAW16
              || bytes.size() < stride * static_cast<std::size_t>(height-1) + packed
              || age < -0.01 || age > c.max_age_sec || capture <= previous_frame) {
            continue; // Replayed/stale frames cannot renew the hold.
          }
          if (previous_frame >= 0 && capture - previous_frame > c.max_age_sec) { hold.clear(); }
          const double scan_time = previous_frame < 0 ? 1.0/c.camera_fps : capture - previous_frame;
          previous_frame = capture;
          const auto & metadata = frame->getTransformation();
          if (!metadata.isValid()) { throw std::runtime_error("Missing rectified depth intrinsics"); }
          const auto k = metadata.getIntrinsicMatrix();
          CameraGeometry next{width, height, k[0][0], k[1][1], k[0][2], k[1][2]};
          if (!configured || camera.width != width || camera.height != height
              || camera.fx != next.fx || camera.fy != next.fy || camera.cx != next.cx || camera.cy != next.cy) {
            camera = next; projector.configure(camera, transform, c.projection); hold.clear(); configured = true;
            RCLCPP_INFO(get_logger(), "Fixed projection: %dx%d, %zu cached rays, %d angle bins",
              width, height, projector.rayCount(), c.projection.bins);
          }
          const auto raw = projector.project(reinterpret_cast<const std::uint16_t *>(bytes.data()), stride/2);
          displayed = hold.update(raw, start, c.hold_sec);
          const double processing_ms = (hostSeconds()-start)*1000.0;
          if (!ready) { status("READY / FIXED STARTUP POSE", true); ready = true; }
          last_rx = start;
          const auto stamp = now() - rclcpp::Duration::from_seconds(std::max(0.0, hostSeconds()-capture));
          displayed_grid = publishScan(displayed, c, stamp, scan_time);
          ++frames; processing_sum += processing_ms + last_grid_ms_; delay_sum += age*1000.0;
        }
        if (left_queue) {
          if (auto f = left_queue->tryGet<dai::ImgFrame>()) { left_frame = std::move(f); }
          if (auto f = right_queue->tryGet<dai::ImgFrame>()) { right_frame = std::move(f); }
          if (left_frame && right_frame) {
            if (left_frame->getSequenceNum() < right_frame->getSequenceNum()) { left_frame.reset(); }
            else if (right_frame->getSequenceNum() < left_frame->getSequenceNum()) { right_frame.reset(); }
            else if (configured && clock-last_stereo >= 1.0/c.stereo_preview_fps) {
              auto preview = makeStereoPreview(stereoGrayFrame(*left_frame), stereoGrayFrame(*right_frame),
                projector.roi(), camera.width, camera.height);
              stereo_pub_->publish(matToImageMessage(preview, now(), c.frame_id));
              if (c.stereo_preview_gui) { cv::imshow("depth_lidar stereo ROI", preview); stereo_open = true; }
              last_stereo = clock; left_frame.reset(); right_frame.reset();
            }
          }
        }
        if (clock-metric_start >= c.metrics_interval_sec) {
          fps = frames / (clock-metric_start); ms = frames ? processing_sum/frames : 0.0;
          const auto total = nv12_count.load(); nv12_fps = (total-previous_nv12)/(clock-metric_start);
          RCLCPP_INFO(get_logger(), "depth %.1f FPS | project+grid %.3f ms | bins %zu/%d | delay %.1f ms | NV12 %.1f FPS",
            fps, ms, displayed.valid_bins, c.projection.bins, frames ? delay_sum/frames : 0.0, nv12_fps);
          previous_nv12 = total; frames = 0; processing_sum = delay_sum = 0; metric_start = clock;
        }
        if (c.preview_enabled && clock-last_preview >= 1.0/c.preview_fps) {
          std::string preview_mode;
          {
            std::lock_guard<std::mutex> lock(config_mutex_);
            preview_mode = config_.preview_mode;
          }
          auto preview = makeRadarPreview(displayed, displayed_grid, c.projection, c.preview_size_px, ready, preview_mode);
          std::ostringstream metrics;
          metrics << std::fixed << std::setprecision(1) << "DEPTH " << fps << " FPS | NV12 " << nv12_fps
            << " FPS | PROJECT+GRID " << std::setprecision(3) << ms << " ms";
          cv::putText(preview, metrics.str(), cv::Point(8,preview.rows-10), cv::FONT_HERSHEY_SIMPLEX, .35, cv::Scalar(65,65,65),1,cv::LINE_AA);
          cv::putText(preview, ready ? "FIXED POSE | C: CAMERA | 1: POINTS  2: CLUSTERS  3: BOTH" : "WAITING FOR DEPTH | 1: POINTS  2: CLUSTERS  3: BOTH",
            cv::Point(8,56),cv::FONT_HERSHEY_SIMPLEX,.35,cv::Scalar(65,65,65),1,cv::LINE_AA);
          std::ostringstream band;
          band << "HEIGHT " << c.projection.min_height_m << ".." << c.projection.max_height_m
            << "m | BLUE: points / ORANGE: clusters / GRAY: held";
          cv::putText(preview, band.str(), cv::Point(8,74),cv::FONT_HERSHEY_SIMPLEX,.35,cv::Scalar(65,65,65),1,cv::LINE_AA);
          preview_pub_->publish(matToImageMessage(preview, now(), c.frame_id));
          if (c.preview_gui) { cv::imshow("depth_lidar radar preview", preview); radar_open = true; }
          last_preview = clock;
        }
        if (!frame) { std::this_thread::sleep_for(1ms); }
      }
    } catch (...) { close_windows(); throw; }
    close_windows();
  }

  std::mutex config_mutex_;
  NodeConfig config_;
  oak_startup::OakStartupMeasurementConfig startup_;
  std::atomic_bool stop_requested_{false}, restart_requested_{false};
  std::thread worker_;
  double last_grid_ms_{0.0}; // Worker-owned; excludes ROS publication and rendering.
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr contours_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clusters_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_, pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr preview_pub_, stereo_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};
} // namespace depth_lidar
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<depth_lidar::DepthLidarNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("depth_lidar"), "Fatal error: %s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
