#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "bev_processor/bev_geometry.hpp"
#include "bev_processor/cuda_bev_processor.hpp"
#include "bev_processor/oak_startup_measurement.hpp"

namespace bev_processor
{

namespace
{

using SteadyClock = std::chrono::steady_clock;

struct BevFrame
{
  cv::Mat image;
  std_msgs::msg::Header header;
  SteadyClock::time_point input_received_at;
  std::uint64_t generation{0U};
};

bool graphicalDisplayAvailable()
{
#ifdef __APPLE__
  return true;
#else
  const char * display = std::getenv("DISPLAY");
  const char * wayland_display = std::getenv("WAYLAND_DISPLAY");
  return
    (display != nullptr && display[0] != '\0') ||
    (wayland_display != nullptr && wayland_display[0] != '\0');
#endif
}

std::unique_ptr<sensor_msgs::msg::Image> makeBgr8Message(
  const BevFrame & frame,
  const std::string & frame_id)
{
  if (frame.image.type() != CV_8UC3) {
    throw std::invalid_argument("BEV output must be a BGR8 image");
  }

  auto message = std::make_unique<sensor_msgs::msg::Image>();
  message->header = frame.header;
  message->header.frame_id = frame_id;
  message->height = static_cast<std::uint32_t>(frame.image.rows);
  message->width = static_cast<std::uint32_t>(frame.image.cols);
  message->encoding = sensor_msgs::image_encodings::BGR8;
  message->is_bigendian = false;
  message->step = static_cast<std::uint32_t>(frame.image.cols * 3);
  message->data.resize(
    static_cast<std::size_t>(message->step) *
    static_cast<std::size_t>(message->height));

  for (int row = 0; row < frame.image.rows; ++row) {
    std::memcpy(
      message->data.data() +
      static_cast<std::size_t>(row) * message->step,
      frame.image.ptr(row),
      message->step);
  }
  return message;
}

}  // namespace

class BevProcessorNode final : public rclcpp::Node
{
public:
  explicit BevProcessorNode(const rclcpp::NodeOptions & options)
  : Node("bev_processor", options)
  {
    declareParameters();
    readParameters();
    validateParameters();

    RCLCPP_INFO(
      get_logger(),
      "Measuring startup camera height from the OAK stereo ground plane "
      "and roll/pitch from the configured '%s' source. "
      "Keep the vehicle stationary and the center view on flat ground.",
      startupAttitudeSourceName(startup_measurement_config_.attitude_source));
    RCLCPP_INFO(
      get_logger(),
      "Measurement quality: warmup=%.1fs, IR-dot=%.2f, IMU=%d samples, "
      "depth ROI=%dx%d step=%d (%d valid points minimum), "
      "RANSAC=%d iterations, stable planes=%d frames, attitude=%s.",
      startup_measurement_config_.warmup_sec,
      startup_measurement_config_.ir_dot_projector_intensity,
      startup_measurement_config_.imu_sample_count,
      startup_measurement_config_.roi_width,
      startup_measurement_config_.roi_height,
      startup_measurement_config_.point_sample_step,
      startup_measurement_config_.minimum_valid_points,
      startup_measurement_config_.plane_ransac_iterations,
      startup_measurement_config_.stable_plane_frame_count,
      startupAttitudeSourceName(startup_measurement_config_.attitude_source));
    const auto measurement =
      measureOakStartupExtrinsics(startup_measurement_config_);
    camera_model_.position_vehicle_m[2] = measurement.height_m;
    startup_roll_deg_ = measurement.roll_deg;
    startup_pitch_down_deg_ = measurement.pitch_down_deg;
    RCLCPP_INFO(
      get_logger(),
      "BEV_STARTUP_MEASUREMENT: source=%s, "
      "height=%.4fm, roll=%.3fdeg, "
      "pitch=%.3fdeg, downward_pitch=%.3fdeg, "
      "height_stddev=%.4fm, plane_normal_RMS=%.3fdeg",
      measurement.attitude_source.c_str(),
      measurement.height_m,
      measurement.roll_deg,
      -measurement.pitch_down_deg,
      measurement.pitch_down_deg,
      measurement.height_stddev_m,
      measurement.plane_normal_rms_deg);
    RCLCPP_INFO(
      get_logger(),
      "Startup IMU: raw=(roll=%.3f,pitch_down=%.3fdeg), "
      "corrected=(roll=%.3f,pitch_down=%.3fdeg), "
      "direction_RMS=%.3fdeg",
      measurement.imu_roll_deg,
      measurement.imu_pitch_down_deg,
      measurement.corrected_imu_roll_deg,
      measurement.corrected_imu_pitch_down_deg,
      measurement.imu_direction_rms_deg);
    RCLCPP_INFO(
      get_logger(),
      "Startup attitude selection: selected=%s, "
      "IMU corrected=(roll=%.3f,pitch_down=%.3fdeg), "
      "depth=(roll=%.3f,pitch_down=%.3fdeg), difference=%.3fdeg",
      measurement.attitude_source.c_str(),
      measurement.corrected_imu_roll_deg,
      measurement.corrected_imu_pitch_down_deg,
      measurement.depth_roll_deg,
      measurement.depth_pitch_down_deg,
      measurement.plane_imu_difference_deg);
    RCLCPP_INFO(
      get_logger(),
      "Startup ground-plane diagnostics: depth=%.3fm, "
      "points=%zu, inliers=%zu (%.1f%%), residual_MAD=%.4fm",
      measurement.median_depth_m,
      measurement.valid_point_count,
      measurement.plane_inlier_count,
      100.0 * measurement.plane_inlier_ratio,
      measurement.plane_residual_mad_m);
    installProcessor(
      startup_roll_deg_,
      startup_pitch_down_deg_,
      "startup depth height + selected attitude");

    const auto image_qos = rclcpp::SensorDataQoS().keep_last(1);
    input_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic_,
      image_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        onImage(std::move(message));
      });
    if (publish_enabled_) {
      output_publisher_ = create_publisher<sensor_msgs::msg::Image>(
        output_topic_, image_qos);
    }

    processing_thread_ = std::thread(&BevProcessorNode::processingLoop, this);
    if (publish_enabled_) {
      publishing_thread_ = std::thread(&BevProcessorNode::publishingLoop, this);
    }
    if (preview_enabled_) {
      if (graphicalDisplayAvailable()) {
        preview_thread_ = std::thread(&BevProcessorNode::previewLoop, this);
      } else {
        preview_enabled_ = false;
        RCLCPP_WARN(
          get_logger(),
          "Preview disabled because DISPLAY/WAYLAND_DISPLAY is unavailable.");
      }
    }

    status_started_at_ = SteadyClock::now();
    status_timer_ = create_wall_timer(
      std::chrono::duration<double>(status_log_interval_sec_),
      std::bind(&BevProcessorNode::logStatus, this));
    const auto startup_processor = std::atomic_load_explicit(
      &gpu_processor_, std::memory_order_acquire);
    RCLCPP_INFO(
      get_logger(),
      "==================== BEV PROCESSOR START ====================");
    RCLCPP_INFO(
      get_logger(),
      "BEV processor started: input=%s "
      "(%dx%d NV12, expected=%.1fHz), "
      "output=%s (%dx%d), "
      "range X=[%.2f, %.2f]m Y=[%.2f, %.2f]m, %.3fm/px, "
      "camera=(x=%.3f, y=%.3f, z=%.3fm, "
      "roll=%.2f, pitch_down=%.2f, yaw=%.2fdeg), "
      "valid_lut=%.2f%%, GPU=%s, processing=NV12-to-BEV/latest-only, "
      "ROS=%s (max=%.1fHz, 0=unlimited), preview=%s (max=%.1fHz)",
      input_topic_.c_str(),
      camera_model_.image_width,
      camera_model_.image_height,
      expected_input_fps_,
      output_topic_.c_str(),
      bev_config_.output_width,
      bev_config_.output_height,
      bev_config_.x_min_m,
      bev_config_.x_max_m,
      bev_config_.y_min_m,
      bev_config_.y_max_m,
      bev_config_.meter_per_pixel,
      camera_model_.position_vehicle_m[0],
      camera_model_.position_vehicle_m[1],
      camera_model_.position_vehicle_m[2],
      applied_roll_deg_.load(std::memory_order_relaxed),
      applied_pitch_down_deg_.load(std::memory_order_relaxed),
      camera_yaw_deg_,
      valid_lut_percent_.load(std::memory_order_relaxed),
      startup_processor->deviceName().c_str(),
      publish_enabled_ ? "on" : "off",
      publish_max_fps_,
      preview_enabled_ ? "on" : "off",
      preview_max_fps_);
    RCLCPP_INFO(
      get_logger(),
      "Startup extrinsics: height_source=depth, attitude_source=%s, "
      "height=%.4fm, roll=%.3fdeg, "
      "pitch=%.3fdeg, downward_pitch=%.3fdeg, fixed yaw=%.3fdeg. "
      "The LUT remains fixed after this measurement.",
      measurement.attitude_source.c_str(),
      camera_model_.position_vehicle_m[2],
      applied_roll_deg_.load(std::memory_order_relaxed),
      -applied_pitch_down_deg_.load(std::memory_order_relaxed),
      applied_pitch_down_deg_.load(std::memory_order_relaxed),
      camera_yaw_deg_);
    RCLCPP_INFO(
      get_logger(),
      "============================================================");
  }

  ~BevProcessorNode() override
  {
    stop_.store(true, std::memory_order_release);
    input_cv_.notify_all();
    output_cv_.notify_all();

    if (processing_thread_.joinable()) {
      processing_thread_.join();
    }
    if (publishing_thread_.joinable()) {
      publishing_thread_.join();
    }
    if (preview_thread_.joinable()) {
      preview_thread_.join();
    }
  }

private:
  void declareParameters()
  {
    // A missing or node-name-mismatched YAML must not fall back silently to
    // C++ defaults because the measured pose and BEV bounds are safety-critical.
    declare_parameter<int>("configuration_version", 0);

    declare_parameter<std::string>("input_topic", "/camera/image_rect");
    declare_parameter<std::string>("output_topic", "/camera/image_bev");
    declare_parameter<std::string>("output_frame_id", "front_axle_bev");
    declare_parameter<double>("expected_input_fps", 120.0);

    declare_parameter<bool>("publish_enabled", true);
    declare_parameter<double>("publish_max_fps", 0.0);
    declare_parameter<bool>("preview_enabled", true);
    declare_parameter<double>("preview_max_fps", 30.0);
    declare_parameter<std::string>("preview_window_name", "BEV image");
    declare_parameter<int>("preview_max_width", 1280);
    declare_parameter<int>("preview_max_height", 720);

    declare_parameter<int>("input_width", 1280);
    declare_parameter<int>("input_height", 720);
    declare_parameter<double>("fx", 561.400939941);
    declare_parameter<double>("fy", 561.136352539);
    declare_parameter<double>("cx", 643.032653809);
    declare_parameter<double>("cy", 352.621124268);

    declare_parameter<double>("camera_x_m", 0.0);
    declare_parameter<double>("camera_y_m", 0.0);
    declare_parameter<double>("camera_yaw_deg", 0.0);

    declare_parameter<double>("measurement_stereo_fps", 30.0);
    declare_parameter<int>("measurement_stereo_width", 640);
    declare_parameter<int>("measurement_stereo_height", 400);
    declare_parameter<int>("measurement_depth_queue_size", 2);
    declare_parameter<double>("measurement_imu_rate_hz", 400.0);
    declare_parameter<int>("measurement_imu_queue_size", 200);
    declare_parameter<double>("measurement_warmup_sec", 1.0);
    declare_parameter<double>(
      "measurement_ir_dot_projector_intensity", 1.0);
    declare_parameter<int>("measurement_roi_width", 228);
    declare_parameter<int>("measurement_roi_height", 114);
    declare_parameter<int>("measurement_point_sample_step", 2);
    declare_parameter<int>("measurement_minimum_valid_points", 1270);
    declare_parameter<double>("measurement_minimum_depth_m", 0.30);
    declare_parameter<double>("measurement_maximum_depth_m", 3.00);
    declare_parameter<double>("measurement_minimum_height_m", 0.10);
    declare_parameter<double>("measurement_maximum_height_m", 1.00);
    declare_parameter<int>("measurement_plane_ransac_iterations", 200);
    declare_parameter<double>(
      "measurement_plane_inlier_threshold_m", 0.008);
    declare_parameter<int>("measurement_plane_minimum_inliers", 914);
    declare_parameter<double>(
      "measurement_plane_minimum_inlier_ratio", 0.70);
    declare_parameter<double>(
      "measurement_plane_maximum_residual_mad_m", 0.005);
    declare_parameter<double>(
      "measurement_plane_maximum_imu_difference_deg", 15.0);
    declare_parameter<std::string>("measurement_attitude_source", "depth");
    declare_parameter<double>("measurement_imu_roll_bias_deg", 0.0);
    declare_parameter<double>("measurement_imu_pitch_bias_deg", 0.0);
    declare_parameter<int>("measurement_imu_sample_count", 800);
    declare_parameter<double>(
      "measurement_imu_max_direction_rms_deg", 0.25);
    declare_parameter<double>("measurement_imu_accel_min_mps2", 7.50);
    declare_parameter<double>("measurement_imu_accel_max_mps2", 12.00);
    declare_parameter<int>("measurement_stable_plane_frame_count", 30);
    declare_parameter<double>("measurement_maximum_height_stddev_m", 0.004);
    declare_parameter<double>(
      "measurement_maximum_plane_normal_rms_deg", 0.25);
    declare_parameter<double>("measurement_timeout_sec", 30.0);

    declare_parameter<double>("x_min_m", 0.18);
    declare_parameter<double>("x_max_m", 3.0);
    declare_parameter<double>("y_min_m", -1.0);
    declare_parameter<double>("y_max_m", 1.0);
    declare_parameter<double>("meter_per_pixel", 0.01);
    declare_parameter<int>("output_width", 200);
    declare_parameter<int>("output_height", 282);

    declare_parameter<double>("status_log_interval_sec", 5.0);
    declare_parameter<double>("startup_timeout_sec", 5.0);
  }

  void readParameters()
  {
    configuration_version_ = static_cast<int>(
      get_parameter("configuration_version").as_int());

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    output_frame_id_ = get_parameter("output_frame_id").as_string();
    expected_input_fps_ = get_parameter("expected_input_fps").as_double();

    publish_enabled_ = get_parameter("publish_enabled").as_bool();
    publish_max_fps_ = get_parameter("publish_max_fps").as_double();
    preview_enabled_ = get_parameter("preview_enabled").as_bool();
    preview_max_fps_ = get_parameter("preview_max_fps").as_double();
    preview_window_name_ = get_parameter("preview_window_name").as_string();
    preview_max_width_ =
      static_cast<int>(get_parameter("preview_max_width").as_int());
    preview_max_height_ =
      static_cast<int>(get_parameter("preview_max_height").as_int());

    camera_model_.fx = get_parameter("fx").as_double();
    camera_model_.fy = get_parameter("fy").as_double();
    camera_model_.cx = get_parameter("cx").as_double();
    camera_model_.cy = get_parameter("cy").as_double();
    camera_model_.image_width =
      static_cast<int>(get_parameter("input_width").as_int());
    camera_model_.image_height =
      static_cast<int>(get_parameter("input_height").as_int());
    camera_model_.position_vehicle_m = cv::Vec3d(
      get_parameter("camera_x_m").as_double(),
      get_parameter("camera_y_m").as_double(),
      0.0);
    camera_yaw_deg_ = get_parameter("camera_yaw_deg").as_double();
    camera_model_.rotation_vehicle_from_camera =
      mountRotationVehicleFromCamera(
      0.0,
      0.0,
      degToRad(camera_yaw_deg_));

    startup_measurement_config_.stereo_fps =
      get_parameter("measurement_stereo_fps").as_double();
    startup_measurement_config_.stereo_width = static_cast<int>(
      get_parameter("measurement_stereo_width").as_int());
    startup_measurement_config_.stereo_height = static_cast<int>(
      get_parameter("measurement_stereo_height").as_int());
    startup_measurement_config_.depth_queue_size = static_cast<int>(
      get_parameter("measurement_depth_queue_size").as_int());
    startup_measurement_config_.imu_rate_hz =
      get_parameter("measurement_imu_rate_hz").as_double();
    startup_measurement_config_.imu_queue_size = static_cast<int>(
      get_parameter("measurement_imu_queue_size").as_int());
    startup_measurement_config_.warmup_sec =
      get_parameter("measurement_warmup_sec").as_double();
    startup_measurement_config_.ir_dot_projector_intensity =
      get_parameter(
      "measurement_ir_dot_projector_intensity").as_double();
    startup_measurement_config_.roi_width = static_cast<int>(
      get_parameter("measurement_roi_width").as_int());
    startup_measurement_config_.roi_height = static_cast<int>(
      get_parameter("measurement_roi_height").as_int());
    startup_measurement_config_.point_sample_step = static_cast<int>(
      get_parameter("measurement_point_sample_step").as_int());
    startup_measurement_config_.minimum_valid_points = static_cast<int>(
      get_parameter("measurement_minimum_valid_points").as_int());
    startup_measurement_config_.minimum_depth_m =
      get_parameter("measurement_minimum_depth_m").as_double();
    startup_measurement_config_.maximum_depth_m =
      get_parameter("measurement_maximum_depth_m").as_double();
    startup_measurement_config_.minimum_height_m =
      get_parameter("measurement_minimum_height_m").as_double();
    startup_measurement_config_.maximum_height_m =
      get_parameter("measurement_maximum_height_m").as_double();
    startup_measurement_config_.plane_ransac_iterations = static_cast<int>(
      get_parameter("measurement_plane_ransac_iterations").as_int());
    startup_measurement_config_.plane_inlier_threshold_m =
      get_parameter("measurement_plane_inlier_threshold_m").as_double();
    startup_measurement_config_.plane_minimum_inliers = static_cast<int>(
      get_parameter("measurement_plane_minimum_inliers").as_int());
    startup_measurement_config_.plane_minimum_inlier_ratio =
      get_parameter("measurement_plane_minimum_inlier_ratio").as_double();
    startup_measurement_config_.plane_maximum_residual_mad_m =
      get_parameter(
      "measurement_plane_maximum_residual_mad_m").as_double();
    startup_measurement_config_.plane_maximum_imu_difference_deg =
      get_parameter(
      "measurement_plane_maximum_imu_difference_deg").as_double();
    startup_measurement_config_.attitude_source =
      parseStartupAttitudeSource(
      get_parameter("measurement_attitude_source").as_string());
    startup_measurement_config_.imu_roll_bias_deg =
      get_parameter("measurement_imu_roll_bias_deg").as_double();
    startup_measurement_config_.imu_pitch_bias_deg =
      get_parameter("measurement_imu_pitch_bias_deg").as_double();
    startup_measurement_config_.imu_sample_count = static_cast<int>(
      get_parameter("measurement_imu_sample_count").as_int());
    startup_measurement_config_.imu_max_direction_rms_deg =
      get_parameter(
      "measurement_imu_max_direction_rms_deg").as_double();
    startup_measurement_config_.imu_accel_min_mps2 =
      get_parameter("measurement_imu_accel_min_mps2").as_double();
    startup_measurement_config_.imu_accel_max_mps2 =
      get_parameter("measurement_imu_accel_max_mps2").as_double();
    startup_measurement_config_.stable_plane_frame_count = static_cast<int>(
      get_parameter("measurement_stable_plane_frame_count").as_int());
    startup_measurement_config_.maximum_height_stddev_m =
      get_parameter(
      "measurement_maximum_height_stddev_m").as_double();
    startup_measurement_config_.maximum_plane_normal_rms_deg =
      get_parameter(
      "measurement_maximum_plane_normal_rms_deg").as_double();
    startup_measurement_config_.timeout_sec =
      get_parameter("measurement_timeout_sec").as_double();

    bev_config_.x_min_m = get_parameter("x_min_m").as_double();
    bev_config_.x_max_m = get_parameter("x_max_m").as_double();
    bev_config_.y_min_m = get_parameter("y_min_m").as_double();
    bev_config_.y_max_m = get_parameter("y_max_m").as_double();
    bev_config_.meter_per_pixel =
      get_parameter("meter_per_pixel").as_double();
    bev_config_.output_width =
      static_cast<int>(get_parameter("output_width").as_int());
    bev_config_.output_height =
      static_cast<int>(get_parameter("output_height").as_int());

    status_log_interval_sec_ =
      get_parameter("status_log_interval_sec").as_double();
    startup_timeout_sec_ = get_parameter("startup_timeout_sec").as_double();
  }

  void validateParameters() const
  {
    if (configuration_version_ != 1) {
      throw std::invalid_argument(
              "configuration_version must be 1; check that bev_config.yaml "
              "was loaded for the bev_processor node");
    }
    if (input_topic_.empty()) {
      throw std::invalid_argument("input_topic must not be empty");
    }
    if (publish_enabled_ && output_topic_.empty()) {
      throw std::invalid_argument(
              "output_topic must not be empty when publishing is enabled");
    }
    if (
      camera_model_.image_width <= 1 ||
      camera_model_.image_height <= 1 ||
      camera_model_.fx <= 0.0 ||
      camera_model_.fy <= 0.0)
    {
      throw std::invalid_argument(
              "input dimensions and focal lengths must be positive");
    }
    if (
      bev_config_.x_min_m < 0.0 ||
      bev_config_.x_max_m <= bev_config_.x_min_m ||
      bev_config_.y_max_m <= bev_config_.y_min_m ||
      bev_config_.meter_per_pixel <= 0.0 ||
      bev_config_.output_width <= 0 ||
      bev_config_.output_height <= 0)
    {
      throw std::invalid_argument("invalid BEV bounds");
    }
    const int expected_width = static_cast<int>(std::llround(
        (bev_config_.y_max_m - bev_config_.y_min_m) /
        bev_config_.meter_per_pixel));
    const int expected_height = static_cast<int>(std::llround(
        (bev_config_.x_max_m - bev_config_.x_min_m) /
        bev_config_.meter_per_pixel));
    if (
      bev_config_.output_width != expected_width ||
      bev_config_.output_height != expected_height)
    {
      throw std::invalid_argument(
              "output_width/output_height do not match BEV bounds and "
              "meter_per_pixel");
    }
    if (
      expected_input_fps_ <= 0.0 ||
      publish_max_fps_ < 0.0 ||
      preview_max_fps_ <= 0.0 ||
      preview_max_width_ <= 0 ||
      preview_max_height_ <= 0 ||
      status_log_interval_sec_ <= 0.0 ||
      startup_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("invalid rate, preview, or status parameter");
    }
  }

  void installProcessor(
    const double roll_deg,
    const double pitch_down_deg,
    const char * source)
  {
    auto camera_model = camera_model_;
    camera_model.rotation_vehicle_from_camera =
      mountRotationVehicleFromCamera(
      degToRad(roll_deg),
      degToRad(pitch_down_deg),
      degToRad(camera_yaw_deg_));
    const auto lut = generateRemap(camera_model, bev_config_);
    auto processor = std::make_shared<CudaBevProcessor>(
      camera_model.image_width,
      camera_model.image_height,
      lut.map_x,
      lut.map_y);

    const int valid_pixels = cv::countNonZero(lut.valid_mask);
    const int output_pixels =
      bev_config_.output_width * bev_config_.output_height;
    const double valid_percent =
      100.0 * static_cast<double>(valid_pixels) /
      static_cast<double>(output_pixels);

    std::atomic_store_explicit(
      &gpu_processor_, std::move(processor), std::memory_order_release);
    valid_lut_percent_.store(valid_percent, std::memory_order_relaxed);
    applied_roll_deg_.store(roll_deg, std::memory_order_relaxed);
    applied_pitch_down_deg_.store(
      pitch_down_deg, std::memory_order_relaxed);

    if (source != nullptr) {
      RCLCPP_INFO(
        get_logger(),
        "BEV LUT installed from %s: height=%.3fm, roll=%.3f deg, "
        "pitch_down=%.3f deg, fixed yaw=%.3f deg, valid=%.2f%%",
        source, camera_model.position_vehicle_m[2],
        roll_deg, pitch_down_deg, camera_yaw_deg_, valid_percent);
    }
  }

  void onImage(sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    received_total_.fetch_add(1U, std::memory_order_relaxed);
    received_interval_.fetch_add(1U, std::memory_order_relaxed);

    const bool dimensions_valid =
      static_cast<int>(message->width) == camera_model_.image_width &&
      static_cast<int>(message->height) == camera_model_.image_height;
    const std::size_t minimum_step =
      static_cast<std::size_t>(camera_model_.image_width);
    const std::size_t nv12_rows =
      static_cast<std::size_t>(camera_model_.image_height) * 3U / 2U;
    const bool memory_valid =
      message->step >= minimum_step &&
      message->data.size() >=
      static_cast<std::size_t>(message->step) * nv12_rows;
    if (
      !dimensions_valid ||
      message->encoding != "nv12" ||
      !memory_valid)
    {
      invalid_total_.fetch_add(1U, std::memory_order_relaxed);
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Rejected image: expected %dx%d nv12, got %ux%u %s "
        "(step=%u, data=%zu).",
        camera_model_.image_width,
        camera_model_.image_height,
        message->width,
        message->height,
        message->encoding.c_str(),
        message->step,
        message->data.size());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      latest_input_ = std::move(message);
      latest_input_received_at_ = SteadyClock::now();
      ++input_generation_;
    }
    accepted_total_.fetch_add(1U, std::memory_order_relaxed);
    input_cv_.notify_one();
  }

  void processingLoop()
  {
    std::uint64_t processed_input_generation = 0U;

    while (!stop_.load(std::memory_order_acquire)) {
      sensor_msgs::msg::Image::ConstSharedPtr input;
      SteadyClock::time_point input_received_at;
      std::uint64_t generation = 0U;
      {
        std::unique_lock<std::mutex> lock(input_mutex_);
        input_cv_.wait(
          lock,
          [this, processed_input_generation]() {
            return
              stop_.load(std::memory_order_acquire) ||
              input_generation_ != processed_input_generation;
          });
        if (stop_.load(std::memory_order_acquire)) {
          break;
        }
        input = latest_input_;
        input_received_at = latest_input_received_at_;
        generation = input_generation_;
      }

      if (generation > processed_input_generation + 1U) {
        skipped_total_.fetch_add(
          generation - processed_input_generation - 1U,
          std::memory_order_relaxed);
        skipped_interval_.fetch_add(
          generation - processed_input_generation - 1U,
          std::memory_order_relaxed);
      }
      processed_input_generation = generation;

      try {
        const auto started_at = SteadyClock::now();
        auto output = std::make_shared<BevFrame>();
        const auto processor = std::atomic_load_explicit(
          &gpu_processor_, std::memory_order_acquire);
        output->image = processor->process(
          input->data.data(),
          input->data.size(),
          static_cast<std::size_t>(input->step));
        output->header = input->header;
        output->input_received_at = input_received_at;
        output->generation = generation;
        const auto finished_at = SteadyClock::now();

        const auto process_ns = static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            finished_at - started_at).count());
        process_ns_interval_.fetch_add(process_ns, std::memory_order_relaxed);
        updateMaximum(process_ns_max_interval_, process_ns);
        processed_total_.fetch_add(1U, std::memory_order_relaxed);
        processed_interval_.fetch_add(1U, std::memory_order_relaxed);

        {
          std::lock_guard<std::mutex> lock(output_mutex_);
          std::atomic_store_explicit(
            &latest_output_,
            std::shared_ptr<const BevFrame>(std::move(output)),
            std::memory_order_release);
        }
        output_cv_.notify_all();
      } catch (const std::exception & exception) {
        processing_error_total_.fetch_add(1U, std::memory_order_relaxed);
        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
          "BEV conversion failed: %s",
          exception.what());
      }
    }
  }

  void publishingLoop()
  {
    std::uint64_t last_generation = 0U;
    SteadyClock::time_point last_published_at{};

    while (!stop_.load(std::memory_order_acquire)) {
      const auto frame = waitForNewOutput(last_generation);
      if (!frame) {
        continue;
      }
      last_generation = frame->generation;

      const auto now = SteadyClock::now();
      if (
        publish_max_fps_ > 0.0 &&
        last_published_at.time_since_epoch().count() != 0)
      {
        const auto minimum_period =
          std::chrono::duration<double>(1.0 / publish_max_fps_);
        if (now - last_published_at < minimum_period) {
          publish_throttled_total_.fetch_add(1U, std::memory_order_relaxed);
          continue;
        }
      }

      try {
        output_publisher_->publish(
          makeBgr8Message(*frame, output_frame_id_));
        last_published_at = now;
        published_total_.fetch_add(1U, std::memory_order_relaxed);
        published_interval_.fetch_add(1U, std::memory_order_relaxed);
      } catch (const std::exception & exception) {
        publish_error_total_.fetch_add(1U, std::memory_order_relaxed);
        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
          "BEV publish failed: %s",
          exception.what());
      }
    }
  }

  static constexpr int kPreviewLeftMargin = 55;
  static constexpr int kPreviewRightMargin = 35;
  static constexpr int kPreviewTopMargin = 24;
  static constexpr int kPreviewBottomMargin = 30;

  static void drawPreviewText(
    cv::Mat & image,
    const std::string & text,
    const cv::Point & origin)
  {
    constexpr double font_scale = 0.32;
    constexpr int font_face = cv::FONT_HERSHEY_SIMPLEX;
    cv::putText(
      image, text, origin, font_face, font_scale,
      cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
  }

  cv::Mat makeCoordinatePreview(const cv::Mat & bev_image) const
  {
    constexpr double grid_step_m = 0.1;
    constexpr double label_step_m = 0.5;
    constexpr double epsilon = 1.0e-9;
    const cv::Scalar margin_color(245, 245, 245);
    const cv::Scalar grid_color(210, 210, 210);
    const cv::Scalar border_color(180, 180, 180);
    const cv::Scalar centerline_color(0, 200, 0);

    cv::Mat preview(
      bev_image.rows + kPreviewTopMargin + kPreviewBottomMargin,
      bev_image.cols + kPreviewLeftMargin + kPreviewRightMargin,
      CV_8UC3,
      margin_color);
    const cv::Rect bev_region(
      kPreviewLeftMargin,
      kPreviewTopMargin,
      bev_image.cols,
      bev_image.rows);
    bev_image.copyTo(preview(bev_region));
    cv::Mat displayed_bev = preview(bev_region);
    cv::Mat grid_overlay = displayed_bev.clone();

    const double first_x =
      std::ceil(bev_config_.x_min_m / grid_step_m) * grid_step_m;
    for (
      double x_m = first_x;
      x_m <= bev_config_.x_max_m + epsilon;
      x_m += grid_step_m)
    {
      const int row = std::clamp(
        static_cast<int>(std::lround(
          (bev_config_.x_max_m - x_m) /
          bev_config_.meter_per_pixel)),
        0,
        displayed_bev.rows - 1);
      cv::line(
        grid_overlay,
        cv::Point(0, row),
        cv::Point(displayed_bev.cols - 1, row),
        grid_color,
        1,
        cv::LINE_AA);
      const double label_units = x_m / label_step_m;
      if (std::abs(label_units - std::round(label_units)) < epsilon) {
        const std::string label = cv::format("X %.1f", x_m);
        int baseline = 0;
        const cv::Size label_size = cv::getTextSize(
          label, cv::FONT_HERSHEY_SIMPLEX, 0.32, 1, &baseline);
        drawPreviewText(
          preview,
          label,
          cv::Point(
            kPreviewLeftMargin - label_size.width - 5,
            kPreviewTopMargin + row + label_size.height / 2));
      }
    }

    cv::line(
      grid_overlay,
      cv::Point(0, displayed_bev.rows - 1),
      cv::Point(displayed_bev.cols - 1, displayed_bev.rows - 1),
      grid_color,
      1,
      cv::LINE_AA);
    const std::string minimum_x_label =
      cv::format("X %.2f", bev_config_.x_min_m);
    int minimum_x_baseline = 0;
    const cv::Size minimum_x_label_size = cv::getTextSize(
      minimum_x_label,
      cv::FONT_HERSHEY_SIMPLEX,
      0.32,
      1,
      &minimum_x_baseline);
    drawPreviewText(
      preview,
      minimum_x_label,
      cv::Point(
        kPreviewLeftMargin - minimum_x_label_size.width - 5,
        kPreviewTopMargin + displayed_bev.rows - 2));

    const double first_y =
      std::ceil(bev_config_.y_min_m / grid_step_m) * grid_step_m;
    for (
      double y_m = first_y;
      y_m <= bev_config_.y_max_m + epsilon;
      y_m += grid_step_m)
    {
      const int column = std::clamp(
        static_cast<int>(std::lround(
          (bev_config_.y_max_m - y_m) /
          bev_config_.meter_per_pixel)),
        0,
        displayed_bev.cols - 1);
      cv::line(
        grid_overlay,
        cv::Point(column, 0),
        cv::Point(column, displayed_bev.rows - 1),
        grid_color,
        1,
        cv::LINE_AA);
      const double label_units = y_m / label_step_m;
      if (std::abs(label_units - std::round(label_units)) < epsilon) {
        const std::string label = cv::format("Y %+.1f", y_m);
        int baseline = 0;
        const cv::Size label_size = cv::getTextSize(
          label, cv::FONT_HERSHEY_SIMPLEX, 0.32, 1, &baseline);
        drawPreviewText(
          preview,
          label,
          cv::Point(
            kPreviewLeftMargin + column - label_size.width / 2,
            kPreviewTopMargin + displayed_bev.rows + 16));
      }
    }

    cv::addWeighted(
      grid_overlay, 0.35, displayed_bev, 0.65, 0.0, displayed_bev);

    const int center_column = std::clamp(
      static_cast<int>(std::lround(
        bev_config_.y_max_m / bev_config_.meter_per_pixel)),
      0,
      displayed_bev.cols - 1);
    cv::line(
      displayed_bev,
      cv::Point(center_column, 0),
      cv::Point(center_column, displayed_bev.rows - 1),
      centerline_color,
      1,
      cv::LINE_AA);
    cv::rectangle(preview, bev_region, border_color, 1, cv::LINE_AA);

    const std::string direction_label = "+X forward / +Y left";
    int direction_baseline = 0;
    const cv::Size direction_size = cv::getTextSize(
      direction_label,
      cv::FONT_HERSHEY_SIMPLEX,
      0.32,
      1,
      &direction_baseline);
    drawPreviewText(
      preview,
      direction_label,
      cv::Point(
        kPreviewLeftMargin +
        (displayed_bev.cols - direction_size.width) / 2,
        15));
    return preview;
  }

  void previewLoop()
  {
    try {
      cv::namedWindow(preview_window_name_, cv::WINDOW_NORMAL);
      cv::resizeWindow(
        preview_window_name_,
        std::min(
          preview_max_width_,
          bev_config_.output_width +
          kPreviewLeftMargin + kPreviewRightMargin),
        std::min(
          preview_max_height_,
          bev_config_.output_height +
          kPreviewTopMargin + kPreviewBottomMargin));

      const auto preview_period =
        std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(1.0 / preview_max_fps_));
      auto next_preview_at = SteadyClock::now();
      bool window_was_visible = false;

      while (!stop_.load(std::memory_order_acquire)) {
        const auto now = SteadyClock::now();
        if (now < next_preview_at) {
          const auto remaining =
            std::chrono::duration_cast<std::chrono::milliseconds>(
            next_preview_at - now);
          const int key = cv::waitKey(
            std::clamp(static_cast<int>(remaining.count()), 1, 10));
          if (key == 27 || key == 'q' || key == 'Q') {
            break;
          }
          continue;
        }

        const auto frame = std::atomic_load_explicit(
          &latest_output_, std::memory_order_acquire);
        if (frame) {
          cv::Mat preview_image = makeCoordinatePreview(frame->image);
          cv::imshow(preview_window_name_, preview_image);
          previewed_total_.fetch_add(1U, std::memory_order_relaxed);
          previewed_interval_.fetch_add(1U, std::memory_order_relaxed);
          next_preview_at = now + preview_period;
        } else {
          next_preview_at = now + std::chrono::milliseconds(5);
        }

        const int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q') {
          break;
        }
        const double visible = cv::getWindowProperty(
          preview_window_name_, cv::WND_PROP_VISIBLE);
        if (visible >= 1.0) {
          window_was_visible = true;
        } else if (window_was_visible) {
          break;
        }
      }
    } catch (const cv::Exception & exception) {
      RCLCPP_WARN(
        get_logger(),
        "BEV preview disabled after OpenCV error: %s",
        exception.what());
    }

    try {
      cv::destroyWindow(preview_window_name_);
    } catch (const cv::Exception &) {
    }
    RCLCPP_INFO(
      get_logger(),
      "BEV preview closed; processing and ROS publishing continue.");
  }

  std::shared_ptr<const BevFrame> waitForNewOutput(
    const std::uint64_t last_generation)
  {
    std::unique_lock<std::mutex> lock(output_mutex_);
    output_cv_.wait(
      lock,
      [this, last_generation]() {
        const auto frame = std::atomic_load_explicit(
          &latest_output_, std::memory_order_acquire);
        return
          stop_.load(std::memory_order_acquire) ||
          (frame && frame->generation != last_generation);
      });
    if (stop_.load(std::memory_order_acquire)) {
      return nullptr;
    }
    return std::atomic_load_explicit(
      &latest_output_, std::memory_order_acquire);
  }

  static void updateMaximum(
    std::atomic<std::uint64_t> & target,
    const std::uint64_t candidate)
  {
    auto current = target.load(std::memory_order_relaxed);
    while (
      current < candidate &&
      !target.compare_exchange_weak(
        current,
        candidate,
        std::memory_order_relaxed,
        std::memory_order_relaxed))
    {
    }
  }

  void logStatus()
  {
    const auto now = SteadyClock::now();
    const double elapsed_sec =
      std::chrono::duration<double>(now - status_started_at_).count();
    status_started_at_ = now;

    const auto received =
      received_interval_.exchange(0U, std::memory_order_relaxed);
    const auto processed =
      processed_interval_.exchange(0U, std::memory_order_relaxed);
    const auto skipped =
      skipped_interval_.exchange(0U, std::memory_order_relaxed);
    const auto published =
      published_interval_.exchange(0U, std::memory_order_relaxed);
    const auto previewed =
      previewed_interval_.exchange(0U, std::memory_order_relaxed);
    const auto process_ns =
      process_ns_interval_.exchange(0U, std::memory_order_relaxed);
    const auto process_ns_max =
      process_ns_max_interval_.exchange(0U, std::memory_order_relaxed);

    double latest_age_ms = 0.0;
    const auto latest = std::atomic_load_explicit(
      &latest_output_, std::memory_order_acquire);
    if (latest) {
      latest_age_ms =
        std::chrono::duration<double, std::milli>(
        now - latest->input_received_at).count();
    }
    const double average_process_ms =
      processed > 0U ?
      static_cast<double>(process_ns) /
      static_cast<double>(processed) / 1.0e6 :
      0.0;
    RCLCPP_INFO(
      get_logger(),
      "\nBEV status: input=%.1fHz (%llu total), processed=%.1fHz "
      "(%llu total, skipped=%llu/%llu interval/total), "
      "ROS=%.1fHz, preview=%.1fHz, "
      "gpu=%.3f/%.3fms avg/max, latest_age=%.2fms, "
      "extrinsics=startup_measured, fixed_lut=true, "
      "(height=%.3fm,roll=%.2f,pitch_down=%.2fdeg), "
      "errors(invalid/process/publish)=%llu/%llu/%llu",
      static_cast<double>(received) / elapsed_sec,
      static_cast<unsigned long long>(
        received_total_.load(std::memory_order_relaxed)),
      static_cast<double>(processed) / elapsed_sec,
      static_cast<unsigned long long>(
        processed_total_.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(skipped),
      static_cast<unsigned long long>(
        skipped_total_.load(std::memory_order_relaxed)),
      static_cast<double>(published) / elapsed_sec,
      static_cast<double>(previewed) / elapsed_sec,
      average_process_ms,
      static_cast<double>(process_ns_max) / 1.0e6,
      latest_age_ms,
      camera_model_.position_vehicle_m[2],
      applied_roll_deg_.load(std::memory_order_relaxed),
      applied_pitch_down_deg_.load(std::memory_order_relaxed),
      static_cast<unsigned long long>(
        invalid_total_.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(
        processing_error_total_.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(
        publish_error_total_.load(std::memory_order_relaxed)));

    if (
      accepted_total_.load(std::memory_order_relaxed) == 0U &&
      std::chrono::duration<double>(now - node_started_at_).count() >=
      startup_timeout_sec_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "No valid input received on %s. Check that camera_driver publishing "
        "is enabled and the image is %dx%d nv12.",
        input_topic_.c_str(),
        camera_model_.image_width,
        camera_model_.image_height);
    }
  }

  int configuration_version_{0};
  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  double expected_input_fps_{120.0};
  bool publish_enabled_{true};
  double publish_max_fps_{0.0};
  bool preview_enabled_{true};
  double preview_max_fps_{30.0};
  std::string preview_window_name_;
  int preview_max_width_{1280};
  int preview_max_height_{720};
  double status_log_interval_sec_{5.0};
  double startup_timeout_sec_{5.0};
  double camera_yaw_deg_{0.0};
  OakStartupMeasurementConfig startup_measurement_config_{};

  RectifiedCameraModel camera_model_{};
  BevConfig bev_config_{};
  double startup_roll_deg_{0.0};
  double startup_pitch_down_deg_{14.0};
  std::atomic<double> valid_lut_percent_{0.0};
  std::atomic<double> applied_roll_deg_{0.0};
  std::atomic<double> applied_pitch_down_deg_{14.0};
  std::shared_ptr<CudaBevProcessor> gpu_processor_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr input_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr output_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::mutex input_mutex_;
  std::condition_variable input_cv_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_input_;
  SteadyClock::time_point latest_input_received_at_;
  std::uint64_t input_generation_{0U};

  std::mutex output_mutex_;
  std::condition_variable output_cv_;
  std::shared_ptr<const BevFrame> latest_output_;

  std::atomic<bool> stop_{false};
  std::thread processing_thread_;
  std::thread publishing_thread_;
  std::thread preview_thread_;

  const SteadyClock::time_point node_started_at_{SteadyClock::now()};
  SteadyClock::time_point status_started_at_{SteadyClock::now()};

  std::atomic<std::uint64_t> received_total_{0U};
  std::atomic<std::uint64_t> accepted_total_{0U};
  std::atomic<std::uint64_t> processed_total_{0U};
  std::atomic<std::uint64_t> skipped_total_{0U};
  std::atomic<std::uint64_t> published_total_{0U};
  std::atomic<std::uint64_t> previewed_total_{0U};
  std::atomic<std::uint64_t> publish_throttled_total_{0U};
  std::atomic<std::uint64_t> invalid_total_{0U};
  std::atomic<std::uint64_t> processing_error_total_{0U};
  std::atomic<std::uint64_t> publish_error_total_{0U};
  std::atomic<std::uint64_t> received_interval_{0U};
  std::atomic<std::uint64_t> processed_interval_{0U};
  std::atomic<std::uint64_t> skipped_interval_{0U};
  std::atomic<std::uint64_t> published_interval_{0U};
  std::atomic<std::uint64_t> previewed_interval_{0U};
  std::atomic<std::uint64_t> process_ns_interval_{0U};
  std::atomic<std::uint64_t> process_ns_max_interval_{0U};
};

}  // namespace bev_processor

RCLCPP_COMPONENTS_REGISTER_NODE(bev_processor::BevProcessorNode)
