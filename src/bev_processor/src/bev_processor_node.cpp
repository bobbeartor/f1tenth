#include <bev_handoff/runtime_monitor.hpp>
#include <bev_handoff/stage_meter.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

#include "bev_handoff/direct_bev_handoff.hpp"
#include "bev_processor/bev_geometry.hpp"
#include "bev_handoff/direct_obstacle_handoff.hpp"
#include "bev_processor/cuda_bev_processor.hpp"
#include "bev_processor/oak_startup_measurement.hpp"
#include "camera_driver/msg/bev_input.hpp"

namespace bev_processor
{

namespace
{

using SteadyClock = std::chrono::steady_clock;

using BevFrame = bev_handoff::DirectBevFrame;

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
  if (frame.bgr.type() != CV_8UC3) {
    throw std::invalid_argument("BEV output must be a BGR8 image");
  }

  auto message = std::make_unique<sensor_msgs::msg::Image>();
  message->header = frame.header;
  message->header.frame_id = frame_id;
  message->height = static_cast<std::uint32_t>(frame.bgr.rows);
  message->width = static_cast<std::uint32_t>(frame.bgr.cols);
  message->encoding = sensor_msgs::image_encodings::BGR8;
  message->is_bigendian = false;
  message->step = static_cast<std::uint32_t>(frame.bgr.cols * 3);
  message->data.resize(
    static_cast<std::size_t>(message->step) *
    static_cast<std::size_t>(message->height));

  for (int row = 0; row < frame.bgr.rows; ++row) {
    std::memcpy(
      message->data.data() +
      static_cast<std::size_t>(row) * message->step,
      frame.bgr.ptr(row),
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
    monitor_=std::make_unique<bev_handoff::RuntimeMonitor>(*this);
    declareParameters();
    readParameters();
    validateParameters();
    initializeInputCropGeometry();

    // Publish before the blocking measurement; a separate process owns the GUI.
    rclcpp::PublisherOptions startup_options;
    startup_options.use_intra_process_comm=rclcpp::IntraProcessSetting::Disable;
    startup_status_publisher_=create_publisher<std_msgs::msg::String>(
      "~/startup_status",rclcpp::QoS(1).reliable().transient_local(),startup_options);
    const bool startup_inspection=declare_parameter<bool>("startup_inspection_enabled",false);
    // remote_preview mode keeps the startup status text but publishes no camera image.
    const bool startup_preview_image=declare_parameter<bool>("startup_preview_image_enabled",true);
    // Remote preview stage 1b (BEV input received -> BEV ready), 1 s windows.
    perf_publisher_=create_publisher<std_msgs::msg::String>(
      "/perf/bev_stage",rclcpp::QoS(1).reliable(),startup_options);
    if (startup_inspection && startup_preview_image) {
      startup_image_publisher_=create_publisher<sensor_msgs::msg::Image>(
        "~/startup_preview",rclcpp::SensorDataQoS().keep_last(1),startup_options);
      startup_measurement_config_.roi_preview_enabled=false;
      startup_measurement_config_.preview_callback=[this](const cv::Mat & image) {
        const auto clock=SteadyClock::now();
        if (clock-last_startup_preview_<std::chrono::milliseconds(100)) {return;}
        last_startup_preview_=clock;
        cv::Mat resized;
        cv::resize(image,resized,cv::Size(640,400));
        sensor_msgs::msg::Image message;
        message.header.stamp=now(); message.header.frame_id="camera_optical_frame";
        message.height=resized.rows; message.width=resized.cols; message.encoding="bgr8";
        message.step=resized.cols*3;
        message.data.assign(resized.data,resized.data+resized.total()*resized.elemSize());
        startup_image_publisher_->publish(message);
      };
    }
    publishStartupStatus("MEASURING | keep stationary; checking camera attitude and ground");

    if (
      startup_measurement_config_.roi_preview_enabled &&
      !graphicalDisplayAvailable())
    {
      startup_measurement_config_.roi_preview_enabled = false;
      RCLCPP_WARN(
        get_logger(),
        "Startup measurement ROI preview disabled because "
        "DISPLAY/WAYLAND_DISPLAY is unavailable.");
    }

    if (startup_measurement_config_.manual_camera_height_enabled) {
      RCLCPP_INFO(
        get_logger(),
        "Using manual camera height %.4fm and measuring startup roll/pitch "
        "from the calibrated IMU. Keep the vehicle stationary.",
        startup_measurement_config_.manual_camera_height_m);
      RCLCPP_INFO(
        get_logger(),
        "Measurement quality: warmup=%.1fs, IMU=%d samples, "
        "depth/IR ground-plane measurement=disabled, attitude=imu.",
        startup_measurement_config_.warmup_sec,
        startup_measurement_config_.imu_sample_count);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Measuring startup camera height from the OAK stereo ground plane "
        "and roll/pitch from the configured '%s' source. "
        "Keep the vehicle stationary and the center view on flat ground.",
        startupAttitudeSourceName(startup_measurement_config_.attitude_source));
      RCLCPP_INFO(
        get_logger(),
        "Measurement quality: warmup=%.1fs, IR-dot=%.2f, IMU=%d samples, "
        "stereo=%dx%d@%.1fHz/5-bit-subpixel/shift=%d, "
        "depth ROI=%dx%d offset_y=%+dpx preview=%s step=%d "
        "(%d valid points minimum), "
        "RANSAC=%d iterations, stable planes=%d frames, attitude=%s.",
        startup_measurement_config_.warmup_sec,
        startup_measurement_config_.ir_dot_projector_intensity,
        startup_measurement_config_.imu_sample_count,
        startup_measurement_config_.stereo_width,
        startup_measurement_config_.stereo_height,
        startup_measurement_config_.stereo_fps,
        startup_measurement_config_.stereo_disparity_shift,
        startup_measurement_config_.roi_width,
        startup_measurement_config_.roi_height,
        startup_measurement_config_.roi_vertical_offset_px,
        startup_measurement_config_.roi_preview_enabled ? "on" : "off",
        startup_measurement_config_.point_sample_step,
        startup_measurement_config_.minimum_valid_points,
        startup_measurement_config_.plane_ransac_iterations,
        startup_measurement_config_.stable_plane_frame_count,
        startupAttitudeSourceName(startup_measurement_config_.attitude_source));
    }
    OakStartupMeasurement measurement;
    try {measurement=measureOakStartupExtrinsics(startup_measurement_config_);}
    catch (const std::exception & error) {
      publishStartupStatus(std::string("FAILED | ")+error.what()); throw;
    }
    startup_measurement_config_.preview_callback={};
    const bool share_obstacle_reference=get_parameter("obstacles.share_reference").as_bool();
    camera_model_.position_vehicle_m[2] = measurement.height_m;
    startup_roll_deg_ = measurement.roll_deg;
    startup_pitch_down_deg_ = measurement.pitch_down_deg;
    RCLCPP_INFO(
      get_logger(),
      "BEV_STARTUP_MEASUREMENT: height_source=%s, attitude_source=%s, "
      "height=%.4fm, roll=%.3fdeg, "
      "pitch=%.3fdeg, downward_pitch=%.3fdeg, "
      "height_stddev=%.4fm, plane_normal_RMS=%.3fdeg",
      measurement.height_source.c_str(),
      measurement.attitude_source.c_str(),
      measurement.height_m,
      measurement.roll_deg,
      -measurement.pitch_down_deg,
      measurement.pitch_down_deg,
      measurement.height_stddev_m,
      measurement.plane_normal_rms_deg);
    RCLCPP_INFO(
      get_logger(),
      "Startup IMU: measured=(roll=%.3f,pitch_down=%.3fdeg), "
      "corrected=(roll=%.3f,pitch_down=%.3fdeg), "
      "direction_RMS=%.3fdeg, gyro_mean/stddev=%.3f/%.3fdegps",
      measurement.imu_roll_deg,
      measurement.imu_pitch_down_deg,
      measurement.corrected_imu_roll_deg,
      measurement.corrected_imu_pitch_down_deg,
      measurement.imu_direction_rms_deg,
      measurement.imu_gyroscope_mean_degps,
      measurement.imu_gyroscope_stddev_degps);
    if (!startup_measurement_config_.manual_camera_height_enabled) {
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
    }
    installProcessor(
      startup_roll_deg_,
      startup_pitch_down_deg_,
      startup_measurement_config_.manual_camera_height_enabled ?
      "IMU attitude + manual camera height" :
      measurement.attitude_source == "depth" ?
      "depth-plane attitude + depth-plane offset height" :
      "IMU attitude + depth-plane offset height");

    if (share_obstacle_reference) {
      auto reference=std::make_shared<bev_handoff::ObstacleReference>();
      reference->device_id=measurement.device_id; reference->frame_id=output_frame_id_;
      reference->input_topic=input_topic_; reference->settle_sec=stabilization_settle_sec_;
      reference->rotation_vehicle_from_camera=mountRotationVehicleFromCamera(
        degToRad(startup_roll_deg_),degToRad(startup_pitch_down_deg_),degToRad(camera_yaw_deg_));
      reference->position_vehicle_m=camera_model_.position_vehicle_m;
      reference->fx=camera_model_.fx; reference->fy=camera_model_.fy;
      reference->cx=camera_model_.cx; reference->cy=camera_model_.cy;
      reference->source_width=camera_model_.image_width; reference->source_height=camera_model_.image_height;
      reference->x_min=bev_config_.x_min_m; reference->x_max=bev_config_.x_max_m;
      reference->y_min=bev_config_.y_min_m; reference->y_max=bev_config_.y_max_m;
      reference->meter_per_pixel=bev_config_.meter_per_pixel;
      reference->width=bev_config_.output_width; reference->height=bev_config_.output_height;
      bev_handoff::publishObstacleReference(std::move(reference));
    }

    auto reference_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    reference_qos.reliable().transient_local();
    // Humble's intra-process path cannot provide transient-local durability
    // to the camera driver that is loaded after this one-shot publication.
    rclcpp::PublisherOptions reference_publisher_options;
    reference_publisher_options.use_intra_process_comm =
      rclcpp::IntraProcessSetting::Disable;
    startup_ground_reference_publisher_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(
      startup_ground_reference_topic_, reference_qos,
      reference_publisher_options);
    publishStartupGroundReference(measurement.attitude_source);
    {
      std::ostringstream detail;
      detail<<"READY | height="<<measurement.height_m<<"m roll="<<measurement.roll_deg
        <<"deg pitch_down="<<measurement.pitch_down_deg<<"deg source="<<measurement.attitude_source
        <<" height_source="<<measurement.height_source;
      publishStartupStatus(detail.str());
    }

    const auto image_qos = rclcpp::SensorDataQoS().keep_last(1);
    if (declare_parameter<bool>("tunnel.enabled",false)) {
      rclcpp::SubscriptionOptions tunnel_options;
      tunnel_options.use_intra_process_comm=rclcpp::IntraProcessSetting::Disable;
      tunnel_compute_sub_=create_subscription<std_msgs::msg::String>(
        "/tunnel/compute_source",rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr message) {
          if (message->data=="IR") {tunnel_rgb_standby_=true;}
          else if (message->data=="RGB" || message->data=="BOTH") {tunnel_rgb_standby_=false;}
        },tunnel_options);
    }
    input_subscription_ = create_subscription<camera_driver::msg::BevInput>(
      input_topic_,
      image_qos,
      [this](camera_driver::msg::BevInput::ConstSharedPtr message) {
        onBevInput(std::move(message));
      });
    if (publish_enabled_) {
      output_publisher_ = create_publisher<sensor_msgs::msg::Image>(
        output_topic_, image_qos);
    }
    preview_stop_publisher_ = create_publisher<std_msgs::msg::Bool>(
      preview_stop_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    capture_joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
      capture_joy_topic_,
      rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::Joy::ConstSharedPtr message) {
        onCaptureJoy(std::move(message));
      });

    if (preview_enabled_ && !graphicalDisplayAvailable()) {
      preview_enabled_ = false;
      RCLCPP_WARN(
        get_logger(),
        "Preview disabled because DISPLAY/WAYLAND_DISPLAY is unavailable.");
    }
    if (
      dataset_collection_enabled_ ||
      dataset_collection_manual_capture_mode_)
    {
      initializeDatasetCollection();
    }
    processing_thread_ = std::thread(&BevProcessorNode::processingLoop, this);
    if (publish_enabled_) {
      publishing_thread_ = std::thread(&BevProcessorNode::publishingLoop, this);
    }
    if (preview_enabled_) {
      preview_thread_ = std::thread(&BevProcessorNode::previewLoop, this);
    }
    if (dataset_collection_enabled_) {
      dataset_collection_thread_ = std::thread(
        &BevProcessorNode::datasetCollectionLoop, this);
    }

    status_started_at_ = SteadyClock::now();
    status_timer_ = create_wall_timer(
      std::chrono::duration<double>(status_log_interval_sec_),
      std::bind(&BevProcessorNode::logStatus, this));
    perf_started_at_ = SteadyClock::now();
    perf_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
        const auto now = SteadyClock::now();
        const double elapsed = std::chrono::duration<double>(now - perf_started_at_).count();
        perf_started_at_ = now;
        std_msgs::msg::String message;
        message.data = "{\"input\":" + bev_input_meter_.drain(elapsed) +
          ",\"bev\":" + bev_stage_meter_.drain(elapsed) + "}";
        perf_publisher_->publish(message);
      });
    const auto startup_processor = std::atomic_load_explicit(
      &gpu_processor_, std::memory_order_acquire);
    RCLCPP_INFO(
      get_logger(),
      "==================== BEV PROCESSOR START ====================");
    RCLCPP_INFO(
      get_logger(),
      "BEV processor started: input=%s "
      "(%dx%d bottom crop of %dx%d, top=%dpx, expected=%.1fHz), "
      "output=%s (%dx%d), "
      "range X=[%.2f, %.2f]m Y=[%.2f, %.2f]m, %.3fm/px, "
      "camera=(x=%.3f, y=%.3f, z=%.3fm, "
      "roll=%.2f, pitch_down=%.2f, yaw=%.2fdeg), "
      "valid_lut=%.2f%%, GPU=%s, interpolation=%s, "
      "processing=NV12-to-BEV/latest-only, "
      "direct=%s (host_copy=%s), ROS=%s (max=%.1fHz, 0=unlimited), "
      "preview=%s (max=%.1fHz)",
      input_topic_.c_str(),
      input_crop_width_,
      input_crop_height_,
      camera_model_.image_width,
      camera_model_.image_height,
      input_crop_top_,
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
      bev_interpolation_.c_str(),
      direct_output_enabled_ ? "on" : "off",
      direct_host_copy_enabled_ ? "on" : "off",
      publish_enabled_ ? "on" : "off",
      publish_max_fps_,
      preview_enabled_ ? "on" : "off",
      preview_max_fps_);
    RCLCPP_INFO(
      get_logger(),
      "BEV capture: directory=%s, joy=%s button=%d (B), keyboard=B.",
      capture_directory_.c_str(),
      capture_joy_topic_.c_str(),
      capture_joy_button_);
    RCLCPP_INFO(
      get_logger(),
      "Startup extrinsics: height_source=%s, "
      "attitude_source=%s, "
      "height=%.4fm, roll=%.3fdeg, "
      "pitch=%.3fdeg, downward_pitch=%.3fdeg, fixed yaw=%.3fdeg. "
      "The LUT remains fixed after this measurement.",
      measurement.height_source.c_str(),
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
    if (dataset_collection_thread_.joinable()) {
      dataset_collection_thread_.join();
    }
  }

private:
  void declareParameters()
  {
    // A missing or node-name-mismatched YAML must not fall back silently to
    // C++ defaults because the measured pose and BEV bounds are safety-critical.
    declare_parameter<int>("configuration_version", 0);
    rcl_interfaces::msg::ParameterDescriptor obstacle_descriptor;
    obstacle_descriptor.read_only=true;
    declare_parameter<bool>("obstacles.share_reference",false,obstacle_descriptor);
    declare_parameter<bool>("perf_log_enabled", false);

    declare_parameter<std::string>("input_topic", "/camera/bev_input");
    declare_parameter<std::string>("output_topic", "/camera/image_bev");
    declare_parameter<std::string>("output_frame_id", "front_axle_bev");
    declare_parameter<std::string>(
      "startup_ground_reference_topic", "/camera/startup_ground_normal");
    declare_parameter<std::string>(
      "startup_ground_reference_frame_id", "camera_optical_frame");
    declare_parameter<double>("expected_input_fps", 70.0);
    declare_parameter<double>("input_bottom_fraction", 0.70);

    declare_parameter<bool>("publish_enabled", true);
    declare_parameter<bool>("direct_output_enabled", false);
    declare_parameter<bool>("direct_host_copy_enabled", false);
    declare_parameter<double>("publish_max_fps", 0.0);
    declare_parameter<bool>("preview_enabled", true);
    declare_parameter<double>("preview_max_fps", 60.0);
    declare_parameter<std::string>("preview_window_name", "BEV image");
    declare_parameter<std::string>("preview_stop_topic", "/auto/enabled");
    declare_parameter<int>("preview_max_width", 1280);
    declare_parameter<int>("preview_max_height", 800);
    declare_parameter<std::string>("capture_directory", ".");
    declare_parameter<std::string>("capture_joy_topic", "/joy");
    declare_parameter<int>("capture_joy_button", 1);
    declare_parameter<bool>("dataset_collection_enabled", false);
    declare_parameter<bool>(
      "dataset_collection_manual_capture_mode", false);
    declare_parameter<int>("dataset_collection_manual_capture_button", 0);
    declare_parameter<std::string>(
      "dataset_collection_root_directory", "datasets");
    declare_parameter<double>("dataset_collection_fps", 10.0);
    declare_parameter<int>("dataset_collection_target_count", 1000);
    declare_parameter<bool>("dataset_collection_stop_auto_on_complete", true);

    declare_parameter<int>("input_width", 1280);
    declare_parameter<int>("input_height", 800);
    declare_parameter<double>("fx", 701.751174926);
    declare_parameter<double>("fy", 701.420440674);
    declare_parameter<double>("cx", 643.032653809);
    declare_parameter<double>("cy", 392.621124268);

    declare_parameter<double>("camera_x_m", -0.16);
    declare_parameter<double>("camera_y_m", 0.0);
    declare_parameter<double>("camera_yaw_deg", 0.0);

    declare_parameter<double>("measurement_stereo_fps", 30.0);
    declare_parameter<int>("measurement_stereo_width", 1280);
    declare_parameter<int>("measurement_stereo_height", 800);
    declare_parameter<int>("measurement_depth_queue_size", 2);
    declare_parameter<int>("measurement_stereo_subpixel_fractional_bits", 5);
    declare_parameter<int>("measurement_stereo_left_right_check_threshold", 5);
    declare_parameter<int>("measurement_stereo_confidence_threshold", 55);
    declare_parameter<int>("measurement_stereo_disparity_shift", 0);
    declare_parameter<double>("measurement_imu_rate_hz", 400.0);
    declare_parameter<int>("measurement_imu_queue_size", 200);
    declare_parameter<int>("measurement_imu_max_batch_reports", 5);
    declare_parameter<double>("measurement_maximum_imu_pair_skew_sec", 0.003);
    declare_parameter<double>("measurement_warmup_sec", 2.0);
    declare_parameter<double>(
      "measurement_ir_dot_projector_intensity", 1.0);
    declare_parameter<bool>("manual_camera_height_enabled", false);
    declare_parameter<double>("manual_camera_height_m", 0.20);
    declare_parameter<int>("measurement_roi_width", 456);
    declare_parameter<int>("measurement_roi_height", 228);
    declare_parameter<int>("measurement_roi_vertical_offset_px", 80);
    declare_parameter<bool>("measurement_roi_preview_enabled", true);
    declare_parameter<int>("measurement_point_sample_step", 2);
    declare_parameter<int>("measurement_minimum_valid_points", 5080);
    declare_parameter<double>("measurement_minimum_depth_m", 0.30);
    declare_parameter<double>("measurement_maximum_depth_m", 3.00);
    declare_parameter<double>("measurement_minimum_height_m", 0.10);
    declare_parameter<double>("measurement_maximum_height_m", 0.40);
    declare_parameter<int>("measurement_plane_ransac_iterations", 200);
    declare_parameter<double>(
      "measurement_plane_inlier_threshold_m", 0.008);
    declare_parameter<int>("measurement_plane_minimum_inliers", 3656);
    declare_parameter<double>(
      "measurement_plane_minimum_inlier_ratio", 0.70);
    declare_parameter<double>(
      "measurement_plane_maximum_residual_mad_m", 0.005);
    declare_parameter<double>(
      "measurement_plane_maximum_imu_difference_deg", 5.0);
    declare_parameter<std::string>("measurement_attitude_source", "depth");
    declare_parameter<double>("measurement_imu_roll_bias_deg", 0.0);
    declare_parameter<double>("measurement_imu_pitch_bias_deg", 0.0);
    declare_parameter<int>("measurement_imu_sample_count", 1200);
    declare_parameter<double>(
      "measurement_imu_max_direction_rms_deg", 0.50);
    declare_parameter<double>("measurement_imu_accel_min_mps2", 8.30);
    declare_parameter<double>("measurement_imu_accel_max_mps2", 11.30);
    declare_parameter<double>(
      "measurement_imu_gyroscope_mean_maximum_degps", 0.80);
    declare_parameter<double>(
      "measurement_imu_gyroscope_stddev_maximum_degps", 1.40);
    declare_parameter<int>("measurement_stable_plane_frame_count", 45);
    declare_parameter<double>("measurement_maximum_height_stddev_m", 0.003);
    declare_parameter<double>(
      "measurement_maximum_plane_normal_rms_deg", 0.25);
    declare_parameter<double>("measurement_timeout_sec", 45.0);

    declare_parameter<double>("x_min_m", 0.0);
    declare_parameter<double>("x_max_m", 3.0);
    declare_parameter<double>("y_min_m", -0.6);
    declare_parameter<double>("y_max_m", 0.6);
    declare_parameter<double>("meter_per_pixel", 0.01);
    declare_parameter<int>("output_width", 120);
    declare_parameter<int>("output_height", 300);
    declare_parameter<std::string>("bev_interpolation", "bilinear");
    declare_parameter<double>("edge_adaptive_start_x_m", 1.20);
    declare_parameter<double>("edge_adaptive_full_x_m", 2.00);
    declare_parameter<double>("edge_adaptive_strength", 0.75);
    declare_parameter<double>("edge_gradient_low", 8.0);
    declare_parameter<double>("edge_gradient_high", 32.0);
    declare_parameter<double>("edge_coherence_minimum", 0.30);
    declare_parameter<double>("edge_maximum_anisotropy", 3.0);

    declare_parameter<double>("status_log_interval_sec", 5.0);
    declare_parameter<double>("startup_timeout_sec", 12.0);
    declare_parameter<double>("stabilization_settle_sec", 5.5);
  }

  void readParameters()
  {
    configuration_version_ = static_cast<int>(
      get_parameter("configuration_version").as_int());
    perf_log_enabled_ = get_parameter("perf_log_enabled").as_bool();

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    output_frame_id_ = get_parameter("output_frame_id").as_string();
    startup_ground_reference_topic_ =
      get_parameter("startup_ground_reference_topic").as_string();
    startup_ground_reference_frame_id_ =
      get_parameter("startup_ground_reference_frame_id").as_string();
    expected_input_fps_ = get_parameter("expected_input_fps").as_double();
    input_bottom_fraction_ =
      get_parameter("input_bottom_fraction").as_double();

    publish_enabled_ = get_parameter("publish_enabled").as_bool();
    direct_output_enabled_ =
      get_parameter("direct_output_enabled").as_bool();
    direct_host_copy_enabled_ =
      get_parameter("direct_host_copy_enabled").as_bool();
    publish_max_fps_ = get_parameter("publish_max_fps").as_double();
    preview_enabled_ = get_parameter("preview_enabled").as_bool();
    preview_max_fps_ = get_parameter("preview_max_fps").as_double();
    preview_window_name_ = get_parameter("preview_window_name").as_string();
    preview_stop_topic_ = get_parameter("preview_stop_topic").as_string();
    preview_max_width_ =
      static_cast<int>(get_parameter("preview_max_width").as_int());
    preview_max_height_ =
      static_cast<int>(get_parameter("preview_max_height").as_int());
    capture_directory_ = get_parameter("capture_directory").as_string();
    capture_joy_topic_ = get_parameter("capture_joy_topic").as_string();
    capture_joy_button_ =
      static_cast<int>(get_parameter("capture_joy_button").as_int());
    dataset_collection_enabled_ =
      get_parameter("dataset_collection_enabled").as_bool();
    dataset_collection_manual_capture_mode_ =
      get_parameter("dataset_collection_manual_capture_mode").as_bool();
    dataset_collection_manual_capture_button_ = static_cast<int>(
      get_parameter("dataset_collection_manual_capture_button").as_int());
    dataset_collection_root_directory_ =
      get_parameter("dataset_collection_root_directory").as_string();
    dataset_collection_fps_ =
      get_parameter("dataset_collection_fps").as_double();
    dataset_collection_target_count_ =
      get_parameter("dataset_collection_target_count").as_int();
    dataset_collection_stop_auto_on_complete_ =
      get_parameter("dataset_collection_stop_auto_on_complete").as_bool();

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
    startup_measurement_config_.stereo_subpixel_fractional_bits =
      static_cast<int>(get_parameter(
        "measurement_stereo_subpixel_fractional_bits").as_int());
    startup_measurement_config_.stereo_left_right_check_threshold =
      static_cast<int>(get_parameter(
        "measurement_stereo_left_right_check_threshold").as_int());
    startup_measurement_config_.stereo_confidence_threshold =
      static_cast<int>(get_parameter(
        "measurement_stereo_confidence_threshold").as_int());
    startup_measurement_config_.stereo_disparity_shift =
      static_cast<int>(get_parameter(
        "measurement_stereo_disparity_shift").as_int());
    startup_measurement_config_.imu_rate_hz =
      get_parameter("measurement_imu_rate_hz").as_double();
    startup_measurement_config_.imu_queue_size = static_cast<int>(
      get_parameter("measurement_imu_queue_size").as_int());
    startup_measurement_config_.imu_max_batch_reports = static_cast<int>(
      get_parameter("measurement_imu_max_batch_reports").as_int());
    startup_measurement_config_.maximum_imu_pair_skew_sec =
      get_parameter("measurement_maximum_imu_pair_skew_sec").as_double();
    startup_measurement_config_.warmup_sec =
      get_parameter("measurement_warmup_sec").as_double();
    startup_measurement_config_.ir_dot_projector_intensity =
      get_parameter(
      "measurement_ir_dot_projector_intensity").as_double();
    startup_measurement_config_.manual_camera_height_enabled =
      get_parameter("manual_camera_height_enabled").as_bool();
    startup_measurement_config_.manual_camera_height_m =
      get_parameter("manual_camera_height_m").as_double();
    startup_measurement_config_.roi_width = static_cast<int>(
      get_parameter("measurement_roi_width").as_int());
    startup_measurement_config_.roi_height = static_cast<int>(
      get_parameter("measurement_roi_height").as_int());
    startup_measurement_config_.roi_vertical_offset_px = static_cast<int>(
      get_parameter("measurement_roi_vertical_offset_px").as_int());
    startup_measurement_config_.roi_preview_enabled =
      get_parameter("measurement_roi_preview_enabled").as_bool();
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
    startup_measurement_config_.imu_gyroscope_mean_maximum_degps =
      get_parameter(
      "measurement_imu_gyroscope_mean_maximum_degps").as_double();
    startup_measurement_config_.imu_gyroscope_stddev_maximum_degps =
      get_parameter(
      "measurement_imu_gyroscope_stddev_maximum_degps").as_double();
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
    bev_interpolation_ = get_parameter("bev_interpolation").as_string();
    edge_adaptive_config_.start_x_m =
      get_parameter("edge_adaptive_start_x_m").as_double();
    edge_adaptive_config_.full_x_m =
      get_parameter("edge_adaptive_full_x_m").as_double();
    edge_adaptive_config_.strength =
      get_parameter("edge_adaptive_strength").as_double();
    edge_adaptive_config_.gradient_low =
      get_parameter("edge_gradient_low").as_double();
    edge_adaptive_config_.gradient_high =
      get_parameter("edge_gradient_high").as_double();
    edge_adaptive_config_.coherence_minimum =
      get_parameter("edge_coherence_minimum").as_double();
    edge_adaptive_config_.maximum_anisotropy =
      get_parameter("edge_maximum_anisotropy").as_double();
    edge_adaptive_config_.bev_x_max_m = bev_config_.x_max_m;
    edge_adaptive_config_.meter_per_pixel = bev_config_.meter_per_pixel;

    status_log_interval_sec_ =
      get_parameter("status_log_interval_sec").as_double();
    startup_timeout_sec_ = get_parameter("startup_timeout_sec").as_double();
    stabilization_settle_sec_ =
      get_parameter("stabilization_settle_sec").as_double();
  }

  void validateParameters() const
  {
    if (configuration_version_ != 4) {
      throw std::invalid_argument(
              "configuration_version must be 4; check that bev_config.yaml "
              "was loaded for the bev_processor node");
    }
    if (input_topic_.empty()) {
      throw std::invalid_argument("input_topic must not be empty");
    }
    if (
      startup_ground_reference_topic_.empty() ||
      startup_ground_reference_frame_id_.empty())
    {
      throw std::invalid_argument(
              "startup ground reference topic and frame_id must not be empty");
    }
    if (publish_enabled_ && output_topic_.empty()) {
      throw std::invalid_argument(
              "output_topic must not be empty when publishing is enabled");
    }
    if (preview_stop_topic_.empty()) {
      throw std::invalid_argument("preview_stop_topic must not be empty");
    }
    if (
      capture_directory_.empty() || capture_joy_topic_.empty() ||
      capture_joy_button_ < 0)
    {
      throw std::invalid_argument(
              "capture directory/topic must not be empty and button must "
              "be non-negative");
    }
    if (dataset_collection_root_directory_.empty()) {
      throw std::invalid_argument(
              "dataset_collection_root_directory must not be empty");
    }
    if (dataset_collection_manual_capture_button_ < 0) {
      throw std::invalid_argument(
              "dataset_collection_manual_capture_button must be "
              "non-negative");
    }
    if (
      dataset_collection_enabled_ &&
      dataset_collection_manual_capture_mode_)
    {
      throw std::invalid_argument(
              "automatic and manual dataset collection modes cannot both "
              "be enabled");
    }
    if (
      !std::isfinite(dataset_collection_fps_) ||
      dataset_collection_fps_ <= 0.0)
    {
      throw std::invalid_argument(
              "dataset_collection_fps must be finite and positive");
    }
    if (dataset_collection_target_count_ <= 0) {
      throw std::invalid_argument(
              "dataset_collection_target_count must be positive");
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
      bev_interpolation_ != "bilinear" &&
      bev_interpolation_ != "bicubic" &&
      bev_interpolation_ != "adaptive")
    {
      throw std::invalid_argument(
              "bev_interpolation must be 'bilinear', 'bicubic', or "
              "'adaptive'");
    }
    if (
      edge_adaptive_config_.start_x_m < bev_config_.x_min_m ||
      edge_adaptive_config_.full_x_m <= edge_adaptive_config_.start_x_m ||
      edge_adaptive_config_.full_x_m > bev_config_.x_max_m ||
      edge_adaptive_config_.strength < 0.0 ||
      edge_adaptive_config_.strength > 1.0 ||
      edge_adaptive_config_.gradient_low < 0.0 ||
      edge_adaptive_config_.gradient_high <=
      edge_adaptive_config_.gradient_low ||
      edge_adaptive_config_.coherence_minimum < 0.0 ||
      edge_adaptive_config_.coherence_minimum >= 1.0 ||
      edge_adaptive_config_.maximum_anisotropy < 1.0)
    {
      throw std::invalid_argument("invalid edge-adaptive interpolation settings");
    }
    if (
      expected_input_fps_ <= 0.0 ||
      !std::isfinite(input_bottom_fraction_) ||
      input_bottom_fraction_ <= 0.0 ||
      input_bottom_fraction_ > 1.0 ||
      publish_max_fps_ < 0.0 ||
      preview_max_fps_ <= 0.0 ||
      preview_max_width_ <= 0 ||
      preview_max_height_ <= 0 ||
      status_log_interval_sec_ <= 0.0 ||
      startup_timeout_sec_ <= 0.0 ||
      !std::isfinite(stabilization_settle_sec_) ||
      stabilization_settle_sec_ < 0.0)
    {
      throw std::invalid_argument("invalid rate, preview, or status parameter");
    }
  }

  void initializeInputCropGeometry()
  {
    input_crop_width_ = camera_model_.image_width;
    input_crop_height_ = static_cast<int>(2 * std::llround(
        static_cast<double>(camera_model_.image_height) *
        input_bottom_fraction_ / 2.0));
    input_crop_height_ = std::clamp(
      input_crop_height_, 2, camera_model_.image_height);
    input_crop_top_ = camera_model_.image_height - input_crop_height_;
  }

  void publishStartupGroundReference(const std::string & attitude_source)
  {
    const cv::Vec3d up_camera = attitudeUpVector(
      startup_roll_deg_, startup_pitch_down_deg_);
    geometry_msgs::msg::Vector3Stamped message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = startup_ground_reference_frame_id_;
    message.vector.x = up_camera[0];
    message.vector.y = up_camera[1];
    message.vector.z = up_camera[2];
    startup_ground_reference_publisher_->publish(message);
    RCLCPP_INFO(
      get_logger(),
      "Published startup ground reference for camera stabilization: "
      "topic=%s frame=%s source=%s normal=(%.6f, %.6f, %.6f).",
      startup_ground_reference_topic_.c_str(),
      startup_ground_reference_frame_id_.c_str(),
      attitude_source.c_str(),
      up_camera[0], up_camera[1], up_camera[2]);
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
      input_crop_width_,
      input_crop_height_,
      lut.map_x,
      lut.map_y,
      bev_interpolation_ == "adaptive" ?
      BevInterpolation::Adaptive :
      bev_interpolation_ == "bicubic" ?
      BevInterpolation::Bicubic : BevInterpolation::Bilinear,
      edge_adaptive_config_);

    const int valid_pixels = cv::countNonZero(lut.valid_mask);
    const int output_pixels =
      bev_config_.output_width * bev_config_.output_height;
    const double valid_percent =
      100.0 * static_cast<double>(valid_pixels) /
      static_cast<double>(output_pixels);

    bev_lut_ = lut;
    fused_coverage_reported_ = false;
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

  void onBevInput(camera_driver::msg::BevInput::ConstSharedPtr message)
  {
    const auto now=SteadyClock::now();
    if (tunnel_rgb_standby_ && now-last_tunnel_rgb_input_<std::chrono::milliseconds(200)) {return;}
    last_tunnel_rgb_input_=now;
    received_total_.fetch_add(1U, std::memory_order_relaxed);
    received_interval_.fetch_add(1U, std::memory_order_relaxed);
    bev_input_meter_.add();

    const bool dimensions_valid =
      static_cast<int>(message->source_width) == camera_model_.image_width &&
      static_cast<int>(message->source_height) == camera_model_.image_height &&
      static_cast<int>(message->source_crop_top) == input_crop_top_ &&
      static_cast<int>(message->cropped_width) == input_crop_width_ &&
      static_cast<int>(message->cropped_height) == input_crop_height_;
    const std::size_t minimum_step =
      static_cast<std::size_t>(input_crop_width_);
    const std::size_t nv12_rows =
      static_cast<std::size_t>(input_crop_height_) * 3U / 2U;
    const bool memory_valid =
      message->step >= minimum_step &&
      message->nv12.size() >=
      static_cast<std::size_t>(message->step) * nv12_rows;
    if (!dimensions_valid || !memory_valid)
    {
      invalid_total_.fetch_add(1U, std::memory_order_relaxed);
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Rejected fused BEV input: expected source=%dx%d crop=top %dpx/%dx%d, "
        "got source=%ux%u crop=top %u/%ux%u (step=%u, data=%zu).",
        camera_model_.image_width,
        camera_model_.image_height,
        input_crop_top_,
        input_crop_width_,
        input_crop_height_,
        message->source_width,
        message->source_height,
        message->source_crop_top,
        message->cropped_width,
        message->cropped_height,
        message->step,
        message->nv12.size());
      return;
    }

    const auto received_at = SteadyClock::now();
    if (!first_camera_input_seen_) {
      first_camera_input_seen_ = true;
      first_camera_input_at_ = received_at;
      RCLCPP_INFO(
        get_logger(),
        "First camera frame received; suppressing BEV output for %.1fs "
        "while the unchanged fixed-reference stabilizer calibrates.",
        stabilization_settle_sec_);
    }
    const double camera_elapsed_sec = std::chrono::duration<double>(
      received_at - first_camera_input_at_).count();
    if (camera_elapsed_sec < stabilization_settle_sec_) {
      stabilization_settle_total_.fetch_add(1U, std::memory_order_relaxed);
      return;
    }

    recordPipelineLatency(
      message->header,
      stabilized_latency_samples_interval_,
      stabilized_latency_ns_interval_,
      stabilized_latency_ns_max_interval_);
    accepted_interval_.fetch_add(1U, std::memory_order_relaxed);

    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      latest_input_ = std::move(message);
      latest_input_received_at_ = received_at;
      ++input_generation_;
    }
    accepted_total_.fetch_add(1U, std::memory_order_relaxed);
    input_cv_.notify_one();
  }

  void processingLoop()
  {
    std::uint64_t processed_input_generation = 0U;

    while (!stop_.load(std::memory_order_acquire)) {
      camera_driver::msg::BevInput::ConstSharedPtr input;
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
        bev_handoff::RuntimeSample runtime_sample(monitor_.get(),"bev_transform");
        const auto started_at = SteadyClock::now();
        auto output = std::make_shared<BevFrame>();
        const auto processor = std::atomic_load_explicit(
          &gpu_processor_, std::memory_order_acquire);
        cv::Matx33d source_to_stabilized;
        for (int row = 0; row < 3; ++row) {
          for (int column = 0; column < 3; ++column) {
            source_to_stabilized(row, column) =
              input->source_to_stabilized_homography[
              static_cast<std::size_t>(row * 3 + column)];
          }
        }
        const double determinant = cv::determinant(
          cv::Mat(source_to_stabilized));
        if (
          !cv::checkRange(cv::Mat(source_to_stabilized)) ||
          !std::isfinite(determinant) || std::abs(determinant) < 1.0e-9)
        {
          throw std::runtime_error(
                  "camera supplied an invalid stabilization homography");
        }
        const cv::Matx33d stabilized_to_source =
          source_to_stabilized.inv(cv::DECOMP_LU);
        if (!fused_coverage_reported_) {
          const auto coverage = assessFusedRemapCoverage(
            bev_lut_,
            source_to_stabilized,
            static_cast<int>(input->source_width),
            static_cast<int>(input->source_height),
            static_cast<int>(input->source_crop_top));
          const double coverage_percent = 100.0 * coverage.coverage_ratio;
          if (coverage.coverage_ratio < 0.995) {
            RCLCPP_WARN(
              get_logger(),
              "Fused bottom-crop coverage is %.2f%% (%d/%d static LUT "
              "pixels). Increase bev_input_bottom_fraction if the far BEV "
              "edge is black.",
              coverage_percent,
              coverage.covered_pixels,
              coverage.valid_lut_pixels);
          } else {
            RCLCPP_INFO(
              get_logger(),
              "Fused bottom-crop coverage: %.2f%% (%d/%d static LUT pixels).",
              coverage_percent,
              coverage.covered_pixels,
              coverage.valid_lut_pixels);
          }
          fused_coverage_reported_ = true;
        }
        CudaBevResult cuda_result = processor->process(
          input->nv12.data(),
          input->nv12.size(),
          static_cast<std::size_t>(input->step),
          stabilized_to_source,
          static_cast<int>(input->source_crop_top),
          publish_enabled_ || preview_enabled_ ||
          dataset_collection_enabled_ ||
          dataset_collection_manual_capture_mode_);
        output->bgr = std::move(cuda_result.bgr);
        if(direct_host_copy_enabled_)output->download_bgr=std::move(cuda_result.download_bgr);
        output->device_bgr = cuda_result.device_bgr;
        output->device_stride = cuda_result.device_stride;
        output->width = bev_config_.output_width;
        output->height = bev_config_.output_height;
        output->device_owner = std::move(cuda_result.device_owner);
        output->header = input->header;
        output->header.frame_id = output_frame_id_;
        output->bev_input_received_at = input_received_at;
        output->source_generation = generation;
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
        if (direct_output_enabled_) {
          const auto direct_frame = std::atomic_load_explicit(
            &latest_output_, std::memory_order_acquire);
          if (bev_handoff::deliverDirectBevFrame(direct_frame)) {
            const auto delivered_at = SteadyClock::now();
            direct_delivered_total_.fetch_add(1U, std::memory_order_relaxed);
            direct_delivered_interval_.fetch_add(1U, std::memory_order_relaxed);
            recordPipelineLatency(
              direct_frame->header,
              bev_ready_latency_samples_interval_,
              bev_ready_latency_ns_interval_,
              bev_ready_latency_ns_max_interval_);
            bev_stage_meter_.add(std::chrono::duration_cast<std::chrono::nanoseconds>(
              delivered_at - direct_frame->bev_input_received_at));
            recordSteadyLatency(
              delivered_at - direct_frame->bev_input_received_at,
              bev_stage_latency_samples_interval_,
              bev_stage_latency_ns_interval_,
              bev_stage_latency_ns_max_interval_);
          } else {
            direct_unavailable_total_.fetch_add(1U, std::memory_order_relaxed);
          }
        }
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
      last_generation = frame->source_generation;

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
        if (publish_enabled_) {
          output_publisher_->publish(
            makeBgr8Message(*frame, output_frame_id_));
        }
        const auto published_at = SteadyClock::now();
        if (!direct_output_enabled_) {
          bev_stage_meter_.add(std::chrono::duration_cast<std::chrono::nanoseconds>(
            published_at - frame->bev_input_received_at));
          recordPipelineLatency(
            frame->header,
            bev_ready_latency_samples_interval_,
            bev_ready_latency_ns_interval_,
            bev_ready_latency_ns_max_interval_);
          recordSteadyLatency(
            published_at - frame->bev_input_received_at,
            bev_stage_latency_samples_interval_,
            bev_stage_latency_ns_interval_,
            bev_stage_latency_ns_max_interval_);
        }
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

  void captureLatestBev(const char * trigger)
  {
    const auto frame = std::atomic_load_explicit(
      &latest_output_, std::memory_order_acquire);
    if (!frame || (frame->bgr.empty() && !frame->download_bgr)) {
      RCLCPP_WARN(
        get_logger(),
        "%s capture requested before a BEV frame was available.",
        trigger);
      return;
    }

    try {
      const std::filesystem::path directory(capture_directory_);
      std::error_code error;
      std::filesystem::create_directories(directory, error);
      if (error) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to create BEV capture directory '%s': %s",
          directory.string().c_str(),
          error.message().c_str());
        return;
      }
      const std::filesystem::path filename = directory /
        ("bev_capture_" +
        std::to_string(get_clock()->now().nanoseconds()) + ".png");
      const auto image=frame->bgr.empty() && frame->download_bgr ? frame->download_bgr() : frame->bgr;
      if (!cv::imwrite(filename.string(), image)) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to capture BEV image: %s",
          filename.string().c_str());
        return;
      }
      const auto absolute_filename = std::filesystem::absolute(filename, error);
      const std::string saved_path = error ?
        filename.string() : absolute_filename.string();
      RCLCPP_INFO(
        get_logger(),
        "BEV image captured by %s: %s",
        trigger,
        saved_path.c_str());
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to capture BEV image: %s",
        exception.what());
    }
  }

  static std::string zeroPaddedNumber(
    const std::uint64_t value,
    const int minimum_width)
  {
    std::ostringstream stream;
    stream << std::setfill('0') << std::setw(minimum_width) << value;
    return stream.str();
  }

  void initializeDatasetCollection()
  {
    namespace fs = std::filesystem;

    const fs::path root(dataset_collection_root_directory_);
    fs::create_directories(root);

    const std::string prefix = "dataset_";
    std::uint64_t maximum_index = 0U;
    for (const fs::directory_entry & entry : fs::directory_iterator(root)) {
      if (!entry.is_directory()) {
        continue;
      }
      const std::string name = entry.path().filename().string();
      if (name.compare(0, prefix.size(), prefix) != 0) {
        continue;
      }
      const std::string suffix = name.substr(prefix.size());
      if (
        suffix.empty() ||
        !std::all_of(
          suffix.begin(), suffix.end(),
          [](const char character) {
            return character >= '0' && character <= '9';
          }))
      {
        continue;
      }
      try {
        maximum_index = std::max(
          maximum_index,
          static_cast<std::uint64_t>(std::stoull(suffix)));
      } catch (const std::exception &) {
        // Ignore unrelated directories with an out-of-range numeric suffix.
      }
    }

    std::uint64_t next_index = maximum_index + 1U;
    while (true) {
      const fs::path candidate = root /
        (prefix + zeroPaddedNumber(next_index, 3));
      if (fs::create_directory(candidate)) {
        dataset_collection_directory_ = fs::absolute(candidate);
        break;
      }
      ++next_index;
    }

    fs::create_directories(dataset_collection_directory_ / "origin_bev");
    if (dataset_collection_manual_capture_mode_) {
      RCLCPP_INFO(
        get_logger(),
        "BEV manual dataset capture enabled: directory=%s, joy=%s, "
        "button=%d (A), target=%lld frames.",
        dataset_collection_directory_.string().c_str(),
        capture_joy_topic_.c_str(),
        dataset_collection_manual_capture_button_,
        static_cast<long long>(dataset_collection_target_count_));
    } else {
      RCLCPP_INFO(
        get_logger(),
        "BEV automatic dataset collection enabled: directory=%s, "
        "rate=%.2fHz, target=%lld frames, stop_auto_on_complete=%s.",
        dataset_collection_directory_.string().c_str(),
        dataset_collection_fps_,
        static_cast<long long>(dataset_collection_target_count_),
        dataset_collection_stop_auto_on_complete_ ? "true" : "false");
    }
  }

  std::int64_t captureTimeNanoseconds(const BevFrame & frame)
  {
    const std::int64_t timestamp =
      static_cast<std::int64_t>(frame.header.stamp.sec) * 1000000000LL +
      static_cast<std::int64_t>(frame.header.stamp.nanosec);
    return timestamp > 0 ? timestamp : get_clock()->now().nanoseconds();
  }

  // BEV capture remains available; legacy automatic lane labels are no longer produced.
  void saveDatasetFrame(const BevFrame & frame, const std::uint64_t matching_number)
  {
    namespace fs = std::filesystem;
    if (
      frame.bgr.type() != CV_8UC3 || frame.bgr.cols != bev_config_.output_width ||
      frame.bgr.rows != bev_config_.output_height)
    {
      throw std::runtime_error("dataset frame has invalid BGR BEV dimensions");
    }
    const auto stem = std::to_string(captureTimeNanoseconds(frame)) + "_" +
      zeroPaddedNumber(matching_number, 6);
    const fs::path output = dataset_collection_directory_ / "origin_bev" / (stem + ".png");
    const fs::path temporary = dataset_collection_directory_ / "origin_bev" / (stem + ".tmp.png");
    try {
      if (!cv::imwrite(temporary.string(), frame.bgr)) {
        throw std::runtime_error("failed to encode origin BEV PNG");
      }
      fs::rename(temporary, output);
    } catch (...) {
      std::error_code error;
      fs::remove(temporary, error);
      throw;
    }
  }

  void requestAutomaticStop(const char * reason)
  {
    std_msgs::msg::Bool enabled_message;
    enabled_message.data = false;
    preview_stop_publisher_->publish(enabled_message);
    RCLCPP_WARN(
      get_logger(),
      "%s Requested automatic duty 0 on %s.",
      reason,
      preview_stop_topic_.c_str());
  }

  void captureManualDatasetFrame(const char * trigger)
  {
    const auto frame = std::atomic_load_explicit(
      &latest_output_, std::memory_order_acquire);
    if (!frame || frame->bgr.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "%s dataset capture requested before a BEV frame was available.",
        trigger);
      return;
    }

    std::lock_guard<std::mutex> lock(dataset_collection_save_mutex_);
    const std::uint64_t saved_count =
      dataset_collection_saved_total_.load(std::memory_order_relaxed);
    const std::uint64_t target_count =
      static_cast<std::uint64_t>(dataset_collection_target_count_);
    if (saved_count >= target_count) {
      dataset_collection_complete_.store(true, std::memory_order_release);
      RCLCPP_WARN(
        get_logger(),
        "Manual BEV dataset target already reached: %llu/%llu frames.",
        static_cast<unsigned long long>(saved_count),
        static_cast<unsigned long long>(target_count));
      return;
    }

    try {
      saveDatasetFrame(*frame, saved_count + 1U);
    } catch (const std::exception & exception) {
      dataset_collection_failed_.store(true, std::memory_order_release);
      RCLCPP_ERROR(
        get_logger(),
        "Manual BEV dataset capture failed at frame %llu: %s",
        static_cast<unsigned long long>(saved_count + 1U),
        exception.what());
      return;
    }

    const std::uint64_t new_count = saved_count + 1U;
    dataset_collection_saved_total_.store(
      new_count, std::memory_order_release);
    dataset_collection_failed_.store(false, std::memory_order_release);
    RCLCPP_INFO(
      get_logger(),
      "BEV dataset frame captured by %s: %llu/%llu in %s.",
      trigger,
      static_cast<unsigned long long>(new_count),
      static_cast<unsigned long long>(target_count),
      dataset_collection_directory_.string().c_str());
    if (new_count == target_count) {
      dataset_collection_complete_.store(true, std::memory_order_release);
      RCLCPP_INFO(
        get_logger(),
        "Manual BEV dataset capture completed: %llu frames saved in %s.",
        static_cast<unsigned long long>(new_count),
        dataset_collection_directory_.string().c_str());
    }
  }

  void datasetCollectionLoop()
  {
    const auto minimum_period =
      std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(1.0 / dataset_collection_fps_));
    const std::uint64_t target_count =
      static_cast<std::uint64_t>(dataset_collection_target_count_);
    std::uint64_t last_generation = 0U;
    std::uint64_t saved_count = 0U;
    SteadyClock::time_point last_saved_at{};

    while (
      !stop_.load(std::memory_order_acquire) &&
      saved_count < target_count)
    {
      const auto frame = waitForNewOutput(last_generation);
      if (!frame) {
        continue;
      }
      last_generation = frame->source_generation;

      const auto now = SteadyClock::now();
      if (
        last_saved_at.time_since_epoch().count() != 0 &&
        now - last_saved_at < minimum_period)
      {
        continue;
      }

      try {
        saveDatasetFrame(*frame, saved_count + 1U);
      } catch (const std::exception & exception) {
        dataset_collection_failed_.store(true, std::memory_order_release);
        RCLCPP_ERROR(
          get_logger(),
          "BEV dataset collection stopped after %llu/%llu frames: %s",
          static_cast<unsigned long long>(saved_count),
          static_cast<unsigned long long>(target_count),
          exception.what());
        if (dataset_collection_stop_auto_on_complete_) {
          requestAutomaticStop("Dataset write failed.");
        }
        return;
      }

      ++saved_count;
      dataset_collection_saved_total_.store(
        saved_count, std::memory_order_release);
      last_saved_at = SteadyClock::now();
    }

    if (saved_count == target_count) {
      dataset_collection_complete_.store(true, std::memory_order_release);
      RCLCPP_INFO(
        get_logger(),
        "BEV dataset collection completed: %llu frames saved in %s.",
        static_cast<unsigned long long>(saved_count),
        dataset_collection_directory_.string().c_str());
      if (dataset_collection_stop_auto_on_complete_) {
        requestAutomaticStop("Dataset target reached.");
      }
    }
  }

  void onCaptureJoy(const sensor_msgs::msg::Joy::ConstSharedPtr message)
  {
    const bool bev_button_available =
      capture_joy_button_ < static_cast<int>(message->buttons.size());
    const bool bev_pressed =
      bev_button_available &&
      message->buttons[static_cast<std::size_t>(capture_joy_button_)] != 0;
    const bool bev_was_pressed = capture_joy_button_pressed_.exchange(
      bev_pressed, std::memory_order_relaxed);
    if (!bev_button_available) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Joy message on %s has no capture button index %d.",
        capture_joy_topic_.c_str(),
        capture_joy_button_);
    } else if (bev_pressed && !bev_was_pressed) {
      captureLatestBev("controller B");
    }

    if (!dataset_collection_manual_capture_mode_) {
      return;
    }
    const bool dataset_button_available =
      dataset_collection_manual_capture_button_ <
      static_cast<int>(message->buttons.size());
    const bool dataset_pressed =
      dataset_button_available &&
      message->buttons[static_cast<std::size_t>(
        dataset_collection_manual_capture_button_)] != 0;
    const bool dataset_was_pressed =
      dataset_collection_manual_capture_button_pressed_.exchange(
      dataset_pressed, std::memory_order_relaxed);
    if (!dataset_button_available) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Joy message on %s has no manual dataset button index %d.",
        capture_joy_topic_.c_str(),
        dataset_collection_manual_capture_button_);
    } else if (dataset_pressed && !dataset_was_pressed) {
      captureManualDatasetFrame("controller A");
    }
  }

  bool handlePreviewKey(const int key)
  {
    if (key == 'b' || key == 'B') {
      captureLatestBev("keyboard B");
      return false;
    }
    if (key == ' ') {
      std_msgs::msg::Bool enabled_message;
      enabled_message.data = false;
      preview_stop_publisher_->publish(enabled_message);
      RCLCPP_WARN(
        get_logger(),
        "SPACE pressed in the BEV preview: requested automatic duty 0 on %s.",
        preview_stop_topic_.c_str());
      return false;
    }
    return key == 27 || key == 'q' || key == 'Q';
  }

  void previewLoop()
  {
    try {
      cv::namedWindow(preview_window_name_, cv::WINDOW_NORMAL);
      cv::resizeWindow(
        preview_window_name_,
        std::min(preview_max_width_, 2 * bev_config_.output_width),
        std::min(preview_max_height_, 2 * bev_config_.output_height));

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
          if (handlePreviewKey(key)) {
            break;
          }
          continue;
        }

        const auto frame = std::atomic_load_explicit(
          &latest_output_, std::memory_order_acquire);
        if (frame) {
          const cv::Mat & displayed_image = frame->bgr;
          cv::imshow(preview_window_name_, displayed_image);
          previewed_total_.fetch_add(1U, std::memory_order_relaxed);
          previewed_interval_.fetch_add(1U, std::memory_order_relaxed);
          next_preview_at = now + preview_period;
        } else {
          next_preview_at = now + std::chrono::milliseconds(5);
        }

        const int key = cv::waitKey(1);
        if (handlePreviewKey(key)) {
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
          (frame && frame->source_generation != last_generation);
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

  void recordPipelineLatency(
    const std_msgs::msg::Header & header,
    std::atomic<std::uint64_t> & sample_count,
    std::atomic<std::uint64_t> & latency_ns_sum,
    std::atomic<std::uint64_t> & latency_ns_max)
  {
    if (!perf_log_enabled_) {
      return;
    }

    const rclcpp::Time frame_stamp(
      header.stamp,
      get_clock()->get_clock_type());
    const std::int64_t latency_ns =
      (get_clock()->now() - frame_stamp).nanoseconds();
    constexpr std::int64_t maximum_valid_latency_ns =
      60LL * 1000LL * 1000LL * 1000LL;
    if (latency_ns < 0 || latency_ns > maximum_valid_latency_ns) {
      return;
    }

    const auto valid_latency_ns = static_cast<std::uint64_t>(latency_ns);
    sample_count.fetch_add(1U, std::memory_order_relaxed);
    latency_ns_sum.fetch_add(valid_latency_ns, std::memory_order_relaxed);
    updateMaximum(latency_ns_max, valid_latency_ns);
  }

  void recordSteadyLatency(
    const SteadyClock::duration latency,
    std::atomic<std::uint64_t> & sample_count,
    std::atomic<std::uint64_t> & latency_ns_sum,
    std::atomic<std::uint64_t> & latency_ns_max)
  {
    if (!perf_log_enabled_) {
      return;
    }

    const auto latency_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(latency).count();
    constexpr std::int64_t maximum_valid_latency_ns =
      60LL * 1000LL * 1000LL * 1000LL;
    if (latency_ns < 0 || latency_ns > maximum_valid_latency_ns) {
      return;
    }

    const auto valid_latency_ns = static_cast<std::uint64_t>(latency_ns);
    sample_count.fetch_add(1U, std::memory_order_relaxed);
    latency_ns_sum.fetch_add(valid_latency_ns, std::memory_order_relaxed);
    updateMaximum(latency_ns_max, valid_latency_ns);
  }

  void logStatus()
  {
    const auto now = SteadyClock::now();
    const double elapsed_sec =
      std::chrono::duration<double>(now - status_started_at_).count();
    status_started_at_ = now;

    const auto received =
      received_interval_.exchange(0U, std::memory_order_relaxed);
    const auto accepted =
      accepted_interval_.exchange(0U, std::memory_order_relaxed);
    const auto processed =
      processed_interval_.exchange(0U, std::memory_order_relaxed);
    const auto skipped =
      skipped_interval_.exchange(0U, std::memory_order_relaxed);
    const auto published =
      published_interval_.exchange(0U, std::memory_order_relaxed);
    const auto direct_delivered =
      direct_delivered_interval_.exchange(0U, std::memory_order_relaxed);
    const auto bev_ready = direct_output_enabled_ ?
      direct_delivered : published;
    const auto previewed =
      previewed_interval_.exchange(0U, std::memory_order_relaxed);
    const auto process_ns =
      process_ns_interval_.exchange(0U, std::memory_order_relaxed);
    const auto process_ns_max =
      process_ns_max_interval_.exchange(0U, std::memory_order_relaxed);
    const auto stabilized_latency_samples =
      stabilized_latency_samples_interval_.exchange(
      0U, std::memory_order_relaxed);
    const auto stabilized_latency_ns =
      stabilized_latency_ns_interval_.exchange(0U, std::memory_order_relaxed);
    const auto stabilized_latency_ns_max =
      stabilized_latency_ns_max_interval_.exchange(
      0U, std::memory_order_relaxed);
    const auto bev_ready_latency_samples =
      bev_ready_latency_samples_interval_.exchange(
      0U, std::memory_order_relaxed);
    const auto bev_ready_latency_ns =
      bev_ready_latency_ns_interval_.exchange(0U, std::memory_order_relaxed);
    const auto bev_ready_latency_ns_max =
      bev_ready_latency_ns_max_interval_.exchange(
      0U, std::memory_order_relaxed);
    const auto bev_stage_latency_samples =
      bev_stage_latency_samples_interval_.exchange(
      0U, std::memory_order_relaxed);
    const auto bev_stage_latency_ns =
      bev_stage_latency_ns_interval_.exchange(0U, std::memory_order_relaxed);
    const auto bev_stage_latency_ns_max =
      bev_stage_latency_ns_max_interval_.exchange(
      0U, std::memory_order_relaxed);

    double latest_age_ms = 0.0;
    const auto latest = std::atomic_load_explicit(
      &latest_output_, std::memory_order_acquire);
    if (latest) {
      latest_age_ms =
        std::chrono::duration<double, std::milli>(
        now - latest->bev_input_received_at).count();
    }
    const double average_process_ms =
      processed > 0U ?
      static_cast<double>(process_ns) /
      static_cast<double>(processed) / 1.0e6 :
      0.0;
    const double average_stabilized_latency_ms =
      stabilized_latency_samples > 0U ?
      static_cast<double>(stabilized_latency_ns) /
      static_cast<double>(stabilized_latency_samples) / 1.0e6 :
      0.0;
    const double average_bev_ready_latency_ms =
      bev_ready_latency_samples > 0U ?
      static_cast<double>(bev_ready_latency_ns) /
      static_cast<double>(bev_ready_latency_samples) / 1.0e6 :
      0.0;
    const double average_bev_stage_latency_ms =
      bev_stage_latency_samples > 0U ?
      static_cast<double>(bev_stage_latency_ns) /
      static_cast<double>(bev_stage_latency_samples) / 1.0e6 :
      0.0;
    if (perf_log_enabled_) {
      RCLCPP_INFO(
        get_logger(),
        "[PERF][PIPELINE] stabilized_fps=%.1f bev_ready_fps=%.1f "
        "processed_fps=%.1f "
        "latency_ms(depthai_to_bev_input_avg/max=%.2f/%.2f,"
        "depthai_to_bev_ready_avg/max=%.2f/%.2f,"
        "bev_input_to_ready_avg/max=%.3f/%.3f) "
        "bev_compute_ms(avg/max)=%.3f/%.3f skipped=%llu "
        "errors(invalid/process/publish)=%llu/%llu/%llu",
        static_cast<double>(accepted) / elapsed_sec,
        static_cast<double>(bev_ready) / elapsed_sec,
        static_cast<double>(processed) / elapsed_sec,
        average_stabilized_latency_ms,
        static_cast<double>(stabilized_latency_ns_max) / 1.0e6,
        average_bev_ready_latency_ms,
        static_cast<double>(bev_ready_latency_ns_max) / 1.0e6,
        average_bev_stage_latency_ms,
        static_cast<double>(bev_stage_latency_ns_max) / 1.0e6,
        average_process_ms,
        static_cast<double>(process_ns_max) / 1.0e6,
        static_cast<unsigned long long>(skipped),
        static_cast<unsigned long long>(
          invalid_total_.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
          processing_error_total_.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
          publish_error_total_.load(std::memory_order_relaxed)));
    } else {
      RCLCPP_INFO(
        get_logger(),
        "\nBEV status: input=%.1fHz (%llu total), processed=%.1fHz "
        "(%llu total, skipped=%llu/%llu interval/total), "
        "direct=%.1fHz, ROS=%.1fHz, preview=%.1fHz, "
        "compute=%.3f/%.3fms avg/max, latest_age=%.2fms, "
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
        static_cast<double>(direct_delivered) / elapsed_sec,
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
    }


    if (
      dataset_collection_enabled_ ||
      dataset_collection_manual_capture_mode_)
    {
      const char * state =
        dataset_collection_failed_.load(std::memory_order_relaxed) ?
        "failed" :
        dataset_collection_complete_.load(std::memory_order_relaxed) ?
        "complete" : "collecting";
      RCLCPP_INFO(
        get_logger(),
        "BEV dataset: mode=%s, %llu/%lld frames, state=%s, directory=%s",
        dataset_collection_manual_capture_mode_ ? "manual-A" : "automatic",
        static_cast<unsigned long long>(
          dataset_collection_saved_total_.load(std::memory_order_relaxed)),
        static_cast<long long>(dataset_collection_target_count_),
        state,
        dataset_collection_directory_.string().c_str());
    }

    if (
      received_total_.load(std::memory_order_relaxed) == 0U &&
      std::chrono::duration<double>(now - node_started_at_).count() >=
      startup_timeout_sec_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "No fused camera input received on %s. Check that camera_driver "
        "fused_bev_output_enabled is true and its crop is %dx%d.",
        input_topic_.c_str(),
        input_crop_width_,
        input_crop_height_);
    }
    if (
      direct_output_enabled_ &&
      direct_delivered_total_.load(std::memory_order_relaxed) == 0U &&
      std::chrono::duration<double>(now - node_started_at_).count() >=
      startup_timeout_sec_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "No process-local BEV consumer is registered; load line_detactor "
        "into bev_processor_container with direct_bev_input_enabled=true.");
    }
  }

  int configuration_version_{0};
  bool perf_log_enabled_{false};
  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  std::string startup_ground_reference_topic_;
  std::string startup_ground_reference_frame_id_;
  std::unique_ptr<bev_handoff::RuntimeMonitor> monitor_;
  double expected_input_fps_{70.0};
  double input_bottom_fraction_{0.70};
  int input_crop_width_{1280};
  int input_crop_height_{560};
  int input_crop_top_{240};
  bool publish_enabled_{true};
  bool direct_output_enabled_{false};
  bool direct_host_copy_enabled_{false};
  double publish_max_fps_{0.0};
  bool preview_enabled_{true};
  double preview_max_fps_{60.0};
  std::string preview_window_name_;
  std::string preview_stop_topic_{"/auto/enabled"};
  int preview_max_width_{1280};
  int preview_max_height_{800};
  std::string capture_directory_{"."};
  std::string capture_joy_topic_{"/joy"};
  int capture_joy_button_{1};
  std::atomic<bool> capture_joy_button_pressed_{false};
  bool dataset_collection_enabled_{false};
  bool dataset_collection_manual_capture_mode_{false};
  int dataset_collection_manual_capture_button_{0};
  std::atomic<bool> dataset_collection_manual_capture_button_pressed_{false};
  std::string dataset_collection_root_directory_{"datasets"};
  double dataset_collection_fps_{10.0};
  std::int64_t dataset_collection_target_count_{1000};
  bool dataset_collection_stop_auto_on_complete_{true};
  std::filesystem::path dataset_collection_directory_;
  std::atomic<std::uint64_t> dataset_collection_saved_total_{0U};
  std::atomic<bool> dataset_collection_complete_{false};
  std::atomic<bool> dataset_collection_failed_{false};
  std::mutex dataset_collection_save_mutex_;
  double status_log_interval_sec_{5.0};
  double startup_timeout_sec_{12.0};
  double stabilization_settle_sec_{5.5};
  double camera_yaw_deg_{0.0};
  std::string bev_interpolation_{"bilinear"};
  EdgeAdaptiveConfig edge_adaptive_config_{};
  OakStartupMeasurementConfig startup_measurement_config_{};

  RectifiedCameraModel camera_model_{};
  BevConfig bev_config_{};
  RemapLut bev_lut_{};
  bool fused_coverage_reported_{false};
  double startup_roll_deg_{0.0};
  double startup_pitch_down_deg_{14.0};
  std::atomic<double> valid_lut_percent_{0.0};
  std::atomic<double> applied_roll_deg_{0.0};
  std::atomic<double> applied_pitch_down_deg_{14.0};
  std::shared_ptr<CudaBevProcessor> gpu_processor_;

  rclcpp::Subscription<camera_driver::msg::BevInput>::SharedPtr
    input_subscription_;
  void publishStartupStatus(const std::string & text)
  {
    std_msgs::msg::String message; message.data=text; startup_status_publisher_->publish(message);
  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr startup_status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr perf_publisher_;
  rclcpp::TimerBase::SharedPtr perf_timer_;
  SteadyClock::time_point perf_started_at_{};
  bev_handoff::StageMeter bev_input_meter_, bev_stage_meter_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr startup_image_publisher_;
  SteadyClock::time_point last_startup_preview_{};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr output_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr preview_stop_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr
    capture_joy_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    startup_ground_reference_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::mutex input_mutex_;
  bool tunnel_rgb_standby_{false};
  SteadyClock::time_point last_tunnel_rgb_input_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tunnel_compute_sub_;
  std::condition_variable input_cv_;
  camera_driver::msg::BevInput::ConstSharedPtr latest_input_;
  SteadyClock::time_point latest_input_received_at_;
  std::uint64_t input_generation_{0U};
  bool first_camera_input_seen_{false};
  SteadyClock::time_point first_camera_input_at_;

  std::mutex output_mutex_;
  std::condition_variable output_cv_;
  std::shared_ptr<const BevFrame> latest_output_;

  std::atomic<bool> stop_{false};
  std::thread processing_thread_;
  std::thread publishing_thread_;
  std::thread preview_thread_;
  std::thread dataset_collection_thread_;

  const SteadyClock::time_point node_started_at_{SteadyClock::now()};
  SteadyClock::time_point status_started_at_{SteadyClock::now()};

  std::atomic<std::uint64_t> received_total_{0U};
  std::atomic<std::uint64_t> accepted_total_{0U};
  std::atomic<std::uint64_t> processed_total_{0U};
  std::atomic<std::uint64_t> skipped_total_{0U};
  std::atomic<std::uint64_t> published_total_{0U};
  std::atomic<std::uint64_t> direct_delivered_total_{0U};
  std::atomic<std::uint64_t> direct_unavailable_total_{0U};
  std::atomic<std::uint64_t> previewed_total_{0U};
  std::atomic<std::uint64_t> publish_throttled_total_{0U};
  std::atomic<std::uint64_t> invalid_total_{0U};
  std::atomic<std::uint64_t> stabilization_settle_total_{0U};
  std::atomic<std::uint64_t> processing_error_total_{0U};
  std::atomic<std::uint64_t> publish_error_total_{0U};
  std::atomic<std::uint64_t> received_interval_{0U};
  std::atomic<std::uint64_t> accepted_interval_{0U};
  std::atomic<std::uint64_t> processed_interval_{0U};
  std::atomic<std::uint64_t> skipped_interval_{0U};
  std::atomic<std::uint64_t> published_interval_{0U};
  std::atomic<std::uint64_t> direct_delivered_interval_{0U};
  std::atomic<std::uint64_t> previewed_interval_{0U};
  std::atomic<std::uint64_t> process_ns_interval_{0U};
  std::atomic<std::uint64_t> process_ns_max_interval_{0U};
  std::atomic<std::uint64_t> stabilized_latency_samples_interval_{0U};
  std::atomic<std::uint64_t> stabilized_latency_ns_interval_{0U};
  std::atomic<std::uint64_t> stabilized_latency_ns_max_interval_{0U};
  std::atomic<std::uint64_t> bev_ready_latency_samples_interval_{0U};
  std::atomic<std::uint64_t> bev_ready_latency_ns_interval_{0U};
  std::atomic<std::uint64_t> bev_ready_latency_ns_max_interval_{0U};
  std::atomic<std::uint64_t> bev_stage_latency_samples_interval_{0U};
  std::atomic<std::uint64_t> bev_stage_latency_ns_interval_{0U};
  std::atomic<std::uint64_t> bev_stage_latency_ns_max_interval_{0U};
};

}  // namespace bev_processor

RCLCPP_COMPONENTS_REGISTER_NODE(bev_processor::BevProcessorNode)
