#include "camera_driver/camera_driver_node.hpp"
#include "camera_driver/imu_image_stabilizer.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "depthai/depthai.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace camera_driver
{

using namespace std::chrono_literals;

namespace
{

constexpr std::uint32_t kOv9782The720PWidth = 1280U;
constexpr std::uint32_t kOv9782The720PHeight = 720U;

void update_maximum(
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

void record_steady_latency(
  const std::chrono::steady_clock::duration latency,
  std::atomic<std::uint64_t> & sample_count,
  std::atomic<std::uint64_t> & latency_ns_sum,
  std::atomic<std::uint64_t> & latency_ns_max)
{
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
  update_maximum(latency_ns_max, valid_latency_ns);
}

double average_milliseconds(
  const std::uint64_t duration_ns,
  const std::uint64_t sample_count)
{
  return sample_count > 0U ?
         static_cast<double>(duration_ns) /
         static_cast<double>(sample_count) / 1.0e6 :
         0.0;
}

std::string uppercase(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char character) {
      return static_cast<char>(std::toupper(character));
    });
  return value;
}

dai::CameraBoardSocket parse_camera_socket(const std::string & value)
{
  const auto normalized = uppercase(value);
  if (normalized == "CAM_A") {
    return dai::CameraBoardSocket::CAM_A;
  }
  if (normalized == "CAM_B") {
    return dai::CameraBoardSocket::CAM_B;
  }
  if (normalized == "CAM_C") {
    return dai::CameraBoardSocket::CAM_C;
  }
  if (normalized == "CAM_D") {
    return dai::CameraBoardSocket::CAM_D;
  }
  throw std::invalid_argument(
          "camera_socket must be CAM_A, CAM_B, CAM_C, or CAM_D");
}

dai::ImgResizeMode parse_resize_mode(const std::string & value)
{
  const auto normalized = uppercase(value);
  if (normalized == "CROP") {
    return dai::ImgResizeMode::CROP;
  }
  if (normalized == "STRETCH") {
    return dai::ImgResizeMode::STRETCH;
  }
  if (normalized == "LETTERBOX") {
    return dai::ImgResizeMode::LETTERBOX;
  }
  throw std::invalid_argument(
          "resize_mode must be CROP, STRETCH, or LETTERBOX");
}

cv::Matx33d matrix3x3_from_calibration(
  const std::vector<std::vector<float>> & rows,
  const char * label)
{
  if (
    rows.size() < 3U || rows[0].size() < 3U ||
    rows[1].size() < 3U || rows[2].size() < 3U)
  {
    throw std::runtime_error(
            std::string(label) + " is missing or smaller than 3x3");
  }
  cv::Matx33d result;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      result(row, column) = static_cast<double>(rows[row][column]);
    }
  }
  return result;
}

void validate_rotation_matrix(
  const cv::Matx33d & rotation,
  const char * label)
{
  const double orthogonality_error = cv::norm(
    rotation * rotation.t() - cv::Matx33d::eye());
  const double determinant = cv::determinant(cv::Mat(rotation));
  if (
    !cv::checkRange(cv::Mat(rotation)) ||
    !std::isfinite(orthogonality_error) || orthogonality_error > 1.0e-3 ||
    !std::isfinite(determinant) || std::abs(determinant - 1.0) > 1.0e-3)
  {
    throw std::runtime_error(std::string(label) + " is not a proper rotation");
  }
}

const char * usb_speed_name(dai::UsbSpeed speed)
{
  switch (speed) {
    case dai::UsbSpeed::LOW:
      return "LOW";
    case dai::UsbSpeed::FULL:
      return "FULL";
    case dai::UsbSpeed::HIGH:
      return "HIGH";
    case dai::UsbSpeed::SUPER:
      return "SUPER";
    case dai::UsbSpeed::SUPER_PLUS:
      return "SUPER_PLUS";
    case dai::UsbSpeed::UNKNOWN:
    default:
      return "UNKNOWN";
  }
}

bool graphical_display_available()
{
#if defined(__linux__)
  return std::getenv("DISPLAY") != nullptr ||
         std::getenv("WAYLAND_DISPLAY") != nullptr;
#else
  return true;
#endif
}

}  // namespace

class CameraDriverNode::Impl
{
public:
  explicit Impl(CameraDriverNode & node)
  : node_(node),
    started_at_(std::chrono::steady_clock::now()),
    last_status_at_(started_at_)
  {
    read_parameters();
    if (imu_stabilization_enabled_) {
      imu_stabilizer_ =
        std::make_unique<ImuImageStabilizer>(imu_stabilizer_config_);
    }

    if (!enabled_) {
      RCLCPP_WARN(node_.get_logger(), "Camera is disabled by parameter.");
      return;
    }

    if (performance_measurement_enabled_) {
      RCLCPP_INFO(
        node_.get_logger(),
        "Performance measurement mode enabled: camera GUI preview is off; "
        "capture and stabilized-output FPS will be reported.");
    }

    if (preview_enabled_ && !graphical_display_available()) {
      preview_enabled_ = false;
      RCLCPP_WARN(
        node_.get_logger(),
        "Preview disabled because DISPLAY/WAYLAND_DISPLAY is not available.");
    }
    preview_active_.store(preview_enabled_);

    if (publish_enabled_) {
      auto qos = rclcpp::SensorDataQoS();
      qos.keep_last(1);
      publisher_ = node_.create_publisher<sensor_msgs::msg::Image>(
        image_topic_, qos);
    }
    if (imu_bridge_enabled_) {
      auto qos = rclcpp::SensorDataQoS();
      qos.keep_last(5);
      imu_publisher_ = node_.create_publisher<sensor_msgs::msg::Imu>(
        imu_topic_, qos);
    }

    try {
      start_pipeline();
      started_at_ = std::chrono::steady_clock::now();
      last_status_at_ = started_at_;
      const auto status_period =
        std::chrono::duration<double>(status_log_interval_sec_);
      status_timer_ = node_.create_wall_timer(
        status_period, std::bind(&Impl::report_status, this));
      capture_thread_ = std::thread(&Impl::capture_loop, this);
      if (imu_stream_enabled_) {
        imu_thread_ = std::thread(&Impl::imu_loop, this);
      }
      if (publish_enabled_) {
        publish_thread_ = std::thread(&Impl::publish_loop, this);
      }
      if (preview_enabled_) {
        preview_thread_ = std::thread(&Impl::preview_loop, this);
      }
    } catch (...) {
      stop();
      throw;
    }
  }

  ~Impl()
  {
    stop();
  }

private:
  struct FrameSnapshot
  {
    std::shared_ptr<dai::ImgFrame> packet;
    rclcpp::Time ros_stamp;
    std::chrono::steady_clock::time_point sensor_timestamp;
    std::chrono::steady_clock::time_point received_timestamp;
    std::uint64_t generation;
    std::int64_t device_sequence;
  };

  struct StabilizationTransform
  {
    std::optional<cv::Matx33d> homography;
    bool frame_usable{true};
  };

  template<typename IntegerT>
  static void require_positive(IntegerT value, const char * parameter_name)
  {
    if (value <= 0) {
      throw std::invalid_argument(
              std::string(parameter_name) + " must be greater than zero");
    }
  }

  void read_parameters()
  {
    enabled_ = node_.declare_parameter<bool>("enabled", true);
    performance_measurement_enabled_ = node_.declare_parameter<bool>(
      "performance_measurement_enabled", false);
    camera_socket_name_ =
      node_.declare_parameter<std::string>("camera_socket", "CAM_A");
    width_ = node_.declare_parameter<int>("width", 1280);
    height_ = node_.declare_parameter<int>("height", 720);
    sensor_fps_ = node_.declare_parameter<double>("sensor_fps", 120.0);
    resize_mode_name_ =
      node_.declare_parameter<std::string>("resize_mode", "CROP");
    undistort_enabled_ =
      node_.declare_parameter<bool>("undistort_enabled", true);
    queue_size_ = node_.declare_parameter<int>("queue_size", 8);
    queue_blocking_ =
      node_.declare_parameter<bool>("queue_blocking", false);
    frame_id_ = node_.declare_parameter<std::string>(
      "frame_id", "camera_optical_frame");
    image_topic_ = node_.declare_parameter<std::string>(
      "image_topic", "/camera/image_rect");
    imu_bridge_enabled_ =
      node_.declare_parameter<bool>("imu_bridge_enabled", false);
    imu_topic_ = node_.declare_parameter<std::string>(
      "imu_topic", "/camera/imu");
    imu_frame_id_ = node_.declare_parameter<std::string>(
      "imu_frame_id", "camera_optical_frame");
    imu_rate_hz_ = node_.declare_parameter<double>("imu_rate_hz", 400.0);
    imu_queue_size_ = node_.declare_parameter<int>("imu_queue_size", 80);
    imu_max_batch_reports_ =
      node_.declare_parameter<int>("imu_max_batch_reports", 5);
    maximum_imu_pair_skew_sec_ = node_.declare_parameter<double>(
      "maximum_accel_gyro_timestamp_skew_sec", 0.003);
    maximum_timestamp_domain_delta_sec_ = node_.declare_parameter<double>(
      "maximum_timestamp_domain_delta_sec", 1.0);
    imu_stabilization_enabled_ =
      node_.declare_parameter<bool>("imu_stabilization_enabled", true);
    imu_stabilizer_config_.startup_discard_duration_sec =
      node_.declare_parameter<double>(
      "imu_stabilization_startup_discard_duration_sec", 1.0);
    imu_stabilizer_config_.reference_calibration_duration_sec =
      node_.declare_parameter<double>(
      "imu_stabilization_reference_calibration_duration_sec", 4.0);
    imu_stabilizer_config_.calibration_maximum_angular_speed_degps =
      node_.declare_parameter<double>(
      "imu_stabilization_calibration_maximum_angular_speed_degps", 0.5);
    imu_stabilizer_config_.gyroscope_bias_enabled =
      node_.declare_parameter<bool>(
      "imu_stabilization_gyroscope_bias_enabled", true);
    imu_stabilizer_config_.gravity_mps2 =
      node_.declare_parameter<double>(
      "imu_stabilization_gravity_mps2", 9.80665);
    imu_stabilizer_config_.accelerometer_full_trust_deviation_mps2 =
      node_.declare_parameter<double>(
      "imu_stabilization_accelerometer_full_trust_deviation_mps2", 0.15);
    imu_stabilizer_config_.accelerometer_zero_trust_deviation_mps2 =
      node_.declare_parameter<double>(
      "imu_stabilization_accelerometer_zero_trust_deviation_mps2", 1.50);
    imu_stabilizer_config_.acceleration_correction_time_constant_sec =
      node_.declare_parameter<double>(
      "imu_stabilization_accelerometer_time_constant_sec", 4.3);
    imu_stabilizer_config_.acceleration_correction_gate_deg =
      node_.declare_parameter<double>(
      "imu_stabilization_accelerometer_direction_gate_deg", 4.3);
    imu_stabilizer_config_.roll_acceleration_correction_time_constant_sec =
      node_.declare_parameter<double>(
      "imu_stabilization_roll_accelerometer_time_constant_sec", 6.0);
    imu_stabilizer_config_.roll_acceleration_direction_gate_deg =
      node_.declare_parameter<double>(
      "imu_stabilization_roll_accelerometer_direction_gate_deg", 4.3);
    imu_stabilizer_config_.online_gyroscope_tilt_bias_enabled =
      node_.declare_parameter<bool>(
      "imu_stabilization_online_gyroscope_tilt_bias_enabled", true);
    imu_stabilizer_config_.online_gyroscope_tilt_bias_time_constant_sec =
      node_.declare_parameter<double>(
      "imu_stabilization_online_gyroscope_tilt_bias_time_constant_sec", 10.0);
    imu_stabilizer_config_.stationary_detection_window_sec =
      node_.declare_parameter<double>(
      "imu_stabilization_stationary_detection_window_sec", 1.0);
    imu_stabilizer_config_.stationary_accelerometer_norm_tolerance_mps2 =
      node_.declare_parameter<double>(
      "imu_stabilization_stationary_accelerometer_norm_tolerance_mps2", 0.20);
    imu_stabilizer_config_.stationary_accelerometer_norm_stddev_mps2 =
      node_.declare_parameter<double>(
      "imu_stabilization_stationary_accelerometer_norm_stddev_mps2", 0.08);
    imu_stabilizer_config_.stationary_accelerometer_direction_error_deg =
      node_.declare_parameter<double>(
      "imu_stabilization_stationary_accelerometer_direction_error_deg", 1.5);
    imu_stabilizer_config_.stationary_accelerometer_direction_change_deg =
      node_.declare_parameter<double>(
      "imu_stabilization_stationary_accelerometer_direction_change_deg", 0.15);
    imu_stabilizer_config_.stationary_gyroscope_mean_maximum_degps =
      node_.declare_parameter<double>(
      "imu_stabilization_stationary_gyroscope_mean_maximum_degps", 0.5);
    imu_stabilizer_config_.stationary_gyroscope_stddev_maximum_degps =
      node_.declare_parameter<double>(
      "imu_stabilization_stationary_gyroscope_stddev_maximum_degps", 0.8);
    imu_stabilizer_config_.pitch_correction_enabled =
      node_.declare_parameter<bool>(
      "imu_stabilization_pitch_correction_enabled", true);
    imu_stabilizer_config_.roll_correction_enabled =
      node_.declare_parameter<bool>(
      "imu_stabilization_roll_correction_enabled", true);
    imu_stabilizer_config_.maximum_correction_deg =
      node_.declare_parameter<double>(
      "imu_stabilization_maximum_correction_deg", 12.0);
    imu_stabilizer_config_.maximum_frame_imu_wait_sec =
      node_.declare_parameter<double>(
      "imu_stabilization_maximum_frame_imu_wait_sec", 0.001);
    imu_stabilizer_config_.maximum_frame_imu_age_sec =
      node_.declare_parameter<double>(
      "imu_stabilization_maximum_frame_imu_age_sec", 0.006);
    imu_stabilizer_config_.maximum_frame_imu_prediction_sec =
      node_.declare_parameter<double>(
      "imu_stabilization_maximum_prediction_sec", 0.015);
    fixed_view_zoom_ =
      node_.declare_parameter<double>("fixed_view_zoom", 1.25);
    fixed_view_border_margin_px_ = node_.declare_parameter<double>(
      "fixed_view_border_margin_px", 1.5);
    output_crop_top_px_ =
      node_.declare_parameter<int>("output_crop_top_px", 0);
    publish_enabled_ =
      node_.declare_parameter<bool>("publish_enabled", false);
    publish_fps_ =
      node_.declare_parameter<double>("publish_fps", 120.0);
    preview_enabled_ =
      node_.declare_parameter<bool>("preview_enabled", false);
    preview_fps_ =
      node_.declare_parameter<double>("preview_fps", 60.0);
    preview_window_name_ = node_.declare_parameter<std::string>(
      "preview_window_name", "OAK fixed-reference pitch-roll stabilization");
    preview_max_width_ =
      node_.declare_parameter<int>("preview_max_width", 1280);
    preview_max_height_ =
      node_.declare_parameter<int>("preview_max_height", 720);
    preview_grid_enabled_ =
      node_.declare_parameter<bool>("preview_grid_enabled", true);
    preview_grid_spacing_px_ =
      node_.declare_parameter<int>("preview_grid_spacing_px", 20);
    startup_timeout_sec_ =
      node_.declare_parameter<double>("startup_timeout_sec", 5.0);
    status_log_interval_sec_ =
      node_.declare_parameter<double>("status_log_interval_sec", 1.0);

    if (performance_measurement_enabled_) {
      preview_enabled_ = false;
    }

    require_positive(width_, "width");
    require_positive(height_, "height");
    if (width_ % 2 != 0 || height_ % 2 != 0) {
      throw std::invalid_argument(
              "width and height must be even for NV12");
    }
    if (
      output_crop_top_px_ < 0 ||
      output_crop_top_px_ >= height_ ||
      output_crop_top_px_ % 2 != 0)
    {
      throw std::invalid_argument(
              "output_crop_top_px must be an even value in [0, height)");
    }
    require_positive(sensor_fps_, "sensor_fps");
    require_positive(queue_size_, "queue_size");
    require_positive(startup_timeout_sec_, "startup_timeout_sec");
    require_positive(status_log_interval_sec_, "status_log_interval_sec");
    imu_stream_enabled_ =
      imu_bridge_enabled_ || imu_stabilization_enabled_;
    if (imu_stream_enabled_) {
      require_positive(imu_rate_hz_, "imu_rate_hz");
      require_positive(imu_queue_size_, "imu_queue_size");
      if (
        imu_max_batch_reports_ <= 1 ||
        imu_max_batch_reports_ > imu_queue_size_ ||
        !std::isfinite(maximum_imu_pair_skew_sec_) ||
        maximum_imu_pair_skew_sec_ <= 0.0 ||
        !std::isfinite(maximum_timestamp_domain_delta_sec_) ||
        maximum_timestamp_domain_delta_sec_ <= 0.0)
      {
        throw std::invalid_argument(
                "invalid IMU batching or accel/gyro timestamp skew limit");
      }
      if (
        imu_bridge_enabled_ &&
        (imu_topic_.empty() || imu_frame_id_.empty()))
      {
        throw std::invalid_argument(
                "imu_topic and imu_frame_id must not be empty when "
                "the internal IMU bridge is enabled");
      }
    }
    if (
      !std::isfinite(fixed_view_zoom_) ||
      fixed_view_zoom_ < 1.0 || fixed_view_zoom_ > 3.0 ||
      !std::isfinite(fixed_view_border_margin_px_) ||
      fixed_view_border_margin_px_ < 0.0 ||
      fixed_view_border_margin_px_ >=
      0.25 * static_cast<double>(std::min(width_, height_)))
    {
      throw std::invalid_argument(
              "invalid fixed-view zoom or source border margin");
    }
    if (publish_enabled_) {
      require_positive(publish_fps_, "publish_fps");
      if (image_topic_.empty()) {
        throw std::invalid_argument(
                "image_topic must not be empty when publishing is enabled");
      }
    }
    if (preview_enabled_) {
      require_positive(preview_fps_, "preview_fps");
      if (preview_window_name_.empty()) {
        throw std::invalid_argument(
                "preview_window_name must not be empty when preview is enabled");
      }
    }
    if (preview_max_width_ < 0 || preview_max_height_ < 0) {
      throw std::invalid_argument(
              "preview maximum dimensions must not be negative");
    }
    require_positive(preview_grid_spacing_px_, "preview_grid_spacing_px");

    camera_socket_ = parse_camera_socket(camera_socket_name_);
    resize_mode_ = parse_resize_mode(resize_mode_name_);
  }

  void start_pipeline()
  {
    auto device = std::make_shared<dai::Device>(
      dai::UsbSpeed::SUPER);
    pipeline_ = std::make_unique<dai::Pipeline>(device);
    pipeline_->setXLinkChunkSize(0);

    // Explicit OV9782 THE_720_P sensor mode: 1280x720, up to 143 FPS.
    auto camera = pipeline_->create<dai::node::Camera>()->build(
      camera_socket_,
      std::make_pair(kOv9782The720PWidth, kOv9782The720PHeight),
      static_cast<float>(sensor_fps_));
    auto * output = camera->requestOutput(
      std::make_pair(
        static_cast<std::uint32_t>(width_),
        static_cast<std::uint32_t>(height_)),
      // Keep the full-rate device/USB path compact. Stabilization warps the
      // NV12 planes directly, so no full-resolution host BGR frame is needed.
      dai::ImgFrame::Type::NV12,
      resize_mode_,
      static_cast<float>(sensor_fps_),
      undistort_enabled_);

    output_queue_ = output->createOutputQueue(
      static_cast<unsigned int>(queue_size_), queue_blocking_);

    if (imu_stream_enabled_) {
      try {
        const auto imu_name = device->getConnectedIMU();
        if (imu_name.empty()) {
          throw std::runtime_error("the OAK device reported no connected IMU");
        }

        // CALIBRATED reports already contain the EEPROM IMU output rotation.
        // Apply only the relative rotation from that calibrated output frame
        // to the selected camera optical frame, avoiding a double rotation.
        const auto calibration = device->getCalibration();
        const cv::Matx33d imu_to_camera_rotation =
          matrix3x3_from_calibration(
          calibration.getImuToCameraExtrinsics(camera_socket_, false),
          "IMU-to-camera calibration rotation");
        validate_rotation_matrix(
          imu_to_camera_rotation, "IMU-to-camera calibration rotation");
        const cv::Matx33d imu_to_calibrated_output_rotation =
          matrix3x3_from_calibration(
          calibration.getEepromData().imuExtrinsics.rotationMatrix,
          "runtime calibrated IMU output rotation");
        validate_rotation_matrix(
          imu_to_calibrated_output_rotation,
          "runtime calibrated IMU output rotation");
        calibrated_imu_output_to_camera_rotation_ =
          imu_to_camera_rotation * imu_to_calibrated_output_rotation.t();
        validate_rotation_matrix(
          calibrated_imu_output_to_camera_rotation_,
          "calibrated IMU output-to-camera rotation");

        auto imu = pipeline_->create<dai::node::IMU>();
        imu->enableIMUSensor(
          {
            dai::IMUSensor::ACCELEROMETER_CALIBRATED,
            dai::IMUSensor::GYROSCOPE_CALIBRATED,
          },
          static_cast<int>(std::lround(imu_rate_hz_)));
        imu->setBatchReportThreshold(1);
        imu->setMaxBatchReports(imu_max_batch_reports_);
        imu_queue_ = imu->out.createOutputQueue(
          static_cast<unsigned int>(imu_queue_size_), false);
        imu_name_ = imu_name;
      } catch (const std::exception & exception) {
        imu_bridge_enabled_ = false;
        imu_stabilization_enabled_ = false;
        imu_stream_enabled_ = false;
        imu_stabilizer_.reset();
        imu_queue_.reset();
        RCLCPP_ERROR(
          node_.get_logger(),
          "OAK IMU stream and image stabilization disabled: %s",
          exception.what());
      }
    }

    // The host output queue above is already latest-only, but DepthAI creates
    // a separate implicit XLinkOut input queue on the device. Build first so
    // that bridge exists, then make its queue latest-only as well.
    pipeline_->build();
    const auto xlink_bridge = output->getXLinkBridge();
    if (!xlink_bridge || !xlink_bridge->xLinkOut) {
      throw std::runtime_error("DepthAI did not create the camera XLink output bridge");
    }
    xlink_bridge->xLinkOut->input.setMaxSize(1);
    xlink_bridge->xLinkOut->input.setBlocking(false);

    pipeline_->start();

    RCLCPP_INFO(
      node_.get_logger(),
      "OAK: THE_720_P %dx%d @ %.1f FPS, USB=%s, "
      "transport=NV12, XLink chunks=off, XLink device queue=1/non-blocking",
      width_, height_, sensor_fps_, usb_speed_name(device->getUsbSpeed()));
    RCLCPP_INFO(
      node_.get_logger(),
      "Options: undistort=%s, publish=%s, preview=%s, "
      "preview_grid=%s/%dpx, imu_stabilization=%s, "
      "fixed_view_zoom=%.2fx, output_crop=top %dpx -> %dx%d, queue=%d/%s",
      undistort_enabled_ ? "on" : "off",
      publish_enabled_ ? "on" : "off",
      preview_enabled_ ? "on" : "off",
      preview_grid_enabled_ ? "on" : "off",
      preview_grid_spacing_px_,
      imu_stabilization_enabled_ ? "on" : "off",
      fixed_view_zoom_,
      output_crop_top_px_,
      width_,
      height_ - output_crop_top_px_,
      queue_size_,
      queue_blocking_ ? "blocking" : "non-blocking");
    if (imu_stream_enabled_) {
      RCLCPP_INFO(
        node_.get_logger(),
        "IMU: %s calibrated accelerometer+gyroscope @ %.1f Hz, ROS_bridge=%s, "
        "stabilization=%s, calibrated-output-to-%s relative rotation applied",
        imu_name_.c_str(), imu_rate_hz_,
        imu_bridge_enabled_ ? imu_topic_.c_str() : "off",
        imu_stabilization_enabled_ ? "on" : "off",
        camera_socket_name_.c_str());
    }
    if (imu_stabilization_enabled_) {
      RCLCPP_INFO(
        node_.get_logger(),
        "Fixed-reference stabilization: keep camera still for %.1f s "
        "startup discard + %.1f s stationary calibration; pitch/roll limit "
        "%.1f deg, crop overflow policy=drop frame",
        imu_stabilizer_config_.startup_discard_duration_sec,
        imu_stabilizer_config_.reference_calibration_duration_sec,
        imu_stabilizer_config_.maximum_correction_deg);
    }
  }

  rclcpp::Time ros_timestamp_for(
    const std::chrono::steady_clock::time_point & sensor_timestamp)
  {
    const auto steady_now = std::chrono::steady_clock::now();
    const auto ros_now = node_.get_clock()->now();
    const auto offset_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      sensor_timestamp - steady_now).count();

    constexpr std::int64_t max_past_offset_ns = -60LL * 1000LL * 1000LL * 1000LL;
    constexpr std::int64_t max_future_offset_ns = 1000LL * 1000LL * 1000LL;
    if (offset_ns < max_past_offset_ns || offset_ns > max_future_offset_ns) {
      if (!timestamp_fallback_reported_.exchange(true)) {
        RCLCPP_WARN(
          node_.get_logger(),
          "DepthAI host timestamp was outside the expected range; "
          "using ROS receive time.");
      }
      return ros_now;
    }

    return ros_now + rclcpp::Duration::from_nanoseconds(offset_ns);
  }

  void capture_loop()
  {
    while (!stop_requested_.load(std::memory_order_relaxed)) {
      try {
        if (!pipeline_ || !pipeline_->isRunning()) {
          break;
        }

        auto packet = output_queue_->tryGet<dai::ImgFrame>();
        if (!packet) {
          std::this_thread::sleep_for(100us);
          continue;
        }

        const auto received_at = std::chrono::steady_clock::now();
        // Stabilization is synchronized to the middle of RGB exposure, not
        // host arrival time or the beginning/end of exposure.
        const auto sensor_timestamp = packet->getTimestamp(
          dai::CameraExposureOffset::MIDDLE);
        if (performance_measurement_enabled_) {
          record_steady_latency(
            received_at - sensor_timestamp,
            sensor_to_host_samples_interval_,
            sensor_to_host_ns_interval_,
            sensor_to_host_ns_max_interval_);
        }
        const auto device_sequence = packet->getSequenceNum();
        if (last_device_sequence_.has_value() &&
          device_sequence > *last_device_sequence_ + 1)
        {
          const auto dropped = static_cast<std::uint64_t>(
            device_sequence - *last_device_sequence_ - 1);
          device_drops_total_.fetch_add(dropped);
          device_drops_interval_.fetch_add(dropped);
        }
        last_device_sequence_ = device_sequence;

        const auto generation =
          received_total_.fetch_add(1, std::memory_order_relaxed) + 1;
        received_interval_.fetch_add(1, std::memory_order_relaxed);

        if (publish_enabled_ || preview_enabled_) {
          const bool valid_nv12 =
            packet->getType() == dai::ImgFrame::Type::NV12 &&
            static_cast<int>(packet->getWidth()) == width_ &&
            static_cast<int>(packet->getHeight()) == height_;
          if (!valid_nv12) {
            invalid_frames_total_.fetch_add(1);
            RCLCPP_ERROR_THROTTLE(
              node_.get_logger(), *node_.get_clock(), 1000,
              "Expected a %dx%d NV12 frame from DepthAI.",
              width_, height_);
            continue;
          }

          std::shared_ptr<const FrameSnapshot> snapshot =
            std::make_shared<FrameSnapshot>(
            FrameSnapshot{
              packet,
              ros_timestamp_for(sensor_timestamp),
              sensor_timestamp,
              received_at,
              generation,
              device_sequence});
          std::atomic_store_explicit(
            &latest_frame_, std::move(snapshot), std::memory_order_release);
          frame_available_.notify_all();
        }

        if (!first_frame_received_.exchange(true)) {
          RCLCPP_INFO(
            node_.get_logger(),
            "First frame received: %dx%d, sequence=%ld",
            width_, height_, static_cast<long>(device_sequence));

          const auto & transformation = packet->getTransformation();
          if (transformation.isValid()) {
            const auto [intrinsics_width, intrinsics_height] =
              transformation.getSize();
            const auto k_rect = transformation.getIntrinsicMatrix();
            RCLCPP_INFO(
              node_.get_logger(),
              "K_rect %zux%zu: fx=%.9f, fy=%.9f, cx=%.9f, cy=%.9f",
              intrinsics_width, intrinsics_height,
              static_cast<double>(k_rect[0][0]),
              static_cast<double>(k_rect[1][1]),
              static_cast<double>(k_rect[0][2]),
              static_cast<double>(k_rect[1][2]));
            RCLCPP_INFO(
              node_.get_logger(),
              "Published output %dx%d: fx=%.9f, fy=%.9f, cx=%.9f, cy=%.9f",
              width_, height_ - output_crop_top_px_,
              static_cast<double>(k_rect[0][0]),
              static_cast<double>(k_rect[1][1]),
              static_cast<double>(k_rect[0][2]),
              static_cast<double>(k_rect[1][2]) -
              static_cast<double>(output_crop_top_px_));
          } else {
            RCLCPP_WARN(
              node_.get_logger(),
              "The first frame did not contain valid DepthAI image "
              "transformation metadata; K_rect could not be reported.");
          }
        }
      } catch (const std::exception & exception) {
        capture_errors_total_.fetch_add(1);
        RCLCPP_ERROR_THROTTLE(
          node_.get_logger(), *node_.get_clock(), 1000,
          "Camera capture error: %s", exception.what());
        std::this_thread::sleep_for(1ms);
      }
    }
  }

  static double timestampSeconds(
    const std::chrono::steady_clock::time_point & timestamp)
  {
    return std::chrono::duration<double>(
      timestamp.time_since_epoch()).count();
  }

  StabilizationTransform stabilizationTransform(
    dai::ImgFrame & packet,
    const std::chrono::steady_clock::time_point & sensor_timestamp)
  {
    if (!imu_stabilization_enabled_ || !imu_stabilizer_) {
      return StabilizationTransform{};
    }

    const auto & transformation = packet.getTransformation();
    if (!transformation.isValid()) {
      stabilization_missed_total_.fetch_add(1U);
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 5000,
        "IMU stabilization skipped because frame intrinsics are unavailable.");
      return StabilizationTransform{std::nullopt, false};
    }
    const auto intrinsics = transformation.getIntrinsicMatrix();
    const cv::Matx33d camera_matrix(
      static_cast<double>(intrinsics[0][0]), 0.0,
      static_cast<double>(intrinsics[0][2]),
      0.0, static_cast<double>(intrinsics[1][1]),
      static_cast<double>(intrinsics[1][2]),
      0.0, 0.0, 1.0);
    const cv::Matx33d fixed_view_zoom_homography =
      makeFixedViewZoomHomography(camera_matrix, fixed_view_zoom_);

    // Match stabilized_preview: calibration frames keep the same fixed FOV,
    // then initialized frames must have a timestamp-valid absolute attitude.
    if (!imu_stabilizer_->initialized()) {
      return StabilizationTransform{
        fixed_view_zoom_ > 1.0 ?
        std::optional<cv::Matx33d>(fixed_view_zoom_homography) : std::nullopt,
        true};
    }

    const double frame_timestamp_sec = timestampSeconds(sensor_timestamp);
    const double latest_imu_timestamp_sec = latest_imu_timestamp_sec_.load(
      std::memory_order_relaxed);
    if (
      std::isfinite(latest_imu_timestamp_sec) &&
      std::abs(frame_timestamp_sec - latest_imu_timestamp_sec) >
      maximum_timestamp_domain_delta_sec_)
    {
      stabilization_missed_total_.fetch_add(1U);
      RCLCPP_ERROR_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 5000,
        "RGB/IMU timestamps are not in the same clock domain.");
      return StabilizationTransform{std::nullopt, false};
    }

    const auto correction = imu_stabilizer_->correctionAt(
      frame_timestamp_sec);
    if (!correction) {
      stabilization_missed_total_.fetch_add(1U);
      return StabilizationTransform{std::nullopt, false};
    }
    if (!correction->within_correction_limit) {
      stabilization_angle_rejections_total_.fetch_add(1U);
      return StabilizationTransform{std::nullopt, false};
    }

    const cv::Matx33d stabilization_homography =
      makeImageStabilizationHomography(
      camera_matrix(0, 0), camera_matrix(1, 1),
      camera_matrix(0, 2), camera_matrix(1, 2), *correction);
    const cv::Matx33d output_homography =
      fixed_view_zoom_homography * stabilization_homography;
    if (!outputIsCoveredBySource(
        output_homography,
        cv::Size(width_, height_),
        cv::Size(width_, height_),
        fixed_view_border_margin_px_))
    {
      stabilization_crop_rejections_total_.fetch_add(1U);
      return StabilizationTransform{std::nullopt, false};
    }
    if (correction->predicted) {
      stabilization_predictions_total_.fetch_add(1U);
    }
    return StabilizationTransform{output_homography, true};
  }

  bool copy_nv12_to_message(
    dai::ImgFrame & packet,
    const std::chrono::steady_clock::time_point & sensor_timestamp,
    sensor_msgs::msg::Image & message)
  {
    const auto & nv12 = packet.getData();
    const auto stride =
      packet.getStride() > 0U ? packet.getStride() : packet.getWidth();
    const auto nv12_rows =
      static_cast<std::size_t>(packet.getHeight()) * 3U / 2U;
    const auto expected_bytes =
      static_cast<std::size_t>(stride) * nv12_rows;
    if (nv12.size() < expected_bytes) {
      throw std::runtime_error(
              "DepthAI returned an undersized NV12 frame");
    }

    const int frame_width = static_cast<int>(packet.getWidth());
    const int frame_height = static_cast<int>(packet.getHeight());
    cv::Mat input_y(
      frame_height,
      frame_width,
      CV_8UC1,
      const_cast<std::uint8_t *>(nv12.data()),
      stride);
    cv::Mat input_uv(
      frame_height / 2,
      frame_width / 2,
      CV_8UC2,
      const_cast<std::uint8_t *>(
        nv12.data() + stride * static_cast<std::size_t>(frame_height)),
      stride);
    cv::Mat stabilized_y;
    cv::Mat stabilized_uv;
    cv::Mat processed_y = input_y;
    cv::Mat processed_uv = input_uv;

    const auto transform = stabilizationTransform(
      packet, sensor_timestamp);
    if (!transform.frame_usable) {
      stabilization_output_drops_total_.fetch_add(1U);
      return false;
    }
    if (transform.homography) {
      stabilized_y.create(frame_height, frame_width, CV_8UC1);
      stabilized_uv.create(frame_height / 2, frame_width / 2, CV_8UC2);
      cv::warpPerspective(
        input_y,
        stabilized_y,
        cv::Mat(*transform.homography),
        stabilized_y.size(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT,
        cv::Scalar(0));

      const cv::Matx33d half_scale(
        0.5, 0.0, 0.0,
        0.0, 0.5, 0.0,
        0.0, 0.0, 1.0);
      const cv::Matx33d double_scale(
        2.0, 0.0, 0.0,
        0.0, 2.0, 0.0,
        0.0, 0.0, 1.0);
      const cv::Matx33d uv_homography =
        half_scale * (*transform.homography) * double_scale;
      cv::warpPerspective(
        input_uv,
        stabilized_uv,
        cv::Mat(uv_homography),
        stabilized_uv.size(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT,
        cv::Scalar(128, 128));
      processed_y = stabilized_y;
      processed_uv = stabilized_uv;
      stabilized_frames_total_.fetch_add(1U);
    }

    const int output_height = frame_height - output_crop_top_px_;
    const cv::Mat cropped_y = processed_y(
      cv::Rect(0, output_crop_top_px_, frame_width, output_height));
    const cv::Mat cropped_uv = processed_uv(
      cv::Rect(
        0,
        output_crop_top_px_ / 2,
        frame_width / 2,
        output_height / 2));

    message.width = static_cast<std::uint32_t>(frame_width);
    message.height = static_cast<std::uint32_t>(output_height);
    message.step = static_cast<std::uint32_t>(frame_width);
    const std::size_t y_bytes =
      static_cast<std::size_t>(frame_width) *
      static_cast<std::size_t>(output_height);
    const std::size_t uv_bytes = y_bytes / 2U;
    message.data.resize(y_bytes + uv_bytes);
    for (int row = 0; row < cropped_y.rows; ++row) {
      std::memcpy(
        message.data.data() +
        static_cast<std::size_t>(row) *
        static_cast<std::size_t>(frame_width),
        cropped_y.ptr(row),
        static_cast<std::size_t>(frame_width));
    }
    for (int row = 0; row < cropped_uv.rows; ++row) {
      std::memcpy(
        message.data.data() + y_bytes +
        static_cast<std::size_t>(row) *
        static_cast<std::size_t>(frame_width),
        cropped_uv.ptr(row),
        static_cast<std::size_t>(frame_width));
    }
    return true;
  }

  void imu_loop()
  {
    while (!stop_requested_.load(std::memory_order_relaxed)) {
      try {
        if (!pipeline_ || !pipeline_->isRunning()) {
          break;
        }

        auto data = imu_queue_->tryGet<dai::IMUData>();
        if (!data) {
          std::this_thread::sleep_for(200us);
          continue;
        }

        for (const auto & packet : data->packets) {
          const auto & calibrated_acceleration = packet.acceleroMeter;
          const auto & calibrated_gyroscope = packet.gyroscope;
          const double acceleration_timestamp_sec = timestampSeconds(
            calibrated_acceleration.getTimestamp());
          const double gyroscope_timestamp_sec = timestampSeconds(
            calibrated_gyroscope.getTimestamp());
          const double pair_skew_sec = std::abs(
            acceleration_timestamp_sec - gyroscope_timestamp_sec);
          if (std::isfinite(pair_skew_sec)) {
            update_maximum(
              maximum_imu_pair_skew_ns_,
              static_cast<std::uint64_t>(pair_skew_sec * 1.0e9));
          }

          const cv::Vec3d acceleration_camera =
            calibrated_imu_output_to_camera_rotation_ * cv::Vec3d(
            static_cast<double>(calibrated_acceleration.x),
            static_cast<double>(calibrated_acceleration.y),
            static_cast<double>(calibrated_acceleration.z));
          const cv::Vec3d angular_velocity_camera =
            calibrated_imu_output_to_camera_rotation_ * cv::Vec3d(
            static_cast<double>(calibrated_gyroscope.x),
            static_cast<double>(calibrated_gyroscope.y),
            static_cast<double>(calibrated_gyroscope.z));
          std::optional<cv::Vec3d> synchronized_acceleration;
          if (
            std::isfinite(pair_skew_sec) &&
            pair_skew_sec <= maximum_imu_pair_skew_sec_)
          {
            synchronized_acceleration = acceleration_camera;
          } else {
            rejected_acceleration_samples_total_.fetch_add(
              1U, std::memory_order_relaxed);
          }

          if (imu_stabilization_enabled_ && imu_stabilizer_) {
            imu_stabilizer_->update(
              synchronized_acceleration,
              angular_velocity_camera,
              gyroscope_timestamp_sec);
          }
          latest_imu_timestamp_sec_.store(
            gyroscope_timestamp_sec, std::memory_order_relaxed);
          imu_processed_total_.fetch_add(1U, std::memory_order_relaxed);
          imu_processed_interval_.fetch_add(1U, std::memory_order_relaxed);

          if (!imu_bridge_enabled_ || !imu_publisher_) {
            continue;
          }
          auto message = std::make_unique<sensor_msgs::msg::Imu>();
          message->header.stamp = ros_timestamp_for(
            calibrated_gyroscope.getTimestamp());
          message->header.frame_id = imu_frame_id_;
          // Orientation is intentionally left unset; this bridge exposes the
          // calibrated camera-frame measurements, not the private estimator.
          message->orientation_covariance[0] = -1.0;
          message->angular_velocity.x = angular_velocity_camera[0];
          message->angular_velocity.y = angular_velocity_camera[1];
          message->angular_velocity.z = angular_velocity_camera[2];
          message->linear_acceleration.x = acceleration_camera[0];
          message->linear_acceleration.y = acceleration_camera[1];
          message->linear_acceleration.z = acceleration_camera[2];
          imu_publisher_->publish(std::move(message));
          imu_published_total_.fetch_add(1U, std::memory_order_relaxed);
          imu_published_interval_.fetch_add(1U, std::memory_order_relaxed);
        }
      } catch (const std::exception & exception) {
        imu_errors_total_.fetch_add(1U, std::memory_order_relaxed);
        RCLCPP_ERROR_THROTTLE(
          node_.get_logger(), *node_.get_clock(), 1000,
          "IMU read error: %s", exception.what());
        std::this_thread::sleep_for(1ms);
      }
    }
  }

  void publish_loop()
  {
    const bool publish_every_frame =
      publish_fps_ >= sensor_fps_ * 0.999;
    const auto period = std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / publish_fps_));
    auto next_deadline = std::chrono::steady_clock::now();
    std::uint64_t published_generation = 0;

    while (!stop_requested_.load(std::memory_order_relaxed)) {
      if (publish_every_frame) {
        std::unique_lock<std::mutex> lock(wait_mutex_);
        frame_available_.wait(
          lock,
          [this, &published_generation]() {
            if (stop_requested_.load(std::memory_order_relaxed)) {
              return true;
            }
            auto latest = std::atomic_load_explicit(
              &latest_frame_, std::memory_order_acquire);
            return latest && latest->generation != published_generation;
          });
      } else {
        std::unique_lock<std::mutex> lock(wait_mutex_);
        frame_available_.wait_until(
          lock, next_deadline,
          [this]() {
            return stop_requested_.load(std::memory_order_relaxed);
          });
      }
      if (stop_requested_.load(std::memory_order_relaxed)) {
        break;
      }

      const auto now = std::chrono::steady_clock::now();
      if (!publish_every_frame && now < next_deadline) {
        continue;
      }

      auto snapshot = std::atomic_load_explicit(
        &latest_frame_, std::memory_order_acquire);
      if (snapshot && snapshot->generation != published_generation) {
        try {
          auto message = std::make_unique<sensor_msgs::msg::Image>();
          message->header.stamp = snapshot->ros_stamp;
          message->header.frame_id = frame_id_;
          message->height = snapshot->packet->getHeight();
          message->width = snapshot->packet->getWidth();
          message->encoding = "nv12";
          message->is_bigendian = false;
          const auto stabilization_started_at =
            std::chrono::steady_clock::now();
          const bool output_available = copy_nv12_to_message(
            *snapshot->packet,
            snapshot->sensor_timestamp,
            *message);
          published_generation = snapshot->generation;
          if (output_available && performance_measurement_enabled_) {
            const auto stabilization_finished_at =
              std::chrono::steady_clock::now();
            const auto stabilization_ns = static_cast<std::uint64_t>(
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                stabilization_finished_at - stabilization_started_at).count());
            stabilization_process_samples_interval_.fetch_add(
              1U, std::memory_order_relaxed);
            stabilization_process_ns_interval_.fetch_add(
              stabilization_ns, std::memory_order_relaxed);
            update_maximum(
              stabilization_process_ns_max_interval_, stabilization_ns);
            record_steady_latency(
              stabilization_finished_at - snapshot->received_timestamp,
              host_to_stabilized_samples_interval_,
              host_to_stabilized_ns_interval_,
              host_to_stabilized_ns_max_interval_);
            record_steady_latency(
              stabilization_finished_at - snapshot->sensor_timestamp,
              sensor_to_stabilized_samples_interval_,
              sensor_to_stabilized_ns_interval_,
              sensor_to_stabilized_ns_max_interval_);
          }

          if (output_available) {
            publisher_->publish(std::move(message));
            published_total_.fetch_add(1);
            published_interval_.fetch_add(1);
          }
        } catch (const std::exception & exception) {
          publish_errors_total_.fetch_add(1);
          RCLCPP_ERROR_THROTTLE(
            node_.get_logger(), *node_.get_clock(), 1000,
            "Image publish error: %s", exception.what());
        }
      }

      if (!publish_every_frame) {
        next_deadline += period;
        if (next_deadline < now - period) {
          next_deadline = now + period;
        }
      }
    }
  }

  void resize_preview_window(const cv::Mat & frame)
  {
    if (preview_window_sized_) {
      return;
    }

    double scale = 1.0;
    if (preview_max_width_ > 0) {
      scale = std::min(
        scale,
        static_cast<double>(preview_max_width_) /
        static_cast<double>(frame.cols));
    }
    if (preview_max_height_ > 0) {
      scale = std::min(
        scale,
        static_cast<double>(preview_max_height_) /
        static_cast<double>(frame.rows));
    }

    const auto window_width =
      std::max(1, static_cast<int>(frame.cols * scale));
    const auto window_height =
      std::max(1, static_cast<int>(frame.rows * scale));
    cv::resizeWindow(
      preview_window_name_, window_width, window_height);
    preview_window_sized_ = true;
  }

  void draw_preview_grid(cv::Mat & frame) const
  {
    const cv::Scalar grid_color(210, 210, 210);
    for (
      int x = preview_grid_spacing_px_;
      x < frame.cols;
      x += preview_grid_spacing_px_)
    {
      cv::line(
        frame,
        cv::Point(x, 0),
        cv::Point(x, frame.rows - 1),
        grid_color,
        1,
        cv::LINE_AA);
    }
    for (
      int y = preview_grid_spacing_px_;
      y < frame.rows;
      y += preview_grid_spacing_px_)
    {
      cv::line(
        frame,
        cv::Point(0, y),
        cv::Point(frame.cols - 1, y),
        grid_color,
        1,
        cv::LINE_AA);
    }
  }

  void preview_loop()
  {
    try {
      cv::namedWindow(preview_window_name_, cv::WINDOW_NORMAL);
    } catch (const std::exception & exception) {
      preview_active_.store(false);
      RCLCPP_ERROR(
        node_.get_logger(), "Could not create preview window: %s",
        exception.what());
      return;
    }

    const auto period = std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / preview_fps_));
    auto next_deadline = std::chrono::steady_clock::now();
    std::uint64_t previewed_generation = 0;
    bool window_was_visible = false;

    while (!stop_requested_.load(std::memory_order_relaxed)) {
      {
        std::unique_lock<std::mutex> lock(wait_mutex_);
        frame_available_.wait_until(
          lock, next_deadline,
          [this]() {return stop_requested_.load();});
      }
      if (stop_requested_.load()) {
        break;
      }

      const auto now = std::chrono::steady_clock::now();
      if (now < next_deadline) {
        continue;
      }

      try {
        auto snapshot = std::atomic_load_explicit(
          &latest_frame_, std::memory_order_acquire);
        if (snapshot && snapshot->generation != previewed_generation) {
          const auto transform = stabilizationTransform(
            *snapshot->packet, snapshot->sensor_timestamp);
          previewed_generation = snapshot->generation;
          if (!transform.frame_usable) {
            stabilization_output_drops_total_.fetch_add(1U);
          } else {
            auto preview_frame = snapshot->packet->getCvFrame();
            if (preview_frame.empty() || preview_frame.type() != CV_8UC3) {
              throw std::runtime_error(
                      "DepthAI could not convert the NV12 preview to BGR");
            }
            if (transform.homography) {
              cv::Mat stabilized;
              cv::warpPerspective(
                preview_frame,
                stabilized,
                cv::Mat(*transform.homography),
                preview_frame.size(),
                cv::INTER_LINEAR,
                cv::BORDER_CONSTANT,
                cv::Scalar(0, 0, 0));
              preview_frame = std::move(stabilized);
              stabilized_frames_total_.fetch_add(1U);
            }
            if (output_crop_top_px_ > 0) {
              preview_frame = preview_frame(
                cv::Rect(
                  0,
                  output_crop_top_px_,
                  preview_frame.cols,
                  preview_frame.rows - output_crop_top_px_)).clone();
            }
            if (preview_grid_enabled_) {
              draw_preview_grid(preview_frame);
            }
            resize_preview_window(preview_frame);
            cv::imshow(preview_window_name_, preview_frame);
            previewed_total_.fetch_add(1);
            previewed_interval_.fetch_add(1);
          }
        }

        const auto key = cv::waitKey(1) & 0xff;
        const auto visible = cv::getWindowProperty(
          preview_window_name_, cv::WND_PROP_VISIBLE);
        if (visible >= 1.0) {
          window_was_visible = true;
        }
        if (key == 'q' || key == 'Q' || key == 27 ||
          (window_was_visible && visible < 1.0))
        {
          RCLCPP_INFO(
            node_.get_logger(),
            "Preview closed; camera capture and ROS publishing continue.");
          break;
        }
      } catch (const std::exception & exception) {
        RCLCPP_ERROR(
          node_.get_logger(), "Preview stopped: %s", exception.what());
        break;
      }

      next_deadline += period;
      if (next_deadline < now - period) {
        next_deadline = now + period;
      }
    }

    preview_active_.store(false);
    try {
      cv::destroyWindow(preview_window_name_);
      cv::waitKey(1);
    } catch (const std::exception &) {
      // The GUI backend may already have destroyed the window.
    }
  }

  void report_status()
  {
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed =
      std::chrono::duration<double>(now - last_status_at_).count();
    last_status_at_ = now;

    const auto capture_count = received_interval_.exchange(0);
    const auto published_count = published_interval_.exchange(0);
    const auto preview_count = previewed_interval_.exchange(0);
    const auto dropped_count = device_drops_interval_.exchange(0);
    const auto stabilization_process_samples =
      stabilization_process_samples_interval_.exchange(
      0U, std::memory_order_relaxed);
    const auto stabilization_process_ns =
      stabilization_process_ns_interval_.exchange(
      0U, std::memory_order_relaxed);
    const auto stabilization_process_ns_max =
      stabilization_process_ns_max_interval_.exchange(
      0U, std::memory_order_relaxed);
    const auto sensor_to_host_samples =
      sensor_to_host_samples_interval_.exchange(0U, std::memory_order_relaxed);
    const auto sensor_to_host_ns =
      sensor_to_host_ns_interval_.exchange(0U, std::memory_order_relaxed);
    const auto sensor_to_host_ns_max =
      sensor_to_host_ns_max_interval_.exchange(0U, std::memory_order_relaxed);
    const auto host_to_stabilized_samples =
      host_to_stabilized_samples_interval_.exchange(
      0U, std::memory_order_relaxed);
    const auto host_to_stabilized_ns =
      host_to_stabilized_ns_interval_.exchange(0U, std::memory_order_relaxed);
    const auto host_to_stabilized_ns_max =
      host_to_stabilized_ns_max_interval_.exchange(
      0U, std::memory_order_relaxed);
    const auto sensor_to_stabilized_samples =
      sensor_to_stabilized_samples_interval_.exchange(
      0U, std::memory_order_relaxed);
    const auto sensor_to_stabilized_ns =
      sensor_to_stabilized_ns_interval_.exchange(0U, std::memory_order_relaxed);
    const auto sensor_to_stabilized_ns_max =
      sensor_to_stabilized_ns_max_interval_.exchange(
      0U, std::memory_order_relaxed);
    const auto capture_hz = static_cast<double>(capture_count) / elapsed;
    const auto published_hz = static_cast<double>(published_count) / elapsed;
    const auto preview_hz = static_cast<double>(preview_count) / elapsed;
    const auto average_stabilization_process_ms =
      stabilization_process_samples > 0U ?
      static_cast<double>(stabilization_process_ns) /
      static_cast<double>(stabilization_process_samples) / 1.0e6 :
      0.0;
    const double average_sensor_to_host_ms =
      average_milliseconds(sensor_to_host_ns, sensor_to_host_samples);
    const double average_host_to_stabilized_ms =
      average_milliseconds(host_to_stabilized_ns, host_to_stabilized_samples);
    const double average_sensor_to_stabilized_ms =
      average_milliseconds(
      sensor_to_stabilized_ns, sensor_to_stabilized_samples);
    if (imu_stream_enabled_) {
      const auto imu_count = imu_processed_interval_.exchange(0);
      const auto imu_hz = static_cast<double>(imu_count) / elapsed;
      std::string stabilization_state = "off";
      std::optional<ImageStabilizerCalibrationProgress> stabilization_progress;
      if (imu_stabilization_enabled_) {
        if (imu_stabilizer_ && imu_stabilizer_->initialized()) {
          stabilization_state = "fixed-reference-ready";
        } else if (imu_stabilizer_) {
          const auto progress = imu_stabilizer_->calibrationProgress();
          stabilization_progress = progress;
          if (progress.discarding_startup_samples) {
            stabilization_state = "discarding-startup-imu";
          } else {
            stabilization_state = "stationary-calibration";
          }
        }
      }
      if (stabilization_progress.has_value()) {
        const auto & progress = *stabilization_progress;
        if (progress.discarding_startup_samples) {
          RCLCPP_INFO(
            node_.get_logger(),
            "[warmup discard] %.2f/%.2f s "
            "(samples are intentionally not used)",
            progress.discard_elapsed_sec,
            progress.discard_target_sec);
        } else if (progress.last_rejection_reason.empty()) {
          RCLCPP_INFO(
            node_.get_logger(),
            "[calibration] %.2f/%.2f s samples=%lu resets=%lu "
            "gyro_sample=%.3f deg/s accel_conf=%.3f",
            progress.calibration_elapsed_sec,
            progress.calibration_target_sec,
            static_cast<unsigned long>(progress.accepted_samples),
            static_cast<unsigned long>(progress.reset_count),
            progress.last_angular_speed_degps,
            progress.last_accelerometer_confidence);
        } else {
          RCLCPP_INFO(
            node_.get_logger(),
            "[calibration] %.2f/%.2f s samples=%lu resets=%lu "
            "gyro_sample=%.3f deg/s accel_conf=%.3f last_reject=\"%s\"",
            progress.calibration_elapsed_sec,
            progress.calibration_target_sec,
            static_cast<unsigned long>(progress.accepted_samples),
            static_cast<unsigned long>(progress.reset_count),
            progress.last_angular_speed_degps,
            progress.last_accelerometer_confidence,
            progress.last_rejection_reason.c_str());
        }
      }
      if (performance_measurement_enabled_) {
        RCLCPP_INFO(
          node_.get_logger(),
          "[PERF][CAMERA] capture_fps=%.1f stabilized_fps=%.1f "
          "stabilized_compute_ms(avg/max)=%.3f/%.3f "
          "latency_ms(depthai_to_host_avg/max=%.2f/%.2f,"
          "host_to_stabilized_avg/max=%.2f/%.2f,"
          "depthai_to_stabilized_avg/max=%.2f/%.2f) "
          "imu_fps=%.1f stabilizer=%s warps=%lu misses=%lu predicted=%lu "
          "reject(angle/crop/output/accel)=%lu/%lu/%lu/%lu "
          "max_imu_pair_skew_ms=%.3f dropped=%lu "
          "errors(capture/publish)=%lu/%lu",
          capture_hz,
          published_hz,
          average_stabilization_process_ms,
          static_cast<double>(stabilization_process_ns_max) / 1.0e6,
          average_sensor_to_host_ms,
          static_cast<double>(sensor_to_host_ns_max) / 1.0e6,
          average_host_to_stabilized_ms,
          static_cast<double>(host_to_stabilized_ns_max) / 1.0e6,
          average_sensor_to_stabilized_ms,
          static_cast<double>(sensor_to_stabilized_ns_max) / 1.0e6,
          imu_hz,
          stabilization_state.c_str(),
          static_cast<unsigned long>(stabilized_frames_total_.load()),
          static_cast<unsigned long>(stabilization_missed_total_.load()),
          static_cast<unsigned long>(stabilization_predictions_total_.load()),
          static_cast<unsigned long>(
            stabilization_angle_rejections_total_.load()),
          static_cast<unsigned long>(
            stabilization_crop_rejections_total_.load()),
          static_cast<unsigned long>(stabilization_output_drops_total_.load()),
          static_cast<unsigned long>(
            rejected_acceleration_samples_total_.load()),
          static_cast<double>(maximum_imu_pair_skew_ns_.load()) / 1.0e6,
          static_cast<unsigned long>(dropped_count),
          static_cast<unsigned long>(capture_errors_total_.load()),
          static_cast<unsigned long>(publish_errors_total_.load()));
      } else {
        RCLCPP_INFO(
          node_.get_logger(),
          "FPS: capture=%.1f/%.1f, preview=%.1f, IMU=%.1f, "
          "stabilizer=%s (warps=%lu, misses=%lu, output_drops=%lu), "
          "dropped=%lu",
          capture_hz, sensor_fps_, preview_hz, imu_hz,
          stabilization_state.c_str(),
          static_cast<unsigned long>(stabilized_frames_total_.load()),
          static_cast<unsigned long>(stabilization_missed_total_.load()),
          static_cast<unsigned long>(stabilization_output_drops_total_.load()),
          static_cast<unsigned long>(dropped_count));
      }
    } else {
      if (performance_measurement_enabled_) {
        RCLCPP_INFO(
          node_.get_logger(),
          "[PERF][CAMERA] capture_fps=%.1f output_fps=%.1f "
          "stabilizer=off dropped=%lu errors(capture/publish)=%lu/%lu",
          capture_hz,
          published_hz,
          static_cast<unsigned long>(dropped_count),
          static_cast<unsigned long>(capture_errors_total_.load()),
          static_cast<unsigned long>(publish_errors_total_.load()));
      } else {
        RCLCPP_INFO(
          node_.get_logger(),
          "FPS: capture=%.1f/%.1f, preview=%.1f, dropped=%lu",
          capture_hz, sensor_fps_, preview_hz,
          static_cast<unsigned long>(dropped_count));
      }
    }

    const auto running_for =
      std::chrono::duration<double>(now - started_at_).count();
    if (!first_frame_received_.load() &&
      running_for >= startup_timeout_sec_ &&
      !startup_timeout_reported_.exchange(true))
    {
      RCLCPP_ERROR(
        node_.get_logger(),
        "No camera frame was received within %.1f seconds.",
        startup_timeout_sec_);
    }

    if (capture_count > 0 && capture_hz < sensor_fps_ * 0.90) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 10000,
        "Low capture FPS: %.1f/%.1f",
        capture_hz, sensor_fps_);
    }
  }

  void stop()
  {
    if (shutdown_started_.exchange(true)) {
      return;
    }

    stop_requested_.store(true);
    frame_available_.notify_all();

    join_thread(publish_thread_);
    join_thread(preview_thread_);
    join_thread(imu_thread_);
    join_thread(capture_thread_);

    if (status_timer_) {
      status_timer_->cancel();
      status_timer_.reset();
    }

    output_queue_.reset();
    imu_queue_.reset();
    if (pipeline_) {
      try {
        if (pipeline_->isRunning()) {
          pipeline_->stop();
          pipeline_->wait();
        }
      } catch (const std::exception & exception) {
        RCLCPP_WARN(
          node_.get_logger(),
          "DepthAI pipeline shutdown reported an error: %s",
          exception.what());
      }
      pipeline_.reset();
    }
  }

  static void join_thread(std::thread & thread)
  {
    if (thread.joinable() && thread.get_id() != std::this_thread::get_id()) {
      thread.join();
    }
  }

  CameraDriverNode & node_;

  bool enabled_{true};
  bool performance_measurement_enabled_{false};
  std::string camera_socket_name_;
  int width_{1280};
  int height_{720};
  double sensor_fps_{120.0};
  std::string resize_mode_name_;
  bool undistort_enabled_{true};
  int queue_size_{8};
  bool queue_blocking_{false};
  std::string frame_id_;
  std::string image_topic_;
  bool imu_bridge_enabled_{false};
  bool imu_stream_enabled_{false};
  std::string imu_topic_;
  std::string imu_frame_id_;
  double imu_rate_hz_{400.0};
  int imu_queue_size_{80};
  int imu_max_batch_reports_{5};
  double maximum_imu_pair_skew_sec_{0.003};
  double maximum_timestamp_domain_delta_sec_{1.0};
  bool imu_stabilization_enabled_{true};
  ImuImageStabilizerConfig imu_stabilizer_config_{};
  double fixed_view_zoom_{1.25};
  double fixed_view_border_margin_px_{1.5};
  int output_crop_top_px_{0};
  bool publish_enabled_{false};
  double publish_fps_{120.0};
  bool preview_enabled_{false};
  double preview_fps_{60.0};
  std::string preview_window_name_;
  int preview_max_width_{1280};
  int preview_max_height_{720};
  bool preview_grid_enabled_{true};
  int preview_grid_spacing_px_{20};
  double startup_timeout_sec_{5.0};
  double status_log_interval_sec_{1.0};
  dai::CameraBoardSocket camera_socket_{dai::CameraBoardSocket::CAM_A};
  dai::ImgResizeMode resize_mode_{dai::ImgResizeMode::CROP};

  std::unique_ptr<dai::Pipeline> pipeline_;
  std::shared_ptr<dai::MessageQueue> output_queue_;
  std::shared_ptr<dai::MessageQueue> imu_queue_;
  std::unique_ptr<ImuImageStabilizer> imu_stabilizer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::thread capture_thread_;
  std::thread imu_thread_;
  std::thread publish_thread_;
  std::thread preview_thread_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> shutdown_started_{false};
  std::atomic<bool> preview_active_{false};
  std::mutex wait_mutex_;
  std::condition_variable frame_available_;

  std::shared_ptr<const FrameSnapshot> latest_frame_;
  std::optional<std::int64_t> last_device_sequence_;
  cv::Matx33d calibrated_imu_output_to_camera_rotation_{
    cv::Matx33d::eye()};
  std::string imu_name_;
  bool preview_window_sized_{false};

  std::atomic<bool> first_frame_received_{false};
  std::atomic<bool> startup_timeout_reported_{false};
  std::atomic<bool> timestamp_fallback_reported_{false};
  std::atomic<std::uint64_t> received_total_{0};
  std::atomic<std::uint64_t> received_interval_{0};
  std::atomic<std::uint64_t> published_total_{0};
  std::atomic<std::uint64_t> published_interval_{0};
  std::atomic<std::uint64_t> previewed_total_{0};
  std::atomic<std::uint64_t> previewed_interval_{0};
  std::atomic<std::uint64_t> device_drops_total_{0};
  std::atomic<std::uint64_t> device_drops_interval_{0};
  std::atomic<std::uint64_t> capture_errors_total_{0};
  std::atomic<std::uint64_t> publish_errors_total_{0};
  std::atomic<std::uint64_t> invalid_frames_total_{0};
  std::atomic<std::uint64_t> imu_published_total_{0};
  std::atomic<std::uint64_t> imu_published_interval_{0};
  std::atomic<std::uint64_t> imu_processed_total_{0};
  std::atomic<std::uint64_t> imu_processed_interval_{0};
  std::atomic<std::uint64_t> imu_errors_total_{0};
  std::atomic<std::uint64_t> stabilized_frames_total_{0};
  std::atomic<std::uint64_t> stabilization_missed_total_{0};
  std::atomic<std::uint64_t> stabilization_angle_rejections_total_{0};
  std::atomic<std::uint64_t> stabilization_crop_rejections_total_{0};
  std::atomic<std::uint64_t> stabilization_predictions_total_{0};
  std::atomic<std::uint64_t> stabilization_output_drops_total_{0};
  std::atomic<std::uint64_t> rejected_acceleration_samples_total_{0};
  std::atomic<std::uint64_t> maximum_imu_pair_skew_ns_{0};
  std::atomic<double> latest_imu_timestamp_sec_{
    std::numeric_limits<double>::quiet_NaN()};
  std::atomic<std::uint64_t> stabilization_process_samples_interval_{0};
  std::atomic<std::uint64_t> stabilization_process_ns_interval_{0};
  std::atomic<std::uint64_t> stabilization_process_ns_max_interval_{0};
  std::atomic<std::uint64_t> sensor_to_host_samples_interval_{0};
  std::atomic<std::uint64_t> sensor_to_host_ns_interval_{0};
  std::atomic<std::uint64_t> sensor_to_host_ns_max_interval_{0};
  std::atomic<std::uint64_t> host_to_stabilized_samples_interval_{0};
  std::atomic<std::uint64_t> host_to_stabilized_ns_interval_{0};
  std::atomic<std::uint64_t> host_to_stabilized_ns_max_interval_{0};
  std::atomic<std::uint64_t> sensor_to_stabilized_samples_interval_{0};
  std::atomic<std::uint64_t> sensor_to_stabilized_ns_interval_{0};
  std::atomic<std::uint64_t> sensor_to_stabilized_ns_max_interval_{0};

  std::chrono::steady_clock::time_point started_at_;
  std::chrono::steady_clock::time_point last_status_at_;
};

CameraDriverNode::CameraDriverNode(const rclcpp::NodeOptions & options)
: Node("camera_driver", options),
  impl_(std::make_unique<Impl>(*this))
{
}

CameraDriverNode::~CameraDriverNode() = default;

}  // namespace camera_driver

RCLCPP_COMPONENTS_REGISTER_NODE(camera_driver::CameraDriverNode)
