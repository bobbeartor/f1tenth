#include <bev_handoff/runtime_monitor.hpp>
#include "camera_driver/camera_driver_node.hpp"
#include "camera_driver/imu_image_stabilizer.hpp"
#include "camera_driver/tunnel_ir_input_policy.hpp"
#include "camera_driver/ir_imu_synchronizer.hpp"
#include "camera_driver/rgb_exposure_recovery.hpp"
#include "camera_driver/msg/bev_input.hpp"
#include "bev_handoff/direct_camera_handoff.hpp"
#include "bev_handoff/direct_obstacle_handoff.hpp"
#include "point_cloud/depth_source.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <functional>
#include <iterator>
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
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "bev_handoff/stage_meter.hpp"

namespace camera_driver
{

using namespace std::chrono_literals;

namespace
{

constexpr std::uint32_t kFullSensorWidth = 1280U;
constexpr std::uint32_t kFullSensorHeight = 800U;
// Host RGB rate while the tunnel state is INSIDE (sensor rate is unchanged).
constexpr int kTunnelRgbInsideFps = 30;
constexpr double kTwoPi =
  2.0 * 3.141592653589793238462643383279502884;

std::int64_t steady_now_nanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

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

void update_maximum(
  std::atomic<double> & target,
  const double candidate)
{
  if (!std::isfinite(candidate)) {
    return;
  }
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

bool uses_gray8_transport(const dai::CameraBoardSocket socket)
{
  return
    socket == dai::CameraBoardSocket::CAM_B ||
    socket == dai::CameraBoardSocket::CAM_C;
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
    monitor_=std::make_unique<bev_handoff::RuntimeMonitor>(node_);
    read_parameters();
    if (imu_stabilization_enabled_) {
      imu_stabilizer_ =
        std::make_unique<ImuImageStabilizer>(imu_stabilizer_config_);
      if (vehicle_motion_compensation_enabled_) {
        RCLCPP_INFO(
          node_.get_logger(),
          "Vehicle acceleration compensation uses time-aligned CAN "
          "dynamics on %s.",
          can_acceleration_topic_.c_str());
      }
      auto reference_qos = rclcpp::QoS(rclcpp::KeepLast(1));
      reference_qos.reliable().transient_local();
      // Humble's intra-process path only supports volatile durability.
      rclcpp::SubscriptionOptions reference_subscription_options;
      reference_subscription_options.use_intra_process_comm =
        rclcpp::IntraProcessSetting::Disable;
      startup_ground_reference_subscription_ =
        node_.create_subscription<geometry_msgs::msg::Vector3Stamped>(
        startup_ground_reference_topic_,
        reference_qos,
        [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr message) {
          on_startup_ground_reference(*message);
        },
        reference_subscription_options);
      measured_erpm_subscription_ =
        node_.create_subscription<std_msgs::msg::Int32>(
          measured_erpm_topic_,
          rclcpp::SensorDataQoS().keep_last(1),
        [this](std_msgs::msg::Int32::ConstSharedPtr message) {
          on_measured_erpm(*message);
        });
      if (vehicle_motion_compensation_enabled_) {
        can_acceleration_subscription_ =
          node_.create_subscription<geometry_msgs::msg::Vector3Stamped>(
            can_acceleration_topic_,
            rclcpp::SensorDataQoS().keep_last(40),
          [this](
            geometry_msgs::msg::Vector3Stamped::ConstSharedPtr message)
          {
            on_can_vehicle_acceleration(*message);
          });
      }
    }

    if (!enabled_) {
      RCLCPP_WARN(node_.get_logger(), "Camera is disabled by parameter.");
      return;
    }


    if (preview_enabled_ && !graphical_display_available()) {
      preview_enabled_ = false;
      RCLCPP_WARN(
        node_.get_logger(),
        "Preview disabled because DISPLAY/WAYLAND_DISPLAY is not available.");
    }
    preview_active_.store(preview_enabled_);
    if (preview_enabled_) {
      capture_joy_subscription_ =
        node_.create_subscription<sensor_msgs::msg::Joy>(
        capture_joy_topic_,
        rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::Joy::ConstSharedPtr message) {
          on_capture_joy(std::move(message));
        });
      RCLCPP_INFO(
        node_.get_logger(),
        "Camera capture: directory=%s, joy=%s button=%d (A), keyboard=A, "
        "debounce=%.2fs, clean image only.",
        capture_directory_.c_str(),
        capture_joy_topic_.c_str(),
        capture_joy_button_,
        capture_debounce_sec_);
    }

    if (publish_enabled_) {
      auto qos = rclcpp::SensorDataQoS();
      qos.keep_last(1);
      publisher_ = node_.create_publisher<sensor_msgs::msg::Image>(
        image_topic_, qos);
    }
    if (fused_bev_output_enabled_) {
      auto qos = rclcpp::SensorDataQoS();
      qos.keep_last(1);
      fused_bev_publisher_ = node_.create_publisher<
        camera_driver::msg::BevInput>(fused_bev_topic_, qos);
    }
    if (imu_bridge_enabled_) {
      auto qos = rclcpp::SensorDataQoS();
      qos.keep_last(5);
      imu_publisher_ = node_.create_publisher<sensor_msgs::msg::Imu>(
        imu_topic_, qos);
    }

    try {
      start_pipeline();
      node_.declare_parameter<std::vector<std::string>>("runtime_editable_parameters",{"publish_fps"});
      runtime_fps_callback_=node_.add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& values) {
          rcl_interfaces::msg::SetParametersResult result;result.successful=false;
          double fps=publish_fps_.load();
          for(const auto& value:values) {
            if(value.get_name()!="publish_fps" || value.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE) {
              result.reason="Only publish_fps is live; restart for sensor changes";return result;
            }
            fps=value.as_double();
            if(!std::isfinite(fps) || fps<1 || fps>sensor_fps_) {
              result.reason="publish_fps must be 1..sensor_fps";return result;
            }
          }
          publish_fps_.store(fps);result.successful=true;return result;
        });

      started_at_ = std::chrono::steady_clock::now();
      last_status_at_ = started_at_;
      const auto status_period =
        std::chrono::duration<double>(status_log_interval_sec_);
      status_timer_ = node_.create_wall_timer(
        status_period, std::bind(&Impl::report_status, this));
      // Remote preview stage 1a (host receipt -> IMU-stabilized output), 1 s windows.
      perf_publisher_ = node_.create_publisher<std_msgs::msg::String>(
        "/perf/camera_stage", rclcpp::QoS(1).reliable());
      perf_started_at_ = std::chrono::steady_clock::now();
      perf_timer_ = node_.create_wall_timer(std::chrono::seconds(1), [this]() {
          const auto now = std::chrono::steady_clock::now();
          const double elapsed = std::chrono::duration<double>(now - perf_started_at_).count();
          perf_started_at_ = now;
          std_msgs::msg::String message;
          message.data = "{\"capture\":" + capture_meter_.drain(elapsed) +
            ",\"stabilize\":" + stabilize_meter_.drain(elapsed) + "}";
          perf_publisher_->publish(message);
        });
      capture_thread_ = std::thread(&Impl::capture_loop, this);
      if (depth_enabled_) {depth_thread_=std::thread(&Impl::depth_loop,this);}
      if (tunnel_enabled_) {ir_thread_=std::thread(&Impl::ir_loop,this);}
      if (tunnel_enabled_ && rgb_exposure_recovery_enabled_) {
        rgb_exposure_thread_=std::thread(&Impl::rgb_exposure_loop,this);
      }
      if (imu_stream_enabled_) {
        imu_thread_ = std::thread(&Impl::imu_loop, this);
      }
      if (publish_enabled_ || fused_bev_output_enabled_) {
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
    bool dynamic_correction_applied{false};
  };

  struct VehicleAxesCamera
  {
    cv::Vec3d forward;
    cv::Vec3d left;
    cv::Vec3d up;
  };

  struct TimedVehicleAcceleration
  {
    double timestamp_sec;
    double longitudinal_mps2;
    double lateral_mps2;
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
    perf_log_enabled_ = node_.declare_parameter<bool>("perf_log_enabled", false);
    camera_socket_name_ =
      node_.declare_parameter<std::string>("camera_socket", "CAM_A");
    width_ = node_.declare_parameter<int>("width", 1280);
    height_ = node_.declare_parameter<int>("height", 800);
    sensor_fps_ = node_.declare_parameter<double>("sensor_fps", 70.0);
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
    high_frequency_only_ = node_.declare_parameter<bool>(
      "imu_stabilization_high_frequency_only", false);
    high_frequency_vibration_cutoff_hz_ = node_.declare_parameter<double>(
      "imu_stabilization_high_frequency_vibration_cutoff_hz", 3.0);
    can_acceleration_topic_ = node_.declare_parameter<std::string>(
      "imu_stabilization_can_acceleration_topic",
      "/vehicle/dynamics/acceleration");
    can_acceleration_frame_id_ = node_.declare_parameter<std::string>(
      "imu_stabilization_can_acceleration_frame_id", "base_link");
    can_acceleration_timeout_sec_ = node_.declare_parameter<double>(
      "imu_stabilization_can_acceleration_timeout_sec", 0.10);
    can_longitudinal_compensation_gain_ = node_.declare_parameter<double>(
      "imu_stabilization_can_longitudinal_compensation_gain", 1.0);
    can_lateral_compensation_gain_ = node_.declare_parameter<double>(
      "imu_stabilization_can_lateral_compensation_gain", 1.0);
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
      "imu_stabilization_accelerometer_time_constant_sec", 1.5);
    imu_stabilizer_config_.acceleration_correction_gate_deg =
      node_.declare_parameter<double>(
      "imu_stabilization_accelerometer_direction_gate_deg", 4.3);
    imu_stabilizer_config_.roll_acceleration_correction_time_constant_sec =
      node_.declare_parameter<double>(
      "imu_stabilization_roll_accelerometer_time_constant_sec", 2.0);
    imu_stabilizer_config_.roll_acceleration_direction_gate_deg =
      node_.declare_parameter<double>(
      "imu_stabilization_roll_accelerometer_direction_gate_deg", 4.3);
    imu_stabilizer_config_.acceleration_correction_stationary_only =
      node_.declare_parameter<bool>(
      "imu_stabilization_accelerometer_stationary_only", false);
    imu_stabilizer_config_.moving_accelerometer_nudge_enabled =
      node_.declare_parameter<bool>(
      "imu_stabilization_moving_accelerometer_nudge_enabled", true);
    imu_stabilizer_config_.moving_accelerometer_nudge_time_constant_sec =
      node_.declare_parameter<double>(
      "imu_stabilization_moving_accelerometer_nudge_time_constant_sec", 0.15);
    imu_stabilizer_config_.moving_accelerometer_nudge_strength =
      node_.declare_parameter<double>(
      "imu_stabilization_moving_accelerometer_nudge_strength", 0.15);
    imu_stabilizer_config_.moving_accelerometer_pitch_nudge_maximum_deg =
      node_.declare_parameter<double>(
      "imu_stabilization_moving_accelerometer_pitch_nudge_maximum_deg", 0.20);
    imu_stabilizer_config_.moving_accelerometer_roll_nudge_maximum_deg =
      node_.declare_parameter<double>(
      "imu_stabilization_moving_accelerometer_roll_nudge_maximum_deg", 0.15);
    imu_stabilizer_config_.moving_gravity_anchor_maximum_correction_rate_degps =
      node_.declare_parameter<double>(
      "imu_stabilization_moving_gravity_anchor_maximum_correction_rate_degps",
      0.50);
    startup_ground_reference_topic_ = node_.declare_parameter<std::string>(
      "imu_stabilization_external_reference_topic",
      "/camera/startup_ground_normal");
    imu_stabilizer_config_.external_reference_required =
      node_.declare_parameter<bool>(
      "imu_stabilization_external_reference_required", false);
    measured_erpm_topic_ = node_.declare_parameter<std::string>(
      "imu_stabilization_measured_erpm_topic", "/vesc/measured_erpm");
    stationary_erpm_enter_threshold_ = node_.declare_parameter<int>(
      "imu_stabilization_stationary_erpm_enter_threshold", 100);
    stationary_erpm_exit_threshold_ = node_.declare_parameter<int>(
      "imu_stabilization_stationary_erpm_exit_threshold", 500);
    stationary_erpm_filter_time_constant_sec_ = node_.declare_parameter<double>(
      "imu_stabilization_stationary_erpm_filter_time_constant_sec", 0.15);
    stationary_erpm_enter_duration_sec_ = node_.declare_parameter<double>(
      "imu_stabilization_stationary_erpm_enter_duration_sec", 1.0);
    measured_erpm_timeout_sec_ = node_.declare_parameter<double>(
      "imu_stabilization_measured_erpm_timeout_sec", 1.0);
    vehicle_motion_compensation_enabled_ = node_.declare_parameter<bool>(
      "imu_stabilization_vehicle_motion_compensation_enabled", true);
    maximum_longitudinal_acceleration_mps2_ =
      node_.declare_parameter<double>(
      "imu_stabilization_maximum_longitudinal_acceleration_mps2", 15.0);
    maximum_lateral_acceleration_mps2_ = node_.declare_parameter<double>(
      "imu_stabilization_maximum_lateral_acceleration_mps2", 15.0);
    imu_stabilizer_config_.reference_tilt_leak_time_constant_sec =
      node_.declare_parameter<double>(
      "imu_stabilization_reference_tilt_leak_time_constant_sec", 4.0);
    imu_stabilizer_config_.stationary_tilt_recovery_time_constant_sec =
      node_.declare_parameter<double>(
      "imu_stabilization_stationary_tilt_recovery_time_constant_sec", 0.35);
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
    imu_stabilizer_config_.gyroscope_correction_gain =
      node_.declare_parameter<double>(
      "imu_stabilization_gyroscope_correction_gain", 1.0);
    imu_stabilizer_config_.maximum_correction_deg =
      node_.declare_parameter<double>(
      "imu_stabilization_maximum_correction_deg", 3.0);
    imu_stabilizer_config_.maximum_frame_imu_wait_sec =
      node_.declare_parameter<double>(
      "imu_stabilization_maximum_frame_imu_wait_sec", 0.005);
    imu_stabilizer_config_.maximum_frame_imu_age_sec =
      node_.declare_parameter<double>(
      "imu_stabilization_maximum_frame_imu_age_sec", 0.012);
    imu_stabilizer_config_.maximum_frame_imu_prediction_sec =
      node_.declare_parameter<double>(
      "imu_stabilization_maximum_prediction_sec", 0.015);
    invalid_correction_hold_frames_ = node_.declare_parameter<int>(
      "imu_stabilization_invalid_correction_hold_frames", 2);
    fixed_view_zoom_ =
      node_.declare_parameter<double>("fixed_view_zoom", 1.25);
    fixed_view_border_margin_px_ = node_.declare_parameter<double>(
      "fixed_view_border_margin_px", 1.5);
    output_crop_top_px_ =
      node_.declare_parameter<int>("output_crop_top_px", 0);
    publish_enabled_ =
      node_.declare_parameter<bool>("publish_enabled", false);
    publish_fps_ =
      node_.declare_parameter<double>("publish_fps", 70.0);
    fused_bev_output_enabled_ =
      node_.declare_parameter<bool>("fused_bev_output_enabled", false);
    fused_bev_topic_ = node_.declare_parameter<std::string>(
      "fused_bev_topic", "/camera/bev_input");
    bev_input_bottom_fraction_ = node_.declare_parameter<double>(
      "bev_input_bottom_fraction", 0.70);
    preview_enabled_ =
      node_.declare_parameter<bool>("preview_enabled", false);
    preview_fps_ =
      node_.declare_parameter<double>("preview_fps", 60.0);
    preview_window_name_ = node_.declare_parameter<std::string>(
      "preview_window_name", "OAK fixed-reference pitch-roll stabilization");
    preview_max_width_ =
      node_.declare_parameter<int>("preview_max_width", 1280);
    preview_max_height_ =
      node_.declare_parameter<int>("preview_max_height", 800);
    preview_grid_enabled_ =
      node_.declare_parameter<bool>("preview_grid_enabled", false);
    preview_grid_spacing_px_ =
      node_.declare_parameter<int>("preview_grid_spacing_px", 20);
    capture_directory_ =
      node_.declare_parameter<std::string>("capture_directory", ".");
    capture_joy_topic_ =
      node_.declare_parameter<std::string>("capture_joy_topic", "/joy");
    capture_joy_button_ =
      node_.declare_parameter<int>("capture_joy_button", 0);
    capture_debounce_sec_ =
      node_.declare_parameter<double>("capture_debounce_sec", 0.30);
    startup_timeout_sec_ =
      node_.declare_parameter<double>("startup_timeout_sec", 5.0);
    status_log_interval_sec_ =
      node_.declare_parameter<double>("status_log_interval_sec", 1.0);

    if (
      !std::isfinite(high_frequency_vibration_cutoff_hz_) ||
      high_frequency_vibration_cutoff_hz_ <= 0.0 ||
      high_frequency_vibration_cutoff_hz_ >= 0.5 * imu_rate_hz_)
    {
      throw std::invalid_argument(
              "imu_stabilization_high_frequency_vibration_cutoff_hz must "
              "be positive and below half the IMU rate");
    }
    if (high_frequency_only_) {
      vehicle_motion_compensation_enabled_ = false;
      imu_stabilizer_config_.acceleration_correction_stationary_only = true;
      imu_stabilizer_config_.moving_accelerometer_nudge_enabled = false;
      imu_stabilizer_config_.reference_tilt_leak_time_constant_sec =
        1.0 / (kTwoPi * high_frequency_vibration_cutoff_hz_);
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
    if (
      !std::isfinite(bev_input_bottom_fraction_) ||
      bev_input_bottom_fraction_ <= 0.0 ||
      bev_input_bottom_fraction_ > 1.0)
    {
      throw std::invalid_argument(
              "bev_input_bottom_fraction must be in (0, 1]");
    }
    fused_bev_crop_height_ = static_cast<int>(2 * std::llround(
        static_cast<double>(height_) * bev_input_bottom_fraction_ / 2.0));
    fused_bev_crop_height_ = std::clamp(
      fused_bev_crop_height_, 2, height_);
    fused_bev_crop_top_ = height_ - fused_bev_crop_height_;
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
      if (
        imu_stabilization_enabled_ &&
        startup_ground_reference_topic_.empty())
      {
        throw std::invalid_argument(
                "imu_stabilization_external_reference_topic must not be "
                "empty when stabilization is enabled");
      }
      if (
        imu_stabilization_enabled_ &&
        (measured_erpm_topic_.empty() ||
        stationary_erpm_enter_threshold_ < 0 ||
        stationary_erpm_exit_threshold_ <=
        stationary_erpm_enter_threshold_ ||
        !std::isfinite(stationary_erpm_filter_time_constant_sec_) ||
        stationary_erpm_filter_time_constant_sec_ <= 0.0 ||
        !std::isfinite(stationary_erpm_enter_duration_sec_) ||
        stationary_erpm_enter_duration_sec_ <= 0.0 ||
        !std::isfinite(measured_erpm_timeout_sec_) ||
        measured_erpm_timeout_sec_ <= 0.0))
      {
        throw std::invalid_argument(
                "invalid measured-ERPM stationary detection parameters");
      }
      if (
        imu_stabilization_enabled_ &&
        vehicle_motion_compensation_enabled_)
      {
        if (
          can_acceleration_topic_.empty() ||
          can_acceleration_frame_id_.empty() ||
          !std::isfinite(can_acceleration_timeout_sec_) ||
          can_acceleration_timeout_sec_ <= 0.0 ||
          !std::isfinite(can_longitudinal_compensation_gain_) ||
          can_longitudinal_compensation_gain_ < 0.0 ||
          can_longitudinal_compensation_gain_ > 1.0 ||
          !std::isfinite(can_lateral_compensation_gain_) ||
          can_lateral_compensation_gain_ < 0.0 ||
          can_lateral_compensation_gain_ > 1.0 ||
          !std::isfinite(maximum_longitudinal_acceleration_mps2_) ||
          maximum_longitudinal_acceleration_mps2_ <= 0.0 ||
          !std::isfinite(maximum_lateral_acceleration_mps2_) ||
          maximum_lateral_acceleration_mps2_ <= 0.0)
        {
          throw std::invalid_argument(
                  "invalid CAN vehicle acceleration compensation parameters");
        }
        if (!imu_bridge_enabled_) {
          RCLCPP_WARN(
            node_.get_logger(),
            "CAN vehicle acceleration compensation is enabled without the "
            "internal IMU bridge; the dynamics monitor needs another fresh "
            "yaw-rate source for lateral acceleration.");
        }
      } else if (imu_stabilization_enabled_) {
        if (imu_stabilizer_config_.moving_accelerometer_nudge_enabled) {
          RCLCPP_WARN(
            node_.get_logger(),
            "Vehicle motion compensation is off; disabling bounded moving "
            "accelerometer nudge to reject raw vehicle acceleration.");
          imu_stabilizer_config_.moving_accelerometer_nudge_enabled = false;
        }
        if (!imu_stabilizer_config_.acceleration_correction_stationary_only) {
          RCLCPP_WARN(
            node_.get_logger(),
            "Vehicle motion compensation is off; overriding "
            "accelerometer_stationary_only=true to reject raw moving "
            "acceleration.");
          imu_stabilizer_config_.acceleration_correction_stationary_only = true;
        }
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
    if (
      invalid_correction_hold_frames_ < 0 ||
      invalid_correction_hold_frames_ > 5)
    {
      throw std::invalid_argument(
              "imu_stabilization_invalid_correction_hold_frames must be "
              "between 0 and 5");
    }
    if (publish_enabled_ || fused_bev_output_enabled_) {
      require_positive(publish_fps_.load(), "publish_fps");
      if (publish_enabled_ && image_topic_.empty()) {
        throw std::invalid_argument(
                "image_topic must not be empty when publishing is enabled");
      }
      if (fused_bev_output_enabled_ && fused_bev_topic_.empty()) {
        throw std::invalid_argument(
                "fused_bev_topic must not be empty when fused BEV output is enabled");
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
    if (
      capture_directory_.empty() || capture_joy_topic_.empty() ||
      capture_joy_button_ < 0 ||
      !std::isfinite(capture_debounce_sec_) || capture_debounce_sec_ < 0.0)
    {
      throw std::invalid_argument(
              "capture directory/topic must not be empty and button must "
              "be non-negative; debounce must be finite/non-negative");
    }

    camera_socket_ = parse_camera_socket(camera_socket_name_);
    gray8_transport_ = uses_gray8_transport(camera_socket_);
    resize_mode_ = parse_resize_mode(resize_mode_name_);
    last_valid_stabilization_homography_.setMaximumHoldFrames(
      static_cast<std::size_t>(invalid_correction_hold_frames_));
  }

  bool set_vehicle_axes_from_up_camera(const cv::Vec3d & up_camera)
  {
    const double up_norm = cv::norm(up_camera);
    if (!std::isfinite(up_norm) || up_norm <= 1.0e-6) {
      return false;
    }
    const cv::Vec3d up = up_camera / up_norm;
    const cv::Vec3d optical_forward(0.0, 0.0, 1.0);
    cv::Vec3d forward = optical_forward - up * optical_forward.dot(up);
    const double forward_norm = cv::norm(forward);
    if (!std::isfinite(forward_norm) || forward_norm <= 1.0e-6) {
      return false;
    }
    forward /= forward_norm;
    cv::Vec3d left = up.cross(forward);
    const double left_norm = cv::norm(left);
    if (!std::isfinite(left_norm) || left_norm <= 1.0e-6) {
      return false;
    }
    left /= left_norm;
    std::lock_guard<std::mutex> lock(vehicle_axes_mutex_);
    vehicle_axes_camera_ = VehicleAxesCamera{forward, left, up};
    return true;
  }

  void ensure_vehicle_axes_from_stabilizer_reference()
  {
    if (!vehicle_motion_compensation_enabled_ || !imu_stabilizer_) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(vehicle_axes_mutex_);
      if (vehicle_axes_camera_.has_value()) {
        return;
      }
    }
    const auto reference_up = imu_stabilizer_->referenceUpCamera();
    if (!reference_up || !set_vehicle_axes_from_up_camera(*reference_up)) {
      return;
    }
    if (!fallback_vehicle_axes_reported_.exchange(true)) {
      RCLCPP_INFO(
        node_.get_logger(),
        "Vehicle axes derived from the calibrated IMU startup reference; "
        "no external BEV ground reference was received.");
    }
  }

  void on_startup_ground_reference(
    const geometry_msgs::msg::Vector3Stamped & message)
  {
    if (!imu_stabilizer_) {
      return;
    }
    if (
      !message.header.frame_id.empty() &&
      message.header.frame_id != frame_id_)
    {
      RCLCPP_ERROR(
        node_.get_logger(),
        "Rejected BEV startup ground reference in frame '%s'; expected '%s'.",
        message.header.frame_id.c_str(), frame_id_.c_str());
      return;
    }
    const cv::Vec3d up_camera(
      message.vector.x, message.vector.y, message.vector.z);
    if (!imu_stabilizer_->setExternalReferenceUpCamera(up_camera)) {
      if (!late_external_reference_reported_.exchange(true)) {
        RCLCPP_WARN(
          node_.get_logger(),
          "Ignored invalid or late BEV startup ground reference; the virtual "
          "gimbal reference is immutable after calibration.");
      }
      return;
    }
    if (
      vehicle_motion_compensation_enabled_ &&
      !set_vehicle_axes_from_up_camera(up_camera))
    {
      RCLCPP_ERROR(
        node_.get_logger(),
        "Cannot derive vehicle forward axis from startup ground normal.");
    }
    RCLCPP_INFO(
      node_.get_logger(),
      "Accepted BEV startup ground reference in %s: "
      "normal=(%.6f, %.6f, %.6f).",
      frame_id_.c_str(), up_camera[0], up_camera[1], up_camera[2]);
  }

  void on_can_vehicle_acceleration(
    const geometry_msgs::msg::Vector3Stamped & message)
  {
    if (
      !message.header.frame_id.empty() &&
      message.header.frame_id != can_acceleration_frame_id_)
    {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 5000,
        "Rejected CAN vehicle acceleration in frame '%s'; expected '%s'.",
        message.header.frame_id.c_str(),
        can_acceleration_frame_id_.c_str());
      can_acceleration_rejected_total_.fetch_add(
        1U, std::memory_order_relaxed);
      return;
    }
    if (
      !std::isfinite(message.vector.x) ||
      !std::isfinite(message.vector.y))
    {
      can_acceleration_rejected_total_.fetch_add(
        1U, std::memory_order_relaxed);
      return;
    }

    const std::int64_t received_ns = steady_now_nanoseconds();
    double timestamp_sec = static_cast<double>(received_ns) * 1.0e-9;
    const std::int64_t header_ns =
      static_cast<std::int64_t>(message.header.stamp.sec) * 1000000000LL +
      static_cast<std::int64_t>(message.header.stamp.nanosec);
    if (header_ns > 0) {
      const std::int64_t ros_now_ns = node_.get_clock()->now().nanoseconds();
      const double ros_delta_sec =
        static_cast<double>(header_ns - ros_now_ns) * 1.0e-9;
      if (
        std::isfinite(ros_delta_sec) &&
        std::abs(ros_delta_sec) <= maximum_timestamp_domain_delta_sec_)
      {
        timestamp_sec += ros_delta_sec;
      }
    }

    const TimedVehicleAcceleration sample{
      timestamp_sec,
      std::clamp(
        message.vector.x,
        -maximum_longitudinal_acceleration_mps2_,
        maximum_longitudinal_acceleration_mps2_),
      std::clamp(
        message.vector.y,
        -maximum_lateral_acceleration_mps2_,
        maximum_lateral_acceleration_mps2_)};
    {
      std::lock_guard<std::mutex> lock(can_acceleration_mutex_);
      if (
        !can_acceleration_history_.empty() &&
        timestamp_sec <= can_acceleration_history_.back().timestamp_sec)
      {
        can_acceleration_history_.back().longitudinal_mps2 =
          sample.longitudinal_mps2;
        can_acceleration_history_.back().lateral_mps2 = sample.lateral_mps2;
      } else {
        can_acceleration_history_.push_back(sample);
      }
      const double newest_timestamp_sec =
        can_acceleration_history_.back().timestamp_sec;
      const double history_duration_sec = std::max(
        1.0, 2.0 * can_acceleration_timeout_sec_);
      while (
        can_acceleration_history_.size() > 2U &&
        newest_timestamp_sec -
        can_acceleration_history_.front().timestamp_sec >
        history_duration_sec)
      {
        can_acceleration_history_.pop_front();
      }
    }
    last_can_acceleration_received_ns_.store(
      received_ns, std::memory_order_relaxed);
    latest_longitudinal_acceleration_mps2_.store(
      sample.longitudinal_mps2, std::memory_order_relaxed);
    latest_lateral_acceleration_mps2_.store(
      sample.lateral_mps2, std::memory_order_relaxed);
    can_acceleration_received_total_.fetch_add(
      1U, std::memory_order_relaxed);
  }

  std::optional<TimedVehicleAcceleration> can_acceleration_at(
    const double timestamp_sec)
  {
    if (!std::isfinite(timestamp_sec)) {
      return std::nullopt;
    }
    std::lock_guard<std::mutex> lock(can_acceleration_mutex_);
    if (can_acceleration_history_.empty()) {
      return std::nullopt;
    }
    const auto next = std::lower_bound(
      can_acceleration_history_.begin(),
      can_acceleration_history_.end(),
      timestamp_sec,
      [](const TimedVehicleAcceleration & sample, const double time) {
        return sample.timestamp_sec < time;
      });
    if (next == can_acceleration_history_.begin()) {
      if (
        next->timestamp_sec - timestamp_sec >
        can_acceleration_timeout_sec_)
      {
        return std::nullopt;
      }
      return *next;
    }
    if (next == can_acceleration_history_.end()) {
      const TimedVehicleAcceleration & latest =
        can_acceleration_history_.back();
      if (
        timestamp_sec - latest.timestamp_sec >
        can_acceleration_timeout_sec_)
      {
        return std::nullopt;
      }
      return latest;
    }

    const TimedVehicleAcceleration & previous = *std::prev(next);
    if (
      timestamp_sec - previous.timestamp_sec >
      can_acceleration_timeout_sec_ ||
      next->timestamp_sec - timestamp_sec >
      can_acceleration_timeout_sec_)
    {
      return std::nullopt;
    }
    const double interval_sec = next->timestamp_sec - previous.timestamp_sec;
    if (interval_sec <= 0.0) {
      return *next;
    }
    const double amount = std::clamp(
      (timestamp_sec - previous.timestamp_sec) / interval_sec,
      0.0, 1.0);
    return TimedVehicleAcceleration{
      timestamp_sec,
      previous.longitudinal_mps2 + amount *
      (next->longitudinal_mps2 - previous.longitudinal_mps2),
      previous.lateral_mps2 + amount *
      (next->lateral_mps2 - previous.lateral_mps2)};
  }

  void on_measured_erpm(const std_msgs::msg::Int32 & message)
  {
    const std::int64_t now_ns = steady_now_nanoseconds();
    const std::int64_t previous_ns =
      last_measured_erpm_received_ns_.exchange(
      now_ns, std::memory_order_relaxed);
    latest_measured_erpm_.store(message.data, std::memory_order_relaxed);

    const auto timeout_ns = static_cast<std::int64_t>(
      measured_erpm_timeout_sec_ * 1.0e9);
    const bool reset_filter =
      previous_ns <= 0 || now_ns - previous_ns > timeout_ns;
    if (
      reset_filter)
    {
      erpm_stationary_.store(false, std::memory_order_relaxed);
      stationary_erpm_candidate_started_ns_.store(
        0, std::memory_order_relaxed);
    }

    const std::int64_t raw_absolute_erpm = std::abs(
      static_cast<std::int64_t>(message.data));
    double filtered_absolute_erpm = static_cast<double>(raw_absolute_erpm);
    if (!reset_filter) {
      const double elapsed_sec =
        static_cast<double>(now_ns - previous_ns) * 1.0e-9;
      const double alpha = -std::expm1(
        -elapsed_sec / stationary_erpm_filter_time_constant_sec_);
      const double previous_filtered_absolute_erpm =
        latest_filtered_absolute_erpm_.load(std::memory_order_relaxed);
      filtered_absolute_erpm = previous_filtered_absolute_erpm +
        alpha *
        (static_cast<double>(raw_absolute_erpm) -
        previous_filtered_absolute_erpm);
    }
    latest_filtered_absolute_erpm_.store(
      filtered_absolute_erpm, std::memory_order_relaxed);

    // A real departure must cancel stationary recovery immediately. Filtering
    // is intentionally used only for entry so it cannot delay this transition.
    if (raw_absolute_erpm >= stationary_erpm_exit_threshold_) {
      stationary_erpm_candidate_started_ns_.store(
        0, std::memory_order_relaxed);
      erpm_stationary_.store(false, std::memory_order_relaxed);
      return;
    }

    if (filtered_absolute_erpm <= stationary_erpm_enter_threshold_) {
      if (erpm_stationary_.load(std::memory_order_relaxed)) {
        return;
      }
      const std::int64_t candidate_started_ns =
        stationary_erpm_candidate_started_ns_.load(
        std::memory_order_relaxed);
      if (candidate_started_ns <= 0) {
        stationary_erpm_candidate_started_ns_.store(
          now_ns, std::memory_order_relaxed);
        return;
      }
      const auto enter_duration_ns = static_cast<std::int64_t>(
        stationary_erpm_enter_duration_sec_ * 1.0e9);
      if (now_ns - candidate_started_ns >= enter_duration_ns) {
        erpm_stationary_.store(true, std::memory_order_relaxed);
      }
      return;
    }

    stationary_erpm_candidate_started_ns_.store(
      0, std::memory_order_relaxed);
  }

  bool measured_erpm_is_fresh(const std::int64_t now_ns) const
  {
    const std::int64_t received_ns =
      last_measured_erpm_received_ns_.load(std::memory_order_relaxed);
    if (received_ns <= 0 || now_ns < received_ns) {
      return false;
    }
    return static_cast<double>(now_ns - received_ns) * 1.0e-9 <=
           measured_erpm_timeout_sec_;
  }

  bool measured_erpm_stationary()
  {
    if (!measured_erpm_is_fresh(steady_now_nanoseconds())) {
      erpm_stationary_.store(false, std::memory_order_relaxed);
      stationary_erpm_candidate_started_ns_.store(
        0, std::memory_order_relaxed);
      return false;
    }
    return erpm_stationary_.load(std::memory_order_relaxed);
  }

  std::optional<VehicleAxesCamera> vehicle_axes_camera()
  {
    std::lock_guard<std::mutex> lock(vehicle_axes_mutex_);
    return vehicle_axes_camera_;
  }

  void start_pipeline()
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only=true;
    descriptor.description="Edit auto_control/config/obstacles.yaml and restart to change shared depth";
    depth_enabled_=node_.declare_parameter<bool>("obstacles.depth.enabled",false,descriptor);
    if (depth_enabled_) {
      if (camera_socket_!=dai::CameraBoardSocket::CAM_A || !fused_bev_output_enabled_) {
        throw std::invalid_argument("Obstacle depth requires CAM_A and fused BEV output");
      }
      depth_config_.resolution=node_.declare_parameter<std::string>("obstacles.camera.resolution","400p",descriptor);
      depth_config_.fps=node_.declare_parameter<double>("obstacles.camera.fps",30.0,descriptor);
      depth_config_.mode=node_.declare_parameter<std::string>("obstacles.depth.mode","high_density",descriptor);
      depth_config_.dot_intensity=node_.declare_parameter<double>("obstacles.depth.ir_dot_projector_intensity",0.7,descriptor);
      depth_config_.flood_intensity=node_.declare_parameter<double>("obstacles.depth.ir_flood_light_intensity",0.0,descriptor);
      depth_config_.confidence=node_.declare_parameter<int>("obstacles.depth.confidence_threshold",200,descriptor);
      depth_config_.lr_check=node_.declare_parameter<bool>("obstacles.depth.left_right_check",true,descriptor);
      depth_config_.lr_threshold=node_.declare_parameter<int>("obstacles.depth.left_right_threshold",10,descriptor);
      depth_config_.subpixel=node_.declare_parameter<bool>("obstacles.depth.subpixel",true,descriptor);
      depth_config_.subpixel_bits=node_.declare_parameter<int>("obstacles.depth.subpixel_fractional_bits",3,descriptor);
      depth_config_.median=node_.declare_parameter<std::string>("obstacles.depth.median_filter","off",descriptor);
      point_cloud::validate(depth_config_);
    }
    tunnel_enabled_=node_.declare_parameter<bool>("tunnel.enabled",false);
    tunnel_validation_mode_=node_.declare_parameter<bool>("tunnel.validation_mode",false);
    tunnel_ir_imu_enabled_=node_.declare_parameter<bool>("tunnel.ir_imu_enabled",true);
    tunnel_ir_imu_max_age_sec_=node_.declare_parameter<double>("tunnel.ir_imu_max_age_sec",0.05);
    tunnel_ir_imu_wait_sec_=node_.declare_parameter<double>("tunnel.ir_imu_wait_sec",0.10);
    rgb_exposure_recovery_enabled_=node_.declare_parameter<bool>("tunnel.rgb_exit_exposure_recovery_enabled",true);
    rgb_exposure_max_hold_sec_=node_.declare_parameter<double>("tunnel.rgb_exit_exposure_max_hold_sec",0.20);
    // Validate even when disabled, so an invalid startup setting is visible.
    (void)RgbExposureRecovery(rgb_exposure_max_hold_sec_);
    if (!std::isfinite(tunnel_ir_imu_max_age_sec_) || tunnel_ir_imu_max_age_sec_<=0 ||
      tunnel_ir_imu_max_age_sec_>0.1 || !std::isfinite(tunnel_ir_imu_wait_sec_) ||
      tunnel_ir_imu_wait_sec_<=0 || tunnel_ir_imu_wait_sec_>0.2) {
      throw std::invalid_argument("Tunnel IR IMU max age must be (0,0.1]s and wait (0,0.2]s");
    }
    if (tunnel_enabled_) {
      if (camera_socket_!=dai::CameraBoardSocket::CAM_A || !imu_stabilization_enabled_) {
        throw std::runtime_error("Tunnel integration requires CAM_A and IMU stabilization");
      }
      tcn_publisher_=node_.create_publisher<sensor_msgs::msg::Image>(
        "/tunnel/rgb_raw",rclcpp::SensorDataQoS().keep_last(1));
      rclcpp::SubscriptionOptions tunnel_options;
      tunnel_options.use_intra_process_comm=rclcpp::IntraProcessSetting::Disable;
      tunnel_mode_sub_=node_.create_subscription<std_msgs::msg::String>(
        "/tunnel/target_source",rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr message) {
          if (message->data=="IR" || message->data=="RGB") {
            tunnel_requested_.store(message->data=="IR");
          }
        },tunnel_options);
      tunnel_state_sub_=node_.create_subscription<std_msgs::msg::String>(
        "/tunnel/state",rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr message) {
          if (message->data=="OUTSIDE") {rgb_tunnel_phase_.store(RgbTunnelPhase::outside);}
          else if (message->data=="ENTERING") {rgb_tunnel_phase_.store(RgbTunnelPhase::entering);}
          else if (message->data=="INSIDE") {rgb_tunnel_phase_.store(RgbTunnelPhase::inside);}
          else if (message->data=="EXITING") {rgb_tunnel_phase_.store(RgbTunnelPhase::exiting);}
        },tunnel_options);
      // RGB remains 70 FPS. Stereo/IR use 30 FPS so the verified 30ms
      // tunnel exposure fits within a sensor frame without rebuilding OAK.
      depth_config_.fps=30.0;
      sensor_fps_=70.0;
      publish_fps_=70.0;
    }
    const auto shared_device_id=(depth_enabled_ || tunnel_enabled_) ? bev_handoff::obstacleDeviceId() : std::string{};
    if ((depth_enabled_ || tunnel_enabled_) && shared_device_id.empty()) {
      throw std::runtime_error("Load bev_processor with obstacles.share_reference before camera_driver in the same container");
    }
    auto device=shared_device_id.empty() ? std::make_shared<dai::Device>(dai::UsbSpeed::SUPER) :
      std::make_shared<dai::Device>(dai::DeviceInfo(shared_device_id),dai::UsbSpeed::SUPER);
    if (depth_enabled_ || tunnel_enabled_) {depth_calibration_=std::make_shared<dai::CalibrationHandler>(device->getCalibration());}
    tunnel_device_=device;
    pipeline_ = std::make_unique<dai::Pipeline>(device);
    if (tunnel_enabled_) {pipeline_->setAutoCalibrationMode(dai::Pipeline::AutoCalibrationMode::OFF);}
    pipeline_->setXLinkChunkSize(0);
    std::shared_ptr<dai::node::StereoDepth> stereo;
    if (depth_enabled_) {
      pipeline_->setAutoCalibrationMode(dai::Pipeline::AutoCalibrationMode::OFF);
      stereo=point_cloud::addStereoDepth(*pipeline_,depth_config_,tunnel_enabled_ ? &tunnel_cameras_ : nullptr,
        tunnel_enabled_ ? &depth_gate_control_ : nullptr);
      depth_queue_=stereo->depth.createOutputQueue(1,false);
      bev_handoff::publishDirectDepth(nullptr);
    }

    dai::Node::Output * ir_output=nullptr;
    if (tunnel_enabled_) {
      if (!tunnel_cameras_.left) {
        tunnel_cameras_.left=pipeline_->create<dai::node::Camera>()->build(
          dai::CameraBoardSocket::CAM_B,std::make_pair(1280,800),30.0F);
      }
      ir_output=tunnel_cameras_.left->requestOutput(
        std::make_pair(1280,800),dai::ImgFrame::Type::GRAY8,dai::ImgResizeMode::CROP,30.0F,true);
      // Outside the tunnel the lane IR stream is unused: stop it on the device
      // so 1280x800 frames are not sent over USB only to be discarded.
      auto ir_gate=pipeline_->create<dai::node::Script>();
      ir_gate->setScript(R"PY(
active = False
while True:
    control = node.io['control'].tryGet()
    if control is not None:
        active = bool(control.getData()[0])
    frame = node.io['image'].get()
    if active:
        node.io['out'].send(frame)
)PY");
      ir_output->link(ir_gate->inputs["image"]);
      for (const auto * name:{"image","control"}) {
        ir_gate->inputs[name].setMaxSize(1);
        ir_gate->inputs[name].setBlocking(false);
      }
      ir_gate_control_=ir_gate->inputs["control"].createInputQueue();
      ir_output=&ir_gate->outputs["out"];
      ir_queue_=ir_output->createOutputQueue(1,false);
      ir_controls_.push_back(tunnel_cameras_.left->inputControl.createInputQueue());
      if (tunnel_cameras_.right) {
        ir_controls_.push_back(tunnel_cameras_.right->inputControl.createInputQueue());
      }
    }

    // CAM_A is the native color/NV12 path. The stereo sensors on CAM_B/C are
    // native monochrome, so keep them GRAY8 on the device and synthesize
    // neutral chroma only when adapting to the existing NV12 ROS/BEV API.
    auto camera = pipeline_->create<dai::node::Camera>()->build(
      camera_socket_,
      std::make_pair(kFullSensorWidth, kFullSensorHeight),
      static_cast<float>(sensor_fps_));
    if (tunnel_enabled_ && rgb_exposure_recovery_enabled_) {
      rgb_controls_=camera->inputControl.createInputQueue();
    }
    const auto transport_type = gray8_transport_ ?
      dai::ImgFrame::Type::GRAY8 : dai::ImgFrame::Type::NV12;
    auto * output = camera->requestOutput(
      std::make_pair(
        static_cast<std::uint32_t>(width_),
        static_cast<std::uint32_t>(height_)),
      transport_type,
      resize_mode_,
      static_cast<float>(sensor_fps_),
      undistort_enabled_);
    if (tunnel_enabled_) {
      // INSIDE only needs RGB for TCN exit detection (and 5 FPS RGB lane standby).
      // Keep the sensor rate (and its exposure behaviour) unchanged and forward
      // ~30 FPS to the host instead; every frame passes otherwise.
      const int sensor_rate=static_cast<int>(std::lround(sensor_fps_));
      auto rgb_gate=pipeline_->create<dai::node::Script>();
      rgb_gate->setScript(
        "sensor = " + std::to_string(sensor_rate) + "\n"
        "target = " + std::to_string(kTunnelRgbInsideFps) + "\n" R"PY(
decimate = False
last_slot = None
while True:
    control = node.io['control'].tryGet()
    if control is not None:
        decimate = bool(control.getData()[0])
        last_slot = None
    frame = node.io['in'].get()
    if not decimate:
        node.io['out'].send(frame)
        continue
    slot = (frame.getSequenceNum() * target) // sensor
    if slot != last_slot:
        last_slot = slot
        node.io['out'].send(frame)
)PY");
      output->link(rgb_gate->inputs["in"]);
      rgb_gate->inputs["in"].setMaxSize(2);
      rgb_gate->inputs["in"].setBlocking(false);
      rgb_gate->inputs["control"].setMaxSize(1);
      rgb_gate->inputs["control"].setBlocking(false);
      rgb_gate_control_=rgb_gate->inputs["control"].createInputQueue();
      output=&rgb_gate->outputs["out"];
    }

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

    if (tunnel_enabled_ && (!imu_stream_enabled_ || !imu_stabilizer_)) {
      throw std::runtime_error("Tunnel IR requires a working calibrated IMU; refusing unstabilized fallback");
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

    if (ir_output) {
      const auto bridge=ir_output->getXLinkBridge();
      if (!bridge || !bridge->xLinkOut) {throw std::runtime_error("Missing IR XLink bridge");}
      bridge->xLinkOut->input.setMaxSize(1);
      bridge->xLinkOut->input.setBlocking(false);
    }

    if (stereo) {
      const auto bridge=stereo->depth.getXLinkBridge();
      if (!bridge || !bridge->xLinkOut) {throw std::runtime_error("Missing depth XLink bridge");}
      bridge->xLinkOut->input.setMaxSize(1);
      bridge->xLinkOut->input.setBlocking(false);
    }
    pipeline_->start();
    if (depth_enabled_) {
      if (!device->setIrLaserDotProjectorIntensity(static_cast<float>(depth_config_.dot_intensity)) && depth_config_.dot_intensity>0) {
        throw std::runtime_error("IR dot unavailable: set obstacles.depth.ir_dot_projector_intensity to 0.0");
      }
      if (!device->setIrFloodLightIntensity(static_cast<float>(depth_config_.flood_intensity)) && depth_config_.flood_intensity>0) {
        throw std::runtime_error("IR flood unavailable: set obstacles.depth.ir_flood_light_intensity to 0.0");
      }
      RCLCPP_INFO(node_.get_logger(),"Shared RGB/depth device=%s, stereo=%s @ %.1f requested FPS",
        shared_device_id.c_str(),depth_config_.resolution.c_str(),depth_config_.fps);
    }

    RCLCPP_INFO(
      node_.get_logger(),
      "OAK: full sensor %dx%d @ %.1f FPS on %s, USB=%s, "
      "transport=%s, XLink chunks=off, XLink device queue=1/non-blocking",
      width_, height_, sensor_fps_, camera_socket_name_.c_str(),
      usb_speed_name(device->getUsbSpeed()),
      gray8_transport_ ? "GRAY8 (host neutral-chroma NV12 adapter)" : "NV12");
    RCLCPP_INFO(
      node_.get_logger(),
      "Options: undistort=%s, image_publish=%s, fused_bev_publish=%s, "
      "preview=%s, "
      "preview_grid=%s/%dpx, imu_stabilization=%s, "
      "fixed_view_zoom=%.2fx, output_crop=top %dpx -> %dx%d, "
      "BEV_input=bottom %.1f%%/top %dpx -> %dx%d on %s, queue=%d/%s",
      undistort_enabled_ ? "on" : "off",
      publish_enabled_ ? "on" : "off",
      fused_bev_output_enabled_ ? "on" : "off",
      preview_enabled_ ? "on" : "off",
      preview_grid_enabled_ ? "on" : "off",
      preview_grid_spacing_px_,
      imu_stabilization_enabled_ ? "on" : "off",
      fixed_view_zoom_,
      output_crop_top_px_,
      width_,
      height_ - output_crop_top_px_,
      100.0 * static_cast<double>(fused_bev_crop_height_) /
      static_cast<double>(height_),
      fused_bev_crop_top_,
      width_,
      fused_bev_crop_height_,
      fused_bev_topic_.c_str(),
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
        "Virtual-gimbal stabilization: keep camera still for %.1f s "
        "startup discard + %.1f s stationary calibration; yaw-free tilt, "
        "%s correction (cutoff=%.2fHz, gyro gain=%.2f), "
        "CAN vehicle acceleration=%s on %s "
        "(timeout=%.3fs, gain_long/lat=%.2f/%.2f), "
        "BEV reference=%s on %s, runtime stationary=%s "
        "(filtered enter<=%d, raw exit>=%d, filter tau=%.2fs, hold=%.2fs), "
        "moving accel=%s (anchor tau pitch/roll=%.2f/%.2fs, "
        "anchor rate<=%.2fdeg/s, nudge tau=%.2fs, strength=%.2f, "
        "pitch/roll cap=%.2f/%.2fdeg), "
        "pitch/roll limit %.1f deg, "
        "invalid correction policy=hold-last-%d-frames then zoom-only",
        imu_stabilizer_config_.startup_discard_duration_sec,
        imu_stabilizer_config_.reference_calibration_duration_sec,
        high_frequency_only_ ? "high-frequency-only" : "full-band",
        high_frequency_only_ ? high_frequency_vibration_cutoff_hz_ : 0.0,
        imu_stabilizer_config_.gyroscope_correction_gain,
        vehicle_motion_compensation_enabled_ ? "on" : "off",
        can_acceleration_topic_.c_str(),
        can_acceleration_timeout_sec_,
        can_longitudinal_compensation_gain_,
        can_lateral_compensation_gain_,
        imu_stabilizer_config_.external_reference_required ?
        "required" : "optional",
        startup_ground_reference_topic_.c_str(),
        measured_erpm_topic_.c_str(),
        stationary_erpm_enter_threshold_,
        stationary_erpm_exit_threshold_,
        stationary_erpm_filter_time_constant_sec_,
        stationary_erpm_enter_duration_sec_,
        !imu_stabilizer_config_.acceleration_correction_stationary_only ?
        (imu_stabilizer_config_.moving_accelerometer_nudge_enabled ?
        "persistent-anchor+bounded-nudge" : "persistent-anchor") :
        (imu_stabilizer_config_.moving_accelerometer_nudge_enabled ?
        "bounded-nudge-only" : "off"),
        imu_stabilizer_config_.acceleration_correction_time_constant_sec,
        imu_stabilizer_config_.roll_acceleration_correction_time_constant_sec,
        imu_stabilizer_config_.
          moving_gravity_anchor_maximum_correction_rate_degps,
        imu_stabilizer_config_.moving_accelerometer_nudge_time_constant_sec,
        imu_stabilizer_config_.moving_accelerometer_nudge_strength,
        imu_stabilizer_config_.moving_accelerometer_pitch_nudge_maximum_deg,
        imu_stabilizer_config_.moving_accelerometer_roll_nudge_maximum_deg,
        imu_stabilizer_config_.maximum_correction_deg,
        invalid_correction_hold_frames_);
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

  void rgb_exposure_loop()
  {
    RgbExposureRecovery recovery(rgb_exposure_max_hold_sec_);
    auto previous_phase=RgbTunnelPhase::outside;
    RCLCPP_INFO(node_.get_logger(),"RGB exit exposure recovery enabled; automatic exposure resumes after 2 matching frames or <=%.0fms",
      rgb_exposure_max_hold_sec_*1000.);
    while (!stop_requested_.load()) {
      try {
        const auto observed=bev_handoff::latestCameraExposure(false);
        std::optional<RgbExposureRecovery::Sample> sample;
        if (observed) {sample=RgbExposureRecovery::Sample{observed->captured_at,observed->exposure_us,observed->iso};}
        const auto phase=rgb_tunnel_phase_.load();
        const auto action=recovery.step(phase,std::chrono::steady_clock::now(),sample);
        if (previous_phase==RgbTunnelPhase::outside && phase!=previous_phase) {
          const auto saved=recovery.saved();
          if (saved) {
            RCLCPP_INFO(node_.get_logger(),"[RGB EXPOSURE] exterior snapshot %ldus ISO%d",saved->exposure_us,saved->iso);
          } else {
            RCLCPP_INFO(node_.get_logger(),"[RGB EXPOSURE] no stable exterior snapshot; keeping AE on exit");
          }
        }
        if (phase==RgbTunnelPhase::exiting && previous_phase!=phase && action.command==RgbExposureRecovery::Command::none) {
          RCLCPP_INFO(node_.get_logger(),"[RGB EXPOSURE] EXITING: keep AE (no snapshot/fresh RGB metadata, or exposure already low)");
        }
        previous_phase=phase;
        if (action.command!=RgbExposureRecovery::Command::none) {
          auto control=std::make_shared<dai::CameraControl>();
          if (action.command==RgbExposureRecovery::Command::manual) {
            bev_handoff::publishRgbExposureControl(bev_handoff::RgbExposureControl::restoring);
            control->setManualExposure(action.exposure_us,action.iso);
          } else {
            bev_handoff::publishRgbExposureControl(bev_handoff::RgbExposureControl::auto_pending);
            control->setAutoExposureEnable();
          }
          rgb_controls_->send(control);
          if (action.command==RgbExposureRecovery::Command::automatic) {
            recovery.autoSent();
            bev_handoff::publishRgbExposureControl(bev_handoff::RgbExposureControl::automatic);
            RCLCPP_INFO(node_.get_logger(),"[RGB EXPOSURE] automatic exposure re-enabled");
          } else {
            RCLCPP_INFO(node_.get_logger(),"[RGB EXPOSURE] EXITING: temporary restore %ldus ISO%d; bounded hold %.0fms",
              action.exposure_us,action.iso,rgb_exposure_max_hold_sec_*1000.);
          }
        }
      } catch (const std::exception & e) {
        // Sending a manual command may have succeeded before the error. Keep
        // retrying AE until the queue accepts it; never retry the manual pulse.
        recovery.requestAuto();
        bev_handoff::publishRgbExposureControl(bev_handoff::RgbExposureControl::auto_pending);
        RCLCPP_ERROR_THROTTLE(node_.get_logger(),*node_.get_clock(),1000,"RGB exposure recovery: %s; restoring AE",e.what());
        std::this_thread::sleep_for(20ms);
      }
      std::this_thread::sleep_for(2ms);
    }
    if (recovery.restoring()) {
      try {
        auto control=std::make_shared<dai::CameraControl>(); control->setAutoExposureEnable(); rgb_controls_->send(control);
      } catch (const std::exception &) {} // The pipeline is closed immediately after workers join.
    }
    bev_handoff::publishRgbExposureControl(bev_handoff::RgbExposureControl::automatic);
  }

  void ir_loop()
  {
    bool applied=false;
    std::optional<RgbTunnelPhase> routed_phase;
    bool dirty=true;
    auto changed_at=std::chrono::steady_clock::time_point{};
    auto last=std::chrono::steady_clock::time_point{};
    auto retry_at=last;
    RCLCPP_INFO(node_.get_logger(),"Tunnel IR input mode=%s",
      tunnel_validation_mode_ ? "standalone validation (no settle/age/exposure gate)" : "guarded");
    RCLCPP_INFO(node_.get_logger(),"Tunnel IR IMU=%s; max bracket sample age=%.0fms, pending wait=%.0fms",
      tunnel_ir_imu_enabled_ ? "ON (capture-time synchronized)" : "OFF (fixed startup ground pose)",
      tunnel_ir_imu_max_age_sec_*1000.,tunnel_ir_imu_wait_sec_*1000.);
    IrImuSynchronizer<std::shared_ptr<dai::ImgFrame>,ImageStabilizationCorrection> pending(tunnel_ir_imu_wait_sec_);
    std::uint64_t previous_expired=0,previous_overflow=0,previous_superseded=0;
    auto report_started=std::chrono::steady_clock::now();
    std::uint64_t received=0,published=0,time_rejected=0,exposure_rejected=0,imu_tilt_rejected=0;
    long last_exposure_us=0; int last_iso=0;
    while (!stop_requested_.load()) {
      try {
        const bool requested=tunnel_requested_.load();
        const auto phase=rgb_tunnel_phase_.load();
        // OUTSIDE: no lane IR. INSIDE: no obstacle stereo, RGB ~30 FPS for TCN.
        // ENTERING/EXITING keep every stream so either lane source is available.
        const bool ir_active=phase!=RgbTunnelPhase::outside;
        const bool depth_active=phase!=RgbTunnelPhase::inside;
        const bool rgb_decimate=phase==RgbTunnelPhase::inside;
        if (!routed_phase || *routed_phase!=phase) {
          const auto send_gate=[](const auto & queue,bool active) {
            if (!queue) {return;}
            auto control=std::make_shared<dai::Buffer>();
            control->setData(std::vector<std::uint8_t>{static_cast<std::uint8_t>(active)});
            queue->send(control);
          };
          send_gate(ir_gate_control_,ir_active);
          send_gate(depth_gate_control_,depth_active);
          send_gate(rgb_gate_control_,rgb_decimate);
          rgb_decimated_.store(rgb_decimate);
          if (!ir_active) {pending.clear(); bev_handoff::publishDirectIr(nullptr);}
          if (!depth_active) {bev_handoff::publishDirectDepth(nullptr); bev_handoff::clearObstacles();}
          routed_phase=phase;
          RCLCPP_INFO(node_.get_logger(),"OAK routes: IR=%s depth=%s RGB=%s",
            ir_active ? "ON" : "OFF",depth_enabled_ ? (depth_active ? "ON" : "OFF") : "disabled",
            rgb_decimate ? "30 FPS" : "full");
        }
        const auto now=std::chrono::steady_clock::now();
        if (now-report_started>=2s) {
          if (requested) {
            const double elapsed=std::chrono::duration<double>(now-report_started).count();
            RCLCPP_INFO(node_.get_logger(),
              "[IR CAMERA] received=%.1fHz published=%.1fHz rejected(time/exposure/imu_tilt)=%llu/%llu/%llu "
              "imu_wait_expired=%llu imu_queue_overflow=%llu superseded=%llu pending=%zu imu_initialized=%s actual=%ldus ISO%d",
              received/elapsed,published/elapsed,static_cast<unsigned long long>(time_rejected),
              static_cast<unsigned long long>(exposure_rejected),static_cast<unsigned long long>(imu_tilt_rejected),
              static_cast<unsigned long long>(pending.expired()-previous_expired),
              static_cast<unsigned long long>(pending.overflow()-previous_overflow),
              static_cast<unsigned long long>(pending.superseded()-previous_superseded),pending.size(),
              imu_stabilizer_->initialized() ? "yes" : "no",last_exposure_us,last_iso);
          }
          received=published=time_rejected=exposure_rejected=imu_tilt_rejected=0; report_started=now;
          previous_expired=pending.expired(); previous_overflow=pending.overflow();
          previous_superseded=pending.superseded();
        }
        if ((dirty || requested!=applied) && now>=retry_at) {
          dirty=true;
          tunnel_profile_ready_.store(false);
          // One owner changes BOTH stereo exposures and the shared IR lights.
          for (const auto & queue:ir_controls_) {
            auto control=std::make_shared<dai::CameraControl>();
            if (requested) {control->setManualExposure(30000,800);}
            else {control->setAutoExposureEnable();}
            queue->send(control);
          }
          const double dot=requested ? 0.0 : (depth_enabled_ ? depth_config_.dot_intensity : 0.0);
          const double flood=requested ? 0.5 : (depth_enabled_ ? depth_config_.flood_intensity : 0.0);
          if (!tunnel_device_->setIrLaserDotProjectorIntensity(static_cast<float>(dot)) ||
            !tunnel_device_->setIrFloodLightIntensity(static_cast<float>(flood))) {
            throw std::runtime_error("Tunnel IR lighting command failed");
          }
          applied=requested;
          dirty=false;
          tunnel_profile_applied_.store(applied);
          changed_at=now;
          pending.clear();
          depth_resume_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
            (now+150ms).time_since_epoch()).count());
          tunnel_profile_ready_.store(true);
          bev_handoff::publishDirectIr(nullptr);
          bev_handoff::publishDirectDepth(nullptr);
          bev_handoff::clearObstacles();
          RCLCPP_INFO(node_.get_logger(),"Camera profile=%s",applied ? "IR 30ms ISO800 dot0 flood0.5" : "OUTSIDE auto exposure/depth lights restored");
        }
        if (!ir_active) {
          // Drain anything sent before the device gate closed.
          ir_queue_->tryGet<dai::ImgFrame>();
          std::this_thread::sleep_for(1ms);
          continue;
        }
        auto packet=ir_queue_->tryGet<dai::ImgFrame>();
        if (packet) {
          ++received;
          last_exposure_us=packet->getExposureTime().count(); last_iso=packet->getSensitivity();
          bev_handoff::publishCameraExposure(true,std::make_shared<bev_handoff::CameraExposureObservation>(
            bev_handoff::CameraExposureObservation{last_exposure_us,last_iso,
              packet->getTimestamp(dai::CameraExposureOffset::MIDDLE)}));
          if (dirty) {continue;}
          const auto captured=packet->getTimestamp(dai::CameraExposureOffset::MIDDLE);
          const auto rejection=tunnelIrRejection(tunnel_validation_mode_,applied,
            captured,last,now,changed_at,last_exposure_us,last_iso);
          if (rejection==TunnelIrRejection::time) {++time_rejected; continue;}
          if (rejection==TunnelIrRejection::exposure) {++exposure_rejected; continue;}
          last=captured;
          if (packet->getType()!=dai::ImgFrame::Type::GRAY8) {throw std::runtime_error("Expected LEFT GRAY8");}
          if (tunnel_ir_imu_enabled_) {pending.push(packet,captured,now);}
        }
        cv::Matx33d tilt=cv::Matx33d::eye();
        if (tunnel_ir_imu_enabled_) {
          // Keep captures briefly while sensor streams arrive independently.
          // Poll without blocking the IMU thread; latest matched capture wins.
          const auto match=pending.takeReady(std::chrono::steady_clock::now(),[this](auto captured) {
            return imu_stabilizer_->correctionAt(
              std::chrono::duration<double>(captured.time_since_epoch()).count(),
              tunnel_ir_imu_max_age_sec_,0.0);
          });
          if (!match) {std::this_thread::sleep_for(1ms); continue;}
          if (!match->correction.within_correction_limit) {++imu_tilt_rejected; continue;}
          packet=match->entry.frame;
          tilt=match->correction.camera_to_reference_tilt;
        } else if (!packet || dirty) {std::this_thread::sleep_for(1ms); continue;}
        const auto captured=packet->getTimestamp(dai::CameraExposureOffset::MIDDLE);
        const auto transform=point_cloud::rgbFromFrame(*packet,*depth_calibration_);
        const auto k=packet->getTransformation().getIntrinsicMatrix();
        auto frame=std::make_shared<bev_handoff::DirectIrFrame>();
        frame->mono=packet->getFrame(false); frame->owner=packet;
        frame->intrinsics={k[0][0],k[1][1],k[0][2],k[1][2]};
        for (int r=0;r<3;++r) {
          frame->view_center_rgb_m[r]=transform.translation[r];
          for (int c=0;c<3;++c) {frame->rgb_from_view(r,c)=transform.rotation[r*3+c];}
        }
        frame->current_rgb_to_reference=tilt;
        frame->captured_at=captured; frame->header.stamp=ros_timestamp_for(captured);
        frame->header.frame_id="front_axle_bev"; frame->tunnel_profile=applied;
        bev_handoff::publishDirectIr(std::move(frame));
        ++published;
      } catch (const std::exception & e) {
        bev_handoff::publishDirectIr(nullptr);
        pending.clear();
        // Repeated profile failures must not hammer device control or expose
        // frames as if the requested profile had been applied.
        retry_at=std::chrono::steady_clock::now()+1s;
        RCLCPP_ERROR_THROTTLE(node_.get_logger(),*node_.get_clock(),1000,"Tunnel IR input: %s",e.what());
        std::this_thread::sleep_for(10ms);
      }
    }
    bev_handoff::publishDirectIr(nullptr);
  }

  void depth_loop()
  {
    auto last=std::chrono::steady_clock::time_point{};
    while (!stop_requested_.load(std::memory_order_relaxed)) {
      try {
        if (!pipeline_->isRunning()) {break;}
        auto packet=depth_queue_->tryGet<dai::ImgFrame>();
        if (!packet) {std::this_thread::sleep_for(1ms); continue;}
        if (tunnel_enabled_ && (!tunnel_profile_ready_.load() || tunnel_requested_.load() || tunnel_profile_applied_.load() ||
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            packet->getTimestamp(dai::CameraExposureOffset::MIDDLE).time_since_epoch()).count()<depth_resume_ns_.load())) {
          bev_handoff::publishDirectDepth(nullptr);
          continue;
        }
        const auto captured=packet->getTimestamp(dai::CameraExposureOffset::MIDDLE);
        const double age=std::chrono::duration<double>(std::chrono::steady_clock::now()-captured).count();
        if (age<0 || age>0.25 || captured<=last) {continue;}
        last=captured;
        if (packet->getType()!=dai::ImgFrame::Type::RAW16) {throw std::runtime_error("Expected RAW16 stereo depth");}
        const auto transform=point_cloud::rgbFromFrame(*packet,*depth_calibration_);
        const auto k=packet->getTransformation().getIntrinsicMatrix();
        const auto & bytes=packet->getData();
        auto frame=std::make_shared<bev_handoff::DirectDepthFrame>();
        frame->data=bytes.data(); frame->size=bytes.size(); frame->owner=packet;
        frame->width=packet->getWidth(); frame->height=packet->getHeight();
        frame->stride=packet->getStride() ? packet->getStride() : packet->getWidth()*2;
        frame->intrinsics={k[0][0],k[1][1],k[0][2],k[1][2]};
        frame->rgb_from_depth_rotation=transform.rotation;
        frame->rgb_from_depth_translation=transform.translation;
        frame->header.stamp=ros_timestamp_for(captured); frame->header.frame_id=frame_id_;
        frame->captured_at=captured;
        bev_handoff::publishDirectDepth(std::move(frame));
      } catch (const std::exception & e) {
        bev_handoff::publishDirectDepth(nullptr);
        RCLCPP_ERROR_THROTTLE(node_.get_logger(),*node_.get_clock(),1000,"Depth input: %s",e.what());
        std::this_thread::sleep_for(10ms);
      }
    }
    bev_handoff::publishDirectDepth(nullptr);
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
        // Stabilization is synchronized to the middle of camera exposure, not
        // host arrival time or the beginning/end of exposure.
        const auto sensor_timestamp = packet->getTimestamp(
          dai::CameraExposureOffset::MIDDLE);
        bev_handoff::publishCameraExposure(false,std::make_shared<bev_handoff::CameraExposureObservation>(
          bev_handoff::CameraExposureObservation{static_cast<long>(packet->getExposureTime().count()),static_cast<int>(packet->getSensitivity()),sensor_timestamp}));
        // INSIDE frames arrive already decimated to ~30 FPS on the device; pass
        // them all (the 33.3ms grid would drop some of the uneven 2/3-frame gaps).
        const bool tcn_decimated=rgb_decimated_.load();
        if (tunnel_enabled_ && packet->getType()==dai::ImgFrame::Type::NV12 &&
          (last_tcn_frame_==std::chrono::steady_clock::time_point{} ||
          (tcn_decimated ? sensor_timestamp>=last_tcn_frame_+25ms : sensor_timestamp>=last_tcn_frame_+33333333ns))) {
          // Publish raw, unstabilized luminance at approximately 30Hz for the
          // existing TCN feature history. Keep source capture stamps and padding.
          auto message=std::make_unique<sensor_msgs::msg::Image>();
          message->header.stamp=ros_timestamp_for(sensor_timestamp);
          message->header.frame_id=frame_id_; message->encoding="nv12";
          message->width=packet->getWidth(); message->height=packet->getHeight();
          message->step=packet->getStride() ? packet->getStride() : packet->getWidth();
          const auto & frame_bytes=packet->getData();
          message->data.assign(frame_bytes.begin(),frame_bytes.end());
          tcn_publisher_->publish(std::move(message));
          // Advance a source-time grid rather than quantizing 80Hz to 26.7Hz.
          if (last_tcn_frame_==std::chrono::steady_clock::time_point{} || tcn_decimated) {last_tcn_frame_=sensor_timestamp;}
          else {last_tcn_frame_+=33333333ns; if (sensor_timestamp-last_tcn_frame_>33333333ns) {last_tcn_frame_=sensor_timestamp;}}
        }
        capture_meter_.add();
        if (perf_log_enabled_) {
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

        if (publish_enabled_ || fused_bev_output_enabled_ || preview_enabled_) {
          const auto expected_type = gray8_transport_ ?
            dai::ImgFrame::Type::GRAY8 : dai::ImgFrame::Type::NV12;
          const bool valid_frame =
            packet->getType() == expected_type &&
            static_cast<int>(packet->getWidth()) == width_ &&
            static_cast<int>(packet->getHeight()) == height_;
          if (!valid_frame) {
            invalid_frames_total_.fetch_add(1);
            RCLCPP_ERROR_THROTTLE(
              node_.get_logger(), *node_.get_clock(), 1000,
              "Expected a %dx%d %s frame from DepthAI.",
              width_, height_, gray8_transport_ ? "GRAY8" : "NV12");
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

        if(monitor_->enabled())monitor_->sample("rgb_capture",0.);
        if (bev_handoff::hasDirectCameraConsumer()) {
          if (packet->getType() == dai::ImgFrame::Type::NV12) {
            const auto & data = packet->getData();
            auto frame = std::make_shared<bev_handoff::DirectCameraFrame>();
            frame->nv12 = data.data();
            frame->size = data.size();
            frame->width = packet->getWidth();
            frame->height = packet->getHeight();
            frame->stride = packet->getStride() > 0U ? packet->getStride() : packet->getWidth();
            frame->owner = packet;
            frame->header.stamp = ros_timestamp_for(sensor_timestamp);
            frame->header.frame_id = frame_id_;
            frame->captured_at = sensor_timestamp;
            frame->generation = generation;
            bev_handoff::deliverDirectCameraFrame(std::move(frame));
          } else {
            RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
              "Traffic signal color detection requires CAM_A color NV12; grayscale is unsupported.");
          }
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
    const std::chrono::steady_clock::time_point & sensor_timestamp,
    const std::uint64_t frame_generation)
  {
    const bool fixed_view_zoom_enabled = fixed_view_zoom_ > 1.0;
    if (
      !fixed_view_zoom_enabled &&
      (!imu_stabilization_enabled_ || !imu_stabilizer_))
    {
      return StabilizationTransform{};
    }

    const auto & transformation = packet.getTransformation();
    if (!transformation.isValid()) {
      stabilization_missed_total_.fetch_add(1U);
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 5000,
        "IMU stabilization skipped because frame intrinsics are unavailable.");
      // The BEV input model includes the fixed zoom, so an unzoomed raw frame
      // would be geometrically wrong. This is the one case that must drop.
      return StabilizationTransform{std::nullopt, false, false};
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
    const auto zoom_only = [&]() {
        return StabilizationTransform{
          fixed_view_zoom_enabled ?
          std::optional<cv::Matx33d>(fixed_view_zoom_homography) :
          std::nullopt,
          true,
          false};
      };
    const auto held_or_zoom_only = [&]() {
        const auto held_homography =
          last_valid_stabilization_homography_.reuseForFrame(
          frame_generation);
        if (!held_homography.has_value()) {
          return zoom_only();
        }
        stabilization_held_last_total_.fetch_add(
          1U, std::memory_order_relaxed);
        return StabilizationTransform{
          held_homography, true, true};
      };

    // The fixed crop is independent of IMU stabilization. This keeps the
    // calibrated 1.25x camera view when pitch/roll correction is disabled.
    if (!imu_stabilization_enabled_ || !imu_stabilizer_) {
      return zoom_only();
    }

    // Calibration frames keep the same fixed FOV. Initialized frames use a
    // timestamp-valid relative tilt from the immutable startup view.
    if (!imu_stabilizer_->initialized()) {
      return zoom_only();
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
        "Camera/IMU timestamps are not in the same clock domain.");
      return held_or_zoom_only();
    }

    const auto correction = imu_stabilizer_->correctionAt(
      frame_timestamp_sec);
    if (!correction) {
      stabilization_missed_total_.fetch_add(1U);
      return held_or_zoom_only();
    }
    latest_stabilization_roll_error_deg_.store(
      correction->roll_error_deg, std::memory_order_relaxed);
    latest_stabilization_pitch_error_deg_.store(
      correction->pitch_error_deg, std::memory_order_relaxed);
    latest_stabilization_correction_angle_deg_.store(
      correction->correction_angle_deg, std::memory_order_relaxed);
    if (!correction->within_correction_limit) {
      stabilization_angle_rejections_total_.fetch_add(1U);
      last_valid_stabilization_homography_.clear();
      return zoom_only();
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
      last_valid_stabilization_homography_.clear();
      return zoom_only();
    }
    if (correction->predicted) {
      stabilization_predictions_total_.fetch_add(1U);
    }
    last_valid_stabilization_homography_.remember(output_homography);
    return StabilizationTransform{output_homography, true, true};
  }

  bool copy_frame_to_nv12_message(
    dai::ImgFrame & packet,
    const StabilizationTransform & transform,
    sensor_msgs::msg::Image & message)
  {
    const auto & frame_data = packet.getData();
    const auto stride =
      packet.getStride() > 0U ? packet.getStride() : packet.getWidth();
    const int frame_width = static_cast<int>(packet.getWidth());
    const int frame_height = static_cast<int>(packet.getHeight());
    const bool input_has_chroma =
      packet.getType() == dai::ImgFrame::Type::NV12;
    const bool input_is_gray8 =
      packet.getType() == dai::ImgFrame::Type::GRAY8;
    if (!input_has_chroma && !input_is_gray8) {
      throw std::runtime_error(
              "DepthAI returned neither an NV12 nor a GRAY8 frame");
    }
    const auto source_rows = input_has_chroma ?
      static_cast<std::size_t>(frame_height) * 3U / 2U :
      static_cast<std::size_t>(frame_height);
    const auto expected_bytes =
      static_cast<std::size_t>(stride) * source_rows;
    if (frame_data.size() < expected_bytes) {
      throw std::runtime_error(
              "DepthAI returned an undersized camera frame");
    }

    cv::Mat input_y(
      frame_height,
      frame_width,
      CV_8UC1,
      const_cast<std::uint8_t *>(frame_data.data()),
      stride);
    cv::Mat input_uv;
    if (input_has_chroma) {
      input_uv = cv::Mat(
        frame_height / 2,
        frame_width / 2,
        CV_8UC2,
        const_cast<std::uint8_t *>(
          frame_data.data() +
          stride * static_cast<std::size_t>(frame_height)),
        stride);
    }
    cv::Mat stabilized_y;
    cv::Mat stabilized_uv;
    cv::Mat processed_y = input_y;
    cv::Mat processed_uv = input_uv;

    if (transform.homography) {
      stabilized_y.create(frame_height, frame_width, CV_8UC1);
      cv::warpPerspective(
        input_y,
        stabilized_y,
        cv::Mat(*transform.homography),
        stabilized_y.size(),
        cv::INTER_LINEAR,
        cv::BORDER_CONSTANT,
        cv::Scalar(0));

      processed_y = stabilized_y;
      if (input_has_chroma) {
        stabilized_uv.create(frame_height / 2, frame_width / 2, CV_8UC2);
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
        processed_uv = stabilized_uv;
      }
      stabilized_frames_total_.fetch_add(1U);
    }

    const int output_height = frame_height - output_crop_top_px_;
    const cv::Mat cropped_y = processed_y(
      cv::Rect(0, output_crop_top_px_, frame_width, output_height));
    cv::Mat cropped_uv;
    if (input_has_chroma) {
      cropped_uv = processed_uv(
        cv::Rect(
          0,
          output_crop_top_px_ / 2,
          frame_width / 2,
          output_height / 2));
    }

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
    if (input_has_chroma) {
      for (int row = 0; row < cropped_uv.rows; ++row) {
        std::memcpy(
          message.data.data() + y_bytes +
          static_cast<std::size_t>(row) *
          static_cast<std::size_t>(frame_width),
          cropped_uv.ptr(row),
          static_cast<std::size_t>(frame_width));
      }
    } else {
      std::fill_n(
        message.data.data() + y_bytes,
        uv_bytes,
        static_cast<std::uint8_t>(128));
    }
    return true;
  }

  bool copy_frame_to_bev_input(
    dai::ImgFrame & packet,
    const StabilizationTransform & transform,
    camera_driver::msg::BevInput & message)
  {
    const auto & frame_data = packet.getData();
    const auto source_stride =
      packet.getStride() > 0U ? packet.getStride() : packet.getWidth();
    const int frame_width = static_cast<int>(packet.getWidth());
    const int frame_height = static_cast<int>(packet.getHeight());
    const bool input_has_chroma =
      packet.getType() == dai::ImgFrame::Type::NV12;
    const bool input_is_gray8 =
      packet.getType() == dai::ImgFrame::Type::GRAY8;
    if (!input_has_chroma && !input_is_gray8) {
      throw std::runtime_error(
              "DepthAI returned neither an NV12 nor a GRAY8 frame");
    }
    const std::size_t source_rows = input_has_chroma ?
      static_cast<std::size_t>(frame_height) * 3U / 2U :
      static_cast<std::size_t>(frame_height);
    if (
      frame_data.size() <
      static_cast<std::size_t>(source_stride) * source_rows)
    {
      throw std::runtime_error(
              "DepthAI returned an undersized camera frame");
    }

    message.source_width = static_cast<std::uint32_t>(frame_width);
    message.source_height = static_cast<std::uint32_t>(frame_height);
    message.source_crop_top = static_cast<std::uint32_t>(fused_bev_crop_top_);
    message.cropped_width = static_cast<std::uint32_t>(frame_width);
    message.cropped_height = static_cast<std::uint32_t>(
      fused_bev_crop_height_);
    message.step = static_cast<std::uint32_t>(frame_width);

    const cv::Matx33d source_to_stabilized =
      transform.homography.value_or(cv::Matx33d::eye());
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        message.source_to_stabilized_homography[
          static_cast<std::size_t>(row * 3 + column)] =
          source_to_stabilized(row, column);
      }
    }
    message.dynamic_correction_applied =
      transform.dynamic_correction_applied;
    if (depth_enabled_) {
      const auto k=packet.getTransformation().getIntrinsicMatrix();
      const auto rgb_from_source=point_cloud::rgbFromFrame(packet,*depth_calibration_);
      for (int row=0;row<3;++row) {for (int col=0;col<3;++col) {
        message.source_intrinsics[row*3+col]=k[row][col];
      }}
      message.rgb_from_source_rotation=rgb_from_source.rotation;
      message.rgb_from_source_translation=rgb_from_source.translation;
      message.depth_geometry_valid=true;
    }

    const std::size_t output_y_bytes =
      static_cast<std::size_t>(frame_width) *
      static_cast<std::size_t>(fused_bev_crop_height_);
    message.nv12.resize(output_y_bytes + output_y_bytes / 2U);
    const std::uint8_t * source_y =
      frame_data.data() +
      static_cast<std::size_t>(fused_bev_crop_top_) * source_stride;
    for (int row = 0; row < fused_bev_crop_height_; ++row) {
      std::memcpy(
        message.nv12.data() +
        static_cast<std::size_t>(row) * static_cast<std::size_t>(frame_width),
        source_y + static_cast<std::size_t>(row) * source_stride,
        static_cast<std::size_t>(frame_width));
    }
    if (input_has_chroma) {
      const std::uint8_t * source_uv =
        frame_data.data() +
        static_cast<std::size_t>(frame_height) * source_stride +
        static_cast<std::size_t>(fused_bev_crop_top_ / 2) * source_stride;
      for (int row = 0; row < fused_bev_crop_height_ / 2; ++row) {
        std::memcpy(
          message.nv12.data() + output_y_bytes +
          static_cast<std::size_t>(row) * static_cast<std::size_t>(frame_width),
          source_uv + static_cast<std::size_t>(row) * source_stride,
          static_cast<std::size_t>(frame_width));
      }
    } else {
      std::fill_n(
        message.nv12.data() + output_y_bytes,
        output_y_bytes / 2U,
        static_cast<std::uint8_t>(128));
    }
    if (transform.homography) {
      stabilized_frames_total_.fetch_add(1U);
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

        const double batch_received_steady_sec =
          static_cast<double>(steady_now_nanoseconds()) * 1.0e-9;
        double newest_gyroscope_timestamp_sec =
          -std::numeric_limits<double>::infinity();
        for (const auto & packet : data->packets) {
          newest_gyroscope_timestamp_sec = std::max(
            newest_gyroscope_timestamp_sec,
            timestampSeconds(packet.gyroscope.getTimestamp()));
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
            const bool vehicle_stationary = measured_erpm_stationary();
            ensure_vehicle_axes_from_stabilizer_reference();
            if (vehicle_stationary) {
              latest_residual_longitudinal_acceleration_mps2_.store(
                0.0, std::memory_order_relaxed);
              latest_residual_lateral_acceleration_mps2_.store(
                0.0, std::memory_order_relaxed);
            }
            if (
              vehicle_motion_compensation_enabled_ &&
              imu_stabilizer_->initialized())
            {
              // CAN dynamics are stamped in the host clock domain. Preserve
              // each IMU report's relative age within the received batch so
              // the vehicle acceleration is queried at the same instant.
              double acceleration_query_timestamp_sec =
                batch_received_steady_sec;
              const double age_in_batch_sec =
                newest_gyroscope_timestamp_sec - gyroscope_timestamp_sec;
              if (
                std::isfinite(age_in_batch_sec) &&
                age_in_batch_sec >= 0.0 &&
                age_in_batch_sec <= maximum_timestamp_domain_delta_sec_)
              {
                acceleration_query_timestamp_sec -= age_in_batch_sec;
              }
              const auto vehicle_acceleration = can_acceleration_at(
                acceleration_query_timestamp_sec);
              const auto axes = vehicle_axes_camera();
              if (synchronized_acceleration && vehicle_acceleration && axes) {
                const cv::Vec3d vehicle_acceleration_camera =
                  axes->forward *
                  (can_longitudinal_compensation_gain_ *
                  vehicle_acceleration->longitudinal_mps2) +
                  axes->left *
                  (can_lateral_compensation_gain_ *
                  vehicle_acceleration->lateral_mps2);
                const cv::Vec3d residual_acceleration_camera =
                  *synchronized_acceleration - vehicle_acceleration_camera;
                synchronized_acceleration = residual_acceleration_camera;
                latest_residual_longitudinal_acceleration_mps2_.store(
                  residual_acceleration_camera.dot(axes->forward),
                  std::memory_order_relaxed);
                latest_residual_lateral_acceleration_mps2_.store(
                  residual_acceleration_camera.dot(axes->left),
                  std::memory_order_relaxed);
                update_maximum(
                  maximum_absolute_lateral_acceleration_mps2_interval_,
                  std::abs(vehicle_acceleration->lateral_mps2));
                motion_compensated_samples_total_.fetch_add(
                  1U, std::memory_order_relaxed);
              } else {
                // Raw moving acceleration is not a gravity observation. If
                // time-aligned vehicle motion is unavailable, omit the sample
                // instead of injecting longitudinal/lateral acceleration into
                // roll or pitch.
                const bool acceleration_missing =
                  !synchronized_acceleration.has_value();
                synchronized_acceleration.reset();
                motion_compensation_misses_total_.fetch_add(
                  1U, std::memory_order_relaxed);
                if (acceleration_missing) {
                  motion_missing_acceleration_total_.fetch_add(
                    1U, std::memory_order_relaxed);
                }
                if (!vehicle_acceleration) {
                  motion_missing_state_total_.fetch_add(
                    1U, std::memory_order_relaxed);
                }
                if (!axes) {
                  motion_missing_axes_total_.fetch_add(
                    1U, std::memory_order_relaxed);
                }
              }
            }
            imu_stabilizer_->update(
              synchronized_acceleration,
              angular_velocity_camera,
              gyroscope_timestamp_sec,
              vehicle_stationary);
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
    auto next_deadline = std::chrono::steady_clock::now();
    std::uint64_t published_generation = 0;

    while (!stop_requested_.load(std::memory_order_relaxed)) {
      const double publish_fps=publish_fps_.load();
      const bool publish_every_frame=publish_fps>=sensor_fps_*.999;
      const auto period=std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1./publish_fps));
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
          bev_handoff::RuntimeSample runtime_sample(monitor_.get(),"camera_to_bev");
          const auto stabilization_started_at =
            std::chrono::steady_clock::now();
          const auto transform = stabilizationTransform(
            *snapshot->packet,
            snapshot->sensor_timestamp,
            snapshot->generation);
          published_generation = snapshot->generation;

          if (!transform.frame_usable) {
            stabilization_output_drops_total_.fetch_add(1U);
            if (!publish_every_frame) {
              next_deadline = now + period;
            }
            continue;
          }

          bool output_available = false;
          std::unique_ptr<sensor_msgs::msg::Image> image_message;
          if (publish_enabled_) {
            image_message = std::make_unique<sensor_msgs::msg::Image>();
            image_message->header.stamp = snapshot->ros_stamp;
            image_message->header.frame_id = frame_id_;
            image_message->height = snapshot->packet->getHeight();
            image_message->width = snapshot->packet->getWidth();
            image_message->encoding = "nv12";
            image_message->is_bigendian = false;
            output_available = copy_frame_to_nv12_message(
              *snapshot->packet, transform, *image_message) ||
              output_available;
          }

          std::unique_ptr<camera_driver::msg::BevInput> bev_message;
          if (fused_bev_output_enabled_) {
            bev_message = std::make_unique<camera_driver::msg::BevInput>();
            bev_message->header.stamp = snapshot->ros_stamp;
            bev_message->header.frame_id = frame_id_;
            output_available = copy_frame_to_bev_input(
              *snapshot->packet, transform, *bev_message) ||
              output_available;
          }

          if (output_available) {
            stabilize_meter_.add(std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now() - snapshot->received_timestamp));
          }
          if (output_available && perf_log_enabled_) {
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
            if (image_message) {
              publisher_->publish(std::move(image_message));
            }
            if (bev_message) {
              fused_bev_publisher_->publish(std::move(bev_message));
            }
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

  void capture_camera_frame(const char * trigger)
  {
    cv::Mat camera_image;
    {
      std::lock_guard<std::mutex> lock(latest_capture_frame_mutex_);
      camera_image = latest_capture_frame_.clone();
    }
    if (camera_image.empty()) {
      RCLCPP_WARN(
        node_.get_logger(),
        "%s capture requested before a camera frame was available.",
        trigger);
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(capture_debounce_mutex_);
      const auto debounce_duration = std::chrono::duration<double>(
        capture_debounce_sec_);
      if (
        last_capture_at_.time_since_epoch().count() != 0 &&
        now - last_capture_at_ < debounce_duration)
      {
        return;
      }
      last_capture_at_ = now;
    }

    try {
      const std::filesystem::path directory(capture_directory_);
      std::error_code error;
      std::filesystem::create_directories(directory, error);
      if (error) {
        RCLCPP_ERROR(
          node_.get_logger(),
          "Failed to create camera capture directory '%s': %s",
          directory.string().c_str(),
          error.message().c_str());
        return;
      }
      const std::filesystem::path filename = directory /
        ("camera_capture_" +
        std::to_string(node_.get_clock()->now().nanoseconds()) + ".png");
      if (!cv::imwrite(filename.string(), camera_image)) {
        RCLCPP_ERROR(
          node_.get_logger(),
          "Failed to capture camera image: %s",
          filename.string().c_str());
        return;
      }
      const auto absolute_filename = std::filesystem::absolute(filename, error);
      const std::string saved_path = error ?
        filename.string() : absolute_filename.string();
      RCLCPP_INFO(
        node_.get_logger(),
        "Camera image captured by %s: %s",
        trigger,
        saved_path.c_str());
    } catch (const cv::Exception & exception) {
      RCLCPP_ERROR(
        node_.get_logger(),
        "Failed to capture camera image: %s",
        exception.what());
    }
  }

  void on_capture_joy(const sensor_msgs::msg::Joy::ConstSharedPtr message)
  {
    const bool button_available =
      capture_joy_button_ < static_cast<int>(message->buttons.size());
    const bool pressed =
      button_available &&
      message->buttons[static_cast<std::size_t>(capture_joy_button_)] != 0;
    const bool was_pressed =
      capture_joy_button_pressed_.exchange(pressed, std::memory_order_relaxed);
    if (!button_available) {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(),
        *node_.get_clock(),
        5000,
        "Joy message on %s has no capture button index %d.",
        capture_joy_topic_.c_str(),
        capture_joy_button_);
      return;
    }
    if (pressed && !was_pressed) {
      capture_camera_frame("controller A");
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
            *snapshot->packet, snapshot->sensor_timestamp,
            snapshot->generation);
          previewed_generation = snapshot->generation;
          if (!transform.frame_usable) {
            stabilization_output_drops_total_.fetch_add(1U);
          } else {
            auto preview_frame = snapshot->packet->getCvFrame();
            if (preview_frame.empty()) {
              throw std::runtime_error(
                      "DepthAI returned an empty camera preview");
            }
            if (preview_frame.type() == CV_8UC1) {
              cv::Mat preview_bgr;
              cv::cvtColor(
                preview_frame, preview_bgr, cv::COLOR_GRAY2BGR);
              preview_frame = std::move(preview_bgr);
            } else if (preview_frame.type() != CV_8UC3) {
              throw std::runtime_error(
                      "DepthAI returned an unsupported camera preview type");
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
            {
              std::lock_guard<std::mutex> lock(latest_capture_frame_mutex_);
              // Always own an immutable clean frame. Preview-only grids and
              // OpenCV window decorations must never enter the saved PNG.
              latest_capture_frame_ = preview_frame.clone();
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
        if (key == 'a' || key == 'A') {
          capture_camera_frame("keyboard A");
        }
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
          stabilization_state = "virtual-gimbal-ready";
        } else if (imu_stabilizer_) {
          const auto progress = imu_stabilizer_->calibrationProgress();
          stabilization_progress = progress;
          if (
            imu_stabilizer_config_.external_reference_required &&
            !imu_stabilizer_->externalReferenceReceived())
          {
            stabilization_state = "waiting-bev-reference";
          } else if (progress.discarding_startup_samples) {
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
      if (perf_log_enabled_) {
        RCLCPP_INFO(
          node_.get_logger(),
          "[PERF][CAMERA] capture_fps=%.1f stabilized_fps=%.1f "
          "stabilized_compute_ms(avg/max)=%.3f/%.3f "
          "latency_ms(depthai_to_host_avg/max=%.2f/%.2f,"
          "host_to_stabilized_avg/max=%.2f/%.2f,"
          "depthai_to_stabilized_avg/max=%.2f/%.2f) "
          "imu_fps=%.1f stabilizer=%s mapped_frames=%lu misses=%lu predicted=%lu "
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
          "stabilizer=%s (mapped_frames=%lu, misses=%lu, held=%lu, "
          "output_drops=%lu), "
          "dropped=%lu",
          capture_hz, sensor_fps_, preview_hz, imu_hz,
          stabilization_state.c_str(),
          static_cast<unsigned long>(stabilized_frames_total_.load()),
          static_cast<unsigned long>(stabilization_missed_total_.load()),
          static_cast<unsigned long>(
            stabilization_held_last_total_.load()),
          static_cast<unsigned long>(stabilization_output_drops_total_.load()),
          static_cast<unsigned long>(dropped_count));
      }
      if (imu_stabilizer_ && imu_stabilizer_->initialized()) {
        constexpr double radians_to_degrees =
          180.0 / 3.141592653589793238462643383279502884;
        const cv::Vec3d bias_degps =
          imu_stabilizer_->gyroscopeBiasRadps() * radians_to_degrees;
        const cv::Vec2d moving_accelerometer_nudge_deg =
          imu_stabilizer_->movingAccelerometerNudgeDegrees();
        const std::int64_t steady_now_ns = steady_now_nanoseconds();
        const bool erpm_fresh = measured_erpm_is_fresh(
          steady_now_ns);
        const std::int64_t can_received_ns =
          last_can_acceleration_received_ns_.load(std::memory_order_relaxed);
        const bool can_acceleration_fresh =
          can_received_ns > 0 && steady_now_ns >= can_received_ns &&
          static_cast<double>(steady_now_ns - can_received_ns) * 1.0e-9 <=
          can_acceleration_timeout_sec_;
        const double peak_lateral_acceleration_mps2 =
          maximum_absolute_lateral_acceleration_mps2_interval_.exchange(
          0.0, std::memory_order_relaxed);
        RCLCPP_INFO(
          node_.get_logger(),
          "Virtual gimbal: tilt_error(roll/pitch)=%.3f/%.3fdeg, "
          "correction=%.3fdeg, stationary=%s, "
          "measured_erpm=%d (filtered_abs=%.1f)/%s, "
          "CAN_vehicle_accel=%s/%s (gain_long/lat=%.2f/%.2f, "
          "ax/ay/peak_ay=%.3f/%.3f/%.3f), "
          "imu_residual_accel(forward/left)=%.3f/%.3f, motion_fusion=%s "
          "(samples/misses=%lu/%lu, no_accel/CAN/axes=%lu/%lu/%lu, "
          "received/rejected=%lu/%lu), "
          "gravity_anchor_updates=%lu, "
          "accel_nudge(roll/pitch)=%.3f/%.3fdeg, "
          "gyro_bias_xyz=%.4f/%.4f/%.4fdegps, bias_updates=%lu",
          latest_stabilization_roll_error_deg_.load(
            std::memory_order_relaxed),
          latest_stabilization_pitch_error_deg_.load(
            std::memory_order_relaxed),
          latest_stabilization_correction_angle_deg_.load(
            std::memory_order_relaxed),
          imu_stabilizer_->stationaryConfirmed() ? "yes" : "no",
          latest_measured_erpm_.load(std::memory_order_relaxed),
          latest_filtered_absolute_erpm_.load(std::memory_order_relaxed),
          erpm_fresh ? "fresh" : "stale",
          vehicle_motion_compensation_enabled_ ? "on" : "off",
          can_acceleration_fresh ? "fresh" : "stale",
          can_longitudinal_compensation_gain_,
          can_lateral_compensation_gain_,
          latest_longitudinal_acceleration_mps2_.load(
            std::memory_order_relaxed),
          latest_lateral_acceleration_mps2_.load(
            std::memory_order_relaxed),
          peak_lateral_acceleration_mps2,
          latest_residual_longitudinal_acceleration_mps2_.load(
            std::memory_order_relaxed),
          latest_residual_lateral_acceleration_mps2_.load(
            std::memory_order_relaxed),
          vehicle_motion_compensation_enabled_ ? "on" : "off",
          static_cast<unsigned long>(
            motion_compensated_samples_total_.load()),
          static_cast<unsigned long>(
            motion_compensation_misses_total_.load()),
          static_cast<unsigned long>(
            motion_missing_acceleration_total_.load()),
          static_cast<unsigned long>(
            motion_missing_state_total_.load()),
          static_cast<unsigned long>(
            motion_missing_axes_total_.load()),
          static_cast<unsigned long>(can_acceleration_received_total_.load()),
          static_cast<unsigned long>(can_acceleration_rejected_total_.load()),
          static_cast<unsigned long>(
            imu_stabilizer_->movingGravityAnchorUpdateCount()),
          moving_accelerometer_nudge_deg[0],
          moving_accelerometer_nudge_deg[1],
          bias_degps[0], bias_degps[1], bias_degps[2],
          static_cast<unsigned long>(
            imu_stabilizer_->onlineTiltBiasUpdateCount()));
      }
    } else {
      if (perf_log_enabled_) {
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

    if (capture_count > 0 && !rgb_decimated_.load() && capture_hz < sensor_fps_ * 0.90) {
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
    join_thread(depth_thread_);
    join_thread(ir_thread_);
    join_thread(rgb_exposure_thread_);
    rgb_controls_.reset();
    bev_handoff::publishCameraExposure(false,nullptr);
    bev_handoff::publishCameraExposure(true,nullptr);
    if (tunnel_enabled_) {bev_handoff::publishDirectIr(nullptr);}
    if (depth_enabled_) {bev_handoff::publishDirectDepth(nullptr);}

    if (status_timer_) {
      status_timer_->cancel();
      status_timer_.reset();
    }

    depth_queue_.reset();
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
  bool perf_log_enabled_{false};
  std::string camera_socket_name_;
  int width_{1280};
  int height_{800};
  double sensor_fps_{70.0};
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
  bool high_frequency_only_{false};
  double high_frequency_vibration_cutoff_hz_{3.0};
  ImuImageStabilizerConfig imu_stabilizer_config_{};
  std::string startup_ground_reference_topic_;
  std::string measured_erpm_topic_;
  int stationary_erpm_enter_threshold_{100};
  int stationary_erpm_exit_threshold_{500};
  double stationary_erpm_filter_time_constant_sec_{0.15};
  double stationary_erpm_enter_duration_sec_{1.0};
  double measured_erpm_timeout_sec_{1.0};
  bool vehicle_motion_compensation_enabled_{true};
  std::string can_acceleration_topic_{"/vehicle/dynamics/acceleration"};
  std::string can_acceleration_frame_id_{"base_link"};
  double can_acceleration_timeout_sec_{0.10};
  double can_longitudinal_compensation_gain_{1.0};
  double can_lateral_compensation_gain_{1.0};
  double maximum_longitudinal_acceleration_mps2_{15.0};
  double maximum_lateral_acceleration_mps2_{15.0};
  int invalid_correction_hold_frames_{2};
  double fixed_view_zoom_{1.25};
  double fixed_view_border_margin_px_{1.5};
  int output_crop_top_px_{0};
  bool publish_enabled_{false};
  std::unique_ptr<bev_handoff::RuntimeMonitor> monitor_;
  std::atomic<double> publish_fps_{70.0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr runtime_fps_callback_;
  bool fused_bev_output_enabled_{false};
  std::string fused_bev_topic_{"/camera/bev_input"};
  double bev_input_bottom_fraction_{0.70};
  int fused_bev_crop_top_{240};
  int fused_bev_crop_height_{560};
  bool preview_enabled_{false};
  double preview_fps_{60.0};
  std::string preview_window_name_;
  int preview_max_width_{1280};
  int preview_max_height_{800};
  bool preview_grid_enabled_{false};
  int preview_grid_spacing_px_{20};
  std::string capture_directory_{"."};
  std::string capture_joy_topic_{"/joy"};
  int capture_joy_button_{0};
  double capture_debounce_sec_{0.30};
  std::atomic<bool> capture_joy_button_pressed_{false};
  double startup_timeout_sec_{5.0};
  double status_log_interval_sec_{1.0};
  dai::CameraBoardSocket camera_socket_{dai::CameraBoardSocket::CAM_A};
  bool gray8_transport_{false};
  dai::ImgResizeMode resize_mode_{dai::ImgResizeMode::CROP};

  std::unique_ptr<dai::Pipeline> pipeline_;
  bool tunnel_enabled_{false},tunnel_validation_mode_{false},tunnel_ir_imu_enabled_{true};
  bool rgb_exposure_recovery_enabled_{true};
  double rgb_exposure_max_hold_sec_{.20};
  std::atomic<RgbTunnelPhase> rgb_tunnel_phase_{RgbTunnelPhase::outside};
  std::shared_ptr<dai::InputQueue> rgb_controls_;
  std::thread rgb_exposure_thread_;
  double tunnel_ir_imu_max_age_sec_{0.05},tunnel_ir_imu_wait_sec_{0.10};
  std::atomic<bool> tunnel_requested_{false};
  // Device-side routing per tunnel phase (see tunnel loop).
  std::shared_ptr<dai::InputQueue> depth_gate_control_,ir_gate_control_,rgb_gate_control_;
  std::atomic<bool> rgb_decimated_{false};
  std::atomic<bool> tunnel_profile_applied_{false};
  std::atomic<bool> tunnel_profile_ready_{false};
  std::atomic<std::int64_t> depth_resume_ns_{0};
  point_cloud::StereoCameras tunnel_cameras_;
  std::shared_ptr<dai::Device> tunnel_device_;
  std::shared_ptr<dai::MessageQueue> ir_queue_;
  std::vector<std::shared_ptr<dai::InputQueue>> ir_controls_;
  std::thread ir_thread_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tunnel_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tunnel_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tcn_publisher_;
  std::chrono::steady_clock::time_point last_tcn_frame_{};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr perf_publisher_;
  rclcpp::TimerBase::SharedPtr perf_timer_;
  std::chrono::steady_clock::time_point perf_started_at_{};
  bev_handoff::StageMeter capture_meter_, stabilize_meter_;
  bool depth_enabled_{false};
  point_cloud::Config depth_config_;
  std::shared_ptr<dai::CalibrationHandler> depth_calibration_;
  std::shared_ptr<dai::MessageQueue> depth_queue_;
  std::thread depth_thread_;
  std::shared_ptr<dai::MessageQueue> output_queue_;
  std::shared_ptr<dai::MessageQueue> imu_queue_;
  std::unique_ptr<ImuImageStabilizer> imu_stabilizer_;
  LastValidStabilizationHomography last_valid_stabilization_homography_{2U};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<camera_driver::msg::BevInput>::SharedPtr
    fused_bev_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    startup_ground_reference_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr
    measured_erpm_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    can_acceleration_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr
    capture_joy_subscription_;
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
  std::mutex latest_capture_frame_mutex_;
  cv::Mat latest_capture_frame_;
  std::mutex capture_debounce_mutex_;
  std::chrono::steady_clock::time_point last_capture_at_{};
  std::optional<std::int64_t> last_device_sequence_;
  cv::Matx33d calibrated_imu_output_to_camera_rotation_{
    cv::Matx33d::eye()};
  std::mutex vehicle_axes_mutex_;
  std::optional<VehicleAxesCamera> vehicle_axes_camera_;
  std::mutex can_acceleration_mutex_;
  std::deque<TimedVehicleAcceleration> can_acceleration_history_;
  std::string imu_name_;
  bool preview_window_sized_{false};

  std::atomic<bool> first_frame_received_{false};
  std::atomic<bool> startup_timeout_reported_{false};
  std::atomic<bool> timestamp_fallback_reported_{false};
  std::atomic<bool> late_external_reference_reported_{false};
  std::atomic<std::int32_t> latest_measured_erpm_{0};
  std::atomic<double> latest_filtered_absolute_erpm_{0.0};
  std::atomic<std::int64_t> last_measured_erpm_received_ns_{0};
  std::atomic<std::int64_t> last_can_acceleration_received_ns_{0};
  std::atomic<std::int64_t> stationary_erpm_candidate_started_ns_{0};
  std::atomic<bool> erpm_stationary_{false};
  std::atomic<double> latest_longitudinal_acceleration_mps2_{0.0};
  std::atomic<double> latest_lateral_acceleration_mps2_{0.0};
  std::atomic<double>
  maximum_absolute_lateral_acceleration_mps2_interval_{0.0};
  std::atomic<double>
  latest_residual_longitudinal_acceleration_mps2_{0.0};
  std::atomic<double> latest_residual_lateral_acceleration_mps2_{0.0};
  std::atomic<std::uint64_t> motion_compensated_samples_total_{0};
  std::atomic<std::uint64_t> motion_compensation_misses_total_{0};
  std::atomic<std::uint64_t> motion_missing_acceleration_total_{0};
  std::atomic<std::uint64_t> motion_missing_state_total_{0};
  std::atomic<std::uint64_t> motion_missing_axes_total_{0};
  std::atomic<std::uint64_t> can_acceleration_received_total_{0};
  std::atomic<std::uint64_t> can_acceleration_rejected_total_{0};
  std::atomic<bool> fallback_vehicle_axes_reported_{false};
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
  std::atomic<std::uint64_t> stabilization_held_last_total_{0};
  std::atomic<std::uint64_t> stabilization_angle_rejections_total_{0};
  std::atomic<std::uint64_t> stabilization_crop_rejections_total_{0};
  std::atomic<std::uint64_t> stabilization_predictions_total_{0};
  std::atomic<std::uint64_t> stabilization_output_drops_total_{0};
  std::atomic<std::uint64_t> rejected_acceleration_samples_total_{0};
  std::atomic<double> latest_stabilization_roll_error_deg_{0.0};
  std::atomic<double> latest_stabilization_pitch_error_deg_{0.0};
  std::atomic<double> latest_stabilization_correction_angle_deg_{0.0};
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
