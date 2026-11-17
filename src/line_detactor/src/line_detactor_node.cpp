#include <bev_handoff/model_load_status.hpp>
#include <bev_handoff/runtime_monitor.hpp>
#include "line_detactor/line_detactor_node.hpp"

#include "line_detactor/tensorrt_lane_backend.hpp"
#include "line_detactor/lane_connector.hpp"
#include "line_detactor/observed_lane_length.hpp"
#include "line_detactor/msg/lane_result.hpp"
#include "line_detactor/stop_line_distance.hpp"
#include "line_detactor/bev_theme.hpp"
#include "std_msgs/msg/string.hpp"
#include "line_detactor/obstacle_overlay.hpp"

#include "bev_handoff/direct_bev_handoff.hpp"
#include "line_detactor/frame_rate_gate.hpp"
#include "line_detactor/ir_preview.hpp"
#include "line_detactor/camera_exposure_hud.hpp"
#include "bev_handoff/direct_camera_handoff.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <functional>
#include <fstream>
#include <sstream>
#include <sys/utsname.h>
#if defined(__linux__)
#include <sys/resource.h>
#endif
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace line_detactor
{
namespace
{

constexpr int kDefaultInputWidth = 120;
constexpr int kDefaultInputHeight = 300;
constexpr int kBannerHeight = 144;
constexpr char kModelFilename[] =
  "fast_scnn_stop_line_120x300_batch_1.onnx";

bool graphical_display_available()
{
#if defined(__linux__)
  return std::getenv("DISPLAY") != nullptr ||
         std::getenv("WAYLAND_DISPLAY") != nullptr;
#else
  return true;
#endif
}

std::string default_model_path()
{
  const auto package_share =
    ament_index_cpp::get_package_share_directory("line_detactor");
  return (
    std::filesystem::path(package_share) / "models" / kModelFilename).string();
}

double nanoseconds_to_milliseconds(const std::uint64_t nanoseconds)
{
  return static_cast<double>(nanoseconds) / 1.0e6;
}

struct StageStats
{
  std::uint64_t count{0U};
  std::uint64_t total_nanoseconds{0U};
  std::uint64_t maximum_nanoseconds{0U};

  void record(const std::uint64_t nanoseconds)
  {
    ++count;
    total_nanoseconds += nanoseconds;
    maximum_nanoseconds = std::max(maximum_nanoseconds, nanoseconds);
  }

  double average_milliseconds() const
  {
    return count == 0U ? 0.0 :
      nanoseconds_to_milliseconds(total_nanoseconds) /
      static_cast<double>(count);
  }

  double maximum_milliseconds() const
  {
    return nanoseconds_to_milliseconds(maximum_nanoseconds);
  }

  void reset()
  {
    count = 0U;
    total_nanoseconds = 0U;
    maximum_nanoseconds = 0U;
  }
};

}  // namespace

class LineDetactorNode::Impl
{
public:
  explicit Impl(LineDetactorNode & node)
  : node_(node)
  {
    read_parameters();
    validate_parameters();
    if (obstacle_overlay_enabled_ && (!connection_.enabled || !direct_bev_input_enabled_ || bev_theme_enable_.load())) {
      throw std::invalid_argument("Obstacle overlay requires same-process direct BEV, connection_enabled and bev_theme_enable=false");
    }
    if (!std::isfinite(obstacle_sync_sec_) || obstacle_sync_sec_<.001 || obstacle_sync_sec_>.1 ||
      !std::isfinite(obstacle_age_sec_) || obstacle_age_sec_<.02 || obstacle_age_sec_>.5) {
      throw std::invalid_argument("Obstacle overlay sync/age invalid");
    }

    if (preview_enabled_ && !graphical_display_available()) {
      throw std::runtime_error(
              "Preview is enabled but DISPLAY/WAYLAND_DISPLAY is unavailable");
    }
    rclcpp::PublisherOptions model_status_options;
    model_status_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    model_status_pub_ = node_.create_publisher<std_msgs::msg::String>(
      "~/model_status", rclcpp::QoS(1).reliable().transient_local(), model_status_options);
    model_load_status_ = std::make_unique<bev_handoff::ModelLoadStatus>(
      [publisher = model_status_pub_](const std::string & status) {
        std_msgs::msg::String message; message.data = status; publisher->publish(message);
      });
    model_load_status_->update("LOADING | model=" + model_path_);
    try {
      if (!std::filesystem::is_regular_file(model_path_)) {
        throw std::runtime_error("Three-channel ONNX model not found: " + model_path_);
      }
      backend_ = std::make_unique<TensorRtLaneBackend>(
        model_path_, engine_cache_path_, engine_precision_, model_input_width_, model_input_height_,
        static_cast<std::size_t>(tensorrt_workspace_size_mb_) * 1024U * 1024U,
        mask_threshold_, overlay_alpha_, connection_.enabled, stop_line_mask_threshold_,
        [this](const std::string & stage) {model_load_status_->update("LOADING | " + stage);});
      model_load_status_->update("LOADING | Trial inference / CUDA warmup");
      warm_up();
      model_load_status_->update("READY | model=" + model_path_ + " | engine=" +
        backend_->engine_cache_path() + " | trial inference=PASS");
    } catch (const std::exception & e) {
      model_load_status_->update(std::string("FAILED | ") + e.what());
      RCLCPP_ERROR(node_.get_logger(), "[MODEL CHECK] FAIL: %s", e.what());
      throw;
    }
    monitor_=std::make_unique<bev_handoff::RuntimeMonitor>(node_);
    start_profiling();

    const auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .best_effort()
      .durability_volatile();
    if (connection_.enabled && result_publish_enabled_) {
      result_publisher_ = node_.create_publisher<line_detactor::msg::LaneResult>(
        result_topic_, image_qos);
      result_image_publisher_ = node_.create_publisher<Image>(result_image_topic_, image_qos);
    }
    if (!direct_bev_input_enabled_) {
      subscription_ = node_.create_subscription<sensor_msgs::msg::Image>(
        input_topic_, image_qos,
        std::bind(&Impl::on_image, this, std::placeholders::_1));
    }
    control_latency_subscription_ = node_.create_subscription<std_msgs::msg::Float32>(
      control_latency_topic_, image_qos,
      std::bind(&Impl::on_control_latency, this, std::placeholders::_1));
    if (preview_enabled_) {
      speed_subscription_ = node_.create_subscription<std_msgs::msg::Float32>(
        bev_theme_speed_topic_, image_qos,
        [this](const std_msgs::msg::Float32::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(preview_mutex_);
          theme_speed_mps_ = message->data;
          theme_speed_received_at_ = SteadyClock::now();
        });
      target_speed_subscription_ = node_.create_subscription<std_msgs::msg::Float32>(
        preview_target_speed_topic_, image_qos,
        [this](const std_msgs::msg::Float32::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(preview_mutex_);
          preview_target_speed_mps_ = message->data;
          preview_target_speed_received_at_ = SteadyClock::now();
        });
      duty_subscription_ = node_.create_subscription<std_msgs::msg::Float32>(
        preview_duty_topic_, image_qos,
        [this](const std_msgs::msg::Float32::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(preview_mutex_);
          preview_duty_ = message->data;
          preview_duty_received_at_ = SteadyClock::now();
        });
      corner_state_subscription_ = node_.create_subscription<std_msgs::msg::UInt8>(
        preview_corner_state_topic_, image_qos,
        [this](const std_msgs::msg::UInt8::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(preview_mutex_);
          preview_corner_state_ = message->data;
          preview_corner_state_received_at_ = SteadyClock::now();
        });
    }
    tunnel_enabled_=node_.declare_parameter<bool>("tunnel.enabled",false);
    inference_max_fps_=node_.declare_parameter<double>("inference_max_fps",0.0);
    if (!std::isfinite(inference_max_fps_) || inference_max_fps_<0) {
      throw std::invalid_argument("inference_max_fps must be finite and nonnegative");
    }
    if (tunnel_enabled_) {
      rclcpp::SubscriptionOptions tunnel_options;
      tunnel_options.use_intra_process_comm=rclcpp::IntraProcessSetting::Disable;
      tunnel_compute_sub_=node_.create_subscription<std_msgs::msg::String>(
        "/tunnel/compute_source",rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr message) {
          if (message->data=="IR") {inference_max_fps_.store(5.0);}
          else if (message->data=="RGB" || message->data=="BOTH") {inference_max_fps_.store(70.0);}
        },tunnel_options);
      if (preview_enabled_) {
        tunnel_selected_sub_=node_.create_subscription<std_msgs::msg::String>(
          "/tunnel/selected_source",rclcpp::QoS(1).reliable().transient_local(),
          [this](std_msgs::msg::String::ConstSharedPtr message) {
            if (message->data=="IR" || message->data=="RGB") {
              const bool selected_ir=message->data=="IR";
              if (selected_ir && !tunnel_selected_ir_.exchange(true)) {
                bev_handoff::publishIrLanePreview(nullptr); // Discard the previous tunnel visit.
              } else if (!selected_ir) {tunnel_selected_ir_.store(false);}
            }
          },tunnel_options);
        tunnel_preview_sub_=node_.create_subscription<std_msgs::msg::String>(
          "/tunnel/preview_status",rclcpp::QoS(1).reliable().transient_local(),
          [this](std_msgs::msg::String::ConstSharedPtr message) {
            std::array<std::string,3> lines;
            std::istringstream stream(message->data);
            for (auto & line:lines) {std::getline(stream,line); if (line.size()>160) {line.resize(160);}}
            std::lock_guard<std::mutex> lock(preview_mutex_);
            tunnel_hud_=std::move(lines); tunnel_hud_received_at_=SteadyClock::now();
          },tunnel_options);
      }
    }
    theme_parameter_callback_ = node_.add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        bool requested = bev_theme_enable_.load();
        for (const auto & parameter : parameters) {
          if (parameter.get_name() != "bev_theme_enable") {continue;}
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
            result.successful = false;
            result.reason = "bev_theme_enable must be bool";
            return result;
          }
          requested = parameter.as_bool();
          if (requested && (!connection_.enabled || obstacle_overlay_enabled_)) {
            result.successful = false;
            result.reason = "bev_theme_enable requires connection_enabled and obstacle overlay disabled";
            return result;
          }
        }
        bev_theme_enable_.store(requested);
        return result;
      });

    try {
      worker_ = std::thread(&Impl::worker_loop, this);
      if (preview_enabled_) {
        preview_worker_ = std::thread(&Impl::preview_loop, this);
      }
      if (direct_bev_input_enabled_) {
        direct_bev_consumer_id_ = bev_handoff::registerDirectBevConsumer(
          [this](std::shared_ptr<const bev_handoff::DirectBevFrame> frame) {
            on_direct_bev(std::move(frame));
          });
      }
    } catch (...) {
      stop();
      throw;
    }

    RCLCPP_INFO(
      node_.get_logger(),
      "Line detector ready: input=%s%s (bgr8 %dx%d), model=%s, backend="
      "TensorRT %s, threshold=%.3f, preview=%s @ %.1f FPS",
      direct_bev_input_enabled_ ? "direct-process-local" : input_topic_.c_str(),
      direct_bev_input_enabled_ ? "" : " ROS topic",
      model_input_width_, model_input_height_,
      model_path_.c_str(), engine_precision_.c_str(), static_cast<double>(mask_threshold_),
      preview_enabled_ ? "on" : "off", preview_fps_);
    RCLCPP_INFO(
      node_.get_logger(),
      "GPU path: %s -> CUDA RGB FP32 NCHW -> TensorRT -> "
      "%s; engine cache=%s",
      direct_bev_input_enabled_ ? "direct device BGR8" : "pinned BGR8 H2D",
      connection_.enabled ? "CUDA labels D2H (raw overlay skipped)" : "CUDA overlay BGR8 D2H",
      backend_->engine_cache_path().c_str());
    RCLCPP_INFO(node_.get_logger(), "Preview content=%s",
      connection_.enabled && preview_result_only_enabled_ ? "results only" : "camera overlay");
    RCLCPP_INFO(node_.get_logger(),
      "Border interpolation=%s, result=%dx%d (padding=%d each side), publish=%s, topic=%s",
      connection_.enabled ? "on" : "off", result_width(), model_input_height_,
      connection_.padding_px, result_publisher_ ? "on" : "off", result_topic_.c_str());
    RCLCPP_INFO(node_.get_logger(),
      "Yellow centerline=%s, lane width=%.2fm, BEV=%.2fx%.2fm, smoothing=%s strength=%.2f window=%.2fm",
      centerline_.enabled && connection_.enabled ? "on" : "off", centerline_.lane_width_m,
      centerline_.bev_width_m, centerline_.bev_height_m, centerline_.smoothing_enabled ? "on" : "off",
      centerline_.smoothing_strength, centerline_.smoothing_window_m);
    RCLCPP_INFO(node_.get_logger(),
      "Corner outer reference=%s weight=%.2f turn window=%.2fm min support=%.2fm "
      "outward offset=%.3fm entry distance=%.2fm full offset distance=%.2fm",
      centerline_.corner_outer_enabled ? "on" : "off", centerline_.corner_outer_weight,
      centerline_.corner_outer_window_m, centerline_.corner_outer_min_length_m,
      centerline_.corner_outward_offset_m, centerline_.corner_entry_distance_m,
      centerline_.corner_entry_full_offset_distance_m);
  }

  ~Impl()
  {
    stop();
  }

private:
  using Image = sensor_msgs::msg::Image;
  using SteadyClock = std::chrono::steady_clock;

  struct InputFrame
  {
    std_msgs::msg::Header header;
    cv::Mat bgr;
    std::function<cv::Mat()> download_bgr;
    const std::uint8_t * device_bgr{nullptr};
    std::size_t device_stride{0U};
    int width{0};
    int height{0};
    std::shared_ptr<const void> owner;
  };

  struct PreviewFrame
  {
    std::shared_ptr<const InputFrame> input;
    std_msgs::msg::Header header;
    LaneConnectionResult result;
    cv::Mat raw;
    double average_inference_milliseconds{0.0};
    double average_postprocess_milliseconds{0.0};
    std::uint64_t generation{0U};
  };

  void read_parameters()
  {
    rcl_interfaces::msg::ParameterDescriptor obstacle_descriptor;
    obstacle_descriptor.read_only=true;
    obstacle_overlay_enabled_=node_.declare_parameter<bool>("obstacles.overlay_enabled",false,obstacle_descriptor);
    obstacle_sync_sec_=node_.declare_parameter<double>("obstacles.max_sync_sec",0.06,obstacle_descriptor);
    obstacle_age_sec_=node_.declare_parameter<double>("obstacles.max_age_sec",0.25,obstacle_descriptor);
    bev_theme_enable_.store(node_.declare_parameter<bool>("bev_theme_enable", false));
    bev_theme_preview_fps_ = node_.declare_parameter<double>("bev_theme_preview_fps", 15.0);
    bev_theme_speed_topic_ = node_.declare_parameter<std::string>(
      "bev_theme_speed_topic", "/auto/current_speed");
    preview_target_speed_topic_ = node_.declare_parameter<std::string>(
      "preview_target_speed_topic", "/auto/target_speed");
    preview_duty_topic_ = node_.declare_parameter<std::string>(
      "preview_duty_topic", "/auto/current_duty");
    preview_corner_state_topic_ = node_.declare_parameter<std::string>(
      "preview_corner_state_topic", "/auto/corner_state");
    input_topic_ = node_.declare_parameter<std::string>(
      "input_topic", "/camera/image_bev");
    direct_bev_input_enabled_ = node_.declare_parameter<bool>(
      "direct_bev_input_enabled", false);
    model_path_ = node_.declare_parameter<std::string>(
      "model_path", default_model_path());
    // Packaged defaults may select a model by filename without embedding a
    // particular vehicle's install path. Absolute/custom paths stay untouched.
    if (!model_path_.empty() && !std::filesystem::path(model_path_).has_parent_path()) {
      model_path_ = (std::filesystem::path(default_model_path()).parent_path() / model_path_).string();
    }
    engine_cache_path_ = node_.declare_parameter<std::string>(
      "engine_cache_path", "");
    engine_precision_ = node_.declare_parameter<std::string>(
      "engine_precision", "fp32");
    tensorrt_workspace_size_mb_ = node_.declare_parameter<int>(
      "tensorrt_workspace_size_mb", 1024);
    model_input_width_ = node_.declare_parameter<int>(
      "model_input_width", kDefaultInputWidth);
    model_input_height_ = node_.declare_parameter<int>(
      "model_input_height", kDefaultInputHeight);
    mask_threshold_ = static_cast<float>(
      node_.declare_parameter<double>("mask_threshold", 0.5));
    stop_line_mask_threshold_ = static_cast<float>(node_.declare_parameter<double>(
      "stop_line_mask_threshold", -1.0));
    stop_line_min_present_pixels_ = node_.declare_parameter<int>("stop_line_min_present_pixels", 1);
    stop_line_distance_.enabled = node_.declare_parameter<bool>("stop_line_distance_enabled", true);
    stop_line_distance_.min_pixels = node_.declare_parameter<int>("stop_line_min_pixels", 6);
    stop_line_distance_.max_fit_samples = node_.declare_parameter<int>("stop_line_max_fit_samples", 256);
    stop_line_distance_.support_trim_quantile = node_.declare_parameter<double>(
      "stop_line_support_trim_quantile", 0.02);
    stop_line_distance_.support_margin_px = node_.declare_parameter<double>(
      "stop_line_support_margin_px", 5.0);
    stop_line_distance_.near_edge_quantile = node_.declare_parameter<double>(
      "stop_line_near_edge_quantile", 0.05);
    stop_line_distance_.min_crossing_alignment = node_.declare_parameter<double>(
      "stop_line_min_crossing_alignment", 0.5);
    stop_line_distance_.fit_distance_tolerance_px = node_.declare_parameter<double>(
      "stop_line_fit_distance_tolerance_px", 0.01);
    stop_line_distance_.fit_angle_tolerance_rad = node_.declare_parameter<double>(
      "stop_line_fit_angle_tolerance_rad", 0.01);
    overlay_alpha_ = static_cast<float>(
      node_.declare_parameter<double>("overlay_alpha", 0.75));
    connection_.ccl_serial_enabled = node_.declare_parameter<bool>(
      "connection_ccl_serial_enabled", true);
    connection_.enabled = node_.declare_parameter<bool>(
      "connection_enabled", true);
    connection_.padding_px = node_.declare_parameter<int>(
      "result_padding_px", 30);
    connection_.min_component_area_px = node_.declare_parameter<int>(
      "connection_min_component_area_px", 8);
    connection_.skeleton_downsample_factor = node_.declare_parameter<int>(
      "connection_skeleton_downsample_factor", 1);
    connection_.min_fragment_length_px = node_.declare_parameter<double>(
      "connection_min_fragment_length_px", 8.0);
    connection_.max_fragments = node_.declare_parameter<int>(
      "connection_max_fragments", 24);
    connection_.tangent_window_px = node_.declare_parameter<double>(
      "connection_tangent_window_px", 8.0);
    connection_.max_gap_px = node_.declare_parameter<double>(
      "connection_max_gap_px", 80.0);
    connection_.corridor_half_width_px = node_.declare_parameter<double>(
      "connection_corridor_half_width_px", 4.0);
    connection_.direction_tolerance_deg = node_.declare_parameter<double>(
      "connection_direction_tolerance_deg", 20.0);
    connection_.max_turn_deg = node_.declare_parameter<double>(
      "connection_max_turn_deg", 180.0);
    connection_.max_curvature_per_px = node_.declare_parameter<double>(
      "connection_max_curvature_per_px", 0.12);
    connection_.max_arc_ratio = node_.declare_parameter<double>(
      "connection_max_arc_ratio", 1.8);
    connection_.border_endpoint_distance_px = node_.declare_parameter<double>(
      "connection_border_endpoint_distance_px", 6.0);
    connection_.line_width_px = node_.declare_parameter<int>(
      "result_line_width_px", 2);
    centerline_.enabled = node_.declare_parameter<bool>("centerline_enabled", true);
    centerline_.lane_width_m = node_.declare_parameter<double>("centerline_lane_width_m", 0.65);
    centerline_.bev_width_m = node_.declare_parameter<double>("centerline_bev_width_m", 1.2);
    centerline_.bev_height_m = node_.declare_parameter<double>("centerline_bev_height_m", 3.0);
    centerline_.sample_spacing_m = node_.declare_parameter<double>(
      "centerline_sample_spacing_m", 0.015);
    centerline_.output_spacing_m = node_.declare_parameter<double>(
      "centerline_output_spacing_m", 0.01);
    centerline_.clearance_check_spacing_m = node_.declare_parameter<double>(
      "centerline_clearance_check_spacing_m", 0.005);
    centerline_.min_fragment_length_m = node_.declare_parameter<double>(
      "centerline_min_fragment_length_m", 0.08);
    centerline_.tangent_window_m = node_.declare_parameter<double>(
      "centerline_tangent_window_m", 0.06);
    centerline_.width_tolerance_m = node_.declare_parameter<double>(
      "centerline_width_tolerance_m", 0.12);
    centerline_.pair_along_tolerance_m = node_.declare_parameter<double>(
      "centerline_pair_along_tolerance_m", 0.055);
    centerline_.pair_heading_tolerance_deg = node_.declare_parameter<double>(
      "centerline_pair_heading_tolerance_deg", 40.0);
    centerline_.max_gap_m = node_.declare_parameter<double>("centerline_max_gap_m", 0.12);
    centerline_.max_start_distance_m = node_.declare_parameter<double>(
      "centerline_max_start_distance_m", 0.65);
    centerline_.min_clearance_m = node_.declare_parameter<double>(
      "centerline_min_clearance_m", 0.16);
    centerline_.outside_margin_m = node_.declare_parameter<double>(
      "centerline_outside_margin_m", 0.12);
    centerline_.max_samples = node_.declare_parameter<int>("centerline_max_samples", 2000);
    centerline_.line_width_px = node_.declare_parameter<int>("centerline_line_width_px", 2);
    centerline_.corner_outer_enabled = node_.declare_parameter<bool>(
      "centerline_corner_outer_enabled", true);
    centerline_.corner_outer_weight = node_.declare_parameter<double>(
      "centerline_corner_outer_weight", 0.85);
    centerline_.corner_outward_offset_m = node_.declare_parameter<double>(
      "centerline_corner_outward_offset_m", 0.05);
    centerline_.corner_entry_distance_m = node_.declare_parameter<double>(
      "centerline_corner_entry_distance_m", 0.40);
    centerline_.corner_entry_full_offset_distance_m = node_.declare_parameter<double>(
      "centerline_corner_entry_full_offset_distance_m", 0.0);
    centerline_.corner_outer_window_m = node_.declare_parameter<double>(
      "centerline_corner_outer_window_m", 0.60);
    centerline_.corner_outer_tangent_window_m = node_.declare_parameter<double>(
      "centerline_corner_outer_tangent_window_m", 0.15);
    centerline_.corner_outer_min_length_m = node_.declare_parameter<double>(
      "centerline_corner_outer_min_length_m", 0.30);
    centerline_.corner_outer_min_turn_deg = node_.declare_parameter<double>(
      "centerline_corner_outer_min_turn_deg", 8.0);
    centerline_.corner_outer_full_turn_deg = node_.declare_parameter<double>(
      "centerline_corner_outer_full_turn_deg", 25.0);
    centerline_.smoothing_enabled = node_.declare_parameter<bool>(
      "centerline_smoothing_enabled", true);
    centerline_.smoothing_sigma_m = node_.declare_parameter<double>(
      "centerline_smoothing_sigma_m", 0.04);
    centerline_.smoothing_window_m = node_.declare_parameter<double>(
      "centerline_smoothing_window_m", 0.65);
    centerline_.smoothing_max_shift_m = node_.declare_parameter<double>(
      "centerline_smoothing_max_shift_m", 0.03);
    centerline_.smoothing_strength = node_.declare_parameter<double>(
      "centerline_smoothing_strength", 1.0);
    centerline_.straight_turn_deg = node_.declare_parameter<double>(
      "centerline_straight_turn_deg", 12.0);
    centerline_.corner_turn_deg = node_.declare_parameter<double>(
      "centerline_corner_turn_deg", 35.0);
    centerline_.turn_window_m = node_.declare_parameter<double>("centerline_turn_window_m", 0.30);
    result_publish_enabled_ = node_.declare_parameter<bool>("result_publish_enabled", true);
    result_topic_ = node_.declare_parameter<std::string>("result_topic", "/line_detactor/result");
    result_image_topic_ = node_.declare_parameter<std::string>(
      "result_image_topic", "/line_detactor/result_image");
    control_latency_topic_ = node_.declare_parameter<std::string>(
      "control_latency_topic", "/auto/detector_input_to_control_decision_ms");
    warmup_iterations_ = node_.declare_parameter<int>(
      "warmup_iterations", 10);
    preview_enabled_ = node_.declare_parameter<bool>(
      "preview_enabled", true);
    shared_ir_preview_output_enabled_=node_.declare_parameter<bool>(
      "shared_ir_preview_output_enabled",false);
    if (shared_ir_preview_output_enabled_) {bev_handoff::publishIrLanePreview(nullptr);}
    preview_result_only_enabled_ = node_.declare_parameter<bool>(
      "preview_result_only_enabled", true);
    preview_fps_ = node_.declare_parameter<double>(
      "preview_fps", 30.0);
    preview_scale_ = node_.declare_parameter<double>(
      "preview_scale", 2.0);
    preview_window_name_ = node_.declare_parameter<std::string>(
      "preview_window_name", "BEV lane TensorRT preview");
    profiling_enabled_ = node_.declare_parameter<bool>("profiling_enabled", false);
    profiling_directory_ = node_.declare_parameter<std::string>(
      "profiling_directory", "/tmp/line_detactor_profiles");
    status_log_interval_sec_ = node_.declare_parameter<double>(
      "status_log_interval_sec", 1.0);
  }

  void start_profiling()
  {
    if (!profiling_enabled_) {return;}
    std::vector<std::pair<std::string, std::string>> metadata{
      {"node", node_.get_fully_qualified_name()},
      {"compiler", __VERSION__}, {"opencv", CV_VERSION},
      {"build_configuration", LINE_DETACTOR_BUILD_CONFIGURATION},
      {"build_revision", LINE_DETACTOR_BUILD_REVISION},
      {"engine_cache_path_resolved", backend_->engine_cache_path()},
      {"hardware_threads", std::to_string(std::thread::hardware_concurrency())}};
#ifdef __OPTIMIZE__
    metadata.emplace_back("compiler_optimization_enabled", "true");
#else
    metadata.emplace_back("compiler_optimization_enabled", "false");
#endif
    struct utsname system_info{};
    if (uname(&system_info) == 0) {
      metadata.emplace_back("system", std::string(system_info.sysname) + " " +
        system_info.release + " " + system_info.machine);
    }
    std::ifstream hardware("/proc/device-tree/model");
    if (hardware) {
      std::string model;
      std::getline(hardware, model, '\0');
      metadata.emplace_back("hardware_model", model);
    }
    metadata.emplace_back("stop_line_mask_threshold_resolved", std::to_string(
      stop_line_mask_threshold_ == -1.0F ? mask_threshold_ : stop_line_mask_threshold_));
    // Capture declared, effective parameters AFTER YAML and launch overrides.
    const auto names = node_.list_parameters({}, 0U).names;
    for (const auto & name : names) {
      const auto parameter = node_.get_parameter(name);
      metadata.emplace_back("parameter." + name, parameter.value_to_string());
    }
    try {
      profile_writer_ = std::make_unique<ProcessingProfileWriter>(profiling_directory_, metadata);
      profile_started_at_ = SteadyClock::now();
      RCLCPP_INFO(node_.get_logger(), "Lane processing profile: %s", profile_writer_->path().c_str());
    } catch (const std::exception & error) {
      // Logging availability must not determine whether the detector can run.
      RCLCPP_ERROR(node_.get_logger(), "Lane profiling disabled: %s", error.what());
    }
  }

  void save_profile(ProfileFrame & frame, const SteadyClock::time_point started)
  {
    if (!profile_writer_) {return;}
    const auto now = SteadyClock::now();
    frame.elapsed_ms = std::chrono::duration<double, std::milli>(started - profile_started_at_).count();
    frame.total_ms = std::chrono::duration<double, std::milli>(now - started).count();
    profile_writer_->submit(frame);
  }

  void validate_parameters() const
  {
    validate_lane_connection(connection_);
    validate_centerline(centerline_);
    validate_stop_line_distance(stop_line_distance_);
    if (stop_line_min_present_pixels_ < 1) {
      throw std::invalid_argument("stop_line_min_present_pixels must be >= 1");
    }
    if (!std::isfinite(stop_line_mask_threshold_) ||
      (stop_line_mask_threshold_ != -1.0F &&
      (stop_line_mask_threshold_ < 0.0F || stop_line_mask_threshold_ > 1.0F)))
    {throw std::invalid_argument("stop_line_mask_threshold must be -1 (inherit) or in [0,1]");}
    if (centerline_.enabled && !connection_.enabled) {
      RCLCPP_WARN(node_.get_logger(),
        "centerline_enabled requires connection_enabled; raw preview mode has no centerline");
    }
    if (result_topic_.empty() || result_image_topic_.empty() || result_topic_ == result_image_topic_ ||
      result_topic_ == input_topic_ || result_image_topic_ == input_topic_)
    {
      throw std::invalid_argument("Result topics must be nonempty, distinct and different from input");
    }
    if (input_topic_.empty()) {
      throw std::invalid_argument("input_topic must not be empty");
    }
    if (control_latency_topic_.empty()) {
      throw std::invalid_argument("control_latency_topic must not be empty");
    }
    if (model_path_.empty()) {
      throw std::invalid_argument("model_path must not be empty");
    }
    if (engine_precision_ != "fp32" && engine_precision_ != "fp16" &&
      engine_precision_ != "int8")
    {
      throw std::invalid_argument("engine_precision must be fp32, fp16 or int8");
    }
    if (model_input_width_ <= 0 || model_input_height_ <= 0) {
      throw std::invalid_argument("model input dimensions must be positive");
    }
    if (tensorrt_workspace_size_mb_ <= 0 ||
      tensorrt_workspace_size_mb_ > 16384)
    {
      throw std::invalid_argument(
              "tensorrt_workspace_size_mb must be in [1,16384]");
    }
    if (!std::isfinite(mask_threshold_) ||
      mask_threshold_ < 0.0F || mask_threshold_ > 1.0F)
    {
      throw std::invalid_argument("mask_threshold must be in [0,1]");
    }
    if (!std::isfinite(overlay_alpha_) ||
      overlay_alpha_ < 0.0F || overlay_alpha_ > 1.0F)
    {
      throw std::invalid_argument("overlay_alpha must be in [0,1]");
    }
    if (warmup_iterations_ < 0 || warmup_iterations_ > 1000) {
      throw std::invalid_argument("warmup_iterations must be in [0,1000]");
    }
    if (!std::isfinite(preview_fps_) || preview_fps_ <= 0.0) {
      throw std::invalid_argument("preview_fps must be finite and positive");
    }
    if (!std::isfinite(bev_theme_preview_fps_) || bev_theme_preview_fps_ <= 0.0 ||
      bev_theme_speed_topic_.empty() || preview_target_speed_topic_.empty() ||
      preview_duty_topic_.empty() || preview_corner_state_topic_.empty() || (bev_theme_enable_.load() && !connection_.enabled))
    {
      throw std::invalid_argument(
              "BEV theme requires positive preview FPS, a speed topic, and connection_enabled");
    }
    if (!std::isfinite(preview_scale_) || preview_scale_ <= 0.0) {
      throw std::invalid_argument("preview_scale must be finite and positive");
    }
    if (preview_window_name_.empty()) {
      throw std::invalid_argument("preview_window_name must not be empty");
    }
    if (!std::isfinite(status_log_interval_sec_) ||
      status_log_interval_sec_ <= 0.0)
    {
      throw std::invalid_argument(
              "status_log_interval_sec must be finite and positive");
    }
  }

  void publish_model_status(const std::string & status)
  {
    std_msgs::msg::String message;
    message.data = status;
    model_status_pub_->publish(message);
    RCLCPP_INFO(node_.get_logger(), "[MODEL CHECK] %s", status.c_str());
  }

  void warm_up()
  {
    const auto bytes =
      static_cast<std::size_t>(model_input_width_) *
      static_cast<std::size_t>(model_input_height_) * 3U;
    std::vector<std::uint8_t> black(bytes, 0U);
    const auto stride = static_cast<std::size_t>(model_input_width_) * 3U;
    for (int iteration = 0; iteration < std::max(1, warmup_iterations_); ++iteration) {
      static_cast<void>(backend_->infer_bgr(black.data(), bytes, stride));
    }
    RCLCPP_INFO(
      node_.get_logger(), "TensorRT warmup completed: %d iterations",
      std::max(1, warmup_iterations_));
  }

  void on_image(const Image::ConstSharedPtr message)
  {
    const std::size_t minimum_step =
      static_cast<std::size_t>(message->width) * 3U;
    const std::size_t required_size =
      static_cast<std::size_t>(message->step) *
      static_cast<std::size_t>(message->height);
    if (
      message->encoding != "bgr8" || message->width == 0U ||
      message->height == 0U || message->step < minimum_step ||
      message->data.size() < required_size)
    {
      RCLCPP_ERROR_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 5000,
        "Rejected malformed ROS BEV image: encoding=%s size=%ux%u "
        "step=%u data=%zu.",
        message->encoding.c_str(), message->width, message->height,
        message->step, message->data.size());
      return;
    }
    auto frame = std::make_shared<InputFrame>();
    frame->header = message->header;
    frame->width = static_cast<int>(message->width);
    frame->height = static_cast<int>(message->height);
    frame->bgr = cv::Mat(
      static_cast<int>(message->height),
      static_cast<int>(message->width),
      CV_8UC3,
      const_cast<std::uint8_t *>(message->data.data()),
      static_cast<std::size_t>(message->step));
    frame->owner = message;
    accept_frame(std::move(frame));
  }

  void on_direct_bev(
    std::shared_ptr<const bev_handoff::DirectBevFrame> message)
  {
    auto frame = std::make_shared<InputFrame>();
    frame->header = message->header;
    frame->bgr = message->bgr;
    frame->download_bgr = message->download_bgr;
    frame->device_bgr = message->device_bgr;
    frame->device_stride = message->device_stride;
    frame->width = message->width;
    frame->height = message->height;
    frame->owner = std::move(message);
    accept_frame(std::move(frame));
  }

  void accept_frame(std::shared_ptr<const InputFrame> frame)
  {
    const auto received_at = SteadyClock::now();
    const auto received_stamp =
      static_cast<builtin_interfaces::msg::Time>(node_.get_clock()->now());
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (!rate_gate_.accept(std::chrono::duration<double>(received_at.time_since_epoch()).count(),
          inference_max_fps_.load())) {return;}
      latest_message_ = std::move(frame);
      latest_input_received_at_ = received_at;
      latest_input_received_stamp_ = received_stamp;
      ++latest_generation_;
      ++received_interval_;
      ++received_total_;
    }
    frame_condition_.notify_one();
  }

  void on_control_latency(const std_msgs::msg::Float32::ConstSharedPtr message)
  {
    if (!std::isfinite(message->data) || message->data <= 0.0F) {
      return;
    }
    std::lock_guard<std::mutex> lock(preview_mutex_);
    ++control_latency_sample_count_;
    control_latency_total_milliseconds_ += message->data;
  }

  void validate_image(const InputFrame & message) const
  {
    if (message.width != model_input_width_ ||
      message.height != model_input_height_)
    {
      throw std::invalid_argument(
              "Expected " + std::to_string(model_input_width_) + "x" +
              std::to_string(model_input_height_) + " BEV, received " +
              std::to_string(message.width) + "x" +
              std::to_string(message.height));
    }
    const std::size_t minimum_step =
      static_cast<std::size_t>(model_input_width_) * 3U;
    if (message.device_bgr != nullptr) {
      if (message.device_stride != minimum_step) {
        throw std::invalid_argument("CUDA BEV image stride is invalid");
      }
    } else if (
      message.bgr.type() != CV_8UC3 || message.bgr.step < minimum_step ||
      message.bgr.data == nullptr)
    {
      throw std::invalid_argument("CPU BEV image buffer is invalid");
    }
    if (
      (preview_enabled_ || shared_ir_preview_output_enabled_) && !preview_result_only_enabled_ &&
      message.bgr.empty() && !message.download_bgr)
    {
      throw std::invalid_argument(
              "Camera-overlay preview requires direct_host_copy_enabled=true");
    }
  }

  int result_width() const
  {
    return model_input_width_ + 2 * connection_.padding_px;
  }

  Image image_message(
    const cv::Mat & image,
    const InputFrame & input,
    const std::string & encoding) const
  {
    Image message;
    message.header = input.header;
    message.height = static_cast<std::uint32_t>(image.rows);
    message.width = static_cast<std::uint32_t>(image.cols);
    message.encoding = encoding;
    message.is_bigendian = false;
    message.step = static_cast<std::uint32_t>(image.cols * image.elemSize());
    message.data.resize(static_cast<std::size_t>(message.step) * image.rows);
    for (int y = 0; y < image.rows; ++y) {
      std::memcpy(message.data.data() + static_cast<std::size_t>(y) * message.step,
        image.ptr(y), message.step);
    }
    return message;
  }

  void publish_result(
    const LaneConnectionResult & result,
    const InputFrame & input,
    const builtin_interfaces::msg::Time & input_received_stamp,
    const SteadyClock::time_point input_received_at,
    const SteadyClock::time_point processing_started_at,
    const LaneInferenceTiming & timing,
    const std::uint64_t lane_geometry_nanoseconds,
    const std::uint64_t detector_sequence)
  {
    if (!result_publisher_) {return;}
    const auto message_build_started_at = SteadyClock::now();
    line_detactor::msg::LaneResult message;
    message.header = input.header;
    message.detector_input_received_stamp = input_received_stamp;
    message.engine_precision = engine_precision_;
    message.detector_sequence = detector_sequence;
    message.detector_queue_nanoseconds = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        processing_started_at - input_received_at).count());
    message.h2d_preprocess_nanoseconds = timing.preprocessing_nanoseconds;
    message.pure_inference_nanoseconds = timing.execution_nanoseconds;
    message.label_export_nanoseconds = timing.label_export_nanoseconds;
    message.backend_postprocess_nanoseconds = timing.postprocessing_nanoseconds;
    message.lane_geometry_nanoseconds = lane_geometry_nanoseconds;
    const double observed_length = observed_lane_length(result.observed_paths,
      input.width, input.height, connection_.padding_px, centerline_.bev_width_m,
      centerline_.bev_height_m, centerline_.max_start_distance_m);
    message.observed_lane_length_valid = std::isfinite(observed_length);
    message.observed_lane_length_m = static_cast<float>(observed_length);
    message.state = result.state;
    message.processing_mode = line_detactor::msg::LaneResult::BORDER_ONLY;
    message.source_width = static_cast<std::uint32_t>(input.width);
    message.source_height = static_cast<std::uint32_t>(input.height);
    message.padding_left = connection_.padding_px;
    message.padding_right = connection_.padding_px;
    message.centerline_valid = result.centerline.points.size() >= 2U;
    message.centerline_sample_limit_reached = result.centerline.sample_limit_reached;
    message.centerline_support = result.centerline.support;
    message.centerline_bev_width_m = static_cast<float>(centerline_.bev_width_m);
    message.centerline_bev_height_m = static_cast<float>(centerline_.bev_height_m);
    for (const auto & point : result.centerline.points) {
      geometry_msgs::msg::Point32 output;
      output.x = point.x;
      output.y = point.y;
      output.z = 0.0F;
      message.centerline_points.push_back(output);
    }
    message.stop_line_present = result.stop_line_present;
    message.stop_line_distance_m = result.stop_line_distance_m;
    const auto result_ready_at = SteadyClock::now();
    message.result_message_build_nanoseconds = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        result_ready_at - message_build_started_at).count());
    message.detector_total_compute_nanoseconds = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        result_ready_at - processing_started_at).count());
    message.detector_result_ready_stamp =
      static_cast<builtin_interfaces::msg::Time>(node_.get_clock()->now());
    result_publisher_->publish(std::move(message));
    // Only the direct RGB BEV path supplies the outside obstacle planner.
    if (direct_bev_input_enabled_ && bev_handoff::avoidancePreviewEnabled()) {
      auto lane=std::make_shared<bev_handoff::PlanningLane>();
      lane->header=input.header; lane->received_at=input_received_at;
      lane->width=input.width; lane->height=input.height;
      lane->x_max=centerline_.bev_height_m; lane->y_max=centerline_.bev_width_m/2;
      lane->meter_per_pixel=centerline_.bev_width_m/input.width;
      lane->lane_width_m=centerline_.lane_width_m;
      const auto metric=[&](const cv::Point2f & p) {
        return cv::Point2d(lane->x_max-(p.y+.5)*centerline_.bev_height_m/input.height,
          lane->y_max-(p.x-connection_.padding_px+.5)*lane->meter_per_pixel);
      };
      // The existing centerline generator already supports a single observed side.
      // Do not add a stricter two-boundary requirement just for avoidance.
      lane->valid=!result.centerline.sample_limit_reached && result.centerline.points.size()>=3 &&
        result.centerline.points.size()<=2000;
      if (lane->valid) {
        for (const auto & p:result.centerline.points) {lane->center.push_back(metric(p));}
        std::size_t count=0;
        for (const auto & side:result.observed_paths) {
          for (const auto & curve:side) {
            if ((count+=curve.size())>2000) {lane->valid=false; break;}
            std::vector<cv::Point2d> boundary;
            for (const auto & p:curve) {boundary.push_back(metric(p));}
            if (boundary.size()>=2) {lane->boundaries.push_back(std::move(boundary));}
          }
          if (!lane->valid) {break;}
        }
      }
      bev_handoff::publishPlanningLane(std::move(lane));
    }
    if (!result.image.empty() &&
      (result_image_publisher_->get_subscription_count() > 0U ||
      result_image_publisher_->get_intra_process_subscription_count() > 0U))
    {
      if (obstacle_overlay_enabled_) {
        auto display=result.image.clone();
        drawObstacleOverlay(display,input.header,model_input_width_,model_input_height_,connection_.padding_px,
          obstacle_sync_sec_,obstacle_age_sec_);
        drawAvoidancePreview(display,input.header,model_input_width_,model_input_height_,connection_.padding_px,
          obstacle_sync_sec_,obstacle_age_sec_);
        result_image_publisher_->publish(image_message(display,input,"bgr8"));
      } else {result_image_publisher_->publish(image_message(result.image,input,"bgr8"));}
    }
  }

  cv::Mat result_overlay(
    const LaneConnectionResult & result,
    const InputFrame & input) const
  {
    cv::Mat overlay;
    const auto bgr=input.bgr.empty() && input.download_bgr ? input.download_bgr() : input.bgr;
    cv::copyMakeBorder(bgr, overlay, 0, 0,
      connection_.padding_px, connection_.padding_px,
      cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    cv::Mat blended;
    cv::addWeighted(overlay, 1.0 - overlay_alpha_, result.image, overlay_alpha_, 0.0, blended);
    cv::Mat visible;
    cv::bitwise_or(result.labels, result.stop_line_mask, visible);
    blended.copyTo(overlay, visible);
    // Solid yellow centerline stays visible regardless of lane overlay alpha.
    overlay.setTo(cv::Scalar(0, 255, 255), result.centerline.mask);
    return overlay;
  }

  cv::Mat preview_canvas(
    const cv::Mat & overlay,
    const double average_inference_milliseconds,
    const double average_postprocess_milliseconds,
    const double average_control_milliseconds,
    const double stop_line_distance_m,
    const double preview_fps,
    const std::uint8_t traffic_signal,
    const double current_speed,
    const double target_speed,
    const int corner_state,
    const double duty,
    const std::string & obstacle_status = "",
    const std::string & avoidance_status = "",
    const std::array<std::string,3> & tunnel_hud = {},
    const std::array<std::string,2> & exposure_hud = std::array<std::string,2>()) const
  {
    const int banner_base_height=kBannerHeight+(tunnel_enabled_ ? 56 : 0);
    // Daylight theme (BGR): white banner with dark text stays readable outdoors.
    const cv::Scalar ink(26,24,20), dim(100,96,86), accent(184,79,28), good(60,122,15),
      caution(0,94,168), alert(29,38,194), idle(168,176,170), rule(193,201,196);
    cv::Mat banner(
      banner_base_height+(obstacle_overlay_enabled_ ? 14 : 0)+(avoidance_status.empty()?0:28), overlay.cols, CV_8UC3,
      cv::Scalar(255,255,255));
    const double inference_fps = average_inference_milliseconds > 0.0 ?
      1000.0 / average_inference_milliseconds : 0.0;
    const double postprocess_fps = average_postprocess_milliseconds > 0.0 ?
      1000.0 / average_postprocess_milliseconds : 0.0;
    const double control_fps = average_control_milliseconds > 0.0 ?
      1000.0 / average_control_milliseconds : 0.0;
    std::vector<std::pair<std::string, cv::Scalar>> lines{
      {cv::format("infer avg %.2fms %.1fFPS", average_inference_milliseconds, inference_fps),
        ink},
      {connection_.enabled ?
        cv::format("post avg %.2fms %.1fFPS", average_postprocess_milliseconds, postprocess_fps) :
        "post OFF", ink},
      {std::isfinite(average_control_milliseconds) ?
        cv::format("control avg %.2fms %.1fFPS", average_control_milliseconds, control_fps) :
        "control -- ms -- FPS", ink},
      {cv::format("preview %.1f FPS", preview_fps), dim},
      {std::isfinite(stop_line_distance_m) ?
        cv::format("stop %.2f m (front axle)", stop_line_distance_m) :
        "stop -- m", std::isfinite(stop_line_distance_m) ? good : dim},
      {traffic_signal == 1 ? "Traffic: RED" :
        traffic_signal == 2 ? "Traffic: GREEN" : "Traffic: UNKNOWN",
        cv::Scalar(255, 255, 255)},
      {std::isfinite(target_speed) ? cv::format("Target: %.2f m/s", target_speed) :
        "Target: -- m/s", accent},
      {std::isfinite(current_speed) ? cv::format("Speed: %.2f m/s", current_speed) :
        "Speed: -- m/s", ink},
      {std::isfinite(duty) ? cv::format("Duty: %.4f", duty) :
        "Duty: --", ink},
      {corner_state == 2 ? "Corner: TURNING" : corner_state == 1 ? "Corner: APPROACH" :
        corner_state == 0 ? "Corner: NO" : corner_state == 3 ? "Corner: STOP" : "Corner: --",
        corner_state == 2 ? caution : dim}};
    if (tunnel_enabled_) {
      lines.insert(lines.begin()+6,std::make_pair(tunnel_hud[1],ink));
      lines.insert(lines.begin()+7,std::make_pair(tunnel_hud[2],dim));
      lines.insert(lines.begin()+8,std::make_pair(exposure_hud[0],ink));
      lines.insert(lines.begin()+9,std::make_pair(exposure_hud[1],ink));
    }
    // The traffic row is a filled label so the signal state reads at a glance.
    cv::rectangle(banner, cv::Rect(0, 5 * 14 + 2, banner.cols, 13),
      traffic_signal == 1 ? alert : traffic_signal == 2 ? good : idle, cv::FILLED);
    cv::line(banner, {0, 0}, {banner.cols - 1, 0}, rule, 1);
    for (std::size_t index = 0; index < lines.size(); ++index) {
      const std::string text=lines[index].first+(
        tunnel_enabled_ && index==5 ? " | "+tunnel_hud[0] : "");
      int baseline=0;
      const auto extent=cv::getTextSize(text,cv::FONT_HERSHEY_SIMPLEX,.28,1,&baseline);
      const double font=std::min(.28,.28*static_cast<double>(banner.cols-6)/std::max(1,extent.width));
      cv::putText(
        banner, text,
        cv::Point(3, 12 + static_cast<int>(index) * 14),
        cv::FONT_HERSHEY_SIMPLEX, font, lines[index].second, 1,
        cv::LINE_AA);
    }
    if (obstacle_overlay_enabled_) {
      cv::putText(banner,obstacle_status,{3,banner_base_height+10},cv::FONT_HERSHEY_SIMPLEX,.28,
        accent,1,cv::LINE_AA);
    }
    if (!avoidance_status.empty()) {
      const int top=banner_base_height+(obstacle_overlay_enabled_?14:0);
      cv::putText(banner,avoidance_status,{3,top+10},cv::FONT_HERSHEY_SIMPLEX,.21,ink,1,cv::LINE_AA);
      cv::putText(banner,bev_handoff::avoidanceControlRequested()?"CONTROL PATH REQUESTED":"PREVIEW ONLY - no control",
        {3,top+24},cv::FONT_HERSHEY_SIMPLEX,.25,bev_handoff::avoidanceControlRequested()?alert:dim,1,cv::LINE_AA);
    }
    cv::Mat canvas;
    cv::vconcat(overlay, banner, canvas);
    return canvas;
  }

  bool window_quit_requested(bool & window_seen) const
  {
    const int key = cv::waitKey(1) & 0xff;
    const double visible = cv::getWindowProperty(
      preview_window_name_, cv::WND_PROP_VISIBLE);
    if (visible >= 1.0) {
      window_seen = true;
    }
    return key == 'q' || key == 'Q' || key == 27 ||
           (window_seen && visible < 1.0);
  }

  void request_shutdown()
  {
    {
      // Synchronize the stop predicate with both waits so shutdown cannot miss a wakeup.
      std::scoped_lock lock(frame_mutex_, preview_mutex_);
      stop_requested_.store(true, std::memory_order_release);
    }
    frame_condition_.notify_all();
    preview_condition_.notify_all();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void worker_loop()
  {
    auto report_started_at = std::chrono::steady_clock::now();
    auto last_error_at = report_started_at - std::chrono::seconds(5);
    std::uint64_t processed_generation = 0U;
    std::uint64_t processed_interval = 0U;
    StageStats preprocessing;
    StageStats execution;
    StageStats correction;
    StageStats connection;
    StageStats postprocessing;
    StageStats preview_execution;
    StageStats preview_postprocess;
    FrameRateGate display_gate;

    try {
      while (!stop_requested_.load(std::memory_order_acquire)) {
        std::shared_ptr<const InputFrame> message;
        std::uint64_t generation = 0U;
        SteadyClock::time_point input_received_at;
        builtin_interfaces::msg::Time input_received_stamp;
        {
          std::unique_lock<std::mutex> lock(frame_mutex_);
          frame_condition_.wait(
            lock, [this, processed_generation]() {
              return stop_requested_.load(std::memory_order_acquire) ||
                     latest_generation_ != processed_generation;
            });
          if (stop_requested_.load(std::memory_order_acquire)) {
            break;
          }
          message = latest_message_;
          generation = latest_generation_;
          input_received_at = latest_input_received_at_;
          input_received_stamp = latest_input_received_stamp_;
        }

        auto now = std::chrono::steady_clock::now();
        if (!message || generation == processed_generation) {
          continue;
        }
        if (processed_generation > 0U && generation > processed_generation + 1U) {
          skipped_total_ += generation - processed_generation - 1U;
        }
        processed_generation = generation;
        const auto processing_started_at = SteadyClock::now();

        ProfileFrame profile_frame;
        auto * profile = profile_writer_ ? &profile_frame.detail : nullptr;
        if (profile) {
          profile_frame.generation = generation;
          profile_frame.skipped_total = skipped_total_;
          profile_frame.source_stamp_ns = static_cast<std::int64_t>(message->header.stamp.sec) *
            1000000000LL + message->header.stamp.nanosec;
          profile_frame.received_stamp_ns = static_cast<std::int64_t>(input_received_stamp.sec) *
            1000000000LL + input_received_stamp.nanosec;
          profile_frame.queue_wait_ms = std::chrono::duration<double, std::milli>(
            processing_started_at - input_received_at).count();
        }
        LaneInferenceTiming timing;
        LaneConnectionResult result;
        std::uint64_t connection_nanoseconds = 0U;
        // Control geometry is always computed; only owning display buffers are sampled.
        const double display_fps=preview_enabled_ && bev_theme_enable_.load() ?
          std::min(preview_fps_,bev_theme_preview_fps_) : preview_fps_;
        const bool display_due=(preview_enabled_ || shared_ir_preview_output_enabled_) &&
          display_gate.accept(std::chrono::duration<double>(SteadyClock::now().time_since_epoch()).count(),display_fps);
        try {
          validate_image(*message);
          ProfileTimer backend_timer(profile, ProfileStage::backend_wall);
          timing = message->device_bgr != nullptr ?
            backend_->infer_device_bgr(
              message->device_bgr, message->device_stride) :
            backend_->infer_bgr(
              message->bgr.data,
              message->bgr.step * static_cast<std::size_t>(message->bgr.rows),
              message->bgr.step);
          backend_timer.stop();
          if(monitor_->enabled()) {
            monitor_->sample("lane_inference",timing.execution_nanoseconds/1.e6);
            monitor_->sample("lane_gpu_preprocess",timing.preprocessing_nanoseconds/1.e6);
          }
          if (profile) {
            profile_frame.gpu_preprocess_ms = nanoseconds_to_milliseconds(timing.preprocessing_nanoseconds);
            profile_frame.gpu_inference_ms = nanoseconds_to_milliseconds(timing.execution_nanoseconds);
            profile_frame.gpu_label_export_ms = nanoseconds_to_milliseconds(timing.label_export_nanoseconds);
            profile_frame.gpu_postprocess_ms = nanoseconds_to_milliseconds(timing.postprocessing_nanoseconds);
          }
          if (connection_.enabled) {
            const auto started = std::chrono::steady_clock::now();
            const bool image_subscribers = result_image_publisher_ &&
              (result_image_publisher_->get_subscription_count() > 0U ||
              result_image_publisher_->get_intra_process_subscription_count() > 0U);
            // Keep the existing bridge/label policy independent of theme state.
            // Only skip display buffers; control geometry remains identical.
            const bool render_result = preview_enabled_ || shared_ir_preview_output_enabled_ || image_subscribers;
            const bool render_legacy_image =
              (display_due && (shared_ir_preview_output_enabled_ || !bev_theme_enable_.load())) || image_subscribers;
            cv::Mat labels(model_input_height_, model_input_width_, CV_8UC1,
              const_cast<std::uint8_t *>(backend_->label_data()));
            profile_frame.render_result = render_legacy_image;
            result = connect_lane_fragments(
              labels, connection_, render_result, profile, render_legacy_image);
            ProfileTimer stop_mask_timer(profile, ProfileStage::stop_mask_render);
            // Stop lines never enter the left/right connector. Retain overlaps
            // in independent masks, painting green only in the display image.
            cv::Mat stop_mask(model_input_height_, model_input_width_, CV_8UC1,
              const_cast<std::uint8_t *>(backend_->stop_line_mask_data()));
            result.stop_line_present = cv::countNonZero(stop_mask) >= stop_line_min_present_pixels_;
            if (render_legacy_image) {
              cv::copyMakeBorder(stop_mask, result.stop_line_mask, 0, 0,
                connection_.padding_px, connection_.padding_px,
                cv::BORDER_CONSTANT, cv::Scalar(0));
              result.image.setTo(cv::Scalar(0, 255, 0), result.stop_line_mask);
            }
            stop_mask_timer.stop();
            result.centerline = generate_centerline(
              result.labels, result.observed_paths,
              model_input_width_, connection_.padding_px, centerline_, render_legacy_image, profile);
            ProfileTimer stop_distance_timer(profile, ProfileStage::stop_distance);
            result.stop_line_distance_m = estimate_stop_line_distance_m(
              stop_mask, result.centerline.points, connection_.padding_px,
              centerline_.bev_width_m, centerline_.bev_height_m, stop_line_distance_);
            stop_distance_timer.stop();
            ProfileTimer centerline_render_timer(profile, ProfileStage::centerline_render);
            if (render_legacy_image) {
              result.image.setTo(cv::Scalar(0, 255, 255), result.centerline.mask);
            }
            centerline_render_timer.stop();
            profile_frame.sample_limit_reached = result.centerline.sample_limit_reached;
            if (result.centerline.sample_limit_reached) {
              RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                "Centerline sample budget exceeded; publishing an empty centerline for this frame");
            }
            connection_nanoseconds = static_cast<std::uint64_t>(
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - started).count());
            timing.correction_nanoseconds += timing.label_export_nanoseconds + connection_nanoseconds;
            if(monitor_->enabled())monitor_->sample("lane_postprocess",connection_nanoseconds/1.e6);
            profile_frame.correction_ms = nanoseconds_to_milliseconds(timing.correction_nanoseconds);
            ProfileTimer publish_timer(profile, ProfileStage::publish);
            publish_result(
              result, *message, input_received_stamp, input_received_at,
              processing_started_at, timing, connection_nanoseconds,
              generation);
          }
          profile_frame.success = true;
        } catch (const std::exception & exception) {
          save_profile(profile_frame, processing_started_at);
          now = std::chrono::steady_clock::now();
          if (now - last_error_at >= std::chrono::seconds(5)) {
            RCLCPP_ERROR(
              node_.get_logger(), "BEV inference frame rejected: %s",
              exception.what());
            last_error_at = now;
          }
          continue;
        }

        save_profile(profile_frame, processing_started_at);
        ++processed_total_;
        ++processed_interval;
        preprocessing.record(timing.preprocessing_nanoseconds);
        execution.record(timing.execution_nanoseconds);
        correction.record(timing.correction_nanoseconds);
        connection.record(connection_nanoseconds);
        postprocessing.record(timing.postprocessing_nanoseconds);
        preview_execution.record(timing.execution_nanoseconds);
        preview_postprocess.record(timing.correction_nanoseconds);

        if (display_due && shared_ir_preview_output_enabled_ && !result.image.empty()) {
          auto shared=std::make_shared<bev_handoff::IrLanePreview>();
          shared->header=message->header;
          shared->image=preview_result_only_enabled_ ? result.image : result_overlay(result,*message);
          shared->inference_ms=preview_execution.average_milliseconds();
          shared->postprocess_ms=preview_postprocess.average_milliseconds();
          shared->stop_distance_m=result.stop_line_distance_m;
          shared->generation=generation;
          bev_handoff::publishIrLanePreview(std::move(shared));
        }

        if (display_due && preview_enabled_) {
          auto snapshot = std::make_shared<PreviewFrame>();
          snapshot->header=message->header;
          if (connection_.enabled && !preview_result_only_enabled_) {
            snapshot->input = message;
          }
          snapshot->result = std::move(result);
          snapshot->average_inference_milliseconds = preview_execution.average_milliseconds();
          snapshot->average_postprocess_milliseconds =
            preview_postprocess.average_milliseconds();
          snapshot->generation = generation;
          if (!connection_.enabled) {
            // Backend buffers are reused by the next inference. The GUI owns this copy.
            const cv::Mat raw(model_input_height_, model_input_width_, CV_8UC3,
              const_cast<std::uint8_t *>(backend_->preview_bgr_data()));
            snapshot->raw = raw.clone();
          }
          {
            std::lock_guard<std::mutex> lock(preview_mutex_);
            latest_preview_ = std::move(snapshot);
          }
        }

        now = std::chrono::steady_clock::now();
        const double report_elapsed =
          std::chrono::duration<double>(now - report_started_at).count();
        if (report_elapsed >= status_log_interval_sec_) {
          std::uint64_t received = 0U;
          {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            received = received_interval_;
            received_interval_ = 0U;
          }
          const double preview_fps = static_cast<double>(
            previewed_interval_.exchange(0U, std::memory_order_relaxed)) / report_elapsed;
          RCLCPP_INFO(
            node_.get_logger(),
            "FPS: input=%.1f, processed=%.1f, preview=%.1f/%.1f | AVG/MAX ms: "
            "H2D+preprocess=%.3f/%.3f, pure-inference=%.3f/%.3f, "
            "correction=%.3f/%.3f, connect=%.3f/%.3f, postprocess+D2H=%.3f/%.3f | "
            "skipped=%llu, processed=%llu",
            static_cast<double>(received) / report_elapsed,
            static_cast<double>(processed_interval) / report_elapsed,
            preview_fps, preview_fps_,
            preprocessing.average_milliseconds(),
            preprocessing.maximum_milliseconds(),
            execution.average_milliseconds(), execution.maximum_milliseconds(),
            correction.average_milliseconds(), correction.maximum_milliseconds(),
            connection.average_milliseconds(), connection.maximum_milliseconds(),
            postprocessing.average_milliseconds(),
            postprocessing.maximum_milliseconds(),
            static_cast<unsigned long long>(skipped_total_),
            static_cast<unsigned long long>(processed_total_));
          if (profile_writer_ && (profile_writer_->failed() || profile_writer_->dropped() > 0U)) {
            RCLCPP_WARN(node_.get_logger(), "Lane profile: write_failed=%s dropped_rows=%llu path=%s",
              profile_writer_->failed() ? "true" : "false",
              static_cast<unsigned long long>(profile_writer_->dropped()),
              profile_writer_->path().c_str());
          }
          preprocessing.reset();
          execution.reset();
          correction.reset();
          connection.reset();
          postprocessing.reset();
          processed_interval = 0U;
          report_started_at = now;
        }
      }
    } catch (const std::exception & exception) {
      RCLCPP_FATAL(
        node_.get_logger(), "Lane inference worker failed: %s", exception.what());
      request_shutdown();
    }
  }

  // All HighGUI calls live here. Slow rendering never blocks inference or result publication.
  void preview_loop()
  {
#if defined(__linux__)
    // Linux nice is per-thread. GUI drawing yields CPU scheduling priority to
    // the inference/control threads in this component container.
    (void)setpriority(PRIO_PROCESS, 0, 10);
#endif
    bool window_seen = false;
    try {
      cv::namedWindow(preview_window_name_, cv::WINDOW_NORMAL);
      cv::resizeWindow(
        preview_window_name_,
        static_cast<int>(result_width() * preview_scale_),
        static_cast<int>((model_input_height_ + kBannerHeight + (obstacle_overlay_enabled_ ? 14 : 0)) * preview_scale_));
      auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / preview_fps_));
      BevThemeRenderer theme_renderer(model_input_width_, model_input_height_,
        connection_.padding_px, centerline_.bev_width_m, centerline_.bev_height_m,
        centerline_.lane_width_m);
      std::uint64_t theme_generation = 0U;
      bool theme_geometry_ready = false;
      bool displayed_theme = false;
      bool displayed_ir = false;
      auto last_motion_at = SteadyClock::now();
      double motion_distance_m = 0.0;
      auto next_preview_at = std::chrono::steady_clock::now();
      auto report_started_at = next_preview_at;
      std::uint64_t displayed_generation = 0U;
      std::uint8_t displayed_signal = 255U;
      std::string displayed_hud;
      cv::Size displayed_canvas_size;
      std::uint64_t displayed_count = 0U;
      double display_fps = 0.0;
      while (!stop_requested_.load(std::memory_order_acquire)) {
        std::shared_ptr<const PreviewFrame> frame;
        double average_control_milliseconds = std::numeric_limits<double>::quiet_NaN();
        double theme_speed = std::numeric_limits<double>::quiet_NaN();
        double target_speed = std::numeric_limits<double>::quiet_NaN();
        double duty = std::numeric_limits<double>::quiet_NaN();
        int corner_state = -1;
        std::array<std::string,3> tunnel_hud;
        {
          std::unique_lock<std::mutex> lock(preview_mutex_);
          preview_condition_.wait_until(lock, next_preview_at, [this]() {
            return stop_requested_.load(std::memory_order_acquire);
          });
          if (stop_requested_.load(std::memory_order_acquire)) {break;}
          frame = latest_preview_;
          tunnel_hud=tunnel_hud_;
          if (tunnel_hud_received_at_!=SteadyClock::time_point{} &&
            SteadyClock::now()-tunnel_hud_received_at_>std::chrono::seconds(1)) {
            tunnel_hud={"Tunnel: STATUS STALE","TCN avg --FPS","TCN: selector unavailable"};
          }
          const auto speed_age = SteadyClock::now() - theme_speed_received_at_;
          if (speed_age >= SteadyClock::duration::zero() &&
            speed_age <= std::chrono::milliseconds(500) && std::isfinite(theme_speed_mps_))
          {
            theme_speed = theme_speed_mps_;
          }
          const auto target_age = SteadyClock::now() - preview_target_speed_received_at_;
          if (target_age >= SteadyClock::duration::zero() &&
            target_age <= std::chrono::milliseconds(500) && std::isfinite(preview_target_speed_mps_))
          {target_speed = preview_target_speed_mps_;}
          const auto duty_age = SteadyClock::now() - preview_duty_received_at_;
          if (duty_age >= SteadyClock::duration::zero() &&
            duty_age <= std::chrono::milliseconds(500) && std::isfinite(preview_duty_))
          {duty = preview_duty_;}
          const auto corner_age = SteadyClock::now() - preview_corner_state_received_at_;
          if (corner_age >= SteadyClock::duration::zero() &&
            corner_age <= std::chrono::milliseconds(500) && preview_corner_state_ <= 3)
          {corner_state = preview_corner_state_;}
          if (control_latency_sample_count_ > 0U) {
            average_control_milliseconds = control_latency_total_milliseconds_ /
              static_cast<double>(control_latency_sample_count_);
          }
        }
        const auto traffic_signal = bev_handoff::latestTrafficSignalState();
        const auto exposure_hud=camera_exposure_hud(bev_handoff::latestCameraExposure(false),
          bev_handoff::latestCameraExposure(true),bev_handoff::latestRgbExposureControl(),SteadyClock::now());
        const bool show_ir=tunnel_enabled_ && tunnel_selected_ir_.load();
        const bool theme = bev_theme_enable_.load() && !show_ir;
        const std::string hud = cv::format("%.2f/%.2f/%d/%.4f", theme_speed, target_speed, corner_state, duty) +
          tunnel_hud[0]+tunnel_hud[1]+tunnel_hud[2]+exposure_hud[0]+exposure_hud[1];
        const auto motion_now = SteadyClock::now();
        const double motion_dt = std::clamp(
          std::chrono::duration<double>(motion_now - last_motion_at).count(), 0.0, 0.25);
        last_motion_at = motion_now;
        if (theme && std::isfinite(theme_speed)) {
          motion_distance_m += std::max(0.0, theme_speed) * motion_dt;
        }
        // Switching off may briefly encounter a snapshot without legacy image
        // buffers. Wait for the next worker result rather than blocking it.
        cv::Mat canvas;
        bool draw=false;
        if (show_ir) {
          const auto ir=bev_handoff::latestIrLanePreview();
          const auto stamp=ir ? rclcpp::Time(ir->header.stamp).nanoseconds() : 0;
          const double age=static_cast<double>(node_.get_clock()->now().nanoseconds()-stamp)*1e-9;
          const bool available=ir && !ir->image.empty();
          const bool fresh=ir_preview_fresh(available,age);
          const cv::Mat overlay=ir_preview_image(available ? ir->image : cv::Mat(),age,
            model_input_height_,result_width());
          canvas=preview_canvas(overlay,fresh ? ir->inference_ms : 0.0,
            fresh ? ir->postprocess_ms : 0.0,average_control_milliseconds,
            fresh ? ir->stop_distance_m : std::numeric_limits<double>::quiet_NaN(),
            display_fps,traffic_signal,theme_speed,target_speed,corner_state,duty,"","",tunnel_hud,exposure_hud);
          displayed_generation=fresh ? ir->generation : 0;
          draw=true;
        } else if (frame && (theme || !connection_.enabled || !frame->result.image.empty()) &&
          (obstacle_overlay_enabled_ || theme || displayed_theme || displayed_ir || frame->generation != displayed_generation ||
          traffic_signal != displayed_signal || hud != displayed_hud))
        {
          if (theme) {
            if (!theme_geometry_ready || theme_generation != frame->generation) {
              theme_renderer.update(frame->result);
              theme_generation = frame->generation;
              theme_geometry_ready = true;
            }
            BevThemeTelemetry telemetry;
            telemetry.speed_mps = theme_speed;
            telemetry.distance_m = motion_distance_m;
            telemetry.inference_ms = frame->average_inference_milliseconds;
            telemetry.postprocess_ms = frame->average_postprocess_milliseconds;
            telemetry.control_ms = average_control_milliseconds;
            telemetry.traffic_fps = bev_handoff::latestTrafficInferenceFps();
            telemetry.preview_fps = display_fps;
            telemetry.signal = traffic_signal;
            canvas = theme_renderer.render(telemetry);
            if (tunnel_enabled_) {append_camera_exposure_hud(canvas,exposure_hud);}
          } else {
            cv::Mat overlay;
            if (connection_.enabled) {
              overlay = preview_result_only_enabled_ ? frame->result.image :
                result_overlay(frame->result, *frame->input);
            } else {
              cv::copyMakeBorder(frame->raw, overlay, 0, 0,
                connection_.padding_px, connection_.padding_px,
                cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
            }
            std::string obstacle_status;
            std::string avoidance_status;
            if (obstacle_overlay_enabled_) {
              overlay=overlay.clone();
              obstacle_status=drawObstacleOverlay(overlay,frame->header,model_input_width_,model_input_height_,
                connection_.padding_px,obstacle_sync_sec_,obstacle_age_sec_);
              avoidance_status=drawAvoidancePreview(overlay,frame->header,model_input_width_,model_input_height_,
                connection_.padding_px,obstacle_sync_sec_,obstacle_age_sec_);
            }
            canvas = preview_canvas(
              overlay,
              frame->average_inference_milliseconds,
              frame->average_postprocess_milliseconds,
              average_control_milliseconds,
              frame->result.stop_line_distance_m, display_fps, traffic_signal,
              theme_speed, target_speed, corner_state, duty, obstacle_status, avoidance_status,tunnel_hud,exposure_hud);
          }
          displayed_generation = frame->generation;
          draw=true;
        } else if (!frame) {
          cv::Mat waiting=cv::Mat::zeros(model_input_height_,result_width(),CV_8UC3);
          cv::putText(waiting,"RGB: waiting for first result",cv::Point(4,14),
            cv::FONT_HERSHEY_SIMPLEX,.3,cv::Scalar(255,255,255),1,cv::LINE_AA);
          canvas=preview_canvas(waiting,0.0,0.0,average_control_milliseconds,
            std::numeric_limits<double>::quiet_NaN(),display_fps,traffic_signal,
            theme_speed,target_speed,corner_state,duty,"","",tunnel_hud,exposure_hud);
          draw=true;
        }
        if (draw) {
          if (canvas.size() != displayed_canvas_size || theme != displayed_theme) {
            // The theme renders at 2x source resolution; preserve the BEV's
            // original visual size when switching themes.
            const double scale = preview_scale_*(theme ? .5 : 1.);
            cv::resizeWindow(preview_window_name_,
              cvRound(canvas.cols*scale), cvRound(canvas.rows*scale));
            displayed_canvas_size = canvas.size();
          }
          cv::imshow(preview_window_name_, canvas);
          displayed_theme = theme;
          displayed_ir = show_ir;
          displayed_signal = traffic_signal;
          displayed_hud = hud;
          ++displayed_count;
          previewed_interval_.fetch_add(1U, std::memory_order_relaxed);
        }
        if (window_quit_requested(window_seen)) {
          request_shutdown();
          break;
        }
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - report_started_at).count();
        if (elapsed >= status_log_interval_sec_) {
          display_fps = static_cast<double>(displayed_count) / elapsed;
          displayed_count = 0U;
          report_started_at = now;
        }
        const double effective_fps = theme ?
          std::min(preview_fps_, bev_theme_preview_fps_) : preview_fps_;
        period = std::chrono::duration_cast<SteadyClock::duration>(
          std::chrono::duration<double>(1.0 / effective_fps));
        next_preview_at += period;
        if (next_preview_at < now) {next_preview_at = now + period;}
      }
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(node_.get_logger(),
        "Lane GUI disabled after error; inference/control continue: %s", exception.what());
    }
    try {
      cv::destroyWindow(preview_window_name_);
    } catch (const cv::Exception &) {
    }
  }

  void stop()
  {
    bev_handoff::unregisterDirectBevConsumer(direct_bev_consumer_id_);
    direct_bev_consumer_id_ = 0U;
    {
      // Synchronize the stop predicate with both waits so shutdown cannot miss a wakeup.
      std::scoped_lock lock(frame_mutex_, preview_mutex_);
      stop_requested_.store(true, std::memory_order_release);
    }
    frame_condition_.notify_all();
    preview_condition_.notify_all();
    if (worker_.joinable() && worker_.get_id() != std::this_thread::get_id()) {
      worker_.join();
    }
    if (preview_worker_.joinable() && preview_worker_.get_id() != std::this_thread::get_id()) {
      preview_worker_.join();
    }
    if (shared_ir_preview_output_enabled_) {bev_handoff::publishIrLanePreview(nullptr);}
    if (profile_writer_) {
      profile_writer_->close();
      if (profile_writer_->failed()) {
        RCLCPP_ERROR(node_.get_logger(), "Lane profile write failed; partial file: %s",
          profile_writer_->path().c_str());
      } else {
        RCLCPP_INFO(node_.get_logger(), "Lane profile saved: %s (dropped rows=%llu)",
          profile_writer_->path().c_str(),
          static_cast<unsigned long long>(profile_writer_->dropped()));
      }
      profile_writer_.reset();
    }
  }

  LineDetactorNode & node_;
  std::string input_topic_;
  bool direct_bev_input_enabled_{false};
  std::string model_path_;
  std::string engine_cache_path_;
  std::string engine_precision_{"fp32"};
  int tensorrt_workspace_size_mb_{1024};
  int model_input_width_{kDefaultInputWidth};
  int model_input_height_{kDefaultInputHeight};
  float mask_threshold_{0.5F};
  float overlay_alpha_{0.75F};
  float stop_line_mask_threshold_{-1.0F};
  int stop_line_min_present_pixels_{1};
  StopLineDistanceConfig stop_line_distance_;
  LaneConnectionConfig connection_;
  CenterlineConfig centerline_;
  bool result_publish_enabled_{true};
  std::string result_topic_;
  std::string result_image_topic_;
  std::string control_latency_topic_;
  rclcpp::Publisher<line_detactor::msg::LaneResult>::SharedPtr result_publisher_;
  rclcpp::Publisher<Image>::SharedPtr result_image_publisher_;
  int warmup_iterations_{10};
  bool obstacle_overlay_enabled_{false};
  double obstacle_sync_sec_{.06},obstacle_age_sec_{.25};
  bool preview_enabled_{true};
  std::atomic<bool> bev_theme_enable_{false};
  double bev_theme_preview_fps_{15.0};
  std::string bev_theme_speed_topic_;
  std::string preview_target_speed_topic_, preview_corner_state_topic_, preview_duty_topic_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr theme_parameter_callback_;
  bool preview_result_only_enabled_{true};
  std::unique_ptr<bev_handoff::RuntimeMonitor> monitor_;
  double preview_fps_{30.0};
  double preview_scale_{2.0};
  std::string preview_window_name_;
  double status_log_interval_sec_{1.0};

  bool profiling_enabled_{false};
  std::string profiling_directory_;
  SteadyClock::time_point profile_started_at_{};
  std::unique_ptr<ProcessingProfileWriter> profile_writer_;
  std::unique_ptr<TensorRtLaneBackend> backend_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr model_status_pub_;
  std::unique_ptr<bev_handoff::ModelLoadStatus> model_load_status_;
  rclcpp::Subscription<Image>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr control_latency_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_speed_subscription_, duty_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr corner_state_subscription_;
  std::mutex frame_mutex_;
  FrameRateGate rate_gate_;
  bool tunnel_enabled_{false};
  bool shared_ir_preview_output_enabled_{false};
  std::atomic<bool> tunnel_selected_ir_{false};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tunnel_selected_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tunnel_preview_sub_;
  std::array<std::string,3> tunnel_hud_{
    "Tunnel: WAITING","TCN avg --FPS","TCN: waiting"};
  SteadyClock::time_point tunnel_hud_received_at_{};
  std::atomic<double> inference_max_fps_{0.0};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tunnel_compute_sub_;
  std::condition_variable frame_condition_;
  std::shared_ptr<const InputFrame> latest_message_;
  SteadyClock::time_point latest_input_received_at_{};
  builtin_interfaces::msg::Time latest_input_received_stamp_;
  std::uint64_t latest_generation_{0U};
  std::uint64_t received_interval_{0U};
  std::uint64_t received_total_{0U};
  std::uint64_t processed_total_{0U};
  std::uint64_t skipped_total_{0U};
  std::atomic<bool> stop_requested_{false};
  std::thread worker_;
  std::uint64_t direct_bev_consumer_id_{0U};
  std::mutex preview_mutex_;
  double theme_speed_mps_{std::numeric_limits<double>::quiet_NaN()};
  SteadyClock::time_point theme_speed_received_at_{};
  double preview_target_speed_mps_{std::numeric_limits<double>::quiet_NaN()};
  double preview_duty_{std::numeric_limits<double>::quiet_NaN()};
  SteadyClock::time_point preview_duty_received_at_{};
  int preview_corner_state_{-1};
  SteadyClock::time_point preview_target_speed_received_at_{}, preview_corner_state_received_at_{};
  std::condition_variable preview_condition_;
  std::shared_ptr<const PreviewFrame> latest_preview_;
  std::uint64_t control_latency_sample_count_{0U};
  double control_latency_total_milliseconds_{0.0};
  std::atomic<std::uint64_t> previewed_interval_{0U};
  std::thread preview_worker_;
};

LineDetactorNode::LineDetactorNode(const rclcpp::NodeOptions & options)
: Node("line_detactor", options), impl_(std::make_unique<Impl>(*this))
{
}

LineDetactorNode::~LineDetactorNode() = default;

}  // namespace line_detactor

RCLCPP_COMPONENTS_REGISTER_NODE(line_detactor::LineDetactorNode)
