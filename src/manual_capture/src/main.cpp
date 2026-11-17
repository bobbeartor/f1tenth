// manual_capture: OAK RGB + stereo preview/capture GUI for labeling datasets.

#include <QApplication>
#include <QMetaObject>

#include <atomic>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "manual_capture/main_window.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"

namespace manual_capture
{
namespace
{

StereoMode parseStereoMode(const std::string & value)
{
  if (value == "left_right") {return StereoMode::LEFT_RIGHT;}
  if (value == "left_only") {return StereoMode::LEFT_ONLY;}
  throw std::invalid_argument("stereo_mode must be left_right or left_only");
}

RgbFormat parseRgbFormat(const std::string & value)
{
  if (value == "NV12" || value == "nv12") {return RgbFormat::NV12;}
  if (value == "RGB888" || value == "rgb888" || value == "RGB") {return RgbFormat::RGB888;}
  throw std::invalid_argument("rgb_format must be NV12 or RGB888");
}

AppParams readParameters(rclcpp::Node & node)
{
  AppParams p;
  auto & camera = p.camera;
  camera.device_id = node.declare_parameter<std::string>("device_id", "");
  camera.rgb_width = node.declare_parameter<int>("rgb_width", 1280);
  camera.rgb_height = node.declare_parameter<int>("rgb_height", 800);
  camera.rgb_fps = node.declare_parameter<double>("rgb_fps", 30.0);
  camera.rgb_format = parseRgbFormat(node.declare_parameter<std::string>("rgb_format", "NV12"));
  camera.rgb_undistort = node.declare_parameter<bool>("rgb_undistort", true);
  camera.stereo_width = node.declare_parameter<int>("stereo_width", 1280);
  camera.stereo_height = node.declare_parameter<int>("stereo_height", 800);
  camera.stereo_fps = node.declare_parameter<double>("stereo_fps", 30.0);
  camera.stereo_mode = parseStereoMode(
    node.declare_parameter<std::string>("stereo_mode", "left_right"));
  camera.virtual_camera_position_ratio =
    node.declare_parameter<double>("virtual_camera_position_ratio", 0.5);
  camera.sync_threshold_ms = node.declare_parameter<double>("sync_threshold_ms", 2.0);

  auto read_exposure = [&node](const std::string & prefix, ExposureSetting defaults) {
      defaults.automatic = node.declare_parameter<bool>(prefix + "_auto_exposure", defaults.automatic);
      defaults.exposure_us = node.declare_parameter<int>(prefix + "_exposure_us", defaults.exposure_us);
      defaults.iso = node.declare_parameter<int>(prefix + "_iso", defaults.iso);
      return defaults;
    };
  p.controls.rgb = read_exposure("rgb", {true, 8000, 400});
  p.controls.left = read_exposure("stereo", {false, 5000, 800});
  p.controls.right = p.controls.left;
  p.dot_intensity_when_needed = node.declare_parameter<double>("dot_projector_intensity", 1.0);

  auto & grid = p.grid;
  grid.camera_x_m = node.declare_parameter<double>("bev.camera_x_m", grid.camera_x_m);
  grid.camera_y_m = node.declare_parameter<double>("bev.camera_y_m", grid.camera_y_m);
  grid.camera_yaw_deg = node.declare_parameter<double>("bev.camera_yaw_deg", grid.camera_yaw_deg);
  grid.x_min_m = node.declare_parameter<double>("bev.x_min_m", grid.x_min_m);
  grid.x_max_m = node.declare_parameter<double>("bev.x_max_m", grid.x_max_m);
  grid.y_min_m = node.declare_parameter<double>("bev.y_min_m", grid.y_min_m);
  grid.y_max_m = node.declare_parameter<double>("bev.y_max_m", grid.y_max_m);
  grid.meter_per_pixel = node.declare_parameter<double>("bev.meter_per_pixel", grid.meter_per_pixel);
  grid.output_width = node.declare_parameter<int>("bev.output_width", grid.output_width);
  grid.output_height = node.declare_parameter<int>("bev.output_height", grid.output_height);
  validateGrid(grid);
  p.default_pose.height_m = node.declare_parameter<double>("bev.default_camera_height_m", 0.20);
  p.default_pose.roll_deg = node.declare_parameter<double>("bev.default_camera_roll_deg", 0.0);
  p.default_pose.pitch_down_deg =
    node.declare_parameter<double>("bev.default_camera_pitch_down_deg", 14.0);
  p.startup_measurement_enabled =
    node.declare_parameter<bool>("bev.startup_measurement_enabled", true);

  auto & m = p.measurement;
  m.stereo_fps = node.declare_parameter<double>("measurement_stereo_fps", m.stereo_fps);
  m.warmup_sec = node.declare_parameter<double>("measurement_warmup_sec", m.warmup_sec);
  m.timeout_sec = node.declare_parameter<double>("measurement_timeout_sec", m.timeout_sec);
  m.ir_dot_projector_intensity = node.declare_parameter<double>(
    "measurement_ir_dot_projector_intensity", m.ir_dot_projector_intensity);
  m.manual_camera_height_enabled = node.declare_parameter<bool>(
    "measurement_manual_camera_height_enabled", m.manual_camera_height_enabled);
  m.manual_camera_height_m = node.declare_parameter<double>(
    "measurement_manual_camera_height_m", m.manual_camera_height_m);
  m.attitude_source = parseStartupAttitudeSource(
    node.declare_parameter<std::string>("measurement_attitude_source", "depth"));
  m.roi_width = node.declare_parameter<int>("measurement_roi_width", m.roi_width);
  m.roi_height = node.declare_parameter<int>("measurement_roi_height", m.roi_height);
  m.roi_vertical_offset_px = node.declare_parameter<int>(
    "measurement_roi_vertical_offset_px", m.roi_vertical_offset_px);
  m.minimum_valid_points = node.declare_parameter<int>(
    "measurement_minimum_valid_points", m.minimum_valid_points);
  m.minimum_height_m = node.declare_parameter<double>("measurement_minimum_height_m", m.minimum_height_m);
  m.maximum_height_m = node.declare_parameter<double>("measurement_maximum_height_m", m.maximum_height_m);
  m.roi_preview_enabled = false;

  p.capture_root = node.declare_parameter<std::string>("capture_root", "captures");
  p.jpeg_quality = node.declare_parameter<int>("jpeg_quality", 95);
  p.writer_queue_events = node.declare_parameter<int>("writer_queue_events", 64);
  p.auto_capture_fps = node.declare_parameter<double>("auto_capture_fps", 5.0);
  p.camera_preview_fps = node.declare_parameter<int>("camera_preview_fps", 15);
  p.bev_preview_fps = node.declare_parameter<int>("bev_preview_fps", 10);
  p.refit_maximum_abs_erpm = node.declare_parameter<int>("refit_maximum_abs_erpm", 100);
  p.erpm_timeout_sec = node.declare_parameter<double>("erpm_timeout_sec", 1.0);
  return p;
}

std::atomic<QApplication *> g_app{nullptr};

}  // namespace
}  // namespace manual_capture

int main(int argc, char * argv[])
{
  using namespace manual_capture;
  rclcpp::init(argc, argv);
  int exit_code = 0;
  {
    auto node = std::make_shared<rclcpp::Node>("manual_capture");
    AppParams params;
    try {
      params = readParameters(*node);
    } catch (const std::exception & exception) {
      RCLCPP_FATAL(node->get_logger(), "invalid parameters: %s", exception.what());
      rclcpp::shutdown();
      return 1;
    }
    const auto joy_topic = node->declare_parameter<std::string>("capture_joy_topic", "/autopilot03/joy");
    const auto capture_button = node->declare_parameter<int>("capture_button_index", 0);
    const auto erpm_topic = node->declare_parameter<std::string>("measured_erpm_topic", "/vesc/measured_erpm");
    const auto servo_topic = node->declare_parameter<std::string>("servo_position_topic", "/vesc/servo_position");

    QApplication app(argc, argv);
    auto telemetry = std::make_shared<VehicleTelemetry>();
    MainWindow window(params, telemetry);

    ButtonEdge edge;
    auto joy = node->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic, rclcpp::SensorDataQoS(),
      [&window, &edge, capture_button, telemetry](sensor_msgs::msg::Joy::ConstSharedPtr message) {
        telemetry->joy_connected.store(true);
        const bool pressed = capture_button >= 0 &&
        static_cast<std::size_t>(capture_button) < message->buttons.size() &&
        message->buttons[static_cast<std::size_t>(capture_button)] != 0;
        if (edge.update(pressed)) {
          QMetaObject::invokeMethod(&window, [&window]() {window.onJoyCapture();}, Qt::QueuedConnection);
        }
      });
    auto erpm = node->create_subscription<std_msgs::msg::Int32>(
      erpm_topic, 10, [telemetry](std_msgs::msg::Int32::ConstSharedPtr message) {
        telemetry->erpm.store(message->data);
        telemetry->erpm_time_sec.store(steadyNowSec());
        telemetry->has_erpm.store(true);
      });
    auto servo = node->create_subscription<std_msgs::msg::Float32>(
      servo_topic, 10, [telemetry](std_msgs::msg::Float32::ConstSharedPtr message) {
        telemetry->servo.store(message->data);
        telemetry->has_servo.store(true);
      });
    RCLCPP_INFO(node->get_logger(), "capture root: %s, joy: %s (button %ld)",
      params.capture_root.c_str(), joy_topic.c_str(), static_cast<long>(capture_button));

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() {executor.spin();});
    // Ctrl+C in the launch terminal closes the window cleanly.
    g_app.store(&app);
    rclcpp::on_shutdown([]() {
        if (auto * application = g_app.exchange(nullptr)) {
          QMetaObject::invokeMethod(application, []() {QApplication::quit();}, Qt::QueuedConnection);
        }
      });

    window.showMaximized();
    exit_code = app.exec();
    g_app.store(nullptr);
    window.close();
    executor.cancel();
    if (spin_thread.joinable()) {
      spin_thread.join();
    }
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
