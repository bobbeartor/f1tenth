#include <bev_handoff/runtime_monitor.hpp>
#include <bev_handoff/stage_meter.hpp>
#include <iomanip>
#include <sstream>
#include <map>
#include <set>
#include "auto_control/control_core.hpp"
#include "auto_control/capture_pair_order.hpp"
#include "auto_control/lane_capture_order.hpp"
#include "auto_control/model_startup_gate.hpp"
#include "auto_control/start_authorization.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "auto_control/runtime_speed_limit.hpp"
#include "line_detactor/msg/speed_limit_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "auto_control/msg/avoidance_plan.hpp"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include "auto_control/driving_log.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "line_detactor/msg/lane_result.hpp"
#include "traffic_detection_test/msg/traffic_light_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace auto_control
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kUnavailable = std::numeric_limits<double>::quiet_NaN();
double radians(double degrees) {return degrees * kPi / 180.0;}
double seconds(std::int64_t nanoseconds) {return nanoseconds / 1.0e9;}
std::optional<std::int64_t> stamp_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec == 0 && stamp.nanosec == 0U) {return std::nullopt;}
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}
}  // namespace

class AutoControlNode : public rclcpp::Node
{
public:
  AutoControlNode()
  : Node("auto_control")
  {
    declare_and_read_parameters();
    validate_parameters();
    monitor_=std::make_unique<bev_handoff::RuntimeMonitor>(*this);
    runtime_speed_limit_ = std::make_unique<RuntimeSpeedLimit>(minimum_speed_mps_, maximum_speed_mps_);
    speed_pid_ = std::make_unique<SpeedPid>(
      speed_pid_kp_, speed_pid_ki_, speed_pid_kd_, speed_pid_integral_limit_,
      minimum_duty_, maximum_duty_);
    longitudinal_pid_ = std::make_unique<SpeedPid>(
      longitudinal_kp_, longitudinal_ki_, longitudinal_kd_, longitudinal_integral_limit_, -1.0, 1.0);
    brake_profile_ = std::make_unique<AutomaticBrakeProfile>(BrakeConfig{
      brake_entry_speed_error_mps_, brake_exit_speed_error_mps_,
      brake_minimum_vehicle_speed_mps_, brake_minimum_current_amps_,
      brake_maximum_current_amps_, brake_current_gain_amps_per_mps_,
      brake_current_rise_amps_per_sec_, brake_current_fall_amps_per_sec_});

    auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    auto connection_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    duty_pub_ = create_publisher<std_msgs::msg::Float32>(duty_topic_, command_qos);
    brake_pub_ = create_publisher<std_msgs::msg::Float32>(brake_current_topic_, command_qos);
    servo_pub_ = create_publisher<std_msgs::msg::Float32>(servo_position_topic_, command_qos);
    command_duty_pub_ = create_publisher<std_msgs::msg::Float32>(command_duty_topic_, command_qos);
    command_brake_pub_ = create_publisher<std_msgs::msg::Float32>(command_brake_current_topic_, command_qos);
    target_speed_pub_ = create_publisher<std_msgs::msg::Float32>(target_speed_topic_, command_qos);
    current_speed_pub_ = create_publisher<std_msgs::msg::Float32>(current_speed_topic_, command_qos);
    corner_state_pub_ = create_publisher<std_msgs::msg::UInt8>(corner_state_topic_, command_qos);
    curvature_pub_ = create_publisher<std_msgs::msg::Float32>(curvature_topic_, command_qos);
    steering_pub_ = create_publisher<std_msgs::msg::Float32>(steering_angle_topic_, command_qos);
    cross_track_pub_ = create_publisher<std_msgs::msg::Float32>(cross_track_error_topic_, command_qos);
    heading_pub_ = create_publisher<std_msgs::msg::Float32>(heading_error_topic_, command_qos);
    raw_steering_pub_ = create_publisher<std_msgs::msg::Float32>(raw_steering_angle_topic_, command_qos);
    command_servo_pub_ = create_publisher<std_msgs::msg::Float32>(command_servo_position_topic_, command_qos);
    input_to_control_decision_pub_ = create_publisher<std_msgs::msg::Float32>(
      input_to_control_decision_topic_, command_qos);
    // Remote preview stage 4: lane result ready (postprocess done) -> steering decision.
    result_to_control_decision_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/perf/lane_result_to_control_ms", command_qos);
    // One 30 Hz JSON summary for the remote preview, built only while someone subscribes,
    // so the Python bridge does not subscribe to every diagnostic topic and LaneResult.
    preview_snapshot_pub_ = create_publisher<std_msgs::msg::String>(
      "/auto/preview_snapshot", rclcpp::QoS(1).best_effort());
    preview_meters_drained_at_ = std::chrono::steady_clock::now();
    preview_snapshot_timer_ = create_wall_timer(
      std::chrono::milliseconds(33), std::bind(&AutoControlNode::publish_preview_snapshot, this));
    model_startup_check_required_ = declare_parameter<bool>("model_startup_check_required", false);
    startup_safety_enabled_=declare_parameter<bool>("startup_safety_enabled",false);
    if (startup_safety_enabled_) {
      model_startup_check_required_=true;
      const auto state_qos=rclcpp::QoS(1).reliable().transient_local();
      startup_state_pub_=create_publisher<std_msgs::msg::String>("~/startup_state",state_qos);
      camera_startup_sub_=create_subscription<std_msgs::msg::String>(
        "/bev_processor/startup_status",state_qos,
        [this](std_msgs::msg::String::ConstSharedPtr message) {
          camera_startup_ready_=message->data.rfind("READY | ",0)==0;
          if (!camera_startup_ready_) {start_authorization_.revoke("Camera pose not ready");}
        });
      startup_mode_sub_=create_subscription<std_msgs::msg::String>(
        declare_parameter<std::string>("startup_drive_mode_topic","/autopilot03/drive/mode"),state_qos,
        [this](std_msgs::msg::String::ConstSharedPtr message) {
          if (message->data!="AUTO" && message->data!="MANUAL" && message->data!="STOPPING") {return;}
          startup_drive_mode_=message->data;
          startup_manual_=message->data=="MANUAL" || message->data=="STOPPING";
          if (startup_manual_) {start_authorization_.revoke("Manual mode");}
        });
      startup_start_service_=create_service<std_srvs::srv::Trigger>("~/start",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          refresh_start_authorization();
          const auto reason=stop_reason(now().nanoseconds(),false);
          response->success=start_authorization_.request(!reason.has_value() && control_mode_=="drive");
          response->message=response->success ? "STARTED: automatic duty permitted" :
            (reason ? *reason : "critical readiness not satisfied or automatic drive mode required");
          if(response->success)operator_stopped_=false;
          publish_startup_state();
        });
      startup_state_timer_=create_wall_timer(std::chrono::milliseconds(100),[this]() {
        refresh_start_authorization(); publish_startup_state();
      });
    }
    if (model_startup_check_required_) {
      model_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/perception/models_ready", rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::Bool::ConstSharedPtr message) {
          model_startup_gate_.observe(message->data, std::chrono::steady_clock::now());
          if (!message->data) {start_authorization_.revoke("Model readiness lost");}
        });
    }
    tunnel_enabled_=declare_parameter<bool>("tunnel.enabled",false);
    if (tunnel_enabled_) {
      // The selector forwards each camera's unmodified detector output on its
      // own topic. The camera identity is the topic, not a rewritten field.
      lane_sub_ = create_subscription<line_detactor::msg::LaneResult>(
        lane_result_topic_ + "/rgb", sensor_qos,
        [this](line_detactor::msg::LaneResult::ConstSharedPtr message) {on_lane_result(message, "RGB");});
      tunnel_ir_lane_sub_ = create_subscription<line_detactor::msg::LaneResult>(
        lane_result_topic_ + "/ir", sensor_qos,
        [this](line_detactor::msg::LaneResult::ConstSharedPtr message) {on_lane_result(message, "IR");});
    } else {
      lane_sub_ = create_subscription<line_detactor::msg::LaneResult>(
        lane_result_topic_, sensor_qos,
        [this](line_detactor::msg::LaneResult::ConstSharedPtr message) {on_lane_result(message, "");});
    }
    if (tunnel_enabled_) {
      outside_avoidance_=avoidance_control_enabled_;
      outside_slowdown_=obstacle_slowdown_enabled_;
      outside_traffic_=traffic_stop_enabled_;
      tunnel_state_sub_=create_subscription<std_msgs::msg::String>(
        "/tunnel/state",rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr message) {
          if (message->data!="OUTSIDE" && message->data!="ENTERING" &&
            message->data!="INSIDE" && message->data!="EXITING") {return;}
          const bool inside=message->data!="OUTSIDE";
          if (inside==tunnel_active_) {return;}
          tunnel_active_=inside;
          avoidance_control_enabled_=!inside && outside_avoidance_;
          obstacle_slowdown_enabled_=!inside && outside_slowdown_;
          traffic_stop_enabled_=!inside && outside_traffic_;
          // No outside detection/plan or red-light stop may survive entry.
          avoidance_path_.reset(); avoidance_lane_ns_.reset(); avoidance_depth_ns_.reset();
          avoidance_follow_centerline_=false; avoidance_confirmations_=0;
          obstacle_slowdown_active_=false; obstacle_last_seen_at_.reset();
          obstacle_last_depth_ns_.reset(); obstacle_planned_speed_.reset();
          obstacle_profile_active_=false;
          traffic_stop_active_=false; traffic_holding_=false;
          traffic_pass_committed_=false; traffic_red_=false;
          clear_stop_line_track();
          RCLCPP_INFO(get_logger(),"Tunnel mode=%s; outside obstacle/traffic control=%s",
            message->data.c_str(),inside ? "suspended" : "restored with fresh inputs");
        });
    }
    if (avoidance_control_enabled_ || obstacle_slowdown_enabled_) {
      avoidance_sub_=create_subscription<auto_control::msg::AvoidancePlan>(
        "/auto/avoidance_plan",sensor_qos,
        std::bind(&AutoControlNode::on_avoidance_plan,this,std::placeholders::_1));
    }
    if (traffic_stop_enabled_ || driving_log_enabled_) {
      traffic_sub_ = create_subscription<traffic_detection_test::msg::TrafficLightState>(
        traffic_state_topic_, sensor_qos,
        std::bind(&AutoControlNode::on_traffic_state, this, std::placeholders::_1));
    }
    erpm_sub_ = create_subscription<std_msgs::msg::Int32>(
      measured_erpm_topic_, sensor_qos,
      std::bind(&AutoControlNode::on_measured_erpm, this, std::placeholders::_1));
    if (curvature_speed_control_enabled_ || obstacle_slowdown_enabled_) {
      dynamics_duty_sub_ = create_subscription<std_msgs::msg::Float32>(
        dynamics_duty_topic_, sensor_qos,
        [this](const std_msgs::msg::Float32::ConstSharedPtr message) {
          if (!std::isfinite(message->data)) {return;}
          dynamics_duty_ = message->data;
          dynamics_duty_seen_ns_ = now().nanoseconds();
        });
      dynamics_motor_current_sub_ = create_subscription<std_msgs::msg::Float32>(
        dynamics_motor_current_topic_, sensor_qos,
        [this](const std_msgs::msg::Float32::ConstSharedPtr message) {
          if (!std::isfinite(message->data)) {return;}
          dynamics_motor_current_ = message->data;
          dynamics_motor_current_seen_ns_ = now().nanoseconds();
        });
    }
    connection_sub_ = create_subscription<std_msgs::msg::Bool>(
      connection_status_topic_, connection_qos,
      std::bind(&AutoControlNode::on_connection_status, this, std::placeholders::_1));
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      enable_topic_, command_qos,
      std::bind(&AutoControlNode::on_enable, this, std::placeholders::_1));

    // Reliable volatile requests cannot be replayed by a newly started controller.
    speed_limit_state_pub_ = create_publisher<line_detactor::msg::SpeedLimitState>(
      "/auto/maximum_speed_state", rclcpp::QoS(1).reliable());
    speed_limit_request_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/auto/maximum_speed_request", rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::Float64::ConstSharedPtr message) {
        if (!runtime_speed_limit_->request(message->data)) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "Rejected maximum speed %.3f (allowed %.3f..%.3f m/s)", message->data,
            runtime_speed_limit_->minimum(), runtime_speed_limit_->ceiling());
          publish_speed_limit_state();
          return;
        }
        const double previous = maximum_speed_mps_;
        maximum_speed_mps_ = runtime_speed_limit_->applied();
        if (previous != maximum_speed_mps_) {
          // Retained curve/obstacle profiles must also respect a lowered ceiling.
          if (obstacle_planned_speed_) {
            *obstacle_planned_speed_ = std::min(*obstacle_planned_speed_, maximum_speed_mps_);
          }
          corner_lower_speed_mps_ = std::min(corner_lower_speed_mps_, maximum_speed_mps_);
          corner_nominal_speed_mps_ = std::min(corner_nominal_speed_mps_, maximum_speed_mps_);
          corner_upper_speed_mps_ = std::min(corner_upper_speed_mps_, maximum_speed_mps_);
          speed_pid_->reset();
          longitudinal_pid_->reset();
          RCLCPP_INFO(get_logger(), "BEV maximum speed applied: %.2f m/s", maximum_speed_mps_);
        }
        publish_speed_limit_state();
      });
    speed_limit_state_timer_ = create_wall_timer(std::chrono::milliseconds(200),
      std::bind(&AutoControlNode::publish_speed_limit_state, this));
    publish_speed_limit_state();

    latest_servo_position_ = servo_center_;
    last_control_ns_ = now().nanoseconds();
    watchdog_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_hz_),
      std::bind(&AutoControlNode::on_watchdog, this));
    status_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / status_log_rate_hz_),
      std::bind(&AutoControlNode::log_status, this));

    install_runtime_controls();
    if (driving_log_enabled_) {start_driving_log();}
    if (enabled_ && control_mode_ == "drive" && startup_safety_enabled_) {
      RCLCPP_INFO(get_logger(),"Startup inspection active: automatic duty awaits camera/models/telemetry/lane checks and the GUI START button.");
    } else if (enabled_ && control_mode_ == "drive") {
      RCLCPP_WARN(get_logger(),
        "Automatic control is armed at launch. The vehicle will move when VESC telemetry, "
        "a valid ML centerline and all required models are ready.");
    } else if (enabled_) {
      RCLCPP_WARN(get_logger(),
        "Automatic control starts in %s mode; suppressed actuator topics will not be published.",
        control_mode_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Automatic control starts disabled.");
    }
    RCLCPP_INFO(get_logger(), "Speed control: PID=%s, staged=%s, corner slowdown=%s, gains=%.3f/%.3f/%.3f",
      longitudinal_pid_enabled_ ? "on" : "off (red stop still uses PID)",
      staged_control_enabled_ ? "on" : "off",
      curvature_speed_control_enabled_ ? "on" : "off", longitudinal_kp_, longitudinal_ki_, longitudinal_kd_);
    RCLCPP_INFO(get_logger(), "Staged brake gains: speed=%.3f, missing deceleration=%.3f",
      longitudinal_brake_speed_gain_, longitudinal_brake_decel_gain_);
    RCLCPP_INFO(get_logger(),
      "Corner envelope: width=%.2fm/s coast_margin=%.2fm/s brake=%.2f/%.2fm/s "
      "decel_margin=%.2fm/s2 probe=%.2fs confirm/release=%d/%d",
      corner_speed_band_width_mps_, corner_coast_entry_margin_mps_,
      corner_brake_entry_margin_mps_, corner_brake_exit_margin_mps_,
      corner_brake_decel_margin_mps2_, corner_coast_probe_sec_,
      corner_confirm_frames_, corner_release_frames_);
    RCLCPP_INFO(get_logger(),"Avoidance control=%s: plan=/auto/avoidance_plan speed=maximum_speed_mps (%.2fm/s), no avoidance cap; age=%.2fs guard_brake=%.2fA",
      avoidance_control_enabled_?"ON":"OFF",maximum_speed_mps_,avoidance_max_age_,avoidance_brake_current_);
    RCLCPP_INFO(get_logger(),"Avoidance mode=%s",avoidance_deformation_only_?"DEFORM: unavailable plan uses centerline; normal longitudinal control":"CHECKED: unavailable plan brakes");
    RCLCPP_INFO(get_logger(),
      "Obstacle slowdown=%s: final speed=%.2fm/s clear=%.2fs decel/accel=%.2f/%.2fm/s2, recovery waits for turn exit",
      obstacle_slowdown_enabled_?"ON":"OFF", obstacle_slowdown_speed_mps_, obstacle_slowdown_clear_sec_,
      obstacle_planning_deceleration_, obstacle_planning_acceleration_);
    RCLCPP_INFO(get_logger(), "Staged departure: start duty=%.3f (0=disabled), then rise=%.3f/s",
      longitudinal_start_duty_, duty_rise_rate_per_sec_);
    RCLCPP_INFO(get_logger(),
      "Traffic stop=%s | bumper offset=%.2fm, margin=%.2fm, accepted range=0..%.2fm, planning decel=%.2fm/s2, "
      "response=%.2fs, brake cap=%.2fA (independent of normal electrical_brake_enabled)",
      traffic_stop_enabled_ ? "ON" : "OFF", traffic_front_offset_, traffic_margin_,
      traffic_stop_range_max_m_, planning_deceleration_mps2_, traffic_response_time_, brake_maximum_current_amps_);
    RCLCPP_INFO(get_logger(),
      "Auto control C++ ready: mode=%s, lane=%s, speed=%.2f..%.2fm/s, "
      "duty=%.3f..%.3f, auto_brake=%s/%.1fA, control=on_lane_result, watchdog=%.1fHz",
      control_mode_.c_str(), lane_result_topic_.c_str(), minimum_speed_mps_, maximum_speed_mps_,
      minimum_duty_, maximum_duty_, electrical_brake_enabled_ ? "on" : "off",
      brake_maximum_current_amps_, control_rate_hz_);
  }

  bool operator_stopped_{false};
  std::unique_ptr<bev_handoff::RuntimeMonitor> monitor_;
  std::map<std::string,rclcpp::Parameter> staged_parameters_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr runtime_parameter_callback_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;


private:
  static const std::vector<std::string>& runtime_parameter_names() {
    static const std::vector<std::string> names={
      "brake_current_fall_amps_per_sec", "brake_current_gain_amps_per_mps", "brake_current_rise_amps_per_sec",
      "brake_entry_speed_error_mps", "brake_exit_speed_error_mps", "brake_maximum_current_amps",
      "brake_minimum_current_amps", "brake_minimum_vehicle_speed_mps", "corner_brake_deceleration_margin_mps2",
      "corner_brake_entry_margin_mps", "corner_brake_exit_margin_mps", "corner_coast_entry_margin_mps",
      "corner_coast_probe_sec", "corner_confirm_frames", "corner_exit_angle_deg",
      "corner_exit_confirm_sec", "corner_exit_curvature_per_m", "corner_planning_acceleration_mps2",
      "corner_planning_response_sec", "corner_release_frames", "corner_speed_band_width_mps",
      "curvature_percentile", "curvature_speed_control_enabled", "driving_log_rate_hz",
      "duty_fall_rate_per_sec", "duty_rise_rate_per_sec", "electrical_brake_enabled",
      "erpm_timeout_sec", "longitudinal_brake_deceleration_gain", "longitudinal_brake_release_response_sec",
      "longitudinal_brake_speed_gain", "longitudinal_deceleration_filter_sec", "longitudinal_feedforward_duty_per_mps",
      "longitudinal_feedforward_fade_speed_mps", "longitudinal_feedforward_offset_duty", "longitudinal_pid_enabled",
      "longitudinal_pid_integral_limit", "longitudinal_pid_kd", "longitudinal_pid_ki",
      "longitudinal_pid_kp", "longitudinal_planning_deceleration_mps2", "longitudinal_recovery_duty_rise_per_sec",
      "longitudinal_recovery_start_ramp_sec", "longitudinal_staged_control_enabled", "longitudinal_start_duty",
      "maximum_duty", "maximum_lateral_acceleration_mps2", "maximum_speed_mps",
      "maximum_steering_angle_deg", "minimum_duty", "obstacle_brake_deceleration_margin_mps2",
      "obstacle_brake_entry_margin_mps", "obstacle_brake_exit_margin_mps", "obstacle_coast_probe_sec",
      "obstacle_planning_acceleration_mps2", "obstacle_planning_deceleration_mps2", "path_capture_maximum_age_sec",
      "path_geometry_window_m", "path_hold_timeout_sec", "path_maximum_gap_m",
      "path_maximum_x_m", "path_minimum_points", "path_minimum_span_m",
      "path_minimum_x_m", "path_timeout_sec", "speed_filter_time_constant_sec",
      "speed_pid_integral_limit", "speed_pid_kd", "speed_pid_ki",
      "speed_pid_kp", "stanley_corner_heading_threshold_deg", "stanley_corner_opposing_correction_ratio",
      "stanley_gain", "stanley_heading_lookahead_m", "stanley_softening_speed_mps",
      "steering_current_weight", "steering_rate_limit_deg_per_sec", "stop_steering_hold_enabled",
      "traffic_brake_amps_per_mps2", "traffic_brake_deceleration_hysteresis_mps2", "traffic_brake_response_time_sec",
      "traffic_brake_speed_hysteresis_mps", "traffic_brake_urgent_rise_amps_per_sec", "traffic_coast_probe_sec",
      "traffic_front_axle_to_bumper_m", "traffic_hold_current_amps", "traffic_pass_if_unstoppable_enabled",
      "traffic_pass_overshoot_m", "traffic_reapproach_speed_mps", "traffic_state_timeout_sec",
      "traffic_stop_enabled", "traffic_stop_line_confirm_frames", "traffic_stop_line_current_weight",
      "traffic_stop_line_match_tolerance_m", "traffic_stop_line_timeout_sec", "traffic_stop_margin_m",
      "traffic_stop_position_tolerance_m", "traffic_stop_range_max_m", "traffic_terminal_tracking_distance_m",
      "traffic_terminal_tracking_timeout_sec"};
    return names;
  }
  void install_runtime_controls() {
    declare_parameter<std::vector<std::string>>("runtime_editable_parameters",runtime_parameter_names());
    runtime_parameter_callback_=add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult reply;reply.successful=false;
        static const std::set<std::string> editable(runtime_parameter_names().begin(),runtime_parameter_names().end());
        if(parameters.empty()){reply.successful=true;return reply;}
        staged_parameters_.clear();
        for(const auto& p:parameters) {
          if(!editable.count(p.get_name())) {reply.reason="Startup-only or unsupported live parameter: "+p.get_name();staged_parameters_.clear();return reply;}
          if(p.get_type()!=get_parameter(p.get_name()).get_type()) {reply.reason="Parameter type mismatch";staged_parameters_.clear();return reply;}
          if(p.get_type()==rclcpp::ParameterType::PARAMETER_DOUBLE && !std::isfinite(p.as_double())) {reply.reason="Finite number required";staged_parameters_.clear();return reply;}
          staged_parameters_.emplace(p.get_name(),p);
        }
        const bool previous_enabled=enabled_;
        const double previous_speed=maximum_speed_mps_;
        staged_parameters_.emplace("enabled",rclcpp::Parameter("enabled",enabled_));
        staged_parameters_.emplace("maximum_speed_mps",rclcpp::Parameter("maximum_speed_mps",maximum_speed_mps_));
        try {
          declare_and_read_parameters();validate_parameters();
          const double ceiling=runtime_speed_limit_->ceiling();
          if(maximum_speed_mps_>ceiling)throw std::invalid_argument("maximum_speed_mps exceeds startup ceiling");
          // Default mutually-exclusive callback group serializes this transaction
          // with lane/telemetry/control callbacks.
          auto next_speed_pid=std::make_unique<SpeedPid>(speed_pid_kp_,speed_pid_ki_,speed_pid_kd_,
            speed_pid_integral_limit_,minimum_duty_,maximum_duty_);
          auto next_longitudinal_pid=std::make_unique<SpeedPid>(longitudinal_kp_,longitudinal_ki_,
            longitudinal_kd_,longitudinal_integral_limit_,-1.,1.);
          auto next_brake_profile=std::make_unique<AutomaticBrakeProfile>(BrakeConfig{
            brake_entry_speed_error_mps_,brake_exit_speed_error_mps_,brake_minimum_vehicle_speed_mps_,
            brake_minimum_current_amps_,brake_maximum_current_amps_,brake_current_gain_amps_per_mps_,
            brake_current_rise_amps_per_sec_,brake_current_fall_amps_per_sec_});
          speed_pid_=std::move(next_speed_pid);
          longitudinal_pid_=std::move(next_longitudinal_pid);
          brake_profile_=std::move(next_brake_profile);
        }catch(const std::exception& e) {
          staged_parameters_.clear();declare_and_read_parameters();
          enabled_=previous_enabled;maximum_speed_mps_=previous_speed;
          reply.reason=e.what();return reply;
        }
        runtime_speed_limit_->request(maximum_speed_mps_);
        staged_parameters_.clear();publish_speed_limit_state();
        reply.successful=true;return reply;
      });

    stop_service_=create_service<std_srvs::srv::Trigger>("~/stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        operator_stopped_=true;start_authorization_.revoke("Operator STOP");
        publish_guard_stop("operator_stop");
        response->success=true;response->message="STOPPED: press START to drive again";
        publish_startup_state();
      });
  }
  void publish_speed_limit_state()
  {
    line_detactor::msg::SpeedLimitState state;
    state.minimum_speed_mps = runtime_speed_limit_->minimum();
    state.ceiling_speed_mps = runtime_speed_limit_->ceiling();
    state.applied_speed_mps = runtime_speed_limit_->applied();
    state.revision = runtime_speed_limit_->revision();
    speed_limit_state_pub_->publish(state);
  }

  template<typename T>
  T parameter(const std::string & name, const T & default_value)
  {
    const auto pending=staged_parameters_.find(name);
    if(pending!=staged_parameters_.end())return pending->second.get_value<T>();
    if(has_parameter(name))return get_parameter(name).get_value<T>();
    return declare_parameter<T>(name, default_value);
  }

  template<typename T>
  T startup_parameter(const std::string& name,T value,const rcl_interfaces::msg::ParameterDescriptor& descriptor) {
    if(has_parameter(name))return get_parameter(name).get_value<T>();
    return declare_parameter<T>(name,value,descriptor);
  }

  void declare_and_read_parameters()
  {
    rcl_interfaces::msg::ParameterDescriptor avoidance_descriptor;
    avoidance_descriptor.read_only=true;
    avoidance_descriptor.description="Startup-only mode; deformation-only planning never gates centerline departure";
    avoidance_control_enabled_=startup_parameter<bool>("avoidance_control_enabled",false,avoidance_descriptor);
    avoidance_deformation_only_=startup_parameter<bool>("avoidance_deformation_only",true,avoidance_descriptor);
    obstacle_slowdown_enabled_=startup_parameter<bool>("obstacle_slowdown_enabled",true,avoidance_descriptor);
    obstacle_slowdown_speed_mps_=startup_parameter<double>("obstacle_slowdown_speed_mps",1.2,avoidance_descriptor);
    obstacle_slowdown_clear_sec_=startup_parameter<double>("obstacle_slowdown_clear_sec",1.0,avoidance_descriptor);
    obstacle_planning_deceleration_ = parameter("obstacle_planning_deceleration_mps2", 0.35);
    obstacle_planning_acceleration_ = parameter("obstacle_planning_acceleration_mps2", 0.35);
    obstacle_coast_probe_sec_ = parameter("obstacle_coast_probe_sec", 0.20);
    obstacle_brake_decel_margin_ = parameter("obstacle_brake_deceleration_margin_mps2", 0.10);
    obstacle_brake_entry_margin_ = parameter("obstacle_brake_entry_margin_mps", 0.10);
    obstacle_brake_exit_margin_ = parameter("obstacle_brake_exit_margin_mps", 0.03);
    brake_release_response_sec_ = parameter("longitudinal_brake_release_response_sec", 0.15);
    turn_exit_curvature_ = parameter("corner_exit_curvature_per_m", 0.25);
    turn_exit_angle_rad_ = radians(parameter("corner_exit_angle_deg", 8.0));
    turn_exit_confirm_sec_ = parameter("corner_exit_confirm_sec", 0.20);
    avoidance_max_age_=startup_parameter<double>("avoidance_max_age_sec",.20,avoidance_descriptor);
    // Compatibility with external YAMLs; never affects speed or validation.
    auto legacy_speed_descriptor=avoidance_descriptor;
    legacy_speed_descriptor.description="Deprecated and ignored; use maximum_speed_mps";
    startup_parameter<double>("avoidance_speed_cap_mps",.4,legacy_speed_descriptor);
    avoidance_brake_current_=startup_parameter<double>("avoidance_brake_current_amps",2.5,avoidance_descriptor);
    avoidance_wheelbase_=startup_parameter<double>("avoidance_wheelbase_m",.33,avoidance_descriptor);
    driving_log_enabled_ = parameter("driving_log_enabled", false);
    driving_log_rate_hz_ = parameter("driving_log_rate_hz", 20.0);
    driving_log_directory_ = parameter<std::string>("driving_log_directory", "driving_logs");
    stop_steering_hold_enabled_ = parameter("stop_steering_hold_enabled", true);
    traffic_stop_enabled_ = parameter("traffic_stop_enabled", false);
    traffic_state_topic_ = parameter<std::string>("traffic_state_topic", "/traffic_light/state");
    traffic_state_timeout_ = parameter("traffic_state_timeout_sec", 0.30);
    traffic_line_timeout_ = parameter("traffic_stop_line_timeout_sec", 0.50);
    traffic_margin_ = parameter("traffic_stop_margin_m", 0.15);
    traffic_stop_range_max_m_ = parameter("traffic_stop_range_max_m", 0.30);
    traffic_front_offset_ = parameter("traffic_front_axle_to_bumper_m", 0.10);
    planning_deceleration_mps2_ = parameter("longitudinal_planning_deceleration_mps2", 0.50);
    // Accepted for older parameter files; the shared planning value is authoritative.
    parameter("traffic_stop_deceleration_mps2", 0.60);
    parameter("corner_planning_deceleration_mps2", 0.35);
    traffic_response_time_ = parameter("traffic_brake_response_time_sec", 0.15);
    traffic_pass_enabled_ = parameter("traffic_pass_if_unstoppable_enabled", true);
    traffic_pass_overshoot_m_ = parameter("traffic_pass_overshoot_m", 0.60);
    traffic_brake_gain_ = parameter("traffic_brake_amps_per_mps2", 2.0);
    // Accepted for older parameter files; brake_maximum_current_amps is authoritative.
    parameter("traffic_brake_max_current_amps", 2.5);
    traffic_hold_current_ = parameter("traffic_hold_current_amps", 0.7);
    longitudinal_pid_enabled_ = parameter("longitudinal_pid_enabled", true);
    staged_control_enabled_ = parameter("longitudinal_staged_control_enabled", true);
    longitudinal_start_duty_ = parameter("longitudinal_start_duty", 0.05);
    drive_ff_offset_ = parameter("longitudinal_feedforward_offset_duty", 0.015);
    drive_ff_slope_ = parameter("longitudinal_feedforward_duty_per_mps", 0.055);
    drive_ff_fade_speed_ = parameter("longitudinal_feedforward_fade_speed_mps", 0.10);
    recovery_duty_rise_ = parameter("longitudinal_recovery_duty_rise_per_sec", 0.15);
    recovery_start_ramp_sec_ = parameter("longitudinal_recovery_start_ramp_sec", 0.20);
    traffic_brake_urgent_rise_ = parameter("traffic_brake_urgent_rise_amps_per_sec", 6.0);
    deceleration_filter_sec_ = parameter("longitudinal_deceleration_filter_sec", 0.20);
    traffic_coast_probe_sec_ = parameter("traffic_coast_probe_sec", 0.15);
    traffic_brake_decel_hysteresis_ = parameter("traffic_brake_deceleration_hysteresis_mps2", 0.10);
    traffic_brake_speed_hysteresis_ = parameter("traffic_brake_speed_hysteresis_mps", 0.05);
    longitudinal_brake_decel_gain_ = parameter("longitudinal_brake_deceleration_gain", 0.50);
    parameter("traffic_brake_deceleration_gain", 0.50);
    traffic_terminal_distance_ = parameter("traffic_terminal_tracking_distance_m", 0.10);
    traffic_terminal_timeout_ = parameter("traffic_terminal_tracking_timeout_sec", 1.50);
    curvature_speed_control_enabled_ = parameter("curvature_speed_control_enabled", false);
    corner_planning_acceleration_mps2_ = parameter("corner_planning_acceleration_mps2", 0.70);
    corner_planning_response_sec_ = parameter("corner_planning_response_sec", 0.20);
    corner_speed_band_width_mps_ = parameter("corner_speed_band_width_mps", 0.15);
    corner_coast_entry_margin_mps_ = parameter("corner_coast_entry_margin_mps", 0.03);
    corner_brake_entry_margin_mps_ = parameter("corner_brake_entry_margin_mps", 0.10);
    corner_brake_exit_margin_mps_ = parameter("corner_brake_exit_margin_mps", 0.05);
    corner_brake_decel_margin_mps2_ = parameter(
      "corner_brake_deceleration_margin_mps2", 0.10);
    corner_coast_probe_sec_ = parameter("corner_coast_probe_sec", 0.20);
    corner_confirm_frames_ = parameter("corner_confirm_frames", 2);
    corner_release_frames_ = parameter("corner_release_frames", 3);
    // Compatibility only. The spatial exit profile now replaces a fixed hold distance.
    parameter("corner_exit_hold_distance_m", 0.33);
    longitudinal_brake_speed_gain_ = parameter("longitudinal_brake_speed_gain", 1.0);
    longitudinal_kp_ = parameter("longitudinal_pid_kp", 1.0);
    longitudinal_ki_ = parameter("longitudinal_pid_ki", 0.5);
    longitudinal_kd_ = parameter("longitudinal_pid_kd", 0.0);
    longitudinal_integral_limit_ = parameter("longitudinal_pid_integral_limit", 2.0);
    traffic_line_gate_ = parameter("traffic_stop_line_match_tolerance_m", 0.30);
    traffic_line_weight_ = parameter("traffic_stop_line_current_weight", 0.35);
    traffic_line_confirm_frames_ = parameter("traffic_stop_line_confirm_frames", 3);
    traffic_arrival_tolerance_ = parameter("traffic_stop_position_tolerance_m", 0.03);
    traffic_reapproach_speed_ = parameter("traffic_reapproach_speed_mps", 0.20);
    enabled_ = parameter("enabled", true);
    control_mode_ = parameter<std::string>("control_mode", "drive");
    enable_topic_ = parameter<std::string>("enable_topic", "/auto/enabled");
    lane_result_topic_ = parameter<std::string>("lane_result_topic", "/line_detactor/result");
    lane_result_frame_id_ = parameter<std::string>("lane_result_frame_id", "front_axle_bev");
    path_maximum_gap_m_ = parameter("path_maximum_gap_m", 0.15);
    measured_erpm_topic_ = parameter<std::string>("measured_erpm_topic", "/vesc/measured_erpm");
    dynamics_duty_topic_ = parameter<std::string>(
      "dynamics_duty_topic", "/vehicle/dynamics/duty_cycle");
    dynamics_motor_current_topic_ = parameter<std::string>(
      "dynamics_motor_current_topic", "/vehicle/dynamics/motor_current_a");
    connection_status_topic_ = parameter<std::string>("connection_status_topic", "/vesc/connected");
    duty_topic_ = parameter<std::string>("duty_topic", "/vesc/duty");
    brake_current_topic_ = parameter<std::string>("brake_current_topic", "/vesc/brake_current");
    servo_position_topic_ = parameter<std::string>("servo_position_topic", "/vesc/servo_position");
    command_duty_topic_ = parameter<std::string>("command_duty_topic", "/auto/current_duty");
    command_brake_current_topic_ = parameter<std::string>(
      "command_brake_current_topic", "/auto/current_brake_current");
    target_speed_topic_ = parameter<std::string>("target_speed_topic", "/auto/target_speed");
    current_speed_topic_ = parameter<std::string>("current_speed_topic", "/auto/current_speed");
    corner_state_topic_ = parameter<std::string>("corner_state_topic", "/auto/corner_state");
    curvature_topic_ = parameter<std::string>("curvature_topic", "/auto/path_curvature");
    steering_angle_topic_ = parameter<std::string>("steering_angle_topic", "/auto/steering_angle_rad");
    cross_track_error_topic_ = parameter<std::string>("cross_track_error_topic", "/auto/cross_track_error_m");
    heading_error_topic_ = parameter<std::string>("heading_error_topic", "/auto/heading_error_rad");
    raw_steering_angle_topic_ = parameter<std::string>(
      "raw_steering_angle_topic", "/auto/raw_steering_angle_rad");
    command_servo_position_topic_ = parameter<std::string>(
      "command_servo_position_topic", "/auto/current_servo_position");
    input_to_control_decision_topic_ = parameter<std::string>(
      "input_to_control_decision_topic", "/auto/detector_input_to_control_decision_ms");

    control_rate_hz_ = parameter("control_rate_hz", 80.0);
    status_log_rate_hz_ = parameter("status_log_rate_hz", 2.0);
    path_timeout_sec_ = parameter("path_timeout_sec", 0.15);
    path_hold_timeout_sec_ = parameter("path_hold_timeout_sec", 0.08);
    path_capture_maximum_age_sec_ = parameter("path_capture_maximum_age_sec", 0.20);
    erpm_timeout_sec_ = parameter("erpm_timeout_sec", 0.15);
    bev_x_max_m_ = parameter("bev_x_max_m", 3.0);
    bev_y_max_m_ = parameter("bev_y_max_m", 0.60);
    bev_meter_per_pixel_ = parameter("bev_meter_per_pixel", 0.01);
    path_minimum_x_m_ = parameter("path_minimum_x_m", 0.05);
    path_maximum_x_m_ = parameter("path_maximum_x_m", 2.20);
    path_minimum_points_ = parameter("path_minimum_points", 8);
    path_minimum_span_m_ = parameter("path_minimum_span_m", 0.12);
    path_geometry_window_m_ = parameter("path_geometry_window_m", 0.14);
    stanley_gain_ = parameter("stanley_gain", 1.40);
    stanley_softening_speed_mps_ = parameter("stanley_softening_speed_mps", 0.40);
    stanley_heading_lookahead_m_ = parameter("stanley_heading_lookahead_m", 0.15);
    stanley_corner_heading_threshold_rad_ = radians(parameter(
      "stanley_corner_heading_threshold_deg", 4.0));
    stanley_corner_opposing_correction_ratio_ = parameter(
      "stanley_corner_opposing_correction_ratio", 0.45);
    maximum_steering_angle_rad_ = radians(parameter("maximum_steering_angle_deg", 30.0));
    steering_current_weight_ = parameter("steering_current_weight", 0.47);
    steering_rate_limit_rad_per_sec_ = radians(parameter("steering_rate_limit_deg_per_sec", 240.0));
    servo_left_ = parameter("servo_left", 1.0);
    servo_center_ = parameter("servo_center", 0.46);
    servo_right_ = parameter("servo_right", 0.0);
    steering_servo_inverted_ = parameter("steering_servo_inverted", true);
    minimum_speed_mps_ = parameter("minimum_speed_mps", 0.80);
    maximum_speed_mps_ = parameter("maximum_speed_mps", 1.8);
    maximum_lateral_acceleration_mps2_ = parameter("maximum_lateral_acceleration_mps2", 0.6);
    // Legacy diagnostic window settings no longer limit curvature reporting.
    parameter("curvature_lookahead_minimum_x_m", 0.50);
    parameter("curvature_lookahead_maximum_x_m", 1.60);
    curvature_percentile_ = parameter("curvature_percentile", 90.0);
    minimum_duty_ = parameter("minimum_duty", 0.070);
    maximum_duty_ = parameter("maximum_duty", 0.090);
    duty_rise_rate_per_sec_ = parameter("duty_rise_rate_per_sec", 0.04);
    duty_fall_rate_per_sec_ = parameter("duty_fall_rate_per_sec", 0.08);
    speed_pid_kp_ = parameter("speed_pid_kp", 0.012);
    speed_pid_ki_ = parameter("speed_pid_ki", 0.004);
    speed_pid_kd_ = parameter("speed_pid_kd", 0.0);
    speed_pid_integral_limit_ = parameter("speed_pid_integral_limit", 1.0);
    speed_filter_time_constant_sec_ = parameter("speed_filter_time_constant_sec", 0.05);
    electrical_brake_enabled_ = parameter("electrical_brake_enabled", true);
    brake_entry_speed_error_mps_ = parameter("brake_entry_speed_error_mps", 0.10);
    brake_exit_speed_error_mps_ = parameter("brake_exit_speed_error_mps", 0.03);
    brake_minimum_vehicle_speed_mps_ = parameter("brake_minimum_vehicle_speed_mps", 0.20);
    brake_minimum_current_amps_ = parameter("brake_minimum_current_amps", 0.5);
    brake_maximum_current_amps_ = parameter("brake_maximum_current_amps", 2.5);
    brake_current_gain_amps_per_mps_ = parameter("brake_current_gain_amps_per_mps", 6.0);
    brake_current_rise_amps_per_sec_ = parameter("brake_current_rise_amps_per_sec", 6.0);
    brake_current_fall_amps_per_sec_ = parameter("brake_current_fall_amps_per_sec", 16.0);
    wheel_diameter_m_ = parameter("wheel_diameter_m", 0.1095);
    motor_pole_pairs_ = parameter("motor_pole_pairs", 2);
    motor_pinion_teeth_ = parameter("motor_pinion_teeth", 13);
    spur_gear_teeth_ = parameter("spur_gear_teeth", 54);
    differential_pinion_teeth_ = parameter("differential_pinion_teeth", 13);
    differential_ring_teeth_ = parameter("differential_ring_teeth", 37);
    erpm_direction_sign_ = parameter("erpm_direction_sign", 1.0);
    speed_scale_correction_ = parameter("speed_scale_correction", 1.0);
  }

  void validate_parameters() const
  {
    if (!std::isfinite(obstacle_slowdown_speed_mps_) || obstacle_slowdown_speed_mps_<=0 ||
      !std::isfinite(obstacle_slowdown_clear_sec_) || obstacle_slowdown_clear_sec_<=0) {
      throw std::invalid_argument("obstacle slowdown speed/clear time must be finite and positive");
    }
    if (!std::isfinite(avoidance_max_age_) || avoidance_max_age_<.05 || avoidance_max_age_>.20 ||
      !std::isfinite(avoidance_brake_current_) || avoidance_brake_current_<=0 || avoidance_brake_current_>10 ||
      !std::isfinite(avoidance_wheelbase_) || avoidance_wheelbase_<.1 || avoidance_wheelbase_>1) {
      throw std::invalid_argument("avoidance control: age 0.05..0.20s, brake (0,10]A, wheelbase 0.1..1m");
    }
    if (!std::isfinite(driving_log_rate_hz_) || driving_log_rate_hz_ <= 0.0 ||
      driving_log_rate_hz_ > 100.0 || driving_log_directory_.empty())
    {throw std::invalid_argument("driving log needs a directory and rate in (0, 100] Hz");}
    const auto finite_positive = [](std::initializer_list<double> values) {
        return std::all_of(values.begin(), values.end(), [](double value) {
            return std::isfinite(value) && value > 0.0;
          });
      };
    const auto finite = [](std::initializer_list<double> values) {
        return std::all_of(values.begin(), values.end(), [](double value) {
            return std::isfinite(value);
          });
      };
    if (!finite_positive({obstacle_planning_deceleration_, obstacle_planning_acceleration_,
        obstacle_brake_decel_margin_, obstacle_brake_entry_margin_, obstacle_brake_exit_margin_,
        brake_release_response_sec_, turn_exit_curvature_,
        turn_exit_angle_rad_, turn_exit_confirm_sec_}) ||
      obstacle_brake_exit_margin_ >= obstacle_brake_entry_margin_ ||
      !std::isfinite(obstacle_coast_probe_sec_) || obstacle_coast_probe_sec_ < 0.0)
    {throw std::invalid_argument("invalid gradual slowdown/recovery parameters");}
    if (!finite_positive({traffic_state_timeout_, traffic_line_timeout_, planning_deceleration_mps2_,
        traffic_brake_gain_, traffic_hold_current_, longitudinal_kp_, longitudinal_integral_limit_,
        traffic_line_gate_, traffic_line_weight_, traffic_reapproach_speed_}) ||
      !finite({traffic_margin_, traffic_front_offset_, traffic_response_time_,
        longitudinal_ki_, longitudinal_kd_, traffic_arrival_tolerance_}) ||
      traffic_margin_ < 0 || traffic_front_offset_ < 0 || traffic_response_time_ < 0 ||
      traffic_hold_current_ > brake_maximum_current_amps_ || traffic_state_topic_.empty() ||
      longitudinal_ki_ < 0 || longitudinal_kd_ < 0 || traffic_arrival_tolerance_ < 0 || traffic_line_weight_ > 1 ||
      traffic_line_confirm_frames_ < 1)
    {throw std::invalid_argument("invalid traffic stop parameters");}
    if (!std::isfinite(traffic_stop_range_max_m_) || traffic_stop_range_max_m_ <= 0.0 ||
      traffic_margin_ + traffic_arrival_tolerance_ > traffic_stop_range_max_m_)
    {throw std::invalid_argument(
        "traffic_stop_range_max_m must be positive and include stop margin plus position tolerance");}
    if (!finite_positive({traffic_pass_overshoot_m_})) {
      throw std::invalid_argument("traffic_pass_overshoot_m must be positive");
    }
    if (!finite_positive({drive_ff_fade_speed_, recovery_duty_rise_, recovery_start_ramp_sec_,
        traffic_brake_urgent_rise_, deceleration_filter_sec_,
        longitudinal_brake_speed_gain_,
        traffic_brake_decel_hysteresis_, traffic_brake_speed_hysteresis_, longitudinal_brake_decel_gain_,
        traffic_terminal_distance_, traffic_terminal_timeout_}) ||
      !finite({drive_ff_offset_, drive_ff_slope_, traffic_coast_probe_sec_}) ||
      drive_ff_offset_ < 0 || drive_ff_offset_ > maximum_duty_ || drive_ff_slope_ < 0 ||
      traffic_coast_probe_sec_ < 0)
    {throw std::invalid_argument("invalid staged longitudinal control parameters");}
    if (!std::isfinite(longitudinal_start_duty_) || longitudinal_start_duty_ < 0.0 ||
      longitudinal_start_duty_ > maximum_duty_)
    {throw std::invalid_argument("longitudinal_start_duty must be between 0 and maximum_duty");}
    if (!finite({path_hold_timeout_sec_}) || path_hold_timeout_sec_ < 0.0 ||
      path_hold_timeout_sec_ > path_timeout_sec_)
    {throw std::invalid_argument("path_hold_timeout_sec must be between 0 and path_timeout_sec");}
    if (control_mode_ != "drive" && control_mode_ != "steering_only" && control_mode_ != "monitor_only") {
      throw std::invalid_argument("control_mode must be drive, steering_only, or monitor_only");
    }
    if (lane_result_topic_.empty() || lane_result_frame_id_.empty()) {
      throw std::invalid_argument("lane result topic and frame ID must not be empty");
    }
    if (!finite_positive({
        control_rate_hz_, status_log_rate_hz_, path_timeout_sec_,
        path_capture_maximum_age_sec_, erpm_timeout_sec_, path_maximum_gap_m_,
        bev_x_max_m_, bev_y_max_m_, bev_meter_per_pixel_, path_minimum_span_m_,
        path_geometry_window_m_, maximum_steering_angle_rad_,
        steering_rate_limit_rad_per_sec_, minimum_speed_mps_,
        maximum_lateral_acceleration_mps2_, duty_rise_rate_per_sec_,
        duty_fall_rate_per_sec_, speed_filter_time_constant_sec_, wheel_diameter_m_,
        speed_scale_correction_}))
    {throw std::invalid_argument("positive auto-control parameter is not positive");}
    if (!finite_positive({corner_planning_acceleration_mps2_, corner_planning_response_sec_,
      corner_brake_entry_margin_mps_, corner_brake_exit_margin_mps_,
      corner_brake_decel_margin_mps2_}) ||
      !finite({corner_speed_band_width_mps_, corner_coast_entry_margin_mps_,
      corner_coast_probe_sec_}) || corner_speed_band_width_mps_ < 0.0 ||
      corner_coast_entry_margin_mps_ < 0.0 || corner_coast_probe_sec_ < 0.0 ||
      corner_brake_exit_margin_mps_ >= corner_brake_entry_margin_mps_ ||
      corner_confirm_frames_ < 1 || corner_release_frames_ < 1)
    {throw std::invalid_argument("invalid corner speed planning parameters");}
    if (path_minimum_points_ < 3 || path_minimum_x_m_ >= path_maximum_x_m_) {
      throw std::invalid_argument("invalid path acceptance parameters");
    }
    if (!(minimum_duty_ > 0.0 && minimum_duty_ <= maximum_duty_ && maximum_duty_ <= 1.0)) {
      throw std::invalid_argument("duty limits must satisfy 0 < minimum <= maximum <= 1");
    }
    if (!finite({maximum_speed_mps_, steering_current_weight_, curvature_percentile_,
        speed_pid_integral_limit_, stanley_corner_heading_threshold_rad_,
        stanley_corner_opposing_correction_ratio_}) || maximum_speed_mps_ < minimum_speed_mps_ ||
      steering_current_weight_ <= 0.0 || steering_current_weight_ > 1.0 ||
      curvature_percentile_ < 0.0 || curvature_percentile_ > 100.0 ||
      speed_pid_integral_limit_ < 0.0 || stanley_corner_heading_threshold_rad_ < 0.0 ||
      stanley_corner_opposing_correction_ratio_ < 0.0 ||
      stanley_corner_opposing_correction_ratio_ >= 1.0)
    {throw std::invalid_argument("invalid speed, steering weight, or curvature percentile");}
    if (!finite({brake_entry_speed_error_mps_, brake_exit_speed_error_mps_,
        brake_minimum_vehicle_speed_mps_, brake_minimum_current_amps_,
        brake_maximum_current_amps_, brake_current_gain_amps_per_mps_,
        brake_current_rise_amps_per_sec_, brake_current_fall_amps_per_sec_}) ||
      brake_entry_speed_error_mps_ <= 0.0 || brake_exit_speed_error_mps_ < 0.0 ||
      brake_exit_speed_error_mps_ >= brake_entry_speed_error_mps_ ||
      brake_minimum_vehicle_speed_mps_ < 0.0 || brake_minimum_current_amps_ < 0.0 ||
      brake_maximum_current_amps_ < brake_minimum_current_amps_ ||
      brake_maximum_current_amps_ <= 0.0 || brake_current_gain_amps_per_mps_ <= 0.0 ||
      brake_current_rise_amps_per_sec_ <= 0.0 || brake_current_fall_amps_per_sec_ <= 0.0)
    {throw std::invalid_argument("invalid automatic brake parameters");}
    if (std::min({motor_pole_pairs_, motor_pinion_teeth_, spur_gear_teeth_,
        differential_pinion_teeth_, differential_ring_teeth_}) <= 0)
    {throw std::invalid_argument("motor and gear parameters must be positive");}
    if (input_to_control_decision_topic_.empty()) {
      throw std::invalid_argument("input_to_control_decision_topic must not be empty");
    }
  }

  void on_lane_result(
    const line_detactor::msg::LaneResult::ConstSharedPtr message, const char * lane_source)
  {
    const auto received_ros = now();
    ++lane_result_count_;
    latest_lane_state_ = message->state;
    latest_lane_source_ = lane_source;
    // Remote preview stages 2 (BEV -> inference) and 3 (inference -> centerline result).
    preview_infer_meter_.add(std::chrono::nanoseconds(static_cast<std::int64_t>(
      message->detector_queue_nanoseconds + message->h2d_preprocess_nanoseconds +
      message->pure_inference_nanoseconds)));
    preview_post_meter_.add(std::chrono::nanoseconds(static_cast<std::int64_t>(
      message->label_export_nanoseconds + message->backend_postprocess_nanoseconds +
      message->lane_geometry_nanoseconds + message->result_message_build_nanoseconds)));
    latest_lane_sequence_ = message->detector_sequence;
    latest_stop_line_present_ = message->stop_line_present;
    latest_stop_line_raw_m_ = message->stop_line_distance_m;
    latest_stop_line_corrected_m_ = kUnavailable;
    lane_result_points_ = message->centerline_points.size();
    lane_capture_age_sec_.reset();
    auto previous_path = std::move(path_);
    path_.reset();
    bool may_hold_path = false;
    path_hold_active_ = false;
    last_path_received_ns_ = received_ros.nanoseconds();
    try {
      const auto capture_ns = stamp_nanoseconds(message->header.stamp);
      if (!capture_ns) {throw std::invalid_argument("missing source capture timestamp");}
      const double age = seconds(received_ros.nanoseconds() - *capture_ns);
      lane_capture_age_sec_ = age;
      if (age < -0.05 || age > path_capture_maximum_age_sec_) {
        throw std::invalid_argument("source capture timestamp is stale or in the future");
      }
      if (!lane_capture_order_.accept(*capture_ns, lane_source)) {
        throw std::invalid_argument("duplicate or out-of-order lane result");
      }
      if (traffic_stop_enabled_ && message->header.frame_id == lane_result_frame_id_) {
        advance_stop_distance(received_ros.nanoseconds());
        if (message->stop_line_present && std::isfinite(message->stop_line_distance_m) &&
          message->stop_line_distance_m >= 0.0 && message->stop_line_distance_m <= bev_x_max_m_ * 2.0)
        {
          const double distance = message->stop_line_distance_m - traffic_front_offset_ -
            traffic_margin_ - std::max(0.0, current_speed_mps_) * std::max(0.0, age);
          latest_stop_line_corrected_m_ = distance;
          observe_stop_distance(distance, received_ros.nanoseconds());
        } else {stop_candidate_count_ = 0;}
      }
      if (message->centerline_sample_limit_reached) {
        lane_result_status_ = "centerline_sample_limit";
      } else if (!message->centerline_valid) {
        lane_result_status_ = "no_centerline(state=" + std::to_string(message->state) + ")";
      } else {
        validate_and_build_path(*message);
      }
      may_hold_path = message->header.frame_id == lane_result_frame_id_;
      if (path_) {
        path_capture_ns_ = capture_ns;
        last_valid_path_received_ns_ = received_ros.nanoseconds();
        preview_valid_lane_meter_.add();
      }
    } catch (const std::exception & exception) {
      path_.reset();
      lane_result_status_ = exception.what();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Rejected ML centerline: %s", exception.what());
    }
    if ((!avoidance_control_enabled_ || avoidance_deformation_only_ || avoidance_follow_centerline_) && !path_ && may_hold_path && previous_path && last_valid_path_received_ns_ &&
      path_capture_ns_ && path_hold_timeout_sec_ > 0.0 &&
      received_ros.nanoseconds() >= *last_valid_path_received_ns_ &&
      seconds(received_ros.nanoseconds() - *last_valid_path_received_ns_) <= path_hold_timeout_sec_ &&
      received_ros.nanoseconds() >= *path_capture_ns_ &&
      seconds(received_ros.nanoseconds() - *path_capture_ns_) <= path_capture_maximum_age_sec_)
    {
      path_ = std::move(previous_path);
      path_hold_active_ = true;
      lane_result_status_ = "holding_previous_path:" + lane_result_status_;
    }
    std::int64_t decision_finished_ros_ns = 0;
    const bool servo_calculated = update_control_from_path(decision_finished_ros_ns);
    if (servo_calculated) {
      const auto detector_input_ns = stamp_nanoseconds(message->detector_input_received_stamp);
      if (detector_input_ns && decision_finished_ros_ns >= *detector_input_ns) {
        const double total_milliseconds =
          static_cast<double>(decision_finished_ros_ns - *detector_input_ns) / 1.0e6;
        publish_float(input_to_control_decision_pub_, total_milliseconds);
      }
      const auto result_ready_ns = stamp_nanoseconds(message->detector_result_ready_stamp);
      std::chrono::nanoseconds decision_latency{-1};
      if (result_ready_ns && decision_finished_ros_ns >= *result_ready_ns) {
        publish_float(result_to_control_decision_pub_,
          static_cast<double>(decision_finished_ros_ns - *result_ready_ns) / 1.0e6);
        decision_latency=std::chrono::nanoseconds(decision_finished_ros_ns - *result_ready_ns);
      }
      preview_control_meter_.add(decision_latency);
    }
  }

  bool refresh_obstacle_slowdown()
  {
    if (!obstacle_slowdown_active_ || !obstacle_last_seen_at_) {return false;}
    if (std::chrono::duration<double>(std::chrono::steady_clock::now()-*obstacle_last_seen_at_).count()<
      obstacle_slowdown_clear_sec_) {return false;}
    obstacle_slowdown_active_=false;
    RCLCPP_INFO(get_logger(),"Obstacle slowdown OFF: no new in-lane detection for %.2fs; plan gradual recovery after turn exit",
      obstacle_slowdown_clear_sec_);
    return true;
  }

  bool observe_obstacle_slowdown(const auto_control::msg::AvoidancePlan & message,std::int64_t received)
  {
    if (!obstacle_slowdown_enabled_) {return false;}
    const auto source=stamp_nanoseconds(message.header.stamp),depth=stamp_nanoseconds(message.depth_stamp);
    // Invalid/unavailable plans have no detection timestamp. Replayed depth
    // must not keep extending the last-detection timer.
    if (!source || !depth || message.header.frame_id!=lane_result_frame_id_ ||
      received<*source || received<*depth || seconds(received-*source)>avoidance_max_age_ ||
      seconds(received-*depth)>avoidance_max_age_ || std::abs(seconds(*source-*depth))>.10 ||
      (obstacle_last_depth_ns_ && *depth<=*obstacle_last_depth_ns_)) {return false;}
    obstacle_last_depth_ns_=depth;
    if (!message.lane_obstacle_detected) {return false;}
    obstacle_last_seen_at_=std::chrono::steady_clock::now();
    if (obstacle_slowdown_active_) {return false;}
    obstacle_slowdown_active_=true;
    RCLCPP_INFO(get_logger(),"Obstacle slowdown ON: gradually approach %.2fm/s, release after %.2fs without detection",
      std::min(maximum_speed_mps_,obstacle_slowdown_speed_mps_),obstacle_slowdown_clear_sec_);
    return true;
  }

  // A plan only updates the stored avoidance/slowdown state. The command is
  // recomputed by the next lane result (and the watchdog), so each plan does
  // not add a second full control + publish + log cycle. Guard stops stay
  // immediate.
  void on_avoidance_plan(const auto_control::msg::AvoidancePlan::ConstSharedPtr message)
  {
    const auto received=now().nanoseconds();
    observe_obstacle_slowdown(*message,received);
    if (!avoidance_control_enabled_) {return;}
    const auto steady=std::chrono::steady_clock::now();
    const auto source=stamp_nanoseconds(message->header.stamp),depth=stamp_nanoseconds(message->depth_stamp);
    auto reject=[&](const std::string & reason) {
      avoidance_path_.reset(); avoidance_follow_centerline_=false;
      avoidance_confirmations_=0;
      avoidance_status_=reason;
      // Deformation mode drops only the optional deformation. Normal
      // lane/traffic/vehicle control remains responsible for driving.
      if (!avoidance_deformation_only_) {publish_guard_stop("avoidance_"+reason);}
    };
    if (message->deformation_only!=avoidance_deformation_only_) {reject("planner_mode_mismatch"); return;}
    if (!message->control_ready || !message->valid) {reject(message->status.empty()?"unavailable":message->status); return;}
    if (!source || !depth || message->header.frame_id!=lane_result_frame_id_ ||
      received<*source || received<*depth || seconds(received-*source)>avoidance_max_age_ ||
      seconds(received-*depth)>avoidance_max_age_ || std::abs(seconds(*source-*depth))>.10) {reject("invalid_timestamp_or_frame"); return;}
    // A fresh depth can update the same lane. Duplicate/late pairs cannot
    // replace the accepted path, clear it, or extend its freshness lifetime.
    if(capture_pair_order(*source,*depth,avoidance_lane_ns_,avoidance_depth_ns_)!=CapturePairOrder::New) {return;}
    if (message->follow_centerline) {
      if (!message->points.empty()) {
        reject("invalid_centerline_decision"); return;
      }
      avoidance_path_.reset(); avoidance_follow_centerline_=true;
      avoidance_lane_ns_=source; avoidance_depth_ns_=depth; avoidance_received_at_=steady;
      avoidance_confirmations_=3; avoidance_status_="CENTERLINE";
      return;
    }
    if (!std::isfinite(message->max_curvature_per_m) || message->max_curvature_per_m<0 ||
      (!avoidance_deformation_only_ && message->max_curvature_per_m>std::tan(maximum_steering_angle_rad_)/avoidance_wheelbase_) ||
      message->points.size()<static_cast<std::size_t>(avoidance_deformation_only_?2:path_minimum_points_) || message->points.size()>160) {reject("invalid_limits"); return;}
    OrderedPath candidate; candidate.geometry_window_m=path_geometry_window_m_;
    double arc=0;
    for (const auto & p:message->points) {
      if (!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||std::abs(p.z)>1e-6 ||
        (!avoidance_deformation_only_ && (p.x<0 || p.x>bev_x_max_m_ || std::abs(p.y)>bev_y_max_m_))) {reject("invalid_point"); return;}
      if (!candidate.points.empty()) {
        const auto & q=candidate.points.back();
        const double gap=std::hypot(p.x-q.x,p.y-q.y);
        if (avoidance_deformation_only_ && gap<1e-5) {continue;}
        if (gap<1e-5 || (!avoidance_deformation_only_ && gap>path_maximum_gap_m_)) {reject("invalid_path_order"); return;}
        arc+=gap;
      }
      candidate.points.push_back({p.x,p.y}); candidate.arc_m.push_back(arc);
    }
    if (candidate.points.size()<2 || arc<(avoidance_deformation_only_?1e-5:path_minimum_span_m_)) {
      reject("insufficient_path_span"); return;
    }
    // Use the same geometry window as normal central-path steering, rather
    // than amplifying pixel-scale wiggles with an adjacent-three-point test.
    const double limit=std::tan(maximum_steering_angle_rad_)/avoidance_wheelbase_;
    for (const double s:candidate.arc_m) {
      const double k=std::abs(candidate.curvature(s));
      if (!std::isfinite(k) || (!avoidance_deformation_only_ && k>limit+1e-3)) {reject("curvature_limit"); return;}
    }
    const bool contiguous=(avoidance_path_ || avoidance_follow_centerline_) && avoidance_lane_ns_ &&
      seconds(*source-*avoidance_lane_ns_)<=avoidance_max_age_ &&
      std::chrono::duration<double>(steady-avoidance_received_at_).count()<=avoidance_max_age_;
    const bool new_depth=!avoidance_depth_ns_ || *depth>*avoidance_depth_ns_;
    avoidance_confirmations_=avoidance_deformation_only_?3:(contiguous?std::min(3,avoidance_confirmations_+(new_depth?1:0)):1);
    avoidance_lane_ns_=source; avoidance_depth_ns_=depth; avoidance_received_at_=steady;
    avoidance_path_=std::move(candidate); avoidance_follow_centerline_=false;
    avoidance_status_=avoidance_confirmations_<3?"confirming":message->status;
  }

  void validate_and_build_path(const line_detactor::msg::LaneResult & message)
  {
    if (message.state != line_detactor::msg::LaneResult::LEFT_ONLY &&
      message.state != line_detactor::msg::LaneResult::RIGHT_ONLY &&
      message.state != line_detactor::msg::LaneResult::BOTH)
    {throw std::invalid_argument("centerline has no observed lane support");}
    if (message.header.frame_id != lane_result_frame_id_) {
      throw std::invalid_argument("unexpected lane result frame: " + message.header.frame_id);
    }
    const int width = static_cast<int>(message.source_width);
    const int height = static_cast<int>(message.source_height);
    const int padding_left = static_cast<int>(message.padding_left);
    const int padding_right = static_cast<int>(message.padding_right);
    if (width <= 0 || height <= 0) {throw std::invalid_argument("invalid source geometry");}
    const double scale_x = message.centerline_bev_width_m / width;
    const double scale_y = message.centerline_bev_height_m / height;
    const auto close = [](double a, double b) {
        return std::abs(a - b) <= 1.0e-4 * std::max(std::abs(a), std::abs(b));};
    if (!std::isfinite(scale_x) || !std::isfinite(scale_y) || scale_x <= 0.0 || scale_y <= 0.0 ||
      !close(scale_x, bev_meter_per_pixel_) || !close(scale_y, bev_meter_per_pixel_) ||
      !close(width * scale_x, 2.0 * bev_y_max_m_) || !close(height * scale_y, bev_x_max_m_))
    {throw std::invalid_argument("lane result scale disagrees with configured symmetric BEV");}
    const auto count = message.centerline_points.size();
    if (count < 2U || count > 10000U || message.centerline_support.size() != count) {
      throw std::invalid_argument("invalid centerline point/support lengths");
    }
    std::vector<Point> metric_points;
    metric_points.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
      const auto & point = message.centerline_points[index];
      const auto support = message.centerline_support[index];
      if (support < line_detactor::msg::LaneResult::CENTER_SINGLE ||
        support > line_detactor::msg::LaneResult::CENTER_OUTER)
      {throw std::invalid_argument("unknown centerline support value");}
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
        std::abs(point.z) > 1.0e-6 || point.x < 0.0F ||
        point.x >= width + padding_left + padding_right || point.y < 0.0F || point.y >= height)
      {throw std::invalid_argument("centerline points are outside their result canvas");}
      metric_points.push_back({
        bev_x_max_m_ - (point.y + 0.5) * scale_y,
        bev_y_max_m_ - (point.x - padding_left + 0.5) * scale_x});
    }
    path_ = build_ordered_path(
      metric_points, path_minimum_points_, path_minimum_span_m_, path_minimum_x_m_,
      path_maximum_x_m_, path_maximum_gap_m_, path_geometry_window_m_);
    lane_result_status_ = path_ ? "accepted" : "insufficient_contiguous_path";
  }

  void clear_stop_line_track()
  {
    terminal_tracking_active_ = false;
    stop_remaining_.reset();
    stop_line_seen_ns_.reset();
    stop_candidate_count_ = 0;
    stop_candidate_ns_ = 0;
    last_observed_remaining_ = 0.0;
  }

  bool stop_line_fresh(std::int64_t now_ns) const
  {
    return stop_remaining_ && stop_line_seen_ns_ && now_ns >= *stop_line_seen_ns_ &&
      seconds(now_ns - *stop_line_seen_ns_) <= traffic_line_timeout_;
  }

  void advance_stop_distance(std::int64_t now_ns)
  {
    if (stop_distance_updated_ns_) {
      const double elapsed = seconds(now_ns - *stop_distance_updated_ns_);
      const bool speed_fresh = last_erpm_ns_ && now_ns >= *last_erpm_ns_ &&
        seconds(now_ns - *last_erpm_ns_) <= erpm_timeout_sec_;
      if (elapsed < 0 || elapsed > traffic_line_timeout_ || !speed_fresh) {
        clear_stop_line_track();
        // Without valid motion integration a previous pass cannot remain armed
        // indefinitely. Normal input guards stop control; recovery needs a new pair.
        if (traffic_pass_committed_) {
          traffic_pass_committed_ = traffic_red_ = traffic_red_from_green_ = false;
          traffic_red_seen_ns_.reset();
        }
      } else {
        const double travel = std::max(0.0, current_speed_mps_) * elapsed;
        if (stop_remaining_) {*stop_remaining_ -= travel;}
        if (stop_candidate_count_) {stop_candidate_distance_ -= travel;}
      }
    }
    stop_distance_updated_ns_ = now_ns;
    // Retire a committed pass only after the bumper has crossed its line.
    // A fresh RED/line pair is required for the next target.
    if (traffic_pass_committed_ && stop_remaining_ &&
      *stop_remaining_ + traffic_margin_ <= 0.0)
    {
      traffic_pass_committed_ = false;
      traffic_red_ = false;
      traffic_red_seen_ns_.reset();
      traffic_red_from_green_ = false;
      clear_stop_line_track();
    }
    // Uncommitted observations expire by LAST OBSERVATION age, not callback dt.
    // Never match the next intersection against an old, negative distance.
    // An active stop keeps its target identity and cannot silently switch lines.
    if (!traffic_stop_active_ && !traffic_pass_committed_ &&
      (!stop_line_fresh(now_ns) || (stop_remaining_ &&
      *stop_remaining_ + traffic_margin_ <= 0.0)))
    {
      // Keep a new candidate long enough to reach the confirmation count.
      if (stop_remaining_ || (stop_candidate_count_ > 0 &&
        (now_ns < stop_candidate_ns_ ||
        seconds(now_ns - stop_candidate_ns_) > traffic_line_timeout_)))
      {clear_stop_line_track();}
    }
  }

  void observe_stop_distance(double distance, std::int64_t now_ns)
  {
    // Distance continuity is a proxy for line identity, not semantic tracking.
    // Reject sudden jumps in EITHER direction. A matched observation can
    // correct the estimate upward too; a short noisy sample is not permanent.
    if (stop_remaining_ && std::abs(distance - *stop_remaining_) > traffic_line_gate_) {
      stop_candidate_count_ = 0;
      ++stop_line_rejections_;
      return;
    }
    const bool consecutive = stop_candidate_count_ > 0 &&
      now_ns > stop_candidate_ns_ && seconds(now_ns - stop_candidate_ns_) <= traffic_line_timeout_ &&
      std::abs(distance - stop_candidate_distance_) <= traffic_line_gate_;
    stop_candidate_count_ = consecutive ?
      stop_candidate_count_ + (stop_candidate_count_ < traffic_line_confirm_frames_ ? 1 : 0) : 1;
    stop_candidate_distance_ = distance;
    stop_candidate_ns_ = now_ns;
    if (stop_candidate_count_ < traffic_line_confirm_frames_) {return;}
    stop_remaining_ = stop_remaining_ ?
      *stop_remaining_ + traffic_line_weight_ * (distance - *stop_remaining_) : distance;
    stop_line_seen_ns_ = now_ns;
    last_observed_remaining_ = distance;
  }

  bool traffic_stop_requested() const
  {
    return traffic_stop_enabled_ && traffic_stop_active_;
  }

  const char * traffic_status() const
  {
    if (traffic_stop_requested()) {return traffic_phase_.c_str();}
    if (traffic_pass_committed_) {return "pass_committed";}
    if (traffic_red_) {return "waiting_for_red_line_pair";}
    return "inactive";
  }

  void update_traffic_decision(std::int64_t now_ns)
  {
    if (!traffic_stop_enabled_ || traffic_stop_active_ || traffic_pass_committed_) {return;}
    if (!traffic_red_ || !traffic_red_seen_ns_) {return;}
    if (now_ns < *traffic_red_seen_ns_ ||
      seconds(now_ns - *traffic_red_seen_ns_) > traffic_state_timeout_)
    {
      // A RED with no confirmed line is not a stop request for a later,
      // unrelated stop-line-only section. Signal loss AFTER activation differs.
      traffic_red_ = false;
      traffic_red_seen_ns_.reset();
      traffic_red_from_green_ = false;
      return;
    }
    if (!stop_line_fresh(now_ns) || *stop_remaining_ + traffic_margin_ <= 0.0) {return;}
    const double speed = std::max(0.0, current_speed_mps_);
    traffic_predicted_stop_distance_m_ = speed * traffic_response_time_ +
      speed * speed / (2.0 * effective_planning_deceleration(now_ns));
    // stop_remaining_ is to the buffered target; add the margin back to
    // compare predicted BUMPER position with the physical stop line.
    traffic_predicted_overshoot_m_ = traffic_predicted_stop_distance_m_ -
      (*stop_remaining_ + traffic_margin_);
    if (traffic_red_from_green_ && traffic_pass_enabled_ &&
      traffic_predicted_overshoot_m_ >= traffic_pass_overshoot_m_)
    {
      traffic_pass_committed_ = true;
      RCLCPP_WARN(get_logger(),
        "GREEN->RED pass committed: predicted stop=%.3fm, overshoot=%.3fm >= %.3fm",
        traffic_predicted_stop_distance_m_, traffic_predicted_overshoot_m_, traffic_pass_overshoot_m_);
      return;
    }
    traffic_stop_active_ = true;
    traffic_holding_ = traffic_reapproach_ = false;
    traffic_range_stop_committed_ = false;
    traffic_phase_ = "approach";
    longitudinal_pid_->reset();
    speed_pid_->reset();
    RCLCPP_WARN(get_logger(), "RED + confirmed stop line: approach latched, remaining=%.3fm",
      *stop_remaining_);
  }

  void on_traffic_state(const traffic_detection_test::msg::TrafficLightState::ConstSharedPtr message)
  {
    using Signal = traffic_detection_test::msg::TrafficLightState;
    const auto captured = stamp_nanoseconds(message->header.stamp);
    const auto now_ns = now().nanoseconds();
    latest_signal_state_ = message->state;
    latest_signal_score_ = message->detection_score;
    latest_signal_capture_ns_ = captured;
    latest_signal_accepted_ = false;
    if (!captured || *captured > now_ns ||
      seconds(now_ns - *captured) > traffic_state_timeout_ ||
      (traffic_capture_ns_ && *captured <= *traffic_capture_ns_)) {return;}
    traffic_capture_ns_ = captured;
    latest_signal_accepted_ = true;
    if (!traffic_stop_enabled_) {return;}
    if (message->state != Signal::RED && message->state != Signal::GREEN) {return;}
    traffic_signal_seen_ = true;
    if (message->state == Signal::RED) {
      if (!traffic_red_) {
        traffic_red_ = true;
        traffic_red_from_green_ = traffic_green_seen_;
        traffic_green_seen_ = false;
        traffic_predicted_stop_distance_m_ = traffic_predicted_overshoot_m_ = kUnavailable;
      }
      traffic_red_seen_ns_ = captured;
    } else if (message->state == Signal::GREEN) {
      const bool was_stopping = traffic_stop_active_;
      const bool had_target = traffic_stop_active_ || traffic_pass_committed_;
      traffic_green_seen_ = true;
      traffic_red_ = traffic_stop_active_ = traffic_pass_committed_ = false;
      traffic_holding_ = traffic_reapproach_ = traffic_red_from_green_ = false;
      traffic_range_stop_committed_ = false;
      traffic_red_seen_ns_.reset();
      if (had_target) {clear_stop_line_track();}
      if (was_stopping) {
        longitudinal_pid_->reset();
        speed_pid_->reset();
        RCLCPP_INFO(get_logger(), "Traffic GREEN: stop request released; normal drive gates still apply.");
      }
    }
    // UNKNOWN/stale messages cannot cancel a red stop. A new valid green can.
  }

  double effective_planning_deceleration(std::int64_t /* now_ns */) const
  {
    // This is the shared feasible service-deceleration profile for traffic
    // stops and corners. Duty-only coast deceleration is measured separately
    // and decides whether the electrical brake is needed to meet this profile.
    return planning_deceleration_mps2_;
  }

  double traffic_brake_delay(double current) const
  {
    if (!staged_control_enabled_) {return traffic_response_time_;}
    // Reference current is a planning estimate from existing gains, not an
    // identified current-to-deceleration model. Half the linear ramp accounts
    // for partial braking while current builds. Do not assume the urgent ramp
    // is available when planning the normal approach.
    const double reference = std::max(traffic_hold_current_, brake_maximum_current_amps_ *
      clamp(longitudinal_brake_decel_gain_ *
      effective_planning_deceleration(now().nanoseconds()), 0.0, 1.0));
    return traffic_response_time_ +
      0.5 * std::max(0.0, reference - current) / brake_current_rise_amps_per_sec_;
  }

  double traffic_distance_speed_limit(double remaining) const
  {
    // Fixed planning delay: current buildup must not raise the speed target.
    const double deceleration = effective_planning_deceleration(now().nanoseconds());
    const double at = deceleration * traffic_brake_delay(0.0);
    return std::sqrt(at * at + 2.0 * deceleration *
      std::max(0.0, remaining - traffic_arrival_tolerance_)) - at;
  }

  double traffic_speed_limit(std::int64_t now_ns)
  {
    advance_stop_distance(now_ns);
    terminal_tracking_active_ = staged_control_enabled_ && stop_remaining_ && stop_line_seen_ns_ &&
      last_observed_remaining_ >= -traffic_arrival_tolerance_ &&
      last_observed_remaining_ <= traffic_terminal_distance_ &&
      now_ns >= *stop_line_seen_ns_ &&
      seconds(now_ns - *stop_line_seen_ns_) <= traffic_terminal_timeout_ &&
      last_erpm_ns_ && now_ns >= *last_erpm_ns_ && seconds(now_ns - *last_erpm_ns_) <= erpm_timeout_sec_;
    if (traffic_holding_) {traffic_phase_ = "position_hold"; return 0.0;}
    // Once braking into the accepted bumper-to-line range has begun, neither
    // observation jitter nor an input outage may cause another forward request.
    // Keep this latch through guard stops; a valid GREEN releases it.
    if (traffic_range_stop_committed_) {
      if (std::abs(current_speed_mps_) < 0.05) {traffic_holding_ = true;}
      traffic_phase_ = traffic_holding_ ? "position_hold" : "range_braking";
      return 0.0;
    }
    if (!stop_line_fresh(now_ns) && !terminal_tracking_active_)
    {
      traffic_reapproach_ = true;
      traffic_phase_ = "temporary_stop_line_unavailable";
      return 0.0;
    }
    // stop_remaining_ is measured from the nominal buffered target. Add its
    // margin back to compare with the physical line from the FRONT BUMPER.
    // A negative distance is an overshoot and must still command braking.
    if (*stop_remaining_ + traffic_margin_ <= traffic_stop_range_max_m_) {
      traffic_range_stop_committed_ = true;
      traffic_holding_ = std::abs(current_speed_mps_) < 0.05;
      traffic_phase_ = traffic_holding_ ? "position_hold" : "range_braking";
      return 0.0;
    }
    const double limit = traffic_distance_speed_limit(*stop_remaining_);
    traffic_phase_ = traffic_reapproach_ ? "slow_reapproach" : "approach";
    return traffic_reapproach_ ? std::min(limit, traffic_reapproach_speed_) : limit;
  }

  std::string longitudinal_motor(double target, double dt, bool traffic_stop)
  {
    if (!staged_control_enabled_) {
      longitudinal_phase_ = "legacy_pid";
      return legacy_longitudinal_motor(target, dt, traffic_stop);
    }
    speed_pid_->reset();
    brake_profile_->reset();
    const auto now_ns = now().nanoseconds();
    const double speed = std::max(0.0, current_speed_mps_);
    const double overspeed = speed - target;
    const bool zero_target = target <= 0.0;
    if (zero_target != longitudinal_zero_target_) {longitudinal_pid_->reset();}
    longitudinal_zero_target_ = zero_target;
    const bool braking_allowed = traffic_stop || electrical_brake_enabled_ ||
      obstacle_profile_active_ || corner_braking_allowed_;
    const double brake_cap = brake_maximum_current_amps_;
    required_deceleration_mps2_ = kUnavailable;
    const bool approaching = traffic_stop && !zero_target && stop_remaining_;
    const double corner_speed_error = speed - corner_upper_speed_mps_;
    const bool obstacle_is_limiting = obstacle_profile_active_ && obstacle_planned_speed_ &&
      *obstacle_planned_speed_ < corner_nominal_speed_mps_;
    const bool approaching_corner = !traffic_stop && corner_profile_active_ &&
      corner_braking_allowed_ && !obstacle_is_limiting &&
      (corner_required_deceleration_mps2_ > 0.0 || corner_speed_error > 0.0 || service_brake_requested_);
    const bool approaching_obstacle = !traffic_stop && !approaching_corner && obstacle_profile_active_;
    latest_brake_delay_sec_ = traffic_stop ? traffic_brake_delay(command_brake_current_) :
      approaching_corner ? corner_planning_response_sec_ : 0.0;
    const double observed_decel = std::isfinite(measured_deceleration_mps2_) ?
      std::max(0.0, measured_deceleration_mps2_) : 0.0;
    if (approaching) {
      const double usable = *stop_remaining_ - traffic_arrival_tolerance_ - speed * latest_brake_delay_sec_;
      required_deceleration_mps2_ = speed * speed / (2.0 * std::max(0.01, usable));
    } else if (approaching_corner) {
      required_deceleration_mps2_ = corner_required_deceleration_mps2_;
    } else if (approaching_obstacle) {
      // Follow the moving obstacle target, not the final cap. Remove an
      // accumulated tracking error over the probe/response horizon.
      required_deceleration_mps2_ = obstacle_plan_deceleration_ +
        std::max(0.0, overspeed) / std::max(0.10, obstacle_coast_probe_sec_ + brake_release_response_sec_);
    }
    const double coast_decel = coast_deceleration_seen_ns_ && now_ns >= *coast_deceleration_seen_ns_ &&
      seconds(now_ns - *coast_deceleration_seen_ns_) <= 1.0 ? coast_deceleration_mps2_ : 0.0;
    const bool was_braking = service_brake_requested_;
    // Include current ramp-down, filtered speed lag and recovery to starting
    // torque. This is an estimate, not a measurement of actuator latency.
    const double release_horizon = command_brake_current_ / brake_current_fall_amps_per_sec_ +
      brake_release_response_sec_ + speed_filter_time_constant_sec_ + recovery_start_ramp_sec_;
    predicted_release_speed_ = std::max(0.0, speed - observed_decel * release_horizon);
    const double release_target = approaching_obstacle ?
      std::min(target, std::max(obstacle_requested_speed_, target - obstacle_plan_deceleration_ * release_horizon)) :
      target;
    const double release_margin = approaching_corner ? corner_brake_exit_margin_mps_ :
      approaching_obstacle ? obstacle_brake_exit_margin_ : brake_exit_speed_error_mps_;
    const bool predicted_undershoot = predicted_release_speed_ < release_target - release_margin;
    if (zero_target) {
      departure_pending_ = true;
      service_brake_requested_ = braking_allowed;
      coast_probe_elapsed_ = 0.0;
      traction_recovery_ = false;
    } else if (approaching) {
      const bool urgent = required_deceleration_mps2_ >=
        2.0 * effective_planning_deceleration(now_ns);
      // Braking continues while current ramps down and the actuator responds.
      // Release ahead of a predicted undershoot when distance still permits it.
      const double release_time =
        command_brake_current_ / brake_current_fall_amps_per_sec_ + traffic_response_time_;
      const double release_speed = std::max(0.0, speed - observed_decel * release_time);
      const double release_target = std::min(target, traffic_distance_speed_limit(
        *stop_remaining_ - 0.5 * (speed + release_speed) * release_time));
      if (service_brake_requested_) {
        if (overspeed <= 0.5 * traffic_brake_speed_hysteresis_ ||
          (!urgent && (release_speed < release_target - 0.5 * traffic_brake_speed_hysteresis_ ||
          required_deceleration_mps2_ <= coast_decel + 0.5 * traffic_brake_decel_hysteresis_)))
        {service_brake_requested_ = false;}
      } else if (overspeed > traffic_brake_speed_hysteresis_ && (!brake_mode_active_ || urgent)) {
        coast_probe_elapsed_ += dt;
        service_brake_requested_ = urgent ||
          (coast_probe_elapsed_ >= traffic_coast_probe_sec_ &&
          required_deceleration_mps2_ > coast_decel + traffic_brake_decel_hysteresis_);
      } else {coast_probe_elapsed_ = 0.0;}
    } else if (approaching_corner) {
      const bool urgent = required_deceleration_mps2_ > 0.0 &&
        (required_deceleration_mps2_ >= 2.0 * effective_planning_deceleration(now_ns) ||
        (corner_constraint_distance_m_ <= speed * corner_planning_response_sec_ + 0.05 &&
        required_deceleration_mps2_ > coast_decel + 2.0 * corner_brake_decel_margin_mps2_));
      if (service_brake_requested_) {
        if (corner_speed_error <= -corner_brake_exit_margin_mps_ ||
          (!urgent && predicted_undershoot) ||
          (!urgent && required_deceleration_mps2_ <=
          coast_decel + 0.5 * corner_brake_decel_margin_mps2_))
        {service_brake_requested_ = false;}
      } else if (corner_speed_error > corner_brake_entry_margin_mps_ &&
        (!brake_mode_active_ || urgent) && (!predicted_undershoot || urgent))
      {
        coast_probe_elapsed_ += dt;
        service_brake_requested_ = urgent ||
          (coast_probe_elapsed_ >= corner_coast_probe_sec_ &&
          required_deceleration_mps2_ > coast_decel + corner_brake_decel_margin_mps2_);
      } else {coast_probe_elapsed_ = 0.0;}
    } else if (approaching_obstacle) {
      const bool insufficient_coast = required_deceleration_mps2_ >
        coast_decel + obstacle_brake_decel_margin_;
      if (service_brake_requested_) {
        if (overspeed <= obstacle_brake_exit_margin_ || predicted_undershoot ||
          required_deceleration_mps2_ <= coast_decel + 0.5 * obstacle_brake_decel_margin_)
        {service_brake_requested_ = false;}
      } else if (overspeed > 0.0 && !brake_mode_active_) {
        coast_probe_elapsed_ += dt;
        service_brake_requested_ = coast_probe_elapsed_ >= obstacle_coast_probe_sec_ &&
          overspeed > obstacle_brake_entry_margin_ && insufficient_coast && !predicted_undershoot;
      } else {coast_probe_elapsed_ = 0.0;}
    } else {
      // Ordinary cruise retains the explicit electrical-brake enable switch.
      service_brake_requested_ = braking_allowed && overspeed >
        (service_brake_requested_ ? brake_exit_speed_error_mps_ : brake_entry_speed_error_mps_);
      coast_probe_elapsed_ = 0.0;
    }
    if (!braking_allowed) {service_brake_requested_ = false;}
    requested_brake_current_ = 0.0;
    if (service_brake_requested_) {
      const double missing_decel = (approaching || approaching_corner || approaching_obstacle) ?
        std::max(0.0, required_deceleration_mps2_ - coast_decel) : 0.0;
      const double brake_speed_error = approaching_corner ?
        std::max(0.0, corner_speed_error) :
        (zero_target ? std::abs(current_speed_mps_) : std::max(0.0, overspeed));
      requested_brake_current_ = staged_brake_current(
        brake_speed_error,
        missing_decel, longitudinal_brake_speed_gain_, longitudinal_brake_decel_gain_, brake_cap);
      if (traffic_stop && zero_target && std::abs(current_speed_mps_) < 0.05) {
        requested_brake_current_ = std::max(requested_brake_current_, traffic_hold_current_);
      } else if (traffic_stop && zero_target) {
        // Crossing the position tolerance must not suddenly drop an already
        // active braking request while the vehicle is still moving.
        requested_brake_current_ = std::max(requested_brake_current_, command_brake_current_);
      }
    }
    if (was_braking && !service_brake_requested_) {
      traction_recovery_ = !zero_target;
      coast_probe_elapsed_ = 0.0;
    }
    if (service_brake_requested_ || brake_mode_active_) {
      // Recheck standstill on the first drive cycle after release: the car can
      // finish stopping during the release cycle, even with a positive target.
      departure_pending_ = true;
      longitudinal_pid_->reset();
      command_duty_ = 0.0;
      latest_brake_rise_ = brake_current_rise_amps_per_sec_;
      if (((traffic_stop && stop_remaining_) || corner_braking_allowed_) && speed > 0.05 &&
        requested_brake_current_ > command_brake_current_)
      {
        // A corner already under the axle has no remaining coast-probe time.
        const double remaining = traffic_stop && stop_remaining_ ? *stop_remaining_ :
          corner_constraint_distance_m_;
        const double response = traffic_stop ? traffic_response_time_ :
          corner_planning_response_sec_;
        const double available_time = std::max(dt,
          (remaining - (traffic_stop ? traffic_arrival_tolerance_ : 0.0)) / speed - response);
        latest_brake_rise_ = clamp(
          (requested_brake_current_ - command_brake_current_) / available_time,
          brake_current_rise_amps_per_sec_,
          std::max(brake_current_rise_amps_per_sec_, traffic_brake_urgent_rise_));
      }
      command_brake_current_ = move_toward(command_brake_current_, requested_brake_current_,
        (requested_brake_current_ > command_brake_current_ ?
        latest_brake_rise_ : brake_current_fall_amps_per_sec_) * dt);
      if (command_brake_current_ > 1.0e-6) {
        brake_mode_active_ = true;
        longitudinal_phase_ = service_brake_requested_ ?
          (zero_target ? "stop_brake" : "additional_brake") : "release_brake";
        return "brake";
      }
      command_brake_current_ = 0.0;
      if (brake_mode_active_) {
        // The bridge replaces pending commands by mode: release must be a
        // separate publication cycle before any positive duty is issued.
        brake_mode_active_ = false;
        traction_recovery_ = !zero_target;
        longitudinal_phase_ = "release_brake";
        return "brake_release";
      }
    }
    if (zero_target) {
      command_duty_ = command_brake_current_ = 0.0;
      longitudinal_pid_->reset();
      longitudinal_phase_ = "zero_target";
      return "duty";
    }
    if (!traffic_stop && corner_profile_active_ && corner_speed_in_band_) {
      // The band is a dead zone: discard stored error so the controller does
      // not keep pulling toward an obsolete exact speed inside the corner.
      longitudinal_pid_->reset();
    }
    // Calibratable speed feedforward supplies running torque at small error.
    // Fade the offset to zero with target speed; never impose a creep floor.
    latest_feedforward_duty_ = clamp(drive_ff_offset_ *
      std::min(1.0, target / drive_ff_fade_speed_) + drive_ff_slope_ * target, 0.0, maximum_duty_);
    const double effort = longitudinal_pid_->update(
      target, current_speed_mps_, latest_feedforward_duty_ / maximum_duty_, dt);
    const double desired = clamp(effort * maximum_duty_, 0.0, maximum_duty_);
    latest_pid_effort_ = effort;
    latest_desired_duty_ = desired;
    // All input guards and the separate zero-current release cycle have passed.
    // Seed a real standstill departure once, like manual start_duty, but do not
    // exceed the PID request on a slow stop-line reapproach. This is no duty floor.
    const bool start_drive = departure_pending_ && longitudinal_start_duty_ > 0.0 &&
      std::abs(current_speed_mps_) < 0.05 && command_duty_ <= 1.0e-6 && desired > 0.0;
    // Standstill departures share the ordinary acceleration ramp. The recovery
    // ramp remains for releasing a brake while already moving.
    if (start_drive) {traction_recovery_ = false;}
    const double rise = traction_recovery_ ? recovery_duty_rise_ : duty_rise_rate_per_sec_;
    const double previous_duty = command_duty_;
    const double recovery_duty = std::min(longitudinal_start_duty_, desired);
    const double time_to_recover = std::max(0.0, recovery_duty - command_duty_) / rise;
    const bool recovery_distance_ok = !approaching ||
      *stop_remaining_ - traffic_arrival_tolerance_ >
      speed * latest_brake_delay_sec_ + speed * speed /
      (2.0 * effective_planning_deceleration(now_ns));
    const bool recover_motion = !start_drive && (traction_recovery_ || departure_pending_) &&
      recovery_duty > command_duty_ && target > 0.05 &&
      (overspeed < -0.5 * traffic_brake_speed_hysteresis_ ||
      (!traffic_stop && predicted_undershoot)) && recovery_distance_ok &&
      speed - observed_decel * (time_to_recover + traffic_response_time_) < 0.10;
    command_duty_ = start_drive ? std::min(longitudinal_start_duty_, desired) :
      move_toward(command_duty_, desired,
      (desired > command_duty_ ? rise : duty_fall_rate_per_sec_) * dt);
    if (recover_motion) {
      // Continuous bounded ramp, not repeated launch pulses. Assistance stops
      // at the lesser of manual-like starting duty and the PID request.
      command_duty_ = std::max(command_duty_, move_toward(previous_duty, recovery_duty,
        longitudinal_start_duty_ / recovery_start_ramp_sec_ * dt));
      traction_recovery_ = true;
    }
    // Do not consume departure eligibility just because the first post-release
    // sample was still moving; the vehicle may stall later in this ramp.
    if (start_drive || longitudinal_start_duty_ <= 0.0 ||
      (recovery_duty > 0.0 && command_duty_ >= recovery_duty) ||
      (speed >= 0.10 && std::abs(overspeed) <= traffic_brake_speed_hysteresis_))
    {departure_pending_ = false;}
    longitudinal_pid_->apply_output_limit(effort, command_duty_ / maximum_duty_);
    if (traction_recovery_ && std::abs(command_duty_ - desired) < 1.0e-4 &&
      overspeed >= -traffic_brake_speed_hysteresis_) {traction_recovery_ = false;}
    longitudinal_phase_ = start_drive ? "start_drive" : recover_motion ? "recover_motion" :
      traction_recovery_ ? "recover_drive" :
      (overspeed > 0.0 || command_duty_ < previous_duty ? "reduce_duty" : "track_speed");
    return "duty";
  }

  std::string legacy_longitudinal_motor(double target, double dt, bool traffic_stop)
  {
    speed_pid_->reset();
    brake_profile_->reset();
    // One signed PID controls BOTH traction and braking from target-speed error.
    // Distance is used only by traffic_speed_limit(), never to calculate current.
    // Reset on entering/leaving a zero-speed request; never propel at zero target.
    if ((target <= 0) != longitudinal_zero_target_) {longitudinal_pid_->reset();}
    longitudinal_zero_target_ = target <= 0;
    const double feedback_speed = target <= 0 ? std::abs(current_speed_mps_) : current_speed_mps_;
    const double effort = longitudinal_pid_->update(target, feedback_speed, 0.0, dt);
    latest_pid_effort_ = effort;
    latest_desired_duty_ = target > 0 ? std::max(0.0, effort) * maximum_duty_ : 0.0;
    const bool braking_allowed = traffic_stop || electrical_brake_enabled_ ||
      obstacle_profile_active_ || corner_braking_allowed_;
    const double brake_cap = brake_maximum_current_amps_;
    double brake = braking_allowed ? std::max(0.0, -effort) * brake_cap : 0.0;
    if (traffic_stop && target <= 0 && std::abs(current_speed_mps_) < 0.05) {
      brake = std::max(brake, traffic_hold_current_);
    }
    requested_brake_current_ = brake;
    if (brake > 0.0) {
      command_duty_ = 0.0;
      command_brake_current_ = brake;
      brake_mode_active_ = true;
      return "brake";
    }
    command_brake_current_ = 0.0;
    if (brake_mode_active_) {
      command_duty_ = 0.0;
      brake_mode_active_ = false;
      return "brake_release";
    }
    const double desired = target > 0 ? std::max(0.0, effort) * maximum_duty_ : 0.0;
    // Restore the pre-PID start behavior only for ordinary departure. With
    // a stop request, retain zero-minimum duty for the slow final approach.
    if (!traffic_stop && std::abs(current_speed_mps_) < 0.05 && command_duty_ == 0.0 &&
      desired > 0.0 && (last_motor_mode_ == "stop" || last_motor_mode_ == "brake_release"))
    {
      command_duty_ = std::min(minimum_duty_, desired);
    }
    command_duty_ = target <= 0 || effort < 0 ? 0.0 : move_toward(command_duty_, desired,
      (desired > command_duty_ ? duty_rise_rate_per_sec_ : duty_fall_rate_per_sec_) * dt);
    return "duty";
  }

  void publish_guard_stop(const std::string & reason)
  {
    // A missing lane must not release a red-light brake. Retain the latest
    // steering command while stopping. Motor handling of disconnected/disabled
    // modes is unchanged; steering follows stop_steering_hold_enabled.
    const bool avoidance_braking=avoidance_control_enabled_ && !avoidance_deformation_only_ && enabled_ && control_mode_=="drive" && vesc_connected_;
    const bool keep_braking = traffic_stop_requested() && enabled_ &&
      control_mode_ == "drive" && vesc_connected_;
    stop_control(reason);
    if(operator_stopped_ && enabled_ && control_mode_=="drive" && vesc_connected_) {
      publish_float(duty_pub_,0.);
      command_brake_current_=brake_maximum_current_amps_;
      requested_brake_current_=command_brake_current_;brake_mode_active_=true;
      publish_commands(0.,command_brake_current_,latest_servo_position_,"brake");
    } else if (keep_braking) {
      // This is a recoverable input failure, not arrival at the stop position.
      traffic_reapproach_ = true;
      traffic_phase_ = "temporary_stop_control_input";
      stop_line_seen_ns_.reset();
      stop_candidate_count_ = 0;
      command_brake_current_ = avoidance_braking ? std::max(brake_maximum_current_amps_,avoidance_brake_current_) : brake_maximum_current_amps_;
      requested_brake_current_ = command_brake_current_;
      brake_mode_active_ = true;
      publish_commands(0.0, command_brake_current_, latest_servo_position_, "brake");
    } else if (avoidance_braking) {
      command_brake_current_=avoidance_brake_current_;
      requested_brake_current_=command_brake_current_; brake_mode_active_=true;
      publish_commands(0.0,command_brake_current_,latest_servo_position_,"brake");
    } else {publish_commands(0.0, 0.0, latest_servo_position_, "stop");}
  }

  std::optional<std::string> stop_reason(std::int64_t now_ns, bool check_start=true) const
  {
    if (check_start && operator_stopped_)return "operator_stop";
    if (!enabled_) {return "disabled";}
    if (startup_safety_enabled_ && control_mode_=="drive") {
      if (startup_manual_) {return "manual_mode";}
      if (!camera_startup_ready_) {return "waiting_for_camera_pose";}
    }
    if (model_startup_check_required_ &&
      !model_startup_gate_.ready(std::chrono::steady_clock::now()))
    {return "waiting_for_models";}
    if (control_mode_ != "monitor_only") {
      if (!vesc_connected_) {return "vesc_disconnected";}
      if (!last_erpm_ns_) {return "waiting_for_erpm";}
      if (seconds(now_ns - *last_erpm_ns_) > erpm_timeout_sec_) {return "erpm_timeout";}
    }
    if (avoidance_control_enabled_ && !avoidance_deformation_only_) {
      if ((!avoidance_path_ && !avoidance_follow_centerline_) || !avoidance_lane_ns_ || !avoidance_depth_ns_) {
        return "avoidance_"+avoidance_status_;
      }
      if (now_ns<*avoidance_lane_ns_ || now_ns<*avoidance_depth_ns_ ||
        seconds(now_ns-*avoidance_lane_ns_)>avoidance_max_age_ ||
        seconds(now_ns-*avoidance_depth_ns_)>avoidance_max_age_ ||
        std::chrono::duration<double>(std::chrono::steady_clock::now()-avoidance_received_at_).count()>avoidance_max_age_) {
        return "avoidance_stale";
      }
      if (!avoidance_follow_centerline_ && avoidance_confirmations_<3) {return "avoidance_confirming";}
    }
    if (!path_ || !last_path_received_ns_) {return "centerline_missing";}
    if (path_hold_active_ && (!last_valid_path_received_ns_ ||
      now_ns < *last_valid_path_received_ns_ ||
      seconds(now_ns - *last_valid_path_received_ns_) > path_hold_timeout_sec_))
    {return "centerline_hold_timeout";}
    if (seconds(now_ns - *last_path_received_ns_) > path_timeout_sec_) {return "centerline_timeout";}
    if (path_capture_ns_ && seconds(now_ns - *path_capture_ns_) > path_capture_maximum_age_sec_) {
      return "centerline_stale";
    }
    if (check_start && startup_safety_enabled_ && control_mode_=="drive" &&
      !start_authorization_.approved()) {return "waiting_for_start_button";}
    return std::nullopt;
  }

  void refresh_start_authorization()
  {
    const auto clock=now().nanoseconds();
    const bool erpm_fresh=last_erpm_ns_ && clock>=*last_erpm_ns_ &&
      seconds(clock-*last_erpm_ns_)<=erpm_timeout_sec_;
    const bool models=!model_startup_check_required_ || model_startup_gate_.ready(std::chrono::steady_clock::now());
    const std::string reason=!enabled_ ? "Automatic control disabled" :
      !camera_startup_ready_ ? "Camera pose not ready" : !models ? "Model readiness lost" :
      !vesc_connected_ ? "VESC disconnected" : !erpm_fresh ? "ERPM telemetry expired" : "";
    start_authorization_.observe(enabled_ && camera_startup_ready_ && models && vesc_connected_ && erpm_fresh,
      startup_manual_ || control_mode_!="drive", reason);
  }
  void publish_startup_state()
  {
    if (!startup_state_pub_) {return;}
    std_msgs::msg::String message;
    const auto reason=stop_reason(now().nanoseconds(),false);
    if (startup_drive_mode_=="STOPPING") {message.data="WAITING | switching to manual; braking until stopped";}
    else if (startup_manual_ || control_mode_!="drive") {message.data="MANUAL | automatic start bypassed";}
    else if (start_authorization_.approved()) {message.data="STARTED | "+reason.value_or("automatic duty permitted");}
    else if (reason) {message.data="WAITING | "+*reason;}
    else {message.data="READY_TO_START | press START to permit automatic duty";}
    if (!start_authorization_.approved() && !start_authorization_.last_reason().empty()) {
      message.data+=" | last stop: "+start_authorization_.last_reason();
    }
    startup_state_pub_->publish(message);
  }

  bool update_control_from_path(
    std::int64_t & decision_finished_ros_ns)
  {
    bev_handoff::RuntimeSample runtime_sample(monitor_.get(),"control");
    const std::int64_t now_ns = now().nanoseconds();
    const double dt = clamp(seconds(now_ns - last_control_ns_), 1.0e-6,
      std::min({path_timeout_sec_, path_capture_maximum_age_sec_, erpm_timeout_sec_}));
    last_control_ns_ = now_ns;
    latest_pid_effort_ = latest_desired_duty_ = kUnavailable;
    latest_feedforward_duty_ = required_deceleration_mps2_ = kUnavailable;
    latest_brake_delay_sec_ = latest_brake_rise_ = kUnavailable;
    latest_planning_deceleration_mps2_ = kUnavailable;
    requested_brake_current_ = 0.0;
    if (const auto reason = stop_reason(now_ns)) {
      if (avoidance_control_enabled_ && *reason!="avoidance_confirming") {avoidance_confirmations_=0;}
      publish_guard_stop(*reason);
      decision_finished_ros_ns = now().nanoseconds();
      return false;
    }
    advance_stop_distance(now_ns);
    update_traffic_decision(now_ns);
    const bool fresh_deformation=avoidance_path_ && avoidance_lane_ns_ && avoidance_depth_ns_ &&
      now_ns>=*avoidance_lane_ns_ && now_ns>=*avoidance_depth_ns_ &&
      seconds(now_ns-*avoidance_lane_ns_)<=avoidance_max_age_ &&
      seconds(now_ns-*avoidance_depth_ns_)<=avoidance_max_age_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now()-avoidance_received_at_).count()<=avoidance_max_age_;
    const bool following_avoidance=avoidance_control_enabled_ && !avoidance_follow_centerline_ &&
      (avoidance_deformation_only_?fresh_deformation:bool(avoidance_path_));
    if (avoidance_control_enabled_ && avoidance_deformation_only_ && !following_avoidance) {
      avoidance_status_="CENTERLINE: no current deformation";
    }
    const auto & selected_path=following_avoidance?*avoidance_path_:*path_;
    latest_planning_deceleration_mps2_ = effective_planning_deceleration(now_ns);
    const double curvature = representative_curvature(
      selected_path, -std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(), curvature_percentile_);
    // Use near geometry and the outstanding steering command, not a distant
    // bend or one missing curve sample, to allow acceleration after a turn.
    const bool turning = std::abs(selected_path.curvature(0.0)) > turn_exit_curvature_ ||
      std::abs(selected_path.heading(stanley_heading_lookahead_m_)) > turn_exit_angle_rad_ ||
      std::abs(steering_angle_rad_) > turn_exit_angle_rad_;
    if (turning) {
      turn_recovery_hold_ = true;
      turn_exit_elapsed_ = 0.0;
    } else {
      turn_exit_elapsed_ += dt;
      if (turn_exit_elapsed_ >= turn_exit_confirm_sec_) {turn_recovery_hold_ = false;}
    }
    refresh_obstacle_slowdown();
    obstacle_requested_speed_ = obstacle_slowdown_active_ ?
      std::min(maximum_speed_mps_, obstacle_slowdown_speed_mps_) : maximum_speed_mps_;
    if (!obstacle_planned_speed_) {
      // A fresh departure under an existing detection starts with its cap;
      // only a detection during motion needs a deceleration transition.
      obstacle_planned_speed_ = obstacle_slowdown_active_ ?
        clamp(std::max(current_speed_mps_, obstacle_requested_speed_), 0.0, maximum_speed_mps_) :
        maximum_speed_mps_;
    }
    const double previous_obstacle_speed = *obstacle_planned_speed_;
    const bool obstacle_recovering = obstacle_requested_speed_ > previous_obstacle_speed;
    if (!obstacle_recovering || !turn_recovery_hold_) {
      *obstacle_planned_speed_ = move_toward(previous_obstacle_speed, obstacle_requested_speed_,
        (obstacle_recovering ? obstacle_planning_acceleration_ : obstacle_planning_deceleration_) * dt);
    }
    obstacle_plan_deceleration_ = std::max(0.0,
      (previous_obstacle_speed - *obstacle_planned_speed_) / dt);
    obstacle_profile_active_ = obstacle_slowdown_active_ ||
      *obstacle_planned_speed_ < maximum_speed_mps_ - 1.0e-6;
    const double obstacle_limit = *obstacle_planned_speed_;
    double target_speed = obstacle_limit;
    corner_braking_allowed_ = false;
    corner_speed_in_band_ = false;
    corner_start_distance_m_ = 0.0;
    corner_constraint_distance_m_ = 0.0;
    corner_required_deceleration_mps2_ = 0.0;
    if (curvature_speed_control_enabled_) {
      PathSpeedPlan plan;
      const double response_distance_m =
        std::max(0.0, current_speed_mps_) * corner_planning_response_sec_;
      try {
        plan = plan_path_speeds(
          selected_path, minimum_speed_mps_, maximum_speed_mps_, maximum_lateral_acceleration_mps2_,
          latest_planning_deceleration_mps2_, corner_planning_acceleration_mps2_, maximum_speed_mps_,
          response_distance_m, corner_speed_band_width_mps_);
      } catch (const std::exception & exception) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Rejected speed plan: %s", exception.what());
        publish_guard_stop("speed_plan_invalid");
        decision_finished_ros_ns = now().nanoseconds();
        return false;
      }
      const bool raw_corner_profile = plan.corner_start_m >= 0.0 &&
        plan.target_upper_speed_mps < maximum_speed_mps_ - 0.03;
      const bool had_corner_profile = corner_profile_active_;
      const double previous_corner_upper = corner_upper_speed_mps_;
      double raw_required_deceleration = 0.0;
      double raw_constraint_distance = plan.corner_start_m;
      if (raw_corner_profile) {
        const double speed_squared = std::max(0.0, current_speed_mps_) *
          std::max(0.0, current_speed_mps_);
        for (std::size_t index = 0; index < plan.arc_m.size(); ++index) {
          const double usable_distance = plan.arc_m[index] - response_distance_m;
          if (usable_distance <= 0.0) {continue;}
          const double limit_squared = plan.upper_speed_mps[index] *
            plan.upper_speed_mps[index];
          const double required = std::max(0.0, speed_squared - limit_squared) /
            (2.0 * std::max(0.01, usable_distance));
          if (required > raw_required_deceleration) {
            raw_required_deceleration = required;
            raw_constraint_distance = plan.arc_m[index];
          }
        }
        corner_detection_count_ = std::min(corner_confirm_frames_, corner_detection_count_ + 1);
        corner_miss_count_ = 0;
      } else {
        corner_detection_count_ = 0;
        corner_miss_count_ = std::min(corner_release_frames_, corner_miss_count_ + 1);
      }
      const bool urgent_corner = raw_corner_profile &&
        (plan.corner_start_m <= response_distance_m + 0.05 ||
        raw_required_deceleration >= 2.0 * latest_planning_deceleration_mps2_);
      if (!corner_profile_active_ &&
        (corner_detection_count_ >= corner_confirm_frames_ || urgent_corner))
      {corner_profile_active_ = true;}
      if (corner_profile_active_ && !raw_corner_profile &&
        corner_miss_count_ >= corner_release_frames_ && !turn_recovery_hold_)
      {corner_profile_active_ = false;}

      if (corner_profile_active_) {
        corner_braking_allowed_ = true;
        if (raw_corner_profile) {
          corner_lower_speed_mps_ = plan.target_lower_speed_mps;
          corner_nominal_speed_mps_ = plan.target_nominal_speed_mps;
          corner_upper_speed_mps_ = plan.target_upper_speed_mps;
          corner_start_distance_m_ = plan.corner_start_m;
          corner_constraint_distance_m_ = raw_constraint_distance;
          corner_required_deceleration_mps2_ = raw_required_deceleration;
        } else {
          // Keep the band through a turn even when near curvature briefly
          // disappears. Recovery is allowed only after confirmed turn exit.
          const double recovery = turn_recovery_hold_ ? 0.0 : corner_planning_acceleration_mps2_ * dt;
          corner_lower_speed_mps_ = std::min(maximum_speed_mps_,
            corner_lower_speed_mps_ + recovery);
          corner_nominal_speed_mps_ = std::min(maximum_speed_mps_,
            corner_nominal_speed_mps_ + recovery);
          corner_upper_speed_mps_ = std::min(maximum_speed_mps_,
            corner_upper_speed_mps_ + recovery);
        }
        if (had_corner_profile && turn_recovery_hold_) {
          corner_upper_speed_mps_ = std::min(corner_upper_speed_mps_, previous_corner_upper);
          corner_lower_speed_mps_ = std::max(minimum_speed_mps_,
            corner_upper_speed_mps_ - corner_speed_band_width_mps_);
          corner_nominal_speed_mps_ = 0.5 * (corner_lower_speed_mps_ + corner_upper_speed_mps_);
        }
        const double speed = std::max(0.0, current_speed_mps_);
        if (speed < corner_lower_speed_mps_) {
          target_speed = corner_nominal_speed_mps_;
        } else if (speed >= corner_upper_speed_mps_ - corner_coast_entry_margin_mps_ &&
          (corner_required_deceleration_mps2_ > 0.0 || speed > corner_upper_speed_mps_))
        {
          target_speed = corner_nominal_speed_mps_;
        } else {
          // Inside the band, remove speed error and let feed-forward maintain
          // the present speed instead of chasing one exact corner value.
          corner_speed_in_band_ = true;
          target_speed = clamp(speed, corner_lower_speed_mps_, corner_upper_speed_mps_);
        }
      } else {
        corner_lower_speed_mps_ = corner_nominal_speed_mps_ =
          corner_upper_speed_mps_ = obstacle_limit;
      }
      // Limit only upward recovery; a confirmed future safety limit may lower
      // the target immediately.
      if (current_speed_mps_ >= 0.05 && latest_target_speed_mps_ > 0.0) {
        target_speed = std::min(target_speed,
          latest_target_speed_mps_ + corner_planning_acceleration_mps2_ * dt);
      }
    }
    // Obstacle caps evolve independently of curve planning. A lower traffic
    // or curvature restriction must never be raised by the comfort ramp.
    if (obstacle_limit < target_speed) {corner_speed_in_band_ = false;}
    target_speed = std::min(target_speed, obstacle_limit);
    // Traffic/curvature may request a lower target; release never overrides them.
    const bool traffic_stop = traffic_stop_requested();
    if (traffic_stop) {target_speed = std::min(target_speed, traffic_speed_limit(now_ns));}
    // A live UI ceiling applies immediately to the target, even while an old
    // curve/obstacle recovery profile is retained. Physical deceleration still
    // uses the existing duty/brake controller and its configured limits.
    if (target_speed > maximum_speed_mps_) {corner_speed_in_band_ = false;}
    target_speed = std::min(target_speed, maximum_speed_mps_);
    const auto stanley = stanley_control(
      selected_path, current_speed_mps_, stanley_gain_, stanley_softening_speed_mps_,
      stanley_heading_lookahead_m_, maximum_steering_angle_rad_,
      stanley_corner_heading_threshold_rad_, stanley_corner_opposing_correction_ratio_);
    // Freeze the final command at standstill, with speed hysteresis to prevent
    // ERPM noise from toggling the hold. Resume as soon as motion is requested.
    if (!stop_steering_hold_enabled_ || target_speed > 0.0 ||
      std::abs(current_speed_mps_) > 0.10) {steering_held_ = false;}
    else if (std::abs(current_speed_mps_) < 0.05) {steering_held_ = true;}
    const bool corner_reset =
      !steering_held_ &&
      std::abs(stanley.heading_error_rad) >= stanley_corner_heading_threshold_rad_ &&
      stanley.steering_angle_rad * steering_angle_rad_ < 0.0;
    double filtered = stanley.steering_angle_rad;
    if (steering_held_) {
      filtered = steering_angle_rad_;
    } else if (corner_reset) {
      steering_angle_rad_ = 0.0;
    } else {
      const double current_weight = 1.0 - std::pow(
        1.0 - steering_current_weight_, dt * control_rate_hz_);
      filtered = current_weight * stanley.steering_angle_rad +
        (1.0 - current_weight) * steering_angle_rad_;
    }
    steering_angle_rad_ = move_toward(
      steering_angle_rad_, filtered, steering_rate_limit_rad_per_sec_ * dt);
    const double servo = steering_angle_to_servo(
      steering_angle_rad_, maximum_steering_angle_rad_, servo_left_, servo_center_,
      servo_right_, steering_servo_inverted_);
    std::string motor_mode;
    if (control_mode_ != "drive") {
      command_duty_ = 0.0;
      command_brake_current_ = 0.0;
      brake_mode_active_ = false;
      speed_pid_->reset();
      longitudinal_pid_->reset();
      brake_profile_->reset();
      motor_mode = "suppressed";
      longitudinal_phase_ = "suppressed";
      service_brake_requested_ = traction_recovery_ = false;
      coast_probe_elapsed_ = 0.0;
    } else if (longitudinal_pid_enabled_ || traffic_stop || obstacle_profile_active_ ||
      corner_braking_allowed_) {
      motor_mode = longitudinal_motor(target_speed, dt, traffic_stop);
    } else {
      longitudinal_phase_ = "legacy_speed_pid";
      service_brake_requested_ = traction_recovery_ = false;
      command_brake_current_ = electrical_brake_enabled_ ?
        brake_profile_->update(target_speed, current_speed_mps_, dt) : 0.0;
      if (!electrical_brake_enabled_) {brake_profile_->reset();}
      if (command_brake_current_ > 0.0) {
        command_duty_ = 0.0;
        speed_pid_->reset();
        motor_mode = "brake";
        brake_mode_active_ = true;
      } else if (brake_mode_active_) {
        command_duty_ = 0.0;
        speed_pid_->reset();
        motor_mode = "brake_release";
        brake_mode_active_ = false;
      } else {
        const double feedforward = speed_feedforward_duty(
          target_speed, minimum_speed_mps_, runtime_speed_limit_->ceiling(), minimum_duty_, maximum_duty_);
        const double desired = speed_pid_->update(target_speed, current_speed_mps_, feedforward, dt);
        latest_desired_duty_ = desired;
        if (command_duty_ < minimum_duty_) {
          command_duty_ = minimum_duty_;
        } else {
          const double rate = desired >= command_duty_ ? duty_rise_rate_per_sec_ : duty_fall_rate_per_sec_;
          command_duty_ = move_toward(command_duty_, desired, rate * dt);
        }
        motor_mode = "duty";
      }
    }
    last_stop_reason_ = traffic_stop ? "traffic_" + traffic_phase_ : "running";
    latest_target_speed_mps_ = target_speed;
    latest_curvature_per_m_ = curvature;
    latest_cross_track_error_m_ = stanley.cross_track_error_m;
    latest_heading_error_rad_ = stanley.heading_error_rad;
    latest_raw_steering_angle_rad_ = stanley.steering_angle_rad;
    latest_direction_guard_used_ = stanley.direction_guard_used || corner_reset;
    decision_finished_ros_ns = now().nanoseconds();
    publish_commands(command_duty_, command_brake_current_, servo, motor_mode);
    return true;
  }

  void stop_control(const std::string & reason)
  {
    departure_pending_ = true;
    service_brake_requested_ = traction_recovery_ = false;
    corner_profile_active_ = corner_speed_in_band_ = false;
    corner_detection_count_ = corner_miss_count_ = 0;
    turn_recovery_hold_ = false;
    turn_exit_elapsed_ = 0.0;
    obstacle_planned_speed_.reset();
    obstacle_profile_active_ = false;
    obstacle_plan_deceleration_ = 0.0;
    predicted_release_speed_ = kUnavailable;
    terminal_tracking_active_ = false;
    coast_probe_elapsed_ = 0.0;
    coast_deceleration_seen_ns_.reset();
    latest_feedforward_duty_ = required_deceleration_mps2_ = kUnavailable;
    latest_brake_delay_sec_ = latest_brake_rise_ = kUnavailable;
    requested_brake_current_ = 0.0;
    longitudinal_phase_ = "guard_stop";
    last_control_ns_ = now().nanoseconds();
    command_duty_ = 0.0;
    command_brake_current_ = 0.0;
    brake_mode_active_ = false;
    steering_held_ = stop_steering_hold_enabled_;
    if (!steering_held_) {
      steering_angle_rad_ = 0.0;
      latest_servo_position_ = servo_center_;
    }
    latest_pid_effort_ = latest_desired_duty_ = kUnavailable;
    latest_target_speed_mps_ = 0.0;
    latest_curvature_per_m_ = 0.0;
    latest_cross_track_error_m_ = 0.0;
    latest_heading_error_rad_ = 0.0;
    latest_raw_steering_angle_rad_ = 0.0;
    latest_direction_guard_used_ = false;
    speed_pid_->reset();
    longitudinal_pid_->reset();
    brake_profile_->reset();
    if (reason != last_stop_reason_) {
      RCLCPP_WARN(get_logger(), "Automatic drive stop condition: %s.", reason.c_str());
    }
    last_stop_reason_ = reason;
  }

  void publish_float(
    const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr & publisher, double value)
  {
    std_msgs::msg::Float32 message;
    message.data = static_cast<float>(value);
    publisher->publish(message);
  }

  void publish_diagnostic(
    const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr & publisher, double value)
  {
    if (!diagnostic_subscribed(*publisher)) {return;}
    publish_float(publisher, value);
  }

  // Subscriber counts are middleware queries; refresh them once per second
  // instead of for every diagnostic topic in every control cycle.
  bool diagnostic_subscribed(const rclcpp::PublisherBase & publisher)
  {
    const auto checked = std::chrono::steady_clock::now();
    if (checked - diagnostic_subscriptions_checked_at_ >= std::chrono::seconds(1)) {
      diagnostic_subscriptions_.clear();
      diagnostic_subscriptions_checked_at_ = checked;
    }
    const auto found = diagnostic_subscriptions_.find(&publisher);
    if (found != diagnostic_subscriptions_.end()) {return found->second;}
    const bool subscribed = publisher.get_subscription_count() > 0U ||
      publisher.get_intra_process_subscription_count() > 0U;
    diagnostic_subscriptions_.emplace(&publisher, subscribed);
    return subscribed;
  }

  void publish_commands(
    double duty, double brake_current, double servo_position, const std::string & motor_mode)
  {
    if(operator_stopped_ && motor_mode=="duty")duty=0.;
    // Last line of defence: every automatic duty publisher goes through here.
    if (startup_safety_enabled_ && control_mode_=="drive") {
      refresh_start_authorization();
      if (motor_mode=="duty" && !start_authorization_.approved()) {duty=0.0;}
    }
    if (motor_mode == "brake" || motor_mode == "brake_release") {
      last_brake_command_ns_ = now().nanoseconds();
    }
    if (control_mode_ == "drive") {
      if (motor_mode == "duty") {publish_float(duty_pub_, duty);}
      else if (motor_mode == "brake") {publish_float(brake_pub_, brake_current);}
      else if (motor_mode == "brake_release") {publish_float(brake_pub_, 0.0);}
      else if (motor_mode == "stop") {
        publish_float(brake_pub_, 0.0);
        publish_float(duty_pub_, 0.0);
      } else {throw std::invalid_argument("unsupported motor mode: " + motor_mode);}
    } else if (motor_mode != "suppressed" && motor_mode != "stop") {
      throw std::invalid_argument("unsupported passive motor mode: " + motor_mode);
    }
    last_motor_mode_ = motor_mode;
    latest_duty_command_ = duty;
    latest_brake_command_ = brake_current;
    latest_servo_position_ = servo_position;
    if (control_mode_ != "monitor_only") {publish_float(servo_pub_, servo_position);}
    publish_diagnostic(command_duty_pub_, duty);
    publish_diagnostic(command_brake_pub_, brake_current);
    publish_diagnostic(target_speed_pub_, latest_target_speed_mps_);
    publish_diagnostic(current_speed_pub_, current_speed_mps_);
    if (diagnostic_subscribed(*corner_state_pub_)) {
      // Controller estimate: 0=straight, 1=approach, 2=turning, 3=stopped/inhibited.
      // A speed profile alone may describe a future corner; turning uses the
      // near-path/steering exit latch, including its confirmation interval.
      std_msgs::msg::UInt8 state;
      state.data = longitudinal_phase_ == "guard_stop" || latest_target_speed_mps_ <= 0.0 ||
        std::abs(current_speed_mps_) < 0.05 ? 3U :
        turn_recovery_hold_ ? 2U : corner_profile_active_ ? 1U : 0U;
      corner_state_pub_->publish(state);
    }
    publish_diagnostic(curvature_pub_, latest_curvature_per_m_);
    publish_diagnostic(steering_pub_, steering_angle_rad_);
    publish_diagnostic(cross_track_pub_, latest_cross_track_error_m_);
    publish_diagnostic(heading_pub_, latest_heading_error_rad_);
    publish_diagnostic(raw_steering_pub_, latest_raw_steering_angle_rad_);
    publish_diagnostic(command_servo_pub_, latest_servo_position_);
    record_driving_log();
  }

  static std::string json_number(double value, int digits = 4)
  {
    if (!std::isfinite(value)) {return "null";}
    std::ostringstream out;
    out << std::fixed << std::setprecision(digits) << value;
    return out.str();
  }

  void publish_preview_snapshot()
  {
    if (!diagnostic_subscribed(*preview_snapshot_pub_)) {return;}
    const auto steady = std::chrono::steady_clock::now();
    const double drained = std::chrono::duration<double>(steady - preview_meters_drained_at_).count();
    if (drained >= 1.0) {
      preview_stages_json_ = "{\"infer\":" + preview_infer_meter_.drain(drained) +
        ",\"post\":" + preview_post_meter_.drain(drained) +
        ",\"control\":" + preview_control_meter_.drain(drained) +
        ",\"valid_lane\":" + preview_valid_lane_meter_.drain(drained) + "}";
      preview_meters_drained_at_ = steady;
    }
    const std::int64_t now_ns = now().nanoseconds();
    const unsigned corner = longitudinal_phase_ == "guard_stop" || latest_target_speed_mps_ <= 0.0 ||
      std::abs(current_speed_mps_) < 0.05 ? 3U :
      turn_recovery_hold_ ? 2U : corner_profile_active_ ? 1U : 0U;
    std::ostringstream out;
    out << "{\"speed\":" << json_number(current_speed_mps_)
        << ",\"target\":" << json_number(latest_target_speed_mps_)
        << ",\"duty\":" << json_number(latest_duty_command_)
        << ",\"brake\":" << json_number(latest_brake_command_)
        << ",\"servo\":" << json_number(latest_servo_position_)
        << ",\"steer\":" << json_number(steering_angle_rad_)
        << ",\"raw_steer\":" << json_number(latest_raw_steering_angle_rad_)
        << ",\"cte\":" << json_number(latest_cross_track_error_m_)
        << ",\"heading\":" << json_number(latest_heading_error_rad_)
        << ",\"curv\":" << json_number(latest_curvature_per_m_)
        << ",\"corner\":" << corner
        << ",\"vesc\":" << (vesc_connected_ ? "true" : "false")
        << ",\"lane\":{\"age_ms\":" << (last_path_received_ns_ ?
          json_number(static_cast<double>(now_ns - *last_path_received_ns_) / 1.0e6, 1) : "null")
        << ",\"state\":" << static_cast<unsigned>(latest_lane_state_)
        << ",\"source\":\"" << (latest_lane_source_.empty() ? "RGB" : latest_lane_source_) << "\""
        << ",\"stop_m\":" << json_number(latest_stop_line_raw_m_, 3)
        << ",\"points\":[";
    if (path_) {
      const auto & points = path_->points;
      const std::size_t count = points.size();
      const std::size_t shown = std::min<std::size_t>(count, 24U);
      for (std::size_t i = 0; i < shown; ++i) {
        const std::size_t index = shown > 1U ? i * (count - 1U) / (shown - 1U) : 0U;
        out << (i ? "," : "") << "[" << json_number(points[index].x, 3) << ","
            << json_number(points[index].y, 3) << "]";
      }
    }
    out << "]},\"stages\":" << preview_stages_json_ << "}";
    std_msgs::msg::String message;
    message.data = out.str();
    preview_snapshot_pub_->publish(message);
  }

  void on_measured_erpm(const std_msgs::msg::Int32::ConstSharedPtr message)
  {
    const auto now_ns = now().nanoseconds();
    const double previous_speed = current_speed_mps_;
    const double raw = erpm_to_speed_mps(
      message->data, wheel_diameter_m_, motor_pole_pairs_, motor_pinion_teeth_,
      spur_gear_teeth_, differential_pinion_teeth_, differential_ring_teeth_,
      erpm_direction_sign_, speed_scale_correction_);
    latest_erpm_ = message->data;
    latest_raw_speed_mps_ = raw;
    if (!last_erpm_ns_) {
      current_speed_mps_ = raw;
      measured_deceleration_mps2_ = kUnavailable;
    } else {
      const double dt = seconds(now_ns - *last_erpm_ns_);
      if (dt <= 0.0 || dt > erpm_timeout_sec_) {
        current_speed_mps_ = raw;
        measured_deceleration_mps2_ = kUnavailable;
        coast_deceleration_seen_ns_.reset();
      }
      else {
        const double gain = 1.0 - std::exp(-dt / std::max(1.0e-4, speed_filter_time_constant_sec_));
        current_speed_mps_ += gain * (raw - current_speed_mps_);
        const double decel = clamp((previous_speed - current_speed_mps_) / dt, -5.0, 5.0);
        const double decel_weight = 1.0 - std::exp(-dt / deceleration_filter_sec_);
        measured_deceleration_mps2_ = std::isfinite(measured_deceleration_mps2_) ?
          measured_deceleration_mps2_ + decel_weight * (decel - measured_deceleration_mps2_) : decel;
        // Braking-induced deceleration must not be mistaken for what duty
        // reduction alone can achieve on the next control cycle.
        const bool brake_settled = !last_brake_command_ns_ ||
          (now_ns >= *last_brake_command_ns_ && seconds(now_ns - *last_brake_command_ns_) >=
          traffic_response_time_ + 3.0 * (speed_filter_time_constant_sec_ + deceleration_filter_sec_));
        // Fresh CAN status may veto a coast estimate while the motor is still
        // applying more duty than requested or reports braking current. UART
        // ERPM remains usable when the optional CAN monitor is unavailable.
        const bool duty_matches = !dynamics_duty_seen_ns_ ||
          now_ns < *dynamics_duty_seen_ns_ ||
          seconds(now_ns - *dynamics_duty_seen_ns_) > 0.20 ||
          dynamics_duty_ <= command_duty_ + 0.02;
        const bool no_uncommanded_brake = !dynamics_motor_current_seen_ns_ ||
          now_ns < *dynamics_motor_current_seen_ns_ ||
          seconds(now_ns - *dynamics_motor_current_seen_ns_) > 0.20 ||
          dynamics_motor_current_ >= -0.10;
        if (brake_settled && longitudinal_phase_ == "reduce_duty" && last_motor_mode_ == "duty" &&
          command_brake_current_ == 0.0 && duty_matches && no_uncommanded_brake)
        {
          coast_deceleration_mps2_ = std::max(0.0, measured_deceleration_mps2_);
          coast_deceleration_seen_ns_ = now_ns;
        }
      }
    }
    last_erpm_ns_ = now_ns;
  }

  void on_connection_status(const std_msgs::msg::Bool::ConstSharedPtr message)
  {
    vesc_connected_ = message->data;
    if (!vesc_connected_) {start_authorization_.revoke("VESC disconnected");}
  }

  void on_enable(const std_msgs::msg::Bool::ConstSharedPtr message)
  {
    enabled_ = message->data;
    if (!enabled_) {
      start_authorization_.revoke("Automatic control disabled");
      stop_control("disabled");
      publish_commands(0.0, 0.0, latest_servo_position_, "stop");
    }
    RCLCPP_WARN(get_logger(), "Automatic control %s.", enabled_ ? "enabled" : "disabled");
  }

  void on_watchdog()
  {
    if (startup_safety_enabled_) {refresh_start_authorization();}
    const bool slowdown_released=refresh_obstacle_slowdown();
    if (const auto reason = stop_reason(now().nanoseconds())) {
      if (avoidance_control_enabled_ && *reason!="avoidance_confirming") {avoidance_confirmations_=0;}
      publish_guard_stop(*reason);
    } else if (slowdown_released) {
      std::int64_t finished=0; update_control_from_path(finished);
    }
  }

  void start_driving_log()
  {
    try {
      std::string parameters = "effective_control_mode=" + control_mode_ + "\n";
      for (const auto & name : list_parameters({}, 0U).names) {
        parameters += name + "=" + get_parameter(name).value_to_string() + "\n";
      }
      driving_log_ = std::make_unique<DrivingLog>(driving_log_directory_,
        "ros_time_ns,elapsed_sec,logger_dropped_rows,control_mode,control_state,traffic_phase,"
        "motor_mode,lane_status,signal_state,longitudinal_phase,enabled,vesc_connected,traffic_stop_enabled,"
        "red_latched,position_hold,steering_held,speed_mps,raw_speed_mps,measured_erpm,"
        "target_speed_mps,speed_error_mps,pid_effort,desired_duty,command_duty,brake_current_a,"
        "raw_steering_rad,steering_rad,servo_position,cte_m,heading_error_rad,curvature_per_m,"
        "stop_line_present,stop_line_raw_m,stop_line_corrected_m,stop_remaining_m,"
        "stop_last_confirmed_m,stop_confirmed_age_s,stop_candidate_count,stop_line_rejections,"
        "slow_reapproach,signal_score,signal_age_s,signal_message_accepted,erpm_age_s,"
        "lane_receive_age_s,lane_capture_age_at_rx_s,path_capture_age_s,lane_sequence,"
        "path_valid,path_point_count,stop_active,pass_committed,red_from_green,"
        "predicted_stop_distance_m,predicted_overshoot_m,red_age_s,path_held,"
        "feedforward_duty,requested_brake_current_a,required_deceleration_mps2,"
        "measured_deceleration_mps2,coast_deceleration_mps2,coast_age_s,coast_probe_elapsed_s,"
        "traction_recovery,terminal_tracking,brake_delay_s,brake_rise_limit_a_per_s,obstacle_slowdown_active,"
        "corner_braking_allowed,corner_profile_active,corner_speed_in_band,corner_start_m,"
        "corner_constraint_m,corner_lower_speed_mps,corner_nominal_speed_mps,"
        "corner_upper_speed_mps,corner_required_deceleration_mps2,planning_deceleration_mps2,"
        "can_duty,can_motor_current_a,obstacle_requested_speed_mps,obstacle_planned_speed_mps,"
        "obstacle_profile_active,obstacle_plan_deceleration_mps2,turn_recovery_hold,predicted_release_speed_mps,"
        "runtime_maximum_speed_mps,centerline_xy_m",
        parameters);
      driving_log_started_ = std::chrono::steady_clock::now();
      RCLCPP_INFO(get_logger(), "Driving CSV: %s (%.1f Hz + state changes; parameters beside CSV)",
        driving_log_->path().c_str(), driving_log_rate_hz_);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "Driving log unavailable: %s", exception.what());
    }
  }

  void record_driving_log()
  {
    if (!driving_log_ || driving_log_->failed()) {return;}
    const auto steady = std::chrono::steady_clock::now();
    const bool changed = last_logged_state_ != last_stop_reason_ ||
      last_logged_motor_ != last_motor_mode_ || last_logged_signal_ != latest_signal_state_ ||
      last_logged_steering_held_ != steering_held_ ||
      last_logged_traffic_status_ != traffic_status() || last_logged_path_held_ != path_hold_active_ ||
      last_logged_longitudinal_phase_ != longitudinal_phase_ ||
      last_logged_obstacle_slowdown_active_ != obstacle_slowdown_active_;
    if (!changed && last_driving_log_ &&
      std::chrono::duration<double>(steady - *last_driving_log_).count() <
      1.0 / driving_log_rate_hz_) {return;}
    last_driving_log_ = steady;
    last_logged_state_ = last_stop_reason_;
    last_logged_motor_ = last_motor_mode_;
    last_logged_signal_ = latest_signal_state_;
    last_logged_steering_held_ = steering_held_;
    last_logged_traffic_status_ = traffic_status();
    last_logged_path_held_ = path_hold_active_;
    last_logged_longitudinal_phase_ = longitudinal_phase_;
    last_logged_obstacle_slowdown_active_=obstacle_slowdown_active_;
    const auto ns = now().nanoseconds();
    const auto age = [ns](const std::optional<std::int64_t> & stamp) {
        return stamp ? seconds(ns - *stamp) : kUnavailable;
      };
    DrivingLogFrame frame;
    frame.ros_ns = ns;
    frame.elapsed_sec = std::chrono::duration<double>(steady - driving_log_started_).count();
    frame.labels = {control_mode_, last_stop_reason_,
      traffic_status(), last_motor_mode_, lane_result_status_,
      latest_signal_state_ == 1 ? "RED" : latest_signal_state_ == 2 ? "GREEN" : "UNKNOWN",
      longitudinal_phase_};
    frame.values = {
      double(enabled_), double(vesc_connected_), double(traffic_stop_enabled_),
      double(traffic_red_), double(traffic_holding_), double(steering_held_),
      current_speed_mps_, latest_raw_speed_mps_, double(latest_erpm_),
      latest_target_speed_mps_, latest_target_speed_mps_ - current_speed_mps_,
      latest_pid_effort_, latest_desired_duty_, command_duty_, command_brake_current_,
      latest_raw_steering_angle_rad_, steering_angle_rad_, latest_servo_position_,
      latest_cross_track_error_m_, latest_heading_error_rad_, latest_curvature_per_m_,
      double(latest_stop_line_present_), latest_stop_line_raw_m_, latest_stop_line_corrected_m_,
      stop_remaining_.value_or(kUnavailable),
      stop_line_seen_ns_ ? last_observed_remaining_ : kUnavailable, age(stop_line_seen_ns_),
      double(stop_candidate_count_), double(stop_line_rejections_), double(traffic_reapproach_),
      latest_signal_score_, age(latest_signal_capture_ns_), double(latest_signal_accepted_),
      age(last_erpm_ns_), age(last_path_received_ns_), lane_capture_age_sec_.value_or(kUnavailable),
      age(path_capture_ns_), double(latest_lane_sequence_), double(path_.has_value()),
      path_ ? double(path_->points.size()) : 0.0,
      double(traffic_stop_active_), double(traffic_pass_committed_), double(traffic_red_from_green_),
      traffic_predicted_stop_distance_m_, traffic_predicted_overshoot_m_, age(traffic_red_seen_ns_),
      double(path_hold_active_), latest_feedforward_duty_, requested_brake_current_,
      required_deceleration_mps2_, measured_deceleration_mps2_,
      coast_deceleration_seen_ns_ ? coast_deceleration_mps2_ : kUnavailable,
      age(coast_deceleration_seen_ns_), coast_probe_elapsed_, double(traction_recovery_),
      double(terminal_tracking_active_), latest_brake_delay_sec_, latest_brake_rise_, double(obstacle_slowdown_active_),
      double(corner_braking_allowed_), double(corner_profile_active_),
      double(corner_speed_in_band_), corner_start_distance_m_, corner_constraint_distance_m_,
      corner_lower_speed_mps_, corner_nominal_speed_mps_, corner_upper_speed_mps_,
      corner_required_deceleration_mps2_, latest_planning_deceleration_mps2_,
      dynamics_duty_seen_ns_ && ns >= *dynamics_duty_seen_ns_ &&
      seconds(ns - *dynamics_duty_seen_ns_) <= 0.20 ? dynamics_duty_ : kUnavailable,
      dynamics_motor_current_seen_ns_ && ns >= *dynamics_motor_current_seen_ns_ &&
      seconds(ns - *dynamics_motor_current_seen_ns_) <= 0.20 ? dynamics_motor_current_ : kUnavailable,
      obstacle_requested_speed_, obstacle_planned_speed_.value_or(kUnavailable),
      double(obstacle_profile_active_), obstacle_plan_deceleration_, double(turn_recovery_hold_),
      predicted_release_speed_, maximum_speed_mps_};
    if (path_) {frame.path = path_->points;}
    driving_log_->enqueue(std::move(frame));
  }

  void log_status()
  {
    if (driving_log_ && driving_log_->failed()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "Driving CSV write failed: %s", driving_log_->path().c_str());
    }
    if (driving_log_ && driving_log_->dropped() > 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Driving CSV dropped rows=%llu (writer backlog/contention)",
        static_cast<unsigned long long>(driving_log_->dropped()));
    }
    const auto current_ns = now().nanoseconds();
    const std::string receive_age = last_path_received_ns_ ?
      std::to_string(seconds(current_ns - *last_path_received_ns_)) + "s" : "never";
    const std::string capture_age = lane_capture_age_sec_ ?
      std::to_string(*lane_capture_age_sec_) + "s" : "unknown";
    RCLCPP_INFO(get_logger(),
      "Auto status | state=%s | path_points=%zu | speed=%.2f/%.2fm/s | curvature=%.3f/m | "
      "cte=%+.3fm | heading=%+.1fdeg | guard=%s | raw/final_steering=%+.1f/%+.1fdeg | "
      "servo=%.3f | motor=%s | duty=%.4f | brake=%.2fA | longitudinal=%s | lane_rx=%zu | lane_status=%s | "
      "source_points=%zu | last_rx_age=%s | capture_age_at_rx=%s | obstacle_slowdown=%s",
      last_stop_reason_.c_str(), path_ ? path_->points.size() : 0U,
      current_speed_mps_, latest_target_speed_mps_, latest_curvature_per_m_,
      latest_cross_track_error_m_, latest_heading_error_rad_ * 180.0 / kPi,
      latest_direction_guard_used_ ? "on" : "off",
      latest_raw_steering_angle_rad_ * 180.0 / kPi, steering_angle_rad_ * 180.0 / kPi,
      latest_servo_position_, last_motor_mode_.c_str(), command_duty_, command_brake_current_,
      longitudinal_phase_.c_str(),
      lane_result_count_, lane_result_status_.c_str(), lane_result_points_,
      receive_age.c_str(), capture_age.c_str(),obstacle_slowdown_active_?"ON":"OFF");
    if (traffic_stop_enabled_) {
      RCLCPP_INFO(get_logger(), "Traffic stop | red_latched=%s | hold=%s | remaining=%s | phase=%s | line_rejected=%llu | signal_seen=%s",
        traffic_red_ ? "yes" : "no", traffic_holding_ ? "yes" : "no",
        stop_remaining_ ? std::to_string(*stop_remaining_).c_str() : "unknown",
        traffic_status(),
        static_cast<unsigned long long>(stop_line_rejections_), traffic_signal_seen_ ? "yes" : "no");
    }
  }

  bool driving_log_enabled_{false}, stop_steering_hold_enabled_{true}, steering_held_{false};
  double driving_log_rate_hz_{20.0};
  std::string driving_log_directory_, last_logged_state_, last_logged_motor_, last_logged_traffic_status_;
  std::unique_ptr<DrivingLog> driving_log_;
  std::chrono::steady_clock::time_point driving_log_started_;
  std::optional<std::chrono::steady_clock::time_point> last_driving_log_;
  int latest_signal_state_{0}, last_logged_signal_{-1}, latest_erpm_{0};
  bool latest_signal_accepted_{false}, latest_stop_line_present_{false};
  bool last_logged_steering_held_{false};
  bool last_logged_path_held_{false}, path_hold_active_{false};
  std::uint64_t latest_lane_sequence_{0};
  std::optional<std::int64_t> latest_signal_capture_ns_;
  double latest_signal_score_{kUnavailable}, latest_stop_line_raw_m_{kUnavailable};
  double latest_stop_line_corrected_m_{kUnavailable}, latest_raw_speed_mps_{kUnavailable};
  double latest_pid_effort_{kUnavailable}, latest_desired_duty_{kUnavailable};
  bool tunnel_enabled_{false},tunnel_active_{false},outside_avoidance_{false},outside_slowdown_{false},outside_traffic_{false};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tunnel_state_sub_;
  bool traffic_stop_enabled_{false}, traffic_red_{false}, traffic_holding_{false};
  bool traffic_range_stop_committed_{false};
  double traffic_stop_range_max_m_;
  bool traffic_stop_active_{false}, traffic_pass_committed_{false};
  bool traffic_green_seen_{false}, traffic_red_from_green_{false}, traffic_pass_enabled_{true};
  double traffic_pass_overshoot_m_{0.60};
  double traffic_predicted_stop_distance_m_{kUnavailable}, traffic_predicted_overshoot_m_{kUnavailable};
  std::optional<std::int64_t> traffic_red_seen_ns_;
  bool traffic_signal_seen_{false};
  std::string traffic_state_topic_;
  double traffic_state_timeout_, traffic_line_timeout_, traffic_margin_, traffic_front_offset_;
  double planning_deceleration_mps2_, traffic_response_time_, traffic_brake_gain_;
  double traffic_hold_current_;
  bool longitudinal_pid_enabled_{true}, curvature_speed_control_enabled_{false};
  bool corner_braking_allowed_{false};
  double corner_planning_acceleration_mps2_{0.70};
  double corner_planning_response_sec_{0.20}, corner_speed_band_width_mps_{0.15};
  double corner_coast_entry_margin_mps_{0.03}, corner_brake_entry_margin_mps_{0.10};
  double corner_brake_exit_margin_mps_{0.05}, corner_brake_decel_margin_mps2_{0.10};
  double corner_coast_probe_sec_{0.20};
  double turn_exit_curvature_{0.25}, turn_exit_angle_rad_{0.0}, turn_exit_confirm_sec_{0.20};
  double turn_exit_elapsed_{0.0};
  bool turn_recovery_hold_{false};
  int corner_confirm_frames_{2}, corner_release_frames_{3};
  int corner_detection_count_{0}, corner_miss_count_{0};
  bool corner_profile_active_{false};
  bool corner_speed_in_band_{false};
  double corner_start_distance_m_{0.0}, corner_constraint_distance_m_{0.0};
  double corner_lower_speed_mps_{0.0}, corner_nominal_speed_mps_{0.0};
  double corner_upper_speed_mps_{0.0}, corner_required_deceleration_mps2_{0.0};
  double latest_planning_deceleration_mps2_{kUnavailable};
  bool staged_control_enabled_{true}, service_brake_requested_{false}, traction_recovery_{false};
  bool departure_pending_{true};
  double longitudinal_start_duty_;
  bool terminal_tracking_active_{false};
  std::string longitudinal_phase_{"startup"}, last_logged_longitudinal_phase_;
  double drive_ff_offset_, drive_ff_slope_, drive_ff_fade_speed_, recovery_duty_rise_;
  double recovery_start_ramp_sec_, traffic_brake_urgent_rise_;
  double latest_brake_delay_sec_{kUnavailable}, latest_brake_rise_{kUnavailable};
  double deceleration_filter_sec_, traffic_coast_probe_sec_, traffic_brake_decel_hysteresis_;
  double traffic_brake_speed_hysteresis_, longitudinal_brake_decel_gain_;
  double longitudinal_brake_speed_gain_;
  double traffic_terminal_distance_, traffic_terminal_timeout_;
  double coast_probe_elapsed_{0.0}, measured_deceleration_mps2_{kUnavailable};
  double coast_deceleration_mps2_{0.0}, required_deceleration_mps2_{kUnavailable};
  double latest_feedforward_duty_{kUnavailable}, requested_brake_current_{0.0};
  std::optional<std::int64_t> coast_deceleration_seen_ns_;
  std::optional<std::int64_t> last_brake_command_ns_;
  bool longitudinal_zero_target_{false};
  double longitudinal_kp_, longitudinal_ki_, longitudinal_kd_, longitudinal_integral_limit_;
  double traffic_line_gate_, traffic_line_weight_, traffic_arrival_tolerance_, traffic_reapproach_speed_;
  int traffic_line_confirm_frames_, stop_candidate_count_{0};
  double stop_candidate_distance_{0.0}, last_observed_remaining_{0.0};
  std::int64_t stop_candidate_ns_{0};
  std::uint64_t stop_line_rejections_{0};
  bool traffic_reapproach_{false};
  std::string traffic_phase_{"inactive"};
  std::optional<double> stop_remaining_;
  std::optional<std::int64_t> stop_line_seen_ns_, stop_distance_updated_ns_, traffic_capture_ns_;
  std::unique_ptr<SpeedPid> longitudinal_pid_;
  rclcpp::Subscription<traffic_detection_test::msg::TrafficLightState>::SharedPtr traffic_sub_;
  bool enabled_{true};
  bool electrical_brake_enabled_{true};
  bool steering_servo_inverted_{true};
  std::string control_mode_, enable_topic_, lane_result_topic_, lane_result_frame_id_;
  std::string measured_erpm_topic_, connection_status_topic_, duty_topic_;
  std::string dynamics_duty_topic_, dynamics_motor_current_topic_;
  std::string brake_current_topic_, servo_position_topic_, command_duty_topic_;
  std::string command_brake_current_topic_, target_speed_topic_, current_speed_topic_;
  std::string corner_state_topic_;
  std::string curvature_topic_, steering_angle_topic_, cross_track_error_topic_;
  std::string heading_error_topic_, raw_steering_angle_topic_, command_servo_position_topic_;
  std::string input_to_control_decision_topic_;
  double control_rate_hz_, status_log_rate_hz_, path_timeout_sec_, path_hold_timeout_sec_;
  double path_capture_maximum_age_sec_, erpm_timeout_sec_, bev_x_max_m_, bev_y_max_m_;
  double bev_meter_per_pixel_, path_minimum_x_m_, path_maximum_x_m_, path_minimum_span_m_;
  double path_maximum_gap_m_, path_geometry_window_m_, stanley_gain_;
  double stanley_softening_speed_mps_, stanley_heading_lookahead_m_;
  double stanley_corner_heading_threshold_rad_, stanley_corner_opposing_correction_ratio_;
  double maximum_steering_angle_rad_, steering_current_weight_, steering_rate_limit_rad_per_sec_;
  double servo_left_, servo_center_, servo_right_, minimum_speed_mps_, maximum_speed_mps_;
  double maximum_lateral_acceleration_mps2_, curvature_percentile_, minimum_duty_, maximum_duty_;
  double duty_rise_rate_per_sec_, duty_fall_rate_per_sec_, speed_pid_kp_, speed_pid_ki_;
  double speed_pid_kd_, speed_pid_integral_limit_, speed_filter_time_constant_sec_;
  double brake_entry_speed_error_mps_, brake_exit_speed_error_mps_;
  double brake_minimum_vehicle_speed_mps_, brake_minimum_current_amps_;
  double brake_maximum_current_amps_, brake_current_gain_amps_per_mps_;
  double brake_current_rise_amps_per_sec_, brake_current_fall_amps_per_sec_;
  double wheel_diameter_m_, erpm_direction_sign_, speed_scale_correction_;
  int path_minimum_points_, motor_pole_pairs_, motor_pinion_teeth_, spur_gear_teeth_;
  int differential_pinion_teeth_, differential_ring_teeth_;

  bool obstacle_slowdown_enabled_{true},obstacle_slowdown_active_{false};
  bool last_logged_obstacle_slowdown_active_{false};
  double obstacle_slowdown_speed_mps_{1.0},obstacle_slowdown_clear_sec_{1.0};
  double obstacle_planning_deceleration_{0.35}, obstacle_planning_acceleration_{0.35};
  double obstacle_coast_probe_sec_{0.20}, obstacle_brake_decel_margin_{0.10};
  double obstacle_brake_entry_margin_{0.10}, obstacle_brake_exit_margin_{0.03};
  double brake_release_response_sec_{0.15}, predicted_release_speed_{kUnavailable};
  double obstacle_requested_speed_{0.0}, obstacle_plan_deceleration_{0.0};
  std::optional<double> obstacle_planned_speed_;
  bool obstacle_profile_active_{false};
  std::optional<std::chrono::steady_clock::time_point> obstacle_last_seen_at_;
  std::optional<std::int64_t> obstacle_last_depth_ns_;
  bool avoidance_control_enabled_{false};
  bool avoidance_deformation_only_{true};
  bool avoidance_follow_centerline_{false};
  double avoidance_max_age_{.20},avoidance_brake_current_{2.5},avoidance_wheelbase_{.33};
  int avoidance_confirmations_{0};
  std::string avoidance_status_{"waiting_for_plan"};
  std::optional<OrderedPath> avoidance_path_;
  std::optional<std::int64_t> avoidance_lane_ns_,avoidance_depth_ns_;
  std::chrono::steady_clock::time_point avoidance_received_at_;
  rclcpp::Subscription<auto_control::msg::AvoidancePlan>::SharedPtr avoidance_sub_;
  std::optional<OrderedPath> path_;
  std::optional<std::int64_t> last_path_received_ns_, path_capture_ns_, last_erpm_ns_;
  std::optional<std::int64_t> dynamics_duty_seen_ns_, dynamics_motor_current_seen_ns_;
  double dynamics_duty_{0.0}, dynamics_motor_current_{0.0};
  std::optional<std::int64_t> last_valid_path_received_ns_;
  LaneCaptureOrder lane_capture_order_;
  std::optional<double> lane_capture_age_sec_;
  std::int64_t last_control_ns_{0};
  std::size_t lane_result_count_{0U}, lane_result_points_{0U};
  std::string lane_result_status_{"waiting_for_lane_result"};
  bool vesc_connected_{false}, brake_mode_active_{false}, latest_direction_guard_used_{false};
  double current_speed_mps_{0.0}, command_duty_{0.0}, command_brake_current_{0.0};
  double steering_angle_rad_{0.0}, latest_target_speed_mps_{0.0}, latest_curvature_per_m_{0.0};
  double latest_cross_track_error_m_{0.0}, latest_heading_error_rad_{0.0};
  double latest_raw_steering_angle_rad_{0.0}, latest_servo_position_{0.0};
  std::string last_motor_mode_{"stop"}, last_stop_reason_{"startup"};
  std::unique_ptr<RuntimeSpeedLimit> runtime_speed_limit_;
  rclcpp::Publisher<line_detactor::msg::SpeedLimitState>::SharedPtr speed_limit_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_limit_request_sub_;
  rclcpp::TimerBase::SharedPtr speed_limit_state_timer_;
  std::unique_ptr<SpeedPid> speed_pid_;
  std::unique_ptr<AutomaticBrakeProfile> brake_profile_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr duty_pub_, brake_pub_, servo_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr command_duty_pub_, command_brake_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_speed_pub_, current_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr corner_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr curvature_pub_, steering_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cross_track_pub_, heading_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr raw_steering_pub_, command_servo_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr input_to_control_decision_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr result_to_control_decision_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr preview_snapshot_pub_;
  rclcpp::TimerBase::SharedPtr preview_snapshot_timer_;
  std::chrono::steady_clock::time_point preview_meters_drained_at_{};
  bev_handoff::StageMeter preview_infer_meter_, preview_post_meter_, preview_control_meter_, preview_valid_lane_meter_;
  std::string preview_stages_json_{"{}"};
  std::string latest_lane_source_;
  std::uint8_t latest_lane_state_{0U};
  double latest_duty_command_{0.0}, latest_brake_command_{0.0};
  rclcpp::Subscription<line_detactor::msg::LaneResult>::SharedPtr lane_sub_, tunnel_ir_lane_sub_;
  std::map<const rclcpp::PublisherBase *, bool> diagnostic_subscriptions_;
  std::chrono::steady_clock::time_point diagnostic_subscriptions_checked_at_{};
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr erpm_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr dynamics_duty_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr dynamics_motor_current_sub_;
  bool model_startup_check_required_{false};
  bool startup_safety_enabled_{false}, camera_startup_ready_{false}, startup_manual_{false};
  std::string startup_drive_mode_{"AUTO"};
  StartAuthorization start_authorization_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr startup_state_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr camera_startup_sub_, startup_mode_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr startup_start_service_;
  rclcpp::TimerBase::SharedPtr startup_state_timer_;
  ModelStartupGate model_startup_gate_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr model_ready_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr connection_sub_, enable_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_, status_timer_;
};

}  // namespace auto_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<auto_control::AutoControlNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("auto_control"), "%s", exception.what());
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return 1;
  }
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return 0;
}
