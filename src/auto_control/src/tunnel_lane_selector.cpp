// C++ composable replacement for scripts/tunnel_lane_selector (auto_control_tunnel/node.py).
// Loaded into the detector container, so RGB/IR LaneResult arrive intra-process and only the
// forwarded result crosses to the controller process.
#include <array>
#include <chrono>
#include <cstdio>
#include <functional>
#include <optional>
#include <vector>
#include <memory>
#include <string>

#include "auto_control/tunnel_selection.hpp"
#include "line_detactor/msg/lane_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

namespace auto_control
{

using line_detactor::msg::LaneResult;
using std_msgs::msg::String;

class TunnelLaneSelector : public rclcpp::Node
{
public:
  explicit TunnelLaneSelector(const rclcpp::NodeOptions & options)
  : rclcpp::Node("tunnel_lane_selector", options),
    policy_(make_policy())
  {
    const auto & exit = policy_.ir_length_exit;
    RCLCPP_INFO(get_logger(), "IR length exit: enabled=%s ratio=%.3f confirm_frames=%d minimum_reference_m=%.2f",
      exit.enabled() ? "True" : "False", exit.threshold(), exit.confirm_frames(), exit.minimum_reference());

    const auto sensor = rclcpp::QoS(1).best_effort();
    // Humble intra-process cannot carry transient_local; latched status stays inter-process.
    rclcpp::PublisherOptions latched_options;
    latched_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    rgb_output_ = create_publisher<LaneResult>("/tunnel/lane_result/rgb", sensor);
    ir_output_ = create_publisher<LaneResult>("/tunnel/lane_result/ir", sensor);
    for (const char * key : {"state", "target_source", "selected_source", "compute_source"}) {
      status_.push_back(create_publisher<String>(std::string("/tunnel/") + key, latched, latched_options));
    }
    preview_pub_ = create_publisher<String>("/tunnel/preview_status", latched, latched_options);
    preview_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {publish_preview();});

    rclcpp::SubscriptionOptions inter_process;
    inter_process.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    tcn_sub_ = create_subscription<String>("/tunnel_tcn/result", sensor,
        std::bind(&TunnelLaneSelector::on_event, this, std::placeholders::_1), inter_process);
    rgb_sub_ = create_subscription<LaneResult>(rgb_topic_, sensor,
        [this](LaneResult::ConstSharedPtr message) {on_lane("RGB", message);});
    ir_sub_ = create_subscription<LaneResult>("/ir_bev_line_detection/result", sensor,
        [this](LaneResult::ConstSharedPtr message) {on_lane("IR", message);});
    publish_status();
  }

private:
  tunnel::LaneSelector make_policy()
  {
    tunnel::PathRules rules;
    rules.frame_id = declare_parameter<std::string>("lane_result_frame_id", "front_axle_bev");
    rules.minimum_points = declare_parameter<int>("path_minimum_points", 8);
    rules.minimum_span = declare_parameter<double>("path_minimum_span_m", .12);
    rules.minimum_x = declare_parameter<double>("path_minimum_x_m", .05);
    rules.maximum_x = declare_parameter<double>("path_maximum_x_m", 2.20);
    rules.maximum_gap = declare_parameter<double>("path_maximum_gap_m", .15);
    rules.x_max = declare_parameter<double>("bev_x_max_m", 3.);
    rules.y_max = declare_parameter<double>("bev_y_max_m", .6);
    rules.mpp = declare_parameter<double>("bev_meter_per_pixel", .01);
    tunnel::IrLengthExit exit(
      declare_parameter<bool>("ir_length_exit_enabled", false),
      declare_parameter<double>("ir_length_exit_ratio", .15),
      declare_parameter<int>("ir_length_exit_confirm_frames", 3),
      declare_parameter<double>("ir_length_exit_minimum_reference_m", .5));
    rgb_topic_ = declare_parameter<std::string>("rgb_result_topic", "/line_detactor/result");
    return tunnel::LaneSelector(declare_parameter<double>("max_age_sec", .2),
      declare_parameter<bool>("validation_mode", false), rules, exit);
  }

  static double monotonic()
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void publish_status()
  {
    const std::array<std::string, 4> values{policy_.state, policy_.camera(), policy_.selected, policy_.compute()};
    if (published_ && *published_ == values) {return;}
    for (std::size_t i = 0; i < values.size(); ++i) {
      String message;
      message.data = values[i];
      status_[i]->publish(message);
    }
    published_ = values;
    RCLCPP_INFO(get_logger(), "Tunnel %s | selected=%s compute=%s camera=%s reason=%s",
      values[0].c_str(), values[2].c_str(), values[3].c_str(), values[1].c_str(), policy_.reason.c_str());
    if (!policy_.last_handover.empty() && !policy_.transitioning()) {
      RCLCPP_INFO(get_logger(), "Tunnel handover %s", policy_.last_handover.c_str());
    }
    publish_preview();
  }

  void publish_preview()
  {
    String message;
    message.data = preview_.text(policy_.state, monotonic());
    preview_pub_->publish(message);
  }

  void on_event(const String::ConstSharedPtr message)
  {
    try {
      const auto report = tunnel::json_object(message->data);
      const auto stamp = tunnel::json_int(report, "source_stamp_ns");
      const std::int64_t now_ns = now().nanoseconds();
      if (!stamp) {return;}
      const double age = (now_ns - *stamp) / 1e9;
      if (!policy_.validation_mode && !(0 <= age && age <= .25)) {return;}
      preview_.update(report, monotonic());
      last_tcn_received_ = monotonic();
      const std::string state = tunnel::json_string(report, "state").value_or("");
      const std::string status = tunnel::json_string(report, "status").value_or("");
      tcn_live_ = status.rfind("LIVE", 0) == 0 && (state == "inside" || state == "outside");
      if (!tcn_live_ || !policy_.observe_tcn_state(state, *stamp, now_ns)) {return;}
      const std::string requested = state == "inside" ? "IR" : "RGB";
      // Event packets may be lost with sensor QoS; the TCN retains its accepted state in
      // later reports, so reconcile that state too. Matching reports do not repeat a change.
      if (requested == policy_.requested) {return;}
      const std::string kind = requested == "IR" ? "entry" : "exit";
      const auto event = report.find("event");
      if (event != report.end() && !event->second.empty() && event->second.front() == '{') {
        const auto type = tunnel::json_string(tunnel::json_object(event->second), "type");
        if (type && *type != kind) {return;}
      }
      if (policy_.event(kind, *stamp, now_ns)) {publish_status();}
    } catch (const std::exception &) {
      RCLCPP_WARN(get_logger(), "Ignoring malformed TCN result");
    }
  }

  void on_lane(const std::string & source, const LaneResult::ConstSharedPtr & message)
  {
    // Validation follows the last selected source independently of TCN health. TCN only switches.
    if (!policy_.validation_mode && (!tcn_live_ || !last_tcn_received_ || monotonic() - *last_tcn_received_ > .5)) {
      return;
    }
    const bool previous_latch = policy_.ir_exit_latched_at.has_value();
    const bool forward = policy_.lane(source, *message, now().nanoseconds());
    const auto & monitor = policy_.ir_length_exit;
    const bool triggered = !previous_latch && policy_.ir_exit_latched_at.has_value();
    if (monitor.enabled() && source == "IR" && (triggered || monotonic() - last_ir_length_log_ >= 1.)) {
      last_ir_length_log_ = monotonic();
      const auto fmt = [](std::optional<double> v) {
          if (!v) {return std::string("--");}
          char text[32];
          std::snprintf(text, sizeof(text), "%.3f", *v);
          return std::string(text);
        };
      RCLCPP_INFO(get_logger(), "[IR LENGTH EXIT] current_m=%s max_m=%.3f ratio=%s threshold=%.3f "
        "low_frames=%d/%d triggered=%s", fmt(monitor.current()).c_str(), monitor.reference(),
        fmt(monitor.ratio()).c_str(), monitor.threshold(), monitor.below_count(), monitor.confirm_frames(),
        triggered ? "True" : "False");
    }
    publish_status();
    if (forward) {
      // Unmodified detector output; the controller learns the source from the topic.
      (source == "IR" ? ir_output_ : rgb_output_)->publish(*message);
    }
  }

  std::string rgb_topic_;
  tunnel::LaneSelector policy_;
  tunnel::PreviewTelemetry preview_;
  std::optional<std::array<std::string, 4>> published_;
  std::optional<double> last_tcn_received_;
  bool tcn_live_{false};
  double last_ir_length_log_{0.};
  rclcpp::Publisher<LaneResult>::SharedPtr rgb_output_, ir_output_;
  std::vector<rclcpp::Publisher<String>::SharedPtr> status_;
  rclcpp::Publisher<String>::SharedPtr preview_pub_;
  rclcpp::TimerBase::SharedPtr preview_timer_;
  rclcpp::Subscription<String>::SharedPtr tcn_sub_;
  rclcpp::Subscription<LaneResult>::SharedPtr rgb_sub_, ir_sub_;
};

}  // namespace auto_control

RCLCPP_COMPONENTS_REGISTER_NODE(auto_control::TunnelLaneSelector)
