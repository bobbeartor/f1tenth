#include <bev_handoff/runtime_monitor.hpp>
#include <map>
#pragma once
#include "auto_control/avoidance_planner.hpp"
#include "auto_control/capture_pair_order.hpp"
#include "auto_control/msg/avoidance_plan.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <std_msgs/msg/string.hpp>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace auto_control
{
// No subscriptions or waits are added to the lane inference/control thread.
class AvoidanceWorker
{
public:
  explicit AvoidanceWorker(rclcpp::Node & node):node_(node)
  {
    rcl_interfaces::msg::ParameterDescriptor mode; mode.read_only=true;
    mode.description="Startup-only mode; deformation mode falls back to the original centerline";
    control_requested_=node_.declare_parameter<bool>("obstacles.avoidance.control_requested",false,mode);
    avoidance::Options o;
    o.deformation_only=node_.declare_parameter<bool>("obstacles.avoidance.deformation_only",true,mode);
    deformation_only_=o.deformation_only;
    // auto_drive imports these from auto_control's existing path settings.
    o.minimum_points=node_.declare_parameter<int>("obstacles.avoidance.path_minimum_points",8,mode);
    parameter("path_minimum_span_m",o.minimum_span);
    parameter("path_minimum_x_m",o.minimum_x); parameter("path_maximum_x_m",o.maximum_x);
    parameter("path_maximum_gap_m",o.maximum_gap); parameter("path_geometry_window_m",o.geometry_window);
    o.centerline_fallback=node_.declare_parameter<bool>(
      "obstacles.avoidance.centerline_fallback_enabled",true,mode);
    parameter("vehicle_half_width_m",o.half_width); parameter("vehicle_half_length_m",o.half_length);
    parameter("obstacle_size_m",o.obstacle_size);
    parameter("safety_margin_m",o.margin); parameter("unknown_extent_m",o.unknown_extent);
    parameter("sample_step_m",o.step); parameter("max_offset_m",o.max_offset);
    parameter("transition_m",o.transition);
    parameter("max_curvature_per_m",o.max_curvature);
    // Imported from the controller for preview and checked-mode geometry only.
    parameter("reference_speed_mps",o.reference_speed);
    rcl_interfaces::msg::ParameterDescriptor legacy_speed; legacy_speed.read_only=true;
    legacy_speed.description="Deprecated and ignored; use auto_control.maximum_speed_mps";
    node_.declare_parameter<double>("obstacles.avoidance.max_speed_mps",.5,legacy_speed);
    parameter("lateral_acceleration_mps2",o.lateral_acceleration); parameter("deceleration_mps2",o.deceleration);
    parameter("clear_confirm_sec",o.clear_sec);
    // Read old files without retaining the global-offset enumeration behavior.
    double legacy_offset_step=.05,legacy_stop_response=.35;
    parameter("offset_step_m",legacy_offset_step); parameter("stop_response_sec",legacy_stop_response);
    parameter("max_age_sec",max_age_); parameter("max_sync_sec",max_sync_);
    parameter("max_fps",max_fps_);configured_fps_=max_fps_;
    avoidance::validate(o);
    if (!std::isfinite(max_age_)||max_age_<.05||max_age_>.3||!std::isfinite(max_sync_)||
      max_sync_<.001||max_sync_>.1||!std::isfinite(max_fps_)||max_fps_<1||max_fps_>80) {
      throw std::invalid_argument("avoidance: max_age_sec must be 0.05..0.30, max_sync_sec 0.001..0.10, max_fps 1..80");
    }
    const double raw_margin=o.margin;
    // Bound uncorrected capture-to-control movement in low-speed apply mode.
    o.control=control_requested_;
    if (o.control && !o.deformation_only) {
      o.motion_margin=o.reference_speed*.20;
      o.margin+=.01+.5*o.max_curvature*o.motion_margin*o.motion_margin;
    }
    options_=o;
    extra_margin_=o.margin-raw_margin;
    planner_=std::make_unique<avoidance::Planner>(o);
    monitor_=std::make_unique<bev_handoff::RuntimeMonitor>(node_);
    RCLCPP_INFO(node_.get_logger(),"AVOIDANCE mode=%s",o.deformation_only?"DEFORM (nonblocking centerline offsets)":"CHECKED");
    RCLCPP_INFO(node_.get_logger(),
      "AVOIDANCE geometry: model=fixed-square-at-detected-point vehicle_width=%.3fm half_length=%.3fm effective_margin=%.3fm "
      "motion_margin=%.3fm unknown_extent=%.3fm obstacle_size=%.3fm "
      "nominal_width_with_margin=%.3fm local_transition/max_offset=%.3f/%.3fm "
      "curvature_limit=%.3f/m reference_speed=%.3fm/s (display/geometry only)",
      2*o.half_width,o.half_length,o.margin,o.motion_margin,o.unknown_extent,o.obstacle_size,
      2*(o.half_width+o.margin),o.transition,o.max_offset,o.max_curvature,o.reference_speed);
    // Read on the worker thread: ros2 param set can enable/disable at runtime.
    node_.declare_parameter<bool>("obstacles.avoidance.enabled",false);
    status_=node_.create_publisher<std_msgs::msg::String>("/auto/avoidance_preview/status",rclcpp::QoS(1));
    plans_=node_.create_publisher<auto_control::msg::AvoidancePlan>("/auto/avoidance_plan",rclcpp::QoS(1).best_effort());
    bev_handoff::setAvoidanceDeformationOnly(deformation_only_);
    bev_handoff::setAvoidanceControlRequested(control_requested_);
    node_.declare_parameter<std::vector<std::string>>("runtime_editable_parameters",{
"obstacles.avoidance.vehicle_half_width_m","obstacles.avoidance.vehicle_half_length_m","obstacles.avoidance.obstacle_size_m","obstacles.avoidance.safety_margin_m","obstacles.avoidance.unknown_extent_m","obstacles.avoidance.sample_step_m","obstacles.avoidance.max_offset_m","obstacles.avoidance.transition_m","obstacles.avoidance.max_curvature_per_m","obstacles.avoidance.lateral_acceleration_mps2","obstacles.avoidance.deceleration_mps2","obstacles.avoidance.clear_confirm_sec","obstacles.avoidance.max_fps","obstacles.avoidance.enabled"});
    callback_=node_.add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& values) {
      rcl_interfaces::msg::SetParametersResult result;result.successful=false;
      std::lock_guard<std::mutex> lock(mutex_);
      auto next=options_;double fps=configured_fps_;
      bool changed=false;
      for(const auto& value:values) {
        const auto name=value.get_name();
        const std::string prefix="obstacles.avoidance.";
        // This node also declares detector parameters later, once BEV calibration
        // arrives. Declaration invokes on-set callbacks too. Other parameter
        // families are owned by their descriptors / callbacks, not this planner.
        if(name.rfind(prefix,0)!=0)continue;
        changed=true;
        if(name=="obstacles.avoidance.enabled")continue;
        if(value.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result.reason="Startup-only parameter";return result;
        }
        const auto key=name.substr(prefix.size());
        const double v=value.as_double();
        if(!std::isfinite(v)){result.reason="Finite number required";return result;}
        if(key=="max_fps")fps=v;
        else if(live_options().count(key)) {
          next.*live_options().at(key)=v+(key=="safety_margin_m"?extra_margin_:0.);
        }
        else {result.reason="Startup-only parameter";return result;}
      }
      if(!changed) {result.successful=true;return result;}
      try {
        avoidance::validate(next);
        if(fps<1 || fps>80)throw std::invalid_argument("max_fps must be 1..80");
      }catch(const std::exception& error){result.reason=error.what();return result;}
      options_=next;configured_fps_=fps;configuration_dirty_=true;wake_.notify_all();
      result.successful=true;return result;
    });
    thread_=std::thread([this]() {run();});
  }
  ~AvoidanceWorker()
  {
    {std::lock_guard<std::mutex> lock(mutex_); stop_=true;}
    wake_.notify_all();
    if (thread_.joinable()) {thread_.join();}
    bev_handoff::setAvoidancePreviewEnabled(false);
    bev_handoff::setAvoidanceControlRequested(false);
  }
private:
  static const std::map<std::string,double avoidance::Options::*>& live_options() {
    static const std::map<std::string,double avoidance::Options::*> fields={
      {"vehicle_half_width_m",&avoidance::Options::half_width},
      {"vehicle_half_length_m",&avoidance::Options::half_length},
      {"obstacle_size_m",&avoidance::Options::obstacle_size},
      {"safety_margin_m",&avoidance::Options::margin},
      {"unknown_extent_m",&avoidance::Options::unknown_extent},
      {"sample_step_m",&avoidance::Options::step},
      {"max_offset_m",&avoidance::Options::max_offset},
      {"transition_m",&avoidance::Options::transition},
      {"max_curvature_per_m",&avoidance::Options::max_curvature},
      {"lateral_acceleration_mps2",&avoidance::Options::lateral_acceleration},
      {"deceleration_mps2",&avoidance::Options::deceleration},
      {"clear_confirm_sec",&avoidance::Options::clear_sec}};
    return fields;
  }
  void parameter(const std::string & name,double & value)
  {
    rcl_interfaces::msg::ParameterDescriptor d; d.read_only=true;
    d.description="Avoidance setting";
    if(live_options().count(name) || name=="max_fps")d.read_only=false;
    value=node_.declare_parameter<double>("obstacles.avoidance."+name,value,d);
  }
  void report(const std::string & status)
  {
    if (!rclcpp::ok()) {return;}
    const auto now=std::chrono::steady_clock::now();
    // First state/change is immediate; identical status uses a 1Hz heartbeat.
    if (status==last_status_ && now-last_status_at_<std::chrono::seconds(1)) {return;}
    last_status_=status;last_status_at_=now;
    std_msgs::msg::String message; message.data=status; status_->publish(message);
    RCLCPP_INFO_THROTTLE(node_.get_logger(),*node_.get_clock(),1000,
      "AVOIDANCE: %s (control_requested=%s)",status.c_str(),control_requested_?"true":"false");
  }
  void unavailable(const std::string & why)
  {
    if (!unavailable_ || unavailable_reason_!=why) {
      planner_->unavailable();bev_handoff::publishAvoidancePreview(nullptr);
    }
    unavailable_=true;unavailable_reason_=why;report(why);
    if (control_requested_ && rclcpp::ok()) {
      auto_control::msg::AvoidancePlan invalid;
      invalid.deformation_only=deformation_only_; invalid.control_ready=true; invalid.status=why; plans_->publish(invalid);
    }
  }
  void run()
  {
    using Clock=std::chrono::steady_clock;
    bool previous_enabled=false;
    auto next_start=Clock::now();
    std::optional<std::int64_t> previous_lane,previous_depth;
    while (rclcpp::ok()) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait_until(lock,next_start,[this]() {return stop_ || configuration_dirty_;});
        if(stop_) {break;}
      }
      if (!rclcpp::ok()) {break;}
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if(configuration_dirty_) {
          planner_=std::make_unique<avoidance::Planner>(options_);
          max_fps_=configured_fps_;configuration_dirty_=false;next_start=Clock::now();
          unavailable_=false;previous_lane.reset();previous_depth.reset();
        }
      }
      if(Clock::now()<next_start)continue;
      // Also bound OFF/stale/sync/error attempts and invalid-plan publication.
      next_start=Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1/max_fps_));
      try {
        const bool enabled=node_.get_parameter("obstacles.avoidance.enabled").as_bool();
        if (enabled!=previous_enabled) {
          planner_->reset();previous_lane.reset();previous_depth.reset();
          unavailable_=false;bev_handoff::setAvoidancePreviewEnabled(enabled);
          previous_enabled=enabled; report(enabled?"WAIT":"OFF");
        }
        if (!enabled) {if (control_requested_) {unavailable("OFF: planning disabled");} continue;}
        const auto lane=bev_handoff::latestPlanningLane();
        const auto now=Clock::now();
        const double lane_age=lane?std::chrono::duration<double>(now-lane->received_at).count():1e9;
        if (!lane || lane_age<0 || lane_age>max_age_) {unavailable("WAIT: stale lane"); continue;}
        const auto obstacles=bev_handoff::matchingObstacles(lane->header,max_sync_,max_age_);
        if (!obstacles) {unavailable("WAIT: depth sync"); continue;}
        if (obstacles->width!=lane->width || obstacles->height!=lane->height ||
          std::abs(obstacles->x_max-lane->x_max)>1e-5 || std::abs(obstacles->y_max-lane->y_max)>1e-5 ||
          std::abs(obstacles->meter_per_pixel-lane->meter_per_pixel)>1e-5) {
          unavailable("WAIT: geometry mismatch"); continue;
        }
        const auto capture=[](const std_msgs::msg::Header& h) {
          return std::int64_t(h.stamp.sec)*1000000000LL+h.stamp.nanosec;
        };
        const auto lane_capture=capture(lane->header),depth_capture=capture(obstacles->header);
        if(capture_pair_order(lane_capture,depth_capture,previous_lane,previous_depth)!=CapturePairOrder::New)continue;
        previous_lane=lane_capture;previous_depth=depth_capture;
        bev_handoff::RuntimeSample runtime_sample(monitor_.get(),"avoidance_plan");
        const auto planning_started=Clock::now();
        auto result=std::make_shared<bev_handoff::AvoidancePreview>(planner_->plan(
          *lane,*obstacles,double(obstacles->header.stamp.sec)+obstacles->header.stamp.nanosec*1e-9));
        // Deformation planning already projected every cluster onto the lane;
        // repeat that search only when the plan did not evaluate membership.
        const bool lane_obstacle_detected=planner_->laneObstacle() ?
          *planner_->laneObstacle() : avoidance::hasInLaneObstacle(*lane,*obstacles);
        result->display_delta_sec=std::min(max_age_,1/max_fps_+max_sync_);
        const auto finished=Clock::now();
        if (std::chrono::duration<double>(finished-lane->received_at).count()>max_age_ ||
          std::chrono::duration<double>(finished-obstacles->captured_at).count()>max_age_) {
          unavailable("WAIT: planning expired"); continue;
        }
        RCLCPP_INFO_THROTTLE(node_.get_logger(),*node_.get_clock(),2000,
          "AVOIDANCE local: observed=%zu blocking=%zu zones=%zu attempts=%zu plan=%.2fms status=%s",
          obstacles->clusters.size(),result->obstacle_count,result->region_count,result->candidates.size(),
          std::chrono::duration<double,std::milli>(finished-planning_started).count(),result->status.c_str());
        if (result->selected.empty() && !result->candidates.empty()) {
          std::string failures;
          for (const auto & candidate:result->candidates) {
            if (!failures.empty()) {failures+="; ";}
            failures+=cv::format("transition=%.2fm offset_max=%.2fm:%s(k=%.2f)",
              candidate.transition_m,candidate.max_offset_m,candidate.reason.c_str(),candidate.max_curvature);
          }
          RCLCPP_INFO_THROTTLE(node_.get_logger(),*node_.get_clock(),2000,
            "AVOIDANCE rejected (first failed check per local profile): %s",failures.c_str());
        }
        result->control_requested=control_requested_;
        auto_control::msg::AvoidancePlan message;
        message.header=lane->header; message.depth_stamp=obstacles->header.stamp;
        message.control_ready=control_requested_;
        message.deformation_only=deformation_only_;
        message.lane_obstacle_detected=lane_obstacle_detected;
        message.follow_centerline=result->follow_centerline;
        message.valid=!result->selected.empty();
        message.status=result->status; message.speed_limit_mps=result->recommended_speed;
        message.max_curvature_per_m=result->max_curvature;
        if (!result->follow_centerline) {
          for (const auto & p:result->selected) {
            geometry_msgs::msg::Point32 point; point.x=p.x; point.y=p.y; message.points.push_back(point);
          }
        }
        plans_->publish(message);unavailable_=false;
        report(result->status); bev_handoff::publishAvoidancePreview(std::move(result));
      } catch (const std::exception & e) {
        unavailable("WAIT: planner error");
        RCLCPP_ERROR_THROTTLE(node_.get_logger(),*node_.get_clock(),5000,"Avoidance preview: %s",e.what());
      }
    }
  }
  avoidance::Options options_;
  double configured_fps_{80},extra_margin_{0};
  bool configuration_dirty_{false},unavailable_{false};
  std::string unavailable_reason_,last_status_;
  std::chrono::steady_clock::time_point last_status_at_{};
  std::unique_ptr<bev_handoff::RuntimeMonitor> monitor_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_;
  bool deformation_only_{true};
  rclcpp::Node & node_;
  std::unique_ptr<avoidance::Planner> planner_;
  bool control_requested_{false};
  rclcpp::Publisher<auto_control::msg::AvoidancePlan>::SharedPtr plans_;
  double max_age_{.25},max_sync_{.06},max_fps_{10};
  std::mutex mutex_;
  std::condition_variable wake_;
  bool stop_{false};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  std::thread thread_;
};
}
