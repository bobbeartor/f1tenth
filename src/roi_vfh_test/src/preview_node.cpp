#include "roi_vfh_test/planner.hpp"
#include "roi_vfh_test/motion.hpp"
#include "roi_vfh_test/hazard_filter.hpp"
#include "roi_vfh_test/obstacle_memory.hpp"
#include "roi_vfh_test/policy_state.hpp"
#include "roi_vfh_test/speed_policy.hpp"
#include "roi_vfh_test/render.hpp"
#include "roi_vfh_test/drive.hpp"
#include "roi_vfh_test/controls.hpp"
#include <auto_control/driving_log.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <line_detactor/msg/lane_result.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <cstring>
#include <thread>
namespace roi_vfh_test {
class Node:public rclcpp::Node {
  Config config_; Planner planner_; Plan result_;TargetSpeedPolicy speed_policy_;
  LaneMemory lane_memory_;LaneReference lane_reference_;
  PlannerControls controls_;
  HazardFilter hazard_filter_;HazardStats hazard_stats_;
  ObstacleMemory obstacle_memory_;
  std::uint64_t hazard_motion_generation_{0},terrain_cache_revision_{0};
  std::optional<HazardResult> terrain_cache_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr terrain_cache_input_;
  bool terrain_cache_motion_ready_{false};
  struct TimingAverage {
    std::deque<double> samples;
    double sum{0};

    void add(double milliseconds) {
      samples.push_back(milliseconds);
      sum+=milliseconds;
      if (samples.size()>30) {
        sum-=samples.front();
        samples.pop_front();
      }
    }
    std::string display() const {
      if (samples.empty()) {return "--";}
      return cv::format("%.2f",sum/samples.size());
    }
  };
  TimingAverage filter_average_,plan_average_;
  std::array<double,4> roi_timing_{};
  std::optional<std::chrono::steady_clock::time_point> roi_timing_received_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr roi_timing_sub_;
  double hazard_filter_ms_{0};
  double planner_deceleration_ceiling_{.5},speed_ceiling_{1.8};
  double steering_ceiling_{pi/3},steering_rate_ceiling_{20};
  DriveOptions drive_options_base_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr planner_parameters_callback_;
  bool drive_enabled_{false},vesc_connected_{false},servo_commanded_{false};
  double drive_max_age_{.30};
  std::unique_ptr<DriveController> drive_;
  DriveCommand drive_command_;
  std::chrono::steady_clock::time_point last_drive_tick_{std::chrono::steady_clock::now()};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr duty_pub_,brake_pub_,servo_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr duty_log_pub_,brake_log_pub_,servo_log_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr connection_sub_;
  bool manual_handover_{false},manual_requested_{false};
  double handover_default_stop_current_{2.0};
  std::string handover_token_,handover_mode_{"WAIT"};
  std::uint64_t handover_sequence_{0};
  std::optional<std::chrono::steady_clock::time_point> handover_ack_at_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr handover_request_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr handover_ack_sub_;
  MotionHistory motion_;
  std::uint64_t motion_generation_{0};
  double imu_received_at_{-1e9};
  std::string imu_issue_{"no messages received"};
  double measured_yaw_rate_{0},measured_speed_{0},speed_at_{-1e9},imu_at_{-1e9};
  bool motion_ok_{false};
  MotionPose previous_pose_;bool previous_pose_valid_{false};
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  // Only the sensor executor writes this inbox. Planning/GUI and actuator
  // callbacks use a coherent copy, without blocking sensor integration while
  // filtering, planning or pumping the window event loop.
  struct SensorInbox {
    MotionHistory motion;
    double imu_received_at{-1e9},imu_at{-1e9},speed_at{-1e9},lane_received_at{-1e9};
    double speed{0},yaw_rate{0};
    bool motion_ok{false};
    std::string imu_issue{"no messages received"};
    std::deque<nav_msgs::msg::OccupancyGrid::ConstSharedPtr> grids;
    line_detactor::msg::LaneResult::ConstSharedPtr lane;
  };
  std::mutex sensor_mutex_;
  SensorInbox sensor_inbox_;
  rclcpp::CallbackGroup::SharedPtr sensor_callbacks_;
  bool gui_{true},requested_{false},locked_{false},window_{false};
  bool visible_supported_{false},autosize_supported_{false};
  bool suspended_{false},input_recoverable_{true},waiting_for_pair_{false};
  double resume_stable_sec_{.5};
  std::optional<std::chrono::steady_clock::time_point> healthy_since_,last_recovery_check_;
  std::optional<double> planned_lane_stamp_;
  line_detactor::msg::LaneResult::ConstSharedPtr planned_lane_;
  double max_age_{.30},plan_ms_{0};
  double lane_receipt_timeout_{.30},erpm_timeout_{.15},imu_timeout_{.25},lane_received_at_{-1e9};
  double planning_rate_{80},status_rate_{2},preview_rate_{30};
  std::optional<std::chrono::steady_clock::time_point> last_preview_at_;
  int max_lane_points_{2000};
  std::string frame_id_{"front_axle_bev"},lane_topic_{"/line_detactor/result"};
  std::deque<nav_msgs::msg::OccupancyGrid::ConstSharedPtr> grids_;
  line_detactor::msg::LaneResult::ConstSharedPtr lane_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr last_grid_;
  std::vector<Point> lane_points_; Grid display_;
  std::string status_{"STOPPED: configure ROI, then Start"};
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<line_detactor::msg::LaneResult>::SharedPtr lane_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lock_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr running_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_,steer_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_,actuator_timer_;
  std::unique_ptr<auto_control::DrivingLog> driving_log_;
  double driving_log_rate_{30},last_log_at_{-1e9};
  double obstacle_speed_cap_{1},obstacle_clear_sec_{1};
  bool obstacle_slowdown_{true};
  std::string last_log_state_;
  std::chrono::steady_clock::time_point log_started_{std::chrono::steady_clock::now()};
  static double stamp(const builtin_interfaces::msg::Time & t) {return t.sec+t.nanosec*1e-9;}
  bool fresh(const builtin_interfaces::msg::Time & t) {const double age=now().seconds()-stamp(t);return age>=-.02 && age<=max_age_;}
  void snapshotSensors() {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    motion_=sensor_inbox_.motion;motion_ok_=sensor_inbox_.motion_ok;
    imu_received_at_=sensor_inbox_.imu_received_at;imu_at_=sensor_inbox_.imu_at;
    imu_issue_=sensor_inbox_.imu_issue;measured_yaw_rate_=sensor_inbox_.yaw_rate;
    measured_speed_=sensor_inbox_.speed;speed_at_=sensor_inbox_.speed_at;
    grids_=sensor_inbox_.grids;lane_=sensor_inbox_.lane;lane_received_at_=sensor_inbox_.lane_received_at;
  }
  void publishLock(bool value) {std_msgs::msg::Bool m;m.data=value;running_pub_->publish(m);}
  void requestDriveMode(bool manual) {
    if(!manual_handover_)return;
    manual_requested_=manual;handover_ack_at_.reset();handover_mode_="WAIT";
    handover_token_=std::to_string(now().nanoseconds())+"-"+std::to_string(++handover_sequence_);
    std_msgs::msg::String request;
    request.data=handover_token_+(manual?"|MANUAL":"|AUTO");
    // Keep the request readable by older installed handover nodes when the
    // launch already configured the selected stop current on the receiver.
    if(config_.drive_brake_max!=handover_default_stop_current_)
      request.data+="|"+std::to_string(config_.drive_brake_max);
    handover_request_pub_->publish(request);
  }
  bool automaticOutputReady()const {
    return !manual_handover_ || (!manual_requested_ && handover_mode_=="AUTO" && handover_ack_at_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now()-*handover_ack_at_).count()<.35);
  }
  void userStop() {
    stop("STOPPED: parameters unlocked");
    requestDriveMode(true);
  }
  void resetHazards() {
    hazard_filter_.reset();obstacle_memory_.reset();hazard_stats_={};terrain_cache_.reset();terrain_cache_input_.reset();
    hazard_motion_generation_=motion_.generation;
  }
  void resetPolicyState() {
    lane_memory_.reset();lane_reference_=LaneReference{};lane_points_.clear();
    planned_lane_stamp_.reset();
  }
  void stop(const std::string& reason) {
    controls_.blur();controls_.message("Stopped. ROI and VFH parameters unlocked.");
    requested_=false;locked_=false;suspended_=false;healthy_since_.reset();last_recovery_check_.reset();planned_lane_.reset();resetPolicyState();resetHazards();planner_.reset();previous_pose_valid_=false;result_=Plan{};last_grid_.reset();speed_policy_.reset();status_=reason;publishLock(false);applyDrive(false);
  }
  void start() {
    if(!applyPlannerEdits())return;
    controls_.show(false);
    requested_=true;locked_=false;suspended_=false;healthy_since_.reset();last_recovery_check_.reset();planned_lane_.reset();resetPolicyState();resetHazards();planner_.reset();previous_pose_valid_=false;result_=Plan{};last_grid_.reset();speed_policy_.reset();
    requestDriveMode(false);
    status_="WAIT: ROI parameter lock / automatic output acknowledgement";publishLock(true);
  }
  bool active()const {return requested_ && locked_ && !suspended_ && automaticOutputReady();}
  void pauseForInput(const std::string& reason) {
    if(!requested_ || !locked_)return;
    const bool entering=!suspended_;
    if(entering) {
      // Keep geometric memories while odometry remains continuous. A changed
      // motion generation clears them before the next plan.
      result_=Plan{};last_grid_.reset();planned_lane_.reset();resetHazards();planner_.reset();previous_pose_valid_=false;
      speed_policy_.reset();
    }
    suspended_=true;healthy_since_.reset();last_recovery_check_.reset();
    status_="WAIT input: "+reason;
    controls_.message("Input wait: parameters locked. Stop cancels automatic restart.");
    // The actuator timer keeps braking at its existing control rate. Only the
    // transition needs an extra immediate command from the planning loop.
    if(entering)applyDrive(false);
  }
  void updateRecovery(bool inputs_ok) {
    if(!suspended_ || !requested_ || !locked_)return;
    if(!inputs_ok || !last_grid_ || result_.speed<=0) {
      healthy_since_.reset();last_recovery_check_.reset();return;
    }
    const auto clock=std::chrono::steady_clock::now();
    if(last_recovery_check_ && std::chrono::duration<double>(clock-*last_recovery_check_).count()>std::max(.1,2/planning_rate_))
      healthy_since_.reset(); // A stalled event loop is not continuous healthy input.
    last_recovery_check_=clock;
    if(!healthy_since_)healthy_since_=clock;
    const double elapsed=std::chrono::duration<double>(clock-*healthy_since_).count();
    if(elapsed>=resume_stable_sec_) {
      suspended_=false;healthy_since_.reset();last_recovery_check_.reset();status_=result_.status;
      controls_.message("Inputs recovered. Driving resumed; parameters remain locked.");
    }else status_=cv::format("WAIT: inputs stable %.2f / %.2fs (braking)",elapsed,resume_stable_sec_);
  }
  static void mouse(int event,int x,int y,int,void * ptr) {
    if (event!=cv::EVENT_LBUTTONDOWN) return;
    auto & self=*static_cast<Node*>(ptr);
    if (stopButton().contains(cv::Point(x,y))) {self.userStop();self.publish();return;}
    if (PlannerControls::toggleButton().contains(cv::Point(x,y))) {self.controls_.toggle();return;}
    if (startButton().contains(cv::Point(x,y)) && !self.requested_) {self.start();return;}
    self.controlAction(self.controls_.click(x,y,self.requested_));
  }
  Config plannerParameters()const {
    auto result=config_;
    for(const auto& field:plannerFields())result.*(field.member)=get_parameter(field.name).as_double()/field.scale;
    return result;
  }
  void syncPlannerParameters() {
    const auto next=plannerParameters();bool changed=false;
    for(const auto& field:plannerFields())if(next.*(field.member)!=config_.*(field.member))changed=true;
    if(!changed)return;
    validatePlannerEdits(next,planner_deceleration_ceiling_,speed_ceiling_,steering_ceiling_,
      steering_rate_ceiling_,drive_options_base_.maximum_duty);
    auto updated=std::make_unique<DriveController>(tunedDriveOptions(drive_options_base_,next));
    updated->retainSteering(drive_command_.steer);drive_=std::move(updated);
    drive_command_=drive_->update(false,0,measured_speed_,0,.001);
    config_=next;resetPolicyState();resetHazards();planner_.reset();result_=Plan{};last_grid_.reset();previous_pose_valid_=false;
    controls_.sync(config_);
  }
  bool applyPlannerEdits() {
    if(requested_){controls_.message("Locked: press Stop before editing VFH parameters.");return false;}
    try {
      syncPlannerParameters();
      std::vector<rclcpp::Parameter> edits;
      for(const auto& change:controls_.changes())edits.emplace_back(change.first,change.second);
      if(!edits.empty()) {
        const auto result=set_parameters_atomically(edits);
        if(!result.successful)throw std::invalid_argument(result.reason);
      }
      syncPlannerParameters();controls_.reload(config_);controls_.message("Applied. Start uses these values.");return true;
    }catch(const std::exception& e) {
      controls_.show(true);controls_.message(std::string("REJECTED: ")+e.what());
      status_="STOPPED: invalid VFH parameters; correct highlighted edits";
      RCLCPP_WARN(get_logger(),"VFH settings rejected: %s",e.what());return false;
    }
  }
  void controlAction(PlannerControls::Action action) {
    if(requested_)return;
    if(action==PlannerControls::Action::Apply)applyPlannerEdits();
    if(action==PlannerControls::Action::Reload){controls_.reload(config_);controls_.message("Reloaded applied values. Draft edits discarded.");}
  }
  const HazardResult& confirmedTerrain(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& match) {
    if(hazard_motion_generation_!=motion_.generation)resetHazards();
    MotionPose selected_pose;
    const double selected_clock=now().seconds();
    const bool motion_ready=motion_ok_ && selected_clock-speed_at_<=erpm_timeout_ &&
      selected_clock-imu_at_<=imu_timeout_ && motion_.at(stamp(match->header.stamp),selected_pose);
    if(terrain_cache_ && terrain_cache_input_==match && terrain_cache_motion_ready_==motion_ready)
      return *terrain_cache_;
    // Process every available depth observation up to the selected capture, in
    // time order. A blank intervening frame breaks a weak hazard's streak even
    // if that frame was not chosen for the lane/depth synchronization pair.
    std::vector<nav_msgs::msg::OccupancyGrid::ConstSharedPtr> frames;
    for(const auto& g:grids_)if(stamp(g->header.stamp)<=stamp(match->header.stamp) && fresh(g->header.stamp))frames.push_back(g);
    std::stable_sort(frames.begin(),frames.end(),[](const auto& a,const auto& b){return stamp(a->header.stamp)<stamp(b->header.stamp);});
    HazardResult output;bool found=false;
    for(const auto& g:frames) {
      if(g->header.frame_id!=frame_id_ || std::abs(g->info.origin.orientation.x)>1e-6 ||
        std::abs(g->info.origin.orientation.y)>1e-6 || std::abs(g->info.origin.orientation.z)>1e-6 ||
        std::abs(g->info.origin.orientation.w-1)>1e-6) {hazard_filter_.reset();obstacle_memory_.reset();continue;}
      Grid raw;raw.width=g->info.width;raw.height=g->info.height;raw.resolution=g->info.resolution;
      raw.x0=g->info.origin.position.x;raw.y0=g->info.origin.position.y;raw.cells=g->data;
      MotionPose pose;std::optional<MotionPose> capture_pose;
      const double clock=now().seconds();
      if(motion_ok_ && clock-speed_at_<=erpm_timeout_ && clock-imu_at_<=imu_timeout_ && motion_.at(stamp(g->header.stamp),pose))capture_pose=pose;
      auto filtered=hazard_filter_.apply(raw,stamp(g->header.stamp),capture_pose,config_);
      filtered=obstacle_memory_.apply(filtered,stamp(g->header.stamp),capture_pose,config_);
      if(g==match){output=std::move(filtered);found=true;}
    }
    if(!found)throw std::invalid_argument("selected terrain missing from hazard confirmation input");
    terrain_cache_=std::move(output);terrain_cache_input_=match;terrain_cache_motion_ready_=motion_ready;
    ++terrain_cache_revision_;
    return *terrain_cache_;
  }
  bool inputs() {
    input_recoverable_=true;waiting_for_pair_=false;
    const bool lane_fresh=lane_ && now().seconds()-lane_received_at_<=lane_receipt_timeout_ && fresh(lane_->header.stamp);
    const auto lane_input=lane_fresh && lane_->centerline_valid && lane_->centerline_points.size()>=2 &&
      lane_->centerline_points.size()<=static_cast<std::size_t>(max_lane_points_) ? lane_ : nullptr;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr match,newest;
    const bool need_motion=requested_ && locked_;
    for(const auto& g:grids_) {
      if(!fresh(g->header.stamp))continue;
      if(!newest || stamp(g->header.stamp)>stamp(newest->header.stamp))newest=g;
      MotionPose pose;
      if(need_motion && (!motion_ok_ || !motion_.at(stamp(g->header.stamp),pose)))continue;
      if(!match || stamp(g->header.stamp)>stamp(match->header.stamp))match=g;
    }
    if(!match) {
      waiting_for_pair_=true;
      status_=newest?cv::format("STOP: terrain awaits motion alignment (terrain age %.3fs, IMU age %.3fs; %s)",
        now().seconds()-stamp(newest->header.stamp),now().seconds()-imu_at_,imu_issue_.c_str()):"STOP: no fresh terrain";
      return false;
    }
    // Use the current usable observation even when inference lags terrain.
    // Reproject by capture pose when possible; no time-skew/confidence veto.
    const auto paired_lane=lane_input;
    if (match->header.frame_id!=frame_id_ || (paired_lane && paired_lane->header.frame_id!=match->header.frame_id) ||
      std::abs(match->info.origin.orientation.x)>1e-6 || std::abs(match->info.origin.orientation.y)>1e-6 ||
      std::abs(match->info.origin.orientation.z)>1e-6 || std::abs(match->info.origin.orientation.w-1)>1e-6) {
      input_recoverable_=false;status_="STOP: coordinate/frame mismatch";return false;
    }
    if (paired_lane && (!paired_lane->source_width || !paired_lane->source_height || paired_lane->source_width>4096 || paired_lane->source_height>4096 || !std::isfinite(paired_lane->centerline_bev_width_m) ||
      !std::isfinite(paired_lane->centerline_bev_height_m) || paired_lane->centerline_bev_width_m<=0 || paired_lane->centerline_bev_height_m<=0)) {
      input_recoverable_=false;status_="STOP: invalid lane geometry";return false;
    }
    std::vector<Point> points;
    if (paired_lane) for (const auto & p:paired_lane->centerline_points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x<0 || p.x>=double(paired_lane->source_width)+paired_lane->padding_left+paired_lane->padding_right || p.y<0 || p.y>=paired_lane->source_height) {
        input_recoverable_=false;status_="STOP: invalid lane point";return false;
      }
      Point q{paired_lane->centerline_bev_height_m-(p.y+.5)*paired_lane->centerline_bev_height_m/paired_lane->source_height,
        paired_lane->centerline_bev_width_m/2-(p.x-paired_lane->padding_left+.5)*paired_lane->centerline_bev_width_m/paired_lane->source_width};
      points.push_back(q);
    }
    if(points.size()<2)points.clear();
    if(!requested_ || !locked_)lane_points_=points;
    const auto filter_begin=std::chrono::steady_clock::now();
    const auto cache_revision=terrain_cache_revision_;
    const auto& confirmed=confirmedTerrain(match);
    if(terrain_cache_revision_!=cache_revision) {
      hazard_filter_ms_=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-filter_begin).count();
      filter_average_.add(hazard_filter_ms_);
      display_=confirmed.grid;hazard_stats_=confirmed.stats;
    }
    if(requested_ && locked_ && now().seconds()-speed_at_>erpm_timeout_){status_="STOP: missing/stale measured speed (check VESC ERPM topic/bridge)";return false;}
    if(requested_ && locked_ && now().seconds()-imu_at_>imu_timeout_){status_="STOP: missing/stale vehicle-frame IMU (/roi_lidar/imu)";return false;}
    MotionPose pose;
    if(requested_ && locked_) {
      MotionPose lane_pose;
      if(!motion_ok_ || !motion_.at(stamp(match->header.stamp),pose)) {
        status_="STOP: motion history cannot align sensor captures";return false;
      }
      if(!points.empty() && motion_.at(stamp(paired_lane->header.stamp),lane_pose))
        for(auto& point:points)point=reproject(point,lane_pose,pose);
    }
    // Message instances can be re-published for the same exposure. Only a
    // changed capture pair (including lane loss/recovery) requires a new plan.
    const auto same_capture=[](const auto& a,const auto& b) {
      return (!a && !b) || (a && b && a->header.stamp==b->header.stamp);
    };
    if (requested_ && locked_ && (motion_generation_!=motion_.generation ||
        !same_capture(match,last_grid_) || !same_capture(paired_lane,planned_lane_))) {
      if(motion_generation_!=motion_.generation) {
        previous_pose_valid_=false;resetPolicyState();planner_.reset();motion_generation_=motion_.generation;
      }
      if(previous_pose_valid_)planner_.rotateReference(wrap(pose.yaw-previous_pose_.yaw));
      previous_pose_=pose;previous_pose_valid_=true;
      const auto begin=std::chrono::steady_clock::now();
      auto working=config_;
      working.delay=std::max(config_.delay,std::max(0.0,now().seconds()-stamp(match->header.stamp))+1.0/planning_rate_);
      working.current_speed=measured_speed_;
      working.current_steer=drive_enabled_ && servo_commanded_ ? drive_command_.steer :
        std::clamp(std::atan(config_.wheelbase*measured_yaw_rate_/std::max(.1,std::abs(measured_speed_))),-config_.max_steer,config_.max_steer);
      lane_reference_=lane_memory_.resolve(points,pose,working,
        paired_lane?std::optional<double>(stamp(paired_lane->header.stamp)):std::nullopt);
      lane_points_=lane_reference_.points;
      const bool lane_missing=!lane_reference_.trusted;
      if(lane_missing && !lane_reference_.has_memory)working.avoidance_lane_weight=0;
      result_=planner_.plan(display_,lane_points_,working,lane_missing);
      result_.status="LANE "+lane_reference_.state+" | "+result_.status;
      const double planned_at=now().seconds();
      speed_policy_.apply(result_,lane_missing,planned_at,measured_speed_,obstacle_slowdown_,
        obstacle_speed_cap_,obstacle_clear_sec_,config_.target_recovery_acceleration);
      plan_ms_=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
      plan_average_.add(plan_ms_);
      last_grid_=match;planned_lane_=paired_lane;planned_lane_stamp_.reset();
      if(!lane_missing)planned_lane_stamp_=stamp(paired_lane->header.stamp);
      status_=result_.status;
    }
    return true;
  }
  static void publishFloat(const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr& publisher,double value) {
    if(!publisher)return;
    std_msgs::msg::Float32 msg;msg.data=static_cast<float>(value);publisher->publish(msg);
  }
  void logDrive(bool active) {
    if(!driving_log_)return;
    const double clock=now().seconds();
    const std::string state=status_+(active?" active":" stopped")+(drive_command_.brake_mode?" brake":" duty");
    if(state==last_log_state_ && clock-last_log_at_<1/driving_log_rate_)return;
    last_log_state_=state;last_log_at_=clock;
    auto_control::DrivingLogFrame frame;frame.ros_ns=now().nanoseconds();
    frame.elapsed_sec=std::chrono::duration<double>(std::chrono::steady_clock::now()-log_started_).count();
    frame.labels={state};
    frame.values={active?result_.speed:0,measured_speed_,drive_command_.steer,drive_command_.duty,drive_command_.brake,
      requested_?1.:0.,locked_?1.:0.,result_.length,double(result_.candidates.size()),result_.used_vfh?1.:0.,result_.used_fallback?1.:0.,
      double(hazard_stats_.raw),double(hazard_stats_.confirmed),double(hazard_stats_.pending),double(hazard_stats_.immediate),
      double(hazard_stats_.held),double(hazard_stats_.remembered),hazard_filter_ms_,
      lane_reference_.trusted?1.:0.,lane_reference_.has_memory?1.:0.,lane_reference_.memory_age};
    for(const auto& p:result_.path)frame.path.push_back({p.x,p.y});
    driving_log_->enqueue(std::move(frame));
  }
  void applyDrive(bool active) {
    if(!drive_enabled_ || !drive_)return;
    const auto clock=std::chrono::steady_clock::now();
    const double dt=std::chrono::duration<double>(clock-last_drive_tick_).count();last_drive_tick_=clock;
    drive_command_=drive_->update(active,result_.speed,measured_speed_,result_.steer,dt,result_.used_vfh);
    // Publish one motor mode per cycle: duty and brake messages replace each
    // other in the bridge. Zero propulsion during braking is recorded below.
    if(drive_command_.brake_mode)publishFloat(brake_pub_,drive_command_.brake);
    else publishFloat(duty_pub_,drive_command_.duty);
    if(active && result_.speed>0)servo_commanded_=true;
    if(servo_commanded_)publishFloat(servo_pub_,drive_command_.servo);
    publishFloat(duty_log_pub_,drive_command_.duty);
    publishFloat(brake_log_pub_,drive_command_.brake);
    publishFloat(servo_log_pub_,drive_command_.servo);
    logDrive(active);
  }
  std::string driveGuard(bool* recoverable=nullptr) {
    if(recoverable)*recoverable=false;
    if(drive_enabled_ && !vesc_connected_)return "STOP: VESC not connected";
    if(drive_enabled_ && (count_publishers(duty_pub_->get_topic_name())>1 || count_publishers(brake_pub_->get_topic_name())>1 ||
      count_publishers(servo_pub_->get_topic_name())>1))return "STOP: another actuator publisher (stop auto/manual controller)";
    if(recoverable)*recoverable=true;
    if(!automaticOutputReady())return "STOP: waiting for AUTO output gate (release controls and stop)";
    if(locked_) {
      const double clock=now().seconds();
      if(clock-speed_at_>erpm_timeout_ || clock<speed_at_-.02 || !std::isfinite(measured_speed_))
        return "STOP: drive speed telemetry expired";
      if(imu_received_at_<0)return "STOP: no IMU received on /roi_lidar/imu";
      if(clock-imu_received_at_>imu_timeout_)return cv::format("STOP: IMU receive timeout %.3fs",clock-imu_received_at_);
      if(clock-imu_at_>imu_timeout_ || clock<imu_at_-.02)
        return cv::format("STOP: IMU capture age %.3fs; %s",clock-imu_at_,imu_issue_.c_str());
      if(!motion_ok_)return "STOP: IMU motion history unavailable; "+imu_issue_;
      if(!last_grid_)return "STOP: no drive plan input";
      if(motion_generation_!=motion_.generation)return "STOP: motion history reset; waiting for a new plan";
      const double age=clock-stamp(last_grid_->header.stamp),lane_age=planned_lane_stamp_?clock-*planned_lane_stamp_:0.;
      if(age<-.02 || age>drive_max_age_ || lane_age<-.02 || lane_age>drive_max_age_)
        return cv::format("STOP: capture expired terrain %.3fs lane %.3fs (limit %.2fs)",age,lane_age,drive_max_age_);
    }
    return {};
  }
  bool checkDriveGuard() {
    if(!requested_ || !locked_)return false;
    bool recoverable=false;const auto reason=driveGuard(&recoverable);
    if(reason.empty())return true;
    // Keep the actual missing-input/alignment reason while waiting for a new
    // plan, rather than replacing it with the consequence of clearing it.
    if(recoverable && suspended_ && !last_grid_ && reason=="STOP: no drive plan input")return false;
    if(recoverable)pauseForInput(reason);else stop(reason);
    return false;
  }
  void publish() {
    checkDriveGuard();
    nav_msgs::msg::Path path;path.header.stamp=now();path.header.frame_id=frame_id_;
    const bool active=this->active();
    if(active && last_grid_)path.header=last_grid_->header;
    if (active) for (const auto & p:result_.path) {
      geometry_msgs::msg::PoseStamped pose;pose.header=path.header;pose.pose.position.x=p.x;pose.pose.position.y=p.y;
      pose.pose.orientation.z=std::sin(p.yaw/2);pose.pose.orientation.w=std::cos(p.yaw/2);path.poses.push_back(pose);
    }
    path_pub_->publish(path);
    std_msgs::msg::Float32 speed,steer;speed.data=active?result_.speed:0;steer.data=active?result_.steer:0;
    speed_pub_->publish(speed);steer_pub_->publish(steer);
    std_msgs::msg::String message;
    message.data=status_+cv::format(" | target v=%.2fm/s steer=%.1fdeg plan=%.2fms candidates=%zu | rear retained checked; unseen unverified",
      speed.data,steer.data*180/pi,plan_ms_,result_.candidates.size());
    if(drive_enabled_)message.data+=cv::format(" | DRIVE duty=%.3f brake=%.2fA command_steer=%.1fdeg",drive_command_.duty,drive_command_.brake,drive_command_.steer*180/pi);
    if(manual_handover_)message.data+=" | output_gate="+handover_mode_+(manual_requested_?" requested=MANUAL":" requested=AUTO");
    message.data+=cv::format(" | lane_source=%s direction_memory=%d age=%.2fs longest=%.2fm",
      lane_reference_.state.c_str(),int(lane_reference_.has_memory),lane_reference_.memory_age,
      result_.longest_candidate);
    message.data+=cv::format(" | hazard raw=%zu confirmed=%zu pending=%zu immediate=%zu held=%zu memory=%zu filter=%.2fms",hazard_stats_.raw,hazard_stats_.confirmed,hazard_stats_.pending,hazard_stats_.immediate,hazard_stats_.held,hazard_stats_.remembered,hazard_filter_ms_);
    message.data+=config_.lane_memory_enabled==0?" | lane_memory_enabled=off":" | lane_memory_enabled=on";
    const double clock=now().seconds();
    message.data+=cv::format(" | input age terrain=%.3fs IMU=%.3fs speed=%.3fs motion_generation=%llu",
      grids_.empty()?-1.:clock-stamp(grids_.back()->header.stamp),imu_at_<0?-1.:clock-imu_at_,
      speed_at_<0?-1.:clock-speed_at_,static_cast<unsigned long long>(motion_.generation));
    status_pub_->publish(message);
    if(!drive_enabled_)logDrive(active);
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),static_cast<int64_t>(1000/status_rate_),"%s",message.data.c_str());
  }
  bool windowClosed() {
    // Older GTK HighGUI returns -1 for unsupported VISIBLE, even for an open window.
    // AUTOSIZE is available on GTK. Observe support before treating a missing
    // property/window as closure, so unsupported queries never terminate startup.
    try {
      const double visible=cv::getWindowProperty("ROI VFH+ test",cv::WND_PROP_VISIBLE);
      if (visible>0) {visible_supported_=true;return false;}
      if (visible==0 && visible_supported_) return true;
      const double autosize=cv::getWindowProperty("ROI VFH+ test",cv::WND_PROP_AUTOSIZE);
      if (autosize>=0) {autosize_supported_=true;return false;}
      return visible_supported_ || autosize_supported_;
    } catch (const cv::Exception&) {
      // Some backends throw after the last native window has been destroyed.
      return visible_supported_ || autosize_supported_;
    }
  }
  void draw() {
    if(!gui_ && !image_pub_->get_subscription_count())return;
    const auto clock=std::chrono::steady_clock::now();
    if(last_preview_at_ && std::chrono::duration<double>(clock-*last_preview_at_).count()<1/preview_rate_)return;
    last_preview_at_=clock;
    auto canvas=renderPreview(display_,lane_points_,result_,config_,requested_,locked_ && !suspended_,status_,plan_ms_,drive_enabled_,drive_command_.duty,drive_command_.brake,&hazard_stats_,&lane_reference_);
    if(manual_handover_ && !automaticOutputReady()) {
      cv::rectangle(canvas,{0,52,760,48},{245,245,245},-1);
      cv::putText(canvas,"OUTPUT GATE: "+handover_mode_+" | VFH commands blocked",{12,70},cv::FONT_HERSHEY_SIMPLEX,.43,{35,35,35},1);
      cv::putText(canvas,manual_requested_?"Stop -> MANUAL: release RT/LT/RB, center steering; wait for standstill":
        "Start -> AUTO: release controls; wait for standstill and fresh acknowledgement",{12,92},cv::FONT_HERSHEY_SIMPLEX,.39,{35,35,35},1);
    }
    const bool roi_fresh=roi_timing_received_ &&
      std::chrono::steady_clock::now()-*roi_timing_received_<std::chrono::seconds(3);
    const std::string roi_line=roi_fresh?
      cv::format("ROI avg ms | host %.2f | scan %.2f | grid + cluster %.2f | %.1f FPS",
        roi_timing_[0],roi_timing_[1],roi_timing_[2],roi_timing_[3]):
      "ROI avg ms | waiting for processing timing";
    const std::string vfh_line="VFH avg ms (last 30 calls) | hazard filter "+filter_average_.display()+
      " | plan "+plan_average_.display();
    cv::putText(canvas,roi_line,{12,603},cv::FONT_HERSHEY_SIMPLEX,.34,{35,35,35},1,cv::LINE_AA);
    cv::putText(canvas,vfh_line,{12,616},cv::FONT_HERSHEY_SIMPLEX,.34,{35,35,35},1,cv::LINE_AA);
    controls_.sync(config_);controls_.draw(canvas,requested_,planner_deceleration_ceiling_,speed_ceiling_,
      steering_ceiling_,steering_rate_ceiling_,drive_options_base_.maximum_duty);
    if (image_pub_->get_subscription_count()) {
      sensor_msgs::msg::Image msg;msg.header.stamp=now();msg.header.frame_id=frame_id_;
      msg.width=canvas.cols;msg.height=canvas.rows;msg.encoding="bgr8";msg.step=canvas.cols*3;
      msg.data.assign(canvas.data,canvas.data+canvas.total()*3);image_pub_->publish(msg);
    }
    if (gui_) {
      if (!window_) {cv::namedWindow("ROI VFH+ test",cv::WINDOW_AUTOSIZE);cv::setMouseCallback("ROI VFH+ test",mouse,this);window_=true;}
      cv::imshow("ROI VFH+ test",canvas);
      const int key=cv::waitKeyEx(1);const bool editing=controls_.editing();
      if (key==' ' || key=='s' || key=='S'){userStop();publish();}
      else if (key=='q' || (key==27 && !editing) || windowClosed()) {
        stop("STOPPED: window closed");publish();
        RCLCPP_INFO(get_logger(),"VFH window closed: stopping and unlocking ROI parameters");
        rclcpp::shutdown();
      }else {
        if(key=='p' || key=='P')controls_.toggle();
        controlAction(controls_.key(key,requested_));
      }
    }
  }
  void tick() {
    try {
      snapshotSensors();
      syncPlannerParameters();
      const bool ok=inputs();
      if (requested_ && !locked_)publishLock(true);
      bool held=false;
      if(!ok && requested_ && locked_) {
        bool recoverable=false;
        const bool cached_ok=last_grid_ && driveGuard(&recoverable).empty();
        if(input_recoverable_ && waiting_for_pair_ && !suspended_ && cached_ok) {
          held=true;status_="HOLD: previous valid path; waiting for synchronized input";
        }else if(input_recoverable_)pauseForInput(status_);
        else {const auto reason=status_;stop(reason);}
      }
      snapshotSensors();
      const bool guarded=checkDriveGuard();
      updateRecovery(ok && guarded && !held);
      if (!requested_)result_=Plan{};
      publish();
    } catch (const std::exception& e) {stop(std::string("STOP: ")+e.what());publish();}
    try {draw();}catch(const cv::Exception& e){stop("STOP: preview GUI error");publish();gui_=false;RCLCPP_ERROR(get_logger(),"%s",e.what());}
  }
public:
  rclcpp::CallbackGroup::SharedPtr sensorCallbacks()const{return sensor_callbacks_;}
  Node():rclcpp::Node("roi_vfh_test") {
    rcl_interfaces::msg::ParameterDescriptor fixed;fixed.read_only=true;
    rcl_interfaces::msg::ParameterDescriptor editable;editable.description="Editable only before Start or after Stop";
    gui_=declare_parameter("gui",true,fixed);
    drive_enabled_=declare_parameter("drive_enabled",false,fixed);
#define PARAM(name,field) config_.field=declare_parameter(name,config_.field,plannerField(name)?editable:fixed)
    PARAM("nominal_speed_mps",speed);PARAM("horizon_m",horizon);PARAM("sample_step_m",step);
    PARAM("wheelbase_m",wheelbase);PARAM("vehicle_width_m",width);PARAM("rear_extent_m",rear);PARAM("front_overhang_m",front);
    PARAM("safety_margin_m",margin);PARAM("lane_width_m",lane_width);
    PARAM("lane_lookahead_m",lane_lookahead);
    PARAM("minimum_path_length_m",minimum_path_length);PARAM("obstacle_uncertainty_m",uncertainty);
    PARAM("max_curvature_per_m",max_curvature);PARAM("lane_lateral_acceleration_mps2",lane_lateral_acceleration);
    PARAM("avoidance_lateral_acceleration_mps2",avoidance_lateral_acceleration);
    PARAM("stanley_gain",stanley_gain);PARAM("stanley_softening_speed_mps",stanley_softening);
    PARAM("stanley_corner_opposing_correction_ratio",stanley_opposing);
    PARAM("path_geometry_window_m",geometry_window);PARAM("steering_current_weight",steering_weight);
    PARAM("control_rate_hz",control_rate);
    PARAM("lane_memory_enabled",lane_memory_enabled);
    PARAM("hazard_filter_enabled",hazard_filter_enabled);PARAM("hazard_confirm_frames",hazard_confirm_frames);
    PARAM("hazard_small_max_cells",hazard_small_max_cells);PARAM("hazard_match_radius_cells",hazard_match_radius_cells);
    PARAM("hazard_max_gap_sec",hazard_max_gap_sec);
    PARAM("hazard_hold_sec",hazard_hold_sec);PARAM("obstacle_memory_enabled",obstacle_memory_enabled);
    PARAM("obstacle_memory_sec",obstacle_memory_sec);PARAM("obstacle_memory_distance_m",obstacle_memory_distance_m);
    PARAM("obstacle_memory_clearance_m",obstacle_memory_clearance_m);
    PARAM("avoidance_curve_enabled",avoidance_curve_enabled);PARAM("avoidance_curve_samples",avoidance_curve_samples);
    PARAM("avoidance_length_slack_m",avoidance_length_slack_m);PARAM("avoidance_lane_weight",avoidance_lane_weight);
    PARAM("avoidance_steering_weight",avoidance_steering_weight);
    PARAM("avoidance_clearance_weight",clearance_weight);
    config_.curvature_speed_control=declare_parameter("curvature_speed_control_enabled",true,fixed);
    PARAM("deceleration_mps2",deceleration);PARAM("response_delay_sec",delay);PARAM("reference_distance_m",reference_distance);
#undef PARAM
    config_.stanley_corner_heading=declare_parameter("stanley_corner_heading_threshold_deg",60.,editable)*pi/180;
    config_.max_steer=declare_parameter("max_steering_deg",30.,editable)*pi/180;
    config_.steer_rate=declare_parameter("steering_rate_deg_per_sec",240.,editable)*pi/180;
    config_.hist_low=declare_parameter("histogram_low",1.,editable);config_.hist_high=declare_parameter("histogram_high",2.,editable);
    config_.validate();motion_.front_axle_offset=config_.wheelbase;sensor_inbox_.motion.front_axle_offset=config_.wheelbase;
    steering_ceiling_=declare_parameter("steering_ceiling_deg",config_.max_steer*180/pi,fixed)*pi/180;
    steering_rate_ceiling_=declare_parameter("steering_rate_ceiling_deg_per_sec",config_.steer_rate*180/pi,fixed)*pi/180;
    if(!std::isfinite(steering_ceiling_) || steering_ceiling_<.05 || steering_ceiling_>pi/3 ||
      !std::isfinite(steering_rate_ceiling_) || steering_rate_ceiling_<.1 || steering_rate_ceiling_>20)
      throw std::invalid_argument("invalid configured steering ceilings");
    planner_deceleration_ceiling_=declare_parameter("deceleration_ceiling_mps2",config_.deceleration,fixed);
    if(!std::isfinite(planner_deceleration_ceiling_) || planner_deceleration_ceiling_<.1 || planner_deceleration_ceiling_>3)
      throw std::invalid_argument("invalid configured deceleration ceiling");
    speed_ceiling_=declare_parameter("speed_ceiling_mps",1.8,fixed);
    if(!std::isfinite(speed_ceiling_) || speed_ceiling_<.05 || speed_ceiling_>3)throw std::invalid_argument("invalid configured speed ceiling");
    max_age_=declare_parameter("max_age_sec",.30,fixed);
    if(!std::isfinite(max_age_) || max_age_<.05 || max_age_>.5)throw std::invalid_argument("invalid input age limit");
    frame_id_=declare_parameter<std::string>("frame_id","front_axle_bev",fixed);
    lane_topic_=declare_parameter<std::string>("lane_result_topic","/line_detactor/result",fixed);
    lane_receipt_timeout_=declare_parameter("lane_receipt_timeout_sec",.30,fixed);
    erpm_timeout_=declare_parameter("erpm_timeout_sec",.15,fixed);imu_timeout_=declare_parameter("imu_timeout_sec",.25,fixed);
    planning_rate_=declare_parameter("planning_rate_hz",80.,fixed);
    preview_rate_=declare_parameter("preview_fps",30.,fixed);status_rate_=declare_parameter("status_log_rate_hz",2.,fixed);
    max_lane_points_=declare_parameter("max_lane_points",2000,fixed);
    for(double v:{lane_receipt_timeout_,erpm_timeout_,imu_timeout_,planning_rate_,preview_rate_,status_rate_})
      if(!std::isfinite(v) || v<=0)throw std::invalid_argument("invalid source controller input/watchdog settings");
    if(frame_id_.empty() || lane_receipt_timeout_>1 || erpm_timeout_>1 || imu_timeout_>1 || planning_rate_>100 || preview_rate_>60 || status_rate_>20 ||
      max_lane_points_<2 || max_lane_points_>2000)throw std::invalid_argument("source controller input/watchdog limits exceeded");
    obstacle_slowdown_=declare_parameter("obstacle_slowdown_enabled",true,fixed);
    obstacle_speed_cap_=declare_parameter("obstacle_slowdown_speed_mps",1.,fixed);
    obstacle_clear_sec_=declare_parameter("obstacle_slowdown_clear_sec",1.,fixed);
    config_.target_recovery_acceleration=declare_parameter("target_recovery_acceleration_mps2",config_.target_recovery_acceleration,editable);
    if(!std::isfinite(obstacle_speed_cap_) || obstacle_speed_cap_<=0 || !std::isfinite(obstacle_clear_sec_) || obstacle_clear_sec_<0 ||
      !std::isfinite(config_.target_recovery_acceleration) || config_.target_recovery_acceleration<=0)throw std::invalid_argument("invalid source slowdown/recovery settings");
    DriveOptions drive_options;drive_options.max_steer=config_.max_steer;drive_options.servo_angle_range=steering_ceiling_;
    drive_options.steering_weight=config_.steering_weight;drive_options.control_rate=config_.control_rate;
    drive_options.steer_rate=config_.steer_rate;drive_options.planning_deceleration=config_.deceleration;
#define DRIVE_PARAM(name,field) do { \
  const std::string key="drive." name;const auto* tuning=plannerField(key); \
  drive_options.field=declare_parameter(key,drive_options.field,tuning?editable:fixed); \
  if(tuning)config_.*(tuning->member)=drive_options.field/tuning->scale; \
} while(false)
    DRIVE_PARAM("maximum_duty",maximum_duty);DRIVE_PARAM("longitudinal_start_duty",start_duty);
    DRIVE_PARAM("duty_rise_rate_per_sec",rise);DRIVE_PARAM("duty_fall_rate_per_sec",fall);
    DRIVE_PARAM("longitudinal_pid_kp",kp);DRIVE_PARAM("longitudinal_pid_ki",ki);
    DRIVE_PARAM("longitudinal_pid_kd",kd);DRIVE_PARAM("longitudinal_pid_integral_limit",integral_limit);
    DRIVE_PARAM("longitudinal_feedforward_offset_duty",ff_offset);
    DRIVE_PARAM("longitudinal_feedforward_duty_per_mps",ff_slope);
    DRIVE_PARAM("longitudinal_feedforward_fade_speed_mps",ff_fade_speed);
    DRIVE_PARAM("brake_maximum_current_amps",brake_max);DRIVE_PARAM("longitudinal_brake_speed_gain",brake_speed_gain);

    DRIVE_PARAM("brake_current_rise_amps_per_sec",brake_rise);DRIVE_PARAM("brake_current_fall_amps_per_sec",brake_fall);
    DRIVE_PARAM("servo_left",servo_left);DRIVE_PARAM("servo_center",servo_center);DRIVE_PARAM("servo_right",servo_right);
    DRIVE_PARAM("steering_servo_inverted",inverted);
    DRIVE_PARAM("longitudinal_recovery_duty_rise_per_sec",recovery_rise);
    DRIVE_PARAM("longitudinal_recovery_start_ramp_sec",recovery_ramp);
    DRIVE_PARAM("electrical_brake_enabled",electrical_brake);
    DRIVE_PARAM("longitudinal_deceleration_filter_sec",deceleration_filter);
    DRIVE_PARAM("longitudinal_brake_deceleration_gain",brake_deceleration_gain);
    DRIVE_PARAM("brake_entry_speed_error_mps",normal_entry);DRIVE_PARAM("brake_exit_speed_error_mps",normal_exit);
    DRIVE_PARAM("obstacle_brake_entry_margin_mps",obstacle_entry);DRIVE_PARAM("obstacle_brake_exit_margin_mps",obstacle_exit);
    DRIVE_PARAM("obstacle_coast_probe_sec",obstacle_probe);DRIVE_PARAM("corner_coast_probe_sec",normal_probe);
    DRIVE_PARAM("obstacle_brake_deceleration_margin_mps2",obstacle_deceleration_margin);
    DRIVE_PARAM("corner_brake_deceleration_margin_mps2",normal_deceleration_margin);
#undef DRIVE_PARAM
    drive_options_base_=drive_options;
    validatePlannerEdits(config_,planner_deceleration_ceiling_,speed_ceiling_,steering_ceiling_,
      steering_rate_ceiling_,drive_options_base_.maximum_duty);
    drive_=std::make_unique<DriveController>(tunedDriveOptions(drive_options_base_,config_));controls_.reload(config_);
    drive_max_age_=declare_parameter("drive.max_command_age_sec",.30,fixed);
    if(!std::isfinite(drive_max_age_) || drive_max_age_<.05 || drive_max_age_>.3)throw std::invalid_argument("invalid drive command age");
    resume_stable_sec_=declare_parameter("input_resume_stable_sec",.5,fixed);
    if(!std::isfinite(resume_stable_sec_) || resume_stable_sec_<.1 || resume_stable_sec_>5)
      throw std::invalid_argument("invalid input resume stability period");
    const auto sensor=rclcpp::SensorDataQoS().keep_last(1);
    const auto latch=rclcpp::QoS(1).reliable().transient_local();
    const auto duty_topic=declare_parameter<std::string>("duty_topic","/vesc/duty",fixed);
    const auto brake_topic=declare_parameter<std::string>("brake_current_topic","/vesc/brake_current",fixed);
    const auto servo_topic=declare_parameter<std::string>("servo_position_topic","/vesc/servo_position",fixed);
    const auto connection_topic=declare_parameter<std::string>("vesc_connection_topic","/vesc/connected",fixed);
    manual_handover_=declare_parameter("manual_handover_enabled",false,fixed) && drive_enabled_;
    handover_default_stop_current_=declare_parameter("handover_stop_brake_current_amps",2.0,fixed);
    const auto mode_request_topic=declare_parameter<std::string>("manual_mode_request_topic","/roi_vfh_test/drive_mode_request",fixed);
    const auto mode_ack_topic=declare_parameter<std::string>("manual_mode_ack_topic","/roi_vfh_test/drive_mode_ack",fixed);
    if(manual_handover_) {
      handover_request_pub_=create_publisher<std_msgs::msg::String>(mode_request_topic,latch);
      handover_ack_sub_=create_subscription<std_msgs::msg::String>(mode_ack_topic,latch,
        [this](std_msgs::msg::String::ConstSharedPtr message) {
          const auto separator=message->data.find('|');
          if(separator==std::string::npos || message->data.substr(0,separator)!=handover_token_)return;
          const auto mode=message->data.substr(separator+1);
          if(mode!="AUTO" && mode!="MANUAL" && mode!="STOPPING_MANUAL" && mode!="STOPPING_AUTO")return;
          const bool returning=mode=="AUTO" && handover_mode_!="AUTO";
          handover_mode_=mode;handover_ack_at_=std::chrono::steady_clock::now();
          if(returning) {servo_commanded_=false;drive_command_=DriveCommand{};if(drive_)drive_->centerAtStandstill();}
        });
    }
    if(drive_enabled_) {
      const auto command=rclcpp::QoS(1).reliable().durability_volatile();
      duty_pub_=create_publisher<std_msgs::msg::Float32>(duty_topic,command);
      brake_pub_=create_publisher<std_msgs::msg::Float32>(brake_topic,command);
      servo_pub_=create_publisher<std_msgs::msg::Float32>(servo_topic,command);
      duty_log_pub_=create_publisher<std_msgs::msg::Float32>("~/command_duty",1);
      brake_log_pub_=create_publisher<std_msgs::msg::Float32>("~/command_brake_current",1);
      servo_log_pub_=create_publisher<std_msgs::msg::Float32>("~/command_servo_position",1);
      connection_sub_=create_subscription<std_msgs::msg::Bool>(connection_topic,latch,
        [this](std_msgs::msg::Bool::ConstSharedPtr p){
          vesc_connected_=p->data;
          if(!vesc_connected_ && requested_){stop("STOP: VESC disconnected");publish();}
        });
      RCLCPP_INFO(get_logger(),"DRIVE output after Start: duty=%s brake=%s servo=%s connection=%s",duty_topic.c_str(),brake_topic.c_str(),servo_topic.c_str(),connection_topic.c_str());
    }
    running_pub_=create_publisher<std_msgs::msg::Bool>("~/running",latch);
    path_pub_=create_publisher<nav_msgs::msg::Path>("~/path",1);
    speed_pub_=create_publisher<std_msgs::msg::Float32>("~/target_speed",1);
    steer_pub_=create_publisher<std_msgs::msg::Float32>("~/steering_angle_rad",1);
    status_pub_=create_publisher<std_msgs::msg::String>("~/status",1);
    image_pub_=create_publisher<sensor_msgs::msg::Image>("~/preview",sensor);
    roi_timing_sub_=create_subscription<std_msgs::msg::Float64MultiArray>("/roi_lidar/processing_timing",sensor,
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr message) {
        if (message->data.size()!=roi_timing_.size() ||
          !std::all_of(message->data.begin(),message->data.end(),[](double value) {
            return std::isfinite(value) && value>=0;
          })) {return;}
        std::copy(message->data.begin(),message->data.end(),roi_timing_.begin());
        roi_timing_received_=std::chrono::steady_clock::now();
      });
    sensor_callbacks_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    rclcpp::SubscriptionOptions sensor_options;sensor_options.callback_group=sensor_callbacks_;
    grid_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>("/roi_lidar/terrain_for_planning",sensor,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr p){
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        sensor_inbox_.grids.push_back(p);while(sensor_inbox_.grids.size()>16)sensor_inbox_.grids.pop_front();
      },sensor_options);
    const auto speed_topic=declare_parameter<std::string>("measured_speed_topic","/vehicle/dynamics/speed_mps",fixed);
    speed_sub_=create_subscription<std_msgs::msg::Float32>(speed_topic,sensor,
      [this](std_msgs::msg::Float32::ConstSharedPtr p){
        const double received=now().seconds();
        std::lock_guard<std::mutex> lock(sensor_mutex_);auto& input=sensor_inbox_;
        if(!std::isfinite(p->data)){input.speed_at=-1e9;input.motion.clear();input.motion_ok=false;return;}
        input.speed=p->data;input.speed_at=received;
      },sensor_options);
    // Integrate IMU independently of the planning/GUI executor. Preserve every
    // accepted capture time; the inbox never retimestamps a queued sample.
    imu_sub_=create_subscription<sensor_msgs::msg::Imu>("/roi_lidar/imu",rclcpp::SensorDataQoS().keep_last(200),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr p){
        const double received=now().seconds();
        std::lock_guard<std::mutex> lock(sensor_mutex_);auto& input=sensor_inbox_;
        input.imu_received_at=received;
        if(p->header.frame_id!=frame_id_ || !std::isfinite(p->angular_velocity.z)) {
          input.imu_issue="invalid frame or yaw rate";input.motion.clear();input.motion_ok=false;return;
        }
        const double capture=stamp(p->header.stamp),age=received-capture;
        if(age<-.02 || age>max_age_) {input.imu_issue="stale/future IMU sample discarded";return;}
        if(capture<=input.imu_at)return; // A duplicate must not erase the integration history.
        input.imu_at=capture;input.yaw_rate=p->angular_velocity.z;
        if(received-input.speed_at>erpm_timeout_) {
          input.imu_issue="waiting for fresh VESC speed";input.motion.clear();input.motion_ok=false;return;
        }
        input.motion.add(capture,input.speed,input.yaw_rate);
        MotionPose pose;input.motion_ok=input.motion.at(capture,pose);
        input.imu_issue=input.motion_ok?"ok":"motion sample rejected (speed/yaw/time)";
      },sensor_options);
    lane_sub_=create_subscription<line_detactor::msg::LaneResult>(lane_topic_,sensor,
      [this](line_detactor::msg::LaneResult::ConstSharedPtr p){
        const double received=now().seconds();std::lock_guard<std::mutex> lock(sensor_mutex_);
        sensor_inbox_.lane=p;sensor_inbox_.lane_received_at=received;
      },sensor_options);
    lock_sub_=create_subscription<std_msgs::msg::Bool>("/roi_lidar/parameters_locked",latch,
      [this](std_msgs::msg::Bool::ConstSharedPtr p){
        if(requested_ && locked_ && !p->data){stop("STOP: ROI parameter lock lost");publish();return;}
        locked_=requested_ && p->data;
      });
    RCLCPP_INFO(get_logger(),"VFH telemetry: speed=%s (Float32 m/s), IMU=/roi_lidar/imu (vehicle-frame Z rad/s)",speed_sub_->get_topic_name());
    const bool log_enabled=declare_parameter("driving_log_enabled",true,fixed);
    driving_log_rate_=declare_parameter("driving_log_rate_hz",30.,fixed);
    const auto log_directory=declare_parameter<std::string>("driving_log_directory","driving_logs",fixed);
    declare_parameter<std::string>("reference_profile","",fixed);
    if(!std::isfinite(driving_log_rate_) || driving_log_rate_<=0 || driving_log_rate_>200)throw std::invalid_argument("invalid driving log rate");
    planner_parameters_callback_=add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      if(requested_){result.reason="VFH is locked: press Stop before changing parameters";return result;}
      try {
        auto next=plannerParameters();
        for(const auto& parameter:parameters) {
          const auto* field=plannerField(parameter.get_name());
          if(!field)throw std::invalid_argument("Only VFH tuning fields are runtime-editable");
          if(parameter.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE)
            throw std::invalid_argument(parameter.get_name()+": expected a floating-point number");
          next.*(field->member)=parameter.as_double()/field->scale;
        }
        validatePlannerEdits(next,planner_deceleration_ceiling_,speed_ceiling_,steering_ceiling_,
          steering_rate_ceiling_,drive_options_base_.maximum_duty);
        tunedDriveOptions(drive_options_base_,next);result.successful=true;
      }catch(const std::exception& e){result.reason=e.what();}
      return result;
    });
    if(log_enabled) {
      std::string parameters;
      for(const auto& name:list_parameters({},10).names)parameters+=name+"="+get_parameter(name).value_to_string()+"\n";
      driving_log_=std::make_unique<auto_control::DrivingLog>(log_directory,
        "ros_ns,elapsed_sec,dropped,state,target_speed_mps,speed_mps,steer_rad,duty,brake_amps,requested,locked,path_length_m,candidates,vfh,curve_fallback,hazard_raw,hazard_confirmed,hazard_pending,hazard_immediate,hazard_held,hazard_remembered,hazard_filter_ms,lane_trusted,lane_memory,lane_memory_age_sec,path",parameters);
      RCLCPP_INFO(get_logger(),"VFH driving CSV: %s",driving_log_->path().c_str());
    }
    publishLock(false);
    requestDriveMode(false); // Initial state remains stopped; no manual input before Stop.
    RCLCPP_INFO(get_logger(),"Source geometry: width=%.3f front=%.3f rear=%.3f margin=%.3f min_path=%.3f; Stanley lookahead=%.3f, nominal=%.2f ceiling=%.2f",
      config_.width,config_.front,config_.rear,config_.margin,config_.minimum_path_length,config_.lane_lookahead,config_.speed,speed_ceiling_);
    timer_=create_wall_timer(std::chrono::duration<double>(1/planning_rate_),[this]{tick();});
    if(drive_enabled_)actuator_timer_=create_wall_timer(std::chrono::duration<double>(1/config_.control_rate),[this]{
      try {
        snapshotSensors();
        checkDriveGuard();
        applyDrive(active() && bool(last_grid_));
      }catch(const std::exception& e){stop(std::string("STOP: actuator loop ")+e.what());publish();}
    });
  }
};
}
int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  auto node=std::make_shared<roi_vfh_test::Node>();
  rclcpp::executors::SingleThreadedExecutor sensors;
  sensors.add_callback_group(node->sensorCallbacks(),node->get_node_base_interface());
  std::thread receiver([&]{
    try{sensors.spin();}
    catch(const std::exception& e){
      RCLCPP_ERROR(node->get_logger(),"Sensor executor failed: %s",e.what());rclcpp::shutdown();
    }
  });
  try{rclcpp::spin(node);} // Keep HighGUI on the original main thread.
  catch(...){if(rclcpp::ok())rclcpp::shutdown();sensors.cancel();receiver.join();throw;}
  sensors.cancel();receiver.join();if(rclcpp::ok())rclcpp::shutdown();
}
