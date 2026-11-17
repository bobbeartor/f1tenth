#pragma once
#include <auto_control/control_core.hpp>
#include "roi_vfh_test/planner.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace roi_vfh_test {
struct DriveOptions {
  double maximum_duty{.12},start_duty{.053},rise{.04},fall{.03};
  double kp{1},ki{.5},kd{0},integral_limit{1.5};
  double ff_offset{.01},ff_slope{.05},ff_fade_speed{.1};
  double brake_max{2},brake_speed_gain{.8};
  double brake_rise{1.5},brake_fall{6};
  double servo_left{1},servo_center{.5},servo_right{0},max_steer{.5235987755982988};
  double steering_weight{.4},control_rate{100},steer_rate{4.1887902047863905};
  double servo_angle_range{0}; // Fixed calibration span; 0 retains standalone legacy mapping.
  double planning_deceleration{.5};
  double recovery_rise{.045},recovery_ramp{.20},deceleration_filter{.20},brake_deceleration_gain{.5};
  double normal_entry{.3},normal_exit{.1},obstacle_entry{.1},obstacle_exit{.03};
  double normal_probe{.35},obstacle_probe{.20},normal_deceleration_margin{.2},obstacle_deceleration_margin{.1};
  bool inverted{true},electrical_brake{true};
  void validate()const {
    for(double v:{maximum_duty,start_duty,rise,fall,kp,ki,kd,integral_limit,ff_offset,ff_slope,
      ff_fade_speed,brake_max,brake_speed_gain,brake_rise,brake_fall,steering_weight,control_rate,steer_rate,recovery_rise,recovery_ramp,
      planning_deceleration,deceleration_filter,brake_deceleration_gain,normal_entry,normal_exit,obstacle_entry,obstacle_exit,
      normal_probe,obstacle_probe,normal_deceleration_margin,obstacle_deceleration_margin,
      servo_left,servo_center,servo_right,max_steer,servo_angle_range})
      if(!std::isfinite(v) || v<0)throw std::invalid_argument("invalid VFH drive parameter");
    if(maximum_duty<=0 || maximum_duty>1 || start_duty>maximum_duty || rise<=0 || fall<=0 ||
      ff_fade_speed<=0 || brake_max<=0 || brake_max>8 || brake_speed_gain<=0 ||
      steering_weight<=0 || steering_weight>1 || control_rate<10 || control_rate>200 || steer_rate<=0 ||
      recovery_rise<=0 || recovery_ramp<=0 || deceleration_filter<=0 || planning_deceleration<=0 || normal_exit>=normal_entry || obstacle_exit>=obstacle_entry || brake_rise<=0 || brake_fall<=0 ||
      servo_left>1 || servo_center>1 || servo_right>1 || max_steer<=0 || max_steer>1.0472 ||
      servo_angle_range>1.0472 || (servo_angle_range>0 && max_steer>servo_angle_range+1e-9) ||
      (servo_left-servo_center)*(servo_right-servo_center)>=0)
      throw std::invalid_argument("VFH drive limits/servo calibration invalid");
  }
};
struct DriveCommand {
  double duty{0},brake{0},servo{.5},steer{0};
  bool brake_mode{true};
};
inline DriveOptions tunedDriveOptions(DriveOptions options,const Config& c) {
  options.start_duty=c.drive_start_duty;options.rise=c.drive_rise;options.fall=c.drive_fall;
  options.recovery_rise=c.drive_recovery_rise;options.recovery_ramp=c.drive_recovery_ramp;
  options.brake_max=c.drive_brake_max;options.brake_rise=c.drive_brake_rise;options.brake_fall=c.drive_brake_fall;
  options.brake_speed_gain=c.drive_brake_speed_gain;options.planning_deceleration=c.deceleration;
  options.max_steer=c.max_steer;options.steer_rate=c.steer_rate;options.steering_weight=c.steering_weight;
  options.validate();return options;
}
// Lightweight longitudinal adapter: reuse auto_control's normalized PID,
// feedforward calibration, slew helper, staged braking and exact servo mapping.
// It does not run auto_control's lane planner or its higher speed floor.
class DriveController {
  DriveOptions options_;
  auto_control::SpeedPid pid_;
  DriveCommand output_;
  bool braking_{true},departure_{true},recovering_{false},previous_speed_valid_{false};
  double previous_speed_{0},deceleration_{0},coast_time_{0};
public:
  explicit DriveController(const DriveOptions& options):options_(options),
    pid_(options.kp,options.ki,options.kd,options.integral_limit,-1,1) {
    options_.validate();output_.servo=options_.servo_center;
  }
  void retainSteering(double steer) {
    output_.steer=std::clamp(steer,-options_.max_steer,options_.max_steer);
    output_.servo=auto_control::steering_angle_to_servo(output_.steer,options_.servo_angle_range>0?options_.servo_angle_range:options_.max_steer,
      options_.servo_left,options_.servo_center,options_.servo_right,options_.inverted);
  }
  void centerAtStandstill() {retainSteering(0);}
  DriveCommand update(bool active,double target,double speed,double steer,double dt,bool avoiding=false) {
    if(!active || !std::isfinite(target) || !std::isfinite(speed) || !std::isfinite(steer) || target<=0) {
      pid_.reset();departure_=true;braking_=true;recovering_=false;previous_speed_valid_=false;coast_time_=0;deceleration_=0;
      output_.duty=0;output_.brake=options_.brake_max;output_.brake_mode=true;
      return output_; // Retain last commanded steering on all stop paths.
    }
    dt=std::isfinite(dt)?std::clamp(dt,0.001,.1):.001;
    const double steering_weight=1-std::pow(1-options_.steering_weight,dt*options_.control_rate);
    const double filtered=steering_weight*std::clamp(steer,-options_.max_steer,options_.max_steer)+(1-steering_weight)*output_.steer;
    output_.steer=auto_control::move_toward(output_.steer,filtered,options_.steer_rate*dt);
    output_.servo=auto_control::steering_angle_to_servo(output_.steer,options_.servo_angle_range>0?options_.servo_angle_range:options_.max_steer,
      options_.servo_left,options_.servo_center,options_.servo_right,options_.inverted);
    const double overspeed=std::abs(speed)-target;
    const double entry=avoiding?options_.obstacle_entry:options_.normal_entry;
    const double exit=avoiding?options_.obstacle_exit:options_.normal_exit;
    const double probe=avoiding?options_.obstacle_probe:options_.normal_probe;
    const double margin=avoiding?options_.obstacle_deceleration_margin:options_.normal_deceleration_margin;
    if(previous_speed_valid_) {
      const double raw=std::clamp((previous_speed_-std::abs(speed))/dt,-15.,15.);
      deceleration_+=(1-std::exp(-dt/options_.deceleration_filter))*(raw-deceleration_);
    }
    previous_speed_=std::abs(speed);previous_speed_valid_=true;
    coast_time_=overspeed>entry?coast_time_+dt:0;
    const bool need_brake=options_.electrical_brake && overspeed>(braking_?exit:entry) && (braking_ || coast_time_>=probe);
    if(need_brake || braking_) {
      pid_.reset();output_.duty=0;output_.brake_mode=true;
      const double requested=need_brake?auto_control::staged_brake_current(overspeed,std::max(0.,options_.planning_deceleration-deceleration_-margin),
        options_.brake_speed_gain,options_.brake_deceleration_gain,options_.brake_max):0;
      output_.brake=auto_control::move_toward(output_.brake,requested,
        (requested>output_.brake?options_.brake_rise:options_.brake_fall)*dt);
      if(output_.brake<=1e-6)output_.brake=0;
      braking_=need_brake || output_.brake>0;
      if(!braking_)recovering_=true;
      // Even when current becomes zero, publish ONLY brake=0 this cycle.
      // The VESC bridge replaces pending motor commands by mode.
      return output_;
    }
    output_.brake=0;output_.brake_mode=false;
    const double ff=std::clamp(options_.ff_offset*std::min(1.,target/options_.ff_fade_speed)+
      options_.ff_slope*target,0.,options_.maximum_duty);
    const double effort=pid_.update(target,speed,ff/options_.maximum_duty,dt);
    const double desired=overspeed>entry?0:std::clamp(effort*options_.maximum_duty,0.,options_.maximum_duty);
    if(overspeed>entry)pid_.reset();
    if(departure_ && std::abs(speed)<.05 && desired>0) {
      output_.duty=std::min(options_.start_duty,desired);departure_=false;
    } else {
      double rise=recovering_?options_.recovery_rise:options_.rise;
      if(recovering_ && std::abs(speed)<.1 && output_.duty<options_.start_duty)
        rise=std::max(rise,options_.start_duty/options_.recovery_ramp);
      output_.duty=auto_control::move_toward(output_.duty,desired,(desired>output_.duty?rise:options_.fall)*dt);
      if(std::abs(speed)>=.05)departure_=false;
    }
    if(std::abs(output_.duty-desired)<1e-4 && overspeed>=-exit)recovering_=false;
    pid_.apply_output_limit(effort,output_.duty/options_.maximum_duty);
    return output_;
  }
};
}
