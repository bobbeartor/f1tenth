#pragma once
#include "roi_vfh_test/planner.hpp"

namespace roi_vfh_test {
// Post-planning target-speed envelope, shared by ROS and the native simulator.
class TargetSpeedPolicy {
  double last_avoidance_{-1e9},last_target_{0},last_plan_{-1e9};
public:
  void reset() {last_avoidance_=-1e9;last_target_=0;last_plan_=-1e9;}
  void apply(Plan& result,bool lane_missing,double now,double measured_speed,
      bool slowdown,double speed_cap,double clear_sec,double recovery_acceleration) {
    if(result.reference_blocked || (result.used_vfh && !lane_missing))last_avoidance_=now;
    if(slowdown && now-last_avoidance_<=clear_sec)result.speed=std::min(result.speed,speed_cap);
    if(last_target_>0 && std::abs(measured_speed)>=.05 && now>last_plan_)
      result.speed=std::min(result.speed,last_target_+recovery_acceleration*(now-last_plan_));
    last_target_=result.speed;last_plan_=now;
  }
};
}
