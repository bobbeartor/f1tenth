#pragma once
#include "roi_vfh_test/planner.hpp"
#include <deque>
namespace roi_vfh_test {
struct MotionPose {double t{},x{},y{},yaw{},distance{};};
inline Point reproject(Point p,const MotionPose& from,const MotionPose& to){
  const double wx=from.x+std::cos(from.yaw)*p.x-std::sin(from.yaw)*p.y;
  const double wy=from.y+std::sin(from.yaw)*p.x+std::cos(from.yaw)*p.y;
  const double dx=wx-to.x,dy=wy-to.y;
  return {std::cos(to.yaw)*dx+std::sin(to.yaw)*dy,-std::sin(to.yaw)*dx+std::cos(to.yaw)*dy};
}
class MotionHistory {
  std::deque<MotionPose> poses_;double last_speed_{},last_yaw_rate_{};
public:
  double front_axle_offset{.33}; // VESC longitudinal speed is modeled at the rear axle.
  std::uint64_t generation{0};
  void clear(){poses_.clear();++generation;}
  void add(double t,double speed,double yaw_rate){
    if(!std::isfinite(t)||!std::isfinite(speed)||!std::isfinite(yaw_rate)||std::abs(speed)>6||std::abs(yaw_rate)>8){clear();return;}
    if(poses_.empty()){poses_.push_back({t,0,0,0,0});last_speed_=speed;last_yaw_rate_=yaw_rate;return;}
    const auto p=poses_.back();const double dt=t-p.t;
    if(dt<=0 || dt>.1){clear();poses_.push_back({t,0,0,0,0});last_speed_=speed;last_yaw_rate_=yaw_rate;return;}
    const double v=(speed+last_speed_)/2,w=(yaw_rate+last_yaw_rate_)/2,angle=p.yaw+w*dt/2;
    const double lateral=front_axle_offset*w;
    poses_.push_back({t,p.x+dt*(v*std::cos(angle)-lateral*std::sin(angle)),p.y+dt*(v*std::sin(angle)+lateral*std::cos(angle)),wrap(p.yaw+w*dt),p.distance+std::hypot(v,lateral)*dt});
    last_speed_=speed;last_yaw_rate_=yaw_rate;
    while(poses_.size()>1500 || poses_.front().t<t-3)poses_.pop_front();
  }
  bool at(double t,MotionPose& out)const{
    if(poses_.empty() || t<poses_.front().t || t>poses_.back().t+.03)return false;
    for(std::size_t i=1;i<poses_.size();++i)if(t<=poses_[i].t){
      const auto a=poses_[i-1],b=poses_[i];const double f=(t-a.t)/(b.t-a.t);
      out={t,a.x+f*(b.x-a.x),a.y+f*(b.y-a.y),wrap(a.yaw+f*wrap(b.yaw-a.yaw)),a.distance+f*(b.distance-a.distance)};return true;
    }
    const auto p=poses_.back();const double dt=t-p.t;
    const double angle=p.yaw+last_yaw_rate_*dt/2,lateral=front_axle_offset*last_yaw_rate_;
    out={t,p.x+dt*(last_speed_*std::cos(angle)-lateral*std::sin(angle)),p.y+dt*(last_speed_*std::sin(angle)+lateral*std::cos(angle)),wrap(p.yaw+last_yaw_rate_*dt),p.distance+std::hypot(last_speed_,lateral)*dt};return true;
  }
};
}
