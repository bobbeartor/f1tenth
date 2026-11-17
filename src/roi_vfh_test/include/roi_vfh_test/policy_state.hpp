#pragma once
#include "roi_vfh_test/motion.hpp"
#include <optional>
#include <string>

namespace roi_vfh_test {
struct LaneReference {
  std::vector<Point> points;
  std::string state{"forward"};
  bool trusted{false},has_memory{false}; // trusted means usable CURRENT geometry, not a temporal confidence gate.
  double memory_age{0};
};
// Never retain a lane polyline/offset. Remember only the last observed tangent
// in a fixed yaw frame, so translation cannot turn it into a stale return path.
class LaneMemory {
  std::optional<double> world_heading_;
  double seen_at_{-1};
  std::vector<Point> direction(const MotionPose& pose,const Config& c)const {
    const double angle=world_heading_?wrap(*world_heading_-pose.yaw):0.;
    const double length=std::max(c.horizon,c.reference_distance);
    return {{0,0},{length*std::cos(angle),length*std::sin(angle)}};
  }
public:
  void reset(){world_heading_.reset();seen_at_=-1;}
  LaneReference resolve(const std::vector<Point>& observed,const MotionPose& pose,const Config& c,
      std::optional<double> capture=std::nullopt) {
    LaneReference result;
    if(c.lane_memory_enabled==0)reset();
    if(observed.size()>2000)throw std::invalid_argument("lane sample limit");
    for(auto p:observed) {
      if(!std::isfinite(p.x) || !std::isfinite(p.y))throw std::invalid_argument("nonfinite current lane");
      if(result.points.empty() || std::hypot(p.x-result.points.back().x,p.y-result.points.back().y)>1e-5)
        result.points.push_back(p);
    }
    result.trusted=result.points.size()>=2;
    if(result.trusted) {
      result.state="current";
      if(c.lane_memory_enabled!=0) {
        const Polyline line(result.points);
        const double nearest=line.nearest({0,0});
        const auto a=line.at(nearest),b=line.at(std::min(line.length(),nearest+c.lane_lookahead));
        const double heading=std::hypot(b.x-a.x,b.y-a.y)>1e-5?std::atan2(b.y-a.y,b.x-a.x):line.heading({0,0});
        world_heading_=wrap(pose.yaw+heading);seen_at_=capture.value_or(pose.t);
      }
    }else {
      result.points=direction(pose,c);
      result.state=world_heading_?"direction_memory":"forward";
    }
    result.has_memory=world_heading_.has_value();
    result.memory_age=result.has_memory?std::max(0.,pose.t-seen_at_):0.;
    return result;
  }
  std::vector<Point> remembered(const MotionPose& pose)const {
    return world_heading_?direction(pose,Config{}):std::vector<Point>{};
  }
};
}
