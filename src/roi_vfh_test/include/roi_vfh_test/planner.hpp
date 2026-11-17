// Shared native VFH+ policy for the ROS vehicle and local simulator.
#pragma once
#include <auto_control/control_core.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>
namespace roi_vfh_test {
constexpr double pi=3.14159265358979323846;
struct Point {double x{},y{};};
struct Pose {double x{},y{},yaw{},steer{};};
struct Grid {
  int width{},height{}; double resolution{},x0{},y0{};
  std::vector<std::int8_t> cells;
  int at(double x,double y) const {
    const int u=int(std::floor((x-x0)/resolution)),v=int(std::floor((y-y0)/resolution));
    return u<0 || v<0 || u>=width || v>=height ? -1 : cells[std::size_t(v)*width+u];
  }
  void validate() const {
    if (width<1 || height<1 || width>2000 || height>2000 ||
      std::size_t(width)*height>250000 || cells.size()!=std::size_t(width)*height ||
      !std::isfinite(resolution) || resolution<.01 || resolution>.25 || !std::isfinite(x0) || !std::isfinite(y0))
      throw std::invalid_argument("invalid metric terrain grid");
  }
};
// Match the preview: mint/gray are traversable; only red occupancy (>=50) blocks.
inline bool blockedTerrain(int value) {return value>=50;}
constexpr std::int8_t held_hazard=70,remembered_hazard=80;
inline bool retainedTerrain(int value) {return value==held_hazard || value==remembered_hazard;}
struct Config {
  double speed{1.2},horizon{1.2},step{.03},wheelbase{.33},width{.30},rear{.45},front{.05};
  double margin{.02},max_steer{30*pi/180},steer_rate{240*pi/180};
  double lane_width{.65},deceleration{.4},delay{.01};
  double current_steer{0},current_speed{.7};
  double minimum_path_length{.12},uncertainty{0},max_curvature{2};
  double lane_lateral_acceleration{1.6},avoidance_lateral_acceleration{.4};
  double stanley_gain{1.3},stanley_softening{.3},stanley_corner_heading{60*pi/180},stanley_opposing{.8};
  double geometry_window{.16},steering_weight{.4},control_rate{100};
  bool curvature_speed_control{true};
  // Numeric 0/1 and integer-valued doubles share the pre-Start GUI/ROS edit path.
  double lane_memory_enabled{1};
  double hazard_filter_enabled{1},hazard_confirm_frames{4},hazard_small_max_cells{8};
  double hazard_match_radius_cells{1},hazard_max_gap_sec{.15};
  double hazard_hold_sec{.30},obstacle_memory_enabled{1},obstacle_memory_sec{2.0};
  double obstacle_memory_distance_m{1.0},obstacle_memory_clearance_m{.10};
  double avoidance_curve_enabled{1},avoidance_curve_samples{17},avoidance_length_slack_m{.50};
  double avoidance_lane_weight{2.2},avoidance_steering_weight{.1};
  double clearance_weight{.6};
  double reference_distance{1.0},lane_lookahead{.05},hist_low{1.0},hist_high{2.0};
  // Pre-Start control tuning shares the numeric editor with geometric policy.
  // Duty/current fields are used only by the real VESC adapter, not by rollouts.
  double target_recovery_acceleration{.8},drive_start_duty{.053},drive_rise{.04},drive_fall{.03};
  double drive_recovery_rise{.045},drive_recovery_ramp{.20};
  double drive_brake_max{2},drive_brake_rise{1.5},drive_brake_fall{6},drive_brake_speed_gain{.8};
  void validate() const {
    for(double value:{target_recovery_acceleration,drive_start_duty,drive_rise,drive_fall,drive_recovery_rise,
      drive_recovery_ramp,drive_brake_max,drive_brake_rise,drive_brake_fall,drive_brake_speed_gain})
      if(!std::isfinite(value))throw std::invalid_argument("nonfinite acceleration/brake tuning");
    if(target_recovery_acceleration<.05 || target_recovery_acceleration>3 || drive_start_duty<0 || drive_start_duty>1 ||
      drive_rise<.001 || drive_rise>1 || drive_fall<.001 || drive_fall>1 || drive_recovery_rise<.001 || drive_recovery_rise>1 ||
      drive_recovery_ramp<.01 || drive_recovery_ramp>5 || drive_brake_max<.1 || drive_brake_max>8 ||
      drive_brake_rise<.1 || drive_brake_rise>64 || drive_brake_fall<.1 || drive_brake_fall>64 ||
      drive_brake_speed_gain<.01 || drive_brake_speed_gain>10)
      throw std::invalid_argument("acceleration/brake tuning outside editable bounds");
    for(double value:{hazard_hold_sec,obstacle_memory_enabled,obstacle_memory_sec,obstacle_memory_distance_m,obstacle_memory_clearance_m})
      if(!std::isfinite(value))throw std::invalid_argument("nonfinite obstacle retention parameter");
    if(hazard_hold_sec<0 || hazard_hold_sec>1 || (obstacle_memory_enabled!=0 && obstacle_memory_enabled!=1) ||
      obstacle_memory_sec<0 || obstacle_memory_sec>5 || obstacle_memory_distance_m<.1 || obstacle_memory_distance_m>2 ||
      obstacle_memory_clearance_m<0 || obstacle_memory_clearance_m>.5)
      throw std::invalid_argument("obstacle retention: hold 0..1s, memory 0/1, age 0..5s, distance .1..2m, rear clearance 0..0.5m");
    if(!std::isfinite(lane_memory_enabled) || (lane_memory_enabled!=0 && lane_memory_enabled!=1))
      throw std::invalid_argument("lane_memory_enabled must be 0 or 1");
    for (double a:{speed,horizon,step,wheelbase,width,rear,front,margin,max_steer,steer_rate,
      lane_width,deceleration,delay,reference_distance,lane_lookahead,hist_low,hist_high,
      minimum_path_length,uncertainty,max_curvature,lane_lateral_acceleration,avoidance_lateral_acceleration,
      stanley_gain,stanley_softening,stanley_corner_heading,stanley_opposing,geometry_window,steering_weight,control_rate})
      if (!std::isfinite(a) || a<0) throw std::invalid_argument("nonfinite/negative VFH parameter");
    for(double a:{hazard_filter_enabled,hazard_confirm_frames,hazard_small_max_cells,hazard_match_radius_cells,hazard_max_gap_sec})
      if(!std::isfinite(a))throw std::invalid_argument("nonfinite hazard filter parameter");
    if((hazard_filter_enabled!=0 && hazard_filter_enabled!=1) ||
      hazard_confirm_frames<1 || hazard_confirm_frames>5 || std::floor(hazard_confirm_frames)!=hazard_confirm_frames ||
      hazard_small_max_cells<1 || hazard_small_max_cells>50 || std::floor(hazard_small_max_cells)!=hazard_small_max_cells ||
      hazard_match_radius_cells<0 || hazard_match_radius_cells>2 || std::floor(hazard_match_radius_cells)!=hazard_match_radius_cells ||
      hazard_max_gap_sec<.02 || hazard_max_gap_sec>.5)
      throw std::invalid_argument("invalid hazard confirmation limits/integer values");
    for(double a:{avoidance_curve_enabled,avoidance_curve_samples,avoidance_length_slack_m,avoidance_lane_weight,avoidance_steering_weight})
      if(!std::isfinite(a))throw std::invalid_argument("nonfinite avoidance tuning");
    if((avoidance_curve_enabled!=0 && avoidance_curve_enabled!=1) ||
      avoidance_curve_samples<3 || avoidance_curve_samples>17 || std::floor(avoidance_curve_samples)!=avoidance_curve_samples ||
      static_cast<int>(avoidance_curve_samples)%2!=1 || avoidance_length_slack_m<0 || avoidance_length_slack_m>.5 ||
      avoidance_lane_weight<0 || avoidance_lane_weight>5 || avoidance_steering_weight<0 || avoidance_steering_weight>5)
      throw std::invalid_argument("avoidance: curves 0/1, samples odd 3..17, slack 0..0.5m, weights 0..5");
    if(!std::isfinite(current_speed) || std::abs(current_speed)>6 || minimum_path_length<.02 || minimum_path_length>1 ||
      uncertainty>.5 || max_curvature<.1 || max_curvature>10 || lane_lateral_acceleration<=0 ||
      avoidance_lateral_acceleration<=0 || geometry_window<=0 || steering_weight<=0 || steering_weight>1 ||
      control_rate<10 || control_rate>200 || stanley_opposing>=1 || stanley_corner_heading>pi)
      throw std::invalid_argument("Invalid source controller planning settings");
    if(!std::isfinite(clearance_weight) || clearance_weight<0 || clearance_weight>5 || rear<wheelbase)
      throw std::invalid_argument("invalid clearance weight or vehicle geometry");
    if (!std::isfinite(current_steer) || std::abs(current_steer)>max_steer || speed<.05 || speed>3 || horizon<.3 || horizon>2.5 || step<.02 || step>.08 ||
      wheelbase<.1 || wheelbase>1 || width<.1 || width>1 || rear>1 || front>.5 || margin>.2 ||
      max_steer<.05 || max_steer>pi/3 || steer_rate<.1 || steer_rate>20 ||
      lane_width<width || lane_width>2 || deceleration<.1 || deceleration>3 || delay>1 || reference_distance<.3 || lane_lookahead<.01 || lane_lookahead>.8 || hist_low>=hist_high || hist_high>1000)
      throw std::invalid_argument("VFH parameter outside test limits");
  }
};
inline double wrap(double a) {return std::atan2(std::sin(a),std::cos(a));}
inline double laneDistance(Point p,const std::vector<Point>& lane) {
  double best_squared=1e18;
  for (std::size_t i=1;i<lane.size();++i) {
    const auto a=lane[i-1],b=lane[i]; const double dx=b.x-a.x,dy=b.y-a.y,d2=dx*dx+dy*dy;
    if (d2<1e-9) continue;
    const double t=std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/d2,0.,1.);
    const double ex=p.x-a.x-t*dx,ey=p.y-a.y-t*dy;
    best_squared=std::min(best_squared,ex*ex+ey*ey);
  }
  return std::sqrt(best_squared);
}
struct Candidate {std::vector<Pose> path; double score{},length{},direction{},command_steer{}; bool valid{};};
// Follow the complete ordered lane polyline, not a single far target.
class Polyline {
  const std::vector<Point>& points_;
  std::vector<double> arc_;
public:
  explicit Polyline(const std::vector<Point>& points):points_(points),arc_(points.size(),0) {
    for(std::size_t i=1;i<points.size();++i)arc_[i]=arc_[i-1]+std::hypot(points[i].x-points[i-1].x,points[i].y-points[i-1].y);
  }
  double length()const{return arc_.empty()?0:arc_.back();}
  double nearest(Point p,double minimum=0)const {
    double best=1e30,result=minimum;
    for(std::size_t i=1;i<points_.size();++i) {
      if(arc_[i]<minimum)continue;
      const auto a=points_[i-1],b=points_[i];const double dx=b.x-a.x,dy=b.y-a.y,d2=dx*dx+dy*dy;
      const double span=arc_[i]-arc_[i-1];if(span<1e-9)continue;
      const double t=std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/d2,std::clamp((minimum-arc_[i-1])/span,0.,1.),1.);
      const double ex=p.x-a.x-t*dx,ey=p.y-a.y-t*dy,d=ex*ex+ey*ey;
      if(d<best){best=d;result=arc_[i-1]+t*span;}
    }
    return result;
  }
  Point at(double s)const {
    if(s<=0)return points_.front();if(s>=length())return points_.back();
    const auto upper=std::upper_bound(arc_.begin(),arc_.end(),s);
    const std::size_t i=static_cast<std::size_t>(upper-arc_.begin());
    const double t=(s-arc_[i-1])/(arc_[i]-arc_[i-1]);
    return {points_[i-1].x+t*(points_[i].x-points_[i-1].x),points_[i-1].y+t*(points_[i].y-points_[i-1].y)};
  }
  double heading(Point p)const {
    const double s=nearest(p);const auto a=at(std::max(0.,s-.025)),b=at(std::min(length(),s+.025));
    return std::atan2(b.y-a.y,b.x-a.x);
  }
};
inline double steeringStep(double current,double desired,double dt,const Config& c) {
  const double weight=1-std::pow(1-c.steering_weight,dt*c.control_rate);
  const double filtered=weight*desired+(1-weight)*current;
  return current+std::clamp(filtered-current,-c.steer_rate*dt,c.steer_rate*dt);
}
inline Candidate followReference(const std::vector<Point>& reference,const Config& c) {
  Candidate out;if(reference.size()<2)return out;
  const Polyline curve(reference);if(curve.length()<1e-6)return out;
  auto_control::OrderedPath local;local.geometry_window_m=c.geometry_window;
  local.points.resize(reference.size());local.arc_m.assign(reference.size(),0);
  for(std::size_t i=1;i<reference.size();++i)local.arc_m[i]=local.arc_m[i-1]+std::hypot(reference[i].x-reference[i-1].x,reference[i].y-reference[i-1].y);
  Pose p;p.steer=c.current_steer;out.path.push_back(p);
  const double dt=c.step/c.speed,limit=std::min(c.max_steer,std::atan(c.wheelbase*c.max_curvature));
  for(double distance=c.step;distance<=c.horizon+1e-8;distance+=c.step) {
    // Continue with the last observed tangent beyond a short current fragment.
    // The complete predicted body sweep is still checked against terrain below.
    const double cs=std::cos(p.yaw),sn=std::sin(p.yaw);
    for(std::size_t i=0;i<reference.size();++i) {
      const double x=reference[i].x-p.x,y=reference[i].y-p.y;
      local.points[i]={cs*x+sn*y,-sn*x+cs*y};
    }
    const auto steering=auto_control::stanley_control(local,out.path.size()==1?c.current_speed:c.speed,
      c.stanley_gain,c.stanley_softening,c.lane_lookahead,limit,c.stanley_corner_heading,c.stanley_opposing);
    const double desired=steering.steering_angle_rad;
    if(out.path.size()==1)out.command_steer=desired;
    p.steer=steeringStep(p.steer,desired,dt,c);
    const double turn=c.step*std::tan(p.steer)/c.wheelbase;
    const double middle=p.yaw+turn/2;
    p.x+=c.step*std::cos(middle)-c.wheelbase*turn*std::sin(middle);
    p.y+=c.step*std::sin(middle)+c.wheelbase*turn*std::cos(middle);p.yaw=wrap(p.yaw+turn);
    if(std::abs(p.yaw)>=pi/2)break;
    out.path.push_back(p);out.length=distance;
  }
  return out;
}
enum class Block {None,Hazard};
inline Block footprintBlock(const Grid& g,const Config& c,const Pose& p) {
  const double half=c.width/2+c.margin+c.uncertainty,cs=std::cos(p.yaw),sn=std::sin(p.yaw);
  const double back=c.rear+c.uncertainty,front=c.front+c.margin+c.uncertainty;
  const double longitudinal_half=(front+back)/2,offset=(front-back)/2;
  const double center_x=p.x+offset*cs,center_y=p.y+offset*sn,cell_half=g.resolution/2;
  const double x_radius=longitudinal_half*std::abs(cs)+half*std::abs(sn);
  const double y_radius=longitudinal_half*std::abs(sn)+half*std::abs(cs);
  const int u0=std::max(0,int(std::floor((center_x-x_radius-cell_half-g.x0)/g.resolution)));
  const int u1=std::min(g.width-1,int(std::ceil((center_x+x_radius+cell_half-g.x0)/g.resolution)));
  const int v0=std::max(0,int(std::floor((center_y-y_radius-cell_half-g.y0)/g.resolution)));
  const int v1=std::min(g.height-1,int(std::ceil((center_y+y_radius+cell_half-g.y0)/g.resolution)));
  const double cell_projection=cell_half*(std::abs(cs)+std::abs(sn));
  for(int v=v0;v<=v1;++v)for(int u=u0;u<=u1;++u) {
    const double x=g.x0+(u+.5)*g.resolution,y=g.y0+(v+.5)*g.resolution;
    const double dx=x-center_x,dy=y-center_y;
    // Rectangle/cell separating-axis test. Cell geometry is not a second fixed
    // safety margin: use the supplied body + clearance + uncertainty once.
    if(std::abs(dx)>x_radius+cell_half || std::abs(dy)>y_radius+cell_half ||
      std::abs(dx*cs+dy*sn)>longitudinal_half+cell_projection ||
      std::abs(-dx*sn+dy*cs)>half+cell_projection)continue;
    const auto index=std::size_t(v)*g.width+u;
    // Ignore fresh self-returns only. Retained hazards behind the front axle
    // remain collision evidence, including when motion uncertainty overlaps us.
    if(!retainedTerrain(g.cells[index]) && x>=-c.rear && x<=c.front && std::abs(y)<=c.width/2)continue;
    if(blockedTerrain(g.cells[index]))return Block::Hazard;
  }
  return Block::None;
}
struct Plan {
  std::vector<Candidate> candidates; std::vector<Pose> path;
  std::array<double,61> histogram{};
  std::array<bool,61> binary{},masked{};
  double speed{},steer{},length{},longest_candidate{}; bool used_vfh{},used_fallback{},reference_blocked{},rear_unverified{true};
  std::size_t fallback_candidates{};
  std::size_t hazard_ends{},short_candidates{};
  std::string status{"STOP: no feasible route"};
};
// VFH+ four-stage histogram reduction, followed by Ackermann rollout verification.
// Retained observations behind the front axle also enter footprint verification.
class Planner {
  std::array<bool,61> binary_{};
  double previous_angle_{0},previous_command_{0},binary_rotation_residual_{0};
  bool have_previous_{false};
public:
  void reset() {previous_angle_=previous_command_=binary_rotation_residual_=0;have_previous_=false;binary_.fill(false);}
  void rotateReference(double yaw) {
    previous_angle_=wrap(previous_angle_-yaw);
    binary_rotation_residual_+=yaw;
    const int shift=int(std::lround(binary_rotation_residual_/(2.5*pi/180)));
    binary_rotation_residual_-=shift*(2.5*pi/180);
    if(shift) {
      const auto old=binary_;
      for(int b=0;b<61;++b)binary_[b]=(b+shift>=0 && b+shift<61)?old[b+shift]:true;
    }
  }
  Plan plan(const Grid& g,const std::vector<Point>& lane,const Config& c,bool direction_only=false) {
    g.validate();c.validate();Plan out;
    const int curve_samples=static_cast<int>(c.avoidance_curve_samples);
    const std::size_t candidate_capacity=1+51+(c.avoidance_curve_enabled!=0?(curve_samples+1)*4:0);
    out.candidates.reserve(candidate_capacity);
    if(lane.size()>2000){out.status="STOP: lane sample limit";return out;}
    for(auto p:lane)if(!std::isfinite(p.x)||!std::isfinite(p.y)){out.status="STOP: invalid lane";return out;}
    if(lane.size()<2){out.status="STOP: lane reference unavailable";return out;}
    const auto& reference=lane;
    const Polyline reference_curve(reference);
    if(reference_curve.length()<1e-6){out.status="STOP: degenerate lane reference";return out;}
    Point ref=reference.back();
    for(auto p:reference)if(std::hypot(p.x,p.y)>=c.reference_distance){ref=p;break;}
    const double target_angle=std::clamp(std::atan2(ref.y,std::max(.05,ref.x)),-pi/2,pi/2);
    const double half=c.width/2+c.margin+c.uncertainty,radius=half+g.resolution*.71;
    const double steering_limit=std::min(c.max_steer,std::atan(c.wheelbase*c.max_curvature));
    const double turn_radius=c.wheelbase/std::tan(steering_limit);
    const double front_turn_radius=std::hypot(turn_radius,c.wheelbase);
    // Cache lane distances only for cells visited by the footprint or scorer.
    std::vector<double> lane_distance(g.cells.size(),-1.);
    const auto cellLaneDistance=[&](int u,int v) {
      auto& distance=lane_distance[std::size_t(v)*g.width+u];
      if(distance<0)distance=laneDistance({g.x0+(u+.5)*g.resolution,g.y0+(v+.5)*g.resolution},lane);
      return distance;
    };
    const auto distanceToLane=[&](Point p) {
      const int u=int(std::floor((p.x-g.x0)/g.resolution)),v=int(std::floor((p.y-g.y0)/g.resolution));
      if(u>=0 && v>=0 && u<g.width && v<g.height)return cellLaneDistance(u,v);
      return laneDistance(p,lane);
    };
    const auto setSpeed=[&]() {
      const auto last=out.path.back();previous_angle_=std::atan2(last.y,last.x);
      previous_command_=out.steer;have_previous_=true;
      out.speed=std::min(c.speed,-c.deceleration*c.delay+
        std::sqrt(c.deceleration*c.deceleration*c.delay*c.delay+2*c.deceleration*out.length));
      double curvature=0;for(auto p:out.path)curvature=std::max(curvature,std::abs(std::tan(p.steer)/c.wheelbase));
      if(out.used_vfh || c.curvature_speed_control) {
        const double lateral=out.used_vfh?c.avoidance_lateral_acceleration:c.lane_lateral_acceleration;
        out.speed=std::min(out.speed,std::sqrt(lateral/std::max(.01,curvature)));
      }
    };
    // Resume direct tracking only when its swept footprint is clear.
    if(!direction_only) {
      auto track=followReference(reference,c);
      out.reference_blocked=std::any_of(track.path.begin(),track.path.end(),[&](const Pose& p) {
        return footprintBlock(g,c,p)==Block::Hazard;
      });
      const double usable=std::max(0.,track.length-c.step);
      if(!out.reference_blocked && usable+1e-9>=c.minimum_path_length && track.path.size()>2) {
        binary_.fill(false);binary_rotation_residual_=0;
        out.path=std::move(track.path);out.length=usable;out.steer=track.command_steer;setSpeed();
        out.status="LANE: clear tracking/return trajectory (VFH bypass)";
        return out;
      }
    }
    out.candidates.emplace_back();
    out.used_vfh=true;
    double left_limit=pi,right_limit=-pi;
    constexpr double first_angle=-75*pi/180,bin_step=2.5*pi/180;
    const double histogram_range=c.horizon+.3,cell_weight=std::pow(g.resolution/.05,2);
    for (int v=0;v<g.height;++v) for (int u=0;u<g.width;++u) {
      if (!blockedTerrain(g.cells[std::size_t(v)*g.width+u])) continue;
      const double x=g.x0+(u+.5)*g.resolution,y=g.y0+(v+.5)*g.resolution;
      if (x<0 || (x<c.front && std::abs(y)<c.width/2)) continue;
      const double d=std::hypot(x,y); if (d<.05 || d>histogram_range) continue;
      const double angle=std::atan2(y,x);
      // Minimum-turn-circle masking, VFH+ stage 3. Rectangle verification follows.
      if (angle>0 && std::hypot(x+c.wheelbase,y-turn_radius)<front_turn_radius+radius) left_limit=std::min(left_limit,angle);
      if (angle<0 && std::hypot(x+c.wheelbase,y+turn_radius)<front_turn_radius+radius) right_limit=std::max(right_limit,angle);
      const double spread=std::asin(std::min(1.,radius/d));
      const int first=std::max(0,int(std::ceil((angle-spread-first_angle)/bin_step-1e-12)));
      const int last=std::min(60,int(std::floor((angle+spread-first_angle)/bin_step+1e-12)));
      const double weight=std::max(0.,1-d*d/(histogram_range*histogram_range))*cell_weight;
      for(int b=first;b<=last;++b)out.histogram[b]+=weight;
    }
    for (int b=0;b<61;++b) {
      if (out.histogram[b]>c.hist_high) binary_[b]=true;
      else if (out.histogram[b]<c.hist_low) binary_[b]=false;
      out.binary[b]=binary_[b];
      const double angle=(-75+2.5*b)*pi/180;
      out.masked[b]=binary_[b] || angle>=left_limit || angle<=right_limit;
    }
    // Keep the preferred lane direction, prior direction, and low-density valley directions.
    std::vector<double> angles;
    for (int b=0;b<61;) {
      if (out.masked[b]) {++b;continue;}
      const int first=b;while (b+1<61 && !out.masked[b+1])++b;
      const double lo=(-75+2.5*first)*pi/180,hi=(-75+2.5*b)*pi/180;
      // Narrow valley: center. Wide valley: two inset borders and in-valley references.
      if (hi-lo<=40*pi/180) angles.push_back((lo+hi)/2);
      else {
        const double a=lo+20*pi/180,z=hi-20*pi/180;
        angles.push_back(a);angles.push_back(z);
        for (double ref:{target_angle,0.,previous_angle_}) if (ref>=a && ref<=z) angles.push_back(ref);
      }
      ++b;
    }
    // Additional in-valley directions let a long Ackermann body form a lane-change curve.
    for(int b=6;b<61;b+=4)if(!out.masked[b])angles.push_back((-75+2.5*b)*pi/180);
    std::sort(angles.begin(),angles.end(),[&](double a,double b){const double da=std::abs(a-target_angle),db=std::abs(b-target_angle);return std::abs(da-db)<1e-9 ? a<b : da<db;});
    std::vector<double> targets;
    for (double a:angles) {
      bool duplicate=false; for (double b:targets) if (std::abs(a-b)<.04) duplicate=true;
      if (!duplicate) targets.push_back(a);
      if (targets.size()>=17) break;
    }
    const bool all_masked=targets.empty();
    const double dt=c.step/c.speed;
    std::vector<bool> fallback;fallback.reserve(candidate_capacity);fallback.push_back(false); // Parallel to candidates, including reference.
    // All alternatives use the SAME rectangle/cell, and grid checks.
    const auto rollout=[&](auto steering,bool is_fallback,double direction) {
      Candidate candidate;candidate.direction=direction;candidate.path.reserve(int(c.horizon/c.step)+1);
      Pose p;p.steer=c.current_steer;candidate.path.push_back(p);
      for(double s=c.step;s<=c.horizon+1e-8;s+=c.step) {
        const double desired=std::clamp(steering(p,s),-steering_limit,steering_limit);
        if(candidate.path.size()==1)candidate.command_steer=desired;
        p.steer=steeringStep(p.steer,desired,dt,c);
        const double turn=c.step*std::tan(p.steer)/c.wheelbase,middle=p.yaw+turn/2;
        p.x+=c.step*std::cos(middle)-c.wheelbase*turn*std::sin(middle);
        p.y+=c.step*std::sin(middle)+c.wheelbase*turn*std::cos(middle);p.yaw=wrap(p.yaw+turn);
        if(std::abs(p.yaw)>=pi/2 || p.x<0)break;
        if(footprintBlock(g,c,p)==Block::Hazard){++out.hazard_ends;break;}
        candidate.path.push_back(p);candidate.length=s;
      }
      const double usable=std::max(0.,candidate.length-c.step);
      out.longest_candidate=std::max(out.longest_candidate,usable);
      candidate.valid=usable+1e-9>=c.minimum_path_length && candidate.path.size()>2;
      if(!candidate.valid)++out.short_candidates;
      if(is_fallback)++out.fallback_candidates;
      out.candidates.push_back(std::move(candidate));fallback.push_back(is_fallback);
    };
    const auto generateCandidates=[&]() {
      // Preferred VFH+ valleys and their three original steering profiles.
      for(double angle:targets)for(int profile=0;profile<3;++profile) {
        const Point goal{c.reference_distance*std::cos(angle),c.reference_distance*std::sin(angle)};
        const double initial=std::atan(2*c.wheelbase*std::sin(angle)/c.reference_distance)*(profile==1?2.:3.);
        rollout([&](const Pose& p,double s) {
          if(profile==0) {
            const double dx=goal.x-p.x,dy=goal.y-p.y,d2=dx*dx+dy*dy;
            return std::atan2(2*c.wheelbase*(-std::sin(p.yaw)*dx+std::cos(p.yaw)*dy),std::max(.1,d2));
          }
          return s<c.horizon/3?initial:(s<2*c.horizon/3?-initial:0.);
        },false,angle);
      }
      // A blocked polar ray does not prove every curved path collides. These
      // bounded extra profiles are still subject to the complete footprint check.
      if(c.avoidance_curve_enabled!=0) {
        std::vector<double> seeds;seeds.reserve(curve_samples+1);
        const int half_samples=(curve_samples-1)/2;
        for(int i=-half_samples;i<=half_samples;++i)seeds.push_back(steering_limit*i/half_samples);
        const double current=std::clamp(c.current_steer,-steering_limit,steering_limit);
        if(std::none_of(seeds.begin(),seeds.end(),[&](double a){return std::abs(a-current)<1e-4;}))seeds.push_back(current);
        for(double initial:seeds)for(int profile=0;profile<4;++profile) {
          rollout([&](const Pose&,double s) {
            if(profile==3)return s<std::min(.20,c.horizon*.2)?0.:initial;
            if(profile==0 || s<c.horizon*.4)return initial;
            if(profile==1 || s>=c.horizon*.8)return 0.;
            return -initial;
          },true,std::atan(std::tan(initial)*c.reference_distance/c.wheelbase));
        }
      }
    };
    generateCandidates();
    double longest=0;
    for(std::size_t i=1;i<out.candidates.size();++i)if(out.candidates[i].valid)
      longest=std::max(longest,std::max(0.,out.candidates[i].length-c.step));
    // Capped distance to the nearest hazard, in cells. This affects ranking
    // only; the rectangle intersection remains the collision decision.
    std::vector<int> clearance;
    const int clearance_cap=std::max(1,int(std::ceil((half+.12)/g.resolution)));
    clearance.resize(g.cells.size(),clearance_cap);
    for(std::size_t i=0;i<g.cells.size();++i)if(blockedTerrain(g.cells[i]))clearance[i]=0;
    for(int v=0;v<g.height;++v)for(int u=0;u<g.width;++u) {
      const auto i=std::size_t(v)*g.width+u;
      if(u>0)clearance[i]=std::min(clearance[i],clearance[i-1]+1);
      if(v>0)clearance[i]=std::min(clearance[i],clearance[i-g.width]+1);
    }
    for(int v=g.height-1;v>=0;--v)for(int u=g.width-1;u>=0;--u) {
      const auto i=std::size_t(v)*g.width+u;
      if(u+1<g.width)clearance[i]=std::min(clearance[i],clearance[i+1]+1);
      if(v+1<g.height)clearance[i]=std::min(clearance[i],clearance[i+g.width]+1);
    }
    // Progress first: lane proximity must not select a short dead-end prefix
    // instead of a substantially longer feasible detour. Within the configured
    // length slack, use soft lane-return and steering-continuity costs.
    const double length_slack=c.avoidance_length_slack_m;
    double best_score=std::numeric_limits<double>::infinity();std::size_t best_index=0;
    for(std::size_t i=1;i<out.candidates.size();++i) {
      auto& candidate=out.candidates[i];if(!candidate.valid)continue;
      const double usable=std::max(0.,candidate.length-c.step);
      if(usable+length_slack+1e-9<longest)continue;
      const auto last=candidate.path.back();
      double deviation=0.;
      for(const auto& p:candidate.path)deviation+=distanceToLane({p.x,p.y});
      deviation/=candidate.path.size();
      const double lane_cost=std::min(3.,deviation+.5*distanceToLane({last.x,last.y})+
        .2*std::abs(wrap(last.yaw-reference_curve.heading({last.x,last.y}))));
      double clearance_reward=0;
      for(const auto& p:candidate.path) {
        const int u=int(std::floor((p.x-g.x0)/g.resolution)),v=int(std::floor((p.y-g.y0)/g.resolution));
        if(u>=0 && v>=0 && u<g.width && v<g.height)
          clearance_reward+=clearance[std::size_t(v)*g.width+u]*g.resolution;
      }
      clearance_reward/=candidate.path.size();
      const double continuity_cost=.35*std::abs(candidate.command_steer-c.current_steer)/steering_limit+
        (have_previous_?.2*std::abs(candidate.command_steer-previous_command_)/steering_limit+
          .25*std::abs(wrap(std::atan2(last.y,last.x)-previous_angle_)):0.);
      candidate.score=2*(longest-usable)-c.clearance_weight*clearance_reward-.5*last.x+
        .15*c.avoidance_steering_weight*continuity_cost+
        .18*std::abs(last.yaw)+.6*c.avoidance_lane_weight*lane_cost;
      if(candidate.score<best_score) {
        best_score=candidate.score;best_index=i;
      }
    }
    if(best_index==0) {
      out.status="STOP: no collision-free curve >="+std::to_string(c.minimum_path_length)+"m; hazard="+
        std::to_string(out.hazard_ends);
      return out;
    }
    const auto& best=out.candidates[best_index];
    out.path=best.path;out.length=std::max(0.,best.length-c.step);
    out.steer=best.command_steer;out.used_fallback=fallback[best_index];
    setSpeed();
    out.status=out.used_fallback?(all_masked?"AVOID: checked curve (VFH fully masked); lane return preferred":
      "AVOID: checked curve; lane return preferred"):"AVOID: VFH+ valley; lane return preferred";
    return out;
  }
};
}
