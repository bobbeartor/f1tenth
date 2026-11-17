#include "roi_vfh_test/hazard_filter.hpp"
#include "roi_vfh_test/obstacle_memory.hpp"
#include "roi_vfh_test/policy_state.hpp"
#include "roi_vfh_test/speed_policy.hpp"
#include "frenet_planner.hpp"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <unordered_map>

using namespace roi_vfh_test;
using Clock = std::chrono::steady_clock;
struct SimulationProfile {
  Config config;
  double speed_ceiling{3},deceleration_ceiling{3},max_age{.30};
  bool slowdown{true};
  double slowdown_cap{1},clear_sec{1},recovery_acceleration{.8};
  double steering_ceiling_deg{60},steering_rate_ceiling_deg_per_sec{1000};
  std::string fingerprint;
};
#if __has_include("simulation_profile.hpp")
#include "simulation_profile.hpp"
#else
// Legacy offline tools may compile this source directly. The served simulator
// always compiles the generated header from the real ROS YAML profile.
SimulationProfile simulationProfile() {return {};}
#endif

int main(int argc,char** argv) {
  const bool use_vfh=argc<=1 || std::string(argv[1])!="frenet";
  const auto profile=simulationProfile();
  struct Session {
    Planner planner;
    HazardFilter filter;
    ObstacleMemory obstacle_memory;
    LaneMemory lane_memory;
    TargetSpeedPolicy speed_policy;
    std::optional<MotionPose> previous_pose;
    std::optional<HazardResult> filtered;
    Plan result;
    LaneReference reference;
    double filter_capture{-1},last_lane{-2},last_depth{-2},last_now{-1};
    std::size_t last_use{0};
  };
  std::unordered_map<std::string,Session> sessions;
  std::size_t request_count=0;
  std::cout << std::setprecision(12);
  for (std::string line; std::getline(std::cin, line);) {
    try {
      std::istringstream input(line);
      std::string session,version;
      input >> version;
      const bool v2=version=="V2";
      if(v2) {
        std::string fingerprint;input>>fingerprint;
        if(fingerprint!=profile.fingerprint)throw std::invalid_argument("page/worker profile mismatch: reload the page");
        input>>session;
      }else session=version;
      Config c=profile.config;
      MotionPose pose;
      double plan_hz,unused_seconds,unused_length,unused_offset,unused_angle;
      Grid grid;
      int lane_count,hazard_count;
      input >> pose.t >> pose.x >> pose.y >> pose.yaw >> c.current_speed >> c.current_steer
        >> plan_hz >> c.speed >> c.horizon >> c.step >> c.margin
        >> c.lane_width >> c.width >> c.rear >> c.front >> c.wheelbase
        >> c.max_steer >> c.steer_rate >> c.deceleration >> c.delay
        >> c.lane_lookahead >> c.reference_distance >> c.hist_low >> c.hist_high
        >> c.hazard_filter_enabled >> c.hazard_confirm_frames >> c.hazard_small_max_cells
        >> c.hazard_match_radius_cells >> c.hazard_max_gap_sec >> c.avoidance_curve_enabled
        >> c.avoidance_curve_samples >> c.avoidance_length_slack_m >> c.avoidance_lane_weight
        >> c.avoidance_steering_weight >> grid.width >> grid.height >> grid.resolution
        >> grid.y0 >> unused_seconds >> unused_length >> unused_offset >> unused_angle
        >> c.clearance_weight >> lane_count >> hazard_count;
      if (!input || !std::isfinite(plan_hz) || plan_hz<5 || plan_hz>100 ||
          grid.width<1 || grid.width>2000 || grid.height<1 || grid.height>2000 ||
          grid.width*grid.height>250000 || lane_count<0 || lane_count>2000 ||
          hazard_count<0 || hazard_count>grid.width*grid.height)throw std::invalid_argument("invalid simulation input");
      grid.cells.assign(grid.width*grid.height,-1);
      std::vector<Point> observed(lane_count);
      for(auto& point:observed)input>>point.x>>point.y;
      const auto read_hazards=[&](Grid& g,int count) {
        for(int i=0;i<count;++i) {
          int index=-1;
          if(!(input>>index) || index<0 || index>=int(g.cells.size()))throw std::invalid_argument("invalid terrain cell index");
          g.cells[index]=100;
        }
      };
      read_hazards(grid,hazard_count);
      double now=pose.t,lane_capture=pose.t;
      MotionPose lane_pose=pose;
      struct Depth {MotionPose pose;Grid grid;};
      std::vector<Depth> depth_frames;
      if(v2) {
        int count=0;
        input>>c.lane_memory_enabled>>now>>lane_capture>>lane_pose.x>>lane_pose.y>>lane_pose.yaw>>count;
        lane_pose.t=lane_capture;
        if(!input || count<0 || count>64)throw std::invalid_argument("invalid depth batch");
        for(int i=0;i<count;++i) {
          Depth frame;int hazards=0;
          input>>frame.pose.t>>frame.pose.x>>frame.pose.y>>frame.pose.yaw>>hazards;
          if(!input || !std::isfinite(frame.pose.t) || !std::isfinite(frame.pose.x) ||
              !std::isfinite(frame.pose.y) || !std::isfinite(frame.pose.yaw) ||
              hazards<0 || hazards>int(grid.cells.size()))
            throw std::invalid_argument("invalid depth capture");
          frame.grid=grid;std::fill(frame.grid.cells.begin(),frame.grid.cells.end(),-1);
          read_hazards(frame.grid,hazards);depth_frames.push_back(std::move(frame));
        }
        input>>c.uncertainty>>c.hazard_hold_sec>>c.obstacle_memory_enabled>>c.obstacle_memory_sec
          >>c.obstacle_memory_distance_m>>c.obstacle_memory_clearance_m;
        input>>c.target_recovery_acceleration>>c.steering_weight>>c.stanley_gain>>c.stanley_softening
          >>c.max_curvature>>c.stanley_corner_heading>>c.stanley_opposing;
      }else depth_frames.push_back({pose,grid});
      if(!input)throw std::invalid_argument("incomplete simulation input");
      input>>std::ws;
      if(!input.eof())throw std::invalid_argument("unexpected data: evaluation marks are not planner inputs");
      for(double value:{now,pose.t,pose.x,pose.y,pose.yaw,lane_capture,lane_pose.x,lane_pose.y,lane_pose.yaw})
        if(!std::isfinite(value))throw std::invalid_argument("nonfinite capture pose");
      if(now-pose.t<-.02 || now-pose.t>profile.max_age)throw std::invalid_argument("terrain capture expired");
      if(c.speed>profile.speed_ceiling || c.deceleration>profile.deceleration_ceiling)
        throw std::invalid_argument("speed/deceleration exceeds vehicle YAML ceiling");
      if(c.max_steer*180/pi>profile.steering_ceiling_deg+1e-9 || c.steer_rate*180/pi>profile.steering_rate_ceiling_deg_per_sec+1e-9)
        throw std::invalid_argument("steering exceeds vehicle YAML ceiling");
      c.delay=std::max(c.delay,std::max(0.,now-pose.t)+1/plan_hz);
      c.validate();grid.validate();
      if(sessions.size()>=8 && !sessions.count(session)) {
        auto oldest=std::min_element(sessions.begin(),sessions.end(),[](const auto& a,const auto& b){return a.second.last_use<b.second.last_use;});
        sessions.erase(oldest);
      }
      auto& state=sessions[session];
      if((state.last_now>=0 && now<state.last_now) || (state.previous_pose && pose.t<state.previous_pose->t))state=Session{};
      state.last_use=++request_count;state.last_now=now;
      if(now-lane_capture<-.02 || now-lane_capture>profile.max_age || observed.size()<2)observed.clear();
      for(auto& point:observed)point=reproject(point,lane_pose,pose);
      const double usable_lane=observed.empty()?-1:lane_capture;
      const bool replanned=!state.previous_pose || state.last_depth!=pose.t || state.last_lane!=usable_lane;
      const auto start=Clock::now();
      std::stable_sort(depth_frames.begin(),depth_frames.end(),[](const auto& a,const auto& b){return a.pose.t<b.pose.t;});
      std::size_t filtered_frames=0;
      for(const auto& frame:depth_frames) {
        if(frame.pose.t<=state.filter_capture || frame.pose.t>pose.t || now-frame.pose.t>profile.max_age)continue;
        state.filtered=state.obstacle_memory.apply(state.filter.apply(frame.grid,frame.pose.t,frame.pose,c),frame.pose.t,frame.pose,c);
        state.filter_capture=frame.pose.t;++filtered_frames;
      }
      if(!state.filtered || state.filter_capture!=pose.t) {
        state.filtered=state.obstacle_memory.apply(state.filter.apply(grid,pose.t,pose,c),pose.t,pose,c);state.filter_capture=pose.t;++filtered_frames;
      }
      const auto after_filter=Clock::now();
      if(replanned) {
        if(use_vfh && state.previous_pose)state.planner.rotateReference(wrap(pose.yaw-state.previous_pose->yaw));
        state.reference=state.lane_memory.resolve(observed,pose,c,observed.empty()?std::nullopt:std::optional<double>(lane_capture));
        const bool missing=!state.reference.trusted;
        if(missing && !state.reference.has_memory)c.avoidance_lane_weight=0;
        state.result=use_vfh?state.planner.plan(state.filtered->grid,state.reference.points,c,missing):
          planFrenet(state.filtered->grid,state.reference.points,c,missing);
        state.result.status="LANE "+state.reference.state+" | "+state.result.status;
        state.speed_policy.apply(state.result,missing,now,c.current_speed,profile.slowdown,
          profile.slowdown_cap,profile.clear_sec,c.target_recovery_acceleration);
        state.previous_pose=pose;state.last_depth=pose.t;state.last_lane=usable_lane;
      }
      const auto finish=Clock::now();
      const auto& result=state.result;
      const auto& filtered=*state.filtered;
      const auto remembered=state.lane_memory.remembered(pose);
      const auto ms=[](auto duration){return std::chrono::duration<double,std::milli>(duration).count();};
      std::cout << "{\"status\":" << std::quoted(result.status)
        << ",\"speed\":" << result.speed << ",\"steer\":" << result.steer
        << ",\"laneMissing\":" << !state.reference.trusted
        << ",\"laneTrust\":" << std::quoted(state.reference.state)
        << ",\"hasLaneMemory\":" << state.reference.has_memory
        << ",\"memoryAge\":" << state.reference.memory_age
        << ",\"replanned\":" << replanned << ",\"depthFrames\":" << filtered_frames
        << ",\"terrainCapture\":" << pose.t << ",\"laneCapture\":" << lane_capture
        << ",\"memoryPath\":[";
      for(std::size_t i=0;i<remembered.size();++i)std::cout<<(i?",":"")<<'['<<remembered[i].x<<','<<remembered[i].y<<']';
      std::cout << "],\"referencePath\":[";
      for(std::size_t i=0;i<state.reference.points.size();++i)
        std::cout<<(i?",":"")<<'['<<state.reference.points[i].x<<','<<state.reference.points[i].y<<']';
      std::cout << "]"
        << ",\"usedVfh\":" << result.used_vfh << ",\"fallback\":" << result.used_fallback
        << ",\"length\":" << result.length << ",\"longestCandidate\":" << result.longest_candidate
        << ",\"hazardEnds\":" << result.hazard_ends << ",\"shortCandidates\":" << result.short_candidates
        << ",\"checkedCandidates\":" << result.candidates.size() << ",\"referenceBlocked\":" << result.reference_blocked
        << ",\"filterMs\":" << ms(after_filter-start) << ",\"planMs\":" << (replanned?ms(finish-after_filter):0.)
        << ",\"raw\":" << filtered.stats.raw << ",\"confirmed\":" << filtered.stats.confirmed
        << ",\"pending\":" << filtered.stats.pending << ",\"heldHazards\":" << filtered.stats.held
        << ",\"nearBodyHazards\":" << filtered.stats.remembered
        << ",\"rememberedHazards\":" << filtered.stats.held+filtered.stats.remembered
        << ",\"grid\":{\"width\":" << filtered.grid.width << ",\"height\":" << filtered.grid.height
        << ",\"res\":" << filtered.grid.resolution << ",\"x0\":" << filtered.grid.x0 << ",\"y0\":" << filtered.grid.y0
        << "},\"path\":[";
      for(std::size_t i=0;i<result.path.size();++i)std::cout<<(i?",":"")<<'['<<result.path[i].x<<','<<result.path[i].y<<']';
      std::cout << "],\"histogram\":[";
      for(std::size_t i=0;i<result.histogram.size();++i)std::cout<<(i?",":"")<<'['<<result.histogram[i]<<','<<result.masked[i]<<']';
      std::cout << "],\"hazards\":[";
      bool comma=false;
      for(std::size_t i=0;i<filtered.grid.cells.size();++i)if(blockedTerrain(filtered.grid.cells[i])) {
        if(comma)std::cout<<',';std::cout<<i;comma=true;
      }
      std::cout << "],\"retainedHazards\":[";
      comma=false;
      for(std::size_t i=0;i<filtered.grid.cells.size();++i)if(retainedTerrain(filtered.grid.cells[i])) {
        if(comma)std::cout<<',';std::cout<<i;comma=true;
      }
      std::cout << "]}\n" << std::flush;
    }catch(const std::exception& error){std::cout << "{\"error\":" << std::quoted(error.what()) << "}\n" << std::flush;}
  }
}
