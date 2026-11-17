#pragma once
#include "roi_vfh_test/hazard_filter.hpp"
#include <unordered_map>

namespace roi_vfh_test {
// Sparse, bounded history of CONFIRMED obstacle cells only. Raw/green/unknown
// cells never become a saved map. Missing samples never refresh last_seen.
class ObstacleMemory {
  struct Sample {Point world;double seen;};
  std::unordered_map<std::uint64_t,Sample> samples_;
  std::deque<std::pair<double,HazardResult>> history_;
  int width_{0},height_{0};double resolution_{0},x0_{0},y0_{0},capture_{-1e9};
  static std::uint64_t key(Point p,double resolution) {
    if(!std::isfinite(p.x) || !std::isfinite(p.y) || std::abs(p.x/resolution)>2e9 || std::abs(p.y/resolution)>2e9)
      throw std::invalid_argument("obstacle memory coordinate outside bounded voxel map");
    const auto x=static_cast<std::int32_t>(std::floor(p.x/resolution));
    const auto y=static_cast<std::int32_t>(std::floor(p.y/resolution));
    return (std::uint64_t(static_cast<std::uint32_t>(x))<<32)|static_cast<std::uint32_t>(y);
  }
  static bool selfReturn(Point p,const Config& c) {
    return p.x>=-c.rear && p.x<=c.front && std::abs(p.y)<=c.width/2;
  }
public:
  void reset() {samples_.clear();history_.clear();width_=height_=0;resolution_=0;capture_=-1e9;}
  HazardResult apply(const HazardResult& current,double capture,const std::optional<MotionPose>& pose,const Config& c) {
    current.grid.validate();c.validate();
    if(!std::isfinite(capture))throw std::invalid_argument("nonfinite obstacle memory capture");
    const auto& raw=current.grid;
    const bool memory=c.obstacle_memory_enabled!=0 && c.obstacle_memory_sec>0;
    if(!pose || (!memory && c.hazard_hold_sec==0)) {reset();return current;}
    for(double value:{pose->x,pose->y,pose->yaw})
      if(!std::isfinite(value))throw std::invalid_argument("nonfinite obstacle memory pose");
    const bool same=width_==raw.width && height_==raw.height && resolution_==raw.resolution && x0_==raw.x0 && y0_==raw.y0;
    if(same)for(const auto& saved:history_)if(saved.first==capture)return saved.second;
    if(!same || capture<=capture_)reset();
    width_=raw.width;height_=raw.height;resolution_=raw.resolution;x0_=raw.x0;y0_=raw.y0;capture_=capture;
    const double cs=std::cos(pose->yaw),sn=std::sin(pose->yaw),res=raw.resolution;
    const auto local=[&](Point p) {const double x=p.x-pose->x,y=p.y-pose->y;return Point{cs*x+sn*y,-sn*x+cs*y};};
    const double passed=-c.rear-c.margin-c.uncertainty-c.obstacle_memory_clearance_m-res*.71;
    const auto nearBody=[&](Point p) {
      const double dx=std::max({-c.rear-p.x,0.,p.x-c.front}),dy=std::max(0.,std::abs(p.y)-c.width/2);
      return std::hypot(dx,dy)<=c.obstacle_memory_distance_m;
    };
    // Remove stale, clearly re-observed free, or rear-cleared evidence. A gray
    // cell is absence of observation, never positive evidence of a free cell.
    for(auto it=samples_.begin();it!=samples_.end();) {
      const auto p=local(it->second.world);const double age=capture-it->second.seen;
      const bool held=age<=c.hazard_hold_sec;
      const bool remembered=memory && age<=c.obstacle_memory_sec && nearBody(p);
      if(p.x<passed || (!held && (!remembered || raw.at(p.x,p.y)==0)))it=samples_.erase(it);
      else ++it;
    }
    for(int v=0;v<raw.height;++v)for(int u=0;u<raw.width;++u) {
      if(!blockedTerrain(raw.cells[std::size_t(v)*raw.width+u]))continue;
      const Point p{raw.x0+(u+.5)*res,raw.y0+(v+.5)*res};
      if(selfReturn(p,c))continue;
      const Point world{pose->x+cs*p.x-sn*p.y,pose->y+sn*p.x+cs*p.y};
      samples_[key(world,res)]={world,capture};
      if(samples_.size()>250000)throw std::invalid_argument("confirmed obstacle memory cell budget exceeded");
    }
    // Fixed aligned apron prevents pixel-origin jitter and lets the EXISTING
    // rectangle/cell SAT check examine retained side/rear hazards efficiently.
    const double side=memory?c.width/2+c.obstacle_memory_distance_m+c.margin+c.uncertainty+2*res:0.;
    const double rear=memory?passed-res:raw.x0;
    const int back=memory?std::max(0,int(std::ceil((raw.x0-rear)/res))):0;
    const int left=memory?std::max(0,int(std::ceil((raw.y0+side)/res))):0;
    const int right=memory?std::max(0,int(std::ceil((side-(raw.y0+raw.height*res))/res))):0;
    HazardResult out;out.stats=current.stats;auto& g=out.grid;g.resolution=res;
    g.width=raw.width+back;g.height=raw.height+left+right;g.x0=raw.x0-back*res;g.y0=raw.y0-left*res;
    if(g.width>2000 || g.height>2000 || std::size_t(g.width)*g.height>250000)
      throw std::invalid_argument("obstacle memory apron exceeds cell budget; reduce memory distance or increase terrain cell size");
    g.cells.assign(std::size_t(g.width)*g.height,-1);
    for(int v=0;v<raw.height;++v)std::copy_n(raw.cells.begin()+std::size_t(v)*raw.width,raw.width,
      g.cells.begin()+std::size_t(v+left)*g.width+back);
    for(const auto& item:samples_) {
      if(item.second.seen==capture)continue; // Fresh hazards already occupy raw cells.
      const auto p=local(item.second.world);
      const int u=int(std::floor((p.x-g.x0)/res)),v=int(std::floor((p.y-g.y0)/res));
      if(u<0 || v<0 || u>=g.width || v>=g.height)continue;
      auto& cell=g.cells[std::size_t(v)*g.width+u];
      if(blockedTerrain(cell))continue;
      if(capture-item.second.seen<=c.hazard_hold_sec){cell=held_hazard;++out.stats.held;}
      else {cell=remembered_hazard;++out.stats.remembered;}
    }
    history_.emplace_back(capture,out);while(history_.size()>16)history_.pop_front();
    return out;
  }
};
}
