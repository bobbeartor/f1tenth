#pragma once
#include "roi_vfh_test/motion.hpp"
#include <optional>

namespace roi_vfh_test {
struct HazardStats {
  std::size_t raw{},confirmed{},pending{},immediate{};
  std::size_t held{},remembered{};
  bool enabled{false},motion_ready{false};
};
struct HazardResult {Grid grid;HazardStats stats;};
// Stores only the previous observation's current hazard evidence, never free
// terrain or absent obstacles. Every output hazard must also exist NOW.
class HazardFilter {
  struct Evidence {Point point;std::uint8_t hits;};
  std::vector<Evidence> previous_;
  std::optional<MotionPose> previous_pose_;
  std::optional<HazardResult> cached_;
  std::deque<std::pair<double,HazardResult>> history_;
  double capture_{-1e9};
  bool sameGeometry(const Grid& grid)const {
    return cached_ && grid.width==cached_->grid.width && grid.height==cached_->grid.height &&
      grid.resolution==cached_->grid.resolution && grid.x0==cached_->grid.x0 && grid.y0==cached_->grid.y0;
  }
public:
  void reset(){previous_.clear();previous_pose_.reset();cached_.reset();history_.clear();capture_=-1e9;}
  HazardResult apply(const Grid& raw,double capture,const std::optional<MotionPose>& pose,const Config& c) {
    raw.validate();c.validate();
    if(!std::isfinite(capture))throw std::invalid_argument("nonfinite hazard capture time");
    const bool enabled=c.hazard_filter_enabled!=0;
    if(enabled && pose && previous_pose_ && sameGeometry(raw))for(const auto& entry:history_)
      if(entry.first==capture)return entry.second; // Replanning cannot count a depth twice.
    if(!enabled || !pose || !sameGeometry(raw) || capture<=capture_ || capture-capture_>c.hazard_max_gap_sec)
      reset();
    HazardResult out{raw,{}};out.stats.enabled=enabled;out.stats.motion_ready=bool(pose);
    const int needed=static_cast<int>(c.hazard_confirm_frames),small=static_cast<int>(c.hazard_small_max_cells);
    const int radius=static_cast<int>(c.hazard_match_radius_cells);
    const auto size=raw.cells.size();
    std::vector<std::uint8_t> prior(size,0),visited(size,0);
    if(enabled && pose && previous_pose_)for(const auto& evidence:previous_) {
      const auto p=reproject(evidence.point,*previous_pose_,*pose);
      const int u=static_cast<int>(std::floor((p.x-raw.x0)/raw.resolution));
      const int v=static_cast<int>(std::floor((p.y-raw.y0)/raw.resolution));
      // Include evidence just outside the boundary if its match neighborhood
      // intersects the current grid. Match work is bounded to at most 5x5.
      for(int dy=-radius;dy<=radius;++dy)for(int dx=-radius;dx<=radius;++dx) {
        const int x=u+dx,y=v+dy;if(x<0 || y<0 || x>=raw.width || y>=raw.height)continue;
        auto& value=prior[static_cast<std::size_t>(y)*raw.width+x];value=std::max(value,evidence.hits);
      }
    }
    std::vector<Evidence> current;current.reserve(previous_.size());
    std::vector<std::size_t> component;component.reserve(64);
    for(std::size_t seed=0;seed<size;++seed) {
      if(visited[seed] || !blockedTerrain(raw.cells[seed]))continue;
      component.clear();component.push_back(seed);visited[seed]=1;
      for(std::size_t head=0;head<component.size();++head) {
        const auto i=component[head];const int u=i%raw.width,v=i/raw.width;
        for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
          const int x=u+dx,y=v+dy;if(x<0 || y<0 || x>=raw.width || y>=raw.height)continue;
          const auto j=static_cast<std::size_t>(y)*raw.width+x;
          if(!visited[j] && blockedTerrain(raw.cells[j])){visited[j]=1;component.push_back(j);}
        }
      }
      const bool immediate=!enabled || component.size()>static_cast<std::size_t>(small);
      for(const auto i:component) {
        ++out.stats.raw;
        const int hits=immediate?needed:std::min(needed,static_cast<int>(prior[i])+1);
        if(immediate || (pose && hits>=needed)) {
          ++out.stats.confirmed;if(immediate)++out.stats.immediate;
        }else {out.grid.cells[i]=-1;++out.stats.pending;}
        if(enabled && pose)current.push_back({
          {raw.x0+(i%raw.width+.5)*raw.resolution,raw.y0+(i/raw.width+.5)*raw.resolution},
          static_cast<std::uint8_t>(hits)});
      }
    }
    if(enabled && pose){previous_=std::move(current);previous_pose_=pose;capture_=capture;cached_=out;
      history_.emplace_back(capture,out);while(history_.size()>16)history_.pop_front();}
    return out;
  }
};
}
