#pragma once
#include "roi_lidar/scan.hpp"
#include "roi_lidar/view.hpp"

namespace roi_lidar
{
struct TerrainOptions
{
  double cell_m{0.07}, below_ground_m{0.30};
  int min_ground_samples{2}, min_obstacle_samples{8};
  bool fill_enabled{true};
  int fill_max_cells{2}, fill_clearance_cells{1};
  double vehicle_width_m{2.0}; // Reference only; no footprint inflation.
};
inline void validateTerrain(const TerrainOptions & t, const ViewOptions & view)
{
  if (!std::isfinite(t.cell_m) || t.cell_m<0.02 || t.cell_m>0.25 ||
    !std::isfinite(t.below_ground_m) || t.below_ground_m<0 || t.below_ground_m>0.30 ||
    !std::isfinite(t.vehicle_width_m) || t.vehicle_width_m<=0 || t.vehicle_width_m>3 ||
    t.fill_max_cells<1 || t.fill_max_cells>16 || t.fill_clearance_cells<0 || t.fill_clearance_cells>4 ||
    t.min_ground_samples<1 || t.min_ground_samples>1000 ||
    t.min_obstacle_samples<1 || t.min_obstacle_samples>1000 ||
    std::ceil(view.forward_m/t.cell_m)*std::ceil(view.width_m/t.cell_m)>250000) {
    throw std::invalid_argument("terrain: cell=0.02..0.25m, below_ground=0..0.30m, samples=1..1000, grid<=250000 cells, fill_max_cells=1..16, fill_clearance_cells=0..4");
  }
}
struct Terrain
{
  // ROS grid order: X increases by column, Y increases by row.
  int width{0}, height{0};
  double cell_m{0.07}, y_min{0}, x_max{0}, y_max{0};
  std::vector<std::int8_t> cells; // -1 unknown, 0 observed ground, 100 obstacle/depression
  std::vector<std::int8_t> raw_cells, planning_cells;
  std::vector<std::uint8_t> inferred;
  std::size_t filled_cells{0};
  std::size_t ground_cells{0}, obstacle_cells{0};
};
// Examine raw unknown components only; inferred ground never seeds another fill.
inline void fillSmallHoles(Terrain & out, const std::vector<int> & hazard,
  const TerrainOptions & t)
{
  out.raw_cells=out.cells;
  out.inferred.assign(out.cells.size(),0);
  out.filled_cells=0;
  if (!t.fill_enabled) {return;}
  std::vector<std::uint8_t> visited(out.cells.size(),0);
  std::vector<std::size_t> queue; queue.reserve(out.cells.size());
  for (std::size_t seed=0;seed<out.cells.size();++seed) {
    if (visited[seed] || out.raw_cells[seed]!=-1) {continue;}
    queue.clear(); queue.push_back(seed); visited[seed]=1;
    bool eligible=true;
    for (std::size_t head=0;head<queue.size();++head) {
      const auto index=queue[head];
      const int x=index%out.width,y=index/out.width;
      if (x==0 || y==0 || x==out.width-1 || y==out.height-1 || hazard[index]) {eligible=false;}
      for (int dy=-1;dy<=1;++dy) {for (int dx=-1;dx<=1;++dx) {
        if (!dx && !dy) {continue;}
        const int nx=x+dx,ny=y+dy;
        if (nx<0 || ny<0 || nx>=out.width || ny>=out.height) {continue;}
        const auto n=static_cast<std::size_t>(ny)*out.width+nx;
        if (out.raw_cells[n]==-1) {
          if (!visited[n]) {visited[n]=1; queue.push_back(n);}
        } else if (out.raw_cells[n]!=0) {eligible=false;}
      }}
    }
    if (!eligible || queue.size()>static_cast<std::size_t>(t.fill_max_cells)) {continue;}
    for (const auto index:queue) {
      const int x=index%out.width,y=index/out.width;
      for (int dy=-t.fill_clearance_cells;dy<=t.fill_clearance_cells;++dy) {
        for (int dx=-t.fill_clearance_cells;dx<=t.fill_clearance_cells;++dx) {
          const int nx=x+dx,ny=y+dy;
          if (nx>=0 && ny>=0 && nx<out.width && ny<out.height &&
            hazard[static_cast<std::size_t>(ny)*out.width+nx]) {eligible=false;}
        }
      }
    }
    if (!eligible) {continue;}
    for (const auto index:queue) {out.cells[index]=0; out.inferred[index]=1; ++out.filled_cells;}
  }
}
inline Terrain terrainGrid(const Scan & scan, const Options & o,
  const ViewOptions & view, const TerrainOptions & t)
{
  validateView(view); validateTerrain(t,view);
  Terrain out;
  out.width=static_cast<int>(std::ceil(view.forward_m/t.cell_m));
  out.height=static_cast<int>(std::ceil(view.width_m/t.cell_m));
  out.cell_m=t.cell_m; out.y_min=-view.width_m/2;
  out.x_max=view.forward_m; out.y_max=view.width_m/2;
  const auto size=static_cast<std::size_t>(out.width)*out.height;
  out.cells.assign(size,-1);
  std::vector<int> ground(size,0), obstacle(size,0);
  for (const auto & p:scan.surface_points) {
    if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2]) ||
      p[0]<0 || p[0]>=view.forward_m || p[1]<out.y_min || p[1]>=out.y_max ||
      p[2]>o.max_height) {continue;}
    const int x=static_cast<int>(p[0]/t.cell_m);
    const int y=static_cast<int>((p[1]-out.y_min)/t.cell_m);
    const auto index=static_cast<std::size_t>(y)*out.width+x;
    const double margin=groundMargin(p[0],p[1],o,&scan);
    if (p[2]<-t.below_ground_m || p[2]>margin) {++obstacle[index];}
    else {++ground[index];}
  }
  for (std::size_t i=0;i<size;++i) {
    if (obstacle[i]>=t.min_obstacle_samples) {out.cells[i]=100; ++out.obstacle_cells;}
    // Even one unsupported hazard sample vetoes a green cell.
    else if (obstacle[i]==0 && ground[i]>=t.min_ground_samples) {out.cells[i]=0; ++out.ground_cells;}
  }
  auto fill_veto=obstacle;
  for(const auto & p:scan.rejected_points){
    if(!std::isfinite(p[0]) || !std::isfinite(p[1]) || p[0]<0 || p[0]>=view.forward_m || p[1]<out.y_min || p[1]>=out.y_max)continue;
    const int x=static_cast<int>(p[0]/t.cell_m),y=static_cast<int>((p[1]-out.y_min)/t.cell_m);
    fill_veto[static_cast<std::size_t>(y)*out.width+x]=1;
  }
  fillSmallHoles(out,fill_veto,t);
  out.planning_cells=out.cells;
  // Planning uses the same configured sample threshold and colors as the ROI grid.
  return out;
}
}
