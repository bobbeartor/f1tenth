#include "depth_lidar/depth_lidar_grid.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace depth_lidar {
bool validateGridConfig(const GridConfig & g, const ClusterConfig & c, std::string & reason)
{
  if (c.connectivity != 4 && c.connectivity != 8) {
    reason = "cluster.connectivity must be 4 or 8";
    return false;
  }
  if (!std::isfinite(g.resolution_m) || g.resolution_m < 0.01 || g.resolution_m > 0.25
      || !std::isfinite(g.x_min_m) || !std::isfinite(g.x_max_m)
      || !std::isfinite(g.y_min_m) || !std::isfinite(g.y_max_m)
      || g.x_min_m < 0 || g.x_max_m <= g.x_min_m || g.y_max_m <= g.y_min_m
      || g.min_returns_per_cell < 1 || g.min_returns_per_cell > 2048
      || c.min_cells < 1 || c.min_cells > 100000 || c.min_returns < 1 || c.min_returns > 2048) {
    reason = "invalid grid bounds, resolution [0.01,0.25], cell/cluster support";
    return false;
  }
  const double w = (g.x_max_m-g.x_min_m)/g.resolution_m;
  const double h = (g.y_max_m-g.y_min_m)/g.resolution_m;
  if (!std::isfinite(w) || !std::isfinite(h) || w < 1 || h < 1 || w > 1000 || h > 1000
      || std::round(w)*std::round(h) > 100000
      || std::abs(w-std::round(w)) > 1e-6 || std::abs(h-std::round(h)) > 1e-6) {
    reason = "grid extents must be multiples of resolution, <=1000/axis, <=100000 cells";
    return false;
  }
  reason.clear();
  return true;
}
GridResult clusterScan(const ScanResult & scan, const ProjectionConfig & p,
  const GridConfig & g, const ClusterConfig & c)
{
  std::string reason;
  if (!validateGridConfig(g,c,reason) || !validateProjectionConfig(p,reason)
      || scan.ranges.size() != static_cast<std::size_t>(p.bins) || scan.ages.size() != scan.ranges.size()) {
    throw std::invalid_argument("invalid scan grid input: " + reason);
  }
  GridResult result;
  result.config = g;
  result.width = static_cast<int>(std::lround((g.x_max_m-g.x_min_m)/g.resolution_m));
  result.height = static_cast<int>(std::lround((g.y_max_m-g.y_min_m)/g.resolution_m));
  result.cells.resize(static_cast<std::size_t>(result.width)*result.height);
  constexpr double rad = 3.14159265358979323846 / 180.0;
  for (std::size_t i=0; i<scan.ranges.size(); ++i) {
    const double range = scan.ranges[i], age = scan.ages[i];
    if (!std::isfinite(range) || range < p.min_range_m || range > p.max_range_m
        || !std::isfinite(age) || age < 0.0) { continue; }
    const double angle = (p.angle_min_deg+i*(p.angle_max_deg-p.angle_min_deg)/(p.bins-1))*rad;
    const double x=range*std::cos(angle), y=range*std::sin(angle);
    if (x < g.x_min_m || x >= g.x_max_m || y < g.y_min_m || y >= g.y_max_m) { continue; }
    const int col=static_cast<int>(std::floor((x-g.x_min_m)/g.resolution_m));
    const int row=static_cast<int>(std::floor((y-g.y_min_m)/g.resolution_m));
    if (col < 0 || col >= result.width || row < 0 || row >= result.height) { continue; }
    auto & cell=result.cells[static_cast<std::size_t>(row)*result.width+col];
    cell.age_sec = cell.returns ? std::min(cell.age_sec,age) : age;
    ++cell.returns; cell.sum_x+=x; cell.sum_y+=y;
    cell.nearest_range_m=std::min(cell.nearest_range_m,range);
  }
  for (auto & cell : result.cells) {
    if (cell.returns < static_cast<std::uint32_t>(g.min_returns_per_cell)) { cell={}; }
  }
  std::vector<std::uint8_t> visited(result.cells.size(),0);
  std::vector<std::size_t> component;
  for (std::size_t seed=0; seed<result.cells.size(); ++seed) {
    if (visited[seed] || !result.cells[seed].returns) { continue; }
    component.clear(); component.push_back(seed); visited[seed]=1;
    std::size_t returns=0;
    for (std::size_t head=0; head<component.size(); ++head) {
      const auto id=component[head]; returns+=result.cells[id].returns;
      const int col=static_cast<int>(id%result.width), row=static_cast<int>(id/result.width);
      for (int dy=-1; dy<=1; ++dy) {
        for (int dx=-1; dx<=1; ++dx) {
          if (dx == 0 && dy == 0) { continue; }
          if (c.connectivity == 4 && dx != 0 && dy != 0) { continue; }
          const int x=col+dx,y=row+dy;
          if (x<0 || x>=result.width || y<0 || y>=result.height) { continue; }
          const auto next=static_cast<std::size_t>(y)*result.width+x;
          if (!visited[next] && result.cells[next].returns) {
            visited[next]=1; component.push_back(next);
          }
        }
      }
    }
    if (component.size()<static_cast<std::size_t>(c.min_cells) || returns<static_cast<std::size_t>(c.min_returns)) {
      for (auto id:component) { result.cells[id]={}; }
      continue;
    }
    GridCluster cluster;
    cluster.cells=component; cluster.returns=returns;
    cluster.min_x=g.x_max_m; cluster.max_x=g.x_min_m;
    cluster.min_y=g.y_max_m; cluster.max_y=g.y_min_m;
    for (auto id:component) {
      const auto & cell=result.cells[id];
      const double x=g.x_min_m+(id%result.width)*g.resolution_m;
      const double y=g.y_min_m+(id/result.width)*g.resolution_m;
      cluster.center_x+=cell.sum_x; cluster.center_y+=cell.sum_y;
      cluster.min_x=std::min(cluster.min_x,x); cluster.max_x=std::max(cluster.max_x,x+g.resolution_m);
      cluster.min_y=std::min(cluster.min_y,y); cluster.max_y=std::max(cluster.max_y,y+g.resolution_m);
      cluster.nearest_range_m=std::min(cluster.nearest_range_m,cell.nearest_range_m);
      cluster.age_sec=std::max(cluster.age_sec,cell.age_sec);
    }
    cluster.center_x/=returns; cluster.center_y/=returns;
    result.occupied_cells+=component.size(); result.clusters.push_back(std::move(cluster));
  }
  // Only exposed cell edges, including holes. Never bridge missing data or inflate a hull.
  for (std::size_t group=0; group<result.clusters.size(); ++group) {
    for (auto id:result.clusters[group].cells) {
      const int col=static_cast<int>(id%result.width),row=static_cast<int>(id/result.width);
      const double x=g.x_min_m+col*g.resolution_m,y=g.y_min_m+row*g.resolution_m;
      const double xx=x+g.resolution_m,yy=y+g.resolution_m;
      const auto edge=[&](double x1,double y1,double x2,double y2) {
        result.boundary.push_back({x1,y1,x2,y2,result.cells[id].age_sec,group});
      };
      if (col==0 || !result.cells[id-1].returns) { edge(x,y,x,yy); }
      if (col==result.width-1 || !result.cells[id+1].returns) { edge(xx,yy,xx,y); }
      if (row==0 || !result.cells[id-result.width].returns) { edge(xx,y,x,y); }
      if (row==result.height-1 || !result.cells[id+result.width].returns) { edge(x,yy,xx,yy); }
    }
  }
  return result;
}
} // namespace depth_lidar
