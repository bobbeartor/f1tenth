#include "roi_lidar/terrain.hpp"
#include <iostream>
using namespace roi_lidar;
static void check(bool ok,const char * text) {if (!ok) {throw std::runtime_error(text);}}
int main()
{
  Options o; ViewOptions v; TerrainOptions t;
  v.width_m=1.2; v.forward_m=3; t.min_obstacle_samples=2;
  t.cell_m=.05; t.below_ground_m=.05; // Fixed synthetic scene geometry.
  auto scan=emptyScan(o);
  auto grid=terrainGrid(scan,o,v,t);
  check(grid.width==60 && grid.height==24,"Metric terrain shape incorrect");
  for (const auto cell:grid.cells) {check(cell==-1,"Unobserved cell became ground");}
  const auto index=[&](double x,double y) {
    return static_cast<std::size_t>(std::floor((y+v.width_m/2)/t.cell_m))*grid.width+
      static_cast<std::size_t>(std::floor(x/t.cell_m));
  };
  scan.surface_points={{1.01,.01,0.01},{1.02,.02,-.01},
    {2.01,.01,.2},{2.02,.02,.3},{2.51,.21,-.12},{2.52,.22,-.13}};
  grid=terrainGrid(scan,o,v,t);
  check(grid.cells[index(1.01,.01)]==0,"Measured floor lost");
  check(grid.cells[index(2.01,.01)]==100,"Positive obstacle lost");
  check(grid.cells[index(2.51,.21)]==100,"Below-plane hazard called floor");
  check(grid.cells[index(.51,.01)]==-1,"Ray was filled as free without ground evidence");
  check(grid.cells[index(2.51,.01)]==-1,"Unknown behind obstacle filled");
  scan.surface_points.push_back({1.03,.03,.2});
  grid=terrainGrid(scan,o,v,t);
  check(grid.cells[index(1.01,.01)]==-1,"Single hazard outlier did not veto green floor");
  check(grid.planning_cells[index(1.01,.01)]==-1,"Planning output bypassed the configured hazard sample threshold");
  scan.surface_points.push_back({1.03,.03,.25});
  grid=terrainGrid(scan,o,v,t);
  check(grid.cells[index(1.01,.01)]==100,"Obstacle did not dominate ground");
  scan.surface_points={{.01,.01,2},{-1,0,0},{1,2,0},{4,0,0}};
  grid=terrainGrid(scan,o,v,t);
  check(grid.ground_cells==0 && grid.obstacle_cells==0,"Height/XY limits ignored");
  grid=terrainGrid(emptyScan(o),o,v,t);
  check(grid.ground_cells==0 && grid.obstacle_cells==0,"History retained after empty input");
  Terrain fixture; fixture.width=7; fixture.height=7;
  const auto reset=[&]() {fixture.cells.assign(49,0);};
  std::vector<int> hazard(49,0);
  reset(); fixture.cells[24]=-1;
  fillSmallHoles(fixture,hazard,t);
  check(fixture.cells[24]==0 && fixture.raw_cells[24]==-1 && fixture.inferred[24],"Enclosed hole not tagged/filled");
  reset(); fixture.cells[24]=-1; auto off=t; off.fill_enabled=false;
  fillSmallHoles(fixture,hazard,off); check(fixture.cells[24]==-1,"OFF changed raw cells");
  reset(); fixture.cells[24]=fixture.cells[25]=-1;
  fillSmallHoles(fixture,hazard,t); check(fixture.filled_cells==2,"Two-cell hole lost");
  reset(); fixture.cells[24]=fixture.cells[25]=fixture.cells[26]=-1;
  fillSmallHoles(fixture,hazard,t); check(!fixture.filled_cells,"Oversized hole filled");
  reset(); fixture.cells[0]=fixture.cells[8]=fixture.cells[16]=fixture.cells[24]=-1;
  auto large=t; large.fill_max_cells=16;
  fillSmallHoles(fixture,hazard,large); check(!fixture.filled_cells,"Diagonal exterior connection filled");
  reset(); fixture.cells[24]=-1; hazard[24]=1;
  fillSmallHoles(fixture,hazard,t); check(!fixture.filled_cells,"Unsupported hazard erased");
  hazard[24]=0; hazard[23]=1;
  fillSmallHoles(fixture,hazard,t); check(!fixture.filled_cells,"Nearby hazard ignored");
  hazard[23]=0; fixture.cells[23]=100;
  fillSmallHoles(fixture,hazard,t); check(!fixture.filled_cells,"Obstacle boundary filled");
  bool rejected=false;
  try {auto bad=t; bad.cell_m=0; terrainGrid(scan,o,v,bad);} catch (const std::invalid_argument &) {rejected=true;}
  check(rejected,"Invalid cell size accepted");
  std::cout<<"Terrain tests passed: ground, obstacles, depression, unknown, conflict, bounds, no history\n";
}
