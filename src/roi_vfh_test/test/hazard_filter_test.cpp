#include "roi_vfh_test/hazard_filter.hpp"
#include <iostream>
using namespace roi_vfh_test;
void check(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
Grid emptyGrid(){Grid g;g.width=30;g.height=40;g.resolution=.1;g.y0=-2;g.cells.assign(1200,-1);return g;}
std::size_t cell(int x,int y){return std::size_t(y)*30+x;}
int main(){
  Config c;c.hazard_confirm_frames=2;c.hazard_small_max_cells=5;c.hazard_match_radius_cells=0;
  HazardFilter filter;auto g=emptyGrid();g.cells[cell(10,20)]=100;
  const auto pose=[](double t,double x=0,double yaw=0){return std::optional<MotionPose>{MotionPose{t,x,0,yaw,0}};};
  auto result=filter.apply(g,1,pose(1),c);
  check(result.stats.raw==1 && result.stats.pending==1 && result.grid.cells[cell(10,20)]==-1,"First weak hazard bypassed confirmation");
  result=filter.apply(g,1,pose(1),c);
  check(result.stats.pending==1,"Repeated depth capture increased the streak");
  result=filter.apply(g,1.05,pose(1.05),c);
  check(result.stats.confirmed==1 && result.grid.cells[cell(10,20)]==100,"Consecutive hazard not confirmed");
  result=filter.apply(g,1,pose(1),c);
  check(result.stats.pending==1,"Cached old capture leaked later confirmation");
  auto blank=emptyGrid();result=filter.apply(blank,1.10,pose(1.10),c);
  check(result.stats.raw==0 && result.grid.cells[cell(10,20)]==-1,"Absent obstacle was retained");
  result=filter.apply(g,1.15,pose(1.15),c);
  check(result.stats.pending==1,"Missing observation did not break the streak");
  result=filter.apply(g,1.40,pose(1.40),c);
  check(result.stats.pending==1,"Excess observation gap did not reset confirmation");
  filter.reset();filter.apply(g,2,pose(2),c);
  auto moved=emptyGrid();moved.cells[cell(9,20)]=100;
  result=filter.apply(moved,2.05,pose(2.05,.1),c);
  check(result.stats.confirmed==1,"Translation compensation failed");
  filter.reset();filter.apply(g,3,pose(3),c);
  auto rotated=emptyGrid();rotated.cells[cell(0,9)]=100;
  result=filter.apply(rotated,3.05,pose(3.05,0,pi/2),c);
  check(result.stats.confirmed==1,"Yaw compensation failed");
  filter.reset();auto large=emptyGrid();for(int x=10;x<16;++x)large.cells[cell(x,20)]=100;
  result=filter.apply(large,4,pose(4),c);
  check(result.stats.immediate==6 && result.stats.confirmed==6,"Large component not immediate");
  result=filter.apply(g,4.05,pose(4.05),c);
  check(result.stats.confirmed==1,"Fragmented previously strong hazard lost confirmation");
  filter.reset();result=filter.apply(g,5,std::nullopt,c);
  check(result.stats.pending==1 && !result.stats.motion_ready,"Unaligned weak hazard confirmed");
  c.hazard_filter_enabled=0;result=filter.apply(g,5.05,std::nullopt,c);
  check(result.grid.cells==g.cells && !result.stats.enabled,"OFF changed the source grid");
  bool rejected=false;c.hazard_confirm_frames=2.5;
  try{c.validate();}catch(const std::invalid_argument&){rejected=true;}
  check(rejected,"Fractional frame count accepted");
  std::cout<<"Hazard filter checks passed\n";
}
