#include "roi_vfh_test/planner.hpp"
#include <iostream>
using namespace roi_vfh_test;
void check(bool ok,const char* reason){if(!ok)throw std::runtime_error(reason);}
int main(){
  Grid g;g.width=60;g.height=24;g.resolution=.05;g.y0=-.6;g.cells.assign(1440,0);
  std::vector<Point> lane;for(double x=.05;x<2.9;x+=.05)lane.push_back({x,0});
  Config c;c.speed=.7;c.horizon=2.3;Planner planner;
  auto plan=planner.plan(g,lane,c);
  check(!plan.path.empty() && plan.speed>0 && plan.speed<=.7,"open lane failed");
  check(plan.candidates.size()<=124,"candidate cap exceeded");
  check(std::abs(plan.path.back().y)<.1,"open lane swerved");
  check(!plan.used_vfh && plan.status.find("LANE:")==0,"clear lane did not bypass VFH");
  std::vector<Point> curve;
  for(double x=.05;x<2.9;x+=.05)curve.push_back({x,.18*(1-std::cos(1.5*x))});
  planner.reset();const auto curve_plan=planner.plan(g,curve,c);
  check(!curve_plan.used_vfh && curve_plan.speed>0 && curve_plan.path.back().y>.1,"curved lane was replaced by a straight goal");
  // The supplied reference begins 5cm ahead of the current axle origin.
  for(std::size_t i=1;i<curve_plan.path.size();++i) {
    const auto& p=curve_plan.path[i];check(laneDistance({p.x,p.y},curve)<.045,"full curve tracking error too large");
  }
  auto off_route=g;
  for(int x=5;x<32;++x)off_route.cells[2*60+x]=100;
  planner.reset();const auto side_plan=planner.plan(off_route,curve,c);
  check(!side_plan.used_vfh && side_plan.path.size()==curve_plan.path.size(),"off-route hazard forced avoidance");
  for(std::size_t i=0;i<side_plan.path.size();++i)
    check(std::hypot(side_plan.path[i].x-curve_plan.path[i].x,side_plan.path[i].y-curve_plan.path[i].y)<1e-10,"off-route hazard changed a clear curve");
  auto unknown=g;unknown.cells.assign(1440,-1);planner.reset();
  check(planner.plan(unknown,lane,c).speed>0,"unknown terrain was blocked");
  auto gray=g;gray.cells.assign(1440,25);planner.reset();
  check(planner.plan(gray,lane,c).speed>0,"gray low-occupancy cells did not match display policy");
  auto cone=g;
  for(int x=18;x<=20;++x)for(int y=11;y<=12;++y)cone.cells[y*60+x]=100;
  planner.reset();plan=planner.plan(cone,lane,c);
  std::cout<<"cone path="<<plan.path.size()<<" speed="<<plan.speed<<" candidates="<<plan.candidates.size()<<"\n";
  check(plan.used_vfh && plan.reference_blocked,"cone did not trigger ordinary VFH avoidance");
  check(plan.length>1.2,"single cone route stops before avoidance");
  check(!plan.path.empty(),"single cone did not produce a local route");
  for(auto p:plan.path)check(std::abs(p.steer)<=c.max_steer+1e-9,"steering exceeds limit");
  const auto cleared=planner.plan(g,lane,c);
  check(!cleared.used_vfh && cleared.speed>0,"old hazard/histogram retained after current frame cleared");
  std::vector<Point> return_lane;for(const auto& p:lane)return_lane.push_back({p.x,p.y-.25});
  planner.reset();const auto returning=planner.plan(g,return_lane,c);
  check(!returning.used_vfh && !returning.path.empty() && laneDistance({returning.path.back().x,returning.path.back().y},return_lane)<.06,
    "clear lane did not return from avoidance offset");
  auto narrow=g;narrow.cells.assign(1440,100);planner.reset();
  check(planner.plan(narrow,lane,c).speed==0,"blocked route moved");
  check(planner.plan(g,{},c).speed==0,"missing ordinary lane moved");
  // Binary hysteresis preserves its prior decision within the threshold band.
  const double density=plan.histogram[30];check(density>0,"missing primary histogram");
  auto tuning=c;tuning.hist_low=density*.4;tuning.hist_high=density*.8;
  planner.reset();auto first=planner.plan(cone,lane,tuning);check(first.binary[30],"high threshold failed");
  tuning.hist_low=density*.8;tuning.hist_high=density*1.2;
  check(planner.plan(cone,lane,tuning).binary[30],"hysteresis state was discarded");
  planner.reset();check(!planner.plan(cone,lane,tuning).binary[30],"reset retained histogram state");
  // Sensor map boundaries and departure limits must never activate VFH.
  auto small_grid=g;small_grid.width=10;small_grid.cells.assign(240,-1);
  planner.reset();const auto beyond_grid=planner.plan(small_grid,lane,c);
  check(!beyond_grid.used_vfh && beyond_grid.speed>0 && beyond_grid.length>.8,"map boundary vetoed clear lane tracking");
  // Cone rows use the same flow, with no mission classifier or route boundary.
  auto walls=g;for(int x=16;x<38;++x){walls.cells[9*60+x]=100;walls.cells[21*60+x]=100;}
  planner.reset();const auto detour=planner.plan(walls,lane,c);
  check(detour.used_vfh && detour.reference_blocked && !detour.path.empty(),"paired walls did not use ordinary avoidance");
  for(std::size_t i=1;i<detour.path.size();++i)
    check(footprintBlock(walls,c,detour.path[i])==Block::None,"detour skipped obstacle checking");
  const auto rejoined=planner.plan(g,lane,c);
  check(!rejoined.used_vfh && rejoined.speed>0,"clear return trajectory did not restore lane tracking");
  check(planner.plan(walls,{},c).speed==0,"missing lane bypassed reference requirement");
  // Nearby side hazards + a forward cone can mask every polar sector while a
  // collision-checked forward prefix remains. Search curves before stopping.
  auto masked_scene=cone;masked_scene.cells[7*60+2]=100;masked_scene.cells[16*60+2]=100;
  auto masked_config=c;masked_config.margin=.02;masked_config.hist_low=0;masked_config.hist_high=.001;
  planner.reset();const auto masked_plan=planner.plan(masked_scene,lane,masked_config);
  check(std::all_of(masked_plan.masked.begin(),masked_plan.masked.end(),[](bool b){return b;}),"fixture did not mask every sector");
  check(masked_plan.fallback_candidates>0 && masked_plan.used_fallback && masked_plan.speed>0,"fully masked histogram skipped checked curves");
  for(std::size_t i=1;i<masked_plan.path.size();++i)
    check(footprintBlock(masked_scene,masked_config,masked_plan.path[i])==Block::None,"fallback bypassed collision checks");
  const std::vector<Point> remembered_heading{{0,0},{1,.6}};
  planner.reset();const auto no_lane=planner.plan(unknown,remembered_heading,c,true);
  check(no_lane.used_vfh && no_lane.speed>0,"lane loss did not advance through unknown terrain");
  for(std::size_t i=1;i<no_lane.path.size();++i)
    check(footprintBlock(unknown,c,no_lane.path[i])==Block::None,"lane preference bypassed collision checking");
  const auto blocked_return=planner.plan(cone,lane,c);
  check(blocked_return.used_vfh,"visible but blocked return canceled the detour");
  const auto clear_return=planner.plan(g,return_lane,c);
  check(!clear_return.used_vfh && clear_return.speed>0,"clear lane did not resume tracking");
  planner.reset();check(planner.plan(narrow,remembered_heading,c,true).speed==0,"lane-loss mode drove into an occupied grid");
  bool threw=false;try {auto bad=c;bad.hist_low=bad.hist_high;bad.validate();}catch(...){threw=true;}
  check(threw,"invalid hysteresis accepted");
  std::cout<<"Planner tests passed\n";
}
