#include "roi_vfh_test/planner.hpp"
#include <chrono>
#include <iostream>
#include <numeric>
using namespace roi_vfh_test;
int main(){
  Grid base;base.width=60;base.height=24;base.resolution=.05;base.y0=-.6;base.cells.assign(1440,0);
  std::vector<Point> lane;for(double x=.05;x<2.9;x+=.05)lane.push_back({x,0});
  Config config;
  for(int scene=0;scene<4;++scene){
    auto grid=base;const char* name="open";
    if(scene==1){name="cone";for(int x=18;x<=20;++x)for(int y=11;y<=12;++y)grid.cells[y*60+x]=100;}
    if(scene==2){name="cone_walls";for(int x=16;x<38;++x){grid.cells[9*60+x]=100;grid.cells[21*60+x]=100;}}
    if(scene==3){name="unknown";grid.cells.assign(1440,-1);}
    Planner planner;std::vector<double> ms;
    for(int i=0;i<320;++i){
      const auto start=std::chrono::steady_clock::now();
      auto plan=planner.plan(grid,lane,config);
      const double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
      if(i>=20)ms.push_back(elapsed);
    }
    std::sort(ms.begin(),ms.end());
    std::cout<<name<<" cells=1440 mean_ms="<<std::accumulate(ms.begin(),ms.end(),0.)/ms.size()<<" p95_ms="<<ms[ms.size()*95/100]<<"\n";
  }
}
