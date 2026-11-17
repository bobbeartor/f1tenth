#include "roi_vfh_test/render.hpp"
#include <iostream>
using namespace roi_vfh_test;
void check(bool b,const char* m){if(!b)throw std::runtime_error(m);}
int main(){
  Grid g;g.width=60;g.height=24;g.resolution=.05;g.y0=-.6;g.cells.assign(1440,0);
  for(int x=16;x<38;++x){g.cells[9*60+x]=100;g.cells[21*60+x]=100;}
  std::vector<Point> lane;for(double x=.05;x<2.9;x+=.05)lane.push_back({x,0});
  Config config;Planner planner;Plan plan;for(int i=0;i<3;++i)plan=planner.plan(g,lane,config);
  const auto active=renderPreview(g,lane,plan,config,true,true,plan.status,1.0);
  const auto paused=renderPreview(g,lane,plan,config,false,false,"STOPPED: parameters unlocked",0);
  check(active.cols==760 && active.rows==620,"Preview dimensions changed");
  check(active.at<cv::Vec3b>(15,15)==cv::Vec3b(160,160,160),"Running Start button not disabled");
  check(paused.at<cv::Vec3b>(15,15)==cv::Vec3b(70,150,70),"Stopped Start button not enabled");
  check(active.at<cv::Vec3b>(15,190)==cv::Vec3b(70,70,200),"Stop button missing");
  check(startButton().contains(cv::Point(20,20)) && !startButton().contains(cv::Point(200,20)) && stopButton().contains(cv::Point(200,20)),"Button hit regions incorrect");
  // A stale plan must never paint selected/candidate trajectories while stopped.
  const auto empty=renderPreview(g,lane,Plan{},config,false,false,"STOPPED: parameters unlocked",0);
  check(cv::norm(paused(cv::Rect(280,190,450,410)),empty(cv::Rect(280,190,450,410)),cv::NORM_INF)==0,"Paused preview retained selected path");
  std::cout<<"Render tests passed: canvas, Start/Stop controls, paused trajectory suppression\n";
}
