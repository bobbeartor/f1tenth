#pragma once
#include "roi_vfh_test/planner.hpp"
#include "roi_vfh_test/hazard_filter.hpp"
#include "roi_vfh_test/policy_state.hpp"
#include <opencv2/imgproc.hpp>
namespace roi_vfh_test {
inline cv::Rect startButton(){return {12,12,160,36};}
inline cv::Rect stopButton(){return {184,12,160,36};}
inline cv::Mat renderPreview(const Grid& grid,const std::vector<Point>& lane,
  const Plan& plan,const Config& config,bool requested,bool locked,const std::string& status,double plan_ms,
  bool drive_enabled=false,double duty=0,double brake=0,const HazardStats* hazards=nullptr,const LaneReference* lane_reference=nullptr) {
    cv::Mat canvas(620,760,CV_8UC3,cv::Scalar(245,245,245));
    auto text=[&](std::string s,int y){cv::putText(canvas,s,{12,y},cv::FONT_HERSHEY_SIMPLEX,.43,{35,35,35},1,cv::LINE_AA);};
    cv::rectangle(canvas,startButton(),requested?cv::Scalar(160,160,160):cv::Scalar(70,150,70),-1);
    cv::rectangle(canvas,stopButton(),{70,70,200},-1);
    cv::putText(canvas,drive_enabled?"Start driving":"Start preview",{24,36},cv::FONT_HERSHEY_SIMPLEX,.55,{255,255,255},1);
    cv::putText(canvas,"Stop / unlock",{196,36},cv::FONT_HERSHEY_SIMPLEX,.55,{255,255,255},1);
    text(drive_enabled?cv::format("DRIVE ENABLED | duty %.3f | brake %.2f A",duty,brake):
      "PREVIEW ONLY - no steering, motor or brake commands",70);
    text(status,92);
    const bool active=requested && locked;
    text(cv::format("target %.2f / nominal %.2f m/s | steer %.1f deg | %.2f ms",
      active?plan.speed:0,config.speed,active?plan.steer*180/pi:0,plan_ms),114);
    text("Known retained rear hazards checked; unknown / outside-grid assumed free",136);
    text(cv::format("Checked candidates: %zu | checked length: %.2fm",plan.candidates.size(),plan.length),156);
    if(active) {
      const double peak=std::max(1.,*std::max_element(plan.histogram.begin(),plan.histogram.end()));
      for(int b=0;b<61;++b) {
        const int height=int(70*plan.histogram[b]/peak);
        cv::rectangle(canvas,{12+4*b,260-height,3,std::max(1,height)},{70,150,70},-1);
        cv::rectangle(canvas,{12+4*b,270,3,8},plan.binary[b]?cv::Scalar(60,60,210):cv::Scalar(200,200,200),-1);
        cv::rectangle(canvas,{12+4*b,284,3,8},plan.masked[b]?cv::Scalar(210,80,50):cv::Scalar(200,200,200),-1);
      }
      cv::putText(canvas,plan.used_vfh?(plan.used_fallback?"VFH mask + collision-checked curve":"primary / binary / masked"):"Reference clear - VFH bypass",{12,180},cv::FONT_HERSHEY_SIMPLEX,.40,{35,35,35},1);
    }
    cv::putText(canvas,"Green: current terrain",{12,320},cv::FONT_HERSHEY_SIMPLEX,.40,{35,35,35},1);
    cv::putText(canvas,"Orange: held / remembered hazard",{12,340},cv::FONT_HERSHEY_SIMPLEX,.38,{35,35,35},1);
    cv::putText(canvas,"Gray: assumed free / Red: current hazard",{12,360},cv::FONT_HERSHEY_SIMPLEX,.40,{35,35,35},1);
    cv::putText(canvas,"Blue: current lane / saved direction",{12,382},cv::FONT_HERSHEY_SIMPLEX,.40,{220,90,30},1);
    cv::putText(canvas,"Yellow: selected driving path",{12,402},cv::FONT_HERSHEY_SIMPLEX,.40,{25,130,160},1);
    cv::putText(canvas,cv::format("Hazard ends %zu / short %zu",plan.hazard_ends,plan.short_candidates),
      {12,424},cv::FONT_HERSHEY_SIMPLEX,.38,{35,35,35},1);
    cv::putText(canvas,config.obstacle_memory_enabled!=0 && config.obstacle_memory_sec>0?"Obstacle source: current + short near-body memory":"Obstacle source: current + flicker hold only",
      {12,444},cv::FONT_HERSHEY_SIMPLEX,.38,{35,35,35},1);
    cv::putText(canvas,cv::format("Longest candidate: %.2fm",plan.longest_candidate),
      {12,464},cv::FONT_HERSHEY_SIMPLEX,.38,{35,35,35},1);
    if(hazards) {
      cv::putText(canvas,cv::format("Hazard raw %zu / confirmed %zu",hazards->raw,hazards->confirmed),{12,488},cv::FONT_HERSHEY_SIMPLEX,.37,{35,35,35},1);
      cv::putText(canvas,cv::format("Pending %zu / held %zu / memory %zu",hazards->pending,hazards->held,hazards->remembered),{12,508},cv::FONT_HERSHEY_SIMPLEX,.37,{35,35,35},1);
      if(hazards->enabled && !hazards->motion_ready)
        cv::putText(canvas,"Confirmation waiting for speed + IMU",{12,528},cv::FONT_HERSHEY_SIMPLEX,.34,{35,65,190},1);
    }
    if(plan.used_vfh)
      cv::putText(canvas,cv::format("Extra curves %zu | selected %s",plan.fallback_candidates,plan.used_fallback?"yes":"no"),{12,548},cv::FONT_HERSHEY_SIMPLEX,.35,{35,35,35},1);
    if(lane_reference) {
      cv::putText(canvas,"Lane: "+lane_reference->state,{12,570},cv::FONT_HERSHEY_SIMPLEX,.35,{35,35,35},1);
      cv::putText(canvas,config.lane_memory_enabled==0?"Lane direction memory: OFF":
        cv::format("Direction memory: %s, age %.2fs; no departure cap",
        lane_reference->has_memory?"yes":"no",lane_reference->memory_age),{12,590},cv::FONT_HERSHEY_SIMPLEX,.32,{35,35,35},1);
    }
    if (grid.width>0) {
      const double scale=std::min(430./(grid.width*grid.resolution),400./(grid.height*grid.resolution));
      const double middle_y=grid.y0+grid.height*grid.resolution/2;
      const auto pixel=[&](double x,double y){return cv::Point(int(490-(y-middle_y)*scale),int(570-(x-grid.x0)*scale));};
      for (int v=0;v<grid.height;++v)for (int u=0;u<grid.width;++u){
        const int state=grid.cells[std::size_t(v)*grid.width+u];
        const auto a=pixel(grid.x0+u*grid.resolution,grid.y0+v*grid.resolution);
        const auto b=pixel(grid.x0+(u+1)*grid.resolution,grid.y0+(v+1)*grid.resolution);
        cv::rectangle(canvas,cv::Rect(cv::Point(std::min(a.x,b.x),std::min(a.y,b.y)),cv::Point(std::max(a.x,b.x)+1,std::max(a.y,b.y)+1)),
          state==0?cv::Scalar(175,220,175):(retainedTerrain(state)?cv::Scalar(45,155,235):
            (state>=50?cv::Scalar(75,75,220):cv::Scalar(125,125,125))),-1);
      }
      for(std::size_t i=1;i<lane.size();++i)cv::line(canvas,pixel(lane[i-1].x,lane[i-1].y),pixel(lane[i].x,lane[i].y),{220,90,30},2);
      if (active) {
        for (const auto& c:plan.candidates)for(std::size_t i=1;i<c.path.size();++i)
          cv::line(canvas,pixel(c.path[i-1].x,c.path[i-1].y),pixel(c.path[i].x,c.path[i].y),{175,175,175},1);
        for(std::size_t i=1;i<plan.path.size();++i)cv::line(canvas,pixel(plan.path[i-1].x,plan.path[i-1].y),pixel(plan.path[i].x,plan.path[i].y),{0,220,255},3);
      }
      cv::circle(canvas,pixel(0,0),5,{220,60,30},-1);
      cv::rectangle(canvas,pixel(-config.rear,config.width/2),pixel(config.front,-config.width/2),{55,55,55},1);
    }
  return canvas;
}
}
