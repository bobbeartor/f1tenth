#pragma once

namespace roi_vfh_test {

// Sim-only planner. Build lateral profiles in (s,d), then check the actual
// Ackermann rollout of each profile against the current terrain.
inline Plan planFrenet(const Grid& grid, const std::vector<Point>& lane,
    const Config& c, bool lane_missing) {
  grid.validate();
  c.validate();
  Plan result;
  const double steer_limit=std::min(c.max_steer,std::atan(c.wheelbase*c.max_curvature));
  const double lookahead=std::clamp(c.reference_distance*.35,.25,.55);
  const auto finish=[&](const Candidate& candidate) {
    result.path=candidate.path;
    result.length=std::max(0.,candidate.length-c.step);
    result.steer=candidate.command_steer;
    double curvature=0;
    for(const auto& p:result.path)curvature=std::max(curvature,std::abs(std::tan(p.steer)/c.wheelbase));
    const double lateral=result.reference_blocked || lane_missing ?
      c.avoidance_lateral_acceleration : c.lane_lateral_acceleration;
    result.speed=std::min({c.speed,std::sqrt(lateral/std::max(.01,curvature)),
      std::sqrt(std::pow(c.deceleration*c.delay,2)+2*c.deceleration*result.length)-c.deceleration*c.delay});
  };

  // Use the usual lane controller when the complete preview is clear.
  Candidate track=lane_missing?Candidate{}:followReference(lane,c);
  for(std::size_t i=1;i<track.path.size();++i) {
    if(footprintBlock(grid,c,track.path[i])!=Block::Hazard)continue;
    result.reference_blocked=true;
    break;
  }
  if(!lane_missing && !result.reference_blocked && track.length-c.step>=c.minimum_path_length &&
      laneDistance({track.path.back().x,track.path.back().y},lane)<=laneDistance({0,0},lane)+.06) {
    finish(track);
    result.status="FRENET: clear lane tracking";
    return result;
  }

  Candidate best;
  double best_score=-1e9;
  const Polyline curve(lane);
  const double start=curve.nearest({0,0});
  const auto origin=curve.at(start);
  const double heading=curve.heading(origin);
  const double offset=origin.x*std::sin(heading)-origin.y*std::cos(heading);
  const int half_samples=static_cast<int>(c.avoidance_curve_samples)/2;
  const double span=std::max(.8,grid.height*grid.resolution/2);
  for(int index=-half_samples;index<=half_samples;++index) {
    const double target=span*index/half_samples;
    for(double transition:{.55,1.0,1.5}) {
      std::vector<Point> guide;
      const double guide_step=.06;
      for(double ds=0;ds<=c.horizon+lookahead+guide_step;ds+=guide_step) {
        const double s=std::min(curve.length(),start+ds);
        auto point=curve.at(s);
        const auto a=curve.at(std::max(0.,s-.04)), b=curve.at(std::min(curve.length(),s+.04));
        const double angle=std::atan2(b.y-a.y,b.x-a.x);
        const double t=std::min(1.,ds/transition);
        const double blend=t*t*t*(10+t*(-15+6*t));
        const double d=offset+(target-offset)*blend;
        const double extra=start+ds-s;
        point.x+=extra*std::cos(angle)-d*std::sin(angle);
        point.y+=extra*std::sin(angle)+d*std::cos(angle);
        guide.push_back(point);
      }
      const Polyline path(guide);
      Pose p;p.steer=c.current_steer;
      Candidate candidate;candidate.path.push_back(p);
      double progress=0;
      for(double distance=c.step;distance<=c.horizon+1e-8;distance+=c.step) {
        progress=path.nearest({p.x,p.y},progress);
        const auto goal=path.at(progress+lookahead);
        const double dx=goal.x-p.x,dy=goal.y-p.y;
        const double desired=std::clamp(std::atan2(2*c.wheelbase*(std::cos(p.yaw)*dy-std::sin(p.yaw)*dx),
          std::max(.01,dx*dx+dy*dy)),-steer_limit,steer_limit);
        if(candidate.path.size()==1)candidate.command_steer=desired;
        p.steer=steeringStep(p.steer,desired,c.step/c.speed,c);
        const double turn=c.step*std::tan(p.steer)/c.wheelbase, middle=p.yaw+turn/2;
        p.x+=c.step*std::cos(middle)-c.wheelbase*turn*std::sin(middle);
        p.y+=c.step*std::sin(middle)+c.wheelbase*turn*std::cos(middle);
        p.yaw=wrap(p.yaw+turn);
        if(std::abs(p.yaw)>=pi/2 || p.x<0)break;
        if(footprintBlock(grid,c,p)==Block::Hazard){++result.hazard_ends;break;}
        candidate.path.push_back(p);
        candidate.length=distance;
      }
      if(candidate.length-c.step<c.minimum_path_length)continue;
      const auto& end=candidate.path.back();
      const double score=3*candidate.length+.3*end.x-
        .28*c.avoidance_lane_weight*laneDistance({end.x,end.y},lane)-
        .1*std::abs(end.yaw)-.12*c.avoidance_steering_weight*std::abs(candidate.command_steer-c.current_steer);
      if(score>best_score){best_score=score;best=std::move(candidate);}
    }
  }
  if(best.path.empty()) {
    result.status="STOP: no collision-free Frenet trajectory";
    return result;
  }
  finish(best);
  result.status="FRENET: collision-checked detour";
  return result;
}
}
