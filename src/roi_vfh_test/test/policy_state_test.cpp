#include "roi_vfh_test/policy_state.hpp"
#include <iostream>
using namespace roi_vfh_test;
void check(bool value,const char* reason){if(!value)throw std::runtime_error(reason);}
int main() {
  Config c;LaneMemory memory;
  const std::vector<Point> lane{{.1,.02},{.14,.025}};
  auto reference=memory.resolve(lane,MotionPose{},c,0.);
  check(reference.trusted && reference.has_memory,"first short lane observation was rejected");
  const double heading=std::atan2(.005,.04);
  const MotionPose moved{1,2,.5,.2,0};
  reference=memory.resolve({},moved,c);
  check(!reference.trusted && reference.has_memory && reference.points.size()==2,"missing lane did not use direction-only memory");
  check(reference.points.front().x==0 && reference.points.front().y==0,"memory retained the old lane's lateral position");
  const auto end=reference.points.back();
  check(std::abs(wrap(std::atan2(end.y,end.x)-(heading-moved.yaw)))<1e-9,"saved direction was not yaw compensated");
  const auto translated=memory.resolve({},MotionPose{2,20,50,.2,0},c);
  check(std::hypot(translated.points.back().x-end.x,translated.points.back().y-end.y)<1e-9,"translation restored old geometry");
  const std::vector<Point> changed{{.1,-.2},{.6,-.4}};
  reference=memory.resolve(changed,moved,c);
  check(reference.trusted && reference.points.front().y==-.2,"large observation change was rejected by a confidence gate");
  c.lane_memory_enabled=0;reference=memory.resolve({},moved,c);
  check(!reference.has_memory && memory.remembered(moved).empty(),"disabled direction memory retained data");
  reference=memory.resolve(lane,moved,c);
  check(reference.trusted && !reference.has_memory,"memory OFF blocked a current lane");
  c.lane_memory_enabled=1;reference=memory.resolve({},moved,c);
  check(!reference.has_memory,"reenabling memory resurrected an old direction");
  std::cout<<"Current lane preference and direction-only memory checks passed\n";
}
