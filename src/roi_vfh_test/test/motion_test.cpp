#include "roi_vfh_test/motion.hpp"
#include <iostream>
using namespace roi_vfh_test;
void check(bool b,const char* m){if(!b)throw std::runtime_error(m);}
int main(){
  const auto aligned=reproject({1,0},{1,1,2,pi/2,0},{1,0,0,0,0});
  check(std::abs(aligned.x-1)<1e-6 && std::abs(aligned.y-3)<1e-6,"capture-frame reprojection failed");
  MotionHistory motion;motion.add(0,.7,0);motion.add(.05,.7,0);MotionPose pose;
  check(motion.at(.025,pose)&&std::abs(pose.x-.0175)<1e-6,"speed integration/interpolation failed");
  auto generation=motion.generation;motion.add(1,.7,0);check(motion.generation>generation,"motion gap not reset");
  check(!motion.at(.5,pose),"motion extrapolated across outage");
  MotionHistory turning;turning.add(0,0,1);turning.add(.05,0,1);
  check(turning.at(.05,pose) && pose.y>.016,"front axle lateral motion omitted");
  std::cout<<"Motion tests passed: capture alignment, interpolation, outages, front axle motion\n";
}
