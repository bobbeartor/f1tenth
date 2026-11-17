#include "camera_driver/ir_imu_synchronizer.hpp"
#include "camera_driver/imu_image_stabilizer.hpp"
#include <iostream>
#include <memory>
#include <stdexcept>

using namespace std::chrono_literals;
using Clock=std::chrono::steady_clock;
using camera_driver::ImageStabilizationCorrection;
using camera_driver::ImuImageStabilizer;
using Queue=camera_driver::IrImuSynchronizer<int,ImageStabilizationCorrection>;
void require(bool value,const char * message) {if (!value) {throw std::runtime_error(message);}}

int main()
{
  camera_driver::ImuImageStabilizerConfig config;
  config.startup_discard_duration_sec=0.;
  config.reference_calibration_duration_sec=.01;
  config.maximum_frame_imu_age_sec=.012;
  config.maximum_frame_imu_wait_sec=0.0001;
  ImuImageStabilizer imu(config);
  const cv::Vec3d gravity(0.,-9.80665,0.);
  for (int i=0;i<=4;++i) {imu.update(gravity,{0.,0.,0.},i*.0025);}
  require(imu.initialized(),"calibration failed");
  imu.update(gravity,{0.,0.,.1},.05);
  require(!imu.correctionAt(.03),"RGB should retain 12ms sample age bound");
  const auto interpolated=imu.correctionAt(.03,.05,0.);
  require(interpolated && !interpolated->predicted,"IR should interpolate bracketing sparse samples");
  require(interpolated->within_correction_limit,"small tilt should remain valid");
  require(!imu.correctionAt(.03),"IR lookup must not mutate RGB bounds");
  require(!imu.correctionAt(.09,.05,0.),"IR must not expand gyro extrapolation limit");

  const auto start=Clock::now();
  Queue pending(.1);
  const auto capture=Clock::time_point{}+90ms;
  require(pending.push(1,capture,start),"capture enqueue failed");
  auto lookup=[&imu](auto stamp) {
    return imu.correctionAt(std::chrono::duration<double>(stamp.time_since_epoch()).count(),.05,0.);
  };
  require(!pending.takeReady(start,lookup),"future IMU has not arrived");
  require(pending.size()==1,"one failed lookup must not discard the capture");
  imu.update(gravity,{0.,0.,.1},.095);
  const auto ready=pending.takeReady(start+30ms,lookup);
  require(ready && ready->entry.frame==1,"capture should match after delayed IMU arrival");
  require(ready->entry.captured==capture,"capture timestamp must be preserved");
  require(!ready->correction.predicted,"late bracketing sample should be interpolated");
  require(pending.size()==0 && !pending.takeReady(start+31ms,lookup),"must not replay a match");
  require(!pending.push(2,capture,start+32ms),"duplicate capture must be rejected");

  Queue burst(.1);
  burst.push(1,Clock::time_point{}+100ms,start);
  burst.push(2,Clock::time_point{}+110ms,start+10ms);
  burst.push(3,Clock::time_point{}+120ms,start+20ms);
  auto always=[](auto) {return std::optional<ImageStabilizationCorrection>(ImageStabilizationCorrection{});};
  const auto newest=burst.takeReady(start+30ms,always);
  require(newest && newest->entry.frame==3 && burst.superseded()==2,"latest match must win without FIFO burst");
  require(!burst.takeReady(start+31ms,always),"discarded frames must not return");
  Queue partially_ready(.1);
  partially_ready.push(1,capture,start);
  partially_ready.push(2,capture+10ms,start+10ms);
  const auto earlier=partially_ready.takeReady(start+20ms,[capture](auto stamp) {
    return stamp==capture ? std::optional<ImageStabilizationCorrection>(ImageStabilizationCorrection{}) : std::nullopt;
  });
  require(earlier && earlier->entry.frame==1 && partially_ready.size()==1,
    "a newer unmatched capture must survive an older match");
  const auto later=partially_ready.takeReady(start+30ms,always);
  require(later && later->entry.frame==2,"newer capture must match when its IMU arrives");
  burst.push(4,Clock::time_point{}+130ms,start+40ms);
  auto unavailable=[](auto) {return std::optional<ImageStabilizationCorrection>{};};
  require(!burst.takeReady(start+141ms,unavailable),"unmatched capture must expire");
  require(burst.expired()==1 && burst.size()==0,"pending wait must stay bounded");

  camera_driver::IrImuSynchronizer<std::shared_ptr<int>,ImageStabilizationCorrection> owning(.1,2);
  auto owner=std::make_shared<int>(7); std::weak_ptr<int> weak=owner;
  owning.push(owner,capture,start); owner.reset();
  require(!weak.expired(),"queued DepthAI frame owner must be retained");
  owning.clear(); require(weak.expired(),"profile change must release old captures");
  owning.push(std::make_shared<int>(1),capture+1ms,start);
  owning.push(std::make_shared<int>(2),capture+2ms,start);
  owning.push(std::make_shared<int>(3),capture+3ms,start);
  require(owning.size()==2 && owning.overflow()==1,"capture queue capacity must stay bounded");

  bool invalid=false;
  try {imu.correctionAt(.03,0.,0.);} catch (const std::invalid_argument &) {invalid=true;}
  require(invalid,"invalid lookup bounds must be rejected");
  std::cout << "ir_imu_synchronizer_test passed\n";
}
