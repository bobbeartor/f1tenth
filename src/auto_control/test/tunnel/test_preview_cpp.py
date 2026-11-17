"""Compile the real HUD renderer and preview handoff without ROS/GPU/GUI."""
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile
import unittest

SOURCE = Path(__file__).resolve().parents[3]


def function(source, signature):
    start = source.index(signature)
    opening = source.index('{', start)
    # Default array parameter has {} before the function body.
    if source[opening:opening+2] == '{}':
        opening = source.index('{', opening+2)
    depth, end = 1, opening+1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


class PreviewNativeTests(unittest.TestCase):
    def test_real_renderer_and_owning_latest_frame_handoff(self):
        compiler, pkg = shutil.which('c++'), shutil.which('pkg-config')
        if not compiler or not pkg:
            self.skipTest('C++ compiler/OpenCV development files unavailable')
        config = subprocess.run([pkg, '--cflags', '--libs', 'opencv4'],
                                text=True, capture_output=True)
        if config.returncode:
            self.skipTest('OpenCV development files unavailable')
        source = (SOURCE / 'line_detactor/src/line_detactor_node.cpp').read_text()
        renderer = function(source, '  cv::Mat preview_canvas(')
        warmup = function(source, '  void warm_up()')
        code = r'''
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include "bev_handoff/direct_bev_handoff.hpp"
#include "line_detactor/ir_preview.hpp"
#include "line_detactor/camera_exposure_hud.hpp"
namespace bev_handoff {bool avoidanceControlRequested() {return false;}}
constexpr int kBannerHeight=144;
struct Harness {
  bool obstacle_overlay_enabled_{true},tunnel_enabled_{true};
  struct {bool enabled{true};} connection_;
RENDERER
};
void require(bool condition) {
  if (!condition) {throw std::runtime_error("Preview handoff/HUD regression");}
}
struct TrialBackend {
  int calls{0};
  bool fail{false};
  void infer_bgr(const std::uint8_t * data, std::size_t bytes, std::size_t stride) {
    require(data && bytes==120*300*3 && stride==120*3);
    require(std::all_of(data,data+bytes,[](std::uint8_t x) {return x==0;}));
    if (fail) {throw std::runtime_error("trial inference failed");}
    ++calls;
  }
};
#define RCLCPP_INFO(...) ((void)0)
struct WarmupHarness {
  int model_input_width_{120},model_input_height_{300},warmup_iterations_{0};
  std::unique_ptr<TrialBackend> backend_{std::make_unique<TrialBackend>()};
WARMUP
};
int main() {
  WarmupHarness check;
  check.warm_up();
  require(check.backend_->calls==1); // warmup=0 still verifies the model once.
  check.warmup_iterations_=3;
  check.warm_up();
  require(check.backend_->calls==4);
  check.backend_->fail=true;
  bool failed=false;
  try {check.warm_up();} catch (const std::runtime_error &) {failed=true;}
  require(failed); // Do not turn a failed engine execution into a PASS report.
  cv::Mat live(300,180,CV_8UC3,cv::Scalar(10,20,30));
  const auto fresh=line_detactor::ir_preview_image(live,.05,300,180);
  const auto delayed=line_detactor::ir_preview_image(live,.26,300,180);
  const auto very_old=line_detactor::ir_preview_image(live,5.,300,180);
  const cv::Rect below_label(0,20,180,280);
  require(cv::norm(fresh(below_label),delayed(below_label),cv::NORM_L1)==0);
  require(cv::norm(fresh(below_label),very_old(below_label),cv::NORM_L1)==0);
  require(cv::norm(fresh,delayed,cv::NORM_L1)>0);
  cv::Mat display_comparison;
  cv::hconcat(fresh,delayed,display_comparison);
  cv::imwrite("/tmp/f1tenth_ir_preview_retention.png",display_comparison);
  require(!line_detactor::ir_preview_fresh(true,.26));
  require(line_detactor::ir_preview_fresh(true,.05));
  const auto waiting=line_detactor::ir_preview_image({},.05,300,180);
  require(cv::countNonZero(waiting(below_label).reshape(1))==0);
  require(!bev_handoff::latestIrLanePreview());
  {
    auto frame=std::make_shared<bev_handoff::IrLanePreview>();
    frame->image=cv::Mat::zeros(300,180,CV_8UC3);
    frame->image.at<cv::Vec3b>(0,0)={1,2,3};
    frame->generation=7;
    bev_handoff::publishIrLanePreview(frame);
  }
  auto retained=bev_handoff::latestIrLanePreview();
  require(retained && retained->generation==7);
  require(retained->image.at<cv::Vec3b>(0,0)==cv::Vec3b(1,2,3));
  bev_handoff::publishIrLanePreview(nullptr);
  require(!bev_handoff::latestIrLanePreview());
  require(retained->image.at<cv::Vec3b>(0,0)==cv::Vec3b(1,2,3));
  cv::Mat image=cv::Mat::zeros(300,180,CV_8UC3);
  cv::line(image,{50,299},{35,0},{255,0,0},2);
  cv::line(image,{130,299},{145,0},{0,0,255},2);
  cv::line(image,{90,299},{90,0},{0,255,255},2);
  Harness preview;
  const std::array<std::string,3> hud{
    "Tunnel: INSIDE","TCN avg 1234.5FPS","TCN LIVE E:0.75 X:0.05"};
  using Exposure=bev_handoff::CameraExposureObservation;
  using Mode=bev_handoff::RgbExposureControl;
  const auto now=std::chrono::steady_clock::now();
  require(!bev_handoff::latestCameraExposure(false));
  bev_handoff::publishCameraExposure(false,std::make_shared<Exposure>(Exposure{2000,200,now}));
  bev_handoff::publishCameraExposure(true,std::make_shared<Exposure>(Exposure{30000,800,now}));
  bev_handoff::publishRgbExposureControl(Mode::automatic);
  auto exposure_hud=line_detactor::camera_exposure_hud(bev_handoff::latestCameraExposure(false),
    bev_handoff::latestCameraExposure(true),bev_handoff::latestRgbExposureControl(),now);
  require(exposure_hud[0]=="RGB 2.00ms ISO200 AE");
  require(exposure_hud[1]=="IR 30.00ms ISO800");
  const auto stale_hud=line_detactor::camera_exposure_hud(bev_handoff::latestCameraExposure(false),
    bev_handoff::latestCameraExposure(true),Mode::restoring,now+std::chrono::seconds(1));
  require(stale_hud[0].find("STALE RESTORE")!=std::string::npos);
  require(stale_hud[1].find("STALE")!=std::string::npos);
  bev_handoff::publishRgbExposureControl(Mode::auto_pending);
  require(bev_handoff::latestRgbExposureControl()==Mode::auto_pending);
  const auto missing_hud=line_detactor::camera_exposure_hud({}, {},Mode::auto_pending,now);
  require(missing_hud[0]=="RGB --ms ISO-- AE PENDING");
  require(missing_hud[1]=="IR --ms ISO--");
  cv::Mat themed=cv::Mat::zeros(300,180,CV_8UC3);
  line_detactor::append_camera_exposure_hud(themed,exposure_hud);
  require(themed.rows==328 && themed.cols==180);
  require(cv::countNonZero(themed(cv::Rect(0,300,180,28)).reshape(1))>0);
  require(cv::countNonZero(themed(cv::Rect(0,0,180,300)).reshape(1))==0);
  const auto canvas=preview.preview_canvas(image,2.,3.,8.,.8,30.,1,1.,1.8,0,.075,"","",hud,exposure_hud);
  require(canvas.cols==180 && canvas.rows==300+144+56+14);
  require(cv::countNonZero(canvas(cv::Rect(0,300+14*5,180,14)).reshape(1))>0);
  const auto missing_canvas=preview.preview_canvas(image,2.,3.,8.,.8,30.,1,1.,1.8,0,.075,"","",hud,missing_hud);
  require(cv::norm(canvas(cv::Rect(0,300+14*8,180,28)),missing_canvas(cv::Rect(0,300+14*8,180,28)),cv::NORM_L1)>0);
  require(cv::norm(canvas(cv::Rect(0,300,180,14*8)),missing_canvas(cv::Rect(0,300,180,14*8)),cv::NORM_L1)==0);
  cv::Mat enlarged;
  cv::resize(canvas,enlarged,{},3.,3.,cv::INTER_NEAREST);
  cv::imwrite("/tmp/f1tenth_tunnel_preview_hud.png",enlarged);
  preview.tunnel_enabled_=false;
  const auto legacy=preview.preview_canvas(image,2.,3.,8.,.8,30.,1,1.,1.8,0,.075);
  require(legacy.rows==300+144+14);
  const auto unknown=preview.preview_canvas(image,2.,3.,8.,.8,30.,1,1.,1.8,0,
    std::numeric_limits<double>::quiet_NaN());
  const cv::Rect duty_row(0,300+14*8,180,14);
  require(cv::norm(legacy(duty_row),unknown(duty_row),cv::NORM_L1)>0);
  require(cv::norm(legacy(cv::Rect(0,300,180,14*8)),
    unknown(cv::Rect(0,300,180,14*8)),cv::NORM_L1)==0);
}
'''.replace('RENDERER', renderer).replace('WARMUP', warmup)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            header = root / 'std_msgs/msg/header.hpp'
            header.parent.mkdir(parents=True)
            header.write_text('#pragma once\nnamespace std_msgs {namespace msg {struct Header {};}}\n')
            file, binary = root / 'preview.cpp', root / 'preview_test'
            file.write_text(code)
            compiled = subprocess.run([compiler, '-std=c++17', '-O2', '-DNDEBUG', '-Wall', '-Wextra',
                            '-Werror', '-I'+str(root), '-I'+str(SOURCE/'bev_handoff/include'),
                            '-I'+str(SOURCE/'line_detactor/include'),
                            str(file), str(SOURCE/'bev_handoff/src/direct_bev_handoff.cpp'),
                            str(SOURCE/'bev_handoff/src/direct_camera_handoff.cpp'),
                            *shlex.split(config.stdout), '-o', str(binary)],
                           capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    unittest.main()
