#include "line_detactor/speed_limit_panel.hpp"
#include <cassert>
#include <cmath>
#include <limits>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

using line_detactor::SpeedLimitPanel;
using line_detactor::SpeedLimitSnapshot;
int main(int argc, char ** argv)
{
  SpeedLimitPanel panel;
  cv::Mat bev(776,360,CV_8UC3,cv::Scalar(255,255,255));
  panel.update({}, false, 0.);
  auto offline = panel.append(bev, true);
  assert(!panel.available() && offline.cols == 540 && offline.rows > bev.rows);
  panel.mouse(cv::EVENT_LBUTTONUP,10,offline.rows-5,0);
  assert(panel.take_action()=="metrics");
  panel.mouse(cv::EVENT_LBUTTONUP,200,offline.rows-5,0);
  assert(panel.take_action()=="parameters");
  panel.mouse(cv::EVENT_LBUTTONUP,500,offline.rows-5,0);
  assert(panel.take_action()=="stop");
  panel.mouse(cv::EVENT_LBUTTONDOWN, 414, 400, 0);
  assert(!panel.take_request(0.));
  panel.update({.8,5.,5.,0}, true, .1);
  auto initial = panel.append(bev, true);
  assert(panel.available() && panel.selected() == 5.);
  assert(cv::norm(initial(cv::Rect(0,0,360,776)), bev, cv::NORM_INF) == 0.);
  // The BEV and numeric readout must not operate the slider.
  panel.mouse(cv::EVENT_LBUTTONDOWN, 100, 400, 0);
  panel.mouse(cv::EVENT_LBUTTONDOWN, 420, 90, 0);
  assert(!panel.take_request(.2));
  // Click in track, then drag beyond its ends; actual value stays unconfirmed.
  panel.mouse(cv::EVENT_LBUTTONDOWN, 414, 400, 0);
  auto request = panel.take_request(.2);
  assert(request && *request >= .8 && *request <= 5. && *request != 5.);
  panel.update({.8,5.,5.,0}, true, .21);
  assert(panel.selected() == *request);
  panel.mouse(cv::EVENT_MOUSEMOVE, 414, 190, cv::EVENT_FLAG_LBUTTON);
  assert(!panel.take_request(.25)); // throttle intermediate drag requests
  panel.mouse(cv::EVENT_LBUTTONUP, 414, 190, 0);
  request = panel.take_request(.26); // release always sends final value
  assert(request && *request == 5.);
  panel.update({.8,5.,5.,1}, true, .27);
  panel.mouse(cv::EVENT_LBUTTONDOWN, 414, 400, 0);
  panel.mouse(cv::EVENT_LBUTTONUP, 414, 900, 0);
  assert(panel.take_request(.3).value() == .8);
  panel.update({.8,5.,.8,2}, true, .4);
  assert(panel.selected() == .8);
  assert(panel.display_key().find("/1/0/") != std::string::npos); // confirmed, not pending
  // Fresh independent state changes synchronize the idle UI.
  panel.update({.8,5.,2.5,3}, true, .5);
  assert(panel.selected() == 2.5);
  panel.update({.8,5.,2.5,3}, false, 2.);
  panel.mouse(cv::EVENT_LBUTTONDOWN, 414, 400, 0);
  assert(!panel.available() && !panel.take_request(2.));
  panel.update({.8,4.,4.,0}, true, 2.1); // controller restart restores its ceiling
  assert(panel.selected() == 4.);
  assert(!(SpeedLimitSnapshot{.8,5.,std::numeric_limits<double>::quiet_NaN(),0}.valid()));
  // Pending requests that are not acknowledged revert to the applied value.
  panel.append(bev, true);
  panel.mouse(cv::EVENT_LBUTTONDOWN, 414, 400, 0);
  panel.mouse(cv::EVENT_LBUTTONUP, 414, 400, 0);
  assert(panel.take_request(2.2));
  panel.update({.8,4.,4.,0}, true, 4.);
  assert(panel.selected() == 4.);
  // Degenerate YAML range is visible but not interactive.
  panel.update({1.8,1.8,1.8,0}, true, 4.1);
  panel.append(bev, true);
  panel.mouse(cv::EVENT_LBUTTONDOWN, 414, 400, 0);
  assert(!panel.take_request(4.2));
  if (argc > 1) {
    cv::Mat themed(776,360,CV_8UC3,cv::Scalar(255,255,255));
    cv::line(themed,{110,599},{150,0},cv::Scalar(212,204,199),3);
    cv::line(themed,{250,599},{290,0},cv::Scalar(212,204,199),3);
    cv::line(themed,{180,599},{220,0},cv::Scalar(230,153,38),10);
    cv::putText(themed,"Preview 30.0 FPS",{16,655},cv::FONT_HERSHEY_SIMPLEX,.5,{139,116,100},1);
    SpeedLimitPanel themed_panel, basic_panel;
    themed_panel.update({.8,5.,3.,1},true,1.);
    basic_panel.update({.8,5.,3.,1},true,1.);
    auto themed_image = themed_panel.append(themed,true);
    cv::Mat basic(430,180,CV_8UC3,cv::Scalar(0,0,0));
    cv::line(basic,{50,299},{70,0},{255,255,0},2);
    cv::line(basic,{130,299},{150,0},{255,255,0},2);
    cv::line(basic,{90,299},{110,0},{0,255,0},2);
    cv::putText(basic,"Speed: 1.42 m/s",{3,330},cv::FONT_HERSHEY_SIMPLEX,.3,{255,255,255},1);
    auto basic_image=basic_panel.append(basic,false);
    cv::resize(basic_image,basic_image,{},2.,2.,cv::INTER_NEAREST);
    cv::copyMakeBorder(themed_image,themed_image,0,84,0,0,cv::BORDER_CONSTANT,{230,230,230});
    cv::Mat both;
    cv::hconcat(themed_image,basic_image,both);
    cv::imwrite(argv[1],both);
  }
}
