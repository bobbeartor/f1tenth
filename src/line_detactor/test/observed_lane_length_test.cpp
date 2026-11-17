#include "line_detactor/observed_lane_length.hpp"
#include "line_detactor/lane_connector.hpp"
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <iostream>
using namespace line_detactor;
void check(bool v) {if (!v) throw std::runtime_error("Observed length mismatch");}
int main() {
  auto measure=[](const ObservedLanePaths & p) {return observed_lane_length(p,120,300,30,1.2,3.,.65);};
  ObservedLanePaths p;
  check(measure(p)==0);
  p[0].push_back({{57,299},{57,199}});
  check(std::abs(measure(p)-1.)<1e-6);
  p[1].push_back({{122,299},{122,99}});
  check(std::abs(measure(p)-2.)<1e-6); // max of either side, not sum
  p[0].push_back({{57,180},{57,0}});
  check(std::abs(measure(p)-2.)<1e-6); // disconnected far island excluded
  p[1].clear();
  check(std::abs(measure(p)-1.)<1e-6);
  p[0][0][0].x=0; check(std::isnan(measure(p))); // padding is not observed
  check(std::isnan(observed_lane_length(p,0,300,30,1.2,3.,.65)));
  cv::Mat mask=cv::Mat::zeros(300,120,CV_8UC1);
  cv::line(mask,{25,299},{25,265},cv::Scalar(1),3);
  cv::line(mask,{25,220},{25,0},cv::Scalar(1),3);
  LaneConnectionConfig config;
  auto result=connect_lane_fragments(mask,config,true);
  double length=measure(result.observed_paths);
  check(length>.25 && length<.45); // actual connector output, no far island/bridge
  std::cout << "Observed component length tests passed\n";
}
