#pragma once
#include <cmath>
#include <string>
#include <opencv2/imgproc.hpp>

namespace line_detactor
{
inline bool ir_preview_fresh(bool available, double capture_age)
{
  return available && std::isfinite(capture_age) && capture_age >= -.05 && capture_age <= .25;
}

// Display retention only: this image is never sent to the lane selector/control.
inline cv::Mat ir_preview_image(const cv::Mat & image, double capture_age, int height, int width)
{
  const bool available=!image.empty();
  const bool fresh=ir_preview_fresh(available,capture_age);
  cv::Mat overlay=available ? image.clone() : cv::Mat::zeros(height,width,CV_8UC3);
  const std::string label=!available ? "IR: waiting for first result" :
    fresh ? "IR" : std::isfinite(capture_age) && capture_age>=0 ?
    cv::format("IR STALE: %.0fms",capture_age*1000.) : "IR STALE: clock invalid";
  cv::putText(overlay,label,cv::Point(4,14),cv::FONT_HERSHEY_SIMPLEX,.3,
    fresh ? cv::Scalar(255,255,255) : cv::Scalar(0,165,255),1,cv::LINE_AA);
  return overlay;
}
}
