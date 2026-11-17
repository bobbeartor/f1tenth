#pragma once
#include "bev_handoff/direct_camera_handoff.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <opencv2/imgproc.hpp>

namespace line_detactor
{
inline std::array<std::string,2> camera_exposure_hud(
  const std::shared_ptr<const bev_handoff::CameraExposureObservation> & rgb,
  const std::shared_ptr<const bev_handoff::CameraExposureObservation> & ir,
  bev_handoff::RgbExposureControl mode, std::chrono::steady_clock::time_point now)
{
  const auto line=[now](const char * name, const auto & observation, const std::string & status) {
    std::ostringstream text;
    text << name << " ";
    if (!observation || observation->exposure_us<=0 || observation->iso<=0) {
      text << "--ms ISO--";
    } else {
      text << std::fixed << std::setprecision(2) << observation->exposure_us/1000. << "ms ISO" << observation->iso;
      if (now<observation->captured_at || now-observation->captured_at>std::chrono::milliseconds(500)) {
        text << " STALE";
      }
    }
    text << status;
    return text.str();
  };
  const std::string status=mode==bev_handoff::RgbExposureControl::restoring ? " RESTORE" :
    mode==bev_handoff::RgbExposureControl::auto_pending ? " AE PENDING" : " AE";
  return {line("RGB",rgb,status),line("IR",ir,"")};
}

inline void append_camera_exposure_hud(cv::Mat & canvas, const std::array<std::string,2> & lines)
{
  // Daylight theme: dark text on white, matching the driving HUD banner.
  cv::Mat footer(28,canvas.cols,CV_8UC3,cv::Scalar(255,255,255));
  for (int i=0;i<2;++i) {
    int baseline=0;
    const auto extent=cv::getTextSize(lines[i],cv::FONT_HERSHEY_SIMPLEX,.28,1,&baseline);
    const double font=std::min(.28,.28*(footer.cols-6)/std::max(1,extent.width));
    cv::putText(footer,lines[i],cv::Point(3,12+14*i),cv::FONT_HERSHEY_SIMPLEX,font,
      cv::Scalar(26,24,20),1,cv::LINE_AA);
  }
  cv::Mat combined; cv::vconcat(canvas,footer,combined); canvas=std::move(combined);
}
}
