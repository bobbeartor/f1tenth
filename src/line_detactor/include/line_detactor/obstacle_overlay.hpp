#pragma once
#include "bev_handoff/direct_obstacle_handoff.hpp"
#include "bev_handoff/avoidance_handoff.hpp"
#include "line_detactor/avoidance_overlay.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <cmath>

namespace line_detactor
{
// Paint only on a disposable result image, never on model input/labels/paths.
inline std::string drawObstacleOverlay(cv::Mat & image,const std_msgs::msg::Header & header,
  int source_width,int source_height,int padding,double max_delta,double max_age)
{
  const auto frame=bev_handoff::matchingObstacles(header,max_delta,max_age);
  if (!frame) {return "depth WAIT / stale or unsynced";}
  if (frame->width!=source_width || frame->height!=source_height ||
    image.cols!=source_width+2*padding || image.rows!=source_height ||
    !std::isfinite(frame->meter_per_pixel) || frame->meter_per_pixel<=0) {
    return "depth geometry mismatch";
  }
  auto plot=image(cv::Rect(padding,0,source_width,source_height));
  const auto pixel=[&](const cv::Point2f & p) {
    return cv::Point(static_cast<int>(std::lround((frame->y_max-p.y)/frame->meter_per_pixel-.5)),
      static_cast<int>(std::lround((frame->x_max-p.x)/frame->meter_per_pixel-.5)));
  };
  const std::array<cv::Scalar,4> colors{{cv::Scalar(255,180,70),cv::Scalar(230,80,230),
    cv::Scalar(70,170,255),cv::Scalar(255,230,90)}};
  std::size_t visible=0;
  for (std::size_t i=0;i<frame->clusters.size();++i) {
    const auto & cluster=frame->clusters[i];
    const auto color=colors[i%colors.size()];
    std::vector<cv::Point> points;
    bool inside=false;
    for (const auto & p:cluster.surface_xy) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y)) {continue;}
      const auto q=pixel(p); points.push_back(q);
      inside|=q.x>=0 && q.y>=0 && q.x<plot.cols && q.y<plot.rows;
    }
    if (!inside) {continue;}
    ++visible;
    // Observations locate the object; their spread is not its physical outline.
    // The avoidance preview separately draws the assumed fixed-size footprint.
    for (const auto & q:points) {cv::circle(plot,q,1,color,-1,cv::LINE_AA);}
    if (std::isfinite(cluster.center.x) && std::isfinite(cluster.center.y)) {
      const auto center=pixel(cluster.center);
      if (center.x>=0 && center.y>=0 && center.x<plot.cols && center.y<plot.rows) {
        cv::drawMarker(plot,center,color,cv::MARKER_CROSS,5,1,cv::LINE_AA);
        const cv::Point at(std::clamp(center.x+3,0,std::max(0,plot.cols-50)),
          std::clamp(center.y-4,9,std::max(9,plot.rows-2)));
        const auto text=cv::format("#%zu %.2fm",i+1,cluster.nearest_range);
        cv::putText(plot,text,at,cv::FONT_HERSHEY_SIMPLEX,.23,cv::Scalar(0,0,0),2,cv::LINE_AA);
        cv::putText(plot,text,at,cv::FONT_HERSHEY_SIMPLEX,.23,color,1,cv::LINE_AA);
      }
    }
  }
  const double delta=(double(header.stamp.sec)-frame->header.stamp.sec)*1000+
    (double(header.stamp.nanosec)-frame->header.stamp.nanosec)*1e-6;
  return cv::format("depth %.0fFPS n=%zu dt=%.0fms",frame->fps,visible,std::abs(delta));
}
}
