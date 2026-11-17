#pragma once
#include "bev_handoff/avoidance_handoff.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>

namespace line_detactor
{
inline std::string drawAvoidancePreview(cv::Mat & image,const std_msgs::msg::Header & header,
  int width,int height,int padding,double max_delta,double max_age)
{
  if (!bev_handoff::avoidancePreviewEnabled()) {return {};}
  const auto plan=bev_handoff::latestAvoidancePreview();
  const std::string mode=bev_handoff::avoidanceControlRequested()?"APPLY ":"TEST ";
  const auto unavailable=[&](const std::string & why) {
    return mode+(bev_handoff::avoidanceDeformationOnly()?"CENTERLINE: deformation unavailable - ":"WAIT: ")+why;
  };
  if (!plan) {return unavailable("lane/depth");}
  const auto now=std::chrono::steady_clock::now();
  const double age=std::chrono::duration<double>(now-plan->lane_received_at).count();
  const double depth_age=std::chrono::duration<double>(now-plan->depth_captured_at).count();
  const double delta=std::abs(double(header.stamp.sec)-plan->header.stamp.sec+
    (double(header.stamp.nanosec)-plan->header.stamp.nanosec)*1e-9);
  const double display_delta=std::min(max_age,std::max(max_delta,plan->display_delta_sec));
  if (plan->header.frame_id!=header.frame_id || age<0 || age>max_age || depth_age<0 || depth_age>max_age || delta>display_delta) {
    return unavailable("stale/unsynced");
  }
  if (plan->width!=width || plan->height!=height || image.cols!=width+2*padding || image.rows!=height) {
    return unavailable("geometry");
  }
  auto plot=image(cv::Rect(padding,0,width,height));
  const auto pixel=[&](double x,double y) {
    return cv::Point(static_cast<int>(std::lround((plan->y_max-y)/plan->meter_per_pixel-.5)),
      static_cast<int>(std::lround((plan->x_max-x)/plan->meter_per_pixel-.5)));
  };
  for (const auto & box:plan->boxes) {
    cv::rectangle(plot,pixel(box.x0,box.y0),pixel(box.x1,box.y1),cv::Scalar(45,45,85),1);
  }
  for (const auto & box:plan->assumed_boxes) {
    cv::rectangle(plot,pixel(box.x0,box.y0),pixel(box.x1,box.y1),cv::Scalar(0,165,255),1);
    cv::drawMarker(plot,pixel((box.x0+box.x1)/2,(box.y0+box.y1)/2),
      cv::Scalar(0,165,255),cv::MARKER_CROSS,5,1,cv::LINE_AA);
  }
  const auto path=[&](const std::vector<cv::Point2d> & points,const cv::Scalar & color,int thickness) {
    for (std::size_t i=1;i<points.size();++i) {
      cv::line(plot,pixel(points[i-1].x,points[i-1].y),pixel(points[i].x,points[i].y),color,thickness,cv::LINE_AA);
    }
  };
  // No fan of global left/right candidates. Only show the attempted local
  // profile when blocked, or the selected complete multi-obstacle route.
  if (plan->selected.empty() && !plan->candidates.empty()) {
    path(plan->candidates.back().path,cv::Scalar(65,65,160),1);
  }
  path(plan->original,cv::Scalar(0,255,255),1);
  path(plan->selected,cv::Scalar(255,255,0),2);
  // Show the reason instead of a generic WAIT with zero speed/curvature.
  if (plan->selected.empty()) {return bev_handoff::avoidanceDeformationOnly()?unavailable(plan->status):mode+plan->status;}
  // C=inferred corridor from centerline width (only for avoidance candidates).
  // Full details remain on /auto/avoidance_preview/status.
  return mode+plan->status.substr(0,plan->status.find(':'))+
    (plan->inferred_boundaries?" C":"")+
    cv::format(" n%zu L%.1f vref%.2f",plan->obstacle_count,plan->horizon_m,plan->recommended_speed);
}
}
