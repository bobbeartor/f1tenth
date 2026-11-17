#pragma once
#include <cmath>
namespace roi_lidar
{
inline bool freshCapture(double capture, double now, double max_age)
{
  const double age=now-capture;
  return std::isfinite(capture) && std::isfinite(now) && age>=-0.01 && age<=max_age;
}
inline bool overlayAllowed(double depth_capture,double rgb_capture,double now,double max_age,double max_skew)
{
  return freshCapture(depth_capture,now,max_age) && freshCapture(rgb_capture,now,max_age) &&
    std::abs(depth_capture-rgb_capture)<=max_skew;
}
}
