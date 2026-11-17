#pragma once
#include <cmath>
#include <stdexcept>
namespace roi_lidar
{
// Display footprint in the front-axle frame; independent of LiDAR range limits.
struct ViewOptions
{
  double width_m{1.5};
  double forward_m{4.0};
};
inline void validateView(const ViewOptions & v)
{
  if (!std::isfinite(v.width_m) || !std::isfinite(v.forward_m) ||
    v.width_m<0.1 || v.width_m>65.535 || v.forward_m<0.1 || v.forward_m>65.535) {
    throw std::invalid_argument("preview.width_m/forward_m must be finite values in 0.1..65.535m");
  }
}
inline bool insideView(double x,double y,const ViewOptions & v)
{
  return x>=0 && x<=v.forward_m && y>=-v.width_m/2 && y<=v.width_m/2;
}
}
