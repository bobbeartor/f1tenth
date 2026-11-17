#pragma once
#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <vector>

namespace multiclass_yolo {
inline std::vector<double> distanceFeatures(double x, double y, double w, double h,
  double width, double height, int cls) {
  if (width<=0 || height<=0 || cls<0 || cls>=4 || !std::isfinite(x+y+w+h) ||
      x<0 || y<0 || w<=0 || h<=0 || x+w>width+.01 || y+h>height+.01)
    throw std::invalid_argument("Invalid distance features");
  std::vector<double> f{(x+w/2)/width,(y+h/2)/height,w/width,h/height,w*h/(width*height)};
  for(int i=0;i<4;++i) f.push_back(i==cls ? 1:0);
  return f;
}
inline double median(std::vector<double> v) {
  std::sort(v.begin(),v.end());
  return (v[(v.size()-1)/2]+v[v.size()/2])/2;
}
struct DepthQuality {
  double core{.6}, min_m{.2}, max_m{10}, min_ratio{.25}, max_relative_spread{.15};
  int min_pixels{20};
  std::optional<double> extract(const std::vector<double>& valid, int total) const {
    if(total<=0 || int(valid.size())<min_pixels || double(valid.size())/total<min_ratio) return {};
    const auto m=median(valid);
    std::vector<double> residual, clean;
    for(auto z:valid) residual.push_back(std::abs(z-m));
    const auto mad=median(residual);
    // Reject mixed/unstable depth before clipping, so a broad background is not hidden.
    auto sorted=valid; std::sort(sorted.begin(),sorted.end());
    if((sorted[(sorted.size()-1)*9/10]-sorted[(sorted.size()-1)/10])/m>max_relative_spread) return {};
    for(auto z:valid) if(std::abs(z-m)<=std::max(.02,3*1.4826*mad)) clean.push_back(z);
    if(int(clean.size())<min_pixels || double(clean.size())/total<min_ratio) return {};
    return median(clean);
  }
};
}
