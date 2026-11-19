#pragma once
#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
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
  struct Measurement {
    std::optional<double> distance;
    std::string reason;
    int roi_pixels{0},valid_pixels{0},inlier_pixels{0};
    double valid_ratio{0},relative_spread{0};
  };
  Measurement assess(const std::vector<double>& valid, int total) const {
    Measurement out;out.roi_pixels=total;out.valid_pixels=static_cast<int>(valid.size());
    if(total<=0) {out.reason="empty_roi";return out;}
    out.valid_ratio=double(valid.size())/total;
    if(int(valid.size())<min_pixels) {out.reason="too_few_valid_pixels";return out;}
    if(out.valid_ratio<min_ratio) {out.reason="low_valid_pixel_ratio";return out;}
    const auto m=median(valid);
    std::vector<double> residual, clean;
    for(auto z:valid) residual.push_back(std::abs(z-m));
    const auto mad=median(residual);
    // Reject mixed/unstable depth before clipping, so a broad background is not hidden.
    auto sorted=valid; std::sort(sorted.begin(),sorted.end());
    out.relative_spread=(sorted[(sorted.size()-1)*9/10]-sorted[(sorted.size()-1)/10])/m;
    if(out.relative_spread>max_relative_spread) {out.reason="unstable_depth_distribution";return out;}
    for(auto z:valid) if(std::abs(z-m)<=std::max(.02,3*1.4826*mad)) clean.push_back(z);
    out.inlier_pixels=static_cast<int>(clean.size());
    if(int(clean.size())<min_pixels) {out.reason="too_few_inliers";return out;}
    if(double(clean.size())/total<min_ratio) {out.reason="low_inlier_ratio";return out;}
    out.distance=median(clean);return out;
  }
  std::optional<double> extract(const std::vector<double>& valid, int total) const {
    return assess(valid,total).distance;
  }
};
}
