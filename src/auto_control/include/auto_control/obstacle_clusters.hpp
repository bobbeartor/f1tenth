#pragma once
#include "auto_control/obstacle_scan.hpp"
#include <utility>
namespace auto_control::obstacle
{
struct ClusterOptions
{
  bool enabled{true};
  double tolerance_m{0.15}, min_height_m{0.10};
  int min_bins{2}, max_gap_bins{1};
};
inline void validateClusters(const ClusterOptions & c)
{
  if (!std::isfinite(c.tolerance_m) || c.tolerance_m<=0 || c.tolerance_m>1 ||
    !std::isfinite(c.min_height_m) || c.min_height_m<0 || c.min_height_m>10 ||
    c.min_bins<1 || c.min_bins>2048 || c.max_gap_bins<0 || c.max_gap_bins>10) {
    throw std::invalid_argument("cluster: tolerance=(0,1]m, min_height=0..10m, min_bins=1..2048, max_gap_bins=0..10");
  }
}
struct Cluster
{
  std::vector<std::size_t> bins;
  double x{0}, y{0}, nearest_range{0}, max_height{0};
};
struct Clusters
{
  std::vector<Cluster> objects;
  std::size_t height_rejected_bins{0}, small_rejected_bins{0};
};
// Segment the angularly ordered observed surface. Only height-supported bins
// connect; low floor returns cannot bridge two objects. Labels are frame-local.
inline Clusters clusterScan(const Scan & scan,const Options & o,const ClusterOptions & c)
{
  validateClusters(c);
  Clusters out;
  if (!c.enabled) {return out;}
  Cluster current;
  std::size_t previous=0;
  double previous_x=0,previous_y=0;
  const auto finish=[&]() {
    if (current.bins.size()>=static_cast<std::size_t>(c.min_bins)) {
      current.x/=current.bins.size(); current.y/=current.bins.size();
      out.objects.push_back(std::move(current));
    } else {out.small_rejected_bins+=current.bins.size();}
    current=Cluster{};
  };
  for (std::size_t i=0;i<scan.ranges.size();++i) {
    const double r=scan.ranges[i];
    if (!std::isfinite(r) || r<o.min_range || r>o.max_range) {continue;}
    if (i>=scan.max_heights.size() || !std::isfinite(scan.max_heights[i]) || scan.max_heights[i]<c.min_height_m) {
      ++out.height_rejected_bins; continue;
    }
    const double a=(o.angle_min+i*(o.angle_max-o.angle_min)/(o.bins-1))*radians;
    const double x=r*std::cos(a),y=r*std::sin(a);
    if (!current.bins.empty() && (i-previous>static_cast<std::size_t>(c.max_gap_bins+1) ||
      std::hypot(x-previous_x,y-previous_y)>c.tolerance_m)) {finish();}
    if (current.bins.empty()) {current.nearest_range=r;}
    current.bins.push_back(i); current.x+=x; current.y+=y;
    current.nearest_range=std::min(current.nearest_range,r);
    current.max_height=std::max(current.max_height,static_cast<double>(scan.max_heights[i]));
    previous=i; previous_x=x; previous_y=y;
  }
  finish();
  return out;
}
}
