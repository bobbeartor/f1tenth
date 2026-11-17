#pragma once

#include <point_cloud/bev_crop.hpp>
#include "roi_lidar/filters.hpp"
#include <algorithm>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <vector>

namespace roi_lidar
{
constexpr double radians = 3.14159265358979323846 / 180.0;
struct Options
{
  double roi_x{0}, roi_y{0}, roi_width{1}, roi_height{1}; // normalized image rectangle
  int pixel_stride{1};
  bool ground_enabled{true};
  bool use_startup_plane{false};
  double obstacle_min_height{0.06}; // Above fitted ground; independent of plane inlier tolerance.
  RansacOptions ransac;
  OutlierOptions outlier;
  double ground_distance{0.06}, ground_distance_per_meter{0.01}, ground_max_distance{0.12};
  double max_height{1.0}; // measured plane is vehicle Z=0
  double min_depth{0.1}, max_depth{20.0};
  double min_range{0.1}, max_range{12.0};
  double angle_min{-70}, angle_max{70};
  int bins{281}, min_samples{2};
  double support_distance{0.15};
};
inline void validate(const Options & o)
{
  validateFilters(o.ransac,o.outlier);
  for (const auto v : {o.roi_x,o.roi_y,o.roi_width,o.roi_height,o.ground_distance,
    o.ground_distance_per_meter,o.ground_max_distance,o.obstacle_min_height,o.max_height,o.min_depth,o.max_depth,o.min_range,o.max_range,o.angle_min,
    o.angle_max,o.support_distance}) {
    if (!std::isfinite(v)) {throw std::invalid_argument("ROI/scan values must be finite");}
  }
  if (o.obstacle_min_height<0 || o.obstacle_min_height>=o.max_height) {
    throw std::invalid_argument("obstacle.min_height_m must be >= 0 and < height.max_m");
  }
  if (o.roi_x < 0 || o.roi_y < 0 || o.roi_width <= 0 || o.roi_height <= 0 ||
    o.roi_x+o.roi_width > 1.0000001 || o.roi_y+o.roi_height > 1.0000001 ||
    o.pixel_stride < 1 || o.pixel_stride > 16 || o.ground_distance < 0 ||
    o.ground_distance > 0.3 || o.ground_distance_per_meter<0 || o.ground_distance_per_meter>0.1 ||
    o.ground_max_distance<o.ground_distance || o.ground_max_distance>0.5 ||
    o.max_height <= o.ground_max_distance || o.max_height > 10 ||
    o.min_depth <= 0 || o.max_depth <= o.min_depth || o.max_depth > 65.535 ||
    o.min_range <= 0 || o.max_range <= o.min_range || o.max_range > 65.535 ||
    o.angle_min < -90 || o.angle_max > 90 || o.angle_max <= o.angle_min ||
    o.bins < 2 || o.bins > 2048 || o.min_samples < 1 || o.min_samples > 1000 ||
    o.support_distance <= 0 || o.support_distance > 2) {
    throw std::invalid_argument("Invalid normalized ROI, ground/height band, range or angular scan settings");
  }
}
struct Rect {int x, y, width, height;};
inline Rect imageRoi(int width, int height, const Options & o)
{
  validate(o);
  if (width < 1 || height < 1 || width > 4096 || height > 4096) {
    throw std::invalid_argument("Invalid depth dimensions");
  }
  const int x=std::min(width-1, static_cast<int>(std::floor(o.roi_x*width+1e-9)));
  const int y=std::min(height-1, static_cast<int>(std::floor(o.roi_y*height+1e-9)));
  const int right=std::clamp(static_cast<int>(std::ceil((o.roi_x+o.roi_width)*width-1e-9)),x+1,width);
  const int bottom=std::clamp(static_cast<int>(std::ceil((o.roi_y+o.roi_height)*height-1e-9)),y+1,height);
  return {x,y,right-x,bottom-y};
}
struct Scan
{
  // Measured XY endpoints, including ground. Z is signed plane distance when RANSAC succeeds.
  // Otherwise Z retains the startup-ground height. No point is ray-filled.
  std::vector<point_cloud::Vector3> surface_points;
  std::vector<point_cloud::Vector3> rejected_points; // Isolated returns: unknown evidence, never invented floor.
  std::vector<std::size_t> obstacle_indices;
  std::vector<int> obstacle_selection_ids; // Populated only when a point selector is supplied.
  GroundPlane ground_plane;
  bool processed{false};
  std::size_t outlier_candidates{0}, outlier_removed{0};
  std::vector<float> ranges;
  std::vector<float> max_heights; // Height evidence only from samples near the selected range.
  std::size_t depth_points{0}, ground_removed{0}, height_removed{0}, accepted_points{0}, valid_bins{0};
};
inline double groundLimit(double range,const Options & o,const Scan * scan=nullptr)
{
  if(((scan && scan->ground_plane.detected) || (!o.ransac.enabled && o.use_startup_plane)) &&
    range<=o.ransac.max_range_m)return o.obstacle_min_height;
  return std::min(o.ground_max_distance,o.ground_distance+o.ground_distance_per_meter*range);
}
inline double groundMargin(double x,double y,const Options & o,const Scan * scan=nullptr)
{
  if (((scan && scan->ground_plane.detected) || (!o.ransac.enabled && o.use_startup_plane)) &&
    x*x+y*y<=o.ransac.max_range_m*o.ransac.max_range_m) {return o.obstacle_min_height;}
  return groundLimit(std::hypot(x,y),o);
}
inline bool projectedObstacle(const point_cloud::Vector3 & p,const Options & o,const Scan * scan=nullptr)
{
  return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]) &&
    p[2]<=o.max_height && (!o.ground_enabled || p[2]>groundMargin(p[0],p[1],o,scan));
}
template<class Visitor>
inline void forEachObstacle(const Scan& scan,const Options& o,Visitor visit)
{
  if(scan.processed){for(auto index:scan.obstacle_indices)visit(scan.surface_points[index]);}
  else {for(const auto& p:scan.surface_points)if(projectedObstacle(p,o,&scan))visit(p);}
}
inline Scan emptyScan(const Options & o)
{
  Scan s; s.ranges.assign(o.bins, std::numeric_limits<float>::quiet_NaN());
  s.max_heights=s.ranges; return s;
}
class Projector
{
  struct Ray {int u,v; point_cloud::Vector3 direction;};
  std::vector<Ray> rays_;
  point_cloud::RigidTransform transform_;
  Options options_;
  int width_{0}, height_{0};
public:
  void configure(int width, int height, const point_cloud::Intrinsics & k,
    const point_cloud::RigidTransform & transform, const Options & o)
  {
    const auto roi=imageRoi(width,height,o);
    for (const auto v : {k.fx,k.fy,k.cx,k.cy}) {
      if (!std::isfinite(v)) {throw std::invalid_argument("Invalid depth intrinsics");}
    }
    if (k.fx <= 0 || k.fy <= 0) {throw std::invalid_argument("Invalid focal lengths");}
    for (const auto v : transform.rotation) {
      if (!std::isfinite(v)) {throw std::invalid_argument("Invalid rotation");}
    }
    for (const auto v : transform.translation) {
      if (!std::isfinite(v)) {throw std::invalid_argument("Invalid translation");}
    }
    width_=width; height_=height; transform_=transform; options_=o;
    rays_.clear();
    for (int v=roi.y;v<roi.y+roi.height;v+=o.pixel_stride) {
      for (int u=roi.x;u<roi.x+roi.width;u+=o.pixel_stride) {
        rays_.push_back({u,v,point_cloud::rotate(transform.rotation,
          {(u-k.cx)/k.fx,(v-k.cy)/k.fy,1.0})});
      }
    }
  }
  std::size_t rayCount() const {return rays_.size();}
  // Return a nonnegative association ID to keep a sample, or -1 to omit it.
  using PointSelector = std::function<int(int u, int v, double depth_m)>;
  Scan project(const std::uint8_t * bytes, std::size_t size, std::size_t stride,
    double below_ground_m = .30, const PointSelector * select = nullptr) const
  {
    if(!std::isfinite(below_ground_m) || below_ground_m<0 || below_ground_m>.30)throw std::invalid_argument("Invalid below-ground limit");
    if (!bytes || !width_ || stride < static_cast<std::size_t>(width_)*2 ||
      size < static_cast<std::size_t>(width_)*2 ||
      (height_>1 && stride>(size-static_cast<std::size_t>(width_)*2)/static_cast<std::size_t>(height_-1))) {
      throw std::invalid_argument("Unconfigured projector or truncated RAW16 depth");
    }
    const auto & o=options_;
    auto out=emptyScan(o);
    out.surface_points.reserve(rays_.size());
    struct Location {float range;double angle;};
    std::vector<Location> locations;locations.reserve(rays_.size());
    std::vector<int> selection_ids;
    const double source_height=o.max_height+(o.ransac.enabled ? o.ransac.max_offset_m+std::tan(o.ransac.max_tilt_deg*radians)*o.ransac.max_range_m : 0);
    struct Sample {int bin; float range, height;};
    std::vector<Sample> samples; samples.reserve(rays_.size());
    for (const auto & ray : rays_) {
      const auto offset=static_cast<std::size_t>(ray.v)*stride+ray.u*2;
      const auto raw=static_cast<std::uint16_t>(bytes[offset] | (bytes[offset+1]<<8));
      const double depth=raw*0.001;
      if (!raw || depth<o.min_depth || depth>o.max_depth) {continue;}
      ++out.depth_points;
      const int selected = select ? (*select)(ray.u, ray.v, depth) : 0;
      if (selected < 0) continue;
      const double z=ray.direction[2]*depth+transform_.translation[2];
      if (z>source_height) {++out.height_removed; continue;}
      const double x=ray.direction[0]*depth+transform_.translation[0];
      const double y=ray.direction[1]*depth+transform_.translation[1];
      if (x<=0) {continue;}
      const double range=std::hypot(x,y);
      if (range<o.min_range || range>o.max_range) {continue;}
      const double angle=std::atan2(y,x)/radians;
      if (angle<o.angle_min || angle>o.angle_max) {continue;}
      out.surface_points.push_back({x,y,z});
      locations.push_back({static_cast<float>(range),angle});
      if (select) selection_ids.push_back(selected);
    }
    if (o.ransac.enabled) {out.ground_plane=fitGround(out.surface_points,o.ransac);}
    std::vector<std::uint8_t> keep(out.surface_points.size(),1);
    std::vector<IndexedPoint> candidates;
    if(o.outlier.enabled)candidates.reserve(out.surface_points.size()/8);
    for(std::size_t i=0;i<out.surface_points.size();++i){
      const auto physical=out.surface_points[i];
      const bool fitted=out.ground_plane.detected && physical[0]*physical[0]+physical[1]*physical[1]<=o.ransac.max_range_m*o.ransac.max_range_m;
      const double height=fitted?planeDistance(out.ground_plane.coefficients,physical):physical[2];
      if(height>o.max_height){keep[i]=0;++out.height_removed;continue;}
      out.surface_points[i][2]=height;
      const double limit=groundMargin(physical[0],physical[1],o,&out);
      if(o.outlier.enabled && (height>limit || height<-below_ground_m))candidates.push_back({physical,i});
    }
    out.outlier_candidates=candidates.size();
    if(o.outlier.enabled){
      const auto supported=supportedPoints(candidates,o.outlier);
      for(std::size_t i=0;i<candidates.size();++i)if(!supported[i]){
        const auto index=candidates[i].index;keep[index]=0;++out.outlier_removed;
        out.rejected_points.push_back(out.surface_points[index]);
      }
    }
    std::size_t retained=0;
    for(std::size_t i=0;i<out.surface_points.size();++i){
      if(!keep[i])continue;
      const auto point=out.surface_points[i];out.surface_points[retained]=point;
      const auto location=locations[i];const auto index=retained++;
      if(o.ground_enabled && point[2]<=groundMargin(point[0],point[1],o,&out)){++out.ground_removed;continue;}
      out.obstacle_indices.push_back(index);
      if (select) out.obstacle_selection_ids.push_back(selection_ids[i]);
      const int bin=std::clamp(static_cast<int>(std::lround(
        (location.angle-o.angle_min)*(o.bins-1)/(o.angle_max-o.angle_min))),0,o.bins-1);
      auto & nearest=out.ranges[bin];
      if(!std::isfinite(nearest) || location.range<nearest)nearest=location.range;
      samples.push_back({bin,location.range,static_cast<float>(point[2])});++out.accepted_points;
    }
    out.surface_points.resize(retained);out.processed=true;
    std::vector<int> support(o.bins,0);
    for (const auto & sample : samples) {
      if (sample.range-out.ranges[sample.bin]<=o.support_distance) {
        ++support[sample.bin];
        auto & height=out.max_heights[sample.bin];
        if (!std::isfinite(height) || sample.height>height) {height=sample.height;}
      }
    }
    for (int i=0;i<o.bins;++i) {
      if (support[i]<o.min_samples) {out.ranges[i]=out.max_heights[i]=std::numeric_limits<float>::quiet_NaN();}
      out.valid_bins+=std::isfinite(out.ranges[i]);
    }
    return out;
  }
};
} // namespace roi_lidar
