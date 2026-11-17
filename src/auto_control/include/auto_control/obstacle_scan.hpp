#pragma once

#include <point_cloud/bev_crop.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace auto_control::obstacle
{
constexpr double radians = 3.14159265358979323846 / 180.0;
struct Options
{
  double roi_x{0}, roi_y{0}, roi_width{1}, roi_height{1}; // normalized image rectangle
  int pixel_stride{2};
  bool ground_enabled{true};
  double ground_distance{0.05}, ground_distance_per_meter{0.01}, ground_max_distance{0.12};
  double max_height{1.0}; // measured plane is vehicle Z=0
  double min_depth{0.1}, max_depth{20.0};
  double min_range{0.1}, max_range{12.0};
  double angle_min{-70}, angle_max{70};
  int bins{281}, min_samples{2};
  double support_distance{0.15};
};
inline void validate(const Options & o)
{
  for (const auto v : {o.roi_x,o.roi_y,o.roi_width,o.roi_height,o.ground_distance,
    o.ground_distance_per_meter,o.ground_max_distance,o.max_height,o.min_depth,o.max_depth,o.min_range,o.max_range,o.angle_min,
    o.angle_max,o.support_distance}) {
    if (!std::isfinite(v)) {throw std::invalid_argument("ROI/scan values must be finite");}
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
  std::vector<float> ranges;
  std::vector<float> max_heights; // Height evidence only from samples near the selected range.
  std::size_t depth_points{0}, ground_removed{0}, height_removed{0}, accepted_points{0}, valid_bins{0};
};
inline Scan emptyScan(const Options & o)
{
  Scan s; s.ranges.assign(o.bins, std::numeric_limits<float>::quiet_NaN());
  s.max_heights=s.ranges; return s;
}
class Projector
{
  // Compact per-ray table: RAW16 pixel index within its row plus the
  // normalized image-plane direction (z is always 1).
  struct Ray {std::uint32_t row, byte; float x, y;};
  struct Sample {int bin; float range, height;};
  std::vector<Ray> rays_;
  // Per-frame scratch, reused so a frame does not allocate ray-sized buffers.
  mutable std::vector<Sample> samples_;
  mutable std::vector<int> support_;
  point_cloud::RigidTransform transform_;
  Options options_;
  int width_{0}, height_{0};
  std::array<double,11> ray_key_{};
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
    const std::array<double,11> key{double(width),double(height),double(roi.x),double(roi.y),
      double(roi.width),double(roi.height),double(o.pixel_stride),k.fx,k.fy,k.cx,k.cy};
    if(key!=ray_key_) {
      rays_.clear();
      const auto columns=(roi.width+o.pixel_stride-1)/o.pixel_stride;
      const auto rows=(roi.height+o.pixel_stride-1)/o.pixel_stride;
      rays_.reserve(static_cast<std::size_t>(columns)*rows);
      for (int v=roi.y;v<roi.y+roi.height;v+=o.pixel_stride) {
        for (int u=roi.x;u<roi.x+roi.width;u+=o.pixel_stride) {
          rays_.push_back({static_cast<std::uint32_t>(v),static_cast<std::uint32_t>(u)*2U,
            static_cast<float>((u-k.cx)/k.fx),static_cast<float>((v-k.cy)/k.fy)});
        }
      }
      ray_key_=key;
    }
    // The changing RGB/IMU pose is never frozen in the intrinsic-ray cache.
    width_=width; height_=height; transform_=transform; options_=o;
  }
  std::size_t rayCount() const {return rays_.size();}
  Scan project(const std::uint8_t * bytes, std::size_t size, std::size_t stride) const
  {
    if (!bytes || !width_ || stride < static_cast<std::size_t>(width_)*2 ||
      size < static_cast<std::size_t>(width_)*2 ||
      (height_>1 && stride>(size-static_cast<std::size_t>(width_)*2)/static_cast<std::size_t>(height_-1))) {
      throw std::invalid_argument("Unconfigured projector or truncated RAW16 depth");
    }
    const auto & o=options_;
    auto out=emptyScan(o);
    auto & samples=samples_; samples.clear();
    if (samples.capacity()<rays_.size()) {samples.reserve(rays_.size());}
    const double bin_scale=(o.bins-1)/(o.angle_max-o.angle_min);
    for (const auto & ray : rays_) {
      const auto offset=static_cast<std::size_t>(ray.row)*stride+ray.byte;
      const auto raw=static_cast<std::uint16_t>(bytes[offset] | (bytes[offset+1]<<8));
      const double depth=raw*0.001;
      if (!raw || depth<o.min_depth || depth>o.max_depth) {continue;}
      ++out.depth_points;
      // Rotate only valid depth samples, with the same operation order as before.
      const auto direction=point_cloud::rotate(transform_.rotation,
        point_cloud::Vector3{static_cast<double>(ray.x),static_cast<double>(ray.y),1.0});
      const double z=direction[2]*depth+transform_.translation[2];
      if (z>o.max_height) {++out.height_removed; continue;}
      const double x=direction[0]*depth+transform_.translation[0];
      const double y=direction[1]*depth+transform_.translation[1];
      if (x<=0) {continue;}
      // The floor margin is never below ground_distance, so most floor pixels
      // are rejected here without computing their range.
      if (o.ground_enabled && z<=o.ground_distance) {++out.ground_removed; continue;}
      const double range=std::sqrt(x*x+y*y);
      const double floor_margin=std::min(o.ground_max_distance,
        o.ground_distance+o.ground_distance_per_meter*range);
      if (o.ground_enabled && z<=floor_margin) {++out.ground_removed; continue;}
      if (range<o.min_range || range>o.max_range) {continue;}
      const double angle=std::atan2(y,x)/radians;
      if (angle<o.angle_min || angle>o.angle_max) {continue;}
      const int bin=std::clamp(static_cast<int>(std::lround((angle-o.angle_min)*bin_scale)),0,o.bins-1);
      auto & nearest=out.ranges[bin];
      if (!std::isfinite(nearest) || range<nearest) {nearest=range;}
      samples.push_back({bin,static_cast<float>(range),static_cast<float>(z)});
      ++out.accepted_points;
    }
    auto & support=support_; support.assign(o.bins,0);
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
} // namespace auto_control::obstacle
