#include "depth_lidar/depth_lidar_geometry.hpp"
#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace depth_lidar
{
namespace {
constexpr double rad = 3.14159265358979323846 / 180.0;
cv::Matx33d matrix(const FixedTransform & t) {
  return cv::Matx33d(t.rotation.data());
}
}
FixedTransform cameraMount(double roll_deg, double pitch_down_deg, double yaw_deg,
  double x_m, double y_m, double height_m)
{
  const double r = roll_deg * rad, p = pitch_down_deg * rad, y = yaw_deg * rad;
  const cv::Matx33d rx(1,0,0, 0,std::cos(r),-std::sin(r), 0,std::sin(r),std::cos(r));
  const cv::Matx33d ry(std::cos(p),0,std::sin(p), 0,1,0, -std::sin(p),0,std::cos(p));
  const cv::Matx33d rz(std::cos(y),-std::sin(y),0, std::sin(y),std::cos(y),0, 0,0,1);
  const cv::Matx33d optical_to_vehicle(0,0,1, -1,0,0, 0,-1,0);
  const auto rotation = rz * ry * rx * optical_to_vehicle; // Same convention as BEV.
  FixedTransform result;
  std::copy(rotation.val, rotation.val + 9, result.rotation.begin());
  result.translation = {{x_m, y_m, height_m}};
  return result;
}
FixedTransform compose(const FixedTransform & a, const FixedTransform & b)
{
  FixedTransform result;
  const auto rotation = matrix(a) * matrix(b);
  const auto translation = matrix(a) * cv::Vec3d(b.translation.data()) + cv::Vec3d(a.translation.data());
  std::copy(rotation.val, rotation.val + 9, result.rotation.begin());
  std::copy(translation.val, translation.val + 3, result.translation.begin());
  return result;
}
bool validateProjectionConfig(const ProjectionConfig & c, std::string & reason)
{
  if (!std::isfinite(c.roi_width_ratio) || c.roi_width_ratio <= 0 || c.roi_width_ratio > 1
      || !std::isfinite(c.roi_height_ratio) || c.roi_height_ratio <= 0 || c.roi_height_ratio > 1
      || !std::isfinite(c.roi_bottom_offset_ratio) || c.roi_bottom_offset_ratio < 0
      || c.roi_height_ratio + c.roi_bottom_offset_ratio > 1 || c.pixel_stride < 1 || c.pixel_stride > 16
      || !std::isfinite(c.min_depth_m) || !std::isfinite(c.max_depth_m)
      || c.min_depth_m <= 0 || c.max_depth_m <= c.min_depth_m || c.max_depth_m > 65.535
      || !std::isfinite(c.min_height_m) || !std::isfinite(c.max_height_m)
      || c.min_height_m < 0 || c.max_height_m <= c.min_height_m
      || !std::isfinite(c.min_range_m) || !std::isfinite(c.max_range_m)
      || c.min_range_m <= 0 || c.max_range_m <= c.min_range_m || !std::isfinite(c.range_offset_m)
      || !std::isfinite(c.angle_min_deg) || !std::isfinite(c.angle_max_deg)
      || c.angle_min_deg < -90 || c.angle_max_deg > 90 || c.angle_min_deg >= c.angle_max_deg
      || c.bins < 2 || c.bins > 2048 || c.min_points_per_bin < 1 || c.min_points_per_bin > 1000
      || c.min_neighbors < 0 || c.min_neighbors > 4 || !std::isfinite(c.neighbor_delta_m)
      || c.neighbor_delta_m <= 0) {
    reason = "invalid ROI, height/depth/range band, angular bins or neighbor filter";
    return false;
  }
  reason.clear();
  return true;
}
RoiRect computeRoi(int width, int height, double wr, double hr, double bottom)
{
  if (width <= 0 || height <= 0) { throw std::invalid_argument("invalid image size"); }
  const int w = std::clamp(static_cast<int>(std::lround(width * wr)), 1, width);
  const int h = std::clamp(static_cast<int>(std::lround(height * hr)), 1, height);
  const int offset = std::clamp(static_cast<int>(std::lround(height * bottom)), 0, height - h);
  return {(width - w) / 2, height - offset - h, w, h};
}
ScanResult emptyScan(const ProjectionConfig & c)
{
  ScanResult result;
  result.ranges.assign(c.bins, std::numeric_limits<float>::quiet_NaN());
  result.ages.assign(c.bins, std::numeric_limits<float>::quiet_NaN());
  return result;
}
void ScanProjector::configure(const CameraGeometry & camera, const FixedTransform & transform,
  const ProjectionConfig & c)
{
  std::string reason;
  if (!validateProjectionConfig(c, reason) || camera.width <= 0 || camera.height <= 0
      || camera.width > 4096 || camera.height > 4096 || !std::isfinite(camera.fx) || camera.fx <= 0
      || !std::isfinite(camera.fy) || camera.fy <= 0 || !std::isfinite(camera.cx) || !std::isfinite(camera.cy)) {
    throw std::invalid_argument("invalid scan projector: " + reason);
  }
  const auto rotation = matrix(transform);
  if (!cv::checkRange(cv::Mat(rotation)) || cv::norm(rotation * rotation.t() - cv::Matx33d::eye()) > 1e-3
      || std::abs(cv::determinant(cv::Mat(rotation)) - 1.0) > 1e-3
      || !std::all_of(transform.translation.begin(), transform.translation.end(), [](double v) { return std::isfinite(v); })) {
    throw std::invalid_argument("invalid fixed camera transform");
  }
  configured_ = false;
  camera_ = camera; transform_ = transform; config_ = c;
  roi_ = computeRoi(camera.width, camera.height, c.roi_width_ratio, c.roi_height_ratio, c.roi_bottom_offset_ratio);
  rays_.clear();
  rays_.reserve(((roi_.width + c.pixel_stride - 1) / c.pixel_stride) * ((roi_.height + c.pixel_stride - 1) / c.pixel_stride));
  for (int v = roi_.y; v < roi_.y + roi_.height; v += c.pixel_stride) {
    for (int u = roi_.x; u < roi_.x + roi_.width; u += c.pixel_stride) {
      const auto ray = rotation * cv::Vec3d((u - camera.cx) / camera.fx, (v - camera.cy) / camera.fy, 1.0);
      double lo = c.min_depth_m, hi = c.max_depth_m;
      if (std::abs(ray[2]) < 1e-10) {
        if (transform.translation[2] < c.min_height_m || transform.translation[2] > c.max_height_m) { continue; }
      } else {
        double a = (c.min_height_m - transform.translation[2]) / ray[2];
        double b = (c.max_height_m - transform.translation[2]) / ray[2];
        if (a > b) { std::swap(a, b); }
        lo = std::max(lo, a); hi = std::min(hi, b);
      }
      if (lo > hi) { continue; }
      const double low_mm = std::ceil(lo * 1000.0), high_mm = std::floor(hi * 1000.0);
      if (low_mm > high_mm) { continue; }
      rays_.push_back({u, v, ray[0], ray[1],
        static_cast<std::uint16_t>(low_mm), static_cast<std::uint16_t>(high_mm)});
    }
  }
  if (rays_.empty()) { throw std::runtime_error("height band is outside the selected depth ROI"); }
  configured_ = true;
}
ScanResult ScanProjector::project(const std::uint16_t * depth, std::size_t stride) const
{
  if (!configured_ || !depth || stride < static_cast<std::size_t>(camera_.width)) {
    throw std::invalid_argument("scan projector not configured or invalid depth stride");
  }
  auto result = emptyScan(config_);
  std::vector<int> counts(config_.bins, 0);
  const double min_angle = config_.angle_min_deg * rad;
  const double inverse_step = (config_.bins - 1) / ((config_.angle_max_deg - config_.angle_min_deg) * rad);
  const auto & t = transform_.translation;
  for (const auto & ray : rays_) {
    const auto raw = depth[static_cast<std::size_t>(ray.v) * stride + ray.u];
    if (!raw || raw < ray.min_depth || raw > ray.max_depth) { continue; }
    if (config_.min_neighbors > 0) {
      int support = 0;
      const int step = config_.pixel_stride;
      for (const auto offset : {std::pair<int,int>{-step,0}, {step,0}, {0,-step}, {0,step}}) {
        const int u = ray.u + offset.first, v = ray.v + offset.second;
        if (u < roi_.x || u >= roi_.x + roi_.width || v < roi_.y || v >= roi_.y + roi_.height) { continue; }
        const auto neighbor = depth[static_cast<std::size_t>(v) * stride + u];
        if (neighbor && std::abs(static_cast<int>(neighbor) - raw) * 0.001 <= config_.neighbor_delta_m) { ++support; }
      }
      if (support < config_.min_neighbors) { continue; }
    }
    const double z = raw * 0.001;
    const double x = ray.x * z + t[0], y = ray.y * z + t[1];
    if (x <= 0.0) { continue; }
    const double range = std::hypot(x, y) + config_.range_offset_m;
    if (range < config_.min_range_m || range > config_.max_range_m) { continue; }
    const double angle = std::atan2(y, x);
    if (angle < min_angle || angle > config_.angle_max_deg * rad) { continue; }
    const int bin = std::clamp(static_cast<int>(std::lround((angle - min_angle) * inverse_step)), 0, config_.bins - 1);
    ++counts[bin]; ++result.accepted_points;
    if (!std::isfinite(result.ranges[bin]) || range < result.ranges[bin]) { result.ranges[bin] = static_cast<float>(range); }
  }
  for (int i = 0; i < config_.bins; ++i) {
    if (counts[i] < config_.min_points_per_bin) { result.ranges[i] = std::numeric_limits<float>::quiet_NaN(); }
    else { result.ages[i] = 0.0F; ++result.valid_bins; }
  }
  return result;
}
} // namespace depth_lidar
