#include "manual_capture/bev_projector.hpp"

#include <cmath>
#include <stdexcept>

namespace manual_capture
{

void validateGrid(const BevGridConfig & grid)
{
  const bool finite =
    std::isfinite(grid.camera_x_m) && std::isfinite(grid.camera_y_m) &&
    std::isfinite(grid.camera_yaw_deg) && std::isfinite(grid.x_min_m) &&
    std::isfinite(grid.x_max_m) && std::isfinite(grid.y_min_m) &&
    std::isfinite(grid.y_max_m) && std::isfinite(grid.meter_per_pixel);
  if (!finite || grid.x_max_m <= grid.x_min_m || grid.y_max_m <= grid.y_min_m ||
    grid.meter_per_pixel <= 0.0 || grid.output_width <= 0 || grid.output_height <= 0)
  {
    throw std::invalid_argument("invalid BEV grid configuration");
  }
  const int expected_width = static_cast<int>(std::lround(
      (grid.y_max_m - grid.y_min_m) / grid.meter_per_pixel));
  const int expected_height = static_cast<int>(std::lround(
      (grid.x_max_m - grid.x_min_m) / grid.meter_per_pixel));
  if (grid.output_width != expected_width || grid.output_height != expected_height) {
    throw std::invalid_argument(
            "BEV output size does not match metric extents and meter_per_pixel");
  }
}

BevProjector::BevProjector(BevGridConfig grid)
: grid_(grid)
{
  validateGrid(grid_);
}

void BevProjector::configure(const LensCalibration & lens, const MountPose & pose)
{
  if (!std::isfinite(lens.fx) || !std::isfinite(lens.fy) || !std::isfinite(lens.cx) ||
    !std::isfinite(lens.cy) || lens.fx <= 0.0 || lens.fy <= 0.0 || lens.width <= 1 ||
    lens.height <= 1 || !cv::checkRange(cv::Mat(lens.rotation_rgb_from_view)) ||
    !cv::checkRange(cv::Mat(lens.view_center_rgb_m)))
  {
    throw std::invalid_argument("invalid lens calibration");
  }
  lens_ = lens;
  has_lens_ = true;
  pose_ = pose;
  rebuild();
}

void BevProjector::setPose(const MountPose & pose)
{
  pose_ = pose;
  if (has_lens_) {
    rebuild();
  }
}

void BevProjector::reset()
{
  has_lens_ = false;
  lut_ = RemapLut{};
}

bool BevProjector::ready() const noexcept
{
  return !lut_.map_x.empty() && !lut_.map_y.empty();
}

void BevProjector::rebuild()
{
  if (!std::isfinite(pose_.height_m) || pose_.height_m <= 0.0 ||
    !std::isfinite(pose_.roll_deg) || !std::isfinite(pose_.pitch_down_deg))
  {
    throw std::invalid_argument("invalid BEV mount pose");
  }
  // Same math as the vehicle BEV: CAM_A pose measured on the ground plane,
  // composed with the calibrated CAM_A->lens transform.
  const cv::Matx33d rotation_vehicle_from_rgb = mountRotationVehicleFromCamera(
    degToRad(pose_.roll_deg), degToRad(pose_.pitch_down_deg), degToRad(grid_.camera_yaw_deg));
  RectifiedCameraModel camera{
    lens_.fx, lens_.fy, lens_.cx, lens_.cy, lens_.width, lens_.height,
    cv::Vec3d(grid_.camera_x_m, grid_.camera_y_m, pose_.height_m) +
    rotation_vehicle_from_rgb * lens_.view_center_rgb_m,
    rotation_vehicle_from_rgb * lens_.rotation_rgb_from_view};
  const BevConfig bev{grid_.x_min_m, grid_.x_max_m, grid_.y_min_m, grid_.y_max_m,
    grid_.meter_per_pixel, grid_.output_width, grid_.output_height};
  lut_ = generateRemap(camera, bev);
}

cv::Mat BevProjector::project(const cv::Mat & image) const
{
  if (!ready()) {
    throw std::runtime_error("BEV projector is not configured");
  }
  if (image.empty() || (image.type() != CV_8UC1 && image.type() != CV_8UC3) ||
    image.cols != lens_.width || image.rows != lens_.height)
  {
    throw std::invalid_argument("BEV input image does not match the lens calibration");
  }
  return convertToBev(image, lut_);
}

}  // namespace manual_capture
