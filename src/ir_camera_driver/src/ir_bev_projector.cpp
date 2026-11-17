#include "ir_camera_driver/ir_bev_projector.hpp"

#include <cmath>
#include <stdexcept>

#include "opencv2/imgproc.hpp"

namespace ir_camera_driver
{
namespace
{

void validateConfig(const IrBevConfig & config)
{
  const bool finite =
    std::isfinite(config.camera_x_m) &&
    std::isfinite(config.camera_y_m) &&
    std::isfinite(config.camera_height_m) &&
    std::isfinite(config.camera_roll_deg) &&
    std::isfinite(config.camera_pitch_down_deg) &&
    std::isfinite(config.camera_yaw_deg) &&
    std::isfinite(config.x_min_m) &&
    std::isfinite(config.x_max_m) &&
    std::isfinite(config.y_min_m) &&
    std::isfinite(config.y_max_m) &&
    std::isfinite(config.meter_per_pixel);
  if (!finite || config.camera_height_m <= 0.0 ||
    config.x_max_m <= config.x_min_m || config.y_max_m <= config.y_min_m ||
    config.meter_per_pixel <= 0.0 || config.output_width <= 0 ||
    config.output_height <= 0)
  {
    throw std::invalid_argument("invalid IR BEV geometry configuration");
  }

  const int expected_width = static_cast<int>(std::lround(
      (config.y_max_m - config.y_min_m) / config.meter_per_pixel));
  const int expected_height = static_cast<int>(std::lround(
      (config.x_max_m - config.x_min_m) / config.meter_per_pixel));
  if (config.output_width != expected_width || config.output_height != expected_height) {
    throw std::invalid_argument(
            "IR BEV output dimensions do not match metric extents and meter_per_pixel");
  }
}

}  // namespace

IrBevProjector::IrBevProjector(IrBevConfig config)
: config_(config)
{
  validateConfig(config_);
}

void IrBevProjector::configure(
  const double fx, const double fy, const double cx, const double cy,
  const int input_width, const int input_height,
  const cv::Matx33d & rotation_rgb_from_view,
  const cv::Vec3d & view_center_rgb_m)
{
  if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(cx) ||
    !std::isfinite(cy) || fx <= 0.0 || fy <= 0.0 || input_width <= 1 ||
    input_height <= 1 || !cv::checkRange(cv::Mat(rotation_rgb_from_view)) ||
    !cv::checkRange(cv::Mat(view_center_rgb_m)))
  {
    throw std::invalid_argument("invalid rectified IR camera calibration");
  }
  camera_ = {fx, fy, cx, cy, input_width, input_height,
    cv::Vec3d(0.0, 0.0, 0.0), cv::Matx33d::eye()};
  grid_ = {config_.x_min_m, config_.x_max_m, config_.y_min_m, config_.y_max_m,
    config_.meter_per_pixel, config_.output_width, config_.output_height};
  rotation_vehicle_from_rgb_ = bev_processor::mountRotationVehicleFromCamera(
    bev_processor::degToRad(config_.camera_roll_deg),
    bev_processor::degToRad(config_.camera_pitch_down_deg),
    bev_processor::degToRad(config_.camera_yaw_deg));
  rotation_rgb_from_view_ = rotation_rgb_from_view;
  view_center_rgb_m_ = view_center_rgb_m;
  updateMaps(cv::Matx33d::eye());
}

void IrBevProjector::updateMaps(const cv::Matx33d & current_rgb_to_reference)
{
  // Same vehicle axes, metric pixel centers and inverse projection as
  // bev_processor. The startup measurement is in CAM_A optical coordinates.
  const cv::Matx33d rotation_vehicle_from_current_rgb =
    rotation_vehicle_from_rgb_ * current_rgb_to_reference;
  camera_.rotation_vehicle_from_camera =
    rotation_vehicle_from_current_rgb * rotation_rgb_from_view_;
  camera_.position_vehicle_m =
    cv::Vec3d(config_.camera_x_m, config_.camera_y_m, config_.camera_height_m) +
    rotation_vehicle_from_current_rgb * view_center_rgb_m_;
  // Rotate the calibrated RGB-to-IR lever arm as well as the optical axes.
  // Translation/heave of the whole device is not observable from tilt alone.
  lut_ = bev_processor::generateRemap(camera_, grid_);
  last_correction_ = current_rgb_to_reference;
}

cv::Mat IrBevProjector::project(
  const cv::Mat & rectified_image,
  const cv::Matx33d & current_rgb_to_reference)
{
  if (!ready()) {
    throw std::runtime_error("IR BEV projector has not received camera intrinsics");
  }
  if (rectified_image.empty() || rectified_image.type() != CV_8UC1 ||
    rectified_image.cols != camera_.image_width ||
    rectified_image.rows != camera_.image_height ||
    !cv::checkRange(cv::Mat(current_rgb_to_reference)))
  {
    throw std::invalid_argument("invalid IR BEV image or IMU correction");
  }
  if (cv::norm(cv::Mat(current_rgb_to_reference - last_correction_)) > 1.0e-10) {
    updateMaps(current_rgb_to_reference);
  }
  return bev_processor::convertToBev(rectified_image, lut_);
}

bool IrBevProjector::ready() const noexcept
{
  return !lut_.map_x.empty() && !lut_.map_y.empty();
}

void IrBevProjector::setMountPose(
  const cv::Matx33d & rotation_vehicle_from_rgb, const cv::Vec3d & position)
{
  if (!ready() || !cv::checkRange(cv::Mat(rotation_vehicle_from_rgb)) ||
    !cv::checkRange(cv::Mat(position)) || position[2]<=0.0) {
    throw std::invalid_argument("invalid shared BEV mounting pose");
  }
  rotation_vehicle_from_rgb_=rotation_vehicle_from_rgb;
  config_.camera_x_m=position[0]; config_.camera_y_m=position[1];
  config_.camera_height_m=position[2];
  updateMaps(cv::Matx33d::eye());
}

double IrBevProjector::validPixelRatio() const noexcept
{
  if (lut_.valid_mask.empty()) {
    return 0.0;
  }
  return static_cast<double>(cv::countNonZero(lut_.valid_mask)) /
         static_cast<double>(lut_.valid_mask.total());
}

const cv::Mat & IrBevProjector::validMask() const noexcept
{
  return lut_.valid_mask;
}

}  // namespace ir_camera_driver
