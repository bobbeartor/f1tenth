#ifndef IR_CAMERA_DRIVER__IR_BEV_PROJECTOR_HPP_
#define IR_CAMERA_DRIVER__IR_BEV_PROJECTOR_HPP_

#include "opencv2/core.hpp"
#include "bev_processor/bev_geometry.hpp"

namespace ir_camera_driver
{

struct IrBevConfig
{
  // All mounting coordinates refer to CAM_A, the shared startup measurement frame.
  double camera_x_m{-0.16};
  double camera_y_m{0.0};
  double camera_height_m{0.20};
  double camera_roll_deg{0.0};
  double camera_pitch_down_deg{14.0};
  double camera_yaw_deg{0.0};
  double x_min_m{0.0};
  double x_max_m{3.0};
  double y_min_m{-0.6};
  double y_max_m{0.6};
  double meter_per_pixel{0.01};
  int output_width{120};
  int output_height{300};
};

class IrBevProjector
{
public:
  explicit IrBevProjector(IrBevConfig config);

  void configure(
    double fx, double fy, double cx, double cy,
    int input_width, int input_height,
    const cv::Matx33d & rotation_rgb_from_view,
    const cv::Vec3d & view_center_rgb_m);

  cv::Mat project(
    const cv::Mat & rectified_image,
    const cv::Matx33d & current_rgb_to_reference);
  bool ready() const noexcept;
  // Reuse the measured RGB mount pose from the integrated BEV reference.
  void setMountPose(const cv::Matx33d & rotation_vehicle_from_rgb,
    const cv::Vec3d & rgb_center_vehicle_m);
  double validPixelRatio() const noexcept;
  const cv::Mat & validMask() const noexcept;

private:
  IrBevConfig config_;
  bev_processor::RectifiedCameraModel camera_{};
  bev_processor::BevConfig grid_{};
  bev_processor::RemapLut lut_;
  cv::Matx33d rotation_vehicle_from_rgb_{cv::Matx33d::eye()};
  cv::Matx33d rotation_rgb_from_view_{cv::Matx33d::eye()};
  cv::Vec3d view_center_rgb_m_{0.0, 0.0, 0.0};
  cv::Matx33d last_correction_{cv::Matx33d::eye()};
  void updateMaps(const cv::Matx33d & current_rgb_to_reference);
};

}  // namespace ir_camera_driver

#endif  // IR_CAMERA_DRIVER__IR_BEV_PROJECTOR_HPP_
