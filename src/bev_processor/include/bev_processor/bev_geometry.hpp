#ifndef BEV_PROCESSOR__BEV_GEOMETRY_HPP_
#define BEV_PROCESSOR__BEV_GEOMETRY_HPP_

#include <cstdint>

#include <opencv2/core.hpp>

namespace bev_processor
{

struct EulerAngles
{
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

struct RectifiedCameraModel
{
  double fx;
  double fy;
  double cx;
  double cy;
  int image_width;
  int image_height;
  cv::Vec3d position_vehicle_m;
  cv::Matx33d rotation_vehicle_from_camera;
};

struct BevConfig
{
  double x_min_m;
  double x_max_m;
  double y_min_m;
  double y_max_m;
  double meter_per_pixel;
  int output_width;
  int output_height;
};

struct RemapLut
{
  cv::Mat map_x;
  cv::Mat map_y;
  cv::Mat valid_mask;
};

struct FusedRemapCoverage
{
  int valid_lut_pixels{0};
  int covered_pixels{0};
  double coverage_ratio{0.0};
};

double degToRad(double degrees);

cv::Matx33d rotationX(double angle);
cv::Matx33d rotationY(double angle);
cv::Matx33d rotationZ(double angle);

cv::Matx33d mountRotationVehicleFromCamera(
  double roll_rad,
  double downward_pitch_rad,
  double yaw_rad);

EulerAngles cameraAttitudeFromSpecificForce(
  const cv::Vec3d & acceleration_camera_mps2);

RemapLut generateRemap(
  const RectifiedCameraModel & camera,
  const BevConfig & bev);

FusedRemapCoverage assessFusedRemapCoverage(
  const RemapLut & lut,
  const cv::Matx33d & source_to_stabilized_homography,
  int source_width,
  int source_height,
  int source_crop_top);

cv::Mat convertToBev(
  const cv::Mat & rectified_image,
  const RemapLut & lut);

}  // namespace bev_processor

#endif  // BEV_PROCESSOR__BEV_GEOMETRY_HPP_
