#include "bev_processor/bev_geometry.hpp"

#include <cmath>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace bev_processor
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

}  // namespace

double degToRad(const double degrees)
{
  return degrees * kPi / 180.0;
}

cv::Matx33d rotationX(const double angle)
{
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  return cv::Matx33d(
    1.0, 0.0, 0.0,
    0.0, cosine, -sine,
    0.0, sine, cosine);
}

cv::Matx33d rotationY(const double angle)
{
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  return cv::Matx33d(
    cosine, 0.0, sine,
    0.0, 1.0, 0.0,
    -sine, 0.0, cosine);
}

cv::Matx33d rotationZ(const double angle)
{
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  return cv::Matx33d(
    cosine, -sine, 0.0,
    sine, cosine, 0.0,
    0.0, 0.0, 1.0);
}

cv::Matx33d mountRotationVehicleFromCamera(
  const double roll_rad,
  const double downward_pitch_rad,
  const double yaw_rad)
{
  // At zero angles: camera +X=vehicle -Y, camera +Y=vehicle -Z,
  // camera +Z=vehicle +X. Positive pitch points the camera downward.
  const cv::Matx33d optical_to_vehicle(
    0.0, 0.0, 1.0,
    -1.0, 0.0, 0.0,
    0.0, -1.0, 0.0);
  return rotationZ(yaw_rad) *
         rotationY(downward_pitch_rad) *
         rotationX(roll_rad) *
         optical_to_vehicle;
}

EulerAngles cameraAttitudeFromSpecificForce(
  const cv::Vec3d & acceleration_camera_mps2)
{
  const double magnitude = cv::norm(acceleration_camera_mps2);
  if (!std::isfinite(magnitude) || magnitude <= 1.0e-9) {
    throw std::invalid_argument(
            "camera-frame acceleration must be finite and non-zero");
  }

  // Camera optical coordinates are RDF: +X right, +Y down, +Z forward.
  // At rest an accelerometer measures specific force opposite gravity, so a
  // level camera reads approximately (0, -g, 0). These equations match the
  // roll and positive-downward-pitch convention used by the BEV mount model.
  return EulerAngles{
    std::atan2(
      -acceleration_camera_mps2[0],
      -acceleration_camera_mps2[1]),
    std::atan2(
      -acceleration_camera_mps2[2],
      std::hypot(
        acceleration_camera_mps2[0],
        acceleration_camera_mps2[1])),
    0.0};
}

RemapLut generateRemap(
  const RectifiedCameraModel & camera,
  const BevConfig & bev)
{
  RemapLut lut{
    cv::Mat(
      bev.output_height, bev.output_width,
      CV_32FC1, cv::Scalar(-1.0F)),
    cv::Mat(
      bev.output_height, bev.output_width,
      CV_32FC1, cv::Scalar(-1.0F)),
    cv::Mat(
      bev.output_height, bev.output_width,
      CV_8UC1, cv::Scalar(0))};

  // P_c = R_cv * (P_v - C_v), where R_cv = R_vc^T.
  const cv::Matx33d rotation_camera_from_vehicle =
    camera.rotation_vehicle_from_camera.t();

  for (int v_bev = 0; v_bev < bev.output_height; ++v_bev) {
    const double x_vehicle =
      bev.x_max_m -
      (static_cast<double>(v_bev) + 0.5) * bev.meter_per_pixel;

    for (int u_bev = 0; u_bev < bev.output_width; ++u_bev) {
      const double y_vehicle =
        bev.y_max_m -
        (static_cast<double>(u_bev) + 0.5) * bev.meter_per_pixel;
      const cv::Vec3d point_vehicle(x_vehicle, y_vehicle, 0.0);
      const cv::Vec3d point_camera =
        rotation_camera_from_vehicle *
        (point_vehicle - camera.position_vehicle_m);

      if (point_camera[2] <= 1.0e-6) {
        continue;
      }

      const double u_source =
        camera.fx * point_camera[0] / point_camera[2] + camera.cx;
      const double v_source =
        camera.fy * point_camera[1] / point_camera[2] + camera.cy;

      // Keep one interpolation pixel inside the right and bottom edges.
      if (
        u_source < 0.0 || v_source < 0.0 ||
        u_source >= static_cast<double>(camera.image_width - 1) ||
        v_source >= static_cast<double>(camera.image_height - 1))
      {
        continue;
      }

      lut.map_x.at<float>(v_bev, u_bev) =
        static_cast<float>(u_source);
      lut.map_y.at<float>(v_bev, u_bev) =
        static_cast<float>(v_source);
      lut.valid_mask.at<std::uint8_t>(v_bev, u_bev) = 255U;
    }
  }
  return lut;
}

FusedRemapCoverage assessFusedRemapCoverage(
  const RemapLut & lut,
  const cv::Matx33d & source_to_stabilized_homography,
  const int source_width,
  const int source_height,
  const int source_crop_top)
{
  if (
    lut.map_x.empty() || lut.map_y.empty() || lut.valid_mask.empty() ||
    lut.map_x.size() != lut.map_y.size() ||
    lut.map_x.size() != lut.valid_mask.size() ||
    lut.map_x.type() != CV_32FC1 || lut.map_y.type() != CV_32FC1 ||
    lut.valid_mask.type() != CV_8UC1 ||
    !cv::checkRange(cv::Mat(source_to_stabilized_homography)) ||
    std::abs(cv::determinant(cv::Mat(source_to_stabilized_homography))) <
    1.0e-9 ||
    source_width <= 1 || source_height <= 1 ||
    source_width % 2 != 0 || source_height % 2 != 0 ||
    source_crop_top < 0 || source_crop_top >= source_height ||
    source_crop_top % 2 != 0)
  {
    throw std::invalid_argument("invalid fused BEV remap geometry");
  }

  FusedRemapCoverage result;
  const cv::Matx33d stabilized_to_source =
    source_to_stabilized_homography.inv(cv::DECOMP_LU);
  const int cropped_height = source_height - source_crop_top;
  for (int row = 0; row < lut.map_x.rows; ++row) {
    for (int column = 0; column < lut.map_x.cols; ++column) {
      if (lut.valid_mask.at<std::uint8_t>(row, column) == 0U) {
        continue;
      }
      ++result.valid_lut_pixels;
      const cv::Vec3d source_homogeneous =
        stabilized_to_source * cv::Vec3d(
        static_cast<double>(lut.map_x.at<float>(row, column)),
        static_cast<double>(lut.map_y.at<float>(row, column)),
        1.0);
      if (
        !std::isfinite(source_homogeneous[0]) ||
        !std::isfinite(source_homogeneous[1]) ||
        !std::isfinite(source_homogeneous[2]) ||
        std::abs(source_homogeneous[2]) < 1.0e-9)
      {
        continue;
      }
      const double source_x =
        source_homogeneous[0] / source_homogeneous[2];
      const double cropped_source_y =
        source_homogeneous[1] / source_homogeneous[2] - source_crop_top;
      if (
        source_x >= 0.0 && source_x < source_width - 1.0 &&
        cropped_source_y >= 0.0 &&
        cropped_source_y < cropped_height - 1.0)
      {
        ++result.covered_pixels;
      }
    }
  }
  if (result.valid_lut_pixels > 0) {
    result.coverage_ratio =
      static_cast<double>(result.covered_pixels) /
      static_cast<double>(result.valid_lut_pixels);
  }
  return result;
}

cv::Mat convertToBev(
  const cv::Mat & rectified_image,
  const RemapLut & lut)
{
  cv::Mat bev_image;
  cv::remap(
    rectified_image,
    bev_image,
    lut.map_x,
    lut.map_y,
    cv::INTER_LINEAR,
    cv::BORDER_CONSTANT,
    cv::Scalar(0, 0, 0));
  return bev_image;
}

}  // namespace bev_processor
