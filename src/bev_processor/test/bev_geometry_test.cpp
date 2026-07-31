#include <cmath>
#include <cstdlib>
#include <iostream>

#include <opencv2/core.hpp>

#include "bev_processor/bev_geometry.hpp"

namespace
{

bool require(const bool condition, const char * message)
{
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    return false;
  }
  return true;
}

}  // namespace

int main()
{
  constexpr double gravity_mps2 = 9.80665;
  constexpr double expected_roll_deg = 3.0;
  constexpr double expected_pitch_deg = 14.0;
  const double expected_roll_rad =
    bev_processor::degToRad(expected_roll_deg);
  const double expected_pitch_rad =
    bev_processor::degToRad(expected_pitch_deg);
  const cv::Vec3d specific_force_camera(
    -gravity_mps2 * std::sin(expected_roll_rad) *
    std::cos(expected_pitch_rad),
    -gravity_mps2 * std::cos(expected_roll_rad) *
    std::cos(expected_pitch_rad),
    -gravity_mps2 * std::sin(expected_pitch_rad));
  const auto imu_attitude =
    bev_processor::cameraAttitudeFromSpecificForce(specific_force_camera);

  bev_processor::RectifiedCameraModel camera{
    561.400939941,
    561.136352539,
    643.032653809,
    352.621124268,
    1280,
    720,
    cv::Vec3d(0.0, 0.0, 0.20),
    bev_processor::mountRotationVehicleFromCamera(
      0.0, bev_processor::degToRad(14.0), 0.0)};
  const bev_processor::BevConfig config{
    0.18, 3.0, -1.0, 1.0, 0.01, 200, 282};

  const auto lut = bev_processor::generateRemap(camera, config);
  bool passed = true;
  passed &= require(
    std::abs(imu_attitude.roll - expected_roll_rad) < 1.0e-9,
    "IMU specific force must recover camera roll");
  passed &= require(
    std::abs(imu_attitude.pitch - expected_pitch_rad) < 1.0e-9,
    "IMU specific force must recover positive downward camera pitch");
  passed &= require(
    lut.map_x.rows == 282 && lut.map_x.cols == 200,
    "remap dimensions must be 200x282");
  passed &= require(lut.map_x.type() == CV_32FC1, "map_x must be float32");
  passed &= require(lut.map_y.type() == CV_32FC1, "map_y must be float32");
  passed &= require(
    lut.valid_mask.type() == CV_8UC1, "valid mask must be uint8");

  const int valid = cv::countNonZero(lut.valid_mask);
  const double valid_ratio = static_cast<double>(valid) / (200.0 * 282.0);
  passed &= require(
    valid_ratio > 0.90,
    "more than 90 percent of the configured BEV must project into the image");

  const int far_row = 0;
  const int left_column = 0;
  const int right_column = config.output_width - 1;
  passed &= require(
    lut.map_x.at<float>(far_row, left_column) <
    lut.map_x.at<float>(far_row, right_column),
    "vehicle-left BEV pixels must map left of vehicle-right pixels");
  int nearest_valid_center_row = config.output_height - 1;
  while (
    nearest_valid_center_row > far_row &&
    lut.valid_mask.at<std::uint8_t>(
      nearest_valid_center_row, config.output_width / 2) == 0U)
  {
    --nearest_valid_center_row;
  }
  passed &= require(
    nearest_valid_center_row > far_row,
    "the configured range must contain a valid near center pixel");
  if (nearest_valid_center_row > far_row) {
    passed &= require(
      lut.map_y.at<float>(
        nearest_valid_center_row, config.output_width / 2) >
      lut.map_y.at<float>(far_row, config.output_width / 2),
      "near ground must map lower in the camera image than far ground");
  }

  cv::Mat input(720, 1280, CV_8UC3);
  for (int row = 0; row < input.rows; ++row) {
    for (int column = 0; column < input.cols; ++column) {
      input.at<cv::Vec3b>(row, column) = cv::Vec3b(
        static_cast<std::uint8_t>(column % 256),
        static_cast<std::uint8_t>(row % 256),
        127U);
    }
  }
  const cv::Mat output = bev_processor::convertToBev(input, lut);
  passed &= require(
    output.rows == 282 && output.cols == 200 && output.type() == CV_8UC3,
    "converted image must be 200x282 BGR8");
  passed &= require(
    cv::countNonZero(output.reshape(1)) > 0,
    "converted image must contain projected source pixels");

  bev_processor::RectifiedCameraModel driving_camera{
    561.400939941,
    561.136352539,
    643.032653809,
    352.621124268,
    1280,
    720,
    cv::Vec3d(0.0, 0.0, 0.17),
    bev_processor::mountRotationVehicleFromCamera(
      0.0, bev_processor::degToRad(13.0), 0.0)};
  const bev_processor::BevConfig driving_config{
    0.10, 3.5, -0.6, 0.6, 0.01, 120, 340};
  const auto driving_lut =
    bev_processor::generateRemap(driving_camera, driving_config);
  const int driving_valid = cv::countNonZero(driving_lut.valid_mask);
  const double driving_valid_ratio =
    static_cast<double>(driving_valid) / (120.0 * 340.0);
  passed &= require(
    driving_lut.valid_mask.at<std::uint8_t>(0, 60) != 0U,
    "the full-height image must retain the 3.5 meter center projection");
  passed &= require(
    driving_lut.map_y.at<float>(0, 60) >= 250.0F &&
    driving_lut.map_y.at<float>(0, 60) < 253.0F,
    "the 3.5 meter projection must retain its full-frame row");

  if (!passed) {
    return EXIT_FAILURE;
  }
  std::cout << "BEV geometry test passed; valid LUT ratio="
            << valid_ratio * 100.0 << "%, driving ratio="
            << driving_valid_ratio * 100.0 << "%\n";
  return EXIT_SUCCESS;
}
