#include <gtest/gtest.h>

#include "manual_capture/bev_projector.hpp"

using namespace manual_capture;

namespace
{
LensCalibration lens()
{
  LensCalibration calibration;
  calibration.fx = 560.0;
  calibration.fy = 560.0;
  calibration.cx = 640.0;
  calibration.cy = 400.0;
  calibration.width = 1280;
  calibration.height = 800;
  return calibration;
}
}  // namespace

TEST(BevProjector, ProjectsColorAndGrayOntoTheSameGrid)
{
  BevGridConfig grid;
  BevProjector projector(grid);
  EXPECT_FALSE(projector.ready());
  projector.configure(lens(), MountPose{0.20, 0.0, 14.0});
  ASSERT_TRUE(projector.ready());

  const cv::Mat gray(800, 1280, CV_8UC1, cv::Scalar(200));
  const cv::Mat color(800, 1280, CV_8UC3, cv::Scalar(10, 20, 30));
  const cv::Mat gray_bev = projector.project(gray);
  const cv::Mat color_bev = projector.project(color);
  EXPECT_EQ(gray_bev.size(), cv::Size(grid.output_width, grid.output_height));
  EXPECT_EQ(color_bev.size(), gray_bev.size());
  EXPECT_EQ(color_bev.type(), CV_8UC3);
  // Ground straight ahead of a centered camera is visible on the center column.
  EXPECT_EQ(gray_bev.at<std::uint8_t>(grid.output_height / 2, grid.output_width / 2), 200);
}

TEST(BevProjector, SymmetricCenteredLensGivesSymmetricCoverage)
{
  BevProjector projector(BevGridConfig{});
  projector.configure(lens(), MountPose{0.20, 0.0, 14.0});
  const cv::Mat bev = projector.project(cv::Mat(800, 1280, CV_8UC1, cv::Scalar(255)));
  const int row = bev.rows / 3;
  for (int offset = 0; offset < bev.cols / 2; ++offset) {
    EXPECT_EQ(bev.at<std::uint8_t>(row, bev.cols / 2 - 1 - offset) > 0,
      bev.at<std::uint8_t>(row, bev.cols / 2 + offset) > 0) << offset;
  }
}

TEST(BevProjector, RejectsWrongImageSizeAndBadGrid)
{
  BevProjector projector(BevGridConfig{});
  projector.configure(lens(), MountPose{});
  EXPECT_THROW(projector.project(cv::Mat(400, 640, CV_8UC1)), std::invalid_argument);
  BevGridConfig bad;
  bad.output_width = 100;
  EXPECT_THROW(BevProjector{bad}, std::invalid_argument);
}
