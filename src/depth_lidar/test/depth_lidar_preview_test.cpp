#include "depth_lidar/depth_lidar_preview.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

TEST(DepthLidarPreview, StereoUsesActualDepthRoiAndPreservesBothImages)
{
  // Non-contiguous camera buffers exercise row stride handling in rendering.
  const cv::Mat storage(400, 660, CV_8UC1, cv::Scalar(80));
  const cv::Mat left = storage(cv::Rect(0, 0, 640, 400));
  const cv::Mat right(400, 640, CV_8UC1, cv::Scalar(120));
  const auto roi = depth_lidar::computeRoi(640, 400, 1.0, 0.10, 0.35);
  const auto preview = depth_lidar::makeStereoPreview(left, right, roi, 640, 400);
  ASSERT_EQ(preview.cols, 1280);
  ASSERT_EQ(preview.rows, 482);
  EXPECT_EQ(preview.at<cv::Vec3b>(54 + 220, 320), cv::Vec3b(0, 220, 0));
  EXPECT_EQ(preview.at<cv::Vec3b>(54 + 259, 960), cv::Vec3b(0, 220, 0));
  EXPECT_EQ(preview.at<cv::Vec3b>(54 + 240, 320), cv::Vec3b(80, 80, 80));
  EXPECT_EQ(preview.at<cv::Vec3b>(54 + 240, 960), cv::Vec3b(120, 120, 120));
  EXPECT_EQ(left.at<unsigned char>(220, 320), 80);
  EXPECT_EQ(right.at<unsigned char>(259, 320), 120);
}

TEST(DepthLidarPreview, ScalesDecimatedDepthRoiAndReflectsChangedParameters)
{
  const cv::Mat frame(400, 640, CV_8UC1, cv::Scalar(80));
  const auto roi = depth_lidar::computeRoi(320, 200, 0.5, 0.10, 0.35);
  const auto preview = depth_lidar::makeStereoPreview(frame, frame, roi, 320, 200);
  EXPECT_EQ(preview.at<cv::Vec3b>(54 + 220, 320), cv::Vec3b(0, 220, 0));
  EXPECT_EQ(preview.at<cv::Vec3b>(54 + 240, 160), cv::Vec3b(0, 220, 0));
  EXPECT_EQ(preview.at<cv::Vec3b>(54 + 240, 479), cv::Vec3b(0, 220, 0));
  const auto moved = depth_lidar::computeRoi(320, 200, 0.5, 0.10, 0.50);
  const auto updated = depth_lidar::makeStereoPreview(frame, frame, moved, 320, 200);
  EXPECT_EQ(updated.at<cv::Vec3b>(54 + 160, 320), cv::Vec3b(0, 220, 0));
  EXPECT_EQ(updated.at<cv::Vec3b>(54 + 220, 320), cv::Vec3b(80, 80, 80));
}

TEST(DepthLidarPreview, RejectsInvalidFramesAndRoi)
{
  const cv::Mat frame(400, 640, CV_8UC1, cv::Scalar(0));
  const depth_lidar::RoiRect roi{0, 220, 640, 40};
  EXPECT_THROW(depth_lidar::makeStereoPreview(cv::Mat{}, frame, roi, 640, 400), std::invalid_argument);
  EXPECT_THROW(depth_lidar::makeStereoPreview(frame, frame, roi, 320, 200), std::invalid_argument);
  const cv::Mat color(400, 640, CV_8UC3);
  EXPECT_THROW(depth_lidar::makeStereoPreview(color, frame, roi, 640, 400), std::invalid_argument);
}
