#include "depth_lidar/depth_lidar_geometry.hpp"
#include <gtest/gtest.h>

TEST(DepthLidarGeometry, ComputesCenteredRoi)
{
  const auto roi = depth_lidar::computeRoi(8, 4, 0.5, 0.25, 0.25);
  EXPECT_EQ(roi.x, 2);
  EXPECT_EQ(roi.y, 2);
  EXPECT_EQ(roi.width, 4);
  EXPECT_EQ(roi.height, 1);
}
