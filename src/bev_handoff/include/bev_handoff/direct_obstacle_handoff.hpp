#pragma once
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <std_msgs/msg/header.hpp>

namespace bev_handoff
{
// Raw LEFT IR input from the SAME OAK session as RGB/depth. Tilt is in CAM_A
// coordinates and evaluated at this frame's exposure midpoint.
struct DirectIrFrame
{
  cv::Mat mono;
  std::shared_ptr<const void> owner;
  std_msgs::msg::Header header;
  std::chrono::steady_clock::time_point captured_at;
  std::array<double, 4> intrinsics{};
  cv::Matx33d rgb_from_view{cv::Matx33d::eye()};
  cv::Vec3d view_center_rgb_m{0, 0, 0};
  cv::Matx33d current_rgb_to_reference{cv::Matx33d::eye()};
  bool tunnel_profile{false};
};
void publishDirectIr(std::shared_ptr<const DirectIrFrame> frame);
std::shared_ptr<const DirectIrFrame> latestDirectIr();
// Process-local, immutable depth buffer. owner keeps DepthAI storage alive.
// Millimetres, optical-axis Z, calibrated rectified camera coordinates.
struct DirectDepthFrame
{
  const std::uint8_t * data{nullptr};
  std::size_t size{0}, stride{0};
  int width{0}, height{0};
  std::shared_ptr<const void> owner;
  std_msgs::msg::Header header;
  std::chrono::steady_clock::time_point captured_at;
  std::array<double,4> intrinsics{}; // fx,fy,cx,cy
  std::array<double,9> rgb_from_depth_rotation{};
  std::array<double,3> rgb_from_depth_translation{};
};
struct ObstacleReference
{
  std::string device_id, frame_id, input_topic;
  cv::Matx33d rotation_vehicle_from_camera;
  cv::Vec3d position_vehicle_m;
  double fx{0},fy{0},cx{0},cy{0},settle_sec{0};
  int source_width{0},source_height{0};
  double x_min{0},x_max{3},y_min{-.6},y_max{.6},meter_per_pixel{.01};
  int width{120},height{300};
};
void publishObstacleReference(std::shared_ptr<const ObstacleReference> reference);
std::shared_ptr<const ObstacleReference> obstacleReference();
std::string obstacleDeviceId();
void publishDirectDepth(std::shared_ptr<const DirectDepthFrame> frame);
std::shared_ptr<const DirectDepthFrame> latestDirectDepth();
// Blocks until a publishDirectDepth() newer than `revision` (start at 0) or the
// timeout, then returns the latest frame (possibly null/unchanged) and updates
// `revision`. Lets consumers sleep between depth frames instead of polling.
std::shared_ptr<const DirectDepthFrame> waitDirectDepth(
  std::uint64_t & revision, std::chrono::steady_clock::duration timeout);
struct ObstacleCluster
{
  std::vector<cv::Point2f> surface_xy;
  cv::Point2f center;
  double nearest_range{0};
};
struct ObstacleFrame
{
  std_msgs::msg::Header header;
  std::chrono::steady_clock::time_point captured_at;
  double x_max{3.0}, y_max{0.6}, meter_per_pixel{0.01};
  int width{120}, height{300};
  double fps{0}, host_ms{0};
  std::vector<ObstacleCluster> clusters;
};
// Small bounded history for asynchronous RGB inference/depth completion.
void publishObstacles(std::shared_ptr<const ObstacleFrame> frame);
void clearObstacles();
std::shared_ptr<const ObstacleFrame> matchingObstacles(
  const std_msgs::msg::Header & image, double max_delta_sec, double max_age_sec);
}  // namespace bev_handoff
