#pragma once
#include "bev_handoff/direct_obstacle_handoff.hpp"

namespace bev_handoff
{
// Process-local display channel. Control receives a separate ROS path with an explicit planning mode.
struct PlanningLane
{
  std_msgs::msg::Header header;
  std::chrono::steady_clock::time_point received_at;
  std::vector<cv::Point2d> center;
  std::vector<std::vector<cv::Point2d>> boundaries;
  bool valid{false};
  double x_max{3}, y_max{.6}, meter_per_pixel{.01};
  double lane_width_m{.65}; // Same configured width used to generate the centerline.
  int width{120}, height{300};
};
struct SafetyBox {double x0, x1, y0, y1;};
struct AvoidanceCandidate
{
  std::vector<cv::Point2d> path;
  bool valid{false};
  double transition_m{0}, max_offset_m{0}, max_curvature{0};
  std::string reason;
};
struct AvoidancePreview
{
  std_msgs::msg::Header header;
  std::chrono::steady_clock::time_point lane_received_at, depth_captured_at;
  bool control_requested{false};
  bool follow_centerline{false};
  bool inferred_boundaries{false};
  double horizon_m{0};
  std::size_t obstacle_count{0}, region_count{0};
  double x_max{3}, y_max{.6}, meter_per_pixel{.01};
  int width{120}, height{300};
  std::vector<cv::Point2d> original, selected;
  std::vector<SafetyBox> assumed_boxes; // Fixed object footprints, centered on detected representative points.
  std::vector<SafetyBox> boxes; // Expanded clearance envelopes, not measured object dimensions.
  std::vector<AvoidanceCandidate> candidates;
  std::string status{"WAIT"};
  double recommended_speed{0}, max_curvature{0};
  double display_delta_sec{.16};
};
bool avoidancePreviewEnabled();
bool avoidanceControlRequested();
bool avoidanceDeformationOnly();
void setAvoidanceDeformationOnly(bool enabled);
void setAvoidanceControlRequested(bool enabled);
void setAvoidancePreviewEnabled(bool enabled);
void publishPlanningLane(std::shared_ptr<const PlanningLane> lane);
std::shared_ptr<const PlanningLane> latestPlanningLane();
void publishAvoidancePreview(std::shared_ptr<const AvoidancePreview> plan);
std::shared_ptr<const AvoidancePreview> latestAvoidancePreview();
}  // namespace bev_handoff
