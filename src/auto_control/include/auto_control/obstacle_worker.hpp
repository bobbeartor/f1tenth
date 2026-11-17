#include <bev_handoff/runtime_monitor.hpp>
#pragma once
#include "bev_processor/bev_geometry.hpp"
#include "auto_control/obstacle_clusters.hpp"
#include "camera_driver/msg/bev_input.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <atomic>
#include <deque>
#include <mutex>
#include <thread>

namespace auto_control
{
using bev_processor::RectifiedCameraModel;
using bev_processor::BevConfig;
// Independent processing thread; never opens a camera or waits on BEV inference.
class ObstacleWorker
{
public:
  ObstacleWorker(rclcpp::Node & node,const RectifiedCameraModel & camera,
    const BevConfig & bev,const std::string & frame_id);
  ~ObstacleWorker();
  void observe(const camera_driver::msg::BevInput & input);
private:
  struct Geometry
  {
    double stamp{0};
    point_cloud::RigidTransform vehicle_from_rgb;
  };
  void run();
  rclcpp::Node & node_;
  std::unique_ptr<bev_handoff::RuntimeMonitor> monitor_;
  RectifiedCameraModel camera_;
  BevConfig bev_;
  std::string frame_id_;
  obstacle::Options scan_;
  obstacle::ClusterOptions cluster_;
  double max_age_{0.25}, max_sync_{0.04};
  std::mutex mutex_;
  std::deque<Geometry> geometry_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
};
}
