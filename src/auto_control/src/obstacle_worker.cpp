#include "auto_control/obstacle_worker.hpp"
#include "bev_handoff/direct_obstacle_handoff.hpp"
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <chrono>
#include <optional>

namespace auto_control
{
namespace
{
double stampSeconds(const std_msgs::msg::Header & h)
{return double(h.stamp.sec)+double(h.stamp.nanosec)*1e-9;}
template<class T> void parameter(rclcpp::Node & node,const std::string & name,T & value)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only=true;
  descriptor.description="Startup setting: edit YAML and restart the auto_drive launch";
  value=node.declare_parameter<T>("obstacles."+name,value,descriptor);
}
cv::Matx33d matrix(const std::array<double,9> & values)
{return cv::Matx33d(values.data());}
}
ObstacleWorker::ObstacleWorker(rclcpp::Node & node,const RectifiedCameraModel & camera,
  const BevConfig & bev,const std::string & frame_id)
: node_(node),camera_(camera),bev_(bev),frame_id_(frame_id)
{
  parameter(node,"roi.x",scan_.roi_x); parameter(node,"roi.y",scan_.roi_y);
  parameter(node,"roi.width",scan_.roi_width); parameter(node,"roi.height",scan_.roi_height);
  parameter(node,"points.pixel_stride",scan_.pixel_stride);
  parameter(node,"points.min_depth_m",scan_.min_depth); parameter(node,"points.max_depth_m",scan_.max_depth);
  parameter(node,"ground.enabled",scan_.ground_enabled);
  parameter(node,"ground.distance_m",scan_.ground_distance);
  parameter(node,"ground.distance_per_meter",scan_.ground_distance_per_meter);
  parameter(node,"ground.max_distance_m",scan_.ground_max_distance);
  parameter(node,"height.max_m",scan_.max_height);
  parameter(node,"range.min_m",scan_.min_range); parameter(node,"range.max_m",scan_.max_range);
  parameter(node,"scan.angle_min_deg",scan_.angle_min); parameter(node,"scan.angle_max_deg",scan_.angle_max);
  parameter(node,"scan.bins",scan_.bins); parameter(node,"scan.min_samples",scan_.min_samples);
  parameter(node,"scan.support_distance_m",scan_.support_distance);
  parameter(node,"cluster.tolerance_m",cluster_.tolerance_m);
  parameter(node,"cluster.min_bins",cluster_.min_bins);
  parameter(node,"cluster.max_gap_bins",cluster_.max_gap_bins);
  parameter(node,"cluster.min_height_m",cluster_.min_height_m);
  parameter(node,"max_age_sec",max_age_); parameter(node,"max_sync_sec",max_sync_);
  obstacle::validate(scan_); obstacle::validateClusters(cluster_);
  if (cluster_.min_height_m>scan_.max_height || !std::isfinite(max_age_) || max_age_<.02 || max_age_>.5 ||
    !std::isfinite(max_sync_) || max_sync_<.001 || max_sync_>.1) {
    throw std::invalid_argument("Obstacle height/age/sync settings invalid");
  }
  publisher_=node.create_publisher<geometry_msgs::msg::PoseArray>("/auto/obstacles",rclcpp::SensorDataQoS().keep_last(1));
  bev_handoff::clearObstacles();
  thread_=std::thread(&ObstacleWorker::run,this);
}
ObstacleWorker::~ObstacleWorker()
{
  stop_=true;
  if (thread_.joinable()) {thread_.join();}
  bev_handoff::clearObstacles();
}
void ObstacleWorker::observe(const camera_driver::msg::BevInput & input)
{
  if (!input.depth_geometry_valid) {return;}
  const auto k=matrix(input.source_intrinsics);
  const auto h=matrix(input.source_to_stabilized_homography);
  const auto rgb_from_source=matrix(input.rgb_from_source_rotation);
  const cv::Vec3d rgb_translation(input.rgb_from_source_translation.data());
  const cv::Matx33d virtual_k(camera_.fx,0,camera_.cx,0,camera_.fy,camera_.cy,0,0,1);
  // Exact camera-ray mapping used by the BEV LUT: K_virtual^-1 * H * K_source.
  // H includes both fixed zoom and the accepted (or held) per-RGB-frame correction.
  const auto ray_map=virtual_k.inv()*h*k;
  const auto rotation=camera_.rotation_vehicle_from_camera*ray_map*rgb_from_source.t();
  const auto translation=camera_.position_vehicle_m-rotation*rgb_translation;
  const double determinant=cv::determinant(cv::Mat(rotation));
  if (!std::isfinite(determinant) || determinant<=1e-8) {return;}
  Geometry g; g.stamp=stampSeconds(input.header);
  for (int i=0;i<9;++i) {
    if (!std::isfinite(rotation.val[i])) {return;}
    g.vehicle_from_rgb.rotation[i]=rotation.val[i];
  }
  for (int i=0;i<3;++i) {
    if (!std::isfinite(translation[i])) {return;}
    g.vehicle_from_rgb.translation[i]=translation[i];
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!geometry_.empty() && g.stamp<=geometry_.back().stamp) {geometry_.clear();}
  geometry_.push_back(g);
  while (geometry_.size()>32) {geometry_.pop_front();}
}
void ObstacleWorker::run()
{
  monitor_=std::make_unique<bev_handoff::RuntimeMonitor>(node_);
  using Clock=std::chrono::steady_clock;
  obstacle::Projector projector;
  auto last_capture=Clock::time_point{};
  auto report=Clock::now();
  std::size_t frames=0, rejected_sync=0;
  double fps=0,host_ms=0,total_ms=0;
  bool cleared=false;
  auto last_result_at=Clock::now();
  const auto clear=[&]() {
    if (cleared) {return;}
    bev_handoff::clearObstacles();
    geometry_msgs::msg::PoseArray empty; empty.header.stamp=node_.now(); empty.header.frame_id=frame_id_;
    publisher_->publish(empty); cleared=true;
  };
  std::uint64_t depth_revision=0;
  bool retry_sync=false;
  while (!stop_ && rclcpp::ok()) {
    try {
      // Sleep until the camera publishes the next depth frame. The timeout only
      // bounds stale-result clearing and shutdown; it is not a polling period.
      std::shared_ptr<const bev_handoff::DirectDepthFrame> depth;
      if (retry_sync) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        depth=bev_handoff::latestDirectDepth();
        retry_sync=false;
      } else {
        depth=bev_handoff::waitDirectDepth(depth_revision,std::chrono::milliseconds(20));
      }
      const auto now=Clock::now();
      const double age=depth ? std::chrono::duration<double>(now-depth->captured_at).count() : max_age_+1;
      if (!depth || age<0 || age>max_age_) {
        clear();
        continue;
      }
      if (std::chrono::duration<double>(now-last_result_at).count()>max_age_) {clear();}
      if (depth->captured_at<=last_capture) {continue;}
      std::optional<Geometry> match;
      double delta=max_sync_;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto & geometry:geometry_) {
          const double d=std::abs(geometry.stamp-stampSeconds(depth->header));
          if (d<=delta) {delta=d; match=geometry;}
        }
      }
      // Retry the latest depth while waiting for nearby RGB geometry; no queue growth.
      if (!match) {++rejected_sync; retry_sync=true; continue;}
      last_capture=depth->captured_at;
      bev_handoff::RuntimeSample runtime_sample(monitor_.get(),"obstacle_detection");
      const auto started=Clock::now();
      point_cloud::RigidTransform rgb_from_depth;
      rgb_from_depth.rotation=depth->rgb_from_depth_rotation;
      rgb_from_depth.translation=depth->rgb_from_depth_translation;
      const auto transform=point_cloud::compose(match->vehicle_from_rgb,rgb_from_depth);
      const auto & k=depth->intrinsics;
      projector.configure(depth->width,depth->height,{k[0],k[1],k[2],k[3]},transform,scan_);
      const auto scan=projector.project(depth->data,depth->size,depth->stride);
      const auto clusters=obstacle::clusterScan(scan,scan_,cluster_);
      auto result=std::make_shared<bev_handoff::ObstacleFrame>();
      result->header=depth->header; result->header.frame_id=frame_id_;
      result->captured_at=depth->captured_at;
      result->x_max=bev_.x_max_m; result->y_max=bev_.y_max_m;
      result->meter_per_pixel=bev_.meter_per_pixel; result->width=bev_.output_width; result->height=bev_.output_height;
      geometry_msgs::msg::PoseArray poses; poses.header=result->header;
      for (const auto & object:clusters.objects) {
        bev_handoff::ObstacleCluster item;
        item.center={static_cast<float>(object.x),static_cast<float>(object.y)};
        item.nearest_range=object.nearest_range;
        for (const auto bin:object.bins) {
          const double a=(scan_.angle_min+bin*(scan_.angle_max-scan_.angle_min)/(scan_.bins-1))*obstacle::radians;
          item.surface_xy.emplace_back(scan.ranges[bin]*std::cos(a),scan.ranges[bin]*std::sin(a));
        }
        result->clusters.push_back(std::move(item));
        geometry_msgs::msg::Pose pose; pose.position.x=object.x; pose.position.y=object.y; pose.orientation.w=1;
        poses.poses.push_back(pose);
      }
      ++frames; total_ms+=std::chrono::duration<double,std::milli>(Clock::now()-started).count();
      const double elapsed=std::chrono::duration<double>(Clock::now()-report).count();
      if (elapsed>=1) {
        fps=frames/elapsed; host_ms=total_ms/frames;
        RCLCPP_INFO(node_.get_logger(),"OBSTACLES depth=%.1f FPS host=%.2fms age=%.1fms RGB-delta=%.1fms clusters=%zu ground_removed=%zu low_bins=%zu small_bins=%zu sync_wait_polls=%zu",
          fps,host_ms,age*1000,delta*1000,clusters.objects.size(),scan.ground_removed,clusters.height_rejected_bins,clusters.small_rejected_bins,rejected_sync);
        report=Clock::now(); frames=0; total_ms=0; rejected_sync=0;
      }
      result->fps=fps; result->host_ms=host_ms;
      if (std::chrono::duration<double>(Clock::now()-depth->captured_at).count()>max_age_) {continue;}
      publisher_->publish(poses); bev_handoff::publishObstacles(std::move(result)); cleared=false; last_result_at=Clock::now();
    } catch (const std::exception & e) {
      clear();
      RCLCPP_ERROR_THROTTLE(node_.get_logger(),*node_.get_clock(),1000,"Obstacle detection: %s",e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}
}
