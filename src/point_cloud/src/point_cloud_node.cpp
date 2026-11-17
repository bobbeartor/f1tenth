#include "point_cloud/depth_source.hpp"
#include "point_cloud/startup_parameters.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <visualization_msgs/msg/marker.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace point_cloud
{
// A single table keeps declaration, snapshots and parameter validation in sync.
#define CLOUD_PARAMETERS(X) \
  X("camera.resolution", resolution, as_string) \
  X("camera.fps", fps, as_double) \
  X("depth.mode", mode, as_string) \
  X("depth.ir_dot_projector_intensity", dot_intensity, as_double) \
  X("depth.ir_flood_light_intensity", flood_intensity, as_double) \
  X("depth.confidence_threshold", confidence, as_int) \
  X("depth.left_right_check", lr_check, as_bool) \
  X("depth.left_right_threshold", lr_threshold, as_int) \
  X("depth.subpixel", subpixel, as_bool) \
  X("depth.subpixel_fractional_bits", subpixel_bits, as_int) \
  X("depth.extended_disparity", extended, as_bool) \
  X("depth.median_filter", median, as_string) \
  X("depth.spatial_filter", spatial, as_bool) \
  X("depth.speckle_filter", speckle, as_bool) \
  X("depth.hole_filling", hole_filling, as_bool) \
  X("depth.adaptive_median_filter", adaptive_median, as_bool) \
  X("points.pixel_stride", projection.pixel_stride, as_int) \
  X("points.min_depth_m", projection.min_depth_m, as_double) \
  X("points.max_depth_m", projection.max_depth_m, as_double) \
  X("bev.enabled", bev.enabled, as_bool) \
  X("bev.x_min_m", bev.x_min_m, as_double) \
  X("bev.x_max_m", bev.x_max_m, as_double) \
  X("bev.y_min_m", bev.y_min_m, as_double) \
  X("bev.y_max_m", bev.y_max_m, as_double) \
  X("ground.enabled", ground.enabled, as_bool) \
  X("ground.distance_m", ground.distance_m, as_double) \
  X("ground.max_depth_m", ground.max_depth_m, as_double) \
  X("ground.min_height_m", ground.min_height_m, as_double) \
  X("ground.max_height_m", ground.max_height_m, as_double) \
  X("ground.max_tilt_deg", ground.max_tilt_deg, as_double) \
  X("ground.min_inlier_ratio", ground.min_inlier_ratio, as_double) \
  X("publish.depth_image", publish_depth, as_bool) \
  X("input.max_age_sec", max_age_sec, as_double) \
  X("metrics.print_interval_sec", metrics_interval, as_double)

void applyParameter(Config & c, const rclcpp::Parameter & p)
{
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER &&
    (p.as_int() < std::numeric_limits<int>::min() || p.as_int() > std::numeric_limits<int>::max()))
  {
    throw std::invalid_argument("Integer parameter is outside the supported range");
  }
#define APPLY(name, field, getter) if (p.get_name() == name) {c.field = p.getter(); return;}
  CLOUD_PARAMETERS(APPLY)
#undef APPLY
}

Config fromParameters(const std::vector<rclcpp::Parameter> & parameters)
{
  Config config;
  for (const auto & p : parameters) {applyParameter(config, p);}
  validate(config);
  return config;
}

bool hostIsBigEndian()
{
  const std::uint16_t value = 1;
  return *reinterpret_cast<const std::uint8_t *>(&value) == 0;
}

class PointCloudNode : public rclcpp::Node
{
public:
  PointCloudNode() : Node("point_cloud")
  {
#define DECLARE(name, field, getter) \
    declare_parameter(name, config_.field); parameter_names_.emplace_back(name);
    CLOUD_PARAMETERS(DECLARE)
#undef DECLARE
    rcl_interfaces::msg::ParameterDescriptor read_only;
    read_only.read_only = true;
    device_id_ = declare_parameter<std::string>("device_id", "", read_only);
    frame_id_ = declare_parameter<std::string>("frame_id", "point_cloud_optical_frame", read_only);
    view_frame_id_ = declare_parameter<std::string>("view_frame_id", "point_cloud_view", read_only);
    if (frame_id_.empty() || view_frame_id_.empty() || frame_id_ == view_frame_id_) {
      throw std::invalid_argument("frame_id and view_frame_id must be nonempty and different");
    }
    bev_frame_id_ = declare_parameter<std::string>("bev.frame_id", "front_axle_bev", read_only);
    if (bev_frame_id_.empty() || bev_frame_id_ == frame_id_ || bev_frame_id_ == view_frame_id_) {
      throw std::invalid_argument("bev.frame_id must be nonempty and distinct from camera frames");
    }
    startup_config_ = startupParameters(*this);
    startup_config_.device_id = device_id_;
    camera_x_ = declare_parameter<double>("bev.camera_x_m", -0.16, read_only);
    camera_y_ = declare_parameter<double>("bev.camera_y_m", 0.0, read_only);
    camera_yaw_ = declare_parameter<double>("bev.camera_yaw_deg", 0.0, read_only);
    if (!std::isfinite(camera_x_) || !std::isfinite(camera_y_) || !std::isfinite(camera_yaw_)) {
      throw std::invalid_argument("BEV camera position/yaw must be finite");
    }
    last_parameters_ = get_parameters(parameter_names_);
    config_ = fromParameters(last_parameters_);
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/points", qos);
    filtered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/points_filtered", qos);
    bev_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/points_bev", qos);
    boundary_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/bev_bounds", rclcpp::QoS(1).transient_local());
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("~/depth/image_raw", qos);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("~/depth/camera_info", qos);
    status_pub_ = create_publisher<std_msgs::msg::String>("~/status", rclcpp::QoS(1).transient_local());
    tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = view_frame_id_;
    tf.child_frame_id = frame_id_;
    // Optical (+X right, +Y down, +Z forward) -> view (+X forward, +Y left, +Z up).
    // Both origins are the rectified right camera; this is NOT a ground/mount transform.
    tf.transform.rotation.x = -0.5;
    tf.transform.rotation.y = 0.5;
    tf.transform.rotation.z = -0.5;
    tf.transform.rotation.w = 0.5;
    tf_broadcaster_->sendTransform(tf);
    parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        try {
          auto candidate = fromParameters(get_parameters(parameter_names_));
          for (const auto & p : parameters) {applyParameter(candidate, p);}
          validate(candidate);
          result.successful = true;
        } catch (const std::exception & e) {
          result.successful = false;
          result.reason = e.what();
        }
        return result;  // Validation has no side effects, even if another callback rejects it.
      });
    parameter_timer_ = create_wall_timer(100ms, [this]() {syncParameters();});
    worker_ = std::thread([this]() {cameraLoop();});
  }

  ~PointCloudNode() override
  {
    stop_.store(true);
    if (worker_.joinable()) {worker_.join();}
  }

private:
  bool stopping() const {return stop_.load() || !rclcpp::ok();}

  void syncParameters()
  {
    const auto parameters = get_parameters(parameter_names_);
    bool changed = false, reopen = false;
    for (std::size_t i = 0; i < parameters.size(); ++i) {
      if (parameters[i].to_parameter_msg() != last_parameters_[i].to_parameter_msg()) {
        changed = true;
        if (parameter_names_[i].compare(0, 7, "ground.") != 0 &&
          parameter_names_[i].compare(0, 4, "bev.") != 0) {reopen = true;}
      }
    }
    if (!changed) {return;}
    const auto next = fromParameters(parameters);
    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      config_ = next;
      if (reopen) {++revision_;}
    }
    last_parameters_ = parameters;
    RCLCPP_INFO(get_logger(), "%s", reopen ?
      "Parameters committed; reopening depth pipeline with the new settings" :
      "Ground/BEV settings applied without restarting the camera");
  }

  void status(const std::string & value)
  {
    std_msgs::msg::String message;
    message.data = value;
    status_pub_->publish(message);
  }

  sensor_msgs::msg::PointCloud2 cloudMessage(const Cloud & cloud, const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::PointCloud2 message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    message.width = cloud.width;
    message.height = cloud.height;
    message.is_bigendian = hostIsBigEndian();
    message.is_dense = cloud.valid_points == static_cast<std::size_t>(cloud.width) * cloud.height;
    message.point_step = 3 * sizeof(float);
    message.row_step = message.width * message.point_step;
    for (std::uint32_t i = 0; i < 3; ++i) {
      sensor_msgs::msg::PointField field;
      field.name = std::string(1, "xyz"[i]);
      field.offset = i * sizeof(float);
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field.count = 1;
      message.fields.push_back(field);
    }
    message.data.resize(cloud.xyz.size() * sizeof(float));
    if (!message.data.empty()) {std::memcpy(message.data.data(), cloud.xyz.data(), message.data.size());}
    return message;
  }

  void clearCloud()
  {
    const auto stamp = now();
    const auto empty = cloudMessage(Cloud{}, stamp);
    cloud_pub_->publish(empty);
    filtered_pub_->publish(empty);
    auto bev_empty = empty;
    bev_empty.header.frame_id = bev_frame_id_;
    bev_pub_->publish(bev_empty);
  }

  void publishImages(
    const dai::ImgFrame & frame, const Intrinsics & k, const rclcpp::Time & stamp, bool publish_depth)
  {
    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = stamp;
    info.header.frame_id = frame_id_;
    info.width = frame.getWidth();
    info.height = frame.getHeight();
    info.distortion_model = "plumb_bob";
    info.d.assign(5, 0.0);
    info.k = {k.fx, 0, k.cx, 0, k.fy, k.cy, 0, 0, 1};
    info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    info.p = {k.fx, 0, k.cx, 0, 0, k.fy, k.cy, 0, 0, 0, 1, 0};
    info_pub_->publish(info);
    if (!publish_depth) {return;}
    sensor_msgs::msg::Image image;
    image.header = info.header;
    image.width = info.width;
    image.height = info.height;
    image.encoding = "16UC1";
    image.is_bigendian = hostIsBigEndian();
    image.step = image.width * sizeof(std::uint16_t);
    image.data.resize(static_cast<std::size_t>(image.step) * image.height);
    const auto & bytes = frame.getData();
    const auto stride = frame.getStride() ? frame.getStride() : image.step;
    for (std::uint32_t v = 0; v < image.height; ++v) {
      std::memcpy(image.data.data() + static_cast<std::size_t>(v) * image.step,
        bytes.data() + static_cast<std::size_t>(v) * stride, image.step);
    }
    depth_pub_->publish(std::move(image));
  }

  void publishBevView(const RigidTransform & vehicle_from_optical,
    const BevOptions & options, const rclcpp::Time & stamp)
  {
    // Keep original optical/view topics connected to the new RViz fixed frame.
    const RigidTransform optical_from_view{{0,-1,0, 0,0,-1, 1,0,0}, {0,0,0}};
    const auto vehicle_from_view = compose(vehicle_from_optical, optical_from_view);
    const auto & r = vehicle_from_view.rotation;
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = bev_frame_id_;
    tf.child_frame_id = view_frame_id_;
    tf.transform.translation.x = vehicle_from_view.translation[0];
    tf.transform.translation.y = vehicle_from_view.translation[1];
    tf.transform.translation.z = vehicle_from_view.translation[2];
    tf2::Quaternion q;
    tf2::Matrix3x3(r[0],r[1],r[2],r[3],r[4],r[5],r[6],r[7],r[8]).getRotation(q);
    tf.transform.rotation.x = q.x(); tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z(); tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
    visualization_msgs::msg::Marker marker;
    marker.header = tf.header;
    marker.ns = "bev_footprint";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.color.r = 0.1f; marker.color.g = 1.0f; marker.color.b = 0.5f; marker.color.a = 1.0f;
    for (const auto & xy : std::array<std::array<double, 2>, 5>{{
      {options.x_min_m, options.y_min_m}, {options.x_max_m, options.y_min_m},
      {options.x_max_m, options.y_max_m}, {options.x_min_m, options.y_max_m},
      {options.x_min_m, options.y_min_m}}})
    {
      geometry_msgs::msg::Point point;
      point.x = xy[0]; point.y = xy[1]; point.z = 0;
      marker.points.push_back(point);
    }
    boundary_pub_->publish(marker);
  }

  void cameraLoop()
  {
    std::string selected_id = device_id_;
    RigidTransform vehicle_from_rgb;
    bool mount_ready = false;
    while (!stopping()) {
      Config c;
      std::uint64_t revision;
      {
        std::lock_guard<std::mutex> lock(config_mutex_);
        c = config_;
        revision = revision_.load();
      }
      try {
        clearCloud();
        if (!mount_ready) {
          status("MEASURING BEV MOUNT: keep stationary on flat ground");
          RCLCPP_INFO(get_logger(), "Measuring BEV mount with oak_startup; keep stationary on flat ground");
          const auto measurement = oak_startup::measureOakStartupExtrinsics(
            startup_config_, [this]() {return stopping();});
          selected_id = measurement.device_id;
          vehicle_from_rgb = vehicleFromRgb(camera_x_, camera_y_, measurement.height_m,
            measurement.roll_deg, measurement.pitch_down_deg, camera_yaw_);
          mount_ready = true;
          RCLCPP_INFO(get_logger(),
            "BEV mount: CAM_A height=%.4fm roll=%.3fdeg pitch_down=%.3fdeg; front axle X/Y=%.3f/%.3fm",
            measurement.height_m, measurement.roll_deg, measurement.pitch_down_deg, camera_x_, camera_y_);
        }
        if (stopping()) {break;}
        status("OPENING");
        DepthSource source(c, selected_id);
        selected_id = source.deviceId();
        RCLCPP_INFO(get_logger(), "Device=%s requested=%s FPS=%.1f dot=%.2f flood=%.2f stride=%d",
          selected_id.c_str(), c.resolution.c_str(), c.fps, c.dot_intensity, c.flood_intensity,
          c.projection.pixel_stride);
        status("WAITING FOR DEPTH");
        auto last_frame = std::chrono::steady_clock::now();
        auto report_start = last_frame;
        auto previous_capture = std::chrono::steady_clock::time_point::min();
        std::size_t frames = 0;
        double processing_ms = 0;
        bool stale = false, first = true;
        auto last_view_publish = std::chrono::steady_clock::time_point::min();
        while (!stopping() && revision_.load() == revision) {
          auto frame = source.tryGet();
          const auto host_now = std::chrono::steady_clock::now();
          if (std::chrono::duration<double>(host_now - last_frame).count() > c.max_age_sec && !stale) {
            clearCloud();
            status("STALE: no fresh depth");
            stale = true;
          }
          if (!frame) {std::this_thread::sleep_for(2ms); continue;}
          const auto capture = frame->getTimestamp();
          const double age = std::chrono::duration<double>(host_now - capture).count();
          if (age < -0.01 || age > c.max_age_sec || capture <= previous_capture) {continue;}
          auto stamp = now();
          const auto delay = rclcpp::Duration::from_seconds(std::max(0.0, age));
          if (stamp.nanoseconds() >= delay.nanoseconds()) {stamp = stamp - delay;}
          if (frame->getType() != dai::ImgFrame::Type::RAW16) {
            throw std::runtime_error("Expected RAW16 millimeter depth");
          }
          const auto & transform = frame->getTransformation();
          if (!transform.isValid()) {throw std::runtime_error("Missing rectified depth intrinsics");}
          const auto matrix = transform.getIntrinsicMatrix();
          const Intrinsics k{matrix[0][0], matrix[1][1], matrix[0][2], matrix[1][2]};
          const auto & bytes = frame->getData();
          const auto stride = frame->getStride() ? frame->getStride() : frame->getWidth() * 2;
          auto cloud = projectDepth(bytes.data(), bytes.size(), frame->getWidth(),
            frame->getHeight(), stride, k, c.projection);
          if (revision_.load() != revision || stopping()) {break;}
          cloud_pub_->publish(cloudMessage(cloud, stamp));
          const auto raw_valid = cloud.valid_points;
          GroundOptions ground;
          BevOptions bev;
          {
            std::lock_guard<std::mutex> lock(config_mutex_);
            ground = config_.ground;
            bev = config_.bev;
          }
          const auto ground_result = removeGround(cloud, ground);
          if (ground.enabled && !ground_result.detected) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
              "Ground plane not found; points_filtered contains the original points");
          }
          filtered_pub_->publish(cloudMessage(cloud, stamp));
          const auto ground_points = cloud.valid_points;
          const auto vehicle_from_optical = compose(vehicle_from_rgb, source.rgbFromFrame(*frame));
          transformAndCrop(cloud, vehicle_from_optical, bev);
          const auto bev_points = cloud.valid_points;
          if (first || last_view_publish == std::chrono::steady_clock::time_point::min() ||
            std::chrono::duration<double>(host_now - last_view_publish).count() >= 0.2)
          {
            publishBevView(vehicle_from_optical, bev, stamp);
            last_view_publish = host_now;
          }
          auto bev_message = cloudMessage(cloud, stamp);
          bev_message.header.frame_id = bev_frame_id_;
          bev_pub_->publish(bev_message);
          publishImages(*frame, k, stamp, c.publish_depth);
          if (first || stale) {
            status("STREAMING");
            if (first) {
              RCLCPP_INFO(get_logger(), "Actual depth=%ux%u cloud=%ux%u fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
                frame->getWidth(), frame->getHeight(), cloud.width, cloud.height, k.fx, k.fy, k.cx, k.cy);
            }
            first = false;
            stale = false;
          }
          previous_capture = capture;
          last_frame = host_now;
          ++frames;
          processing_ms += std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - host_now).count();
          const double elapsed = std::chrono::duration<double>(host_now - report_start).count();
          if (elapsed >= c.metrics_interval) {
            const std::size_t total = static_cast<std::size_t>(cloud.width) * cloud.height;
            RCLCPP_INFO(get_logger(),
              "depth=%ux%u cloud=%ux%u FPS=%.1f valid=%zu/%zu (%.1f%%) age=%.1fms host=%.2fms XYZ=%.1fMB/s "
              "ground_points=%zu bev_points=%zu",
              frame->getWidth(), frame->getHeight(), cloud.width, cloud.height, frames / elapsed,
              raw_valid, total, 100.0 * raw_valid / total, age * 1000,
              processing_ms / frames, cloud.xyz.size() * sizeof(float) * frames / elapsed / 1e6,
              ground_points, bev_points);
            frames = 0;
            processing_ms = 0;
            report_start = host_now;
          }
        }
      } catch (const std::exception & e) {
        if (!stopping()) {
          clearCloud();
          status(std::string("ERROR: ") + e.what());
          RCLCPP_ERROR(get_logger(), "%s", e.what());
          for (int i = 0; i < 20 && !stopping() && revision_.load() == revision; ++i) {
            std::this_thread::sleep_for(100ms);
          }
        }
      }
    }
  }

  Config config_;
  oak_startup::OakStartupMeasurementConfig startup_config_;
  double camera_x_{-0.16}, camera_y_{0.0}, camera_yaw_{0.0};
  std::string bev_frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr bev_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr boundary_pub_;
  std::string device_id_, frame_id_, view_frame_id_;
  std::vector<std::string> parameter_names_;
  std::vector<rclcpp::Parameter> last_parameters_;
  std::mutex config_mutex_;
  std::atomic_bool stop_{false};
  std::atomic<std::uint64_t> revision_{0};
  std::thread worker_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_, filtered_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rclcpp::TimerBase::SharedPtr parameter_timer_;
};
#undef CLOUD_PARAMETERS
}  // namespace point_cloud

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {rclcpp::spin(std::make_shared<point_cloud::PointCloudNode>());}
  catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("point_cloud"), "%s", e.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
