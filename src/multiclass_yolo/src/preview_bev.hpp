#pragma once

#include "multiclass_yolo/object_localization.hpp"
#include "multiclass_yolo/yolox_detector.hpp"
#include "preview_colors.hpp"

#include <point_cloud/startup_parameters.hpp>
#include <roi_lidar/config.hpp>
#include <roi_lidar/preview.hpp>

#include <opencv2/imgproc.hpp>

#include <chrono>

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace multiclass_yolo {

struct BevSettings {
  roi_lidar::Config roi;
  oak_startup::OakStartupMeasurementConfig startup;
  double camera_x{-0.09}, camera_y{0}, camera_yaw{0};
  double sync_ms{20};
  bool estimate_ground{true};
  oak_startup::OakStartupMeasurement fixed_pose;
  // Top-view class sizes. Sedan = own vehicle; truck = own vehicle scaled; cone is square.
  double vehicle_length{.50}, vehicle_width{.30}, truck_scale{1.2}, cone_size{.13};
  // Centre share of each box whose points are preferred; box edges carry background.
  double core_fraction{.60};
  LocalizationOptions localization;

  Footprint footprint(int class_id) const {
    switch (class_id) {
      case 0: return {cone_size, cone_size};
      case 1: return {vehicle_length * truck_scale, vehicle_width * truck_scale};
      case 2: return {vehicle_length, vehicle_width};
      default: return {};  // traffic light: position only
    }
  }

  explicit BevSettings(rclcpp::Node & node) {
    roi.camera.fps = 40;
    roi.scan.roi_x = .01;
    roi.scan.roi_y = .43;
    roi.scan.roi_width = .98;
    roi.scan.roi_height = .55;
    roi.scan.max_depth = 10;
    roi.scan.angle_min = -60;
    roi.scan.angle_max = 60;
    roi.scan.outlier.enabled = true;
    roi.scan.outlier.radius_m = .08;
    roi.scan.outlier.min_neighbors = 8;
    roi.terrain.cell_m = .025;
    roi.terrain.min_obstacle_samples = 8;
    roi.preview_size = 800;
#define MULTICLASS_DECLARE_ROI(name, member, converter) \
    roi.member = node.declare_parameter<std::decay_t<decltype(roi.member)>>(name, roi.member);
    ROI_PARAMETERS(MULTICLASS_DECLARE_ROI)
#undef MULTICLASS_DECLARE_ROI
    startup = point_cloud::startupParameters(node);
    startup.roi_preview_enabled = false;
    camera_x = node.declare_parameter<double>("bev.camera_x_m", camera_x);
    camera_y = node.declare_parameter<double>("bev.camera_y_m", camera_y);
    camera_yaw = node.declare_parameter<double>("bev.camera_yaw_deg", camera_yaw);
    sync_ms = node.declare_parameter<double>("fusion_sync_ms", sync_ms);
    estimate_ground = node.declare_parameter<bool>("ground_estimation_enabled", true);
    fixed_pose.height_m = node.declare_parameter<double>("fixed_camera_height_m", .20);
    fixed_pose.roll_deg = node.declare_parameter<double>("fixed_camera_roll_deg", 0.0);
    fixed_pose.pitch_down_deg = node.declare_parameter<double>("fixed_camera_pitch_down_deg", 14.0);
    vehicle_from_camera(fixed_pose);
    vehicle_length = node.declare_parameter<double>("object.vehicle_length_m", vehicle_length);
    vehicle_width = node.declare_parameter<double>("object.vehicle_width_m", vehicle_width);
    truck_scale = node.declare_parameter<double>("object.truck_scale", truck_scale);
    cone_size = node.declare_parameter<double>("object.cone_size_m", cone_size);
    core_fraction = node.declare_parameter<double>("object.box_core_fraction", core_fraction);
    for (const double v : {vehicle_length, vehicle_width, truck_scale, cone_size})
      if (!std::isfinite(v) || v <= 0 || v > 5) throw std::invalid_argument("object sizes must be in (0, 5]");
    if (!std::isfinite(core_fraction) || core_fraction < .2 || core_fraction > 1)
      throw std::invalid_argument("object.box_core_fraction must be in [0.2, 1]");
    roi_lidar::validate(roi);
    if (!std::isfinite(sync_ms) || sync_ms < 1 || sync_ms > 100)
      throw std::invalid_argument("fusion_sync_ms must be in [1, 100]");
  }

  point_cloud::RigidTransform vehicle_from_camera(const oak_startup::OakStartupMeasurement & pose) const {
    return point_cloud::vehicleFromRgb(camera_x, camera_y, pose.height_m,
      pose.roll_deg, pose.pitch_down_deg, camera_yaw);
  }
};

enum class SettingGroup { stereo, ground, object, fit };

struct BevControl {
  const char * label;
  SettingGroup group;
  bool integer;
  double min, max;
  double (*read)(const BevSettings &);
  void (*write)(BevSettings &, double);
};

inline const std::vector<BevControl> & bev_controls() {
#define BEV_CONTROL(label, group, field, lo, hi) \
  {label, SettingGroup::group, !std::is_floating_point_v<decltype(std::declval<BevSettings &>().field)>, lo, hi, \
    [](const BevSettings & s) {return static_cast<double>(s.field);}, \
    [](BevSettings & s, double v) {s.field = static_cast<std::decay_t<decltype(s.field)>>(v);}}
  static const std::vector<BevControl> controls{
    BEV_CONTROL("Depth FPS", stereo, roi.camera.fps, 1, 120),
    {"Depth height px", SettingGroup::stereo, true, 400, 800,
      [](const BevSettings & s) {return static_cast<double>(point_cloud::resolutionSize(s.roi.camera.resolution).second);},
      [](BevSettings & s, double v) {s.roi.camera.resolution = std::to_string(static_cast<int>(v)) + "p";}},
    {"Preset 0/1/2", SettingGroup::stereo, true, 0, 2,
      [](const BevSettings & s) {return s.roi.camera.mode == "default" ? 0.0 : s.roi.camera.mode == "high_density" ? 1.0 : 2.0;},
      [](BevSettings & s, double v) {s.roi.camera.mode = v == 0 ? "default" : v == 1 ? "high_density" : "high_accuracy";}},
    BEV_CONTROL("Confidence", stereo, roi.camera.confidence, 0, 255),
    BEV_CONTROL("LR check 0/1", stereo, roi.camera.lr_check, 0, 1),
    BEV_CONTROL("LR threshold", stereo, roi.camera.lr_threshold, 0, 128),
    BEV_CONTROL("Subpixel 0/1", stereo, roi.camera.subpixel, 0, 1),
    BEV_CONTROL("Subpixel bits", stereo, roi.camera.subpixel_bits, 3, 5),
    BEV_CONTROL("Extended 0/1", stereo, roi.camera.extended, 0, 1),
    {"Median 0/3/5/7", SettingGroup::stereo, true, 0, 7,
      [](const BevSettings & s) {return s.roi.camera.median == "off" ? 0.0 : static_cast<double>(s.roi.camera.median.front() - '0');},
      [](BevSettings & s, double v) {
        const auto k = std::to_string(static_cast<int>(v));
        s.roi.camera.median = v == 0 ? "off" : k + "x" + k;
      }},
    BEV_CONTROL("IR dot", stereo, roi.camera.dot_intensity, 0, 1),
    BEV_CONTROL("IR flood", stereo, roi.camera.flood_intensity, 0, 1),
    BEV_CONTROL("Spatial 0/1", stereo, roi.camera.spatial, 0, 1),
    BEV_CONTROL("Speckle 0/1", stereo, roi.camera.speckle, 0, 1),
    BEV_CONTROL("Hole filling 0/1", stereo, roi.camera.hole_filling, 0, 1),
    BEV_CONTROL("Sync tolerance ms", ground, sync_ms, 1, 100),
    BEV_CONTROL("Min depth m", ground, roi.scan.min_depth, .01, 65.535),
    BEV_CONTROL("Max depth m", ground, roi.scan.max_depth, .01, 65.535),
    BEV_CONTROL("Min height m", ground, roi.scan.obstacle_min_height, 0, 10),
    BEV_CONTROL("Max height m", ground, roi.scan.max_height, .01, 10),
    BEV_CONTROL("Ground filter 0/1", ground, roi.scan.ground_enabled, 0, 1),
    BEV_CONTROL("Ground margin m", ground, roi.scan.ground_distance, 0, .3),
    BEV_CONTROL("Margin per m", ground, roi.scan.ground_distance_per_meter, 0, .1),
    BEV_CONTROL("Max margin m", ground, roi.scan.ground_max_distance, 0, .5),
    BEV_CONTROL("Outlier filter 0/1", ground, roi.scan.outlier.enabled, 0, 1),
    BEV_CONTROL("Outlier radius m", ground, roi.scan.outlier.radius_m, .01, 1),
    BEV_CONTROL("Outlier neighbors", ground, roi.scan.outlier.min_neighbors, 1, 100),
    BEV_CONTROL("Vehicle length m", object, vehicle_length, .05, 5),
    BEV_CONTROL("Vehicle width m", object, vehicle_width, .05, 5),
    BEV_CONTROL("Truck scale", object, truck_scale, .5, 3),
    BEV_CONTROL("Cone size m", object, cone_size, .02, 1),
    BEV_CONTROL("Box core share", object, core_fraction, .2, 1),
    BEV_CONTROL("Range window m", object, localization.window_base_m, .01, 1),
    BEV_CONTROL("Window per m", object, localization.window_per_m, 0, .5),
    BEV_CONTROL("Min object points", object, localization.min_points, 1, 1000),
    BEV_CONTROL("Min point share", object, localization.min_fraction, 0, 1),
    BEV_CONTROL("Near percentile", object, localization.near_percentile, 0, 1),
    BEV_CONTROL("Fit ROI width px", fit, startup.roi_width, 1, 1280),
    BEV_CONTROL("Fit ROI height px", fit, startup.roi_height, 1, 800),
    BEV_CONTROL("Fit ROI Y offset", fit, startup.roi_vertical_offset_px, -800, 800),
    BEV_CONTROL("Fit sample stride", fit, startup.point_sample_step, 1, 16),
    BEV_CONTROL("Fit min depth m", fit, startup.minimum_depth_m, .01, 10),
    BEV_CONTROL("Fit max depth m", fit, startup.maximum_depth_m, .01, 20),
    BEV_CONTROL("Fit min height m", fit, startup.minimum_height_m, .01, 2),
    BEV_CONTROL("Fit max height m", fit, startup.maximum_height_m, .01, 2),
    BEV_CONTROL("Fit min points", fit, startup.minimum_valid_points, 3, 1024000),
    BEV_CONTROL("Fit inlier band m", fit, startup.plane_inlier_threshold_m, .001, .1),
    BEV_CONTROL("Fit min inliers", fit, startup.plane_minimum_inliers, 3, 1024000),
    BEV_CONTROL("Fit inlier ratio", fit, startup.plane_minimum_inlier_ratio, .01, 1),
    BEV_CONTROL("Fit residual MAD m", fit, startup.plane_maximum_residual_mad_m, .001, .1),
    BEV_CONTROL("Fit IMU delta deg", fit, startup.plane_maximum_imu_difference_deg, .1, 45),
    BEV_CONTROL("Fit iterations", fit, startup.plane_ransac_iterations, 1, 10000),
    BEV_CONTROL("Fit stable frames", fit, startup.stable_plane_frame_count, 1, 1000),
    BEV_CONTROL("Fit timeout s", fit, startup.timeout_sec, 1, 120)
  };
#undef BEV_CONTROL
  return controls;
}

struct BevFrame {
  cv::Mat image;
  std::vector<std::pair<int, int>> points;
  std::vector<ObjectEstimate> objects;  // one per detection, in detection order
  std::optional<double> sync_ms;
  std::optional<std::chrono::steady_clock::time_point> filter_completed;
  bool has_position{false};
};

class BevFusion {
 public:
  void reset() {rgb_key_.clear(); depth_key_.clear();}

  template<class MakeDisplay>
  BevFrame process(dai::ImgFrame & rgb, dai::ImgFrame * depth, const dai::CalibrationHandler & calibration,
    const oak_startup::OakStartupMeasurement & pose, const BevSettings & settings,
    const std::vector<Detection> & detections, bool render, bool camera_background, const MakeDisplay & make_display)
  {
    const auto rgb_k = intrinsics(rgb);
    const auto cam_a_from_rgb = point_cloud::rgbFromFrame(rgb, calibration);
    const auto vehicle_from_cam_a = settings.vehicle_from_camera(pose);
    const auto vehicle_from_rgb = point_cloud::compose(vehicle_from_cam_a, cam_a_from_rgb);
    const auto & config = settings.roi;
    BevFrame out;
    roi_lidar::Scan scan;
    if (depth) {
      out.points.resize(detections.size());
      out.sync_ms = std::abs(std::chrono::duration<double, std::milli>(
        rgb.getTimestamp(dai::CameraExposureOffset::MIDDLE) -
        depth->getTimestamp(dai::CameraExposureOffset::MIDDLE)).count());
      if (!detections.empty()) {
        scan = associate(rgb, *depth, calibration, cam_a_from_rgb, vehicle_from_cam_a,
          rgb_k, config, settings.estimate_ground, settings.core_fraction, detections, out.points);
        out.objects = estimate(scan, detections, rgb_k, vehicle_from_rgb, settings);
      }
    }
    out.filter_completed = std::chrono::steady_clock::now();
    out.has_position = std::any_of(out.objects.begin(), out.objects.end(),
      [](const ObjectEstimate & object) {return object.valid;});
    if (!render) return out;
    cv::Mat background;
    if (camera_background) {
      const auto key = geometry_key(rgb, rgb_k, vehicle_from_rgb);
      if (key != rgb_key_) {
        background_.configure(rgb.getWidth(), rgb.getHeight(), rgb_k, vehicle_from_rgb,
          config.view, config.preview_size);
        rgb_key_ = key;
      }
      background = background_.render(make_display(), config.interpolation);
    }
    const auto status = out.sync_ms ? cv::format("RGB-depth %.1f ms | %zu object points", *out.sync_ms,
      scan.obstacle_indices.size()) : std::string("Waiting for synchronized stereo depth");
    const roi_lidar::Clusters no_clusters;
    out.image = roi_lidar::mapPreview({}, config.scan, config.view, config.preview_size,
      status, background, &no_clusters);
    const auto layout = roi_lidar::mapLayout(config.view, config.preview_size);
    cv::Mat area = out.image(layout.area);
    const auto pixel = [&layout](double x, double y) {
      return cv::Point(static_cast<int>(std::lround(layout.center_x - y * layout.scale)),
        static_cast<int>(std::lround(layout.origin_y - x * layout.scale))) - layout.area.tl();
    };
    const double origin_x = vehicle_from_rgb.translation[0], origin_y = vehicle_from_rgb.translation[1];
    for (std::size_t i = 0; i < scan.obstacle_indices.size(); ++i) {
      const auto & p = scan.surface_points[scan.obstacle_indices[i]];
      if (!roi_lidar::insideView(p[0], p[1], config.view)) continue;
      const auto id = static_cast<std::size_t>(scan.obstacle_selection_ids[i] >> 1);
      // Points outside the selected range peak are background inside the box: grey.
      const auto & object = out.objects[id];
      const double range = std::hypot(p[0] - origin_x, p[1] - origin_y);
      const bool selected = object.valid && range >= object.cluster_min && range <= object.cluster_max;
      cv::circle(area, pixel(p[0], p[1]), std::max(3, config.point_radius),
        selected ? obstacle_colors[detections[id].class_id] : cv::Scalar(150, 150, 150), cv::FILLED, cv::LINE_8);
    }
    for (std::size_t id = 0; id < out.objects.size(); ++id) {
      const auto & object = out.objects[id];
      if (!object.valid) continue;
      const auto & color = obstacle_colors[detections[id].class_id];
      if (object.footprint.length > 0) {
        std::vector<cv::Point> outline;
        for (const auto & corner : object.corners) outline.push_back(pixel(corner.x, corner.y));
        cv::polylines(area, outline, true, color, 2, cv::LINE_AA);
      }
      // Measured front face: white ring so it reads on any class colour.
      cv::circle(area, pixel(object.front_x, object.front_y), 5, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    }
    return out;
  }

 private:
  static point_cloud::Intrinsics intrinsics(dai::ImgFrame & frame) {
    const auto & geometry = frame.getTransformation();
    if (!geometry.isValid()) throw std::runtime_error("Missing calibrated camera frame");
    const auto k = geometry.getIntrinsicMatrix();
    point_cloud::Intrinsics result{k[0][0], k[1][1], k[0][2], k[1][2]};
    if (!std::isfinite(result.fx) || !std::isfinite(result.fy) || !std::isfinite(result.cx) ||
        !std::isfinite(result.cy) || result.fx <= 0 || result.fy <= 0)
      throw std::runtime_error("Invalid camera intrinsics");
    return result;
  }

  static std::vector<double> geometry_key(dai::ImgFrame & frame, const point_cloud::Intrinsics & k,
    const point_cloud::RigidTransform & t)
  {
    std::vector<double> key{double(frame.getWidth()), double(frame.getHeight()), k.fx, k.fy, k.cx, k.cy};
    key.insert(key.end(), t.rotation.begin(), t.rotation.end());
    key.insert(key.end(), t.translation.begin(), t.translation.end());
    return key;
  }

  static point_cloud::RigidTransform inverse(const point_cloud::RigidTransform & t) {
    point_cloud::RigidTransform result;
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 3; ++col) result.rotation[row*3+col] = t.rotation[col*3+row];
    result.translation = point_cloud::rotate(result.rotation, t.translation);
    for (auto & v : result.translation) v = -v;
    return result;
  }

  roi_lidar::Scan associate(dai::ImgFrame & rgb, dai::ImgFrame & depth,
    const dai::CalibrationHandler & calibration, const point_cloud::RigidTransform & cam_a_from_rgb,
    const point_cloud::RigidTransform & vehicle_from_cam_a, const point_cloud::Intrinsics & rgb_k,
    const roi_lidar::Config & config, bool estimate_ground, double core_fraction,
    const std::vector<Detection> & detections, std::vector<std::pair<int, int>> & counts)
  {
    const auto depth_k = intrinsics(depth);
    const auto cam_a_from_depth = point_cloud::rgbFromFrame(depth, calibration);
    const auto rgb_from_depth = point_cloud::compose(inverse(cam_a_from_rgb), cam_a_from_depth);
    const auto vehicle_from_depth = point_cloud::compose(vehicle_from_cam_a, cam_a_from_depth);
    const auto key = geometry_key(depth, depth_k, vehicle_from_depth);
    if (key != depth_key_) {
      auto scan_options = config.scan;
      if (!estimate_ground) scan_options.ransac.enabled = false;
      projector_.configure(depth.getWidth(), depth.getHeight(), depth_k, vehicle_from_depth, scan_options);
      depth_key_ = key;
    }
    mask_.create(rgb.getHeight(), rgb.getWidth(), CV_32SC1);
    mask_.setTo(cv::Scalar(-1));
    // The detector returns boxes in score order. The highest score owns overlaps.
    // Mask value = detection index * 2, plus 1 inside the box's centre share.
    const auto paint = [this](const cv::Rect2f & box, int value) {
      const int x0 = std::clamp(static_cast<int>(std::ceil(box.x)), 0, mask_.cols);
      const int y0 = std::clamp(static_cast<int>(std::ceil(box.y)), 0, mask_.rows);
      const int x1 = std::clamp(static_cast<int>(std::ceil(box.x + box.width)), x0, mask_.cols);
      const int y1 = std::clamp(static_cast<int>(std::ceil(box.y + box.height)), y0, mask_.rows);
      if (x1 > x0 && y1 > y0) mask_(cv::Rect(x0, y0, x1 - x0, y1 - y0)).setTo(cv::Scalar(value));
    };
    for (std::size_t i = detections.size(); i > 0; --i) {
      const auto & box = detections[i - 1].box;
      const int value = static_cast<int>(i - 1) * 2;
      paint(box, value);
      const float inset_x = box.width * static_cast<float>(1 - core_fraction) / 2;
      const float inset_y = box.height * static_cast<float>(1 - core_fraction) / 2;
      paint({box.x + inset_x, box.y + inset_y, box.width - 2 * inset_x, box.height - 2 * inset_y}, value + 1);
    }
    const roi_lidar::Projector::PointSelector select = [&](int u, int v, double z) {
      auto p = point_cloud::rotate(rgb_from_depth.rotation,
        {(u - depth_k.cx) * z / depth_k.fx, (v - depth_k.cy) * z / depth_k.fy, z});
      for (std::size_t i = 0; i < p.size(); ++i) p[i] += rgb_from_depth.translation[i];
      if (p[2] <= 0) return -1;
      const double x = rgb_k.fx * p[0] / p[2] + rgb_k.cx;
      const double y = rgb_k.fy * p[1] / p[2] + rgb_k.cy;
      if (!(x >= 0 && x < mask_.cols && y >= 0 && y < mask_.rows)) return -1;
      const int value = mask_.at<int>(static_cast<int>(y), static_cast<int>(x));
      if (value >= 0) ++counts[value >> 1].first;
      return value;
    };
    const auto & bytes = depth.getData();
    const auto stride = depth.getStride() ? depth.getStride() : depth.getWidth() * 2U;
    auto scan = projector_.project(bytes.data(), bytes.size(), stride, config.terrain.below_ground_m, &select);
    for (int value : scan.obstacle_selection_ids) ++counts[value >> 1].second;
    return scan;
  }

  // One position and class footprint per detection from its ground-filtered points.
  static std::vector<ObjectEstimate> estimate(const roi_lidar::Scan & scan,
    const std::vector<Detection> & detections, const point_cloud::Intrinsics & rgb_k,
    const point_cloud::RigidTransform & vehicle_from_rgb, const BevSettings & settings)
  {
    std::vector<std::vector<ObjectPoint>> core(detections.size()), all(detections.size());
    for (std::size_t i = 0; i < scan.obstacle_indices.size(); ++i) {
      const auto & p = scan.surface_points[scan.obstacle_indices[i]];
      const int value = scan.obstacle_selection_ids[i];
      const auto id = static_cast<std::size_t>(value >> 1);
      all[id].push_back({p[0], p[1]});
      if (value & 1) core[id].push_back({p[0], p[1]});
    }
    std::vector<ObjectEstimate> objects(detections.size());
    for (std::size_t id = 0; id < detections.size(); ++id) {
      const auto & box = detections[id].box;
      // Direction comes from the box centre ray: stable even with few depth points.
      const auto ray = point_cloud::rotate(vehicle_from_rgb.rotation,
        {(box.x + box.width / 2 - rgb_k.cx) / rgb_k.fx, (box.y + box.height / 2 - rgb_k.cy) / rgb_k.fy, 1.0});
      // Small or distant boxes leave too few centre points; then use the whole box.
      const auto & points = core[id].size() >= settings.localization.min_points ? core[id] : all[id];
      objects[id] = localize(points, vehicle_from_rgb.translation[0], vehicle_from_rgb.translation[1],
        std::atan2(ray[1], ray[0]), settings.footprint(detections[id].class_id), settings.localization);
    }
    return objects;
  }

  roi_lidar::Projector projector_;
  roi_lidar::BevProjector background_;
  std::vector<double> rgb_key_, depth_key_;
  cv::Mat mask_;
};

}  // namespace multiclass_yolo
