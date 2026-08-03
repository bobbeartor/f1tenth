#include "lane_mask/bev_lane_extractor_node.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "lane_mask/bev_lane_detector.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace lane_mask
{
namespace
{

int metersToPixels(const double meters, const double meters_per_pixel)
{
  return std::max(
    1,
    static_cast<int>(std::lround(meters / meters_per_pixel)));
}

void copyImageRows(const cv::Mat & image, sensor_msgs::msg::Image * message)
{
  const std::size_t row_bytes =
    static_cast<std::size_t>(image.cols * image.elemSize());
  message->data.resize(row_bytes * static_cast<std::size_t>(image.rows));
  for (int row = 0; row < image.rows; ++row) {
    std::copy(
      image.ptr<std::uint8_t>(row),
      image.ptr<std::uint8_t>(row) + row_bytes,
      message->data.begin() +
      static_cast<std::ptrdiff_t>(row) *
      static_cast<std::ptrdiff_t>(row_bytes));
  }
}

}  // namespace

class BevLaneExtractorNode::Impl
{
public:
  explicit Impl(rclcpp::Node & node)
  : node_(node)
  {
    declareParameters();
    readParameters();
    validateParameters();

    BevLaneDetectorConfig detector_config;
    detector_config.white_threshold = white_threshold_;
    detector_config.vertical_close_px = vertical_close_px_;
    detector_config.minimum_run_width_px = minimum_run_width_px_;
    detector_config.maximum_run_width_px = maximum_run_width_px_;
    detector_config.row_step_px = row_step_px_;
    detector_config.expected_lane_width_px =
      expected_lane_width_m_ / meter_per_pixel_;
    detector_config.lane_width_tolerance_px =
      lane_width_tolerance_m_ / meter_per_pixel_;
    detector_config.initial_center_tolerance_px =
      initial_center_tolerance_m_ / meter_per_pixel_;
    detector_config.single_lane_initial_tolerance_px =
      single_lane_initial_tolerance_m_ / meter_per_pixel_;
    detector_config.maximum_lateral_step_px =
      maximum_lateral_step_m_ / meter_per_pixel_;
    detector_config.maximum_tracking_gap_rows = maximum_tracking_gap_rows_;
    detector_config.minimum_points_per_lane = minimum_points_per_lane_;
    detector_config.maximum_fit_residual_px =
      maximum_fit_residual_m_ / meter_per_pixel_;
    detector_config.allow_single_lane = allow_single_lane_;
    detector_ = std::make_unique<BevLaneDetector>(detector_config);

    const auto image_qos = rclcpp::SensorDataQoS().keep_last(1);
    mask_publisher_ = node_.create_publisher<sensor_msgs::msg::Image>(
      mask_topic_, image_qos);
    debug_publisher_ = node_.create_publisher<sensor_msgs::msg::Image>(
      debug_topic_, image_qos);
    const auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    path_publisher_ = node_.create_publisher<nav_msgs::msg::Path>(
      path_topic_, path_qos);
    subscription_ = node_.create_subscription<sensor_msgs::msg::Image>(
      input_topic_,
      image_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        onImage(std::move(message));
      });
    status_timer_ = node_.create_wall_timer(
      std::chrono::duration<double>(status_log_interval_sec_),
      [this]() {logStatus();});

    if (preview_enabled_) {
      try {
        cv::namedWindow(preview_window_name_, cv::WINDOW_NORMAL);
        cv::resizeWindow(
          preview_window_name_,
          input_width_ * preview_scale_,
          input_height_ * preview_scale_);
      } catch (const cv::Exception & exception) {
        preview_enabled_ = false;
        RCLCPP_WARN(
          node_.get_logger(),
          "BEV lane preview disabled: %s",
          exception.what());
      }
    }

    RCLCPP_INFO(
      node_.get_logger(),
      "bev_lane_extractor started: in=%s (%dx%d bgr8), "
      "mask=%s, debug=%s, path=%s, lane_width=%.3fm +/- %.3fm, "
      "mark_width=%d..%dpx, path_x=[%.2f, %.2f]m, enabled=%s",
      input_topic_.c_str(),
      input_width_,
      input_height_,
      mask_topic_.c_str(),
      debug_topic_.c_str(),
      path_topic_.c_str(),
      expected_lane_width_m_,
      lane_width_tolerance_m_,
      minimum_run_width_px_,
      maximum_run_width_px_,
      path_minimum_x_m_,
      path_maximum_x_m_,
      enabled_ ? "true" : "false");
  }

  ~Impl()
  {
    if (preview_enabled_) {
      try {
        cv::destroyWindow(preview_window_name_);
      } catch (const cv::Exception &) {
      }
    }
  }

private:
  void declareParameters()
  {
    node_.declare_parameter<bool>("enabled", true);
    node_.declare_parameter<std::string>(
      "input_topic", "/camera/image_bev");
    node_.declare_parameter<std::string>(
      "mask_topic", "/camera/image_bev_lane");
    node_.declare_parameter<std::string>(
      "debug_topic", "/camera/image_bev_lane_debug");
    node_.declare_parameter<std::string>(
      "path_topic", "/lane_center_path");
    node_.declare_parameter<std::string>(
      "output_frame_id", "front_axle_bev");

    node_.declare_parameter<int>("input_width", 200);
    node_.declare_parameter<int>("input_height", 190);
    node_.declare_parameter<double>("x_min_m", 0.10);
    node_.declare_parameter<double>("x_max_m", 2.0);
    node_.declare_parameter<double>("y_min_m", -1.0);
    node_.declare_parameter<double>("y_max_m", 1.0);
    node_.declare_parameter<double>("meter_per_pixel", 0.01);

    node_.declare_parameter<int>("white_threshold", 160);
    node_.declare_parameter<double>("vertical_close_m", 0.05);
    node_.declare_parameter<double>("minimum_lane_mark_width_m", 0.01);
    node_.declare_parameter<double>("maximum_lane_mark_width_m", 0.18);
    node_.declare_parameter<int>("row_step_px", 2);

    node_.declare_parameter<double>("expected_lane_width_m", 0.625);
    node_.declare_parameter<double>("lane_width_tolerance_m", 0.125);
    node_.declare_parameter<double>("initial_center_tolerance_m", 0.45);
    node_.declare_parameter<double>(
      "single_lane_initial_tolerance_m", 0.20);
    node_.declare_parameter<double>("maximum_lateral_step_m", 0.08);
    node_.declare_parameter<int>("maximum_tracking_gap_rows", 20);
    node_.declare_parameter<int>("minimum_points_per_lane", 15);
    node_.declare_parameter<double>("maximum_fit_residual_m", 0.06);
    node_.declare_parameter<bool>("allow_single_lane", true);

    node_.declare_parameter<double>("path_minimum_x_m", 0.20);
    node_.declare_parameter<double>("path_maximum_x_m", 1.80);
    node_.declare_parameter<double>("path_step_m", 0.05);
    node_.declare_parameter<double>("output_line_thickness_m", 0.04);

    node_.declare_parameter<bool>("preview_enabled", false);
    node_.declare_parameter<std::string>(
      "preview_window_name", "BEV lane extraction");
    node_.declare_parameter<int>("preview_scale", 3);
    node_.declare_parameter<double>("status_log_interval_sec", 5.0);
  }

  void readParameters()
  {
    enabled_ = node_.get_parameter("enabled").as_bool();
    input_topic_ = node_.get_parameter("input_topic").as_string();
    mask_topic_ = node_.get_parameter("mask_topic").as_string();
    debug_topic_ = node_.get_parameter("debug_topic").as_string();
    path_topic_ = node_.get_parameter("path_topic").as_string();
    output_frame_id_ =
      node_.get_parameter("output_frame_id").as_string();

    input_width_ = static_cast<int>(
      node_.get_parameter("input_width").as_int());
    input_height_ = static_cast<int>(
      node_.get_parameter("input_height").as_int());
    x_min_m_ = node_.get_parameter("x_min_m").as_double();
    x_max_m_ = node_.get_parameter("x_max_m").as_double();
    y_min_m_ = node_.get_parameter("y_min_m").as_double();
    y_max_m_ = node_.get_parameter("y_max_m").as_double();
    meter_per_pixel_ =
      node_.get_parameter("meter_per_pixel").as_double();

    white_threshold_ = static_cast<int>(
      node_.get_parameter("white_threshold").as_int());
    vertical_close_m_ =
      node_.get_parameter("vertical_close_m").as_double();
    minimum_lane_mark_width_m_ =
      node_.get_parameter("minimum_lane_mark_width_m").as_double();
    maximum_lane_mark_width_m_ =
      node_.get_parameter("maximum_lane_mark_width_m").as_double();
    row_step_px_ = static_cast<int>(
      node_.get_parameter("row_step_px").as_int());

    expected_lane_width_m_ =
      node_.get_parameter("expected_lane_width_m").as_double();
    lane_width_tolerance_m_ =
      node_.get_parameter("lane_width_tolerance_m").as_double();
    initial_center_tolerance_m_ =
      node_.get_parameter("initial_center_tolerance_m").as_double();
    single_lane_initial_tolerance_m_ =
      node_.get_parameter(
      "single_lane_initial_tolerance_m").as_double();
    maximum_lateral_step_m_ =
      node_.get_parameter("maximum_lateral_step_m").as_double();
    maximum_tracking_gap_rows_ = static_cast<int>(
      node_.get_parameter("maximum_tracking_gap_rows").as_int());
    minimum_points_per_lane_ = static_cast<int>(
      node_.get_parameter("minimum_points_per_lane").as_int());
    maximum_fit_residual_m_ =
      node_.get_parameter("maximum_fit_residual_m").as_double();
    allow_single_lane_ =
      node_.get_parameter("allow_single_lane").as_bool();

    path_minimum_x_m_ =
      node_.get_parameter("path_minimum_x_m").as_double();
    path_maximum_x_m_ =
      node_.get_parameter("path_maximum_x_m").as_double();
    path_step_m_ = node_.get_parameter("path_step_m").as_double();
    output_line_thickness_m_ =
      node_.get_parameter("output_line_thickness_m").as_double();

    preview_enabled_ =
      node_.get_parameter("preview_enabled").as_bool();
    preview_window_name_ =
      node_.get_parameter("preview_window_name").as_string();
    preview_scale_ = static_cast<int>(
      node_.get_parameter("preview_scale").as_int());
    status_log_interval_sec_ =
      node_.get_parameter("status_log_interval_sec").as_double();

    if (meter_per_pixel_ > 0.0) {
      vertical_close_px_ =
        metersToPixels(vertical_close_m_, meter_per_pixel_);
      minimum_run_width_px_ =
        metersToPixels(minimum_lane_mark_width_m_, meter_per_pixel_);
      maximum_run_width_px_ =
        metersToPixels(maximum_lane_mark_width_m_, meter_per_pixel_);
      path_step_rows_ = metersToPixels(path_step_m_, meter_per_pixel_);
      output_line_thickness_px_ =
        metersToPixels(output_line_thickness_m_, meter_per_pixel_);
    }
  }

  void validateParameters() const
  {
    if (meter_per_pixel_ <= 0.0) {
      throw std::invalid_argument(
              "meter_per_pixel must be greater than zero");
    }
    const int expected_width = static_cast<int>(std::llround(
        (y_max_m_ - y_min_m_) / meter_per_pixel_));
    const int expected_height = static_cast<int>(std::llround(
        (x_max_m_ - x_min_m_) / meter_per_pixel_));
    if (
      input_topic_.empty() ||
      mask_topic_.empty() ||
      debug_topic_.empty() ||
      path_topic_.empty() ||
      output_frame_id_.empty() ||
      input_width_ <= 0 ||
      input_height_ <= 0 ||
      x_min_m_ < 0.0 ||
      x_max_m_ <= x_min_m_ ||
      y_max_m_ <= y_min_m_ ||
      expected_width != input_width_ ||
      expected_height != input_height_ ||
      white_threshold_ < 0 ||
      white_threshold_ > 255 ||
      vertical_close_m_ <= 0.0 ||
      minimum_lane_mark_width_m_ <= 0.0 ||
      maximum_lane_mark_width_m_ < minimum_lane_mark_width_m_ ||
      minimum_run_width_px_ <= 0 ||
      maximum_run_width_px_ < minimum_run_width_px_ ||
      row_step_px_ <= 0 ||
      expected_lane_width_m_ <= 0.0 ||
      lane_width_tolerance_m_ <= 0.0 ||
      lane_width_tolerance_m_ >= expected_lane_width_m_ ||
      initial_center_tolerance_m_ <= 0.0 ||
      single_lane_initial_tolerance_m_ <= 0.0 ||
      maximum_lateral_step_m_ <= 0.0 ||
      maximum_tracking_gap_rows_ < row_step_px_ ||
      minimum_points_per_lane_ < 3 ||
      maximum_fit_residual_m_ <= 0.0 ||
      path_minimum_x_m_ < x_min_m_ ||
      path_maximum_x_m_ > x_max_m_ ||
      path_maximum_x_m_ <= path_minimum_x_m_ ||
      path_step_m_ <= 0.0 ||
      path_step_rows_ <= 0 ||
      output_line_thickness_m_ <= 0.0 ||
      output_line_thickness_px_ <= 0 ||
      preview_scale_ <= 0 ||
      status_log_interval_sec_ <= 0.0)
    {
      throw std::invalid_argument(
              "invalid BEV lane extractor parameters");
    }
  }

  sensor_msgs::msg::Image makeImageMessage(
    const sensor_msgs::msg::Image & source,
    const cv::Mat & image,
    const std::string & encoding) const
  {
    sensor_msgs::msg::Image output;
    output.header = source.header;
    output.header.frame_id = output_frame_id_;
    output.height = static_cast<std::uint32_t>(image.rows);
    output.width = static_cast<std::uint32_t>(image.cols);
    output.encoding = encoding;
    output.is_bigendian = false;
    output.step = static_cast<std::uint32_t>(
      image.cols * image.elemSize());
    copyImageRows(image, &output);
    return output;
  }

  cv::Mat makeCleanMask(const BevLaneDetection & detection) const
  {
    cv::Mat mask = cv::Mat::zeros(input_height_, input_width_, CV_8UC1);
    const auto draw_lane =
      [this, &mask](const LanePolynomial & lane) {
        if (!lane.valid) {
          return;
        }
        cv::Point previous;
        bool have_previous = false;
        for (
          int row = lane.minimum_row;
          row <= lane.maximum_row;
          ++row)
        {
          const int column = static_cast<int>(
            std::lround(lane.columnAt(row)));
          if (column < 0 || column >= mask.cols) {
            have_previous = false;
            continue;
          }
          const cv::Point current(column, row);
          if (have_previous) {
            cv::line(
              mask,
              previous,
              current,
              cv::Scalar(255),
              output_line_thickness_px_,
              cv::LINE_8);
          }
          previous = current;
          have_previous = true;
        }
      };
    draw_lane(detection.left);
    draw_lane(detection.right);
    return mask;
  }

  cv::Mat makeDebugImage(
    const cv::Mat & input,
    const BevLaneDetection & detection) const
  {
    cv::Mat debug;
    input.convertTo(debug, -1, 0.40);

    for (const auto & point : detection.left_points) {
      cv::circle(
        debug,
        cv::Point(
          static_cast<int>(std::lround(point.x)),
          static_cast<int>(std::lround(point.y))),
        1,
        cv::Scalar(255, 180, 0),
        cv::FILLED,
        cv::LINE_AA);
    }
    for (const auto & point : detection.right_points) {
      cv::circle(
        debug,
        cv::Point(
          static_cast<int>(std::lround(point.x)),
          static_cast<int>(std::lround(point.y))),
        1,
        cv::Scalar(0, 120, 255),
        cv::FILLED,
        cv::LINE_AA);
    }

    const auto draw_fit =
      [&debug](
      const LanePolynomial & lane,
      const cv::Scalar & color) {
        if (!lane.valid) {
          return;
        }
        cv::Point previous;
        bool have_previous = false;
        for (
          int row = lane.minimum_row;
          row <= lane.maximum_row;
          ++row)
        {
          const int column = static_cast<int>(
            std::lround(lane.columnAt(row)));
          if (column < 0 || column >= debug.cols) {
            have_previous = false;
            continue;
          }
          const cv::Point current(column, row);
          if (have_previous) {
            cv::line(
              debug, previous, current, color, 2, cv::LINE_AA);
          }
          previous = current;
          have_previous = true;
        }
      };
    draw_fit(detection.left, cv::Scalar(255, 0, 0));
    draw_fit(detection.right, cv::Scalar(0, 0, 255));

    if (detection.center_valid) {
      cv::Point previous;
      bool have_previous = false;
      for (
        int row = detection.minimum_center_row;
        row <= detection.maximum_center_row;
        ++row)
      {
        const double center = detection.centerColumnAt(
          row, expected_lane_width_m_ / meter_per_pixel_);
        const int column = static_cast<int>(std::lround(center));
        if (
          !std::isfinite(center) ||
          column < 0 ||
          column >= debug.cols)
        {
          have_previous = false;
          continue;
        }
        const cv::Point current(column, row);
        if (have_previous) {
          cv::line(
            debug,
            previous,
            current,
            cv::Scalar(0, 255, 0),
            2,
            cv::LINE_AA);
        }
        previous = current;
        have_previous = true;
      }
    }

    const std::string mode = detection.both_lanes_valid ?
      "BOTH" :
      (detection.left.valid && !detection.right.valid ? "LEFT" :
      (detection.right.valid && !detection.left.valid ? "RIGHT" :
      (detection.center_valid ? "CENTER" : "NONE")));
    const std::string label = cv::format(
      "%s L=%d R=%d W=%.1fpx",
      mode.c_str(),
      detection.left.point_count,
      detection.right.point_count,
      detection.measured_lane_width_px);
    cv::putText(
      debug,
      label,
      cv::Point(4, 14),
      cv::FONT_HERSHEY_SIMPLEX,
      0.35,
      cv::Scalar(0, 255, 255),
      1,
      cv::LINE_AA);
    return debug;
  }

  nav_msgs::msg::Path makePath(
    const sensor_msgs::msg::Image & source,
    const BevLaneDetection & detection) const
  {
    nav_msgs::msg::Path path;
    path.header = source.header;
    path.header.frame_id = output_frame_id_;
    if (!detection.center_valid) {
      return path;
    }

    for (
      int row = detection.maximum_center_row;
      row >= detection.minimum_center_row;
      row -= path_step_rows_)
    {
      const double x_m =
        x_max_m_ -
        (static_cast<double>(row) + 0.5) * meter_per_pixel_;
      if (x_m < path_minimum_x_m_ || x_m > path_maximum_x_m_) {
        continue;
      }
      const double center_column = detection.centerColumnAt(
        row, expected_lane_width_m_ / meter_per_pixel_);
      if (
        !std::isfinite(center_column) ||
        center_column < 0.0 ||
        center_column >= static_cast<double>(input_width_))
      {
        continue;
      }
      const double y_m =
        y_max_m_ -
        (center_column + 0.5) * meter_per_pixel_;
      const double yaw = std::atan(
        detection.centerDerivativeAt(row));

      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = x_m;
      pose.pose.position.y = y_m;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.z = std::sin(0.5 * yaw);
      pose.pose.orientation.w = std::cos(0.5 * yaw);
      path.poses.push_back(std::move(pose));
    }
    return path;
  }

  void onImage(sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    if (!enabled_) {
      return;
    }
    received_.fetch_add(1U, std::memory_order_relaxed);
    const std::size_t required_step =
      static_cast<std::size_t>(input_width_) * 3U;
    if (
      message->encoding != "bgr8" ||
      static_cast<int>(message->width) != input_width_ ||
      static_cast<int>(message->height) != input_height_ ||
      static_cast<std::size_t>(message->step) < required_step ||
      message->data.size() <
      static_cast<std::size_t>(message->step) *
      static_cast<std::size_t>(message->height))
    {
      rejected_.fetch_add(1U, std::memory_order_relaxed);
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(),
        *node_.get_clock(),
        5000,
        "Rejected BEV: expected %dx%d bgr8, got %ux%u %s "
        "(step=%u, data=%zu)",
        input_width_,
        input_height_,
        message->width,
        message->height,
        message->encoding.c_str(),
        message->step,
        message->data.size());
      return;
    }

    try {
      const cv::Mat input(
        input_height_,
        input_width_,
        CV_8UC3,
        const_cast<std::uint8_t *>(message->data.data()),
        static_cast<std::size_t>(message->step));
      const BevLaneDetection detection = detector_->detect(input);
      const cv::Mat clean_mask = makeCleanMask(detection);
      const cv::Mat debug = makeDebugImage(input, detection);
      nav_msgs::msg::Path path = makePath(*message, detection);

      mask_publisher_->publish(
        makeImageMessage(*message, clean_mask, "mono8"));
      debug_publisher_->publish(
        makeImageMessage(*message, debug, "bgr8"));
      path_publisher_->publish(path);

      processed_.fetch_add(1U, std::memory_order_relaxed);
      if (detection.both_lanes_valid) {
        both_valid_.fetch_add(1U, std::memory_order_relaxed);
      } else if (detection.center_valid) {
        single_valid_.fetch_add(1U, std::memory_order_relaxed);
      } else {
        no_lane_.fetch_add(1U, std::memory_order_relaxed);
      }
      latest_left_points_.store(
        detection.left.point_count, std::memory_order_relaxed);
      latest_right_points_.store(
        detection.right.point_count, std::memory_order_relaxed);
      latest_width_px_.store(
        detection.measured_lane_width_px, std::memory_order_relaxed);
      latest_path_points_.store(
        static_cast<int>(path.poses.size()), std::memory_order_relaxed);

      if (preview_enabled_) {
        cv::imshow(preview_window_name_, debug);
        cv::waitKey(1);
      }
    } catch (const std::exception & exception) {
      rejected_.fetch_add(1U, std::memory_order_relaxed);
      RCLCPP_ERROR_THROTTLE(
        node_.get_logger(),
        *node_.get_clock(),
        5000,
        "BEV lane extraction failed: %s",
        exception.what());
    }
  }

  void logStatus()
  {
    const double interval = std::max(0.001, status_log_interval_sec_);
    const auto received =
      received_.exchange(0U, std::memory_order_relaxed);
    const auto processed =
      processed_.exchange(0U, std::memory_order_relaxed);
    const auto rejected =
      rejected_.exchange(0U, std::memory_order_relaxed);
    const auto both =
      both_valid_.exchange(0U, std::memory_order_relaxed);
    const auto single =
      single_valid_.exchange(0U, std::memory_order_relaxed);
    const auto none =
      no_lane_.exchange(0U, std::memory_order_relaxed);
    RCLCPP_INFO(
      node_.get_logger(),
      "bev_lane: in=%.1f fps, out=%.1f fps, rejected=%u, "
      "mode(both/single/none)=%u/%u/%u, points(L/R/path)=%d/%d/%d, "
      "width=%.1fpx",
      static_cast<double>(received) / interval,
      static_cast<double>(processed) / interval,
      rejected,
      both,
      single,
      none,
      latest_left_points_.load(std::memory_order_relaxed),
      latest_right_points_.load(std::memory_order_relaxed),
      latest_path_points_.load(std::memory_order_relaxed),
      latest_width_px_.load(std::memory_order_relaxed));
  }

  rclcpp::Node & node_;
  std::unique_ptr<BevLaneDetector> detector_;

  bool enabled_{true};
  std::string input_topic_;
  std::string mask_topic_;
  std::string debug_topic_;
  std::string path_topic_;
  std::string output_frame_id_;

  int input_width_{200};
  int input_height_{190};
  double x_min_m_{0.10};
  double x_max_m_{2.0};
  double y_min_m_{-1.0};
  double y_max_m_{1.0};
  double meter_per_pixel_{0.01};

  int white_threshold_{160};
  double vertical_close_m_{0.05};
  double minimum_lane_mark_width_m_{0.01};
  double maximum_lane_mark_width_m_{0.18};
  int vertical_close_px_{5};
  int minimum_run_width_px_{1};
  int maximum_run_width_px_{18};
  int row_step_px_{2};

  double expected_lane_width_m_{0.625};
  double lane_width_tolerance_m_{0.125};
  double initial_center_tolerance_m_{0.45};
  double single_lane_initial_tolerance_m_{0.20};
  double maximum_lateral_step_m_{0.08};
  int maximum_tracking_gap_rows_{20};
  int minimum_points_per_lane_{15};
  double maximum_fit_residual_m_{0.06};
  bool allow_single_lane_{true};

  double path_minimum_x_m_{0.20};
  double path_maximum_x_m_{1.80};
  double path_step_m_{0.05};
  int path_step_rows_{5};
  double output_line_thickness_m_{0.04};
  int output_line_thickness_px_{4};

  bool preview_enabled_{false};
  std::string preview_window_name_{"BEV lane extraction"};
  int preview_scale_{3};
  double status_log_interval_sec_{5.0};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::atomic<unsigned> received_{0U};
  std::atomic<unsigned> processed_{0U};
  std::atomic<unsigned> rejected_{0U};
  std::atomic<unsigned> both_valid_{0U};
  std::atomic<unsigned> single_valid_{0U};
  std::atomic<unsigned> no_lane_{0U};
  std::atomic<int> latest_left_points_{0};
  std::atomic<int> latest_right_points_{0};
  std::atomic<int> latest_path_points_{0};
  std::atomic<double> latest_width_px_{0.0};
};

BevLaneExtractorNode::BevLaneExtractorNode(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("bev_lane_extractor", options),
  impl_(std::make_unique<Impl>(*this))
{
}

BevLaneExtractorNode::~BevLaneExtractorNode() = default;

}  // namespace lane_mask

RCLCPP_COMPONENTS_REGISTER_NODE(lane_mask::BevLaneExtractorNode)
