#include "lane_mask/lane_mask_node.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace lane_mask
{
namespace
{

constexpr std::uint8_t kNeutralChroma = 128U;
constexpr double kPi = 3.14159265358979323846;

int makeOdd(const int value, const int minimum)
{
  const int clamped = std::max(value, minimum);
  return (clamped % 2 == 0) ? clamped + 1 : clamped;
}

double toRadians(const double degrees)
{
  return degrees * kPi / 180.0;
}

}  // namespace

class LaneMaskNode::Impl
{
public:
  explicit Impl(rclcpp::Node & node)
  : node_(node)
  {
    declareParameters();
    readParameters();

    const auto image_qos = rclcpp::SensorDataQoS().keep_last(1);

    if (nv12_publish_enabled_) {
      nv12_publisher_ = node_.create_publisher<sensor_msgs::msg::Image>(
        nv12_topic_, image_qos);
    }
    if (mask_publish_enabled_) {
      mask_publisher_ = node_.create_publisher<sensor_msgs::msg::Image>(
        mask_topic_, image_qos);
    }

    subscription_ = node_.create_subscription<sensor_msgs::msg::Image>(
      input_topic_,
      image_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        onImage(std::move(message));
      });

    status_timer_ = node_.create_wall_timer(
      std::chrono::duration<double>(status_log_interval_sec_),
      [this]() {logStatus();});

    // Verification only: assumes the documented 1280x720 (16:9) native
    // input to estimate the working image height here. computeMask()
    // recomputes the real cut_row every frame from the actual image size.
    const int assumed_work_rows = static_cast<int>(
      std::lround(process_width_ * 9.0 / 16.0));
    const int startup_cut_row = computeCutRow(assumed_work_rows);

    RCLCPP_INFO(
      node_.get_logger(),
      "lane_mask started: in=%s, nv12=%s, mono8=%s, process_width=%d, "
      "tophat_k=%d, tophat_thresh=%d, dark=%d/%.2f, "
      "cut_row=%d (est. work_rows=%d)",
      input_topic_.c_str(),
      nv12_publish_enabled_ ? nv12_topic_.c_str() : "disabled",
      mask_publish_enabled_ ? mask_topic_.c_str() : "disabled",
      process_width_,
      tophat_kernel_,
      tophat_threshold_,
      dark_threshold_,
      dark_ratio_,
      startup_cut_row,
      assumed_work_rows);
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
    node_.declare_parameter<std::string>("input_topic", "/camera/image_rect");
    node_.declare_parameter<std::string>("nv12_topic", "/camera/image_lane");
    node_.declare_parameter<std::string>("mask_topic", "/lane_mask");
    node_.declare_parameter<bool>("nv12_publish_enabled", true);
    node_.declare_parameter<bool>("mask_publish_enabled", true);

    // 0 keeps the native input width. Smaller values trade detail for speed.
    node_.declare_parameter<int>("process_width", 960);
    node_.declare_parameter<double>("process_max_fps", 30.0);

    // Size-like parameters below are written for this working width and are
    // rescaled automatically when process_width differs, so changing the
    // processing resolution does not silently invalidate the tuning.
    node_.declare_parameter<int>("param_reference_width", 960);

    // Stage 1: white top-hat keeps thin bright strokes, drops wide bright areas.
    node_.declare_parameter<int>("tophat_kernel", 31);
    node_.declare_parameter<int>("tophat_threshold", 60);
    node_.declare_parameter<int>("blur_kernel", 5);

    // Stage 2: lane tape only exists next to the dark EVA mat.
    node_.declare_parameter<int>("dark_threshold", 70);
    node_.declare_parameter<double>("dark_ratio", 0.12);
    node_.declare_parameter<int>("dark_window", 25);
    node_.declare_parameter<bool>("dark_gate_enabled", true);

    // Stage 2b: bilateral dark condition. Lane tape has dark mat on BOTH
    // sides (left+right for a vertical stroke, or above+below for a
    // horizontal one); a mat edge is dark on only one side. Takes priority
    // over the ratio-based dark gate above when enabled, so both methods
    // can be compared on the real car.
    node_.declare_parameter<bool>("bilateral_dark_enabled", true);
    node_.declare_parameter<int>("bilateral_span_px", 25);

    // Stage 3: shape filtering on connected components.
    node_.declare_parameter<int>("min_area", 200);
    node_.declare_parameter<int>("blob_area", 1200);
    node_.declare_parameter<double>("blob_aspect", 2.5);
    node_.declare_parameter<double>("blob_fill", 0.45);
    node_.declare_parameter<bool>("sliver_filter_enabled", true);
    node_.declare_parameter<int>("sliver_max_height", 12);
    node_.declare_parameter<double>("sliver_top_ratio", 0.35);

    // Geometric horizon cut: rows above the screen row that corresponds to
    // cut_beyond_m of forward distance cannot physically be ground (wall,
    // chair, ceiling, distant floor), computed from camera height/pitch.
    node_.declare_parameter<double>("cut_beyond_m", 2.0);
    node_.declare_parameter<double>("camera_height_m", 0.17);
    node_.declare_parameter<double>("camera_pitch_deg", 13.0);
    node_.declare_parameter<double>("camera_fy", 561.136352539);
    node_.declare_parameter<double>("camera_cy", 352.621124268);
    node_.declare_parameter<int>("reference_image_height", 720);

    // Bottom cut: the bumper and its white sticker sit at a fixed screen
    // position and are otherwise indistinguishable from lane tape.
    node_.declare_parameter<double>("bottom_cut_ratio", 0.15);

    node_.declare_parameter<bool>("preview_enabled", false);
    node_.declare_parameter<std::string>("preview_window_name", "lane mask");
    node_.declare_parameter<double>("status_log_interval_sec", 5.0);
  }

  void readParameters()
  {
    input_topic_ = node_.get_parameter("input_topic").as_string();
    nv12_topic_ = node_.get_parameter("nv12_topic").as_string();
    mask_topic_ = node_.get_parameter("mask_topic").as_string();
    nv12_publish_enabled_ =
      node_.get_parameter("nv12_publish_enabled").as_bool();
    mask_publish_enabled_ =
      node_.get_parameter("mask_publish_enabled").as_bool();

    process_width_ = static_cast<int>(
      node_.get_parameter("process_width").as_int());
    process_max_fps_ = node_.get_parameter("process_max_fps").as_double();
    param_reference_width_ = std::max(
      1, static_cast<int>(
        node_.get_parameter("param_reference_width").as_int()));

    tophat_kernel_ = makeOdd(
      static_cast<int>(node_.get_parameter("tophat_kernel").as_int()), 3);
    tophat_threshold_ = static_cast<int>(
      node_.get_parameter("tophat_threshold").as_int());
    blur_kernel_ = makeOdd(
      static_cast<int>(node_.get_parameter("blur_kernel").as_int()), 1);

    dark_threshold_ = static_cast<int>(
      node_.get_parameter("dark_threshold").as_int());
    dark_ratio_ = node_.get_parameter("dark_ratio").as_double();
    dark_window_ = makeOdd(
      static_cast<int>(node_.get_parameter("dark_window").as_int()), 3);
    dark_gate_enabled_ = node_.get_parameter("dark_gate_enabled").as_bool();

    bilateral_dark_enabled_ =
      node_.get_parameter("bilateral_dark_enabled").as_bool();
    bilateral_span_px_ = std::max(
      1, static_cast<int>(
        node_.get_parameter("bilateral_span_px").as_int()));

    min_area_ = static_cast<int>(node_.get_parameter("min_area").as_int());
    blob_area_ = static_cast<int>(node_.get_parameter("blob_area").as_int());
    blob_aspect_ = node_.get_parameter("blob_aspect").as_double();
    blob_fill_ = node_.get_parameter("blob_fill").as_double();
    sliver_filter_enabled_ =
      node_.get_parameter("sliver_filter_enabled").as_bool();
    sliver_max_height_ = static_cast<int>(
      node_.get_parameter("sliver_max_height").as_int());
    sliver_top_ratio_ = node_.get_parameter("sliver_top_ratio").as_double();

    cut_beyond_m_ = node_.get_parameter("cut_beyond_m").as_double();
    camera_height_m_ = node_.get_parameter("camera_height_m").as_double();
    camera_pitch_deg_ = node_.get_parameter("camera_pitch_deg").as_double();
    camera_fy_ = node_.get_parameter("camera_fy").as_double();
    camera_cy_ = node_.get_parameter("camera_cy").as_double();
    reference_image_height_ = std::max(
      1, static_cast<int>(
        node_.get_parameter("reference_image_height").as_int()));

    bottom_cut_ratio_ = node_.get_parameter("bottom_cut_ratio").as_double();

    preview_enabled_ = node_.get_parameter("preview_enabled").as_bool();
    preview_window_name_ =
      node_.get_parameter("preview_window_name").as_string();
    status_log_interval_sec_ =
      node_.get_parameter("status_log_interval_sec").as_double();
  }

  void onImage(sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    received_.fetch_add(1U, std::memory_order_relaxed);

    const int width = static_cast<int>(message->width);
    const int height = static_cast<int>(message->height);
    const std::size_t step = static_cast<std::size_t>(message->step);
    const std::size_t nv12_rows =
      static_cast<std::size_t>(height) * 3U / 2U;

    if (
      message->encoding != "nv12" ||
      width <= 0 || height <= 0 ||
      step < static_cast<std::size_t>(width) ||
      message->data.size() < step * nv12_rows)
    {
      rejected_.fetch_add(1U, std::memory_order_relaxed);
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 5000,
        "Rejected image: expected nv12, got %ux%u %s (step=%u, data=%zu).",
        message->width, message->height, message->encoding.c_str(),
        message->step, message->data.size());
      return;
    }

    if (process_max_fps_ > 0.0) {
      const auto now = std::chrono::steady_clock::now();
      const double minimum_period = 1.0 / process_max_fps_;
      const double elapsed =
        std::chrono::duration<double>(now - last_processed_at_).count();
      if (elapsed < minimum_period) {
        return;
      }
      last_processed_at_ = now;
    }

    // NV12 luma plane is already the grayscale image. No conversion needed.
    const cv::Mat luma(
      height, width, CV_8UC1,
      const_cast<std::uint8_t *>(message->data.data()), step);

    const cv::Mat mask = computeMask(luma);
    if (nv12_publisher_) {
      publishNv12(*message, mask, width, height);
    }
    if (mask_publisher_) {
      publishMono8(*message, mask, width, height);
    }
    if (preview_enabled_) {
      cv::imshow(preview_window_name_, mask);
      cv::waitKey(1);
    }
    processed_.fetch_add(1U, std::memory_order_relaxed);
  }

  // Screen row (in a work_rows-tall image) beyond which the camera cannot
  // physically see ground, given its height/pitch and the cut-off distance.
  // cut_beyond_m <= 0 means "use the horizon" (infinite distance).
  int computeCutRow(const int work_rows) const
  {
    const double alpha = (cut_beyond_m_ > 0.0)
      ? std::atan(camera_height_m_ / cut_beyond_m_) -
        toRadians(camera_pitch_deg_)
      : -toRadians(camera_pitch_deg_);
    const double row_ref = camera_cy_ + camera_fy_ * std::tan(alpha);
    const double cut_row_d =
      row_ref / static_cast<double>(reference_image_height_) *
      static_cast<double>(work_rows);
    const int cut_row = static_cast<int>(std::lround(cut_row_d));
    return std::max(0, std::min(cut_row, std::max(0, work_rows - 1)));
  }

  cv::Mat computeMask(const cv::Mat & luma)
  {
    cv::Mat work;
    const bool downscale =
      process_width_ > 0 && process_width_ < luma.cols;
    if (downscale) {
      const double scale =
        static_cast<double>(process_width_) / static_cast<double>(luma.cols);
      cv::resize(luma, work, cv::Size(), scale, scale, cv::INTER_AREA);
    } else {
      work = luma;
    }

    // Length-like values scale with the working width, area-like values with
    // its square. Intensity thresholds are resolution independent.
    const double ratio =
      static_cast<double>(work.cols) /
      static_cast<double>(param_reference_width_);
    const int tophat_kernel = makeOdd(
      static_cast<int>(std::lround(tophat_kernel_ * ratio)), 3);
    const int dark_window = makeOdd(
      static_cast<int>(std::lround(dark_window_ * ratio)), 3);
    const int min_area = std::max(
      10, static_cast<int>(std::lround(min_area_ * ratio * ratio)));
    const int blob_area = std::max(
      50, static_cast<int>(std::lround(blob_area_ * ratio * ratio)));
    const int sliver_max_height = std::max(
      2, static_cast<int>(std::lround(sliver_max_height_ * ratio)));
    // Minimum 3 (not 1) so the directional dilation never degenerates to a
    // 1x1 no-op kernel, which would silently disable the bilateral gate.
    const int bilateral_span = std::max(
      3, static_cast<int>(std::lround(bilateral_span_px_ * ratio)));

    cv::Mat blurred;
    if (blur_kernel_ > 1) {
      cv::GaussianBlur(work, blurred, {blur_kernel_, blur_kernel_}, 0);
    } else {
      blurred = work;
    }

    // Stage 1: white top-hat. Wide bright regions (terrazzo floor, walls)
    // survive the opening and are subtracted away; thin bright strokes stay.
    const cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT, {tophat_kernel, tophat_kernel});
    cv::Mat tophat;
    cv::morphologyEx(blurred, tophat, cv::MORPH_TOPHAT, kernel);
    cv::Mat candidate;
    cv::threshold(
      tophat, candidate, tophat_threshold_, 255, cv::THRESH_BINARY);

    // Stage 2: contextual gate. Lane tape sits on the dark mat, terrazzo
    // speckles do not.
    if (bilateral_dark_enabled_) {
      // Bilateral dark condition: a mat edge is dark on only one side, but
      // lane tape has dark mat on BOTH sides (left+right for a vertical
      // stroke, or above+below for a horizontal one). Directional dilation
      // with the anchor at one end of the kernel looks only in that
      // direction from each pixel.
      cv::Mat dark;
      cv::threshold(
        blurred, dark, dark_threshold_, 1, cv::THRESH_BINARY_INV);
      const cv::Mat kh = cv::getStructuringElement(
        cv::MORPH_RECT, {bilateral_span, 1});
      const cv::Mat kv = cv::getStructuringElement(
        cv::MORPH_RECT, {1, bilateral_span});
      cv::Mat left;
      cv::Mat right;
      cv::Mat up;
      cv::Mat down;
      cv::dilate(
        dark, left, kh, {bilateral_span - 1, 0}, 1,
        cv::BORDER_CONSTANT, cv::Scalar(0));
      cv::dilate(
        dark, right, kh, {0, 0}, 1, cv::BORDER_CONSTANT, cv::Scalar(0));
      cv::dilate(
        dark, up, kv, {0, bilateral_span - 1}, 1,
        cv::BORDER_CONSTANT, cv::Scalar(0));
      cv::dilate(
        dark, down, kv, {0, 0}, 1, cv::BORDER_CONSTANT, cv::Scalar(0));
      cv::Mat vertical_stroke;
      cv::Mat horizontal_stroke;
      cv::bitwise_and(left, right, vertical_stroke);
      cv::bitwise_and(up, down, horizontal_stroke);
      cv::Mat gate01;
      cv::bitwise_or(vertical_stroke, horizontal_stroke, gate01);
      cv::Mat gate;
      gate01.convertTo(gate, CV_8UC1, 255.0);
      cv::bitwise_and(candidate, gate, candidate);
    } else if (dark_gate_enabled_) {
      cv::Mat dark;
      cv::threshold(
        blurred, dark, dark_threshold_, 1, cv::THRESH_BINARY_INV);
      cv::Mat dark_ratio_map;
      cv::boxFilter(
        dark, dark_ratio_map, CV_32F, {dark_window, dark_window});
      cv::Mat gate;
      cv::threshold(
        dark_ratio_map, gate, dark_ratio_, 255, cv::THRESH_BINARY);
      gate.convertTo(gate, CV_8UC1);
      cv::bitwise_and(candidate, gate, candidate);
    }

    // Geometric horizon cut: rows above cut_row cannot be ground given the
    // camera height and pitch, so remove them before shape filtering.
    const int cut_row = computeCutRow(candidate.rows);
    if (cut_row > 0) {
      candidate(cv::Rect(0, 0, candidate.cols, cut_row)).setTo(0);
    }

    // Bottom cut: the bumper and its sticker sit at a fixed screen position.
    if (bottom_cut_ratio_ > 0.0) {
      const int bottom_rows = std::min(
        candidate.rows,
        static_cast<int>(
          std::lround(candidate.rows * bottom_cut_ratio_)));
      if (bottom_rows > 0) {
        candidate(
          cv::Rect(
            0, candidate.rows - bottom_rows, candidate.cols, bottom_rows))
        .setTo(0);
      }
    }

    // Stage 3: shape filtering.
    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int count = cv::connectedComponentsWithStats(
      candidate, labels, stats, centroids, 8, CV_32S);
    cv::Mat filtered = cv::Mat::zeros(candidate.size(), CV_8UC1);
    const int rows = candidate.rows;
    for (int index = 1; index < count; ++index) {
      const int area = stats.at<int>(index, cv::CC_STAT_AREA);
      if (area < min_area) {
        continue;
      }
      const int box_x = stats.at<int>(index, cv::CC_STAT_LEFT);
      const int box_y = stats.at<int>(index, cv::CC_STAT_TOP);
      const int box_w = stats.at<int>(index, cv::CC_STAT_WIDTH);
      const int box_h = stats.at<int>(index, cv::CC_STAT_HEIGHT);
      static_cast<void>(box_x);
      const double aspect =
        static_cast<double>(std::max(box_w, box_h)) /
        static_cast<double>(std::max(1, std::min(box_w, box_h)));
      const double fill =
        static_cast<double>(area) /
        static_cast<double>(std::max(1, box_w * box_h));
      if (area > blob_area && aspect < blob_aspect_ && fill > blob_fill_) {
        continue;  // wide, solid blob: background surface
      }
      if (
        sliver_filter_enabled_ &&
        box_w > 3 * box_h &&
        box_h <= sliver_max_height &&
        static_cast<double>(box_y) <
        static_cast<double>(rows) * sliver_top_ratio_)
      {
        continue;  // thin horizontal sliver near the top: far floor seam
      }
      filtered.setTo(255, labels == index);
    }

    if (downscale) {
      cv::Mat upscaled;
      cv::resize(
        filtered, upscaled, luma.size(), 0.0, 0.0, cv::INTER_NEAREST);
      return upscaled;
    }
    return filtered;
  }

  void publishNv12(
    const sensor_msgs::msg::Image & source,
    const cv::Mat & mask,
    const int width,
    const int height)
  {
    auto output = nv12_publisher_->create_ros_message_unique_ptr();
    output->header = source.header;
    output->height = static_cast<std::uint32_t>(height);
    output->width = static_cast<std::uint32_t>(width);
    output->encoding = "nv12";
    output->is_bigendian = source.is_bigendian;
    output->step = static_cast<std::uint32_t>(width);

    const std::size_t luma_bytes =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    const std::size_t chroma_bytes = luma_bytes / 2U;
    output->data.resize(luma_bytes + chroma_bytes);

    for (int row = 0; row < height; ++row) {
      std::copy(
        mask.ptr<std::uint8_t>(row),
        mask.ptr<std::uint8_t>(row) + width,
        output->data.begin() + static_cast<std::ptrdiff_t>(row) * width);
    }
    // Neutral chroma: the downstream BT.601 conversion turns the luma-only
    // frame into a plain grayscale image, so the mask survives unchanged.
    std::fill(
      output->data.begin() + static_cast<std::ptrdiff_t>(luma_bytes),
      output->data.end(),
      kNeutralChroma);

    nv12_publisher_->publish(std::move(output));
  }

  void publishMono8(
    const sensor_msgs::msg::Image & source,
    const cv::Mat & mask,
    const int width,
    const int height)
  {
    auto output = mask_publisher_->create_ros_message_unique_ptr();
    output->header = source.header;
    output->height = static_cast<std::uint32_t>(height);
    output->width = static_cast<std::uint32_t>(width);
    output->encoding = "mono8";
    output->is_bigendian = source.is_bigendian;
    output->step = static_cast<std::uint32_t>(width);
    output->data.resize(
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
    for (int row = 0; row < height; ++row) {
      std::copy(
        mask.ptr<std::uint8_t>(row),
        mask.ptr<std::uint8_t>(row) + width,
        output->data.begin() + static_cast<std::ptrdiff_t>(row) * width);
    }
    mask_publisher_->publish(std::move(output));
  }

  void logStatus()
  {
    const auto received = received_.exchange(0U, std::memory_order_relaxed);
    const auto processed = processed_.exchange(0U, std::memory_order_relaxed);
    const auto rejected = rejected_.exchange(0U, std::memory_order_relaxed);
    const double interval = std::max(0.001, status_log_interval_sec_);
    RCLCPP_INFO(
      node_.get_logger(),
      "lane_mask: in=%.1f fps, out=%.1f fps, rejected=%u",
      static_cast<double>(received) / interval,
      static_cast<double>(processed) / interval,
      rejected);
  }

  rclcpp::Node & node_;

  std::string input_topic_;
  std::string nv12_topic_;
  std::string mask_topic_;
  bool nv12_publish_enabled_{true};
  bool mask_publish_enabled_{true};

  int process_width_{960};
  int param_reference_width_{960};
  double process_max_fps_{30.0};

  int tophat_kernel_{31};
  int tophat_threshold_{60};
  int blur_kernel_{5};

  int dark_threshold_{70};
  double dark_ratio_{0.12};
  int dark_window_{25};
  bool dark_gate_enabled_{true};

  bool bilateral_dark_enabled_{true};
  int bilateral_span_px_{25};

  int min_area_{200};
  int blob_area_{1200};
  double blob_aspect_{2.5};
  double blob_fill_{0.45};
  bool sliver_filter_enabled_{true};
  int sliver_max_height_{12};
  double sliver_top_ratio_{0.35};

  double cut_beyond_m_{2.0};
  double camera_height_m_{0.17};
  double camera_pitch_deg_{13.0};
  double camera_fy_{561.136352539};
  double camera_cy_{352.621124268};
  int reference_image_height_{720};
  double bottom_cut_ratio_{0.15};

  bool preview_enabled_{false};
  std::string preview_window_name_{"lane mask"};
  double status_log_interval_sec_{5.0};

  std::chrono::steady_clock::time_point last_processed_at_{};

  std::atomic<unsigned> received_{0U};
  std::atomic<unsigned> processed_{0U};
  std::atomic<unsigned> rejected_{0U};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr nv12_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

LaneMaskNode::LaneMaskNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("lane_mask", options),
  impl_(std::make_unique<Impl>(*this))
{
}

LaneMaskNode::~LaneMaskNode() = default;

}  // namespace lane_mask

RCLCPP_COMPONENTS_REGISTER_NODE(lane_mask::LaneMaskNode)
