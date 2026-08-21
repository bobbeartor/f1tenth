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
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#ifdef LANE_MASK_HAVE_OPENCV_CUDA
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudawarping.hpp>
#endif

#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace lane_mask
{
namespace
{

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

    mask_publisher_ = node_.create_publisher<sensor_msgs::msg::Image>(
      mask_topic_, image_qos);

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
      "lane_mask started: in=%s, out=%s, backend=%s, process_width=%d, "
      "tophat_k=%d, tophat_thresh=%d, contrast=%d bg_max=%d span=%d, "
      "geometry_cut=%d, roi=%.1f%%..%.1f%% (est. work_rows=%d)",
      input_topic_.c_str(),
      mask_topic_.c_str(),
      cuda_active_ ? "cuda" : "cpu",
      process_width_,
      tophat_kernel_,
      tophat_threshold_,
      bilateral_contrast_threshold_,
      bilateral_background_max_,
      bilateral_span_px_,
      startup_cut_row,
      roi_top_ratio_ * 100.0,
      (1.0 - bottom_cut_ratio_) * 100.0,
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
    node_.declare_parameter<std::string>("mask_topic", "/lane_mask");

    // 0 keeps the native input width. Smaller values trade detail for speed.
    node_.declare_parameter<int>("process_width", 640);
    node_.declare_parameter<bool>("use_cuda", true);

    // Size-like parameters below are written for this working width and are
    // rescaled automatically when process_width differs, so changing the
    // processing resolution does not silently invalidate the tuning.
    node_.declare_parameter<int>("param_reference_width", 960);

    // Stage 1: white top-hat keeps thin bright strokes, drops wide bright areas.
    node_.declare_parameter<int>("tophat_kernel", 31);
    node_.declare_parameter<int>("tophat_threshold", 60);
    node_.declare_parameter<int>("blur_kernel", 5);

    // Stage 2: relative bilateral contrast. Compare each bright candidate
    // with its own surroundings instead of requiring an absolute black level,
    // so the same lane tape works on both gray and black floor tiles.
    node_.declare_parameter<int>("bilateral_contrast_threshold", 30);
    node_.declare_parameter<int>("bilateral_background_max", 90);

    node_.declare_parameter<int>("bilateral_span_px", 45);

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

    // The centerline model only consumes the lower-middle image band. Avoid
    // running expensive morphology above that band.
    node_.declare_parameter<double>("roi_top_ratio", 0.50);

    // Bottom cut: the bumper and its white sticker sit at a fixed screen
    // position and are otherwise indistinguishable from lane tape.
    node_.declare_parameter<double>("bottom_cut_ratio", 0.25);

    node_.declare_parameter<bool>("preview_enabled", false);
    node_.declare_parameter<std::string>("preview_window_name", "lane mask");
    node_.declare_parameter<double>("status_log_interval_sec", 5.0);
  }

  void readParameters()
  {
    input_topic_ = node_.get_parameter("input_topic").as_string();
    mask_topic_ = node_.get_parameter("mask_topic").as_string();

    process_width_ = static_cast<int>(
      node_.get_parameter("process_width").as_int());
    use_cuda_ = node_.get_parameter("use_cuda").as_bool();
    param_reference_width_ = std::max(
      1, static_cast<int>(
        node_.get_parameter("param_reference_width").as_int()));

    tophat_kernel_ = makeOdd(
      static_cast<int>(node_.get_parameter("tophat_kernel").as_int()), 3);
    tophat_threshold_ = static_cast<int>(
      node_.get_parameter("tophat_threshold").as_int());
    blur_kernel_ = makeOdd(
      static_cast<int>(node_.get_parameter("blur_kernel").as_int()), 1);

    bilateral_contrast_threshold_ = std::clamp(
      static_cast<int>(
        node_.get_parameter("bilateral_contrast_threshold").as_int()),
      0, 255);
    bilateral_background_max_ = std::clamp(
      static_cast<int>(
        node_.get_parameter("bilateral_background_max").as_int()),
      0, 255);
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

    roi_top_ratio_ = std::clamp(
      node_.get_parameter("roi_top_ratio").as_double(), 0.0, 1.0);
    bottom_cut_ratio_ = node_.get_parameter("bottom_cut_ratio").as_double();
    bottom_cut_ratio_ = std::clamp(bottom_cut_ratio_, 0.0, 1.0);

    preview_enabled_ = node_.get_parameter("preview_enabled").as_bool();
    preview_window_name_ =
      node_.get_parameter("preview_window_name").as_string();
    status_log_interval_sec_ =
      node_.get_parameter("status_log_interval_sec").as_double();

    initializeCuda();
  }

  void initializeCuda()
  {
#ifdef LANE_MASK_HAVE_OPENCV_CUDA
    if (!use_cuda_) {
      return;
    }
    try {
      const int device_count = cv::cuda::getCudaEnabledDeviceCount();
      if (device_count > 0) {
        cv::cuda::setDevice(0);
        cuda_stream_ = std::make_unique<cv::cuda::Stream>();
        cuda_active_ = true;
        return;
      }
      RCLCPP_WARN(
        node_.get_logger(),
        "CUDA requested but no CUDA device is available; using CPU.");
    } catch (const cv::Exception & error) {
      RCLCPP_WARN(
        node_.get_logger(),
        "CUDA initialization failed (%s); using CPU.", error.what());
    }
#else
    if (use_cuda_) {
      RCLCPP_WARN(
        node_.get_logger(),
        "CUDA requested but lane_detect was built without OpenCV CUDA modules; "
        "using CPU.");
    }
#endif
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

    // NV12 luma plane is already the grayscale image. No conversion needed.
    const cv::Mat luma(
      height, width, CV_8UC1,
      const_cast<std::uint8_t *>(message->data.data()), step);

    const cv::Mat mask = computeMask(luma);
    publishMono8(*message, mask);
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

  struct ProcessingRows
  {
    int active_begin;
    int active_end;
    int processing_begin;
    int processing_end;
  };

  ProcessingRows computeProcessingRows(
    const int work_rows, const int tophat_kernel,
    const int bilateral_span) const
  {
    const int bottom_rows = std::clamp(
      static_cast<int>(std::lround(work_rows * bottom_cut_ratio_)),
      0, work_rows);
    const int active_end = work_rows - bottom_rows;
    const int configured_top = std::clamp(
      static_cast<int>(std::lround(work_rows * roi_top_ratio_)),
      0, work_rows);
    const int active_begin = std::min(
      std::max(computeCutRow(work_rows), configured_top), active_end);

    // Preserve the exact result at the active ROI boundary. Top-hat opening
    // can depend on pixels one full kernel width away, directional erosion on
    // span - 1 rows, and Gaussian blur on its radius.
    const int blur_radius = blur_kernel_ > 1 ? blur_kernel_ / 2 : 0;
    const int halo = blur_radius + std::max(
      tophat_kernel - 1, bilateral_span - 1);
    return {
      active_begin,
      active_end,
      std::max(0, active_begin - halo),
      std::min(work_rows, active_end + halo),
    };
  }

  cv::Mat computeMask(const cv::Mat & luma)
  {
#ifdef LANE_MASK_HAVE_OPENCV_CUDA
    if (cuda_active_) {
      try {
        return computeMaskCuda(luma);
      } catch (const cv::Exception & error) {
        cuda_active_ = false;
        clearCudaFilters();
        cuda_stream_.reset();
        RCLCPP_ERROR(
          node_.get_logger(),
          "CUDA lane masking failed (%s); switching permanently to CPU.",
          error.what());
      }
    }
#endif
    return computeMaskCpu(luma);
  }

  cv::Mat computeMaskCpu(const cv::Mat & luma)
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
    // Minimum 3 prevents the directional erosion from becoming a no-op.
    const int bilateral_span = std::max(
      3, static_cast<int>(std::lround(bilateral_span_px_ * ratio)));
    const ProcessingRows rows = computeProcessingRows(
      work.rows, tophat_kernel, bilateral_span);
    if (rows.active_begin >= rows.active_end) {
      output_.create(work.size(), CV_8UC1);
      output_.setTo(0);
      return output_;
    }
    const cv::Mat work_region = work.rowRange(
      rows.processing_begin, rows.processing_end);

    cv::Mat blurred;
    if (blur_kernel_ > 1) {
      cv::GaussianBlur(
        work_region, blurred, {blur_kernel_, blur_kernel_}, 0);
    } else {
      blurred = work_region;
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

    // Stage 2: contextual gate. Real lane tape has locally darker pavement
    // on both sides; isolated floor/tile highlights generally do not.
    // Directional erosion returns the darkest nearby pixel independently
    // on each side. A valid lane must be brighter than both horizontal
    // sides or both vertical sides by the configured contrast amount.
    const cv::Mat kh = cv::getStructuringElement(
      cv::MORPH_RECT, {bilateral_span, 1});
    const cv::Mat kv = cv::getStructuringElement(
      cv::MORPH_RECT, {1, bilateral_span});
    cv::Mat left_min;
    cv::Mat right_min;
    cv::Mat up_min;
    cv::Mat down_min;
    cv::erode(
      blurred, left_min, kh, {bilateral_span - 1, 0}, 1,
      cv::BORDER_CONSTANT, cv::Scalar(255));
    cv::erode(
      blurred, right_min, kh, {0, 0}, 1,
      cv::BORDER_CONSTANT, cv::Scalar(255));
    cv::erode(
      blurred, up_min, kv, {0, bilateral_span - 1}, 1,
      cv::BORDER_CONSTANT, cv::Scalar(255));
    cv::erode(
      blurred, down_min, kv, {0, 0}, 1,
      cv::BORDER_CONSTANT, cv::Scalar(255));

    // The larger side minimum is the stricter background reference.
    cv::Mat horizontal_reference;
    cv::Mat vertical_reference;
    cv::max(left_min, right_min, horizontal_reference);
    cv::max(up_min, down_min, vertical_reference);
    cv::Mat horizontal_delta;
    cv::Mat vertical_delta;
    cv::subtract(blurred, horizontal_reference, horizontal_delta);
    cv::subtract(blurred, vertical_reference, vertical_delta);

    cv::Mat vertical_stroke;
    cv::Mat horizontal_stroke;
    cv::compare(
      horizontal_delta, cv::Scalar(bilateral_contrast_threshold_),
      vertical_stroke, cv::CMP_GE);
    cv::compare(
      vertical_delta, cv::Scalar(bilateral_contrast_threshold_),
      horizontal_stroke, cv::CMP_GE);
    cv::Mat horizontal_background_ok;
    cv::Mat vertical_background_ok;
    cv::compare(
      horizontal_reference, cv::Scalar(bilateral_background_max_),
      horizontal_background_ok, cv::CMP_LE);
    cv::compare(
      vertical_reference, cv::Scalar(bilateral_background_max_),
      vertical_background_ok, cv::CMP_LE);
    cv::bitwise_and(
      vertical_stroke, horizontal_background_ok, vertical_stroke);
    cv::bitwise_and(
      horizontal_stroke, vertical_background_ok, horizontal_stroke);
    cv::Mat gate;
    cv::bitwise_or(vertical_stroke, horizontal_stroke, gate);
    cv::bitwise_and(candidate, gate, candidate);

    const int local_active_begin =
      rows.active_begin - rows.processing_begin;
    const int local_active_end = local_active_begin +
      (rows.active_end - rows.active_begin);
    return finalizeCandidate(
      candidate.rowRange(local_active_begin, local_active_end), ratio,
      work.rows, rows.active_begin);
  }

  cv::Mat finalizeCandidate(
    const cv::Mat & candidate, const double ratio,
    const int output_rows, const int output_y)
  {
    const int min_area = std::max(
      10, static_cast<int>(std::lround(min_area_ * ratio * ratio)));
    const int blob_area = std::max(
      50, static_cast<int>(std::lround(blob_area_ * ratio * ratio)));
    const int sliver_max_height = std::max(
      2, static_cast<int>(std::lround(sliver_max_height_ * ratio)));

    // Stage 3: shape filtering runs only on the active ground ROI.
    const int count = cv::connectedComponentsWithStats(
      candidate, labels_, stats_, centroids_, 8, CV_32S);
    kept_components_.assign(static_cast<std::size_t>(count), 0U);
    for (int index = 1; index < count; ++index) {
      const int area = stats_.at<int>(index, cv::CC_STAT_AREA);
      if (area < min_area) {
        continue;
      }
      const int box_y =
        output_y + stats_.at<int>(index, cv::CC_STAT_TOP);
      const int box_w = stats_.at<int>(index, cv::CC_STAT_WIDTH);
      const int box_h = stats_.at<int>(index, cv::CC_STAT_HEIGHT);
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
        static_cast<double>(output_rows) * sliver_top_ratio_)
      {
        continue;  // thin horizontal sliver near the top: far floor seam
      }
      kept_components_[static_cast<std::size_t>(index)] = 1U;
    }

    filtered_.create(candidate.size(), CV_8UC1);
    for (int row = 0; row < candidate.rows; ++row) {
      const int * labels = labels_.ptr<int>(row);
      std::uint8_t * output = filtered_.ptr<std::uint8_t>(row);
      for (int column = 0; column < candidate.cols; ++column) {
        output[column] = kept_components_[
          static_cast<std::size_t>(labels[column])] ? 255U : 0U;
      }
    }
    output_.create(output_rows, candidate.cols, CV_8UC1);
    output_.setTo(0);
    filtered_.copyTo(
      output_(cv::Rect(0, output_y, candidate.cols, candidate.rows)));
    return output_;
  }

#ifdef LANE_MASK_HAVE_OPENCV_CUDA
  void clearCudaFilters()
  {
    cuda_gaussian_filter_.release();
    cuda_tophat_filter_.release();
    cuda_erode_left_filter_.release();
    cuda_erode_right_filter_.release();
    cuda_erode_up_filter_.release();
    cuda_erode_down_filter_.release();
    cuda_cached_blur_kernel_ = 0;
    cuda_cached_tophat_kernel_ = 0;
    cuda_cached_bilateral_span_ = 0;
  }

  void ensureCudaFilters(
    const int blur_kernel, const int tophat_kernel,
    const int bilateral_span)
  {
    if (
      cuda_cached_blur_kernel_ == blur_kernel &&
      cuda_cached_tophat_kernel_ == tophat_kernel &&
      cuda_cached_bilateral_span_ == bilateral_span)
    {
      return;
    }

    clearCudaFilters();
    if (blur_kernel > 1) {
      cuda_gaussian_filter_ = cv::cuda::createGaussianFilter(
        CV_8UC1, CV_8UC1, {blur_kernel, blur_kernel}, 0.0);
    }

    const cv::Mat tophat_element = cv::getStructuringElement(
      cv::MORPH_RECT, {tophat_kernel, tophat_kernel});
    cuda_tophat_filter_ = cv::cuda::createMorphologyFilter(
      cv::MORPH_TOPHAT, CV_8UC1, tophat_element);

    const cv::Mat horizontal_element = cv::getStructuringElement(
      cv::MORPH_RECT, {bilateral_span, 1});
    const cv::Mat vertical_element = cv::getStructuringElement(
      cv::MORPH_RECT, {1, bilateral_span});

    cuda_erode_left_filter_ = cv::cuda::createMorphologyFilter(
      cv::MORPH_ERODE, CV_8UC1, horizontal_element,
      {bilateral_span - 1, 0});
    cuda_erode_right_filter_ = cv::cuda::createMorphologyFilter(
      cv::MORPH_ERODE, CV_8UC1, horizontal_element, {0, 0});
    cuda_erode_up_filter_ = cv::cuda::createMorphologyFilter(
      cv::MORPH_ERODE, CV_8UC1, vertical_element,
      {0, bilateral_span - 1});
    cuda_erode_down_filter_ = cv::cuda::createMorphologyFilter(
      cv::MORPH_ERODE, CV_8UC1, vertical_element, {0, 0});

    cuda_cached_blur_kernel_ = blur_kernel;
    cuda_cached_tophat_kernel_ = tophat_kernel;
    cuda_cached_bilateral_span_ = bilateral_span;
  }

  cv::Mat computeMaskCuda(const cv::Mat & luma)
  {
    const bool downscale =
      process_width_ > 0 && process_width_ < luma.cols;
    const int work_width = downscale ? process_width_ : luma.cols;
    const int work_height = downscale
      ? static_cast<int>(std::lround(
          static_cast<double>(luma.rows) * work_width / luma.cols))
      : luma.rows;
    const double ratio =
      static_cast<double>(work_width) /
      static_cast<double>(param_reference_width_);
    const int tophat_kernel = makeOdd(
      static_cast<int>(std::lround(tophat_kernel_ * ratio)), 3);
    const int bilateral_span = std::max(
      3, static_cast<int>(std::lround(bilateral_span_px_ * ratio)));
    const ProcessingRows rows = computeProcessingRows(
      work_height, tophat_kernel, bilateral_span);
    if (rows.active_begin >= rows.active_end) {
      output_.create(work_height, work_width, CV_8UC1);
      output_.setTo(0);
      return output_;
    }

    ensureCudaFilters(blur_kernel_, tophat_kernel, bilateral_span);
    cv::cuda::Stream & stream = *cuda_stream_;

    cuda_input_.upload(luma, stream);
    cv::cuda::GpuMat * work = &cuda_input_;
    if (downscale) {
      cv::cuda::resize(
        cuda_input_, cuda_work_, {work_width, work_height}, 0.0, 0.0,
        cv::INTER_AREA, stream);
      work = &cuda_work_;
    }
    cv::cuda::GpuMat work_region = work->rowRange(
      rows.processing_begin, rows.processing_end);

    cv::cuda::GpuMat * blurred = &work_region;
    if (cuda_gaussian_filter_) {
      cuda_gaussian_filter_->apply(work_region, cuda_blurred_, stream);
      blurred = &cuda_blurred_;
    }

    cuda_tophat_filter_->apply(*blurred, cuda_tophat_, stream);
    cv::cuda::threshold(
      cuda_tophat_, cuda_candidate_, tophat_threshold_, 255,
      cv::THRESH_BINARY, stream);

    cuda_erode_left_filter_->apply(*blurred, cuda_left_min_, stream);
    cuda_erode_right_filter_->apply(*blurred, cuda_right_min_, stream);
    cuda_erode_up_filter_->apply(*blurred, cuda_up_min_, stream);
    cuda_erode_down_filter_->apply(*blurred, cuda_down_min_, stream);

    cv::cuda::max(
      cuda_left_min_, cuda_right_min_, cuda_horizontal_reference_, stream);
    cv::cuda::max(
      cuda_up_min_, cuda_down_min_, cuda_vertical_reference_, stream);
    cv::cuda::subtract(
      *blurred, cuda_horizontal_reference_, cuda_horizontal_delta_,
      cv::noArray(), -1, stream);
    cv::cuda::subtract(
      *blurred, cuda_vertical_reference_, cuda_vertical_delta_,
      cv::noArray(), -1, stream);

    cv::cuda::compareWithScalar(
      cuda_horizontal_delta_, cv::Scalar(bilateral_contrast_threshold_),
      cuda_vertical_stroke_, cv::CMP_GE, stream);
    cv::cuda::compareWithScalar(
      cuda_vertical_delta_, cv::Scalar(bilateral_contrast_threshold_),
      cuda_horizontal_stroke_, cv::CMP_GE, stream);
    cv::cuda::compareWithScalar(
      cuda_horizontal_reference_, cv::Scalar(bilateral_background_max_),
      cuda_horizontal_background_ok_, cv::CMP_LE, stream);
    cv::cuda::compareWithScalar(
      cuda_vertical_reference_, cv::Scalar(bilateral_background_max_),
      cuda_vertical_background_ok_, cv::CMP_LE, stream);
    cv::cuda::bitwise_and(
      cuda_vertical_stroke_, cuda_horizontal_background_ok_,
      cuda_vertical_stroke_,
      cv::noArray(), stream);
    cv::cuda::bitwise_and(
      cuda_horizontal_stroke_, cuda_vertical_background_ok_,
      cuda_horizontal_stroke_,
      cv::noArray(), stream);
    cv::cuda::bitwise_or(
      cuda_vertical_stroke_, cuda_horizontal_stroke_, cuda_gate_,
      cv::noArray(), stream);
    cv::cuda::bitwise_and(
      cuda_candidate_, cuda_gate_, cuda_candidate_, cv::noArray(), stream);

    const int local_active_begin =
      rows.active_begin - rows.processing_begin;
    const int local_active_end = local_active_begin +
      (rows.active_end - rows.active_begin);
    cv::cuda::GpuMat active_candidate = cuda_candidate_.rowRange(
      local_active_begin, local_active_end);
    active_candidate.download(cuda_candidate_host_, stream);
    stream.waitForCompletion();
    return finalizeCandidate(
      cuda_candidate_host_, ratio, work_height, rows.active_begin);
  }
#endif

  void publishMono8(
    const sensor_msgs::msg::Image & source,
    const cv::Mat & mask)
  {
    const int width = mask.cols;
    const int height = mask.rows;
    auto output = std::make_unique<sensor_msgs::msg::Image>();
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
      "lane_mask: backend=%s, in=%.1f fps, out=%.1f fps, rejected=%u",
      cuda_active_ ? "cuda" : "cpu",
      static_cast<double>(received) / interval,
      static_cast<double>(processed) / interval,
      rejected);
  }

  rclcpp::Node & node_;

  std::string input_topic_;
  std::string mask_topic_;

  int process_width_{640};
  int param_reference_width_{960};
  bool use_cuda_{true};
  bool cuda_active_{false};

#ifdef LANE_MASK_HAVE_OPENCV_CUDA
  std::unique_ptr<cv::cuda::Stream> cuda_stream_;
  cv::Ptr<cv::cuda::Filter> cuda_gaussian_filter_;
  cv::Ptr<cv::cuda::Filter> cuda_tophat_filter_;
  cv::Ptr<cv::cuda::Filter> cuda_erode_left_filter_;
  cv::Ptr<cv::cuda::Filter> cuda_erode_right_filter_;
  cv::Ptr<cv::cuda::Filter> cuda_erode_up_filter_;
  cv::Ptr<cv::cuda::Filter> cuda_erode_down_filter_;
  cv::cuda::GpuMat cuda_input_;
  cv::cuda::GpuMat cuda_work_;
  cv::cuda::GpuMat cuda_blurred_;
  cv::cuda::GpuMat cuda_tophat_;
  cv::cuda::GpuMat cuda_candidate_;
  cv::cuda::GpuMat cuda_left_min_;
  cv::cuda::GpuMat cuda_right_min_;
  cv::cuda::GpuMat cuda_up_min_;
  cv::cuda::GpuMat cuda_down_min_;
  cv::cuda::GpuMat cuda_horizontal_reference_;
  cv::cuda::GpuMat cuda_vertical_reference_;
  cv::cuda::GpuMat cuda_horizontal_delta_;
  cv::cuda::GpuMat cuda_vertical_delta_;
  cv::cuda::GpuMat cuda_vertical_stroke_;
  cv::cuda::GpuMat cuda_horizontal_stroke_;
  cv::cuda::GpuMat cuda_horizontal_background_ok_;
  cv::cuda::GpuMat cuda_vertical_background_ok_;
  cv::cuda::GpuMat cuda_gate_;
  cv::Mat cuda_candidate_host_;
  int cuda_cached_blur_kernel_{0};
  int cuda_cached_tophat_kernel_{0};
  int cuda_cached_bilateral_span_{0};
#endif

  cv::Mat labels_;
  cv::Mat stats_;
  cv::Mat centroids_;
  cv::Mat filtered_;
  cv::Mat output_;
  std::vector<std::uint8_t> kept_components_;

  int tophat_kernel_{31};
  int tophat_threshold_{60};
  int blur_kernel_{5};

  int bilateral_contrast_threshold_{30};
  int bilateral_background_max_{90};
  int bilateral_span_px_{45};

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
  double roi_top_ratio_{0.50};
  double bottom_cut_ratio_{0.25};

  bool preview_enabled_{false};
  std::string preview_window_name_{"lane mask"};
  double status_log_interval_sec_{5.0};

  std::atomic<unsigned> received_{0U};
  std::atomic<unsigned> processed_{0U};
  std::atomic<unsigned> rejected_{0U};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
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
