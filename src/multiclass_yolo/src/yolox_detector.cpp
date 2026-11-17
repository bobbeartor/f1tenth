#include "multiclass_yolo/yolox_detector.hpp"
#include "multiclass_yolo/postprocess.hpp"
#include "yolox_geometry.hpp"

#include "multiclass_yolo/tensorrt_yolox_backend.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "opencv2/imgproc.hpp"

namespace multiclass_yolo
{

namespace
{

std::string uppercase(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](const unsigned char character) {
      return static_cast<char>(std::toupper(character));
    });
  return value;
}

std::uint64_t elapsed_nanoseconds(
  const std::chrono::steady_clock::time_point started_at,
  const std::chrono::steady_clock::time_point finished_at)
{
  const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
    finished_at - started_at).count();
  return static_cast<std::uint64_t>(std::max<std::int64_t>(0, elapsed));
}

}  // namespace

YoloxDetector::YoloxDetector(
  const std::string & model_path,
  const std::string & inference_backend,
  const std::string & engine_cache_path,
  const std::string & engine_precision,
  const int input_width,
  const int input_height,
  const float score_threshold,
  const float nms_threshold,
  const std::size_t tensorrt_workspace_size_bytes,
  std::function<void(const std::string &)> progress,
  const std::size_t max_detections)
: input_width_(input_width),
  input_height_(input_height),
  nms_threshold_(nms_threshold),
  max_detections_(max_detections)
{
  if (input_width_ <= 0 || input_height_ <= 0) {
    throw std::invalid_argument("model input dimensions must be positive");
  }
  if (
    !std::isfinite(score_threshold) || score_threshold < 0.0F ||
    score_threshold > 1.0F)
  {
    throw std::invalid_argument("score_threshold must be in [0, 1]");
  }
  if (
    !std::isfinite(nms_threshold_) || nms_threshold_ < 0.0F ||
    nms_threshold_ > 1.0F)
  {
    throw std::invalid_argument("nms_threshold must be in [0, 1]");
  }

  thresholds_.fill(score_threshold);
  const auto normalized_backend = uppercase(inference_backend);
  const auto normalized_precision = uppercase(engine_precision);
  if (
    normalized_precision != "FP32" && normalized_precision != "FP16" &&
    normalized_precision != "INT8")
  {
    throw std::invalid_argument(
            "engine_precision must be fp32, fp16, or int8");
  }
  const auto precision_name = normalized_precision == "FP32" ? "fp32" :
    normalized_precision == "FP16" ? "fp16" : "int8";
  if (normalized_backend == "TENSORRT") {
    tensorrt_backend_ = std::make_unique<TensorRtYoloxBackend>(
      model_path, engine_cache_path, precision_name, input_width_, input_height_,
      tensorrt_workspace_size_bytes, std::move(progress));
    tensorrt_output_.resize(
      static_cast<std::size_t>(tensorrt_backend_->output_row_count()) *
      static_cast<std::size_t>(tensorrt_backend_->output_column_count()));
    backend_name_ = "TensorRT " + normalized_precision;
  } else if (normalized_backend == "CPU") {
    if (normalized_precision != "FP32") {
      throw std::invalid_argument(
              "CPU backend supports only engine_precision=fp32");
    }
    network_ = cv::dnn::readNetFromONNX(model_path);
    if (network_.empty()) {
      throw std::runtime_error("OpenCV could not load the ONNX model");
    }
    network_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    network_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    backend_name_ = "OpenCV DNN CPU FP32";
  } else {
    throw std::invalid_argument("inference_backend must be TENSORRT or CPU");
  }
}

YoloxDetector::~YoloxDetector() = default;

YoloxDetectionResult YoloxDetector::detect(
  const cv::Mat & bgr_image, const bool rgb)
{
  return detect(bgr_image, cv::Rect(0, 0, bgr_image.cols, bgr_image.rows), rgb);
}

YoloxDetectionResult YoloxDetector::detect(
  const cv::Mat & bgr_image, const cv::Rect & roi, const bool rgb)
{
  if (bgr_image.empty()) {
    throw std::invalid_argument("cannot run detection on an empty image");
  }
  if (bgr_image.type() != CV_8UC3) {
    throw std::invalid_argument("YOLOX input must be an 8-bit three-channel image");
  }

  if (
    roi.x < 0 || roi.y < 0 || roi.width <= 0 || roi.height <= 0 ||
    roi.x > bgr_image.cols - roi.width || roi.y > bgr_image.rows - roi.height)
  {
    throw std::invalid_argument("inference ROI is outside the image");
  }

  YoloxStageTiming stage_timing;
  const auto preprocessing_started_at = std::chrono::steady_clock::now();
  const float ratio = std::min(
    static_cast<float>(input_height_) /
    static_cast<float>(roi.height),
    static_cast<float>(input_width_) /
    static_cast<float>(roi.width));
  const int resized_width = std::max(
    1, static_cast<int>(static_cast<float>(roi.width) * ratio));
  const int resized_height = std::max(
    1, static_cast<int>(static_cast<float>(roi.height) * ratio));

  cv::Mat resized;
  cv::resize(
    bgr_image(roi), resized, cv::Size(resized_width, resized_height),
    0.0, 0.0, cv::INTER_LINEAR);

  // Bundled metadata: BGR 0..255 FP32, top-left letterbox, pad=114.
  // rgb=true swaps RGB source channels back to the required BGR tensor.
  cv::Mat padded(
    input_height_, input_width_, CV_8UC3, cv::Scalar(114, 114, 114));
  resized.copyTo(padded(cv::Rect(0, 0, resized.cols, resized.rows)));

  cv::Mat blob = cv::dnn::blobFromImage(
    padded,
    1.0,
    cv::Size(input_width_, input_height_),
    cv::Scalar(),
    rgb,
    false,
    CV_32F);
  if (!tensorrt_backend_) {
    network_.setInput(blob);
  }
  const auto preprocessing_finished_at = std::chrono::steady_clock::now();
  stage_timing.preprocessing_nanoseconds = elapsed_nanoseconds(
    preprocessing_started_at, preprocessing_finished_at);

  int row_count = 0;
  int column_count = 0;
  const float * rows = nullptr;
  cv::Mat output;
  const auto forward_started_at = preprocessing_finished_at;
  if (tensorrt_backend_) {
    if (!blob.isContinuous() || blob.type() != CV_32F) {
      throw std::runtime_error("YOLOX FP32 input blob is not contiguous");
    }
    const auto timing = tensorrt_backend_->infer(
      blob.ptr<float>(), blob.total(), tensorrt_output_.data(),
      tensorrt_output_.size());
    stage_timing.input_transfer_nanoseconds =
      timing.input_transfer_nanoseconds;
    stage_timing.forward_nanoseconds = timing.execution_nanoseconds;
    stage_timing.output_transfer_nanoseconds =
      timing.output_transfer_nanoseconds;
    row_count = tensorrt_backend_->output_row_count();
    column_count = tensorrt_backend_->output_column_count();
    rows = tensorrt_output_.data();
  } else {
    output = network_.forward();
    const auto forward_finished_at = std::chrono::steady_clock::now();
    stage_timing.forward_nanoseconds = elapsed_nanoseconds(
      forward_started_at, forward_finished_at);

    if (output.dims == 3 && output.size[0] == 1) {
      row_count = output.size[1];
      column_count = output.size[2];
    } else if (output.dims == 2) {
      row_count = output.rows;
      column_count = output.cols;
    } else {
      throw std::runtime_error(
              "unexpected YOLOX output rank; expected [1, N, 9] or [N, 9]");
    }
    const auto expected_rows = yolox_output_rows(input_width_, input_height_);
    if (row_count != expected_rows || column_count != 9 || output.type() != CV_32F) {
      throw std::runtime_error(
              "unexpected YOLOX output; expected FP32 with " +
              std::to_string(expected_rows) + " rows and 9 columns");
    }
    if (!output.isContinuous()) {
      output = output.clone();
    }
    rows = output.ptr<float>();
  }

  return decode_output(
    rows, row_count, column_count, ratio, roi.width, roi.height,
    roi.x, roi.y, stage_timing);
}

YoloxDetectionResult YoloxDetector::detect_nv12(
  const std::uint8_t * nv12,
  const std::size_t data_size,
  const std::size_t source_stride,
  const int source_width,
  const int source_height,
  const cv::Rect & roi)
{
  if (!tensorrt_backend_) {
    throw std::logic_error(
            "direct NV12 input requires the TensorRT backend");
  }

  const auto timing = tensorrt_backend_->infer_nv12(
    nv12, data_size, source_stride, source_width, source_height,
    roi.x, roi.y, roi.width, roi.height,
    tensorrt_output_.data(), tensorrt_output_.size());
  YoloxStageTiming stage_timing;
  stage_timing.preprocessing_nanoseconds =
    timing.preprocessing_nanoseconds;
  stage_timing.input_transfer_nanoseconds =
    timing.input_transfer_nanoseconds;
  stage_timing.forward_nanoseconds = timing.execution_nanoseconds;
  stage_timing.output_transfer_nanoseconds =
    timing.output_transfer_nanoseconds;

  const float ratio = std::min(
    static_cast<float>(input_height_) / static_cast<float>(roi.height),
    static_cast<float>(input_width_) / static_cast<float>(roi.width));
  // The CUDA kernel crops and resizes the ROI, adds 114-valued top-left
  // letterbox padding, and writes BGR FP32 NCHW directly to TensorRT input.
  return decode_output(
    tensorrt_output_.data(), tensorrt_backend_->output_row_count(),
    tensorrt_backend_->output_column_count(), ratio, roi.width, roi.height,
    roi.x, roi.y, stage_timing);
}

bool YoloxDetector::supports_nv12_input() const noexcept
{
  return tensorrt_backend_ != nullptr;
}

YoloxDetectionResult YoloxDetector::decode_output(
  const float * rows,
  const int row_count,
  const int column_count,
  const float ratio,
  const int image_width,
  const int image_height,
  const int image_origin_x,
  const int image_origin_y,
  YoloxStageTiming timing) const
{
  if (
    rows == nullptr || row_count <= 0 || column_count != 9 ||
    !std::isfinite(ratio) || ratio <= 0.0F ||
    image_width <= 0 || image_height <= 0)
  {
    throw std::invalid_argument("invalid YOLOX decoded output metadata");
  }

  const auto postprocessing_started_at = std::chrono::steady_clock::now();
  YoloxDetectionResult result;
  result.timing = timing;
  result.detections = decode_detections(rows, row_count, column_count, ratio,
    image_width, image_height, image_origin_x, image_origin_y, thresholds_, nms_threshold_,
    max_detections_);
  const auto postprocessing_finished_at = std::chrono::steady_clock::now();
  result.timing.postprocessing_nanoseconds = elapsed_nanoseconds(
    postprocessing_started_at, postprocessing_finished_at);
  return result;
}

const std::string & YoloxDetector::backend_name() const noexcept
{
  return backend_name_;
}

void YoloxDetector::draw(
  cv::Mat & bgr_image,
  const std::vector<Detection> & detections) const
{
  const cv::Scalar colors[] = {{0, 160, 255}, {255, 160, 0}, {80, 220, 80}, {220, 80, 220}};
  const char * names[] = {"con", "truck", "sedan", "traffic_light"};
  for (const auto & detection : detections) {
    const auto box_color = colors[detection.class_id];
    const int left = static_cast<int>(std::lround(detection.box.x));
    const int top = static_cast<int>(std::lround(detection.box.y));
    const int right = static_cast<int>(
      std::lround(detection.box.x + detection.box.width));
    const int bottom = static_cast<int>(
      std::lround(detection.box.y + detection.box.height));
    cv::rectangle(
      bgr_image, cv::Point(left, top), cv::Point(right, bottom),
      box_color, 2, cv::LINE_AA);

    const std::string label = cv::format(
      "%s %.2f", names[detection.class_id], detection.score);
    int baseline = 0;
    const auto text_size = cv::getTextSize(
      label, cv::FONT_HERSHEY_SIMPLEX, 0.45, 1, &baseline);
    const int label_y = std::max(text_size.height + 3, top);
    cv::rectangle(
      bgr_image,
      cv::Point(left, label_y - text_size.height - 3),
      cv::Point(left + text_size.width + 4, label_y + baseline),
      box_color,
      cv::FILLED);
    cv::putText(
      bgr_image,
      label,
      cv::Point(left + 2, label_y - 2),
      cv::FONT_HERSHEY_SIMPLEX,
      0.45,
      cv::Scalar(255, 255, 255),
      1,
      cv::LINE_AA);
  }
}

void YoloxDetector::set_thresholds(const std::array<float, 4> & values)
{
  for (float v : values) {
    if (!std::isfinite(v) || v < 0 || v > 1) {
      throw std::invalid_argument("class thresholds must be in [0,1]");
    }
  }
  thresholds_ = values;
}

}  // namespace multiclass_yolo
