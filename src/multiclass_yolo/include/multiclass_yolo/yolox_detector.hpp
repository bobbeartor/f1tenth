#ifndef MULTICLASS_YOLO__YOLOX_DETECTOR_HPP_
#define MULTICLASS_YOLO__YOLOX_DETECTOR_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <functional>
#include <string>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/dnn.hpp"

namespace multiclass_yolo
{

class TensorRtYoloxBackend;

struct Detection
{
  cv::Rect2f box;
  float score;
  int class_id;
};

struct YoloxStageTiming
{
  std::uint64_t preprocessing_nanoseconds{0U};
  std::uint64_t input_transfer_nanoseconds{0U};
  std::uint64_t forward_nanoseconds{0U};
  std::uint64_t output_transfer_nanoseconds{0U};
  std::uint64_t postprocessing_nanoseconds{0U};
};

struct YoloxDetectionResult
{
  std::vector<Detection> detections;
  YoloxStageTiming timing;
};

class YoloxDetector
{
public:
  YoloxDetector(
    const std::string & model_path,
    const std::string & inference_backend,
    const std::string & engine_cache_path,
    const std::string & engine_precision,
    int input_width,
    int input_height,
    float score_threshold,
    float nms_threshold,
    std::size_t tensorrt_workspace_size_bytes,
    std::function<void(const std::string &)> progress = {},
    std::size_t max_detections = 0);
  ~YoloxDetector();

  YoloxDetector(const YoloxDetector &) = delete;
  YoloxDetector & operator=(const YoloxDetector &) = delete;

  YoloxDetectionResult detect(const cv::Mat & bgr_image, bool rgb = false);
  // Detect within the ROI and return boxes in full-image coordinates.
  YoloxDetectionResult detect(const cv::Mat & bgr_image, const cv::Rect & roi, bool rgb = false);
  YoloxDetectionResult detect_nv12(
    const std::uint8_t * nv12,
    std::size_t data_size,
    std::size_t source_stride,
    int source_width,
    int source_height,
    const cv::Rect & roi);
  void set_thresholds(const std::array<float, 4> & values);
  bool supports_nv12_input() const noexcept;
  void draw(
    cv::Mat & bgr_image,
    const std::vector<Detection> & detections) const;
  const std::string & backend_name() const noexcept;

private:
  YoloxDetectionResult decode_output(
    const float * rows,
    int row_count,
    int column_count,
    float ratio,
    int image_width,
    int image_height,
    int image_origin_x,
    int image_origin_y,
    YoloxStageTiming timing) const;

  cv::dnn::Net network_;
  std::unique_ptr<TensorRtYoloxBackend> tensorrt_backend_;
  std::vector<float> tensorrt_output_;
  int input_width_;
  int input_height_;
  std::array<float, 4> thresholds_;
  float nms_threshold_;
  std::size_t max_detections_;
  std::string backend_name_;
};

}  // namespace multiclass_yolo

#endif  // MULTICLASS_YOLO__YOLOX_DETECTOR_HPP_
