#ifndef TRAFFIC_DETECTION_TEST__TENSORRT_YOLOX_BACKEND_HPP_
#define TRAFFIC_DETECTION_TEST__TENSORRT_YOLOX_BACKEND_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <functional>
#include <string>

namespace traffic_detection_test
{

struct TensorRtInferenceTiming
{
  std::uint64_t input_transfer_nanoseconds{0U};
  std::uint64_t preprocessing_nanoseconds{0U};
  std::uint64_t execution_nanoseconds{0U};
  std::uint64_t output_transfer_nanoseconds{0U};
};

class TensorRtYoloxBackend
{
public:
  TensorRtYoloxBackend(
    const std::string & model_path,
    const std::string & engine_cache_path,
    const std::string & engine_precision,
    int input_width,
    int input_height,
    std::size_t workspace_size_bytes,
    std::function<void(const std::string &)> progress = {});
  ~TensorRtYoloxBackend();

  TensorRtYoloxBackend(const TensorRtYoloxBackend &) = delete;
  TensorRtYoloxBackend & operator=(const TensorRtYoloxBackend &) = delete;
  TensorRtYoloxBackend(TensorRtYoloxBackend &&) noexcept;
  TensorRtYoloxBackend & operator=(TensorRtYoloxBackend &&) noexcept;

  TensorRtInferenceTiming infer(
    const float * input,
    std::size_t input_element_count,
    float * output,
    std::size_t output_element_count);
  TensorRtInferenceTiming infer_nv12(
    const std::uint8_t * nv12,
    std::size_t data_size,
    std::size_t source_stride,
    int source_width,
    int source_height,
    int roi_left,
    int roi_top,
    int roi_width,
    int roi_height,
    float * output,
    std::size_t output_element_count);

  int output_row_count() const noexcept;
  int output_column_count() const noexcept;
  const std::string & engine_cache_path() const noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace traffic_detection_test

#endif  // TRAFFIC_DETECTION_TEST__TENSORRT_YOLOX_BACKEND_HPP_
