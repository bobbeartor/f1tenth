#ifndef LINE_DETACTOR__TENSORRT_LANE_BACKEND_HPP_
#define LINE_DETACTOR__TENSORRT_LANE_BACKEND_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <functional>
#include <string>

namespace line_detactor
{

struct LaneInferenceTiming
{
  std::uint64_t preprocessing_nanoseconds{0U};
  std::uint64_t execution_nanoseconds{0U};
  std::uint64_t correction_nanoseconds{0U};
  std::uint64_t postprocessing_nanoseconds{0U};
  std::uint64_t label_export_nanoseconds{0U};
};

class TensorRtLaneBackend
{
public:
  TensorRtLaneBackend(
    const std::string & model_path,
    const std::string & engine_cache_path,
    const std::string & engine_precision,
    int input_width,
    int input_height,
    std::size_t workspace_size_bytes,
    float mask_threshold,
    float overlay_alpha,
    bool export_labels = false,
    float stop_line_mask_threshold = -1.0F,
    std::function<void(const std::string &)> progress = {});
  ~TensorRtLaneBackend();

  TensorRtLaneBackend(const TensorRtLaneBackend &) = delete;
  TensorRtLaneBackend & operator=(const TensorRtLaneBackend &) = delete;
  TensorRtLaneBackend(TensorRtLaneBackend &&) noexcept;
  TensorRtLaneBackend & operator=(TensorRtLaneBackend &&) noexcept;

  LaneInferenceTiming infer_bgr(
    const std::uint8_t * bgr,
    std::size_t data_size,
    std::size_t source_stride);
  LaneInferenceTiming infer_device_bgr(
    const std::uint8_t * device_bgr,
    std::size_t device_stride);

  const std::uint8_t * preview_bgr_data() const noexcept;
  const std::uint8_t * label_data() const noexcept;
  const std::uint8_t * stop_line_mask_data() const noexcept;
  int input_width() const noexcept;
  int input_height() const noexcept;
  const std::string & engine_cache_path() const noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace line_detactor

#endif  // LINE_DETACTOR__TENSORRT_LANE_BACKEND_HPP_
