#ifndef LINE_DETACTOR__CUDA_LANE_PIPELINE_HPP_
#define LINE_DETACTOR__CUDA_LANE_PIPELINE_HPP_

#include <cstdint>

#include <cuda_runtime_api.h>

namespace line_detactor
{

cudaError_t launch_bgr_to_rgb_nchw(
  const std::uint8_t * device_bgr,
  float * device_rgb,
  int width,
  int height,
  cudaStream_t stream) noexcept;

cudaError_t launch_lane_labels(
  const float * device_logits,
  // Two uint8 planes: legacy left/right labels, then independent 0/255 stop mask.
  std::uint8_t * device_labels,
  int width,
  int height,
  float mask_threshold,
  float stop_line_mask_threshold,
  cudaStream_t stream) noexcept;

cudaError_t launch_lane_overlay(
  const std::uint8_t * device_bgr,
  const float * device_logits,
  std::uint8_t * device_preview_bgr,
  int width,
  int height,
  float mask_threshold,
  float stop_line_mask_threshold,
  float overlay_alpha,
  cudaStream_t stream) noexcept;

}  // namespace line_detactor

#endif  // LINE_DETACTOR__CUDA_LANE_PIPELINE_HPP_
