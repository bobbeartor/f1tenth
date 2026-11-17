#include "cuda_lane_pipeline.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace line_detactor
{
namespace
{

__device__ __forceinline__ std::uint8_t blended_channel(
  const std::uint8_t source,
  const float target,
  const float alpha)
{
  const float value =
    static_cast<float>(source) * (1.0F - alpha) + target * alpha;
  return static_cast<std::uint8_t>(fminf(fmaxf(value, 0.0F), 255.0F));
}

__global__ void bgr_to_rgb_nchw_kernel(
  const std::uint8_t * bgr,
  float * rgb,
  const int width,
  const int height)
{
  const int x = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const int y = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
  if (x >= width || y >= height) {
    return;
  }

  const std::size_t pixel =
    static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
    static_cast<std::size_t>(x);
  const std::size_t bgr_index = pixel * 3U;
  const std::size_t plane =
    static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  constexpr float scale = 1.0F / 255.0F;
  rgb[pixel] = static_cast<float>(bgr[bgr_index + 2U]) * scale;
  rgb[plane + pixel] = static_cast<float>(bgr[bgr_index + 1U]) * scale;
  rgb[2U * plane + pixel] = static_cast<float>(bgr[bgr_index]) * scale;
}

__global__ void lane_labels_kernel(
  const float * logits, std::uint8_t * labels, const int width, const int height,
  const float threshold, const float stop_threshold)
{
  const int i = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  if (i >= width * height) {return;}
  const float left = logits[i];
  const float right = logits[width * height + i];
  labels[i] = left >= threshold && (!(right >= threshold) || left >= right) ? 1U :
    (right >= threshold ? 2U : 0U);
  // Keep stop-line membership independent: crossing pixels can also be lanes.
  labels[width * height + i] = logits[2 * width * height + i] >= stop_threshold ? 255U : 0U;
}

__global__ void lane_overlay_kernel(
  const std::uint8_t * bgr,
  const float * logits,
  std::uint8_t * preview,
  const int width,
  const int height,
  const float logit_threshold,
  const float stop_logit_threshold,
  const float alpha)
{
  const int x = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const int y = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
  if (x >= width || y >= height) {
    return;
  }

  const std::size_t pixel =
    static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
    static_cast<std::size_t>(x);
  const std::size_t plane =
    static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  const std::size_t bgr_index = pixel * 3U;
  const float left_logit = logits[pixel];
  const float right_logit = logits[plane + pixel];
  const bool left = left_logit >= logit_threshold;
  const bool right = right_logit >= logit_threshold;
  const bool stop_line = logits[2U * plane + pixel] >= stop_logit_threshold;

  float blue = static_cast<float>(bgr[bgr_index]);
  float green = static_cast<float>(bgr[bgr_index + 1U]);
  float red = static_cast<float>(bgr[bgr_index + 2U]);
  if (stop_line) {
    blue = 0.0F;
    green = 255.0F;
    red = 0.0F;
  } else if (left && (!right || left_logit >= right_logit)) {
    blue = 255.0F;
    green = 0.0F;
    red = 0.0F;
  } else if (right) {
    blue = 0.0F;
    green = 0.0F;
    red = 255.0F;
  } else {
    preview[bgr_index] = bgr[bgr_index];
    preview[bgr_index + 1U] = bgr[bgr_index + 1U];
    preview[bgr_index + 2U] = bgr[bgr_index + 2U];
    return;
  }

  preview[bgr_index] = blended_channel(bgr[bgr_index], blue, alpha);
  preview[bgr_index + 1U] = blended_channel(
    bgr[bgr_index + 1U], green, alpha);
  preview[bgr_index + 2U] = blended_channel(
    bgr[bgr_index + 2U], red, alpha);
}

dim3 grid_for(const int width, const int height)
{
  constexpr unsigned int block_width = 16U;
  constexpr unsigned int block_height = 16U;
  return dim3(
    static_cast<unsigned int>(
      (width + static_cast<int>(block_width) - 1) /
      static_cast<int>(block_width)),
    static_cast<unsigned int>(
      (height + static_cast<int>(block_height) - 1) /
      static_cast<int>(block_height)));
}

}  // namespace

cudaError_t launch_bgr_to_rgb_nchw(
  const std::uint8_t * device_bgr,
  float * device_rgb,
  const int width,
  const int height,
  const cudaStream_t stream) noexcept
{
  const dim3 block(16U, 16U);
  bgr_to_rgb_nchw_kernel<<<grid_for(width, height), block, 0U, stream>>>(
    device_bgr, device_rgb, width, height);
  return cudaGetLastError();
}

namespace
{
float threshold_logit(const float threshold)
{
  if (threshold <= 0.0F) {
    return -std::numeric_limits<float>::infinity();
  }
  if (threshold >= 1.0F) {
    return std::numeric_limits<float>::infinity();
  }
  return logf(threshold / (1.0F - threshold));
}
}  // namespace

cudaError_t launch_lane_labels(
  const float * device_logits, std::uint8_t * device_labels, const int width,
  const int height, const float mask_threshold, const float stop_line_mask_threshold,
  const cudaStream_t stream) noexcept
{
  lane_labels_kernel<<<(width * height + 255) / 256, 256, 0U, stream>>>(
    device_logits, device_labels, width, height, threshold_logit(mask_threshold),
    threshold_logit(stop_line_mask_threshold));
  return cudaGetLastError();
}

cudaError_t launch_lane_overlay(
  const std::uint8_t * device_bgr,
  const float * device_logits,
  std::uint8_t * device_preview_bgr,
  const int width,
  const int height,
  const float mask_threshold,
  const float stop_line_mask_threshold,
  const float overlay_alpha,
  const cudaStream_t stream) noexcept
{
  const float logit_threshold = threshold_logit(mask_threshold);
  const dim3 block(16U, 16U);
  lane_overlay_kernel<<<grid_for(width, height), block, 0U, stream>>>(
    device_bgr, device_logits, device_preview_bgr, width, height,
    logit_threshold, threshold_logit(stop_line_mask_threshold), overlay_alpha);
  return cudaGetLastError();
}

}  // namespace line_detactor
