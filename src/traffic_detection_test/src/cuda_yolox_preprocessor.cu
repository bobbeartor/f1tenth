#include "cuda_yolox_preprocessor.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace traffic_detection_test
{

namespace
{

__device__ __forceinline__ int clamp_u8(const int value)
{
  return value < 0 ? 0 : value > 255 ? 255 : value;
}

struct BgrPixel
{
  float blue;
  float green;
  float red;
};

__device__ __forceinline__ BgrPixel read_nv12_pixel(
  const std::uint8_t * nv12,
  const std::size_t source_stride,
  const int source_height,
  const int x,
  const int y)
{
  const int y_value = static_cast<int>(
    nv12[static_cast<std::size_t>(y) * source_stride +
    static_cast<std::size_t>(x)]);
  const std::size_t uv_offset =
    source_stride * static_cast<std::size_t>(source_height) +
    static_cast<std::size_t>(y / 2) * source_stride +
    static_cast<std::size_t>((x / 2) * 2);
  const int u_value = static_cast<int>(nv12[uv_offset]) - 128;
  const int v_value = static_cast<int>(nv12[uv_offset + 1U]) - 128;

  // Match OpenCV's limited-range BT.601 NV12 conversion before converting
  // the uint8 BGR result to the model's unnormalized FP32 tensor.
  constexpr int shift = 20;
  constexpr int rounding = 1 << (shift - 1);
  const int luminance = (y_value > 16 ? y_value - 16 : 0) * 1220542;
  return BgrPixel{
    static_cast<float>(clamp_u8(
      (luminance + 2116026 * u_value + rounding) >> shift)),
    static_cast<float>(clamp_u8(
      (luminance - 409993 * u_value - 852492 * v_value + rounding) >>
      shift)),
    static_cast<float>(clamp_u8(
      (luminance + 1673527 * v_value + rounding) >> shift))};
}

__device__ __forceinline__ float bilinear_channel(
  const float top_left,
  const float top_right,
  const float bottom_left,
  const float bottom_right,
  const float dx,
  const float dy)
{
  const float top = top_left + (top_right - top_left) * dx;
  const float bottom = bottom_left + (bottom_right - bottom_left) * dx;
  return floorf(top + (bottom - top) * dy + 0.5F);
}

__device__ __forceinline__ BgrPixel sample_nv12_roi(
  const std::uint8_t * nv12,
  const std::size_t source_stride,
  const int source_height,
  const int roi_left,
  const int roi_top,
  const int roi_width,
  const int roi_height,
  const int resized_width,
  const int resized_height,
  const int output_x,
  const int output_y)
{
  if (roi_width == resized_width && roi_height == resized_height) {
    return read_nv12_pixel(
      nv12, source_stride, source_height,
      roi_left + output_x, roi_top + output_y);
  }

  float local_x =
    (static_cast<float>(output_x) + 0.5F) *
    static_cast<float>(roi_width) / static_cast<float>(resized_width) - 0.5F;
  float local_y =
    (static_cast<float>(output_y) + 0.5F) *
    static_cast<float>(roi_height) /
    static_cast<float>(resized_height) - 0.5F;
  local_x = fminf(fmaxf(local_x, 0.0F), static_cast<float>(roi_width - 1));
  local_y = fminf(fmaxf(local_y, 0.0F), static_cast<float>(roi_height - 1));

  const int x0_local = static_cast<int>(floorf(local_x));
  const int y0_local = static_cast<int>(floorf(local_y));
  const int x1_local = x0_local + 1 < roi_width ? x0_local + 1 : x0_local;
  const int y1_local = y0_local + 1 < roi_height ? y0_local + 1 : y0_local;
  const float dx = local_x - static_cast<float>(x0_local);
  const float dy = local_y - static_cast<float>(y0_local);

  const BgrPixel top_left = read_nv12_pixel(
    nv12, source_stride, source_height,
    roi_left + x0_local, roi_top + y0_local);
  const BgrPixel top_right = read_nv12_pixel(
    nv12, source_stride, source_height,
    roi_left + x1_local, roi_top + y0_local);
  const BgrPixel bottom_left = read_nv12_pixel(
    nv12, source_stride, source_height,
    roi_left + x0_local, roi_top + y1_local);
  const BgrPixel bottom_right = read_nv12_pixel(
    nv12, source_stride, source_height,
    roi_left + x1_local, roi_top + y1_local);

  return BgrPixel{
    bilinear_channel(
      top_left.blue, top_right.blue, bottom_left.blue, bottom_right.blue,
      dx, dy),
    bilinear_channel(
      top_left.green, top_right.green, bottom_left.green, bottom_right.green,
      dx, dy),
    bilinear_channel(
      top_left.red, top_right.red, bottom_left.red, bottom_right.red,
      dx, dy)};
}

__global__ void nv12_roi_to_bgr_nchw_kernel(
  const std::uint8_t * nv12,
  const std::size_t source_stride,
  const int source_height,
  const int roi_left,
  const int roi_top,
  const int roi_width,
  const int roi_height,
  const int resized_width,
  const int resized_height,
  float * output,
  const int output_width,
  const int output_height)
{
  const int x = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const int y = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
  if (x >= output_width || y >= output_height) {
    return;
  }

  const std::size_t plane_size =
    static_cast<std::size_t>(output_width) *
    static_cast<std::size_t>(output_height);
  const std::size_t output_index =
    static_cast<std::size_t>(y) * static_cast<std::size_t>(output_width) +
    static_cast<std::size_t>(x);

  float blue = 114.0F;
  float green = 114.0F;
  float red = 114.0F;
  if (x < resized_width && y < resized_height) {
    const BgrPixel pixel = sample_nv12_roi(
      nv12, source_stride, source_height, roi_left, roi_top, roi_width,
      roi_height, resized_width, resized_height, x, y);
    blue = pixel.blue;
    green = pixel.green;
    red = pixel.red;
  }

  output[output_index] = blue;
  output[plane_size + output_index] = green;
  output[2U * plane_size + output_index] = red;
}

}  // namespace

cudaError_t launch_nv12_roi_to_bgr_nchw(
  const std::uint8_t * device_nv12,
  const std::size_t source_stride,
  const int source_height,
  const int roi_left,
  const int roi_top,
  const int roi_width,
  const int roi_height,
  float * device_output,
  const int output_width,
  const int output_height,
  const cudaStream_t stream) noexcept
{
  constexpr unsigned int block_width = 16U;
  constexpr unsigned int block_height = 16U;
  const dim3 block(block_width, block_height);
  const dim3 grid(
    static_cast<unsigned int>(
      (output_width + static_cast<int>(block_width) - 1) /
      static_cast<int>(block_width)),
    static_cast<unsigned int>(
      (output_height + static_cast<int>(block_height) - 1) /
      static_cast<int>(block_height)));
  const float width_ratio =
    static_cast<float>(output_width) / static_cast<float>(roi_width);
  const float height_ratio =
    static_cast<float>(output_height) / static_cast<float>(roi_height);
  const float ratio = width_ratio < height_ratio ? width_ratio : height_ratio;
  const int calculated_width = static_cast<int>(
    static_cast<float>(roi_width) * ratio);
  const int calculated_height = static_cast<int>(
    static_cast<float>(roi_height) * ratio);
  const int resized_width = calculated_width < 1 ? 1 :
    calculated_width > output_width ? output_width : calculated_width;
  const int resized_height = calculated_height < 1 ? 1 :
    calculated_height > output_height ? output_height : calculated_height;
  nv12_roi_to_bgr_nchw_kernel<<<grid, block, 0U, stream>>>(
    device_nv12, source_stride, source_height, roi_left, roi_top, roi_width,
    roi_height, resized_width, resized_height, device_output, output_width,
    output_height);
  return cudaGetLastError();
}

}  // namespace traffic_detection_test
