#include "bev_processor/cuda_bev_processor.hpp"

#include <cuda_runtime.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>

namespace bev_processor
{

namespace
{

void checkCuda(const cudaError_t result, const char * operation)
{
  if (result != cudaSuccess) {
    throw std::runtime_error(
            std::string(operation) + ": " + cudaGetErrorString(result));
  }
}

__device__ float clampFloat(
  const float value,
  const float minimum,
  const float maximum)
{
  return fminf(maximum, fmaxf(minimum, value));
}

__device__ float cubicWeight(const float distance)
{
  const float value = fabsf(distance);
  if (value <= 1.0F) {
    return
      (1.5F * value - 2.5F) * value * value + 1.0F;
  }
  if (value < 2.0F) {
    return
      ((-0.5F * value + 2.5F) * value - 4.0F) * value + 2.0F;
  }
  return 0.0F;
}

__device__ float samplePlaneBicubic(
  const std::uint8_t * plane,
  const int stride,
  const int width,
  const int height,
  const float x,
  const float y)
{
  const float clamped_x = clampFloat(x, 0.0F, width - 1.0F);
  const float clamped_y = clampFloat(y, 0.0F, height - 1.0F);
  const int base_x = static_cast<int>(floorf(clamped_x));
  const int base_y = static_cast<int>(floorf(clamped_y));
  float weighted_sum = 0.0F;
  float total_weight = 0.0F;

#pragma unroll
  for (int offset_y = -1; offset_y <= 2; ++offset_y) {
    const int sample_y = max(0, min(height - 1, base_y + offset_y));
    const float weight_y = cubicWeight(
      clamped_y - static_cast<float>(base_y + offset_y));
#pragma unroll
    for (int offset_x = -1; offset_x <= 2; ++offset_x) {
      const int sample_x = max(0, min(width - 1, base_x + offset_x));
      const float weight_x = cubicWeight(
        clamped_x - static_cast<float>(base_x + offset_x));
      const float weight = weight_x * weight_y;
      weighted_sum +=
        static_cast<float>(plane[sample_y * stride + sample_x]) * weight;
      total_weight += weight;
    }
  }

  if (fabsf(total_weight) <= 1.0e-6F) {
    return static_cast<float>(plane[base_y * stride + base_x]);
  }
  return clampFloat(weighted_sum / total_weight, 0.0F, 255.0F);
}

__global__ void yPlaneToBinaryBevKernel(
  const std::uint8_t * y_plane,
  const int input_width,
  const int input_height,
  const float * map_x,
  const float * map_y,
  const int output_width,
  const int output_height,
  const float contrast_gain,
  const float contrast_center,
  const float brightness_offset,
  const int binary_threshold,
  std::uint8_t * output_mask)
{
  const int output_x = blockIdx.x * blockDim.x + threadIdx.x;
  const int output_y = blockIdx.y * blockDim.y + threadIdx.y;
  if (output_x >= output_width || output_y >= output_height) {
    return;
  }

  const int output_index = output_y * output_width + output_x;
  const float source_x = map_x[output_index];
  const float source_y = map_y[output_index];
  if (
    source_x < 0.0F || source_y < 0.0F ||
    source_x >= input_width ||
    source_y >= input_height)
  {
    output_mask[output_index] = 0U;
    return;
  }

  const float luminance = samplePlaneBicubic(
    y_plane,
    input_width,
    input_width,
    input_height,
    source_x,
    source_y);
  const float contrasted = clampFloat(
    (luminance - contrast_center) * contrast_gain +
    contrast_center + brightness_offset,
    0.0F,
    255.0F);
  output_mask[output_index] =
    contrasted >= static_cast<float>(binary_threshold) ? 255U : 0U;
}

__global__ void dilateBinaryKernel(
  const std::uint8_t * input,
  const int width,
  const int height,
  const int radius,
  std::uint8_t * output)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= width || y >= height) {
    return;
  }

  std::uint8_t maximum = 0U;
  for (int offset_y = -radius; offset_y <= radius; ++offset_y) {
    const int sample_y = y + offset_y;
    if (sample_y < 0 || sample_y >= height) {
      continue;
    }
    for (int offset_x = -radius; offset_x <= radius; ++offset_x) {
      const int sample_x = x + offset_x;
      if (sample_x < 0 || sample_x >= width) {
        continue;
      }
      const std::uint8_t value = input[sample_y * width + sample_x];
      if (value > maximum) {
        maximum = value;
      }
    }
  }
  output[y * width + x] = maximum;
}

__global__ void erodeBinaryKernel(
  const std::uint8_t * input,
  const int width,
  const int height,
  const int radius,
  std::uint8_t * output)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= width || y >= height) {
    return;
  }

  std::uint8_t minimum = 255U;
  for (int offset_y = -radius; offset_y <= radius; ++offset_y) {
    const int sample_y = y + offset_y;
    if (sample_y < 0 || sample_y >= height) {
      continue;
    }
    for (int offset_x = -radius; offset_x <= radius; ++offset_x) {
      const int sample_x = x + offset_x;
      if (sample_x < 0 || sample_x >= width) {
        continue;
      }
      const std::uint8_t value = input[sample_y * width + sample_x];
      if (value < minimum) {
        minimum = value;
      }
    }
  }
  output[y * width + x] = minimum;
}

}  // namespace

class CudaBevProcessor::Impl
{
public:
  Impl(
    const int input_width,
    const int input_height,
    const cv::Mat & map_x,
    const cv::Mat & map_y,
    const CudaBevImageConfig & image_config)
  : input_width_(input_width),
    input_height_(input_height),
    output_width_(map_x.cols),
    output_height_(map_x.rows),
    image_config_(image_config)
  {
    if (
      input_width_ <= 0 || input_height_ <= 0 ||
      input_width_ % 2 != 0 || input_height_ % 2 != 0)
    {
      throw std::invalid_argument(
              "CUDA NV12 input dimensions must be positive and even");
    }
    if (
      map_x.empty() || map_y.empty() ||
      map_x.size() != map_y.size() ||
      map_x.type() != CV_32FC1 ||
      map_y.type() != CV_32FC1)
    {
      throw std::invalid_argument(
              "CUDA BEV maps must be equal-sized CV_32FC1 matrices");
    }
    if (
      !std::isfinite(image_config_.contrast_gain) ||
      image_config_.contrast_gain <= 0.0F ||
      !std::isfinite(image_config_.contrast_center) ||
      image_config_.contrast_center < 0.0F ||
      image_config_.contrast_center > 255.0F ||
      !std::isfinite(image_config_.brightness_offset) ||
      image_config_.brightness_offset < -255.0F ||
      image_config_.brightness_offset > 255.0F ||
      image_config_.binary_threshold < 0 ||
      image_config_.binary_threshold > 255 ||
      image_config_.closing_kernel_size <= 0 ||
      image_config_.closing_kernel_size > 15 ||
      image_config_.closing_kernel_size % 2 == 0)
    {
      throw std::invalid_argument(
              "invalid grayscale contrast, threshold, or closing parameter");
    }

    try {
      int device = 0;
      checkCuda(cudaGetDevice(&device), "cudaGetDevice");
      cudaDeviceProp properties{};
      checkCuda(
        cudaGetDeviceProperties(&properties, device),
        "cudaGetDeviceProperties");
      device_name_ = properties.name;

      checkCuda(
        cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking),
        "cudaStreamCreateWithFlags");

      const std::size_t y_plane_bytes =
        static_cast<std::size_t>(input_width_) *
        static_cast<std::size_t>(input_height_);
      const std::size_t map_bytes =
        static_cast<std::size_t>(output_width_) *
        static_cast<std::size_t>(output_height_) * sizeof(float);
      const std::size_t output_bytes =
        static_cast<std::size_t>(output_width_) *
        static_cast<std::size_t>(output_height_);

      checkCuda(
        cudaMalloc(
          reinterpret_cast<void **>(&device_y_plane_),
          y_plane_bytes),
        "cudaMalloc NV12 Y plane");
      checkCuda(
        cudaMalloc(
          reinterpret_cast<void **>(&device_map_x_),
          map_bytes),
        "cudaMalloc map_x");
      checkCuda(
        cudaMalloc(
          reinterpret_cast<void **>(&device_map_y_),
          map_bytes),
        "cudaMalloc map_y");
      checkCuda(
        cudaMalloc(
          reinterpret_cast<void **>(&device_threshold_mask_),
          output_bytes),
        "cudaMalloc threshold mask");
      checkCuda(
        cudaMalloc(
          reinterpret_cast<void **>(&device_dilated_mask_),
          output_bytes),
        "cudaMalloc dilated mask");
      checkCuda(
        cudaMalloc(
          reinterpret_cast<void **>(&device_output_),
          output_bytes),
        "cudaMalloc BEV output");

      const cv::Mat continuous_map_x =
        map_x.isContinuous() ? map_x : map_x.clone();
      const cv::Mat continuous_map_y =
        map_y.isContinuous() ? map_y : map_y.clone();
      checkCuda(
        cudaMemcpyAsync(
          device_map_x_,
          continuous_map_x.ptr<float>(),
          map_bytes,
          cudaMemcpyHostToDevice,
          stream_),
        "cudaMemcpyAsync map_x");
      checkCuda(
        cudaMemcpyAsync(
          device_map_y_,
          continuous_map_y.ptr<float>(),
          map_bytes,
          cudaMemcpyHostToDevice,
          stream_),
        "cudaMemcpyAsync map_y");
      checkCuda(cudaStreamSynchronize(stream_), "upload BEV maps");
    } catch (...) {
      release();
      throw;
    }
  }

  ~Impl()
  {
    release();
  }

  cv::Mat process(
    const std::uint8_t * nv12,
    const std::size_t data_size,
    const std::size_t input_stride)
  {
    if (nv12 == nullptr || input_stride <
      static_cast<std::size_t>(input_width_))
    {
      throw std::invalid_argument("invalid host NV12 buffer or stride");
    }
    const std::size_t input_rows =
      static_cast<std::size_t>(input_height_) * 3U / 2U;
    if (data_size < input_stride * input_rows) {
      throw std::invalid_argument("host NV12 buffer is smaller than expected");
    }

    std::lock_guard<std::mutex> lock(stream_mutex_);
    checkCuda(
      cudaMemcpy2DAsync(
        device_y_plane_,
        static_cast<std::size_t>(input_width_),
        nv12,
        input_stride,
        static_cast<std::size_t>(input_width_),
        static_cast<std::size_t>(input_height_),
        cudaMemcpyHostToDevice,
        stream_),
      "upload NV12 Y plane");

    const dim3 block(16U, 16U);
    const dim3 grid(
      static_cast<unsigned int>((output_width_ + 15) / 16),
      static_cast<unsigned int>((output_height_ + 15) / 16));
    yPlaneToBinaryBevKernel<<<grid, block, 0, stream_>>>(
      device_y_plane_,
      input_width_,
      input_height_,
      device_map_x_,
      device_map_y_,
      output_width_,
      output_height_,
      image_config_.contrast_gain,
      image_config_.contrast_center,
      image_config_.brightness_offset,
      image_config_.binary_threshold,
      device_threshold_mask_);
    checkCuda(
      cudaGetLastError(), "launch bicubic grayscale BEV kernel");

    if (image_config_.closing_kernel_size > 1) {
      const int radius = image_config_.closing_kernel_size / 2;
      dilateBinaryKernel<<<grid, block, 0, stream_>>>(
        device_threshold_mask_,
        output_width_,
        output_height_,
        radius,
        device_dilated_mask_);
      checkCuda(cudaGetLastError(), "launch binary dilation kernel");
      erodeBinaryKernel<<<grid, block, 0, stream_>>>(
        device_dilated_mask_,
        output_width_,
        output_height_,
        radius,
        device_output_);
      checkCuda(cudaGetLastError(), "launch binary erosion kernel");
    } else {
      const std::size_t output_bytes =
        static_cast<std::size_t>(output_width_) *
        static_cast<std::size_t>(output_height_);
      checkCuda(
        cudaMemcpyAsync(
          device_output_,
          device_threshold_mask_,
          output_bytes,
          cudaMemcpyDeviceToDevice,
          stream_),
        "copy threshold mask");
    }

    cv::Mat output(output_height_, output_width_, CV_8UC1);
    const std::size_t output_bytes =
      static_cast<std::size_t>(output_width_) *
      static_cast<std::size_t>(output_height_);
    checkCuda(
      cudaMemcpyAsync(
        output.data,
        device_output_,
        output_bytes,
        cudaMemcpyDeviceToHost,
        stream_),
      "download BEV output");
    checkCuda(cudaStreamSynchronize(stream_), "process NV12 BEV frame");
    return output;
  }

  void updateRemap(const cv::Mat & map_x, const cv::Mat & map_y)
  {
    if (
      map_x.empty() || map_y.empty() ||
      map_x.rows != output_height_ ||
      map_x.cols != output_width_ ||
      map_x.size() != map_y.size() ||
      map_x.type() != CV_32FC1 ||
      map_y.type() != CV_32FC1)
    {
      throw std::invalid_argument(
              "updated CUDA BEV maps must match the configured CV_32FC1 size");
    }

    const cv::Mat continuous_map_x =
      map_x.isContinuous() ? map_x : map_x.clone();
    const cv::Mat continuous_map_y =
      map_y.isContinuous() ? map_y : map_y.clone();
    const std::size_t map_bytes =
      static_cast<std::size_t>(output_width_) *
      static_cast<std::size_t>(output_height_) * sizeof(float);

    std::lock_guard<std::mutex> lock(stream_mutex_);
    checkCuda(
      cudaMemcpyAsync(
        device_map_x_,
        continuous_map_x.ptr<float>(),
        map_bytes,
        cudaMemcpyHostToDevice,
        stream_),
      "update map_x");
    checkCuda(
      cudaMemcpyAsync(
        device_map_y_,
        continuous_map_y.ptr<float>(),
        map_bytes,
        cudaMemcpyHostToDevice,
        stream_),
      "update map_y");
    checkCuda(cudaStreamSynchronize(stream_), "update BEV maps");
  }

  const std::string & deviceName() const
  {
    return device_name_;
  }

private:
  void release() noexcept
  {
    if (device_output_ != nullptr) {
      cudaFree(device_output_);
      device_output_ = nullptr;
    }
    if (device_dilated_mask_ != nullptr) {
      cudaFree(device_dilated_mask_);
      device_dilated_mask_ = nullptr;
    }
    if (device_threshold_mask_ != nullptr) {
      cudaFree(device_threshold_mask_);
      device_threshold_mask_ = nullptr;
    }
    if (device_map_y_ != nullptr) {
      cudaFree(device_map_y_);
      device_map_y_ = nullptr;
    }
    if (device_map_x_ != nullptr) {
      cudaFree(device_map_x_);
      device_map_x_ = nullptr;
    }
    if (device_y_plane_ != nullptr) {
      cudaFree(device_y_plane_);
      device_y_plane_ = nullptr;
    }
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
      stream_ = nullptr;
    }
  }

  int input_width_;
  int input_height_;
  int output_width_;
  int output_height_;
  CudaBevImageConfig image_config_;
  std::string device_name_;
  cudaStream_t stream_{nullptr};
  std::uint8_t * device_y_plane_{nullptr};
  float * device_map_x_{nullptr};
  float * device_map_y_{nullptr};
  std::uint8_t * device_threshold_mask_{nullptr};
  std::uint8_t * device_dilated_mask_{nullptr};
  std::uint8_t * device_output_{nullptr};
  std::mutex stream_mutex_;
};

CudaBevProcessor::CudaBevProcessor(
  const int input_width,
  const int input_height,
  const cv::Mat & map_x,
  const cv::Mat & map_y,
  const CudaBevImageConfig & image_config)
: impl_(std::make_unique<Impl>(
    input_width, input_height, map_x, map_y, image_config))
{
}

CudaBevProcessor::~CudaBevProcessor() = default;

cv::Mat CudaBevProcessor::process(
  const std::uint8_t * nv12,
  const std::size_t data_size,
  const std::size_t input_stride)
{
  return impl_->process(nv12, data_size, input_stride);
}

void CudaBevProcessor::updateRemap(
  const cv::Mat & map_x,
  const cv::Mat & map_y)
{
  impl_->updateRemap(map_x, map_y);
}

const std::string & CudaBevProcessor::deviceName() const
{
  return impl_->deviceName();
}

}  // namespace bev_processor
