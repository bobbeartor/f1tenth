#include "bev_processor/cuda_bev_processor.hpp"

#include <cuda_runtime.h>

#include <algorithm>
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

__device__ float samplePlane(
  const std::uint8_t * plane,
  const int stride,
  const int width,
  const int height,
  const float x,
  const float y)
{
  const float clamped_x = clampFloat(x, 0.0F, width - 1.0F);
  const float clamped_y = clampFloat(y, 0.0F, height - 1.0F);
  const int x0 = static_cast<int>(floorf(clamped_x));
  const int y0 = static_cast<int>(floorf(clamped_y));
  const int x1 = x0 + 1 < width ? x0 + 1 : width - 1;
  const int y1 = y0 + 1 < height ? y0 + 1 : height - 1;
  const float dx = clamped_x - x0;
  const float dy = clamped_y - y0;

  const float top =
    plane[y0 * stride + x0] * (1.0F - dx) +
    plane[y0 * stride + x1] * dx;
  const float bottom =
    plane[y1 * stride + x0] * (1.0F - dx) +
    plane[y1 * stride + x1] * dx;
  return top * (1.0F - dy) + bottom * dy;
}

__device__ float sampleChroma(
  const std::uint8_t * uv_plane,
  const int stride,
  const int width,
  const int height,
  const float source_x,
  const float source_y,
  const int channel)
{
  const int chroma_width = width / 2;
  const int chroma_height = height / 2;
  const float x = clampFloat(
    source_x * 0.5F, 0.0F, chroma_width - 1.0F);
  const float y = clampFloat(
    source_y * 0.5F, 0.0F, chroma_height - 1.0F);
  const int x0 = static_cast<int>(floorf(x));
  const int y0 = static_cast<int>(floorf(y));
  const int x1 = x0 + 1 < chroma_width ? x0 + 1 : chroma_width - 1;
  const int y1 = y0 + 1 < chroma_height ? y0 + 1 : chroma_height - 1;
  const float dx = x - x0;
  const float dy = y - y0;

  const float top =
    uv_plane[y0 * stride + x0 * 2 + channel] * (1.0F - dx) +
    uv_plane[y0 * stride + x1 * 2 + channel] * dx;
  const float bottom =
    uv_plane[y1 * stride + x0 * 2 + channel] * (1.0F - dx) +
    uv_plane[y1 * stride + x1 * 2 + channel] * dx;
  return top * (1.0F - dy) + bottom * dy;
}

__global__ void nv12ToBevKernel(
  const std::uint8_t * nv12,
  const int input_width,
  const int input_height,
  const float * map_x,
  const float * map_y,
  const int output_width,
  const int output_height,
  std::uint8_t * output_bgr)
{
  const int output_x = blockIdx.x * blockDim.x + threadIdx.x;
  const int output_y = blockIdx.y * blockDim.y + threadIdx.y;
  if (output_x >= output_width || output_y >= output_height) {
    return;
  }

  const int output_index = output_y * output_width + output_x;
  const float source_x = map_x[output_index];
  const float source_y = map_y[output_index];
  std::uint8_t * destination = output_bgr + output_index * 3;
  if (
    source_x < 0.0F || source_y < 0.0F ||
    source_x >= input_width - 1.0F ||
    source_y >= input_height - 1.0F)
  {
    destination[0] = 0U;
    destination[1] = 0U;
    destination[2] = 0U;
    return;
  }

  const std::uint8_t * y_plane = nv12;
  const std::uint8_t * uv_plane =
    nv12 + input_width * input_height;
  const float y = samplePlane(
    y_plane,
    input_width,
    input_width,
    input_height,
    source_x,
    source_y);
  const float u = sampleChroma(
    uv_plane,
    input_width,
    input_width,
    input_height,
    source_x,
    source_y,
    0);
  const float v = sampleChroma(
    uv_plane,
    input_width,
    input_width,
    input_height,
    source_x,
    source_y,
    1);

  const float c = fmaxf(0.0F, y - 16.0F);
  const float d = u - 128.0F;
  const float e = v - 128.0F;
  const float red = 1.164F * c + 1.596F * e;
  const float green = 1.164F * c - 0.392F * d - 0.813F * e;
  const float blue = 1.164F * c + 2.017F * d;

  destination[0] = static_cast<std::uint8_t>(
    clampFloat(blue, 0.0F, 255.0F));
  destination[1] = static_cast<std::uint8_t>(
    clampFloat(green, 0.0F, 255.0F));
  destination[2] = static_cast<std::uint8_t>(
    clampFloat(red, 0.0F, 255.0F));
}

}  // namespace

class CudaBevProcessor::Impl
{
public:
  Impl(
    const int input_width,
    const int input_height,
    const cv::Mat & map_x,
    const cv::Mat & map_y)
  : input_width_(input_width),
    input_height_(input_height),
    output_width_(map_x.cols),
    output_height_(map_x.rows)
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

      const std::size_t nv12_bytes =
        static_cast<std::size_t>(input_width_) *
        static_cast<std::size_t>(input_height_) * 3U / 2U;
      const std::size_t map_bytes =
        static_cast<std::size_t>(output_width_) *
        static_cast<std::size_t>(output_height_) * sizeof(float);
      const std::size_t output_bytes =
        static_cast<std::size_t>(output_width_) *
        static_cast<std::size_t>(output_height_) * 3U;

      checkCuda(
        cudaMalloc(
          reinterpret_cast<void **>(&device_nv12_),
          nv12_bytes),
        "cudaMalloc NV12");
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
        device_nv12_,
        static_cast<std::size_t>(input_width_),
        nv12,
        input_stride,
        static_cast<std::size_t>(input_width_),
        input_rows,
        cudaMemcpyHostToDevice,
        stream_),
      "upload NV12 frame");

    const dim3 block(16U, 16U);
    const dim3 grid(
      static_cast<unsigned int>((output_width_ + 15) / 16),
      static_cast<unsigned int>((output_height_ + 15) / 16));
    nv12ToBevKernel<<<grid, block, 0, stream_>>>(
      device_nv12_,
      input_width_,
      input_height_,
      device_map_x_,
      device_map_y_,
      output_width_,
      output_height_,
      device_output_);
    checkCuda(cudaGetLastError(), "launch NV12-to-BEV kernel");

    cv::Mat output(output_height_, output_width_, CV_8UC3);
    const std::size_t output_bytes =
      static_cast<std::size_t>(output_width_) *
      static_cast<std::size_t>(output_height_) * 3U;
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
    if (device_map_y_ != nullptr) {
      cudaFree(device_map_y_);
      device_map_y_ = nullptr;
    }
    if (device_map_x_ != nullptr) {
      cudaFree(device_map_x_);
      device_map_x_ = nullptr;
    }
    if (device_nv12_ != nullptr) {
      cudaFree(device_nv12_);
      device_nv12_ = nullptr;
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
  std::string device_name_;
  cudaStream_t stream_{nullptr};
  std::uint8_t * device_nv12_{nullptr};
  float * device_map_x_{nullptr};
  float * device_map_y_{nullptr};
  std::uint8_t * device_output_{nullptr};
  std::mutex stream_mutex_;
};

CudaBevProcessor::CudaBevProcessor(
  const int input_width,
  const int input_height,
  const cv::Mat & map_x,
  const cv::Mat & map_y)
: impl_(std::make_unique<Impl>(
    input_width, input_height, map_x, map_y))
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

const std::string & CudaBevProcessor::deviceName() const
{
  return impl_->deviceName();
}

}  // namespace bev_processor
