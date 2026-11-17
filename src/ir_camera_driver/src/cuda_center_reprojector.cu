#include "ir_camera_driver/cuda_center_reprojector.hpp"

#include <cuda_runtime.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace ir_camera_driver
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

__device__ bool sampleHorizontalBilinear(
  const std::uint8_t * image,
  const int width,
  const int height,
  const float x,
  const int y,
  float & value)
{
  if (y < 0 || y >= height || x < 0.0F || x > width - 1.0F) {
    return false;
  }

  const int x0 = static_cast<int>(floorf(x));
  const int x1 = x0 + 1 < width ? x0 + 1 : width - 1;
  const float fraction = x - static_cast<float>(x0);
  const auto row_offset = static_cast<std::size_t>(y) * width;
  value =
    static_cast<float>(image[row_offset + x0]) * (1.0F - fraction) +
    static_cast<float>(image[row_offset + x1]) * fraction;
  return true;
}

__global__ void reprojectCenterKernel(
  const std::uint8_t * rectified_left,
  const std::uint8_t * rectified_right,
  const std::uint8_t * center_aligned_disparity,
  std::uint8_t * output,
  const int width,
  const int height,
  const float virtual_camera_position_ratio,
  const bool use_left_for_invalid_disparity)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= width || y >= height) {
    return;
  }

  const auto index = static_cast<std::size_t>(y) * width + x;
  const auto fallback = use_left_for_invalid_disparity ?
    rectified_left[index] : rectified_right[index];
  const float disparity =
    static_cast<float>(center_aligned_disparity[index]);
  if (disparity <= 0.0F) {
    output[index] = fallback;
    return;
  }

  // With a center-alignment factor r, DepthAI expresses the disparity at
  // x_virtual = x_left - r * disparity. The matching right coordinate is
  // x_right = x_virtual - (1-r) * disparity.
  const float left_x =
    static_cast<float>(x) + virtual_camera_position_ratio * disparity;
  const float right_x =
    static_cast<float>(x) -
    (1.0F - virtual_camera_position_ratio) * disparity;

  float left_value = 0.0F;
  float right_value = 0.0F;
  const bool left_valid = sampleHorizontalBilinear(
    rectified_left, width, height, left_x, y, left_value);
  const bool right_valid = sampleHorizontalBilinear(
    rectified_right, width, height, right_x, y, right_value);

  float result = static_cast<float>(fallback);
  if (left_valid && right_valid) {
    result = 0.5F * (left_value + right_value);
  } else if (left_valid) {
    result = left_value;
  } else if (right_valid) {
    result = right_value;
  }
  output[index] = static_cast<std::uint8_t>(
    fminf(255.0F, fmaxf(0.0F, result + 0.5F)));
}

}  // namespace

class CudaCenterReprojector::Impl
{
public:
  Impl()
  {
    checkCuda(
      cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking),
      "cudaStreamCreateWithFlags");
  }

  ~Impl()
  {
    releaseBuffers();
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
  }

  const cv::Mat & process(
    const cv::Mat & rectified_left,
    const cv::Mat & rectified_right,
    const cv::Mat & center_aligned_disparity,
    const double virtual_camera_position_ratio,
    const bool use_left_for_invalid_disparity)
  {
    validateInputs(
      rectified_left,
      rectified_right,
      center_aligned_disparity,
      virtual_camera_position_ratio);
    ensureSize(rectified_left.cols, rectified_left.rows);

    copyToDevice(rectified_left, device_left_, "copy rectified-left to CUDA");
    copyToDevice(rectified_right, device_right_, "copy rectified-right to CUDA");
    copyToDevice(
      center_aligned_disparity,
      device_disparity_,
      "copy center disparity to CUDA");

    const dim3 block(32U, 8U);
    const dim3 grid(
      static_cast<unsigned int>((width_ + block.x - 1) / block.x),
      static_cast<unsigned int>((height_ + block.y - 1) / block.y));
    reprojectCenterKernel<<<grid, block, 0, stream_>>>(
      device_left_,
      device_right_,
      device_disparity_,
      device_output_,
      width_,
      height_,
      static_cast<float>(virtual_camera_position_ratio),
      use_left_for_invalid_disparity);
    checkCuda(cudaGetLastError(), "launch center-view reprojection kernel");

    checkCuda(
      cudaMemcpy2DAsync(
        host_output_.data,
        host_output_.step,
        device_output_,
        static_cast<std::size_t>(width_),
        static_cast<std::size_t>(width_),
        static_cast<std::size_t>(height_),
        cudaMemcpyDeviceToHost,
        stream_),
      "copy center preview from CUDA");
    checkCuda(
      cudaStreamSynchronize(stream_),
      "synchronize center-view reprojection");
    return host_output_;
  }

private:
  static void validateInputs(
    const cv::Mat & rectified_left,
    const cv::Mat & rectified_right,
    const cv::Mat & center_aligned_disparity,
    const double virtual_camera_position_ratio)
  {
    if (
      rectified_left.empty() || rectified_right.empty() ||
      center_aligned_disparity.empty() ||
      rectified_left.type() != CV_8UC1 ||
      rectified_right.type() != CV_8UC1 ||
      center_aligned_disparity.type() != CV_8UC1 ||
      rectified_left.size() != rectified_right.size() ||
      rectified_left.size() != center_aligned_disparity.size())
    {
      throw std::invalid_argument(
              "center reprojection requires equal-sized CV_8UC1 "
              "left/right/disparity images");
    }
    if (
      !std::isfinite(virtual_camera_position_ratio) ||
      virtual_camera_position_ratio < 0.0 ||
      virtual_camera_position_ratio > 1.0)
    {
      throw std::invalid_argument(
              "virtual camera position ratio must be between 0 and 1");
    }
  }

  void copyToDevice(
    const cv::Mat & source,
    std::uint8_t * destination,
    const char * operation)
  {
    checkCuda(
      cudaMemcpy2DAsync(
        destination,
        static_cast<std::size_t>(width_),
        source.data,
        source.step,
        static_cast<std::size_t>(width_),
        static_cast<std::size_t>(height_),
        cudaMemcpyHostToDevice,
        stream_),
      operation);
  }

  void ensureSize(const int width, const int height)
  {
    if (width == width_ && height == height_ && device_left_ != nullptr) {
      return;
    }

    releaseBuffers();
    width_ = width;
    height_ = height;
    const auto byte_count =
      static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
    try {
      checkCuda(
        cudaMalloc(reinterpret_cast<void **>(&device_left_), byte_count),
        "cudaMalloc rectified-left");
      checkCuda(
        cudaMalloc(reinterpret_cast<void **>(&device_right_), byte_count),
        "cudaMalloc rectified-right");
      checkCuda(
        cudaMalloc(reinterpret_cast<void **>(&device_disparity_), byte_count),
        "cudaMalloc center disparity");
      checkCuda(
        cudaMalloc(reinterpret_cast<void **>(&device_output_), byte_count),
        "cudaMalloc center preview");
      host_output_.create(height_, width_, CV_8UC1);
    } catch (...) {
      releaseBuffers();
      throw;
    }
  }

  void releaseBuffers() noexcept
  {
    if (device_output_ != nullptr) {
      cudaFree(device_output_);
      device_output_ = nullptr;
    }
    if (device_disparity_ != nullptr) {
      cudaFree(device_disparity_);
      device_disparity_ = nullptr;
    }
    if (device_right_ != nullptr) {
      cudaFree(device_right_);
      device_right_ = nullptr;
    }
    if (device_left_ != nullptr) {
      cudaFree(device_left_);
      device_left_ = nullptr;
    }
    host_output_.release();
    width_ = 0;
    height_ = 0;
  }

  cudaStream_t stream_{nullptr};
  std::uint8_t * device_left_{nullptr};
  std::uint8_t * device_right_{nullptr};
  std::uint8_t * device_disparity_{nullptr};
  std::uint8_t * device_output_{nullptr};
  cv::Mat host_output_;
  int width_{0};
  int height_{0};
};

CudaCenterReprojector::CudaCenterReprojector()
: impl_(std::make_unique<Impl>())
{
}

CudaCenterReprojector::~CudaCenterReprojector() = default;

const cv::Mat & CudaCenterReprojector::process(
  const cv::Mat & rectified_left,
  const cv::Mat & rectified_right,
  const cv::Mat & center_aligned_disparity,
  const double virtual_camera_position_ratio,
  const bool use_left_for_invalid_disparity)
{
  return impl_->process(
    rectified_left,
    rectified_right,
    center_aligned_disparity,
    virtual_camera_position_ratio,
    use_left_for_invalid_disparity);
}

}  // namespace ir_camera_driver
