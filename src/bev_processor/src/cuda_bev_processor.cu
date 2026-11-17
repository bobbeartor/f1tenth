#include "bev_processor/cuda_bev_processor.hpp"

#include <cuda_runtime.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>

#include <opencv2/imgproc.hpp>

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

__device__ int clampInt(
  const int value,
  const int minimum,
  const int maximum)
{
  return value < minimum ? minimum : value > maximum ? maximum : value;
}

__device__ float readPlane(
  const std::uint8_t * plane,
  const int stride,
  const int width,
  const int height,
  const int x,
  const int y)
{
  return static_cast<float>(plane[
      clampInt(y, 0, height - 1) * stride +
      clampInt(x, 0, width - 1)]);
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

__device__ float cubicKernel(const float distance)
{
  constexpr float a = -0.5F;
  const float absolute = fabsf(distance);
  if (absolute <= 1.0F) {
    return
      (a + 2.0F) * absolute * absolute * absolute -
      (a + 3.0F) * absolute * absolute + 1.0F;
  }
  if (absolute < 2.0F) {
    return
      a * absolute * absolute * absolute -
      5.0F * a * absolute * absolute +
      8.0F * a * absolute - 4.0F * a;
  }
  return 0.0F;
}

__device__ void localRange2x2(
  const std::uint8_t * plane,
  const int stride,
  const int width,
  const int height,
  const float x,
  const float y,
  float & local_minimum,
  float & local_maximum)
{
  const int x0 = clampInt(static_cast<int>(floorf(x)), 0, width - 1);
  const int y0 = clampInt(static_cast<int>(floorf(y)), 0, height - 1);
  const int x1 = x0 + 1 < width ? x0 + 1 : width - 1;
  const int y1 = y0 + 1 < height ? y0 + 1 : height - 1;
  const float value00 = readPlane(plane, stride, width, height, x0, y0);
  const float value10 = readPlane(plane, stride, width, height, x1, y0);
  const float value01 = readPlane(plane, stride, width, height, x0, y1);
  const float value11 = readPlane(plane, stride, width, height, x1, y1);
  local_minimum = fminf(fminf(value00, value10), fminf(value01, value11));
  local_maximum = fmaxf(fmaxf(value00, value10), fmaxf(value01, value11));
}

__device__ float samplePlaneBicubic(
  const std::uint8_t * plane,
  const int stride,
  const int width,
  const int height,
  const float x,
  const float y)
{
  const int base_x = static_cast<int>(floorf(x));
  const int base_y = static_cast<int>(floorf(y));
  float weighted_sum = 0.0F;
  float total_weight = 0.0F;
#pragma unroll
  for (int row_offset = -1; row_offset <= 2; ++row_offset) {
    const float weight_y = cubicKernel(y - (base_y + row_offset));
#pragma unroll
    for (int column_offset = -1; column_offset <= 2; ++column_offset) {
      const float weight_x = cubicKernel(x - (base_x + column_offset));
      const float weight = weight_x * weight_y;
      weighted_sum += weight * readPlane(
        plane, stride, width, height,
        base_x + column_offset, base_y + row_offset);
      total_weight += weight;
    }
  }
  const float result = fabsf(total_weight) > 1.0e-6F ?
    weighted_sum / total_weight :
    samplePlane(plane, stride, width, height, x, y);
  float local_minimum = 0.0F;
  float local_maximum = 0.0F;
  localRange2x2(
    plane, stride, width, height, x, y, local_minimum, local_maximum);
  return clampFloat(result, local_minimum, local_maximum);
}

__device__ float smoothstep(
  const float edge0,
  const float edge1,
  const float value)
{
  if (edge1 <= edge0) {
    return value >= edge1 ? 1.0F : 0.0F;
  }
  const float amount = clampFloat((value - edge0) / (edge1 - edge0), 0.0F, 1.0F);
  return amount * amount * (3.0F - 2.0F * amount);
}

__device__ void sobelGradient(
  const std::uint8_t * plane,
  const int stride,
  const int width,
  const int height,
  const int x,
  const int y,
  float & gradient_x,
  float & gradient_y)
{
  const float top_left = readPlane(plane, stride, width, height, x - 1, y - 1);
  const float top = readPlane(plane, stride, width, height, x, y - 1);
  const float top_right = readPlane(plane, stride, width, height, x + 1, y - 1);
  const float left = readPlane(plane, stride, width, height, x - 1, y);
  const float right = readPlane(plane, stride, width, height, x + 1, y);
  const float bottom_left = readPlane(plane, stride, width, height, x - 1, y + 1);
  const float bottom = readPlane(plane, stride, width, height, x, y + 1);
  const float bottom_right = readPlane(plane, stride, width, height, x + 1, y + 1);
  gradient_x = 0.25F * (
    top_right + 2.0F * right + bottom_right -
    top_left - 2.0F * left - bottom_left);
  gradient_y = 0.25F * (
    bottom_left + 2.0F * bottom + bottom_right -
    top_left - 2.0F * top - top_right);
}

__device__ float samplePlaneAdaptive(
  const std::uint8_t * plane,
  const int stride,
  const int width,
  const int height,
  const float source_x,
  const float source_y,
  const float x_vehicle_m,
  const float start_x_m,
  const float full_x_m,
  const float strength,
  const float gradient_low,
  const float gradient_high,
  const float coherence_minimum,
  const float maximum_anisotropy)
{
  const float standard = samplePlaneBicubic(
    plane, stride, width, height, source_x, source_y);
  const int center_x = static_cast<int>(floorf(source_x + 0.5F));
  const int center_y = static_cast<int>(floorf(source_y + 0.5F));
  constexpr float gaussian_edge = 0.238994F;
  constexpr float gaussian_center = 0.522012F;
  float tensor_xx = 0.0F;
  float tensor_xy = 0.0F;
  float tensor_yy = 0.0F;
#pragma unroll
  for (int row_offset = -1; row_offset <= 1; ++row_offset) {
    const float weight_y = row_offset == 0 ? gaussian_center : gaussian_edge;
#pragma unroll
    for (int column_offset = -1; column_offset <= 1; ++column_offset) {
      const float weight_x = column_offset == 0 ? gaussian_center : gaussian_edge;
      float gradient_x = 0.0F;
      float gradient_y = 0.0F;
      sobelGradient(
        plane, stride, width, height,
        center_x + column_offset, center_y + row_offset,
        gradient_x, gradient_y);
      const float weight = weight_x * weight_y;
      tensor_xx += weight * gradient_x * gradient_x;
      tensor_xy += weight * gradient_x * gradient_y;
      tensor_yy += weight * gradient_y * gradient_y;
    }
  }

  const float trace = fmaxf(0.0F, tensor_xx + tensor_yy);
  const float eigen_gap = sqrtf(fmaxf(
      0.0F,
      (tensor_xx - tensor_yy) * (tensor_xx - tensor_yy) +
      4.0F * tensor_xy * tensor_xy));
  const float largest_eigenvalue = 0.5F * (trace + eigen_gap);
  const float gradient = fminf(255.0F, sqrtf(fmaxf(0.0F, largest_eigenvalue)));
  const float coherence = eigen_gap / (trace + 1.0e-9F);
  const float blend = clampFloat(
    strength *
    smoothstep(gradient_low, gradient_high, gradient) *
    smoothstep(coherence_minimum, 1.0F, coherence) *
    smoothstep(start_x_m, full_x_m, x_vehicle_m),
    0.0F,
    1.0F);
  if (blend <= 1.0e-6F) {
    return standard;
  }

  const float normal_angle = 0.5F * atan2f(
    2.0F * tensor_xy, tensor_xx - tensor_yy);
  const float normal_x = cosf(normal_angle);
  const float normal_y = sinf(normal_angle);
  const float anisotropy = 1.0F + (maximum_anisotropy - 1.0F) * blend;
  const int base_x = static_cast<int>(floorf(source_x));
  const int base_y = static_cast<int>(floorf(source_y));
  float weighted_sum = 0.0F;
  float total_weight = 0.0F;
#pragma unroll
  for (int row_offset = -1; row_offset <= 2; ++row_offset) {
    const float delta_y = base_y + row_offset - source_y;
    const float weight_y = cubicKernel(-delta_y);
#pragma unroll
    for (int column_offset = -1; column_offset <= 2; ++column_offset) {
      const float delta_x = base_x + column_offset - source_x;
      const float weight_x = cubicKernel(-delta_x);
      const float normal_distance = delta_x * normal_x + delta_y * normal_y;
      const float directional_weight = expf(
        -0.5F * (anisotropy - 1.0F) * normal_distance * normal_distance);
      const float weight = weight_x * weight_y * directional_weight;
      weighted_sum += weight * readPlane(
        plane, stride, width, height,
        base_x + column_offset, base_y + row_offset);
      total_weight += weight;
    }
  }
  if (fabsf(total_weight) <= 1.0e-6F) {
    return standard;
  }
  float local_minimum = 0.0F;
  float local_maximum = 0.0F;
  localRange2x2(
    plane, stride, width, height, source_x, source_y,
    local_minimum, local_maximum);
  const float adaptive = clampFloat(
    weighted_sum / total_weight, local_minimum, local_maximum);
  return (1.0F - blend) * standard + blend * adaptive;
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
  const float * stabilized_to_source_homography,
  const int source_crop_top,
  const int output_width,
  const int output_height,
  const int interpolation,
  const float adaptive_start_x_m,
  const float adaptive_full_x_m,
  const float adaptive_strength,
  const float adaptive_gradient_low,
  const float adaptive_gradient_high,
  const float adaptive_coherence_minimum,
  const float adaptive_maximum_anisotropy,
  const float bev_x_max_m,
  const float meter_per_pixel,
  std::uint8_t * output_bgr)
{
  const int output_x = blockIdx.x * blockDim.x + threadIdx.x;
  const int output_y = blockIdx.y * blockDim.y + threadIdx.y;
  if (output_x >= output_width || output_y >= output_height) {
    return;
  }

  const int output_index = output_y * output_width + output_x;
  const float stabilized_x = map_x[output_index];
  const float stabilized_y = map_y[output_index];
  std::uint8_t * destination = output_bgr + output_index * 3;
  if (
    stabilized_x < 0.0F || stabilized_y < 0.0F)
  {
    destination[0] = 0U;
    destination[1] = 0U;
    destination[2] = 0U;

    return;
  }

  const float denominator =
    stabilized_to_source_homography[6] * stabilized_x +
    stabilized_to_source_homography[7] * stabilized_y +
    stabilized_to_source_homography[8];
  if (!isfinite(denominator) || fabsf(denominator) < 1.0e-6F) {
    destination[0] = 0U;
    destination[1] = 0U;
    destination[2] = 0U;

    return;
  }
  const float source_x =
    (stabilized_to_source_homography[0] * stabilized_x +
    stabilized_to_source_homography[1] * stabilized_y +
    stabilized_to_source_homography[2]) / denominator;
  const float source_y_full =
    (stabilized_to_source_homography[3] * stabilized_x +
    stabilized_to_source_homography[4] * stabilized_y +
    stabilized_to_source_homography[5]) / denominator;
  const float source_y = source_y_full - source_crop_top;
  if (
    !isfinite(source_x) || !isfinite(source_y) ||
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
  float y = 0.0F;
  if (interpolation == static_cast<int>(BevInterpolation::Adaptive)) {
    const float x_vehicle_m =
      bev_x_max_m - (output_y + 0.5F) * meter_per_pixel;
    y = samplePlaneAdaptive(
      y_plane,
      input_width,
      input_width,
      input_height,
      source_x,
      source_y,
      x_vehicle_m,
      adaptive_start_x_m,
      adaptive_full_x_m,
      adaptive_strength,
      adaptive_gradient_low,
      adaptive_gradient_high,
      adaptive_coherence_minimum,
      adaptive_maximum_anisotropy);
  } else if (interpolation == static_cast<int>(BevInterpolation::Bicubic)) {
    y = samplePlaneBicubic(
      y_plane,
      input_width,
      input_width,
      input_height,
      source_x,
      source_y);
  } else {
    y = samplePlane(
      y_plane,
      input_width,
      input_width,
      input_height,
      source_x,
      source_y);
  }
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

  const std::uint8_t output_blue = static_cast<std::uint8_t>(
    clampFloat(blue, 0.0F, 255.0F));
  const std::uint8_t output_green = static_cast<std::uint8_t>(
    clampFloat(green, 0.0F, 255.0F));
  const std::uint8_t output_red = static_cast<std::uint8_t>(
    clampFloat(red, 0.0F, 255.0F));
  destination[0] = output_blue;
  destination[1] = output_green;
  destination[2] = output_red;
}

class DeviceOutputPool final :
  public std::enable_shared_from_this<DeviceOutputPool>
{
public:
  DeviceOutputPool(
    const std::size_t bytes_per_buffer,
    const std::size_t initial_buffer_count)
  : bytes_per_buffer_(bytes_per_buffer)
  {
    if (bytes_per_buffer_ == 0U || initial_buffer_count == 0U) {
      throw std::invalid_argument("CUDA BEV output pool dimensions are invalid");
    }
    try {
      for (std::size_t index = 0; index < initial_buffer_count; ++index) {
        addBuffer();
      }
    } catch (...) {
      releaseBuffers();
      throw;
    }
  }

  ~DeviceOutputPool()
  {
    releaseBuffers();
  }

  DeviceOutputPool(const DeviceOutputPool &) = delete;
  DeviceOutputPool & operator=(const DeviceOutputPool &) = delete;

  std::shared_ptr<const void> acquire()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::size_t index = 0U;
    for (; index < buffers_.size(); ++index) {
      if (!buffers_[index].in_use) {
        break;
      }
    }
    if (index == buffers_.size()) {
      // The normal latest-only path reuses the initial buffers. Growing here
      // avoids blocking the BEV thread during a transient slow preview.
      addBuffer();
    }
    buffers_[index].in_use = true;
    auto self = shared_from_this();
    return std::shared_ptr<const void>(
      buffers_[index].data,
      [self = std::move(self), index](const void *) {
        self->release(index);
      });
  }

private:
  struct Buffer
  {
    std::uint8_t * data{nullptr};
    bool in_use{false};
  };

  void addBuffer()
  {
    Buffer buffer;
    checkCuda(
      cudaMalloc(
        reinterpret_cast<void **>(&buffer.data), bytes_per_buffer_),
      "cudaMalloc pooled BEV output");
    buffers_.push_back(buffer);
  }

  void release(const std::size_t index) noexcept
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (index < buffers_.size()) {
      buffers_[index].in_use = false;
    }
  }

  void releaseBuffers() noexcept
  {
    for (auto & buffer : buffers_) {
      if (buffer.data != nullptr) {
        static_cast<void>(cudaFree(buffer.data));
        buffer.data = nullptr;
      }
    }
    buffers_.clear();
  }

  std::size_t bytes_per_buffer_{0U};
  std::mutex mutex_;
  std::vector<Buffer> buffers_;
};

// Independent nonblocking stream: preview/capture never locks the BEV stream.
// The shared downloader and each captured device_owner outlive reconfiguration.
class HostDownloader
{
  int device_;
  cudaStream_t stream_{nullptr};
  std::mutex mutex_;
public:
  explicit HostDownloader(int device):device_(device) {}
  ~HostDownloader() {
    if(stream_) {
      int previous=0;static_cast<void>(cudaGetDevice(&previous));
      static_cast<void>(cudaSetDevice(device_));static_cast<void>(cudaStreamDestroy(stream_));
      static_cast<void>(cudaSetDevice(previous));
    }
  }
  cv::Mat download(const void* source,int width,int height,std::size_t stride) {
    std::lock_guard<std::mutex> lock(mutex_);
    int previous=0;checkCuda(cudaGetDevice(&previous),"preview cudaGetDevice");
    checkCuda(cudaSetDevice(device_),"preview cudaSetDevice");
    cv::Mat image;
    try {
      if(!stream_)checkCuda(cudaStreamCreateWithFlags(&stream_,cudaStreamNonBlocking),"preview stream");
      image.create(height,width,CV_8UC3);
      checkCuda(cudaMemcpy2DAsync(image.data,image.step,source,stride,
        static_cast<std::size_t>(width)*3U,height,cudaMemcpyDeviceToHost,stream_),"lazy BEV download");
      checkCuda(cudaStreamSynchronize(stream_),"finish lazy BEV download");
      checkCuda(cudaSetDevice(previous),"restore preview CUDA device");
      return image;
    } catch(...) {
      // A queued transfer must not outlive its host image on an error path.
      if(stream_)static_cast<void>(cudaStreamSynchronize(stream_));
      static_cast<void>(cudaSetDevice(previous));throw;
    }
  }
};

struct LazyHostImage
{
  std::once_flag once;
  cv::Mat image;
  std::shared_ptr<HostDownloader> downloader;
  std::shared_ptr<const void> owner;
  const void* source;
  int width,height;
  std::size_t stride;
  LazyHostImage(std::shared_ptr<HostDownloader> reader,std::shared_ptr<const void> buffer,
    const void* data,int w,int h,std::size_t pitch)
  :downloader(std::move(reader)),owner(std::move(buffer)),source(data),width(w),height(h),stride(pitch) {}
  cv::Mat get() {
    std::call_once(once,[this] {image=downloader->download(source,width,height,stride);});
    return image;
  }
};

}  // namespace

class CudaBevProcessor::Impl
{
public:
  Impl(
    const int input_width,
    const int input_height,
    const cv::Mat & map_x,
    const cv::Mat & map_y,
    const BevInterpolation interpolation,
    const EdgeAdaptiveConfig & edge_adaptive_config)
  : input_width_(input_width),
    input_height_(input_height),
    output_width_(map_x.cols),
    output_height_(map_x.rows),
    interpolation_(interpolation),
    edge_adaptive_config_(edge_adaptive_config)
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
      host_downloader_=std::make_shared<HostDownloader>(device);

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
          reinterpret_cast<void **>(&device_stabilized_to_source_),
          9U * sizeof(float)),
        "cudaMalloc stabilized-to-source homography");
      output_pool_ = std::make_shared<DeviceOutputPool>(output_bytes, 4U);
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

  CudaBevResult process(
    const std::uint8_t * nv12,
    const std::size_t data_size,
    const std::size_t input_stride,
    const cv::Matx33d & stabilized_to_source_homography,
    const int source_crop_top,
    const bool download_to_host)
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
    if (
      source_crop_top < 0 || source_crop_top % 2 != 0 ||
      !cv::checkRange(cv::Mat(stabilized_to_source_homography)))
    {
      throw std::invalid_argument(
              "invalid source crop or stabilized-to-source homography");
    }

    std::array<float, 9> homography{};
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        homography[static_cast<std::size_t>(row * 3 + column)] =
          static_cast<float>(
          stabilized_to_source_homography(row, column));
      }
    }

    const auto device_owner = output_pool_->acquire();
    auto * device_output = static_cast<std::uint8_t *>(
      const_cast<void *>(device_owner.get()));
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
    checkCuda(
      cudaMemcpyAsync(
        device_stabilized_to_source_,
        homography.data(),
        homography.size() * sizeof(float),
        cudaMemcpyHostToDevice,
        stream_),
      "upload stabilized-to-source homography");

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
      device_stabilized_to_source_,
      source_crop_top,
      output_width_,
      output_height_,
      static_cast<int>(interpolation_),
      static_cast<float>(edge_adaptive_config_.start_x_m),
      static_cast<float>(edge_adaptive_config_.full_x_m),
      static_cast<float>(edge_adaptive_config_.strength),
      static_cast<float>(edge_adaptive_config_.gradient_low),
      static_cast<float>(edge_adaptive_config_.gradient_high),
      static_cast<float>(edge_adaptive_config_.coherence_minimum),
      static_cast<float>(edge_adaptive_config_.maximum_anisotropy),
      static_cast<float>(edge_adaptive_config_.bev_x_max_m),
      static_cast<float>(edge_adaptive_config_.meter_per_pixel),
      device_output);
    checkCuda(cudaGetLastError(), "launch NV12-to-BEV kernel");


    CudaBevResult output;
    output.device_bgr = device_output;
    output.device_stride = static_cast<std::size_t>(output_width_) * 3U;
    output.device_owner = device_owner;
    const std::size_t output_bytes =
      static_cast<std::size_t>(output_width_) *
      static_cast<std::size_t>(output_height_) * 3U;
    if (download_to_host) {
      output.bgr.create(output_height_, output_width_, CV_8UC3);
      checkCuda(
        cudaMemcpyAsync(
          output.bgr.data,
          device_output,
          output_bytes,
          cudaMemcpyDeviceToHost,
          stream_),
        "download BEV output");
    }

    checkCuda(cudaStreamSynchronize(stream_), "process NV12 BEV frame");
    if(!download_to_host) {
      // Publication follows producer completion; consumers may copy this buffer
      // later without a cross-stream event or a producer-thread wait.
      auto lazy=std::make_shared<LazyHostImage>(host_downloader_,device_owner,device_output,
        output_width_,output_height_,output.device_stride);
      output.download_bgr=[lazy] {return lazy->get();};
    }
    return output;
  }

  const std::string & deviceName() const
  {
    return device_name_;
  }

private:
  void release() noexcept
  {
    output_pool_.reset();
    if (device_map_y_ != nullptr) {
      cudaFree(device_map_y_);
      device_map_y_ = nullptr;
    }
    if (device_stabilized_to_source_ != nullptr) {
      cudaFree(device_stabilized_to_source_);
      device_stabilized_to_source_ = nullptr;
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
  BevInterpolation interpolation_;
  EdgeAdaptiveConfig edge_adaptive_config_;
  std::string device_name_;
  cudaStream_t stream_{nullptr};
  std::uint8_t * device_nv12_{nullptr};
  float * device_map_x_{nullptr};
  float * device_map_y_{nullptr};
  float * device_stabilized_to_source_{nullptr};
  std::shared_ptr<DeviceOutputPool> output_pool_;
  std::shared_ptr<HostDownloader> host_downloader_;
  std::mutex stream_mutex_;
};

CudaBevProcessor::CudaBevProcessor(
  const int input_width,
  const int input_height,
  const cv::Mat & map_x,
  const cv::Mat & map_y,
  const BevInterpolation interpolation,
  const EdgeAdaptiveConfig & edge_adaptive_config)
: impl_(std::make_unique<Impl>(
    input_width, input_height, map_x, map_y, interpolation,
    edge_adaptive_config))
{
}

CudaBevProcessor::~CudaBevProcessor() = default;

CudaBevResult CudaBevProcessor::process(
  const std::uint8_t * nv12,
  const std::size_t data_size,
  const std::size_t input_stride,
  const cv::Matx33d & stabilized_to_source_homography,
  const int source_crop_top,
  const bool download_to_host)
{
  return impl_->process(
    nv12,
    data_size,
    input_stride,
    stabilized_to_source_homography,
    source_crop_top,
    download_to_host);
}

const std::string & CudaBevProcessor::deviceName() const
{
  return impl_->deviceName();
}

}  // namespace bev_processor
