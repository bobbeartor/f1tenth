#ifndef MULTICLASS_YOLO__CUDA_YOLOX_PREPROCESSOR_HPP_
#define MULTICLASS_YOLO__CUDA_YOLOX_PREPROCESSOR_HPP_

#include <cstddef>
#include <cstdint>

#include <cuda_runtime_api.h>

namespace multiclass_yolo
{

cudaError_t launch_nv12_roi_to_bgr_nchw(
  const std::uint8_t * device_nv12,
  std::size_t source_stride,
  int source_height,
  int roi_left,
  int roi_top,
  int roi_width,
  int roi_height,
  float * device_output,
  int output_width,
  int output_height,
  cudaStream_t stream) noexcept;

}  // namespace multiclass_yolo

#endif  // MULTICLASS_YOLO__CUDA_YOLOX_PREPROCESSOR_HPP_
