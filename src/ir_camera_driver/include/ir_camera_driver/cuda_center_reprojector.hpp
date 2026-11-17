#ifndef IR_CAMERA_DRIVER__CUDA_CENTER_REPROJECTOR_HPP_
#define IR_CAMERA_DRIVER__CUDA_CENTER_REPROJECTOR_HPP_

#include <memory>

#include "opencv2/core.hpp"

namespace ir_camera_driver
{

class CudaCenterReprojector
{
public:
  CudaCenterReprojector();
  ~CudaCenterReprojector();

  CudaCenterReprojector(const CudaCenterReprojector &) = delete;
  CudaCenterReprojector & operator=(const CudaCenterReprojector &) = delete;

  const cv::Mat & process(
    const cv::Mat & rectified_left,
    const cv::Mat & rectified_right,
    const cv::Mat & center_aligned_disparity,
    double virtual_camera_position_ratio,
    bool use_left_for_invalid_disparity);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ir_camera_driver

#endif  // IR_CAMERA_DRIVER__CUDA_CENTER_REPROJECTOR_HPP_
