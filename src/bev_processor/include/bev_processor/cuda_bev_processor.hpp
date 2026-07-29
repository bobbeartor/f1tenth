#ifndef BEV_PROCESSOR__CUDA_BEV_PROCESSOR_HPP_
#define BEV_PROCESSOR__CUDA_BEV_PROCESSOR_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <opencv2/core.hpp>

namespace bev_processor
{

struct CudaBevImageConfig
{
  float contrast_gain{1.8F};
  float contrast_center{128.0F};
  float brightness_offset{0.0F};
  int binary_threshold{170};
  int closing_kernel_size{3};
};

class CudaBevProcessor
{
public:
  CudaBevProcessor(
    int input_width,
    int input_height,
    const cv::Mat & map_x,
    const cv::Mat & map_y,
    const CudaBevImageConfig & image_config);
  ~CudaBevProcessor();

  CudaBevProcessor(const CudaBevProcessor &) = delete;
  CudaBevProcessor & operator=(const CudaBevProcessor &) = delete;

  cv::Mat process(
    const std::uint8_t * nv12,
    std::size_t data_size,
    std::size_t input_stride);

  void updateRemap(
    const cv::Mat & map_x,
    const cv::Mat & map_y);

  const std::string & deviceName() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bev_processor

#endif  // BEV_PROCESSOR__CUDA_BEV_PROCESSOR_HPP_
