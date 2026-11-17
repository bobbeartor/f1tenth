#ifndef BEV_PROCESSOR__CUDA_BEV_PROCESSOR_HPP_
#define BEV_PROCESSOR__CUDA_BEV_PROCESSOR_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <opencv2/core.hpp>

namespace bev_processor
{

enum class BevInterpolation
{
  Bilinear,
  Bicubic,
  Adaptive,
};

struct EdgeAdaptiveConfig
{
  double start_x_m{1.20};
  double full_x_m{2.00};
  double strength{0.75};
  double gradient_low{8.0};
  double gradient_high{32.0};
  double coherence_minimum{0.30};
  double maximum_anisotropy{3.0};
  double bev_x_max_m{3.0};
  double meter_per_pixel{0.01};
};

struct CudaBevResult
{
  const std::uint8_t * device_bgr{nullptr};
  std::size_t device_stride{0U};
  std::shared_ptr<const void> device_owner;
  cv::Mat bgr;
  std::function<cv::Mat()> download_bgr; // Lazy, cached; retains the source GPU allocation.
};

class CudaBevProcessor
{
public:
  CudaBevProcessor(
    int input_width,
    int input_height,
    const cv::Mat & map_x,
    const cv::Mat & map_y,
    BevInterpolation interpolation,
    const EdgeAdaptiveConfig & edge_adaptive_config);
  ~CudaBevProcessor();

  CudaBevProcessor(const CudaBevProcessor &) = delete;
  CudaBevProcessor & operator=(const CudaBevProcessor &) = delete;

  CudaBevResult process(
    const std::uint8_t * nv12,
    std::size_t data_size,
    std::size_t input_stride,
    const cv::Matx33d & stabilized_to_source_homography,
    int source_crop_top,
    bool download_to_host);

  const std::string & deviceName() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bev_processor

#endif  // BEV_PROCESSOR__CUDA_BEV_PROCESSOR_HPP_
