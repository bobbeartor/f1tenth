#pragma once

#include <opencv2/core.hpp>

#include <array>

namespace multiclass_yolo {

inline const std::array<cv::Scalar, 4> obstacle_colors{{
  {0, 0, 255}, {255, 160, 0}, {50, 220, 50}, {220, 80, 220}
}};

}  // namespace multiclass_yolo
