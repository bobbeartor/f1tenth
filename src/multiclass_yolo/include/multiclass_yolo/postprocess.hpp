#pragma once
#include "multiclass_yolo/yolox_detector.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace multiclass_yolo {
inline std::vector<Detection> decode_detections(
  const float * rows, int count, int columns, float ratio, int width, int height,
  int origin_x, int origin_y, const std::array<float, 4> & thresholds, float nms,
  std::size_t max_detections = 0)
{
  if (!rows || count <= 0 || columns != 9 || !std::isfinite(ratio) || ratio <= 0 ||
      width <= 0 || height <= 0) throw std::invalid_argument("Expected decoded [1,N,9]");
  std::vector<Detection> candidates;
  for (int i = 0; i < count; ++i) {
    const float * r = rows + i * columns;
    bool valid = true;
    for (int j = 0; j < columns; ++j) valid = valid && std::isfinite(r[j]);
    if (!valid || r[2] <= 0 || r[3] <= 0) continue;
    int cls = static_cast<int>(std::max_element(r + 5, r + 9) - (r + 5));
    float score = r[4] * r[5 + cls];
    if (score <= 0 || score < thresholds[cls]) continue;
    float x1 = std::clamp((r[0] - r[2] / 2) / ratio, 0.f, float(width));
    float y1 = std::clamp((r[1] - r[3] / 2) / ratio, 0.f, float(height));
    float x2 = std::clamp((r[0] + r[2] / 2) / ratio, 0.f, float(width));
    float y2 = std::clamp((r[1] + r[3] / 2) / ratio, 0.f, float(height));
    if (x2 <= x1 || y2 <= y1) continue;
    candidates.push_back({cv::Rect2f(x1 + origin_x, y1 + origin_y, x2-x1, y2-y1), score, cls});
  }
  std::stable_sort(candidates.begin(), candidates.end(),
    [](const auto & a, const auto & b) { return a.score > b.score; });
  std::vector<Detection> kept;
  for (const auto & c : candidates) {
    bool suppressed = false;
    for (const auto & k : kept) {
      if (c.class_id != k.class_id) continue;
      float intersection = (c.box & k.box).area();
      float union_area = c.box.area() + k.box.area() - intersection;
      if (union_area > 0 && intersection / union_area > nms) { suppressed = true; break; }
    }
    if (suppressed) continue;
    kept.push_back(c);
    if (max_detections && kept.size() == max_detections) break;
  }
  return kept;
}
}  // namespace multiclass_yolo
