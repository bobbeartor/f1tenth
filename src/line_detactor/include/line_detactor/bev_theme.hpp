#ifndef LINE_DETACTOR__BEV_THEME_HPP_
#define LINE_DETACTOR__BEV_THEME_HPP_

#include <limits>
#include "line_detactor/lane_connector.hpp"

namespace line_detactor
{
struct BevThemeTelemetry
{
  double speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double distance_m{0.0};
  double inference_ms{0.0};
  double postprocess_ms{std::numeric_limits<double>::quiet_NaN()};
  double control_ms{std::numeric_limits<double>::quiet_NaN()};
  double traffic_fps{std::numeric_limits<double>::quiet_NaN()};
  double preview_fps{0.0};
  std::uint8_t signal{0};
};

// GUI-thread-only, display-only cache. No raster mask processing or inference.
// Geometry updates once per displayed result, independently of worker progress.
class BevThemeRenderer
{
public:
  BevThemeRenderer(int width, int height, int padding, double bev_width_m,
    double bev_height_m, double lane_width_m);
  void update(const LaneConnectionResult & result);
  cv::Mat render(const BevThemeTelemetry & telemetry) const;
private:
  int source_width_, source_height_, padding_, road_height_, width_, edge_width_, center_width_;
  double bev_width_m_, bev_height_m_, lane_width_m_, pixels_per_m_, ui_;
  cv::Mat base_;
  struct MarkPath {std::vector<cv::Point2f> points; std::size_t side;};
  std::vector<MarkPath> paths_;
  double stop_distance_m_{std::numeric_limits<double>::quiet_NaN()};
};
}  // namespace line_detactor
#endif
