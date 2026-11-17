#include "line_detactor/bev_theme.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <opencv2/imgproc.hpp>

namespace line_detactor
{
namespace
{
const cv::Scalar kGray(212, 204, 199);  // #C7CCD4 (OpenCV BGR)
const cv::Scalar kBlue(230, 153, 38);   // #2699E6
const cv::Scalar kInk(64, 49, 38), kMuted(139, 116, 100);
void text(cv::Mat & image, const std::string & value, double x, double y,
  double font, const cv::Scalar & color, double ui, bool right = false)
{
  int baseline = 0;
  const auto extent = cv::getTextSize(value, cv::FONT_HERSHEY_SIMPLEX, font * ui, 1, &baseline);
  cv::Point at(cvRound(x * ui), cvRound(y * ui));
  if (right) {at.x -= extent.width;}
  cv::putText(image, value, at, cv::FONT_HERSHEY_SIMPLEX, font * ui, color, 1, cv::LINE_AA);
}
void draw_path(cv::Mat & image, const std::vector<cv::Point2f> & path,
  const cv::Scalar & color, int width)
{
  for (std::size_t i = 1; i < path.size(); ++i) {
    cv::line(image, path[i - 1], path[i], color, width, cv::LINE_AA);
  }
}
std::vector<cv::Point2f> simplified(const std::vector<cv::Point2f> & input)
{
  std::vector<cv::Point2f> result;
  if (input.size() > 2) {cv::approxPolyDP(input, result, 0.6, false);}
  else {result = input;}
  for (auto & p : result) {p *= 2.0F;}
  return result;
}
}  // namespace

BevThemeRenderer::BevThemeRenderer(int width, int height, int padding,
  double bev_width_m, double bev_height_m, double lane_width_m)
: source_width_(width), source_height_(height), padding_(padding),
  road_height_(height * 2), width_((width + 2 * padding) * 2),
  bev_width_m_(bev_width_m), bev_height_m_(bev_height_m), lane_width_m_(lane_width_m),
  pixels_per_m_(height * 2 / bev_height_m), ui_(width_ / 360.0)
{
  edge_width_ = std::max(1, cvRound(9.0 / 510.0 * pixels_per_m_));
  center_width_ = std::max(1, cvRound(28.0 / 510.0 * pixels_per_m_));
}

void BevThemeRenderer::update(const LaneConnectionResult & result)
{
  base_.create(road_height_ + 176, width_, CV_8UC3);
  base_.setTo(cv::Scalar(255, 255, 255));
  cv::Mat road = base_(cv::Rect(0, 0, width_, road_height_));
  paths_.clear();
  const cv::Point2f axle(static_cast<float>(padding_ * 2 + source_width_),
    static_cast<float>(road_height_));
  for (std::size_t side = 0; side < result.observed_paths.size(); ++side) {
    for (const auto & source : result.observed_paths[side]) {
      if (paths_.size() >= 64U) {break;}  // Bound drawing work for noisy detections.
      auto path = simplified(source);
      if (path.size() < 2) {continue;}
      if (cv::norm(path.front() - axle) > cv::norm(path.back() - axle)) {
        std::reverse(path.begin(), path.end());
      }
      draw_path(road, path, kGray, edge_width_);
      paths_.push_back({std::move(path), side});
    }
  }
  const auto center = simplified(result.centerline.points);
  draw_path(road, center, kBlue, center_width_);
  stop_distance_m_ = result.stop_line_distance_m;
  // A simple stop bar uses the already-computed distance. Its orientation and
  // width are schematic; don't re-fit the segmentation mask in the GUI.
  if (result.stop_line_present && std::isfinite(stop_distance_m_) && center.size() >= 2) {
    const double first_x = bev_height_m_ - (center.front().y / 2.0 + 0.5) *
      bev_height_m_ / source_height_;
    const double first_y = bev_width_m_ / 2.0 - (center.front().x / 2.0 - padding_ + 0.5) *
      bev_width_m_ / source_width_;
    double walked = std::hypot(first_x, first_y);
    for (std::size_t i = 1; i < center.size(); ++i) {
      const auto delta = center[i] - center[i - 1];
      const double length = cv::norm(delta);
      if (length < 1.0e-6) {continue;}
      const double meters = length / pixels_per_m_;
      if (stop_distance_m_ >= walked && stop_distance_m_ <= walked + meters) {
        const auto tangent = delta * static_cast<float>(1.0 / length);
        const auto p = center[i - 1] + delta * static_cast<float>((stop_distance_m_ - walked) / meters);
        const cv::Point2f normal(-tangent.y, tangent.x);
        const auto half = normal * static_cast<float>(lane_width_m_ * pixels_per_m_ / 2.0);
        cv::line(road, p - half, p + half, kGray, edge_width_ * 2, cv::LINE_AA);
        break;
      }
      walked += meters;
    }
  }
  for (int dm = 1; dm <= static_cast<int>(std::floor(bev_height_m_ * 10 + 1.0e-6)); ++dm) {
    const int y = std::max(1, cvRound((bev_height_m_ - dm * 0.1) * pixels_per_m_ - 1));
    const bool major = dm % 10 == 0;
    cv::line(road, {cvRound(6 * ui_), y}, {cvRound((major ? 22 : 12) * ui_), y},
      major ? cv::Scalar(211, 203, 196) : cv::Scalar(233, 228, 224), 1, cv::LINE_AA);
    if (major) {text(base_, std::to_string(dm / 10) + "m", 28,
        std::max(11.0, y / ui_ + 3), 0.33, cv::Scalar(208, 198, 190), ui_);}
  }
  cv::line(base_, {0, road_height_}, {width_ - 1, road_height_}, cv::Scalar(241, 236, 232));
  const char * labels[] = {"Lane infer avg", "Post avg", "Control avg",
    "Traffic model avg", "Preview", "Stop distance"};
  for (int i = 0; i < 6; ++i) {
    text(base_, labels[i], 16, road_height_ / ui_ + 20 + i * 23, 0.40, kMuted, ui_);
  }
}

cv::Mat BevThemeRenderer::render(const BevThemeTelemetry & t) const
{
  if (base_.empty()) {return {};}
  cv::Mat canvas = base_.clone();
  cv::Mat road = canvas(cv::Rect(0, 0, width_, road_height_));
  const double dash = 27.0 / 510 * pixels_per_m_;
  const double period = (27.0 + 63.6) / 510 * pixels_per_m_;
  const double phase = std::fmod(std::max(0.0, t.distance_m) * pixels_per_m_, period);
  int mark_budget = 256;
  for (const auto & path : paths_) {
    double arc = 0, next = period - phase;
    for (std::size_t i = 1; i < path.points.size() && mark_budget > 0; ++i) {
      const auto delta = path.points[i] - path.points[i - 1];
      const double length = cv::norm(delta);
      if (length < 1.0e-6) {continue;}
      const auto tangent = delta * static_cast<float>(1.0 / length);
      cv::Point2f normal(-tangent.y, tangent.x);
      // Near-to-far ordering means left exterior is the clockwise normal in
      // screen coordinates. This also works when the lane turns horizontal.
      if (path.side == 0) {normal *= -1;}
      while (next <= arc + length && mark_budget-- > 0) {
        const auto p = path.points[i - 1] + tangent * static_cast<float>(next - arc) +
          normal * static_cast<float>(25.0 / 510 * pixels_per_m_);
        const auto half = tangent * static_cast<float>(dash / 2);
        cv::line(road, p - half, p + half, kGray, 1, cv::LINE_AA);
        next += period;
      }
      arc += length;
    }
  }
  const auto point = [&](double x, double y) {return cv::Point(cvRound(x * ui_), cvRound(y * ui_));};
  cv::circle(road, point(310, 50), cvRound(34 * ui_), cv::Scalar(255, 255, 255), cv::FILLED, cv::LINE_AA);
  cv::circle(road, point(310, 50), cvRound(34 * ui_), cv::Scalar(230, 222, 217), 1, cv::LINE_AA);
  const auto centered = [&](const std::string & value, double y, double font, const cv::Scalar & color) {
      int baseline = 0;
      const auto size = cv::getTextSize(value, cv::FONT_HERSHEY_SIMPLEX, font * ui_, 1, &baseline);
      text(canvas, value, 310 - size.width / (2 * ui_), y, font, color, ui_);
    };
  centered(std::isfinite(t.speed_mps) ? cv::format("%.2f", std::abs(t.speed_mps)) : "--", 50, 0.65, kInk);
  centered("m/s", 68, 0.32, kMuted);
  cv::circle(road, point(310, 114), cvRound(14 * ui_),
    t.signal == 1 ? cv::Scalar(87, 87, 235) :
    t.signal == 2 ? cv::Scalar(107, 163, 40) : cv::Scalar(192, 179, 170),
    t.signal == 1 || t.signal == 2 ? cv::FILLED : 2, cv::LINE_AA);
  const double top = road_height_ / ui_;
  const double stages[] = {t.inference_ms, t.postprocess_ms, t.control_ms};
  for (int i = 0; i < 3; ++i) {
    const bool valid = std::isfinite(stages[i]) && stages[i] > 0;
    text(canvas, valid ? cv::format("%.2f ms", stages[i]) : "-- ms",
      242, top + 20 + 23 * i, 0.36, kMuted, ui_, true);
    text(canvas, valid ? cv::format("%.1f FPS", 1000.0 / stages[i]) : "-- FPS",
      344, top + 20 + 23 * i, 0.38, kInk, ui_, true);
  }
  text(canvas, std::isfinite(t.traffic_fps) ? cv::format("%.1f FPS", t.traffic_fps) : "-- FPS",
    344, top + 89, 0.38, kInk, ui_, true);
  text(canvas, cv::format("%.1f FPS", t.preview_fps), 344, top + 112, 0.38, kInk, ui_, true);
  text(canvas, std::isfinite(stop_distance_m_) ? cv::format("%.2f m", stop_distance_m_) : "-- m",
    344, top + 135, 0.38, kInk, ui_, true);
  return canvas;
}
}  // namespace line_detactor
