#include "line_detactor/stop_line_distance.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace
{

std::vector<cv::Point2f> straight_centerline()
{
  std::vector<cv::Point2f> points;
  points.reserve(300U);
  for (int row = 299; row >= 0; --row) {
    points.emplace_back(90.0F, static_cast<float>(row));  // 60 source px + 30 px padding.
  }
  return points;
}

void require(const bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

}  // namespace

int main()
{
  const auto centerline = straight_centerline();
  cv::Mat horizontal = cv::Mat::zeros(300, 120, CV_8UC1);
  cv::rectangle(horizontal, cv::Point(15, 148), cv::Point(105, 152), cv::Scalar(255), -1);
  const double horizontal_distance = line_detactor::estimate_stop_line_distance_m(
    horizontal, centerline, 30, 1.2, 3.0);
  require(std::isfinite(horizontal_distance), "horizontal stop line was not measured");
  require(std::abs(horizontal_distance - 1.47) < 0.04, "horizontal distance is inaccurate");

  cv::Mat angled = cv::Mat::zeros(300, 120, CV_8UC1);
  cv::line(angled, cv::Point(15, 160), cv::Point(105, 140), cv::Scalar(255), 5);
  const double angled_distance = line_detactor::estimate_stop_line_distance_m(
    angled, centerline, 30, 1.2, 3.0);
  require(std::isfinite(angled_distance), "angled stop line was not measured");
  require(std::abs(angled_distance - 1.46) < 0.06, "angled distance is inaccurate");

  const cv::Mat empty = cv::Mat::zeros(300, 120, CV_8UC1);
  require(std::isnan(line_detactor::estimate_stop_line_distance_m(
    empty, centerline, 30, 1.2, 3.0)), "empty mask must not produce a distance");

  constexpr int kIterations = 10000;
  const auto started = std::chrono::steady_clock::now();
  double checksum = 0.0;
  for (int index = 0; index < kIterations; ++index) {
    checksum += line_detactor::estimate_stop_line_distance_m(
      angled, centerline, 30, 1.2, 3.0);
  }
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  require(checksum > 0.0, "benchmark result is invalid");
  std::cout << "stop-line distance smoke test passed; host benchmark=" <<
    elapsed_ms / kIterations << " ms/call\n";
  return 0;
}
