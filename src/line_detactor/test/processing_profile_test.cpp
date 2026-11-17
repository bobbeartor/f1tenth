#include "line_detactor/lane_connector.hpp"
#include "line_detactor/processing_profile.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <opencv2/imgproc.hpp>
#include <unistd.h>

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}
bool equal(const cv::Mat & a, const cv::Mat & b)
{
  return a.size() == b.size() && a.type() == b.type() &&
         (a.empty() || cv::norm(a, b, cv::NORM_INF) == 0.0);
}
std::vector<std::string> fields(const std::string & line)
{
  std::istringstream input(line);
  std::vector<std::string> result;
  std::string field;
  while (std::getline(input, field, ',')) {result.push_back(field);}
  return result;
}
}

int main()
{
  using namespace line_detactor;
  cv::Mat straight = cv::Mat::zeros(300, 120, CV_8UC1);
  cv::line(straight, {27, 299}, {27, 0}, cv::Scalar(1), 5);
  cv::line(straight, {92, 299}, {92, 0}, cv::Scalar(2), 5);
  cv::Mat corner = cv::Mat::zeros(straight.size(), straight.type());
  cv::polylines(corner, std::vector<cv::Point>{{27, 299}, {27, 210}, {45, 170}, {119, 150}},
    false, cv::Scalar(1), 5);
  cv::polylines(corner, std::vector<cv::Point>{{92, 299}, {92, 230}, {110, 220}, {119, 220}},
    false, cv::Scalar(2), 5);
  cv::Mat fragmented = straight.clone();
  fragmented.rowRange(145, 150).setTo(0);
  cv::rectangle(fragmented, {2, 2}, {3, 3}, cv::Scalar(1), -1);
  cv::Mat border = cv::Mat::zeros(straight.size(), straight.type());
  cv::line(border, {20, 299}, {0, 250}, cv::Scalar(1), 3);
  cv::line(border, {0, 220}, {20, 170}, cv::Scalar(1), 3);
  const std::vector<cv::Mat> masks{
    straight, corner, fragmented, border, cv::Mat::zeros(straight.size(), straight.type())};
  unsigned cases = 0U;
  bool saw_path = false, saw_limit = false, saw_bridge = false;
  for (const auto & mask : masks) {
    for (const bool render : {false, true}) {
      for (const bool smoothing : {false, true}) {
        for (const int factor : {1, 2}) {
          for (const int budget : {32, 2000}) {
            LaneConnectionConfig connection;
            connection.skeleton_downsample_factor = factor;
            CenterlineConfig config;
            config.sample_spacing_m = 0.03;
            config.output_spacing_m = 0.02;
            config.clearance_check_spacing_m = 0.015;
            config.max_samples = budget;
            config.smoothing_enabled = smoothing;
            config.corner_outward_offset_m = 0.02;
            ProcessingProfile profile;
            const auto plain = connect_lane_fragments(mask, connection, render);
            const auto measured = connect_lane_fragments(mask, connection, render, &profile);
            require(equal(plain.labels, measured.labels) && equal(plain.image, measured.image),
              "profiling changed lane pixels");
            require(plain.observed_paths == measured.observed_paths && plain.state == measured.state,
              "profiling changed observed paths");
            const auto a = generate_centerline(plain.labels, plain.observed_paths,
              120, 30, config, render);
            const auto b = generate_centerline(measured.labels, measured.observed_paths,
              120, 30, config, render, &profile);
            require(a.points == b.points && a.support == b.support && equal(a.mask, b.mask) &&
              a.sample_limit_reached == b.sample_limit_reached, "profiling changed centerline");
            const auto count = [&](ProfileCounter id) {return profile.counts[static_cast<std::size_t>(id)];};
            const auto duration = [&](ProfileStage id) {return profile.ns[static_cast<std::size_t>(id)];};
            require(count(ProfileCounter::output_points) == b.points.size(), "output count mismatch");
            require(duration(ProfileStage::connector) >= duration(ProfileStage::skeleton) &&
              duration(ProfileStage::skeleton) >= duration(ProfileStage::thinning),
              "nested skeleton timing mismatch");
            require(duration(ProfileStage::centerline) >= duration(ProfileStage::path_search),
              "nested centerline timing mismatch");
            if (!b.points.empty()) {
              saw_path = true;
              require(count(ProfileCounter::clearance_queries) > 0, "clearance checks missing");
              require(count(ProfileCounter::path_checks) >= 1, "final path validation missing");
              if (!smoothing) {
                require(count(ProfileCounter::path_checks) == 1, "unchanged path was checked twice");
              }
            }
            saw_limit = saw_limit || b.sample_limit_reached;
            saw_bridge = saw_bridge || count(ProfileCounter::valid_bridges) > 0U;
            ++cases;
          }
        }
      }
    }
  }
  require(saw_path && saw_limit && saw_bridge, "fixtures did not exercise path, limit and bridge cases");

  const auto root = std::filesystem::temp_directory_path() /
    ("lane_profile_test_" + std::to_string(getpid()) + "_" + std::to_string(
      ProfileTimer::Clock::now().time_since_epoch().count()));
  std::string path;
  std::uint64_t dropped = 0U;
  {
    ProcessingProfileWriter writer(root.string(), {{"parameter.test", "true\nnext"}});
    path = writer.path();
    ProfileFrame frame;
    frame.success = true;
    frame.total_ms = 12.5;
    frame.detail.ns[static_cast<std::size_t>(ProfileStage::connector)] = 1500000U;
    frame.detail.add(ProfileCounter::output_points, 42U);
    // Exceed the queue capacity to exercise loss accounting as well as flushing.
    for (std::uint64_t i = 1; i <= 5000; ++i) {
      frame.generation = i;
      writer.submit(frame);
    }
    // Shutdown must drain even before the first periodic flush. Repeated close is safe.
    writer.close();
    writer.close();
    dropped = writer.dropped();
    require(!writer.failed(), "writer failed");
    ProcessingProfileWriter other(root.string(), {});
    require(other.path() != writer.path(), "new run overwrote a prior file");
    // Confirm the file is usable during acquisition, before a clean shutdown.
    bool flushed = false;
    for (int attempt = 0; attempt < 30 && !flushed; ++attempt) {
      other.submit(frame);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      std::ifstream live(other.path());
      std::string live_line;
      while (std::getline(live, live_line)) {
        if (live_line.rfind("5000,", 0U) == 0U) {flushed = true;}
      }
    }
    require(flushed, "periodic write did not flush live frames");
  }
  std::ifstream input(path);
  std::string line;
  std::vector<std::string> header;
  unsigned rows = 0U;
  bool completed = false, metadata = false;
  while (std::getline(input, line)) {
    if (line.rfind("# complete=true", 0U) == 0U) {completed = true;}
    if (line == "# parameter.test=true next") {metadata = true;}
    if (line.empty() || line.front() == '#') {continue;}
    if (header.empty()) {header = fields(line); continue;}
    const auto row = fields(line);
    require(row.size() == header.size(), "CSV column mismatch");
    const auto value = [&](const std::string & name) {
        const auto found = std::find(header.begin(), header.end(), name);
        require(found != header.end(), "missing CSV column");
        return row[static_cast<std::size_t>(found - header.begin())];
      };
    require(value("connector_ms") == "1.500000" && value("output_points_count") == "42" &&
      value("total_ms") == "12.500000", "CSV units or field order incorrect");
    ++rows;
  }
  require(completed && metadata && rows > 0U && rows + dropped == 5000U,
    "writer lost rows without accounting or omitted completion/metadata");
  bool invalid_path_rejected = false;
  try {ProcessingProfileWriter invalid(path + "/child", {});}
  catch (const std::exception &) {invalid_path_rejected = true;}
  require(invalid_path_rejected, "invalid output path accepted");
  std::filesystem::remove_all(root);
  std::cout << cases << " profile on/off comparisons and CSV writer checks passed\n";
}
