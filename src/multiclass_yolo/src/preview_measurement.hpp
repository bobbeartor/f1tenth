#pragma once

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <locale>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace multiclass_yolo {

using PreviewClock = std::chrono::steady_clock;

inline double milliseconds(PreviewClock::duration duration) {
  return std::chrono::duration<double, std::milli>(duration).count();
}

enum class Stage : std::size_t {
  queue, input_color, cpu_preprocess, upload, cuda_preprocess, inference,
  download, postprocess, display_color, overlay, panel, imshow, wait_key,
  processing, receive_to_result, receive_to_submit, depth_fusion, rgb_depth_skew, receive_to_position,
  depth_association, bev_render, count
};

struct MeasurementFrame {
  static constexpr std::array<const char *, static_cast<std::size_t>(Stage::count)> columns{{
    "queue_ms", "input_color_ms", "cpu_preprocess_ms", "upload_ms", "cuda_preprocess_ms",
    "inference_ms", "download_ms", "postprocess_ms", "display_color_ms", "overlay_resize_ms",
    "panel_ms", "imshow_ms", "wait_key_ms", "processing_ms", "receive_to_result_ms", "receive_to_submit_ms",
    "depth_fusion_ms", "rgb_depth_skew_ms", "receive_to_position_ms", "depth_association_ms", "bev_render_ms"
  }};
  std::int64_t sequence{};
  std::uint64_t host_skipped{};
  std::size_t detections{};
  double started_ms{};
  std::array<std::optional<double>, columns.size()> times;

  std::optional<double> & operator[](Stage stage) {return times[static_cast<std::size_t>(stage)];}
};

struct CaptureMeasurement {
  PreviewClock::time_point start, end;
  std::uint64_t base{}, received{}, sequence_gaps{};
  std::optional<std::int64_t> previous_sequence;

  void record(PreviewClock::time_point time, std::int64_t sequence) {
    if (time < start || time >= end) return;
    ++received;
    if (sequence < 0) return;
    if (previous_sequence && sequence > *previous_sequence)
      sequence_gaps += static_cast<std::uint64_t>(sequence - *previous_sequence - 1);
    previous_sequence = sequence;
  }
};

class PreviewMeasurement {
 public:
  PreviewMeasurement(PreviewClock::time_point start, int seconds, nlohmann::json settings)
  : start(start), end(start + std::chrono::seconds(seconds)), settings_(std::move(settings)) {}

  bool contains(PreviewClock::time_point time) const {return time >= start && time < end;}

  void record(MeasurementFrame frame, PreviewClock::time_point result,
    std::optional<PreviewClock::time_point> position, std::optional<PreviewClock::time_point> submitted)
  {
    if (contains(result)) ++completed_;
    if (position && contains(*position)) ++positions_;
    if (submitted && contains(*submitted)) ++submitted_;
    skipped_ += frame.host_skipped;
    frames_.push_back(std::move(frame));
  }

  std::filesystem::path save(const std::filesystem::path & root, PreviewClock::time_point stop,
    const CaptureMeasurement & capture, const char * reason) const
  {
    std::filesystem::create_directories(root);
    const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    const auto base = "measurement_" + std::to_string(stamp);
    auto directory = root / base;
    for (unsigned suffix = 1; !std::filesystem::create_directory(directory); ++suffix)
      directory = root / (base + "_" + std::to_string(suffix));

    std::ofstream csv(directory / "frames.csv");
    csv.imbue(std::locale::classic());
    csv << std::fixed << std::setprecision(6) << "sequence,started_ms,host_skipped,detections";
    for (const auto * name : MeasurementFrame::columns) csv << ',' << name;
    csv << '\n';
    for (const auto & frame : frames_) {
      csv << frame.sequence << ',' << frame.started_ms << ',' << frame.host_skipped << ',' << frame.detections;
      for (const auto & time : frame.times) {
        csv << ',';
        if (time) csv << *time;
      }
      csv << '\n';
    }
    csv.close();
    if (!csv) throw std::runtime_error("Cannot write " + (directory / "frames.csv").string());

    const double seconds = std::chrono::duration<double>(std::min(stop, end) - start).count();
    const auto fps = [seconds](std::uint64_t count) {return seconds > 0 ? count / seconds : 0.0;};
    nlohmann::json stats = nlohmann::json::object();
    for (std::size_t i = 0; i < MeasurementFrame::columns.size(); ++i) {
      std::vector<double> values;
      values.reserve(frames_.size());
      for (const auto & frame : frames_) {
        if (frame.times[i]) values.push_back(*frame.times[i]);
      }
      if (values.empty()) continue;
      std::sort(values.begin(), values.end());
      const auto middle = values.size() / 2;
      stats[MeasurementFrame::columns[i]] = {
        {"samples", values.size()},
        {"p50", values.size() % 2 ? values[middle] : (values[middle - 1] + values[middle]) / 2},
        {"p95", values[static_cast<std::size_t>(std::ceil(values.size() * .95)) - 1]}
      };
    }

    const nlohmann::json summary = {
      {"settings", settings_}, {"end_reason", reason}, {"duration_seconds", seconds},
      {"sampled_frames", frames_.size()}, {"received_frames", capture.received},
      {"detection_completed_frames", completed_}, {"position_completed_frames", positions_}, {"preview_submitted_frames", submitted_},
      {"received_fps", fps(capture.received)}, {"detection_fps", fps(completed_)}, {"position_fps", fps(positions_)},
      {"preview_submission_fps", fps(submitted_)}, {"host_skipped_frames", skipped_},
      {"received_sequence_gaps", capture.sequence_gaps}, {"stages_ms", stats}
    };
    std::ofstream output(directory / "summary.json");
    output << summary.dump(2) << '\n';
    output.close();
    if (!output) throw std::runtime_error("Cannot write " + (directory / "summary.json").string());
    return directory;
  }

  const PreviewClock::time_point start, end;

 private:
  nlohmann::json settings_;
  std::vector<MeasurementFrame> frames_;
  std::uint64_t completed_{}, positions_{}, submitted_{}, skipped_{};
};

}  // namespace multiclass_yolo
