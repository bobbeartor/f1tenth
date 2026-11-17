#include "line_detactor/processing_profile.hpp"

#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <locale>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unistd.h>

namespace line_detactor
{
namespace
{
constexpr std::size_t capacity = 4096U;
std::string single_line(std::string value)
{
  for (auto & c : value) {if (c == '\n' || c == '\r') {c = ' ';}}
  return value;
}
}
class ProcessingProfileWriter::Impl
{
public:
  Impl(const std::string & directory,
    const std::vector<std::pair<std::string, std::string>> & metadata)
  {
    if (directory.empty()) {throw std::invalid_argument("profiling_directory must not be empty");}
    const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    std::filesystem::create_directories(directory);
    // Atomically claim a run directory, including when several nodes start together.
    const auto base = std::filesystem::absolute(directory) /
      ("lane_profile_" + std::to_string(stamp) + "_" + std::to_string(getpid()));
    auto run = base;
    for (unsigned suffix = 1U; !std::filesystem::create_directory(run); ++suffix) {
      run = base.string() + "_" + std::to_string(suffix);
    }
    path_ = (run / "profile.csv").string();
    output_.open(path_);
    if (!output_) {throw std::runtime_error("Cannot open lane profile: " + path_);}
    output_.imbue(std::locale::classic());
    output_ << "# schema=lane_processing_profile_v1\n"
      << "# unix_start_ns=" << stamp << '\n'
      << "# durations=milliseconds; CPU wall clock and CUDA events are separate\n"
      << "# nested=connector includes components/skeleton/border/bridge/lane_render; "
      "skeleton includes thinning/skeleton_graph; centerline includes boundary_index/"
      "fragment_prepare/outer_reference/pairing_index/candidates/path_search/path_output; "
      "path_output includes path_check_before/smoothing/path_check_after\n"
      << "# correction_ms=GPU label export plus CPU connector/centerline/stop/render; "
      "total_ms=worker start through publish, excludes logger enqueue and preview handoff\n"
      << "# warmup=engine warmup excluded; initial live frames retained; "
      "zero stage means not reached or below timer resolution\n";
    for (const auto & item : metadata) {
      output_ << "# " << single_line(item.first) << '=' << single_line(item.second) << '\n';
    }
    output_ << "generation,source_stamp_ns,received_stamp_ns,elapsed_ms,queue_wait_ms,"
      "total_ms,correction_ms,gpu_preprocess_ms,gpu_inference_ms,gpu_label_export_ms,"
      "gpu_postprocess_ms,skipped_total,success,render_result,sample_limit_reached";
#define LANE_HEADER(name) output_ << "," #name "_ms";
    LANE_PROFILE_STAGES(LANE_HEADER)
#undef LANE_HEADER
#define LANE_HEADER(name) output_ << "," #name "_count";
    LANE_PROFILE_COUNTERS(LANE_HEADER)
#undef LANE_HEADER
    output_ << ",logger_dropped_total\n" << std::fixed << std::setprecision(6);
    output_.flush();
    if (!output_) {throw std::runtime_error("Cannot write lane profile: " + path_);}
    pending_.reserve(capacity);
    worker_ = std::thread([this]() {write_loop();});
  }
  ~Impl() {close();}
  void close()
  {
    {std::lock_guard<std::mutex> lock(mutex_); stopping_ = true;}
    condition_.notify_one();
    if (worker_.joinable()) {worker_.join();}
  }
  void write_loop() noexcept
  {
    try {
      std::vector<ProfileFrame> batch;
      batch.reserve(capacity);
      for (;;) {
        bool stopping;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          condition_.wait_for(lock, std::chrono::seconds(1), [this]() {return stopping_;});
          pending_.swap(batch);
          stopping = stopping_;
        }
        for (const auto & f : batch) {
          output_ << f.generation << ',' << f.source_stamp_ns << ',' << f.received_stamp_ns
            << ',' << f.elapsed_ms << ',' << f.queue_wait_ms << ',' << f.total_ms
            << ',' << f.correction_ms << ',' << f.gpu_preprocess_ms << ',' << f.gpu_inference_ms
            << ',' << f.gpu_label_export_ms << ',' << f.gpu_postprocess_ms
            << ',' << f.skipped_total << ',' << f.success << ',' << f.render_result
            << ',' << f.sample_limit_reached;
          for (const auto ns : f.detail.ns) {output_ << ',' << static_cast<double>(ns) / 1.0e6;}
          for (const auto count : f.detail.counts) {output_ << ',' << count;}
          output_ << ',' << dropped_.load(std::memory_order_relaxed) << '\n';
        }
        batch.clear();
        if (stopping) {
          output_ << "# complete=true; logger_dropped_total=" << dropped_.load() << '\n';
        }
        output_.flush();
        if (!output_) {failed_.store(true); return;}
        if (stopping) {return;}
      }
    } catch (...) {failed_.store(true);}
  }
  void submit(const ProfileFrame & frame)
  {
    // Never wait for the writer to release the queue mutex on the processing thread.
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock() || stopping_ || failed_.load() || pending_.size() == capacity) {
      dropped_.fetch_add(1U, std::memory_order_relaxed);
      return;
    }
    pending_.push_back(frame);
  }
  std::string path_;
  std::ofstream output_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<ProfileFrame> pending_;
  bool stopping_{false};
  std::atomic<std::uint64_t> dropped_{0U};
  std::atomic<bool> failed_{false};
  std::thread worker_;
};
ProcessingProfileWriter::ProcessingProfileWriter(const std::string & directory,
  const std::vector<std::pair<std::string, std::string>> & metadata)
: impl_(std::make_unique<Impl>(directory, metadata)) {}
ProcessingProfileWriter::~ProcessingProfileWriter() = default;
void ProcessingProfileWriter::submit(const ProfileFrame & frame) {impl_->submit(frame);}
void ProcessingProfileWriter::close() {impl_->close();}
std::uint64_t ProcessingProfileWriter::dropped() const {return impl_->dropped_.load();}
bool ProcessingProfileWriter::failed() const {return impl_->failed_.load();}
const std::string & ProcessingProfileWriter::path() const {return impl_->path_;}
}  // namespace line_detactor
