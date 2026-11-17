#include "auto_control/driving_log.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <locale>
#include <stdexcept>
#include <utility>

namespace auto_control
{
namespace
{
void quoted(std::ostream & out, const std::string & value)
{
  out << '"';
  for (const char c : value) {
    if (c == '"') {out << '"';}
    out << c;
  }
  out << '"';
}
}  // namespace

DrivingLog::DrivingLog(const std::string & directory, const std::string & header,
  const std::string & parameters)
{
  const auto root = std::filesystem::absolute(directory);
  std::filesystem::create_directories(root);
  const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  const auto base = root / ("drive_" + std::to_string(stamp));
  path_ = base.string() + ".csv";
  std::ofstream metadata(base.string() + ".parameters.txt");
  metadata << "Driving log schema: 9\nEffective controller parameters at startup\n"
           << parameters;
  metadata.close();
  if (!metadata) {throw std::runtime_error("cannot write driving log parameters");}
  output_.open(path_);
  output_.imbue(std::locale::classic());
  // 9 significant digits cover every logged float-derived quantity.
  output_ << std::setprecision(9) << header << '\n';
  output_.flush();
  if (!output_) {throw std::runtime_error("cannot open driving CSV: " + path_);}
  writer_ = std::thread(&DrivingLog::run, this);
}

DrivingLog::~DrivingLog()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  wake_.notify_one();
  if (writer_.joinable()) {writer_.join();}
}

void DrivingLog::enqueue(DrivingLogFrame frame)
{
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock() || failed_ || stopping_ || queue_.size() >= 256) {
    ++dropped_;
    return;
  }
  queue_.push_back(std::move(frame));
  // Wake periodically, or sooner when the queue fills. Never wait for disk.
  if (queue_.size() >= 128) {wake_.notify_one();}
}

void DrivingLog::run()
{
  try {
    for (;;) {
      std::deque<DrivingLogFrame> batch;
      bool done;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait_for(lock, std::chrono::seconds(1), [this] {
          return stopping_ || queue_.size() >= 128;
        });
        batch.swap(queue_);
        done = stopping_;
      }
      for (const auto & frame : batch) {
        output_ << frame.ros_ns << ',' << frame.elapsed_sec << ',' << dropped_.load();
        for (const auto & value : frame.labels) {
          output_ << ',';
          quoted(output_, value);
        }
        for (const double value : frame.values) {
          output_ << ',';
          if (std::isfinite(value)) {output_ << value;}
        }
        output_ << ",\"";
        bool first = true;
        // 0.1 mm is far below the 1 cm BEV pixel; full precision only made
        // each row several times larger.
        output_ << std::fixed << std::setprecision(4);
        for (const auto & point : frame.path) {
          if (!first) {output_ << ';';}
          first = false;
          output_ << point.x << ':' << point.y;
        }
        output_ << std::defaultfloat << std::setprecision(9) << "\"\n";
      }
      output_.flush();
      if (!output_) {failed_ = true; return;}
      if (done) {return;}
    }
  } catch (...) {failed_ = true;}
}
}  // namespace auto_control
