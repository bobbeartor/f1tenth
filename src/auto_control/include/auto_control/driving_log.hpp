#ifndef AUTO_CONTROL__DRIVING_LOG_HPP_
#define AUTO_CONTROL__DRIVING_LOG_HPP_

#include "auto_control/control_core.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace auto_control
{
struct DrivingLogFrame
{
  std::int64_t ros_ns{0};
  double elapsed_sec{0.0};
  std::vector<std::string> labels;
  std::vector<double> values;
  std::vector<Point> path;
};

// The controller only copies snapshots into a bounded, non-blocking queue.
// CSV formatting and disk I/O belong to the writer thread.
class DrivingLog
{
public:
  DrivingLog(const std::string & directory, const std::string & header,
    const std::string & parameters);
  ~DrivingLog();
  void enqueue(DrivingLogFrame frame);
  const std::string & path() const {return path_;}
  bool failed() const {return failed_.load();}
  std::uint64_t dropped() const {return dropped_.load();}

private:
  void run();
  std::string path_;
  std::ofstream output_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::deque<DrivingLogFrame> queue_;
  bool stopping_{false};
  std::atomic<bool> failed_{false};
  std::atomic<std::uint64_t> dropped_{0};
  std::thread writer_;
};
}  // namespace auto_control

#endif  // AUTO_CONTROL__DRIVING_LOG_HPP_
