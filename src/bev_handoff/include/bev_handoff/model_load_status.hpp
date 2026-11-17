#pragma once
#include <chrono>
#include <condition_variable>
#include <functional>
#include <exception>
#include <utility>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

namespace bev_handoff {
// A heartbeat proves only that this reporter is alive, not that TensorRT advances.
// Stage changes come from the actual loader; only trial inference may report READY.
class ModelLoadStatus {
  using Clock = std::chrono::steady_clock;
  std::function<void(const std::string &)> publish_;
  std::mutex mutex_;
  std::condition_variable condition_;
  Clock::time_point started_{Clock::now()}, stage_at_{started_};
  std::string status_;
  bool stopping_{false};
  unsigned sequence_{0};
  std::thread worker_;

  void send() {
    if (status_.empty()) {return;}
    auto text = status_;
    {
      const auto now = Clock::now();
      std::ostringstream out;
      out << text << " | elapsed=" << std::chrono::duration_cast<std::chrono::seconds>(now-started_).count()
          << "s | stage_age=" << std::chrono::duration_cast<std::chrono::seconds>(now-stage_at_).count()
          << "s";
      if (status_.rfind("LOADING | ", 0) == 0) {out << " | monitor_seq=" << ++sequence_;}
      text = out.str();
    }
    try {publish_(text);} catch (const std::exception &) {}
  }
public:
  explicit ModelLoadStatus(std::function<void(const std::string &)> publish)
  : publish_(std::move(publish)), worker_([this] {
      std::unique_lock<std::mutex> lock(mutex_);
      while (!condition_.wait_for(lock, std::chrono::seconds(1), [this] {return stopping_;})) {
        if (status_.rfind("LOADING | ", 0) == 0) {send();}
      }
    }) {}
  void update(const std::string & status) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (status != status_) {stage_at_ = Clock::now();}
    status_ = status;
    send();
  }
  ~ModelLoadStatus() {
    {std::lock_guard<std::mutex> lock(mutex_); stopping_ = true;}
    condition_.notify_all();
    worker_.join();
  }
};
}
