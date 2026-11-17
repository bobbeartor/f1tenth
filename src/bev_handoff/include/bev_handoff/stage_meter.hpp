#pragma once
// Always-on, low-overhead stage timing for the remote preview (/perf/* topics).
// add() is called from processing threads; drain() from a 1 s timer.
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>

namespace bev_handoff
{

class StageMeter
{
public:
  // Count one processed frame; duration is optional (event-only stages pass none).
  void add(std::chrono::nanoseconds duration = std::chrono::nanoseconds{-1})
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++events_;
    const auto ns = duration.count();
    if (ns >= 0 && ns < 60LL * 1000000000LL) {
      ++samples_;
      sum_ns_ += static_cast<std::uint64_t>(ns);
      if (static_cast<std::uint64_t>(ns) > max_ns_) {max_ns_ = static_cast<std::uint64_t>(ns);}
    }
  }

  // JSON object {"hz":..,"avg_ms":..,"max_ms":..} for the elapsed window, then reset.
  std::string drain(double elapsed_sec)
  {
    std::uint64_t events, samples, sum_ns, max_ns;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      events = events_; samples = samples_; sum_ns = sum_ns_; max_ns = max_ns_;
      events_ = samples_ = sum_ns_ = max_ns_ = 0U;
    }
    std::ostringstream out;
    out << std::fixed << std::setprecision(3) << "{\"hz\":"
        << (elapsed_sec > 0.0 ? static_cast<double>(events) / elapsed_sec : 0.0);
    if (samples > 0U) {
      out << ",\"avg_ms\":" << static_cast<double>(sum_ns) / static_cast<double>(samples) / 1.0e6
          << ",\"max_ms\":" << static_cast<double>(max_ns) / 1.0e6;
    } else {
      out << ",\"avg_ms\":null,\"max_ms\":null";
    }
    out << "}";
    return out.str();
  }

private:
  std::mutex mutex_;
  std::uint64_t events_{0U}, samples_{0U}, sum_ns_{0U}, max_ns_{0U};
};

}  // namespace bev_handoff
