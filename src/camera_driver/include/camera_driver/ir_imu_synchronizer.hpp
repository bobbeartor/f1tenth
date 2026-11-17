#pragma once
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <stdexcept>
#include <utility>

namespace camera_driver
{
// A bounded capture/IMU rendezvous. Never extrapolates an old correction or
// replays an image. A new frame may wait while the IMU thread catches up.
template<class Frame, class Correction>
class IrImuSynchronizer
{
public:
  using Clock = std::chrono::steady_clock;
  struct Entry {Frame frame; Clock::time_point captured, received;};
  struct Matched {Entry entry; Correction correction;};
  explicit IrImuSynchronizer(double wait_sec, std::size_t capacity=8)
  : capacity_(capacity)
  {
    if (!std::isfinite(wait_sec) || wait_sec<=0 || capacity==0) {
      throw std::invalid_argument("invalid IR/IMU synchronization bounds");
    }
    wait_=std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(wait_sec));
  }
  bool push(Frame frame, Clock::time_point captured, Clock::time_point received)
  {
    if (seen_ && captured<=last_capture_) {return false;}
    last_capture_=captured; seen_=true;
    pending_.push_back({std::move(frame),captured,received});
    while (pending_.size()>capacity_) {pending_.pop_front(); ++overflow_;}
    return true;
  }
  template<class Lookup>
  std::optional<Matched> takeReady(Clock::time_point now, Lookup lookup)
  {
    while (!pending_.empty() && now-pending_.front().received>wait_) {
      pending_.pop_front(); ++expired_;
    }
    // Most recent match wins; do not create a catch-up burst after a delay.
    for (std::size_t i=pending_.size(); i>0; --i) {
      auto correction=lookup(pending_[i-1].captured);
      if (!correction) {continue;}
      Matched result{std::move(pending_[i-1]),std::move(*correction)};
      superseded_+=i-1;
      pending_.erase(pending_.begin(),pending_.begin()+static_cast<std::ptrdiff_t>(i));
      return result;
    }
    return std::nullopt;
  }
  void clear() {pending_.clear();} // Keep monotonic capture identity across mode changes.
  std::size_t size() const {return pending_.size();}
  std::uint64_t expired() const {return expired_;}
  std::uint64_t overflow() const {return overflow_;}
  std::uint64_t superseded() const {return superseded_;}
private:
  Clock::duration wait_;
  std::size_t capacity_;
  std::deque<Entry> pending_;
  Clock::time_point last_capture_{};
  bool seen_{false};
  std::uint64_t expired_{0},overflow_{0},superseded_{0};
};
}
