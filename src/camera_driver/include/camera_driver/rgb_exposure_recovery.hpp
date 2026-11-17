#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <optional>
#include <stdexcept>
#include <vector>

namespace camera_driver
{
enum class RgbTunnelPhase {outside, entering, inside, exiting};

// Camera-control policy only; never holds lane results or disables RGB frames.
// One worker owns it. A transport failure must call requestAuto() so an uncertain
// manual command is followed by AE, rather than another manual attempt.
class RgbExposureRecovery
{
public:
  using Clock = std::chrono::steady_clock;
  struct Sample {Clock::time_point captured; long exposure_us; int iso;};
  enum class Command {none, manual, automatic};
  struct Action {Command command{Command::none}; long exposure_us{0}; int iso{0};};

  explicit RgbExposureRecovery(double max_hold_sec = .20)
  : max_hold_(std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(
      checkedHold(max_hold_sec)))) {}

  Action step(RgbTunnelPhase phase, Clock::time_point now, std::optional<Sample> sample)
  {
    const bool changed = phase != phase_;
    if (changed) {
      if (phase == RgbTunnelPhase::entering || phase == RgbTunnelPhase::inside) {
        // Freeze an exposure-stable OUTSIDE window preceding the boundary.
        saveOutside(now);
        history_.clear();
      }
      phase_ = phase;
    }
    bool new_sample = false;
    if (sample && sample->captured > last_capture_ && sample->captured <= now &&
      now - sample->captured <= std::chrono::milliseconds(250) &&
      sample->exposure_us > 0 && sample->exposure_us <= 1000000 && sample->iso > 0 && sample->iso <= 65535)
    {
      last_capture_ = sample->captured;
      latest_ = sample;
      new_sample = true;
    }
    if (manual_ && (phase != RgbTunnelPhase::exiting || now < started_ || now - started_ >= max_hold_)) {
      requestAuto();
    }
    if (manual_ && new_sample && sample->captured > started_) {
      const bool matches = std::abs(sample->exposure_us - target_.exposure_us) <=
        std::max(100., target_.exposure_us * .05) && sample->iso == target_.iso;
      matched_ = matches ? matched_ + 1 : 0;
      if (matched_ >= 2) {requestAuto();}
    }
    if (auto_pending_) {return {Command::automatic, 0, 0};}
    if (changed && phase == RgbTunnelPhase::exiting) {
      if (!saved_ || !latest_ || now < latest_->captured ||
        now - latest_->captured > std::chrono::milliseconds(250)) {return {};}
      // Only lower exposure/gain: never brighten an already recovered RGB.
      target_ = *saved_;
      target_.exposure_us = std::min(target_.exposure_us, latest_->exposure_us);
      target_.iso = std::min(target_.iso, latest_->iso);
      if (target_.exposure_us == latest_->exposure_us && target_.iso == latest_->iso) {return {};}
      manual_ = true;
      matched_ = 0;
      started_ = now;
      return {Command::manual, target_.exposure_us, target_.iso};
    }
    if (phase == RgbTunnelPhase::outside && !manual_ && new_sample) {
      history_.push_back(*sample);
      while (!history_.empty() && (now - history_.front().captured > std::chrono::seconds(2) ||
        history_.size() > 256)) {history_.pop_front();}
    }
    return {};
  }

  void requestAuto() {manual_ = false; auto_pending_ = true;}
  void autoSent() {auto_pending_ = false;}
  bool restoring() const {return manual_ || auto_pending_;}
  std::optional<Sample> saved() const {return saved_;}

private:
  static double checkedHold(double value)
  {
    if (!std::isfinite(value) || value <= 0. || value > .5) {
      throw std::invalid_argument("RGB exposure recovery hold must be (0,0.5]s");
    }
    return value;
  }
  void saveOutside(Clock::time_point now)
  {
    // A later INSIDE report must not overwrite the snapshot made at ENTERING.
    if (phase_ != RgbTunnelPhase::outside) {return;}
    saved_.reset();
    // Avoid the last 300ms, which may already contain the dark entrance.
    // Search newest-to-oldest for a >=150ms stable window (both time and ISO
    // within 25%). Store an actual median sample, not a synthetic exposure.
    for (auto end = history_.rbegin(); end != history_.rend(); ++end) {
      if (end->captured > now - std::chrono::milliseconds(300) ||
        now - end->captured > std::chrono::seconds(1)) {continue;}
      std::vector<Sample> window;
      long low_us = end->exposure_us, high_us = low_us;
      int low_iso = end->iso, high_iso = low_iso;
      for (auto item = end; item != history_.rend(); ++item) {
        if (end->captured - item->captured > std::chrono::milliseconds(250)) {break;}
        low_us = std::min(low_us, item->exposure_us); high_us = std::max(high_us, item->exposure_us);
        low_iso = std::min(low_iso, item->iso); high_iso = std::max(high_iso, item->iso);
        if (high_us > low_us * 1.25 || high_iso > low_iso * 1.25) {break;}
        window.push_back(*item);
        if (window.size() >= 3 && end->captured - item->captured >= std::chrono::milliseconds(150)) {
          std::sort(window.begin(), window.end(), [](const Sample & a, const Sample & b) {
            return static_cast<double>(a.exposure_us) * a.iso < static_cast<double>(b.exposure_us) * b.iso;
          });
          saved_ = window[window.size()/2];
          return;
        }
      }
    }
  }
  RgbTunnelPhase phase_{RgbTunnelPhase::outside};
  Clock::duration max_hold_;
  Clock::time_point started_{},last_capture_{};
  std::deque<Sample> history_;
  std::optional<Sample> latest_,saved_;
  Sample target_{};
  bool manual_{false},auto_pending_{false};
  unsigned matched_{0};
};
}
