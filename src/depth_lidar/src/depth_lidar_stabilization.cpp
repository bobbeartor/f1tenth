#include "depth_lidar/depth_lidar_stabilization.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace depth_lidar {
namespace {
constexpr float unknown = std::numeric_limits<float>::quiet_NaN();
}
ScanHold::ScanHold(int confirm_hits, int window_frames, double confirm_distance_m)
: confirm_hits_(confirm_hits), window_frames_(window_frames), confirm_distance_m_(confirm_distance_m)
{
  if (window_frames < 1 || window_frames > 30 || confirm_hits < 1 || confirm_hits > window_frames
      || !std::isfinite(confirm_distance_m) || confirm_distance_m <= 0.0) {
    throw std::invalid_argument("invalid scan confirmation window or range tolerance");
  }
}
void ScanHold::clear()
{
  history_.clear(); ranges_.clear(); times_.clear(); next_slot_ = 0;
  previous_update_time_ = -std::numeric_limits<double>::infinity();
  next_expiry_ = std::numeric_limits<double>::infinity();
}
ScanResult ScanHold::render(double now, double hold)
{
  ScanResult result;
  result.ranges = ranges_;
  result.ages.assign(ranges_.size(), unknown);
  next_expiry_ = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < ranges_.size(); ++i) {
    if (!std::isfinite(ranges_[i])) { continue; }
    if (now - times_[i] > hold || now < times_[i]) {
      ranges_[i] = result.ranges[i] = unknown;
      continue;
    }
    result.ages[i] = static_cast<float>(now - times_[i]);
    ++result.valid_bins;
    // With hold disabled, retain this frame for display until the next frame/stale timeout.
    if (hold > 0.0) { next_expiry_ = std::min(next_expiry_, times_[i] + hold); }
  }
  return result;
}
ScanResult ScanHold::update(const ScanResult & raw, double now, double hold)
{
  if (!std::isfinite(now) || !std::isfinite(hold) || hold < 0.0) {
    throw std::invalid_argument("invalid scan observation time or hold duration");
  }
  // Duplicate observations must not count as additional supporting frames.
  if (now <= previous_update_time_) {
    if (now < previous_update_time_) { clear(); }
    else { return render(now, hold); }
  }
  if (ranges_.size() != raw.ranges.size()) {
    clear();
    ranges_.assign(raw.ranges.size(), unknown);
    times_.assign(raw.ranges.size(), 0.0);
    history_.assign(raw.ranges.size() * window_frames_, unknown);
  }
  previous_update_time_ = now;
  const auto bins = raw.ranges.size();
  for (std::size_t i = 0; i < bins; ++i) {
    const float range = raw.ranges[i];
    history_[next_slot_ * bins + i] = std::isfinite(range) ? range : unknown;
    if (!std::isfinite(range)) { continue; }
    int hits = 0;
    for (int frame = 0; frame < window_frames_; ++frame) {
      const float previous = history_[static_cast<std::size_t>(frame) * bins + i];
      if (std::isfinite(previous) && std::abs(previous - range) <= confirm_distance_m_) { ++hits; }
    }
    // Apply confirmation continuously, including already visible bins. Alternating
    // single-frame noise cannot refresh the TTL without enough recent support.
    if (hits >= confirm_hits_) { ranges_[i] = range; times_[i] = now; }
  }
  next_slot_ = (next_slot_ + 1) % static_cast<std::size_t>(window_frames_);
  auto result = render(now, hold);
  result.accepted_points = raw.accepted_points;
  return result;
}
ScanResult ScanHold::snapshot(double now, double hold) { return render(now, hold); }
}
