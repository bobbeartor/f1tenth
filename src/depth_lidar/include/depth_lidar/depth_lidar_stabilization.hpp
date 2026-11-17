#pragma once
#include "depth_lidar/depth_lidar_geometry.hpp"
#include <limits>
namespace depth_lidar {
// Per-angle range confirmation and short missing-return hold; no motion model.
class ScanHold {
public:
  ScanHold(int confirm_hits = 3, int window_frames = 4, double confirm_distance_m = 0.10);
  void clear();
  ScanResult update(const ScanResult & raw, double host_time_sec, double hold_sec);
  ScanResult snapshot(double host_time_sec, double hold_sec);
  double nextExpiryTime() const { return next_expiry_; }
private:
  ScanResult render(double host_time_sec, double hold_sec);
  int confirm_hits_, window_frames_;
  double confirm_distance_m_;
  std::size_t next_slot_{0};
  // Bounded ring: one row per processed depth frame, one column per angular bin.
  std::vector<float> history_;
  std::vector<float> ranges_;
  std::vector<double> times_;
  double previous_update_time_{-std::numeric_limits<double>::infinity()};
  double next_expiry_{std::numeric_limits<double>::infinity()};
};
}
