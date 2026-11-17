#pragma once
#include <cstdint>
#include <optional>

namespace auto_control
{
enum class CapturePairOrder { New, Duplicate, Regressed };

// Either observation may advance independently; neither may go backwards.
inline CapturePairOrder capture_pair_order(
  std::int64_t lane, std::int64_t depth,
  const std::optional<std::int64_t>& previous_lane,
  const std::optional<std::int64_t>& previous_depth)
{
  if ((previous_lane && lane < *previous_lane) ||
      (previous_depth && depth < *previous_depth)) {return CapturePairOrder::Regressed;}
  if (previous_lane && previous_depth && lane == *previous_lane && depth == *previous_depth) {
    return CapturePairOrder::Duplicate;
  }
  return CapturePairOrder::New;
}
}
