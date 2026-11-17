#ifndef LINE_DETACTOR__PROCESSING_PROFILE_HPP_
#define LINE_DETACTOR__PROCESSING_PROFILE_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace line_detactor
{
// Parent durations include their children; CSV columns must not all be summed.
#define LANE_PROFILE_STAGES(X) \
  X(backend_wall) X(connector) X(components) X(skeleton) X(thinning) X(skeleton_graph) \
  X(border_endpoints) X(bridge_candidates) X(bridge_render) X(lane_render) \
  X(centerline) X(boundary_index) X(fragment_prepare) X(outer_reference) \
  X(pairing_index) X(candidates) X(path_search) X(path_output) \
  X(path_check_before) X(smoothing) X(path_check_after) \
  X(stop_mask_render) X(stop_distance) X(centerline_render) X(publish)
#define LANE_PROFILE_COUNTERS(X) \
  X(components_found) X(components_retained) X(skeleton_pixels) X(thinning_passes) \
  X(thinning_pixel_visits) X(border_endpoints) X(bridge_pairs) X(valid_bridges) \
  X(boundary_pixels) X(fragments) X(samples) X(pairing_neighbors) X(candidates) \
  X(graph_neighbor_visits) X(clearance_queries) X(clearance_pixel_tests) \
  X(segment_checks) X(path_checks) X(output_points) X(graph_cost_rejections)

enum class ProfileStage {
#define LANE_ENUM(name) name,
  LANE_PROFILE_STAGES(LANE_ENUM)
#undef LANE_ENUM
  count
};
enum class ProfileCounter {
#define LANE_ENUM(name) name,
  LANE_PROFILE_COUNTERS(LANE_ENUM)
#undef LANE_ENUM
  count
};
struct ProcessingProfile
{
  std::array<std::uint64_t, static_cast<std::size_t>(ProfileStage::count)> ns{};
  std::array<std::uint64_t, static_cast<std::size_t>(ProfileCounter::count)> counts{};
  void add(ProfileCounter counter, std::uint64_t n = 1U)
  {counts[static_cast<std::size_t>(counter)] += n;}
};
class ProfileTimer
{
public:
  using Clock = std::chrono::steady_clock;
  ProfileTimer(ProcessingProfile * profile, ProfileStage stage)
  : profile_(profile), stage_(stage), started_(profile ? Clock::now() : Clock::time_point{}) {}
  ~ProfileTimer() {stop();}
  ProfileTimer(const ProfileTimer &) = delete;
  ProfileTimer & operator=(const ProfileTimer &) = delete;
  void stop()
  {
    if (profile_) {
      profile_->ns[static_cast<std::size_t>(stage_)] +=
        std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - started_).count();
      profile_ = nullptr;
    }
  }
private:
  ProcessingProfile * profile_;
  ProfileStage stage_;
  Clock::time_point started_;
};
struct ProfileFrame
{
  ProcessingProfile detail;
  std::uint64_t generation{0}, skipped_total{0};
  std::int64_t source_stamp_ns{0}, received_stamp_ns{0};
  double elapsed_ms{0}, queue_wait_ms{0}, total_ms{0}, correction_ms{0};
  double gpu_preprocess_ms{0}, gpu_inference_ms{0}, gpu_label_export_ms{0}, gpu_postprocess_ms{0};
  bool success{false}, render_result{false}, sample_limit_reached{false};
};

// Disk formatting/writes run on a separate thread. A bounded queue drops log
// rows rather than waiting for disk; loss/error counters are visible to the node.
class ProcessingProfileWriter
{
public:
  ProcessingProfileWriter(const std::string & directory,
    const std::vector<std::pair<std::string, std::string>> & metadata);
  ~ProcessingProfileWriter();
  ProcessingProfileWriter(const ProcessingProfileWriter &) = delete;
  ProcessingProfileWriter & operator=(const ProcessingProfileWriter &) = delete;
  void submit(const ProfileFrame & frame);
  void close();
  std::uint64_t dropped() const;
  bool failed() const;
  const std::string & path() const;
private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace line_detactor
#endif
