#ifndef BEV_HANDOFF__DIRECT_BEV_HANDOFF_HPP_
#define BEV_HANDOFF__DIRECT_BEV_HANDOFF_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>

#include <opencv2/core.hpp>
#include <std_msgs/msg/header.hpp>

namespace bev_handoff
{

// device_owner keeps the CUDA BEV allocation alive while a consumer uses it.
// The optional cv::Mat or cached download independently supplies the host image.
struct DirectBevFrame
{
  // Optional CPU image for local preview/capture. The autonomous inference
  // path uses device_bgr and does not require this download.
  cv::Mat bgr;
  // Optional cached download. Call only on a GUI/capture thread, never inference.
  std::function<cv::Mat()> download_bgr;
  const std::uint8_t * device_bgr{nullptr};
  std::size_t device_stride{0U};
  int width{0};
  int height{0};
  std::shared_ptr<const void> device_owner;
  std_msgs::msg::Header header;
  std::chrono::steady_clock::time_point bev_input_received_at;
  std::uint64_t source_generation{0U};
};

using DirectBevFrameCallback =
  std::function<void(std::shared_ptr<const DirectBevFrame>)>;

// Exactly one process-local consumer is supported. The callback must only
// enqueue/replace the latest frame; expensive inference must run elsewhere.
std::uint64_t registerDirectBevConsumer(DirectBevFrameCallback callback);
void unregisterDirectBevConsumer(std::uint64_t consumer_id);
bool deliverDirectBevFrame(std::shared_ptr<const DirectBevFrame> frame);

// The IR worker produces an owning rendered image; the RGB detector's one
// GUI thread displays it after selection. No second HighGUI window/thread.
struct IrLanePreview
{
  std_msgs::msg::Header header;
  cv::Mat image;
  double inference_ms{0.0}, postprocess_ms{0.0}, stop_distance_m{0.0};
  std::uint64_t generation{0U};
};
void publishIrLanePreview(std::shared_ptr<const IrLanePreview> frame);
std::shared_ptr<const IrLanePreview> latestIrLanePreview();

}  // namespace bev_handoff

#endif  // BEV_HANDOFF__DIRECT_BEV_HANDOFF_HPP_
