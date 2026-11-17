#include "bev_handoff/direct_bev_handoff.hpp"

#include <mutex>
#include <atomic>
#include <stdexcept>
#include <utility>

namespace bev_handoff
{
namespace
{
std::shared_ptr<const IrLanePreview> ir_preview;

struct HandoffState
{
  std::mutex mutex;
  DirectBevFrameCallback callback;
  std::uint64_t consumer_id{0U};
  std::uint64_t next_consumer_id{1U};
};

HandoffState & state()
{
  static HandoffState handoff;
  return handoff;
}

}  // namespace

void publishIrLanePreview(std::shared_ptr<const IrLanePreview> frame)
{
  std::atomic_store(&ir_preview,std::move(frame));
}

std::shared_ptr<const IrLanePreview> latestIrLanePreview()
{
  return std::atomic_load(&ir_preview);
}

std::uint64_t registerDirectBevConsumer(DirectBevFrameCallback callback)
{
  if (!callback) {
    throw std::invalid_argument("direct BEV consumer callback is empty");
  }

  auto & handoff = state();
  std::lock_guard<std::mutex> lock(handoff.mutex);
  if (handoff.callback) {
    throw std::runtime_error(
            "a direct BEV consumer is already registered in this process");
  }
  const std::uint64_t id = handoff.next_consumer_id++;
  handoff.callback = std::move(callback);
  handoff.consumer_id = id;
  return id;
}

void unregisterDirectBevConsumer(const std::uint64_t consumer_id)
{
  if (consumer_id == 0U) {
    return;
  }
  auto & handoff = state();
  std::lock_guard<std::mutex> lock(handoff.mutex);
  if (handoff.consumer_id == consumer_id) {
    handoff.callback = {};
    handoff.consumer_id = 0U;
  }
}

bool deliverDirectBevFrame(std::shared_ptr<const DirectBevFrame> frame)
{
  if (!frame) {
    throw std::invalid_argument("direct BEV frame is null");
  }
  auto & handoff = state();
  // Keep the lock through the short enqueue callback so unregister returning
  // guarantees that no callback can still reference a destroyed consumer.
  std::lock_guard<std::mutex> lock(handoff.mutex);
  if (!handoff.callback) {
    return false;
  }
  handoff.callback(std::move(frame));
  return true;
}

}  // namespace bev_handoff
