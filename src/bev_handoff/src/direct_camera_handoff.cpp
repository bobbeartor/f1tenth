#include "bev_handoff/direct_camera_handoff.hpp"
#include <atomic>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace bev_handoff
{
namespace
{
std::shared_ptr<const CameraExposureObservation> rgb_exposure, ir_exposure;
std::atomic<RgbExposureControl> rgb_exposure_control{RgbExposureControl::automatic};
struct State
{
  std::mutex mutex;
  std::atomic<bool> active{false};
  DirectCameraCallback callback;
  std::uint64_t id{0}, next{1};
};
State & state() {static State value; return value;}
struct Signal
{
  std::mutex mutex;
  std::uint8_t value{0};
  std::chrono::steady_clock::time_point valid_until;
};
Signal & signal() {static Signal value; return value;}
struct InferenceFps
{
  std::mutex mutex;
  double value{std::numeric_limits<double>::quiet_NaN()};
  std::chrono::steady_clock::time_point valid_until;
};
InferenceFps & inference_fps() {static InferenceFps value; return value;}
}
void publishCameraExposure(bool infrared, std::shared_ptr<const CameraExposureObservation> value)
{std::atomic_store(infrared ? &ir_exposure : &rgb_exposure, std::move(value));}
std::shared_ptr<const CameraExposureObservation> latestCameraExposure(bool infrared)
{return std::atomic_load(infrared ? &ir_exposure : &rgb_exposure);}
void publishRgbExposureControl(RgbExposureControl mode) {rgb_exposure_control.store(mode);}
RgbExposureControl latestRgbExposureControl() {return rgb_exposure_control.load();}
void publishTrafficInferenceFps(
  double average_fps, std::chrono::steady_clock::time_point valid_until)
{
  auto & s = inference_fps();
  std::lock_guard<std::mutex> lock(s.mutex);
  s.value = std::isfinite(average_fps) && average_fps >= 0.0 ?
    average_fps : std::numeric_limits<double>::quiet_NaN();
  s.valid_until = valid_until;
}
double latestTrafficInferenceFps()
{
  auto & s = inference_fps();
  std::lock_guard<std::mutex> lock(s.mutex);
  return std::chrono::steady_clock::now() < s.valid_until ?
    s.value : std::numeric_limits<double>::quiet_NaN();
}
void publishTrafficSignalState(
  std::uint8_t value, std::chrono::steady_clock::time_point valid_until)
{
  auto & s = signal();
  std::lock_guard<std::mutex> lock(s.mutex);
  s.value = value <= 2 ? value : 0;
  s.valid_until = valid_until;
}
std::uint8_t latestTrafficSignalState()
{
  auto & s = signal();
  std::lock_guard<std::mutex> lock(s.mutex);
  return std::chrono::steady_clock::now() < s.valid_until ? s.value : 0;
}
bool hasDirectCameraConsumer() {return state().active.load(std::memory_order_relaxed);}
std::uint64_t registerDirectCameraConsumer(DirectCameraCallback callback)
{
  auto & s = state();
  std::lock_guard<std::mutex> lock(s.mutex);
  if (!callback || s.callback) {throw std::runtime_error("invalid/duplicate camera consumer");}
  s.callback = std::move(callback);
  s.id = s.next++;
  s.active.store(true, std::memory_order_relaxed);
  return s.id;
}
void unregisterDirectCameraConsumer(std::uint64_t id)
{
  auto & s = state();
  std::lock_guard<std::mutex> lock(s.mutex);
  if (id && s.id == id) {
    s.active.store(false, std::memory_order_relaxed);
    s.callback = {};
    s.id = 0;
  }
}
bool deliverDirectCameraFrame(std::shared_ptr<const DirectCameraFrame> frame)
{
  auto & s = state();
  std::unique_lock<std::mutex> lock(s.mutex, std::try_to_lock);
  if (!lock.owns_lock() || !s.callback || !frame) {return false;}
  // Unregister waits only for this short callback, protecting its lifetime.
  s.callback(std::move(frame));
  return true;
}
}  // namespace bev_handoff
