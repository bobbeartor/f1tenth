#ifndef BEV_HANDOFF__DIRECT_CAMERA_HANDOFF_HPP_
#define BEV_HANDOFF__DIRECT_CAMERA_HANDOFF_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <std_msgs/msg/header.hpp>

namespace bev_handoff
{
struct CameraExposureObservation
{
  long exposure_us{0};
  int iso{0};
  std::chrono::steady_clock::time_point captured_at;
};
// Actual sensor metadata, published before IMU/BEV validity filtering.
void publishCameraExposure(bool infrared, std::shared_ptr<const CameraExposureObservation> observation);
std::shared_ptr<const CameraExposureObservation> latestCameraExposure(bool infrared);
enum class RgbExposureControl {automatic, restoring, auto_pending};
// Requested control mode; sensor metadata above independently shows actual values.
void publishRgbExposureControl(RgbExposureControl mode);
RgbExposureControl latestRgbExposureControl();
// Rectified color camera frame BEFORE ground-plane projection/stabilization.
// owner keeps the immutable host NV12 buffer alive. No image copy in delivery.
struct DirectCameraFrame
{
  const std::uint8_t * nv12{nullptr};
  std::size_t size{0}, stride{0};
  int width{0}, height{0};
  std::shared_ptr<const void> owner;
  std_msgs::msg::Header header;
  std::chrono::steady_clock::time_point captured_at;
  std::uint64_t generation{0};
};
using DirectCameraCallback = std::function<void(std::shared_ptr<const DirectCameraFrame>)>;
bool hasDirectCameraConsumer();
std::uint64_t registerDirectCameraConsumer(DirectCameraCallback callback);
void unregisterDirectCameraConsumer(std::uint64_t id);
// Single consumer; drops on lock contention. Callback must only try-enqueue.
bool deliverDirectCameraFrame(std::shared_ptr<const DirectCameraFrame> frame);
// Small process-local observation for the existing BEV GUI, independent of
// camera delivery locks. 0=UNKNOWN, 1=RED, 2=GREEN. Expired states read UNKNOWN.
void publishTrafficSignalState(
  std::uint8_t state, std::chrono::steady_clock::time_point valid_until);
std::uint8_t latestTrafficSignalState();
// Completed model calls / wall time over the latest reporting window.
// Expired/unavailable observations read NaN, independently of detected color.
void publishTrafficInferenceFps(
  double average_fps, std::chrono::steady_clock::time_point valid_until);
double latestTrafficInferenceFps();
}  // namespace bev_handoff
#endif
