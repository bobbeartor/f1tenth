#pragma once

#include "point_cloud/depth_projection.hpp"
#include "point_cloud/ground_filter.hpp"
#include "point_cloud/bev_crop.hpp"
#include <map>

#include <depthai/depthai.hpp>

#include <memory>
#include <string>
#include <utility>

namespace point_cloud
{
struct Config
{
  std::string resolution{"400p"};
  double fps{50.0};
  int rgb_width{640},rgb_height{400};
  int imu_rate_hz{200},imu_queue_size{20},imu_max_batch_reports{5};
  std::string mode{"high_density"};
  double dot_intensity{0.5};
  double flood_intensity{0.0};
  int confidence{200};
  bool lr_check{true};
  int lr_threshold{10};
  bool subpixel{false};
  int subpixel_bits{3};
  bool extended{false};
  std::string median{"off"};
  bool spatial{false};
  bool speckle{false};
  bool hole_filling{false};
  bool adaptive_median{false};
  ProjectionOptions projection;
  GroundOptions ground;
  BevOptions bev;
  bool publish_depth{true};
  double max_age_sec{0.5};
  double metrics_interval{1.0};
};

std::pair<std::uint32_t, std::uint32_t> resolutionSize(const std::string & resolution);
void validate(const Config & config);

// Build stereo on an existing device pipeline (does not open/start a device).
struct StereoCameras
{
  std::shared_ptr<dai::node::Camera> left, right;
};
// With gate_control, a device Script forwards stereo inputs only while the last
// control byte was nonzero (initially open), so StereoDepth can idle in a tunnel.
std::shared_ptr<dai::node::StereoDepth> addStereoDepth(
  dai::Pipeline & pipeline, const Config & config, StereoCameras * cameras = nullptr,
  std::shared_ptr<dai::InputQueue> * gate_control = nullptr);
RigidTransform rgbFromFrame(dai::ImgFrame & frame, const dai::CalibrationHandler & calibration);

// Owns one device session. Closing/reopening also reapplies both IR intensities.
class DepthSource
{
public:
  DepthSource(const Config & config, const std::string & device_id, bool right_preview = false, double rgb_fps = 0.0);
  DepthSource(const Config & config, const std::string & device_id, bool right_preview, double rgb_fps, bool imu_enabled);
  ~DepthSource();
  std::shared_ptr<dai::ImgFrame> tryGet();
  std::shared_ptr<dai::ImgFrame> tryGetRight();
  std::shared_ptr<dai::ImgFrame> tryGetRgb();
  std::shared_ptr<dai::IMUData> tryGetImu();
  Vector3 cameraGyroscope(const Vector3 & report) const {return rotate(rgb_from_imu_,report);}
  std::string deviceId() const;
  RigidTransform rgbFromFrame(dai::ImgFrame & frame);

private:
  std::map<dai::CameraBoardSocket, RigidTransform> rgb_from_reference_;
  std::shared_ptr<dai::Device> device_;
  std::unique_ptr<dai::Pipeline> pipeline_;
  std::shared_ptr<dai::MessageQueue> queue_, right_queue_, rgb_queue_, imu_queue_;
  Matrix3 rgb_from_imu_{1,0,0,0,1,0,0,0,1};
};
}  // namespace point_cloud
