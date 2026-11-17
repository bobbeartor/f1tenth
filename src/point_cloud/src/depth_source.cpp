#include "point_cloud/depth_source.hpp"

#include <cmath>
#include <stdexcept>

namespace point_cloud
{
std::pair<std::uint32_t, std::uint32_t> resolutionSize(const std::string & value)
{
  if (value == "400p") {return {640, 400};}
  if (value == "480p") {return {640, 480};}
  if (value == "720p") {return {1280, 720};}
  if (value == "800p") {return {1280, 800};}
  throw std::invalid_argument("camera.resolution: choose 400p, 480p, 720p or 800p");
}

namespace
{
dai::node::StereoDepth::PresetMode preset(const std::string & value)
{
  using Mode = dai::node::StereoDepth::PresetMode;
  if (value == "default") {return Mode::DEFAULT;}
  if (value == "high_density") {return Mode::FAST_DENSITY;}
  if (value == "high_accuracy") {return Mode::FAST_ACCURACY;}
  throw std::invalid_argument("depth.mode: choose default, high_density or high_accuracy");
}

dai::StereoDepthConfig::MedianFilter median(const std::string & value)
{
  using Filter = dai::StereoDepthConfig::MedianFilter;
  if (value == "off") {return Filter::MEDIAN_OFF;}
  if (value == "3x3") {return Filter::KERNEL_3x3;}
  if (value == "5x5") {return Filter::KERNEL_5x5;}
  if (value == "7x7") {return Filter::KERNEL_7x7;}
  throw std::invalid_argument("depth.median_filter: choose off, 3x3, 5x5 or 7x7");
}
}  // namespace

void validate(const Config & c)
{
  if(c.rgb_width<16 || c.rgb_width>4096 || c.rgb_height<16 || c.rgb_height>4096 ||
    c.imu_rate_hz<1 || c.imu_rate_hz>1000 || c.imu_queue_size<1 || c.imu_queue_size>1000 ||
    c.imu_max_batch_reports<1 || c.imu_max_batch_reports>1000)
    throw std::invalid_argument("Invalid shared RGB size or IMU rate/queue/batch");
  validateGround(c.ground);
  validateBev(c.bev);
  resolutionSize(c.resolution);
  preset(c.mode);
  median(c.median);
  if (!std::isfinite(c.fps) || c.fps < 1 || c.fps > 120) {
    throw std::invalid_argument("camera.fps must be 1..120; achievable FPS depends on the device/mode");
  }
  for (const double intensity : {c.dot_intensity, c.flood_intensity}) {
    if (!std::isfinite(intensity) || intensity < 0 || intensity > 1) {
      throw std::invalid_argument("IR dot/flood intensities must be finite values in 0..1");
    }
  }
  if (c.confidence < 0 || c.confidence > 255 || c.lr_threshold < 0 || c.lr_threshold > 128) {
    throw std::invalid_argument("confidence_threshold must be 0..255; left_right_threshold must be 0..128");
  }
  if (c.subpixel_bits < 3 || c.subpixel_bits > 5) {
    throw std::invalid_argument("depth.subpixel_fractional_bits must be 3, 4 or 5");
  }
  if (c.subpixel && c.extended) {
    throw std::invalid_argument("subpixel and extended_disparity cannot be enabled together");
  }
  if (c.subpixel && c.subpixel_bits > 3 && c.median != "off") {
    throw std::invalid_argument("4/5-bit subpixel requires depth.median_filter: off");
  }
  if (c.projection.pixel_stride < 1 || c.projection.pixel_stride > 16 ||
    !std::isfinite(c.projection.min_depth_m) || !std::isfinite(c.projection.max_depth_m) ||
    c.projection.min_depth_m < 0 || c.projection.max_depth_m < 0 ||
    (c.projection.max_depth_m > 0 && c.projection.max_depth_m <= c.projection.min_depth_m))
  {
    throw std::invalid_argument("points: stride must be 1..16; depth limits must be >=0, with max=0 or max>min");
  }
  if (!std::isfinite(c.max_age_sec) || c.max_age_sec < 0.02 || c.max_age_sec > 5 ||
    !std::isfinite(c.metrics_interval) || c.metrics_interval < 0.1 || c.metrics_interval > 60)
  {
    throw std::invalid_argument("input.max_age_sec must be 0.02..5; metrics.print_interval_sec must be 0.1..60");
  }
}

std::shared_ptr<dai::node::StereoDepth> addStereoDepth(
  dai::Pipeline & pipeline, const Config & c, StereoCameras * cameras,
  std::shared_ptr<dai::InputQueue> * gate_control)
{
  validate(c);
  const auto size = resolutionSize(c.resolution);
  auto left = pipeline.create<dai::node::Camera>();
  auto right = pipeline.create<dai::node::Camera>();
  // An additional undistorted 800p LEFT output is used by tunnel lane inference.
  const auto sensor_size = cameras ? std::make_pair(1280U, 800U) : size;
  left->build(dai::CameraBoardSocket::CAM_B, sensor_size, static_cast<float>(c.fps));
  right->build(dai::CameraBoardSocket::CAM_C, sensor_size, static_cast<float>(c.fps));
  if (cameras) {*cameras = {left, right};}
  auto * left_output = left->requestOutput(size);
  auto * right_output = right->requestOutput(size);
  if (gate_control) {
    // Both sensors run at the same rate; StereoDepth still pairs by sequence.
    auto gate = pipeline.create<dai::node::Script>();
    gate->setScript(R"PY(
active = True
while True:
    control = node.io['control'].tryGet()
    if control is not None:
        active = bool(control.getData()[0])
    left = node.io['left'].get()
    right = node.io['right'].get()
    if active:
        node.io['left_out'].send(left)
        node.io['right_out'].send(right)
)PY");
    left_output->link(gate->inputs["left"]);
    right_output->link(gate->inputs["right"]);
    for (const auto * name : {"left", "right", "control"}) {
      gate->inputs[name].setMaxSize(1);
      gate->inputs[name].setBlocking(false);
    }
    *gate_control = gate->inputs["control"].createInputQueue();
    left_output = &gate->outputs["left_out"];
    right_output = &gate->outputs["right_out"];
  }
  auto stereo = pipeline.create<dai::node::StereoDepth>();
  stereo->build(*left_output, *right_output, preset(c.mode));
  stereo->initialConfig->setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::MILLIMETER);
  stereo->setDepthAlign(dai::StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_RIGHT);
  stereo->initialConfig->setConfidenceThreshold(c.confidence);
  stereo->initialConfig->setLeftRightCheckThreshold(c.lr_threshold);
  stereo->setLeftRightCheck(c.lr_check);
  stereo->setSubpixel(c.subpixel);
  stereo->setSubpixelFractionalBits(c.subpixel_bits);
  stereo->setExtendedDisparity(c.extended);
  stereo->initialConfig->setMedianFilter(median(c.median));
  auto & post = stereo->initialConfig->postProcessing;
  post.decimationFilter.decimationFactor = 1;
  post.temporalFilter.enable = false;  // Each cloud is always one depth frame.
  post.spatialFilter.enable = c.spatial;
  post.speckleFilter.enable = c.speckle;
  post.holeFilling.enable = c.hole_filling;
  post.adaptiveMedianFilter.enable = c.adaptive_median;
  post.bilateralSigmaValue = 0;
  post.thresholdFilter.minRange = 0;
  post.thresholdFilter.maxRange = 65535;
  return stereo;
}

RigidTransform rgbFromFrame(dai::ImgFrame & frame, const dai::CalibrationHandler & calibration)
{
  const auto & transformation=frame.getTransformation();
  if (!transformation.isValid()) {throw std::runtime_error("Missing frame calibration");}
  const auto extrinsics=transformation.getExtrinsics();
  const auto reference=extrinsics.toCameraSocket;
  if (reference==dai::CameraBoardSocket::AUTO) {throw std::runtime_error("Missing reference camera socket");}
  const auto reference_from_frame=calibratedTransform(
    extrinsics.getTransformationMatrix(false,dai::LengthUnit::METER),1.0);
  const auto rgb_from_reference=reference==dai::CameraBoardSocket::CAM_A ? RigidTransform{} :
    calibratedTransform(calibration.getCameraExtrinsics(reference,dai::CameraBoardSocket::CAM_A,false),0.01);
  return compose(rgb_from_reference,reference_from_frame);
}

DepthSource::DepthSource(const Config & c, const std::string & device_id, bool right_preview, double rgb_fps)
: DepthSource(c,device_id,right_preview,rgb_fps,false) {}

DepthSource::DepthSource(const Config & c, const std::string & device_id, bool right_preview, double rgb_fps, bool imu_enabled)
{
  validate(c);
  if (!std::isfinite(rgb_fps) || rgb_fps < 0 || rgb_fps > 120) {
    throw std::invalid_argument("RGB FPS must be 0..120");
  }
  device_ = device_id.empty() ? std::make_shared<dai::Device>(dai::UsbSpeed::SUPER) :
    std::make_shared<dai::Device>(dai::DeviceInfo(device_id), dai::UsbSpeed::SUPER);
  pipeline_ = std::make_unique<dai::Pipeline>(device_);
  pipeline_->setAutoCalibrationMode(dai::Pipeline::AutoCalibrationMode::OFF);
  pipeline_->setXLinkChunkSize(0);
  auto stereo = addStereoDepth(*pipeline_, c);
  queue_ = stereo->depth.createOutputQueue(1, false);
  if (right_preview) {right_queue_ = stereo->rectifiedRight.createOutputQueue(1, false);}
  dai::Node::Output * rgb_output = nullptr;
  if (rgb_fps > 0) {
    auto rgb = pipeline_->create<dai::node::Camera>();
    rgb->build(dai::CameraBoardSocket::CAM_A);
    rgb_output = rgb->requestOutput({c.rgb_width,c.rgb_height}, dai::ImgFrame::Type::BGR888i,
      dai::ImgResizeMode::CROP, static_cast<float>(rgb_fps), true);
    rgb_queue_ = rgb_output->createOutputQueue(1, false);
  }
  if (imu_enabled) {
    const auto calibration=device_->getCalibration();
    const auto a=calibration.getImuToCameraExtrinsics(dai::CameraBoardSocket::CAM_A,false);
    const auto b=calibration.getEepromData().imuExtrinsics.rotationMatrix;
    if(a.size()<3 || b.size()!=3)throw std::runtime_error("Missing IMU calibration");
    for(int i=0;i<3;++i)for(int j=0;j<3;++j){
      if(a[i].size()<3 || b[j].size()!=3)throw std::runtime_error("Malformed IMU calibration");
      rgb_from_imu_[i*3+j]=0;
      for(int k=0;k<3;++k)rgb_from_imu_[i*3+j]+=a[i][k]*b[j][k];
      if(!std::isfinite(rgb_from_imu_[i*3+j]))throw std::runtime_error("Invalid IMU rotation");
    }
    for(int i=0;i<3;++i)for(int j=0;j<3;++j){
      double dot=0;for(int k=0;k<3;++k)dot+=rgb_from_imu_[i*3+k]*rgb_from_imu_[j*3+k];
      if(std::abs(dot-(i==j ? 1.0 : 0.0))>.01)throw std::runtime_error("Non-orthonormal IMU rotation");
    }
    auto imu=pipeline_->create<dai::node::IMU>();
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_CALIBRATED,c.imu_rate_hz);
    imu->setBatchReportThreshold(1);imu->setMaxBatchReports(c.imu_max_batch_reports);
    imu_queue_=imu->out.createOutputQueue(c.imu_queue_size,false);
  }
  pipeline_->build();
  const auto bridge = stereo->depth.getXLinkBridge();
  if (!bridge || !bridge->xLinkOut) {throw std::runtime_error("Missing depth XLink bridge");}
  bridge->xLinkOut->input.setMaxSize(1);
  bridge->xLinkOut->input.setBlocking(false);
  if (right_preview) {
    const auto right_bridge = stereo->rectifiedRight.getXLinkBridge();
    if (!right_bridge || !right_bridge->xLinkOut) {throw std::runtime_error("Missing right preview bridge");}
    right_bridge->xLinkOut->input.setMaxSize(1);
    right_bridge->xLinkOut->input.setBlocking(false);
  }
  if (rgb_output) {
    const auto rgb_bridge = rgb_output->getXLinkBridge();
    if (!rgb_bridge || !rgb_bridge->xLinkOut) {throw std::runtime_error("Missing RGB preview bridge");}
    rgb_bridge->xLinkOut->input.setMaxSize(1);
    rgb_bridge->xLinkOut->input.setBlocking(false);
  }
  pipeline_->start();
  if (!device_->setIrLaserDotProjectorIntensity(static_cast<float>(c.dot_intensity)) && c.dot_intensity > 0) {
    throw std::runtime_error("Dot projector unavailable: use a supported OAK Pro or set depth.ir_dot_projector_intensity=0.0");
  }
  if (!device_->setIrFloodLightIntensity(static_cast<float>(c.flood_intensity)) && c.flood_intensity > 0) {
    throw std::runtime_error("IR flood light unavailable: set depth.ir_flood_light_intensity=0.0");
  }
}

DepthSource::~DepthSource()
{
  try {if (pipeline_) {pipeline_->stop();}} catch (...) {}  // Disconnect must not terminate ROS shutdown.
}

std::shared_ptr<dai::ImgFrame> DepthSource::tryGet()
{
  if (!pipeline_->isRunning()) {throw std::runtime_error("Depth pipeline stopped");}
  return queue_->tryGet<dai::ImgFrame>();
}

std::shared_ptr<dai::ImgFrame> DepthSource::tryGetRight()
{
  return right_queue_ ? right_queue_->tryGet<dai::ImgFrame>() : nullptr;
}

std::shared_ptr<dai::ImgFrame> DepthSource::tryGetRgb()
{
  return rgb_queue_ ? rgb_queue_->tryGet<dai::ImgFrame>() : nullptr;
}

RigidTransform DepthSource::rgbFromFrame(dai::ImgFrame & frame)
{
  const auto & transformation = frame.getTransformation();
  if (!transformation.isValid()) {throw std::runtime_error("Missing depth frame calibration");}
  // Match ir_camera_driver: metadata already accounts for stereo rectification.
  const auto extrinsics = transformation.getExtrinsics();
  const auto reference = extrinsics.toCameraSocket;
  if (reference == dai::CameraBoardSocket::AUTO) {
    throw std::runtime_error("Depth frame has no calibration reference socket");
  }
  const auto reference_from_frame = calibratedTransform(
    extrinsics.getTransformationMatrix(false, dai::LengthUnit::METER), 1.0);
  auto found = rgb_from_reference_.find(reference);
  if (found == rgb_from_reference_.end()) {
    const auto t = reference == dai::CameraBoardSocket::CAM_A ? RigidTransform{} :
      calibratedTransform(device_->getCalibration().getCameraExtrinsics(
        reference, dai::CameraBoardSocket::CAM_A, false), 0.01);
    found = rgb_from_reference_.emplace(reference, t).first;
  }
  return compose(found->second, reference_from_frame);
}

std::shared_ptr<dai::IMUData> DepthSource::tryGetImu()
{return imu_queue_ ? imu_queue_->tryGet<dai::IMUData>() : nullptr;}

std::string DepthSource::deviceId() const {return device_->getDeviceId();}
}  // namespace point_cloud
