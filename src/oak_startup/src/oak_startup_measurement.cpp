#include "oak_startup/oak_startup_measurement.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "depthai/depthai.hpp"
#include "oak_startup/ground_plane_estimator.hpp"
#include "oak_startup/startup_attitude.hpp"

namespace oak_startup
{

namespace
{

using namespace std::chrono_literals;

constexpr std::uint32_t kOv9282FullWidth = 1280U;
constexpr std::uint32_t kOv9282FullHeight = 800U;
constexpr double kRadiansToDegrees =
  180.0 / 3.141592653589793238462643383279502884;
constexpr char kStartupRoiPreviewWindowName[] =
  "OAK startup ground-plane ROI";

struct PlaneCandidate
{
  GroundPlaneEstimate plane;
  double median_depth_m{0.0};
};

cv::Matx33d matrix3x3FromCalibration(
  const std::vector<std::vector<float>> & rows,
  const char * label)
{
  if (
    rows.size() < 3U || rows[0].size() < 3U ||
    rows[1].size() < 3U || rows[2].size() < 3U)
  {
    throw std::runtime_error(
            std::string(label) + " is missing or smaller than 3x3");
  }
  cv::Matx33d result;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      result(row, column) = static_cast<double>(rows[row][column]);
    }
  }
  return result;
}

void validateRotationMatrix(
  const cv::Matx33d & rotation,
  const char * label)
{
  const double orthogonality_error = cv::norm(
    rotation * rotation.t() - cv::Matx33d::eye());
  const double determinant = cv::determinant(cv::Mat(rotation));
  if (
    !cv::checkRange(cv::Mat(rotation)) ||
    !std::isfinite(orthogonality_error) || orthogonality_error > 1.0e-3 ||
    !std::isfinite(determinant) || std::abs(determinant - 1.0) > 1.0e-3)
  {
    throw std::runtime_error(std::string(label) + " is not a proper rotation");
  }
}

double timestampSeconds(
  const std::chrono::steady_clock::time_point & timestamp)
{
  return std::chrono::duration<double>(
    timestamp.time_since_epoch()).count();
}

std::array<double, 3> normalized(
  const std::array<double, 3> & vector)
{
  const double norm = std::sqrt(
    vector[0] * vector[0] +
    vector[1] * vector[1] +
    vector[2] * vector[2]);
  if (!std::isfinite(norm) || norm <= 1.0e-9) {
    throw std::invalid_argument("cannot normalize a zero/non-finite vector");
  }
  return {vector[0] / norm, vector[1] / norm, vector[2] / norm};
}

cv::Vec3d normalized(const cv::Vec3d & vector)
{
  const double norm = cv::norm(vector);
  if (!std::isfinite(norm) || norm <= 1.0e-9) {
    throw std::invalid_argument("cannot normalize a zero/non-finite vector");
  }
  return vector / norm;
}

double median(std::vector<double> values)
{
  if (values.empty()) {
    throw std::invalid_argument("median requires at least one value");
  }
  const auto middle =
    values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2U);
  std::nth_element(values.begin(), middle, values.end());
  if ((values.size() % 2U) != 0U) {
    return *middle;
  }
  const double upper = *middle;
  const auto lower = std::max_element(values.begin(), middle);
  return 0.5 * (*lower + upper);
}

cv::Rect measurementRoi(const OakStartupMeasurementConfig & config)
{
  const int start_u = (config.stereo_width - config.roi_width) / 2;
  const int start_v =
    (config.stereo_height - config.roi_height) / 2 +
    config.roi_vertical_offset_px;
  return cv::Rect(start_u, start_v, config.roi_width, config.roi_height);
}

class StartupRoiPreview final
{
public:
  explicit StartupRoiPreview(const bool enabled)
  : enabled_(enabled)
  {
  }

  ~StartupRoiPreview()
  {
    close();
  }

  void show(
    dai::ImgFrame & packet,
    const OakStartupMeasurementConfig & config)
  {
    if (!enabled_ && !config.preview_callback) {
      return;
    }

    try {
      if (
        packet.getType() != dai::ImgFrame::Type::RAW16 ||
        static_cast<int>(packet.getWidth()) != config.stereo_width ||
        static_cast<int>(packet.getHeight()) != config.stereo_height)
      {
        return;
      }

      const auto & bytes = packet.getData();
      const std::size_t minimum_stride =
        static_cast<std::size_t>(config.stereo_width) *
        sizeof(std::uint16_t);
      const std::size_t stride = std::max(
        minimum_stride, static_cast<std::size_t>(packet.getStride()));
      if (
        bytes.size() <
        stride * static_cast<std::size_t>(config.stereo_height))
      {
        return;
      }

      cv::Mat depth_view(
        config.stereo_height,
        config.stereo_width,
        CV_16UC1,
        const_cast<std::uint8_t *>(bytes.data()),
        stride);
      const double minimum_depth_mm = config.minimum_depth_m * 1000.0;
      const double depth_span_mm =
        (config.maximum_depth_m - config.minimum_depth_m) * 1000.0;
      const double scale = 255.0 / depth_span_mm;
      depth_view.convertTo(
        preview_gray_, CV_8UC1, scale, -minimum_depth_mm * scale);
      cv::cvtColor(preview_gray_, preview_bgr_, cv::COLOR_GRAY2BGR);
      cv::rectangle(
        preview_bgr_, measurementRoi(config), cv::Scalar(0, 0, 255),
        1, cv::LINE_8);

      if (config.preview_callback) {config.preview_callback(preview_bgr_);}
      if (!enabled_) {return;}

      if (!window_created_) {
        cv::namedWindow(kStartupRoiPreviewWindowName, cv::WINDOW_NORMAL);
        cv::resizeWindow(
          kStartupRoiPreviewWindowName,
          config.stereo_width,
          config.stereo_height);
        window_created_ = true;
      }
      cv::imshow(kStartupRoiPreviewWindowName, preview_bgr_);
      cv::waitKey(1);
    } catch (const cv::Exception & exception) {
      std::cerr
        << "Startup measurement ROI preview disabled after OpenCV error: "
        << exception.what() << '\n';
      enabled_ = false;
      close();
    }
  }

private:
  void close() noexcept
  {
    if (!window_created_) {
      return;
    }
    try {
      cv::destroyWindow(kStartupRoiPreviewWindowName);
      cv::waitKey(1);
    } catch (const cv::Exception &) {
    }
    window_created_ = false;
  }

  bool enabled_{false};
  bool window_created_{false};
  cv::Mat preview_gray_;
  cv::Mat preview_bgr_;
};

void validateConfig(const OakStartupMeasurementConfig & config)
{
  const std::int64_t centered_roi_top =
    (static_cast<std::int64_t>(config.stereo_height) -
    static_cast<std::int64_t>(config.roi_height)) / 2;
  const std::int64_t shifted_roi_top =
    centered_roi_top +
    static_cast<std::int64_t>(config.roi_vertical_offset_px);
  if (
    !std::isfinite(config.stereo_fps) || config.stereo_fps <= 0.0 ||
    config.stereo_width <= 0 || config.stereo_height <= 0 ||
    config.depth_queue_size <= 0 ||
    config.stereo_subpixel_fractional_bits < 3 ||
    config.stereo_subpixel_fractional_bits > 5 ||
    config.stereo_left_right_check_threshold < 0 ||
    config.stereo_left_right_check_threshold > 128 ||
    config.stereo_confidence_threshold < 0 ||
    config.stereo_confidence_threshold > 255 ||
    // RVC2 cannot RGB-align depth when disparity shift is enabled. The
    // startup points must remain in CAM_A coordinates to share the exact IMU
    // frame used by the image stabilizer and BEV attitude.
    config.stereo_disparity_shift != 0 ||
    !std::isfinite(config.imu_rate_hz) || config.imu_rate_hz <= 0.0 ||
    config.imu_queue_size <= 0 ||
    config.imu_max_batch_reports <= 1 ||
    config.imu_max_batch_reports > config.imu_queue_size ||
    !std::isfinite(config.maximum_imu_pair_skew_sec) ||
    config.maximum_imu_pair_skew_sec <= 0.0 ||
    !std::isfinite(config.warmup_sec) || config.warmup_sec < 0.0 ||
    !std::isfinite(config.ir_dot_projector_intensity) ||
    config.ir_dot_projector_intensity < 0.0 ||
    config.ir_dot_projector_intensity > 1.0 ||
    !std::isfinite(config.manual_camera_height_m) ||
    config.manual_camera_height_m <= 0.0 ||
    config.roi_width <= 0 || config.roi_height <= 0 ||
    config.roi_width > config.stereo_width ||
    config.roi_height > config.stereo_height ||
    shifted_roi_top < 0 ||
    shifted_roi_top + static_cast<std::int64_t>(config.roi_height) >
    static_cast<std::int64_t>(config.stereo_height) ||
    config.point_sample_step <= 0 ||
    config.minimum_valid_points <= 0 ||
    !std::isfinite(config.minimum_depth_m) ||
    config.minimum_depth_m <= 0.0 ||
    !std::isfinite(config.maximum_depth_m) ||
    config.maximum_depth_m <= config.minimum_depth_m ||
    !std::isfinite(config.minimum_height_m) ||
    config.minimum_height_m <= 0.0 ||
    !std::isfinite(config.maximum_height_m) ||
    config.maximum_height_m <= config.minimum_height_m ||
    (config.manual_camera_height_enabled &&
    (config.manual_camera_height_m < config.minimum_height_m ||
    config.manual_camera_height_m > config.maximum_height_m)) ||
    config.plane_ransac_iterations <= 0 ||
    !std::isfinite(config.plane_inlier_threshold_m) ||
    config.plane_inlier_threshold_m <= 0.0 ||
    config.plane_minimum_inliers < 3 ||
    !std::isfinite(config.plane_minimum_inlier_ratio) ||
    config.plane_minimum_inlier_ratio <= 0.0 ||
    config.plane_minimum_inlier_ratio > 1.0 ||
    !std::isfinite(config.plane_maximum_residual_mad_m) ||
    config.plane_maximum_residual_mad_m <= 0.0 ||
    !std::isfinite(config.plane_maximum_imu_difference_deg) ||
    config.plane_maximum_imu_difference_deg <= 0.0 ||
    config.plane_maximum_imu_difference_deg >= 90.0 ||
    (config.attitude_source != StartupAttitudeSource::kDepth &&
    config.attitude_source != StartupAttitudeSource::kImu) ||
    !std::isfinite(config.imu_roll_bias_deg) ||
    !std::isfinite(config.imu_pitch_bias_deg) ||
    config.imu_sample_count <= 0 ||
    !std::isfinite(config.imu_max_direction_rms_deg) ||
    config.imu_max_direction_rms_deg <= 0.0 ||
    !std::isfinite(config.imu_accel_min_mps2) ||
    config.imu_accel_min_mps2 <= 0.0 ||
    !std::isfinite(config.imu_accel_max_mps2) ||
    config.imu_accel_max_mps2 <= config.imu_accel_min_mps2 ||
    !std::isfinite(config.imu_gyroscope_mean_maximum_degps) ||
    config.imu_gyroscope_mean_maximum_degps <= 0.0 ||
    !std::isfinite(config.imu_gyroscope_stddev_maximum_degps) ||
    config.imu_gyroscope_stddev_maximum_degps <= 0.0 ||
    config.stable_plane_frame_count <= 0 ||
    !std::isfinite(config.maximum_height_stddev_m) ||
    config.maximum_height_stddev_m <= 0.0 ||
    !std::isfinite(config.maximum_plane_normal_rms_deg) ||
    config.maximum_plane_normal_rms_deg <= 0.0 ||
    !std::isfinite(config.timeout_sec) || config.timeout_sec <= 0.0 ||
    config.warmup_sec >= config.timeout_sec)
  {
    throw std::invalid_argument("invalid OAK startup measurement parameter");
  }

  const int sampled_width =
    (config.roi_width + config.point_sample_step - 1) /
    config.point_sample_step;
  const int sampled_height =
    (config.roi_height + config.point_sample_step - 1) /
    config.point_sample_step;
  const int maximum_sample_count = sampled_width * sampled_height;
  if (
    config.minimum_valid_points > maximum_sample_count ||
    config.plane_minimum_inliers > maximum_sample_count)
  {
    throw std::invalid_argument(
            "ground-plane minimum counts exceed the sampled ROI capacity");
  }
}

std::optional<PlaneCandidate> estimateGroundPlane(
  dai::ImgFrame & packet,
  const std::array<double, 3> & specific_force_camera,
  const OakStartupMeasurementConfig & config,
  std::string * rejection)
{
  if (
    packet.getType() != dai::ImgFrame::Type::RAW16 ||
    static_cast<int>(packet.getWidth()) != config.stereo_width ||
    static_cast<int>(packet.getHeight()) != config.stereo_height)
  {
    throw std::runtime_error(
            "DepthAI output is not the requested RAW16 depth geometry");
  }

  const auto & transformation = packet.getTransformation();
  if (!transformation.isValid()) {
    throw std::runtime_error(
            "RGB-aligned depth output has no valid transformation");
  }
  const auto intrinsic_matrix = transformation.getIntrinsicMatrix();
  const double fx = static_cast<double>(intrinsic_matrix[0][0]);
  const double fy = static_cast<double>(intrinsic_matrix[1][1]);
  const double cx = static_cast<double>(intrinsic_matrix[0][2]);
  const double cy = static_cast<double>(intrinsic_matrix[1][2]);
  if (
    !std::isfinite(fx) || !std::isfinite(fy) ||
    !std::isfinite(cx) || !std::isfinite(cy) ||
    fx <= 0.0 || fy <= 0.0)
  {
    throw std::runtime_error("aligned depth intrinsics are invalid");
  }

  const auto up_camera_array = normalized(specific_force_camera);
  const cv::Vec3d up_camera(
    up_camera_array[0], up_camera_array[1], up_camera_array[2]);

  const auto & bytes = packet.getData();
  const std::size_t minimum_stride =
    static_cast<std::size_t>(config.stereo_width) * sizeof(std::uint16_t);
  const std::size_t stride = std::max(
    minimum_stride, static_cast<std::size_t>(packet.getStride()));
  if (
    bytes.size() <
    stride * static_cast<std::size_t>(config.stereo_height))
  {
    throw std::runtime_error("DepthAI returned an undersized depth frame");
  }

  const cv::Rect roi = measurementRoi(config);
  const int sampled_width =
    (config.roi_width + config.point_sample_step - 1) /
    config.point_sample_step;
  const int sampled_height =
    (config.roi_height + config.point_sample_step - 1) /
    config.point_sample_step;
  const auto sample_capacity = static_cast<std::size_t>(
    sampled_width * sampled_height);
  std::vector<cv::Vec3d> points;
  std::vector<double> depths;
  points.reserve(sample_capacity);
  depths.reserve(sample_capacity);

  for (
    int v = roi.y;
    v < roi.y + roi.height;
    v += config.point_sample_step)
  {
    const auto row_offset = static_cast<std::size_t>(v) * stride;
    for (
      int u = roi.x;
      u < roi.x + roi.width;
      u += config.point_sample_step)
    {
      std::uint16_t depth_mm = 0U;
      std::memcpy(
        &depth_mm,
        bytes.data() + row_offset +
        static_cast<std::size_t>(u) * sizeof(depth_mm),
        sizeof(depth_mm));
      const double z_m = static_cast<double>(depth_mm) * 0.001;
      if (
        depth_mm == 0U ||
        z_m < config.minimum_depth_m ||
        z_m > config.maximum_depth_m)
      {
        continue;
      }

      const double x_m = (static_cast<double>(u) - cx) * z_m / fx;
      const double y_m = (static_cast<double>(v) - cy) * z_m / fy;
      points.emplace_back(x_m, y_m, z_m);
      depths.push_back(z_m);
    }
  }

  if (
    points.size() <
    static_cast<std::size_t>(config.minimum_valid_points))
  {
    if (rejection != nullptr) {
      *rejection =
        "insufficient valid depth points for ground-plane fitting";
    }
    return std::nullopt;
  }

  GroundPlaneFitConfig fit_config;
  fit_config.ransac_iterations = config.plane_ransac_iterations;
  fit_config.inlier_threshold_m = config.plane_inlier_threshold_m;
  fit_config.minimum_inliers =
    static_cast<std::size_t>(config.plane_minimum_inliers);
  fit_config.minimum_inlier_ratio = config.plane_minimum_inlier_ratio;
  fit_config.maximum_residual_mad_m =
    config.plane_maximum_residual_mad_m;
  fit_config.maximum_reference_angle_deg =
    config.plane_maximum_imu_difference_deg;
  fit_config.minimum_height_m = config.minimum_height_m;
  fit_config.maximum_height_m = config.maximum_height_m;

  auto plane = fitGroundPlane(points, up_camera, fit_config, rejection);
  if (!plane) {
    return std::nullopt;
  }

  return PlaneCandidate{*plane, median(std::move(depths))};
}

void stopPipeline(
  std::shared_ptr<dai::MessageQueue> & depth_queue,
  std::shared_ptr<dai::MessageQueue> & imu_queue,
  std::unique_ptr<dai::Pipeline> & pipeline,
  std::shared_ptr<dai::Device> & device) noexcept
{
  depth_queue.reset();
  imu_queue.reset();
  if (pipeline) {
    try {
      if (pipeline->isRunning()) {
        pipeline->stop();
        pipeline->wait();
      }
    } catch (...) {
    }
    pipeline.reset();
  }
  device.reset();
}

}  // namespace

OakStartupMeasurement measureOakStartupExtrinsics(
  const OakStartupMeasurementConfig & config,
  const std::function<bool()> & should_stop)
{
  validateConfig(config);

  std::shared_ptr<dai::Device> device;
  std::unique_ptr<dai::Pipeline> pipeline;
  std::shared_ptr<dai::MessageQueue> depth_queue;
  std::shared_ptr<dai::MessageQueue> imu_queue;
  std::shared_ptr<dai::MessageQueue> preview_queue;

  try {
    device = config.device_id.empty()
      ? std::make_shared<dai::Device>(dai::UsbSpeed::SUPER)
      : std::make_shared<dai::Device>(dai::DeviceInfo(config.device_id), dai::UsbSpeed::SUPER);
    pipeline = std::make_unique<dai::Pipeline>(device);
    // DepthAI 3.6 enables startup auto-calibration by default and may flash
    // user EEPROM calibration. Measurement startup must be deterministic and
    // must never mutate device calibration implicitly.
    pipeline->setAutoCalibrationMode(
      dai::Pipeline::AutoCalibrationMode::OFF);
    pipeline->setXLinkChunkSize(0);
    if (config.manual_camera_height_enabled && config.preview_callback) {
      auto camera=pipeline->create<dai::node::Camera>();
      camera->build(dai::CameraBoardSocket::CAM_A);
      preview_queue=camera->requestOutput({640,400},dai::ImgFrame::Type::BGR888i,
        dai::ImgResizeMode::CROP,10.0f,true)->createOutputQueue(1,false);
    }

    if (!config.manual_camera_height_enabled) {
      auto mono_left = pipeline->create<dai::node::Camera>()->build(
        dai::CameraBoardSocket::CAM_B,
        std::make_pair(kOv9282FullWidth, kOv9282FullHeight),
        static_cast<float>(config.stereo_fps));
      auto mono_right = pipeline->create<dai::node::Camera>()->build(
        dai::CameraBoardSocket::CAM_C,
        std::make_pair(kOv9282FullWidth, kOv9282FullHeight),
        static_cast<float>(config.stereo_fps));
      auto * mono_left_output = mono_left->requestOutput(
        std::make_pair(
          static_cast<std::uint32_t>(config.stereo_width),
          static_cast<std::uint32_t>(config.stereo_height)),
        dai::ImgFrame::Type::GRAY8,
        dai::ImgResizeMode::CROP,
        static_cast<float>(config.stereo_fps));
      auto * mono_right_output = mono_right->requestOutput(
        std::make_pair(
          static_cast<std::uint32_t>(config.stereo_width),
          static_cast<std::uint32_t>(config.stereo_height)),
        dai::ImgFrame::Type::GRAY8,
        dai::ImgResizeMode::CROP,
        static_cast<float>(config.stereo_fps));

      auto stereo = pipeline->create<dai::node::StereoDepth>();
      stereo->build(
        *mono_left_output,
        *mono_right_output,
        dai::node::StereoDepth::PresetMode::FAST_ACCURACY);
      stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);
      stereo->setOutputSize(config.stereo_width, config.stereo_height);
      stereo->setOutputKeepAspectRatio(true);
      stereo->setLeftRightCheck(true);
      stereo->setSubpixel(true);
      stereo->setSubpixelFractionalBits(
        config.stereo_subpixel_fractional_bits);
      // Keep disparity shift at zero: RVC2 cannot RGB-align shifted depth,
      // and CAM_A alignment is required for a common height/attitude frame.
      stereo->initialConfig->setLeftRightCheckThreshold(
        config.stereo_left_right_check_threshold);
      stereo->initialConfig->setConfidenceThreshold(
        config.stereo_confidence_threshold);
      depth_queue = stereo->depth.createOutputQueue(
        static_cast<unsigned int>(config.depth_queue_size), false);
    }

    const auto imu_name = device->getConnectedIMU();
    if (imu_name.empty()) {
      throw std::runtime_error("the OAK device reported no connected IMU");
    }
    // CALIBRATED reports already include the EEPROM IMU output rotation.
    // Apply the same relative calibrated-output-to-CAM_A transform used by
    // camera_driver, avoiding both raw affine bias and a double rotation.
    const auto calibration = device->getCalibration();
    const cv::Matx33d imu_to_rgb_rotation = matrix3x3FromCalibration(
      calibration.getImuToCameraExtrinsics(
        dai::CameraBoardSocket::CAM_A, false),
      "IMU-to-CAM_A calibration rotation");
    validateRotationMatrix(
      imu_to_rgb_rotation, "IMU-to-CAM_A calibration rotation");
    const cv::Matx33d imu_to_calibrated_output_rotation =
      matrix3x3FromCalibration(
      calibration.getEepromData().imuExtrinsics.rotationMatrix,
      "runtime calibrated IMU output rotation");
    validateRotationMatrix(
      imu_to_calibrated_output_rotation,
      "runtime calibrated IMU output rotation");
    const cv::Matx33d calibrated_imu_output_to_rgb_rotation =
      imu_to_rgb_rotation * imu_to_calibrated_output_rotation.t();
    validateRotationMatrix(
      calibrated_imu_output_to_rgb_rotation,
      "calibrated IMU output-to-CAM_A rotation");

    auto imu = pipeline->create<dai::node::IMU>();
    imu->enableIMUSensor(
      {
        dai::IMUSensor::ACCELEROMETER_CALIBRATED,
        dai::IMUSensor::GYROSCOPE_CALIBRATED,
      },
      static_cast<int>(std::lround(config.imu_rate_hz)));
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(config.imu_max_batch_reports);
    imu_queue = imu->out.createOutputQueue(
      static_cast<unsigned int>(config.imu_queue_size), false);

    pipeline->start();
    if (!config.manual_camera_height_enabled) {
      // Explicitly apply zero too, so a prior pipeline cannot leave the emitter enabled.
      const bool ir_applied = device->setIrLaserDotProjectorIntensity(
        static_cast<float>(config.ir_dot_projector_intensity));
      if (!ir_applied && config.ir_dot_projector_intensity > 0.0) {
        throw std::runtime_error(
                "failed to enable the OAK IR dot projector; "
                "startup measurement requires a Pro-series device");
      }
    }
    const auto measurement_started_at = std::chrono::steady_clock::now();
    const auto deadline =
      measurement_started_at +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(config.timeout_sec));

    if (config.warmup_sec > 0.0) {
      std::this_thread::sleep_for(
        std::chrono::duration<double>(config.warmup_sec));
      while (imu_queue->tryGet<dai::IMUData>()) {
      }
      if (depth_queue) {
        while (depth_queue->tryGet<dai::ImgFrame>()) {
        }
      }
    }

    std::deque<std::array<double, 3>> imu_direction_samples;
    std::deque<cv::Vec3d> gyroscope_samples_radps;
    std::deque<PlaneCandidate> stable_plane_samples;
    std::array<double, 3> frozen_specific_force{0.0, -1.0, 0.0};
    std::array<double, 3> corrected_specific_force{0.0, -1.0, 0.0};
    double imu_roll_deg = 0.0;
    double imu_pitch_down_deg = 0.0;
    double imu_direction_rms_deg = 0.0;
    double imu_gyroscope_mean_degps = 0.0;
    double imu_gyroscope_stddev_degps = 0.0;
    bool imu_fixed = false;
    std::string last_rejection = "waiting for stable IMU samples";
    StartupRoiPreview roi_preview(
      config.roi_preview_enabled && !config.manual_camera_height_enabled);

    while (std::chrono::steady_clock::now() < deadline) {
      if (should_stop && should_stop()) { throw std::runtime_error("startup measurement cancelled"); }
      if (!pipeline->isRunning()) {
        throw std::runtime_error(
                "OAK pipeline stopped during startup measurement");
      }

      std::shared_ptr<dai::ImgFrame> depth_packet;
      if (preview_queue) {
        if (auto packet=preview_queue->tryGet<dai::ImgFrame>()) {
          const auto & bytes=packet->getData();
          const auto stride=packet->getStride() ? packet->getStride() : packet->getWidth()*3;
          if (packet->getType()==dai::ImgFrame::Type::BGR888i &&
            bytes.size()>=static_cast<std::size_t>(stride)*packet->getHeight()) {
            const cv::Mat image(packet->getHeight(),packet->getWidth(),CV_8UC3,
              const_cast<std::uint8_t *>(bytes.data()),stride);
            config.preview_callback(image);
          }
        }
      }
      if (depth_queue) {
        depth_packet = depth_queue->tryGet<dai::ImgFrame>();
        if (depth_packet) {
          roi_preview.show(*depth_packet, config);
        }
      }

      if (!imu_fixed) {
        auto data = imu_queue->tryGet<dai::IMUData>();
        if (data) {
          for (const auto & packet : data->packets) {
            const auto & acceleration = packet.acceleroMeter;
            const auto & gyroscope = packet.gyroscope;
            const double pair_skew_sec = std::abs(
              timestampSeconds(acceleration.getTimestamp()) -
              timestampSeconds(gyroscope.getTimestamp()));
            if (
              !std::isfinite(pair_skew_sec) ||
              pair_skew_sec > config.maximum_imu_pair_skew_sec)
            {
              imu_direction_samples.clear();
              gyroscope_samples_radps.clear();
              last_rejection = "calibrated accelerometer/gyroscope skew is too large";
              continue;
            }

            const cv::Vec3d acceleration_rgb =
              calibrated_imu_output_to_rgb_rotation * cv::Vec3d(
              static_cast<double>(acceleration.x),
              static_cast<double>(acceleration.y),
              static_cast<double>(acceleration.z));
            const cv::Vec3d gyroscope_rgb =
              calibrated_imu_output_to_rgb_rotation * cv::Vec3d(
              static_cast<double>(gyroscope.x),
              static_cast<double>(gyroscope.y),
              static_cast<double>(gyroscope.z));

            const double magnitude = cv::norm(acceleration_rgb);
            if (
              !std::isfinite(magnitude) ||
              magnitude < config.imu_accel_min_mps2 ||
              magnitude > config.imu_accel_max_mps2)
            {
              imu_direction_samples.clear();
              gyroscope_samples_radps.clear();
              last_rejection = "IMU acceleration magnitude is invalid";
              continue;
            }
            const cv::Vec3d acceleration_direction =
              acceleration_rgb / magnitude;
            imu_direction_samples.push_back({
              acceleration_direction[0],
              acceleration_direction[1],
              acceleration_direction[2]});
            gyroscope_samples_radps.push_back(gyroscope_rgb);
            while (
              static_cast<int>(imu_direction_samples.size()) >
              config.imu_sample_count)
            {
              imu_direction_samples.pop_front();
              gyroscope_samples_radps.pop_front();
            }
          }
        }

        if (
          static_cast<int>(imu_direction_samples.size()) >=
          config.imu_sample_count)
        {
          std::array<double, 3> mean{0.0, 0.0, 0.0};
          for (const auto & sample : imu_direction_samples) {
            for (std::size_t axis = 0; axis < 3U; ++axis) {
              mean[axis] += sample[axis];
            }
          }
          mean = normalized(mean);

          double squared_angle_sum = 0.0;
          for (const auto & sample : imu_direction_samples) {
            const double cosine = std::clamp(
              sample[0] * mean[0] +
              sample[1] * mean[1] +
              sample[2] * mean[2],
              -1.0, 1.0);
            const double angle_deg =
              std::acos(cosine) * kRadiansToDegrees;
            squared_angle_sum += angle_deg * angle_deg;
          }
          imu_direction_rms_deg = std::sqrt(
            squared_angle_sum /
            static_cast<double>(imu_direction_samples.size()));

          cv::Vec3d mean_gyroscope_radps(0.0, 0.0, 0.0);
          for (const auto & sample : gyroscope_samples_radps) {
            mean_gyroscope_radps += sample;
          }
          mean_gyroscope_radps *=
            1.0 / static_cast<double>(gyroscope_samples_radps.size());
          double gyroscope_squared_error_sum = 0.0;
          for (const auto & sample : gyroscope_samples_radps) {
            const cv::Vec3d error = sample - mean_gyroscope_radps;
            gyroscope_squared_error_sum += error.dot(error);
          }
          imu_gyroscope_mean_degps =
            cv::norm(mean_gyroscope_radps) * kRadiansToDegrees;
          imu_gyroscope_stddev_degps = std::sqrt(
            gyroscope_squared_error_sum /
            static_cast<double>(gyroscope_samples_radps.size())) *
            kRadiansToDegrees;
          if (
            imu_direction_rms_deg <=
            config.imu_max_direction_rms_deg &&
            imu_gyroscope_mean_degps <=
            config.imu_gyroscope_mean_maximum_degps &&
            imu_gyroscope_stddev_degps <=
            config.imu_gyroscope_stddev_maximum_degps)
          {
            frozen_specific_force = mean;
            imu_roll_deg =
              std::atan2(-mean[0], -mean[1]) * kRadiansToDegrees;
            imu_pitch_down_deg = std::atan2(
              -mean[2],
              std::hypot(mean[0], mean[1])) * kRadiansToDegrees;
            const cv::Vec3d corrected_up = attitudeUpVector(
              imu_roll_deg - config.imu_roll_bias_deg,
              imu_pitch_down_deg - config.imu_pitch_bias_deg);
            corrected_specific_force = {
              corrected_up[0], corrected_up[1], corrected_up[2]};
            if (config.manual_camera_height_enabled) {
              OakStartupMeasurement result;
              result.device_id = device->getDeviceId();
              result.height_m = config.manual_camera_height_m;
              result.height_source = "manual";
              result.roll_deg =
                imu_roll_deg - config.imu_roll_bias_deg;
              result.pitch_down_deg =
                imu_pitch_down_deg - config.imu_pitch_bias_deg;
              result.attitude_source = "imu";
              result.imu_roll_deg = imu_roll_deg;
              result.imu_pitch_down_deg = imu_pitch_down_deg;
              result.corrected_imu_roll_deg = result.roll_deg;
              result.corrected_imu_pitch_down_deg =
                result.pitch_down_deg;
              result.imu_direction_rms_deg = imu_direction_rms_deg;
              result.imu_gyroscope_mean_degps =
                imu_gyroscope_mean_degps;
              result.imu_gyroscope_stddev_degps =
                imu_gyroscope_stddev_degps;
              stopPipeline(depth_queue, imu_queue, pipeline, device);
              return result;
            }
            imu_fixed = true;
            last_rejection = "waiting for a valid ground plane";
          } else {
            imu_direction_samples.clear();
            gyroscope_samples_radps.clear();
            last_rejection = "calibrated IMU stationary window is not stable";
          }
        }
      } else {
        if (depth_packet) {
          auto candidate = estimateGroundPlane(
            *depth_packet, corrected_specific_force, config, &last_rejection);
          if (!candidate) {
            stable_plane_samples.clear();
          } else {
            stable_plane_samples.push_back(*candidate);
            while (
              static_cast<int>(stable_plane_samples.size()) >
              config.stable_plane_frame_count)
            {
              stable_plane_samples.pop_front();
            }
          }
        }

        if (
          static_cast<int>(stable_plane_samples.size()) >=
          config.stable_plane_frame_count)
        {
          std::vector<double> height_values;
          height_values.reserve(stable_plane_samples.size());
          double mean_height_m = 0.0;
          cv::Vec3d mean_normal(0.0, 0.0, 0.0);
          for (const auto & sample : stable_plane_samples) {
            mean_height_m += sample.plane.height_m;
            height_values.push_back(sample.plane.height_m);
            mean_normal += sample.plane.up_camera;
          }
          mean_height_m /=
            static_cast<double>(stable_plane_samples.size());
          mean_normal = normalized(mean_normal);
          const double median_height_m = median(std::move(height_values));

          double squared_error_sum = 0.0;
          double squared_normal_angle_sum = 0.0;
          for (const auto & sample : stable_plane_samples) {
            const double error = sample.plane.height_m - mean_height_m;
            squared_error_sum += error * error;
            const double cosine = std::clamp(
              sample.plane.up_camera.dot(mean_normal), -1.0, 1.0);
            const double angle_deg =
              std::acos(cosine) * kRadiansToDegrees;
            squared_normal_angle_sum += angle_deg * angle_deg;
          }
          const double height_stddev_m = std::sqrt(
            squared_error_sum /
            static_cast<double>(stable_plane_samples.size()));
          const double plane_normal_rms_deg = std::sqrt(
            squared_normal_angle_sum /
            static_cast<double>(stable_plane_samples.size()));
          if (
            height_stddev_m <= config.maximum_height_stddev_m &&
            plane_normal_rms_deg <=
            config.maximum_plane_normal_rms_deg)
          {
            const cv::Vec3d raw_imu_up(
              frozen_specific_force[0],
              frozen_specific_force[1],
              frozen_specific_force[2]);
            const auto attitude = selectStartupAttitude(
              raw_imu_up,
              mean_normal,
              config.attitude_source,
              config.imu_roll_bias_deg,
              config.imu_pitch_bias_deg);
            std::vector<double> median_depth_values;
            std::vector<double> residual_values;
            std::vector<double> inlier_ratio_values;
            median_depth_values.reserve(stable_plane_samples.size());
            residual_values.reserve(stable_plane_samples.size());
            inlier_ratio_values.reserve(stable_plane_samples.size());
            std::size_t minimum_point_count =
              stable_plane_samples.front().plane.point_count;
            std::size_t minimum_inlier_count =
              stable_plane_samples.front().plane.inlier_count;
            for (const auto & sample : stable_plane_samples) {
              median_depth_values.push_back(sample.median_depth_m);
              residual_values.push_back(sample.plane.residual_mad_m);
              inlier_ratio_values.push_back(sample.plane.inlier_ratio);
              minimum_point_count = std::min(
                minimum_point_count, sample.plane.point_count);
              minimum_inlier_count = std::min(
                minimum_inlier_count, sample.plane.inlier_count);
            }

            OakStartupMeasurement result;
            result.device_id = device->getDeviceId();
            result.height_m = median_height_m;
            result.height_source = "depth_plane_offset";
            result.roll_deg = attitude.roll_deg;
            result.pitch_down_deg = attitude.pitch_down_deg;
            result.attitude_source =
              startupAttitudeSourceName(attitude.source);
            result.imu_roll_deg = imu_roll_deg;
            result.imu_pitch_down_deg = imu_pitch_down_deg;
            result.corrected_imu_roll_deg =
              attitude.corrected_imu_roll_deg;
            result.corrected_imu_pitch_down_deg =
              attitude.corrected_imu_pitch_down_deg;
            result.depth_roll_deg = attitude.depth_roll_deg;
            result.depth_pitch_down_deg = attitude.depth_pitch_down_deg;
            result.imu_direction_rms_deg = imu_direction_rms_deg;
            result.imu_gyroscope_mean_degps =
              imu_gyroscope_mean_degps;
            result.imu_gyroscope_stddev_degps =
              imu_gyroscope_stddev_degps;
            result.height_stddev_m = height_stddev_m;
            result.plane_normal_rms_deg = plane_normal_rms_deg;
            result.median_depth_m =
              median(std::move(median_depth_values));
            result.plane_residual_mad_m =
              median(std::move(residual_values));
            result.plane_inlier_ratio =
              median(std::move(inlier_ratio_values));
            result.plane_imu_difference_deg =
              attitude.imu_depth_difference_deg;
            result.valid_point_count = minimum_point_count;
            result.plane_inlier_count = minimum_inlier_count;
            stopPipeline(depth_queue, imu_queue, pipeline, device);
            return result;
          } else {
            stable_plane_samples.clear();
            last_rejection =
              "ground-plane height or normal is not temporally stable";
          }
        }
      }

      std::this_thread::sleep_for(1ms);
    }

    throw std::runtime_error(
            "OAK startup measurement timed out: " + last_rejection);
  } catch (...) {
    stopPipeline(depth_queue, imu_queue, pipeline, device);
    throw;
  }
}

}  // namespace oak_startup
