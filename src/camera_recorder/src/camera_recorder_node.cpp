#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "depthai/depthai.hpp"
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "rclcpp/rclcpp.hpp"

namespace camera_recorder
{

using namespace std::chrono_literals;

namespace
{

constexpr std::uint32_t kSensorWidth = 1280U;
constexpr std::uint32_t kSensorHeight = 800U;

std::string timestamp_string()
{
  const std::time_t now = std::time(nullptr);
  std::tm local_time{};
  localtime_r(&now, &local_time);
  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y%m%d_%H%M%S");
  return stream.str();
}

std::int64_t sensor_time_ns(const dai::ImgFrame & frame)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    frame.getTimestamp(dai::CameraExposureOffset::MIDDLE).time_since_epoch()).count();
}

const char * usb_speed_name(const dai::UsbSpeed speed)
{
  switch (speed) {
    case dai::UsbSpeed::LOW:
      return "LOW";
    case dai::UsbSpeed::FULL:
      return "FULL";
    case dai::UsbSpeed::HIGH:
      return "HIGH";
    case dai::UsbSpeed::SUPER:
      return "SUPER";
    case dai::UsbSpeed::SUPER_PLUS:
      return "SUPER_PLUS";
    case dai::UsbSpeed::UNKNOWN:
    default:
      return "UNKNOWN";
  }
}

struct FrameSet
{
  cv::Mat rgb;
  cv::Mat left_ir;
  cv::Mat right_ir;
  std::int64_t host_ros_time_ns{};
  std::int64_t rgb_sensor_time_ns{};
  std::int64_t left_sensor_time_ns{};
  std::int64_t right_sensor_time_ns{};
  std::int64_t rgb_sequence{};
  std::int64_t left_sequence{};
  std::int64_t right_sequence{};
  std::int64_t rgb_exposure_us{};
  std::int64_t left_exposure_us{};
  std::int64_t right_exposure_us{};
  int rgb_iso{};
  int left_iso{};
  int right_iso{};
};

}  // namespace

class CameraRecorderNode final : public rclcpp::Node
{
public:
  CameraRecorderNode()
  : Node("camera_recorder")
  {
    load_parameters();
    validate_parameters();

    try {
      start_oak();
      create_session();
      open_segment();
      capture_thread_ = std::thread(&CameraRecorderNode::capture_loop, this);
      writer_thread_ = std::thread(&CameraRecorderNode::writer_loop, this);
      status_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(status_log_interval_sec_)),
        std::bind(&CameraRecorderNode::log_status, this));
    } catch (...) {
      stop();
      throw;
    }
  }

  ~CameraRecorderNode() override
  {
    stop();
  }

private:
  void load_parameters()
  {
    output_directory_ = declare_parameter<std::string>(
      "output_directory", "recordings");
    session_prefix_ = declare_parameter<std::string>("session_prefix", "drive");
    device_id_ = declare_parameter<std::string>("device_id", "");

    width_ = declare_parameter<int>("width", 1280);
    height_ = declare_parameter<int>("height", 800);
    fps_ = declare_parameter<double>("fps", 30.0);
    undistort_enabled_ = declare_parameter<bool>("undistort_enabled", true);
    sync_threshold_ms_ = declare_parameter<double>("sync_threshold_ms", 10.0);

    codec_ = declare_parameter<std::string>("codec", "MJPG");
    file_extension_ = declare_parameter<std::string>("file_extension", ".avi");
    segment_duration_sec_ = declare_parameter<double>(
      "segment_duration_sec", 300.0);
    writer_queue_capacity_ = declare_parameter<int>("writer_queue_capacity", 8);
    status_log_interval_sec_ = declare_parameter<double>(
      "status_log_interval_sec", 2.0);

    rgb_manual_exposure_enabled_ = declare_parameter<bool>(
      "rgb_manual_exposure_enabled", false);
    rgb_manual_exposure_us_ = declare_parameter<int>(
      "rgb_manual_exposure_us", 5000);
    rgb_manual_sensitivity_iso_ = declare_parameter<int>(
      "rgb_manual_sensitivity_iso", 400);
    ir_manual_exposure_enabled_ = declare_parameter<bool>(
      "ir_manual_exposure_enabled", true);
    ir_manual_exposure_us_ = declare_parameter<int>(
      "ir_manual_exposure_us", 5000);
    ir_manual_sensitivity_iso_ = declare_parameter<int>(
      "ir_manual_sensitivity_iso", 800);
    ir_dot_projector_intensity_ = declare_parameter<double>(
      "ir_dot_projector_intensity", 0.0);
    ir_flood_light_intensity_ = declare_parameter<double>(
      "ir_flood_light_intensity", 0.5);
  }

  void validate_parameters()
  {
    if (output_directory_.empty() || session_prefix_.empty()) {
      throw std::invalid_argument(
              "output_directory and session_prefix must not be empty");
    }
    if (
      width_ <= 0 || height_ <= 0 || width_ % 2 != 0 || height_ % 2 != 0 ||
      width_ > static_cast<int>(kSensorWidth) ||
      height_ > static_cast<int>(kSensorHeight))
    {
      throw std::invalid_argument(
              "width and height must be positive, even, and no larger than 1280x800");
    }
    if (!std::isfinite(fps_) || fps_ <= 0.0 || fps_ > 60.0) {
      throw std::invalid_argument("fps must be in (0, 60]");
    }
    if (
      !std::isfinite(sync_threshold_ms_) || sync_threshold_ms_ <= 0.0 ||
      sync_threshold_ms_ >= 1000.0 / fps_)
    {
      throw std::invalid_argument(
              "sync_threshold_ms must be positive and less than one frame period");
    }
    if (codec_.size() != 4U) {
      throw std::invalid_argument("codec must contain exactly four characters");
    }
    if (file_extension_.empty()) {
      throw std::invalid_argument("file_extension must not be empty");
    }
    if (file_extension_.front() != '.') {
      file_extension_.insert(file_extension_.begin(), '.');
    }
    if (
      !std::isfinite(segment_duration_sec_) || segment_duration_sec_ < 0.0 ||
      writer_queue_capacity_ < 1 || writer_queue_capacity_ > 256 ||
      !std::isfinite(status_log_interval_sec_) || status_log_interval_sec_ <= 0.0)
    {
      throw std::invalid_argument(
              "invalid segment duration, queue capacity, or status interval");
    }
    validate_exposure(rgb_manual_exposure_us_, rgb_manual_sensitivity_iso_, "RGB");
    validate_exposure(ir_manual_exposure_us_, ir_manual_sensitivity_iso_, "IR");
    validate_intensity(ir_dot_projector_intensity_, "ir_dot_projector_intensity");
    validate_intensity(ir_flood_light_intensity_, "ir_flood_light_intensity");

    if (segment_duration_sec_ > 0.0) {
      frames_per_segment_ = std::max<std::uint64_t>(
        1U, static_cast<std::uint64_t>(std::llround(fps_ * segment_duration_sec_)));
    }
  }

  void validate_exposure(
    const int exposure_us, const int sensitivity_iso, const char * label) const
  {
    const auto maximum_exposure_us = static_cast<int>(std::floor(1.0e6 / fps_));
    if (
      exposure_us < 10 || exposure_us > maximum_exposure_us ||
      sensitivity_iso < 100 || sensitivity_iso > 1600)
    {
      throw std::invalid_argument(
              std::string(label) + " exposure must fit within one frame and ISO must be 100..1600");
    }
  }

  static void validate_intensity(const double value, const char * name)
  {
    if (!std::isfinite(value) || value < 0.0 || value > 1.0) {
      throw std::invalid_argument(std::string(name) + " must be between 0.0 and 1.0");
    }
  }

  void configure_exposure(
    const std::shared_ptr<dai::node::Camera> & camera,
    const bool manual,
    const int exposure_us,
    const int sensitivity_iso)
  {
    if (manual) {
      camera->initialControl.setManualExposure(
        static_cast<std::uint32_t>(exposure_us),
        static_cast<std::uint32_t>(sensitivity_iso));
    }
  }

  void start_oak()
  {
    device_ = device_id_.empty() ?
      std::make_shared<dai::Device>(dai::UsbSpeed::SUPER) :
      std::make_shared<dai::Device>(
      dai::DeviceInfo(device_id_), dai::UsbSpeed::SUPER);
    pipeline_ = std::make_unique<dai::Pipeline>(device_);
    pipeline_->setAutoCalibrationMode(dai::Pipeline::AutoCalibrationMode::OFF);
    pipeline_->setXLinkChunkSize(0);

    const auto sensor_size = std::make_pair(kSensorWidth, kSensorHeight);
    const auto output_size = std::make_pair(
      static_cast<std::uint32_t>(width_), static_cast<std::uint32_t>(height_));
    const auto requested_fps = static_cast<float>(fps_);

    auto rgb_camera = pipeline_->create<dai::node::Camera>()->build(
      dai::CameraBoardSocket::CAM_A, sensor_size, requested_fps);
    auto left_camera = pipeline_->create<dai::node::Camera>()->build(
      dai::CameraBoardSocket::CAM_B, sensor_size, requested_fps);
    auto right_camera = pipeline_->create<dai::node::Camera>()->build(
      dai::CameraBoardSocket::CAM_C, sensor_size, requested_fps);

    configure_exposure(
      rgb_camera, rgb_manual_exposure_enabled_, rgb_manual_exposure_us_,
      rgb_manual_sensitivity_iso_);
    configure_exposure(
      left_camera, ir_manual_exposure_enabled_, ir_manual_exposure_us_,
      ir_manual_sensitivity_iso_);
    configure_exposure(
      right_camera, ir_manual_exposure_enabled_, ir_manual_exposure_us_,
      ir_manual_sensitivity_iso_);

    auto * rgb_output = rgb_camera->requestOutput(
      output_size, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP,
      requested_fps, undistort_enabled_);
    auto * left_output = left_camera->requestOutput(
      output_size, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP,
      requested_fps, undistort_enabled_);
    auto * right_output = right_camera->requestOutput(
      output_size, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP,
      requested_fps, undistort_enabled_);

    auto sync = pipeline_->create<dai::node::Sync>();
    sync->setSyncThreshold(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double, std::milli>(sync_threshold_ms_)));
    sync->setSyncAttempts(-1);
    rgb_output->link(sync->inputs["rgb"]);
    left_output->link(sync->inputs["left_ir"]);
    right_output->link(sync->inputs["right_ir"]);

    output_queue_ = sync->out.createOutputQueue(
      static_cast<unsigned int>(writer_queue_capacity_), false);
    pipeline_->build();
    const auto bridge = sync->out.getXLinkBridge();
    if (!bridge || !bridge->xLinkOut) {
      throw std::runtime_error("DepthAI did not create the synchronized XLink bridge");
    }
    bridge->xLinkOut->input.setMaxSize(
      static_cast<unsigned int>(writer_queue_capacity_));
    bridge->xLinkOut->input.setBlocking(false);
    pipeline_->start();

    const bool dot_ok = device_->setIrLaserDotProjectorIntensity(
      static_cast<float>(ir_dot_projector_intensity_));
    const bool flood_ok = device_->setIrFloodLightIntensity(
      static_cast<float>(ir_flood_light_intensity_));
    if (
      (!dot_ok && ir_dot_projector_intensity_ > 0.0) ||
      (!flood_ok && ir_flood_light_intensity_ > 0.0))
    {
      device_->setIrLaserDotProjectorIntensity(0.0F);
      device_->setIrFloodLightIntensity(0.0F);
      throw std::runtime_error(
              "requested IR emitter is unavailable; use an OAK Pro or set its intensity to 0");
    }

    RCLCPP_INFO(
      get_logger(),
      "OAK synchronized capture ready: CAM_A RGB + CAM_B/C IR, %dx%d@%.1f FPS, USB=%s",
      width_, height_, fps_, usb_speed_name(device_->getUsbSpeed()));
  }

  void create_session()
  {
    std::error_code error;
    auto root = std::filesystem::absolute(output_directory_, error);
    if (error) {
      throw std::runtime_error(
              "could not resolve output directory: " + error.message());
    }
    std::filesystem::create_directories(root, error);
    if (error) {
      throw std::runtime_error(
              "could not create output directory '" + root.string() + "': " +
              error.message());
    }

    const std::string base_name = session_prefix_ + "_" + timestamp_string();
    session_directory_ = root / base_name;
    for (int suffix = 1; std::filesystem::exists(session_directory_); ++suffix) {
      session_directory_ = root / (base_name + "_" + std::to_string(suffix));
    }
    if (!std::filesystem::create_directory(session_directory_, error) || error) {
      throw std::runtime_error(
              "could not create recording session '" + session_directory_.string() +
              "': " + error.message());
    }

    timestamp_file_.open(session_directory_ / "timestamps.csv");
    if (!timestamp_file_) {
      throw std::runtime_error("could not open timestamps.csv");
    }
    timestamp_file_ <<
      "recorded_frame,segment,segment_frame,host_ros_time_ns,"
      "rgb_sensor_time_ns,left_ir_sensor_time_ns,right_ir_sensor_time_ns,"
      "rgb_sequence,left_ir_sequence,right_ir_sequence,"
      "rgb_exposure_us,rgb_iso,left_ir_exposure_us,left_ir_iso,"
      "right_ir_exposure_us,right_ir_iso\n";

    std::ofstream info(session_directory_ / "session_info.txt");
    if (!info) {
      throw std::runtime_error("could not open session_info.txt");
    }
    info << "width=" << width_ << '\n'
         << "height=" << height_ << '\n'
         << "fps=" << std::setprecision(12) << fps_ << '\n'
         << "codec=" << codec_ << '\n'
         << "file_extension=" << file_extension_ << '\n'
         << "segment_duration_sec=" << segment_duration_sec_ << '\n'
         << "undistort_enabled=" << std::boolalpha << undistort_enabled_ << '\n'
         << "sync_threshold_ms=" << sync_threshold_ms_ << '\n'
         << "ir_dot_projector_intensity=" << ir_dot_projector_intensity_ << '\n'
         << "ir_flood_light_intensity=" << ir_flood_light_intensity_ << '\n';

    RCLCPP_INFO(
      get_logger(), "Recording session: %s", session_directory_.string().c_str());
  }

  std::filesystem::path video_path(
    const std::string & stream_name, const std::uint64_t segment) const
  {
    std::ostringstream filename;
    filename << stream_name << '_' << std::setw(4) << std::setfill('0') << segment
             << file_extension_;
    return session_directory_ / filename.str();
  }

  void open_segment()
  {
    close_video_writers();
    const int fourcc = cv::VideoWriter::fourcc(
      codec_[0], codec_[1], codec_[2], codec_[3]);
    const cv::Size frame_size(width_, height_);
    const auto segment = current_segment_.load(std::memory_order_relaxed);
    const auto rgb_path = video_path("center_rgb", segment);
    const auto left_path = video_path("left_ir", segment);
    const auto right_path = video_path("right_ir", segment);

    rgb_writer_.open(rgb_path.string(), fourcc, fps_, frame_size, true);
    left_writer_.open(left_path.string(), fourcc, fps_, frame_size, false);
    right_writer_.open(right_path.string(), fourcc, fps_, frame_size, false);
    if (!rgb_writer_.isOpened() || !left_writer_.isOpened() || !right_writer_.isOpened()) {
      close_video_writers();
      throw std::runtime_error(
              "could not open all video writers; verify codec and output filesystem support");
    }
    segment_frame_count_ = 0U;
    RCLCPP_INFO(
      get_logger(), "Opened recording segment %04lu (%s)",
      static_cast<unsigned long>(segment), codec_.c_str());
  }

  void close_video_writers()
  {
    rgb_writer_.release();
    left_writer_.release();
    right_writer_.release();
  }

  bool valid_frame(
    const dai::ImgFrame & frame, const dai::ImgFrame::Type expected_type) const
  {
    return frame.getType() == expected_type &&
           static_cast<int>(frame.getWidth()) == width_ &&
           static_cast<int>(frame.getHeight()) == height_;
  }

  void capture_loop()
  {
    try {
      while (!stop_requested_.load(std::memory_order_relaxed)) {
        if (!pipeline_ || !pipeline_->isRunning()) {
          throw std::runtime_error("DepthAI pipeline stopped unexpectedly");
        }
        auto group = output_queue_->tryGet<dai::MessageGroup>();
        if (!group) {
          std::this_thread::sleep_for(100us);
          continue;
        }

        auto rgb = group->get<dai::ImgFrame>("rgb");
        auto left = group->get<dai::ImgFrame>("left_ir");
        auto right = group->get<dai::ImgFrame>("right_ir");
        if (
          !rgb || !left || !right ||
          !valid_frame(*rgb, dai::ImgFrame::Type::NV12) ||
          !valid_frame(*left, dai::ImgFrame::Type::GRAY8) ||
          !valid_frame(*right, dai::ImgFrame::Type::GRAY8))
        {
          invalid_sets_.fetch_add(1U, std::memory_order_relaxed);
          continue;
        }

        FrameSet set;
        set.rgb = rgb->getCvFrame().clone();
        set.left_ir = left->getFrame(false).clone();
        set.right_ir = right->getFrame(false).clone();
        if (
          set.rgb.type() != CV_8UC3 || set.left_ir.type() != CV_8UC1 ||
          set.right_ir.type() != CV_8UC1 || set.rgb.empty() ||
          set.left_ir.empty() || set.right_ir.empty())
        {
          invalid_sets_.fetch_add(1U, std::memory_order_relaxed);
          continue;
        }
        set.host_ros_time_ns = get_clock()->now().nanoseconds();
        set.rgb_sensor_time_ns = sensor_time_ns(*rgb);
        set.left_sensor_time_ns = sensor_time_ns(*left);
        set.right_sensor_time_ns = sensor_time_ns(*right);
        set.rgb_sequence = rgb->getSequenceNum();
        set.left_sequence = left->getSequenceNum();
        set.right_sequence = right->getSequenceNum();
        set.rgb_exposure_us = rgb->getExposureTime().count();
        set.left_exposure_us = left->getExposureTime().count();
        set.right_exposure_us = right->getExposureTime().count();
        set.rgb_iso = rgb->getSensitivity();
        set.left_iso = left->getSensitivity();
        set.right_iso = right->getSensitivity();

        {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          if (frame_queue_.size() >= static_cast<std::size_t>(writer_queue_capacity_)) {
            frame_queue_.pop_front();
            queue_drops_.fetch_add(1U, std::memory_order_relaxed);
          }
          frame_queue_.push_back(std::move(set));
          captured_sets_.fetch_add(1U, std::memory_order_relaxed);
        }
        queue_condition_.notify_one();
      }
    } catch (const std::exception & exception) {
      if (!stop_requested_.exchange(true)) {
        RCLCPP_FATAL(get_logger(), "Camera capture failed: %s", exception.what());
        queue_condition_.notify_all();
        rclcpp::shutdown();
      }
    }
  }

  void writer_loop()
  {
    try {
      while (true) {
        FrameSet set;
        {
          std::unique_lock<std::mutex> lock(queue_mutex_);
          queue_condition_.wait(
            lock,
            [this]() {
              return stop_requested_.load(std::memory_order_relaxed) ||
                     !frame_queue_.empty();
            });
          if (frame_queue_.empty()) {
            if (stop_requested_.load(std::memory_order_relaxed)) {
              break;
            }
            continue;
          }
          set = std::move(frame_queue_.front());
          frame_queue_.pop_front();
        }

        if (frames_per_segment_ > 0U && segment_frame_count_ >= frames_per_segment_) {
          ++current_segment_;
          open_segment();
        }
        rgb_writer_.write(set.rgb);
        left_writer_.write(set.left_ir);
        right_writer_.write(set.right_ir);

        const auto recorded_frame = written_sets_.fetch_add(
          1U, std::memory_order_relaxed);
        timestamp_file_ << recorded_frame << ','
                        << current_segment_.load(std::memory_order_relaxed) << ','
                        << segment_frame_count_ << ',' << set.host_ros_time_ns << ','
                        << set.rgb_sensor_time_ns << ',' << set.left_sensor_time_ns << ','
                        << set.right_sensor_time_ns << ',' << set.rgb_sequence << ','
                        << set.left_sequence << ',' << set.right_sequence << ','
                        << set.rgb_exposure_us << ',' << set.rgb_iso << ','
                        << set.left_exposure_us << ',' << set.left_iso << ','
                        << set.right_exposure_us << ',' << set.right_iso << '\n';
        ++segment_frame_count_;
        if ((recorded_frame + 1U) % static_cast<std::uint64_t>(std::ceil(fps_)) == 0U) {
          timestamp_file_.flush();
        }
        if (!timestamp_file_) {
          throw std::runtime_error("failed while writing timestamps.csv");
        }
      }
    } catch (const std::exception & exception) {
      stop_requested_.store(true, std::memory_order_relaxed);
      RCLCPP_FATAL(get_logger(), "Video writer failed: %s", exception.what());
      queue_condition_.notify_all();
      rclcpp::shutdown();
    }
  }

  void log_status()
  {
    std::size_t queued = 0U;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      queued = frame_queue_.size();
    }
    RCLCPP_INFO(
      get_logger(),
      "[RECORDER] captured=%lu written=%lu queue=%zu/%d dropped=%lu invalid=%lu segment=%04lu",
      static_cast<unsigned long>(captured_sets_.load()),
      static_cast<unsigned long>(written_sets_.load()), queued,
      writer_queue_capacity_,
      static_cast<unsigned long>(queue_drops_.load()),
      static_cast<unsigned long>(invalid_sets_.load()),
      static_cast<unsigned long>(current_segment_.load(std::memory_order_relaxed)));
  }

  void append_final_info() noexcept
  {
    if (session_directory_.empty() || final_info_written_.exchange(true)) {
      return;
    }
    try {
      std::ofstream info(session_directory_ / "session_info.txt", std::ios::app);
      info << "captured_sets=" << captured_sets_.load() << '\n'
           << "written_sets=" << written_sets_.load() << '\n'
           << "queue_drops=" << queue_drops_.load() << '\n'
           << "invalid_sets=" << invalid_sets_.load() << '\n';
    } catch (...) {
    }
  }

  void stop() noexcept
  {
    const bool already_stopping = stop_requested_.exchange(true);
    queue_condition_.notify_all();

    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    try {
      if (device_) {
        device_->setIrLaserDotProjectorIntensity(0.0F);
        device_->setIrFloodLightIntensity(0.0F);
      }
      if (pipeline_ && pipeline_->isRunning()) {
        pipeline_->stop();
      }
    } catch (...) {
    }

    queue_condition_.notify_all();
    if (writer_thread_.joinable()) {
      writer_thread_.join();
    }
    close_video_writers();
    if (timestamp_file_.is_open()) {
      timestamp_file_.flush();
      timestamp_file_.close();
    }
    append_final_info();

    if (!already_stopping && !session_directory_.empty()) {
      RCLCPP_INFO(
        get_logger(), "Recording closed: %s (%lu synchronized frames)",
        session_directory_.string().c_str(),
        static_cast<unsigned long>(written_sets_.load()));
    }
  }

  std::string output_directory_;
  std::string session_prefix_;
  std::string device_id_;
  int width_{1280};
  int height_{800};
  double fps_{30.0};
  bool undistort_enabled_{true};
  double sync_threshold_ms_{10.0};
  std::string codec_{"MJPG"};
  std::string file_extension_{".avi"};
  double segment_duration_sec_{300.0};
  int writer_queue_capacity_{8};
  double status_log_interval_sec_{2.0};
  bool rgb_manual_exposure_enabled_{false};
  int rgb_manual_exposure_us_{5000};
  int rgb_manual_sensitivity_iso_{400};
  bool ir_manual_exposure_enabled_{true};
  int ir_manual_exposure_us_{5000};
  int ir_manual_sensitivity_iso_{800};
  double ir_dot_projector_intensity_{0.0};
  double ir_flood_light_intensity_{0.5};

  std::shared_ptr<dai::Device> device_;
  std::unique_ptr<dai::Pipeline> pipeline_;
  std::shared_ptr<dai::MessageQueue> output_queue_;

  std::filesystem::path session_directory_;
  cv::VideoWriter rgb_writer_;
  cv::VideoWriter left_writer_;
  cv::VideoWriter right_writer_;
  std::ofstream timestamp_file_;
  std::uint64_t frames_per_segment_{0U};
  std::atomic<std::uint64_t> current_segment_{0U};
  std::uint64_t segment_frame_count_{0U};

  std::deque<FrameSet> frame_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_condition_;
  std::thread capture_thread_;
  std::thread writer_thread_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> final_info_written_{false};
  std::atomic<std::uint64_t> captured_sets_{0U};
  std::atomic<std::uint64_t> written_sets_{0U};
  std::atomic<std::uint64_t> queue_drops_{0U};
  std::atomic<std::uint64_t> invalid_sets_{0U};
};

}  // namespace camera_recorder

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<camera_recorder::CameraRecorderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    node.reset();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("camera_recorder"), "Recorder startup failed: %s",
      exception.what());
    rclcpp::shutdown();
    return 1;
  }
  return 0;
}
