#include <bev_handoff/model_load_status.hpp>
#include <atomic>
#include <bev_handoff/runtime_monitor.hpp>
#include "bev_handoff/direct_camera_handoff.hpp"
#include "traffic_detection_test/yolox_detector.hpp"
#include "traffic_detection_test/msg/traffic_light_state.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace traffic_detection_test
{
using Clock = std::chrono::steady_clock;
using Frame = bev_handoff::DirectCameraFrame;
using State = msg::TrafficLightState;

class TrafficLightNode : public rclcpp::Node
{
public:
  explicit TrafficLightNode(const rclcpp::NodeOptions & options)
  : Node("traffic_light_detector", options)
  {
    monitor_=std::make_unique<bev_handoff::RuntimeMonitor>(*this);
    model_ = declare_parameter<std::string>("model_path", "");
    if (model_.empty()) {
      model_ = ament_index_cpp::get_package_share_directory("traffic_detection_test") +
        "/models/traffic_light_yolox_s_640x160_batch_1.int8.qdq.onnx";
    }
    cache_ = declare_parameter<std::string>("engine_cache_path", "");
    const int workspace = declare_parameter<int>("tensorrt_workspace_size_mb", 1024);
    const double fps = declare_parameter<double>("inference_fps", 20.0);
    max_age_ = declare_parameter<double>("max_frame_age_sec", 0.25);
    log_interval_ = declare_parameter<double>("status_log_interval_sec", 2.0);
    left_ = declare_parameter<double>("roi_left", 0.0);
    top_ = declare_parameter<double>("roi_top", 0.1625);
    width_ = declare_parameter<double>("roi_width", 1.0);
    height_ = declare_parameter<double>("roi_height", 0.4);
    score_ = declare_parameter<double>("score_threshold", 0.40);
    confirmation_enabled_ = declare_parameter<bool>("confirmation_enabled", false);
    confirmation_frames_ = declare_parameter<int>("confirmation_frames", 3);
    confirmation_iou_ = declare_parameter<double>("confirmation_min_iou", 0.30);
    confirmation_gap_ = declare_parameter<double>("confirmation_max_gap_sec", 0.25);
    if (confirmation_frames_ < 1 || !std::isfinite(confirmation_iou_) ||
      confirmation_iou_ <= 0 || confirmation_iou_ > 1 ||
      !std::isfinite(confirmation_gap_) || confirmation_gap_ <= 0)
    {throw std::invalid_argument("invalid traffic confirmation parameters");}
    nms_ = declare_parameter<double>("nms_threshold", 0.65);
    min_s_ = declare_parameter<int>("color_min_saturation", 80);
    min_v_ = declare_parameter<int>("color_min_value", 60);
    for (double v : {fps, max_age_, log_interval_, left_, top_, width_, height_, score_, nms_}) {
      if (!std::isfinite(v)) {throw std::invalid_argument("traffic parameters must be finite");}
    }
    if (fps <= 0 || fps > 80 || workspace <= 0 || max_age_ <= 0 || log_interval_ <= 0 ||
      left_ < 0 || top_ < 0 || width_ <= 0 || height_ <= 0 ||
      left_ + width_ > 1 || top_ + height_ > 1 || score_ <= 0 || score_ > 1 ||
      nms_ < 0 || nms_ > 1 || min_s_ < 0 || min_s_ > 255 || min_v_ < 0 || min_v_ > 255)
    {throw std::invalid_argument("invalid traffic ROI/rate/threshold parameters");}
    workspace_ = static_cast<std::size_t>(workspace) * 1024U * 1024U;
    inference_fps_.store(fps);
    declare_parameter<std::vector<std::string>>("runtime_editable_parameters",{"inference_fps"});
    publisher_ = create_publisher<State>(
      declare_parameter<std::string>("state_topic", "/traffic_light/state"), rclcpp::QoS(1));
    runtime_fps_callback_=add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& values) {
        rcl_interfaces::msg::SetParametersResult result;result.successful=false;
        double fps=inference_fps_.load();
        for(const auto& value:values) {
          if(value.get_name()!="inference_fps" || value.get_type()!=rclcpp::ParameterType::PARAMETER_DOUBLE) {
            result.reason="Only inference_fps is live; restart for model changes";return result;
          }
          fps=value.as_double();
          if(!std::isfinite(fps) || fps<1 || fps>80) {
            result.reason="inference_fps must be 1..80";return result;
          }
        }
        inference_fps_.store(fps);result.successful=true;return result;
      });
    rclcpp::PublisherOptions status_options;
    status_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    model_status_pub_ = create_publisher<std_msgs::msg::String>(
      "~/model_status", rclcpp::QoS(1).reliable().transient_local(), status_options);
    model_load_status_ = std::make_unique<bev_handoff::ModelLoadStatus>(
      [publisher = model_status_pub_](const std::string & status) {
        std_msgs::msg::String message; message.data = status; publisher->publish(message);
      });
    model_load_status_->update("LOADING | model=" + model_);
    consumer_ = bev_handoff::registerDirectCameraConsumer(
      [this](std::shared_ptr<const Frame> frame) {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock()) {contention_drops_.fetch_add(1); return;}
        if (pending_) {replaced_.fetch_add(1);}
        pending_ = std::move(frame);
        condition_.notify_one();
      });
    try {
      worker_ = std::thread([this]() {
        try {run();}
        catch (const std::exception & e) {
          bev_handoff::publishTrafficInferenceFps(0.0, Clock::time_point{});
          bev_handoff::publishTrafficSignalState(State::UNKNOWN, Clock::time_point{});
          // A publication/shutdown failure must not terminate the shared
          // camera/BEV/lane process through an uncaught worker exception.
          if (rclcpp::ok(get_node_base_interface()->get_context())) {
            try {model_load_status_->update(std::string("FAILED | worker stopped: ") + e.what());}
            catch (const std::exception &) {}  // Context may be shutting down.
            RCLCPP_ERROR(get_logger(), "Traffic worker stopped: %s", e.what());
          }
        }
      });
    } catch (...) {
      bev_handoff::unregisterDirectCameraConsumer(consumer_);
      throw;
    }
    RCLCPP_INFO(get_logger(), "Traffic settings: score=%.2f, confirmation=%s, frames=%d, IoU=%.2f, gap=%.2fs",
      score_, confirmation_enabled_ ? "on" : "off", confirmation_frames_, confirmation_iou_, confirmation_gap_);
    RCLCPP_INFO(get_logger(),
      "Traffic observation enabled: INT8 640x160, %.1fHz, shared pre-BEV color input, "
      "no preview/control commands. Engine initialization runs on its own worker.", fps);
  }

  ~TrafficLightNode() override
  {
    bev_handoff::unregisterDirectCameraConsumer(consumer_);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    condition_.notify_all();
    if (worker_.joinable()) {worker_.join();}
    bev_handoff::publishTrafficInferenceFps(0.0, Clock::time_point{});
    bev_handoff::publishTrafficSignalState(State::UNKNOWN, Clock::time_point{});
  }

private:
  static double milliseconds(Clock::duration elapsed)
  {return std::chrono::duration<double, std::milli>(elapsed).count();}

  std::uint8_t color(const Frame & frame, const TrafficLightDetection & detection)
  {
    const auto & b = detection.box;
    if (!std::isfinite(b.x) || !std::isfinite(b.y) ||
      !std::isfinite(b.width) || !std::isfinite(b.height)) {return State::UNKNOWN;}
    const int left = std::clamp(static_cast<int>(std::floor(b.x)), 0, frame.width) & ~1;
    const int top = std::clamp(static_cast<int>(std::floor(b.y)), 0, frame.height) & ~1;
    const int right = std::min(frame.width,
      (std::clamp(static_cast<int>(std::ceil(b.x + b.width)), 0, frame.width) + 1) & ~1);
    const int bottom = std::min(frame.height,
      (std::clamp(static_cast<int>(std::ceil(b.y + b.height)), 0, frame.height) + 1) & ~1);
    if (right <= left || bottom <= top) {return State::UNKNOWN;}
    // Only the winning box is converted on CPU; no full-frame BGR or preview.
    cv::Mat y(bottom - top, right - left, CV_8UC1,
      const_cast<std::uint8_t *>(frame.nv12) + top * frame.stride + left, frame.stride);
    cv::Mat uv((bottom - top) / 2, (right - left) / 2, CV_8UC2,
      const_cast<std::uint8_t *>(frame.nv12) + frame.height * frame.stride +
      (top / 2) * frame.stride + left, frame.stride);
    cv::cvtColorTwoPlane(y, uv, bgr_, cv::COLOR_YUV2BGR_NV12);
    cv::cvtColor(bgr_, hsv_, cv::COLOR_BGR2HSV);
    std::uint64_t red = 0, green = 0;
    for (int row = 0; row < hsv_.rows; ++row) {
      const auto * pixels = hsv_.ptr<cv::Vec3b>(row);
      for (int col = 0; col < hsv_.cols; ++col) {
        const auto p = pixels[col];
        if (p[1] < min_s_ || p[2] < min_v_) {continue;}
        const std::uint64_t energy = static_cast<unsigned>(p[1]) * p[2];
        if (p[0] <= 20 || p[0] >= 165) {red += energy;}
        else if (p[0] >= 35 && p[0] <= 95) {green += energy;}
      }
    }
    return red > green ? State::RED : green > red ? State::GREEN : State::UNKNOWN;
  }

  // Worker-only state: no extra queue, inference, or lane-thread locking.
  std::uint8_t confirm(std::uint8_t value, const cv::Rect2f & box, Clock::time_point captured)
  {
    if (!confirmation_enabled_) {return value;}
    if (value == State::UNKNOWN) {
      confirmation_count_ = 0;
      candidate_state_ = State::UNKNOWN;
      return State::UNKNOWN;
    }
    const double intersection = (box & candidate_box_).area();
    const double combined = box.area() + candidate_box_.area() - intersection;
    const double iou = combined > 0 ? intersection / combined : 0;
    const auto gap = captured - candidate_time_;
    const bool matches = confirmation_count_ > 0 && value == candidate_state_ &&
      gap > Clock::duration::zero() && gap <= std::chrono::duration<double>(confirmation_gap_) &&
      iou >= confirmation_iou_;
    confirmation_count_ = matches ? confirmation_count_ + (confirmation_count_ < confirmation_frames_ ? 1 : 0) : 1;
    candidate_state_ = value;
    candidate_box_ = box;
    candidate_time_ = captured;
    return confirmation_count_ >= confirmation_frames_ ? value : State::UNKNOWN;
  }

  void publish_model_status(const std::string & status)
  {
    std_msgs::msg::String message;
    message.data = status;
    model_status_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "[MODEL CHECK] %s", status.c_str());
  }

  std::unique_ptr<bev_handoff::RuntimeMonitor> monitor_;
  void run()
  {
    bev_handoff::publishTrafficInferenceFps(0.0, Clock::time_point{});
    // Building/deserializing TensorRT never blocks component construction or
    // the BEV executor. Cold engine building can still compete for GPU time.
    std::unique_ptr<YoloxDetector> detector;
    State preparing;
    preparing.capture_age_ms = -1.0F;
    bev_handoff::publishTrafficSignalState(State::UNKNOWN, Clock::time_point{});
    publisher_->publish(preparing);
    try {
      detector = std::make_unique<YoloxDetector>(
        model_, "TENSORRT", cache_, "int8", 640, 160, score_, nms_, workspace_,
        [this](const std::string & stage) {model_load_status_->update("LOADING | " + stage);});
      model_load_status_->update("LOADING | Trial NV12 inference / CUDA warmup");
      // Exercise the production NV12 -> CUDA -> TensorRT -> output path.
      cv::Mat black_nv12(240, 640, CV_8UC1, cv::Scalar(16));
      black_nv12.rowRange(160, 240).setTo(128);
      static_cast<void>(detector->detect_nv12(black_nv12.data, black_nv12.total(),
        black_nv12.step[0], 640, 160, cv::Rect(0, 0, 640, 160)));
      model_load_status_->update("READY | model=" + model_ + " | trial NV12 inference=PASS");
      RCLCPP_INFO(get_logger(), "Traffic TensorRT engine ready; awaiting shared color frames.");
    } catch (const std::exception & e) {
      detector.reset();
      model_load_status_->update(std::string("FAILED | ") + e.what());
      RCLCPP_ERROR(get_logger(), "Traffic initialization failed: %s", e.what());
      // Stay alive publishing UNKNOWN; do not disturb the lane pipeline.
    }
    auto deadline = Clock::now();
    auto last_log = deadline;
    auto fps_window_started = deadline;
    std::uint64_t completed_inferences = 0;
    auto last_publish = deadline - std::chrono::seconds(1);
    std::uint64_t count = 0, stale = 0;
    double sum_ms = 0, max_ms = 0, sum_infer = 0, max_infer = 0, max_age_ms = 0;
    for (;;) {
      std::shared_ptr<const Frame> frame;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        // Rate limit inference STARTS. There is never a FIFO or catch-up burst.
        condition_.wait_until(lock, deadline, [this]() {return stopping_;});
        if (stopping_) {break;}
        condition_.wait_for(lock, std::chrono::milliseconds(100),
          [this]() {return stopping_ || pending_;});
        if (stopping_) {break;}
        frame = std::move(pending_);
      }
      const auto started = Clock::now();
      deadline = started + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1. / inference_fps_.load()));
      State message;
      cv::Rect2f detected_box;
      message.state = State::UNKNOWN;
      message.capture_age_ms = -1.0F;
      if (frame) {
        message.header = frame->header;
        message.source_generation = frame->generation;
        const auto age = started - frame->captured_at;
        if (detector && age >= Clock::duration::zero() &&
          age <= std::chrono::duration<double>(max_age_))
        {
          try {
            const cv::Rect roi(
              static_cast<int>(left_ * frame->width), static_cast<int>(top_ * frame->height),
              static_cast<int>(width_ * frame->width), static_cast<int>(height_ * frame->height));
            bev_handoff::RuntimeSample runtime_sample(monitor_.get(),"traffic_inference");
            const auto result = detector->detect_nv12(frame->nv12, frame->size, frame->stride,
              frame->width, frame->height, roi);
            ++completed_inferences;
            message.inference_ms = result.timing.forward_nanoseconds / 1.0e6;
            if (!result.detections.empty()) {
              const auto best = std::max_element(result.detections.begin(), result.detections.end(),
                [](const auto & a, const auto & b) {return a.score < b.score;});
              message.detection_score = best->score;
              message.state = color(*frame, *best);
              detected_box = best->box;
            }
          } catch (const std::exception & e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
              "Traffic processing failed: %s", e.what());
          }
        } else {++stale;}
        message.capture_age_ms = milliseconds(Clock::now() - frame->captured_at);
        if (message.capture_age_ms < 0 || message.capture_age_ms > max_age_ * 1000) {
          message.state = State::UNKNOWN;
        }
        message.state = confirm(message.state, detected_box, frame->captured_at);
        message.processing_ms = milliseconds(Clock::now() - started);
        ++count;
        sum_ms += message.processing_ms;
        max_ms = std::max(max_ms, static_cast<double>(message.processing_ms));
        sum_infer += message.inference_ms;
        max_infer = std::max(max_infer, static_cast<double>(message.inference_ms));
        max_age_ms = std::max(max_age_ms, static_cast<double>(message.capture_age_ms));
      }
      if (!frame) {confirm(State::UNKNOWN, {}, Clock::time_point{});}
      if (frame || started - last_publish >= std::chrono::milliseconds(100)) {
        bev_handoff::publishTrafficSignalState(message.state, frame ?
          frame->captured_at + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(max_age_)) : Clock::time_point{});
        publisher_->publish(message);
        last_publish = started;
      }
      const auto fps_now = Clock::now();
      const double fps_elapsed = std::chrono::duration<double>(fps_now - fps_window_started).count();
      if (fps_elapsed >= 1.0) {
        bev_handoff::publishTrafficInferenceFps(
          static_cast<double>(completed_inferences) / fps_elapsed,
          completed_inferences > 0 ? fps_now + std::chrono::milliseconds(1500) : Clock::time_point{});
        completed_inferences = 0;
        fps_window_started = fps_now;
      }
      const double elapsed = std::chrono::duration<double>(Clock::now() - last_log).count();
      if (elapsed >= log_interval_) {
        RCLCPP_INFO(get_logger(),
          "Traffic | state=%s | input_processed=%.1fHz | inference=%.2f/%.2fms avg/max | "
          "processing=%.2f/%.2fms avg/max | capture_age_max=%.1fms | "
          "replaced=%llu contention=%llu stale_or_unavailable=%llu",
          message.state == State::RED ? "RED" : message.state == State::GREEN ? "GREEN" : "UNKNOWN",
          count / elapsed, count ? sum_infer / count : 0, max_infer,
          count ? sum_ms / count : 0, max_ms, max_age_ms,
          static_cast<unsigned long long>(replaced_.exchange(0)),
          static_cast<unsigned long long>(contention_drops_.exchange(0)),
          static_cast<unsigned long long>(stale));
        if (!count) {
          RCLCPP_WARN(get_logger(), "No shared color frames: load camera_driver in this container with CAM_A.");
        }
        count = stale = 0;
        sum_ms = max_ms = sum_infer = max_infer = max_age_ms = 0;
        last_log = Clock::now();
      }
    }
  }

  std::string model_, cache_;
  std::size_t workspace_;
  double max_age_, log_interval_, left_, top_, width_, height_, score_, nms_;
  int min_s_, min_v_;
  bool confirmation_enabled_{false};
  int confirmation_frames_{3}, confirmation_count_{0};
  double confirmation_iou_{0.30}, confirmation_gap_{0.25};
  std::uint8_t candidate_state_{State::UNKNOWN};
  cv::Rect2f candidate_box_;
  Clock::time_point candidate_time_;
  std::atomic<double> inference_fps_{20.};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr runtime_fps_callback_;
  std::uint64_t consumer_{0};
  rclcpp::Publisher<State>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr model_status_pub_;
  std::unique_ptr<bev_handoff::ModelLoadStatus> model_load_status_;
  std::mutex mutex_;
  std::condition_variable condition_;
  bool stopping_{false};
  std::shared_ptr<const Frame> pending_;
  std::thread worker_;
  std::atomic<std::uint64_t> replaced_{0}, contention_drops_{0};
  cv::Mat bgr_, hsv_;
};
}  // namespace traffic_detection_test
RCLCPP_COMPONENTS_REGISTER_NODE(traffic_detection_test::TrafficLightNode)
