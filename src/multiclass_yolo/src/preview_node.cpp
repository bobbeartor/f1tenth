#include "multiclass_yolo/yolox_detector.hpp"
#include "multiclass_yolo/preview_exit.hpp"
#include "preview_panel.hpp"
#include "preview_bev.hpp"
#include "preview_measurement.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "depthai/depthai.hpp"
#include "rclcpp/rclcpp.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

using Clock = multiclass_yolo::PreviewClock;
using namespace std::chrono_literals;
namespace multiclass_yolo {
class Preview : public rclcpp::Node {
  struct AverageRate {
    std::uint64_t count{};
    double total_ms{};

    void add(double ms) {
      if (ms <= 0) return;
      ++count;
      total_ms += ms;
    }

    double fps() const {return total_ms > 0 ? 1000.0 * count / total_ms : 0.0;}
  };

  static std::string format_fps(double fps) {
    return fps > 0 ? cv::format("%.1f", fps) : "--";
  }

  struct CameraChange {
    double fps;
    std::optional<point_cloud::Config> stereo;
    std::optional<oak_startup::OakStartupMeasurementConfig> fit;
  };

  struct FirstFrames {
    std::shared_ptr<dai::ImgFrame> rgb, depth;
  };

  struct ModelSelection {
    std::string precision{"fp16"};
    bool roi80{false};

    bool operator==(const ModelSelection & other) const {
      return precision == other.precision && roi80 == other.roi80;
    }

    int width() const {return roi80 ? 448 : 512;}
    int height() const {return roi80 ? 224 : 320;}
    const char * variant() const {return roi80 ? "roi80_448x224" : "holdout10";}
  };

 public:
  Preview() : Node("multiclass_yolo") {
    const auto share = ament_index_cpp::get_package_share_directory("multiclass_yolo");
    const auto default_model = share + "/models/multiclass_yolox_tiny_512x320_holdout10.onnx";
    fp_model_path_ = declare_parameter<std::string>("model_path", default_model);
    int8_model_path_ = declare_parameter<std::string>("int8_model_path",
      share + "/models/multiclass_yolox_tiny_512x320_holdout10.int8.qdq.onnx");
    roi80_model_path_ = share + "/models/roi80_448x224.onnx";
    const auto variant = declare_parameter<std::string>("model_variant", "holdout10");
    if (variant != "holdout10" && variant != "roi80_448x224")
      throw std::invalid_argument("model_variant must be holdout10 or roi80_448x224");
    active_model_.roi80 = variant == "roi80_448x224";
    backend_ = declare_parameter<std::string>("inference_backend", "TENSORRT");
    active_model_.precision = declare_parameter<std::string>("engine_precision", "fp16");
    std::transform(backend_.begin(), backend_.end(), backend_.begin(),
      [](unsigned char c) {return static_cast<char>(std::toupper(c));});
    std::transform(active_model_.precision.begin(), active_model_.precision.end(), active_model_.precision.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});
    startup_model_ = active_model_;
    if (active_model_.roi80) {
      if (active_model_.precision == "int8")
        throw std::invalid_argument("roi80_448x224 requires fp16 or fp32 precision");
      if (fp_model_path_ != default_model)
        roi80_model_path_ = std::exchange(fp_model_path_, default_model);
    } else if (active_model_.precision == "int8" && fp_model_path_ != default_model) {
      int8_model_path_ = std::exchange(fp_model_path_, default_model);
    }
    startup_cache_ = declare_parameter<std::string>("engine_cache_path", "");
    const char * home = std::getenv("HOME");
    cache_directory_ = std::filesystem::path(home ? home : "/tmp") / ".cache/multiclass_yolo";
    auto directory = declare_parameter<std::string>("measurement_directory", "");
    measurement_directory_ = directory.empty() ?
      cache_directory_ / "measurements" : std::filesystem::absolute(directory);
    measurement_seconds_ = declare_parameter<int>("measurement_seconds", 15);
    if (measurement_seconds_ < 1 || measurement_seconds_ > max_measurement_seconds_)
      throw std::invalid_argument("measurement_seconds must be in [1, 120]");
    format_ = declare_parameter<std::string>("input_format", "nv12");
    width_ = declare_parameter<int>("width", 1280);
    height_ = declare_parameter<int>("height", 800);
    fps_ = declare_parameter<double>("sensor_fps", 40.0);
    roi_height_percent_ = declare_parameter<int>("roi_height_percent", 65);
    if (roi_height_percent_ < 1 || roi_height_percent_ > 100)
      throw std::invalid_argument("roi_height_percent must be in [1, 100]");
    undistort_ = declare_parameter<bool>("undistort_enabled", true);
    timeout_ = declare_parameter<double>("frame_timeout_sec", 5.0);
    auto workspace = declare_parameter<int>("tensorrt_workspace_size_mb", 1024);
    nms_ = static_cast<float>(declare_parameter<double>("nms_threshold", 0.65));
    if ((format_ != "rgb" && format_ != "nv12") || width_ <= 0 || height_ <= 0 ||
        width_ % 2 || height_ % 2 || !std::isfinite(fps_) || fps_ <= 0 ||
        !std::isfinite(timeout_) || timeout_ <= 0 || workspace < 1 || workspace > 16384)
      throw std::invalid_argument("Invalid input_format/dimensions/FPS/timeout/workspace");
    for (int i=0; i<4; ++i) {
      double v = declare_parameter<double>(std::string(names_[i])+"_threshold", 0.70);
      if (!std::isfinite(v) || v < 0 || v > 1) throw std::invalid_argument("Thresholds must be in [0,1]");
      thresholds_[i] = static_cast<float>(v);
    }
    workspace_bytes_ = std::size_t(workspace) * 1024 * 1024;
    detector_ = make_detector(active_model_);
    if (format_ == "nv12" && !detector_->supports_nv12_input())
      throw std::invalid_argument("input_format=nv12 requires TensorRT; CPU: use input_format=rgb engine_precision=fp32");
    warmup_detector(*detector_, roi_percent_for(active_model_), thresholds_);
    if (declare_parameter<bool>("depth_fusion_enabled", true)) {
      if (!undistort_) throw std::invalid_argument("BEV fusion requires undistort_enabled=true");
      bev_settings_.emplace(*this);
      active_stereo_ = bev_settings_->roi.camera;
      if (bev_settings_->estimate_ground) {
        try {
          ground_pose_ = measure_ground(bev_settings_->startup);
        } catch (const std::exception & error) {
          RCLCPP_WARN(get_logger(), "Ground measurement failed: %s. Use Refit ground to retry.", error.what());
        }
      }
    }
    auto device = ground_pose_ ?
      std::make_shared<dai::Device>(dai::DeviceInfo(ground_pose_->device_id), dai::UsbSpeed::SUPER) :
      std::make_shared<dai::Device>(dai::UsbSpeed::SUPER);
    camera_id_ = device->getDeviceId();
    for (const auto & camera : device->getConnectedCameraFeatures()) {
      if (camera.socket != dai::CameraBoardSocket::CAM_A) continue;
      for (const auto & config : camera.configs) {
        if (config.width == sensor_width_ && config.height == sensor_height_) {
          min_fps_ = config.minFps;
          max_fps_ = config.maxFps;
          break;
        }
      }
      break;
    }
    start_camera(fps_, device, active_stereo_);
    active_calibration_ = stream_calibration_;
    last_frame_ = Clock::now();
    capture_ = std::thread([this] { capture_loop(); });
  }
  ~Preview() override {
    stop_ = true;
    if (capture_.joinable()) capture_.join();
    if (pending_detector_.valid()) pending_detector_.wait();
    if (measurement_) {
      try {finish_measurement("interrupted");} catch (...) {}
    }
    try { close_camera(); } catch (...) {}
  }
  void run() {
    const std::string window = "multiclass_yolo";
    cv::namedWindow(window, cv::WINDOW_NORMAL);
    PreviewPanel panel;
    cv::setMouseCallback(window, [](int event, int x, int y, int, void * data) {
      if (event == cv::EVENT_LBUTTONDOWN) static_cast<PreviewPanel *>(data)->click(x, y);
    }, &panel);
    constexpr int video_width = static_cast<int>(1024 * 1.1 + .5);
    constexpr int video_top = 147;
    const int video_height = std::max(1, static_cast<int>(std::lround(
      static_cast<double>(video_width) * height_ / width_)));
    const int video_left = PreviewPanel::width;
    cv::Mat canvas(std::max(PreviewPanel::min_height, video_top + video_height),
      video_left + video_width, CV_8UC3, cv::Scalar(20,20,20));
    cv::Mat video = canvas(cv::Rect(video_left, video_top, video_width, video_height));
    cv::resizeWindow(window, canvas.cols, canvas.rows);
    cv::putText(video, "Waiting for camera frames...", {20, video_height / 2},
      cv::FONT_HERSHEY_SIMPLEX, .7, {240,240,240}, 1, cv::LINE_AA);
    std::vector<Detection> detections;
    std::vector<std::pair<int, int>> depth_counts;
    std::vector<ObjectEstimate> objects;
    auto detection_thresholds = thresholds_;
    PreviewExit exit_state;
    std::string exit_reason = "ROS shutdown / stop requested";
    const auto property = [&window](int id) {
      try { return cv::getWindowProperty(window, id); }
      catch (const cv::Exception &) { return -1.; }
    };
    std::uint64_t previous = 0, outputs = 0, skipped = 0, session = 0;
    double displayed_fps = 0;
    std::string displayed_status, displayed_toolbar;
    auto last_log = Clock::now();
    const char * compute_label = backend_ == "TENSORRT" ? "TensorRT compute" : "CPU compute";
    AverageRate inference_rate, classification_latency, position_latency;
    AverageRate processing_rate, filter_rate, render_rate, input_color_rate, display_color_rate;
    std::optional<std::uint64_t> input_baseline;
    double compute_fps = 0;
    std::string throughput_line, latency_line, stage_line;
    bool rates_dirty = false;
    const auto clear_rate_window = [&] {
      inference_rate = classification_latency = position_latency = {};
      processing_rate = filter_rate = render_rate = input_color_rate = display_color_rate = {};
      last_log = Clock::now();
    };
    const auto reset_inference_window = [&] {
      clear_rate_window();
      input_baseline.reset();
      compute_fps = 0;
      throughput_line.clear();
      latency_line.clear();
      stage_line.clear();
      rates_dirty = panel.dirty = true;
    };
    while (rclcpp::ok() && !stop_) {
      rclcpp::spin_some(get_node_base_interface());
      if (poll_model_change()) {
        detections.clear();
        depth_counts.clear();
        objects.clear();
        if (video_enabled_) video.setTo(cv::Scalar(20,20,20));
        reset_inference_window();
      }
      if (measurement_ && Clock::now() >= measurement_->end) finish_measurement("completed");
      std::shared_ptr<dai::ImgFrame> frame;
      Clock::time_point received;
      std::uint64_t count;
      double fps;
      bool changing_fps;
      std::string camera_status;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!error_.empty()) throw std::runtime_error(error_);
        if (session != camera_session_) {
          session = camera_session_;
          previous = outputs = skipped = 0;
          reset_inference_window();
        }
        count = received_; received = last_frame_;
        fps = fps_;
        changing_fps = requested_camera_.has_value();
        camera_status = fps_status_;
        if (count != previous) frame = latest_;
      }
      if (!changing_fps && std::chrono::duration<double>(Clock::now()-received).count() > timeout_)
        throw std::runtime_error("No camera frame within frame_timeout_sec; check OAK/USB and other camera processes");
      std::optional<MeasurementFrame> sample;
      std::optional<Clock::time_point> preview_submitted, position_completed;
      Clock::time_point detection_completed;
      if (frame) {
        if (previous == 0) RCLCPP_INFO(get_logger(), "First camera frame received; starting inference");
        const auto began = Clock::now();
        if (measurement_ && measurement_->contains(began)) {
          sample.emplace();
          sample->started_ms = milliseconds(began - measurement_->start);
          sample->sequence = frame->getSequenceNum();
          (*sample)[Stage::queue] = milliseconds(began - received);
          const auto base = std::max(previous, capture_measurement_->base);
          if (measurement_->contains(received) && count > base)
            sample->host_skipped = count - base - 1;
        }
        skipped += count > previous ? count - previous - 1 : 0;
        previous = count;
        detector_->set_thresholds(thresholds_);
        const int roi_percent = roi_percent_for(active_model_);
        const cv::Rect roi = frame_roi(roi_percent);
        const auto & bytes = frame->getData();
        const std::size_t stride = frame->getStride() > 0 ? frame->getStride() : width_;
        if (stride < std::size_t(width_) || bytes.size() < stride*std::size_t(height_)*3/2)
          throw std::runtime_error("Invalid NV12 buffer/pitch");
        YoloxDetectionResult result;
        cv::Mat rgb;
        cv::Mat y(height_, width_, CV_8UC1, const_cast<std::uint8_t *>(bytes.data()), stride);
        cv::Mat uv(height_/2, width_/2, CV_8UC2, const_cast<std::uint8_t *>(bytes.data())+stride*height_, stride);
        double input_color_ms = 0, display_color_ms = 0;
        std::optional<Clock::time_point> filter_completed;
        if (format_ == "nv12") {
          result = detector_->detect_nv12(bytes.data(), bytes.size(), stride, width_, height_, roi);
        } else {
          const auto conversion_started = Clock::now();
          cv::cvtColorTwoPlane(y, uv, rgb, cv::COLOR_YUV2RGB_NV12);
          input_color_ms = milliseconds(Clock::now() - conversion_started);
          // Model metadata requires BGR: blobFromImage swaps this RGB buffer to BGR.
          result = detector_->detect(rgb, roi, true);
        }
        detection_completed = Clock::now();
        if (sample) {
          const auto & timing = result.timing;
          sample->detections = result.detections.size();
          (*sample)[Stage::receive_to_result] = milliseconds(detection_completed - received);
          if (format_ == "rgb") {
            (*sample)[Stage::input_color] = input_color_ms;
            (*sample)[Stage::cpu_preprocess] = timing.preprocessing_nanoseconds / 1e6;
          } else {
            (*sample)[Stage::cuda_preprocess] = timing.preprocessing_nanoseconds / 1e6;
          }
          if (backend_ == "TENSORRT") {
            (*sample)[Stage::upload] = timing.input_transfer_nanoseconds / 1e6;
            (*sample)[Stage::download] = timing.output_transfer_nanoseconds / 1e6;
          }
          (*sample)[Stage::inference] = timing.forward_nanoseconds / 1e6;
          (*sample)[Stage::postprocess] = timing.postprocessing_nanoseconds / 1e6;
        }
        const auto make_display = [&] {
          cv::Mat display;
          const auto display_started = Clock::now();
          if (format_ == "nv12") cv::cvtColorTwoPlane(y, uv, display, cv::COLOR_YUV2BGR_NV12);
          else cv::cvtColor(rgb, display, cv::COLOR_RGB2BGR);
          display_color_ms = milliseconds(Clock::now() - display_started);
          if (sample) (*sample)[Stage::display_color] = display_color_ms;
          return display;
        };
        if (bev_settings_) {
          const auto fusion_started = Clock::now();
          try {
            auto fused = make_bev(*frame, result.detections, session, make_display);
            filter_completed = fused.filter_completed;
            if (filter_completed && fused.sync_ms && !result.detections.empty()) {
              const double filter_ms = milliseconds(*filter_completed - fusion_started);
              filter_rate.add(filter_ms);
              if (sample) (*sample)[Stage::depth_association] = filter_ms;
            }
            if (fused.has_position) position_completed = filter_completed;
            if (sample && position_completed)
              (*sample)[Stage::receive_to_position] = milliseconds(*position_completed - received);
            depth_counts = std::move(fused.points);
            objects = std::move(fused.objects);
            if (video_enabled_) show_bev(video, fused.image);
            if (sample && fused.sync_ms) (*sample)[Stage::rgb_depth_skew] = *fused.sync_ms;
          } catch (const std::exception & error) {
            depth_counts.clear();
            objects.clear();
            if (video_enabled_) show_bev(video, {});
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "BEV fusion: %s", error.what());
          }
          const auto fusion_finished = Clock::now();
          if (filter_completed && video_enabled_) {
            const double render_ms = std::max(0.0, milliseconds(fusion_finished - *filter_completed) - display_color_ms);
            render_rate.add(render_ms);
            if (sample) (*sample)[Stage::bev_render] = render_ms;
          }
          if (sample) (*sample)[Stage::depth_fusion] =
            std::max(0.0, milliseconds(fusion_finished - fusion_started) - display_color_ms);
        } else if (video_enabled_) {
          auto display = make_display();
          const auto overlay_started = Clock::now();
          if (roi.y > 0)
            cv::line(display, {0, roi.y}, {width_ - 1, roi.y}, {0, 220, 255}, 1, cv::LINE_AA);
          detector_->draw(display, result.detections);
          cv::resize(display, video, video.size(), 0, 0, cv::INTER_LINEAR);
          if (sample) (*sample)[Stage::overlay] = milliseconds(Clock::now() - overlay_started);
        }
        const double total_ms = milliseconds(Clock::now() - began);
        if (sample) (*sample)[Stage::processing] = total_ms;
        std::uint64_t current_received;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (session != camera_session_) continue;
          current_received = received_;
        }
        if (!input_baseline) input_baseline = count - 1;
        ++outputs;
        classification_latency.add(milliseconds(detection_completed - received));
        if (position_completed) position_latency.add(milliseconds(*position_completed - received));
        inference_rate.add(result.timing.forward_nanoseconds / 1e6);
        processing_rate.add(total_ms);
        input_color_rate.add(input_color_ms);
        display_color_rate.add(display_color_ms);

        const double window_s = std::chrono::duration<double>(Clock::now() - last_log).count();
        if (window_s >= 2) {
          const double input_fps = (current_received - *input_baseline) / window_s;
          const double classification_fps = classification_latency.count / window_s;
          const double position_fps = position_latency.count / window_s;
          compute_fps = inference_rate.fps();
          const std::string position_fps_text = bev_settings_ ? cv::format("%.1f", position_fps) : "--";
          throughput_line = cv::format("Actual FPS | Input %.1f | Class result %.1f | Obstacle position %s | Target %.6g | Skipped %llu",
            input_fps, classification_fps, position_fps_text.c_str(),
            fps, static_cast<unsigned long long>(skipped));
          latency_line = "Receive-to-result (FPS from mean delay) | Class " + format_fps(classification_latency.fps()) +
            " | Obstacle position " + format_fps(position_latency.fps());
          stage_line = "Stage FPS | Processing " + format_fps(processing_rate.fps()) +
            " | Depth/filter " + format_fps(filter_rate.fps()) + " | BEV render " + format_fps(render_rate.fps()) +
            " | Input color " + format_fps(input_color_rate.fps()) +
            " | Display color " + format_fps(display_color_rate.fps());
          RCLCPP_INFO(get_logger(), "%s; %s; %s; %s %.1f FPS", throughput_line.c_str(),
            latency_line.c_str(), stage_line.c_str(), compute_label, compute_fps);
          input_baseline = current_received;
          clear_rate_window();
          rates_dirty = panel.dirty = true;
        }
        if (video_enabled_) {
          detections = std::move(result.detections);
          detection_thresholds = thresholds_;
        }
      }
      const auto toolbar = toolbar_status();
      const bool show_frame = frame && video_enabled_;
      if (show_frame || panel.dirty || fps != displayed_fps ||
          camera_status != displayed_status || toolbar != displayed_toolbar) {
        const auto panel_started = Clock::now();
        if (rates_dirty) {
          canvas(cv::Rect(video_left, 68, video_width, video_top - 68)).setTo(cv::Scalar(20,20,20));
          cv::putText(canvas, throughput_line, {video_left + 10, 89}, cv::FONT_HERSHEY_SIMPLEX,
            .53, {255,255,255}, 1, cv::LINE_AA);
          cv::putText(canvas, latency_line, {video_left + 10, 112}, cv::FONT_HERSHEY_SIMPLEX,
            .53, {230,230,180}, 1, cv::LINE_AA);
          cv::putText(canvas, stage_line, {video_left + 10, 135}, cv::FONT_HERSHEY_SIMPLEX,
            .5, {200,220,200}, 1, cv::LINE_AA);
          rates_dirty = false;
        }
        panel.draw_toolbar(canvas, active_model_.precision, active_model_.roi80,
          measurement_seconds_, bool(measurement_), video_enabled_, toolbar,
          compute_label, compute_fps, bev_settings_.has_value(), bev_background_,
          bev_settings_ && bev_settings_->estimate_ground);
        panel.draw(canvas, fps, roi_percent_for(active_model_), active_model_.roi80, names_, thresholds_,
          detection_thresholds, detections, camera_status, settings_for(panel), depth_counts, objects);
        const auto before_imshow = Clock::now();
        cv::imshow(window, canvas);
        const auto after_imshow = Clock::now();
        if (sample) {
          (*sample)[Stage::panel] = milliseconds(before_imshow - panel_started);
          (*sample)[Stage::imshow] = milliseconds(after_imshow - before_imshow);
          if (show_frame) {
            preview_submitted = after_imshow;
            (*sample)[Stage::receive_to_submit] = milliseconds(after_imshow - received);
          }
        }
        if (show_frame && outputs == 1) RCLCPP_INFO(get_logger(), "First detection frame submitted to preview");
        panel.dirty = false;
        displayed_fps = fps;
        displayed_status = camera_status;
        displayed_toolbar = toolbar;
      }
      const auto before_wait_key = Clock::now();
      int key = cv::waitKey(1) & 255;
      if (sample) {
        (*sample)[Stage::wait_key] = milliseconds(Clock::now() - before_wait_key);
        measurement_->record(std::move(*sample), detection_completed, position_completed, preview_submitted);
      }
      if (panel.handle_key(key)) key = -1;
      if (const auto edit = panel.take_submission()) panel.finish(apply_preview_input(*edit));
      switch (panel.take_action()) {
        case PreviewPanel::Action::fp16: request_model({"fp16", false}); break;
        case PreviewPanel::Action::int8: request_model({"int8", false}); break;
        case PreviewPanel::Action::roi80: request_model({"fp16", true}); break;
        case PreviewPanel::Action::toggle_ground:
          toolbar_notice_ = toggle_ground_estimation();
          break;
        case PreviewPanel::Action::refit:
          toolbar_notice_ = request_refit();
          break;
        case PreviewPanel::Action::measure:
          if (measurement_) finish_measurement(Clock::now() >= measurement_->end ? "completed" : "stopped");
          else start_measurement();
          break;
        case PreviewPanel::Action::toggle_background:
          if (measurement_) toolbar_notice_ = "Stop measurement before changing the BEV background";
          else {
            bev_background_ = !bev_background_;
            toolbar_notice_ = bev_background_ ? "BEV camera background" : "Obstacle points on metric grid";
          }
          break;
        case PreviewPanel::Action::toggle_video:
          if (measurement_) {
            toolbar_notice_ = "Stop measurement before changing video mode";
          } else {
            video_enabled_ = !video_enabled_;
            toolbar_notice_.clear();
            detections.clear();
            depth_counts.clear();
            objects.clear();
            video.setTo(cv::Scalar(20,20,20));
            canvas(cv::Rect(video_left, 68, video_width, video_top - 68)).setTo(cv::Scalar(20,20,20));
            if (!video_enabled_) cv::putText(video, "Video paused | Detection running", {20, video.rows / 2},
              cv::FONT_HERSHEY_SIMPLEX, .7, {240,240,240}, 1, cv::LINE_AA);
          }
          break;
        default: break;
      }
      const auto reason = exit_state.reason(key, property(cv::WND_PROP_VISIBLE),
        property(cv::WND_PROP_AUTOSIZE));
      if (reason) {exit_reason = reason; break;}
      if (!frame) std::this_thread::sleep_for(1ms);
    }
    if (measurement_) finish_measurement("interrupted");
    RCLCPP_INFO(get_logger(), "Preview stopped: %s", exit_reason.c_str());
    cv::destroyAllWindows();
  }
 private:
  oak_startup::OakStartupMeasurement measure_ground(oak_startup::OakStartupMeasurementConfig config) {
    config.device_id = camera_id_;
    RCLCPP_INFO(get_logger(), "Measuring ground. Keep the vehicle stationary on a flat floor.");
    auto pose = oak_startup::measureOakStartupExtrinsics(config, [this] {return stop_ || !rclcpp::ok();});
    RCLCPP_INFO(get_logger(), "Ground: height %.3f m, roll %.2f deg, pitch %.2f deg",
      pose.height_m, pose.roll_deg, pose.pitch_down_deg);
    return pose;
  }

  std::string toggle_ground_estimation() {
    if (!bev_settings_) return {};
    if (measurement_) return "Stop measurement before changing ground estimation";
    std::lock_guard<std::mutex> lock(mutex_);
    if (requested_camera_) return "Camera change in progress";
    bev_settings_->estimate_ground = !bev_settings_->estimate_ground;
    fusion_.reset();
    if (bev_settings_->estimate_ground) return "Ground estimation enabled. Press Refit ground to measure.";
    return ground_pose_ ? "Ground estimation off. Using last fit." : "Ground estimation off. Using fixed mount.";
  }

  std::string request_refit() {
    if (!bev_settings_) return {};
    if (!bev_settings_->estimate_ground) return "Enable ground estimation before refitting";
    if (measurement_) return "Stop measurement before refitting ground";
    std::lock_guard<std::mutex> lock(mutex_);
    if (requested_camera_) return "Camera change in progress";
    requested_camera_ = CameraChange{fps_, active_stereo_, bev_settings_->startup};
    fps_status_ = "Refitting ground. Keep vehicle still.";
    return {};
  }

  std::vector<std::pair<const char *, double>> settings_for(const PreviewPanel & panel) const {
    std::vector<std::pair<const char *, double>> values;
    if (!bev_settings_ || !panel.show_settings()) return values;
    for (const auto & control : bev_controls()) values.emplace_back(control.label, control.read(*bev_settings_));
    return values;
  }

  std::string apply_bev_setting(const PreviewPanel::Edit & edit) {
    const auto field = static_cast<std::size_t>(edit.field - PreviewPanel::first_setting_field);
    if (!bev_settings_ || field >= bev_controls().size()) return "Unknown setting";
    const auto & control = bev_controls()[field];
    if (!edit.value || !std::isfinite(*edit.value) || *edit.value < control.min ||
        *edit.value > control.max || (control.integer && std::trunc(*edit.value) != *edit.value))
      return cv::format("Enter %s%.6g to %.6g", control.integer ? "integer " : "", control.min, control.max);
    if (*edit.value == control.read(*bev_settings_)) return {};

    auto next = *bev_settings_;
    control.write(next, *edit.value);
    try {
      roi_lidar::validate(next.roi);
      if (next.startup.minimum_depth_m >= next.startup.maximum_depth_m ||
          next.startup.minimum_height_m >= next.startup.maximum_height_m)
        return "Fit minimum must be below maximum";
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Invalid %s: %s", control.label, error.what());
      return "Invalid combination. See console.";
    }
    if (control.group == SettingGroup::stereo) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (requested_camera_) return "Camera change in progress";
      requested_camera_ = CameraChange{fps_, next.roi.camera, std::nullopt};
      fps_status_ = "Applying stereo settings...";
    } else {
      *bev_settings_ = std::move(next);
      if (control.group == SettingGroup::fit) {
        toolbar_notice_ = "Fit settings saved. Press Refit ground.";
      } else if (control.group == SettingGroup::object) {
        // Read per frame; no projector or camera state depends on these.
        toolbar_notice_ = "Object size/position settings applied";
      } else {
        fusion_.reset();
        toolbar_notice_ = "Ground filter settings applied";
      }
    }
    return {};
  }

  template<class MakeDisplay>
  BevFrame make_bev(dai::ImgFrame & rgb, const std::vector<Detection> & detections,
    std::uint64_t session, const MakeDisplay & make_display)
  {
    std::shared_ptr<dai::ImgFrame> nearest;
    std::shared_ptr<const dai::CalibrationHandler> calibration;
    std::optional<oak_startup::OakStartupMeasurement> pose;
    const auto captured = rgb.getTimestamp(dai::CameraExposureOffset::MIDDLE);
    auto best = bev_settings_->sync_ms;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (session != camera_session_) return {};
      calibration = active_calibration_;
      pose = ground_pose_;
      for (const auto & depth : depth_frames_) {
        const auto timestamp = depth->getTimestamp(dai::CameraExposureOffset::MIDDLE);
        const double age = milliseconds(Clock::now() - timestamp);
        const auto skew = std::abs(milliseconds(captured - timestamp));
        if (age >= 0 && age <= bev_settings_->roi.max_age * 1000.0 && skew <= best) {
          nearest = depth;
          best = skew;
        }
      }
    }
    if (!pose && !bev_settings_->estimate_ground) pose = bev_settings_->fixed_pose;
    if (!calibration || !pose) return {};
    return fusion_.process(rgb, nearest.get(), *calibration, *pose, *bev_settings_, detections, video_enabled_, bev_background_, make_display);
  }

  static void show_bev(cv::Mat & video, const cv::Mat & image) {
    video.setTo(cv::Scalar(20,20,20));
    if (image.empty()) {
      cv::putText(video, "Ground unavailable | Press Refit ground", {20, video.rows / 2},
        cv::FONT_HERSHEY_SIMPLEX, .7, {240,240,240}, 1, cv::LINE_AA);
      return;
    }
    const double scale = std::min(double(video.cols) / image.cols, double(video.rows) / image.rows);
    const int w = std::clamp(static_cast<int>(std::lround(image.cols * scale)), 1, video.cols);
    const int h = std::clamp(static_cast<int>(std::lround(image.rows * scale)), 1, video.rows);
    cv::Mat target = video(cv::Rect((video.cols - w) / 2, (video.rows - h) / 2, w, h));
    cv::resize(image, target, target.size(), 0, 0, cv::INTER_LINEAR);
  }

  void start_camera(double fps, const std::shared_ptr<dai::Device> & device,
    const std::optional<point_cloud::Config> & stereo_config)
  {
    auto pipeline = std::make_unique<dai::Pipeline>(device);
    pipeline->setXLinkChunkSize(0);
    auto camera = pipeline->create<dai::node::Camera>()->build(
      dai::CameraBoardSocket::CAM_A, std::make_pair(std::uint32_t(sensor_width_), std::uint32_t(sensor_height_)), float(fps));
    auto output = camera->requestOutput(std::make_pair(std::uint32_t(width_), std::uint32_t(height_)),
      dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, float(fps), undistort_);
    auto queue = output->createOutputQueue(1U, false);
    std::shared_ptr<dai::MessageQueue> depth_queue;
    std::shared_ptr<dai::node::StereoDepth> stereo;
    std::shared_ptr<const dai::CalibrationHandler> calibration;
    if (stereo_config) {
      stereo = point_cloud::addStereoDepth(*pipeline, *stereo_config);
      depth_queue = stereo->depth.createOutputQueue(1U, false);
      calibration = std::make_shared<dai::CalibrationHandler>(device->getCalibration());
    }
    pipeline->build();
    auto bridge = output->getXLinkBridge();
    if (!bridge || !bridge->xLinkOut) throw std::runtime_error("Missing XLink bridge");
    bridge->xLinkOut->input.setMaxSize(1);
    bridge->xLinkOut->input.setBlocking(false);
    if (stereo) {
      const auto depth_bridge = stereo->depth.getXLinkBridge();
      if (!depth_bridge || !depth_bridge->xLinkOut) throw std::runtime_error("Missing stereo XLink bridge");
      depth_bridge->xLinkOut->input.setMaxSize(1);
      depth_bridge->xLinkOut->input.setBlocking(false);
    }
    pipeline->start();
    if (stereo_config) {
      if (!device->setIrLaserDotProjectorIntensity(static_cast<float>(stereo_config->dot_intensity)) &&
          stereo_config->dot_intensity > 0)
        throw std::runtime_error("IR dot is unsupported on this camera");
      if (!device->setIrFloodLightIntensity(static_cast<float>(stereo_config->flood_intensity)) &&
          stereo_config->flood_intensity > 0)
        throw std::runtime_error("IR flood is unsupported on this camera");
    }
    depth_queue_ = std::move(depth_queue);
    stream_calibration_ = std::move(calibration);
    pipeline_ = std::move(pipeline);
    queue_ = std::move(queue);
    RCLCPP_INFO(get_logger(), "CAM_A NV12 %dx%d @ %.6g; path=%s",
      width_, height_, fps, format_.c_str());
    if (device->getUsbSpeed() == dai::UsbSpeed::HIGH)
      RCLCPP_WARN(get_logger(), "USB2 connection detected; measure actual FPS on this connection");
  }

  void close_camera() {
    if (pipeline_) {
      pipeline_->stop();
      pipeline_->wait();
      queue_.reset();
      depth_queue_.reset();
      stream_calibration_.reset();
      pipeline_.reset();
    }
  }

  int roi_percent_for(const ModelSelection & selection) const {
    return selection.roi80 ? 80 : roi_height_percent_;
  }

  cv::Rect frame_roi(int percent) const {
    const int height = std::max(1, static_cast<int>(std::lround(height_ * (percent / 100.0))));
    return {0, height_ - height, width_, height};
  }

  const std::string & model_path(const ModelSelection & selection) const {
    if (selection.roi80) return roi80_model_path_;
    return selection.precision == "int8" ? int8_model_path_ : fp_model_path_;
  }

  std::unique_ptr<YoloxDetector> make_detector(const ModelSelection & selection) const {
    const auto & model = model_path(selection);
    const auto & precision = selection.precision;
    if (!std::filesystem::is_regular_file(model)) throw std::runtime_error("Model missing: " + model);
    auto cache = selection == startup_model_ ? startup_cache_ : std::string{};
    if (cache.empty() && backend_ == "TENSORRT") {
      const auto modified = std::filesystem::last_write_time(model).time_since_epoch().count();
      const auto identity = std::filesystem::canonical(model).string() + "|" +
        std::to_string(std::filesystem::file_size(model)) + "|" +
        std::to_string(static_cast<long long>(modified)) + "|" + std::to_string(workspace_bytes_);
      cache = (cache_directory_ / (std::to_string(std::hash<std::string>{}(identity)) +
        "." + precision + ".engine")).string();
    }
    return std::make_unique<YoloxDetector>(model, backend_, cache, precision,
      selection.width(), selection.height(), 0.0f, nms_, workspace_bytes_,
      [logger = get_logger()](const std::string & status) {RCLCPP_INFO(logger, "%s", status.c_str());},
      selection.roi80 ? 100 : 0);
  }

  void warmup_detector(YoloxDetector & detector, int roi_percent,
    const std::array<float, 4> & thresholds) const
  {
    detector.set_thresholds(thresholds);
    const auto roi = frame_roi(roi_percent);
    if (format_ == "nv12") {
      const auto stride = static_cast<std::size_t>(width_);
      std::vector<std::uint8_t> pixels(stride * height_ * 3 / 2, 128);
      for (int i = 0; i < 3; ++i)
        detector.detect_nv12(pixels.data(), pixels.size(), stride, width_, height_, roi);
    } else {
      const cv::Mat image(height_, width_, CV_8UC3, cv::Scalar(114, 114, 114));
      for (int i = 0; i < 3; ++i) detector.detect(image, roi, true);
    }
  }

  void request_model(ModelSelection selection) {
    if (selection == active_model_) return;
    if (backend_ != "TENSORRT") {
      toolbar_notice_ = "Model buttons require TensorRT";
      return;
    }
    if (measurement_ || pending_detector_.valid()) {
      toolbar_notice_ = "Finish the current measurement or model change first";
      return;
    }
    pending_model_ = selection;
    toolbar_notice_ = std::string("Loading ") + selection.variant() + " " + selection.precision + " engine...";
    try {
      pending_detector_ = std::async(std::launch::async,
        [this, selection, roi_percent = roi_percent_for(selection), thresholds = thresholds_] {
          auto candidate = make_detector(selection);
          warmup_detector(*candidate, roi_percent, thresholds);
          return candidate;
        });
    } catch (const std::exception & error) {
      toolbar_notice_ = "Cannot start engine load";
      RCLCPP_WARN(get_logger(), "Cannot start engine load: %s", error.what());
    }
  }

  bool poll_model_change() {
    if (!pending_detector_.valid() || pending_detector_.wait_for(0s) != std::future_status::ready)
      return false;
    try {
      auto next = pending_detector_.get();
      detector_ = std::move(next);
      active_model_ = pending_model_;
      toolbar_notice_ = "Engine ready";
      return true;
    } catch (const std::exception & error) {
      toolbar_notice_ = "Engine load failed. Previous model active.";
      RCLCPP_WARN(get_logger(), "%s %s engine load failed: %s",
        pending_model_.variant(), pending_model_.precision.c_str(), error.what());
      return false;
    }
  }

  void start_measurement() {
    if (pending_detector_.valid()) {
      toolbar_notice_ = "Wait for the engine to be ready";
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (requested_camera_ || received_ == 0) {
        toolbar_notice_ = "Wait for the camera stream";
        return;
      }
      nlohmann::json settings = {
        {"model", model_path(active_model_)}, {"model_variant", active_model_.variant()},
        {"precision", active_model_.precision}, {"backend", backend_},
        {"model_input_width", active_model_.width()}, {"model_input_height", active_model_.height()},
        {"input_format", format_}, {"video_enabled", video_enabled_}, {"sensor_fps", fps_},
        {"frame_width", width_}, {"frame_height", height_}, {"roi_height_percent", roi_percent_for(active_model_)},
        {"class_thresholds", {{"con", thresholds_[0]}, {"truck", thresholds_[1]},
          {"sedan", thresholds_[2]}, {"traffic_light", thresholds_[3]}}},
        {"nms_threshold", nms_}, {"undistort_enabled", undistort_},
        {"depth_fusion_enabled", bev_settings_.has_value()}, {"bev_background", bev_background_},
        {"workspace_bytes", workspace_bytes_}, {"duration_requested_seconds", measurement_seconds_}
      };
      if (bev_settings_) {
        auto & tuning = settings["bev_tuning"];
        for (const auto & control : bev_controls()) tuning[control.label] = control.read(*bev_settings_);
        settings["ground_estimation_enabled"] = bev_settings_->estimate_ground;
        const auto * pose = ground_pose_ ? &*ground_pose_ :
          (bev_settings_->estimate_ground ? nullptr : &bev_settings_->fixed_pose);
        if (pose) settings["ground_pose"] = {
          {"height_m", pose->height_m}, {"roll_deg", pose->roll_deg},
          {"pitch_down_deg", pose->pitch_down_deg}};
      }
      measurement_.emplace(Clock::now(), measurement_seconds_, std::move(settings));
      capture_measurement_.emplace(CaptureMeasurement{measurement_->start, measurement_->end, received_, 0, 0, std::nullopt});
    }
    toolbar_notice_.clear();
    RCLCPP_INFO(get_logger(), "Measuring for %d seconds; destination: %s",
      measurement_seconds_, measurement_directory_.string().c_str());
  }

  void finish_measurement(const char * reason) {
    CaptureMeasurement capture;
    Clock::time_point stop;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop = Clock::now();
      capture = *capture_measurement_;
      capture_measurement_.reset();
    }
    try {
      const auto directory = measurement_->save(measurement_directory_, stop, capture, reason);
      toolbar_notice_ = "Saved " + directory.string();
      RCLCPP_INFO(get_logger(), "Measurement saved: %s", directory.string().c_str());
    } catch (const std::exception & error) {
      toolbar_notice_ = "Measurement save failed. See console.";
      RCLCPP_ERROR(get_logger(), "Measurement save failed: %s", error.what());
    }
    measurement_.reset();
  }

  std::string toolbar_status() const {
    if (!measurement_) return toolbar_notice_;
    const int seconds = std::max(0, static_cast<int>(std::ceil(
      std::chrono::duration<double>(measurement_->end - Clock::now()).count())));
    return cv::format("Recording (%d s left)", seconds) +
      (toolbar_notice_.empty() ? "" : " | " + toolbar_notice_);
  }

  std::string apply_preview_input(const PreviewPanel::Edit & edit) {
    if (measurement_) return "Stop measurement to change settings";
    if (edit.field >= PreviewPanel::first_setting_field) return apply_bev_setting(edit);
    if (edit.field == PreviewPanel::duration_field) {
      if (!edit.value || *edit.value < 1 || *edit.value > max_measurement_seconds_ ||
          std::trunc(*edit.value) != *edit.value)
        return "Enter whole seconds from 1 to 120";
      measurement_seconds_ = static_cast<int>(*edit.value);
      return {};
    }
    if (edit.field == PreviewPanel::fps_field) return request_camera_fps(edit.value);
    if (edit.field == PreviewPanel::roi_field) {
      if (active_model_.roi80) return "448x224 uses a fixed bottom 80% ROI";
      if (!edit.value || *edit.value < 1 || *edit.value > 100 ||
          std::trunc(*edit.value) != *edit.value)
        return "Enter a whole ROI percent from 1 to 100";
      roi_height_percent_ = static_cast<int>(*edit.value);
      return {};
    }
    if (!edit.value || *edit.value < 0 || *edit.value > 1)
      return "Enter a threshold from 0 to 1";
    thresholds_[edit.field - PreviewPanel::first_threshold_field] = static_cast<float>(*edit.value);
    return {};
  }

  std::string request_camera_fps(std::optional<double> fps) {
    if (max_fps_ <= 0) return "FPS range unavailable";
    if (!fps || *fps <= 0 || *fps < min_fps_ || *fps > max_fps_)
      return cv::format("Enter %.6g to %.6g FPS", std::max(0.0, min_fps_), max_fps_);
    std::lock_guard<std::mutex> lock(mutex_);
    if (requested_camera_) return "Camera change in progress";
    if (*fps != fps_) {
      requested_camera_ = CameraChange{*fps, active_stereo_, std::nullopt};
      fps_status_ = cv::format("Changing to %.6g FPS...", *fps);
    }
    return {};
  }

  std::shared_ptr<dai::ImgFrame> read_camera_frame() {
    if (!pipeline_->isRunning()) throw std::runtime_error("OAK pipeline stopped");
    auto packet = queue_->tryGet<dai::ImgFrame>();
    if (packet && (packet->getType() != dai::ImgFrame::Type::NV12 ||
      int(packet->getWidth()) != width_ || int(packet->getHeight()) != height_))
      throw std::runtime_error("Unexpected OAK frame type or dimensions");
    return packet;
  }

  std::shared_ptr<dai::ImgFrame> read_depth_frame() {
    auto depth = depth_queue_ ? depth_queue_->tryGet<dai::ImgFrame>() : nullptr;
    if (depth && depth->getType() != dai::ImgFrame::Type::RAW16)
      throw std::runtime_error("Expected RAW16 stereo depth");
    return depth;
  }

  FirstFrames reopen_camera(double fps, const std::optional<point_cloud::Config> & stereo) {
    close_camera();
    start_camera(fps, std::make_shared<dai::Device>(dai::DeviceInfo(camera_id_), dai::UsbSpeed::SUPER), stereo);
    const auto started = Clock::now();
    FirstFrames frames;
    while (!stop_ && rclcpp::ok()) {
      if (auto rgb = read_camera_frame()) frames.rgb = std::move(rgb);
      if (auto depth = read_depth_frame()) frames.depth = std::move(depth);
      if (frames.rgb && (!stereo || frames.depth)) return frames;
      if (std::chrono::duration<double>(Clock::now() - started).count() > timeout_)
        throw std::runtime_error("No RGB/stereo frames after reconfiguration");
      std::this_thread::sleep_for(1ms);
    }
    return {};
  }

  void change_camera(CameraChange change, double previous, const std::optional<point_cloud::Config> & old_stereo) {
    std::string status;
    std::optional<oak_startup::OakStartupMeasurement> pose;
    FirstFrames frames;
    try {
      if (change.fit) {
        close_camera();
        pose = measure_ground(*change.fit);
      }
      if (stop_ || !rclcpp::ok()) return;
      frames = reopen_camera(change.fps, change.stereo);
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "Camera change failed: %s. Restoring previous stream.", error.what());
      if (stop_ || !rclcpp::ok()) return;
      pose.reset();
      status = "Failed. Previous stream restored.";
      change.fps = previous;
      change.stereo = old_stereo;
      frames = reopen_camera(previous, old_stereo);
    }
    if (!frames.rgb) return;
    std::lock_guard<std::mutex> lock(mutex_);
    fps_ = change.fps;
    active_stereo_ = std::move(change.stereo);
    active_calibration_ = stream_calibration_;
    if (pose) {
      ground_pose_ = std::move(pose);
      status = "Ground refit complete";
    }
    fps_status_ = std::move(status);
    requested_camera_.reset();
    depth_frames_.clear();
    if (frames.depth) depth_frames_.push_back(std::move(frames.depth));
    last_frame_ = Clock::now();
    latest_ = std::move(frames.rgb);
    received_ = 1;
    ++camera_session_;
  }

  void capture_loop() {
    try {
      while (!stop_ && rclcpp::ok()) {
        std::optional<CameraChange> requested;
        std::optional<point_cloud::Config> previous_stereo;
        double fps;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (requested_camera_) {
            requested = requested_camera_;
            previous_stereo = active_stereo_;
          }
          fps = fps_;
        }
        if (requested) {
          change_camera(std::move(*requested), fps, previous_stereo);
          continue;
        }
        auto packet = read_camera_frame();
        auto depth = read_depth_frame();
        if (!packet && !depth) {std::this_thread::sleep_for(100us); continue;}
        std::lock_guard<std::mutex> lock(mutex_);
        if (depth) {
          depth_frames_.push_back(std::move(depth));
          if (depth_frames_.size() > 8) depth_frames_.pop_front();
        }
        if (packet) {
          last_frame_ = Clock::now();
          if (capture_measurement_) capture_measurement_->record(last_frame_, packet->getSequenceNum());
          latest_ = std::move(packet); ++received_;
        }
      }
    } catch (const std::exception & e) {std::lock_guard<std::mutex> lock(mutex_); error_ = e.what();}
  }
  const std::array<const char *,4> names_{{"con","truck","sedan","traffic_light"}};
  std::array<float,4> thresholds_{};
  int width_, height_, roi_height_percent_;
  bool undistort_;
  double fps_, timeout_;
  static constexpr int sensor_width_ = 1280;
  static constexpr int sensor_height_ = 800;
  double min_fps_{0}, max_fps_{0};
  std::string camera_id_, format_;
  std::string backend_, startup_cache_;
  ModelSelection active_model_, startup_model_, pending_model_;
  std::string fp_model_path_, int8_model_path_, roi80_model_path_, toolbar_notice_;
  std::filesystem::path cache_directory_, measurement_directory_;
  float nms_{};
  std::size_t workspace_bytes_{};
  static constexpr int max_measurement_seconds_ = 120;
  int measurement_seconds_{15};
  bool video_enabled_{true};
  bool bev_background_{true};
  std::optional<PreviewMeasurement> measurement_;
  std::optional<CaptureMeasurement> capture_measurement_;
  std::future<std::unique_ptr<YoloxDetector>> pending_detector_;
  std::unique_ptr<YoloxDetector> detector_;
  std::unique_ptr<dai::Pipeline> pipeline_; std::shared_ptr<dai::MessageQueue> queue_;
  std::atomic<bool> stop_{false}; std::thread capture_; std::mutex mutex_;
  std::shared_ptr<dai::ImgFrame> latest_; std::uint64_t received_{0}; std::string error_;
  Clock::time_point last_frame_;
  std::optional<BevSettings> bev_settings_;
  BevFusion fusion_;
  std::optional<CameraChange> requested_camera_;
  std::optional<point_cloud::Config> active_stereo_;
  std::optional<oak_startup::OakStartupMeasurement> ground_pose_;
  std::shared_ptr<dai::MessageQueue> depth_queue_;
  std::deque<std::shared_ptr<dai::ImgFrame>> depth_frames_;
  std::shared_ptr<const dai::CalibrationHandler> stream_calibration_, active_calibration_;
  std::string fps_status_;
  std::uint64_t camera_session_{0};
};
}
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  int status = 0;
  try {auto node = std::make_shared<multiclass_yolo::Preview>(); node->run();}
  catch (const std::exception & e) {RCLCPP_ERROR(rclcpp::get_logger("multiclass_yolo"), "%s", e.what()); status=1;}
  if (rclcpp::ok()) rclcpp::shutdown();
  return status;
}
