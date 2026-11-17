#ifndef MANUAL_CAPTURE__MAIN_WINDOW_HPP_
#define MANUAL_CAPTURE__MAIN_WINDOW_HPP_

#include <QFrame>
#include <QMainWindow>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <optional>
#include <string>

#include "manual_capture/bev_projector.hpp"
#include "manual_capture/camera_engine.hpp"
#include "manual_capture/capture_logic.hpp"
#include "manual_capture/capture_writer.hpp"
#include "manual_capture/oak_startup_measurement.hpp"

class QCheckBox;
class QCloseEvent;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QRadioButton;
class QSlider;
class QSpinBox;
class QTimer;
class QWidget;

namespace manual_capture
{

class CudaCenterReprojector;

struct AppParams
{
  CameraConfig camera;
  LiveControls controls;  // flood/dot start at 0
  double dot_intensity_when_needed{1.0};
  BevGridConfig grid;
  MountPose default_pose;
  bool startup_measurement_enabled{true};
  OakStartupMeasurementConfig measurement;
  std::string capture_root;
  int jpeg_quality{95};
  int writer_queue_events{64};
  double auto_capture_fps{5.0};
  int camera_preview_fps{15};
  int bev_preview_fps{10};
  int refit_maximum_abs_erpm{100};
  double erpm_timeout_sec{1.0};
};

// Written by the ROS thread, read by the GUI.
struct VehicleTelemetry
{
  std::atomic<bool> has_erpm{false};
  std::atomic<int> erpm{0};
  std::atomic<double> erpm_time_sec{0.0};
  std::atomic<bool> has_servo{false};
  std::atomic<double> servo{0.0};
  std::atomic<bool> joy_connected{false};
};

double steadyNowSec();

class ViewPane : public QFrame
{
  Q_OBJECT

public:
  ViewPane(View view, const QString & title, bool compact, int preview_fps, QWidget * parent);

  View view() const {return view_;}
  int previewFps() const;
  void setState(bool previewing, bool locked, const QString & lock_reason);
  void setCaptureEnabled(bool enabled);
  void setCount(std::uint64_t count);
  void setFooter(const QString & text);
  void showImage(const cv::Mat & image, bool draw_center_line);
  void showMessage(const QString & text);

  double next_preview_sec{0.0};

signals:
  void previewToggled(View view, bool enabled);
  void singleCaptureRequested(View view);

private:
  View view_;
  QLabel * title_;
  QLabel * lock_tag_;
  QLabel * count_;
  QPushButton * preview_button_;
  QPushButton * capture_button_;
  QSpinBox * fps_;
  QLabel * image_;
  QLabel * footer_;
  bool was_previewing_{false};
};

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(AppParams params, std::shared_ptr<VehicleTelemetry> telemetry);
  ~MainWindow() override;

  // Called on the GUI thread for each controller A rising edge.
  void onJoyCapture();

protected:
  void closeEvent(QCloseEvent * event) override;

private:
  struct JobResult
  {
    bool ok{true};
    QString message;
    std::optional<MountPose> pose;
    QString pose_label;
  };
  enum class JobKind {STARTUP, RESTART, REFIT};

  // UI construction
  QWidget * buildTopBar();
  QWidget * buildViews();
  QWidget * buildSidePanel();
  QWidget * buildTargetBox();
  QWidget * buildRgbTab();
  QWidget * buildStereoTab();
  QWidget * buildBevTab();
  QWidget * buildSaveTab();
  QWidget * buildStatusBar();

  // Engine jobs run off the GUI thread; the GUI polls for completion.
  void runJob(JobKind kind, std::function<JobResult()> work, const QString & busy_text);
  void finishJob();
  JobResult measureAndStart(const CameraConfig & config, bool measure);
  void requestRestart(const CameraConfig & config);
  void requestRefit();

  // Periodic work
  void tick();
  void statusTick();
  void refreshControls();
  void refreshPanes();
  void updateDotProjector();
  void ensureBevProjectors();

  // Capture
  enum class Trigger {SINGLE, AUTO, VIEW};
  void handleAction(CaptureAction action);
  void captureViews(const std::vector<View> & views, Trigger trigger);
  bool ensureWriter();
  std::string metaRow(
    std::uint64_t event, Trigger trigger, const std::vector<View> & views,
    const std::shared_ptr<const RgbFrame> & rgb,
    const std::shared_ptr<const StereoFrame> & stereo) const;
  std::map<View, cv::Mat> computeViews(
    const std::vector<View> & views,
    const std::shared_ptr<const RgbFrame> & rgb,
    const std::shared_ptr<const StereoFrame> & stereo);

  // Live controls
  void applyRgbExposure();
  void applyStereoExposure(bool left_changed);
  void updateExposureRanges();
  CameraConfig pendingRgbConfig() const;
  CameraConfig pendingStereoConfig() const;

  AppParams params_;
  std::shared_ptr<VehicleTelemetry> telemetry_;
  CameraEngine engine_;
  CameraConfig applied_config_;
  ViewSelection selection_;
  CaptureController controller_;
  IntervalScheduler auto_scheduler_;
  std::unique_ptr<CaptureWriter> writer_;
  std::uint64_t next_event_index_{1U};
  double recording_started_sec_{0.0};

  std::unique_ptr<CudaCenterReprojector> reprojector_;
  BevProjector rgb_bev_;
  BevProjector left_bev_;
  std::uint64_t bev_generation_{0U};
  MountPose pose_;
  QString pose_label_;

  bool job_running_{false};
  JobKind job_kind_{JobKind::STARTUP};
  bool resume_auto_after_job_{false};
  std::future<JobResult> job_future_;

  QTimer * tick_timer_{nullptr};
  QTimer * status_timer_{nullptr};
  std::array<ViewPane *, kViewCount> panes_{};
  std::array<QCheckBox *, kViewCount> target_checks_{};
  std::array<QLabel *, kViewCount> target_states_{};
  std::array<QLabel *, kViewCount> target_counts_{};

  // top bar
  QLabel * device_label_{nullptr};
  QLabel * mode_label_{nullptr};
  QLabel * drive_label_{nullptr};
  QRadioButton * single_radio_{nullptr};
  QRadioButton * auto_radio_{nullptr};
  QDoubleSpinBox * capture_fps_{nullptr};
  QPushButton * start_button_{nullptr};
  QPushButton * stop_button_{nullptr};
  QWidget * body_{nullptr};

  // RGB tab
  QComboBox * rgb_resolution_{nullptr};
  QDoubleSpinBox * rgb_fps_{nullptr};
  QComboBox * rgb_format_{nullptr};
  QCheckBox * rgb_auto_exposure_{nullptr};
  QSpinBox * rgb_exposure_{nullptr};
  QSlider * rgb_exposure_slider_{nullptr};
  QSpinBox * rgb_iso_{nullptr};
  QSlider * rgb_iso_slider_{nullptr};
  QLabel * rgb_actual_{nullptr};

  // Stereo tab
  QComboBox * stereo_resolution_{nullptr};
  QDoubleSpinBox * stereo_fps_{nullptr};
  QCheckBox * stereo_auto_exposure_{nullptr};
  QCheckBox * stereo_link_{nullptr};
  QSpinBox * left_exposure_{nullptr};
  QSlider * left_exposure_slider_{nullptr};
  QSpinBox * left_iso_{nullptr};
  QSlider * left_iso_slider_{nullptr};
  QSpinBox * right_exposure_{nullptr};
  QSlider * right_exposure_slider_{nullptr};
  QSpinBox * right_iso_{nullptr};
  QSlider * right_iso_slider_{nullptr};
  QSpinBox * flood_{nullptr};
  QSlider * flood_slider_{nullptr};
  QSpinBox * dot_{nullptr};
  QSlider * dot_slider_{nullptr};
  QLabel * dot_state_{nullptr};
  QLabel * stereo_actual_{nullptr};

  // BEV tab
  QPushButton * left_bev_mode_{nullptr};
  QLabel * pose_height_{nullptr};
  QLabel * pose_pitch_{nullptr};
  QLabel * pose_roll_{nullptr};
  QLabel * pose_source_{nullptr};
  QPushButton * refit_button_{nullptr};
  QLabel * refit_message_{nullptr};

  // Save tab
  QSpinBox * jpeg_quality_{nullptr};
  QLabel * session_label_{nullptr};

  // status bar
  QLabel * rec_label_{nullptr};
  QLabel * total_label_{nullptr};
  QLabel * queue_label_{nullptr};
  QLabel * disk_label_{nullptr};
  QLabel * fps_label_{nullptr};
  QLabel * message_label_{nullptr};

  bool updating_ui_{false};
};

}  // namespace manual_capture

#endif  // MANUAL_CAPTURE__MAIN_WINDOW_HPP_
