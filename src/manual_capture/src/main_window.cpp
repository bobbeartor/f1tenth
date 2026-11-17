#include "manual_capture/main_window.hpp"

#include <QButtonGroup>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QLocale>
#include <QPixmap>
#include <QPushButton>
#include <QRadioButton>
#include <QSlider>
#include <QSpinBox>
#include <QStyle>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>

#include "manual_capture/cuda_center_reprojector.hpp"
#include "opencv2/imgproc.hpp"

namespace manual_capture
{

namespace
{

const char * kStyle = R"(
QWidget { background:#1e2127; color:#d7dae0; font-size:12px; }
QGroupBox { border:1px solid #3a3f4b; border-radius:6px; margin-top:14px; padding:8px 6px 6px 6px; }
QGroupBox::title { subcontrol-origin: margin; left:8px; padding:0 4px; color:#8fb3ff; font-weight:bold; }
QPushButton { background:#2d323c; border:1px solid #454b58; border-radius:5px; padding:5px 10px; }
QPushButton#start { background:#1f7a3f; border-color:#2a9d52; font-weight:bold; font-size:14px; }
QPushButton#stop  { background:#8a2b2b; border-color:#b03a3a; font-weight:bold; font-size:14px; }
QPushButton#refit { background:#7a5a12; border-color:#a77d1d; font-weight:bold; font-size:13px; padding:8px; }
QPushButton#apply { background:#24476e; border-color:#3466a0; }
QPushButton#one { background:#2b3a2f; border-color:#3f6b4a; }
QPushButton:checked { background:#2f6fd0; border-color:#4b8af0; color:white; }
QPushButton:disabled { color:#5c626e; background:#25282e; border-color:#30343c; }
QTabWidget::pane { border:1px solid #3a3f4b; border-radius:4px; }
QTabBar::tab { background:#2a2e36; padding:6px 12px; border-top-left-radius:4px; border-top-right-radius:4px; }
QTabBar::tab:selected { background:#3a4150; color:white; }
QFrame#pane { background:#15171b; border:1px solid #343944; border-radius:6px; }
QFrame#paneLocked { background:#15171b; border:1px dashed #6b5a2a; border-radius:6px; }
QFrame#bar { background:#252932; border-radius:6px; }
QLabel#paneTitle { font-weight:bold; font-size:13px; color:white; background:transparent; }
QLabel#tag { background:#3b2f12; color:#f0c35a; border-radius:3px; padding:1px 5px; font-size:11px; }
QLabel#count { color:#9ecbff; background:#1d2a3d; border-radius:3px; padding:1px 6px; font-weight:bold; }
QLabel#ok  { color:#62d48a; background:transparent; }
QLabel#warn{ color:#f0c35a; background:transparent; }
QLabel#err { color:#ff6b6b; background:transparent; }
QLabel#dim { color:#7d8390; background:transparent; }
QLabel#image { background:#0b0c0e; border-radius:4px; color:#6e737d; font-size:15px; }
QSpinBox, QDoubleSpinBox, QComboBox { background:#2a2e36; border:1px solid #454b58; border-radius:4px; padding:2px 4px; }
)";

QString viewTitle(const View view)
{
  switch (view) {
    case View::RGB: return QStringLiteral("① 컬러 (CAM_A)");
    case View::CENTER: return QStringLiteral("② 중앙정렬 (L/R 가상 중앙)");
    case View::LEFT: return QStringLiteral("③ 스테레오 왼쪽 (CAM_B)");
    case View::RIGHT: return QStringLiteral("④ 스테레오 오른쪽 (CAM_C)");
    case View::RGB_BEV: return QStringLiteral("⑤ 컬러 BEV");
    case View::LEFT_BEV: return QStringLiteral("⑥ 왼쪽 렌즈 BEV");
  }
  return {};
}

QString shortTitle(const View view)
{
  switch (view) {
    case View::RGB: return QStringLiteral("① 컬러");
    case View::CENTER: return QStringLiteral("② 중앙정렬");
    case View::LEFT: return QStringLiteral("③ 스테레오 왼쪽");
    case View::RIGHT: return QStringLiteral("④ 스테레오 오른쪽");
    case View::RGB_BEV: return QStringLiteral("⑤ 컬러 BEV");
    case View::LEFT_BEV: return QStringLiteral("⑥ 왼쪽 렌즈 BEV");
  }
  return {};
}

QString lockReason(const View view)
{
  switch (view) {
    case View::RIGHT:
      return QStringLiteral("왼쪽 렌즈 BEV 모드에서는\n오른쪽 렌즈가 잠깁니다");
    case View::CENTER:
      return QStringLiteral("오른쪽 렌즈가 필요해서\n왼쪽 렌즈 BEV 모드에서는 잠깁니다");
    case View::LEFT_BEV:
      return QStringLiteral("왼쪽 렌즈 BEV 모드 OFF\n(BEV / Refit 탭에서 켜기)");
    default:
      return {};
  }
}

bool isBev(const View view) {return view == View::RGB_BEV || view == View::LEFT_BEV;}

QLabel * styled(const QString & text, const char * name)
{
  auto * label = new QLabel(text);
  label->setObjectName(name);
  return label;
}

// Slider + spin box pair kept in sync.
QWidget * sliderRow(int minimum, int maximum, int value, const QString & suffix,
  QSlider ** slider_out, QSpinBox ** spin_out)
{
  auto * row = new QWidget;
  auto * layout = new QHBoxLayout(row);
  layout->setContentsMargins(0, 0, 0, 0);
  auto * slider = new QSlider(Qt::Horizontal);
  slider->setRange(minimum, maximum);
  slider->setValue(value);
  auto * spin = new QSpinBox;
  spin->setRange(minimum, maximum);
  spin->setValue(value);
  spin->setSuffix(suffix);
  spin->setFixedWidth(96);
  QObject::connect(slider, &QSlider::valueChanged, spin, &QSpinBox::setValue);
  QObject::connect(spin, QOverload<int>::of(&QSpinBox::valueChanged), slider, &QSlider::setValue);
  layout->addWidget(slider, 1);
  layout->addWidget(spin);
  *slider_out = slider;
  *spin_out = spin;
  return row;
}

void setRange(QSlider * slider, QSpinBox * spin, int maximum)
{
  slider->setMaximum(maximum);
  spin->setMaximum(maximum);
}

QComboBox * resolutionCombo(const std::vector<std::pair<int, int>> & sizes, int width, int height)
{
  auto * combo = new QComboBox;
  for (const auto & [w, h] : sizes) {
    combo->addItem(QStringLiteral("%1×%2").arg(w).arg(h), QSize(w, h));
    if (w == width && h == height) {
      combo->setCurrentIndex(combo->count() - 1);
    }
  }
  return combo;
}

std::string localTimeString(std::time_t time)
{
  std::tm local{};
  localtime_r(&time, &local);
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &local);
  return buffer;
}

}  // namespace

double steadyNowSec()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

// ---------------------------------------------------------------- ViewPane

ViewPane::ViewPane(
  const View view, const QString & title, const bool compact, const int preview_fps,
  QWidget * parent)
: QFrame(parent), view_(view)
{
  setObjectName("pane");
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(6, 4, 6, 6);
  layout->setSpacing(4);

  title_ = styled(title, "paneTitle");
  lock_tag_ = styled(QStringLiteral("🔒 잠김"), "tag");
  count_ = styled(QStringLiteral("📷 0장"), "count");
  preview_button_ = new QPushButton(QStringLiteral("프리뷰"));
  preview_button_->setCheckable(true);
  capture_button_ = new QPushButton(compact ? QStringLiteral("캡처") : QStringLiteral("이 화면만 캡처"));
  capture_button_->setObjectName("one");
  capture_button_->setToolTip(QStringLiteral("이 화면 1장만 저장 (자동 캡처 중에는 불가)"));
  fps_ = new QSpinBox;
  fps_->setRange(1, 30);
  fps_->setValue(preview_fps);
  fps_->setToolTip(QStringLiteral("프리뷰 FPS"));

  auto * head = new QHBoxLayout;
  head->addWidget(title_);
  head->addWidget(lock_tag_);
  head->addStretch(1);
  head->addWidget(count_);
  if (compact) {
    layout->addLayout(head);
    auto * row = new QHBoxLayout;
    row->addWidget(preview_button_);
    row->addWidget(capture_button_);
    row->addWidget(new QLabel(QStringLiteral("FPS")));
    row->addWidget(fps_);
    layout->addLayout(row);
  } else {
    head->addWidget(preview_button_);
    head->addWidget(capture_button_);
    head->addWidget(new QLabel(QStringLiteral("프리뷰 FPS")));
    head->addWidget(fps_);
    layout->addLayout(head);
  }

  image_ = new QLabel;
  image_->setObjectName("image");
  image_->setAlignment(Qt::AlignCenter);
  image_->setMinimumSize(80, 80);
  image_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  layout->addWidget(image_, 1);
  footer_ = styled(QString(), "dim");
  layout->addWidget(footer_);

  connect(preview_button_, &QPushButton::toggled, this, [this](bool enabled) {
      emit previewToggled(view_, enabled);
    });
  connect(capture_button_, &QPushButton::clicked, this, [this]() {
      emit singleCaptureRequested(view_);
    });
}

int ViewPane::previewFps() const {return fps_->value();}

void ViewPane::setState(const bool previewing, const bool locked, const QString & lock_reason)
{
  const QString name = locked ? "paneLocked" : "pane";
  if (objectName() != name) {
    setObjectName(name);
    style()->unpolish(this);
    style()->polish(this);
  }
  lock_tag_->setVisible(locked);
  preview_button_->blockSignals(true);
  preview_button_->setChecked(previewing);
  preview_button_->blockSignals(false);
  preview_button_->setEnabled(!locked);
  fps_->setEnabled(!locked);
  if (locked) {
    showMessage(lock_reason);
    setFooter(QStringLiteral("스트림 중지 · 저장 안 함"));
  } else if (!previewing) {
    showMessage(QStringLiteral("프리뷰 꺼짐"));
    setFooter(QStringLiteral("프리뷰 꺼짐 · 캡처 대상 아님"));
  } else if (!was_previewing_) {
    showMessage(QStringLiteral("프레임 대기 중..."));
    setFooter(QString());
  }
  was_previewing_ = previewing && !locked;
}

void ViewPane::setCaptureEnabled(const bool enabled) {capture_button_->setEnabled(enabled);}

void ViewPane::setCount(const std::uint64_t count)
{
  count_->setText(QStringLiteral("📷 %1장").arg(QLocale().toString(static_cast<qulonglong>(count))));
}

void ViewPane::setFooter(const QString & text) {footer_->setText(text);}

void ViewPane::showMessage(const QString & text)
{
  image_->setPixmap(QPixmap());
  image_->setText(text);
}

void ViewPane::showImage(const cv::Mat & image, const bool draw_center_line)
{
  if (image.empty()) {
    return;
  }
  const QSize target = image_->size();
  const double scale = std::min(
    static_cast<double>(target.width()) / image.cols,
    static_cast<double>(target.height()) / image.rows);
  if (!(scale > 0.0)) {
    return;
  }
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(
      std::max(1, static_cast<int>(image.cols * scale)),
      std::max(1, static_cast<int>(image.rows * scale))),
    0.0, 0.0, scale < 1.0 ? cv::INTER_AREA : cv::INTER_NEAREST);
  cv::Mat rgb;
  if (resized.channels() == 1) {
    cv::cvtColor(resized, rgb, cv::COLOR_GRAY2RGB);
  } else {
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
  }
  if (draw_center_line) {
    // Vehicle center (y=0) guide, preview only; never saved.
    const int x = rgb.cols / 2;
    for (int y = 0; y < rgb.rows; y += 8) {
      cv::line(rgb, {x, y}, {x, std::min(rgb.rows - 1, y + 4)}, cv::Scalar(80, 200, 255), 1);
    }
  }
  const QImage view(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step), QImage::Format_RGB888);
  image_->setText(QString());
  image_->setPixmap(QPixmap::fromImage(view.copy()));
}

// ---------------------------------------------------------------- MainWindow

MainWindow::MainWindow(AppParams params, std::shared_ptr<VehicleTelemetry> telemetry)
: params_(std::move(params)),
  telemetry_(std::move(telemetry)),
  applied_config_(params_.camera),
  selection_(params_.camera.stereo_mode),
  rgb_bev_(params_.grid),
  left_bev_(params_.grid),
  pose_(params_.default_pose),
  pose_label_(QStringLiteral("기본값 (미측정)"))
{
  setWindowTitle(QStringLiteral("Manual Capture"));
  setStyleSheet(kStyle);
  // Initial launch: only the color preview, IR fully off.
  params_.controls.flood = 0.0;
  params_.controls.dot = 0.0;
  if (selection_.mode() == StereoMode::LEFT_ONLY) {
    selection_.setPreview(View::LEFT_BEV, false);
  }

  auto * root = new QWidget;
  auto * outer = new QVBoxLayout(root);
  outer->setContentsMargins(8, 8, 8, 8);
  outer->setSpacing(6);
  outer->addWidget(buildTopBar());
  body_ = new QWidget;
  auto * body = new QHBoxLayout(body_);
  body->setContentsMargins(0, 0, 0, 0);
  body->setSpacing(6);
  body->addWidget(buildViews(), 1);
  body->addWidget(buildSidePanel());
  outer->addWidget(body_, 1);
  outer->addWidget(buildStatusBar());
  setCentralWidget(root);
  resize(1920, 1040);

  try {
    reprojector_ = std::make_unique<CudaCenterReprojector>();
  } catch (const std::exception & exception) {
    message_label_->setText(QStringLiteral("CUDA 중앙정렬 초기화 실패: %1").arg(exception.what()));
  }

  tick_timer_ = new QTimer(this);
  connect(tick_timer_, &QTimer::timeout, this, &MainWindow::tick);
  tick_timer_->start(5);
  status_timer_ = new QTimer(this);
  connect(status_timer_, &QTimer::timeout, this, &MainWindow::statusTick);
  status_timer_->start(250);

  refreshControls();
  const bool measure = params_.startup_measurement_enabled;
  const CameraConfig config = params_.camera;
  runJob(JobKind::STARTUP, [this, config, measure]() {return measureAndStart(config, measure);},
    measure ? QStringLiteral("초기 지면 측정 중... 차량을 평지에 정지시켜 두세요") :
    QStringLiteral("카메라 시작 중..."));
}

MainWindow::~MainWindow()
{
  if (job_future_.valid()) {
    job_future_.wait();
  }
  engine_.stop();
  writer_.reset();  // flushes queued images
}

void MainWindow::closeEvent(QCloseEvent * event)
{
  controller_.stop();
  if (writer_) {
    message_label_->setText(QStringLiteral("남은 이미지 저장 중..."));
    writer_->flush();
  }
  QMainWindow::closeEvent(event);
}

QWidget * MainWindow::buildTopBar()
{
  auto * bar = new QFrame;
  bar->setObjectName("bar");
  auto * layout = new QHBoxLayout(bar);
  layout->setContentsMargins(10, 6, 10, 6);
  auto * title = new QLabel(QStringLiteral("Manual Capture"));
  title->setStyleSheet("font-size:16px; font-weight:bold; color:white; background:transparent;");
  layout->addWidget(title);
  device_label_ = styled(QStringLiteral("● OAK 연결 대기"), "warn");
  mode_label_ = styled(QString(), "dim");
  drive_label_ = styled(QString(), "dim");
  layout->addWidget(device_label_);
  layout->addWidget(mode_label_);
  layout->addWidget(drive_label_);
  layout->addStretch(1);

  single_radio_ = new QRadioButton(QStringLiteral("단일 캡처"));
  auto_radio_ = new QRadioButton(QStringLiteral("자동 캡처"));
  single_radio_->setChecked(true);
  auto * group = new QButtonGroup(bar);
  group->addButton(single_radio_);
  group->addButton(auto_radio_);
  layout->addWidget(single_radio_);
  layout->addWidget(auto_radio_);
  layout->addWidget(new QLabel(QStringLiteral("캡처 FPS")));
  capture_fps_ = new QDoubleSpinBox;
  capture_fps_->setRange(0.5, 30.0);
  capture_fps_->setDecimals(1);
  capture_fps_->setSingleStep(0.5);
  capture_fps_->setValue(params_.auto_capture_fps);
  capture_fps_->setSuffix(" fps");
  layout->addWidget(capture_fps_);
  start_button_ = new QPushButton(QStringLiteral("●  캡처 시작"));
  start_button_->setObjectName("start");
  stop_button_ = new QPushButton(QStringLiteral("■  캡처 중단"));
  stop_button_->setObjectName("stop");
  layout->addWidget(start_button_);
  layout->addWidget(stop_button_);
  layout->addWidget(styled(QStringLiteral("🎮 A = 단일: 1회 캡처 / 자동: 시작·중단"), "dim"));

  connect(single_radio_, &QRadioButton::toggled, this, [this](bool on) {
      if (on) {controller_.setMode(CaptureMode::SINGLE);}
      refreshControls();
    });
  connect(auto_radio_, &QRadioButton::toggled, this, [this](bool on) {
      if (on) {controller_.setMode(CaptureMode::AUTO);}
      refreshControls();
    });
  connect(capture_fps_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
    [this](double fps) {auto_scheduler_.reset(steadyNowSec(), fps);});
  connect(start_button_, &QPushButton::clicked, this, [this]() {
      if (!job_running_) {handleAction(controller_.start());}
    });
  connect(stop_button_, &QPushButton::clicked, this, [this]() {
      handleAction(controller_.stop());
    });
  return bar;
}

QWidget * MainWindow::buildViews()
{
  auto * area = new QWidget;
  auto * layout = new QHBoxLayout(area);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(6);

  auto * grid = new QGridLayout;
  grid->setSpacing(6);
  // Top: color, center. Bottom: stereo left, stereo right.
  const std::array<std::pair<View, std::pair<int, int>>, 4> positions{{
    {View::RGB, {0, 0}}, {View::CENTER, {0, 1}}, {View::LEFT, {1, 0}}, {View::RIGHT, {1, 1}}}};
  for (const auto & [view, cell] : positions) {
    auto * pane = new ViewPane(view, viewTitle(view), false, params_.camera_preview_fps, area);
    panes_[index(view)] = pane;
    grid->addWidget(pane, cell.first, cell.second);
  }
  grid->setRowStretch(0, 1);
  grid->setRowStretch(1, 1);
  grid->setColumnStretch(0, 1);
  grid->setColumnStretch(1, 1);
  auto * grid_widget = new QWidget;
  grid_widget->setLayout(grid);
  layout->addWidget(grid_widget, 1);

  auto * column = new QVBoxLayout;
  column->setSpacing(6);
  for (const auto view : {View::RGB_BEV, View::LEFT_BEV}) {
    auto * pane = new ViewPane(view, viewTitle(view), true, params_.bev_preview_fps, area);
    panes_[index(view)] = pane;
    column->addWidget(pane, 1);
  }
  auto * column_widget = new QWidget;
  column_widget->setLayout(column);
  column_widget->setFixedWidth(250);
  layout->addWidget(column_widget);

  for (auto * pane : panes_) {
    connect(pane, &ViewPane::previewToggled, this, [this](View view, bool enabled) {
        selection_.setPreview(view, enabled);
        panes_[index(view)]->next_preview_sec = 0.0;
        updateDotProjector();
        refreshControls();
      });
    connect(pane, &ViewPane::singleCaptureRequested, this, [this](View view) {
        if (!job_running_ && controller_.singleViewCaptureAllowed() && selection_.available(view)) {
          captureViews({view}, Trigger::VIEW);
        }
      });
  }
  return area;
}

QWidget * MainWindow::buildSidePanel()
{
  auto * panel = new QWidget;
  auto * layout = new QVBoxLayout(panel);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(6);
  layout->addWidget(buildTargetBox());
  auto * tabs = new QTabWidget;
  tabs->addTab(buildRgbTab(), QStringLiteral("컬러"));
  tabs->addTab(buildStereoTab(), QStringLiteral("스테레오"));
  tabs->addTab(buildBevTab(), QStringLiteral("BEV / Refit"));
  tabs->addTab(buildSaveTab(), QStringLiteral("저장"));
  layout->addWidget(tabs, 1);
  panel->setFixedWidth(430);
  return panel;
}

QWidget * MainWindow::buildTargetBox()
{
  auto * box = new QGroupBox(QStringLiteral("동시 캡처 대상  (🎮 A / 캡처 시작)"));
  auto * grid = new QGridLayout(box);
  grid->setVerticalSpacing(4);
  grid->addWidget(styled(QStringLiteral("화면"), "dim"), 0, 0);
  grid->addWidget(styled(QStringLiteral("상태"), "dim"), 0, 1);
  grid->addWidget(styled(QStringLiteral("누적"), "dim"), 0, 2);
  int row = 1;
  for (const auto view : kAllViews) {
    auto * check = new QCheckBox(shortTitle(view));
    target_checks_[index(view)] = check;
    target_states_[index(view)] = styled(QString(), "dim");
    target_counts_[index(view)] = styled(QStringLiteral("0장"), "count");
    grid->addWidget(check, row, 0);
    grid->addWidget(target_states_[index(view)], row, 1);
    grid->addWidget(target_counts_[index(view)], row, 2);
    connect(check, &QCheckBox::toggled, this, [this, view](bool checked) {
        if (!updating_ui_) {
          selection_.setChecked(view, checked);
          refreshControls();
        }
      });
    ++row;
  }
  grid->addWidget(
    styled(QStringLiteral("프리뷰가 켜진 화면만 체크 가능 · 프리뷰를 켜면 자동 체크"), "dim"),
    row, 0, 1, 3);
  return box;
}

QWidget * MainWindow::buildRgbTab()
{
  auto * tab = new QWidget;
  auto * layout = new QVBoxLayout(tab);
  auto * restart = new QGroupBox(QStringLiteral("컬러 (CAM_A)  ⟳ 재시작 필요"));
  auto * form = new QFormLayout(restart);
  rgb_resolution_ = resolutionCombo(
    {{1280, 800}, {1280, 720}, {960, 600}, {640, 400}},
    params_.camera.rgb_width, params_.camera.rgb_height);
  rgb_fps_ = new QDoubleSpinBox;
  rgb_fps_->setRange(1.0, 60.0);
  rgb_fps_->setDecimals(1);
  rgb_fps_->setValue(params_.camera.rgb_fps);
  rgb_fps_->setSuffix(" fps");
  rgb_format_ = new QComboBox;
  rgb_format_->addItem("NV12");
  rgb_format_->addItem("RGB888");
  rgb_format_->setCurrentIndex(params_.camera.rgb_format == RgbFormat::NV12 ? 0 : 1);
  form->addRow(QStringLiteral("해상도"), rgb_resolution_);
  form->addRow(QStringLiteral("센서 FPS"), rgb_fps_);
  form->addRow(QStringLiteral("포맷"), rgb_format_);
  auto * apply = new QPushButton(QStringLiteral("적용 (파이프라인 재시작 1~3초)"));
  apply->setObjectName("apply");
  form->addRow(apply);
  layout->addWidget(restart);

  auto * live = new QGroupBox(QStringLiteral("실시간 조절"));
  auto * live_form = new QFormLayout(live);
  rgb_auto_exposure_ = new QCheckBox(QStringLiteral("자동 노출"));
  rgb_auto_exposure_->setChecked(params_.controls.rgb.automatic);
  live_form->addRow(rgb_auto_exposure_);
  live_form->addRow(QStringLiteral("노출"), sliderRow(
      20, maximumExposureUs(params_.camera.rgb_fps), params_.controls.rgb.exposure_us,
      QStringLiteral(" µs"), &rgb_exposure_slider_, &rgb_exposure_));
  live_form->addRow(QStringLiteral("ISO"), sliderRow(
      100, 1600, params_.controls.rgb.iso, QString(), &rgb_iso_slider_, &rgb_iso_));
  live_form->addRow(styled(QStringLiteral("노출 상한 = 1/센서FPS"), "dim"));
  rgb_actual_ = styled(QString(), "dim");
  live_form->addRow(rgb_actual_);
  layout->addWidget(live);
  layout->addStretch(1);

  connect(apply, &QPushButton::clicked, this, [this]() {requestRestart(pendingRgbConfig());});
  connect(rgb_auto_exposure_, &QCheckBox::toggled, this, [this]() {applyRgbExposure();});
  connect(rgb_exposure_, QOverload<int>::of(&QSpinBox::valueChanged), this,
    [this]() {applyRgbExposure();});
  connect(rgb_iso_, QOverload<int>::of(&QSpinBox::valueChanged), this,
    [this]() {applyRgbExposure();});
  return tab;
}

QWidget * MainWindow::buildStereoTab()
{
  auto * tab = new QWidget;
  auto * layout = new QVBoxLayout(tab);
  auto * restart = new QGroupBox(QStringLiteral("스테레오 (CAM_B/C)  ⟳ 재시작 필요"));
  auto * form = new QFormLayout(restart);
  stereo_resolution_ = resolutionCombo(
    {{1280, 800}, {640, 400}}, params_.camera.stereo_width, params_.camera.stereo_height);
  stereo_fps_ = new QDoubleSpinBox;
  stereo_fps_->setRange(1.0, 50.0);
  stereo_fps_->setDecimals(1);
  stereo_fps_->setValue(params_.camera.stereo_fps);
  stereo_fps_->setSuffix(" fps");
  form->addRow(QStringLiteral("해상도"), stereo_resolution_);
  form->addRow(QStringLiteral("센서 FPS"), stereo_fps_);
  auto * apply = new QPushButton(QStringLiteral("적용 (파이프라인 재시작 1~3초)"));
  apply->setObjectName("apply");
  form->addRow(apply);
  layout->addWidget(restart);

  auto * exposure = new QGroupBox(QStringLiteral("노출 / ISO  (실시간)"));
  auto * exposure_form = new QFormLayout(exposure);
  stereo_auto_exposure_ = new QCheckBox(QStringLiteral("자동 노출"));
  stereo_auto_exposure_->setChecked(params_.controls.left.automatic);
  stereo_link_ = new QCheckBox(QStringLiteral("왼쪽·오른쪽 동일하게 (연동)"));
  stereo_link_->setChecked(true);
  exposure_form->addRow(stereo_auto_exposure_);
  exposure_form->addRow(stereo_link_);
  const int max_exposure = maximumExposureUs(params_.camera.stereo_fps);
  exposure_form->addRow(QStringLiteral("L 노출"), sliderRow(
      20, max_exposure, params_.controls.left.exposure_us, QStringLiteral(" µs"),
      &left_exposure_slider_, &left_exposure_));
  exposure_form->addRow(QStringLiteral("L ISO"), sliderRow(
      100, 1600, params_.controls.left.iso, QString(), &left_iso_slider_, &left_iso_));
  exposure_form->addRow(QStringLiteral("R 노출"), sliderRow(
      20, max_exposure, params_.controls.right.exposure_us, QStringLiteral(" µs"),
      &right_exposure_slider_, &right_exposure_));
  exposure_form->addRow(QStringLiteral("R ISO"), sliderRow(
      100, 1600, params_.controls.right.iso, QString(), &right_iso_slider_, &right_iso_));
  stereo_actual_ = styled(QString(), "dim");
  exposure_form->addRow(stereo_actual_);
  layout->addWidget(exposure);

  auto * ir = new QGroupBox(QStringLiteral("IR 조명  (실시간)"));
  auto * ir_form = new QFormLayout(ir);
  ir_form->addRow(QStringLiteral("Flood 세기"), sliderRow(
      0, 100, 0, QStringLiteral(" %"), &flood_slider_, &flood_));
  auto * dot_note = styled(
    QStringLiteral("도트 프로젝터: 자동 (중앙정렬 프리뷰가 켜졌을 때 / Refit 중에만 ON)"), "warn");
  dot_note->setWordWrap(true);
  ir_form->addRow(dot_note);
  ir_form->addRow(QStringLiteral("도트 세기(ON일 때)"), sliderRow(
      0, 100, static_cast<int>(std::lround(params_.dot_intensity_when_needed * 100.0)),
      QStringLiteral(" %"), &dot_slider_, &dot_));
  dot_state_ = styled(QStringLiteral("현재 상태: 꺼짐"), "dim");
  ir_form->addRow(dot_state_);
  layout->addWidget(ir);
  layout->addStretch(1);

  connect(apply, &QPushButton::clicked, this, [this]() {requestRestart(pendingStereoConfig());});
  connect(stereo_auto_exposure_, &QCheckBox::toggled, this, [this]() {
      applyStereoExposure(true);
      refreshControls();
    });
  connect(stereo_link_, &QCheckBox::toggled, this, [this]() {
      applyStereoExposure(true);
      refreshControls();
    });
  for (auto * spin : {left_exposure_, left_iso_}) {
    connect(spin, QOverload<int>::of(&QSpinBox::valueChanged), this,
      [this]() {applyStereoExposure(true);});
  }
  for (auto * spin : {right_exposure_, right_iso_}) {
    connect(spin, QOverload<int>::of(&QSpinBox::valueChanged), this,
      [this]() {applyStereoExposure(false);});
  }
  connect(flood_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int percent) {
      engine_.setFlood(percent / 100.0);
    });
  connect(dot_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int percent) {
      params_.dot_intensity_when_needed = percent / 100.0;
      updateDotProjector();
    });
  return tab;
}

QWidget * MainWindow::buildBevTab()
{
  auto * tab = new QWidget;
  auto * layout = new QVBoxLayout(tab);
  auto * mode = new QGroupBox(QStringLiteral("스테레오 BEV 모드  ⟳ 재시작 필요"));
  auto * mode_layout = new QVBoxLayout(mode);
  left_bev_mode_ = new QPushButton;
  left_bev_mode_->setCheckable(true);
  mode_layout->addWidget(left_bev_mode_);
  mode_layout->addWidget(styled(
      QStringLiteral("ON이면 왼쪽 렌즈 단독 동작 (차량 중심 정렬 BEV).\n"
      "오른쪽 렌즈·중앙정렬은 잠김."), "dim"));
  layout->addWidget(mode);

  auto * pose = new QGroupBox(QStringLiteral("지면 자세 (고정)"));
  auto * pose_form = new QFormLayout(pose);
  pose_height_ = styled(QString(), "ok");
  pose_pitch_ = styled(QString(), "ok");
  pose_roll_ = styled(QString(), "ok");
  pose_source_ = styled(QString(), "dim");
  pose_form->addRow(QStringLiteral("카메라 높이"), pose_height_);
  pose_form->addRow(QStringLiteral("Pitch (아래)"), pose_pitch_);
  pose_form->addRow(QStringLiteral("Roll"), pose_roll_);
  pose_form->addRow(QStringLiteral("측정"), pose_source_);
  layout->addWidget(pose);

  auto * refit = new QGroupBox(QStringLiteral("지면 재추정"));
  auto * refit_layout = new QVBoxLayout(refit);
  refit_button_ = new QPushButton(QStringLiteral("⟲  Refit (지면 재추정)"));
  refit_button_->setObjectName("refit");
  refit_layout->addWidget(refit_button_);
  refit_layout->addWidget(styled(
      QStringLiteral("• 차량 정지 상태에서만 실행 (주행 중이면 거부)\n"
      "• 약 5~10초간 프리뷰·자동캡처 일시정지\n"
      "• R 렌즈 + 도트 프로젝터 임시 ON 후 원복\n"
      "• 실패 시 이전 자세 유지"), "dim"));
  refit_message_ = styled(QString(), "dim");
  refit_message_->setWordWrap(true);
  refit_layout->addWidget(refit_message_);
  layout->addWidget(refit);

  auto * grid = new QGroupBox(QStringLiteral("BEV 범위 (두 BEV 공통 grid)"));
  auto * grid_form = new QFormLayout(grid);
  grid_form->addRow(QStringLiteral("전방 x"), new QLabel(
      QStringLiteral("%1 ~ %2 m").arg(params_.grid.x_min_m).arg(params_.grid.x_max_m)));
  grid_form->addRow(QStringLiteral("좌우 y"), new QLabel(
      QStringLiteral("%1 ~ %2 m (차량 중심 = 0)").arg(params_.grid.y_min_m).arg(params_.grid.y_max_m)));
  grid_form->addRow(QStringLiteral("해상도"), new QLabel(
      QStringLiteral("%1 cm/px · %2×%3").arg(params_.grid.meter_per_pixel * 100.0)
      .arg(params_.grid.output_width).arg(params_.grid.output_height)));
  layout->addWidget(grid);
  layout->addStretch(1);

  connect(left_bev_mode_, &QPushButton::clicked, this, [this](bool on) {
      CameraConfig config = applied_config_;
      config.stereo_mode = on ? StereoMode::LEFT_ONLY : StereoMode::LEFT_RIGHT;
      requestRestart(config);
    });
  connect(refit_button_, &QPushButton::clicked, this, [this]() {requestRefit();});
  return tab;
}

QWidget * MainWindow::buildSaveTab()
{
  auto * tab = new QWidget;
  auto * layout = new QVBoxLayout(tab);
  auto * box = new QGroupBox(QStringLiteral("저장"));
  auto * form = new QFormLayout(box);
  auto * root = new QLabel(QString::fromStdString(params_.capture_root));
  root->setWordWrap(true);
  form->addRow(QStringLiteral("저장 위치"), root);
  session_label_ = styled(QStringLiteral("(첫 캡처 때 생성)"), "dim");
  session_label_->setWordWrap(true);
  form->addRow(QStringLiteral("세션 폴더"), session_label_);
  form->addRow(QStringLiteral("포맷"), new QLabel(QStringLiteral("JPEG (전 화면 공통)")));
  jpeg_quality_ = new QSpinBox;
  jpeg_quality_->setRange(50, 100);
  jpeg_quality_->setValue(params_.jpeg_quality);
  form->addRow(QStringLiteral("JPEG 품질"), jpeg_quality_);
  layout->addWidget(box);
  auto * meta = new QGroupBox(QStringLiteral("세션 기록 (meta.csv)"));
  auto * meta_layout = new QVBoxLayout(meta);
  meta_layout->addWidget(styled(
      QStringLiteral("이벤트 번호, 시각, 트리거, 저장 화면,\n노출/ISO/IR, ERPM, 조향,\n"
      "BEV 높이·pitch·roll, 해상도·포맷"), "dim"));
  layout->addWidget(meta);
  layout->addStretch(1);
  connect(jpeg_quality_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int quality) {
      if (writer_) {writer_->setJpegQuality(quality);}
    });
  return tab;
}

QWidget * MainWindow::buildStatusBar()
{
  auto * bar = new QFrame;
  bar->setObjectName("bar");
  auto * layout = new QHBoxLayout(bar);
  layout->setContentsMargins(10, 4, 10, 4);
  rec_label_ = styled(QStringLiteral("대기 중"), "dim");
  total_label_ = styled(QString(), "dim");
  queue_label_ = styled(QString(), "dim");
  disk_label_ = styled(QString(), "dim");
  fps_label_ = styled(QString(), "dim");
  message_label_ = styled(QString(), "warn");
  for (auto * label : {rec_label_, total_label_, queue_label_, disk_label_, fps_label_}) {
    layout->addWidget(label);
  }
  layout->addStretch(1);
  layout->addWidget(message_label_);
  return bar;
}

// ---------------------------------------------------------------- jobs

void MainWindow::runJob(JobKind kind, std::function<JobResult()> work, const QString & busy_text)
{
  if (job_running_) {
    return;
  }
  resume_auto_after_job_ = controller_.recording();
  handleAction(controller_.stop());
  job_running_ = true;
  job_kind_ = kind;
  message_label_->setObjectName("warn");
  message_label_->setText(busy_text);
  for (auto * pane : panes_) {
    if (selection_.preview(pane->view())) {
      pane->showMessage(busy_text);
    }
  }
  bev_generation_ = 0U;  // reconfigure BEV after the restart
  job_future_ = std::async(std::launch::async, std::move(work));
  refreshControls();
}

MainWindow::JobResult MainWindow::measureAndStart(const CameraConfig & config, const bool measure)
{
  JobResult result;
  CameraConfig start_config = config;
  if (measure) {
    try {
      auto measurement_config = params_.measurement;
      measurement_config.device_id = config.device_id;
      measurement_config.roi_preview_enabled = false;  // never open an OpenCV window
      const auto measured = measureOakStartupExtrinsics(measurement_config);
      start_config.device_id = measured.device_id;
      result.pose = MountPose{measured.height_m, measured.roll_deg, measured.pitch_down_deg};
      result.pose_label = QStringLiteral("%1 측정 · 잔차 %2 mm")
        .arg(QString::fromStdString(localTimeString(std::time(nullptr))))
        .arg(measured.plane_residual_mad_m * 1000.0, 0, 'f', 1);
      result.message = QStringLiteral("지면 측정 완료: 높이 %1 m, pitch %2°, roll %3°")
        .arg(measured.height_m, 0, 'f', 3).arg(measured.pitch_down_deg, 0, 'f', 2)
        .arg(measured.roll_deg, 0, 'f', 2);
    } catch (const std::exception & exception) {
      result.ok = false;
      result.message = QStringLiteral("지면 측정 실패 (이전 자세 유지): %1").arg(exception.what());
    }
  }
  try {
    // Keep live controls across restarts; the very first start uses the defaults.
    engine_.start(start_config, engine_.generation() > 0 ? engine_.controls() : params_.controls);
  } catch (const std::exception & exception) {
    result.ok = false;
    result.message += QStringLiteral(" / 카메라 시작 실패: %1").arg(exception.what());
  }
  return result;
}

void MainWindow::requestRestart(const CameraConfig & config)
{
  if (job_running_) {
    return;
  }
  const CameraConfig previous = applied_config_;
  runJob(JobKind::RESTART, [this, config, previous]() {
      JobResult result;
      const auto controls = engine_.controls();
      try {
        engine_.start(config, controls);
        result.message = QStringLiteral("카메라 설정 적용 완료");
      } catch (const std::exception & exception) {
        result.ok = false;
        result.message = QStringLiteral("설정 적용 실패, 이전 설정으로 복구: %1").arg(exception.what());
        try {
          engine_.start(previous, controls);
        } catch (const std::exception & second) {
          result.message += QStringLiteral(" / 복구 실패: %1").arg(second.what());
        }
      }
      return result;
    }, QStringLiteral("카메라 재시작 중..."));
}

void MainWindow::requestRefit()
{
  if (job_running_) {
    return;
  }
  const double now = steadyNowSec();
  const std::optional<int> erpm = telemetry_->has_erpm.load() ?
    std::optional<int>(telemetry_->erpm.load()) : std::nullopt;
  const auto decision = decideRefit(
    erpm, now - telemetry_->erpm_time_sec.load(), params_.refit_maximum_abs_erpm,
    params_.erpm_timeout_sec);
  refit_message_->setText(QString::fromStdString(decision.reason));
  refit_message_->setObjectName(decision.allowed ? "dim" : "err");
  refit_message_->style()->unpolish(refit_message_);
  refit_message_->style()->polish(refit_message_);
  if (!decision.allowed) {
    return;
  }
  const CameraConfig config = applied_config_;
  runJob(JobKind::REFIT, [this, config]() {
      engine_.stop();  // the measurement opens the device itself
      return measureAndStart(config, true);
    }, QStringLiteral("지면 재추정 중... 차량을 움직이지 마세요"));
}

void MainWindow::finishJob()
{
  JobResult result = job_future_.get();
  job_running_ = false;
  if (result.pose) {
    pose_ = *result.pose;
    pose_label_ = result.pose_label;
  }
  if (engine_.running()) {
    applied_config_ = engine_.config();
  }
  if (selection_.mode() != applied_config_.stereo_mode) {
    selection_.setMode(applied_config_.stereo_mode);
    if (applied_config_.stereo_mode == StereoMode::LEFT_ONLY) {
      selection_.setPreview(View::LEFT_BEV, true);
    }
  }
  message_label_->setObjectName(result.ok ? "ok" : "err");
  message_label_->style()->unpolish(message_label_);
  message_label_->style()->polish(message_label_);
  message_label_->setText(result.message);
  if (job_kind_ == JobKind::REFIT) {
    refit_message_->setText(result.message);
  }
  updateExposureRanges();
  updateDotProjector();
  for (auto * pane : panes_) {
    pane->next_preview_sec = 0.0;
  }
  refreshControls();
  if (resume_auto_after_job_ && engine_.running()) {
    handleAction(controller_.start());
  }
  resume_auto_after_job_ = false;
}

// ---------------------------------------------------------------- periodic

void MainWindow::ensureBevProjectors()
{
  if (!engine_.running() || bev_generation_ == engine_.generation()) {
    return;
  }
  const auto rgb = engine_.rgbCalibration();
  const bool need_left = applied_config_.stereo_mode == StereoMode::LEFT_ONLY;
  const auto left = need_left ? engine_.leftCalibration() : std::nullopt;
  if (!rgb || (need_left && !left)) {
    return;  // wait for the first frames of this generation
  }
  try {
    rgb_bev_.configure(*rgb, pose_);
    if (left) {
      left_bev_.configure(*left, pose_);
    } else {
      left_bev_.reset();
    }
    bev_generation_ = engine_.generation();
  } catch (const std::exception & exception) {
    message_label_->setText(QStringLiteral("BEV 설정 실패: %1").arg(exception.what()));
    bev_generation_ = engine_.generation();
  }
}

std::map<View, cv::Mat> MainWindow::computeViews(
  const std::vector<View> & views, const std::shared_ptr<const RgbFrame> & rgb,
  const std::shared_ptr<const StereoFrame> & stereo)
{
  std::map<View, cv::Mat> output;
  for (const auto view : views) {
    try {
      switch (view) {
        case View::RGB:
          if (rgb) {output[view] = rgb->bgr;}
          break;
        case View::LEFT:
          if (stereo) {output[view] = stereo->left;}
          break;
        case View::RIGHT:
          if (stereo && !stereo->right.empty()) {output[view] = stereo->right;}
          break;
        case View::CENTER:
          if (stereo && !stereo->disparity.empty() && reprojector_) {
            output[view] = reprojector_->process(
              stereo->left, stereo->right, stereo->disparity,
              applied_config_.virtual_camera_position_ratio, true).clone();
          }
          break;
        case View::RGB_BEV:
          if (rgb && rgb_bev_.ready()) {output[view] = rgb_bev_.project(rgb->bgr);}
          break;
        case View::LEFT_BEV:
          if (stereo && left_bev_.ready() && stereo->right.empty()) {
            output[view] = left_bev_.project(stereo->left);
          }
          break;
      }
    } catch (const std::exception & exception) {
      message_label_->setText(
        QStringLiteral("%1 처리 실패: %2").arg(shortTitle(view)).arg(exception.what()));
    }
  }
  return output;
}

void MainWindow::tick()
{
  if (job_running_) {
    if (job_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      finishJob();
    }
    return;
  }
  if (!engine_.running()) {
    return;
  }
  ensureBevProjectors();
  const double now = steadyNowSec();

  if (controller_.recording() && auto_scheduler_.due(now)) {
    captureViews(selection_.captureTargets(), Trigger::AUTO);
  }

  std::vector<View> due;
  for (auto * pane : panes_) {
    const View view = pane->view();
    if (!selection_.available(view) || now < pane->next_preview_sec) {
      continue;
    }
    pane->next_preview_sec = now + 1.0 / std::max(1, pane->previewFps());
    due.push_back(view);
  }
  if (due.empty()) {
    return;
  }
  const auto rgb = engine_.latestRgb();
  const auto stereo = engine_.latestStereo();
  const auto images = computeViews(due, rgb, stereo);
  for (const auto view : due) {
    auto * pane = panes_[index(view)];
    const auto found = images.find(view);
    if (found == images.end() || found->second.empty()) {
      pane->showMessage(isBev(view) ? QStringLiteral("BEV 준비 중...") :
        QStringLiteral("프레임 대기 중..."));
      continue;
    }
    pane->showImage(found->second, isBev(view));
    const auto & image = found->second;
    QString footer = QStringLiteral("%1×%2").arg(image.cols).arg(image.rows);
    if (view == View::RGB) {
      footer += QStringLiteral(" %1 · 센서 %2fps").arg(
        applied_config_.rgb_format == RgbFormat::NV12 ? "NV12" : "RGB888")
        .arg(engine_.measuredRgbFps(), 0, 'f', 1);
    } else if (!isBev(view)) {
      footer += QStringLiteral(" GRAY · 센서 %1fps").arg(engine_.measuredStereoFps(), 0, 'f', 1);
    } else {
      footer += QStringLiteral(" · %1 cm/px · 차량 중심 정렬").arg(params_.grid.meter_per_pixel * 100.0);
    }
    footer += QStringLiteral(" · 프리뷰 %1fps").arg(pane->previewFps());
    pane->setFooter(footer);
  }
}

void MainWindow::statusTick()
{
  // Camera / vehicle state
  if (engine_.running()) {
    device_label_->setObjectName("ok");
    device_label_->setText(QStringLiteral("● OAK %1").arg(QString::fromStdString(engine_.deviceId())));
  } else {
    device_label_->setObjectName(job_running_ ? "warn" : "err");
    device_label_->setText(job_running_ ? QStringLiteral("● OAK 준비 중") : QStringLiteral("● OAK 끊김"));
  }
  device_label_->style()->unpolish(device_label_);
  device_label_->style()->polish(device_label_);
  mode_label_->setText(applied_config_.stereo_mode == StereoMode::LEFT_ONLY ?
    QStringLiteral("모드: 컬러 + 왼쪽 BEV (R 잠김)") : QStringLiteral("모드: 컬러 + 스테레오 L/R"));

  const double now = steadyNowSec();
  const bool fresh = telemetry_->has_erpm.load() &&
    now - telemetry_->erpm_time_sec.load() <= params_.erpm_timeout_sec;
  const int erpm = telemetry_->erpm.load();
  const bool moving = fresh && std::abs(erpm) > params_.refit_maximum_abs_erpm;
  drive_label_->setObjectName(!fresh ? "dim" : (moving ? "warn" : "ok"));
  drive_label_->setText(!fresh ? QStringLiteral("● ERPM 미수신") :
    QStringLiteral("● ERPM %1 (%2)").arg(erpm).arg(moving ? QStringLiteral("주행 중") : QStringLiteral("정지")));
  drive_label_->style()->unpolish(drive_label_);
  drive_label_->style()->polish(drive_label_);
  refit_button_->setEnabled(!job_running_ && !moving);  // also recovers a failed start
  refit_button_->setText(moving ? QStringLiteral("⟲  Refit — 주행 중 불가") :
    QStringLiteral("⟲  Refit (지면 재추정)"));

  // Capture counters
  std::uint64_t total = 0U;
  for (const auto view : kAllViews) {
    const std::uint64_t count = writer_ ? writer_->written(view) : 0U;
    total += count;
    panes_[index(view)]->setCount(count);
    target_counts_[index(view)]->setText(
      QStringLiteral("%1장").arg(QLocale().toString(static_cast<qulonglong>(count))));
  }
  total_label_->setText(QStringLiteral("│  이번 실행 총 %1장").arg(QLocale().toString(static_cast<qulonglong>(total))));
  if (writer_) {
    queue_label_->setText(QStringLiteral("│  저장 큐 %1/%2 · 드롭 %3 · 실패 %4")
      .arg(writer_->queued()).arg(writer_->capacity()).arg(writer_->dropped()).arg(writer_->failed()));
  } else {
    queue_label_->setText(QStringLiteral("│  저장 큐 0"));
  }
  std::error_code error;
  std::filesystem::path probe(params_.capture_root);
  while (!probe.empty() && !std::filesystem::exists(probe, error)) {
    probe = probe.parent_path();
  }
  const auto space = std::filesystem::space(probe.empty() ? "." : probe, error);
  if (!error) {
    disk_label_->setText(QStringLiteral("│  디스크 여유 %1 GB").arg(
        static_cast<double>(space.available) / 1.0e9, 0, 'f', 1));
  }
  fps_label_->setText(QStringLiteral("│  컬러 %1fps · 스테레오 %2fps")
    .arg(engine_.measuredRgbFps(), 0, 'f', 1).arg(engine_.measuredStereoFps(), 0, 'f', 1));
  if (controller_.recording()) {
    const int elapsed = static_cast<int>(now - recording_started_sec_);
    rec_label_->setObjectName("err");
    rec_label_->setText(QStringLiteral("● REC 자동 %1 fps · %2:%3:%4")
      .arg(capture_fps_->value(), 0, 'f', 1)
      .arg(elapsed / 3600, 2, 10, QChar('0')).arg((elapsed / 60) % 60, 2, 10, QChar('0'))
      .arg(elapsed % 60, 2, 10, QChar('0')));
  } else {
    rec_label_->setObjectName("dim");
    rec_label_->setText(QStringLiteral("대기 중"));
  }
  rec_label_->style()->unpolish(rec_label_);
  rec_label_->style()->polish(rec_label_);

  const auto controls = engine_.controls();
  dot_state_->setText(controls.dot > 0.0 ?
    QStringLiteral("현재 상태: ON (%1%)").arg(static_cast<int>(controls.dot * 100.0)) :
    QStringLiteral("현재 상태: 꺼짐"));
  if (const auto rgb = engine_.latestRgb()) {
    rgb_actual_->setText(QStringLiteral("실제: %1 µs · ISO %2").arg(rgb->exposure_us).arg(rgb->iso));
  }
  if (const auto stereo = engine_.latestStereo()) {
    QString text = QStringLiteral("실제 L: %1 µs · ISO %2").arg(stereo->left_exposure_us).arg(stereo->left_iso);
    if (!stereo->right.empty()) {
      text += QStringLiteral("  /  R: %1 µs · ISO %2").arg(stereo->right_exposure_us).arg(stereo->right_iso);
    }
    stereo_actual_->setText(text);
  }
  const auto error_text = engine_.lastError();
  if (!error_text.empty() && !job_running_) {
    message_label_->setText(QString::fromStdString(error_text));
  }
}

void MainWindow::refreshControls()
{
  updating_ui_ = true;
  const bool idle = !job_running_ && engine_.running();
  const bool recording = controller_.recording();
  for (const auto view : kAllViews) {
    auto * pane = panes_[index(view)];
    const bool locked = viewLocked(view, selection_.mode());
    pane->setState(selection_.preview(view), locked, lockReason(view));
    pane->setCaptureEnabled(idle && selection_.available(view) && controller_.singleViewCaptureAllowed());
    auto * check = target_checks_[index(view)];
    check->setEnabled(selection_.available(view));
    check->setChecked(selection_.checked(view) && selection_.available(view));
    auto * state = target_states_[index(view)];
    state->setText(locked ? QStringLiteral("🔒 잠김") :
      (selection_.preview(view) ? QStringLiteral("프리뷰 ON") : QStringLiteral("프리뷰 OFF")));
    state->setObjectName(selection_.available(view) ? "ok" : "dim");
    state->style()->unpolish(state);
    state->style()->polish(state);
  }
  single_radio_->setEnabled(!recording);
  auto_radio_->setEnabled(!recording);
  start_button_->setEnabled(idle && !recording);
  stop_button_->setEnabled(recording);
  body_->setEnabled(!job_running_);

  const bool left_only = applied_config_.stereo_mode == StereoMode::LEFT_ONLY;
  left_bev_mode_->setChecked(left_only);
  left_bev_mode_->setText(left_only ? QStringLiteral("왼쪽 렌즈 BEV  ON") :
    QStringLiteral("왼쪽 렌즈 BEV  OFF"));
  const bool manual = !stereo_auto_exposure_->isChecked();
  for (QWidget * widget : std::initializer_list<QWidget *>{
      left_exposure_, left_exposure_slider_, left_iso_, left_iso_slider_})
  {
    widget->setEnabled(manual);
  }
  const bool right_editable = manual && !left_only && !stereo_link_->isChecked();
  for (QWidget * widget : std::initializer_list<QWidget *>{
      right_exposure_, right_exposure_slider_, right_iso_, right_iso_slider_})
  {
    widget->setEnabled(right_editable);
  }
  const bool rgb_manual = !rgb_auto_exposure_->isChecked();
  for (QWidget * widget : std::initializer_list<QWidget *>{
      rgb_exposure_, rgb_exposure_slider_, rgb_iso_, rgb_iso_slider_})
  {
    widget->setEnabled(rgb_manual);
  }

  pose_height_->setText(QStringLiteral("%1 m").arg(pose_.height_m, 0, 'f', 3));
  pose_pitch_->setText(QStringLiteral("%1°").arg(pose_.pitch_down_deg, 0, 'f', 2));
  pose_roll_->setText(QStringLiteral("%1°").arg(pose_.roll_deg, 0, 'f', 2));
  pose_source_->setText(pose_label_);
  updating_ui_ = false;
}

void MainWindow::updateDotProjector()
{
  if (!engine_.running()) {
    return;
  }
  const double wanted = dotProjectorRequired(selection_) ? params_.dot_intensity_when_needed : 0.0;
  if (std::abs(engine_.controls().dot - wanted) > 1.0e-6) {
    engine_.setDot(wanted);
  }
}

void MainWindow::updateExposureRanges()
{
  setRange(rgb_exposure_slider_, rgb_exposure_, maximumExposureUs(applied_config_.rgb_fps));
  const int stereo_max = maximumExposureUs(applied_config_.stereo_fps);
  setRange(left_exposure_slider_, left_exposure_, stereo_max);
  setRange(right_exposure_slider_, right_exposure_, stereo_max);
}

void MainWindow::applyRgbExposure()
{
  if (updating_ui_) {
    return;
  }
  engine_.setRgbExposure({rgb_auto_exposure_->isChecked(), rgb_exposure_->value(), rgb_iso_->value()});
  refreshControls();
}

void MainWindow::applyStereoExposure(const bool left_changed)
{
  if (updating_ui_) {
    return;
  }
  const bool automatic = stereo_auto_exposure_->isChecked();
  if (stereo_link_->isChecked() && left_changed) {
    updating_ui_ = true;
    right_exposure_->setValue(left_exposure_->value());
    right_iso_->setValue(left_iso_->value());
    updating_ui_ = false;
  }
  engine_.setStereoExposure(true, {automatic, left_exposure_->value(), left_iso_->value()});
  if (applied_config_.stereo_mode == StereoMode::LEFT_RIGHT) {
    engine_.setStereoExposure(false, {automatic, right_exposure_->value(), right_iso_->value()});
  }
}

CameraConfig MainWindow::pendingRgbConfig() const
{
  CameraConfig config = applied_config_;
  const QSize size = rgb_resolution_->currentData().toSize();
  config.rgb_width = size.width();
  config.rgb_height = size.height();
  config.rgb_fps = rgb_fps_->value();
  config.rgb_format = rgb_format_->currentIndex() == 0 ? RgbFormat::NV12 : RgbFormat::RGB888;
  return config;
}

CameraConfig MainWindow::pendingStereoConfig() const
{
  CameraConfig config = applied_config_;
  const QSize size = stereo_resolution_->currentData().toSize();
  config.stereo_width = size.width();
  config.stereo_height = size.height();
  config.stereo_fps = stereo_fps_->value();
  return config;
}

// ---------------------------------------------------------------- capture

void MainWindow::onJoyCapture()
{
  if (!job_running_ && engine_.running()) {
    handleAction(controller_.trigger());
  }
}

void MainWindow::handleAction(const CaptureAction action)
{
  switch (action) {
    case CaptureAction::CAPTURE_ONCE:
      captureViews(selection_.captureTargets(), Trigger::SINGLE);
      break;
    case CaptureAction::START_AUTO:
      recording_started_sec_ = steadyNowSec();
      auto_scheduler_.reset(recording_started_sec_, capture_fps_->value());
      break;
    case CaptureAction::STOP_AUTO:
    case CaptureAction::NONE:
      break;
  }
  refreshControls();
}

bool MainWindow::ensureWriter()
{
  if (writer_) {
    return true;
  }
  try {
    const auto session = std::filesystem::path(params_.capture_root) /
      sessionDirectoryName(std::time(nullptr));
    writer_ = std::make_unique<CaptureWriter>(
      session,
      "event,unix_time_ns,local_time,trigger,views,stereo_mode,rgb_size,rgb_format,"
      "stereo_size,rgb_exposure_us,rgb_iso,left_exposure_us,left_iso,right_exposure_us,"
      "right_iso,flood,dot,erpm,servo,bev_height_m,bev_roll_deg,bev_pitch_down_deg,bev_pose",
      static_cast<std::size_t>(params_.writer_queue_events));
    writer_->setJpegQuality(jpeg_quality_->value());
    session_label_->setText(QString::fromStdString(session.string()));
    std::ofstream settings(session / "settings.yaml");
    settings << "device_id: \"" << engine_.deviceId() << "\"\n"
             << "bev_grid: {x_min_m: " << params_.grid.x_min_m << ", x_max_m: " << params_.grid.x_max_m
             << ", y_min_m: " << params_.grid.y_min_m << ", y_max_m: " << params_.grid.y_max_m
             << ", meter_per_pixel: " << params_.grid.meter_per_pixel
             << ", camera_x_m: " << params_.grid.camera_x_m << "}\n"
             << "folders: [rgb, stereo_center, stereo_left, stereo_right, rgb_bev, stereo_left_bev]\n";
    return true;
  } catch (const std::exception & exception) {
    message_label_->setText(QStringLiteral("세션 폴더 생성 실패: %1").arg(exception.what()));
    return false;
  }
}

std::string MainWindow::metaRow(
  const std::uint64_t event, const Trigger trigger, const std::vector<View> & views,
  const std::shared_ptr<const RgbFrame> & rgb, const std::shared_ptr<const StereoFrame> & stereo) const
{
  const auto wall = std::chrono::system_clock::now();
  const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(wall.time_since_epoch()).count();
  std::ostringstream row;
  row << event << ',' << ns << ',' << localTimeString(std::chrono::system_clock::to_time_t(wall)) << ','
      << (trigger == Trigger::AUTO ? "auto" : (trigger == Trigger::SINGLE ? "single" : "view")) << ',';
  for (std::size_t i = 0; i < views.size(); ++i) {
    row << (i ? ";" : "") << viewKey(views[i]);
  }
  const auto controls = engine_.controls();
  const bool fresh = telemetry_->has_erpm.load() &&
    steadyNowSec() - telemetry_->erpm_time_sec.load() <= params_.erpm_timeout_sec;
  row << ',' << (applied_config_.stereo_mode == StereoMode::LEFT_ONLY ? "left_only" : "left_right")
      << ',' << applied_config_.rgb_width << 'x' << applied_config_.rgb_height
      << ',' << (applied_config_.rgb_format == RgbFormat::NV12 ? "NV12" : "RGB888")
      << ',' << applied_config_.stereo_width << 'x' << applied_config_.stereo_height
      << ',' << (rgb ? rgb->exposure_us : 0) << ',' << (rgb ? rgb->iso : 0)
      << ',' << (stereo ? stereo->left_exposure_us : 0) << ',' << (stereo ? stereo->left_iso : 0)
      << ',' << (stereo ? stereo->right_exposure_us : 0) << ',' << (stereo ? stereo->right_iso : 0)
      << ',' << controls.flood << ',' << controls.dot << ',';
  if (fresh) {row << telemetry_->erpm.load();}
  row << ',';
  if (telemetry_->has_servo.load()) {row << telemetry_->servo.load();}
  row << ',' << std::fixed << std::setprecision(4) << pose_.height_m
      << ',' << std::setprecision(3) << pose_.roll_deg << ',' << pose_.pitch_down_deg
      << ",\"" << pose_label_.toStdString() << '"';
  return row.str();
}

void MainWindow::captureViews(const std::vector<View> & views, const Trigger trigger)
{
  if (views.empty() || !engine_.running()) {
    if (views.empty() && trigger != Trigger::AUTO) {
      message_label_->setText(QStringLiteral("캡처 대상이 없습니다 (프리뷰를 켜고 체크하세요)"));
    }
    return;
  }
  if (!ensureWriter()) {
    return;
  }
  const auto rgb = engine_.latestRgb();
  const auto stereo = engine_.latestStereo();
  auto images = computeViews(views, rgb, stereo);
  CaptureEvent event;
  event.index = next_event_index_;
  std::vector<View> saved;
  for (const auto view : views) {
    auto found = images.find(view);
    if (found == images.end() || found->second.empty()) {
      continue;
    }
    // Raw frames are shared snapshots; give the writer its own copy.
    event.images.push_back({view, found->second.clone()});
    saved.push_back(view);
  }
  if (saved.empty()) {
    return;
  }
  event.meta_row = metaRow(event.index, trigger, saved, rgb, stereo);
  if (writer_->enqueue(std::move(event))) {
    ++next_event_index_;
  }
}

}  // namespace manual_capture
