#include "manual_capture/capture_logic.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace manual_capture
{

const char * viewKey(const View view)
{
  switch (view) {
    case View::RGB: return "rgb";
    case View::CENTER: return "stereo_center";
    case View::LEFT: return "stereo_left";
    case View::RIGHT: return "stereo_right";
    case View::RGB_BEV: return "rgb_bev";
    case View::LEFT_BEV: return "stereo_left_bev";
  }
  return "unknown";
}

bool viewLocked(const View view, const StereoMode mode)
{
  if (mode == StereoMode::LEFT_ONLY) {
    return view == View::RIGHT || view == View::CENTER;
  }
  return view == View::LEFT_BEV;
}

ViewSelection::ViewSelection(const StereoMode mode)
: mode_(mode)
{
  // Initial launch: only the color preview is on.
  preview_[index(View::RGB)] = true;
  checked_[index(View::RGB)] = true;
}

void ViewSelection::setMode(const StereoMode mode)
{
  mode_ = mode;
  for (const auto view : kAllViews) {
    if (viewLocked(view, mode_)) {
      preview_[index(view)] = false;
      checked_[index(view)] = false;
    }
  }
}

bool ViewSelection::available(const View view) const
{
  return preview_[index(view)] && !viewLocked(view, mode_);
}

bool ViewSelection::setPreview(const View view, const bool enabled)
{
  if (enabled && viewLocked(view, mode_)) {
    return false;
  }
  preview_[index(view)] = enabled;
  checked_[index(view)] = enabled;
  return true;
}

bool ViewSelection::setChecked(const View view, const bool checked)
{
  if (checked && !available(view)) {
    return false;
  }
  checked_[index(view)] = checked;
  return true;
}

std::vector<View> ViewSelection::captureTargets() const
{
  std::vector<View> targets;
  for (const auto view : kAllViews) {
    if (checked_[index(view)] && available(view)) {
      targets.push_back(view);
    }
  }
  return targets;
}

bool dotProjectorRequired(const ViewSelection & selection)
{
  return selection.mode() == StereoMode::LEFT_RIGHT && selection.available(View::CENTER);
}

bool CaptureController::setMode(const CaptureMode mode)
{
  if (recording_) {
    return false;
  }
  mode_ = mode;
  return true;
}

CaptureAction CaptureController::trigger()
{
  if (mode_ == CaptureMode::SINGLE) {
    return CaptureAction::CAPTURE_ONCE;
  }
  return recording_ ? stop() : start();
}

CaptureAction CaptureController::start()
{
  if (mode_ == CaptureMode::SINGLE) {
    return CaptureAction::CAPTURE_ONCE;
  }
  if (recording_) {
    return CaptureAction::NONE;
  }
  recording_ = true;
  return CaptureAction::START_AUTO;
}

CaptureAction CaptureController::stop()
{
  if (!recording_) {
    return CaptureAction::NONE;
  }
  recording_ = false;
  return CaptureAction::STOP_AUTO;
}

bool ButtonEdge::update(const bool pressed)
{
  // The first sample only primes the state so a button held during startup
  // does not fire.
  const bool rising = initialized_ && pressed && !previous_;
  previous_ = pressed;
  initialized_ = true;
  return rising;
}

RefitDecision decideRefit(
  const std::optional<int> latest_erpm, const double erpm_age_sec,
  const int maximum_abs_erpm, const double erpm_timeout_sec)
{
  if (!latest_erpm || !std::isfinite(erpm_age_sec) || erpm_age_sec > erpm_timeout_sec) {
    return {true, "ERPM 미수신: 정지 상태로 가정하고 진행"};
  }
  if (std::abs(*latest_erpm) > maximum_abs_erpm) {
    return {false, "주행 중이라 Refit 불가 (ERPM " + std::to_string(*latest_erpm) + ")"};
  }
  return {true, "정지 확인"};
}

int maximumExposureUs(const double fps)
{
  if (!std::isfinite(fps) || fps <= 0.0) {
    return 0;
  }
  return static_cast<int>(std::floor(1.0e6 / fps));
}

std::string sessionDirectoryName(const std::time_t time)
{
  std::tm local{};
  localtime_r(&time, &local);
  char buffer[64];
  std::strftime(buffer, sizeof(buffer), "session_%Y%m%d_%H%M%S", &local);
  return buffer;
}

std::string frameFileName(const std::uint64_t event_index)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%06llu.jpg",
    static_cast<unsigned long long>(event_index));
  return buffer;
}

void IntervalScheduler::reset(const double now_sec, const double fps)
{
  period_sec_ = (std::isfinite(fps) && fps > 0.0) ? 1.0 / fps : 1.0;
  next_sec_ = now_sec;
}

bool IntervalScheduler::due(const double now_sec)
{
  if (now_sec < next_sec_) {
    return false;
  }
  next_sec_ += period_sec_;
  if (next_sec_ <= now_sec) {
    next_sec_ = now_sec + period_sec_;
  }
  return true;
}

}  // namespace manual_capture
