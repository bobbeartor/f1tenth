#ifndef MANUAL_CAPTURE__CAPTURE_LOGIC_HPP_
#define MANUAL_CAPTURE__CAPTURE_LOGIC_HPP_

// Camera/Qt independent rules of the capture GUI. Kept separate so the
// behaviour the operator relies on can be unit tested without an OAK device.

#include <array>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <optional>
#include <string>
#include <vector>

namespace manual_capture
{

// GUI order: top row RGB/CENTER, bottom row LEFT/RIGHT, side column BEVs.
enum class View : std::size_t
{
  RGB = 0,
  CENTER = 1,
  LEFT = 2,
  RIGHT = 3,
  RGB_BEV = 4,
  LEFT_BEV = 5,
};
constexpr std::size_t kViewCount = 6U;
constexpr std::array<View, kViewCount> kAllViews{
  View::RGB, View::CENTER, View::LEFT, View::RIGHT, View::RGB_BEV, View::LEFT_BEV};

inline constexpr std::size_t index(View view) {return static_cast<std::size_t>(view);}

// Folder name inside a capture session.
const char * viewKey(View view);

// LEFT_RIGHT: both mono lenses + stereo depth (center view available).
// LEFT_ONLY : the left lens alone, used for the vehicle-aligned left BEV.
enum class StereoMode
{
  LEFT_RIGHT,
  LEFT_ONLY,
};

// A locked view has no stream in the current mode and can neither be
// previewed nor captured.
bool viewLocked(View view, StereoMode mode);

// Tracks which panes are previewed and which are selected for the combined
// capture (A button / start button). A view can only be selected while it is
// previewed; turning a preview on selects it, turning it off deselects it.
class ViewSelection
{
public:
  explicit ViewSelection(StereoMode mode = StereoMode::LEFT_RIGHT);

  StereoMode mode() const {return mode_;}
  void setMode(StereoMode mode);

  bool preview(View view) const {return preview_[index(view)];}
  bool checked(View view) const {return checked_[index(view)];}
  bool available(View view) const;

  // Returns false (and changes nothing) for a locked view.
  bool setPreview(View view, bool enabled);
  // Returns false (and changes nothing) when the view is not available.
  bool setChecked(View view, bool checked);

  std::vector<View> captureTargets() const;

private:
  StereoMode mode_;
  std::array<bool, kViewCount> preview_{};
  std::array<bool, kViewCount> checked_{};
};

// The dot projector is only needed for disparity (center view) and is never
// left on otherwise, so mono images used for labeling stay dot free.
bool dotProjectorRequired(const ViewSelection & selection);

enum class CaptureMode
{
  SINGLE,
  AUTO,
};

enum class CaptureAction
{
  NONE,
  CAPTURE_ONCE,
  START_AUTO,
  STOP_AUTO,
};

// Single mode: every trigger saves one set. Auto mode: a trigger toggles.
class CaptureController
{
public:
  CaptureMode mode() const {return mode_;}
  bool recording() const {return recording_;}
  // Mode cannot change while auto capture is running.
  bool setMode(CaptureMode mode);

  // A button and the start button share these rules.
  CaptureAction trigger();
  // Explicit start/stop buttons.
  CaptureAction start();
  CaptureAction stop();

  // Per-pane capture is forbidden during auto capture.
  bool singleViewCaptureAllowed() const {return !recording_;}

private:
  CaptureMode mode_{CaptureMode::SINGLE};
  bool recording_{false};
};

// Rising-edge detector for a joystick button.
class ButtonEdge
{
public:
  bool update(bool pressed);

private:
  bool previous_{false};
  bool initialized_{false};
};

struct RefitDecision
{
  bool allowed{false};
  std::string reason;
};

// Refit needs a stationary vehicle. Missing telemetry is allowed (the car may
// simply be powered off) but reported.
RefitDecision decideRefit(
  std::optional<int> latest_erpm, double erpm_age_sec, int maximum_abs_erpm,
  double erpm_timeout_sec);

// Exposure must fit inside one frame period.
int maximumExposureUs(double fps);

// session_YYYYMMDD_HHMMSS (local time).
std::string sessionDirectoryName(std::time_t time);
// 000042.jpg
std::string frameFileName(std::uint64_t event_index);

// Fixed-interval scheduler for auto capture that never bursts after a stall.
class IntervalScheduler
{
public:
  void reset(double now_sec, double fps);
  bool due(double now_sec);

private:
  double period_sec_{1.0};
  double next_sec_{0.0};
};

}  // namespace manual_capture

#endif  // MANUAL_CAPTURE__CAPTURE_LOGIC_HPP_
