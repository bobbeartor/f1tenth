#include "line_detactor/speed_limit_panel.hpp"
#include <algorithm>
#include <cmath>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace line_detactor
{
bool SpeedLimitSnapshot::valid() const
{
  return std::isfinite(minimum) && std::isfinite(ceiling) && std::isfinite(applied) &&
         minimum > 0.0 && ceiling >= minimum && applied >= minimum && applied <= ceiling;
}

void SpeedLimitPanel::update(const SpeedLimitSnapshot & state, bool fresh, double now)
{
  now_ = now;
  const bool online = fresh && state.valid();
  const bool reset = !initialized_ || !available_ || state.minimum != state_.minimum ||
    state.ceiling != state_.ceiling || state.revision < state_.revision;
  available_ = online;
  if (!online) {
    dragging_ = dirty_ = pending_ = released_ = false;
    return;
  }
  state_ = state;
  if (reset) {
    selected_ = state.applied;
    dragging_ = dirty_ = pending_ = released_ = false;
    initialized_ = true;
    failed_until_ = -1.;
  }
  if (pending_ && !dirty_ && std::abs(state.applied-selected_) < 1e-6) {pending_ = false;}
  if (pending_ && !dirty_ && !dragging_ && now-last_sent_ > 1.5) {
    pending_ = false;
    failed_until_ = now+2.;
  }
  if (!dragging_ && !dirty_ && !pending_) {selected_ = state.applied;}
}

double SpeedLimitPanel::value_at(int y) const
{
  const double fraction = std::clamp(
    double(track_bottom_-y)/std::max(1, track_bottom_-track_top_), 0.0, 1.0);
  const double raw = state_.minimum + fraction*(state_.ceiling-state_.minimum);
  // 0.05 m/s increments, preserving non-grid YAML endpoints.
  if (y <= track_top_) {return state_.ceiling;}
  if (y >= track_bottom_) {return state_.minimum;}
  return std::clamp(std::round(raw*20.)/20., state_.minimum, state_.ceiling);
}

void SpeedLimitPanel::select(int y)
{
  const double value = value_at(y);
  if (value != selected_) {
    selected_ = value;
    dirty_ = pending_ = true;
    failed_until_ = -1.;
  }
}

void SpeedLimitPanel::mouse(int event, int x, int y, int flags)
{
  if(event==cv::EVENT_LBUTTONUP) {
    if(metrics_button_.contains(cv::Point(x,y)))action_="metrics";
    if(parameters_button_.contains(cv::Point(x,y)))action_="parameters";
    if(stop_button_.contains(cv::Point(x,y)))action_="stop";
  }
  if (!available_ || state_.ceiling <= state_.minimum) {return;}
  if (event == cv::EVENT_LBUTTONDOWN && hit_.contains(cv::Point(x, y))) {
    dragging_ = true;
    select(y);
  } else if (event == cv::EVENT_MOUSEMOVE && dragging_) {
    if (!(flags & cv::EVENT_FLAG_LBUTTON)) {
      dragging_ = false;
      released_ = dirty_;
    } else {select(y);}
  } else if (event == cv::EVENT_LBUTTONUP && dragging_) {
    select(y);
    dragging_ = false;
    released_ = dirty_;
  }
}

void SpeedLimitPanel::mouse_callback(int event, int x, int y, int flags, void * userdata)
{
  static_cast<SpeedLimitPanel *>(userdata)->mouse(event, x, y, flags);
}

std::optional<double> SpeedLimitPanel::take_request(double now)
{
  if (!available_ || !dirty_ || (!released_ && now-last_sent_ < .1)) {return std::nullopt;}
  dirty_ = released_ = false;
  pending_ = true;
  last_sent_ = now;
  return selected_;
}

std::string SpeedLimitPanel::display_key() const
{
  return cv::format("%.4f/%.4f/%.4f/%.4f/%d/%d/%d", state_.minimum, state_.ceiling,
    state_.applied, selected_, int(available_), int(pending_), int(now_ < failed_until_));
}

cv::Mat SpeedLimitPanel::append(const cv::Mat & bev, bool themed)
{
  const int width = std::max(90, bev.cols/2);
  const double ui = width/180.;
  const cv::Scalar background = themed ? cv::Scalar(250, 250, 250) : cv::Scalar(38, 38, 38);
  const cv::Scalar ink = themed ? cv::Scalar(64, 49, 38) : cv::Scalar(240, 240, 240);
  const cv::Scalar muted = themed ? cv::Scalar(139, 116, 100) : cv::Scalar(175, 175, 175);
  const cv::Scalar track = themed ? cv::Scalar(224, 218, 213) : cv::Scalar(95, 95, 95);
  const cv::Scalar accent = available_ ? (themed ? cv::Scalar(230, 153, 38) : ink) : muted;
  cv::Mat canvas;
  cv::copyMakeBorder(bev, canvas, 0, 0, 0, width, cv::BORDER_CONSTANT, background);
  const int left = bev.cols;
  cv::line(canvas, {left, 0}, {left, canvas.rows-1}, track);
  const auto text = [&](const std::string & label, double x, double y, double font,
      const cv::Scalar & color) {
      cv::putText(canvas, label, {left+cvRound(x*ui), cvRound(y*ui)},
        cv::FONT_HERSHEY_SIMPLEX, font*ui, color, 1, cv::LINE_AA);
    };
  text("TARGET MAX", 14, 28, .48, ink);
  text("SPEED", 14, 50, .48, ink);
  text(initialized_ ? cv::format("%.2f", selected_) : "--", 14, 91, 1.05, accent);
  text("m/s", 16, 116, .45, muted);
  track_top_ = std::max(cvRound(152*ui), canvas.rows/4);
  track_bottom_ = std::max(track_top_+1, canvas.rows-cvRound(94*ui));
  track_x_ = left+cvRound(54*ui);
  hit_ = cv::Rect(left+cvRound(20*ui), track_top_-cvRound(16*ui),
    cvRound(65*ui), track_bottom_-track_top_+cvRound(32*ui));
  const int thickness = std::max(2, cvRound(9*ui));
  cv::line(canvas, {track_x_, track_top_}, {track_x_, track_bottom_}, track, thickness, cv::LINE_AA);
  if (initialized_) {
    const double range = state_.ceiling-state_.minimum;
    const double fraction = range > 0. ? (selected_-state_.minimum)/range : 0.;
    const int thumb_y = cvRound(track_bottom_-fraction*(track_bottom_-track_top_));
    cv::line(canvas, {track_x_, thumb_y}, {track_x_, track_bottom_}, accent, thickness, cv::LINE_AA);
    cv::circle(canvas, {track_x_, thumb_y}, std::max(4, cvRound(13*ui)), accent, cv::FILLED, cv::LINE_AA);
    std::vector<double> ticks{state_.minimum};
    // Whole-m/s marks are easier to scan than equally spaced fractional labels.
    const double step = std::max(1., std::ceil(range/6.));
    for (double value = std::ceil(state_.minimum/step)*step; value < state_.ceiling; value += step) {
      if (value <= state_.minimum+1e-6) {continue;}
      const double gap = range > 0. ? (value-ticks.back())/range*(track_bottom_-track_top_) : 0.;
      const double end_gap = (state_.ceiling-value)/range*(track_bottom_-track_top_);
      if (gap >= 16*ui && end_gap >= 16*ui) {ticks.push_back(value);}
    }
    if (range > 0.) {ticks.push_back(state_.ceiling);}
    for (double value : ticks) {
      const double ratio = range > 0. ? (value-state_.minimum)/range : 0.;
      const int y = cvRound(track_bottom_-ratio*(track_bottom_-track_top_));
      cv::line(canvas, {left+cvRound(77*ui), y}, {left+cvRound(88*ui), y}, muted);
      text(cv::format("%.2f", value), 98, y/ui+4, .36, muted);
    }
  }
  const double bottom = canvas.rows/ui;
  text(initialized_ ? cv::format("Applied %.2f", state_.applied) : "Applied --", 12, bottom-64, .39, ink);
  const char * status = !available_ ? "CONTROLLER OFFLINE" :
    (now_ < failed_until_ ? "NOT APPLIED" : (pending_ ? "APPLYING..." : "LIVE"));
  text(status, 12, bottom-41, .32, !available_ || pending_ || now_ < failed_until_ ? muted : accent);
  text("DRAG TO ADJUST", 12, bottom-18, .32, muted);
  const int button_top=canvas.rows,height=std::max(30,cvRound(55*ui));
  cv::copyMakeBorder(canvas,canvas,0,height,0,0,cv::BORDER_CONSTANT,background);
  const int third=canvas.cols/3;
  metrics_button_={0,button_top,third,height};
  parameters_button_={third,button_top,third,height};
  stop_button_={2*third,button_top,canvas.cols-2*third,height};
  const std::vector<std::pair<cv::Rect,std::string>> buttons={
    {metrics_button_,"Timings"},{parameters_button_,"Params"},{stop_button_,"STOP"}};
  for(const auto& button:buttons) {
    cv::rectangle(canvas,button.first,button.second=="STOP"?cv::Scalar(50,50,190):track,-1);
    cv::putText(canvas,button.second,button.first.tl()+cv::Point(4,height*2/3),
      cv::FONT_HERSHEY_SIMPLEX,.40*ui,cv::Scalar(255,255,255),1,cv::LINE_AA);
  }
  return canvas;
}
}  // namespace line_detactor
