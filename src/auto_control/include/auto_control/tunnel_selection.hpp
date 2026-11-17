#pragma once
// C++ port of auto_control_tunnel/{path_validation,ir_length_exit,selection,telemetry}.py.
// Templated on the lane message so the policy has no ROS dependency (LaneResult in production).
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace auto_control::tunnel
{

template<typename Msg>
std::int64_t stamp_ns(const Msg & message)
{
  return static_cast<std::int64_t>(message.header.stamp.sec) * 1000000000LL + message.header.stamp.nanosec;
}

inline bool close_rel(double a, double b, double rel = 1e-4)
{
  return std::abs(a - b) <= rel * std::max(std::abs(a), std::abs(b));
}

struct PathRules
{
  std::string frame_id{"front_axle_bev"};
  int minimum_points{8};
  double minimum_span{.12}, minimum_x{.05}, maximum_x{2.20}, maximum_gap{.15};
  double x_max{3.}, y_max{.6}, mpp{.01};

  void validate() const
  {
    const auto positive = [](double v) {return std::isfinite(v) && v > 0;};
    if (frame_id.empty() || minimum_points < 3 || !positive(minimum_span) || !positive(maximum_x) ||
      !positive(maximum_gap) || !positive(x_max) || !positive(y_max) || !positive(mpp) ||
      !std::isfinite(minimum_x) || !(0 <= minimum_x && minimum_x < maximum_x && maximum_x <= x_max))
    {
      throw std::invalid_argument("Invalid tunnel path geometry rules");
    }
  }
};

// Controller-compatible readiness: canvas, support and contiguous-path requirements.
template<typename Msg>
bool metric_path_ready(const Msg & m, const PathRules & rules)
{
  const double width = m.source_width, height = m.source_height;
  if (width <= 0 || height <= 0) {return false;}
  const double sx = m.centerline_bev_width_m / width, sy = m.centerline_bev_height_m / height;
  const std::size_t count = m.centerline_points.size();
  if (!m.centerline_valid || m.centerline_sample_limit_reached || m.state < 1 || m.state > 3 ||
    m.header.frame_id != rules.frame_id || !std::isfinite(sx) || !std::isfinite(sy) || sx <= 0 || sy <= 0 ||
    !close_rel(sx, rules.mpp) || !close_rel(sy, rules.mpp) || !close_rel(width * sx, 2 * rules.y_max) ||
    !close_rel(height * sy, rules.x_max) || count < 2 || count > 10000 || count != m.centerline_support.size())
  {
    return false;
  }
  std::vector<std::pair<double, double>> raw;
  raw.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const auto & p = m.centerline_points[i];
    const auto support = m.centerline_support[i];
    if (support < 1 || support > 4 || !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      std::abs(p.z) > 1e-6 || p.x < 0 || p.x >= width + m.padding_left + m.padding_right ||
      p.y < 0 || p.y >= height)
    {
      return false;
    }
    raw.emplace_back(rules.x_max - (p.y + .5) * sy, rules.y_max - (p.x - m.padding_left + .5) * sx);
  }
  std::vector<std::pair<double, double>> points;
  double span = 0.;
  for (const auto & p : raw) {
    if (!(rules.minimum_x <= p.first && p.first <= rules.maximum_x)) {
      if (!points.empty()) {break;}
      continue;
    }
    if (!points.empty()) {
      const double gap = std::hypot(p.first - points.back().first, p.second - points.back().second);
      if (gap < 1e-5) {continue;}
      if (gap > rules.maximum_gap) {break;}
      span += gap;
    }
    points.push_back(p);
  }
  return static_cast<int>(points.size()) >= rules.minimum_points && span >= rules.minimum_span;
}

class IrLengthExit
{
public:
  IrLengthExit(bool enabled = false, double ratio = .15, int confirm_frames = 3, double minimum_reference_m = .5)
  : enabled_(enabled), threshold_(ratio), confirm_frames_(confirm_frames), minimum_reference_(minimum_reference_m)
  {
    if (!std::isfinite(ratio) || ratio <= 0 || ratio >= 1 || confirm_frames < 1 || confirm_frames > 300 ||
      !std::isfinite(minimum_reference_m) || minimum_reference_m <= 0)
    {
      throw std::invalid_argument("Invalid IR length exit settings");
    }
    reset();
  }

  void reset()
  {
    window_.clear();
    reference_ = 0.;
    current_.reset();
    ratio_.reset();
    below_count_ = 0;
    last_stamp_.reset();
  }

  template<typename Msg>
  bool observe(const Msg & m, std::int64_t now_ns, double max_age, const PathRules & rules)
  {
    if (!enabled_) {return false;}
    const auto stamp = stamp_ns(m);
    if (last_stamp_ && stamp <= *last_stamp_) {return false;}
    const double age = (now_ns - stamp) / 1e9;
    const bool geometry = m.source_width > 0 && m.source_height > 0 &&
      close_rel(m.centerline_bev_width_m / m.source_width, rules.mpp) &&
      close_rel(m.centerline_bev_height_m / m.source_height, rules.mpp) &&
      close_rel(m.centerline_bev_height_m, rules.x_max) && close_rel(m.centerline_bev_width_m, 2 * rules.y_max);
    if (!(0 <= age && age <= max_age) || !m.observed_lane_length_valid || m.header.frame_id != rules.frame_id ||
      !geometry)
    {
      break_sequence();
      return false;
    }
    const double length = m.observed_lane_length_m;
    if (!std::isfinite(length) || length < 0) {
      break_sequence();
      return false;
    }
    if (last_stamp_ && (stamp - *last_stamp_) / 1e9 > max_age) {break_sequence();}
    last_stamp_ = stamp;
    current_ = length;
    window_.push_back(length);
    while (static_cast<int>(window_.size()) > confirm_frames_) {window_.pop_front();}
    // A single long false detection cannot increase the reference.
    if (static_cast<int>(window_.size()) == confirm_frames_) {
      const double stable = *std::min_element(window_.begin(), window_.end());
      if (stable >= minimum_reference_) {reference_ = std::max(reference_, stable);}
    }
    if (reference_ > 0) {ratio_ = length / reference_;} else {ratio_.reset();}
    const bool low = ratio_ && *ratio_ <= threshold_ + 1e-8;
    below_count_ = low ? below_count_ + 1 : 0;
    return below_count_ >= confirm_frames_;
  }

  bool enabled() const {return enabled_;}
  double threshold() const {return threshold_;}
  int confirm_frames() const {return confirm_frames_;}
  double minimum_reference() const {return minimum_reference_;}
  double reference() const {return reference_;}
  std::optional<double> current() const {return current_;}
  std::optional<double> ratio() const {return ratio_;}
  int below_count() const {return below_count_;}

private:
  void break_sequence()
  {
    window_.clear();
    below_count_ = 0;
    current_.reset();
    ratio_.reset();
  }

  bool enabled_;
  double threshold_;
  int confirm_frames_;
  double minimum_reference_;
  std::deque<double> window_;
  double reference_{0.};
  std::optional<double> current_, ratio_;
  int below_count_{0};
  std::optional<std::int64_t> last_stamp_;
};

// Prepare the target source while routing the current source, then switch once.
class LaneSelector
{
public:
  LaneSelector(double max_age, bool validation_mode, PathRules rules, IrLengthExit length_exit)
  : validation_mode(validation_mode), max_age(max_age), rules(std::move(rules)), ir_length_exit(length_exit)
  {
    if (!std::isfinite(max_age) || max_age <= 0) {throw std::invalid_argument("Invalid lane freshness setting");}
    this->rules.validate();
  }

  bool transitioning() const {return started_ns.has_value();}
  std::string compute() const {return transitioning() ? "BOTH" : selected;}
  std::string camera() const {return transitioning() ? "IR" : selected;}

  bool observe_tcn_state(const std::string & tcn_state, std::int64_t stamp, std::int64_t now_ns)
  {
    // Retained TCN inside reports must not undo a length-triggered exit.
    if (!ir_exit_latched_at) {return true;}
    const double age = (now_ns - stamp) / 1e9;
    if (tcn_state == "outside" && stamp > *ir_exit_latched_at && 0 <= age && age <= max_age) {
      ir_exit_latched_at.reset();
      return true;
    }
    return false;
  }

  bool event(const std::string & kind, std::int64_t stamp, std::int64_t now_ns)
  {
    const double age = (now_ns - stamp) / 1e9;
    if ((kind != "entry" && kind != "exit") || stamp <= last_event_ns ||
      (!validation_mode && !(0 <= age && age <= max_age)))
    {
      return false;
    }
    if (kind == "entry" && ir_exit_latched_at) {return false;}
    last_event_ns = stamp;
    const std::string target = kind == "entry" ? "IR" : "RGB";
    if (target == requested) {return false;}
    if (target == "IR") {ir_length_exit.reset();}
    transition_trigger.clear();
    requested = target;
    if (target == selected) {
      finish(now_ns, "reversed_request");
      return true;
    }
    started_ns = now_ns;
    reason = "waiting_target_path";
    state = target == "IR" ? "ENTERING" : "EXITING";
    return true;
  }

  // True when this message is the selected source and should be forwarded.
  template<typename Msg>
  bool lane(const std::string & source, const Msg & m, std::int64_t now_ns)
  {
    auto last = last_input_ns.find(source);
    if (last == last_input_ns.end()) {return false;}
    const auto stamp = stamp_ns(m);
    const double age = (now_ns - stamp) / 1e9;
    const bool fresh = (validation_mode && source == "IR" && !transitioning()) || (0 <= age && age <= max_age);
    if (source == "IR" && state == "INSIDE" && ir_length_exit.observe(m, now_ns, max_age, rules)) {
      if (event("exit", now_ns, now_ns)) {
        ir_exit_latched_at = now_ns;
        transition_trigger = reason = "ir_length_drop";
      }
    }
    if (stamp <= last->second || !fresh) {return false;}
    last->second = stamp;
    if (transitioning() && source == requested) {
      if (!metric_path_ready(m, rules)) {return false;}  // An unready target must not clear the current path.
      finish(now_ns, "first_valid_target");
    }
    return source == selected;
  }

  bool validation_mode;
  double max_age;
  PathRules rules;
  IrLengthExit ir_length_exit;
  std::string state{"OUTSIDE"}, selected{"RGB"}, requested{"RGB"}, reason{"steady"};
  std::string last_handover;  // JSON text for the log
  std::string transition_trigger;
  std::int64_t last_event_ns{0};
  std::map<std::string, std::int64_t> last_input_ns{{"RGB", 0}, {"IR", 0}};
  std::optional<std::int64_t> started_ns, ir_exit_latched_at;

private:
  void finish(std::int64_t now_ns, const std::string & why)
  {
    const double total_ms = started_ns ? (now_ns - *started_ns) / 1e6 : 0.;
    std::ostringstream out;
    out << "{\"target\": \"" << requested << "\", \"reason\": \"" << why << "\", \"total_ms\": " << total_ms;
    if (!transition_trigger.empty()) {out << ", \"trigger\": \"" << transition_trigger << "\"";}
    out << "}";
    last_handover = out.str();
    selected = requested;
    state = selected == "IR" ? "INSIDE" : "OUTSIDE";
    started_ns.reset();
    reason = why;
  }
};

// ---- Minimal JSON reader for the TCN report (flat object with one-level nested objects) ----
// Returns raw value text per top-level key: strings keep quotes, objects keep braces.
inline std::map<std::string, std::string> json_object(const std::string & text)
{
  std::map<std::string, std::string> out;
  std::size_t i = text.find('{');
  if (i == std::string::npos) {throw std::invalid_argument("not an object");}
  ++i;
  const auto skip = [&]() {while (i < text.size() && std::isspace(static_cast<unsigned char>(text[i]))) {++i;}};
  const auto read_string = [&]() {
      if (text[i] != '"') {throw std::invalid_argument("expected string");}
      const std::size_t start = i++;
      while (i < text.size() && text[i] != '"') {i += text[i] == '\\' ? 2 : 1;}
      if (i >= text.size()) {throw std::invalid_argument("unterminated string");}
      return text.substr(start, ++i - start);
    };
  while (true) {
    skip();
    if (i >= text.size()) {throw std::invalid_argument("unterminated object");}
    if (text[i] == '}') {break;}
    if (text[i] == ',') {++i; continue;}
    const std::string key = read_string();
    skip();
    if (i >= text.size() || text[i] != ':') {throw std::invalid_argument("expected colon");}
    ++i;
    skip();
    const std::size_t start = i;
    if (text[i] == '"') {
      read_string();
    } else if (text[i] == '{' || text[i] == '[') {
      int depth = 0;
      for (; i < text.size(); ++i) {
        if (text[i] == '"') {read_string(); --i; continue;}
        if (text[i] == '{' || text[i] == '[') {++depth;}
        if (text[i] == '}' || text[i] == ']') {if (--depth == 0) {++i; break;}}
      }
    } else {
      while (i < text.size() && text[i] != ',' && text[i] != '}') {++i;}
    }
    std::string value = text.substr(start, i - start);
    while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back()))) {value.pop_back();}
    out[key.substr(1, key.size() - 2)] = value;
  }
  return out;
}

inline std::optional<std::string> json_string(const std::map<std::string, std::string> & o, const std::string & k)
{
  const auto found = o.find(k);
  if (found == o.end() || found->second.size() < 2 || found->second.front() != '"') {return std::nullopt;}
  return found->second.substr(1, found->second.size() - 2);
}

inline std::optional<std::int64_t> json_int(const std::map<std::string, std::string> & o, const std::string & k)
{
  const auto found = o.find(k);
  if (found == o.end() || found->second.empty() ||
    found->second.find_first_not_of("-0123456789") != std::string::npos)
  {
    return std::nullopt;
  }
  return std::stoll(found->second);
}

inline std::optional<double> json_double(const std::map<std::string, std::string> & o, const std::string & k)
{
  const auto found = o.find(k);
  if (found == o.end() || found->second.empty() || found->second == "null" || found->second.front() == '"' ||
    found->second.front() == '{' || found->second == "true" || found->second == "false")
  {
    return std::nullopt;
  }
  const double value = std::stod(found->second);
  return std::isfinite(value) ? std::optional<double>(value) : std::nullopt;
}

// Port of TunnelPreviewTelemetry: three HUD lines for /tunnel/preview_status.
class PreviewTelemetry
{
public:
  void update(const std::map<std::string, std::string> & report, double now)
  {
    const auto stamp = json_int(report, "source_stamp_ns");
    if (!stamp) {return;}
    const std::string status = json_string(report, "status").value_or("WAITING");
    const bool reset = (last_stamp_ && *stamp < *last_stamp_) ||
      (status.rfind("LIVE", 0) == 0 && status_.rfind("LIVE", 0) != 0);
    if (reset) {infer_sum_ms_ = 0.; infer_count_ = 0;}
    const bool duplicate = last_stamp_ && *stamp == *last_stamp_;
    last_stamp_ = stamp;
    received_at_ = now;
    status_ = status;
    const auto scores = report.find("scores");
    if (scores != report.end() && !scores->second.empty() && scores->second.front() == '{') {
      const auto values = json_object(scores->second);
      entry_ = json_double(values, "entry");
      exit_ = json_double(values, "exit");
    }
    if (status.rfind("LIVE", 0) != 0 || duplicate) {return;}
    const auto ms = json_double(report, "inference_wall_ms");
    if (ms && *ms > 0) {infer_sum_ms_ += *ms; ++infer_count_;}
  }

  std::string text(const std::string & state, double now) const
  {
    const bool fresh = received_at_ && 0 <= now - *received_at_ && now - *received_at_ <= .5;
    std::string status = !received_at_ ? "WAITING" : !fresh ? "STALE" : status_.substr(0, status_.find(' '));
    const auto fixed = [](double v, int digits) {
        std::ostringstream out;
        out.setf(std::ios::fixed);
        out.precision(digits);
        out << v;
        return out.str();
      };
    std::string fps = "--";
    if (fresh && status_.rfind("LIVE", 0) == 0 && infer_count_ && infer_sum_ms_ > 0) {
      fps = fixed(1000.0 * infer_count_ / infer_sum_ms_, 1);
    }
    const std::string entry = fresh && entry_ ? fixed(*entry_, 2) : "--";
    const std::string exit = fresh && exit_ ? fixed(*exit_, 2) : "--";
    return "Tunnel: " + state + "\nTCN avg " + fps + "FPS\nTCN " + status + " E:" + entry + " X:" + exit;
  }

private:
  std::optional<double> received_at_;
  std::optional<std::int64_t> last_stamp_;
  std::string status_{"WAITING"};
  std::optional<double> entry_, exit_;
  double infer_sum_ms_{0.};
  int infer_count_{0};
};

}  // namespace auto_control::tunnel
