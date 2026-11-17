#include "auto_control/control_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace auto_control
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
double norm(const Point & point) {return std::hypot(point.x, point.y);}
Point operator+(const Point & a, const Point & b) {return {a.x + b.x, a.y + b.y};}
Point operator-(const Point & a, const Point & b) {return {a.x - b.x, a.y - b.y};}
Point operator*(const Point & point, double scale) {return {point.x * scale, point.y * scale};}
double cross(const Point & a, const Point & b) {return a.x * b.y - a.y * b.x;}

double percentile(std::vector<double> values, double requested)
{
  if (values.empty()) {return 0.0;}
  std::sort(values.begin(), values.end());
  const double position = (values.size() - 1U) * clamp(requested, 0.0, 100.0) / 100.0;
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  const double fraction = position - static_cast<double>(lower);
  return values[lower] * (1.0 - fraction) + values[upper] * fraction;
}
}  // namespace

double clamp(double value, double minimum, double maximum)
{
  return std::max(minimum, std::min(maximum, value));
}

double move_toward(double value, double target, double maximum_step)
{
  maximum_step = std::max(0.0, maximum_step);
  if (value < target) {return std::min(target, value + maximum_step);}
  if (value > target) {return std::max(target, value - maximum_step);}
  return target;
}

Point OrderedPath::point(double requested_arc) const
{
  if (points.empty()) {return {};}
  if (requested_arc <= arc_m.front()) {return points.front();}
  if (requested_arc >= arc_m.back()) {return points.back();}
  const auto upper = std::upper_bound(arc_m.begin(), arc_m.end(), requested_arc);
  const auto index = static_cast<std::size_t>(upper - arc_m.begin());
  const double amount = (requested_arc - arc_m[index - 1U]) /
    std::max(arc_m[index] - arc_m[index - 1U], 1.0e-12);
  return points[index - 1U] + (points[index] - points[index - 1U]) * amount;
}

double OrderedPath::heading(double requested_arc) const
{
  const double half = geometry_window_m / 2.0;
  double lower = std::max(0.0, requested_arc - half);
  double upper = std::min(arc_m.back(), requested_arc + half);
  if (upper - lower < half) {
    lower = std::max(0.0, upper - geometry_window_m);
    upper = std::min(arc_m.back(), lower + geometry_window_m);
  }
  const Point delta = point(upper) - point(lower);
  return std::atan2(delta.y, delta.x);
}

double OrderedPath::curvature(double requested_arc) const
{
  const double window = std::min(geometry_window_m, arc_m.back() / 2.0);
  if (window <= 1.0e-9) {return 0.0;}
  const double middle_arc = clamp(requested_arc, window, arc_m.back() - window);
  const Point first = point(middle_arc - window);
  const Point middle = point(middle_arc);
  const Point last = point(middle_arc + window);
  const Point a = middle - first;
  const Point b = last - first;
  const double denominator = norm(a) * norm(last - middle) * norm(b);
  return denominator > 1.0e-9 ? 2.0 * cross(a, b) / denominator : 0.0;
}

std::optional<OrderedPath> build_ordered_path(
  const std::vector<Point> & input, int minimum_points,
  double minimum_span_m, double minimum_x_m, double maximum_x_m,
  double maximum_gap_m, double geometry_window_m)
{
  OrderedPath path;
  path.geometry_window_m = geometry_window_m;
  path.points.reserve(input.size());
  for (const auto & point : input) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      throw std::invalid_argument("ordered centerline contains nonfinite coordinates");
    }
    if (point.x < minimum_x_m || point.x > maximum_x_m) {
      if (!path.points.empty()) {break;}
      continue;
    }
    if (!path.points.empty()) {
      const double gap = norm(point - path.points.back());
      if (gap < 1.0e-5) {continue;}
      if (gap > maximum_gap_m) {break;}
    }
    path.points.push_back(point);
  }
  if (path.points.size() < static_cast<std::size_t>(minimum_points)) {return std::nullopt;}
  path.arc_m.resize(path.points.size(), 0.0);
  for (std::size_t index = 1; index < path.points.size(); ++index) {
    path.arc_m[index] = path.arc_m[index - 1U] + norm(path.points[index] - path.points[index - 1U]);
  }
  if (path.arc_m.back() < minimum_span_m) {return std::nullopt;}
  return path;
}

StanleyResult stanley_control(
  const OrderedPath & path, double speed_mps, double gain,
  double softening_speed_mps, double heading_lookahead_m,
  double maximum_steering_angle_rad, double corner_heading_threshold_rad,
  double corner_opposing_correction_ratio)
{
  double nearest_squared = std::numeric_limits<double>::infinity();
  double cross_track_error = 0.0;
  double closest_arc = 0.0;
  for (std::size_t index = 0; index + 1U < path.points.size(); ++index) {
    const Point start = path.points[index];
    const Point delta = path.points[index + 1U] - start;
    const double length_squared = delta.x * delta.x + delta.y * delta.y;
    if (length_squared <= 1.0e-12) {continue;}
    const double projection = clamp(
      -(start.x * delta.x + start.y * delta.y) / length_squared, 0.0, 1.0);
    const Point closest = start + delta * projection;
    const double squared = closest.x * closest.x + closest.y * closest.y;
    if (squared >= nearest_squared) {continue;}
    nearest_squared = squared;
    const double length = std::sqrt(length_squared);
    cross_track_error = (-delta.y * closest.x + delta.x * closest.y) / length;
    closest_arc = path.arc_m[index] + projection * length;
  }
  const double heading_arc = std::min(path.arc_m.back(),
    closest_arc + std::max(0.0, heading_lookahead_m));
  const double heading_error = path.heading(heading_arc);
  double correction = std::atan2(
    std::max(0.0, gain) * cross_track_error,
    std::max(0.0, std::abs(speed_mps)) + std::max(0.0, softening_speed_mps));
  bool guard = false;
  if (std::abs(heading_error) >= std::max(0.0, corner_heading_threshold_rad) &&
    heading_error * correction < 0.0)
  {
    const double maximum_opposing = std::abs(heading_error) *
      clamp(corner_opposing_correction_ratio, 0.0, 0.99);
    const double limited = clamp(correction, -maximum_opposing, maximum_opposing);
    guard = limited != correction;
    correction = limited;
  }
  return {
    clamp(heading_error + correction, -maximum_steering_angle_rad, maximum_steering_angle_rad),
    cross_track_error, heading_error, guard};
}

double representative_curvature(
  const OrderedPath & path, double lookahead_minimum_x_m,
  double lookahead_maximum_x_m, double requested_percentile, int sample_count)
{
  std::vector<double> all;
  std::vector<double> selected;
  for (int index = 0; index < std::max(2, sample_count); ++index) {
    const double arc = path.arc_m.back() * index / std::max(1, sample_count - 1);
    const double value = std::abs(path.curvature(arc));
    if (!std::isfinite(value)) {throw std::invalid_argument("nonfinite ordered path curvature");}
    all.push_back(value);
    const double x = path.point(arc).x;
    if (x >= lookahead_minimum_x_m && x <= lookahead_maximum_x_m) {selected.push_back(value);}
  }
  return percentile(selected.empty() ? all : selected, requested_percentile);
}

double curvature_target_speed(
  double curvature_per_m, double maximum_lateral_acceleration_mps2,
  double minimum_speed_mps, double maximum_speed_mps)
{
  const double curvature = std::abs(curvature_per_m);
  if (curvature <= 1.0e-6) {return maximum_speed_mps;}
  const double safe = std::sqrt(std::max(0.0, maximum_lateral_acceleration_mps2) / curvature);
  return clamp(safe, minimum_speed_mps, maximum_speed_mps);
}

PathSpeedPlan plan_path_speeds(
  const OrderedPath & path, double minimum_speed_mps, double maximum_speed_mps,
  double maximum_lateral_acceleration_mps2, double planning_deceleration_mps2,
  double planning_acceleration_mps2, double obstacle_speed_limit_mps,
  double response_distance_m, double corner_speed_band_width_mps,
  double sample_spacing_m)
{
  if (path.arc_m.empty() || !std::isfinite(path.arc_m.back()) ||
    path.arc_m.back() <= 0.0 || path.arc_m.back() > 25.0 ||
    !std::isfinite(minimum_speed_mps) || minimum_speed_mps < 0.0 ||
    minimum_speed_mps > maximum_speed_mps ||
    maximum_speed_mps <= 0.0 || maximum_lateral_acceleration_mps2 <= 0.0 ||
    planning_deceleration_mps2 <= 0.0 || planning_acceleration_mps2 <= 0.0 ||
    corner_speed_band_width_mps < 0.0 || sample_spacing_m <= 0.0)
  {throw std::invalid_argument("invalid path speed planning input");}
  PathSpeedPlan plan;
  const double length = path.arc_m.back();
  const auto segments = std::min<std::size_t>(512U,
    std::max<std::size_t>(1U, static_cast<std::size_t>(std::ceil(length / sample_spacing_m))));
  plan.arc_m.reserve(segments + 1U);
  plan.lower_speed_mps.reserve(segments + 1U);
  plan.nominal_speed_mps.reserve(segments + 1U);
  plan.upper_speed_mps.reserve(segments + 1U);
  std::vector<double> curve_limits;
  curve_limits.reserve(segments + 1U);
  for (std::size_t index = 0; index <= segments; ++index) {
    const double arc = length * static_cast<double>(index) / static_cast<double>(segments);
    const double curvature = std::abs(path.curvature(arc));
    if (!std::isfinite(curvature)) {throw std::invalid_argument("nonfinite path curvature");}
    const double curve_limit = curvature > 1.0e-9 ?
      clamp(std::sqrt(maximum_lateral_acceleration_mps2 / curvature),
      minimum_speed_mps, maximum_speed_mps) : maximum_speed_mps;
    plan.arc_m.push_back(arc);
    curve_limits.push_back(curve_limit);
  }
  // Dilate a local curvature limit by one sample so a narrow corner is not
  // missed between the fixed spatial samples.
  bool first_corner_finished = false;
  for (std::size_t index = 0; index < curve_limits.size(); ++index) {
    double limit = curve_limits[index];
    if (index > 0U) {limit = std::min(limit, curve_limits[index - 1U]);}
    if (index + 1U < curve_limits.size()) {limit = std::min(limit, curve_limits[index + 1U]);}
    plan.upper_speed_mps.push_back(std::min(limit, obstacle_speed_limit_mps));
    if (plan.corner_exit_m >= 0.0 &&
      plan.arc_m[index] - plan.corner_exit_m > 0.15)
    {first_corner_finished = true;}
    if (!first_corner_finished && limit < maximum_speed_mps - 0.03) {
      if (plan.corner_start_m < 0.0) {plan.corner_start_m = plan.arc_m[index];}
      plan.corner_exit_m = plan.arc_m[index];
      plan.corner_speed_mps = plan.corner_speed_mps > 0.0 ?
        std::min(plan.corner_speed_mps, limit) : limit;
    }
  }
  // Keep each point's local limit: the tightest part of a corner must not
  // impose its speed on the entire entry. The backward pass below still
  // reserves enough distance to reach every future curvature restriction.
  // Future restrictions propagate backward to a feasible entry speed.
  // The camera horizon is not a stop point; stale/missing path watchdogs own
  // that safety action. Only real curve and obstacle limits are propagated.
  for (std::size_t index = plan.upper_speed_mps.size() - 1U; index > 0U; --index) {
    const double ds = plan.arc_m[index] - plan.arc_m[index - 1U];
    plan.upper_speed_mps[index - 1U] = std::min(plan.upper_speed_mps[index - 1U],
      std::sqrt(plan.upper_speed_mps[index] * plan.upper_speed_mps[index] +
      2.0 * planning_deceleration_mps2 * ds));
  }
  // The exit profile must respect acceleration as well as the local limits.
  for (std::size_t index = 1U; index < plan.upper_speed_mps.size(); ++index) {
    const double ds = plan.arc_m[index] - plan.arc_m[index - 1U];
    plan.upper_speed_mps[index] = std::min(plan.upper_speed_mps[index],
      std::sqrt(plan.upper_speed_mps[index - 1U] * plan.upper_speed_mps[index - 1U] +
      2.0 * planning_acceleration_mps2 * ds));
  }
  const double unrestricted = std::min(maximum_speed_mps, obstacle_speed_limit_mps);
  for (const double upper_speed : plan.upper_speed_mps) {
    const bool curve_profile = upper_speed < unrestricted - 0.03;
    // Floor the entire corner band, not just its upper edge. A lower
    // obstacle limit still takes precedence over the configured cruise floor.
    const double lower_speed = curve_profile ?
      std::max(std::min(minimum_speed_mps, upper_speed),
      upper_speed - corner_speed_band_width_mps) : upper_speed;
    plan.lower_speed_mps.push_back(lower_speed);
    plan.nominal_speed_mps.push_back(0.5 * (lower_speed + upper_speed));
  }
  const double probe = clamp(response_distance_m, 0.0, length);
  const auto upper = std::lower_bound(plan.arc_m.begin(), plan.arc_m.end(), probe);
  const auto index = static_cast<std::size_t>(upper - plan.arc_m.begin());
  plan.target_lower_speed_mps = std::min(
    plan.lower_speed_mps.front(), plan.lower_speed_mps[index]);
  plan.target_nominal_speed_mps = std::min(
    plan.nominal_speed_mps.front(), plan.nominal_speed_mps[index]);
  plan.target_upper_speed_mps = std::min(
    plan.upper_speed_mps.front(), plan.upper_speed_mps[index]);
  return plan;
}

double erpm_to_speed_mps(
  int measured_erpm, double wheel_diameter_m, int motor_pole_pairs,
  int motor_pinion_teeth, int spur_gear_teeth, int differential_pinion_teeth,
  int differential_ring_teeth, double direction_sign, double scale_correction)
{
  const double gear_ratio = static_cast<double>(spur_gear_teeth) / motor_pinion_teeth *
    static_cast<double>(differential_ring_teeth) / differential_pinion_teeth;
  return measured_erpm * (direction_sign >= 0.0 ? 1.0 : -1.0) * scale_correction *
    kPi * wheel_diameter_m / (60.0 * motor_pole_pairs * gear_ratio);
}

double steering_angle_to_servo(
  double steering_angle_rad, double maximum_steering_angle_rad,
  double servo_left, double servo_center, double servo_right, bool inverted)
{
  if (inverted) {steering_angle_rad = -steering_angle_rad;}
  const double normalized = clamp(steering_angle_rad / maximum_steering_angle_rad, -1.0, 1.0);
  return normalized >= 0.0 ?
    servo_center + (servo_left - servo_center) * normalized :
    servo_center + (servo_right - servo_center) * -normalized;
}

double speed_feedforward_duty(
  double target_speed_mps, double minimum_speed_mps, double maximum_speed_mps,
  double minimum_duty, double maximum_duty)
{
  if (maximum_speed_mps <= minimum_speed_mps) {return minimum_duty;}
  const double amount = clamp(
    (target_speed_mps - minimum_speed_mps) / (maximum_speed_mps - minimum_speed_mps), 0.0, 1.0);
  return minimum_duty + (maximum_duty - minimum_duty) * amount;
}

double staged_brake_current(
  double speed_error_mps, double missing_deceleration_mps2,
  double speed_gain, double deceleration_gain, double maximum_current_amps)
{
  return maximum_current_amps * clamp(
    speed_gain * std::max(0.0, speed_error_mps) +
    deceleration_gain * std::max(0.0, missing_deceleration_mps2), 0.0, 1.0);
}

SpeedPid::SpeedPid(
  double kp, double ki, double kd, double integral_limit,
  double minimum_duty, double maximum_duty)
: kp_(kp), ki_(ki), kd_(kd), integral_limit_(std::abs(integral_limit)),
  minimum_duty_(minimum_duty), maximum_duty_(maximum_duty) {}

void SpeedPid::reset()
{
  integral_ = 0.0;
  integral_before_update_ = latest_error_ = 0.0;
  previous_error_.reset();
}

double SpeedPid::update(
  double target_speed_mps, double current_speed_mps,
  double feedforward_duty, double dt_sec)
{
  dt_sec = std::max(1.0e-4, dt_sec);
  const double error = target_speed_mps - current_speed_mps;
  integral_before_update_ = integral_;
  latest_error_ = error;
  const double derivative = previous_error_ ? (error - *previous_error_) / dt_sec : 0.0;
  const double candidate_integral = clamp(
    integral_ + error * dt_sec, -integral_limit_, integral_limit_);
  const double candidate = feedforward_duty + kp_ * error +
    ki_ * candidate_integral + kd_ * derivative;
  const double output = clamp(candidate, minimum_duty_, maximum_duty_);
  const bool saturated_high = candidate > maximum_duty_ && error > 0.0;
  const bool saturated_low = candidate < minimum_duty_ && error < 0.0;
  if (!saturated_high && !saturated_low) {integral_ = candidate_integral;}
  previous_error_ = error;
  return output;
}

void SpeedPid::apply_output_limit(double requested, double applied)
{
  if (latest_error_ * (requested - applied) > 1.0e-9) {
    integral_ = integral_before_update_;
  }
}

AutomaticBrakeProfile::AutomaticBrakeProfile(const BrakeConfig & config) : config_(config) {}

void AutomaticBrakeProfile::reset()
{
  current_amps_ = 0.0;
  braking_requested_ = false;
}

double AutomaticBrakeProfile::update(
  double target_speed_mps, double current_speed_mps, double dt_sec)
{
  target_speed_mps = std::isfinite(target_speed_mps) ? std::max(0.0, target_speed_mps) : 0.0;
  current_speed_mps = std::isfinite(current_speed_mps) ? std::max(0.0, current_speed_mps) : 0.0;
  dt_sec = std::isfinite(dt_sec) ? std::max(0.0, dt_sec) : 0.0;
  const double error = current_speed_mps - target_speed_mps;
  if (current_speed_mps <= config_.minimum_vehicle_speed_mps) {
    braking_requested_ = false;
  } else if (braking_requested_) {
    braking_requested_ = error > config_.exit_speed_error_mps + 1.0e-9;
  } else {
    braking_requested_ = error >= config_.entry_speed_error_mps - 1.0e-9;
  }
  double target = 0.0;
  if (braking_requested_) {
    target = clamp(config_.current_gain_amps_per_mps * std::max(0.0, error),
      config_.minimum_brake_current_amps, config_.maximum_brake_current_amps);
  }
  const double rate = target > current_amps_ ? config_.rise_amps_per_sec : config_.fall_amps_per_sec;
  current_amps_ = move_toward(current_amps_, target, rate * dt_sec);
  return current_amps_;
}

}  // namespace auto_control
