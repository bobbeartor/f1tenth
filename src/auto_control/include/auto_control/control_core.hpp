#ifndef AUTO_CONTROL__CONTROL_CORE_HPP_
#define AUTO_CONTROL__CONTROL_CORE_HPP_

#include <cstddef>
#include <optional>
#include <vector>

namespace auto_control
{

struct Point
{
  double x{0.0};
  double y{0.0};
};

struct OrderedPath
{
  std::vector<Point> points;
  std::vector<double> arc_m;
  double geometry_window_m{0.12};

  Point point(double arc) const;
  double heading(double arc) const;
  double curvature(double arc) const;
};

struct PathSpeedPlan
{
  std::vector<double> arc_m;
  std::vector<double> lower_speed_mps;
  std::vector<double> nominal_speed_mps;
  std::vector<double> upper_speed_mps;
  double target_lower_speed_mps{0.0};
  double target_nominal_speed_mps{0.0};
  double target_upper_speed_mps{0.0};
  double corner_start_m{-1.0};
  double corner_exit_m{-1.0};
  double corner_speed_mps{0.0};
};

PathSpeedPlan plan_path_speeds(
  const OrderedPath & path, double minimum_speed_mps, double maximum_speed_mps,
  double maximum_lateral_acceleration_mps2, double planning_deceleration_mps2,
  double planning_acceleration_mps2, double obstacle_speed_limit_mps,
  double response_distance_m, double corner_speed_band_width_mps,
  double sample_spacing_m = 0.05);

std::optional<OrderedPath> build_ordered_path(
  const std::vector<Point> & input, int minimum_points,
  double minimum_span_m, double minimum_x_m, double maximum_x_m,
  double maximum_gap_m, double geometry_window_m);

struct StanleyResult
{
  double steering_angle_rad{0.0};
  double cross_track_error_m{0.0};
  double heading_error_rad{0.0};
  bool direction_guard_used{false};
};

StanleyResult stanley_control(
  const OrderedPath & path, double speed_mps, double gain,
  double softening_speed_mps, double heading_lookahead_m,
  double maximum_steering_angle_rad, double corner_heading_threshold_rad,
  double corner_opposing_correction_ratio);

double representative_curvature(
  const OrderedPath & path, double lookahead_minimum_x_m,
  double lookahead_maximum_x_m, double percentile, int sample_count = 32);
double curvature_target_speed(
  double curvature_per_m, double maximum_lateral_acceleration_mps2,
  double minimum_speed_mps, double maximum_speed_mps);
double erpm_to_speed_mps(
  int measured_erpm, double wheel_diameter_m, int motor_pole_pairs,
  int motor_pinion_teeth, int spur_gear_teeth, int differential_pinion_teeth,
  int differential_ring_teeth, double direction_sign, double scale_correction);
double steering_angle_to_servo(
  double steering_angle_rad, double maximum_steering_angle_rad,
  double servo_left, double servo_center, double servo_right, bool inverted);
double speed_feedforward_duty(
  double target_speed_mps, double minimum_speed_mps, double maximum_speed_mps,
  double minimum_duty, double maximum_duty);
double move_toward(double value, double target, double maximum_step);
double clamp(double value, double minimum, double maximum);

// Called after the staged controller requests braking. Propulsion PID gains
// are independent. Hold current and actuator slew limits are applied by the node.
double staged_brake_current(
  double speed_error_mps, double missing_deceleration_mps2,
  double speed_gain, double deceleration_gain, double maximum_current_amps);

class SpeedPid
{
public:
  SpeedPid(
    double kp, double ki, double kd, double integral_limit,
    double minimum_duty, double maximum_duty);
  void reset();
  double update(
    double target_speed_mps, double current_speed_mps,
    double feedforward_duty, double dt_sec);
  // Undo this sample's integration if the downstream actuator limit prevents
  // output from moving in the direction requested by the speed error.
  void apply_output_limit(double requested, double applied);

private:
  double kp_;
  double ki_;
  double kd_;
  double integral_limit_;
  double minimum_duty_;
  double maximum_duty_;
  double integral_{0.0};
  double integral_before_update_{0.0};
  double latest_error_{0.0};
  std::optional<double> previous_error_;
};

struct BrakeConfig
{
  double entry_speed_error_mps;
  double exit_speed_error_mps;
  double minimum_vehicle_speed_mps;
  double minimum_brake_current_amps;
  double maximum_brake_current_amps;
  double current_gain_amps_per_mps;
  double rise_amps_per_sec;
  double fall_amps_per_sec;
};

class AutomaticBrakeProfile
{
public:
  explicit AutomaticBrakeProfile(const BrakeConfig & config);
  void reset();
  double update(double target_speed_mps, double current_speed_mps, double dt_sec);

private:
  BrakeConfig config_;
  double current_amps_{0.0};
  bool braking_requested_{false};
};

}  // namespace auto_control

#endif  // AUTO_CONTROL__CONTROL_CORE_HPP_
