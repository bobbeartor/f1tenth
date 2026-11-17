# Metric path in front_axle_bev (+X forward, +Y left), NOT padded pixels.
# header is the source lane capture, depth_stamp the matched depth capture.
std_msgs/Header header
builtin_interfaces/Time depth_stamp
bool control_ready
# valid denotes usable path data; deformation_only does not certify collision clearance.
bool valid
# Shape-only experiment; receiver must use the same startup mode.
bool deformation_only
# Fresh matched depth contains an obstacle inside the lane, independent of path validity.
bool lane_obstacle_detected
# Use the original centerline (in deformation mode this is not an obstacle-clear certificate).
# In this mode points is empty; false with valid=true means an avoidance path.
bool follow_centerline
string status
# Legacy field name: reference cruise speed for display, NOT a control limit.
float64 speed_limit_mps
float64 max_curvature_per_m
geometry_msgs/Point32[] points
