# manual_control

Translates raw joystick input from `/joy` into normalized manual-control topics.

Published topics:

```text
/manual/accelerator       std_msgs/msg/Float32  0.0 to 1.0
/manual/brake             std_msgs/msg/Float32  0.0 to 1.0
/manual/steering          std_msgs/msg/Float32  -1.0 to 1.0
/manual/gear_toggle       std_msgs/msg/Bool
/manual/current_duty      std_msgs/msg/Float32
/vesc/duty                std_msgs/msg/Float32  -1.0 to 1.0
/vesc/measured_erpm       std_msgs/msg/Int32  VESC-estimated ERPM
/manual/gear              std_msgs/msg/String
/manual/controller_debug  std_msgs/msg/String  JSON controller state
```

Current mapping:

```text
RT trigger axis -> /manual/accelerator
LT trigger axis -> /manual/brake
left stick X -> /manual/steering
Y button -> /manual/gear_toggle
```

`actuator_commander_node` starts stopped in forward gear. RT increases the
command duty, releasing both pedals coasts toward zero, LT reduces duty toward
zero more strongly, and Y toggles forward/reverse while stopped. Current gear
and command duty are
published on `/manual/gear` and
`/manual/current_duty`. The VESC node publishes measured ERPM on
`/vesc/measured_erpm` and logs target duty and measured ERPM together.

The manual controller runs at 80 Hz. Its configured duty limits and ramps are:

```yaml
forward_max_duty: 0.07
reverse_max_duty: 0.04
start_duty: 0.02
acceleration_duty_per_sec: 0.01
coast_deceleration_duty_per_sec: 0.02
brake_duty_per_sec: 0.07
```

LT does not request VESC brake current; it ramps the duty command toward zero.
