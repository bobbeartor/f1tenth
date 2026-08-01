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
RB button -> /manual/gear_toggle
```

`actuator_commander_node` starts stopped in forward gear. RT increases the
command duty, LT reduces duty toward zero, and RB toggles forward/reverse while
stopped. With the default `immediate_stop_on_accelerator_release: true`,
releasing RT publishes duty 0 on the next 80 Hz control tick instead of keeping
an accumulated command alive. Current gear and command duty are
published on `/manual/gear` and
`/manual/current_duty`. The VESC node publishes measured ERPM on
`/vesc/measured_erpm` and logs target duty and measured ERPM together.

RB is converted from a repeated button state into one reliable rising-edge
event. The first press is handled immediately, holding the button does not
repeat the gear change, and edges within `button_debounce_sec: 0.15` are
discarded as contact bounce.

The manual controller runs at 80 Hz. Its configured duty limits and ramps are:

Controller state and VESC command topics use `KEEP_LAST(1)` best-effort QoS.
Holding RT or a steering input therefore replaces the pending value instead of
accumulating old commands. RB uses a separate reliable event QoS so a short press
is not overwritten by its release. When serial I/O is temporarily delayed, the
VESC node processes only the newest waiting duty/ERPM/servo command.

```yaml
forward_max_duty: 0.10
reverse_max_duty: 0.08
start_duty: 0.05
reverse_start_duty: 0.05
acceleration_duty_per_sec: 0.03
coast_deceleration_duty_per_sec: 0.005
brake_duty_per_sec: 0.07
immediate_stop_on_accelerator_release: true
button_debounce_sec: 0.15
```

`coast_deceleration_duty_per_sec` is used only when
`immediate_stop_on_accelerator_release` is `false`. LT does not request VESC
brake current; it ramps the duty command toward zero.
