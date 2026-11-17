# manual_control

Reads SDL-standard Bluetooth controller input from `/joy` and sends manual
actuator commands.

`auto_handover_node` reuses this manual controller in `auto_drive.launch.py`.
It initially forwards private automatic actuator topics and ignores manual driving
inputs. X (SDL button 2) permanently blocks automatic commands for this run and
starts electrical braking. Fresh stationary ERPM for 0.5 seconds plus released
X/RT/LT/RB and centered steering allow manual control. Only joystick frames received
after handover can drive the vehicle. Missing telemetry or joystick input keeps
the node braking; there is no timed fallback into manual mode. Perception and BEV
preview continue throughout. The normal `actuator_commander_node` remains a
standalone manual controller without this automatic startup mode.

Published topics:

```text
/manual/current_duty      std_msgs/msg/Float32
/manual/current_brake_current std_msgs/msg/Float32  amperes
/vesc/duty                std_msgs/msg/Float32  -1.0 to 1.0
/vesc/brake_current       std_msgs/msg/Float32  non-negative amperes
/vesc/measured_erpm       std_msgs/msg/Int32  VESC-estimated ERPM
/manual/gear              std_msgs/msg/String
```

Current mapping:

```text
RT axis 5 (0.0 released, 1.0 pressed) -> acceleration
LT axis 4 (0.0 released, 1.0 pressed) -> VESC electrical brake current
left stick X axis 0 -> steering angle
RB button 10 -> forward/reverse toggle (reserved until duty 0 while moving)
```

`actuator_commander_node` starts stopped in forward gear. RT increases the
command duty, LT takes priority and switches the VESC to brake-current mode,
and RB toggles forward/reverse. If RB is pressed while duty is non-zero or
LT braking is active, the change is reserved and engages automatically in the
control cycle where command duty reaches 0; pressing RB again cancels the
reservation. A pedal input timeout or a handover into MANUAL clears it. With the configured
`immediate_stop_on_accelerator_release: false`,
releasing RT follows the coast deceleration rate. Current gear and command duty are
published on `/manual/gear` and `/manual/current_duty`; the ramped electrical
brake command is published on `/manual/current_brake_current`. The VESC node publishes measured ERPM on
`/vesc/measured_erpm` and logs target duty and measured ERPM together.

`actuator_commander_node` reads all four controls directly from the same `/joy`
frame. The former intermediate converter and four manual input topics are not
used by `manual_drive.launch.py`. RB changes gear only on its rising edge, so
holding the button does not repeat the change.

After startup and after any pedal input timeout (for example a Bluetooth
disconnect), RT is ignored until it has been seen released once, so a trigger
value still reported as pressed on reconnect cannot start the car. LT braking and
steering are not locked.

The manual controller runs at 80 Hz. Its configured duty limits and ramps are:

Controller state and VESC command topics use `KEEP_LAST(1)` best-effort QoS.
Holding RT or a steering input therefore replaces the pending state instead of
accumulating old commands. When `/joy` stops, the 0.30-second watchdog sends
duty 0 and centers steering. When serial I/O is temporarily delayed, the VESC
node processes only the newest waiting duty/ERPM/servo command.

```yaml
forward_max_duty: 0.10
reverse_max_duty: 0.08
start_duty: 0.05
reverse_start_duty: 0.05
acceleration_duty_per_sec: 0.03
coast_deceleration_duty_per_sec: 0.04
maximum_brake_current_amps: 8.0
brake_current_rise_amps_per_sec: 16.0
brake_current_fall_amps_per_sec: 32.0
immediate_stop_on_accelerator_release: false
```

`coast_deceleration_duty_per_sec` is used only when
`immediate_stop_on_accelerator_release` is `false`. Full LT reaches 8 A in
approximately 0.5 seconds and releases it in approximately 0.25 seconds.
While any brake current remains, propulsion duty is cleared and not published.
The 8 A ROS limit is intentionally far below the supplied VESC motor-brake and
battery-regen limits; first powered checks must still be done at low speed.

## ROI VFH GUI 전환

`vfh_handover_node`는 `roi_vfh_test preview.launch.py`용 양방향 전환 어댑터다.
기존 수동 프로필과 `HandoverState`의 최신 ERPM·정지 유지·중립 확인을 재사용한다.
요청은 `id|MANUAL|stop_current` 또는 `id|AUTO|stop_current`, ACK는 같은 id와 실제 상태를 전송한다.
이전 두 필드 요청도 지원한다. 선택 제동 전류는 수동 프로필의 최대 전류 이하여야 한다.
실제 actuator 출력은 이 노드 하나만 소유하며 자동 명령은 volatile 전용 입력으로 받는다.
Stop으로 MANUAL, Start로 AUTO 복귀하며 X는 모드를 바꾸지 않는다.
기존 `auto_handover_node`의 auto_drive 단방향 X 전환 정책은 유지한다.
