# vesc_bridge

`vesc_bridge` connects ROS 2 command and telemetry topics to a VESC over a
serial device. Both the VESC USB CDC port and a USB-to-UART adapter use the
same VESC packet protocol; only the serial device path and matching baudrate
change.

The installed executable is `vesc_bridge_node`. Runtime parameters are kept
in `vehicle_bringup/config/vesc_config.yaml`.

## Topics

- Subscribes: `/vesc/duty`, `/vesc/brake_current`, `/vesc/erpm`,
  `/vesc/servo_position`
- Publishes: `/vesc/measured_erpm`, `/vesc/connected`

Measured ERPM is selectively requested at 80 Hz by a dedicated UART worker.
Duty/brake-current/ERPM and servo callbacks only replace the newest pending state, so a slow
telemetry response does not block the ROS executor or replay superseded control
commands. The three motor command modes are mutually exclusive in the UART
worker. Brake current is clamped to the configured non-negative
`max_brake_current_amps` before `COMM_SET_CURRENT_BRAKE` is sent. Drive and
steering writes are handled before telemetry. The periodic
status log reports requested/actual telemetry Hz, UART round-trip average/max,
and failed queries for on-vehicle verification.

CRC/framing errors, incomplete values and response timeouts discard only that
ERPM sample while the port is open and the last valid response is within
`telemetry_timeout_sec` (default 0.15 s). Serial open/read/write exceptions,
including a failed telemetry request write, mark the link unavailable immediately.
The result-drain timer also checks freshness even if the worker produces no result;
continued response failures or silence expire the link. Only a fresh successful
telemetry response restores a failed connection; command writes cannot restore it.
With telemetry disabled, this telemetry freshness watchdog is disabled too.

No extrapolated or repeated ERPM is published. `auto_control` holds its last
filtered speed between valid samples and its existing `erpm_timeout_sec` still
stops control. A dropped sample never refreshes either deadline. Request start
time bounds sample age, and queued responses already older than the bridge
deadline are discarded. This avoids treating an acceleration prediction during
braking as a new measurement. The two deadlines belong to different nodes;
keep both at 0.15 s unless changing the intended freshness policy.

The 80 Hz setting matches the camera's nominal publish rate but does not
phase-lock UART replies to image exposure. Each transaction records its host
monotonic start, finish, and midpoint so a later speed/IMU fusion stage can
align samples by time instead of assuming matching sequence numbers.

The port defaults to the Jetson UART device `/dev/ttyTHS1`. Override the
`port` parameter when using a different serial device. Configure UART for
3.3 V logic, cross TX/RX, connect GND, and do not connect the UART power pin
to the VESC.

## UART write diagnostics and port ownership

The default write timeout is 50ms (`write_timeout: 0.05`); serial response timeout
remains 20ms. Telemetry stays at 80Hz and the telemetry/ERPM freshness deadlines
remain 150ms. A write failure still marks the link unavailable immediately and
automatic driving still requires a new START approval after recovery.

`serial_exclusive: true` requests pyserial locking and Linux TIOCEXCL on the
production serial connection. Startup/reopen also inspects accessible `/proc`
file descriptors for other port holders and reports PID/process names. Processes
hidden by permissions cannot be enumerated. This does not terminate other users
or disable system serial consoles. Set `serial_exclusive: false` only when
explicitly opting out of the checks and lock.

Logs include lifetime packet write attempts/failures/timeouts, complete bytes
sent and average/max write-call duration. This includes telemetry request writes;
it is separate from telemetry round-trip time. Fault logs add operation, port,
last write duration and the underlying exception. Fifty milliseconds is a bounded
initial adjustment, not a verified fix for the particular Jetson's UART failure.

The connection topic has a reliable/transient-local sibling diagnostic topic:
`<connection_status_topic>/diagnostics` (normally `/vesc/connected/diagnostics`).
FAULT records the initiating cause and RECOVERED retains it after fresh telemetry
restores the link. The inspection UI shows the cause until START is accepted.
No build or tests were executed for this change at the user's request.
