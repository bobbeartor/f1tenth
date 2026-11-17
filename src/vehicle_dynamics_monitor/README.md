# vehicle_dynamics_monitor

VESC vehicle telemetry is converted into body-frame motion values for logging
and camera IMU accelerometer compensation. It supports ROS topics, kernel
SocketCAN, and a CANable 2 connected as a `python-can` SLCAN serial device.
The monitor never transmits CAN or actuator commands.

## Calculated values

- Motor RPM = ERPM / motor pole pairs
- Wheel RPM = motor RPM / total drivetrain ratio
- Speed = wheel circumference x wheel RPM / 60
- Longitudinal acceleration = filtered time derivative of speed
- Lateral acceleration = speed x yaw rate
- Gyroscope yaw rate is used by default. After the installed steering linkage
  is calibrated, optional `auto` mode can fall back to
  `speed * tan(steering angle) / wheelbase`.

The combined acceleration topic is
`/vehicle/dynamics/acceleration` (`geometry_msgs/Vector3Stamped`). It uses
`base_link`: X is forward acceleration, Y is left acceleration, and Z is zero.
This is the vehicle-motion component to transform into the IMU frame and
subtract from accelerometer measurements before tilt correction.

Other outputs are:

- `/vehicle/dynamics/speed_mps`
- `/vehicle/dynamics/longitudinal_acceleration_mps2`
- `/vehicle/dynamics/lateral_acceleration_mps2`
- `/vehicle/dynamics/yaw_rate_radps`
- `/vehicle/dynamics/motor_rpm`
- `/vehicle/dynamics/wheel_rpm`
- `/vehicle/dynamics/motor_current_a`
- `/vehicle/dynamics/input_current_a`
- `/vehicle/dynamics/input_voltage_v`
- `/vehicle/dynamics/duty_cycle`
- `/vehicle/dynamics/fet_temperature_c`
- `/vehicle/dynamics/motor_temperature_c`
- `/vehicle/dynamics/diagnostics`

## Manual-drive launch

Build and start the existing manual controller, VESC bridge, and monitor:

```bash
colcon build --packages-select vehicle_dynamics_monitor vehicle_bringup
source install/setup.bash
ros2 launch vehicle_bringup manual_drive_with_dynamics.launch.py
```

The repository currently controls and polls the VESC over Jetson UART
`/dev/ttyTHS1`, so `ros_topic` is the default input. This mode consumes the
transport-independent `/vesc/measured_erpm` output from `vesc_bridge`.

## Direct SocketCAN mode

Bring up the physical CAN interface with the bitrate configured in the VESC,
then select the receive-only input:

```bash
sudo ip link set can0 up type can bitrate 500000
ros2 launch vehicle_bringup manual_drive_with_dynamics.launch.py \
  input_mode:=socketcan can_interface:=can0 can_controller_id:=0
```

The VESC must be configured to broadcast CAN STATUS frames. The package
decodes STATUS through STATUS_6, including ERPM,
duty, motor/input current, voltage, FET/motor temperature, amp-hours,
watt-hours, tachometer/electrical revolutions, ADC inputs, and PPM. It opens a
raw CAN socket but sends no frames.

Protocol IDs and scales follow the
[official VESC CAN-bus documentation](https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md).

## CANable 2 SLCAN mode

Use this mode on Jetson kernels without the `slcan` kernel module. It does not
create `can1`; `python-can` opens `/dev/ttyACM0` directly:

```bash
python3 -m pip install 'python-can[serial]'
colcon build --symlink-install \
  --packages-select vehicle_dynamics_monitor vehicle_bringup
source install/setup.bash

ros2 launch vehicle_dynamics_monitor vehicle_dynamics_monitor.launch.py \
  input_mode:=slcan \
  slcan_channel:=/dev/ttyACM0 \
  slcan_bitrate:=500000 \
  can_controller_id:=112
```

No `ip link`, `slcand`, or kernel SocketCAN interface is required. Enable VESC
STATUS 1 for ERPM, motor current, and duty; STATUS 4 for input current and
temperatures; and STATUS 5 for input voltage.

Stop any standalone Python receive script before starting the ROS node because
only one process can own `/dev/ttyACM0`. Internally, the node opens the adapter
using the same verified call:

```python
can.Bus(interface="slcan", channel="/dev/ttyACM0", bitrate=500000)
```

For manual driving with UART actuation and receive-only CANable telemetry, run:

```bash
ros2 launch vehicle_bringup manual_drive_with_dynamics.launch.py \
  input_mode:=slcan \
  slcan_channel:=/dev/ttyACM0 \
  slcan_bitrate:=500000 \
  can_controller_id:=112
```

## Calibration notes

- Measure `wheel_diameter_m` under vehicle load and tune
  `speed_scale_correction` with a known travel distance.
- Measure wheelbase and real left/right tire angles before relying on the
  steering fallback for accelerometer compensation. The 30-degree values are
  unverified placeholders copied from the existing `auto_control` package,
  not published Traxxas steering-angle specifications.
- With the current `/camera/imu` optical frame, vehicle yaw is normally the
  negative Y angular velocity. If the IMU frame changes, update
  `imu_yaw_axis` and `imu_yaw_rate_sign`.
- The default `yaw_rate_source: imu` does not use uncalibrated steering values.
  After calibration, `auto` uses fresh IMU gyro data first and steering second.
  Neither mode feeds accelerometer data back into its own correction.
