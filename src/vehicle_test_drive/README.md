# vehicle_test_drive

Starts the VESC serial node, waits for a confirmed serial connection and ROS
command subscribers, then publishes a steering-servo sweep and ERPM ramps using
the same command path as manual control. Do not run `manual_control` command
publishers at the same time.

Sequence:

1. Servo position: 0.5 down to 0.1, then 0.1 up to 0.9 in 0.1 steps.
2. ERPM: 0 to 5000 over 3 seconds.
3. Hold 5000 for 0.25 seconds, then ERPM 0 for 0.5 seconds.
4. ERPM: 0 to -5000 over 3 seconds, then hold for 0.25 seconds.
5. Stop at ERPM 0 and return the servo to 0.5.

Run the complete test, including VESC initialization:

```bash
ros2 launch vehicle_test_drive vehicle_test_drive.launch.py
```

Use a different serial port when needed:

```bash
ros2 launch vehicle_test_drive vehicle_test_drive.launch.py vesc_port:=/dev/ttyACM1
```

## Fixed servo value for angle calibration

`servo_angle_calibrator` opens the VESC serial port directly, holds motor duty
at zero, and repeatedly applies one servo value. Do not run `vesc_initializer`,
manual control, autonomous control, or the launch above at the same time.

Build and run only the calibrator executable:

```bash
colcon build --packages-select vesc_initializer vehicle_test_drive
source install/setup.bash
ros2 run vehicle_test_drive servo_angle_calibrator --ros-args \
  -p servo_value:=0.46
```

Enter values such as `0.40` or `0.52` at the `servo_value>` prompt. Enter `q`
to return to `center_servo_value` and exit. The value can also be changed from
another terminal:

```bash
ros2 param set /servo_angle_calibrator servo_value 0.40
ros2 param get /servo_angle_calibrator servo_value
```

Use a different VESC port or center value when needed:

```bash
ros2 run vehicle_test_drive servo_angle_calibrator --ros-args \
  -p port:=/dev/ttyACM1 -p servo_value:=0.46 \
  -p center_servo_value:=0.46
```

On normal exit and Ctrl+C, duty remains zero and the servo returns to the
configured center value.
