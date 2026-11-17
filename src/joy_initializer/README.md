# joy_initializer

`joy_initializer` owns the project's Bluetooth SDL game-controller input
executable.

Unlike the upstream ROS `joy_node`, `joy_input_node` initializes only
`SDL_INIT_GAMECONTROLLER`. It contains no haptic initialization, rumble API
call, or `/joy/set_feedback` subscription. SDL's GameController layer converts
the 8BitDo Bluetooth D-input report to one stable logical layout instead of
exposing connection-specific raw axis and button numbers.
The project bundles the Ultimate 2 Linux Bluetooth GUID mapping from
SDL_GameControllerDB, so it does not depend on a recent system controller
database for this model.

The default name filter accepts any SDL game controller containing `8BitDo`,
which tolerates BlueZ/SDL naming differences. An empty filter falls back to
`device_id`. `Opened Bluetooth game controller` is printed once for the
initial open. A disconnect publishes one frame with all axes/buttons released (RT/LT 0)
and then stops `/joy` publication; the node then
waits for BlueZ/SDL to reconnect the controller.

Published SDL-standard layout:

```text
axes:    0=left X, 1=left Y, 2=right X, 3=right Y, 4=LT, 5=RT
buttons: 0=A, 1=B, 2=X, 3=Y, 4=Back, 5=Guide, 6=Start,
         7=LS, 8=RS, 9=LB, 10=RB, 11=Up, 12=Down, 13=Left, 14=Right
```

LT and RT are `0.0` when released and `1.0` when fully pressed. Stick axes
retain the project's previous sign convention: left/up are positive.

## Jetson Bluetooth pairing

Remove the 2.4 GHz USB receiver first. Put the controller in Bluetooth
D-input pairing mode, then run. 8BitDo specifies controller firmware 1.05 or
newer for Ultimate 2 Wireless Bluetooth use with Steam/SteamOS, so update the
controller firmware before pairing if it is older.

```bash
sudo systemctl enable --now bluetooth
bluetoothctl
power on
agent on
default-agent
scan on
# Run `devices`, copy the 8BitDo MAC address, then stop scanning:
devices
scan off
pair XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
quit
```

Confirm `Connected: yes` before launching ROS:

```bash
bluetoothctl info XX:XX:XX:XX:XX:XX
```

Run only the joystick input node:

```bash
ros2 launch joy_initializer joy.launch.py
```

If SDL exposes a generic name without `8BitDo`, select the first recognized
game controller by index:

```bash
ros2 launch joy_initializer joy.launch.py \
  device_name_contains:="" device_id:=0
```

The node intentionally refuses raw joysticks that SDL cannot identify as a
GameController. This prevents a Bluetooth-specific raw layout from silently
swapping accelerator, brake, or gear controls.
