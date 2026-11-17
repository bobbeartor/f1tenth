# vehicle_bringup

`auto_drive.launch.py`는 [출발 전 점검 창](../auto_control/STARTUP_INSPECTION.md)을 자동으로 연다.
카메라 자세·모델·제어 입력 준비 후 START를 눌러야 자율주행 duty를 허용한다.
수동 모드는 시작 버튼을 건너뛰며 승인 후 점검 창은 닫힌다.

터널 RGB↔IR 자동 전환은 [auto_control의 터널 기능](../auto_control/AUTO_CONTROL_TUNNEL.md)으로
통합되어 있다. 기존 `auto_drive.launch.py`에 `tunnel_enabled:=true`를 추가해 사용한다.
외부 RGB 80 FPS, 터널 IR 30 FPS, 외부 IR 추론 대기 0 FPS이며 기존 단독 실행은 유지한다.
공유 카메라 변경이 있으므로 `colcon build --packages-up-to vehicle_bringup`로 함께 빌드한다.

## 기본 주행 설정과 외부 YAML

패키지 기본 YAML은 제공된 `auto_control_test.yaml`, `bev_config_test.yaml`,
`line_detactor_test.yaml`, `obstacles_test.yaml`, `traffic_light_test.yaml`의 파라미터 값을
반영한다. 외부 기본 속도 범위는 1.0~1.8m/s, maximum duty는 0.12이며 코너 감속,
신호등 정지, 장애물 회피와 주행 로그를 활성화한다. 신호등 추론은 최대 30Hz,
일반 외부 stereo/depth는 40FPS다. 터널 통합에서는 앞서 정한 IR 노출을 위해
stereo/depth 30FPS와 터널 내부 장애물·신호등 제어 제외를 적용한다.

RGB 차선 모델은 같은 INT8 Q/DQ ONNX를 사용하며, 특정 차량 폴더 대신 설치된
`line_detactor/models`에서 파일명으로 찾는다. 외부 YAML의 절대 모델 경로는 그대로 사용한다.
새 코드에 필요한 항목 중 제공된 YAML에 없는 키는 기존 패키지 기본값을 유지한다.

파일을 지정하지 않으면 설치된 기본 설정을 읽는다. 기존 파일 경로 인자는 유지하며,
명시적 launch 인자가 YAML 값보다 우선한다. 다음은 경로만 교체해 쓰는 예시다.

```bash
ros2 launch vehicle_bringup auto_drive.launch.py \
  auto_control_params_file:=/path/to/auto_control_test.yaml \
  bev_params_file:=/path/to/bev_config_test.yaml \
  line_detactor_params_file:=/path/to/line_detactor_test.yaml \
  obstacle_params_file:=/path/to/obstacles_test.yaml \
  traffic_light_params_file:=/path/to/traffic_light_test.yaml
```

터널 기능을 켤 때도 같은 다섯 인자를 유지하고 `tunnel_enabled:=true`만 추가한다.

`auto_drive.launch.py`는 신호등 관측과 정지선 제동을 기본 활성화한다.
`traffic_light_yolox_s_640x160_batch_1.int8.qdq.onnx`를 별도 작업 스레드에서
30Hz로 실행하고, 카메라의 BEV 변환 전 컬러 영상을 공유한다.
기존 BEV 프리뷰 하단에 `Traffic: RED / GREEN / UNKNOWN`을 표시한다.
별도 신호등 창은 없다.
개발용 정지선 제동은 `traffic_stop_enabled:=true`로 켜고 `false`로 끈다.
속도 제어는 기본 `longitudinal_pid_enabled:=true`, 코너 감속은
`curvature_speed_control_enabled:=true`다. 정지선 거리로 목표 속도를 갱신하고
PID가 가속/제동을 제어한다.
설정은 [auto_control 문서](../auto_control/README.md)의 신호등 정지 항목을 참고한다.
`/traffic_light/state`의 `state`는
`0=UNKNOWN`, `1=RED`, `2=GREEN`이다. `traffic_light_enabled:=false`로 끄고
차선 처리 지연을 비교할 수 있다. 상세 설정과 빌드는
[신호등 패키지 문서](../traffic_detection_test/README.md)를 참고한다.

`vehicle_bringup` owns the launch files and runtime configuration used to
start the F1TENTH vehicle. It replaces the former `vehicle_launcher` and
`vehicle_config` packages.

## Launch files

- `joy_test.launch.py`: starts only the joystick input path.
- `vesc_only.launch.py`: starts only `vesc_bridge_node`.
- `manual_drive.launch.py`: starts joystick input, direct manual actuator
  commands, and the VESC bridge.
- `manual_drive_with_dynamics.launch.py`: starts manual driving plus the
  read-only vehicle dynamics/CAN monitor, including direct CANable 2 SLCAN
  telemetry when selected.
- `auto_drive.launch.py`: starts BEV images, ML lane/centerline detection, Stanley/PID
  autonomous control, the VESC bridge, and the vehicle dynamics/CAN monitor.

```bash
ros2 launch vehicle_bringup joy_test.launch.py
ros2 launch vehicle_bringup vesc_only.launch.py
ros2 launch vehicle_bringup manual_drive.launch.py
ros2 launch vehicle_bringup manual_drive_with_dynamics.launch.py
ros2 launch vehicle_bringup auto_drive.launch.py
```

Manual driving defaults to the vehicle-unique `/autopilot03` namespace for
all three nodes and the `/autopilot03/joy`, `/autopilot03/manual/*`, and
`/autopilot03/vesc/*` topics. This prevents another vehicle on the same ROS
domain from publishing actuator commands into this vehicle. Override the
identifier when required:

```bash
ros2 launch vehicle_bringup manual_drive.launch.py \
  vehicle_namespace:=autopilot03
```

The dynamics launch defaults to the current UART VESC telemetry path. To use
VESC CAN STATUS broadcasts directly, bring up `can0` first and launch with
`input_mode:=socketcan`. See `vehicle_dynamics_monitor/README.md` for CAN and
accelerometer-compensation details.

On the Jetson L4T kernel without `slcan`, a CANable 2 at `/dev/ttyACM0` can be
read directly without creating `can1`:

```bash
python3 -m pip install 'python-can[serial]'
ros2 launch vehicle_bringup manual_drive_with_dynamics.launch.py \
  input_mode:=slcan \
  slcan_channel:=/dev/ttyACM0 \
  slcan_bitrate:=500000 \
  can_controller_id:=112
```

The VESC continues to receive actuator commands through the existing UART
bridge. The CANable path is receive-only and is used only for vehicle telemetry
and acceleration estimation.

The VESC launch files default to the Jetson UART device `/dev/ttyTHS1`. A
different device path can be supplied without editing YAML:

```bash
ros2 launch vehicle_bringup vesc_only.launch.py \
  vesc_port:=/dev/serial/by-id/usb-your-adapter-id
```

## Configuration

- `config/vesc_config.yaml`: serial transport, command limits, and telemetry.
- `config/manual_vesc_config.yaml`: manual accelerator, 0..8 A LT electrical
  brake ramp, and steering.
- `auto_control/config/auto_control.yaml`: Stanley, speed PID, vehicle, and
  autonomous duty/electrical-brake actuator parameters.
- `config/controller_keymap.yaml`: 8BitDo controller mapping.

The previous empty `joy.yaml` was removed because `joy_initializer` already
owns the joystick node defaults.

## Autonomous driving

### X 버튼으로 실험 종료 후 수동 복귀

일반 `auto_drive.launch.py`의 `drive` 모드는 컨트롤러도 함께 실행한다.
기존 자동주행 실행 명령과 설정 파일을 그대로 사용할 수 있다.

```bash
ros2 launch vehicle_bringup auto_drive.launch.py preview_enabled:=true
```

1. 자동주행 중 컨트롤러 **X**(SDL button 2)를 누르면 자동 구동 명령을 차단한다.
2. 마지막 조향을 유지하며 기존 수동 브레이크 설정으로 제동한다
   (기본 최대 8 A, 증가율 16 A/s). 제동 중에는 duty 명령을 보내지 않는다.
3. X·RT·LT·RB를 놓고 왼쪽 스틱을 중앙에 둔다. 새 ERPM 측정값의 절댓값이
   100 이하로 0.5초간 유지되면 브레이크를 해제하고 `MANUAL`로 전환한다.
4. 이후 RT 가속, LT 제동, 왼쪽 스틱 조향, RB 전후진 전환으로 출발점에 복귀한다.

전환 시 카메라·BEV·차선인식·프리뷰는 종료하거나 재시작하지 않는다.
자동주행 재개는 launch를 종료한 뒤 다시 실행한다. 수동 상태에서 X를 다시 눌러도
자동으로 돌아가지 않는다. 별도로 `manual_drive.launch.py`를 중복 실행하지 않는다.

상태는 터미널과 `/<vehicle_namespace>/drive/mode`의 `AUTO`, `STOPPING`,
`MANUAL`로 확인할 수 있다(기본 namespace: `autopilot03`). ERPM 또는 컨트롤러
입력이 끊기면 정지/중립을 확인할 수 없으므로 `STOPPING`에서 제동을 유지한다.
설정은 `config/manual_vesc_config.yaml`의 `handover_*`와 수동 제동값을 사용한다.
ERPM 판정은 모터 회전 기반이므로 실제 정지 거리·미끄러짐은 차량에서 확인해야 한다.

`manual_test:=true`, `monitor_only`, `steering_only`, 성능 측정 모드는 기존 동작을
유지한다. 컨트롤러 전환 기능을 제외하려면 `manual_handover_enabled:=false`를 사용한다.
이 변경을 차량에 적용할 때는 `manual_control`과 `vehicle_bringup`을 다시 빌드한다.

Optional lightweight white BEV preview (default off):

```bash
ros2 launch vehicle_bringup auto_drive.launch.py bev_theme_enable:=true
ros2 param set /line_detactor bev_theme_enable false
```

The theme uses the same window dimensions and a separate 15 FPS preview cap.
Set `bev_theme_enable` / `bev_theme_preview_fps` in the selected line detector YAML.
The live speed HUD follows the controller's `current_speed_topic`; traffic model
FPS is measured in the shared detector process. Build the changed packages together:
`bev_handoff line_detactor traffic_detection_test vehicle_bringup`.

The pipeline is `camera_driver -> bev_processor -> line_detactor -> auto_control`.
`auto_drive.launch.py` loads the first three perception components into the same
component container. BEV-to-detector transfer is a process-local latest-frame
handoff; `/camera/image_bev` is not published or subscribed in this launch.
The controller consumes `/line_detactor/result` centerline points, keeping their
path order through corners. Each result immediately triggers command calculation
and publication. ML inference runs on new input independently of the GUI refresh
rate; the controller timer only checks stop conditions. Overload keeps the latest
frame instead of queuing old frames. The preview displays detected lanes and the yellow
centerline. Rule-based sliding-window detection and `/camera/image_bev_lane`
publication have been removed.

After pulling `0906ML`, rebuild the changed packages on the Jetson before launch:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select bev_handoff bev_processor line_detactor auto_control vehicle_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The existing configuration-file paths and CAN arguments remain usable:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vehicle_bringup auto_drive.launch.py \
  bev_params_file:=/home/autopilot03/Desktop/0822ver3/f1tenth_project_repo/bev_config_test.yaml \
  auto_control_params_file:=/home/autopilot03/Desktop/0822ver3/f1tenth_project_repo/auto_control_test.yaml \
  camera_params_file:=/home/autopilot03/Desktop/0822ver3/f1tenth_project_repo/camera_config_test.yaml \
  auto_enabled:=true \
  preview_enabled:=true \
  input_mode:=slcan \
  slcan_channel:=/dev/ttyACM0 \
  slcan_bitrate:=500000 \
  can_controller_id:=112
```

These are external YAML paths; replace them if the files move. To override ML
lane/centerline settings, add
`line_detactor_params_file:=/absolute/path/line_detactor_test.yaml`.
Use `src/line_detactor/config/line_detactor.yaml` as the template, with root
`line_detactor.ros__parameters`. Defaults retain 0.65m lane width, spatial smoothing,
outer-boundary corner weighting, and yellow rendering. Parameters load at startup.

`centerline_corner_outward_offset_m:=0.05` shifts the corner path toward the observed
outer boundary by up to 5cm; `0.0` disables that shift.
`centerline_corner_entry_distance_m:=0.40` (default) extends upcoming local corner
evidence up to 40cm back along the same observed boundary, ramping the outward shift
through the straight entry. This is additional to the existing corner detection
window, not an exact distance from the geometric bend. Increase it to start moving
outward earlier; `0.0` restores local corner shifting only. Shape blending retains
local evidence; the shift uses the current boundary normal. No evidence is carried
across missing fragments or retained from old frames.
The final yellow path is the controller input.
`preview_result_only_enabled:=true` (default) displays lanes,
stop lines and the yellow path on black; `false` restores the camera overlay.
These arguments override the corresponding `line_detactor_params_file` values.
Offset magnitude fades with corner evidence and is bounded by observed-line clearance;
it does not guarantee clearance of the full vehicle footprint.

`bev_params_file` remains necessary for projection geometry, startup measurement
and capture settings. Old `lane_*` BEV keys are ignored and can be deleted.
The launch derives ML input topic, physical extents, result topic, and controller
scale/frame consistently. It requires X=0 at the front axle, symmetric Y bounds,
and BEV dimensions matching the static model. It requires connection, centerline
and result publication enabled. Individual supported launch arguments override YAML.
Old controller `lane_topic`, `lane_pixel_threshold`, `path_local_smoothing_window_m`
and `path_outlier_threshold_m` no longer affect this pipeline.

SLCAN/SocketCAN inputs feed receive-only telemetry and camera acceleration
compensation. Actuation and controller ERPM still use `vesc_bridge` through UART
(`/dev/ttyTHS1`, overridable by `vesc_port`). `python-can[serial]` is required for SLCAN.

`auto_enabled:=true` arms motion once fresh valid path and VESC data arrive.
Use `auto_enabled:=false` to start disabled and `preview_enabled:=false` to hide the GUI.
The ML preview has no BEV Space-key shortcut; disable explicitly with:

```bash
ros2 topic pub --once /auto/enabled std_msgs/msg/Bool "data: false"
```

Invalid/stale path or ERPM and VESC disconnect send duty zero and centered steering.
Stop-line detection alone does not command stopping. Never run manual and auto launches together.
Dataset capture now saves only `origin_bev`; removed rule-based labels are not generated.

## Remote preview (performance test) mode

`remote_preview_enabled:=true` keeps normal driving but turns off every Jetson-side
preview window and image topic. A browser on an iPad/MacBook renders the preview
from numeric telemetry served by `auto_control remote_preview` on port 8765.
See `src/auto_control/REMOTE_PREVIEW_KO.md`.
The main measurements are:

- pure TensorRT inference and H2D/GPU preprocessing
- GPU label export, lane connection/centerline generation, result-message build,
  and their combined lane postprocessing time
- line-detector queue time and compute-only detector time
- source capture to line-detector input delay and LaneResult DDS transfer delay
- LaneResult callback through final steering/servo-position calculation
- BEV detector input to control completion and source capture to control completion
- result throughput FPS, valid servo-position calculation FPS, and latency-derived FPS
- Jetson total-module `VDD_IN`/`5V_IN` power average/minimum/maximum and estimated energy

`lane_result_throughput_fps_excluding_source_transport_delay` and
`servo_position_calculation_throughput_fps` are counts divided by the measurement
window. A fixed source-to-detector latency therefore does not get added to every
throughput interval: pipelined frames can overlap. The source still has to actually
publish at that rate, and detector queueing or dropped latest-only frames can lower it.
The JSON schema version is 2 and records `controller_implementation=rclcpp_cpp`.
It also embeds the throughput and detector-input-to-control latency definitions
so benchmark files remain self-describing.

Power is read-only sampled from the INA3221 sysfs rail every 0.1 seconds. If the
platform does not expose a recognized total-input rail, the JSON marks power as
unavailable while retaining all timing measurements.

`LaneResult.msg` is now a lightweight control message: image, label, mask, and
diagnostic lane-curve arrays were removed. The rendered result remains on
`/line_detactor/result_image`. Rebuild all message producer/consumer packages after
pulling this change:

```bash
source /opt/ros/humble/setup.bash
colcon build \
  --packages-select bev_handoff line_detactor auto_control vehicle_bringup \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## ML pipeline diagnosis

Updating sources with `git pull` does not update an installed launch file.
Rebuild **vehicle_bringup** together with `line_detactor`, `auto_control` and
`bev_processor`; rebuilding only BEV does not install the ML launch integration.
Use a fresh terminal with the Humble environment and this workspace's
`install/local_setup.bash` when checking for an older workspace overlay.

The current launch prints `[ML auto drive]` with its launch path, selected YAML,
model path, direct CUDA-memory pipeline, and preview selection. A loaded detector prints
`Line detector ready`. The default ML window is `BEV lane TensorRT preview`
and contains inference/FPS/lanes information below the image; the raw BEV preview
is disabled by the integrated launch. An ML window can still show just the image
if the model detects no lanes, so use the status logs to distinguish these cases.

Read-only inspection in the same ROS environment as the running launch:

```bash
ros2 pkg prefix vehicle_bringup
ros2 pkg prefix line_detactor
ros2 node list
ros2 param get /bev_processor preview_enabled
ros2 param get /line_detactor model_path
ros2 param get /bev_processor direct_output_enabled
ros2 param get /line_detactor direct_bev_input_enabled
ros2 topic info /line_detactor/result
```

The pipeline needs `/bev_processor`, `/line_detactor`, `/auto_control` and
`/vesc_bridge_node`. `ros2 pkg prefix` should point into the intended 0906ML install.
Both direct parameters must be `true`; `/camera/image_bev` is intentionally absent.
The default result-only preview consumes the CUDA BEV allocation directly and does
not create a full CPU BEV copy. A camera-overlay preview explicitly enables that copy.
The result topic still needs an ML publisher and controller subscriber. Extra
diagnostic subscriptions may also appear.

`Auto status` now separates actuator stop state from ML reception:

- `lane_rx=0`: no LaneResult has reached this controller.
- `lane_status=no_centerline(state=...)`: ML messages arrive, but contain no centerline.
- `centerline_sample_limit`: the generator exceeded its sample budget.
- `insufficient_contiguous_path`: the received points do not meet the configured ROI,
  length, point-count or adjacent-gap requirements.
- `accepted`: the latest received path passed the controller's checks. Use
  `last_rx_age`, `capture_age_at_rx` and `state` to check subsequent staleness.
- `state=vesc_disconnected`, `waiting_for_erpm` or `erpm_timeout`: inspect UART VESC
  telemetry separately; SLCAN dynamics does not replace controller ERPM/connection.

The diagnostic output does not relax path freshness, validity or motor limits.

## 8BitDo Bluetooth input safety

The manual launch uses the project-owned `joy_input_node` for the 8BitDo
controller in Bluetooth D-input mode. Remove the 2.4 GHz receiver before
launch. The node uses SDL GameController's standard layout and matches the
`8BitDo` name substring, publishes only `/joy`, and has no haptic subsystem,
feedback subscription, or rumble API. The actuator commander consumes `/joy`
directly; no conversion/debug process runs between the controller and actuator
commands.

The initial `Opened Bluetooth game controller` message is normal startup. If
BlueZ or SDL removes the controller, the node stops publishing immediately so
the 0.30-second actuator watchdog can stop the vehicle. Reconnection is
automatic after BlueZ restores the trusted Bluetooth link.

The manual mapping is `RT=axis5`, `LT=axis4`, left-stick X=`axis0`, and
`RB=button10`. Both triggers use `0.0` released and `1.0` fully pressed. Verify
these values with `joy_test.launch.py` while the drive wheels are lifted before
the first powered run.

If SDL exposes a generic name without `8BitDo`, clear the name filter and use
the first SDL-recognized controller:

```bash
ros2 launch vehicle_bringup joy_test.launch.py controller_name_contains:=""
```

RB gear changes are edge-triggered and immediate: they are accepted only while
the commanded duty is already zero. A rejected change is not stored or replayed
later. `vesc_bridge` communicates through `/dev/ttyTHS1` and does not access the
Bluetooth controller transport.

## 기존 auto_drive에서 장애물 매핑을 수동으로 확인

`auto_drive.launch.py manual_test:=true`는 기존 차선·신호등 검출과 auto_control의 장애물
검출을 함께 실행한다. auto_control의 액추에이터 출력은 monitor_only로 차단하며 조작은
수동으로 한다. 별도 manual_obstacle_view launch는 이 구성으로 대체했다.
[실행 및 YAML 안내](../auto_control/AUTO_CONTROL_OBSTACLES.md).
