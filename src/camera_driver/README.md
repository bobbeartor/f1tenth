# camera_driver

## BEV startup reference 연동

`bev_processor`와 함께 실행하면 `/camera/startup_ground_normal`의
`geometry_msgs/Vector3Stamped`를 transient-local QoS로 구독한다. 이 벡터는
BEV 고정 LUT에 사용된 시작 지면 법선이며 `camera_optical_frame` 좌표계로
표현된다. 통합 launch는 이 기준을 필수로 설정하므로, 수신 전에는 IMU 중력
방향을 대신 기준으로 확정하지 않는다.

IMU 시작 평균 중력 방향은 별도로 보존해 정지 판정에만 사용한다. 따라서
Depth 법선과 IMU 중력 방향 사이에 고정 오차가 있어도 정지 후 복구 목표는
BEV와 공유한 기준이다. 단독 camera_driver launch에서는 BEV publisher가
없으므로 외부 기준이 optional이며 기존 IMU 시작 기준으로 fallback한다.

OAK/DepthAI 카메라 영상을 낮은 지연시간으로 받는 ROS 2 C++ 패키지다.
기본 설정은 다음과 같다.

- 센서 모드: `1280x800`; CAM_A는 `NV12`, CAM_B/C는 native `GRAY8`
- 기본 프리뷰/ROS 출력: 안정화된 전체 `1280x800` 프레임
- 요청 센서 FPS: `80`
- USB 최대 속도 요청: `SUPER` (5 Gbps)
- XLink 청크 분할: 비활성화 (`setXLinkChunkSize(0)`)
- 렌즈 왜곡 보정: OAK 장치 내부에서 활성화
- ROS 이미지 발행: 기본 비활성화
- IMU 브리지: 기본 비활성화, 활성화 시 camera optical frame의
  가속도+각속도 발행
- 영상 안정화: 기본 활성화, 시작 pitch/roll 기준을 유지하는 고정 초점
  짐벌 방식
- QoS: sensor data, best effort, keep-last 1
- 호스트 큐: 크기 1, non-blocking
- 프리뷰: 캡처와 분리된 최신 프레임 방식

CAM_A는 OV9782의 NV12 출력을 사용한다. 스테레오 CAM_B/C는 장치에서
GRAY8로 직접 받아 불필요한 mono-to-NV12 변환 병목을 피한다. 노드는 요청값과
별도로 측정된 캡처 FPS와 장치 sequence gap을 주기적으로 출력한다.

첫 프레임에는 resize와 장치 내부 왜곡 보정을 반영한 전체 1280x800
`K_rect`의 `fx`, `fy`, `cx`, `cy`를 출력한다. 투영 기하가 필요한 후속
처리에서는 이 값을 사용한다.

## 성능 구조

카메라 캡처, ROS 발행, OpenCV 프리뷰는 서로 다른 스레드에서 실행된다.
캡처 스레드는 DepthAI 큐를 비우고 최신 프레임 포인터만 교체하며, ROS
메시지 복사나 `imshow()`를 수행하지 않는다. 발행이나 프리뷰가 늦어지면
오래된 프레임을 쌓지 않고 최신 프레임으로 건너뛴다.

CAM_A는 OAK에서 `NV12`를 생성해 Jetson으로 전송한다. CAM_B/C는 native
`GRAY8`을 전송하며, 호스트가 기존 ROS/BEV NV12 인터페이스를 유지하기 위해
Y plane을 복사하고 UV를 중립값 128로 채운다. 캡처 스레드는 `ImgFrame`
패킷만 보관하며 색 변환을 하지 않는다.

기본 NV12 메시지는 `encoding="nv12"`, `width=1280`, `height=800`,
`step=1280`을 사용하고, `data`에는 Y plane 800행 다음에 interleaved UV
plane 400행이 연속으로 들어간다. 일반 BGR8 구독자가 아니라 NV12를
이해하는 처리 노드가 받아야 한다.

왜곡 보정은 `Camera::requestOutput(..., enableUndistortion=true)`로 요청한다.
따라서 호스트에서 `cv::remap()`을 수행하지 않는다.

IMU 브리지를 켜면 calibrated accelerometer와 calibrated gyroscope를
400 Hz로 요청한다. DepthAI가 이미 적용한 EEPROM 회전을 다시 곱하지 않고,
calibrated 출력 좌표계에서 선택 카메라 optical frame으로 가는 상대 회전만
적용해 `sensor_msgs/Imu`로 발행한다. orientation 자체는 채우지 않는다.
영상 안정화용 IMU는 브리지 설정과 무관하게 드라이버 내부에서 사용한다.

ROS 발행은 `sensor_msgs/msg/Image`의 `UniquePtr`를 사용한다. 기본 launch는
컴포넌트 컨테이너에서 intra-process 통신을 활성화한다. 향후 C++ 영상 처리
컴포넌트를 같은 컨테이너에 적재하면 DDS 직렬화 없이 메시지 소유권을 넘길
수 있다. 별도 프로세스의 구독자, `ros2 topic hz`, rosbag 등은 DDS 전송과
추가 메모리 복사를 사용한다.

통합 BEV launch에서는 일반 `sensor_msgs/Image` 발행을 끄고
`camera_driver/msg/BevInput`을 사용한다. NV12 또는 GRAY8에서 만든 호환
NV12의 하단 70%(1280x560)만 복사하고, 같은 노출 시점의 원본→안정화
homography를 함께
보낸다. 이 경로는 CPU Y/UV `warpPerspective()`를 수행하지 않으며, 고정 줌과
동적 roll/pitch 보정은 BEV의 CUDA sampling에 합쳐진다. 단독 프리뷰와 일반
이미지 토픽의 기존 CPU 안정화 경로는 호환성을 위해 유지한다.

`publish_fps`가 센서 FPS 이상이면 고정 주기의 타이머로 최신 영상을
샘플링하지 않고 새 프레임 도착 알림에 맞춰 발행한다. 두 143 Hz 주기의
미세한 위상 차이로 프레임을 건너뛰는 현상을 피하기 위한 동작이다.

프리뷰 창의 실제 표시 속도는 모니터 주사율과 OpenCV GUI 성능의 제한을
받는다. 60 Hz 모니터에서는 센서가 143 FPS로 동작해도 143개의 서로 다른
프레임을 모두 눈으로 확인할 수 없다. 상태 로그의 `capture` 값이 센서
수신 속도의 기준이다.

## 요구 사항

- Ubuntu/Jetson의 ROS 2 Humble
- DepthAI C++ 3.x
- OpenCV 4
- OAK 장치와 USB 3 연결

Python의 `pip install depthai`만으로는 이 C++ 패키지를 빌드할 수 없다.
`depthaiConfig.cmake`와 `depthai::core` 공유 라이브러리를 제공하는 DepthAI
C++ 설치가 필요하다.

### 기존 Jetson DepthAI C++ SDK 경로

차량 PC에 이미 설치되어 있던 `/usr/local`의 SDK를 공통으로 사용한다.
CMake config는 `/usr/local/lib/cmake/depthai/depthaiConfig.cmake`,
실행 라이브러리는 `/usr/local/lib/libdepthai-core.so`다.
`/opt/ros/humble`의 별도 DepthAI와 섞이지 않도록 빌드와 실행 경로를 함께 고정한다.

날짜별 워크스페이스 옆에 복사한 `depthai-core` 소스 폴더는 그대로 둔다.
ROS 패키지 빌드 중 SDK 재설정·재빌드·재설치나 빌드 도구 설치를 수행하지 않는다.
`source install/setup.bash`가 기존 SDK의 실행 경로를 적용한다.
상세 절차와 진단은 [공통 DepthAI 설정](../../DEPTHAI_SETUP.md)을 참고한다.

DepthAI는 OpenCV 지원을 켜고 빌드되어야 한다. 프리뷰를 켠 경우에만
`ImgFrame::getCvFrame()`으로 NV12를 CPU BGR로 변환한다. GRAY8 프리뷰는
OpenCV에서 BGR 세 채널로 확장해 같은 창에 표시한다.

## 빌드

Jetson에서 워크스페이스 루트로 이동한 후 Release 모드로 빌드한다.

```bash
cd ~/Desktop/1030ML/f1tenth_project_repo
source /opt/ros/humble/setup.bash

colcon build \
  --packages-up-to camera_driver \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`colcon build`는 `f1tenth_project_repo` 루트에서 실행한다. `src` 안에서
실행하면 그 아래에 별도의 `build`, `install`, `log`가 생성되어 올바른
워크스페이스의 설치 결과와 섞일 수 있다.

## 실행

기본 컴포넌트 실행:

```bash
ros2 launch camera_driver camera_driver.launch.py
```

창 없이 실행:

```bash
ros2 launch camera_driver camera_driver.launch.py preview_enabled:=false
```

기본 독립 프리뷰의 격자는 꺼져 있다. 격자가 필요한 경우에만 다음처럼
명시적으로 켠다.

```bash
ros2 launch camera_driver camera_driver.launch.py \
  preview_enabled:=true preview_grid_enabled:=true
```

프리뷰 창에 포커스를 두고 키보드 `A`를 누르거나, 컨트롤러 A
버튼(SDL button 0)을 누르면 `capture_directory`에 PNG를 저장한다.
저장 영상은 안정화·crop이 적용된 카메라 픽셀만 깊은 복사하며,
프리뷰 격자나 GUI 창 요소는 포함하지 않는다. 컨트롤러는 상승
에지에서만 저장하고 모든 캡처 입력에 `capture_debounce_sec` 기본
0.30초를 공통 적용한다.
수동주행 launch의 기본 차량 namespace를 같이 쓰는 경우 컨트롤러
토픽은 `/autopilot03/joy`이므로 다음처럼 지정한다.

```bash
ros2 launch camera_driver camera_driver.launch.py \
  preview_enabled:=true \
  capture_joy_topic:=/autopilot03/joy \
  capture_directory:=./camera_captures
```

실차에서는 `auto_drive.launch.py`를 실행하지 않고, SocketCAN 차량 dynamics와
`camera_driver` 프리뷰를 각각의 터미널에서 실행한다.

```bash
# 터미널 1: 수동주행 + SocketCAN 종/횡가속도
ros2 launch vehicle_bringup manual_drive_with_dynamics.launch.py \
  input_mode:=socketcan can_interface:=can0 can_controller_id:=112

# 터미널 2: full-band gyro 안정화 + CAN 저주파 자세 anchor
ros2 launch camera_driver camera_driver.launch.py \
  preview_enabled:=true \
  capture_joy_topic:=/autopilot03/joy \
  imu_stabilization_enabled:=true \
  imu_stabilization_can_longitudinal_compensation_gain:=0.7 \
  imu_stabilization_can_lateral_compensation_gain:=0.7 \
  imu_stabilization_moving_accelerometer_nudge_strength:=0.15 \
  imu_stabilization_moving_gravity_anchor_maximum_correction_rate_degps:=0.50 \
  imu_stabilization_invalid_correction_hold_frames:=2
```

안정화 유무를 A/B 비교할 때는 `imu_stabilization_enabled:=true/false`만
바꾼다. CAN gain은 `0.0`이면 해당 차량가속도를 제거하지 않고 `1.0`이면
전량 제거한다. `nudge_strength`는 제거 후 남은 가속도계 자세 오차가 영상에
반영되는 비율이다.

안정화기는 전원 직후 IMU 샘플을 1초간 폐기한 뒤 4초 정지 구간의 중력
방향과 자이로 bias를 시작 기준으로 측정한다. 이 5초 동안 차량과 카메라를
움직이면 기준 측정이 다시 시작된다.

이후 400 Hz calibrated gyro로 카메라 좌표의 노면/중력 법선 벡터를
적분하고, 수정 전과 동일하게 시작 기준 자세와 비교한 전체 대역 보정을
homography로 적용한다. `imu_stabilization_high_frequency_only:=true`이면
CAN과 주행 중 accelerometer anchor/nudge를 끄고, 지정한 cutoff 이상의 gyro
변화만 `imu_stabilization_gyroscope_correction_gain` 비율로 적용한다.

주행 중 가속도계 입력에는 CAN dynamics의 종가속도와 횡가속도를 카메라
좌표로 변환해 뺀다. 이 잔여 중력 방향은 누적 gyro 자세를 천천히 붙잡는
persistent anchor와 빠르지만 비누적인 bounded nudge에 함께 사용된다. Anchor
변화율은 기본 `0.50 deg/s`로 제한해 불완전한 CAN 모델이 자세를 급격하게
움직이지 못하게 한다. CAN 샘플이 0.1초 이상 느려지면 차량 가속도가 섞인
raw 가속도계를 넣지 않고 해당 가속도 샘플을 생략하며 gyro 보정은 계속한다.
통합 BEV에서는 depth 시작 법선으로 차량 축을 구하고, 외부 법선이 없는
단독 카메라 프리뷰에서는 교정된 IMU 시작 중력 방향을 fallback으로 사용한다.

정지 시에는 시작 기준으로 자세를 빠르게 복구하고 gyro bias를 카메라 3축
모두 갱신한다. 정지 판정은 drift가 포함될 수 있는 현재 자세 추정값이 아니라
변경되지 않는 시작 기준과 비교한다. 평지에서 서스펜션이 원래 자세로
복귀한다는 전제는 4초 시정수의 약한 시작 기준 leak으로도 반영한다.

결과 FOV는 광학 중심 기준 1.25배 고정 줌으로 유지한다. 줌 영역으로 원본
경계를 모두 채울 수 없는 자세나 3도 보정 한계를 넘은 불확실한 자세에는
동적 회전을 적용하지 않고 zoom-only 원본을 전달한다. 단, 일시적인 camera/IMU
매칭 실패에는 직전 정상 homography를 기본 2프레임까지 유지해 단일 프레임
점프를 막고, 그 이상 연속 실패할 때만 zoom-only로 전환한다.
단독 프리뷰와 일반 이미지 발행은 CPU OpenCV 워프를 유지한다. 통합 BEV
launch는 프레임별 행렬만 계산하고 CUDA BEV 변환에 합치므로 CPU 워프를
건너뛴다. 활성화 후 상태 로그의 실제 capture FPS와 누락 프레임 수는
Jetson에서 확인한다.

ROS 이미지 발행 없이 캡처와 직접 프리뷰만 측정:

```bash
ros2 launch camera_driver camera_driver.launch.py publish_enabled:=false
```

독립 실행 파일도 제공한다. 이 실행 파일 역시 intra-process 옵션을 켠다.

```bash
ros2 run camera_driver camera_driver_node \
  --ros-args --params-file \
  "$(ros2 pkg prefix camera_driver)/share/camera_driver/config/camera_config.yaml"
```

## 상태 확인

노드는 기본 1초마다 다음 항목을 출력한다. 시작 직후에는 폐기 구간을
`[warmup discard]`, 정지 기준 자세와 gyro bias 측정 구간을
`[calibration]`으로 구분해 진행률을 표시한다.

- `capture`: 실제 DepthAI 프레임 수신 FPS와 요청 FPS
- `preview`: 실제 프리뷰 갱신 FPS
- `IMU`: 안정화/ROS 브리지에서 실제 처리한 IMU 샘플 rate
- `stabilizer`: `off`, `discarding-startup-imu`,
  `stationary-calibration`, `virtual-gimbal-ready` 상태와 누적
  mapping/miss/drop 수
- `Virtual gimbal`: 현재 roll/pitch 오차, 적용 회전각, 정지 판정,
  차량 `v/ax/ay`, motion fusion 적용/누락 원인, 3축 gyro bias와
  온라인 bias 갱신 횟수
- `dropped`: 최근 상태 구간의 sequence 누락 프레임 수

예:

```text
FPS: capture=120.0/120.0, preview=0.0, dropped=0
```

ROS 이미지 발행을 명시적으로 켠 경우 외부 토픽을 다음과 같이 확인할 수
있다. 이 명령 자체가 별도 DDS 구독자를 추가하므로 최종 성능 판정은
드라이버의 `capture` 로그를 우선한다.

```bash
ros2 topic hz /camera/image_rect
ros2 topic info /camera/image_rect --verbose
```

## 주요 파라미터

설정 파일은 `config/camera_config.yaml`이다.

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `perf_log_enabled` | `false` | 캡처·안정화 FPS와 지연 상세 로그 |
| `sensor_fps` | `80.0` | OAK 센서/출력 요청 FPS |
| `camera_socket` | `CAM_A` | RGB `CAM_A`는 NV12, stereo `CAM_B/C`는 GRAY8 전송 |
| `width`, `height` | `1280`, `800` | OAK 입력 및 기본 출력 해상도 |
| `undistort_enabled` | `true` | OAK 장치 내부 왜곡 보정 |
| `queue_size` | `8` | DepthAI 호스트 큐 크기 |
| `queue_blocking` | `false` | 큐가 찼을 때 캡처 차단 여부 |
| `publish_enabled` | `false` | ROS 이미지 발행 |
| `publish_fps` | `80.0` | ROS 발행 목표 최대 FPS |
| `fused_bev_output_enabled` | `false` | 하단 NV12+보정 행렬 BEV 전용 출력; 통합 launch에서 `true` |
| `fused_bev_topic` | `/camera/bev_input` | `camera_driver/msg/BevInput` 출력 |
| `bev_input_bottom_fraction` | `0.70` | BEV 변환 전에 유지할 원본 영상 하단 비율 |
| `capture_directory` | `.` | A 버튼 카메라 PNG 저장 경로 |
| `capture_joy_topic` | `/joy` | 카메라 캡처에 사용할 Joy 토픽 |
| `capture_joy_button` | `0` | 카메라 캡처 버튼; SDL A=0 |
| `capture_debounce_sec` | `0.30` | 모든 캡처 입력의 최소 허용 간격 |
| `imu_bridge_enabled` | `false` | 가속도+자이로 ROS 발행; 기본 launch는 CAN 횡가속도 계산을 위해 `true` override |
| `imu_rate_hz` | `400.0` | calibrated accel+gyro 요청/발행 rate |
| `imu_max_batch_reports` | `5` | 장치측 IMU 묶음 전송 상한 |
| `imu_topic` | `/camera/imu` | `sensor_msgs/Imu` 출력 |
| `imu_stabilization_enabled` | `true` | 영상 roll/pitch 진동 보정 on/off |
| `imu_stabilization_high_frequency_only` | `false` | CAN·주행 accelerometer를 끄고 gyro 고주파만 보정 |
| `imu_stabilization_high_frequency_vibration_cutoff_hz` | `3.0` | 고주파 전용 모드의 gyro high-pass cutoff |
| `imu_stabilization_gyroscope_correction_gain` | `1.0` | 추정 gyro 회전의 영상 상쇄 비율 `0.0~1.0` |
| `imu_stabilization_can_acceleration_topic` | `/vehicle/dynamics/acceleration` | `base_link` X=종, Y=횡가속도 입력 |
| `imu_stabilization_can_acceleration_timeout_sec` | `0.10` | 이보다 오래된 CAN dynamics는 사용하지 않음 |
| `imu_stabilization_can_longitudinal_compensation_gain` | `1.0` | CAN 종가속도 제거 비율 `0.0~1.0` |
| `imu_stabilization_can_lateral_compensation_gain` | `1.0` | CAN 횡가속도 제거 비율 `0.0~1.0` |
| `imu_stabilization_startup_discard_duration_sec` | `1.0` | 전원 직후 IMU 과도값 폐기 시간 |
| `imu_stabilization_reference_calibration_duration_sec` | `4.0` | 정지 기준 자세 측정 시간 |
| `imu_stabilization_external_reference_topic` | `/camera/startup_ground_normal` | BEV 시작 지면 법선 토픽 |
| `imu_stabilization_external_reference_required` | `false` | 외부 기준 필수 여부; 통합 BEV launch에서는 `true` |
| `imu_stabilization_accelerometer_time_constant_sec` | `1.5` | pitch 가속도계 보정 시정수 |
| `imu_stabilization_roll_accelerometer_time_constant_sec` | `2.0` | roll 가속도계 보정 시정수 |
| `imu_stabilization_measured_erpm_topic` | `/vesc/measured_erpm` | 주행 중 정지 판정에 쓰는 VESC 실측 ERPM |
| `imu_stabilization_stationary_erpm_enter_threshold` | `100` | 필터링된 ERPM 절댓값이 이하로 유지되면 정지 후보 |
| `imu_stabilization_stationary_erpm_exit_threshold` | `500` | raw ERPM 절댓값이 이상이면 즉시 주행 판정 |
| `imu_stabilization_stationary_erpm_filter_time_constant_sec` | `0.15` | 정지 진입용 ERPM 절댓값 저역통과 필터 시정수 |
| `imu_stabilization_stationary_erpm_enter_duration_sec` | `1.0` | 정지 진입 debounce 시간 |
| `imu_stabilization_measured_erpm_timeout_sec` | `1.0` | ERPM 수신 중단 시 정지 판정을 해제하는 시간 |
| `imu_stabilization_vehicle_motion_compensation_enabled` | `true` | CAN 종·횡가속도를 IMU에서 제거해 중력 방향을 추정 |
| `imu_stabilization_maximum_longitudinal_acceleration_mps2` | `15.0` | 종가속도 절댓값 제한 |
| `imu_stabilization_maximum_lateral_acceleration_mps2` | `15.0` | CAN 횡가속도 절댓값 제한 |
| `imu_stabilization_accelerometer_stationary_only` | `false` | `false`이면 주행 중 CAN 보정 중력으로 누적 자세 drift를 제한 |
| `imu_stabilization_moving_accelerometer_nudge_enabled` | `true` | 보정된 가속도계의 빠른 성분을 비누적 bounded nudge로 추가 적용 |
| `imu_stabilization_moving_accelerometer_nudge_time_constant_sec` | `0.15` | 주행 중 nudge 반응/제거 시정수 |
| `imu_stabilization_moving_accelerometer_nudge_strength` | `0.15` | 잔여 가속도계 자세 오차의 영상 반영 비율 `0.0~1.0` |
| `imu_stabilization_moving_accelerometer_pitch_nudge_maximum_deg` | `0.20` | 주행 중 가속도계가 만들 수 있는 pitch 최대 영향 |
| `imu_stabilization_moving_accelerometer_roll_nudge_maximum_deg` | `0.15` | 주행 중 가속도계가 만들 수 있는 roll 최대 영향 |
| `imu_stabilization_moving_gravity_anchor_maximum_correction_rate_degps` | `0.50` | CAN 보정 중력 anchor의 roll/pitch 합성 최대 변화율 |
| `imu_stabilization_reference_tilt_leak_time_constant_sec` | `4.0` | 주행 중 시작 tilt 기준으로 복귀하는 시정수 |
| `imu_stabilization_stationary_tilt_recovery_time_constant_sec` | `0.35` | 정지 후 시작 시점 복귀 시정수 |
| `imu_stabilization_maximum_correction_deg` | `3.0` | 축별 최대 동적 보정각; 초과 시 zoom-only fallback |
| `imu_stabilization_maximum_prediction_sec` | `0.015` | 마지막 gyro 기반 최대 예측 시간 |
| `imu_stabilization_invalid_correction_hold_frames` | `2` | camera/IMU 매칭 실패 시 직전 정상 homography 유지 프레임 수 (`0~5`) |
| `fixed_view_zoom` | `1.25` | 고정 출력 FOV 줌 배율 |
| `fixed_view_border_margin_px` | `1.5` | 원본 경계 bilinear 안전 여백 |
| `output_crop_top_px` | `0` | 안정화 후 제거할 상단 행 수 (`0`이면 원본) |
| `preview_enabled` | `false` | OpenCV 직접 프리뷰 |
| `preview_fps` | `60.0` | 프리뷰 갱신 목표 최대 FPS |
| `preview_grid_enabled` | `false` | 독립 프리뷰 격자 표시 |
| `preview_grid_spacing_px` | `20` | 원본 영상 기준 격자 간격 |

시작 Depth 지면 법선은 BEV의 불변 절대 자세로 사용한다. 주행 중
차량 가속도를 제거한 IMU 방향은 시작 IMU와 Depth 법선 사이의 정렬을
적용하여, 정지와 주행 상태가 다른 절대 자세를 목표로 삼지 않는다.

`Virtual gimbal` 로그의 `CAN_vehicle_accel`은 CAN dynamics의 현재
종/횡가속도와 직전 로그 구간의 최대 횡가속도를 표시한다.
`imu_residual_accel(forward/left)`는 이 차량 종/횡가속도를 IMU에서
제거한 뒤, Depth 자세 정렬을 적용하기 전의 잔여값이다.
`accel_nudge(roll/pitch)`는 기본 자이로/중력-anchor 자세에 더해진 현재 비누적
가속도계 보정각이고, `gravity_anchor_updates`가 증가하면 주행 중 persistent
anchor가 실제로 동작한 것이다. FPS 로그의 `held`는 camera/IMU 매칭 실패 때
직전 정상 homography를 재사용한 누적 프레임 수다.

주행 중 bounded nudge는 기본 자세 적분에 다시 넣지 않는다.
따라서 residual 가속도가 오래 틀려도 pitch/roll 영향은
각각 설정한 maximum을 넘어 누적되지 않는다. 유효한 가속도
샘플이 없거나 정지하면 같은 빠른 시정수로 0도로 복귀한다.

속도·가속도와 구동계 보정값은 `vehicle_dynamics_monitor`에서 관리한다.
실차 거리 기준 속도가 `/vehicle/dynamics/speed_mps`와 다르면 monitor의
`speed_scale_correction`을 다음처럼 조정한다.

```text
new_scale = old_scale * actual_speed / logged_speed
```

속도는 맞지만 `ax`가 너무 요동치면 monitor의 speed/acceleration filter
시정수를 늘리고, 반응이 너무 늦으면 줄인다. 기본 횡가속도는 monitor에서
CAN 속도와 `/camera/imu` yaw rate를 결합해 계산한다.

80 FPS에서 `1280x800 NV12`의 순수 영상 데이터는 약 117 MiB/s이고,
GRAY8은 약 78 MiB/s다.
`BGR888i`의 약 234 MiB/s보다 작다. 외부 프로세스 구독자는 DDS 직렬화와
추가 복사를 사용하므로, 후속 C++ 영상 처리는 같은 컴포넌트 컨테이너의
intra-process 통신으로 구성하는 것이 좋다.
