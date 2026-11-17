# bev_processor

`bev_processor`는 `camera_driver`의 하단 rectified NV12와 프레임별 IMU 보정
행렬을 받아 CUDA에서 컬러 BEV를 생성한다. 차선과 중앙선은 `line_detactor`가 담당한다.
기존 Gray/Top-hat, 상대 대비 시드, 슬라이딩 윈도우, 규칙 기반 중앙선 생성과
관련 `lane_*` 파라미터·launch 인자는 제거했다. `/camera/image_bev_lane`도 발행하지 않는다.
기존 외부 BEV YAML에 남은 `lane_*` 키는 읽지 않으며 삭제해도 된다.

## 처리 순서

1. 시작 시 설정에 따라 OAK stereo depth와 IMU로 카메라 높이·roll·pitch를 측정한다.
2. 측정 자세와 설정된 카메라 X/Y/yaw로 고정 BEV LUT를 만든다.
3. OAK 측정 파이프라인을 닫고 `camera_driver`를 시작한다.
4. CUDA가 NV12 안정화, BEV sampling, BGR 생성을 한 번에 수행한다.
5. 독립 실행은 `/camera/image_bev`를 발행하고, `auto_drive.launch.py`는
   같은 프로세스의 딥러닝 검출기에 CUDA BGR BEV 버퍼를 직접 넘긴다.
   기본 결과 프리뷰는 이 버퍼를 TensorRT 입력으로 사용하므로 CPU BEV
   다운로드를 생략한다. 원본 오버레이의 `direct_host_copy_enabled=true`도
   입력마다 복사하지 않고, GUI가 실제 표시할 때 전용 CUDA 스트림으로 가져온다.
   같은 촬영본의 CPU 이미지는 한 번만 복사해 캐시한다. 캡처는 같은 함수를
   공유하며 원래 촬영본의 GPU 버퍼 소유권을 유지한다. 데이터셋 저장·ROS 영상
   발행·독립 BEV 프리뷰에서 필요한 기존 즉시 호스트 복사는 유지한다.

차량은 시작 측정이 완료될 때까지 정지해야 한다.

## 실행

저장소 루트에서 빌드하고 실행한다.

```bash
source /opt/ros/humble/setup.bash
colcon build \
  --packages-up-to bev_processor \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch bev_processor bev_processor.launch.py
```

## 학습 데이터 수집

자동수집의 `dataset_collection_enabled:=true` 또는 수동수집의
`dataset_collection_manual_capture_mode:=true`로 실행하면 지정한 root의
기존 `dataset_숫자` 폴더를 검색해 다음 번호를 생성한다. 수집을 새로
시작할 때마다 `dataset_001`, `dataset_002`, ... 순서로 늘어난다.

```text
datasets/dataset_001/
└── origin_bev/<capture_time_ns>_<matching_number>.png
```

기본 BEV는 세로 300행×가로 120열이다. `origin_bev`에 오버레이 없는 BGR만 저장한다.
규칙 기반 검출을 제거했으므로 `filtered_bev`, `result_bev`, `label`은 새로 생성하지 않는다.
기존 수집 폴더는 유지하며, 새 수집의 라벨링은 별도로 수행해야 한다.

자동주행에서는 기존 명령에 수집 인자만 추가하면 된다. 값을
`bev_params_file`의 `bev_processor.ros__parameters`에 넣어도 동일하다.

```bash
ros2 launch vehicle_bringup auto_drive.launch.py \
  bev_params_file:=/absolute/path/bev_config_test.yaml \
  auto_control_params_file:=/absolute/path/auto_control_test.yaml \
  auto_enabled:=true \
  preview_enabled:=true \
  dataset_collection_enabled:=true \
  dataset_collection_root_directory:=/absolute/path/datasets \
  dataset_collection_fps:=10.0 \
  dataset_collection_target_count:=1000
```

목표 개수를 채우면 수집 스레드가 종료되고, 기본값으로
`/auto/enabled=false`를 발행해 자동주행을 정지한다. 이 발행이 필요
없으면 `dataset_collection_stop_auto_on_complete:=false`로 지정한다.
수동 버튼 수집에서는 `bev_processor`를 수동 캡처 모드로 실행한다.
`/autopilot03/joy`의 컨트롤러 A 버튼(SDL index 0)을 누르는 상승 에지마다
자동수집과 동일하게 원본 BEV 한 장을 저장한다. 기존 B 버튼은
`capture_directory`에 원본 BEV 한 장만 저장하는 기능으로 그대로 남는다.
자동 FPS 수집과 수동 캡처 모드는 동시에 활성화할 수 없다.

```bash
# 터미널 1: BEV 처리 + A 버튼 데이터셋 수집
ros2 launch bev_processor bev_processor.launch.py \
  preview_enabled:=true \
  dataset_collection_manual_capture_mode:=true \
  dataset_collection_root_directory:=/home/autopilot03/Desktop/0906ML/f1tenth_project_repo/datasets \
  dataset_collection_target_count:=1000 \
  capture_joy_topic:=/autopilot03/joy

# 터미널 2: 조이스틱 수동주행
ros2 launch vehicle_bringup manual_drive.launch.py \
  vehicle_namespace:=autopilot03 \
  vesc_port:=/dev/ttyTHS1 \
  controller_name_contains:=8BitDo
```

CAN과 주행 중 가속도계 보정을 모두 끄고 gyro 고주파 진동만 억제하려면:

```bash
ros2 launch bev_processor bev_processor.launch.py \
  preview_enabled:=true \
  imu_stabilization_enabled:=true \
  imu_stabilization_high_frequency_only:=true \
  imu_stabilization_high_frequency_vibration_cutoff_hz:=3.0 \
  imu_stabilization_gyroscope_correction_gain:=1.0 \
  imu_stabilization_invalid_correction_hold_frames:=2
```

`cutoff_hz`를 높이면 더 빠른 진동만 보정하고, 낮추면 느린 흔들림까지
포함한다. `gyroscope_correction_gain`은 `0.0`이면 영상 회전 보정을 적용하지
않고 `1.0`이면 추정된 고주파 회전을 전량 상쇄한다. 권장 시작값은
`cutoff=3.0 Hz`, `gain=1.0`이다. 이 모드에서는 CAN dynamics가 필요 없다.

CAN 차량 가속도 보정을 사용하려면 먼저 다른 터미널에서
`vehicle_dynamics_monitor`를 SocketCAN 모드로 실행한다.

```bash
ros2 launch vehicle_bringup manual_drive_with_dynamics.launch.py \
  input_mode:=socketcan can_interface:=can0 can_controller_id:=112

# 다른 터미널
ros2 launch bev_processor bev_processor.launch.py \
  imu_stabilization_can_longitudinal_compensation_gain:=0.7 \
  imu_stabilization_can_lateral_compensation_gain:=0.7 \
  imu_stabilization_moving_accelerometer_nudge_strength:=0.15 \
  imu_stabilization_moving_gravity_anchor_maximum_correction_rate_degps:=0.50 \
  imu_stabilization_invalid_correction_hold_frames:=2
```

BEV의 빠른 진동 보정은 기존과 동일한 gyro 전체 대역 방식을 유지한다. CAN
종·횡가속도를 IMU 가속도에서 제거한 잔여 중력은 주행 자세 drift를 줄이는
저주파 persistent anchor와 비누적 bounded nudge에 사용한다. 두 CAN gain과
nudge 강도는 `0.0~1.0`, anchor 최대 변화율은 `deg/s` 단위로 실행할 때
조절할 수 있다. camera/IMU 매칭이 한두 프레임 실패하면 직전 정상 보정 행렬을
유지하고, 지정 횟수를 넘긴 연속 실패에만 zoom-only로 돌아간다.

CUDA 컴파일러를 자동으로 찾지 못하면 빌드 인자에
`-DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc`를 추가한다.

BEV 노드는 1초마다 `/perf/bev_stage`(JSON 문자열)로 BEV 입력 Hz와
BEV 입력 수신→BEV 준비 완료 구간의 Hz·평균·최대 ms를 발행한다. 상세 지연 로그는 `perf_log_enabled: true`로 켠다.
GUI 없이 실행하려면 `preview_enabled:=false`를 쓴다.

## BEV 보간과 프리뷰

`bev_interpolation`은 `bilinear`, `bicubic`, `adaptive`를 지원한다.
`edge_*`는 원거리 BEV 영상의 보간 품질에 쓰는 설정이므로 유지한다.
이 설정은 차선 검출이나 슬라이딩 윈도우가 아니다.

단독 실행의 `preview_enabled:=true`는 원본 BGR BEV를 표시한다.
자동주행 launch는 이 창을 끄고 `line_detactor`의 좌우 차선·노란 중앙선 프리뷰를 표시한다.

```bash
ros2 launch bev_processor bev_processor.launch.py \
  bev_params_file:=/absolute/path/bev_config_test.yaml \
  camera_params_file:=/absolute/path/camera_config_test.yaml \
  publish_enabled:=true \
  preview_enabled:=true
```

차선 폭, 중앙선 평활화, 코너 바깥 차선 가중치는
`line_detactor/config/line_detactor.yaml`에서 조정한다. 자동주행에서는
`line_detactor_params_file:=/absolute/path/line_detactor_test.yaml`로 전달한다.

## BEV 범위

BEV 출력 크기는 다음 식과 일치해야 한다.

```text
output_width  = round((y_max_m - y_min_m) / meter_per_pixel)
output_height = round((x_max_m - x_min_m) / meter_per_pixel)
```

기본 범위는 전방 0~3m, 좌우 -0.6~+0.6m, 1cm/pixel이며 출력은
120x300이다. `bev_interpolation`은 `bilinear`(기본),
`bicubic`, `adaptive` 중 하나를 사용한다.

## 시작 측정과 자세 공유

자동 모드는 OAK stereo depth 중앙 ROI의 노면에 RANSAC/PCA 평면을 맞춰
높이·roll·pitch를 구한다. IMU는 평면 후보 검증과 정지 상태 판정에
사용한다. 측정에 실패하면 임의 외부 파라미터로 계속하지 않고 노드 시작을
중단한다.

`measurement_roi_vertical_offset_px`는 영상 중앙을 기준으로 ROI를
세로로 이동한다. 양수는 아래쪽, 음수는 위쪽이며 기본값 `80`은
ROI를 80px 아래로 내린다. `measurement_roi_preview_enabled:=true`이면
시작 depth 측정 동안 회색 depth 영상에 ROI를 1px 빨간색 테두리로
표시하고, 측정이 끝나면 창을 자동으로 닫는다. 그래픽 화면이 없으면 프리뷰를 끄고 측정을
계속한다.

측정 후 사용한 지면 법선은 `/camera/startup_ground_normal`에
reliable + transient-local QoS로 한 번 발행한다. `camera_driver`도
같은 기준을 받아 프레임별 안정화 행렬을 만든다. LUT 생성 후에는 주행 중
자세 변화에 따라 LUT를 다시 만들지 않으며, 동적 보정 행렬이 CUDA sampling에
매 프레임 반영된다.

높이를 수동으로 쓰려면 `config/bev_config.yaml`에서 다음 값을
설정한다.

```yaml
manual_camera_height_enabled: true
manual_camera_height_m: 0.20
```

수동 높이 모드에서도 roll/pitch를 위한 IMU 안정화와 bias 보정은 수행한다.
카메라 X/Y/yaw는 자동 측정 대상이 아니므로 실제 장착값을 설정해야 한다.

사용 파일:

- launch: `launch/bev_processor.launch.py`
- BEV 투영·수집 설정: `config/bev_config.yaml`
- 카메라 설정: `camera_driver/config/camera_config.yaml`

초기 IMU·바닥 평면 측정 구현은 `oak_startup` 공용 패키지로 분리되어 `depth_lidar`도 같은 알고리즘을 사용합니다. BEV의 기존 측정 파라미터와 처리 순서는 유지합니다. 기존 설치에서도 `oak_startup`과 `bev_processor`를 함께 다시 빌드해야 합니다.

## auto_control 장애물 매핑용 기하 공유

`obstacles.share_reference`를 켜면 시작 측정한 장치 ID·카메라 자세·BEV 범위를 같은 프로세스의
`auto_control::ObstacleDetectorNode`에 공유한다. 바닥 제거/군집 검출은 auto_control 패키지에서
수행하고 결과는 기존 차선 프리뷰에 합성한다. [통합 실행](../auto_control/AUTO_CONTROL_OBSTACLES.md).
