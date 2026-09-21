# camera_recorder

OAK 카메라 한 대에서 아래 세 영상을 같은 시각 기준으로 묶어 저장하는 ROS 2
패키지다.

- `CAM_A`: 중앙 RGB
- `CAM_B`: 왼쪽 흑백/IR
- `CAM_C`: 오른쪽 흑백/IR

수동주행 노드와 녹화 노드는 서로 다른 프로세스로 실행된다. 디스크 인코딩도
카메라 수신과 별도 스레드에서 처리하므로 녹화 지연이 조향/가감속 콜백을
막지 않는다. 녹화 큐가 가득 차면 오래된 영상 묶음을 버리고 상태 로그와
`session_info.txt`에 누락 수를 남긴다.

## 빌드

```bash
cd ~/f1tenth_project_repo
source /opt/ros/humble/setup.bash
colcon build --packages-select camera_recorder \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

DepthAI C++ 3.6 이상과 OpenCV 4가 필요하다. DepthAI는 ROS/ament 패키지
의존성이 아니라 외부 CMake SDK로 찾는다. 별도 경로에 설치했다면 기존
`camera_driver`와 마찬가지로 `-Ddepthai_DIR=...`를 전달한다.

## 독립 실행

저장 경로는 쓰기 속도가 충분한 SSD의 절대 경로를 권장한다.

```bash
ros2 launch camera_recorder camera_recording.launch.py \
  output_directory:=/home/autopilot03/recordings
```

이 실행에는 `vehicle_bringup`, `camera_driver`, `bev_processor`,
`point_cloud`, `oak_startup` 등이 필요하지 않다. 필요한 ROS 의존성은 `rclcpp`,
DepthAI C++ 및 OpenCV뿐이다.

## 수동주행과 동시에 녹화

가장 독립적인 실행 방법은 터미널을 분리하는 것이다. 먼저 수동주행을 실행한다.

```bash
ros2 launch vehicle_bringup manual_drive.launch.py \
  vehicle_namespace:=autopilot03 \
  vesc_port:=/dev/ttyTHS1
```

다른 터미널에서 녹화 노드만 실행한다.

```bash
ros2 launch camera_recorder camera_recording.launch.py \
  output_directory:=/home/autopilot03/recordings
```

`vehicle_bringup`이 정상적으로 빌드된 환경에서는 편의용 통합 런치도 사용할 수
있다. 이 런치만 선택적으로 `vehicle_bringup`을 참조하며, `camera_recorder`의
빌드 의존성은 아니다.

```bash
ros2 launch camera_recorder manual_drive_recording.launch.py \
  output_directory:=/home/autopilot03/recordings \
  vehicle_namespace:=autopilot03 \
  vesc_port:=/dev/ttyTHS1
```

종료할 때는 `Ctrl+C`를 누르고 `Recording closed` 로그가 나올 때까지 기다린다.

`camera_driver`, `ir_camera_driver`, `depth_lidar`처럼 같은 OAK 장치를 직접 여는
노드는 녹화 중 함께 실행하면 안 된다. `manual_drive.launch.py`는 카메라를 열지
않으므로 함께 실행해도 장치 충돌이 없다.

## 저장 결과

실행할 때마다 `drive_YYYYMMDD_HHMMSS` 세션 디렉터리가 생성된다.

```text
drive_20260921_143000/
├── center_rgb_0000.avi
├── left_ir_0000.avi
├── right_ir_0000.avi
├── timestamps.csv
└── session_info.txt
```

기본값은 5분마다 다음 번호의 파일로 분할한다. 세 영상은 항상 같은 세그먼트와
프레임 번호로 기록된다. `timestamps.csv`에는 ROS 수신 시각, 각 센서 시각,
각 카메라 시퀀스 번호, 실제 노출 시간(`*_exposure_us`)과 ISO(`*_iso`)를
남겨 후처리 때 정확히 대응시킬 수 있다.

CSV 한 행은 세 AVI의 동기화된 프레임 한 묶음이다. 예를 들어 CSV의
`segment=2`, `segment_frame=150` 행은 `center_rgb_0002.avi`,
`left_ir_0002.avi`, `right_ir_0002.avi` 각각의 **0부터 시작하는 150번 프레임**과
대응한다. `recorded_frame`은 세션 전체에서 0부터 증가하는 프레임 번호다.
영상 분석 코드에서는 파일을 열고 `segment_frame` 위치의 프레임을 읽은 뒤
같은 CSV 행의 타임스탬프·노출·ISO를 사용하면 된다.

## 주요 설정

`config/camera_recorder.yaml`에서 전체 설정을 바꿀 수 있다.

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `width`, `height` | `1280`, `800` | 세 영상의 저장 해상도 |
| `fps` | `30.0` | 세 카메라 공통 FPS |
| `codec` | `MJPG` | OpenCV FourCC |
| `segment_duration_sec` | `300` | 파일 분할 주기, `0`이면 분할 안 함 |
| `writer_queue_capacity` | `8` | 디스크 쓰기 대기 영상 묶음 수 |
| `ir_dot_projector_intensity` | `0.0` | IR 점 패턴 세기 |
| `ir_flood_light_intensity` | `0.5` | IR 플러드 조명 세기 |
| `ir_manual_exposure_us` | `5000` | 좌우 IR 공통 노출 시간 |
| `ir_manual_sensitivity_iso` | `800` | 좌우 IR 공통 ISO |

닷 프로젝터는 학습 영상에 점 패턴이 남을 수 있어 기본으로 끈다. 어두운 환경의
IR 밝기는 먼저 플러드 세기와 IR 노출을 조절한다. 장치가 IR emitter를 지원하지
않으면 두 intensity를 모두 `0.0`으로 설정한다.

고해상도 MJPG 세 스트림은 저장장치와 CPU 부하가 크다. 첫 시험에서는 차량을
띄운 상태로 1~2분 녹화한 뒤 `[RECORDER] dropped=0`인지, 세 AVI가 재생되는지,
남은 디스크 공간이 충분한지 확인한다.
