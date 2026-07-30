# camera_driver

OAK/DepthAI 카메라 영상을 낮은 지연시간으로 받는 ROS 2 C++ 패키지다.
기본 설정은 다음과 같다.

- 센서 모드/출력: OV9782 `THE_720_P`, `1280x720`, `NV12`
- 요청 센서 FPS: `120`
- USB 최대 속도 요청: `SUPER` (5 Gbps)
- XLink 청크 분할: 비활성화 (`setXLinkChunkSize(0)`)
- 렌즈 왜곡 보정: OAK 장치 내부에서 활성화
- ROS 이미지 발행: 기본 비활성화
- IMU 브리지: 기본 비활성화, 활성화 시 camera optical frame의
  가속도+각속도 발행
- QoS: sensor data, best effort, keep-last 1
- 호스트 큐: 크기 8, non-blocking
- 프리뷰: 캡처와 분리된 최신 프레임 방식

센서 모드는 OV9782의 2-lane `THE_720_P`로 명시하며, 이 모드는 센서
상하를 크롭한 1280x720 출력을 최대 143 FPS로 지원한다. 노드는 요청값과
별도로 측정된 캡처 FPS와 장치 sequence gap을 주기적으로 출력한다.

첫 프레임에는 최종 출력에 적용된 resize와 장치 내부 왜곡 보정을 모두
반영한 `K_rect`의 `fx`, `fy`, `cx`, `cy`도 한 번 출력한다. BEV처럼
투영 기하가 필요한 후속 처리에서는 원본 센서 행렬 대신 이 값을 사용한다.

## 성능 구조

카메라 캡처, ROS 발행, OpenCV 프리뷰는 서로 다른 스레드에서 실행된다.
캡처 스레드는 DepthAI 큐를 비우고 최신 프레임 포인터만 교체하며, ROS
메시지 복사나 `imshow()`를 수행하지 않는다. 발행이나 프리뷰가 늦어지면
오래된 프레임을 쌓지 않고 최신 프레임으로 건너뛴다.

OAK에서 `NV12`를 생성해 Jetson으로 전송한다. BGR888i보다 전송량이 절반
이하이므로 USB/XLink 병목을 줄인다. 캡처 스레드는 `ImgFrame` 패킷만
보관하며 색 변환을 하지 않는다. ROS 토픽 발행을 선택한 경우 원본 NV12를
`sensor_msgs/Image` 데이터로 한 번 복사한다.

NV12 메시지는 `encoding="nv12"`, `height=720`, `step=Y/UV plane stride`를
사용하고, `data`에는 Y plane 720행 다음에 interleaved UV plane 360행이
연속으로 들어간다. 일반 BGR8 구독자가 아니라 NV12를 이해하는 처리 노드가
받아야 한다. `bev_processor`는 같은 컴포넌트 컨테이너 안에서 이 메시지를
받아 CUDA로 바로 BEV를 생성한다.

왜곡 보정은 `Camera::requestOutput(..., enableUndistortion=true)`로 요청한다.
따라서 호스트에서 `cv::remap()`을 수행하지 않는다.

IMU 브리지를 켜면 raw accelerometer와 raw gyroscope를 같은 400 Hz로
요청하고, factory IMU-to-camera 회전행렬을 두 벡터에 모두 적용해
`sensor_msgs/Imu`로 발행한다. orientation 자체는 채우지 않으며,
`bev_processor_auto`가 두 벡터를 융합해 roll/pitch 변화량을 추정한다.

ROS 발행은 `sensor_msgs/msg/Image`의 `UniquePtr`를 사용한다. 기본 launch는
컴포넌트 컨테이너에서 intra-process 통신을 활성화한다. 향후 C++ 영상 처리
컴포넌트를 같은 컨테이너에 적재하면 DDS 직렬화 없이 메시지 소유권을 넘길
수 있다. 별도 프로세스의 구독자, `ros2 topic hz`, rosbag 등은 DDS 전송과
추가 메모리 복사를 사용한다.

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

### Jetson에 DepthAI C++ 설치

ROS 2 Humble을 사용하는 Ubuntu/Jetson에서 필요한 기본 패키지를 설치한다.

```bash
sudo apt update
sudo apt install -y \
  build-essential git cmake libudev-dev libopencv-dev
```

DepthAI C++ 3.x 소스를 받아 공유 라이브러리로 빌드한다. 메모리가 부족한
Jetson을 고려해 병렬 빌드 수는 2로 제한한다.

```bash
cd ~/Desktop/f1tenth_test0724
git clone \
  --branch v3.6.1 \
  --depth 1 \
  --recurse-submodules \
  https://github.com/luxonis/depthai-core.git

cmake \
  -S depthai-core \
  -B depthai-core/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON \
  -DDEPTHAI_OPENCV_SUPPORT=ON \
  -DCMAKE_INSTALL_PREFIX=/usr/local

cmake --build depthai-core/build --parallel 2
sudo cmake --install depthai-core/build
sudo ldconfig
```

설치 결과를 확인한다.

```bash
find /usr/local -name depthaiConfig.cmake -print
```

일반적으로 다음 경로가 출력된다.

```text
/usr/local/lib/cmake/depthai/depthaiConfig.cmake
```

다른 prefix에 설치했다면 워크스페이스 빌드 시 해당 위치를 직접 전달한다.

```bash
colcon build \
  --packages-select camera_driver \
  --cmake-clean-cache \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -Ddepthai_DIR=/path/to/depthai-install/lib/cmake/depthai
```

DepthAI는 OpenCV 지원을 켜고 빌드되어야 한다. 카메라 단독 프리뷰를 켠
경우에만 `ImgFrame::getCvFrame()`으로 NV12를 CPU BGR로 변환해 OpenCV
창에 표시한다. GPU BEV 통합 launch에서는 이 프리뷰를 끈다.

## 빌드

Jetson에서 워크스페이스 루트로 이동한 후 Release 모드로 빌드한다.

```bash
cd ~/Desktop/f1tenth_test0724/f1tenth_project_repo
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select camera_driver \
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

기본 독립 프리뷰에는 원본 영상 좌표 기준 20픽셀 간격의 연한 회색 격자가
표시된다. 격자만 끄려면 다음처럼 실행한다.

```bash
ros2 launch camera_driver camera_driver.launch.py \
  preview_enabled:=true preview_grid_enabled:=false
```

OAK IMU로 프리뷰와 ROS NV12 출력의 고주파 roll/pitch 흔들림을
안정화하려면 다음처럼 실행한다.

```bash
ros2 launch camera_driver camera_driver.launch.py \
  preview_enabled:=true imu_stabilization_enabled:=true
```

안정화기는 처음 200개 IMU 샘플로 중력 방향과 자이로 bias를 측정하므로
시작 후 약 0.5초 동안 차량을 정지시킨다.
이후 400 Hz IMU 자세에서 느린 카메라 궤적을 분리하고, 영상 timestamp에
해당하는 고주파 roll/pitch 보정값을 보간한다. 카메라 내부 파라미터로
회전 homography를 만들어 프리뷰 BGR 또는 발행 NV12의 Y/UV 평면을
워핑한다. 영상 가장자리에는 보정으로 인한 검은 영역이 생길 수 있다.
워핑은 현재 CPU OpenCV 경로이므로 활성화 후 상태 로그의 실제 capture
FPS와 누락 프레임 수를 Jetson에서 확인해야 한다.

> `bev_processor`의 `realtime_attitude_enabled`와 카메라 IMU 안정화를
> 동시에 켜면 같은 roll/pitch 회전을 두 번 보정할 수 있다. 안정화된
> 카메라 출력을 BEV 입력으로 사용할 때는 둘 중 하나만 활성화한다.

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

노드는 기본 5초마다 다음 항목만 간단히 출력한다.

- `capture`: 실제 DepthAI 프레임 수신 FPS와 요청 FPS
- `preview`: 실제 프리뷰 갱신 FPS
- `IMU`: 안정화/ROS 브리지에서 실제 처리한 IMU 샘플 rate
- `stabilizer`: `off`, `warmup`, `ready` 상태와 누적 warp/miss 수
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
| `sensor_fps` | `120.0` | OAK 센서/출력 요청 FPS |
| `width`, `height` | `1280`, `720` | 출력 해상도 |
| `undistort_enabled` | `true` | OAK 장치 내부 왜곡 보정 |
| `queue_size` | `8` | DepthAI 호스트 큐 크기 |
| `queue_blocking` | `false` | 큐가 찼을 때 캡처 차단 여부 |
| `publish_enabled` | `false` | ROS 이미지 발행 |
| `publish_fps` | `120.0` | ROS 발행 목표 최대 FPS |
| `imu_bridge_enabled` | `false` | 가속도+자이로 ROS 발행 |
| `imu_rate_hz` | `400.0` | raw accel+gyro 동기 요청/발행 rate |
| `imu_topic` | `/camera/imu` | `sensor_msgs/Imu` 출력 |
| `imu_stabilization_enabled` | `false` | IMU 영상 안정화 |
| `imu_stabilization_warmup_samples` | `200` | 초기 bias/중력 평균 샘플 수 |
| `imu_stabilization_acceleration_time_constant_sec` | `1.5` | 중력 방향 보정 시정수 |
| `imu_stabilization_acceleration_gate_deg` | `8.0` | 동적 가속도 보정 거부 각도 |
| `imu_stabilization_smoothing_time_constant_sec` | `0.30` | 느린 카메라 궤적 평활화 시정수 |
| `imu_stabilization_maximum_correction_deg` | `4.0` | 축별 최대 영상 보정각 |
| `imu_stabilization_roll_gain` | `1.1` | roll 보정 방향/크기 |
| `imu_stabilization_pitch_gain` | `1.1` | pitch 보정 방향/크기 |
| `preview_enabled` | `false` | OpenCV 직접 프리뷰 |
| `preview_fps` | `60.0` | 프리뷰 갱신 목표 최대 FPS |
| `preview_grid_enabled` | `true` | 독립 프리뷰 격자 표시 |
| `preview_grid_spacing_px` | `20` | 원본 영상 기준 격자 간격 |

143 FPS에서 `1280x720 NV12`의 순수 영상 데이터는 약 189 MiB/s다.
`BGR888i`의 약 377 MiB/s보다 작다. 외부 프로세스 구독자는 DDS 직렬화와
추가 복사를 사용하므로, 주 영상 처리는
`bev_processor_auto.launch.py`/`bev_processor_manual.launch.py`처럼 같은
프로세스의 intra-process C++ 컴포넌트로 구성하는 것이 좋다.
