# ir_camera_driver

OAK-D Pro W의 CAM_B/C 모노 카메라와 IR emitter를 사용하는 독립 터널
가시성 튜닝 패키지다. 원본 IR 영상과 자동 추정·IMU 보정을 적용한 지면 BEV를 최대 30 FPS로
표시하고, 별도 제어 창에서 flood·노출 시간·ISO를 실행 중 직접 입력할 수 있다.
다음 두 실행 모드를 제공한다.

- `reprojection_enabled=true`: 좌우 rectified 영상과 center-aligned disparity를
  이용해 스테레오 광학 중심 사이의 가상 시점 영상 하나를 만든다.
- `reprojection_enabled=false`: `selected_camera`로 선택한 LEFT/CAM_B 또는
  RIGHT/CAM_C 렌즈 영상을 그대로 표시한다.

OAK 장치는 한 프로세스가 배타적으로 연다. 이 패키지를 실행하는 동안
`camera_driver`, `bev_processor`, `camera_height_estimator` 또는
`depthai_ros_driver`를 동시에 실행하면 안 된다.

## 중앙 가상 시점 재투영

DepthAI `StereoDepth`가 공장 캘리브레이션으로 왜곡 보정과 stereo
rectification을 수행하고, disparity를 두 카메라 사이의 지정 위치에 맞춘다.
호스트의 CUDA kernel은 각 가상 픽셀에서 다음 위치의 좌우 밝기를 bilinear
sampling한 뒤 평균한다.

```text
r = virtual_camera_position_ratio
x_left  = x_virtual + r * disparity
x_right = x_virtual - (1-r) * disparity
I_virtual = 0.5 * (I_left(x_left) + I_right(x_right))
```

`r=0.5`는 두 스테레오 렌즈의 정확한 중간이다. disparity가 없거나 영상
경계를 벗어난 픽셀은 `selected_camera` 영상으로 채워 검은 구멍을 피한다.

이 결과가 차량 중심 시점이 되려면 스테레오 렌즈 중간이 차량 중심선에
장착되어 있어야 한다. 좌우 렌즈의 차량 기준 횡방향 위치를 각각
`y_left`, `y_right`로 실측했다면 차량 중심 `y=0`에 해당하는 비율은 다음과
같이 설정할 수 있다.

```text
r = (0 - y_left) / (y_right - y_left)
```

장치 전체가 차량 중심에서 앞뒤로 벗어났거나 yaw/roll/pitch가 틀어진 경우는
이 비율만으로 보정되지 않는다. 그 경우에는 차량 `base_link`에 대한 별도
외부 캘리브레이션이 필요하다. 이번 패키지는 IR 차선 가시성 시험을 위한
스테레오 사이 가상 시점까지만 만든다.

## 원본/BEV 프리뷰

기본 카메라 요청 속도와 원본/BEV 화면 갱신 상한은 모두 30 FPS다. 캡처
스레드는 DepthAI의 크기 1/non-blocking 큐를 계속 비우고 최신 프레임만
공유하므로 GUI가 느려져도 과거 프레임이 쌓이지 않는다. BEV는 같은 시각의
원본으로 만들며, IMU 보정을 적용할 수 없는 시각에는 BEV를 표시하지 않는다.

- 중앙 재투영: `1280x800 @ 30 FPS`
  - RVC2 800P integer disparity 구성
  - CENTER 정렬에 필수인 left-right check 활성화
  - subpixel과 추가 후처리 필터 비활성화
  - 중앙 합성은 Jetson CUDA에서 수행
- 단일 렌즈: `1280x800 @ 30 FPS`
  - 초기 높이 자동 측정이 끝나면 선택한 렌즈와 IMU만 사용
  - 실행 중 stereo/disparity/CUDA 재투영을 사용하지 않음

## BEV 자동 추정과 IMU 보정

`bev_processor`의 `bev_geometry`와 `oak_startup` 측정 라이브러리,
`camera_driver`의 `ImuImageStabilizer`를 직접 재사용한다. 다른 카메라 노드를
실행하지 않고 이 노드가 장치 연결과 시작 측정, 실행 중 IMU 수집을 관리한다.

1. 차량을 평평한 지면에 정지시킨다. 기본 2초 warmup 후 calibrated IMU
   1,200개 표본으로 정지 상태를 확인하고 stereo depth ROI에 RANSAC 지면
   평면을 맞춘다. 기본적으로 연속 45개 유효 평면의 높이·법선 안정성을 검사한다.
2. 지면에서 카메라 높이와 roll/pitch를 추정한다. `measurement_attitude_source`
   기본값은 `depth`이고 IMU와의 각도 차이도 검사한다. `imu`로 변경하면
   자세는 IMU에서, 높이는 지면에서 얻는다. 품질 기준을 충족하지 못하면
   45초 timeout 후 종료하며 고정 높이로 자동 대체하지 않는다.
3. 측정에 사용한 장치를 닫고 **같은 device ID**로 프리뷰 파이프라인을 연다.
   단일 렌즈 모드는 이후 선택한 IR 카메라 한쪽과 IMU만 사용한다.
4. 실행용 IMU에서 기본 1초 초기 표본 폐기 + 4초 정지 보정으로 gyro bias를
   구한다. 측정된 지면 법선을 고정 기준으로 사용하고, 각 영상의 **노출 중간
   시각**에 해당하는 roll/pitch 변화를 BEV 투영에 반영한다.

측정 중에는 `measurement_ir_dot_projector_intensity`(기본 `1.0`)로 닷
프로젝터를 사용한다. 프리뷰 시작 후에는 `ir_dot_projector_intensity`와
`ir_flood_light_intensity`를 적용하므로 런타임 dot=`0.0`, flood=`0.5`도 가능하다.
측정 중에도 닷을 끄려면 별도 측정 인자를 `0.0`으로 지정한다. 이때 지면의
텍스처·조명이 부족하면 높이 추정이 실패할 수 있다.

측정 기준 좌표는 `bev_processor`와 같은 **CAM_A(RGB) 광학 좌표**다.
출력 프레임의 extrinsics와 EEPROM의 기준 카메라→CAM_A 변환을 합성해
실제 IR 영상 좌표와 렌즈 위치를 계산한다. 출력 메타데이터에 포함된 stereo
rectification 회전을 중복 적용하지 않는다. 내부 파라미터도 출력 프레임에서
읽으며, intrinsics/extrinsics가 없는 프레임은 BEV로 쓰지 않는다.
단일 렌즈도 `undistort_single_camera: true`가 필요하다.

`bev.camera_x_m`, `bev.camera_y_m`, `bev.camera_yaw_deg`는 **CAM_A의 차량
기준 장착 위치·yaw**를 직접 입력한다. 높이와 roll/pitch만 자동 추정되며,
IMU만으로 차량 기준 x/y/yaw를 자동 결정할 수는 없다. 기존 IR 렌즈 기준
장착값을 사용했다면 CAM_A 기준 실측값으로 변경해야 한다.

기본 BEV 범위는 전방 0~3m, 좌우 ±0.6m, 1cm/pixel (`120x300`)이다.
`bev_processor`와 같은 픽셀 중심과 차량 축(전방/좌측/상방)을 사용한다.
IMU 회전을 반영해 투영 맵을 갱신하고, RGB→IR 렌즈 간 위치 차이도 함께
회전시킨다. 중앙 가상 시점 모드의 BEV는 좌우 rectified 영상을 각각 같은
지면 격자로 투영한 뒤 유효 영역을 합성한다. 가상 시점 프리뷰의 disparity
실패 대체 픽셀을 다른 렌즈의 좌표로 잘못 해석하는 문제를 피한다.

IMU 보정 준비 전, 시각 동기화 실패, 기본 ±3도 보정 한계 초과 시 해당 BEV를
표시·저장하지 않는다. 원본 프리뷰는 계속 표시하고 BEV 창에는 대기 상태를
표시한다. 실행용 IMU 정지 보정도 30초 내 완료되지 않으면 종료한다.
`[IR_CAMERA]` 로그의 `IMU`, `samples`, `BEV_rejected`, `tilt`와
`IR_BEV_STARTUP`의 측정 품질 수치를 확인한다.

## IR와 노출

기본값은 IR laser dot projector `1.0`, flood light `0.0`이다. Dot projector는
stereo disparity용 texture를 만들고, flood light는 어두운 장면을 균일하게
비추는 용도다. 터널 차선 자체의 IR 반사를 확인하려면
`ir_flood_light_intensity`도 단계적으로 올려 비교할 수 있다.

수동 노출이 기본값이다. 제어 창에서 값을 적용하면 좌우 카메라에 같은 노출과
ISO가 즉시 전달된다. `AUTO EXPOSURE` 버튼 또는 `A` 키로 자동 노출을 다시
켤 수 있다. 설정 노출 시간은 프레임 주기보다 짧아야 하며 30 FPS 기본값에서
허용 범위는 `10~33333 us`다.

## 빌드

```bash
cd ~/Desktop/0906ML/f1tenth_project_repo
source /opt/ros/humble/setup.bash

colcon build \
  --packages-up-to ir_camera_driver \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

source install/setup.bash
```

공용 라이브러리 변경을 포함하므로 `--packages-up-to`로 의존 패키지도 빌드한다.
DepthAI C++ 3.6 이상, OpenCV 4, CUDA Toolkit이 필요하다. 기존
`camera_driver`와 `bev_processor`를 빌드할 수 있는 Jetson 환경이면 동일한
의존성을 사용할 수 있다.

## 실행

왼쪽 IR 카메라 + 자동 높이/자세 추정 + IMU 보정, 실행 중 닷 OFF·플루드 ON:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ir_camera_driver ir_camera_driver.launch.py \
  reprojection_enabled:=false \
  selected_camera:=LEFT \
  ir_enabled:=true \
  ir_dot_projector_intensity:=0.0 \
  ir_flood_light_intensity:=0.5
```

초기 자동 높이 측정에만 양쪽 렌즈와 닷을 사용한다. 시작부터 한쪽 렌즈만
사용해야 한다면 **실측한 CAM_A 높이**를 제공한다. 아래 `0.20`은 예시이며,
이 모드는 높이를 자동 추정하지 않고 자세만 IMU로 측정한다.

```bash
ros2 launch ir_camera_driver ir_camera_driver.launch.py \
  reprojection_enabled:=false selected_camera:=LEFT \
  measurement_manual_camera_height_enabled:=true \
  measurement_manual_camera_height_m:=0.20 \
  ir_enabled:=true \
  ir_dot_projector_intensity:=0.0 \
  ir_flood_light_intensity:=0.5
```

자동 높이 추정을 유지하면서 측정 중 닷도 끄려면 첫 명령에
`measurement_ir_dot_projector_intensity:=0.0`을 추가한다.

중앙 가상 시점과 IR dot projector:

```bash
ros2 launch ir_camera_driver ir_camera_driver.launch.py \
  reprojection_enabled:=true \
  virtual_camera_position_ratio:=0.5 \
  ir_dot_projector_intensity:=1.0
```

재투영 없이 왼쪽 렌즈를 기본 30 FPS로 표시:

```bash
ros2 launch ir_camera_driver ir_camera_driver.launch.py \
  reprojection_enabled:=false selected_camera:=LEFT
```

재투영 없이 오른쪽 렌즈를 표시:

```bash
ros2 launch ir_camera_driver ir_camera_driver.launch.py \
  reprojection_enabled:=false selected_camera:=RIGHT
```

IR flood light도 함께 사용:

```bash
ros2 launch ir_camera_driver ir_camera_driver.launch.py \
  ir_dot_projector_intensity:=1.0 \
  ir_flood_light_intensity:=0.5
```

실행하면 원본, BEV, `OAK IR live controls`의 세 창이 열린다. 제어 창의
입력 칸을 마우스로 클릭하고 숫자를 입력한 다음 Enter 또는 `APPLY` 버튼을
누르면 다음 값이 즉시 적용된다.

- Flood intensity: `0.0~1.0`
- Exposure: `10~33333 us` (카메라 FPS를 바꾸면 최대값도 바뀜)
- ISO: `100~1600`

키 조작:

- `I`: 설정된 dot/flood intensity로 IR ON/OFF 전환
- `A`: 자동 노출로 전환
- `Tab`: 다음 입력 칸 선택
- `B`: 현재 원본과 BEV를 각각 PNG로 저장
- `Q` 또는 `Esc`: 종료

저장 파일명에는 `center/left/right`, `ir_on/ir_off`, `original/bev` 상태가
포함된다. 유효 BEV가 없는 동안에는 PNG 쌍을 저장하지 않는다.

## 주요 파라미터

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `reprojection_enabled` | `true` | stereo 중앙 가상 시점 생성 여부 |
| `selected_camera` | `LEFT` | 재투영 OFF 영상 및 invalid disparity fallback |
| `virtual_camera_position_ratio` | `0.5` | LEFT 0.0에서 RIGHT 1.0 사이 가상 위치 |
| `reprojection_fps` | `30.0` | 800P stereo 카메라 요청 FPS |
| `single_camera_fps` | `30.0` | 800P 단일 카메라 요청 FPS |
| `ir_enabled` | `true` | 시작 시 IR emitter 활성화 |
| `ir_dot_projector_intensity` | `1.0` | dot projector 세기 `0.0~1.0` |
| `ir_flood_light_intensity` | `0.0` | flood light 세기 `0.0~1.0` |
| `manual_exposure_enabled` | `true` | 좌우 동일 수동 노출 적용 |
| `manual_exposure_us` | `5000` | 수동 노출 시간 |
| `manual_sensitivity_iso` | `800` | 수동 ISO `100~1600` |
| `preview_max_fps` | `30.0` | 원본/BEV 프리뷰 공통 FPS 상한 |
| `bev.startup_measurement_enabled` | `true` | 시작 시 지면·IMU 자동 측정 |
| `measurement_attitude_source` | `depth` | 자세 추정 기준: `depth` 또는 `imu` |
| `measurement_ir_dot_projector_intensity` | `1.0` | 시작 측정 중 닷 세기; 런타임 설정과 독립 |
| `measurement_manual_camera_height_enabled` | `false` | stereo 높이 측정 생략, 실측 CAM_A 높이 + IMU 자세 |
| `measurement_manual_camera_height_m` | `0.20` | 수동 모드 CAM_A 높이; 반드시 실측값 입력 |
| `imu_stabilization_enabled` | `true` | 실행 중 영상 시각에 맞춘 IMU tilt 보정 |
| `imu_stabilization_maximum_correction_deg` | `3.0` | roll/pitch 보정 허용 범위 |
| `bev.camera_height_m` | `0.20` | 자동 측정을 명시적으로 껐을 때만 쓰는 CAM_A 높이 |
| `bev.camera_pitch_down_deg` | `14.0` | 자동 측정을 껐을 때만 쓰는 CAM_A pitch |
| `bev.*_m`, `bev.meter_per_pixel` | `0~3m`, `±0.6m`, `0.01` | BEV 범위와 해상도 |
| `capture_directory` | `.` | `B` 키 PNG 저장 위치 |

## 제한사항

- 깊이 불연속, 가림 영역, 반사체에서는 disparity가 없거나 잘못될 수 있으며
  해당 픽셀은 선택한 렌즈 영상으로 대체된다.
- 좌우 밝기 차이가 크면 평균 합성 경계가 보일 수 있다. 정량 시험에는 동일
  수동 노출을 권장한다.
- BEV는 평면 노면을 가정한다. IMU는 roll/pitch 변화를 보정하며 장치 전체의
  높이 변화(상하 진동), 차량 이동, 비평면 노면이나 물체의 높이는 복원하지 않는다.
- 실행 중 IMU는 `camera_driver`의 보수적인 기본 융합 설정을 사용한다.
  차량 CAN 가속도 보상은 연결하지 않으며, 움직이는 동안 가속도를 지면
  법선으로 직접 사용하지 않는다. 지속적인 자세 변화·드리프트 보정에는 한계가 있다.
- 추정 품질 기준을 통과해도 실제 거리 정확도는 EEPROM 보정과 x/y/yaw 실측,
  평면 노면 가정에 영향을 받는다. 실차 치수 확인은 별도로 필요하다.
- USB 연결이 HIGH(USB 2)로 표시되거나 Jetson 전력·열 상태가 좋지 않으면
  실제 FPS가 30보다 낮아질 수 있다.
- OAK-D Pro W는 dot projector와 flood LED가 기본적으로 꺼진 장치이므로 이
  패키지가 시작 시 명시적으로 켜고 종료 시 모두 끈다.
