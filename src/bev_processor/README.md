# bev_processor

`camera_driver`가 IMU로 안정화해 발행하는 전체 1280x720 NV12 영상을
CUDA에서 컬러 BEV로 변환하는 ROS 2 C++ 패키지다. 실행 모드는 하나이며,
시작할 때 카메라 높이·roll·하향 pitch를 반드시 측정한다.

## 작동 순서

1. `bev_processor`가 OAK를 먼저 단독으로 연다.
2. 차량이 정지한 상태에서 stereo depth 중앙 ROI의 노면 평면과 IMU 중력
   방향을 측정한다.
3. 노면 평면에서 카메라 높이를 구하고, `measurement_attitude_source`에서
   선택한 `depth` 또는 `imu`로 roll과 하향 pitch를 구한다.
4. 측정한 높이·roll·pitch와 설정 파일의 X/Y/yaw로 BEV LUT를 한 번 만든다.
5. OAK 측정 파이프라인을 닫고 `camera_driver`를 시작한다.
6. 카메라 드라이버가 roll/pitch 흔들림을 영상에서 보정하고,
   `bev_processor`는 시작 LUT를 바꾸지 않은 채 컬러 BEV 변환만 수행한다.

높이는 항상 depth 노면 평면에서 구한다. 시작 측정에 실패하면 임의의
수동 외부 파라미터로 계속하지 않고 노드 시작을 중단한다. LUT 생성 후에는
BEV 노드가 IMU를 구독하거나 자세 변화에 따라 LUT를 다시 만들지 않는다.

카메라 X/Y 위치와 yaw는 시작 측정으로 구하지 않으므로 실제 장착값을
`config/bev_config.yaml`에 입력해야 한다. 높이·roll·pitch 입력 항목은 없고
시작 측정 결과만 사용한다.

## 실행

측정이 끝날 때까지 차량을 완전히 정지시키고, 카메라 중앙에 장애물 없는
평평한 노면이 보이게 한다. 이후 카메라 드라이버 안정화기의 초기 400개
IMU 샘플 수집이 끝날 때까지 약 1초 더 정지 상태를 유지한다.

```bash
ros2 launch bev_processor bev_processor.launch.py
```

사용 파일은 하나씩이다.

- launch: `launch/bev_processor.launch.py`
- BEV 설정: `config/bev_config.yaml`
- 카메라 설정: `camera_driver/config/camera_config.yaml`

launch는 두 노드를 같은 multi-threaded component container에 올리고
intra-process 통신을 사용한다. BEV 시작 측정이 OAK 장치를 반환한 다음
카메라 드라이버가 장치를 연다. 카메라 원본 프리뷰는 끄고 작은 BEV 결과만
프리뷰한다.

## 변환 로직

`CudaBevProcessor`는 다음 처리만 수행한다.

1. BEV LUT 좌표에서 NV12 Y/UV 값을 bilinear 보간한다.
2. YUV를 BGR로 변환한다.
3. LUT 기반 BEV 워핑 결과를 `bgr8`로 발행한다.

Sobel, 미분 필터, 대비 강화, 밝기 임계값, morphology, 차선 추출과 상단
크롭은 적용하지 않는다. BEV 프리뷰에만 격자와 중심선을 표시한다.

## 시작 측정

OAK stereo depth의 중앙 ROI에 RANSAC/PCA 평면을 맞춘다. 안정된 평면
30프레임의 높이와 평균 법선을 구하고, 정지 상태에서 IMU 중력 방향도
평균한다. 높이는 항상 depth 평면을 사용한다. roll/pitch는
`measurement_attitude_source: "depth"`이면 평면 법선,
`measurement_attitude_source: "imu"`이면 bias 보정된 IMU 중력 방향을
그대로 사용하며 두 결과를 융합하지 않는다. Pro-series OAK에서는 시작 측정 동안
IR dot projector를 사용해 무늬가 적은 노면의 stereo 대응점을 보강한다.

각 측정 파라미터의 선정 방법과 조정 방향은 `config/bev_config.yaml`의
한글 주석에 적혀 있다. IMU 장착 bias는
`measurement_imu_roll_bias_deg`와 `measurement_imu_pitch_bias_deg`로
보정하며, 평평한 기준면에서 반복 측정한 일정한 편차가 확인되기 전에는
0을 유지한다.

정상 시작 로그에는 다음 항목이 출력된다.

```text
[bev_processor] Measuring startup camera height ... roll/pitch ... source ...
[bev_processor] BEV_STARTUP_MEASUREMENT: source=..., height=..., roll=..., ...
[bev_processor] Startup IMU: ...
[bev_processor] Startup attitude selection: selected=..., ...
[bev_processor] Startup ground-plane diagnostics: ...
[bev_processor] BEV LUT installed from startup depth height + selected attitude: ...
```

상태 로그의 `extrinsics=startup_measured, fixed_lut=true`는 시작 측정 자세의
고정 LUT를 사용 중이라는 뜻이다.

## BEV 범위

BEV 범위와 현재 값은 `config/bev_config.yaml`에서 관리한다. 출력 크기는
다음 식과 일치해야 한다.

```text
output_width  = round((y_max_m - y_min_m) / meter_per_pixel)
output_height = round((x_max_m - x_min_m) / meter_per_pixel)
```

## 빌드

```bash
source /opt/ros/humble/setup.bash
colcon build \
  --packages-select camera_driver bev_processor \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

CUDA 컴파일러를 자동으로 찾지 못하면
`-DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc`를 추가한다.
