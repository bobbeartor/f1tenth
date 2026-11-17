# traffic_detection_test

## 자동 주행에 연결된 상태 검출

`auto_drive.launch.py`는 기본적으로 `TrafficLightNode`를 기존
`bev_processor_container`에 함께 로드한다. 이 컴포넌트는 카메라를 열지 않고
`camera_driver`의 **BEV 변환 전 컬러 NV12 원본**을 공유한다. 지면 투영된
BEV 영상이나 IMU 지면 보정 영상을 신호등 입력으로 사용하지 않는다.
기존 차선 검출과 함께 동작하지만 신호등 화면과 제동/주행 명령은 생성하지 않는다.

BEV 테마에는 `bev_handoff`를 통해 실제 모델 처리 FPS도 공유한다.
최근 약 1초의 성공한 모델 호출 수 / 경과 시간이며 입력 대기와 처리율 제한을 포함한다.
상태 발행 횟수나 `1000/inference_ms`와 다르다. 엔진 준비 중 또는 측정 유실 시
만료된 값은 BEV에서 `--`로 표시하고, 색상 미인식만으로 FPS를 숨기지는 않는다.

- 모델: `traffic_light_yolox_s_640x160_batch_1.int8.qdq.onnx`, TensorRT INT8.
- 카메라 콜백: 이미지 복사 없이 소유권을 가진 참조만 전달하며, 잠금 경합 시 버린다.
- 별도 작업 스레드에서 기본 20Hz로 최신 프레임만 처리한다. 대기 프레임은 최대 1개.
- 별도 nonblocking CUDA stream에서 ROI Y/UV만 GPU로 전송하고
  crop/resize/NV12→BGR NCHW 전처리와 추론을 수행한다. 영상 전체 CPU BGR 변환은 없다.
- 박스 좌표 복원/NMS와 최상위 점수 박스의 HSV 색상 판별은 CPU에서 수행한다.
  색상 판별용 BGR 변환도 해당 박스에만 적용한다.
- 모든 검출 중 최고 점수 박스를 선택하는 기존 규칙을 사용한다.
  내 주행 방향의 신호등 선택은 아직 구현하지 않았다. 정지선 제동은
  auto_control의 별도 개발 기능 `traffic_stop_enabled`로 켠다(기본 false).
  선택적인 연속 관측 확인은 아래 설정으로 켤 수 있으며 기본값은 비활성화다.
- 같은 GPU이므로 차선 추론과의 자원 경쟁은 남는다. 지연이 없다고 보장하지 않는다.

**기존 BEV 차선 검출 프리뷰 하단에 `Traffic: RED / GREEN / UNKNOWN`을 표시한다.**
빨강/초록/회색 글씨로 구분하며 별도 신호등 창이나 토픽 확인 명령은 필요 없다.
표시 상태는 프로세스 내부에서 공유하고, 그리기는 차선 추론이 아닌 GUI 스레드에서
수행한다. 새 차선 프레임이 없어도 상태 변화/유효 시간 만료는 기존 화면에 반영된다.
아래 토픽은 다른 프로그램 연동 또는 진단이 필요할 때만 사용할 수 있다:

```bash
ros2 topic echo /traffic_light/state
```

메시지 `traffic_detection_test/msg/TrafficLightState`의 `state`는
`0=UNKNOWN`, `1=RED`, `2=GREEN`(파란불/초록불)이다. 미검출, 색상 동률,
처리 오류 또는 오래된 입력은 UNKNOWN이다. 입력이 없으면 UNKNOWN을 주기적으로
발행하며 이 경우 헤더는 비어 있다. 초기 엔진 준비 중에도 첫 UNKNOWN을 발행한다.
`header.stamp`는 촬영 시각이며, 이후 소비자는 수신 시각뿐 아니라 이 시각으로
신선도를 판단해야 한다. `inference_ms`는 GPU 추론, `processing_ms`는 작업 스레드
처리 전체, `capture_age_ms`는 결과 생성 시 촬영 이후 경과 시간이다.
입력이 없는 경우 `capture_age_ms=-1`이다.

### 확신도·검출 FPS·연속 확인 YAML

자동 주행용 파일은 **`src/traffic_detection_test/config/traffic_light.yaml`**이다.
`line_detactor_test.yaml`, `auto_control_test.yaml` 또는 단독 프리뷰용
`traffic_detection_test.yaml`에 넣지 않는다. 현재 기본값:

```yaml
traffic_light_detector:
  ros__parameters:
    score_threshold: 0.40
    inference_fps: 20.0
    confirmation_enabled: false
    confirmation_frames: 3
    confirmation_min_iou: 0.30
    confirmation_max_gap_sec: 0.25
```

`score_threshold`는 모델의 objectness × class probability 하한이며 색상 확신도가
아니다. `inference_fps`는 검출 주기 상한으로, 실제 속도를 보장하지 않는다.
모든 값은 시작 시 읽으므로 수정 후 노드를 재시작한다.

`confirmation_enabled: false`에서는 연속 확인을 건너뛰고 현재 결과를 즉시 표시한다.
추후 `true`로 바꾸면 같은 색, 직전 박스와 IoU가 `confirmation_min_iou` 이상,
촬영 간격이 `confirmation_max_gap_sec` 이내인 관측이 `confirmation_frames`회
연속되어야 RED/GREEN을 표시한다. 횟수는 카메라 전체 프레임이 아닌 **실제 추론한
프레임** 기준이다. 20Hz에서 3회는 첫 관측 이후 약 100ms가 추가된다.
미검출/오류/오래된 프레임/입력 중단은 횟수를 초기화하며, 색이나 박스가 바뀌면
새 관측부터 다시 센다. 확인 전에는 UNKNOWN이고 이전 확정 색상을 유지하지 않는다.
`confirmation_frames: 1`이면 첫 유효 관측부터 표시한다. 낮은 FPS를 사용하면서
확인을 켠다면 `confirmation_max_gap_sec`도 촬영 간격보다 충분히 크게 조절한다.

소스 YAML은 빌드 시 install로 복사된다. 반복 튜닝은 별도 파일을 만들어 기존
실행 명령에 아래 인자를 추가하면 **YAML 수정마다 재빌드할 필요 없이** 재실행하면 된다:

```bash
cp src/traffic_detection_test/config/traffic_light.yaml traffic_light_test.yaml
# 기존 ros2 launch 명령 끝에 추가
# traffic_light_params_file:=/home/autopilot03/Desktop/0906ML/f1tenth_project_repo/traffic_light_test.yaml
```

CLI `traffic_light_inference_fps:=...`를 함께 지정하면 YAML의 `inference_fps`보다
우선한다. YAML만으로 조정할 때는 해당 CLI 인자를 생략한다.
연속 확인은 BEV 텍스트와 상태 메시지에 적용한다. auto_control에서
`traffic_stop_enabled`를 켰을 때만 이 상태를 실제 정지선 제동에 사용한다.

설정은 `config/traffic_light.yaml`에 있다. ROI는 카메라 원본 크기의 비율로
지정한다. 기본값은 640x400 기준 `x=0,y=65,w=640,h=160`에 해당하며
1280x800에서는 `x=0,y=130,w=1280,h=320`을 모델의 640x160으로 축소한다.
CAM_B/C의 흑백 입력으로는 색상 판별을 할 수 없어 UNKNOWN 상태가 유지된다.

기존 자동 주행 명령에 다음 인자를 사용할 수 있다:

```bash
# 신호등 연산을 완전히 끄고 차선 프로파일과 비교
traffic_light_enabled:=false
# 처리량 제한 (생략하면 YAML의 20Hz)
traffic_light_inference_fps:=15.0
# 외부 신호등 YAML (노드 키: traffic_light_detector)
traffic_light_params_file:=/absolute/path/traffic_light.yaml
```

터미널 `Traffic` 로그에는 실제 프레임 처리율, 추론/전체 처리 평균·최댓값,
촬영 이후 최대 경과시간, 최신 프레임 교체 횟수가 나온다. 카메라 80Hz/추론
20Hz에서는 프레임 교체가 정상이다. 차선의 `profiling_enabled:=true` CSV를
신호등 on/off 조건에서 비교해 차선 지연 영향을 판단할 수 있다.

변경된 카메라와 공유 라이브러리까지 함께 빌드해야 한다:

```bash
colcon build --packages-up-to vehicle_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

첫 실행의 엔진 빌드와 준비 직후 추론은 정상 운전 중 성능 측정에서 제외한다.
엔진 초기화도 별도 스레드지만 최초 빌드의 CPU/GPU 부하는 공유한다.
기존 단독 프리뷰는 아래와 같이 별도로 사용할 수 있으며, 같은 카메라를 사용하는
자동 주행과 단독 프리뷰를 동시에 실행하면 안 된다.

`camera_driver`를 수정하지 않고 OAK 카메라의 신호등 검출 결과만 확인하는
ROS 2 C++ 프리뷰 패키지다. OAK CAM_A의 `1280x800` 센서 영상을 `640x400`
NV12로 받아 TensorRT 입력용 전처리를 GPU에서 수행하고, 학습 완료된
YOLOX-S explicit Q/DQ INT8 ONNX 모델로 추론한다.

## 처리 방식

- 카메라 입력: `640x400`, 기본 80 FPS, 장치 왜곡 보정 사용
- 추론 ROI: 원본 픽셀 기준 중심 좌표와 폭·높이로 지정
- 기본 모델 입력: BGR FP32 `[1, 3, 160, 640]`
- TensorRT 전처리: raw NV12를 GPU로 전송한 뒤 CUDA 커널 하나로
  ROI crop·resize, `NV12 -> BGR FP32 NCHW`와 114 패딩을 수행
- CPU 전처리: 호스트 BGR에서 동일한 패딩과 FP32 NCHW 생성
- 기본 모델 출력: decoded `[1, 2100, 6]`
- confidence: `objectness * class probability`
- 기본 threshold: score `0.25`, NMS IoU `0.65`
- 기본 실행 백엔드: TensorRT 직접 실행, INT8 Q/DQ
- 신호 상태: confidence가 가장 높은 검출 박스의 원본 BGR 픽셀을 HSV로
  변환하여 룰베이스로 `Red`, `Green`, `Unknown` 판별
- 출력: OpenCV 프리뷰 창만 사용하며 ROS 이미지나 검출 토픽은 발행하지 않음

캡처 스레드는 큐를 쌓지 않고 가장 최신 프레임만 보관한다. 추론 속도가
카메라 속도보다 낮으면 오래된 프레임을 건너뛰므로 프리뷰 지연이 계속
누적되지 않는다. 프리뷰에는 bounding box, confidence, 검출 개수, 추론
시간과 `Red`/`Green` 상태를 표시한다. `Q` 또는 `ESC`로 종료한다.

TensorRT가 사용되면 프리뷰 표시를 위한 `getCvFrame()` BGR 변환은 추론이
완료된 뒤 별도로 수행된다. 이 BGR 프레임은 화면 표시에만 쓰이며
TensorRT 입력으로 다시 복사되지 않는다.
프리뷰 캔버스는 항상 원본 `640x400`을 유지하며, ROI 밖은 검정색으로
마스크하고 ROI 경계를 노란색으로 표시한다. 검출 상태 글자는 공간이
있으면 ROI 바로 아래의 검정 영역에 표시하므로 추론 영역을 가리지 않는다.

## 추론 ROI와 모델 크기

ROI는 `roi_center_x`, `roi_center_y`, `roi_width`, `roi_height`로 지정하며
좌표와 크기는 모두 `640x400` 원본 영상 픽셀 기준이다. ROI는
반드시 원본 영상 안에 완전히 들어와야 한다.

기본 번들 모델은 기존 `640x640` 체크포인트를 재학습 없이 고정
`640x160` 입력으로 다시 export한 모델이다. 기본 ROI는 중심 `(320,145)`,
크기 `640x160`, 즉 원본의 `y=65..224`이다. 목표 구간 `y=100..189`
주변에 위아래 약 35픽셀 문맥을 남기면서 TensorRT 입력 면적을 기존의
25%로 줄인다.

기본 설정은 별도 모델·ROI 인자 없이 실행할 수 있다.

```bash
ros2 launch traffic_detection_test traffic_detection_test.launch.py
```

다른 고정 입력 ONNX를 사용하려면 `model_input_width`와
`model_input_height`를 실제 ONNX 입력과 같게 설정해야 하며 둘 다 32의
배수여야 한다.

ROI 비율과 모델 입력 비율이 다르면 원본 비율을 유지하여 좌측 상단에
resize하고 나머지를 114로 패딩한다. 검출 박스는 ROI offset을 다시
더해 원본 `640x400` 프리뷰 좌표로 복원한다.

상태 로그는 다음 구간의 평균과 최댓값을 각각 분리한다.

- 카메라 노출 중간 시점부터 Jetson 수신까지 `sensor->host`
- 화면 표시용 DepthAI 프레임의 BGR 변환 `preview-convert`
- TensorRT의 GPU NV12 변환·패딩·FP32 NCHW 생성, 또는 CPU의 호스트
  전처리 `preprocess`
- TensorRT는 host raw NV12, 기존 호스트 경로는 FP32 tensor의 GPU 입력
  전송 `H2D`
- TensorRT의 순수 network 실행 `execute`
- decoded FP32 출력의 host 전송 `D2H`
- score, 좌표 복원과 NMS 후처리
- bounding box와 상태 overlay 그리기
- host 수신 및 sensor 시점부터 `imshow()` 호출까지의 전체 지연

`sensor->display`는 `imshow()`에 프레임을 전달한 시점까지이며 모니터의 실제
화면 주사 완료 시각은 포함하지 않는다.

## 신호 색상 판별

YOLOX 모델은 신호등의 위치만 검출하고, 상태는 검출 박스 안의 원본 BGR
영상을 HSV로 변환하여 판별한다. `color_min_saturation`과
`color_min_value`보다 낮은 무채색·어두운 픽셀을 제외한 뒤 빨강 hue
`0..20`, `165..179`와 초록 hue `35..95`의 `채도 x 밝기` 합을 비교한다.
빨강 합이 크면 `Red`, 초록 합이 크면 `Green`, 유효 픽셀이 없거나 합이
같으면 `Unknown`으로 하단 상태 글자에 표시한다. 기본 임계값은 각각
`80`, `60`이며 OpenCV HSV 범위 `0..255` 기준이다.

현장 조명에서 어두운 신호가 `Unknown`으로 자주 나오면 다음처럼 임계값을
낮춰 확인할 수 있다.

```bash
ros2 launch traffic_detection_test traffic_detection_test.launch.py \
  color_min_saturation:=60 color_min_value:=40
```

## 모델 범위

번들 모델은 학습할 때 `Red`와 `Green`을 단일 `traffic_light` 클래스로
합쳤다. 따라서 모델 출력 자체에는 색상 클래스가 없고, 위 HSV 룰이 검출
박스의 색상을 별도로 구분한다. 기본 TensorRT 경로는 74개 Conv를 INT8로
실행하고 최종 box/objectness/class 출력 Conv 9개를 FP32로 유지하는 Q/DQ
혼합 모델을 사용한다. 모델 입출력은 기존과 동일한 FP32이다.

기본 모델은 다음 설치 경로에서 자동으로 불러온다.

```text
share/traffic_detection_test/models/traffic_light_yolox_s_640x160_batch_1.int8.qdq.onnx
```

`model_path` launch 인자로 다른 decoded YOLOX ONNX 파일을 지정할 수 있지만,
입출력 형식은 FP32 `[1,3,H,W]`과 `[1,N,6]`이어야 한다.

TensorRT는 첫 실행에서 ONNX를 현재 Jetson용 INT8 엔진으로 빌드한다. 이 작업은
몇 분 걸릴 수 있으며, 다음 실행부터는 캐시된 엔진을 역직렬화해 바로 사용한다.
기본 캐시 파일은 ONNX 옆에 TensorRT major 버전을 포함한 다음 형식으로 생성된다.

```text
traffic_light_yolox_s_640x160_batch_1.int8.qdq.onnx.trt<major>.int8.engine
```

FP32로 되돌려 비교하려면 원본 ONNX와 precision을 함께 지정한다.

```bash
ros2 launch traffic_detection_test traffic_detection_test.launch.py \
  model_path:="$(ros2 pkg prefix traffic_detection_test)/share/traffic_detection_test/models/traffic_light_yolox_s_640x160_batch_1.onnx" \
  engine_precision:=fp32
```

ONNX 파일이 캐시보다 새롭거나 캐시가 현재 TensorRT/GPU와 호환되지 않으면 자동으로
다시 빌드한다. `engine_cache_path`로 별도 위치를 지정할 수 있다. TensorRT 엔진은
Jetson GPU와 TensorRT 버전에 종속되므로 다른 장비에서 만든 파일을 복사해 쓰지 않는다.

## 빌드

Ubuntu/Jetson의 ROS 2 Humble, DepthAI C++ 3.x, OpenCV 4의 `dnn`,
`highgui` 모듈, CUDA Toolkit과 TensorRT 개발 패키지가 필요하다. OpenCV 자체는
CUDA 빌드일 필요가 없다. JetPack에 개발 패키지가 빠져 있다면 다음 라이브러리를
설치해야 한다.

```bash
sudo apt install libnvinfer-dev libnvinfer-plugin-dev libnvonnxparsers-dev
```

```bash
cd ~/Desktop/0906ML/f1tenth_project_repo
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select traffic_detection_test \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/Desktop/0906ML/f1tenth_project_repo/install/setup.bash
```

DepthAI C++가 기본 prefix에 없다면 `camera_driver`와 같은 방식으로
`-Ddepthai_DIR=<prefix>/lib/cmake/depthai`를 전달한다.

이 프리뷰만 확인할 때는 위 명령처럼 `traffic_detection_test`만 선택한다.
`vehicle_bringup`을 `--packages-select`에 함께 넣으면 `auto_control`,
`joy_initializer`, `vehicle_dynamics_monitor` 등 아직 설치되지 않은 실행
의존성이 자동으로 추가되지 않아 전체 빌드가 중단될 수 있다. 전체 주행
패키지까지 새로 빌드해야 한다면 의존성을 포함하는 다음 명령을 사용한다.

```bash
colcon build \
  --packages-up-to traffic_detection_test vehicle_bringup \
  --cmake-clean-cache \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc
source ~/Desktop/0906ML/f1tenth_project_repo/install/setup.bash
```

`traffic_detection_test`는 NV12 전처리 CUDA 커널을 `.cu`로 직접 컴파일하므로
`nvcc`가 필요하다. 자동 탐색이 되지 않는 환경에서는 위 명령처럼
`-DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc`를 지정한다.

## 실행

```bash
ros2 launch traffic_detection_test traffic_detection_test.launch.py
```

`ros2 launch`가 `/opt/ros/humble`에서만 패키지를 검색하며
`Package 'traffic_detection_test' not found`를 출력하면, 현재 터미널에서
아직 `install/setup.bash`를 source하지 않았거나 앞선 빌드가 설치 단계 전에
중단된 상태다. 위의 단독 빌드와 source 명령을 다시 실행한다.

threshold 변경 예시는 다음과 같다.

```bash
ros2 launch traffic_detection_test traffic_detection_test.launch.py \
  score_threshold:=0.30 nms_threshold:=0.65
```

기존 OpenCV CPU 기준과 비교할 때만 다음처럼 명시한다. `CPU`는 TensorRT 엔진
캐시를 생성하거나 사용하지 않는다.

```bash
ros2 launch traffic_detection_test traffic_detection_test.launch.py \
  inference_backend:=CPU
```

이 패키지는 OAK 장치를 직접 연다. 같은 OAK 장치를 사용하는
`camera_driver` 또는 다른 카메라 패키지와 동시에 실행할 수 없다. GUI가
필수이므로 SSH에서 실행할 때는 X11 forwarding이나 로컬 디스플레이가
설정되어 있어야 한다.
