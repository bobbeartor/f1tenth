# multiclass_yolo

OAK CAM_A와 스테레오 깊이를 함께 받아 YOLOX 4개 클래스의 장애물 점을 카메라 BEV에서 확인하는 독립 ROS 2 프리뷰입니다.
기존 주행/신호등 패키지는 수정하지 않습니다. 이 노드는 OAK 장치를 직접 열므로 같은 장치를 사용하는
`camera_driver`, `bev_processor`, `traffic_detection_test` 등은 종료한 후 실행하세요.

## 모델 계약

기본 모델은 `multiclass_yolox_tiny_512x320_holdout10.onnx`입니다. 하단 80% 전용 `roi80_448x224.onnx`도 선택할 수 있습니다.
동봉된 `.metadata.json`과 `.labels.txt`에 입력·출력 형식과 클래스 순서가 기록되어 있습니다.

- 입력: 기본 모델 FP32 `[1,3,320,512]`, 448×224 모델 FP32 `[1,3,224,448]`. **BGR**, 0–255, 정규화 없음
- 리사이즈: 종횡비 유지, 좌상단 배치, 오른쪽/아래를 114로 채움
- 출력: 기본 모델 FP32 `[1,3360,9]`, 448×224 모델 FP32 `[1,2058,9]`
- 행: `[cx,cy,w,h,objectness,con,truck,sedan,traffic_light]`
- 좌표는 모델 입력 픽셀 기준으로 이미 디코딩되어 있음. sigmoid/grid decode를 다시 적용하지 않음
- 가장 높은 클래스 확률을 선택하고 `objectness × class_probability`에 클래스별 임계값 적용
- 클래스별 NMS (기본 IoU 0.65), 원본 영상 좌표로 복원

사용자가 처음 설명한 RGB 학습 여부보다 전달된 export 메타데이터의 BGR 계약을 따릅니다.
학습/export 코드가 이 메타데이터와 다르다면 채널 순서를 다시 확인해야 합니다.
`con`은 제공된 라벨 철자를 그대로 유지합니다.

## 빌드

기존 워크스페이스 루트(`src`, `cmake`가 있는 폴더)에서 Jetson 데스크톱 터미널로 실행합니다.
기존 프로젝트와 같은 ROS 2 Humble 및 `/usr/local` DepthAI 3.6 공유 SDK를 사용합니다.
TensorRT 개발 라이브러리(`libnvinfer-dev`, `libnvonnxparsers-dev`, `libnvinfer-plugin-dev`),
CUDA toolkit, OpenCV core/dnn/highgui/imgproc가 필요합니다. 기존 `traffic_detection_test`를 빌드할 수 있는 환경을 기준으로 합니다.
SDK를 새로 빌드하거나 교체하지 않습니다. 설치 경로가 다르면 기존 `DEPTHAI_SETUP.md`를 참고하세요.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to multiclass_yolo --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 실행 및 입력 경로 비교

```bash
# 기본 BEV 프리뷰: NV12 → CUDA 전처리 → TensorRT FP16
ros2 launch multiclass_yolo preview.launch.py input_format:=nv12

# 같은 NV12 카메라 영상 → CPU RGB 영상 → BGR 텐서 → TensorRT FP16
ros2 launch multiclass_yolo preview.launch.py input_format:=rgb

# 초기 클래스별 임계값, NMS 조절
ros2 launch multiclass_yolo preview.launch.py \
  input_format:=nv12 con_threshold:=0.30 truck_threshold:=0.40 \
  sedan_threshold:=0.35 traffic_light_threshold:=0.20 nms_threshold:=0.65

# CPU 진단 경로 (30 FPS 성능용 기본 설정은 아님)
ros2 launch multiclass_yolo preview.launch.py \
  input_format:=rgb inference_backend:=CPU engine_precision:=fp32
```

`input_format`은 **전처리 시작 형식**을 고릅니다. 카메라 USB 전송 형식은 두 모드 모두 NV12로 고정하여
같은 전송 조건에서 비교합니다. `rgb`에서는 OpenCV NV12→RGB 변환을 하고, 모델 blob을 만들 때
BGR 채널 순서로 바꿉니다. `nv12`에서는 NV12를 GPU로 복사한 뒤 색상 변환·리사이즈·패딩·NCHW 작성을
CUDA 커널에서 처리하여 추론용 전체 RGB 중간 영상을 만들지 않습니다.
**NV12 자체를 학습된 ONNX에 직접 넣는 방식은 아닙니다.**

프리뷰 표시는 두 모드 모두 CPU BGR 변환이 필요합니다. 따라서 NV12 모드도 CPU 변환 비용이 0이 아니며,
측정된 출력 FPS는 프리뷰 비용까지 포함합니다. zero-copy 파이프라인은 아니고 NV12 H2D 복사는 남습니다.
CUDA 색 변환은 기존 프로젝트와 동일한 **BT.601 limited range**를 가정하고 plane pitch를 반영합니다.
실제 카메라 스트림이 다른 색 행렬/range를 쓴다면 이 계수를 맞춘 뒤 검출 정확도를 비교해야 합니다.

첫 실행은 ONNX로 TensorRT 엔진을 빌드하므로 시간이 걸립니다. 엔진은 기본적으로
`~/.cache/multiclass_yolo/` 아래 모델 경로/크기/수정시각과 정밀도로 구분하여 저장합니다.
다른 ONNX는 `model_path:=/absolute/model.onnx`, 캐시 위치는 `engine_cache_path:=/absolute/model.engine`으로
지정할 수 있습니다. 다른 GPU/JetPack으로 옮길 때 기존 엔진 대신 ONNX에서 다시 생성하세요.

## 프리뷰

- 기본 화면은 차선 없는 카메라 BEV입니다. 장애물 점과 왼쪽 목록의 색상 표시는 클래스별로 일치합니다
  (`con` 빨강, `truck` 파랑, `sedan` 초록, `traffic_light` 자홍).
- `depth_fusion_enabled:=false`로 시작하면 기존 원본 영상과 검출 박스를 표시합니다.
- 왼쪽 검출 목록에 현재 프레임의 클래스, 검출 점수, 적용된 클래스 임계값 표시. 목록이 길면 `Prev` / `Next`로 이동
- `Actual FPS`: 최근 약 2초간 호스트가 받은 영상, 분류 결과를 완료한 프레임,
  유효 장애물 위치를 하나 이상 산출한 프레임을 각각 경과 시간으로 나눕니다.
- `Receive-to-result`: 호스트 영상 수신부터 각 결과까지의 평균 지연을 FPS로 환산합니다
  (1000 / 평균 ms). 분류는 디코딩·NMS 완료, 장애물 위치는 깊이 정합·필터 완료가 기준이며
  결과를 그리는 시간은 제외합니다. 위치를 얻은 프레임이 없으면 해당 지연 항목은 `--`입니다.
- `Stage FPS`와 `TensorRT compute FPS`: 각 단계의 평균 소요 시간으로 환산한 FPS입니다.
  깊이 정합·필터, BEV 렌더링, 입력 및 표시용 색 변환을 각각 표시합니다.
- `skipped`: 호스트 수신 후 최신 프레임 우선 처리 때문에 검출하지 않은 프레임 수 (장치/USB 드롭 수와 다름)
- `Detection thresholds`: `con`, `truck`, `sedan`, `traffic_light`별 검출 임계값을 0–1로 입력
- `Bottom ROI %`: 화면 하단부터 검출할 높이를 정수 퍼센트로 입력 (1–100, 기본 65). 노란 선이 검출 영역의 상단을 표시
- `Camera FPS`: 카메라 프레임 속도를 숫자로 입력. F 키로 이 입력 칸에 바로 이동
- 입력 칸을 클릭한 뒤 숫자를 입력하고 Enter 또는 `Apply`로 적용
- Q / ESC / 창 닫기로 종료. 숫자 입력 중에는 ESC로 입력을 취소

GUI에서 입력한 값은 현재 실행에만 적용됩니다. 재실행 때 유지하려면 실행 인수 또는 YAML을 수정하세요.
YAML로 실행하려면 아래처럼 `ros2 run`에 전달합니다 (기본 launch는 launch 인수를 사용합니다).

```bash
ros2 run multiclass_yolo multiclass_yolo_node --ros-args \
  --params-file src/multiclass_yolo/config/preview.yaml
```

카메라 기본값은 CAM_A, 1280×800, 40 FPS, 기존 프로젝트처럼 undistort 활성화입니다.
1280×800 프레임에서 ROI 65%는 하단 520행입니다. ROI와 클래스별 임계값은 다음 추론부터 적용됩니다.
시작 ROI는 launch 인수 `roi_height_percent:=65` 또는 YAML의 `roi_height_percent`로 지정합니다.
100%에서는 전체 프레임을 검출합니다.

정합에는 원본 영상 좌표로 복원된 검출 박스를 사용합니다. NV12/CUDA 경로는 ROI에 해당하는
픽셀을 색차 샘플 경계에 맞춰 GPU로 전송합니다. 카메라 USB 전송과 프리뷰용 색 변환에는 전체 프레임을 사용하며,
선택한 TensorRT 모델의 입력 크기와 추론 연산량은 고정됩니다. 로그의 `upload`에서 GPU 입력 전송 시간을 비교할 수 있습니다.

FPS 입력 범위는 연결된 카메라의 1280×800 센서 모드를 따릅니다. 적용할 때 카메라 스트림을
다시 여는 동안 영상이 잠시 멈춥니다. TensorRT 엔진은 유지되고, 새 프레임부터 평균 FPS를 다시 계산합니다.
변경에 실패하면 기존 FPS로 스트림 재개를 시도합니다. 다음 실행의 초기값은 `sensor_fps`로 지정합니다.
`width`, `height`, `sensor_fps`, `undistort_enabled`를 launch 인수로 바꿀 수 있습니다.
예: `width:=640 height:=400`으로 USB 전송과 프리뷰 변환량을 줄일 수 있습니다. 모델 텐서 크기는 선택한 모델에 따릅니다.

수신 스레드와 처리/GUI 루프를 분리하고 장치/호스트 큐를 모두 크기 1, non-blocking으로 설정합니다.
처리가 느리면 오래된 프레임을 쌓지 않고 최신 프레임을 사용합니다. 실제 입력·출력 FPS는 차량 PC에서 측정해야 합니다. 약 2초마다 프리뷰의 FPS 지표가 로그에 나옵니다.
입력 FPS는 센서 내부 FPS가 아닌 유효 프레임의 호스트 도착률입니다.

## BEV 정합과 숫자 조절

기본 실행은 CAM_A 40 FPS, 네 클래스 검출 임계값 0.70입니다. VFH 참조 설정으로 400p
스테레오를 함께 열고, 시작 시 평평한 바닥에서 정지한 차량의 카메라 높이와 자세를 측정합니다.
정합은 같은 OAK의 RGB·깊이 촬영 시각을 맞춘 뒤, 원본 영상의 검출 박스와 VFH 깊이 ROI를
모두 통과한 점에 기존 지면·높이·고립점 필터를 적용합니다. 기존 VFH 기본값처럼 지면 위
1.0 m가 최대 높이이며, 그보다 높은 신호등 점을 보려면 `Max height m`을 조절할 수 있습니다.

BEV는 앞차축 기준 전방 4 m, 좌우 ±0.75 m에 카메라 배경과 네 클래스의 실제 장애물 점을
클래스별 색으로 표시합니다. 상단 `BEV image` / `Points only` 버튼으로 카메라 배경과 점 전용 격자를 전환합니다. 점 전용 보기에서는 표시용 색 변환과 배경 워핑을 건너뜁니다. 위치를 못 구한 검출에 표시되는 점 수는 BEV 화면 범위로 자르기 전의 박스 매칭 점과 필터 통과 점 수입니다.
동기화된 깊이나 지면 보정값이 없으면 점 수를 `--`로 표시합니다.

### 검출별 위치와 위에서 본 크기

필터를 통과한 점을 검출별로 모아 위치 하나와 클래스 크기 사각형을 추정합니다.

1. 박스 가운데 60%(`object.box_core_fraction`) 안의 점을 우선 씁니다. 가장자리에는 배경이 섞입니다.
   가운데 점이 6개 미만이면 박스 전체 점을 씁니다.
2. 카메라 기준 거리순으로 정렬해, 거리 창(0.05 m + 거리의 3%) 안에 점이 충분한(6개 이상이고
   전체의 10% 이상) 가장 가까운 구간을 물체 앞면으로 고릅니다. 앞쪽에 흩어진 점 몇 개나 뒤쪽
   배경은 선택되지 않습니다.
3. 거리는 그 구간의 중앙값, 방향은 박스 중심 픽셀의 시선 방향입니다.
4. 클래스 크기 사각형을 앞면에서 시선 방향으로 놓습니다. 보이는 면을 폭으로 가정하므로
   옆으로 지나가는 차량은 길이가 맞지 않습니다.

| 클래스 | 길이 × 폭 | 파라미터 |
|---|---|---|
| sedan | 0.50 × 0.30 m (내 차량과 동일) | `object.vehicle_length_m`, `object.vehicle_width_m` |
| truck | 0.60 × 0.36 m (내 차량의 1.2배) | `object.truck_scale` |
| con | 0.13 × 0.13 m | `object.cone_size_m` |
| traffic_light | 크기 없음, 위치만 | |

크기와 선택 기준은 `Setup` 목록의 `Vehicle length m`부터 `Near percentile`까지 항목으로 실행 중에
바꿀 수 있고 다음 프레임부터 적용됩니다. `Range window m`과 `Window per m`은 거리 창, `Min object points`와
`Min point share`는 앞면으로 인정할 최소 점 수와 비율, `Near percentile`은 최근접 거리로 쓸 하위 백분위입니다.

BEV에서 선택된 점은 클래스 색, 같은 박스 안의 배경 점은 회색입니다. 흰 고리는 측정된 앞면,
사각형은 추정 크기입니다. 목록의 `X,Y m | pts`는 사각형 중심의 차량 좌표(앞 +X, 왼쪽 +Y)이고,
위치를 못 구한 검출은 `박스 매칭 점/필터 통과 점 pts`를 표시합니다. 결과는 프리뷰 표시뿐이며
토픽으로 발행하지 않습니다.

`Setup`에서 `Prev` / `Next`로 설정 페이지를 이동합니다. 숫자를 입력하고 `Enter` 또는
`Apply`를 누릅니다.

- 스테레오 FPS·해상도·프리셋·신뢰도·LR 검사·서브픽셀·IR·필터는 스트림을 잠시 다시 열어 적용합니다.
  프리셋 0/1/2는 기본/밀도 우선/정확도 우선이며, 0/1 토글은 끔/켬입니다.
- 깊이 범위, 지면 제거, 높이, 고립점, 동기 허용 차이는 다음 처리 프레임부터 적용합니다.
- `Fit` 항목은 다음 지면 추정에 사용합니다. 차량을 평평한 바닥에 멈춘 뒤 `Refit ground`를 누릅니다.

`frames.csv`의 `receive_to_position_ms`에는 수신부터 유효 장애물 위치 산출까지의 시간이 기록됩니다.
`depth_association_ms`와 `bev_render_ms`에는 정합·필터와 BEV 렌더링을 각각 기록합니다.
`depth_fusion_ms`에는 정합과 BEV 렌더링의 합이, `rgb_depth_skew_ms`에는
짝지어진 프레임의 촬영 시각 차이가 기록됩니다. 순수 모델 속도는 `depth_fusion_enabled:=false`와
`Video off`로 비교할 수 있습니다.

## 모델 전환 및 측정

`448x224 FP16` 버튼은 하단 80% ROI 전용 모델을 선택합니다. 1280×800 카메라 프레임에서
상단 160행을 제외한 영역을 448×224로 변환합니다. 이 모델은 검출 결과를 점수순 최대 100개로
표시하며, 측정 로그에 모델 입력 크기와 실제 ROI를 기록합니다. `FP16` 또는 `INT8`으로
돌아가면 512×320 모델에서 사용하던 ROI 설정이 적용됩니다.

시작할 때부터 448×224 모델을 쓰려면 아래처럼 실행합니다.

```bash
ros2 launch multiclass_yolo preview.launch.py model_variant:=roi80_448x224 engine_precision:=fp16
```

448×224 ONNX는 FP32 형식이며 Jetson에서 FP16 엔진으로 실행합니다. `INT8` 버튼은 동봉된
512×320 Q/DQ 모델을 선택합니다.

- `FP16`은 원본 holdout10 ONNX, `INT8`은 동봉된 Entropy Q/DQ ONNX를 사용합니다.
  두 모델은 입력·출력과 클래스 순서가 같고, INT8의 외부 입출력도 FP32입니다.
- 첫 선택 시 현재 Jetson에서 엔진을 생성하고 워밍업합니다. 전환 중 기존 엔진으로 추론을 계속하며,
  새 엔진이 준비되면 전환합니다. 이후 정밀도별로 엔진 캐시를 사용합니다.
  INT8 파일을 바꾸려면 `int8_model_path:=/absolute/model.int8.qdq.onnx`를 지정합니다.
- `Duration (s)`에서 1–120초를 입력하고 `Measure`를 누릅니다. 기본값은 15초입니다.
  `Stop`으로 일찍 끝낼 수 있습니다. 설정을 바꾸려면 현재 측정을 종료한 뒤 새 측정을 시작합니다.
- `Video off`에서는 추론을 계속하면서 표시용 색 변환, 박스·목록 갱신과 영상 확대를 건너뜁니다.
  같은 조건에서 `Video on`과 별도로 측정하면 프리뷰 비용을 비교할 수 있습니다.

측정 파일은 기본적으로 `~/.cache/multiclass_yolo/measurements/` 아래 실행별 폴더에 저장됩니다.
`measurement_directory`와 `measurement_seconds`로 경로와 초기 시간을 지정할 수 있습니다.
`frames.csv`는 프레임별 시간이며 해당하지 않는 단계는 빈 칸입니다. `summary.json`은 단계별 p50/p95,
수신·검출 완료·프리뷰 제출 FPS, 시퀀스 누락, 처리 루프가 건너뛴 프레임과 측정 설정을 담습니다.
`receive_to_submit_ms`는 호스트 수신부터 HighGUI 제출 완료까지이며 모니터 표시 시각과는 별개입니다.
시간 합계를 낼 때 `processing_ms`와 `receive_to_*`는 다른 구간을 포함합니다.

INT8 모델은 보정 스케일을 ONNX 안에 포함하고 있습니다. Jetson에서 같은 전력 모드·ROI·FPS로
`inference_ms`와 전체 검출 FPS를 함께 비교하세요. 레이어 정밀도를 확인할 때는 설치된 TensorRT의
`trtexec`로 Q/DQ 모델을 상세 프로파일링 옵션과 함께 별도로 빌드해 레이어 정보를 확인합니다.
동봉 모델의 기존 검출 평가는 전체 프레임 기준이므로 ROI 100%와 운용 ROI 65%에서 각각 확인합니다.

## 검증

```bash
colcon test --packages-select multiclass_yolo --event-handlers console_direct+
colcon test-result --verbose
```

후처리 테스트는 클래스별 NMS/임계값, 점수 계산, 잘못된 값, 원본 좌표 복원을 검증합니다.
모델 smoke test는 실제 ONNX CPU 실행, 유한한 `[1,3360,9]`, RGB/BGR 경로의 박스·점수 일치와 ROI 박스의 원본 좌표 복원을 검사합니다.
0 임계값으로 낮은 점수 후보도 남겨 경로 일치 검사가 빈 검출 결과만 비교하지 않도록 했습니다.

작성 시 Mac OpenCV 4.13에서 위 C++ 후처리/ONNX 테스트를 직접 컴파일하여 통과했습니다.
ROS launch Python 문법과 package.xml 파싱도 확인했습니다.
ROS 노드 전체 빌드, OAK 실시간 영상, TensorRT/CUDA 실행 및 실제 30 FPS는 이 Mac에 해당 환경이 없어 검증하지 못했습니다.

TensorRT 없이 CPU만 빌드할 때는 `-DMULTICLASS_YOLO_TENSORRT=OFF`를 사용하고,
실행 시 `input_format:=rgb inference_backend:=CPU engine_precision:=fp32`를 지정합니다.
카메라용 DepthAI SDK는 여전히 필요합니다.


### 창이 바로 닫히는 현상

일부 GTK OpenCV는 `WND_PROP_VISIBLE` 조회에 -1(미지원)을 반환한다.
이를 창 닫힘으로 해석하지 않도록, 처음 실제로 지원된 속성의 변화를 보고 종료한다.
GTK에서는 `WND_PROP_AUTOSIZE`도 창 존재 확인에 사용하며 WINDOW_NORMAL의 0은 정상이다.
첫 영상 전에는 대기 화면을 표시하고, 최초 수신·최초 검출 표시·종료 사유를 로그에 남긴다.
엔진 캐시는 유지되므로 GUI 수정만으로 엔진을 다시 만들 필요는 없다.

## 객체별 거리 데이터와 Distance MLP

기존 `multiclass_yolo_node`/BEV 경로와 독립적인 `yolo_distance_node`를 추가했습니다.
YOLOX 검출기, TensorRT backend, `point_cloud::addStereoDepth` 및 프레임 캘리브레이션
변환을 재사용합니다. 기존 주행 launch나 자동제어 입력은 변경하지 않습니다.

```text
collect:  CAM_A BGR → 기존 YOLOX → 원본 BBox ───────────────┐
          CAM_B/C → StereoDepth(mm) → timestamp pairing  │
          frame intrinsics + EEPROM extrinsics → RGB z-buffer(m)
                                  → 중앙 ROI/median/품질 검사
                                  → PNG + samples.jsonl
train:    samples.jsonl → sequence 분리 → 9→64→32→1 PyTorch MLP
                                  → .pt + .json (+ 선택 .onnx)
infer:    CAM_A BGR → YOLOX → 정규화 BBox + one-hot → C++ MLP
                                  → 객체 JSON ROS2 토픽 + 프리뷰
evaluate: collect와 같은 stereo 경로 + MLP → paired metrics JSONL
```

거리는 **RGB optical frame의 Z축 깊이(m)**입니다. 유클리드 직선거리나 차량 전방
좌표가 아닙니다. 바닥 평면/포인트클라우드/BEV 계산은 이 경로에서 사용하지 않습니다.
스테레오는 rectified-right 기준 RAW16을 생성하고, 기존 `rgbFromFrame`으로 프레임의
rectification/crop 변환과 EEPROM extrinsics를 적용하여 RGB에 재투영합니다.
중복 픽셀은 가장 가까운 Z로 선택하고 정합 hole을 보간하지 않습니다.
수집은 RGB와 깊이의 exposure-middle host-steady timestamp 차이가 `sync_ms` 이내일 때만
라벨을 생성합니다. 최대 150 ms 대기 후 일치하는 깊이가 없으면 해당 RGB의 라벨을 생략합니다.
오래된 RGB(500 ms 초과)는 버립니다. 장치별 시간/캘리브레이션 매칭은 실차 확인이 필요합니다.

### 환경과 빌드

저장소의 공통 빌드는 `/usr/local` **DepthAI C++ 3.6 shared SDK**를 요구합니다.
이 기능은 Python DepthAI를 설치하거나 SDK를 교체하지 않습니다.
실행 시 연결된 sensor/socket을 로그에 기록하고 CAM_A, 수집/평가에서는 CAM_B/C와
Pro IR dot projector 지원 여부를 검사합니다. 이 개발 환경에는 ROS2, DepthAI SDK,
CMake와 실제 OAK가 없으므로 C++ 노드의 빌드·USB·실측 정합·FPS는 아직 검증하지 못했습니다.
실제 OAK-D Pro W의 센서명은 실행 로그를 확인하세요. 다른 OAK 프로필에 대한 자동 추측은 하지 않습니다.

위의 기존 빌드 명령을 그대로 사용합니다. 추가 의존성은 ROS `std_msgs`, OpenCV imgcodecs,
`nlohmann_json`입니다. 학습 머신에는 NumPy와 PyTorch가 필요합니다. Jetson에서는 JetPack에
맞는 PyTorch wheel을 사용하세요. 실시간 노드는 PyTorch/Python/ONNX Runtime을 요구하지 않습니다.
MLP는 2,753 파라미터이며 JSON 가중치를 시작 시 배열로 읽어 CPU에서 계산합니다.
현재 새 노드는 BGR host 경로를 사용하므로 기존 NV12 CUDA 프리뷰와 처리 비용이 다릅니다.

### 1. 수집

같은 OAK를 여는 다른 카메라/BEV 노드를 종료하고 실행합니다.

```bash
ros2 launch multiclass_yolo distance.launch.py \
  mode:=collect sequence:=run_001 output_directory:=/tmp/distance_dataset
# 다른 위치/주행 시퀀스를 독립적으로 촬영
ros2 launch multiclass_yolo distance.launch.py \
  mode:=collect sequence:=run_002 output_directory:=/tmp/distance_dataset
```

각 시퀀스 디렉터리를 새로 만들며 기존 시퀀스 덮어쓰기는 거부합니다.
`images/<frame sequence>.png`는 박스를 그리기 전 원본 BGR 영상이고,
`samples.jsonl`은 객체 한 개당 다음 정보를 기록합니다.

- `sequence`, `image`, `width`, `height`, `bbox` (원본 픽셀 x,y,w,h)
- `class_id`, `class`, `confidence`, `distance_m`, `stereo_distance_m`
- `timestamp`, `depth_timestamp` (host steady seconds; Unix/ROS epoch 아님)
- `camera_contract`: 장치 ID, 해상도, 출력 intrinsic, undistort/crop 설정

중앙 60% ROI의 0.2–10 m 유효 픽셀이 최소 20개, 비율 10% 이상이어야 합니다.
P90–P10 폭/median이 15%를 넘는 혼합 깊이를 거부하고,
`max(0.02m, 3×1.4826×MAD)` 밖의 이상치를 제거한 뒤 median을 사용합니다.
시간에 따른 객체 추적/안정화는 하지 않습니다. 이동 장면에서는 동기 오차와 가림에 따른
오라벨을 샘플 영상으로 점검하세요. 광각·먼 거리에서는 재투영 hole 때문에 라벨이
많이 제외될 수 있으므로 유효 비율을 실제 장면에서 평가하세요.

주요 설정 전체는 `config/distance.yaml`에 있습니다. YAML 실행은 launch 대신 다음을 씁니다.

```bash
ros2 run multiclass_yolo yolo_distance_node --ros-args \
  --params-file src/multiclass_yolo/config/distance.yaml
```

`core_fraction`, `min_valid_ratio`, `min_pixels`, `max_relative_spread`,
`min_depth_m`, `max_depth_m`, `sync_ms`, `width`, `height`, `fps`, `preview`를 조절할 수 있습니다.
기본 YOLO 모델은 512×320 holdout10이며 클래스 순서는 `con, truck, sedan, traffic_light`입니다.
`yolo_model`을 변경해도 이 입력/클래스 계약은 유지해야 합니다. `backend:=CPU`는 진단용입니다.

### 2. 학습

```bash
python3 src/multiclass_yolo/tools/distance_mlp.py \
  /tmp/distance_dataset/run_001/samples.jsonl \
  /tmp/distance_dataset/run_002/samples.jsonl \
  --output /tmp/distance_model --epochs 200
# 선택: onnx 설치 환경에서 위 명령에 --onnx 추가
```

BBox 중심/크기/면적 5개 정규화 값과 클래스 one-hot 4개를 입력합니다.
Linear(9,64)–ReLU–Linear(64,32)–ReLU–Linear(32,1), Huber(delta=1m), Adam을 사용합니다.
기본 seed=42로 시퀀스를 섞어 20%를 검증에 할당하며 최소 두 시퀀스가 필요합니다.
같은 연속 촬영/재생 데이터를 다른 시퀀스 ID로 나누면 누수가 생기므로 독립 촬영을 사용하세요.
다른 camera contract의 데이터를 섞으면 학습을 거부합니다.

`.pt`에는 state_dict와 메타데이터, `.json`에는 런타임 가중치·검증 지표·분할 목록·입력 범위를
저장합니다. Python 복원은 `DistanceMLP().load_state_dict(torch.load(path, weights_only=True)['state_dict'])`
후 `eval()`을 사용합니다. ONNX는 batch dimension이 동적인 opset 13이며 feature 입력 계약은 동일합니다.
ONNX 자체에는 거부 정책이 없으므로 사용할 경우 JSON의 camera/class/domain 검사를 함께 적용해야 합니다.

검증 시 클래스별 학습/검증 샘플이 각각 `--min-samples`(기본 30) 이상이고,
검증 MAE가 `--max-mae`(기본 0.3 m) 이하이며 예측이 모두 양수/유한해야 런타임에서 허용됩니다.
검증 클래스의 MAE/RMSE와 전체·거리 구간별 지표를 JSON에 저장합니다.
해당 검증 세트는 모델 선택에도 사용되므로 최종 성능은 **별도의 촬영 시퀀스**로 evaluate하세요.
이 기준은 검증 세트 기준의 허용 정책이며 모든 장면의 정확도를 보장하지 않습니다.
실측 데이터나 사전 학습 거리 가중치는 포함하지 않습니다.

### 3. RGB 전용 실시간 추론

```bash
ros2 launch multiclass_yolo distance.launch.py mode:=infer \
  distance_model:=/tmp/distance_model.json output_directory:=/tmp/distance_inference
```

이 모드는 CAM_A만 열고 StereoDepth/IR/포인트클라우드를 생성하지 않습니다.
저장한 camera contract가 다르면 시작을 거부합니다. 검증되지 않은 클래스,
클래스별 학습 feature 최소/최대 범위 밖의 입력, 학습 거리 범위 밖의 예측,
0 이하/비유한 예측은 `null`/`N/A`입니다. 모델 파일이 없으면 시작에 실패합니다.
프리뷰는 `Class / Confidence / Distance(m)`를 표시합니다. Q/ESC로 종료하며
`preview:=false`이면 GUI 없이 실행합니다.

`/yolo_distance/objects` (`std_msgs/msg/String`, SensorDataQoS)로 프레임별 JSON을 발행합니다.
`objects[]`에 class, confidence, 원본 bbox, `distance_m`, `stereo_distance_m`이 있으며
`timestamp_clock=depthai_host_steady`, `frame_id=oak_rgb_optical_frame`입니다.
`ros2 topic echo /yolo_distance/objects`로 확인할 수 있습니다. custom msg 생성 없이 ROS2에서
구독할 수 있으며 기존 주행 제어에는 자동 연결하지 않습니다. 소비자는 null을 반드시 처리하고,
ROS epoch가 필요하면 별도 clock mapping을 적용해야 합니다.

### 4. 별도 시퀀스 평가와 시간 측정

```bash
ros2 launch multiclass_yolo distance.launch.py mode:=evaluate \
  distance_model:=/tmp/distance_model.json output_directory:=/tmp/distance_eval
python3 src/multiclass_yolo/tools/evaluate_distance.py \
  /tmp/distance_eval/evaluate_metrics_<실제파일명>.jsonl --output /tmp/evaluation.json
```

모든 모드는 `<mode>_metrics_<unique time>.jsonl`을 남깁니다. evaluate는 MLP와 유효한 stereo
라벨이 모두 있는 객체만 거리 차이를 집계하며 전체 객체·paired 객체 수를 함께 보고합니다.
각 프레임에는 `yolo_ms`(전처리/추론/후처리), `mlp_ms`(feature/거부 검사 포함),
`yolo_mlp_ms`(두 단계 합), `processing_ms`, `capture_to_result_ms`, 누적 완료 `fps`,
RGB-depth `sync_ms`를 기록합니다. YOLO 3회 warmup은 측정에서 제외합니다.
평가 스크립트는 평균/P95 시간과 거리 구간 0–1/1–2/2–4/4–8/8m 이상 MAE/RMSE를 계산합니다.
빈 구간은 0 오차 대신 null을 출력합니다.

`capture_to_result_ms`는 exposure-middle부터 host 객체 결과까지이며 USB·큐 대기와
수집 모드의 PNG 저장을 포함하고 ROS 전송/구독자/GUI 완료 지연은 제외합니다.
FPS는 전체 루프 처리율로 저장/표시 부하가 반영되며 카메라 입력 FPS와 다를 수 있습니다.
`yolo_mlp_ms`는 동일 프레임의 연산 시간 합이므로 전체 모드 지연과 구별하세요.
실차 FPS나 정확도 수치는 아직 측정하지 않았습니다.

CPU 검증 명령:

```bash
python3 -m unittest discover -s src/multiclass_yolo/test -p 'test_distance_training.py'
g++ -std=c++17 -Isrc/multiclass_yolo/include \
  src/multiclass_yolo/test/distance_quality_test.cpp -o /tmp/distance_quality_test
/tmp/distance_quality_test
```

주요 추가 파일은 `src/distance_node.cpp`, `include/multiclass_yolo/distance*.hpp`,
`tools/distance_mlp.py`, `tools/evaluate_distance.py`, `config/distance.yaml`,
`launch/distance.launch.py`와 관련 테스트입니다. 기존 파일 수정은 빌드/의존성 등록과 이 문서에 한정됩니다.

수동주행과 수집을 함께 시작하려면 `vehicle_bringup`도 빌드한 뒤 다음을 실행합니다.

```bash
ros2 launch vehicle_bringup manual_drive_distance_collection.launch.py
```

기존 조이스틱/VESC 설정을 사용하며 자동 생성된 시퀀스로 `~/distance_dataset`에 저장합니다.
자세한 인수와 종료 동작은 `vehicle_bringup/README.md`의 수동주행 수집 항목을 참고하세요.
