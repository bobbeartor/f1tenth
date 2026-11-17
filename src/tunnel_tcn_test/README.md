# tunnel_tcn_test

ROS 2 Humble / Jetson에서 CAM_A RGB 채널로 터널 진입·탈출을 확인합니다.
기존 `BrightnessTCN31_2s_v1` 부분 INT8 모델은 제거했고,
`20261023_140042_191723_aug/final_model.pt`로 교체했습니다.

## Jetson에서 빌드·실행

Jetson의 `f1tenth_project_repo` 디렉터리에서 실행합니다.
기존 카메라 드라이버와 동일한 DepthAI C++ 3.6+, CUDA, OpenCV 환경과
JetPack의 TensorRT Python 8.6 또는 10.x가 필요합니다.
런타임에는 PyTorch, ONNX Runtime, PyCUDA가 필요하지 않습니다.

```bash
cd /path/to/f1tenth_project_repo
source /opt/ros/humble/setup.bash
python3 -c "import tensorrt, numpy, cv2; print('TensorRT', tensorrt.__version__)"
colcon build --symlink-install --packages-up-to tunnel_tcn_test \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch tunnel_tcn_test tunnel_tcn_test.launch.py
```

기존 워크스페이스에서 별도 `depthai_DIR`을 사용했다면 동일 CMake 인자를 추가합니다.
TensorRT Python이 없다면 해당 JetPack의 `python3-libnvinfer`를 설치합니다.
카메라를 이미 여는 다른 launch는 종료합니다. 데스크톱 화면이 필요합니다.
첫 실행은 Jetson용 TensorRT 엔진을 생성합니다. `TensorRT ready` 뒤에 시작하며,
캐시는 `~/.cache/tunnel_tcn_test`에 모델 해시별로 저장되어 기존 INT8 엔진과 섞이지 않습니다.

기본 동작: RGB 1280×800 / 30FPS, RGB 자동 노출, 10Hz 추론,
임계값 0.5, 같은 종류 이벤트 쿨다운 1초, 진입·탈출 교대, 초기 상태 UNKNOWN.
IR projector/flood와 IR 제어 창은 끕니다. 기존 카메라 드라이버의 스테레오 파이프라인은
함께 시작하지만 모델은 RGB 토픽만 사용하며 BEV와 IMU 처리는 실행하지 않습니다.
기존 `exposure_us`, `iso`, `projector`, `flood` launch 인자는 제거했습니다.
이 인자들은 IR 카메라 설정이어서 RGB 노출을 제어하지 않습니다.

터널 밖에서 시작한다는 것이 확실하면:

```bash
ros2 launch tunnel_tcn_test tunnel_tcn_test.launch.py initial_state:=outside
```

설정 변경 예시:

```bash
ros2 launch tunnel_tcn_test tunnel_tcn_test.launch.py \
  threshold:=0.5 warmup_seconds:=2.5 initial_state:=unknown
```

장치가 여러 개면 `device_id:=장치_MXID`를 추가합니다.
초기 상태는 `unknown`, `outside`, `inside` 중 하나입니다.
노드만 실행하고 기존 RGB 카메라 토픽에 연결할 수도 있습니다.

```bash
ros2 run tunnel_tcn_test tunnel_tcn_node --ros-args \
  --params-file "$(ros2 pkg prefix tunnel_tcn_test)/share/tunnel_tcn_test/config/tunnel_tcn.yaml" \
  -p image_topic:=/camera/image_rect
```

## 입력·출력과 화면

`/camera/image_rect`의 NV12는 RGB로 변환하지 않고 Y 채널만 읽습니다.
행 패딩(`step`)을 반영해 Y를 640×400 INTER_AREA로 축소한 뒤,
`round(clip((Y - 16) × 255 / 219, 0, 255))` LUT로 밝기 범위를 복원합니다.
UV 채널과 전체 해상도 RGB 중간 버퍼는 처리하지 않으며 미리보기도 이 흑백 영상을 사용합니다.
외부 카메라의 `rgb8`, `bgr8`, `mono8` 입력은 기존 전처리를 유지합니다.

NV12 경로는 제한 범위 BT.601 출력을 가정합니다. 기존 RGB 변환 후 밝기 계산과는
채널별 포화, 반올림, 축소 순서 때문에 완전히 같지 않습니다. 특히 출구의 포화 영상에서
차이가 날 수 있으므로 실제 영상으로 ENTRY/EXIT 점수와 검출 시점을 비교해야 합니다.
모델 가중치와 특징 정규화 값은 변경하지 않습니다.

원본 학습과 동일한 공간 특징과 0.5초 변화량·1초 과거 통계를 계산하고,
체크포인트에 저장된 평균/표준편차로 정규화합니다. 원본 촬영 시각을 사용하며
입력 공백에서는 미래 프레임을 참조하거나 보간하지 않습니다.
학습 영상은 10FPS이며, 라이브 30FPS에서는 동일한 특징 공식을 촬영 시각 기준으로
계산한 후 10Hz 문맥을 선택합니다. 라이브 카메라와 학습 영상의 센서·노출·영상
압축 차이에 따른 실제 검출 성능은 주행으로 확인해야 합니다.

입력 float32 `[1,17,32]`는 특징 16개 + 유효 과거 마스크, 10Hz의 32시점
(첫 시점에서 마지막 시점까지 3.1초)입니다. 출력 `[1,2]` logits에 **독립 sigmoid**를
적용합니다. ENTRY와 EXIT 점수는 합이 1인 클래스 확률이 아니며 보정된 확률도 아닙니다.
NONE 점수나 softmax를 만들지 않습니다. 두 점수가 임계값을 동시에 새로 넘으면
허용된 후보 중 점수가 높은 이벤트를 선택합니다. 거절한 교차는 나중에 재출력하지 않습니다.

`Tunnel TCN - RGB FP32` 창에서 ENTRY/EXIT 점수, 상태, 최근 이벤트, 수신 FPS,
누락 프레임 추정값, TensorRT 및 프레임 처리 시간을 확인합니다.
R은 새 주행으로 초기화하고 Q/Esc는 종료합니다. 상태는 첫 확정 ENTRY 후 INSIDE,
첫 EXIT 후 OUTSIDE로 바뀝니다. R은 설정한 `initial_state`로 돌아갑니다.
입력 단절 후 특징 이력은 재구성하지만 터널 상태는 유지합니다.

시작·R·0.5초 초과 입력 공백 이후 기본 2.5초는 WARMUP으로 이벤트를 보류합니다.
초기 경계 검출이 필요하면 `warmup_seconds:=0.0`을 선택할 수 있지만 시작 직후
검출을 직접 평가해야 합니다. 학습의 초기 과거 마스크는 warmup 0초에서도 지원됩니다.
이미지가 0.25초보다 오래되면 추론하지 않습니다.

```bash
ros2 topic echo /tunnel_tcn/result
ros2 topic hz /camera/image_rect
```

결과는 약 10Hz `std_msgs/String` JSON입니다. `scores: {entry, exit}`, `model_id`,
`event`, `state`, `source_stamp_ns`, 주행 기준 `inference_time_s`, FPS·처리 시간을 담습니다.
이전 `probabilities: [none, entry, exit]` 필드는 제거되었습니다.
`TRT+transfer`는 카메라부터 화면까지의 전체 지연이 아닙니다.

## 검증

Jetson에서 카메라 없이 TensorRT 결과를 원본 PyTorch 참조와 비교합니다.

```bash
ros2 run tunnel_tcn_test verify_model --backend tensorrt \
  --output /tmp/tunnel_rgb_tensorRT_check.json
```

`passed: true`를 확인합니다. 4개 영상, 2,361개 추론 시점에서 점수 오차 0.005 이하와
확정 이벤트 종류·시각의 일치를 검사합니다. 참조 재생은 warmup 0초입니다.
영상 1개는 최종 재학습 데이터이며 나머지는 미라벨 외부 영상입니다.
이 검증은 변환의 수치 일치 검사이고 정답 기준 검출 정확도 평가가 아닙니다.

개발 PC에서는 `numpy`, `opencv-python`, `onnxruntime` 환경에서:

```bash
PYTHONPATH=src/tunnel_tcn_test python3 -m unittest discover -s src/tunnel_tcn_test/test -v
PYTHONPATH=src/tunnel_tcn_test python3 -m tunnel_tcn_test.verify --backend onnx
```

로컬 검증: 테스트 15개 통과, ONNX 최대 점수 오차 약 1.04e-6,
4개 영상의 이벤트 종류·시각 모두 일치.
`test/local_replay_report.json`에 결과가 있습니다.
ROS 2 빌드, TensorRT 실행, 실카메라 주행은 이 Mac 환경에서 검증하지 못했습니다.

## 모델 출처와 재생성

`models/final_model.pt`는 원본 체크포인트의 바이트 그대로입니다.
실행에는 `models/rgb_point_events_fp32.onnx`를 사용합니다.
추가 학습·INT8 양자화 없이 FP32로 변환했고, `inference_config.json`에 정규화 값,
`provenance.json`에 원본·ONNX·참조 데이터 해시를 저장했습니다.

개발용 PyTorch/ONNX/ONNX Runtime/OpenCV 환경에서 재생성하려면:

```bash
python3 src/tunnel_tcn_test/tools/export_rgb_model.py /path/to/brightness_analysis
```

고정 입력과 TensorRT 8.6 호환 opset 13을 위해
[PyTorch ONNX exporter](https://docs.pytorch.org/docs/stable/onnx)로 변환합니다.
최신 모델도 검증 영상의 진입 시점 오차가 남아 있습니다. 실제 주행에서 임계값과
카메라 노출을 평가한 후 사용하세요. 현재 패키지는 주행 제어를 변경하지 않는 테스트 노드입니다.
