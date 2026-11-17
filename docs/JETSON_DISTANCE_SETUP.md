# Jetson: YOLO 거리 수집 노드 받기

브랜치: `feature/yolo-distance-mlp-20261117`

이 브랜치는 현재 `f1tenth_project_repo-1113ML`의 실행 소스와 모델을 담습니다.
원격 기본 브랜치의 예전 소스와 버전 차이가 크므로 새 작업 폴더를 권장합니다.
사진/영상 데이터셋, 성능 로그, build/install/log, 캐시와 TensorRT engine은 제외합니다.
기존 기능의 ONNX 모델, label/metadata, 테스트용 작은 golden replay는 포함합니다.
기존 Git 이력은 유지하며 이력에 있는 과거 이미지까지 삭제하지는 않습니다.
`--depth 1 --single-branch`로 이 브랜치만 받으면 과거 데이터 다운로드를 줄일 수 있습니다.

## 1. 새 폴더로 받기

```bash
git clone --depth 1 --single-branch \
  --branch feature/yolo-distance-mlp-20261117 \
  https://github.com/bobbeartor/f1tenth.git ~/f1tenth-distance-ws
cd ~/f1tenth-distance-ws
git status --short
git log -1 --oneline
```

이미 이 브랜치를 받은 폴더를 업데이트할 때:

```bash
cd ~/f1tenth-distance-ws
git pull --ff-only
```

다른 브랜치에서 작업 중인 기존 차량 워크스페이스에 바로 덮어쓰지 마세요.

## 2. 기존 환경 확인

```bash
source /opt/ros/humble/setup.bash
command -v colcon
command -v nvcc
ls /usr/local/lib/cmake/depthai/depthaiConfig.cmake
```

DepthAI 공유 SDK **3.6**, TensorRT/CUDA 개발 라이브러리 및 OpenCV가 필요합니다.
SDK가 lib64 등에 있다면 `DEPTHAI_SETUP.md`의 기존 경로 설정을 따릅니다.
위 환경 확인 명령에서 경로가 다르다고 새 SDK로 교체하지 마세요.
JSON 개발 헤더가 없다면 다음을 설치합니다.

```bash
sudo apt-get update
sudo apt-get install nlohmann-json3-dev
```

## 3. 빌드

```bash
cd ~/f1tenth-distance-ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to multiclass_yolo \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 pkg executables multiclass_yolo
```

출력에 `multiclass_yolo yolo_distance_node`가 있어야 합니다.
실제 Jetson/SDK에서 이 새 노드의 빌드는 아직 검증되지 않았습니다.
오류가 나면 명령과 첫 번째 오류부터 확인하세요.

## 4. 차량 정지 상태에서 수집 확인

동일 OAK 장치를 여는 기존 YOLO/camera_driver/BEV 노드를 먼저 종료합니다.
이 노드는 YOLO와 StereoDepth를 자체 실행합니다. 자율주행 제어를 실행하지 않습니다.

```bash
mkdir -p ~/distance_dataset
ros2 launch multiclass_yolo distance.launch.py \
  mode:=collect sequence:=stationary_001 \
  output_directory:=$HOME/distance_dataset
```

클래스·검출 점수·거리를 확인하고 Q/ESC로 종료합니다.
유효한 객체를 검출했을 때 저장 결과를 확인합니다.

```bash
ls ~/distance_dataset/stationary_001/images
head -n 1 ~/distance_dataset/stationary_001/samples.jsonl
```

거리는 RGB optical Z(m)이며 유효한 깊이가 없으면 N/A입니다.
정합 확인 후 수동주행으로 거리/각도/조명을 바꾸어 추가 수집합니다.
촬영 회차마다 `sequence`를 새로 지정하고, 최종 평가용 촬영은 학습에서 제외합니다.
학습·RGB 전용 추론·평가는 `src/multiclass_yolo/README.md`를 따릅니다.

## 5. 수동주행과 수집을 한 번에 실행

기존에 받은 폴더에서 브랜치를 업데이트하고 수동주행 패키지도 빌드합니다.

```bash
cd ~/f1tenth-distance-ws
git pull --ff-only
source /opt/ros/humble/setup.bash
colcon build --packages-up-to vehicle_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch vehicle_bringup manual_drive_distance_collection.launch.py
```

기존 수동주행과 같은 컨트롤러 설정을 사용하고, 데이터는
`~/distance_dataset/manual_<실행 시각>/`에 저장합니다. 기존 수동주행·자동주행과 OAK를 여는
다른 노드는 종료한 뒤 실행하세요. 이 launch 하나가 수동주행과 YOLO+Stereo 수집을 함께 시작합니다.
먼저 정지 상태에서 검출·거리·저장을 확인한 뒤 수동주행으로 수집합니다.
촬영 시퀀스 이름은 매번 자동 생성되며 `sequence:=manual_track_001`로 직접 지정할 수도 있습니다.
Q/ESC 또는 수집 노드 종료 시 이 launch 전체에 shutdown을 보냅니다.
실제 Jetson/차량에서 통합 실행은 아직 검증되지 않았습니다.
