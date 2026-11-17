# 기존 Jetson 워크스페이스에 변경파일만 적용하기

`distance-overlay.tar.gz`는 거리 기능의 신규/수정 파일 **11개만** 포함합니다.
전체 저장소를 clone할 필요가 없습니다. 모델·사진·영상·데이터셋·build/install과
기존 수동주행/BEV 설정은 포함하지 않습니다.
기존 `multiclass_yolo`의 CMakeLists.txt/package.xml도 덮어쓰지 않습니다.
독립 패키지 `yolo_distance_runtime`이 기존 YOLO 소스와 DepthAI 카메라 코드를 재사용합니다.

## 전제

- 이 기능을 추가하기 전 사용하던 `f1tenth_project_repo-1113ML` 계열 워크스페이스가 있어야 합니다.
- 기존 YOLO 소스/모델, `cmake/DepthaiWorkspace.cmake`, DepthAI 3.6과
  설치된 `point_cloud::point_cloud_depth_source` 라이브러리가 필요합니다.
- 임의의 오래된 기본 브랜치에는 필요한 기존 소스가 없으므로 변경 묶음만으로는 부족합니다.
- `build/install`에는 절대경로가 포함될 수 있습니다. 다른 경로로 복사하기보다
  원래 빌드에 성공했던 워크스페이스에서 기존 환경을 재사용하세요.

## 1. 작은 변경파일 묶음만 다운로드

```bash
cd /tmp
curl -fL https://raw.githubusercontent.com/bobbeartor/f1tenth/feature/yolo-distance-mlp-20261117/deployment/distance-overlay.tar.gz \
  -o distance-overlay.tar.gz
curl -fL https://raw.githubusercontent.com/bobbeartor/f1tenth/feature/yolo-distance-mlp-20261117/deployment/distance-overlay.sha256 \
  -o distance-overlay.sha256
sha256sum -c distance-overlay.sha256
tar -tzf distance-overlay.tar.gz
```

## 2. 기존 워크스페이스에 적용

아래 경로를 **Jetson에서 기존에 빌드해 사용하던 워크스페이스**로 바꿉니다.

```bash
cd /기존/빌드된/워크스페이스
ls src/multiclass_yolo/src/yolox_detector.cpp
ls src/multiclass_yolo/models/multiclass_yolox_tiny_512x320_holdout10.onnx
ls cmake/DepthaiWorkspace.cmake
cp -a src/multiclass_yolo /tmp/multiclass_yolo_before_distance_$(date +%Y%m%d_%H%M%S)
tar -xzf /tmp/distance-overlay.tar.gz
```

필요한 파일이 없다면 압축을 풀기 전에 해당 워크스페이스가 맞는지 확인하세요.
수동주행·BEV 소스, 기존 모델 및 build/install은 변경하지 않습니다.
새 거리 패키지의 설치 단계에서 Jetson에 이미 있는 기본 YOLO 모델을 로컬로 복사합니다.

## 3. 새 거리 패키지만 빌드

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg prefix point_cloud
colcon build --packages-select yolo_distance_runtime \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

`--packages-select`는 이 패키지만 빌드합니다. **roi_lidar, camera_driver, BEV,
수동주행, 기존 multiclass_yolo 패키지는 빌드하지 않습니다.** `--packages-up-to`는 사용하지 않습니다.
새 런타임 패키지는 manifest에도 roi_lidar/BEV 의존성이 없으며, 기존 YOLO/CUDA 소스만
자체 타깃으로 컴파일합니다. 최초 YOLO/CUDA 컴파일 시간은 필요하며 변경 소스는 빌드 없이 적용되지 않습니다.

실패했던 새 워크스페이스에서 전체 `source install/setup.bash`가 roi_lidar 누락 메시지를 내지만
point_cloud는 이미 성공적으로 설치되어 있는 경우, 전체 setup 대신 다음을 사용합니다.

```bash
source /opt/ros/humble/setup.bash
source install/point_cloud/share/point_cloud/local_setup.bash
ros2 pkg prefix point_cloud
# 이후 위의 --packages-select yolo_distance_runtime 명령 실행
```

point_cloud가 없거나 API가 호환되지 않으면 해당 의존성은 별도로 준비해야 합니다.
이 방법이 모든 구버전 바이너리와 호환되는 것은 아닙니다. 기존 build/install을 삭제하지 마세요.
JSON 개발 패키지를 찾지 못할 때만 `sudo apt-get install nlohmann-json3-dev`로 준비합니다.

## 4. 기존 수동주행과 수집을 두 터미널에서 실행

기존 수동/자동주행 및 OAK를 사용하는 카메라/BEV/YOLO 노드는 종료한 뒤 시작합니다.

터미널 A — 기존 정상 동작 환경에서 기존 수동주행 명령:

```bash
source /opt/ros/humble/setup.bash
source /기존/빌드된/워크스페이스/install/setup.bash
ros2 launch vehicle_bringup manual_drive.launch.py
```

터미널 B — 새 거리 수집 노드:

```bash
cd /거리기능을/적용한/워크스페이스
source /opt/ros/humble/setup.bash
source install/yolo_distance_runtime/share/yolo_distance_runtime/local_setup.bash
ros2 run yolo_distance_runtime yolo_distance_node --ros-args \
  -p mode:=collect -p sequence:=manual_001 \
  -p output_directory:=$HOME/distance_dataset
```

`sequence`는 촬영마다 `manual_002` 등으로 바꿉니다. 기존 폴더는 덮어쓰지 않습니다.
정지 상태에서 거리/저장을 확인한 뒤 수동주행으로 수집합니다.
두 터미널 방식에서는 Q/ESC가 수집만 종료하며 수동주행은 별도로 Ctrl+C로 종료합니다.
학습 스크립트는 `src/multiclass_yolo/tools/distance_mlp.py`를 사용합니다.
추론할 때도 같은 새 런타임 노드에 `-p mode:=infer -p distance_model:=/경로/model.json`을 전달합니다.

실제 Jetson 컴파일/하드웨어 동작은 아직 검증되지 않았습니다.
묶음 재생성은 `python3 deployment/make_distance_overlay.py`를 사용합니다.
파일 목록과 무결성 해시는 `distance-overlay-files.txt`, `distance-overlay.sha256`에 있습니다.
