# 실시간 거리 확인: RGB 전용 / Stereo 비교

기존 OAK CAM_A RGB → 기존 YOLOX → 학습된 MLP/LightGBM → 거리(m)를 한 C++ 노드에서 실행합니다.
`rgb_infer` 모드는 **CAM_B/C, StereoDepth, IR projector, Point Cloud를 생성/실행하지 않습니다.**
거리값은 Stereo 실측이 아니라 학습 모델의 RGB optical Z 추정입니다.

## Stereo가 N/A였던 이유

수집 화면의 N/A는 카메라에서 문자열로 전송한 값이 아니라 수집 코드가 거리 라벨을 만들지
못했음을 뜻합니다. 중앙 60% ROI에서 다음 조건을 통과해야 했습니다.

- 0.2–10m 범위의 유효 깊이 픽셀 최소 20개, 비율 10% 이상
- (P90−P10)/median <= 0.15: 물체/배경이 섞인 분포를 거부
- MAD 이상치 제거 후에도 최소 개수/비율을 충족
- 동기화된 RGB/Depth exposure-middle 차이 <= 20ms

현재 manual_003/004 정리본 784장에는 모두 동기화된 Depth가 있었지만,
YOLO 객체 3384개 중 1802개는 객체 ROI에서 신뢰할 거리 라벨을 얻지 못했습니다.
과거 로그에는 세부 ROI 거부 사유가 없으므로 픽셀 부족과 분포 불안정을 개별적으로 구별할 수 없습니다.
RGB 전용 모드에서는 Stereo 자체를 사용하지 않으므로 `stereo_status=disabled`로 기록합니다.

## 포함 모델

- `models/mlp_rgb.json`: 학습된 11입력 MLP(7개 연속 feature + 4개 클래스 one-hot)
- `models/lightgbm_rgb.json`: 같은 학습의 LightGBM을 이식 가능한 tree ensemble로 내보낸 파일
- `export_manifest.json`: 원본/내보낸 모델 SHA256
- `cpp_parity_report.json`: 실제 BBox 9979개에서 Python과 C++ 추론을 대조한 결과

LightGBM은 결과가 완전히 같았고 MLP 최대 차이는 0.000001m 미만입니다.
이 파일들은 원래 학습 모델에서 내보냈으며 재학습하지 않았습니다.
실시간 추론에는 Python, PyTorch, LightGBM 라이브러리가 필요 없습니다.
YOLO TensorRT/CUDA 및 DepthAI 3.6 공유 SDK는 기존 환경을 재사용합니다.
원래 9입력 MLP의 `infer` 모드와 구별되는 새 `rgb_infer` 모드입니다.
카메라 ID/출력 해상도/intrinsic/crop 계약이 다르면 실행을 거부합니다.
현재 제공 모델은 수집 당시 OAK(`1944301041AD2D7E00`)의 undistorted 1280×800 출력 기준입니다.
카메라 장착 위치와 기울기도 수집 때와 유지하세요. 센서 intrinsic 계약 검사만으로 기계적 장착 변화까지 감지할 수는 없습니다.

## Jetson에 변경 묶음 적용 / 빌드

전체 저장소를 다시 받지 않아도 `deployment/rgb-distance-live.tar.gz`만 기존 워크스페이스에
적용할 수 있습니다. 사진/영상/데이터셋/YOLO 가중치는 포함하지 않고 기존 YOLO 소스를 재사용합니다.
추정용 학습 모델 JSON 두 개만 포함합니다. 사용하던 소스 수정사항은 적용 전 백업하세요.

```bash
cd /tmp
curl -fL https://raw.githubusercontent.com/bobbeartor/f1tenth/feature/yolo-distance-mlp-20261117/deployment/rgb-distance-live.tar.gz -o rgb-distance-live.tar.gz
curl -fL https://raw.githubusercontent.com/bobbeartor/f1tenth/feature/yolo-distance-mlp-20261117/deployment/rgb-distance-live.sha256 -o rgb-distance-live.sha256
sha256sum -c rgb-distance-live.sha256
cd /home/autopilot03/Desktop/bjh/f1tenth_project_repo
tar -xzf /tmp/rgb-distance-live.tar.gz
source /opt/ros/humble/setup.bash
source install/oak_startup/share/oak_startup/package.bash
source install/point_cloud/share/point_cloud/package.bash
colcon build --packages-select yolo_distance_runtime \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/yolo_distance_runtime/share/yolo_distance_runtime/package.bash
```

roi_lidar/BEV/수동주행 패키지는 빌드하지 않습니다. 기존 YOLO 소스/기본 ONNX와
point_cloud의 depth_source 라이브러리는 필요합니다. 라이브러리를 링크하지만 RGB 모드에서
Point Cloud나 Stereo 계산을 하는 것은 아닙니다. SDK 및 기존 의존성을 교체하지 않습니다.
첫 실행에는 YOLO TensorRT 엔진을 빌드할 수 있고 이후 `~/.cache/yolo_distance_runtime`를 재사용합니다.

## 정지 상태에서 RGB 거리 표시 확인

같은 OAK를 여는 기존 수집/카메라/BEV/YOLO 노드는 종료한 뒤 실행합니다.

```bash
ros2 launch yolo_distance_runtime rgb_distance.launch.py model:=lightgbm
# MLP로 비교하려면 종료 후 변경
ros2 launch yolo_distance_runtime rgb_distance.launch.py model:=mlp
```

화면 상단 `RGB ONLY / lightgbm` 또는 `RGB ONLY / mlp`, FPS/YOLO 시간/거리 추론 시간을 확인합니다.
모든 YOLO 박스에 `Class / Confidence / Distance(m)`를 표시합니다.
알 수 없는 클래스/잘못된 입력/범위 밖 예측은 박스를 유지하고 해당 거리만 N/A입니다.
N/A에 임의의 거리를 채우거나 이전 프레임 거리를 재사용하지 않습니다.
현재 Test에서 정상 범위 밖이었던 MLP 출력은 RGB 모드에서도 같은 규칙으로 거부합니다.
Q/ESC로 이 카메라 노드를 종료합니다.

## 수동주행과 함께 실행

터미널 A: 기존에 정상 동작하던 조이스틱 수동주행을 그대로 실행합니다.

```bash
cd /home/autopilot03/Desktop/bjh/f1tenth_project_repo
source /opt/ros/humble/setup.bash
source install/joy_initializer/share/joy_initializer/package.bash
source install/vesc_bridge/share/vesc_bridge/package.bash
source install/manual_control/share/manual_control/package.bash
source install/vehicle_bringup/share/vehicle_bringup/package.bash
ros2 launch vehicle_bringup manual_drive.launch.py
```

터미널 B:

```bash
cd /home/autopilot03/Desktop/bjh/f1tenth_project_repo
source /opt/ros/humble/setup.bash
source install/yolo_distance_runtime/share/yolo_distance_runtime/package.bash
ros2 launch yolo_distance_runtime rgb_distance.launch.py model:=lightgbm
```

먼저 차량 정지 상태에서 표시/로그를 확인한 후 수동주행합니다.
예측 거리는 표시·로그용이며 조향/가속/제동 명령으로 연결하지 않습니다.
수동주행과 카메라 노드는 서로 독립적으로 종료해야 합니다. 카메라 Q/ESC가 수동주행까지 종료하지 않습니다.

## SSH / 로그 / ROS2

GUI가 없는 SSH에서는 `preview:=false`로 실행하고 ROS 토픽을 확인하세요.
창으로 보려면 Jetson 데스크톱 또는 정상 구성된 X11 forwarding 환경이 필요합니다.

```bash
ros2 launch yolo_distance_runtime rgb_distance.launch.py model:=lightgbm preview:=false
# 다른 터미널에서 ROS 환경을 source한 뒤
ros2 topic echo /yolo_distance/objects --qos-reliability best_effort
```

`/yolo_distance/objects`는 기존과 같은 std_msgs/String JSON입니다.
각 객체에 class/confidence/bbox/distance_m/distance_source/prediction_reason/stereo_status를 기록합니다.
`stereo_distance_m=null`, `stereo_status=disabled`, 프레임의 `stereo_enabled=false`가 정상입니다.
즉 모델 추정값은 `distance_m`을 보며 Stereo null을 센서 오류로 해석하지 마세요.
기록은 기본 `~/distance_live_logs/rgb_<실행 시각>/rgb_infer_metrics_*.jsonl`입니다.
카메라 메타데이터·모델 파일·모델 종류·Stereo 비활성 상태도 함께 저장하고 사진/동영상은 저장하지 않습니다.
거리 추정 결과를 정답 학습 라벨로 쓰지 않습니다.

주요 인수: `model`, `distance_model`(사용자 portable JSON), `preview`, `fps`,
`confidence_threshold`, `nms_threshold`, `backend`, `engine_cache`, `output_directory`.
카메라 출력 크기를 변경하면 intrinsic 계약이 달라지므로 모델의 카메라 계약을 다시 검증해야 합니다.
시간 측정은 `distance_model_ms`, `yolo_distance_ms`, `capture_to_result_ms`, `fps`로 기록합니다.

## 검증 범위

- 실제 9979개 BBox에서 Python/C++ 거리 및 거부 사유 일치 검사 완료
- launch 모델 선택/파일 누락/포맷 검사 단위 테스트
- 기존 수집/9입력 추론 코드는 유지
- 현재 개발 환경에 ROS2/DepthAI SDK/Jetson/OAK 장치가 없어 **전체 노드 빌드와 실시간 실기기 검증은 미완료**

실제 장치에서 Stereo 미생성, 화면과 수동주행 동작, FPS를 확인해야 합니다.

## 모델 재내보내기 (학습 PC)

```bash
python3 src/multiclass_yolo/tools/export_rgb_distance_models.py \
  --run results/distance_benchmark/manual_002_to_manual_001 \
  --output src/yolo_distance_runtime/models
```

이는 fitted 모델 직렬화만 변환하며 새 학습은 하지 않습니다. exporter에는 LightGBM이 필요하지만
Jetson C++ 런타임에는 필요하지 않습니다. 현재 exporter는 single-output identity-link GBDT 회귀만 지원합니다.

## 실시간 모델 거리 + Stereo 거리 동시 비교

같은 변경 묶음을 업데이트하고 yolo_distance_runtime만 다시 빌드한 뒤 실행합니다.

```bash
cd /home/autopilot03/Desktop/bjh/f1tenth_project_repo
source /opt/ros/humble/setup.bash
source install/yolo_distance_runtime/share/yolo_distance_runtime/package.bash
ros2 launch yolo_distance_runtime stereo_compare.launch.py model:=lightgbm
# MLP를 확인하려면 위 노드를 종료하고 model:=mlp로 실행
```

또는 기존 launch에 `compare_stereo:=true`를 전달합니다.

```bash
ros2 launch yolo_distance_runtime rgb_distance.launch.py model:=mlp compare_stereo:=true
```

이 모드는 `rgb_compare`이며 CAM_A RGB + CAM_B/C StereoDepth를 함께 실행합니다.
모델 입력에는 여전히 BBox/Class만 사용하며 Stereo 값은 비교용 기준일 뿐 모델에 입력하지 않습니다.
Point Cloud/BEV 계산은 하지 않습니다. 수동주행은 기존처럼 별도 터미널에서 실행합니다.
RGB-only launch 기본값 `compare_stereo:=false`는 그대로이며 Stereo를 생성하지 않습니다.

화면에는 모든 YOLO 박스를 그리고, 박스 번호에 대응하는 오른쪽 목록에 다음을 소수점 셋째 자리로 표시합니다.

```text
#1 con / conf 0.92
P: 0.688 m | S: 0.553 m | D: +0.135
```

- P = 선택한 학습 모델의 추정 거리
- S = 시간 동기화·RGB 정합·품질 검사를 통과한 Stereo 거리
- D = P−S. 둘 다 유효할 때만 표시
- Stereo N/A여도 YOLO 박스/모델 추정값은 유지하며, 모델 N/A여도 Stereo 값은 유지
- 목록이 길면 사진 위 박스 번호/거리와 JSON 로그를 함께 확인

JSON/ROS 객체에는 `distance_m`, `stereo_distance_m`, `prediction_minus_stereo_m`을 함께 저장합니다.
`stereo_status=valid/unavailable/disabled`, `stereo_reason`, ROI/유효 픽셀 수·비율·분포 폭도 기록합니다.
Stereo 무효 사유:

- `no_synchronized_depth`: 허용 시간 차이 안의 Depth 프레임 없음
- `empty_roi`, `too_few_valid_pixels`, `low_valid_pixel_ratio`: ROI/깊이 표본 부족
- `unstable_depth_distribution`: P90−P10 분포 폭 기준 초과
- `too_few_inliers`, `low_inlier_ratio`: MAD 제거 후 표본 부족
- `disabled`: RGB-only로 실행한 정상 상태

로그는 `~/distance_live_logs/rgb_<실행 시각>/rgb_compare_metrics_*.jsonl`에 저장됩니다.
화면을 띄우지 않을 경우 `preview:=false`를 사용하고 `/yolo_distance/objects`를 확인합니다.

비교용 Stereo 설정은 `stereo_resolution:=400p`, `stereo_subpixel:=false`, `stereo_ir_dot:=0.5`,
`sync_ms:=20.0`이 기본입니다. 실험 시 이를 명시적으로 바꿀 수 있지만 가까운 거리의 유효 범위/FPS도 확인해야 합니다.
이 기능에서도 전체 Jetson/카메라 실기기 실행은 미검증이며, 진단 필터·launch 선택·기존 모델 수치 검증을 수행했습니다.

### 0.7m 이하 데이터 확인 결과

현재 학습 시퀀스 manual_002_curated에는 0.7m 이하 65개(5826개 중 약 1.1%)가 있고 최소 Stereo는 0.542m입니다.
검증 manual_001에는 28개, Test manual_003/004 정리본에는 각각 6개/10개가 있습니다.
프로그램에는 0.7m 하한이 없습니다. portable 모델의 허용 범위는 0.2–10m입니다.
학습 근거리 샘플의 현재 모델 예측 최솟값은 LightGBM 약 0.688m, MLP 약 0.536m였습니다.
이는 해당 샘플에서의 관측값이며 모델의 이론적 하한이 아닙니다. 실차에서의 하한처럼 보이는 현상은
모델 추정 특성과 근거리 표본 부족에 부합하지만, BBox/카메라 장착 변화 등은 P/S 실시간 비교로 따로 확인해야 합니다.
