# line_detactor

INT8 PTQ 변환·검증 절차와 현재 결과는 [tools/INT8_PTQ.md](tools/INT8_PTQ.md)에 정리되어 있다.

Fast-SCNN HighRes로 원본 BEV의 좌우 차선과 정지선을 추론한다. 좌우 차선 후처리는 **작은 노이즈
성분 제거와 화면 좌우 경계에서 잘린 차선의 바깥쪽 보간**만 수행한다.

원본 `/camera/image_bev`, BEV 변환, 모델 입력은 **120×300 그대로**다.
프리뷰/결과 도화지에만 좌우 30px씩 검은 여백을 추가해 **180×300**으로 만든다.
좌우 차선은 파란색/빨간색, 정지선은 초록색, 중앙 경로는 노란색이다. 기존 경로계획·조향 제어는 변경하지 않는다.

## 정지선 모델 적용 순서

현재 기본 모델은 **`models/fast_scnn_stop_line_120x300_batch_1.onnx`**다.
기존 2채널 ONNX는 보존하지만 새 노드에서 사용하지 않는다. 진행 중인 0910 학습이
끝나면 **PC에서** 아래 명령으로 최종 best.pt를 변환한 뒤 이 패키지를 젯슨으로 옮긴다.
현재 코드 수정 시점에는 학습이 진행 중이므로 최종 배포 ONNX를 자동으로 확정하지 않았다.

현재 Windows 작업공간 `C:\Users\godld\Desktop\traffic`에서:

```powershell
line_detector/.venv-export/Scripts/python.exe line_detactor_ros/tools/export_stop_line_onnx.py
```

기본적으로 옆 `line_detector/training/highres/stop_line_0910/runs`의 최신 best.pt를
선택한다. 특정 모델은 `--checkpoint "실제/best.pt"`로 지정한다. 기존 학습 환경을
변경하지 않도록 별도 `.venv-export`를 준비했다. 다른 PC에서는 별도 가상환경에
`tools/requirements-export.txt`를 설치하면 된다. 모델 정의는 tools/model_definitions에
포함되어 있어 --checkpoint를 지정하면 원래 학습 코드 없이도 변환할 수 있다.

변환은 정적 FP32 입력/출력 `[1,3,300,120]`, opset17, 로짓 출력이다. 고정 입력의
비균등 adaptive pooling을 같은 구간의 AvgPool/평균 연산으로 표현하고 ONNX checker와
ONNX Runtime CPU로 PyTorch 출력과 비교한다. 최종 테스트26장은 변환 검증에 사용하지 않는다.
`*.onnx` 옆 `*.json`에 원본 체크포인트 epoch/SHA256와 수치 검증 결과를 기록한다.
변환에만 PyTorch가 필요하며 **젯슨 노드 실행에는 PyTorch/ONNX Runtime이 필요 없다.**

명시적으로 예전 `model_path`를 지정한 YAML은 새 ONNX 경로로 수정한다. 기본 경로를
사용한다면 YAML 변경은 필요 없다. 새 ONNX는 다른 이름이므로 기존 엔진 캐시와 분리된다.
명시된 엔진 캐시가 2채널이면 새 엔진으로 재생성한다. 학습 후 ONNX를 다시 갱신했다면
이전 캐시보다 새 수정 시간이 유지되어야 하며, 파일 시간을 보존하며 복사한 경우 해당
캐시를 제거하고 재생성한다. 엔진은 배포할 젯슨에서 생성한다.

공식 참고: [PyTorch ONNX export](https://docs.pytorch.org/docs/stable/onnx),
[TensorRT 엔진 호환성](https://docs.nvidia.com/deeplearning/tensorrt/latest/inference-library/engine-compatibility.html).

## 후처리 범위

1. 모델 출력의 좌우/정지선 채널에 독립 sigmoid threshold를 적용한다. 좌우 중복은
   기존처럼 로짓이 큰 쪽에 배정한다. 정지선은 독립 마스크에 남겨 좌우와의 교차점을
   보존한다. 입력 RGB `[1,3,300,120]`, 출력 로짓 `[1,3,300,120]`이다.
2. 좌우 마스크 각각에서 `connection_min_component_area_px`보다 작은 연결
   성분만 삭제한다. 그 외 성분은 위치·두께·모양·분리 상태를 그대로 유지한다.
3. **실제로 원본 영상의 x=0 또는 x=width-1에 닿은 성분만** 보간 후보로 조사한다.
   중심선은 끝점/방향 추정에만 사용하며 검출 마스크를 중심선으로 다시 그리지 않는다.
4. 같은 모델 채널, 같은 화면 경계에 있는 서로 다른 성분의 끝점끼리만 검사한다.
   양쪽 끝점의 접선이 모두 화면 밖을 향하고, 끝점 사이 거리·방향 허용 폭·곡률
   조건을 통과하면 3차 베지어 곡선으로 보간한다.
5. 곡선은 해당 경계의 바깥에만 있어야 하고 확장 도화지를 벗어나면 제외한다.
   추가 선의 두께가 원본 영역을 침범하지 않도록 래스터화 후에도 원본 영역을
   마스킹한다. 새 선끼리 겹치면 후순위 연결은 제외한다. 짧은 후보부터 선택하며
   한 끝점은 한 번만 사용한다.

정지선에는 위 성분 제거·경계 보간을 적용하지 않는다. 정지선 마스크는 별도로 패딩만
추가하고, 화면 밖 패딩은 항상 0이다. 결과 영상에서는 정지선의 초록색이 겹친 차선보다
우선 표시된다. 노란 중앙 경로는 마지막에 덧그리지만 기존 좌우 라벨/정지선 마스크/상태는 변하지 않는다. 원본에서 흰색이 보이지 않는다는
이유로 모델이 복원한 정지선을 지우지 않는다.

**내부 끊김 연결, 스플라인 평활화, 차선을 한 개로 강제 선택하기, 좌우 순서에
따른 차선 전체 삭제는 제거했다.** 결과적으로 같은 쪽 차선 조각이 여러 개 남을
수 있다. 이 정책은 차선 마스크에 적용된다. 별도 중앙 경로 생성기는 관측 라벨 `1/2`로
경로 후보를 만들고 연결·평활화하며, 그 결과로 차선 마스크를 수정하지 않는다.

노이즈 제거는 영상 전체의 작은 연결 성분에만 적용한다. 화면 밖은 원래 검출
입력이 없으므로 확장 여백의 차선 라벨에는 승인된 보간 선만 생긴다.
노란 중앙 경로는 관측 차선에서 안쪽으로 이동한 결과가 여백에 들어올 수 있다. 한쪽 끝점만 있으면
무작정 연장하지 않는다. 상하 경계 잘림, BEV 내부의 검은 유효영역 경계, 일반적인
가림/내부 검출 공백은 이 보간의 대상이 아니다. 화면 밖 보간은 관측이 아닌 추정이다.

## 빌드 및 실행

Jetson의 CUDA·TensorRT 개발 패키지, OpenCV, ROS 2 Humble,
`rosidl_default_generators`가 필요하다. PyTorch는 사용하지 않는다.
자동주행 launch는 `bev_handoff`를 통해 같은 프로세스의 CUDA BEV
버퍼를 TensorRT 전처리에 직접 넘기고, 아래의 독립 실행만 ROS 영상
토픽과 CPU→GPU 업로드를 사용한다.
이번 변경에는 `LaneResult` 중앙 경로 필드 추가가 있으므로 발행/구독 환경 모두
같은 메시지 버전으로 재빌드하고 환경을 다시 읽는다.

```bash
cd ~/Desktop/0906ML/f1tenth_project_repo
source /opt/ros/humble/setup.bash
colcon build --packages-select bev_handoff line_detactor
source install/setup.bash
```

터미널 1에서 원본 BEV를 발행한다. BEV YAML의 `publish_enabled`는 `true`여야
하며 초기 depth/IMU 측정이 끝날 때까지 차량을 정지시킨다.

```bash
ros2 launch bev_processor bev_processor.launch.py \
  preview_enabled:=false \
  publish_enabled:=true
```

터미널 2: 저장소 루트에서 파라미터 파일을 지정한다.

```bash
ros2 launch line_detactor line_detactor.launch.py \
  params_file:="$(pwd)/src/line_detactor/config/line_detactor.yaml"
```

테스트 YAML을 사용하는 경우:

```bash
ros2 launch line_detactor line_detactor.launch.py \
  params_file:=/absolute/path/test.yaml \
  connection_enabled:=true
```

기존 테스트 YAML에서 **`smoothing_*` 전부와 `connection_min_lane_length_px`를
삭제**한다. 이 파라미터들은 더 이상 읽거나 적용하지 않는다. 예전 YAML에
`smoothing_enabled: true`가 남아 있어도 보정은 다시 켜지지 않는다. 예전
`smoothing_*` launch 인자도 제거한다. 새 항목은 아래처럼 추가한다.

```yaml
line_detactor:
  ros__parameters:
    connection_enabled: true
    connection_min_component_area_px: 8
    connection_border_endpoint_distance_px: 6.0
    result_padding_px: 30
```

같은 키가 이미 있으면 중복 추가하지 말고 수정한다. 다른 기존 `connection_*`
설정은 아래 설명에 따라 재사용할 수 있다. 생략한 항목은 노드 기본값을 쓴다.
우선순위는 명시한 launch 인자 > 지정 YAML > 노드 기본값이다. 변경 후 재실행한다.

`connection_enabled:=false`는 노이즈 제거/보간/결과 발행을 끄고 원본 추론
프리뷰를 표시한다. `result_publish_enabled:=false`는 발행만 끈다.
`preview_enabled:=false`여도 후처리와 결과 발행은 동작한다.

## 가벼운 BEV 테마

`bev_theme_enable`의 기본값은 **false**다. true이면 흰 배경,
`#C7CCD4` 회색 차선, `#2699E6` 두꺼운 중앙선, 차선보다 2배 두꺼운 정지선,
10cm 거리 눈금(1/2/3m 긴 눈금), 오른쪽 위 속도 원과 그 아래 신호 원을 표시한다.
신호는 빨강/초록으로 채우고 UNKNOWN 또는 만료 시 테두리만 표시한다.
하단은 차선 추론·후처리·제어 평균, 신호등 실제 평균 FPS, 프리뷰 FPS, 정지선 거리다.
OpenCV 기본 글꼴을 사용하는 하단 항목명은 영어로 표시한다.
기본 창 콘텐츠 크기는 기존과 같은 **360×776px**이다.

```yaml
line_detactor:
  ros__parameters:
    bev_theme_enable: false
    bev_theme_preview_fps: 15.0
    bev_theme_speed_topic: "/auto/current_speed"
```

자동주행에서는 지정한 `line_detactor_params_file`에 넣거나 launch로 켠다.
`auto_control_test.yaml`의 제어 파라미터가 아니다.

```bash
ros2 launch vehicle_bringup auto_drive.launch.py bev_theme_enable:=true
# 독립 차선 검출 launch도 같은 인자를 지원한다.
ros2 launch line_detactor line_detactor.launch.py bev_theme_enable:=true

# 실행 중 프리뷰 전환. 다른 프리뷰 설정은 시작 시 읽는다.
ros2 param set /line_detactor bev_theme_enable true
ros2 param set /line_detactor bev_theme_enable false
```

`connection_enabled=true`가 필요하며, `preview_enabled=false`이면 화면은 열지 않는다.
테마를 끄면 기존 `preview_result_only_enabled` 설정의 프리뷰로 복귀한다.
테마를 끈 일반 프리뷰 하단에도 `Target: 목표 속도`, `Speed: 현재 속도`(m/s),
`Corner: 상태`를 표시한다. 원본 영상 포함/결과만 표시 모드 모두 적용한다.
목표 속도는 `preview_target_speed_topic`(기본 `/auto/target_speed`), 코너 상태는
`preview_corner_state_topic`(기본 `/auto/corner_state`, UInt8)을 구독한다.
`NO`(0)는 회전/접근 판정 없음, `APPROACH`(1)는 코너 속도 계획 활성,
`TURNING`(2)은 근거리 경로·조향 기준 회전 및 탈출 확인 중,
`STOP`(3)은 정지 목표·저속 정지 또는 제어 중지를 뜻한다.
이는 제어기 추정이며 트랙의 실제 코너 경계를 검출한 결과는 아니다.
특히 연속 코너에서는 `TURNING`이 유지될 수 있고, 코너 감속 OFF에서는 `APPROACH`가 나오지 않는다.
각 데이터가 0.5초 이상 끊기거나 아직 수신되지 않았으면 해당 항목은 `--`로 표시한다.
통합 launch는 외부 auto_control YAML의 목표 속도·코너 상태 토픽과도 자동으로 맞춘다.
주행/튜닝 파라미터 추가 없이 새 노드를 빌드해 실행하면 표시된다. 이번 변경은 빌드·테스트 미진행.

속도는 `bev_theme_speed_topic`에서 읽으며 0.5초 이상 새 값이 없으면 `--`로 표시한다.
자동주행 launch는 이 토픽을 auto_control의 `current_speed_topic`과 맞춘다.
신호등 FPS는 같은 프로세스의 traffic detector가 최근 약 1초 동안 실제 완료한 모델
호출 수를 경과 시간으로 나눈 값이다. 입력 유실·초기화·오류로 측정이 없거나 만료되면
`-- FPS`로 표시한다. 설정된 추론 상한이나 추론시간의 역수를 대신 표시하지 않는다.

프리뷰는 상태 확인용 약도다. 좌우 차선과 중앙선은 이미 계산된 점열을 화면에서만
단순화하며 정지선은 측정 거리와 중앙선 방향, 설정 차선 폭을 이용한 간단한 가로 막대다.
정지선 마스크를 다시 분석하지 않는다. 점선은 기록용 지도가 아닌 속도 기반 이동 표시다.
이 단순화는 발행되는 차선/중앙선 좌표, 정지선 거리 및 제어 로직에 적용되지 않는다.

GUI는 최신 결과 하나만 읽고 기본 최대 15FPS로 표시한다(`preview_fps`가 더 낮으면 그 값).
차선 형상은 새 결과를 표시할 때만 갱신하고, 눈금·고정 항목을 포함한 화면을 캐시한다.
확대 후 축소, Gaussian blur, 추가 추론, 마스크 재가공은 하지 않으며 경계는 선의
안티앨리어싱만 사용한다. 점선은 프레임당 최대 256개로 제한한다.
테마만 표시하고 결과 영상 구독자가 없을 때는 추론 작업의 기존 BGR 표시 영상과
중앙선/정지선 표시 마스크 생성을 생략한다. 보간/라벨 정책은 테마 전환과 독립적으로 유지한다.
Linux GUI 스레드에는 낮은 CPU 우선순위(nice 10)를 요청하며, GUI 예외가 발생하면
프리뷰만 종료하고 추론/제어는 유지한다. 사용자가 창을 닫는 기존 종료 동작은 유지한다.
성능 측정 모드는 기존대로 GUI를 끈다. 이 변경에서는 요청에 따라 빌드·테스트·실차 성능
측정을 실행하지 않았으므로 주행 성능에 영향이 없다는 실측 보장은 하지 않는다.

## 파라미터

거리 단위는 확대 전 BEV 픽셀이다. 모두 테스트/실차 검증 전 시작값이다.

| 파라미터 | 기본값 | 영향 |
|---|---:|---|
| `connection_enabled` | `true` | 작은 성분 제거 + 바깥 경계 보간 활성화 |
| `result_padding_px` | `30` | 좌우 각각의 도화지 여백. 0이면 바깥 보간 없음 |
| `connection_min_component_area_px` | `8` | 면적이 이보다 작은 성분만 삭제. 1이면 삭제 없음 |
| `connection_skeleton_downsample_factor` | `1` | 골격 추출 해상도의 축소 배수(정수 1..4). 2이면 조각 ROI의 가로·세로를 약 절반으로 축소하고 추출 좌표를 복원. 원본 차선 라벨은 유지 |
| `connection_border_endpoint_distance_px` | `6.0` | 중심선 끝점과 실제 화면 경계 검출 픽셀 사이 허용 거리 |
| `connection_min_fragment_length_px` | `8.0` | 보간 방향 추정에 필요한 최소 중심선 길이. 미달해도 원본 마스크 유지 |
| `connection_max_fragments` | `24` | 한 채널에서 조사할 경계 성분 수. 초과 성분도 마스크는 유지 |
| `connection_tangent_window_px` | `8.0` | 끝점 접선을 추정하는 구간 길이 |
| `connection_max_gap_px` | `80.0` | 두 경계 끝점 사이 최대 거리 |
| `connection_corridor_half_width_px` | `4.0` | 방향 예측 허용 반폭. 표시 두께와 별개 |
| `connection_direction_tolerance_deg` | `20.0` | 거리에 따라 예측 폭을 넓히는 각도 |
| `connection_max_turn_deg` | `180.0` | 연결 시작/도착 접선의 최대 방향 차이 |
| `connection_max_curvature_per_px` | `0.12` | 보간 곡선의 최대 곡률. 작게 하면 급한 연결 제외 |
| `connection_max_arc_ratio` | `1.8` | 보간 길이/끝점 직선거리 상한 |
| `result_line_width_px` | `2` | 바깥 보간 선의 두께만 변경. 모델 차선 두께는 유지 |
| `result_publish_enabled` | `true` | 결과 메시지/영상 발행 |
| `mask_threshold` | `0.5` | 좌우/정지선 각각의 sigmoid 임계값 |
| `overlay_alpha` | `0.75` | 프리뷰 오버레이 불투명도 |
| `preview_fps` | `30.0` | 전용 GUI 스레드의 최대 표시 빈도. 추론/결과 발행 제한 아님 |
| `preview_scale` | `2.0` | 프리뷰 확대 배율. 좌표/파라미터에 영향 없음 |
| `warmup_iterations` | `10` | TensorRT 워밍업 횟수 |
| `status_log_interval_sec` | `1.0` | 로그 간격 |

## 노란 중앙 경로와 YAML 조정

### 연산량을 줄이는 샘플링 설정

아래 세 파라미터는 YAML과 `line_detactor.launch.py`의 동일 이름 인자로 설정할 수 있다.
모두 시작 시 읽으므로 변경 후 노드를 재시작한다. 기본값은 기존 기본 설정의 해상도와
출력 간격, 검사 간격을 유지한다. 아래 시험값은 성능·정확도를 측정한 결과가 아니다.

| 파라미터 | 기본값 | 시험 시작값 | 허용 범위 / 영향 |
|---|---:|---:|---|
| `connection_skeleton_downsample_factor` | `1` | `2` | 정수 1..4. 골격 추출과 골격 그래프 탐색을 더 작은 ROI에서 수행 |
| `centerline_output_spacing_m` | `0.01` | `0.02` | 0.005..0.10m. 최종 경로 재샘플링 간격이며 후속 평활화·발행의 점 수도 감소 |
| `centerline_clearance_check_spacing_m` | `0.005` | `0.01` | 0.001..0.05m. 후보 연결·최종 경로·평활화 검증에서 사용하는 최대 구간 검사 간격 |

`centerline_sample_spacing_m`은 관측 차선에서 중심선 **후보**를 만드는 간격이다.
새 `centerline_output_spacing_m`은 경로 탐색 **후** 평활화와 발행에 사용할 간격이며 서로 독립이다.
출력 점 수는 경로 길이에 따라 정해지고 끝점을 포함하므로 실제 간격은 설정값 이하다.
짧은 경로가 7점 미만이 되면 기존 조건에 따라 최종 평활화를 생략한다.

골격 축소는 각 연결 성분을 따로 면적 보간한 뒤, 차선이 조금이라도 차지하는 축소 픽셀을
남긴다. 그 해상도에서 골격을 추출하고 실제 ROI 크기 비율로 원래 픽셀 중심 좌표에 복원한다.
유효한 골격 경로가 나오지 않는 작은/퇴화 조각은 원래 해상도로 다시 추출한다.
검출 라벨과 모델 입력 해상도는 바뀌지 않지만 골격 끝점·접선·코너 형상은 달라질 수 있다.
검사 경계는 계속 원본 관측 마스크 픽셀을 사용한다.

검사 간격을 키우면 구간별 검사 횟수가 줄어든다. 구간 사이의 미검사 거리를 보완하기 위해
기존의 `min_clearance + 픽셀 반대각선 + 검사 간격/2` 여유 거리 계산을 유지한다.
따라서 간격을 늘리면 좁은 경로가 더 많이 제외될 수도 있다. 이 간격은 이제 후보 간격과
독립이다. 이전에 `centerline_sample_spacing_m < 0.01`을 사용했다면 과거와 동일한 검사 간격을
유지하려면 `centerline_clearance_check_spacing_m`을 해당 후보 간격의 절반으로 지정한다.

예를 들어 다음은 골격 해상도만 바꾸는 비교 실행이다. 다른 항목도 한 번에 하나씩 바꾸어
`lane_geometry_nanoseconds`, 중심선 유효율·길이·프레임 사이 흔들림을 비교한다.

```bash
ros2 launch line_detactor line_detactor.launch.py \
  connection_skeleton_downsample_factor:=2
```

`vehicle_bringup/auto_drive.launch.py`에서는 `line_detactor_params_file`로 전달하는 YAML의
`line_detactor.ros__parameters` 아래에 같은 키를 넣는다. 다음은 세 항목을 조합한 시험 설정이다.
기존 YAML에 아래 값을 반영하면 기존 모델 정밀도 등 다른 설정을 유지할 수 있다.

```yaml
line_detactor:
  ros__parameters:
    connection_skeleton_downsample_factor: 2
    centerline_output_spacing_m: 0.02
    centerline_clearance_check_spacing_m: 0.01
```

### 중앙 경로 생성 동작

`centerline_enabled: true`가 기본이다. `connection_enabled: true`에서 동작한다.
`connection_enabled: false`는 기존 raw 프리뷰 모드이며 중앙 경로/결과 메시지를 생성하지 않는다.
`centerline_enabled: false`로 중앙 경로만 끌 수 있다.

중앙 경로 생성은 다음 순서다.

1. 작은 성분 제거 후 **관측 라벨 1/2만** 사용한다. 정지선과 바깥 보간 라벨 3/4는 제외한다.
2. 각 연결 조각에서 순서가 있는 골격 경로를 추출하고 실제 거리 좌표로 변환한다.
   조각의 방향은 아래쪽 영상 중앙(차량 근처)에서 먼 쪽으로 정한다. 폐곡선은 후보에서 제외한다.
3. 대응되는 반대편 차선이 폭·방향 조건을 만족하면 중점, 아니면 안쪽 법선 방향으로
   도로 폭의 절반(기본 32.5cm)을 이동한 후보를 만든다. 코너에서는 아래 설명처럼
   관측이 충분한 바깥 차선의 오프셋 경로와 진행 방향을 더 높은 비중으로 반영한다.
4. 차량 근처부터 거리·진행 방향·관측 경계 여유 거리가 맞는 후보들을 연결한다.
   짧은 공백은 허용하며 큰 공백은 경로를 끝낸다. 원본 좌우 차선을 하나로 강제 병합하지 않는다.
5. 국소 Gaussian 평활화 후, 경로상 거리 기준 2차 다항식 평활화를 추가한다.
   직선에 가까운 곳은 강하게, 코너는 약하게 적용한다. 각 단계에서 이동량을 제한하고
   시작/끝점을 고정한다. 여유 거리/진행 방향 조건을 위반하면 강도를 낮춰 재시도한다.
6. 프리뷰와 `result_image`에 **노란색(BGR 0,255,255)**으로 표시한다.
   노란 선은 `overlay_alpha`와 무관하게 표시되며, 정지선과 겹쳐도 정지선 마스크는 보존한다.

검토한 Python 미리보기의 계산 방식을 C++/OpenCV로 이식했다. ROS에서는 스크린샷 색 추출을
하지 않고 실제 라벨을 사용하며, 경계까지의 거리도 골격 대신 관측 마스크 픽셀로 확인한다.
골격 추출/거리 샘플링과 경계 처리 차이 때문에 미리보기와 픽셀 단위로 같은 출력은 보장하지 않는다.
추가 Python/SciPy 런타임 의존성은 없다.

튜닝용 예제 YAML을 복사하고 실행한다. BEV 노드가 먼저 이미지를 발행해야 한다.

```bash
cp src/line_detactor/config/centerline_preview.yaml /tmp/centerline_preview.yaml
ros2 launch line_detactor line_detactor.launch.py \
  params_file:=/tmp/centerline_preview.yaml
```

노드 이름은 YAML의 `line_detactor.ros__parameters`와 일치해야 한다. 파일에 없는 값은
노드 기본값을 사용한다. **YAML은 시작 시 읽으므로 수정 후 노드를 재시작한다.**
`ros2 param set`으로 이 계산 설정을 실시간 갱신하는 기능은 구현하지 않았다.
명시적 launch 인자는 YAML보다 우선한다.

```bash
ros2 launch line_detactor line_detactor.launch.py \
  params_file:=/tmp/centerline_preview.yaml \
  centerline_smoothing_strength:=0.7 \
  centerline_smoothing_window_m:=0.65
```

아래 표의 이름에는 모두 `centerline_` 접두사를 붙인다.

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `enabled` | true | 중앙 경로 계산·표시 |
| `lane_width_m` | 0.65 | 차선 사이 도로 폭 |
| `bev_width_m`, `bev_height_m` | 1.2, 3.0 | **패딩 제외 원본** BEV 실제 가로/전방 범위 |
| `sample_spacing_m` | 0.015 | 차선 골격 후보의 거리 간격, 0.005~0.10m |
| `min_fragment_length_m` | 0.08 | 후보로 사용할 조각 최소 길이 |
| `tangent_window_m` | 0.06 | 차선 진행 방향을 계산하는 거리 범위 |
| `width_tolerance_m` | 0.12 | 양쪽 대응 시 도로 폭 허용 오차 |
| `pair_along_tolerance_m` | 0.055 | 양쪽 대응 시 진행 방향 위치 오차 |
| `pair_heading_tolerance_deg` | 40.0 | 양쪽 대응 시 접선 방향 차이 |
| `max_gap_m` | 0.12 | 후보 사이 연결 거리 상한 |
| `max_start_distance_m` | 0.65 | 영상 아래 중앙에서 첫 경로 후보까지 허용 거리 |
| `min_clearance_m` | 0.16 | 관측 차선 마스크와 경로 사이 최소 여유 거리 |
| `outside_margin_m` | 0.12 | 좌우 원본 영상 바깥의 경로 허용 범위; 결과 패딩 크기 이내로 제한 |
| `max_samples` | 2000 | 후보 샘플 예산; 초과 프레임은 빈 경로와 경고 출력 |
| `line_width_px` | 2 | 노란 선 두께, 1~10px |
| `corner_outer_enabled` | true | 코너에서 바깥 차선 기준 경로 우선 반영 |
| `corner_outer_weight` | 0.85 | 바깥 차선 기준 경로의 최대 혼합 비중, 0~1 |
| `corner_outward_offset_m` | 0.05 | 코너 및 진입 구간에서 추가로 바깥쪽으로 이동할 목표 거리(m), 0이면 이동 끔 |
| `corner_entry_full_offset_distance_m` | 0.0 | 거리 감쇠 없이 코너 신뢰도를 유지할 경계 거리(m). 0은 기존 동작, 양수는 entry_distance_m 미만 |
| `corner_entry_distance_m` | 0.40 | 관측 코너 신뢰도를 차량 쪽으로 확장할 경계 호 길이(m), 0이면 진입 확장 끔, 범위 0~5 |
| `corner_outer_window_m` | 0.60 | 회전 방향과 관측 길이를 평가하는 거리 범위 |
| `corner_outer_tangent_window_m` | 0.15 | 바깥 차선 기준 접선 추정 범위 |
| `corner_outer_min_length_m` | 0.30 | 기준으로 삼을 최소 연속 관측 길이; 평가 범위보다 작아야 함 |
| `corner_outer_min_turn_deg` | 8.0 | 바깥 차선 가중치를 올리기 시작하는 누적 회전각 |
| `corner_outer_full_turn_deg` | 25.0 | 충분한 관측에서 최대 가중치에 도달하는 누적 회전각 |
| `smoothing_enabled` | true | 최종 경로의 두 단계 평활화 |
| `smoothing_sigma_m` | 0.04 | 첫 단계 국소 평활화 표준편차 |
| `smoothing_window_m` | 0.65 | 두 번째 단계 국소 2차 다항식 계산 범위 |
| `smoothing_max_shift_m` | 0.03 | **각 평활화 단계**의 최대 이동량; 두 단계 누적은 최대 6cm |
| `smoothing_strength` | 1.0 | 최종 평활화 강도, 0~1; 0이면 최종 평활화 생략 |
| `straight_turn_deg` | 12.0 | 이 각도 이하에서는 직선 평활화 강도 유지 |
| `corner_turn_deg` | 35.0 | 이 각도 이상에서는 두 번째 평활화 강도 억제 |
| `turn_window_m` | 0.30 | 직선/코너 판정 방향 변화 측정 범위 |

출렁임이 남으면 `smoothing_window_m`을 0.65→0.8 정도로 늘려 보고,
코너가 과하게 완화되면 `smoothing_strength`를 1.0→0.7로 줄이거나 창 크기를 줄인다.
최종 평활화를 꺼도 골격 방향 추정과 후보 전환에 사용하는 작은 필터는 유지된다.
거리 계산에는 `bev_processor`의 실제 범위를 사용해야 한다. 기본값은 전방 0~3m,
좌우 대칭 1.2m에 대응한다. 아래 중앙을 차량 근처의 시작 기준으로 사용하므로
비대칭/전방 크롭 영상은 원점 변환을 추가하기 전 이 가정을 그대로 적용하면 안 된다.
패딩은 관측 영역이나 도로 폭에 포함되지 않는다.

`LaneResult`의 중앙 경로 필드:

- `centerline_points`: 가까운 곳부터 정렬된 확장 영상 픽셀 좌표, z=0. 미터/TF 좌표가 아님.
- `centerline_support`: 점 수와 같은 길이. 1=한쪽 오프셋, 2=양쪽 중점,
  3=6.5cm보다 긴 짧은 연결 구간, 4=바깥 차선 기준 혼합(`CENTER_OUTER`).
  국소 생성 근거이며 평활화 후 정확도 확률은 아님. 4에도 양쪽/한쪽 관측 모두 가능하다.
- `centerline_valid`: 두 점 이상의 기하 경로가 존재함. 주행 가능 판정은 아님.
- `centerline_sample_limit_reached`: 계산 예산 초과 여부.
- `centerline_bev_width_m`, `centerline_bev_height_m`: 계산에 사용한 원본의 실제 범위.

생성 불가/비활성화 시 점과 support 배열은 비고 valid는 false다.
관측 경계가 화면 밖에 있으면 충돌 여부를 확인할 수 없다. 경계 여유 거리에는
픽셀·샘플 간격 여유분을 추가하지만 차량 외곽/장애물/조향 곡률 제한이나 프레임 간
추적을 포함하지 않는다. `labels=0`은 주행 가능 공간이 아니다.
이번 ROS 이식은 사용자 요청에 따라 빌드·테스트·젯슨 실행을 수행하지 않았다.

### 코너에서 바깥 차선 형상 우선 반영

코너 정점 부근에서 양쪽 경계의 중점 후보가 달라져, 경로 선택의 기준이 바뀌는 문제를
줄이기 위해 바깥 경계의 오프셋 경로를 우선 반영한다. **좌회전은 오른쪽(빨강),
우회전은 왼쪽(파랑)** 차선이 바깥이다. 단순히 가장 긴 조각을 바깥 차선으로 보지 않고,
차선을 따라 측정한 부호 있는 회전각으로 판별한다.

바깥 차선의 곡률 숫자를 중앙선에 그대로 복사하면 반경이 맞지 않으므로,
그 경계를 안쪽으로 도로 폭의 절반만큼 이동한 **경로 위치와 진행 방향**을 혼합한다.
기존 양쪽 중점 후보와 한쪽 오프셋 후보 모두 같은 바깥 기준으로 가까워지도록 해
후보를 선택하는 쪽이 바뀌어도 경로 형태의 차이가 줄도록 구성했다.

```text
최종 후보 = (1 - 가중치) × 기존 후보 + 가중치 × 바깥 경계의 안쪽 오프셋
가중치 상한 = centerline_corner_outer_weight (기본 0.85)
```

실제 가중치는 회전각, 국소 관측 길이, 한 방향으로 도는 일관성, 끝점까지의 여유에
따라 0~상한 사이에서 서서히 변한다. 양쪽 회전 증거가 충돌하면 두 증거의 차이만큼만
가중치를 적용한다. 30cm 미만의 짧은 조각, 직선, 바깥 차선 부재, 너무 작은 곡률 반경으로
뒤집힌 오프셋, 기존 경계 여유 거리 조건 위반에서는 해당 기준의 영향력을 낮추거나 제외한다.
짧은 조각은 경로의 다른 후보로는 계속 사용할 수 있다.

이 변경은 **현재 프레임의 관측을 이용한 공간적 안정화**다. 이전 프레임 경로를 고정하거나
시간 평균하지 않으며, 실제 검출 위치가 움직이는 경우의 흔들림까지 제거한다고 보장하지 않는다.
주행 중 과거 경로를 재사용하려면 차량 이동 보정이 별도로 필요하다.

```yaml
line_detactor:
  ros__parameters:
    centerline_corner_outer_enabled: true
    centerline_corner_outer_weight: 0.85
    centerline_corner_outer_window_m: 0.60
    centerline_corner_outer_tangent_window_m: 0.15
    centerline_corner_outer_min_length_m: 0.30
    centerline_corner_outer_min_turn_deg: 8.0
    centerline_corner_outer_full_turn_deg: 25.0
```

바깥 차선이 안정적으로 보이면 `corner_outer_weight`를 0.85에서 0.95로 올려 비중을
높일 수 있다. 1.0도 관측 신뢰도에 따라 실제 가중치가 낮아진다.
기존 방식과 비교하려면 `corner_outer_enabled: false`로 실행한다.
두 설정 파일(`line_detactor.yaml`, `centerline_preview.yaml`)과 launch 인자에 같은 값을
연결했다. YAML 변경 후 재시작한다. 이번 코너 수정도 요청에 따라 빌드·테스트는 수행하지 않았다.

## 코너 진입 전 바깥쪽 경로 오프셋과 결과 전용 프리뷰

`centerline_corner_outward_offset_m`은 기존 중앙 경로 후보를 관측된 바깥 차선
쪽으로 이동하는 추가 거리다. 기본값은 0.05m이며, `0.0`으로 끌 수 있다.
`centerline_corner_outer_weight`는 기존 형상 혼합 비중이고, 새 오프셋은 거리다.
두 값은 별개이며 `centerline_corner_outer_enabled: true`가 필요하다.

좌회전에서는 오른쪽 바깥 경계, 우회전에서는 왼쪽 바깥 경계를 향해 이동한다.
양쪽 차선에서 만드는 후보 모두 동일한 바깥 경계의 법선과 신뢰도를 사용한다.
기존 코너 판별의 회전각·관측 길이·끝점 신뢰도에 따라 이동량이 0~설정값으로
변한다. 앞에서 코너가 관측되면 아직 직선인 진입 구간부터 바깥쪽으로 이동한다.

`0822ver3`는 관측 경로 전체의 회전으로 코너 모드를 정하고, 진입부를 포함한
경계 전체의 법선 오프셋을 변경했다. 여기서는 같은 진입 전 이동 의도를 반영하되,
`centerline_corner_entry_distance_m`(기본 0.40m) 범위 안의 앞쪽 코너 신뢰도를
차량 쪽으로 확장한다. 거리는 같은 연속 관측 경계를 따라 측정하며, 멀수록
smoothstep으로 이동량을 줄여 직선에서 서서히 바깥쪽으로 붙게 한다.
`centerline_corner_entry_full_offset_distance_m`(기본 0.0m)을 양수로 설정하면
그 거리 안에서는 거리 감쇠 없이 관측 코너 신뢰도를 유지한다. 진입 거리와 이 거리
사이에서만 smoothstep으로 보정량이 증가한다. 0은 기존 동작이고, 양수는 반드시
`centerline_corner_entry_distance_m`보다 작아야 한다.

코너 진입 전에 미리 이동하는 첫 비교 설정은 다음과 같다. 외부 YAML을 수정하지
않아도 기존 `auto_drive` 실행 명령 끝에 넣을 수 있다.

```bash
centerline_corner_outward_offset_m:=0.02 \
centerline_corner_entry_distance_m:=1.2 \
centerline_corner_entry_full_offset_distance_m:=0.6
```

관측 코너 증거로부터 경계 호 길이 1.2m 전에서 이동을 시작해, 0.6m 전부터 거리
감쇠가 없는 보정 강도를 유지한다. 이는 기하학적 코너 시작 위치의 정확한 예측이나
2cm 실차 이동을 보장하지 않는다. 코너 신뢰도·상충 증거·여유 거리 판정과 최종
평활화가 실제 경로 이동량에 계속 영향을 준다. 끝까지 차선이 연결되어 관측되어야
하며, 먼 코너가 보이지 않거나 조각 사이가 끊기면 확장되지 않는다.

`auto_control`의 `curvature_lookahead_minimum_x_m`,
`curvature_lookahead_maximum_x_m`, `curvature_percentile`은 속도 결정용 대표 곡률을
구하는 범위/분위수다. 코너 오프셋 범위나 코너 내 차량 위치 추적을 설정하지 않는다.
BEV 전체로 범위를 늘려도 실제 생성·채택된 경로만 사용한다. 90분위수는 좁은 코너
구간이 긴 직선에 섞이면 작은 값이 될 수 있어, 범위 확대가 항상 더 이른 감속을
보장하지 않는다. 이번 변경은 이 제어 파라미터를 바꾸지 않는다.

이 진입 유지 구간 변경은 사용자 요청에 따라 빌드·테스트를 실행하지 않았다.
기존 코너 판별 창도 주변을 보기 때문에 이 값은 기하학적 코너 시작점 기준의
정확한 거리보다 **기존 국소 코너 적용 범위를 앞당기는 추가 거리**를 뜻한다.

이동 방향은 현재 위치의 경계 법선을 사용한다. 바깥 차선의 곡률 혼합은 기존
국소 신뢰도를 유지한다. 확장은 앞쪽 관측만 사용하며 코너 뒤쪽으로 추가 연장하거나
이전 프레임의 코너 상태를 유지하지 않는다. 앞쪽 코너가 보이지 않거나 같은 경계가
중간에 끊기면 그 너머의 코너 신뢰도를 가져오지 않는다. 상충하는 좌우 증거는
이동량을 줄인다.

더 일찍 이동하려면 `centerline_corner_entry_distance_m`을 키우고, 바깥쪽으로
더 이동하려면 `centerline_corner_outward_offset_m`을 키운다.
진입 거리 `0.0`은 기존 국소 코너 오프셋만 적용하며, 오프셋 `0.0`은 이동 전체를 끈다.

이동 구간이 관측 차선과 `centerline_min_clearance_m` 간격이나 결과 영상 범위를
침범하면 이동량을 줄인다. 후보 평활화·연결 후에도 기존 전체 경로 검사를 수행한다.
설정값은 목표 이동 거리이지 최소 이동량이나 바깥 차선과의 고정 간격 보장이 아니다.
간격 검사는 경로점 기준이며 차량 폭·후륜 궤적까지 검사하는 기능은 아니다.
최종 이동한 경로를 노란색으로 표시하고 그대로 `auto_control`에 전달한다.

```yaml
line_detactor:
  ros__parameters:
    centerline_corner_outer_enabled: true
    centerline_corner_outward_offset_m: 0.05
    centerline_corner_entry_distance_m: 0.40
    preview_enabled: true
    preview_result_only_enabled: true
```

`preview_result_only_enabled: true`는 검은 배경에 좌우 차선·정지선·노란 경로와
성능 배너만 표시한다. 원본 BEV 패딩·알파 합성을 생략한다. `false`는 원본 BEV 위에
결과를 겹쳐 표시한다. `connection_enabled: false`인 raw 모드에서는 이 옵션을
적용하지 않고 기존 raw 추론 프리뷰를 표시한다.
연결 모드에서는 사용되지 않던 GPU 원본 오버레이 생성과 해당 D2H 복사도 생략한다.
프리뷰와 `/line_detactor/result_image` 구독자가 모두 없으면 BGR 결과 영상과 표시용
정지선/중앙선 마스크 및 표시 전용 화면 밖 차선 연결도 만들지 않는다. 제어 중앙선은
원래부터 이 표시용 연결선을 입력으로 쓰지 않으므로 주행 계산 결과는 바뀌지 않는다.
모델 추론 결과 자체도 바뀌지 않는다.

자동주행에서도 YAML 또는 명시적 launch 인자로 조절할 수 있다.

```bash
ros2 launch vehicle_bringup auto_drive.launch.py \
  line_detactor_params_file:=/absolute/path/line_detactor_test.yaml \
  centerline_corner_outward_offset_m:=0.05 \
  centerline_corner_entry_distance_m:=0.40 \
  preview_enabled:=true \
  preview_result_only_enabled:=true
```

기존 BEV/auto_control/CAN 인자를 함께 전달할 수 있다. 명시한 launch 값이
YAML보다 우선하며 변경 후 재실행한다.

## 결과 토픽과 좌표

- `/line_detactor/result`: `line_detactor/msg/LaneResult`
- `/line_detactor/result_image`: `sensor_msgs/msg/Image`, `bgr8`, 차선·정지선·노란 중앙 경로를 그린 검은 배경 영상

제어용 `LaneResult`에는 BGR 영상, 좌우 라벨, 정지선/중앙선 마스크와 좌우 곡선
진단 배열을 넣지 않는다. 타임스탬프·성능 시간·상태·원본 크기/패딩·중앙 경로만
전송해 DDS 직렬화와 복사를 줄인다. `stop_line_present`는 정지선 픽셀이 1개 이상인지를 나타내며
확정된 정지 명령은 아니다. 정지선만 있으면 state=NONE이어도 stop_line_present=true다.
시각 결과가 필요하면 별도 `result_image` 토픽을 구독한다. 이 토픽은 구독자나 로컬
프리뷰가 있을 때만 합성하며, 아무 차선/정지선/경로도 없으면 검은 영상을 발행한다.
원본 timestamp/frame_id를 유지하며 QoS는 best effort, volatile, KeepLast(1)이다.

`processing_mode=BORDER_ONLY`이다. `state`는 **남아 있는 마스크의 좌우 존재 여부**
(`0=NONE`, `1=LEFT_ONLY`, `2=RIGHT_ONLY`, `3=BOTH`)이며 한 개의 연속 차선이
확보되었다는 의미가 아니다. 계획 입력으로 해석할 때 이 차이를 반영해야 한다.

점은 확장 영상의 픽셀 좌표(x 오른쪽, y 아래쪽, z=0)이며 미터/TF 위치가 아니다.

```text
u_source = u_result - padding_left
v_source = v_result
```

미터 변환에는 실제 `bev_processor` 배율/원점을 사용한다. 검은 여백/라벨 0은
자유 공간을 뜻하지 않는다. 입력 중단/프레임 처리 실패 시 새 메시지가 없으므로
소비자는 timestamp로 최신성을 확인해야 한다.
`vehicle_bringup auto_drive.launch.py`는 이 결과의 중앙선 점열을 `auto_control`에 연결한다.
`line_detactor_params_file`로 차선·중앙선 YAML을 지정하며, BEV 배율은 `bev_params_file`에서
검출기와 제어기에 함께 적용한다. 정지선 검출은 이 주행 제어의 정지 명령으로 사용하지 않는다.

## 주행 로그 반영: 경로 생성과 정지선 파라미터

연결요소 처리는 `connection_ccl_serial_enabled: true`일 때 작은 입력에 OpenCV의
16비트 단일 스레드 CCL을 사용한 뒤 기존 32비트 라벨 형식으로 변환한다. 현재
120×300 입력은 모든 픽셀이 별도 조각이어도 16비트 범위 안에 있다. 입력 픽셀 수가
65,535 이상이면 기존 32비트 경로로 돌아간다. 8방향 연결·최소 면적 기준은 유지하고
스레드별 임시 버퍼를 재사용한다. 같은 프로세스의 카메라/BEV에 영향을 주는
`cv::setNumThreads()` 전역 설정은 바꾸지 않는다. `false`로 기존 CCL 경로와 비교할 수 있다.

중심 경로 그래프에서는 기존 방향·비용식을 유지하고, 비용을 개선할 수 있는 간선만
여유 거리 검사한다. `graph_cost_rejections_count`는 비용 때문에 거리 검사를 생략한
횟수다. 최종 smoothing이 비활성화/강도 0/7점 미만이면 이미 검증한 경로를 재검증하지
않는다. 이때 `smoothing_ms`, `path_check_after_ms`는 0이다. 실제 경로를 평활화하는
경우에는 전후 검증을 유지한다. 거리 지도 전환은 이 변경의 재측정 후 판단하는 다음
단계이며, 이번에는 기존 소수점 좌표·패딩 영역 거리 판정을 그대로 사용한다.

정지선 관련 설정은 `config/line_detactor.yaml`의 `stop_line_*` 항목에 노출되어 있다.
기존 테스트 파일 `line_detactor_test.yaml`의 `line_detactor.ros__parameters` 안에도
같은 이름으로 넣을 수 있다. `auto_drive.launch.py`와 독립 검출기 launch 양쪽 모두
동일한 이름의 명령행 재정의를 지원한다. 생략 시 선택한 YAML 값을 유지한다.
모든 항목은 노드 시작 시 읽으며 기본값은 기존 동작을 유지한다.

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `stop_line_mask_threshold` | -1.0 | -1은 기존 `mask_threshold` 상속, 0~1은 정지선 채널만 별도 sigmoid 임계값. GPU 라벨과 raw 미리보기 모두 적용 |
| `stop_line_min_present_pixels` | 1 | `stop_line_present` 상태의 최소 픽셀 수. 마스크 표시/거리 추정과 독립 |
| `stop_line_distance_enabled` | true | 거리 추정 활성화. false면 거리 NaN, 초록 마스크 표시는 유지 |
| `stop_line_min_pixels` | 6 | 거리 추정에 필요한 정지선 마스크 픽셀 수 |
| `stop_line_max_fit_samples` | 256 | Welsch 직선 피팅 표본 수 상한 |
| `stop_line_support_trim_quantile` | 0.02 | 직선 지지 구간의 양끝 제외 비율. 범위 `[0, 0.5)` |
| `stop_line_support_margin_px` | 5.0 | 직선 지지 구간 양끝의 추가 허용 폭, 픽셀 |
| `stop_line_near_edge_quantile` | 0.05 | 차량 쪽 정지선 가장자리 추정에 쓰는 분위수. 범위 `[0, 0.5)` |
| `stop_line_min_crossing_alignment` | 0.5 | 경로와 정지선 법선의 내적 절댓값 최소치. 1이면 직각 교차만 허용 |
| `stop_line_fit_distance_tolerance_px` | 0.01 | 피팅 거리 수렴 기준, 픽셀 |
| `stop_line_fit_angle_tolerance_rad` | 0.01 | 피팅 각도 수렴 기준, rad |

예를 들어 기존 실행 명령 끝에 `stop_line_mask_threshold:=0.6`을 추가하면 좌우 차선
임계값은 유지하면서 정지선 임계값만 높인다. 정지선 거리는 앞차축부터 중앙 경로를
따라 차량 쪽 정지선 가장자리까지 측정한다. 이 설정은 검출/거리 추정 기준이며
자동주행 제어기의 정지 위치·제동 동작을 새로 추가하는 설정은 아니다.

이번 변경은 사용자 요청으로 빌드·테스트를 실행하지 않았다. 차량에서는 업데이트 후
`line_detactor`, `vehicle_bringup`을 재빌드하고 기존 옵션으로 CSV를 다시 수집하여
P99/최대 지연, 거리 검사 횟수와 경로 출력 상태를 비교한다.

## 후처리 병목 측정 파일

`profiling_enabled: true`로 실행하면 프레임별 연산 시간과 연산량을 CSV 한 파일에
저장한다. 알고리즘, 엔진 정밀도, 미리보기, 제어 활성화 여부를 바꾸지 않는다.
기본값은 `false`이고, 실행 중 파라미터 변경 대신 노드를 다시 시작해서 적용한다.

차량 저장소에서 변경 사항을 받은 후 기존 빌드 설정을 유지하여 재빌드한다.
`--cmake-force-configure`는 빌드 출처를 CSV 메타데이터에 갱신한다.

```bash
cd /home/autopilot03/Desktop/0906ML/f1tenth_project_repo
colcon build --packages-select line_detactor vehicle_bringup --cmake-force-configure
source install/setup.bash
```

사용 중인 명령에 마지막 두 옵션만 추가하면 된다. 외부 테스트 YAML을 계속 사용하며,
명령행 옵션을 생략하면 해당 YAML의 `profiling_enabled`, `profiling_directory`를 사용한다.

```bash
ros2 launch vehicle_bringup auto_drive.launch.py \
  auto_control_params_file:=/home/autopilot03/Desktop/0906ML/f1tenth_project_repo/auto_control_test.yaml \
  bev_params_file:=/home/autopilot03/Desktop/0906ML/f1tenth_project_repo/bev_config_test.yaml \
  line_detactor_params_file:=/home/autopilot03/Desktop/0906ML/f1tenth_project_repo/line_detactor_test.yaml \
  auto_enabled:=false \
  input_mode:=slcan \
  slcan_channel:=/dev/ttyACM0 \
  slcan_bitrate:=500000 \
  can_controller_id:=112 \
  centerline_corner_outward_offset_m:=0.02 \
  preview_enabled:=true \
  preview_result_only_enabled:=true \
  profiling_enabled:=true \
  profiling_directory:=/home/autopilot03/Desktop/0906ML/f1tenth_project_repo/performance_logs
```

시작 로그의 `Lane processing profile: .../lane_profile_<시각>_<PID>/profile.csv`가 실제
파일 위치다. 실행마다 새 디렉터리를 생성하며 기존 파일을 덮어쓰지 않는다.
직선·코너·차선이 끊기거나 조각이 많이 나오는 장면을 포함해 30~60초 정도 측정한 뒤
`Ctrl+C`로 정상 종료하고 **그 실행의 `profile.csv` 하나를 전달**하면 된다.
시작 직후 프레임도 저장하므로 분석할 때 초기 구간을 구분할 수 있다.
차량에만 있는 테스트 YAML 원본을 수정하거나 저장소에 복사할 필요가 없다.

파일 앞부분의 `#` 주석에는 YAML과 launch 재정의가 적용된 **검출기의 실제 파라미터**, 모델·엔진
경로, 시스템/장치 정보, 컴파일러, 빌드 구성과 configure 시점 Git 출처가 들어간다.
BEV는 검출기에 전달된 크기·물리 범위만 포함하며, 카메라/BEV 처리 및 제어 노드 전체를
프로파일링하지는 않는다. 테스트 YAML 경로가 같아도 launch에서 바꾼 코너 오프셋 등이
실제 값으로 기록된다. CSV를 읽을 때 `#`로 시작하는 줄을 제외한다.

| 열 | 의미 |
|---|---|
| `generation`, `elapsed_ms`, `source_stamp_ns`, `received_stamp_ns` | 입력 세대, 측정 시작 후 경과 시간, 원본/검출기 수신 ROS 시각 |
| `queue_wait_ms`, `total_ms` | 최신 입력이 작업자를 기다린 시간, 작업자 시작부터 결과 발행 함수 종료까지의 시간 |
| `gpu_*_ms`, `backend_wall_ms` | CUDA 이벤트로 측정한 전처리·추론·라벨 전송·기타 후처리, CPU에서 본 백엔드 호출 전체 시간 |
| `correction_ms` | 화면 `post avg`에 집계하는 GPU 라벨 생성/전송 + CPU 후처리 시간 |
| `connector_ms`, `components_ms`, `skeleton_ms`, `thinning_ms`, `skeleton_graph_ms` | 차선 연결 전체, 연결요소 정리, 골격 추출 전체와 내부 thinning·픽셀 그래프 처리 |
| `border_endpoints_ms`, `bridge_candidates_ms`, `bridge_render_ms`, `lane_render_ms` | 경계 끝점, 연결 곡선 후보, 후보 정렬/충돌 확인/그리기, 차선 이미지·상태 생성 |
| `centerline_ms`, `boundary_index_ms`, `fragment_prepare_ms`, `outer_reference_ms`, `pairing_index_ms` | 중심 경로 전체, 거리 검색 자료구조, 관측 차선 표본화, 코너 기준, 좌우 대응 검색 준비 |
| `candidates_ms`, `path_search_ms`, `path_output_ms` | 좌우 대응·코너 보정·후보 검증, 경로 그래프 탐색, 최종 표본화·검증·마스크 생성 |
| `path_check_before_ms`, `smoothing_ms`, `path_check_after_ms` | 기존 최종 smoothing 전/후 검사와 smoothing 호출 |
| `stop_mask_render_ms`, `stop_distance_ms`, `centerline_render_ms`, `publish_ms` | 정지선 마스크 확인/표시, 정지선 거리, 노란 경로 합성, ROS 결과/이미지 발행 함수 |
| `*_count` | 조각·골격 픽셀·표본·후보·탐색 이웃·거리 검사·최종 경로점 수 등 프레임별 연산량 |
| `skipped_total`, `logger_dropped_total` | 처리하지 못해 건너뛴 입력 누계, 파일 기록 큐에서 누락된 행 누계 |
| `success`, `render_result`, `sample_limit_reached` | 예외 없이 처리 완료, 표시용 결과 생성 여부, 중심 경로 표본 예산 초과 |

부모 시간은 자식 시간을 **포함**한다. 예를 들어 `connector_ms`에 `skeleton_ms`가,
`skeleton_ms`에 `thinning_ms`가 포함되므로 모든 열을 더하면 중복 집계된다.
`path_output_ms`에는 최종 경로 검사와, 활성화된 경우 smoothing 및 후속 검사가 포함된다.
`candidates_ms`와 `path_search_ms`에는 각 단계의 거리 검사가 포함된다.
거리 검사 내부는 픽셀마다 시계를 읽지 않고 횟수만 센다. 계측 자체의 비용은 남는다.
`success=1`이어도 검출 차선/경로가 없을 수 있으며 `output_points_count`를 함께 본다.
중간 반환·예외로 도달하지 않은 단계는 0이고, `success=0` 행은 완료된 프레임과 분리한다.
`total_ms`에는 로그 큐 삽입, GUI 스레드 전달·표시, downstream 제어 처리는 포함되지 않는다.
`logger_dropped_total`은 파일에 쓸 때 관측한 누계라 해당 입력 세대와 정확히 동기화되지는 않는다.

파일 포맷 변환과 쓰기는 별도 스레드가 약 1초마다 수행한다. 큐는 최대 4096행이며,
큐가 가득 차거나 짧은 큐 잠금을 즉시 얻지 못하면 입력 처리를 대기시키지 않고 로그 행만
누락시킨다. 쓰기 오류와 누락은 터미널에도 표시한다. 정상 종료는 대기 중인 행을 저장하고
마지막에 `# complete=true`를 남긴다. 강제 종료·전원 차단 시 마지막 버퍼는 유실될 수 있다.
기본 저장 경로 `/tmp/line_detactor_profiles` 대신 위처럼 영구 저장 경로를 지정할 수 있다.

## 속도와 기타

프리뷰 `infer avg`는 워밍업 이후 순수 TensorRT 추론의 누적 평균이고 FPS는 그 역수다.
`post avg`는 GPU 라벨 생성/D2H와 CPU 차선·중앙선·정지선 기하 처리를 합친 누적
평균이다. 프리뷰 창 표시와 ROS 결과 전송은 포함하지 않는다. `preview FPS`는 전용 GUI
스레드의 실제 표시 빈도다. `control avg`는 검출기가 BEV 영상을 수신한 시점부터
auto_control이 최종 서보/듀티를 결정한 시점까지의 누적 평균이다. 검출기 대기와
`LaneResult` 전송은 포함하고, 검출기 입력 전의 카메라 영상 전송과 액추에이터 명령
발행·VESC 전송은 제외한다. auto_control이 실행되지 않거나 유효한
중앙 경로가 아직 없으면 `control --`로 표시한다.

`stop` 거리는 `front_axle_bev` 원점인 앞차축 중심에서 검출된 정지선의 차량 쪽
경계까지 중앙 경로를 따라 측정한 값이다. 정지선에 Welsch 직선을 맞추고 중앙 경로와의
교점을 찾으므로 비스듬한 정지선과 곡선 경로를 처리한다. 유효한 교점이 없으면 `stop -- m`
로 표시한다.

로그의 `input`, `processed`, `preview`는 각각 수신·추론/후처리 완료·화면 표시 FPS이며
단계별 평균/최대 ms도 표시한다.

추론 작업자는 새 BEV를 받으면 즉시 처리하며 프리뷰 FPS를 기다리지 않는다.
차선·중앙선 결과를 먼저 발행한 뒤 표시용 최신 결과를 GUI 스레드에 전달한다.
영상 합성, `imshow`, `waitKey`는 GUI 스레드에서 수행한다. 프리뷰를 끄더라도
추론 주기는 동일하다. 프리뷰를 닫으면 기존처럼 검출 노드가 종료된다.

입력과 프리뷰 모두 최신 한 프레임을 보관한다. 처리가 입력보다 느리면 중간 영상은
건너뛰고 `skipped`에 집계한다. 모든 카메라 프레임의 처리나 고정 FPS를 보장하지 않으며,
ROS 전송과 GPU/CPU 처리 지연은 남는다. 오래된 프레임을 무제한 적재하지 않는다.

`LaneResult` carries the engine precision, detector input/result-ready stamps,
queue time, H2D/preprocess, pure inference, label export, backend postprocess,
lane geometry, result-message construction, and total detector compute time.
These fields exclude GUI work and allow `auto_drive` performance mode to separate
compute time from ROS transport and source-capture delay.

연결 단계에서 추출한 connected component 골격은 중앙선 생성에서 그대로 재사용한다.
좌우 표본 대응은 전체 조합을 반복 탐색하지 않고 미터 단위 공간 인덱스로 주변 반대편
표본만 조회한다. 제어 메시지에는 중앙 경로와 검증/측정에 필요한 스칼라만 실린다.

logits 전체는 CPU에 복사하지 않고 120×300 mono8 두 장(좌우 라벨 + 정지선 마스크)을
한 번에 전달한다. 36KB에서 72KB로 늘며 CUDA stream, pinned memory, GPU 전처리·threshold·
오버레이 경로는 유지한다. FP16/INT8로 정밀도를 바꾸지 않고 기존 TensorRT FP32를 유지한다.
ONNX 기본 경로는 패키지의 `fast_scnn_stop_line_120x300_batch_1.onnx`다. 최초 실행 시 장치별
FP32 TensorRT 엔진을 생성하고 ONNX 옆 `*.trt<TensorRT-major>.fp32.engine`를 재사용한다.
`Q`/`ESC`/창 닫기로 종료한다. 자동주행·조향 명령은 발행하지 않는다.

Windows에서 ONNX checker/ONNX Runtime CPU 수치 비교를 수행했다. 변경 시점의 best(epoch10)
스냅샷으로 검증 입력10개(검증 이미지8장+합성2개)에서 세 채널 마스크가 모두 일치했고,
로짓 최대 절대오차는 약0.000290이었다. 해당 ONNX는 원본 작업공간의
`line_detector/training/highres/stop_line_0910/onnx_checks/current_best_snapshot.onnx`에 있는
검증용이며 최종 배포 모델이 아니다. 학습이 끝나면 위 변환 명령을 다시 실행한다.

사용자 지침에 따라 **ROS 빌드·노드 실행·젯슨 TensorRT 엔진 검증은 수행하지 않았다.**
새 메시지 필드가 있으므로 실제 젯슨에서 패키지와 관련 소비자를 재빌드해야 한다.
