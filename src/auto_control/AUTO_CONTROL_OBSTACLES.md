# 기존 auto_control 통합 실행에서 장애물 매핑 확인

기존 `vehicle_bringup/auto_drive.launch.py`에 장애물 검출을 연결했다.
**차선 검출 + 신호등 검출 + 장애물 BEV 매핑**을 함께 실행하며,
`manual_test:=true`로 자동 제어 출력을 끄고 조이스틱으로 수동주행할 수 있다.
BEV는 기존 **120cm × 전방 300cm**, 딥러닝 차선 모델 입력은 **120×300px**를 유지한다.

## 이번 수동 실험 실행

차량 PC의 저장소 루트에서:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to vehicle_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch vehicle_bringup auto_drive.launch.py \
  manual_test:=true \
  obstacle_params_file:="$(pwd)/src/auto_control/config/obstacles.yaml"
```

평지에서 정지한 상태로 시작 측정과 모델 준비를 끝낸 후 수동주행한다.
이 실행은 수동조작·VESC·운동 모니터도 함께 켠다. 기존 수동주행이 이미 실행 중이면
`manual_enabled:=false`를 추가한다. 운동 모니터도 실행 중이면 `dynamics_enabled:=false`를 추가한다.
수동 차량 namespace 기본값은 `autopilot03`이며 `vehicle_namespace:=...`로 변경할 수 있다.
기존 카메라/auto_drive 실행은 종료하고 이 명령 하나만 사용한다.

`manual_test:=true`는 기존 `auto_control_node`를 `monitor_only`로 유지한다. 차선 추종 계산과
진단은 계속하고 조향·구동·제동 명령을 발행하지 않으므로 수동조작과 충돌하지 않는다.
외부 auto_control YAML이 drive여도 수동 실험 옵션이 우선한다. 신호등 정지 제어는 OFF이고,
성능 측정용 자동 종료/GUI 비활성화도 해제한다. `auto_enabled=false`만 사용하는 방법은
정지 명령이 수동조작에 섞일 수 있어 이 실험에서는 monitor_only를 사용한다.

## 구성과 표시

- 기존 `camera_driver`가 한 OAK에서 RGB와 스테레오 Depth를 함께 받는다.
- `bev_processor`는 기존 영상 변환과 시작 측정/기하 공유를 담당한다.
- **`auto_control::ObstacleDetectorNode` (`/auto_obstacles`)**가 별도 스레드에서
  ROI 깊이 → 바닥 제거 → 각도별 거리 → 군집 검출을 수행한다. 제어 루프와는 별도로 동작한다.
- 기존 `line_detactor` 딥러닝 모델과 `traffic_light_detector`를 사용한다. 신호등은
  BEV 변환 전 RGB를 공유하고, 장애물은 같은 프로세스의 Depth를 공유한다.
- 기존 차선 결과 프리뷰에 관측 포인트·대표점·거리, 하단에 Depth FPS·시각 차이와
  `Traffic: RED / GREEN / UNKNOWN`이 표시된다.
- 장애물 위치 토픽은 `/auto/obstacles` (PoseArray)다. XY는 앞차축 기준 관측 표면의 평균,
  Z=0, 방향은 추정하지 않는다. 화면 합성에는 군집 표면점도 내부적으로 전달한다.
- 관측 포인트를 외곽선으로 연결해 물체 크기처럼 표시하지 않는다. 회피 ON일 때 대표점을
  중심으로 가정한 13×13cm 정사각형을 별도로 표시한다. 색상/#번호는 프레임 내 구분이며 추적 ID가 아니다.

기본은 장애물 표시만 수행한다. **`avoidance_control_enabled:=true`이면 차선 안 장애물의
반대쪽으로 변형한 중앙선을 조향에 사용한다. 기본 변형 모드에서 계획이 없으면 기존 중앙선으로 주행한다.**
`manual_test:=true`는 이 옵션을 켜도 monitor_only를 유지하므로 액추에이터 출력은 없다.
독립 `roi_lidar` 패키지는 유지하며 이 통합 실행에서는 시작하지 않는다.

## YAML 조정

`src/auto_control/config/obstacles.yaml`을 직접 수정하고 launch를 재시작한다.
위 명령처럼 소스 YAML 경로를 넘기면 YAML 변경 후 재빌드가 필요 없다.
네 섹션은 모두 `obstacles.*` 옵션만 허용한다.

| YAML 섹션 | 담당 |
|---|---|
| `camera_driver` | Depth 해상도/FPS/IR 강도·신뢰도·LR/subpixel |
| `bev_processor` | `obstacles.share_reference`: 시작 기하 공유 |
| `auto_obstacles` | ROI, 바닥 제거, 스캔·군집 및 동기화 |
| `line_detactor` | 장애물 표시·촬영 시각 차이·만료 |

실행 시 `obstacles_enabled`가 카메라 Depth·기하 공유·검출 노드·화면 합성을 한 번에 켜고 끈다.
기본 true이며 부하 비교는 `obstacles_enabled:=false`로 한다.
차선·신호등·제어의 기존 파일은 각각 `line_detactor_params_file`, `traffic_light_params_file`,
`auto_control_params_file`로 지정한다. 신호등은 기존 `traffic_light_enabled` 옵션을 유지한다.

작은 장애물은 `obstacles.points.pixel_stride: 1`로 표본을 늘리고,
`obstacles.cluster.min_bins: 1`로 작은 묶음을 허용할 수 있다. 연산량/노이즈가 증가할 수 있다.
바닥 제거 두께는 `min(max_distance_m, distance_m + distance_per_meter × 수평거리)`다.
기본 1m에서 6cm, 3m에서 8cm 이하를 제외하고 군집 방향에는 10cm 이상의 높이 근거를 요구한다.
낮은 장애물도 제외될 수 있다. `obstacles.avoidance.enabled` 이외의 장애물 옵션은
시작 시 적용하며 YAML 저장 후 재실행한다.

## 시간·성능·검증 범위

시작할 때 한 번 측정한 평면과 RGB에 실제 사용한 고정 줌/자세 보정, 프레임 내부·외부
보정을 사용해 동일 BEV 좌표로 변환한다. Depth와 RGB 보정의 촬영 시각 차이는 기본 40ms,
차선 화면과 장애물은 60ms 이내로 제한한다. 촬영 후 250ms가 지난 군집은 GUI에서 지운다.
`depth WAIT / stale or unsynced`는 신선한 대응 결과를 기다리는 상태다.
차량 이동/Depth 시각으로 자세를 별도 보간하지 않으므로 주행 중 정합은 실측이 필요하다.

카메라/Depth 큐는 최신 한 장을 유지하며 전체 포인트 클라우드의 ROS 직렬화를 하지 않는다.
차선/신호등은 GPU, 군집은 CPU에서 동작하나 USB·메모리·GPU 자원 경쟁은 남는다.
Depth 기본 요청은 400p/60FPS다. 실제 속도는 화면과 `OBSTACLES` 로그에서 확인한다.
코드 수정에서는 요청대로 빌드·테스트·실차 실행을 하지 않았다.

## 실제 회피 제어 연결 (실험 기능, 기본 OFF)

`avoidance_test_enabled`는 화면용이고 **`avoidance_control_enabled`가 실제 제어 적용 스위치**다.
`auto_control/msg/AvoidancePlan`에 계획 모드를 추가했으므로 기존 설치에서 재빌드가 필요하다.

```bash
colcon build --packages-up-to vehicle_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 기존 실행을 종료한 뒤 사용. 아래 명령은 실제 조향/구동/제동을 활성화한다.
ros2 launch vehicle_bringup auto_drive.launch.py \
  manual_test:=false \
  avoidance_control_enabled:=true \
  auto_control_mode:=drive \
  auto_enabled:=true \
  obstacle_params_file:="$(pwd)/obstacles_test.yaml" \
  auto_control_params_file:="$(pwd)/auto_control_test.yaml" \
  bev_params_file:="$(pwd)/bev_config_test.yaml" \
  line_detactor_params_file:="$(pwd)/line_detactor_test.yaml" \
  traffic_light_params_file:="$(pwd)/traffic_light_test.yaml" \
  input_mode:=slcan slcan_channel:=/dev/ttyACM0 slcan_bitrate:=500000 \
  can_controller_id:=112 preview_enabled:=true preview_result_only_enabled:=true
```

- 실제 적용 OFF: 재실행할 때 `avoidance_control_enabled:=false`. 기존 차선 추종으로 돌아간다.
- 실제 적용 알고리즘을 구동 없이 확인: `manual_test:=true avoidance_control_enabled:=true`.
  VESC/조이스틱/운동 모니터도 필요 없으면 `manual_enabled:=false dynamics_enabled:=false` 추가.
- 기존 화면 전용: `manual_test:=true avoidance_control_enabled:=false avoidance_test_enabled:=true`.
- 적용 ON에서는 계획 계산도 자동으로 ON. `avoidance_test_enabled:=false`로 실제 적용을 해제할 수 없다.
- 실행 중 계획 OFF는 아래와 같이 가능하다. 기본 변형 모드에서는 기존 중앙선으로 복귀한다.
  검사 모드(`avoidance_deformation_only: false`)에서는 정지 요청이다. 실제 적용 여부와 모드는 시작 시 고정이다.

```bash
ros2 param set /auto_obstacles obstacles.avoidance.enabled false
ros2 param set /auto_obstacles obstacles.avoidance.enabled true
ros2 topic echo /auto/avoidance_plan
ros2 topic echo /auto/avoidance_preview/status
```

### 기본 동작: 출발을 차단하지 않는 중앙선 변형

기존 외부 YAML에도 새 모드 기본값 **`avoidance_deformation_only: true`**가 적용된다.
`auto_control_test.yaml`의 `auto_control.ros__parameters`에 명시할 수도 있다.

```yaml
    avoidance_control_enabled: true
    avoidance_deformation_only: true
```

`auto_drive`가 이 모드를 `auto_obstacles`의 `obstacles.avoidance.deformation_only`에도
동일하게 전달한다. 단독 노드를 실행할 때는 두 노드의 모드를 맞춘다.
메시지에도 모드를 포함하므로 검사 모드 제어기가 변형 전용 경로를 검사 완료로 오인하지 않는다.

- 기존 중앙선에 가장 가까운 위치로 각 장애물 대표점을 투영한다. 해당 위치의 좌우 차선
  사이에 있는 장애물만 사용한다. 관측 경계가 없으면 기존 `lane_width_m`으로 누락된 쪽을 추정한다.
- 왼쪽 장애물은 오른쪽, 오른쪽 장애물은 왼쪽으로 중앙선을 이동한다. 정확히 중앙이면 오른쪽을 선택한다.
  크기는 대표점을 중심으로 한 **13×13cm** 가정이며 포인트 퍼짐으로 크기를 늘리지 않는다.
- 이동량은 `차량 반폭 + 장애물 반폭 + safety_margin - 중앙선과 장애물의 횡거리`이며
  음수이면 변형하지 않는다. `max_offset_m`과 차선 내 명목상 반폭 여유로 이동량을 제한한다.
  이는 경로 모양 제한이며, 회전한 차체 전체의 차선 이탈/충돌 여부를 검사하는 것은 아니다.
- 장애물 개수에 제한을 두지 않는다. 전방 순서대로 이동 지점을 만들고 5차 smoothstep으로
  연결하여 반대 방향 장애물 사이에서는 S자를 만든다. 같은 전방 위치에 상충하는 장애물이 있으면
  더 큰 이동량 하나를 사용하며 통과 가능성을 판정하지 않는다.
- 충돌 검사, 물체 확장 영역, 곡률 초과, 반대쪽 장애물 구간 겹침, 목표 유실,
  복귀 확인 시간, 연속 3회 경로 확인은 **이 모드의 출발 조건이 아니다**.
- 변형 결과 없음/OFF/유실/만료/숫자·좌표계 오류는 변형 경로만 버리고 기존 중앙선으로 복귀한다.
  회피 때문에 보호 제동을 요청하지 않는다. 중앙선 자체의 유효성, enable, VESC/ERPM,
  수동 모드, 신호등 제어, 조향각·조향 변화율 제한은 기존대로 동작한다.
- 변형 중에도 기존 일반 longitudinal 제어를 사용한다. 따라서 단계 제어를 켰다면
  `longitudinal_start_duty`와 단계식 PID가 적용된다. 회피 전용 signed PID로 전환하지 않는다.
- 일반 주행 속도는 `maximum_speed_mps`로 설정한다. 차선 안 장애물 첫 검출 시 아래의
  일시 감속을 적용하고, 재검출이 없는 시간이 지나면 복귀한다. 신호등/일반 곡률 감속도 유지한다.

변형량은 `obstacles.avoidance.safety_margin_m`(0 허용), `max_offset_m`, 연결 길이는
`transition_m`으로 조정한다. `unknown_extent_m`, `vehicle_half_length_m`, `max_curvature_per_m`,
`lateral_acceleration_mps2`, `deceleration_mps2`, `clear_confirm_sec`은 이 모드에서
경로를 거부하거나 출발을 차단하는 데 사용하지 않는다. 속도에 따른 숨은 여유도 추가하지 않는다.

프리뷰는 원래 중앙선 노랑, 변형 경로 청록, 가정한 장애물 크기 주황 사각형으로 표시한다.
`APPLY DEFORM`은 변형 경로 요청, `APPLY CENTERLINE`은 기존 중앙선 사용 표시다.
충돌 확장 사각형은 그리지 않는다. 프리뷰는 실차 제어 승인 또는 실제 추종 성공의 표시가 아니다.

### 차선 안 장애물 검출 시 일시 감속

제어 YAML의 `auto_control.ros__parameters`:

```yaml
    obstacle_slowdown_enabled: true
    obstacle_slowdown_speed_mps: 1.2
    obstacle_slowdown_clear_sec: 1.0
```

최신 차선과 매칭된 Depth에서 차선 안 장애물을 처음 한 번 검출하면 목표 속도를
`obstacle_slowdown_speed_mps`를 경로 전체의 속도 상한으로 적용한다. 코너 감속을
켜면 각 경로 지점의 곡률 상한과 합친 뒤 감속·가속 계획을 계산한다. 장애물 상한이
일반 목표보다 높으면 목표를 올리지 않는다. 실제 속도는 기존 PID/제동으로 따른다.
기존 BEV 범위 안 대표점으로 판단하고, 차선이 일부 누락되면 중앙선과 기존
차선 폭으로 누락 경계를 추정한다. 경로를 휘게 할 필요가 없는 차선 가장자리 장애물도 포함한다.
차선 밖 장애물, 오래된 프레임, 같은 Depth의 반복 수신은 감속 타이머를 갱신하지 않는다.
회피 경로 유효성이나 연속 관측 확인과 독립적인 검출 신호다.

유효한 차선 안 장애물이 재검출될 때마다 마지막 검출 수신 시각을 갱신한다. 빈 관측 한 번에
감속을 풀지 않고, 마지막 검출 후 `clear_sec`가 지나면 일반 속도 제어로 복귀한다.
새 감속 목표를 시작할 때 기존 적분을 초기화하고, 복귀 가속은 기존 duty 상승률을 따른다.
해제는 제어 watchdog에서도 처리하므로 새 장애물 메시지가 안 와도 동작한다.
이는 마지막 검출 타이머이며 센서 입력 중단/OFF도 재검출 없음으로 취급한다.
기존 중앙선/ERPM 유실 정지와 신호등 정지는 해제로 취소하지 않는다.

감속 중에는 `electrical_brake_enabled: false`여도 속도 오차에 따른 전류 제동을 허용한다.
기존 `brake_entry_speed_error_mps`, `brake_exit_speed_error_mps`, `brake_maximum_current_amps`,
전류 상승/하강률을 사용한다(단계 제어 기준). 별도 장애물 제동 전류를 새로 만들지 않는다.
감속 중에는 일반 longitudinal PID를 사용하며, 감속 기능 OFF 또는 미검출 상태에서는
일반 주행 설정을 따른다. `manual_test`의 실차 출력 차단도 유지된다.

launch는 이 감속 기능이 켜져 있으면 차선 내 장애물 판정을 위한 planner를 자동으로 켠다.
`avoidance_control_enabled: false`로 경로 변형을 끈 상태에서도 감속만 사용할 수 있다.
전체 기능을 끄려면 `obstacle_slowdown_enabled: false`로 재실행한다. 모두 시작 시 설정이다.
로그의 `Obstacle slowdown ON/OFF`, 주기 상태의 `obstacle_slowdown=ON/OFF`,
주행 CSV의 `obstacle_slowdown_active`로 상태를 확인한다. 설정값에 1.0m/s 상한은 없다.

### 기존 검사 모드 (`avoidance_deformation_only: false`)

**아래의 충돌·곡률·복귀 확인·보호 정지 설명은 검사 모드에만 해당하며, 기본 변형 모드에는 적용하지 않는다.**

### 검사 모드의 중앙선 주행과 장애물 회피

기본 주행은 기존 `auto_control`의 중앙선 `path_`와 기존 Stanley/출발/속도 제어를
그대로 사용한다. 신선한 Depth로 중앙선과 차량 크기를 반영한 장애물 충돌을 검사하고,
경로가 막히지 않으면 `CENTERLINE` 결정을 보낸다. 이는 회피 경로가 없어서 무조건
중앙선으로 복귀하는 동작이 아니라, 장애물 충돌 검사를 통과한 중앙선 주행이다.

회피 모듈이 별도로 요구하던 **시작점 전방 25cm / 경로 50cm / X 좌표의 단조 증가**
조건은 제거했다. 계획기와 제어기는 같은 `build_ordered_path` 및 기존 auto_control YAML의
`path_minimum_points`, `path_minimum_span_m`, `path_minimum_x_m`, `path_maximum_x_m`,
`path_maximum_gap_m`, `path_geometry_window_m`을 공유한다. 내부 전달값은 launch가 설정하므로
장애물 YAML에 중복해서 적지 않는다. 중앙선 길이로 별도의 출발 속도 제한을 만들지 않는다.
가로로 꺾이거나 X가 감소하는 코너도 원래 점 순서를 따라 호 길이로 처리한다.

장애물이 중앙선 경로를 막으면 **장애물별 국소 이동 구간**을 만든다. 장애물 개수는
2개로 고정하지 않는다. 각 군집을 기존 중앙선의 가장 가까운 선분에 대응시키고, 해당
구간의 법선을 기준으로 왼쪽 물체는 오른쪽으로, 오른쪽 물체는 왼쪽으로 피한다.
정확히 중앙에 놓인 물체의 최초 방향은 왼쪽으로 정한다. 프레임 사이에는 가까운 군집을
일대일로 대응시켜 물체별 방향을 유지한다. 하나의 전역 LEFT/RIGHT 방향을 유지하지 않는다.

검출 대표점 중심의 고정 크기 정사각형과 차체·관측 오차 여유 설정으로 필요한 이동량과 통과 구간을 계산한다.
필요 이동량은 차체 선분을 중앙선 법선 방향으로 옮겼을 때 확장 장애물 사각형과
충돌하는 이동량 구간을 X/Y/선분 법선의 분리축으로 구해 계산한다. 사각형 전체를
횡방향으로 투영하던 과대 추정을 피하며, 선택된 경로의 실제 방향으로 최종 충돌 검사를
다시 수행한다. 차량·물체 여유를 줄이거나 `max_offset_m` 상한을 높이지 않는다.
각 장애물 옆을 지나는 기준점을 중앙선의 호 길이 순서로 정렬하고, 겹치는 같은 방향
구간은 합친다. 반대 방향 회피의 영향 구간이 겹쳐도 경로 생성 전에 정지하지 않는다.
영향 구간은 차체 길이·관측 오차·속도 여유까지 포함한 보수적인 범위이므로, 구간 중첩만으로
실제 회전하는 차체가 충돌한다고 단정할 수 없다. 겹치는 구간에서는 각 장애물의 통과
기준점까지 필요한 이동량을 유지하고, 그 기준점 사이를 S자로 연결한다.
원래 장애물 충돌 영역은 그대로 두며 최종 경로의 차체 방향·차선 여유·곡률·모든 장애물
충돌 검사를 통과한 경우에만 선택한다. `opposing obstacle zones overlap` 사전 탈락은 제거했다.

각 구간은 5차 smoothstep으로 연결한다. 충분히 떨어진 장애물 사이에는 중앙선으로 복귀하고,
가까운 반대쪽 장애물 사이에는 오른쪽→왼쪽→오른쪽처럼 부호가 바뀌는 S자 경로를 연결한다.
같은 쪽 장애물이 이어지면 불필요하게 중앙선으로 돌아오지 않고 회피 위치를 연결한다.
경로가 이동하면서 원래 중앙선 밖의 다른 물체와 충돌하게 되면 그 물체들도 한 번에
추가해 다시 연결한다. 이 과정과 연결 길이 보정을 합해 프레임당 최대 3번만 경로를 만든다.
이는 장애물 3개 제한이 아니며, 각 시도는 검출된 여러 장애물을 함께 처리한다.
추가 보정이 계속 필요하면 `BLOCKED: additional obstacle constraints`로 정지한다.

경로는 최대 160개 점이며, 최초 연결 길이는 `transition_m`, 이후 보정은 1.5배/2배다.
중앙선 기하는 프레임당 한 번 계산하고, 최종 경로에 대해 모든 관측 장애물, 차체 크기,
관측 차선과 추정 경계, BEV 범위, 조향 가능한 곡률을 검사한다. 기존 중앙선 밖 물체도
최종 충돌 검사에서 제외하지 않는다. `max_offset_m`을 넘는 이동은 허용하지 않는다.
`centerline_fallback_enabled: true`일 때만 누락된 경계를 기존 중앙선과
`centerline_lane_width_m`로 추정한다. 최종 검사를 통과하지 못하면 구동 경로를 발행하지 않는다.

`AvoidancePlan.follow_centerline=true`인 유효 메시지는 점 목록 없이 기존 중앙선 사용을
지시한다. false인 유효 메시지는 검사된 회피 경로를 전달한다. 메시지 정의가 변경되었으므로
발행/수신 노드를 모두 재빌드하고 기존 launch를 종료한 뒤 재시작한다.
제어기는 시간·좌표계·점 유효성·간격·곡률을 검증하며, 회피 경로는 일반 중앙선과 동일한
경로 길이 기준을 사용한다. 정상 중앙선 주행에는 별도의 회피 경로 3회 확인을 요구하지 않는다.
정지 후 회피 경로로 복구할 때의 기존 연속 확인은 유지한다.

Depth 입력 유실/만료나 장애물이 있는데 회피 후보를 찾지 못한 상황을 빈 경로로 간주하지 않는다.
이때와 계획 OFF/잘못된 메시지/중앙선 입력 유실에는 기존 보호 정지 및 watchdog을 유지한다.
`electrical_brake_enabled=false`로 회피 보호 정지 제동이 꺼지지 않는다.
신호등 정지와 일반 주행 속도 설정은 유지한다. 중앙선과 회피 경로 모두 기존 일반 구동
제어를 사용하며, 회피 전용 signed PID 및 회피 과속 보호 정지는 제거했다.

### 13×13cm 장애물과 파라미터

`AVOIDANCE local` 로그는 전체 검출 군집 수(`observed`), 회피에 포함된 군집 수(`blocking`),
합쳐진 이동 구간 수(`zones`), 경로 생성 횟수(`attempts`), 계획 소요 시간(`plan`, ms)을 표시한다.
실제 FPS/지연은 이 로그로 측정해야 하며 이번 변경에서 성능 시험은 하지 않았다.
`AVOIDANCE rejected`는 연결 길이별 첫 탈락 조건과 검사 중 최대 곡률을 2초마다 표시한다.
`curvature`, `lane clearance`, `obstacle`의 `s`는 해당 생성 경로 시작부터의 호 길이(m)다.
`AVOIDANCE geometry`는 속도 보정까지 포함한 차체 여유와 직선 기본 필요 폭을 보여 준다.
이동 상한 초과는 `BLOCKED: offset <필요 거리>m > <상한>m`으로 표시한다.
최초 장애물 검사에서는 프레임 내 군집 순번, 앞차축 기준 XY(m), 중앙선의 영향 구간 s(m)도
같이 기록한다. 군집 순번은 추적 ID가 아니다. `attempts=0`이면 경로 생성 전 단계에서
중단된 것이므로 차선·곡률 검사에서 탈락한 것으로 해석하지 않는다.
`ros2 topic echo /auto/avoidance_preview/status --once`로 정지 상태도 확인할 수 있다.

기존 `obstacles_test.yaml`의 `auto_obstacles.ros__parameters`에 필요하면 추가한다.
생략해도 코드 기본값은 0.13이다.

```yaml
    obstacles.avoidance.obstacle_size_m: 0.13
```

현재는 **화면의 검출 대표점(십자 표시)을 장애물 중심으로 가정**하고, 앞뒤·좌우 각각
6.5cm인 정사각형으로 계획한다. `obstacle_size_m`은 최소 폭이 아니라 가정한 고정 한 변의
길이다. 군집 포인트의 min/max 범위와 퍼짐은 크기에 사용하지 않으며, 관측점 뒤로 13cm를
추가하지 않는다. 이 점은 관측 표면에서 얻은 대표점이므로 실제 물체 중심과 다를 수 있다.
실제 크기·중심 추정은 별도 후속 작업이며 이번 구현은 크기 추정기가 아니다.

`unknown_extent_m`은 그 정사각형 바깥에 추가하는 관측/위치 오차 여유로, 설정한 값을
그대로 사용한다. 이전의 `max(unknown_extent_m, obstacle_size_m/2)` 하한은 제거했다.
따라서 0.02를 지정하면 6.5cm로 커지지 않고 2cm가 적용된다. 차량 반폭·반길이,
`safety_margin_m`, 속도에 따른 여유, 표본 간격 여유와 최종 충돌 검사는 계속 적용한다.
주황색 작은 사각형이 가정한 물체 크기이고, 흐린 붉은 큰 사각형은 차량 통과에 필요한
확장 영역이다. 확장 영역을 실제 물체 크기로 읽으면 안 된다.

`auto_control_test.yaml`의 `auto_control.ros__parameters`에 추가 가능한 제어 설정:

```yaml
    avoidance_control_enabled: false  # launch의 명시값이 우선
    avoidance_max_age_sec: 0.20
    maximum_speed_mps: 1.0
    avoidance_brake_current_amps: 2.5
    avoidance_wheelbase_m: 0.33
```

차량 축간거리/차체 크기/제동 전류는 실차에 맞춰야 한다. 제어 옵션은 모두 시작 시 고정이다.
기존 외부 YAML에 이 항목들이 없어도 기본값으로 동작한다. 실제 적용 시 planner의 곡률·
유효 시간 상한은 제어기 설정과 비교해 더 작은 값으로 맞춘다.

주행 속도는 **`auto_control.maximum_speed_mps` 하나로 설정**한다. 회피 속도의
1.0m/s 하드코딩 상한도 없다. 기존 `avoidance_speed_cap_mps`와
`obstacles.avoidance.max_speed_mps`는 호환을 위해 선언만 하며 값은 무시한다.
기존 파일에 남아 있어도 주행 속도를 제한하거나 값의 범위 때문에 실행을 막지 않는다.
지속적인 회피 속도 상한은 없으며, 차선 안 장애물 검출 시에만 아래의 일시 감속을 적용한다.
`curvature_speed_control_enabled: true`이면 실제 선택된 회피 경로의 곡률과 장애물
상한을 같은 속도 계획에 넣는다. 신호등 정지의 거리·정지 상태 추적은 별도로 유지된다.

launch는 최종 `maximum_speed_mps`(명시적 launch 인자 우선)를 planner의
`obstacles.avoidance.reference_speed_mps`에 전달한다. 이 값은 표시 및 검사 모드의
이동 여유 계산용이며 제어기로 되돌아오는 속도 상한이 아니다. 단독 planner 실행 시에만
이 표시 기준값을 직접 지정할 수 있다. 기존 메시지 필드 `speed_limit_mps`도 호환용 이름을
유지하지만 참고 속도만 담으며 제어기는 속도 계산에 사용하지 않는다.

| `obstacles.avoidance.*` | 기본값 | 의미 |
|---|---:|---|
| `enabled` | false | 계산 ON/OFF. 실시간 변경 가능, 적용 중 OFF는 정지 |
| `max_fps` | 10.0 | 계산 주기 상한, 허용 1..30. Depth FPS와 별개 |
| `max_age_sec` / `max_sync_sec` | 0.25 / 0.06 | 입력 만료 / RGB-Depth 시각 차이. 적용 시 만료는 최대 0.20 |
| `centerline_fallback_enabled` | true | 회피 후보 검사에서 누락된 경계를 중앙선과 차선 폭으로 추정. 기본 중앙선 주행은 양쪽 경계를 필수 요구하지 않음 |
| `vehicle_half_width_m` / `vehicle_half_length_m` | 0.15 / 0.25 | 앞차축 기준 차체 좌우/전후 최대 거리 |
| `obstacle_size_m` | 0.13 | 검출 대표점을 중심으로 가정한 고정 정사각형 한 변 |
| `safety_margin_m` / `unknown_extent_m` | 0.04 / 0.04 | 차량 여유 / 고정 물체 외곽에 더하는 관측 오차 여유(설정값 그대로) |
| `sample_step_m` | 0.025 | 경로 샘플 간격 |
| `max_offset_m` | 0.40 | 장애물별 중앙선으로부터 최대 이동량 |
| `offset_step_m` | 0.05 | 폐기된 전역 후보 간격. 기존 YAML 호환용으로만 읽으며 경로에 영향 없음 |
| `transition_m` | 0.70 | 국소 회피 진입/복귀 연결 길이. 보정 시 1.5배/2배; 가까운 반대 방향 구간 사이는 사용 가능한 간격에서 직접 연결 |
| `max_curvature_per_m` | 2.0 | 최대 곡률. 적용 시 차량 조향 한계로 추가 제한 |
| `max_speed_mps` | 폐기 | 기존 YAML 호환용. 값 무시 |
| `lateral_acceleration_mps2` | 0.4 | 구버전 호환용. 회피 감속 미사용 |
| `deceleration_mps2` | 0.5 | 구버전 호환용. 회피 감속 미사용 |
| `stop_response_sec` | 0.35 | 이전 YAML 호환용. 현재 제어/출발 조건에는 사용하지 않음 |
| `clear_confirm_sec` | 1.0 | 중앙 경로 복귀 전 연속 관측 시간 |

### 화면과 현재 제한

노랑=기존 중앙 경로, 청록=최종 국소 회피 경로, 주황=가정한 13×13cm 물체, 흐린 붉은 영역=차량 통과 여유 영역.
많은 평행 후보 경로를 겹쳐 그리지 않는다. 정지 시에는 마지막으로 시도한 경로 한 개만 흐린 붉은색으로 표시한다.
`APPLY`/`CONTROL PATH REQUESTED`는 제어 적용 설정이며, `manual_test`/`auto_enabled` 등
최종 구동 조건까지 충족해야 실제로 움직인다. `TEST`는 화면 전용이다.
`CENTERLINE`은 기존 중앙선 주행, `LOCAL`은 여러 장애물의 국소 회피 구간을 연결한 경로 사용을 뜻한다.
`n`은 회피에 포함된 장애물 군집 수다.
상태의 `C`는 회피 후보에서 추정 경계를 사용함을 뜻한다.
`L`은 중앙선 길이(m), `vref`는 설정된 일반 주행 기준 속도(m/s)이다. 실제 속도나 신호등까지 반영한 최종 목표 속도가 아니다.
WAIT/BLOCKED일 때는 속도 0만 보여주는 대신 거부 이유를 표시한다.
전체 이유는 `/auto/avoidance_preview/status` 및 제어기 상태 로그에서도 확인할 수 있다.

마지막 장애물 뒤에 복귀 연결 길이가 부족하면 회피 위치를 유지한 채 보이는 경로 끝까지
검사한다. 전체 공간을 탐색하는 전역 경로 계획기는 아니므로, 물리적으로 경로가 존재하더라도
이 국소 변형 방식으로 찾지 못할 수 있다. 좁은 차선이나 가까운 장애물 사이에서 차체가
통과할 수 없거나 곡률이 한계를 넘으면 정지한다. 차체 크기와 명시적인 관측 오차·속도 여유를
포함하므로, 가정한 물체가 13cm여도 차선 내부 통과 공간이 부족하면 정지할 수 있다.

회피 대상으로 보던 물체가 시야에서 사라지면 기존과 같이 통과했다고 간주하지 않고
`BLOCKED: target lost`로 둔다. 이 상태는 물체별로 관리한다. 후방 센서/차량 이동 보정에
의한 통과 검증이 없어 통과 끝에서 정지할 수도 있다. 차량 정지와 주변을 확인한 뒤
planner를 OFF→ON하면 보류 상태를 초기화할 수 있다.

이번 변경은 내부 BEV 프리뷰 자료 구조도 바꾸므로 `auto_control`만 선택 빌드하지 말고
`colcon build --symlink-install --packages-up-to vehicle_bringup`으로 관련 노드를 함께
재빌드하고 launch를 재시작한다. ROS `AvoidancePlan` 메시지 정의와 제어 ON/OFF 방식은 같다.
외부 YAML의 `offset_step_m`은 삭제해도 되고 그대로 두어도 무시된다. 나머지 설정은 유지된다.
빌드·테스트·실차 실행은 사용자 요청에 따라 진행하지 않았다.
