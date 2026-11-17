# depth_lidar: 초기 자세를 고정한 가상 2D LiDAR

차선을 보기 위해 아래로 기울인 OAK 카메라에서 전방 장애물의 대략적인 위치를 얻는다.
시작할 때만 `bev_processor`와 공용인 `oak_startup`으로 roll/pitch/높이를 측정하고,
이후에는 고정 변환 → 높이 필터 → 각도별 최단 거리 → 시간 확인 → 점유격자 →
4/8방향 연결 군집화(기본 8) → 셀 외곽선 추출을 처리한다. 주행 중 평면·자세 재추정은 하지 않는다.

## 실행

DepthAI C++ 3.6+, ROS 2, OpenCV와 보정된 IMU가 있는 OAK가 필요하다.
기본 설정은 초기 측정과 runtime 모두 dot 프로젝터를 사용하므로 이를 지원하는 OAK Pro 장치가 필요하다.
공용 패키지가 추가되었으므로 기존 설치에서도 의존 패키지를 함께 빌드한다.

```bash
colcon build --packages-up-to depth_lidar --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch depth_lidar depth_lidar.launch.py
# GUI/ROI 확인용
ros2 launch depth_lidar depth_lidar.launch.py config_file:=$(ros2 pkg prefix --share depth_lidar)/config/depth_lidar_roi_test.yaml
```

1. 평평한 바닥에 차를 정지시키고 초기 측정 ROI에 충분한 바닥이 보이게 한다.
2. 초기 측정은 RGB(CAM_A)에 정렬된 1280×800 depth와 IMU를 사용한다.
   IMU 정지 여부, 평면 방향/잔차, 여러 프레임의 높이·법선 안정성을 확인한다.
   기본 `startup.attitude_source: depth`는 평면에서 자세를 얻고 IMU로 확인한다.
3. 성공하면 초기 장치 연결을 닫고 **같은 장치 ID**로 저해상도 depth 파이프라인을 연다.
   측정한 roll/pitch/높이는 프로세스 종료까지 고정한다. 이후 IMU 수신도 하지 않는다.
4. `/depth_lidar/ready`가 true가 된 뒤 스캔을 사용한다.
   초기 측정 실패 시 원인을 `/depth_lidar/status`와 로그에 표시하고 검출을 시작하지 않는다.
   정지 상태와 ROI를 확인한 후 노드를 재시작한다. 이동 중 자동 재측정은 없다.

`startup.*`는 실행 중 변경할 수 없다. `preview.mode`는 표시만 즉시 전환한다.
그 외 검출/카메라 파라미터를 변경하면 runtime 카메라
파이프라인과 짧은 거리 보존 이력을 재시작하지만 초기 자세는 유지한다.
C 키는 runtime 카메라 창/USB 스트림을 켜고 끈다. 기본 실행은 GUI를 사용하지 않는다.

## Dot 프로젝터 강도

초기 자세 측정과 장애물 검출에 서로 독립된 강도를 적용한다. 기본값 0.5는 시작 설정이며
환경에 맞춰 조절한다. 최적 강도를 실측한 값은 아니다.

```yaml
startup.ir_dot_projector_intensity: 0.5  # 초기 자세 측정
depth.ir_dot_projector_intensity: 0.5   # 장애물 검출
```

두 값 모두 유한한 0.0~1.0만 허용한다. 각각 0.0으로 끌 수 있고, 프로젝터가 없는 장치는
두 값 모두 0.0으로 설정한다. 양수 설정을 장치에 적용하지 못하면 해당 단계는 오류로
처리하며, 프로젝터가 켜졌다고 가정한 채 검출을 계속하지 않는다.

초기 측정값은 `startup.*`의 기존 규칙대로 실행 중 변경할 수 없다. runtime 강도는 다음처럼
변경하며, runtime 파이프라인만 재시작하고 초기 자세는 다시 측정하지 않는다.
C 키로 카메라 창을 켜거나 재연결할 때도 runtime 강도를 다시 적용한다.

```bash
ros2 param set /depth_lidar depth.ir_dot_projector_intensity 0.5
```

## 좌표와 처리

초기 측정의 기준은 CAM_A optical, runtime depth는 rectified CAM_C optical이다.
장치 EEPROM의 오른쪽 rectification 회전 역변환과 CAM_C→CAM_A 외부 보정을 결합한다.
최종 변환은 다음과 같다.

```text
p_vehicle = T_vehicle_CAM_A · T_CAM_A_CAM_C · R_rectified_right^T · p_rectified_right
T_vehicle_CAM_A.rotation = Rz(yaw) · Ry(pitch_down) · Rx(roll) · optical_to_FLU
```

`frame_id: front_axle`는 앞차축 아래 지면을 원점으로 하는 +X 전방/+Y 왼쪽/+Z 위
좌표계다. `sensor.x_m/y_m/yaw_deg`는 **CAM_A의** 수평 위치와 yaw이며 실제로 적용된다.
높이와 roll/pitch는 초기 측정값이다. 기존 YAML의 센서 위치가 depth 카메라 기준이었다면
CAM_A 기준으로 바꾼다. TF는 자동 발행하지 않으며, 다른 노드와 연결할 때 실제 원점·축을
동일하게 설정해야 한다. 이름만 바꿔서 다른 좌표계를 만들 수는 없다.

픽셀 `(u,v)`와 광축 깊이 `D`로 `(D(u-cx)/fx, D(v-cy)/fy, D)`를 복원한다.
고정 변환 뒤 `height.min_m..height.max_m`에 들어가는 점만 사용한다.
수평 거리 `hypot(X,Y)`와 방향 `atan2(Y,X)`를 구하고 각도 bin마다 가장 가까운 값을 남긴다.
깊이 이웃이 없는 고립점과 표본 수가 부족한 bin은 제외한다.
같은 bin에서 현재 거리와 10cm 이내인 관측이 최근 4개 depth 프레임 중 3개 이상
있을 때만 출력한다. 이 조건은 이미 표시된 점에도 계속 적용한다. 스캔 거리에는 평균 거리나 군집 중심을
사용하지 않으므로 긴 벽을 큰 원으로 앞쪽까지 확장하지 않는다. 군집 평균 위치는 별도로 제공한다.

각 픽셀의 회전된 광선과 높이 조건을 만족하는 raw-depth 구간을 미리 계산한다.
매 프레임에는 깊이 범위 검사 → 필요한 점의 XY 계산 → bin 최솟값 갱신을 수행한다.
전체 3D 포인트클라우드를 만들거나 정렬하지 않는다. 연산은 표본 수에 선형이며,
30 FPS/400p/stride 2/141 bins를 기본으로 둔다. 실제 성능은 하드웨어에서 확인해야 한다.
로그의 `project+grid ms`는 호스트 필터·변환·시간 확인·격자·군집·외곽선 시간이며 OAK stereo 연산,
USB 수신 대기, ROS 발행, 프리뷰 비용을 포함하지 않는다.

## 주요 설정

| 설정 | 의미 / 조정 방향 |
|---|---|
| `startup.roi_*` | 초기 1280×800 CAM_A 정렬 depth의 바닥 영역. runtime `roi.*`와 별개다. |
| `startup.minimum_height_m/maximum_height_m` | 예상 카메라 장착 높이 허용 범위. 기본 0.10~0.40m. |
| `startup.ir_dot_projector_intensity` | 초기 자세 측정 강도. 기본 0.5, 범위 0~1, 0은 OFF. 변경 후 노드 재시작 필요. |
| `depth.ir_dot_projector_intensity` | 장애물 검출 중 강도. 기본 0.5, 범위 0~1, 0은 OFF. 실행 중 변경 가능. |
| `startup.timeout_sec` | 안정된 초기 표본을 기다릴 한도, 기본 45초. |
| `height.min_m` | 바닥으로 제외할 높이. 기본 5cm. 높일수록 바닥 오검출은 줄지만 낮은 물체를 놓친다. |
| `height.max_m` | 검출할 높이 상한. 기본 40cm. 실제 차체 충돌 높이에 맞춘다. |
| `points.pixel_stride` | 기본 2. 3~4로 높이면 표본과 연산이 감소하지만 작은 물체를 놓치기 쉽다. |
| `roi.*` | 기본 전체 영상. 기울어진 카메라에서는 같은 높이가 거리마다 다른 행에 보인다. 좁히면 사각이 생길 수 있다. |
| `scan.bins` | 기본 -70~+70도에 141개(1도). 방향 해상도를 낮추려면 줄인다. |
| `scan.min_points_per_bin` | 기본 2. 높이면 잡음과 작은 장애물 모두 더 많이 제외된다. |
| `filter.min_neighbors/neighbor_delta_m` | 기본 이웃 1개, raw-depth 차이 8cm 이내. 작은 물체가 사라지면 먼저 이 필터와 stride를 확인한다. |
| `scan.confirm_hits` | 기본 3. 표시·갱신에 필요한 최근 관측 횟수. 1이면 시간 확인을 끈다. |
| `scan.confirm_window_frames` | 기본 4. 관측 횟수를 세는 depth 프레임 수. 1~30, confirm_hits 이상이어야 한다. |
| `scan.confirm_distance_m` | 기본 10cm. 현재 거리와 이 차이 이내인 같은 bin의 과거 관측만 횟수에 포함한다. |
| `scan.hold_sec` | 기본 80ms. 마지막으로 확인을 통과한 점을 잠깐 보존한다. 0으로 꺼도 시간 확인은 유지한다. |
| `input.max_age_sec` | 기본 200ms. 오래된 입력/입력 단절 시 결과를 비운다. |
| `grid.resolution_m` | 기본 5cm. 키우면 점들이 연결되기 쉬우나 가까운 물체가 합쳐지고 경계가 거칠어진다. |
| `grid.x_min_m/x_max_m/y_min_m/y_max_m` | 차량 좌표계 격자 범위. 최솟값 포함/최댓값 제외. 범위 길이는 셀 크기의 정수배여야 한다. |
| `grid.min_returns_per_cell` | 기본 1. 한 셀에 필요한 확정 스캔점 수. raw depth 픽셀 수가 아니다. |
| `cluster.connectivity` | 기본 8. 4는 상하좌우만 연결, 8은 대각선 접촉도 연결. 4/8만 허용. |
| `cluster.min_cells` | 기본 2. 이보다 작은 연결 성분은 격자·군집 출력에서 제외한다. |
| `cluster.min_returns` | 기본 2. 연결 성분에 필요한 확정 스캔점 수. |

높이 필터는 초기 바닥 기준이며 지면의 경사 변화나 차체 pitch/roll 변화는 보정하지 않는다.
카메라가 내려다보는 각도/FOV 밖에 있는 물체는 좌표 변환으로 복구할 수 없다.
수평의 얇은 단일 레이저 면 대신 **높이 구간의 가장 가까운 표면**을 투영한 가상 스캔이다.

## 깜빡이는 점 억제

기존에는 한 프레임만 잡혀도 즉시 출력했다. 이제 한 번 나타났다 사라지거나 매 프레임
번갈아 나타나는 점은 기본 3/4 확인을 통과하지 못한다. 같은 방향이어도 거리가 크게
달라지면 새 거리에서 다시 표본이 모여야 한다. 출력은 확인을 통과한 현재 실제 거리이며
평균으로 새 장애물 위치를 지연 이동시키지는 않는다.

기본값은 연속 3개 관측으로 최초 표시되므로 30 FPS에서는 첫 관측 이후 약 67ms가 추가된다.
누락 프레임이 있거나 실제 수신 FPS가 낮으면 더 늦어질 수 있다. 작은 물체나 빠르게 움직여
각도 bin을 바꾸는 물체도 제외될 수 있다. 그런 경우 `confirm_hits: 2`,
`confirm_window_frames: 3`으로 완화한다. 단, 2/3은 번갈아 나타나는 잡음을 통과시킬 수 있다.
더 엄격하게 하려면 4/5로 올린다. window만 키우면 오래된 관측도 더 포함되어 느슨해질 수 있다.

같은 물체의 거리 잡음이 10cm를 넘어서 계속 탈락할 때만 `confirm_distance_m`을 늘린다.
`hold_sec`를 늘리는 것은 점을 오래 남기는 효과이며 잡음 제거 조건을 강화하지 않는다.
깊이 입력 단절·파이프라인 재시작·좌표 변환 변경 때 확인 이력도 지운다.
프리뷰 갱신이나 유지 중 재발행은 확인 횟수에 포함하지 않는다.
지속적으로 검출되는 바닥 오차는 이 필터로 제거되지 않으므로 초기 자세와 높이 문턱값을 확인한다.

## 점유격자와 군집 외곽선

군집 기준은 실행에 사용하는 YAML의 `depth_lidar.ros__parameters`에서 조절한다.
아래 항목이 없는 이전 YAML은 코드 기본값이 적용된다. 기존 파일에 다음 블록을 병합한다.

```yaml
depth_lidar:
  ros__parameters:
    grid.resolution_m: 0.05
    grid.min_returns_per_cell: 1
    cluster.connectivity: 8
    cluster.min_cells: 2
    cluster.min_returns: 2
```

저장한 YAML은 노드 시작 시 읽으므로 수정 후 해당 파일로 다시 실행한다.

```bash
ros2 run depth_lidar depth_lidar_node --ros-args --params-file /path/to/depth_lidar_roi_test.yaml
```

실행 중 `ros2 param set /depth_lidar cluster.connectivity 4` 등으로 변경할 수도 있다.
이 경우 runtime 파이프라인과 확인 이력은 재시작하고 초기 자세는 유지한다.
`scan.confirm_*`는 시간 확인 조건이며, 위 항목들은 확정된 점의 공간 군집 기준이다.


확인된 2D 스캔점으로만 격자를 만든다. 각 방향의 최단 거리로 줄이기 전의 전체 depth
점군을 복원하는 것은 아니다. 카메라에서 관측된 장애물의 앞쪽 표면을 군집화한다.

기본 5cm 셀, 전방 0~3m/좌우 -3~3m에 7,200개 셀을 두며, 설정한 4/8방향의 인접 점유 셀을
한 군집으로 묶는다. 최소 2개 셀·2개 스캔점을 만족해야 격자·외곽선·군집 출력에 남는다.
기존 `/scan`과 `/scan_points`는 군집 크기 필터 이전의 확정 스캔을 계속 제공한다.
작은 장애물도 군집으로 남겨야 하면 두 최소값을 1로 낮춘다.

노출된 셀의 네 변만 외곽선으로 추출한다. 오목한 형태와 내부 구멍을 보존하고 빈 셀을
메우거나 외곽을 원/볼록다각형으로 감싸지 않는다. 셀 단위 양자화 오차는 존재한다.
벽/물체라는 의미 분류는 하지 않으며, 길게 연결된 벽은 긴 셀 경계로 표현한다.
경로 충돌 검사에는 점유 셀/실제 외곽선을 사용하고 군집 중심이나 사각 범위를 대신 쓰지 않는다.

1도 간격의 스캔은 원거리·비스듬한 벽에서 빈 셀이 생겨 여러 군집으로 나뉠 수 있다.
이 경우 `grid.resolution_m: 0.10`으로 키우거나 `scan.bins`를 늘려 비교한다.
각도 bin을 늘려도 카메라의 유효 표본 수가 부족하면 개선되지 않는다.
서로 가까운 물체가 합쳐지면 셀 크기를 줄인다. 대각선 접촉 때문에 합쳐진다면
`cluster.connectivity: 4`로 분리한다. 4방향은 비스듬한 벽도 여러 군집으로 나눌 수 있다.

군집 ID는 매 프레임 다시 부여하며 추적 ID가 아니다. 확인된 점의 짧은 hold는 유지하되
군집을 추가로 장시간 유지하지 않는다. 점의 hold 만료 시 외곽선과 군집도 함께 갱신한다.
새 프레임이나 만료 갱신 때 한 번 계산한 결과를 프리뷰가 재사용한다.
추가 연산은 스캔점 수 + 격자 셀 수에 선형이며 격자는 최대 100,000셀로 제한한다.

`/clusters`는 군집당 한 점인 PointCloud2이며 다음 필드를 제공한다.

- `cluster_id`: 같은 프레임의 `/contours` marker ID.
- `x/y/z`: 지지 스캔점의 평균 XY와 z=0. 실제 물체 부피 중심이 아니다.
- `min_x/max_x/min_y/max_y`: 점유 셀의 축 정렬 범위. 빈 영역을 채워 점유 처리하지 않는다.
- `nearest_range_m`: 군집에 속한 스캔점 중 실제 최단 수평 거리.
- `cell_count/return_count`: 점유 셀 수/지지 스캔점 수.
- `observation_age_sec`: 셀별 최신 관측 나이 중 최댓값. USB 지연은 제외한다.

## 레이더 표시 선택

```yaml
preview.mode: "both"  # points / clusters / both
```

- `points` 또는 창에서 **1**: 군집화 전 확정 스캔점만 표시.
- `clusters` 또는 **2**: 군집 조건을 통과한 셀 외곽선만 표시.
- `both` 또는 **3**: 같은 프레임의 점과 외곽선을 함께 표시(기본값).

파란 채운 점은 현재 확정 스캔점, 주황 선은 현재 군집 외곽선이다.
유지 중인 포인트는 회색 빈 점, 유지 중인 셀 외곽선은 회색 선으로 표시한다.
레이더 또는 스테레오 창에 포커스를 두고 키를 누르면 전환되며 ROS 파라미터에도 반영된다.
실행 중 다음 명령으로도 바꿀 수 있다.

```bash
ros2 param set /depth_lidar preview.mode both
```

`preview.mode`만 바꾸면 파이프라인 재시작이나 시간 확인 이력 초기화 없이 다음
프리뷰부터 적용된다. `/preview` 이미지 토픽과 GUI에 같은 모드를 적용한다.
검출·군집화와 스캔/격자/군집 토픽은 모든 표시 모드에서 계속 처리한다.
키/ROS 명령 변경은 YAML에 저장되지 않으므로 재실행 기본값은 YAML에서 변경한다.

표시 포인트는 raw depth 전체가 아니라 **높이·공간 필터와 시간 확인을 통과한
각도별 스캔점**이며 `/scan_points`와 같은 좌표를 사용한다. 격자 범위나 군집 최소
조건에서 제외된 스캔점도 표시하므로, 점은 있는데 외곽선이 없는 구간을 확인할 수 있다.
점 자체가 없으면 군집화 이전의 깊이 측정/필터 단계를 확인해야 한다.

## 출력

| 토픽 | 형식 | 내용 |
|---|---|---|
| `/depth_lidar/scan` | `sensor_msgs/LaserScan` | 고정 차량 좌표계 방향별 수평 거리. 부재/무효/FOV 밖은 NaN. |
| `/depth_lidar/scan_points` | `sensor_msgs/PointCloud2` | 유효 bin의 x/y, z=0, `observation_age_sec`. 구독 시 생성. |
| `/depth_lidar/occupancy` | `nav_msgs/OccupancyGrid` | 군집 필터 후 셀. 100=점유, -1=미관측. 0/free를 추정하지 않는다. |
| `/depth_lidar/contours` | `visualization_msgs/MarkerArray` | 군집별 LINE_LIST, 내부 구멍 포함. 사라진 군집도 삭제한다. |
| `/depth_lidar/clusters` | `sensor_msgs/PointCloud2` | 군집별 평균 위치·범위·최단 거리·셀 수·스캔점 수. 구독 시 생성. |
| `/depth_lidar/status` | `std_msgs/String` | 초기 측정/준비/입력 장애 상태. transient-local. |
| `/depth_lidar/ready` | `std_msgs/Bool` | 초기 측정 성공 후 유효한 최신 depth를 처리 중인지. 검출점 개수와 별개. |
| `/depth_lidar/startup_pose` | `std_msgs/String` | 고정 CAM_A roll/pitch/높이, 장치 ID, 자세 출처. 진단용, TF 아님. |
| `/depth_lidar/preview` | `sensor_msgs/Image` | `preview.mode`로 스캔점/군집 외곽선/둘 다 선택. 파랑=현재 점, 주황=현재 외곽선, 회색=유지 중. |
| `/depth_lidar/stereo_preview` | `sensor_msgs/Image` | 좌/우 정렬 영상과 초록 샘플링 ROI. 왼쪽 ROI는 위치 안내이며 정확한 대응점 아님. |

스캔 원점은 차량 원점이므로 카메라 원점과 다르고, 여러 높이의 반환값을 합친다.
NaN을 최대거리 또는 통과 가능한 빈 공간으로 치환하지 않는다. 특히 이 스캔을 이용한
자유공간 ray-clearing은 별도로 가시성을 처리해야 한다. `LaserScan` 자체에는 점별 age가
없다. hold를 끄거나 `scan_points.observation_age_sec`를 함께 사용한다.
유지 시간은 호스트가 관측한 시각부터 계산하며 USB 지연은 age 필드에 포함하지 않는다.
현재 프레임 발행 시 header는 추정 촬영 시각, 입력 대기 중 만료 갱신은 발행 시각이다.

기존 `ground.*`, `floor.*`, `stabilization.*`, `bev.*`, `preview.scale`,
`scan.range_selection`과 과거 원 반지름 관련 `cluster.*` 파라미터는 사용하지 않는다.
새 격자는 확정 스캔점을 입력으로 사용하므로 과거 raw depth 개수 기준인
`grid.min_points_per_cell`/`cluster.min_points` 대신 `grid.min_returns_per_cell`/
`cluster.min_returns`를 사용한다. 과거 cells/ground_status/ground_valid 토픽은 복원하지 않는다.

같은 OAK를 `camera_driver` 또는 `bev_processor`가 이미 열고 있으면 독립 depth_lidar와
동시에 사용할 수 없다. 이번 변경은 초기 측정 **알고리즘 공유**이며 장치/스트림의 동시
소유 통합은 아니다. BEV와 동시 운용하려면 한 파이프라인에서 영상을 공유하도록 연결해야 한다.
