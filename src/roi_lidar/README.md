# roi_lidar: RGB BEV + 높이 필터를 통과한 XY 포인트

실제 RGB 카메라의 지면 BEV 위에 스테레오 깊이 포인트를 **위에서 수직으로 내려다본
XY 위치**로 표시한다. 객체를 묶거나 외곽선을 그리지 않고, 설정한 최대 높이 이하의
관측 후처리를 통과한 포인트를 그린다. 차선 인식은 수행하지 않는다.

예를 들어 같은 방향에서 1m와 2m 표면이 각각 관측되었다면 두 포인트를 모두 표시한다.
기존 `/scan`처럼 각도별 최근접점 하나로 줄인 결과를 프리뷰에 쓰지 않는다.
`points.pixel_stride`에 따른 이미지 샘플링, ROI, 거리/깊이/시야각 제한은 적용한다.
장애물 뒤의 가려진 표면은 복원하지 않는다.

## 시작과 실행

ROS 2 Humble, 기존 DepthAI C++ 3.6+ SDK, OpenCV와 OAK 장치가 필요하다.
SDK 경로는 [공통 설정](../../DEPTHAI_SETUP.md)을 따른다. SDK 재설치는 수행하지 않는다.
같은 OAK를 여는 camera_driver, bev_processor, point_cloud, depth_lidar 등은 종료한다.

```bash
cd ~/Desktop/1030ML/f1tenth_project_repo
source /opt/ros/humble/setup.bash
colcon build --packages-up-to roi_lidar --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch roi_lidar roi_lidar.launch.py \
  config_file:="$(pwd)/src/roi_lidar/config/roi_lidar.yaml"
```

평지에서 정지한 채 시작한다. `oak_startup`으로 지면과 카메라 높이·roll·pitch를
한 번 측정한다. 이 지면은 이후 고정이며 실시간 자세 보정이나 경사도 추정을 하지 않는다.
기존 point_cloud의 보정/깊이 입력 라이브러리를 공유하되 해당 노드나 RViz는 시작하지 않는다.
기본 좌표 원점은 앞차축 지면이며 +X는 전방, +Y는 차량 좌측이다.

기본 표시 영역은 폭 1.5m, 전방 4m로 미터당 픽셀 비율을 동일하게 유지한다.
최대 검출 거리는 별도의 `range.max_m`이며 화면 밖 포인트도 토픽에는 남는다.

## 화면과 조작

- **RGB BEV**: 실제 RGB 지면 투영 영상 위에 XY 포인트와 바닥 분류를 표시한다.
- **select depth ROI**: rectified-right 흑백 영상에서 사각형을 드래그한다.
  `preview.camera_image=false`이면 깊이 컬러 영상으로 바뀐다.
- **parameters**: 관련 파라미터를 한 창의 두 열과 페이지로 나눠 숫자 입력칸으로 표시한다. Prev/Next 또는 [ / ]로 페이지를 전환한다.
  슬라이더와 스크롤바는 사용하지 않는다. 프리뷰와 ROI 화면은 그대로 유지한다.

값을 클릭해 숫자를 입력하고 **Enter**, **A** 또는 **Apply edits** 버튼으로 변경분을
한 번에 적용한다. 단위는 ROS/YAML과 같아 최대 높이 50cm는 `height.max_m=0.5`로 입력한다.
켜기/끄기는 `0`/`1`, 표본 수·픽셀 간격은 정수를 입력한다.
Tab으로 다음 항목으로 이동하고 Backspace로 입력한 숫자를 지운다.
편집 중 Esc는 해당 입력을 취소한다. 편집 중이 아닐 때 R은 전체 ROI 복원,
Q/Esc는 노드 종료다. **Reload current values**는 미적용 입력을 모두 버린다.

`*` 표시된 카메라·DepthAI 항목이 실제로 바뀌면 적용 시 스트리밍을 재시작한다.
그 외 값만 변경하면 카메라 재시작 없이 다음 프레임에 반영한다.
지면 기준은 **ground.distance_m <= ground.max_distance_m < height.max_m**을
만족해야 한다. `obstacle.min_height_m`도 0 이상, `height.max_m` 미만이어야 한다. 서로 연관된 값을 함께 수정해 한 번에 적용할 수 있다.
잘못된 숫자나 범위·조합은 배치 전체를 거부하고 창과 로그에 이유를 표시한다.
입력은 남겨 두므로 수정 후 다시 적용하면 된다. 외부 `ros2 param set` 변경도 반영한다.

XY 포인트는 순수 빨강(BGR `0,0,255`)으로 표시한다.
`/roi_lidar/obstacle_points`에는 같은 빨간색을 나타내는 PCL 형식의 `rgb` 필드도 추가했다.
RViz에서 포인트 색상 변환을 `RGB8`로 선택하면 이 색을 사용한다.

## 포인트 높이 필터

각 depth 표본을 시작 보정의 차량 좌표 `(X,Y,Z)`로 변환한다.
RANSAC 성공 범위에서는 정규화된 현재 평면으로부터의 부호 있는 거리 `h=n·p+d`를 높이로 사용하고,
OFF/실패/범위 밖에서는 시작 지면의 `h=Z`를 사용한다. 지면 및 고립점 필터 통과 표본만 `(X,Y,0)`에 표시한다.

```text
선택된 지면 허용 두께 < h <= height.max_m
# RANSAC 성공 범위: obstacle.min_height_m (기본 0.06m)
# 그 외: min(ground.max_distance_m, ground.distance_m + ground.distance_per_meter × hypot(X,Y))
```

`ground.enabled=false`면 포인트의 지면 제거 하한을 해제한다. 최대 높이 제한은 유지한다.
픽셀 하나당 깊이는 하나뿐이므로 실제 LiDAR처럼 가려진 뒷면까지 반환되는 것은 아니다.
화면의 빨간 포인트는 지면·고립점 필터를 통과한 측정 표본이며 셀의 최소 표본 수와 무관하다.

## 바닥·장애물·미관측 분류

현재 프레임의 실제 표본을 기본 7cm 셀에 집계하며 초록/빨강/회색으로 표시한다.
이 색상은 지면 관측 근거의 구분이며 경로 계획이나 차량의 최종 통과 판정이 아니다.

| 셀 | 조건 |
|---|---|
| 초록: 관측 바닥 | `-terrain.below_ground_m <= h <= 선택된 지면 허용 두께`인 표본이 최소 수 이상이고 위험 표본은 없음 |
| 빨강: 위험 관측 | 지면 두께보다 높은 표본 또는 지면 아래 허용값보다 낮은 표본이 최소 수 이상 |
| 회색: 미관측/불확실 | 유효 깊이 없음, ROI 밖, 근거 부족, 또는 최소 수에 못 미치는 위험 표본이 있음 |

최대 높이를 넘는 표본은 제외한다. 같은 셀에 바닥과 충분한 위험 근거가 함께 있으면
위험이 우선한다. 위험 표본이 하나만 있어도 초록색으로 판정하지 않는다.
보이지 않는 공간, 광선 앞쪽과 장애물 뒤쪽을 초록색으로 채우지 않는다.
단, 실제 바닥이 별도로 관측된 셀은 차량과의 연결성 여부와 관계없이 초록색이 될 수 있다.
프레임 간 누적, 메모리, 팽창, 군집 연결은 수행하지 않는다. 작은 내부 구멍 보정은 아래 설정으로 선택한다.
`ground.enabled`는 포인트 하한에만 적용되며 바닥 셀 분류 기준은 유지한다.

표시용 폭 눈금 **2.0m**는 원점의 폭 눈금에만 반영한다. 차체 길이, 조향 궤적, 장애물 여유 간격,
연결된 통로 폭과 바닥 경사는 아직 평가하지 않는다. 녹색을 차량 전체가 통과 가능하다는
뜻으로 사용하지 않는다. 지면 아래 표본은 실제 함몰과 깊이 오류를 구별하지 않는다.

## 주요 파라미터

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `height.max_m` | 1.0 | 표시/발행/셀 집계에 사용할 지면 기준 최대 높이 |
| `ground.distance_m` | 0.06 | 지면 제거 기본 두께 |
| `ground.distance_per_meter` | 0.01 | 수평거리 1m당 두께 증가 |
| `ground.max_distance_m` | 0.12 | 지면 제거 두께 상한 |
| `terrain.below_ground_m` | 0.30 | 지면 아래 허용 두께 |
| `terrain.cell_m` | 0.07 | 셀 크기, 0.02~0.25m |
| `terrain.min_ground_samples` | 2 | 셀을 바닥으로 판정할 최소 표본 수 |
| `terrain.min_obstacle_samples` | 8 | 셀을 위험으로 판정할 최소 표본 수 |
| `terrain.vehicle_width_m` | 2.0 | 폭 표시 기준 |
| `points.pixel_stride` | 1 | ROI 이미지 표본 간격 |
| `preview.show_terrain` | true | 바닥 분류 색상 표시 |
| `preview.show_projected_points` | true | XY 포인트 표시 |
| `preview.terrain_alpha` | 0.80 | 색상 불투명도, 0이면 RGB 원본 유지 |
| `preview.point_radius` | 2 | XY 포인트 반경(px) |

RGB 보간은 `preview.interpolation=nearest/linear/cubic/lanczos4`이며 장애물 좌표에는
영향을 주지 않는다. 샘플링을 줄이면 바닥 셀의 근거도 줄어 회색 영역이 늘 수 있다.
셀 크기·최소 표본 수를 함께 조절한다. 큰 지면 두께는 낮은 장애물을 바닥으로 처리할 수 있다.

## 토픽

| 토픽 | 형식 | 내용 |
|---|---|---|
| `/roi_lidar/obstacle_points` | PointCloud2 | 모든 높이 필터 통과 XY 포인트. x/y/z는 float32(m), z=0, `height`에 원래 지면 기준 Z(m), `rgb`에 빨간색. 화면 밖도 포함 |
| `/roi_lidar/terrain` | OccupancyGrid | -1=미관측/불확실, 0=관측 바닥, 100=위험 관측. X방향 열, Y방향 행. 원점=(0,-표시폭/2), resolution=셀 크기 |
| `/roi_lidar/scan` | LaserScan | 기존 각도별 최근접 반환점. 프리뷰와 새 포인트 토픽의 입력으로 쓰지 않음 |
| `/roi_lidar/bev_preview` | Image bgr8 | RGB BEV + 현재 XY 포인트/바닥 색상 |
| `/roi_lidar/roi_preview` | Image bgr8 | ROI 지정용 영상 |
| `/roi_lidar/status` | String | 측정/입력중단/오류, transient local |

`/obstacles` PoseArray와 `cluster.*`는 이전 기능의 선택적 호환 출력으로 남겼으며
기본 `cluster.enabled=false`다. 켜더라도 새 프리뷰의 XY 포인트는 묶지 않는다.
`preview.show_raw_points`는 예전 프리뷰 설정이며 새 XY 화면에서는
`preview.show_projected_points`를 사용한다.

RGB/depth가 모두 신선하지만 시각 차이가 `preview.max_sync_sec`를 넘으면 프리뷰의
깊이 overlay를 숨긴다. RGB가 없는 동안에는 신선한 깊이 결과를 단색 배경에 표시한다.
새 데이터 토픽은 RGB 시각과 무관하게 깊이 촬영 시각으로 발행한다.
깊이 입력 중단/오류 시 포인트는 비우고 셀은 미관측으로 초기화한다.
GUI가 없는 환경은 `gui:=false`로 영상/데이터 토픽을 사용한다.

## 조절값 저장

숫자 입력·ROI 변경은 원본 YAML에 자동 저장하지 않는다.

```bash
ros2 param dump /roi_lidar > roi_lidar_test.yaml
ros2 launch roi_lidar roi_lidar.launch.py config_file:="$(pwd)/roi_lidar_test.yaml"
```

## 검증

```bash
colcon test --packages-select roi_lidar --event-handlers console_direct+
colcon test-result --verbose
```

ROI/바닥 제거/거리/좌표축/기울기, 현재 프레임 표본의 보존, 미관측 셀, 바닥/위험 충돌,
함몰 근거, 프레임 초기화, 높이 상한과 XY 투영 및 색상을 테스트한다.
실제 장치의 FPS·오검출·시작 지면 측정과 ROS 전체 빌드/GUI 동작은 차량 PC에서 확인한다.

숫자 입력, 명시적 일괄 적용, 잘못된 입력 배치 거부와 외부 파라미터 동기화에 대한
기존 GUI 테스트 코드는 `test/controls_test.cpp`에 있다. 실제 데스크톱 창을 열므로 기본 테스트에서는 제외한다.
디스플레이가 있는 환경에서 `-DROI_LIDAR_GUI_TESTS=ON`으로 빌드하면 함께 실행할 수 있다.

로컬 C++ terrain·scan·preview·config 테스트와 GUI 입력 테스트를 확인했다.
설정 테스트는 설치된 DepthAI SDK와 연결했으며 로컬 Clang의 SDK 헤더 호환 옵션을 사용했다.
ROS 2 전체 노드 빌드 및 실제 OAK 카메라 동작은 차량 PC에서 별도 확인한다.

## 작은 내부 구멍 보정과 축소된 파라미터 창

기본 카메라 요청은 **30 FPS**이며 파라미터 창은 **900×590px** 두 열로 표시한다.
사진의 표시값을 반영해 `preview.terrain_alpha=0.8`, `preview.point_radius=2`,
`preview.width_m=1.2`, `preview.forward_m=3.0`을 기본값으로 사용한다.
투영점 표시 ON, confidence 200, LR ON/10, subpixel ON, dot 0.7, flood 0은 유지한다.

`terrain.fill_enabled` (기본 true)를 실행 중 0/1로 바꾸고 Enter/A/Apply로 적용한다.
`terrain.fill_max_cells` (기본 2, 1~16)는 8방향으로 연결된 회색 묶음의 최대 셀 수다.
`terrain.fill_clearance_cells` (기본 1, 0~4)는 위험 표본 주변 보정 금지 반경이다.
세 설정은 카메라 재시작 없이 다음 프레임부터 반영하며 YAML에 자동 저장하지 않는다.
현재 7cm 격자에서 2셀은 98cm² 면적이다. 셀 크기를 바꾸면 물리적 보정 면적도 바뀐다.

작고 관측 바닥으로 완전히 둘러싸인 묶음만 채운다. 지도 가장자리나 원거리 회색에
대각선으로라도 연결되면 제외한다. 한 개라도 위험 표본이 있는 셀, 위험 경계와
접한 구멍은 보정하지 않는다. 원래 관측만 기준으로 검사해 연쇄적으로 확장하지 않으며
과거 프레임을 누적하지 않는다. 이 보정은 주변 바닥에 근거한 추정이며 새로운 실측이 아니다.

`/roi_lidar/terrain`은 보정 결과, `/roi_lidar/terrain_raw`는 보정 전 판정이다.
보정 OFF이면 두 결과가 같다. 화면은 실측 바닥 초록, 보정 바닥 청록으로 구분하며
상단 `filled`는 해당 프레임에서 채운 셀 수다. Depth 중단 시 두 지도 모두 미관측으로
초기화한다. 장애물 포인트/스캔과 다른 패키지의 주행 제어에는 이 보정을 적용하지 않는다.
기존 본문의 무보간 설명은 `terrain_raw` 및 보정 OFF에 해당한다.

## VFH+ 프리뷰 연결

`roi_vfh_test/README.md`의 통합 launch에서 파라미터 창과 Start/Stop 프리뷰를 함께 실행한다.
`preview.publish_lane_bev=true`이면 같은 RGB의 깨끗한 120×300/1cm BEV를 `~/lane_bev`로 발행한다.
`motion.publish_imu=true`이면 같은 OAK 세션에서 gyro를 읽어 앞차축 좌표의 `~/imu`로 발행한다.
두 기능의 기본값은 false이며 활성화 변경은 카메라 재시작을 요구한다.
`~/terrain_for_planning`은 ROI 격자와 동일한 최소 위험 표본 수·색상을 사용하는 계획 입력이다.
위험 표본이 최소 수보다 적으면 초록으로 만들지 않고 회색으로 두며, 1표본만으로 별도 100을 강제하지 않는다.
화면과 기존 terrain/raw 토픽의 분류 의미는 유지한다.
`/roi_vfh_test/running=true` 수신 시 GUI 편집과 ROS 파라미터 쓰기를 모두 잠그고
`~/parameters_locked`로 ACK한다. Stop(false) 수신 후 다시 변경할 수 있다.

최신 스크린샷 기본값: 높이 상한 1.0m, 거리별 두께 증가 0.01m/m, 아래 허용 깊이 0.30m,
셀 0.07m, 장애물 표본 8개, 표시 폭 눈금 2.0m, 픽셀 간격 1, 카메라 60 FPS를 사용한다.
`terrain.vehicle_width_m`는 표시용이며 VFH+ 차량 검사 폭은 기존 차량 설정을 따른다.
이번 기본값 변경의 빌드·테스트는 사용자 요청에 따라 실행하지 않았다.

## 프레임별 RANSAC·고립점 제거 — 실행창 2페이지

이번 변경은 사용자 요청에 따라 **빌드·테스트를 실행하지 않았다.**
900×590px 설정창의 **Next >** 또는 **]**로 2페이지를 연다. 모든 항목은 Enter/A/Apply로
일괄 적용되며 카메라 재시작이 필요 없다. Prev/[로 되돌아갈 수 있고 Tab도 페이지를 넘어간다.
VFH+ 실행 중에는 기존 GUI/ROS 쓰기 잠금이 적용된다. 페이지 이동과 값 확인은 가능하다.

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `ground.ransac_enabled` | false | 각 depth 프레임에서 평면 재추정 |
| `ground.use_startup_plane` | true | RANSAC OFF일 때 시작 측정 평면 재사용 |
| `ground.ransac_distance_m` | 0.06m | 대표 평면을 찾을 때만 쓰는 inlier 허용 오차 |
| `obstacle.min_height_m` | 0.06m | 대표 평면 위 장애물 최소 높이; 이 값을 초과한 점만 위쪽 장애물 후보 |
| `ground.ransac_max_range_m` | 3.0m | 수평거리 기준 평면 추정·적용 상한 |
| `ground.ransac_max_tilt_deg` | 10° | 시작 지면의 위 방향 대비 최대 기울기 |
| `ground.ransac_max_offset_m` | 0.08m | 시작 지면 원점에서 평면까지 최대 수직 거리 |
| `ground.ransac_min_inlier_ratio` | 0.50 | 샘플 중 필요한 최소 평면 지지 비율 |
| `ground.ransac_max_iterations` | 60 | RANSAC 반복 상한; 충분한 지지 비율이면 조기 종료 |
| `ground.ransac_max_samples` | 700 | 추정에 사용하는 최대 샘플 수 |
| `outlier.enabled` | true | 고립된 비지면 표본 제거 ON/OFF |
| `outlier.radius_m` | 0.08m | 원래 차량 XYZ에서 이웃을 찾는 3D 반경 |
| `outlier.min_neighbors` | 8 | 자신을 제외한 비지면 이웃의 최소 수 |

RANSAC은 범위·시작 지면 주변 후보에서 reservoir sampling으로 점 수를 제한한다.
평면 법선·높이·inlier 비율뿐 아니라 지지점의 전방 25cm/좌우 20cm 이상 분포도 요구한다.
작은 2×2 최소제곱으로 평면을 정련하며 큰 PCL 점군이나 법선 계산을 추가하지 않는다.
실패하면 이전 프레임 평면을 무한 유지하지 않고 시작 지면 기준 거리별 두께로 복귀한다.
성공해도 적용 거리 밖에는 평면을 외삽하지 않는다.

대표 평면은 기존처럼 하나만 추정하며, 성공 범위의 장애물 높이 기준은 별도의
`obstacle.min_height_m`를 사용한다. 기본 5cm 이하의 요철은 위쪽 장애물에서 제외한다.
이 기준은 빨간 포인트·스캔·terrain·고립점 후보 선정에 공통 적용된다.
`ground.ransac_distance_m`는 평면 추정에만 영향을 주며 장애물 높이를 직접 바꾸지 않는다.
두 값은 2페이지에서 각각 조절하고 Apply로 다음 프레임에 반영한다.
기존 `ground.distance_m`,
`ground.distance_per_meter`, `ground.max_distance_m`는 OFF/실패/범위 밖에서 사용하는 값이다.
`terrain.below_ground_m`의 아래쪽 허용 깊이는 기존 설정을 유지한다. `ground.enabled`는
기존처럼 포인트의 지면 제거를 제어하며 terrain의 바닥 분류 자체를 끄지는 않는다.
RANSAC은 분류용 높이를 바꾸며 XY와 RGB BEV의 시작 좌표 변환을 재설정하지 않는다.
PointCloud2의 `height`와 스캔의 높이 근거도 선택된 지면 기준 높이를 사용한다.

고립점 제거는 먼저 비지면 후보를 추리고, 3D 공간 해시의 인접 27개 버킷만 조회한다.
지면 표본을 이웃으로 세지 않으며 최소 수를 찾으면 탐색을 끝낸다. 이웃 판정은 같은 프레임
전체 후보를 기준으로 하므로 삭제 순서에 따라 연쇄적으로 점이 사라지지 않는다.
거부된 점은 빨간 점·스캔·군집·terrain 위험 표본에서 제외한다. 그 셀에 별도의 유효 바닥이
없다면 미관측으로 남기고 작은 구멍 보정으로 바로 바닥을 만들지 않는다.
너무 큰 반경/이웃 수는 작은 콘이나 먼 물체의 표본도 지울 수 있으므로 조절 가능하게 두었다.

점 좌표 복원과 거리/각도 계산은 한 번 수행하고 위치·bin을 재사용한다. 장애물 인덱스도
캐시하여 프리뷰와 점군 발행이 전체 바닥 점을 반복 분류하지 않게 한다.
HUD/로그의 `plane RANSAC/FALLBACK/FIXED`, `inliers/samples`, `isolated`/`outlier_removed`로
적용 상태를 확인한다. 처리 후 입력 만료 시간을 넘은 프레임은 발행하지 않는다.


## 시작 지면 측정 프리뷰

ROI 실행도 BEV/auto_control과 동일한 `oak_startup::measureOakStartupExtrinsics`를 사용한다.
launch는 BEV 설정의 `measurement_roi_width=456`, `measurement_roi_height=228`,
`measurement_roi_vertical_offset_px=80` 등을 가져온다. 시작할 때 차량을 평지에 정지시키고
빨간 ROI 안에 전방 노면이 보이도록 한다. ROI 안의 콘/벽/차체는 측정에서 피한다.

`measurement_roi_preview_enabled=true`와 `preview.gui=true`이면 공유 측정기가 생성한
동일한 회색 depth 영상과 빨간 ROI를 `OAK startup ground-plane ROI` 창에 표시한다.
원본 측정기의 ROI 좌표/영상 크기를 그대로 사용하며, 완료 또는 실패하면 창을 자동으로 닫고
일반 terrain·depth ROI·파라미터 창으로 전환한다. GUI는 ROS/main thread에서만 처리하고,
작업 스레드는 최대 10Hz로 최신 프레임 한 장만 전달한다. headless/GUI OFF에서는 표시하지 않는다.
이후 프레임별 RANSAC은 별도의 보정이며 시작 카메라 좌표·BEV 자세 측정을 대체하지 않는다.
이번 변경은 사용자 요청에 따라 빌드·테스트를 실행하지 않았다.

`preview.show_bev_window`(기본 true)는 별도 RGB BEV창 표시만 제어한다. VFH launch는 false로 설정하여 VFH 창에 BEV를 통합하며, ROI 선택창과 파라미터 창·센서 발행은 유지한다.

IMU ON일 때는 카메라 세션별 전용 수신 스레드를 사용한다. depth RANSAC/고립점 처리 중에도
IMU 배치를 계속 수신하며 원래 측정 시각을 보존한다. 장치를 닫기 전에 수신 스레드를 종료한다.
2026-11-06 16:32 첨부 이미지의 파라미터를 일반 YAML 및 코드 기본값에 반영했다.
terrain 셀 0.07m/최소 장애물 표본 8개, RANSAC 거리 0.06m/표본 700개,
최소 장애물 높이 0.06m, 고립점 반경 0.08m/이웃 8개를 사용한다. 빌드·테스트는 미진행이다.


기본 모드는 시작 평면 재사용이다. 시작 측정 결과로 좌표를 변환하면 지면은 차량 좌표의
Z=0이다. RANSAC이 꺼져 있고 `ground.use_startup_plane=true`이면 기존 적용 반경
(`ground.ransac_max_range_m`, 기본 3m) 안에서 Z=0 기준 `obstacle.min_height_m`를 사용한다.
반경 밖은 기존 거리별 기준을 적용한다. 두 옵션이 모두 꺼져 있으면 전 범위에서 거리별 기준을 적용한다.
경사나 카메라 자세가 주행 중 달라지는 환경에서는 `ground.ransac_enabled=true`로 프레임별
보정을 사용할 수 있다. HUD와 로그에서 시작 평면 재사용 모드는 `STARTUP`으로 표시한다.

ROI 로그의 `scan`은 투영·지면·고립점·스캔 처리 평균 시간이고 `grid_cluster`는
격자·군집 처리 평균 시간이다. `host`와 `age`도 함께 비교한다. ROI BEV창이 숨겨져 있고
구독자가 없으면 해당 영상 합성을 건너뛴다. 차선 입력 BEV와 계획 격자는 계속 발행한다.
