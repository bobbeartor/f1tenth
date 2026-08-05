# auto_control

BEV 변환 없이 C++ 차선 마스크에서 차로 중심을 추정하고 F1TENTH 차량을
제어하는 ROS 2 패키지다. 통합 실행에서는 `lane_detect`가 카메라
영상으로부터 생성한 `/lane_mask`를 입력으로 사용한다.

## 동작 방식

1. `/lane_mask`의 `mono8` 차선 마스크를 받는다.
2. 고정 사다리꼴 ROI로 자르지 않고 추출된 마스크 전체 폭을 사용한다.
3. 네 개 수평 구간에서 좌우 차선 경계 쌍을 고른다. 중앙의 짧은 흰 표시나
   도로 밖 흰 물체를 경계로 선택하지 않도록 예상 차로 폭과 중심 위치를
   함께 평가한다.
4. 양쪽 경계가 보일 때 높이별 차로 폭을 계속 갱신한다. 한쪽 경계만
   검출되면 최근 측정 폭(아직 측정하지 못했으면 설정 폭)의 절반만큼
   이동해 중앙선을 복원한다.
5. 가까운 중심은 횡오차, 먼 중심의 화면 중앙 편차는 진행방향·선행 오차로
   사용한다. 원근 수렴 때문에 `far-near`를 방향 오차로 직접 쓰지 않는다.
6. 조향이 클거나 차선 신뢰도가 낮으면 duty를 줄이고, 차선 소실·영상
   타임아웃·VESC 연결 해제 시 duty 0과 중앙 조향을 보낸다.

`warpPerspective()`나 호모그래피는 사용하지 않는다. 오차와 차로 폭은 실제
미터가 아닌 영상 폭에 대한 정규화 값이다.

## 단독 실행

먼저 카메라, `lane_detect`, VESC 노드를 별도로 실행한 뒤,
구동하지 않는 dry-run으로 결과를 확인한다. 일반적으로는 아래의
`vehicle_launcher` 통합 실행을 사용하는 편이 간단하다.

```bash
ros2 launch auto_control perspective_lane.launch.py \
  drive_enabled:=false publish_debug:=true
```

통합 실행:

```bash
ros2 launch vehicle_launcher auto_drive.launch.py \
  drive_enabled:=false publish_debug:=true
```

확인할 토픽:

```bash
ros2 topic echo /auto/status
ros2 run rqt_image_view rqt_image_view /auto/lane_debug
ros2 run rqt_image_view rqt_image_view /auto/lane_mask
```

디버그 영상에서 파랑/빨강 점은 선택한 좌/우 경계, 노란 점과 선은 추정한
차로 중심이다. 모든 scan 높이에서 노란 점이 실제 차로 중앙에 놓이는지
확인한다.

바퀴를 지면에서 띄우고 조향 방향과 정지 동작을 확인한 뒤에만 구동한다.

```bash
ros2 launch auto_control perspective_lane.launch.py drive_enabled:=true
```

수동 주행 launch나 다른 `/vesc/duty`, `/vesc/servo_position` 발행 노드와
동시에 실행하면 안 된다.

## 차선 마스크 입력

기본 입력은 `lane_detect`의 `/lane_mask`다. 다른 `mono8` 마스크를
사용할 때는 토픽만 교체한다.

```bash
ros2 launch auto_control perspective_lane.launch.py \
  image_topic:=/custom/lane_mask drive_enabled:=false \
  force_lane_mask_input:=true
```

## 우선 튜닝할 파라미터

설정은 `config/perspective_lane.yaml`에 있다.

- `scan_y_ratios`: 전체 마스크에서 차선 위치를 측정할 네 개 높이다.
- `lane_width_far_ratio`, `lane_width_near_ratio`: 가장 먼/가까운 측정
  높이에서 보이는 좌우 경계 간격을 영상 폭으로 나눈 값이다. 한쪽 경계만
  검출될 때 중앙선 복원에도 사용된다.
- C++ 마스크 품질은 `lane_detect/config/lane_mask.yaml`에서 조정한다.
- `lateral_gain`: 차로 중앙에서 벗어난 오차에 대한 조향 반응이다.
- `heading_gain`: 전방 차로가 휘는 방향에 대한 선행 조향 반응이다.
- `base_duty`: 직선 최대 duty다. 초기에는 현재 기본값보다 올리지 않는다.
- `minimum_duty`: 정상 차선 추종 중 정지 마찰을 넘기 위한 최소 duty다.
  차선 소실이나 안전 정지 상태에는 이 값과 무관하게 duty 0을 출력한다.
- `steering_slowdown`: 조향량에 따른 감속 비율이다.

이 노드는 차선 추종만 담당한다. LiDAR 장애물 검출과 회피는 아직 명령에
결합하지 않았으므로 장애물이 없는 폐쇄 트랙에서만 사용해야 한다.
