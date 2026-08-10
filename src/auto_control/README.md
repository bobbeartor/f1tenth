# auto_control

`/lane_mask`에서 곡선 차로 중앙 경로를 만들고 차량을 60 Hz로 조향하는
ROS 2 패키지다. VESC 명령은 기본적으로 비활성화되어 있다.

## 처리 방식

1. 입력 마스크를 고정 좌표계 `160x100`으로 정규화한다.
2. `y=60..98`(양 끝 포함) 밖의 모든 픽셀을 제거한다.
3. ROI의 각 행에서 좌우 경계 후보를 추적한다.
4. 좌우 경계를 각각 2차식 `x(y)=ay^2+by+c`로 강건하게 근사한다.
5. 양쪽이 보이면 두 곡선의 평균으로 중앙선을 만든다.
6. 한쪽만 보이면 최근에 측정한 차로 폭(없으면 설정 폭)의 절반만큼 경계
   곡선을 평행 이동해 중앙선을 복원한다.
7. 제어기는 경계가 아니라 중앙선의 근거리 오차와 선행점 오차만 사용한다.

`processing_width`, `processing_height`, ROI, 차로 폭, 조향 게인은
`config/centerline.yaml`에서 조정할 수 있다.

## 실행

먼저 dry-run과 디버그 영상으로 중앙선 및 조향 방향을 확인한다.

```bash
ros2 launch auto_control centerline.launch.py \
  drive_enabled:=false publish_debug:=true
```

확인 토픽:

```bash
ros2 topic echo /auto/status
ros2 run rqt_image_view rqt_image_view /auto/lane_debug
ros2 run rqt_image_view rqt_image_view /auto/lane_mask
```

바퀴를 지면에서 띄운 상태로 차선 소실 시 duty가 0이 되는지 확인한 다음에만
`drive_enabled:=true`를 사용한다. 수동 제어 등 다른 VESC 명령 발행 노드와
동시에 실행하면 안 된다.
