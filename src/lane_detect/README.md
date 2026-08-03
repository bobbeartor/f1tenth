# lane_detect

카메라 원근 영상에서 개선된 차선 마스크를 생성하는 ROS 2 C++ 패키지다.
선택적으로 BEV 영상의 좌우 경계를 추적하고 2차 곡선 중심 경로도 생성할 수
있지만, 현재 `auto_control` 통합 실행은 BEV 없이 `/lane_mask`만 사용한다.

## 원근 마스크 단계

1. NV12 Y 평면을 grayscale 입력으로 직접 사용
2. white top-hat으로 가늘고 밝은 차선 후보 추출
3. 후보 양쪽에 어두운 노면이 존재하는 bilateral gate 적용
4. 카메라 높이·피치 기반 원거리 영역 및 범퍼 영역 제거
5. connected-component 면적·형상 필터 적용

입력은 `/camera/image_rect`의 `nv12`, 출력은 `/lane_mask`의 `mono8`이다.
`/camera/image_lane`의 NV12 복사본은 BEV용이며 non-BEV 실행에서는 끈다.

```bash
ros2 launch lane_detect lane_detect.launch.py
ros2 run rqt_image_view rqt_image_view
```

`rqt_image_view`에서 `/lane_mask`를 선택해 확인한다.

## BEV 검출 단계

`bev_lane_detector.cpp`는 BEV의 각 행에서 차선 폭에 맞는 흰색 run만 남기고,
가까운 곳부터 좌우 경계를 연속 추적한다. 한쪽 차선만 보일 때는 설정된 차로
폭으로 중심을 복원하며, 이상치를 반복 제거한 뒤 2차 다항식으로 중심선을
근사한다. 이 단계는 현재 `vehicle_launcher/auto_drive.launch.py`에서는
실행하지 않는다.
