# lane_detect

카메라 원근 영상에서 개선된 차선 마스크를 생성하는 ROS 2 C++ 패키지다.

## 원근 마스크 단계

1. NV12 Y 평면을 grayscale 입력으로 직접 사용
2. white top-hat으로 가늘고 밝은 차선 후보 추출
3. 후보 양쪽에 어두운 노면이 존재하는 bilateral gate 적용
4. 카메라 높이·피치 기반 원거리 영역 및 범퍼 영역 제거
5. connected-component 면적·형상 필터 적용

입력은 `/camera/image_rect`의 `nv12`, 출력은 `/lane_mask`의 `mono8`이다.

```bash
ros2 launch lane_detect lane_detect.launch.py
ros2 run rqt_image_view rqt_image_view
```

`rqt_image_view`에서 `/lane_mask`를 선택해 확인한다.
