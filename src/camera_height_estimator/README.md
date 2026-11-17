# camera_height_estimator

OAK-D Pro W의 높이를 한 번 측정하는 독립 ROS 2 노드다. 기존
`camera_driver`, `/camera/image`, `/camera/imu` 토픽을 사용하거나
수정하지 않는다.

## 독립성

이 패키지의 직접 의존성은 `rclcpp`, `depthai`뿐이다.
실행할 때 자체 DepthAI 파이프라인으로 아래 기능만 잠깐 사용한다.

- CAM_B/C 모노 카메라와 `StereoDepth`
- 공장 캘리브레이션을 사용한 CAM_A/RGB 광학 좌표계 깊이 정렬
- OAK 내부 IMU의 raw accelerometer

CAM_A RGB 영상 스트림은 생성하지 않는다. 높이 측정이 끝나거나 제한
시간을 넘기면 DepthAI 파이프라인과 USB 연결을 즉시 닫는다. 결과는
콘솔에만 출력하고 ROS 토픽으로 발행하지 않는다.

OAK 장치는 한 프로세스가 배타적으로 연다. 따라서 측정하는 짧은 시간에는
`camera_driver`나 `depthai_ros_driver`를 동시에 실행할 수 없다. 측정 완료
로그에서 USB 해제를 확인한 뒤 다른 카메라 노드를 실행하면 된다.

## 측정 원리

1. 정지 상태에서 OAK IMU 가속도 10개로 중력의 반대 방향인 위쪽 축을
   구한다.
2. 공장 IMU→RGB 회전행렬로 이 축을 RGB 광학 좌표계로 변환한다.
3. RGB 좌표계에 정렬된 `640x400` 깊이 영상의 중앙 `10x10` 픽셀을
   3차원 점으로 복원한다.
4. 각 3차원 점을 위쪽 축에 투영하여 RGB 카메라 광학 중심의 수직 높이를
   구한다.
5. 공간 MAD 검사를 통과한 연속 깊이 프레임 5개의 평균을 최종 높이로
   사용한다.

깊이값 `z`는 광축 방향 거리이므로 다음 식을 사용한다.

```text
P = ((u - cx) * z / fx, (v - cy) * z / fy, z)
height = -dot(up_camera, P)
```

IMU를 사용하지 않으면 광축 방향 깊이를 수직 높이로 바꿀 수 없으므로,
이 노드는 OAK 내부 IMU를 직접 읽는다. 외부 IMU 노드나 ROS IMU 토픽에는
의존하지 않는다.

중앙 `10x10` 영역은 반드시 평평한 노면을 보고 있어야 하며 차량은 측정
순간 정지해 있어야 한다. 장애물, 연석, 차량 리프트 등의 물체가 중앙
영역을 가리면 잘못된 높이를 구하거나 검증 단계에서 거부될 수 있다.

## 빌드

```bash
cd ~/Desktop/f1tenth_test0724/f1tenth_project_repo
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select camera_height_estimator \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

## 실행

먼저 OAK를 사용하는 다른 프로세스를 종료한 상태에서 실행한다.

```bash
ros2 launch camera_height_estimator camera_height_estimator.launch.py
```

성공하면 다음 형태의 로그가 출력된다.

```text
CAMERA_HEIGHT_RESULT_M=0.2031
OAK pipeline and USB connection released.
```

측정 후에는 OAK 연결이 해제되므로 별도 터미널에서 기존 카메라 노드를
실행해도 된다. 출력된 값은 다른 노드로 전달되지 않는다. 높이 측정 노드는
`Ctrl+C`로 종료하면 된다.
