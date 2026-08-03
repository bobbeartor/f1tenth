# Jetson Resource Monitor

Jetson 보드 전체 부하와 ROS 2 실행 프로세스별 CPU/RAM 사용량을 1초마다
수집하는 독립 패키지입니다. 별도 `pip` 패키지 없이 Linux `/proc`, Jetson
sysfs, NVIDIA `tegrastats`를 사용합니다.

## 빌드와 실행

```bash
cd ~/f1tenth
colcon build --symlink-install --packages-select jetson_resource_monitor
source install/setup.bash
ros2 launch jetson_resource_monitor jetson_resource_monitor.launch.py
```

실행하면 콘솔에 다음 형태의 표가 반복 출력됩니다.

```text
[Jetson] CPU 34.2% | RAM 3120/7620MB (40.9%) | GPU 71.0% | Power 12890mW
    PID     CPU%   RAM(MB)   THR  PACKAGE                 NODE/PROCESS
  15422    186.3     742.1    18  bev_processor           /bev_processor
  15381     42.7     218.5    12  camera_driver           /camera_driver
```

프로세스 CPU가 `100%`이면 CPU 코어 하나를 계속 사용한다는 뜻입니다. 예를
들어 `186%`는 약 1.86개 코어에 해당합니다. 첫 번째 샘플의 CPU 값은 델타를
구할 이전 샘플이 없어 0으로 표시됩니다.

## ROS 토픽

모든 데이터는 JSON 문자열(`std_msgs/msg/String`)로도 발행합니다.

```bash
ros2 topic echo /jetson_monitor/system
ros2 topic echo /jetson_monitor/processes
ros2 topic echo /jetson_monitor/metrics
```

- `/jetson_monitor/system`: 전체 CPU, RAM, swap, GPU, 온도, 전력
- `/jetson_monitor/processes`: ROS 2 프로세스별 CPU, RSS 메모리, 스레드
- `/jetson_monitor/metrics`: 위 데이터와 현재 ROS graph 노드를 합친 데이터

JSON의 프로세스 항목에는 `cpu_percent`와 함께 사용 중인 CPU 코어 수에 가까운
`cpu_cores`도 포함됩니다.

## 설정

기본값은 `config/monitor.yaml`에 있습니다. 실행 중인 카메라/BEV 프로세스만
보려면 다음처럼 정규식 필터를 줄 수 있습니다.

```bash
ros2 run jetson_resource_monitor resource_monitor_node --ros-args \
  -p process_filter:='camera|bev' \
  -p interval_sec:=0.5
```

CSV 기록도 파라미터 하나로 켤 수 있습니다.

```bash
ros2 run jetson_resource_monitor resource_monitor_node --ros-args \
  -p csv_path:=/tmp/jetson_metrics.csv
```

모든 Linux 프로세스를 조사하려면
`include_non_ros_processes:=true`를 사용합니다. 이 경우 출력량이 많으므로
`process_filter`도 함께 쓰는 편이 좋습니다.

## 측정 범위와 한계

- GPU 사용률, 온도, 전력은 보드 전체 값입니다. Jetson의 `tegrastats`는
  일반적으로 프로세스별 GPU 사용률을 제공하지 않습니다.
- ROS 2 graph 표준 API는 node 이름과 Linux PID의 대응 관계를 제공하지
  않습니다. 따라서 PID별 수치는 정확하지만 `node_name`은 실행 경로와
  `__node`, `__ns` remap 인자로 추정합니다.
- 한 component container 안에 여러 composable node가 있으면 이들을
  개별 분리할 수 없고 container 프로세스 하나의 합산 수치로 표시됩니다.
- CUDA 커널별 실행 시간이나 GPU 연산 병목 분석이 필요하면 Nsight Systems를
  별도로 사용해야 합니다. 이 노드는 주행 중 지속 모니터링에 맞춘 도구입니다.
