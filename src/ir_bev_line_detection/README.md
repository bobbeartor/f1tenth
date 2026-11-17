# ir_bev_line_detection

터널 진입부에 차량을 배치하고, 이미 IR 모드인 상태에서 출구까지 저속 차선 주행을 확인하는
독립 테스트 패키지입니다. TCN 판단, RGB 차선 전환, 출구 자동 종료는 포함하지 않습니다.

## 구성

```text
ir_camera_driver: LEFT/CAM_B → 보정·IMU 안정화·IR BEV (mono8, 120×300)
  → mono_bev_adapter: 밝기를 유지하며 동일한 B/G/R 3채널로 복제
  → 기존 line_detactor: IR SCNN TensorRT 추론 → 기존 차선 후처리·중앙선
  → 기존 auto_control: 기존 Stanley 조향·종방향 PID·영상/경로/ERPM 유실 정지
  → 기존 auto_handover_node → 기존 vesc_bridge
```

중앙선과 주행 알고리즘을 복사하거나 별도로 구현하지 않습니다. 설치된 RGB 주행용 YAML을
읽고 이 패키지의 `config/ir_bev_test.yaml` 설정만 덮어씁니다. BEV는 앞차축 기준
전방 0~3 m, 좌우 ±0.6 m, 1 cm/pixel입니다. 검출기와 제어기에도 같은 배율을 적용합니다.
기존 차선·중앙선 미리보기 위에 IR BEV를 표시합니다.

주행 설정: **30 FPS, 노출 30000 μs, ISO 800, LEFT 한쪽, dot OFF, flood 0.5,
기본 목표 속도 0.3 m/s**. `speed_mps`로 목표 속도를 지정하며 0.3 m/s 상한은 없습니다.
0보다 큰 유한한 값을 사용합니다. 런타임 RGB는 사용하지 않습니다. 장애물 회피와 신호등 정지는 제외합니다.
실제 속도는 기존 ERPM 피드백 PID가 제어하며, 지정한 목표 속도는 물리적인 속도 보장이 아닙니다.

기존 IR 드라이버의 정지 상태 시작 보정과 IMU 안정화는 유지합니다. 시작 보정은 일시적으로
양쪽 카메라와 dot projector를 사용해 바닥 평면·높이를 측정하고, 이후 LEFT/dot OFF 설정으로
주행합니다. 따라서 시작 시 차량을 움직이지 않고 바닥이 보이는 상태로 두어야 합니다.
카메라 장착 위치·차선 폭 등 차량별 값은 기존 생산용 YAML의 값을 상속합니다.

## 모델

`stereo_bev/runs/stereo_only_v1/best.pt`의 **epoch 9** 가중치를 배포용 FP32 ONNX로
변환한 `line_detactor/models/ir_stereo_bev_120x300.onnx`를 공용으로 사용합니다. RGB 모델로 대체하지 않습니다.
입력은 RGB float32 `[1,3,300,120]`, 값 범위 [0,1], 출력은 left/right/stop_line logits입니다.
IR 학습의 stop_line 채널은 감독되지 않았으므로 정지선 거리 추정과 신호등 정지는 사용하지 않습니다.
정지선 표시 임계값은 1.0으로 설정합니다.

`line_detactor/models/ir_stereo_bev_120x300.json`에 체크포인트/ONNX SHA256과 CPU 변환 검증 결과가 있습니다.
검증은 합성 입력 2개와 IR validation 이미지 8개를 사용했습니다. 최종 test 세션은 사용하지 않았습니다.
Jetson에서는 기존 line_detactor가 TensorRT 엔진을 생성합니다. 첫 실행에는 엔진 생성 시간이 필요합니다.

재변환은 저장소 루트에서 다음과 같이 수행합니다(학습·ROS 실행 없음).

```bash
python src/ir_bev_line_detection/tools/export_ir_model.py \
  --checkpoint /path/to/stereo_only_v1/best.pt \
  --validation-images /path/to/IR/validation/images
```

## 차량에서 실행

ROS 환경에서 이 패키지와 의존 패키지를 빌드·source한 뒤 실행합니다.
이 개발 환경에서는 ROS 빌드·실행 검사를 수행하지 않았습니다.

```bash
ros2 launch ir_bev_line_detection ir_bev_line_detection.launch.py
```

**기존 auto_drive와 동일하게, 유효한 중앙선·VESC 연결·최신 ERPM이 준비되면 자동으로 출발합니다.**
별도 `auto_drive`, `manual_drive`, 카메라 드라이버를 동시에 실행하지 않습니다.
이 launch가 카메라와 VESC 연결을 각각 하나씩 소유합니다.

출발을 보류하려면:

```bash
ros2 launch ir_bev_line_detection ir_bev_line_detection.launch.py auto_enabled:=false
# 준비 후 별도 터미널에서 활성화
ros2 topic pub --once /auto/enabled std_msgs/msg/Bool "data: true"
```

출구에서는 기존 **X 버튼**으로 자동주행을 종료합니다. 기존 전환 로직에 따라
제동 → 최신 ERPM으로 정지 확인 → 조작 중립 확인 후 수동 모드가 됩니다.
이 전환은 단방향이므로 다음 자동주행 시험은 launch를 다시 시작합니다.
`/auto/enabled=false` 명령은 기존 제어기의 duty 0 요청이며 X 버튼 제동 절차와는 다릅니다.

```bash
# 차선 추론·미리보기만 확인: VESC/조이스틱/수동 전환 노드를 시작하지 않음
ros2 launch ir_bev_line_detection ir_bev_line_detection.launch.py control_mode:=monitor_only
# 사용자 지정 목표 속도(m/s). 0보다 큰 유한한 값, 기본값 0.3
ros2 launch ir_bev_line_detection ir_bev_line_detection.launch.py speed_mps:=1.0
```

추가 launch 인자: `vesc_port`(기본 `/dev/ttyTHS1`), `vehicle_namespace`(기본 `autopilot03`),
`controller_name_contains`(기본 `8BitDo`), `preview_enabled`, `params_file`, `model_path`,
`depthai_library_dir`입니다. `model_path`를 바꿀 때도 동일한 입출력 규격의 IR 모델이어야 합니다.

확인할 토픽: `/camera/image_bev_ir`, `/ir_bev_line_detection/result`,
`/ir_bev_line_detection/result_image`, `/auto/target_speed`, `/auto/current_speed`,
`/autopilot03/drive/mode`. 기존 주행 로그는 `driving_logs/ir_bev_line_detection`에 저장됩니다.
카메라 영상 타임스탬프는 어댑터에서 그대로 전달하므로 기존 지연·유실 정지 조건이 유지됩니다.

## 개발 환경의 오프라인 검사

```bash
PYTHONPATH=src/ir_bev_line_detection python -m unittest discover \
  -s src/ir_bev_line_detection/test -p test_offline.py -v
```

영상 밝기·행 패딩 보존, 잘못된 입력 거부, 기존 제어/중앙선 설정 상속과 시험 속도 범위를 확인합니다.
이 검사는 ROS를 가져오거나 차량 주행을 실행하지 않습니다. 실제 터널 통과 여부는 차량에서 확인해야 합니다.
