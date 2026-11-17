# manual_capture

OAK-D Pro W 한 대로 **라벨링용 이미지**를 모으는 독립 캡처 GUI 패키지다.
수동주행(`vehicle_bringup/manual_drive_with_dynamics.launch.py`)을 같은 launch에서 함께 띄운다.

- 독립 패키지: BEV 기하(`bev_processor`), 지면 측정(`oak_startup`), CUDA 중앙 재투영
  (`ir_camera_driver`) 코드를 이 패키지 안에 복사해 쓴다. 이 패키지를 지워도 다른 패키지에는
  영향이 없다. (실행 시 수동주행 launch만 `vehicle_bringup`을 참조한다.)
- OAK는 한 프로세스만 열 수 있으므로 `bev_processor`, `camera_driver`, `ir_camera_driver`와
  동시에 실행하지 않는다.

## 실행

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch manual_capture manual_capture.launch.py \
  vehicle_namespace:=autopilot03 \
  vesc_port:=/dev/ttyTHS1 \
  controller_name_contains:=8BitDo \
  input_mode:=slcan \
  slcan_channel:=/dev/ttyACM0 \
  slcan_bitrate:=500000 \
  can_controller_id:=112
```

| launch 인자 | 기본값 | 설명 |
|---|---|---|
| `capture_root` | `<워크스페이스 루트>/captures` | 저장 위치 (`install/`이 있는 레포 최상위) |
| `capture_joy_topic` | `/<vehicle_namespace>/joy` | A 버튼을 읽을 토픽 |
| `manual_drive_enabled` | `true` | `false`면 캡처 GUI만 실행 |
| `params_file` | `config/manual_capture.yaml` | 카메라/BEV/캡처 기본값 |
| 나머지 | — | `manual_drive_with_dynamics.launch.py`로 그대로 전달 |

시작하면 차량을 평지에 정지시킨 상태에서 지면 측정(약 5~10초)을 먼저 한다.
측정에 실패하면 config의 기본 자세로 BEV를 만들고, 나중에 Refit으로 다시 측정한다.

## 화면

```
┌ 상단: 장치·모드·ERPM │ 단일/자동 · 캡처 FPS · [캡처 시작] [캡처 중단] ┐
├────────────────┬────────────────┬─────────┬──────────────────────┤
│ ① 컬러          │ ② 중앙정렬      │ ⑤ 컬러   │ 동시 캡처 대상 체크박스 │
├────────────────┼────────────────┤   BEV   │ (화면별 누적 장수)      │
│ ③ 스테레오 왼쪽  │ ④ 스테레오 오른쪽 │ ⑥ 왼쪽   ├──────────────────────┤
│                │                │  렌즈 BEV│ 탭: 컬러/스테레오/BEV/저장 │
└────────────────┴────────────────┴─────────┴──────────────────────┘
  하단: REC·경과시간 │ 총 장수 │ 저장 큐·드롭 │ 디스크 여유 │ 실제 FPS
```

- 화면마다 `프리뷰` on/off, `이 화면만 캡처`, 프리뷰 FPS, 실행 후 누적 캡처 장수.
- 초기 상태: 컬러 프리뷰만 ON, IR flood/도트 모두 0.

## 캡처 규칙

- **동시 캡처 대상**(오른쪽 위 체크박스): 프리뷰가 켜진 화면만 체크할 수 있고,
  프리뷰를 켜면 자동 체크된다. A 버튼과 `캡처 시작`은 체크된 화면을 한 번에 저장한다.
- **단일 모드**: A / `캡처 시작` 한 번에 1세트.
- **자동 모드**: A로 시작, 다시 A로 중단 (`캡처 시작`/`캡처 중단` 버튼도 동일). 캡처 FPS 설정.
  자동 캡처 중에는 `이 화면만 캡처`와 모드 변경이 금지된다.
- **이 화면만 캡처**: 체크 여부와 상관없이 그 화면 1장.

## 실시간 / 재시작 설정

| 구분 | 항목 |
|---|---|
| 실시간 | 컬러 노출·ISO·자동노출, 스테레오 L/R 노출·ISO(연동 또는 개별), IR flood, 도트 세기 |
| ⟳ 재시작(1~3초) | 컬러 해상도·FPS·포맷(NV12/RGB888), 스테레오 해상도·FPS, 왼쪽 렌즈 BEV 모드 |

- 노출 상한은 1/센서FPS(30fps → 33,333µs).
- **도트 프로젝터는 자동**: 중앙정렬 프리뷰가 켜졌을 때(disparity 필요)와 Refit 중에만 켠다.
  이때 L/R/중앙 영상에는 도트가 찍히므로, 라벨링용 모노 영상은 중앙정렬을 끄고 모으는 것이 좋다.

## BEV / Refit

- 두 BEV는 같은 차량 좌표 grid(전방 0~3m, 좌우 ±0.6m, 1cm/px, 120×300)를 쓴다.
  차량 중심(y=0)이 BEV 가로 중앙이며 프리뷰에만 점선으로 표시한다(저장 이미지에는 없음).
- **왼쪽 렌즈 BEV 모드**: 왼쪽 렌즈만 단독 동작하고, CAM_A→CAM_B 캘리브레이션과 측정 자세로
  차량 중심에 정렬된 BEV를 만든다. 오른쪽 렌즈와 중앙정렬 화면은 잠긴다.
- 지면 자세는 고정이다(주행 중 IMU 보정 없음). BEV가 틀어지면 **Refit**:
  - ERPM이 기준(100) 이상이면 거부 (ERPM 미수신이면 정지로 보고 진행)
  - 카메라를 닫고 R 렌즈 + 도트를 임시로 켜서 측정한 뒤 원래 설정으로 재시작
  - 자동 캡처 중이었다면 끝난 뒤 다시 이어서 캡처
  - 실패하면 이전 자세 유지

## 저장 구조

```text
captures/session_YYYYMMDD_HHMMSS/
├── rgb/000001.jpg
├── stereo_center/000001.jpg
├── stereo_left/000001.jpg
├── stereo_right/000001.jpg
├── rgb_bev/000001.jpg
├── stereo_left_bev/000001.jpg
├── meta.csv
└── settings.yaml
```

- 한 번의 캡처(이벤트)는 모든 화면에서 같은 번호를 쓴다. 이 화면만 캡처도 새 번호를 받는다.
- 모든 이미지는 JPEG(품질 기본 95, 저장 탭에서 변경)이며 오버레이 없는 원본이다.
- `meta.csv`: event, 시각, 트리거(single/auto/view), 저장 화면, 모드, 해상도·포맷,
  실제 노출·ISO(컬러/L/R), flood·dot, ERPM, 서보, BEV 높이·roll·pitch, 자세 출처.
- 저장은 별도 스레드에서 하며 큐가 차면 그 이벤트를 버리고 드롭 수를 표시한다.

## 테스트

```bash
colcon build --packages-select manual_capture
colcon test --packages-select manual_capture && colcon test-result --verbose
```

카메라 없이 도는 테스트: 화면 선택·잠금 규칙, 단일/자동 캡처 규칙, A 버튼 엣지, Refit 거부 조건,
자동 캡처 스케줄러, JPEG/메타 저장, BEV 투영(컬러·흑백 동일 grid).
