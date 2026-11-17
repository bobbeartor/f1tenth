# auto_control 터널 주행 통합

터널 밖 RGB 차선 주행 → TCN ENTRY → 터널 IR 차선 주행 → TCN EXIT → RGB 복귀를
`auto_control`이 관리한다. 상태·경로 선택기는 이 패키지의 C++ 컴포넌트 `auto_control::TunnelLaneSelector`(검출기와 같은 컨테이너에서 intra-process로 차선 결과 수신, 기존 Python 구현은 참고용으로 유지)이며,
별도 ROS 패키지나 전용 launch 없이 기존 `vehicle_bringup auto_drive.launch.py`에서
`tunnel_enabled:=true`로 활성화한다. 하나의 카메라 세션·자동제어기·VESC 경로를 사용한다. 항상 터널 밖에서 시작한다.
기존 `tunnel_tcn_test`, `ir_bev_line_detection`의 독립 launch는 그대로 사용할 수 있다.
통합 launch와 기존 카메라/자동/수동 launch를 동시에 실행하지 않는다.

## 현재 기본: 독립 IR 테스트와 맞춘 파이프라인 검증

`tunnel_validation_mode:=true`가 기본이다. 기존 실행 명령·외부 YAML을 그대로 써도
적용되며, TCN ENTRY/EXIT가 카메라 준비를 시작하고 첫 유효 목표 경로에서 소스를 바꾼다. 파이프라인 검증 후 기존 차단
조건을 다시 비교하려면 `tunnel_validation_mode:=false`로 실행한다.

| 항목 | 기본 검증 모드 `true` | 기존 조건 `false` |
|---|---|---|
| IR 노출 변경 후 150ms 대기 | 없음 | 적용 |
| IR 노출 30000±500μs / ISO800 메타데이터 일치 | 관측 로그만 출력 | 불일치 프레임 제외 |
| IR 촬영 후 250ms 제한 / 터널 프로필 준비 확인 | 없음 | 적용 |
| IR BEV timestamp | 독립 테스트 기본값과 같은 BEV 발행 시각 | 실제 촬영 시각 |
| TCN 결과 시각 검사 | 새 LIVE 상태 보고로 전환 준비 | 촬영 후 0.2~0.25초 범위 검사 |
| TCN WARMUP·보고 유실 중 차선 전달 | 마지막 선택 소스 계속 전달 | 전달 보류 |
| 출력 timestamp | 원본 헤더 유지, 카메라별 중복·역순 거부 | 동일 |

촬영 시각은 원본 DirectIrFrame과 `[IR CAMERA]`/`[IR BEV]` 진단에서 유지한다.
BEV 발행 시각을 쓰면 제어기의 capture-age 값은 촬영→BEV 변환까지의 지연을 포함하지
않는다. 결과를 계속 재발행하는 것은 아니며, 새 카메라 프레임당 한 번만 생성한다.
모델·노출 설정값·BEV 규격·흑백 3채널 복제·추론 후처리는 독립 IR 테스트와 같다.

IMU 사용 시 보정 유효성 검사와 C++ 제어기의 중앙선 기하/수신 timeout,
ERPM/VESC 조건, X 버튼 수동 전환은 유지한다. 모든 활성 모델의 시험 추론 완료를
기다리는 조건도 유지한다. RGB 외부 주행·신호등 모델과 속도 설정은 바꾸지 않는다.
통합은 RGB 지면/IMU 기준과 카메라 세션을 공유하므로 독립 실행과 같은 연산 부하를
보장하는 구성은 아니다. IMU 대기 만료·각도 초과는 아래 분리 로그로 확인한다.

## 터널 IR의 실시간 IMU 사용 옵션

기존 `auto_drive.launch.py` 명령 끝에 다음 중 하나를 지정한다.

```bash
# 기본값: 실제 촬영 시각의 IMU roll/pitch로 IR BEV 보정
tunnel_ir_imu_enabled:=true
# 비교용: 실시간 보정 없이 시작 때 측정한 지면 자세로 IR BEV 투영
tunnel_ir_imu_enabled:=false
```

이 옵션은 터널 IR 영상에만 적용한다. `false`여도 카메라 세션의 IMU 수집, 시작 지면
측정, 외부 RGB 보정과 차량 동역학용 IMU는 계속 사용한다. IR 입력은 보정 조회를
호출하거나 IMU를 기다리지 않고 항등 tilt를 전달한다. 물리적 카메라 장착 자세를
항등행렬로 바꾸는 것이 아니라 `setMountPose`로 설치된 측정 자세를 그대로 사용한다.

`true`는 이전의 수신 직후 1회 조회 실패→즉시 폐기 방식을 사용하지 않는다.
새 촬영 프레임을 최대 100ms/8개까지만 보관해 비동기 IMU 도착 후 다시 조회한다.
가장 최신의 시각 일치 결과 하나를 전달하고 이전 후보를 정리해 지연 후 몰아서
처리하지 않는다. 프로필 전환 시 이전 후보를 비운다. 초기 IMU 보정 완료는 필요하다.

IR 조회는 앞뒤 IMU 표본 각각 촬영 시각에서 최대 50ms까지 보간을 허용한다.
RGB의 기존 12ms 설정과 독립 IR의 기존 설정은 바꾸지 않는다. 표본이 더 멀거나
100ms 안에 보정을 못 얻으면 해당 프레임을 제외하며, IMU ON에서 고정 자세로
조용히 대체하지 않는다. 기존 예측 시간 상한과 3도 보정 각도 제한은 유지한다.

IR 전용 범위는 camera YAML의 다음 파라미터로 조정할 수 있다. 기존 YAML은 수정하지
않아도 코드 기본값이 적용된다. `tunnel_validation_mode`와 독립적인 옵션이다.

```yaml
camera_driver:
  ros__parameters:
    tunnel.ir_imu_max_age_sec: 0.05
    tunnel.ir_imu_wait_sec: 0.10
```

모델 로드 대기·속도 설정·경로 timeout은 변경하지 않았다. 실차에서는 IMU OFF/ON을
같은 속도로 비교하고 `[IR CAMERA] published`와 제어기 `centerline_timeout` 감소를
확인해야 한다. 로컬 지연·표본 간격 재현 시험이 차량에서 30Hz를 보장하는 것은 아니다.

## 출구 RGB 노출 복원과 자동노출 복귀

RGB는 평소와 터널 내부에서 자동노출(AE)을 유지한다. 외부 주행 중 실제 RGB 프레임의
노출 시간·ISO를 기록하고 ENTERING 때 외부 기준을 보관한다. 입구의 어두운 프레임을
피하도록 직전 300ms를 제외한 최근 1초에서 150ms 이상 안정된 구간(노출·ISO 각각
최대/최소 비율 1.25 이하)의 실제 중간값 표본을 선택한다. 차선 품질을 따로 판정하는
기준은 아니므로 외부 조명 기준이 충분히 확보되지 않으면 복원 명령을 생략한다.

`/tunnel/state=EXITING` 수신 즉시 RGB CAM_A에 외부 기준 노출·ISO를 한 번 적용한다.
각 값은 현재 RGB 값 이하로 제한해 이미 회복된 영상을 더 밝게 만들지 않는다.
IR 준비 요청인 `/tunnel/target_source`가 RGB로 바뀔 때까지 기다리지 않는다.
RGB 제어는 별도 작업 스레드가 담당하며 카메라/차선/제어 콜백을 기다리게 하지 않는다.

복원값이 확인된 **새 촬영 프레임 2개**를 받거나 기본 **200ms** 상한에 도달하면
`setAutoExposureEnable()`을 보내 자동노출로 복귀한다. OUTSIDE/재진입으로 바뀌어도
즉시 AE 복귀를 요청한다. 같은 EXITING 보고가 반복되어도 수동 노출을 다시 걸지 않는다.
카메라 입력이 끊겨도 호스트 시간으로 복귀 명령을 보내며, 명령 전송 실패 시에는
수동 복원을 반복하지 않고 AE 명령을 재시도한다. 200ms는 호스트 복귀 요청 상한이며
실제 센서 반영 시각은 장치 제어 큐와 ISP 지연에 따라 달라질 수 있다.

설정은 `camera_driver/config/camera_config.yaml`의 `camera_driver.ros__parameters`에서
변경한다. 기존 외부 camera YAML에도 같은 항목을 추가할 수 있고 생략 시 기본값이 적용된다.

```yaml
tunnel.rgb_exit_exposure_recovery_enabled: true
tunnel.rgb_exit_exposure_max_hold_sec: 0.20
```

로그 `[RGB EXPOSURE]`에 외부 기준값, 일시 복원값, 자동노출 재활성화가 표시된다.
외부 기준/최신 실제값이 없거나 이미 노출·ISO가 낮으면 AE를 계속 유지한다.
첫 유효 RGB 경로로 전환하는 기존 순서와 IR 노출 설정은 유지한다. RGB를 공유하는
TCN/신호등 입력에도 같은 짧은 노출 변경이 반영되며 별도 히스토리 초기화는 하지 않는다.

통합 단일 BEV 프리뷰에는 RGB/IR 어느 경로를 표시하든 다음 두 줄을 함께 표시한다.

```text
RGB 2.00ms ISO200 AE
IR 30.00ms ISO800
```

값은 설정 목표가 아니라 원본 카메라 프레임에서 읽은 실제 노출 시간·ISO다.
IMU/BEV 필터 앞에서 기록하므로 IR BEV가 끊겨도 카메라 수신 값은 확인할 수 있다.
RGB의 `RESTORE`는 임시 수동 복원 요청, `AE PENDING`은 AE 명령 재시도 대기,
`AE`는 자동노출 명령 모드를 의미한다. 센서가 AE 상태를 확인했다는 별도 응답은 아니다.
입력이 없으면 `--`, 촬영 시각이 0.5초 넘게 갱신되지 않으면 마지막 값에 `STALE`을 붙인다.

## 처리 빈도와 카메라 설정

| 상태 | RGB 차선 | IR BEV·차선 | 제어 경로 |
|---|---:|---:|---|
| OUTSIDE | 최대 70 FPS | 대기 0 FPS | RGB |
| ENTERING / EXITING | 최대 70 FPS | 최대 30 FPS | 기존 경로 유지 → 첫 유효 목표 경로로 즉시 전환 |
| INSIDE | 최대 5 FPS | 최대 30 FPS | IR |

OAK 장치에서 단계별로 호스트 전달을 바꾼다(센서 촬영 FPS는 그대로). 로그 `OAK routes`로 확인한다.

| 상태 | IR 차선 영상(CAM_B) | 장애물 스테레오 입력 | RGB 호스트 전달 |
|---|---|---|---|
| OUTSIDE | 장치에서 차단 | 전달 | 70 FPS 전체 |
| ENTERING / EXITING | 30 FPS | 전달 | 70 FPS 전체 |
| INSIDE | 30 FPS | 장치에서 차단(StereoDepth 계산 정지) | 약 30 FPS로 솎음 |

INSIDE의 RGB는 센서 70 FPS를 유지하고 장치 Script가 순번 기준으로 약 30 FPS만 보낸다.
노출 동작이 바뀌지 않으므로 TCN 밝기 특징은 그대로이고, TCN 입력에는 솎은 프레임을 모두 전달한다.

수치는 처리 상한/요청값이며 Jetson 연산 부하에 따라 실측 FPS가 낮아질 수 있다.
중복 프레임으로 FPS를 채우지 않는다. 두 TensorRT 모델은 시작 시 로딩·워밍업하며
TCN 이벤트로 프로세스·카메라를 재시작하지 않는다.

RGB CAM_A는 70 FPS로 유지하며, 원본 NV12를 촬영 시각 기준 약 30 FPS로 따로 전달해
TCN의 기존 밝기 특징과 10 Hz 추론·2.5초 시작 워밍업을 유지한다. TCN은 터널 안에서도
실행해야 출구를 감지할 수 있다. 통합 TCN 프리뷰는 끈다.

IR CAM_B는 양쪽 모드에서 1280×800 / 30 FPS로 촬영한다. 외부에서는 밝기가 다른
장애물용 영상에 차선 추론을 하지 않으며, 차선용 IR 영상은 장치에서 막아 USB로 보내지 않는다. 카메라 연결·보정과 모델 로딩·워밍업은 유지하고,
노출과 무관한 투영 LUT만 미리 준비한다. 외부 IR BEV 변환·모델 입력은 0 FPS이며,
TCN ENTRY 수신 즉시 ENTERING에서 IR을 30 FPS로 활성화하고 RGB도 유지한다. RGB의 시작 지면 측정·IMU
기준 자세를 IR 투영에도 공유하며 실제 CAM_B 출력의 내부·외부 파라미터를 사용한다.
IR BEV 컴포넌트가 밝기를 유지한 BGR 3채널도 직접 발행하며 기존 IR 모델·차선 후처리를 거친다.
통합 실행은 독립 테스트의 Python 어댑터를 시작하지 않는다.

ENTRY 요청 시 IR 노출 30000μs / ISO800 / dot OFF / flood0.5를 적용한다.
기본 검증 모드는 설정 명령 전후의 새 프레임도 즉시 투영하며, 노출 메타데이터가
일치할 때까지 기다리지 않는다. 따라서 전환 직후 짧게 외부 노출의 이미지가 포함될 수 있다.
`false` 모드에서는 150ms 대기와 실제 노출·ISO 일치 검사를 적용한다.
EXIT 수신 시 RGB를 70 FPS로 복귀시키고 IR 노출·조명과 IR 추론을 유지한다.
첫 유효 RGB 경로로 전환한 뒤 IR 자동 노출과 외부 장애물 dot/flood 설정을 복원한다.

외부 장애물 기능을 켠 경우 같은 CAM_B/C를 공유한다. 통합 모드의 스테레오 센서와
depth 요청도 **30 FPS**로 설정해 터널의 30ms 노출을 지원한다. 외부 RGB 70 FPS와는
별개이며, 기존 외부 depth의 60 FPS 요청을 유지하는 구성은 아니다.
터널에서는 host depth 전달과 제어기의 장애물 회피·감속 및 신호등 정지를 중지한다.
INSIDE 동안은 StereoDepth 입력을 장치에서 막아 depth 계산과 전송도 멈춘다. 노드 구성은 유지한다. 출구에서 설정 복원 후 새 depth만
외부 처리에 전달하며, 이전 장애물/정지선 제어 상태를 지운다.

## 전환과 유실 처리

TCN ENTRY는 `OUTSIDE → ENTERING → INSIDE`, EXIT는
`INSIDE → EXITING → OUTSIDE`로 전환한다. 준비 중 `/tunnel/compute_source`는 `BOTH`로
두 검출기를 활성화한다. `/tunnel/selected_source`와 실제 제어 입력은 기존 소스를
유지한다. 카메라 요청은 진입 때 즉시 IR로 바꾸고, 출구에서는 첫 유효 RGB 경로가
올 때까지 IR 노출·조명을 유지한다. RGB는 별도 센서이므로 이 동안 정상 빈도로 처리한다.

첫 유효 목표 경로가 도착하면 **같은 콜백에서 그 원본 결과를 전달하고 전환을 완료**한다.
150ms 혼합, 가중 평균, 경로 재표본화, 프레임 여러 회 확인을 사용하지 않는다.
목표 경로는 C++ 제어기와 같은 프레임·스케일·지지점·연속 점 개수/길이/간격 기준을
통과해야 한다. 무효/오래된 목표 결과는 기존 경로를 덮어쓰지 않는다.
목표가 준비되기 전까지 새로 들어오는 기존 소스의 결과를 전달하며, 기존 소스 자체가
무효 결과를 보내면 제어기는 기존 짧은 경로 유지·정지 조건을 적용한다.
양쪽 입력이 끊기면 원래 수신 timeout으로 정지한다. 보관한 경로를 재발행하지 않는다.

준비 시간 상한으로 미준비 소스를 강제로 선택하는 750ms 전환을 제거했다.
목표가 준비될 때까지 동시 처리를 유지하며, 주행 허가는 기존 제어기의 경로
최신성·기하·ERPM·모델 조건을 따른다. 반대 TCN 요청은 준비를 취소하고 기존 소스로
돌아간다. 반복되는 같은 inside/outside 보고는 준비를 다시 시작하지 않는다.

카메라마다 노출·전송 지연이 다르므로 출구의 유효 RGB 촬영 시각이 마지막 IR보다
앞설 수 있다. 선택기는 검출기 출력 바이트를 수정 없이 카메라별 토픽
`/tunnel/lane_result/rgb`, `/tunnel/lane_result/ir`로 전달한다(촬영 시각만 읽고,
목표 경로 검증·IR 길이 탈출에 필요할 때만 전체를 해석). 제어기는 토픽으로 카메라를
구분하므로 `lane_source` 필드는 쓰지 않는다. C++ 제어기는 카메라별로 중복·역순을 검사하며
실제 헤더 나이 검사는 계속 적용한다. 일반/독립 실행은 기존 단일 스트림 검사를 쓴다.
기본 검증 모드의 IR 헤더는 기존처럼 BEV 발행 시각이므로 촬영→BEV 변환 지연을
포함하지 않는다. 전환 준비 중 목표 소스에는 검증 모드에서도 헤더 최신성을 검사한다.

ENTERING/EXITING 중 외부 장애물·신호등 제어는 중지하고 OUTSIDE에서 복원한다.
기본 검증 모드는 TCN 워밍업·보고 유실 중에도 마지막 요청에 따른 처리를 계속한다.
`false` 모드는 TCN이 WARMUP 중이거나 0.5초 넘게 보고가 없으면 차선 전달을 보류한다.
TCN 이벤트 한 개가 유실되어도 이후 보고에 유지되는 inside/outside 상태로 요청을 복구한다.

혼합 전용 `tunnel_handover.yaml`과 해당 파라미터를 제거했다. 별도 전환 설정 없이
기존 제어 YAML/CLI의 경로 기하 조건과 최신성 기준을 선택기에 전달한다.
완료 로그 `Tunnel handover {target, reason, total_ms}`에서
`first_valid_target`은 첫 유효 목표 경로로 즉시 전환, `reversed_request`는 준비 취소를
의미한다. `total_ms`는 요청 수신부터 전환까지의 시간이며 실제 카메라 노출 완료나
물리적인 터널 경계 통과 시각을 뜻하지 않는다.

X 버튼의 AUTO → STOPPING → MANUAL 전환은 기존 방식 그대로다.
터널 이벤트는 자동주행 활성화나 수동 전환 해제를 요청하지 않는다.

## 실험 옵션: IR 관측 길이 감소로 출구 전환 요청 (기본 OFF)

기존 실행 명령에서 옵션을 생략하면 이전 TCN 정책을 유지한다. ON일 때만 `INSIDE`에서
IR 검출기의 실제 관측 길이를 비교해 `EXITING`을 요청한다. 비율은 퍼센트 숫자가 아닌
0~1 사이의 값이다. `0.15`는 기준 최대 길이의 15% 이하(85% 이상 감소)를 뜻한다.

```bash
# ON: 기존 auto_drive 실행 인수에 추가
ros2 launch vehicle_bringup auto_drive.launch.py \
  tunnel_enabled:=true tunnel_ir_length_exit_enabled:=true \
  tunnel_ir_length_exit_ratio:=0.15

# OFF: 기존 정책 (기본값이므로 enabled 옵션 생략도 동일)
ros2 launch vehicle_bringup auto_drive.launch.py \
  tunnel_enabled:=true tunnel_ir_length_exit_enabled:=false

# 비율과 확인 횟수 조정 예시
ros2 launch vehicle_bringup auto_drive.launch.py \
  tunnel_enabled:=true tunnel_ir_length_exit_enabled:=true \
  tunnel_ir_length_exit_ratio:=0.25 tunnel_ir_length_exit_confirm_frames:=3 \
  tunnel_ir_length_exit_minimum_reference_m:=0.5
```

ON/OFF와 파라미터는 실행 시작 시 적용하며, 비교할 때 기존 프로세스를 종료하고 재실행한다.
선택기 노드의 ROS 파라미터 이름은 위 이름에서 `tunnel_`을 뺀
`ir_length_exit_enabled`, `ir_length_exit_ratio`, `ir_length_exit_confirm_frames`,
`ir_length_exit_minimum_reference_m`이다. 실행 중 `ros2 param set` 적용은 지원하지 않는다.

관측 길이는 각 프레임의 `observed_paths`에서 차량 가까이 시작하는 실제 모델 지지 차선 조각의
호 길이를 미터로 계산하고, 좌/우 중 가장 긴 값을 쓴다. 기준 시작 거리에는 기존
`centerline_max_start_distance_m`(기본 0.65m)을 사용한다. 연결선·외삽 중앙선·멀리 떨어진
별도 조각은 길이에 포함하지 않는다. 이는 실제 카메라 관측의 정답 길이가 아니라 모델이
검출한 차선의 길이다. 좌/우 길이의 합이나 생성 중앙선의 전체 길이를 쓰지 않는다.

매 진입 요청 때 최대 길이를 초기화한다. 새롭고 유효한 IR 결과를 기본 3개 모아 그 구간의
최솟값을 얻고, 이 값의 역대 최대를 기준으로 삼아 한 프레임의 과대 검출을 억제한다.
기준 길이가 기본 0.5m 이상 확보되어야 활성화한다. 현재 길이가 기준 대비 설정 비율 이하인
새 IR 결과 기본 3개가 연속하면 전환을 요청한다. 감소 후 기준 길이를 낮추지 않는다.
실제 처리된 빈 프레임의 유효 길이 0도 포함하며, 메시지 유실·오래된/미래 프레임·잘못된
스케일·측정 불가를 길이 0으로 간주하지 않는다. 수신 공백이 선택기의 최신성 상한보다 길면
연속 확인을 초기화한다. 기본 검증 모드에서도 이 실험 판정에는 최신성 검사를 적용한다.

전환 시 `EXITING`, `compute=BOTH`, 기존 IR 프로필을 유지하고 **첫 유효한 새 RGB 경로를
받으면 그 결과를 즉시 전달하며 OUTSIDE로 전환**한다. IR 경로가 없어져도 RGB를 강제 선택하거나
보관 경로를 재발행하지 않는다. RGB 준비가 늦으면 기존 경로 유실 정지 조건은 유지된다.
이 실험 옵션에는 별도 사전 RGB 준비 임계값을 추가하지 않았다.

길이로 전환한 후 TCN의 반복 `inside` 보고가 즉시 IR로 되돌리는 것을 막는다.
트리거 이후 촬영된 최신 `outside` 보고를 확인하면 재진입을 다시 허용한다. TCN이 계속
`inside`로 고착되면 자동 재진입도 보류되므로 해당 상황에서는 이 옵션을 끄고 원인을 확인한다.

로그 `[IR LENGTH EXIT]`에 현재 길이, 기준 최대 길이, 비율, 임계값, 연속 횟수와 트리거 여부를
1초마다, 트리거 발생 시 즉시 남긴다. 상태 전환 사유는 `ir_length_drop`이며, 완료 로그에도
`trigger=ir_length_drop`이 유지된다.

`LaneResult.msg`에 관측 길이 필드를 추가했으므로 차량 PC에서 메시지 소비자까지 다시 빌드한다.
워크스페이스 루트에서 전체 빌드를 권장한다.

```bash
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

로컬에서는 실제 차선 연결기의 관측 조각 길이 C++ 테스트와 기본 OFF/임계값/유실·중복·오래된
프레임/TCN 역전 방지/다음 터널 초기화/RGB 준비 전 대기 재현을 검증한다.
Jetson ROS 전체 빌드 및 실차에서 출구 정지가 해결되는지는 별도로 확인해야 한다.

## 단일 프리뷰와 터널 HUD

RGB 검출기만 GUI 창을 소유하며, IR 검출기는 별도 창을 열지 않는다. 선택된 경로가
IR이면 같은 창에 IR 결과를 표시한다. 최초 결과 전에는 대기 화면을 표시하고,
수신 후 결과 timestamp 기준 0.25초를 넘기면 마지막 화면을 유지하며 `IR STALE: …ms`를 표시한다.
이 화면 유지는 GUI에만 적용하며, 선택기·제어기에 이전 경로를 재전달하지 않는다.
다음 터널 진입 시에는 이전 터널의 화면을 비운다.
IR 결과는 외부 장애물 플래너의 RGB 차선 입력을 덮어쓰지 않는다.

기존 `line_detactor_test.yaml`의 `line_detactor.ros__parameters`에서 다음을 설정한다.

```yaml
preview_enabled: true
bev_theme_enable: false
preview_result_only_enabled: true
```

`preview_result_only_enabled: true`는 검은 배경에 차선·정지선·중앙 경로를 표시한다.
`false`는 실제 BEV 영상 위에 결과를 겹친다. 선택은 RGB와 IR에 동일하게 적용한다.
기존 실행 명령에 `preview_result_only_enabled:=false`가 있으면 YAML보다 우선하므로
제거하거나 `true`로 바꾼다. 이 설정은 재실행해서 적용한다.
초기 지면 보정 ROI 창은 별개이며 `bev_config_test.yaml`의
`measurement_roi_preview_enabled: false`로 끌 수 있다.

신호등 상태 옆에 `Tunnel: OUTSIDE / ENTERING / INSIDE / EXITING`을 표시한다.
바로 아래에는 TCN `avg … FPS`, LIVE/WARMUP/STALE 상태와
ENTRY/EXIT 점수를 표시한다. 평균 FPS는 LIVE 보고의 TensorRT+전송 wall time을 평균한 뒤
`1000 / 평균 ms`로 계산한다. 주행 루프나 실제 실행 주기를 의미하지 않는다.
워밍업·입력 유실 때는 FPS를 `--`로 표시한다.
일반 RGB/IR 프리뷰의 `Duty`는 `/auto/current_duty`의 제어 명령값을 소수점 4자리로 표시한다
(예: `0.0750` = 7.5%). 0.5초 이상 수신되지 않거나 유효하지 않으면 `--`로 표시한다.
`line_detactor.preview_duty_topic`으로 구독 토픽을 변경할 수 있으며, 기존 YAML에는 추가하지 않아도 된다.
`monitor_only`에서는 계산된 명령값이며 실제 모터 출력 피드백을 뜻하지 않는다.

## 시작 시 모델 검사

`auto_drive.launch.py`는 실제 활성화된 RGB 차선 모델과, 터널 활성화 시 IR 차선·TCN,
신호등 활성화 시 신호등 모델을 `model_startup_check`로 확인한다. 장애물은 신경망 모델을 쓰지 않는다.
각 모델은 엔진 로드/생성 후 시험 추론을 성공해야 `READY`를 발행한다.
차선 모델은 `warmup_iterations: 0`이어도 최소 1회 실행하며,
신호등은 실제 운영의 NV12→CUDA→TensorRT 입력 경로를 시험한다.
TCN은 고정 입력 크기와 유한한 출력값까지 확인한다.

터미널에는 `[MODEL CHECK] PASS <노드명>`과 모델 경로(차선은 엔진 경로도)가 표시되고,
모든 활성 모델이 준비되면 `[MODEL CHECK] ALL READY (N models)`를 표시한다.
실패 보고는 `FAIL`, 보고가 없는 모델은 10초마다 `Waiting`으로 표시한다.
기본 300초 안에 준비되지 않으면 `NOT READY` 오류를 표시하면서 계속 감시한다.
콜드 TensorRT 엔진 빌드는 수 분 걸릴 수 있다. `auto_drive`는 모든 활성 모델이 준비될 때까지
자동제어기를 `waiting_for_models`로 유지해 구동 duty를 출력하지 않는다. 300초가 지나도
자동으로 해제하지 않는다. 준비 상태는 `/perception/models_ready`로 1초마다 발행하고,
로드 실패·재로딩 보고 또는 3초 넘는 상태 수신 중단 시 다시 주행을 차단한다.
단독 제어기/기존 IR 독립 테스트는 이 감시 노드를 실행하지 않으므로 기본적으로 이 조건을
요구하지 않는다. 통합 launch가 `model_startup_check_required: true`를 직접 설정하며,
기존 테스트 YAML에 파라미터를 추가할 필요는 없다.
모델 준비는 카메라 로드·영상 입력·실제 검출 정확도 검증과 별개다.

## IR 독립 테스트와 통합 경로의 차이 및 끊김 확인

같은 IR FP32 ONNX 모델, LEFT 1280×800/30 FPS, 30ms/ISO800/dot0/flood0.5,
120×300 BEV, 동일한 흑백→BGR 복제를 사용한다. 통합에서는 RGB·TCN·신호등·depth와
하나의 카메라 세션/연산 자원을 공유하고, RGB에서 측정한 지면·IMU 기준 자세를 재사용한다.
독립 IR은 카메라 설정을 시작부터 적용하지만 통합은 진입 시 변경한다.
독립 IR 드라이버 기본 `use_sensor_timestamps: false`는 BEV 발행 시각을 쓴다.
통합 기본 검증 모드도 같은 방식이며, `tunnel_validation_mode:=false`만 촬영 시각을 쓴다. 독립 프리뷰에는 0.25초 초과 시 검게
지우는 조건이 없어서 같은 처리 지연이 다르게 보일 수 있다.

통합 IR 입력은 카메라의 새로운 촬영 프레임마다 처리하고, 센서가 이미 30 FPS이므로
호스트 도착 시각으로 다시 30 FPS를 샘플링하지 않는다. IR 검출기 내부의 추가 FPS 제한도
끄고 BEV 생성 측에서만 입력 빈도를 관리한다. 모델 자체는 같다. IR IMU 사용 및 조회 범위는 위 옵션을 따르며, 노출 메타데이터 일치 검사는 검증 모드에서 적용하지 않는다.

터널 안에서 2초마다 다음 로그로 끊김 위치를 확인한다.

- `[IR CAMERA] received/published`: 실제 카메라 수신과 투영 입력 전달 빈도.
  `rejected(time/exposure/imu_tilt)`와 실제 노출/ISO로 제외 원인을 확인한다.
  `imu_wait_expired`는 IMU 보정 대기 만료, `imu_queue_overflow`는 큐 용량 초과,
  `superseded`는 더 최신 보정 성공으로 생략한 후보 수다. `pending`은 대기 프레임 수,
  `imu_initialized`는 시작 보정 완료 여부다. OFF에서는 IMU 대기·각도 제외가 없다.
- `[IR BEV] output/source_age/status`: 모델 입력 생성 빈도, 촬영 나이, 입력·보정 대기 상태.
- `[ir_line_detactor] FPS: input/processed`: IR 추론 입력과 처리 빈도.
- 화면 `IR STALE`: 마지막 표시 결과 timestamp가 오래됐다는 뜻이며 새 검출 결과를 뜻하지 않는다.

이 로그 없이 실제 입력 유실이 노출·IMU·연산 지연 중 어디서 발생했는지 단정할 수 없다.

## DepthAI 실행 라이브러리 경로

공용 BEV/카메라 컨테이너는 빌드 때 선택된 `depthai::core` 라이브러리 디렉터리를
`bev_processor/depthai_runtime_dir.txt`에 기록하고, 실행 시 컨테이너의
`LD_LIBRARY_PATH` 맨 앞에 자동 적용한다. 다른 프로세스의 환경은 바꾸지 않는다.
필요하면 `auto_drive.launch.py depthai_library_dir:=/absolute/sdk/lib`로 덮어쓸 수 있다.
파일이 없거나 디렉터리가 사라졌다면 launch가 원인을 표시한다.
`undefined symbol: _ZTIN3dai10DeviceNodeE`가 계속되면 DepthAI를 사용하는 패키지들을
동일한 `-Ddepthai_DIR=.../lib/cmake/depthai`로 캐시를 지우고 다시 빌드해야 한다.
모델의 `ALL READY`는 카메라 컴포넌트 로드 성공을 의미하지 않는다.

## 차량에서 빌드·실행

ROS 2 Humble / CUDA·TensorRT / DepthAI C++ 3.6+가 준비된 Jetson의 워크스페이스
루트(`src`의 부모)에서 실행한다. `LaneResult`에 소스 식별 필드가 추가됐으므로
차선 검출기와 메시지 소비 패키지를 함께 빌드한다. 이전 노드는 모두 종료하고 재시작한다.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to vehicle_bringup --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 먼저 연산·인식·전환만 확인. 기존 외부 주행 YAML 인자를 그대로 추가할 수 있다.
ros2 launch vehicle_bringup auto_drive.launch.py \
  tunnel_enabled:=true \
  auto_control_mode:=monitor_only preview_enabled:=true speed_mps:=1.0

# 위 실행 종료 후 주행 준비. 기본적으로 출발은 보류한다.
ros2 launch vehicle_bringup auto_drive.launch.py \
  tunnel_enabled:=true \
  auto_enabled:=false preview_enabled:=true speed_mps:=1.0
```

`speed_mps`는 0보다 큰 유한한 목표 속도(m/s)이며 0.3 상한은 없다.
생략하면 기존 외부 제어 YAML/launch의 최소·최대 속도를 상속한다.
IR 전용 launch와 마찬가지로 실제 속도는 기존 ERPM 피드백으로 제어한다.
`auto_enabled`는 기존 auto_drive 및 선택한 제어 YAML 값을 따른다. 위 주행 준비 명령은
명시적으로 false를 지정한다. 보정을 끝내고 TCN LIVE·차선을 확인한 뒤
다른 source된 터미널에서 활성화한다.

```bash
ros2 topic pub --once /auto/enabled std_msgs/msg/Bool 'data: true'
ros2 topic echo /tunnel/state
ros2 topic echo /tunnel/selected_source
ros2 topic hz /tunnel/lane_result/rgb   # 터널 안에서는 /tunnel/lane_result/ir
```

기존 인자 `camera_params_file`, `bev_params_file`, `line_detactor_params_file`,
`auto_control_params_file`, `obstacle_params_file`, `vesc_port`, CAN 인자를 함께 쓸 수 있다.
IR 학습과 일치하도록 통합 BEV는 앞차축 기준 120×300, 전방 0~3m, 좌우 ±0.6m,
1cm/pixel을 요구한다. 기본 회피 제어가 활성화된 설정에서 외부 장애물도 제외하려면
`obstacles_enabled:=false avoidance_control_enabled:=false`를 함께 지정한다.
기존 RGB 전용 동작을 유지하도록 `tunnel_enabled` 기본값은 false다.
공용 IR 모델과 검증 보고서는 `line_detactor/models`에 있으며, 단독 IR 테스트와 함께 사용한다.

## 로컬 검증

```bash
PYTHONPATH=src/auto_control python3 -m unittest discover \
  -s src/auto_control/test/tunnel -v
c++ -std=c++17 -Isrc/line_detactor/include \
  src/line_detactor/test/frame_rate_gate_test.cpp -o /tmp/frame_rate_gate_test
/tmp/frame_rate_gate_test
```

이 검사는 지연된 진입/출구의 경로 연속성, 실제 C++ 제어 코어에 대한 원본 경로 기하와
카메라별 순서 검사, 무효·오래된·역순 결과 처리, 모델 준비 상태와 처리 빈도 제한을 확인한다.
카메라의 `rgb_exposure_recovery_test`는 노출 복원·자동복귀·프레임 유실·명령 실패와
반복 터널 처리를 검증하며 프리뷰 시험은 실제 HUD 렌더링과 노출 데이터 전달을 확인한다.
이 개발 환경에서는 ROS 빌드·DepthAI 동시 출력·Jetson TensorRT·실차 전환은 검증하지 않았다.
기존 단독 터널 주행 성공은 이 통합 경로의 검증을 대신하지 않는다.
