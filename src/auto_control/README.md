# auto_control

`vehicle_bringup auto_drive.launch.py` now requires the
[startup inspection and explicit START approval](STARTUP_INSPECTION.md) before automatic duty.

`auto_control_node` follows the yellow centerline published by `line_detactor`
as `line_detactor/msg/LaneResult`. It writes to the VESC duty, brake-current,
and servo topics used by manual driving. Do not run manual and auto driving together.
Stanley/PID and actuator calculations run in C++/`rclcpp`. Optional tunnel
lane selection is performed by this package's C++ `auto_control::TunnelLaneSelector` component
(loaded into the detector container; the Python `auto_control_tunnel/node.py` is kept as the reference implementation);
its implementation module is installed as `auto_control_tunnel`.
The package's older Python files remain only as implementation
reference and are not installed by the `ament_cmake` build.

RGB/IR 터널 전환도 이 패키지가 소유한다. 별도 ROS 패키지나 전용 주행 launch 없이
기존 실행에 `tunnel_enabled:=true`를 추가한다.
자세한 빌드·전환·유실 처리는 [터널 주행 문서](AUTO_CONTROL_TUNNEL.md)를 참고한다.

원격 프리뷰(`remote_preview_enabled:=true`)의 중앙선과 도로 가장자리에는
브라우저 표시 전용 필터를 적용한다. 기존 최대 24개 좌표에 3점 공간 평활화와
시간 평활화(시정수 65ms)를 수행하므로 젯슨의 프레임당 필터 연산과 전송량은 추가되지 않는다.
공간 보정은 최대 1.5cm, 시간 필터의 추가 위치 지연은 최대 3cm로 제한한다.
12cm 이상의 경로 변화, RGB/IR·차선 지지 상태·도로 폭 변경, 250ms 초과 관측 공백에는
시간 필터 이력을 초기화한다. 무효 차선·연결 끊김에도 이력을 지우고, 데이터가 500ms를
넘으면 선을 숨긴다. 화면에 `표시 필터`를 표기하며, CTE·조향 수치와 실제 제어 경로는
원래 값을 유지한다. 필터 상수는 `auto_control_tunnel/web/remote_preview.html`의
`LANE_FILTER`에 있다. 설치된 웹페이지는 `auto_control` 재빌드 후 브라우저 새로고침으로 반영한다.

```bash
ros2 launch vehicle_bringup auto_drive.launch.py \
  tunnel_enabled:=true auto_enabled:=false speed_mps:=1.0
```

## 속도 계획과 PID, 개발 중인 신호등 정지

기본값은 **속도 PID 및 단계 제어 ON, 코너 곡률 감속 ON, 신호등 정지 ON**이다.
`auto_control_test.yaml`의 `auto_control.ros__parameters`에서 조절하고 재실행한다.
검출 확신도/FPS는 별도의 `traffic_light_test.yaml` 설정이다.

```yaml
longitudinal_pid_enabled: true
curvature_speed_control_enabled: false
longitudinal_pid_kp: 1.0
longitudinal_pid_ki: 0.5
longitudinal_pid_kd: 0.0
longitudinal_pid_integral_limit: 2.0
longitudinal_staged_control_enabled: true
longitudinal_start_duty: 0.05
longitudinal_recovery_start_ramp_sec: 0.20
traffic_brake_urgent_rise_amps_per_sec: 6.0
longitudinal_brake_speed_gain: 1.0
longitudinal_feedforward_offset_duty: 0.015
longitudinal_feedforward_duty_per_mps: 0.055
longitudinal_feedforward_fade_speed_mps: 0.10
longitudinal_recovery_duty_rise_per_sec: 0.15
longitudinal_deceleration_filter_sec: 0.20
traffic_stop_enabled: false
traffic_state_topic: "/traffic_light/state"
traffic_state_timeout_sec: 0.30
traffic_stop_line_timeout_sec: 0.50
traffic_terminal_tracking_distance_m: 0.10
traffic_terminal_tracking_timeout_sec: 1.50
traffic_stop_margin_m: 0.15
traffic_stop_range_max_m: 0.30
traffic_front_axle_to_bumper_m: 0.10
longitudinal_planning_deceleration_mps2: 0.50
traffic_brake_response_time_sec: 0.15
traffic_coast_probe_sec: 0.15
traffic_brake_deceleration_hysteresis_mps2: 0.10
traffic_brake_speed_hysteresis_mps: 0.05
longitudinal_brake_deceleration_gain: 0.50
traffic_pass_if_unstoppable_enabled: true
traffic_pass_overshoot_m: 0.60
brake_maximum_current_amps: 2.5
traffic_hold_current_amps: 0.7
traffic_stop_line_match_tolerance_m: 0.30
traffic_stop_line_current_weight: 0.35
traffic_stop_line_confirm_frames: 3
traffic_stop_position_tolerance_m: 0.03
traffic_reapproach_speed_mps: 0.20
```

### 목표 속도 곡선 → duty 감속 → 부족한 감속을 전류 제동으로 보완

코너 감속 OFF에서는 기본 목표가 `maximum_speed_mps`다. Stanley 조향은 그대로 사용한다.
신호등 정지를 켜고 최신 RED와 유효한 정지선 거리를 함께 확인해 접근을 시작하면,
남은 거리에 따른 속도 곡선 `v(d)`에서 매 차선
결과마다 목표를 다시 계산한다. 하나의 고정 속도나 미리 저장한 배열이 아니라,
정지선에 접근할수록 낮아지는 연속 곡선을 온라인으로 샘플링한다.
앞범퍼가 정지선 전방 0~`traffic_stop_range_max_m`(기본 0.30m) 범위에 들어오면
목표 속도와 duty를 0으로 전환해 제동을 시작한다. 30cm는 앞차축이나 명목 목표점 기준이 아니다.
현재 속도에 따라 제동하면서 범위 안으로 더 이동할 수 있으며 실차 정지 위치는 검증이 필요하다.
이미 선을 넘어선 거리도 전진 요청 없이 제동한다.

범위 진입 후에는 `range_braking`을 유지하고 기존 저속 조건(절대 속도 0.05m/s 미만)을
만족하면 `position_hold`로 전환한다. 거리 관측 흔들림이나 입력 유실/복구로 다시 전진하지
않으며 유효 GREEN에서 정지 확정을 해제한다. 차선/ERPM/연결/주행 허용 감시는 계속 적용한다.
이 변경은 저속 ERPM을 0으로 보정하거나 ERPM 기반 정지 판정 기준을 바꾸지는 않는다.

앞차축 기준 정지선 거리에서 범퍼 길이, 여유 거리, 촬영 이후 이동량을 뺀 거리를 d라 할 때
`v(d) = min(주행속도, sqrt((a*t)^2 + 2*a*max(0,d-허용오차)) - a*t)`다.
a는 정지선·코너 공통 `longitudinal_planning_deceleration_mps2`, t는 구동기 응답 여유다. 단계 제어에서는 t에
기준 제동 전류까지 선형 상승하는 시간의 절반을 더한다. 속도 곡선은 0A에서의 고정 지연을
쓰므로 전류가 증가했다고 목표 속도가 올라가지 않는다. a가 작을수록 일찍 감속한다.
`traffic_stop_margin_m`와 `traffic_stop_position_tolerance_m`는 범위 진입 전 감속 계획에
사용한다. 두 값의 합은 허용 범위 상한 이하여야 한다. 범위에 들어온 차량을 이 명목 목표까지
다시 전진시키지 않는다. `auto_drive`에서 `traffic_stop_range_max_m:=0.30`으로도 지정할 수 있다.
기본 범퍼 길이 0.10m는 실측값이 아니므로 차량 치수로 수정한다.

`longitudinal_staged_control_enabled: true`에서는 다음 순서로 구동기를 제어한다.

1. **duty 감속**: 목표 속도의 기본 duty와 속도 PID 보정량을 더한다. 목표보다 빠르면
   duty를 `duty_fall_rate_per_sec`에 따라 줄인다. PID의 음수 출력을 즉시 브레이크로 바꾸지 않는다.
2. **추가 제동**: 현재 속도 v와 남은 거리로
   `필요 감속 = v² / (2*max(0.01, d-허용오차-v*t))`를 계산한다.
   여기의 t에는 현재 전류에서 기준 전류까지의 추가 상승 지연을 반영한다.
   목표보다 `traffic_brake_speed_hysteresis_mps` 이상 빠르고,
   필요 감속이 duty 감속 추정값보다 `traffic_brake_deceleration_hysteresis_mps2` 이상 크면
   `traffic_coast_probe_sec` 관찰 후 제동한다. 필요 감속이 계획 감속도의 2배 이상이면
   관찰 대기는 생략한다. 제동 중에는 duty를 0으로 두어 구동/제동 명령을 겹치지 않는다.
3. **제동 해제와 구동력 회복**: 초과속도가 진입 기준의 절반 이하이거나 duty 감속만으로
   충분해지면 제동을 해제한다. 긴급하지 않을 때는 전류 해제 시간과 응답 지연 동안의
   추가 감속과 이동 거리를 예측해, 그때 남을 거리의 목표 속도 아래로 처지기 전에 해제를 시작한다. 해제 중 재진입은
   긴급한 경우에만 허용한다. 전류가 0까지 내려간 후 해제 명령을 한 주기 발행하고,
   양의 목표 속도가 남아 있으면 주행 중에는 `longitudinal_recovery_duty_rise_per_sec`로 회복한다.
   회복 도중 정체가 예상되면 아래 `recover_motion` 보조를 적용한다.
   정지 상태에서 재출발할 때는 아래 시작 duty를 1회 적용한 뒤 일반 가속률을 사용한다.
4. **범위 진입 후 정지**: 앞범퍼가 허용 범위에 진입하면 목표 속도 0을 확정하고 양의 duty를 내지 않는다.
   실제 속도 0.05m/s 미만에서 `traffic_hold_current_amps`를 적용한다.

추가 제동 전류는 `brake_maximum_current_amps * clamp(longitudinal_brake_speed_gain*초과속도 +
longitudinal_brake_deceleration_gain*부족감속, 0, 1)`로 정한다. 정지선과 코너에 같은 상한과 계수를 쓴다. 실제 전류 명령에는
`brake_current_rise_amps_per_sec`/`brake_current_fall_amps_per_sec` 변화율 제한을 적용한다.
단계 제어의 신호등 제동에서는 `(남은 거리-허용오차)/속도-응답 지연` 안에 요청 전류에
도달하기 어려우면 상승률을 높인다. 추가 상승 상한은 `traffic_brake_urgent_rise_amps_per_sec`
(기본 6.0A/s, 공통 상승률보다 낮게 설정하면 공통값 사용)이며 전류 상한 자체는 유지한다.
목표가 0으로 바뀌어도 실제 속도 0.05m/s 이상에서는 이미 적용 중인 제동 전류를 낮추지 않는다.
저속 진입 후에는 유지 전류 규칙과 기존 전류 하강률을 따른다.

지연 추정의 기준 전류는 `max(유지 전류, 제동 상한*clamp(감속 gain*계획 감속도,0,1))`다.
지연은 `응답 지연 + 0.5*max(0,기준 전류-현재 전류)/공통 전류 상승률`로 계산한다.
선형 상승 중 부분 제동을 근사한 계획값이며 실제 제동력 보정 결과가 아니다.
현재 속도와 거리를 모두 사용하지만, 전류→감속의 실측 모델은 아니므로 이득 보정이 필요하다.
제어 입력 유실에 대한 기존 정지 분기는 이 완만한 변화율 제한을 우회한다.

단계 제어의 `longitudinal_pid_kp/ki/kd`는 구동 duty 보정에만 사용한다.
`longitudinal_brake_speed_gain`은 별도의 양수 계수(단위 `1/(m/s)`)이며,
코너 등 일반 주행 제동과 신호등 제동의 속도 오차 항에 공통 적용한다.
일반 제동에도 같은 `brake_maximum_current_amps` 상한을 쓰며, 거리 제약이 없는 일반 감속에는 부족감속 항이 없다.
목표 0에서는 절대 속도를 속도 오차로 쓰며, 저속 유지 전류는 기존대로 별도 적용한다.
따라서 PID를 조절해도 같은 속도·거리 입력에 대한 제동 전류 계산 계수는 바뀌지 않는다.
구동 응답이 바뀌면 실제 접근 속도와 제동 시점은 달라질 수 있다.

새 계수의 기본값 `1.0`은 이전 기본 Kp `1.0`의 제동 세기를 유지한다.
외부 YAML에서 이전 `longitudinal_pid_kp`를 다른 값으로 조정했다면, 이전 제동 계산을
유지하려면 그 값을 `longitudinal_brake_speed_gain`에도 한 번 복사한 뒤 독립적으로 조절한다.
새 키를 생략하면 구동 Kp와 관계없이 `1.0`을 사용한다. 재실행 시 적용되며 시작 로그와
주행 로그 옆 파라미터 파일에도 기록한다.

감속은 ERPM 속도 차분을 `longitudinal_deceleration_filter_sec`로 평활화해 추정한다.
duty 감속 단계의 표본만 자연 감속 추정에 사용하고, 제동 중 및 제동 해제 후 응답 시간과
필터 정착 시간에는 갱신하지 않는다. 마지막 표본이 1초보다 오래되면 감속 능력을 0으로
취급한다. 이 추정에는 타이어/노면, 속도 필터 지연, duty 수준의 영향이 남는다.

기본 duty는 `offset*min(1, 목표속도/fade속도) + slope*목표속도`이며 최대 duty로 제한한다.
`longitudinal_feedforward_offset_duty`와 `longitudinal_feedforward_duty_per_mps`는
각각 offset/slope다. 기본 0.015/0.055는 조정 시작값이며 실차 보정 완료값이 아니다.
목표가 작아지면 offset도 사라지고, 목표 0에서는 구동하지 않는다. PID는 이 기본 duty에
속도 오차를 보완한다. 단계 제어의 정지 출발은 별도 `longitudinal_start_duty`로 설정하며,
일반 가속에는 `duty_rise_rate_per_sec`를 사용한다. 출력 상한뿐 아니라 실제 duty 변화율
제한으로 보정량을 전달하지 못한 경우에도 해당 방향의 적분 누적을 막는다.
제동 중, 목표 0 진입/복귀, 정지 접근 시작/GREEN 해제에서 PID를 초기화한다.
기본 D=0이므로 PI로 동작한다.

정지 후 출발에서는 기본 `longitudinal_start_duty: 0.05`를 한 번 적용한다.
이는 `manual_vesc_config.yaml`의 수동 전진 `start_duty: 0.05`와 같은 명령값이다.
최초 출발, 비활성/경로 유실 정지 후 복귀, 신호등 정차 후 GREEN 재출발,
임시 정지 후 정지선 재접근, 추가 제동 중 정지했다가 재출발하는 경우에 공통 적용한다.
유효 경로·최신 ERPM·VESC 연결·drive 모드·주행 허용 조건을 만족하고,
목표 속도와 PID 구동 요청이 양수이며 실제 속도의 절댓값이 0.05m/s 미만이어야 한다.
제동 전류가 0까지 내려가고 별도의 0A 해제 주기를 마친 다음에만 적용한다.
주행 중 복구된 첫 표본의 속도가 0.05m/s 이상이어도 회복 자격을 바로 없애지 않는다.
시작 duty/PID 요청에 도달하거나 목표 속도에 복귀할 때까지 이후 정체를 함께 감시한다.

시작 명령은 `min(longitudinal_start_duty, PID가 요청한 duty)`다.
저속 정지선 재접근처럼 요청이 작으면 0.05를 강제하지 않는다.
한 번 적용한 다음 주기부터 PID 목표까지 `duty_rise_rate_per_sec`로 올라가고,
필요하면 `duty_fall_rate_per_sec`로 시작 duty 아래까지 내려갈 수 있다.
시작 순간에는 추가 상승분을 더하지 않으며, 속도가 계속 낮다는 이유로 매 주기 재적용하지 않는다.
목표 0/보호 정지 또는 제동 중 정지를 거쳐 다음 출발을 준비한다.
제동/보호 정지 후 구동력이 회복되는 동안, 시작 duty에 도달하기 전에 속도가 0.10m/s 아래로
떨어질 것으로 예측되면 `recover_motion`을 기록하고 보조 상승률을 적용한다. 예측에는 관측
감속도와 기존 상승률로 시작 duty에 도달할 시간을 사용한다. 이미 정체한 경우도 포함한다.
목표가 0.05m/s보다 크고 실제 속도보다 충분히 높으며, 신호등 접근 중에는 현재 속도에서
계획 감속도와 제동 지연으로 멈출 거리가 남아 있어야 한다.

보조 상승률은 `longitudinal_start_duty / longitudinal_recovery_start_ramp_sec`다.
기본 0.05/0.20은 0.25 duty/s이며, 보조는 `min(시작 duty, PID 요청)`까지만 적용한다.
그 이상은 기존 상승률을 따르며 실제 출력 기준 적분 누적 제한도 유지한다. 반복 점프나
고정 최소 duty를 강제하지 않는다. 보조 시간은 양수이며 길게 설정할수록 더 완만하다.

`longitudinal_start_duty`는 0부터 `maximum_duty` 사이의 유한한 값이어야 한다.
0이면 시작 duty와 정체 방지 보조 상승을 끄고 기존 0부터 상승하는 동작과 제동 회복률을 사용한다.
이 파라미터는 단계 제어 전용이며 FF 오프셋, 기존 `minimum_duty`, 구버전 PID 동작과 독립이다.
YAML 변경은 재실행 시 적용하며 `auto_drive`에서는 `longitudinal_start_duty:=0.05`로도 지정할 수 있다.
외부 YAML에서 키를 생략해도 새 기본값 0.05가 적용된다.
기본 일반 상승률 0.04/s는 수동 최대 상승률 0.03/s와 유사하며,
수동 최대 가속 입력과 같은 상승률을 원하면 `duty_rise_rate_per_sec: 0.03`을 사용한다.
실제 출발의 부드러움은 배터리·노면·속도 피드백에 따라 달라진다. 이번 출발 변경의 빌드와 테스트는 수행하지 않았다.

`longitudinal_staged_control_enabled: false`로 이전 signed PID 방식으로 돌아갈 수 있다.
이 호환 모드는 하나의 signed PID를 유지하므로 새 `longitudinal_brake_speed_gain`을 사용하지 않는다.
`longitudinal_pid_enabled: false`이면 일반 주행은 기존 `speed_pid_*` 제어를 사용하지만,
RED 정지 접근은 `longitudinal_staged_control_enabled`에 따라 새 단계 제어 또는 signed PID를 쓴다.
`longitudinal_pid_*`는 정규화 출력용으로 duty 단위 `speed_pid_*`와 다르다.
`traffic_brake_amps_per_mps2`는 호환용이며 사용하지 않는다. 단계 제어에서는 기존
`brake_minimum_current_amps`, `brake_current_gain_amps_per_mps`도 사용하지 않는다.
일반 주행의 전기 제동은 `electrical_brake_enabled`를 따르고, **RED 정지 제동은
이 옵션과 독립적으로 작동**한다. VESC bridge의 전류 상한도 적용된다.

### 임시 정지와 목표 도달 정차

`stop_steering_hold_enabled: true`가 기본값이다. 목표 속도 0이고 실제 속도가
0.05m/s 미만이면 마지막 최종 조향각/서보 위치를 유지해 정차 중 경로 흔들림이
서보에 전달되지 않게 한다. 속도 노이즈로 유지 상태가 반복 전환되지 않도록
0.10m/s 초과 또는 양의 목표 속도로 복귀할 때 해제한다. 이동 중 감속에서는
경로 추종을 계속한다. 차선/ERPM 유실 등의 제어 중단과 disabled에서도 마지막
조향을 유지하며, 옵션을 false로 설정하면 정차 중 Stanley 계산 결과를 적용하고
제어 중단에서는 중앙 조향으로 복귀한다.

정지선만 보이거나 RED만 보이고 유효한 정지선 거리가 없으면 신호등 정지 요구를 만들지 않는다.
`traffic_state_timeout_sec` 이내의 RED와 `traffic_stop_line_timeout_sec` 이내에 연속 확인한
범퍼 앞 정지선 거리가 함께 있어야 접근을 시작한다. 거리 확인 전 RED가 유실되어 만료되면
대기를 해제하므로 이후의 신호등 없는 정지선에 이전 RED를 적용하지 않는다.
접근을 시작한 뒤에는 RED/UNKNOWN/신호 유실에 관계없이 정지 요구를 유지한다.
최신 GREEN은 감속/정차 중에도 정지 요구를 해제한다. GREEN 이후 UNKNOWN/입력 유실은
새 정지 요구를 만들지 않으며 정상 목표 속도로 계속 주행한다.

GREEN(이후 UNKNOWN 포함)에서 RED로 바뀌면, 유효한 거리와 속도를 확보한 시점에
`예상 정지거리 = v*t + v²/(2*a)`를 계산한다. v는 현재 필터 속도, t는
`traffic_brake_response_time_sec`, a는 공통 `longitudinal_planning_deceleration_mps2`다.
거리 관측은 영상 지연 이동량과 앞차축→범퍼 거리를 보정한다. 정차 여유 거리는
이 판단에서 다시 더해 **물리적 정지선**을 기준으로 비교한다.
`예상 초과거리 = 예상 정지거리 - 현재 범퍼부터 정지선까지 거리`가
`traffic_pass_overshoot_m`(기본 0.60m) 이상이고 `traffic_pass_if_unstoppable_enabled`가
true이면 정지 접근 대신 정상 목표 속도로 통과한다. 최초 인식이 RED인 경우에는
이 예외를 적용하지 않는다. 이는 설정 감속도에 따른 추정이며 실측 정지 위치가 아니다.

같은 대상에 대한 정지/통과 판단은 재계산으로 번갈아 바뀌지 않는다. 통과 결정은
범퍼가 해당 정지선을 지났다고 추정되거나 GREEN이 확인되면 해제한다. 통과 중 속도/시간
정보가 무효가 되어 거리 적분을 잃으면 통과 결정을 폐기하고 새 RED/정지선 쌍을 요구한다.
모든 경우 기존 enabled/drive/차선/ERPM 조건을 우선하며, 계속 주행은 무제한 가속이 아닌
설정된 정상 목표 속도로의 복귀다.

접근 대상이 정해지기 전 관측은 마지막 정지선 확인 시각 기준으로 만료하고,
범퍼가 통과한 관측도 버린다. 이전 정지선의 음수 거리가 다음 정지선을 계속 거부하지 않는다.
접근을 이미 시작한 대상은 임의로 다음 정지선으로 바꾸지 않는다.
RED 정지 접근 중 정지선 거리 미검출은 일반적으로 `traffic_stop_line_timeout_sec`까지
이동량으로 보정하고, 그 이후에는 **임시 정지**한다. 단계 제어에서는 마지막 확인 거리가
목표 정차점까지 `traffic_terminal_tracking_distance_m`(기본 0.10m) 이내라면,
최신 차선/ERPM을 유지하는 동안 마지막 확인부터 `traffic_terminal_tracking_timeout_sec`
(기본 1.50초)까지 이동량으로 근접 접근을 마칠 수 있다. 가까운 정지선이 영상에서
사라져 도착 직전 임시 정지가 걸리는 현상을 줄인다. 멀리서 잃은 정지선에는 적용하지 않는다.
차선/속도 입력 손실은 기존대로 임시 정지다.
정지선이 정상적으로 재확인되고 입력이 복구되면 남은 거리를 따라 저속 재접근한다.
임시 정지를 최종 도착으로 고정하지 않는다. 복구 접근 속도는 GREEN까지
`traffic_reapproach_speed_mps`로 제한한다.

거리 관측은 이동량 보정값과 일치 범위 안에 있는 연속 관측으로 확인한다.
허용 범위 안에서는 증가/감소 양쪽으로 보정하므로 짧게 측정된 거리도 복구할 수 있다.
허용 오차를 넘는 갑작스러운 변화는 두 방향 모두 거부한다. 이는 거리 연속성에 의한
대상 일치 추정이며 정지선의 의미적 ID 추적은 아니다. 큰 추정 오차나 다른 정지선은
자동으로 수용하지 않아 임시 정지가 계속될 수 있다. 정지선 확인 3회는 신호등 색상
연속 확인 설정과 별개이며 한 번의 정지선 오검출을 바로 목표로 고정하지 않게 한다.

최근 확인된 정지선 위치가 허용오차 이내이거나 위의 제한된 근접 추적 조건을 만족하고,
이동량으로 보정한 위치도 허용오차 이내이며,
실제 속도가 0.05m/s 미만일 때에만 최종 `position_hold`로 전환한다.
그 상태는 GREEN까지 유지한다. 정지 위치를 지난 경우 뒤로 복귀하지 않는다.
상태 로그 `phase`는 inactive/waiting_for_red_line_pair/pass_committed/
approach/slow_reapproach/position_braking/position_hold/
temporary_stop_line_unavailable/temporary_stop_control_input을 구분한다.
`line_rejected`는 거리 불일치로 거부한 누적 관측 수다.

임시 정지에서 차선/ERPM이 끊기면 watchdog은 설정 전류 상한으로 제동한다.
`enabled=false`, VESC 단절, steering_only/monitor_only는 기존 출력 제한을 우선한다.
실제 제동거리와 새 PID 이득은 실차 검증 전이며, 전기 제동은 기계식 주차브레이크와 같은
경사면 정지 유지를 보장하지 않는다. 이번 변경의 빌드/테스트는 수행하지 않았다.

실행 인자 `traffic_stop_enabled:=true`, `curvature_speed_control_enabled:=false`,
`longitudinal_pid_enabled:=true`로도 설정할 수 있으며 CLI가 YAML보다 우선한다.

```bash
colcon build --packages-select auto_control vehicle_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

## Control pipeline

Control is triggered by each received `LaneResult`: validate the path, calculate
Stanley/PID and publish duty/brake/servo commands directly in that callback.
The controller does not wait for an 80Hz timer. A short geometric detection failure
can reuse the last valid path within `path_hold_timeout_sec` (default 0.08s).
`control_rate_hz` is the stop-watchdog rate and reference rate for the configured
steering filter weight. PID, brake ramps and steering/duty rate limits use elapsed
time between results, bounded by the input freshness limits. A held path retains its
original capture timestamp, and failures never extend its hold deadline. Set the
hold timeout to zero to stop immediately on geometric failure. Malformed, stale,
duplicate, out-of-order or wrong-frame results stop immediately. With no new results, the watchdog stops on
stale path/ERPM, disconnect or disable. Enabling or recovering telemetry alone does
not initiate motion; another lane result must arrive and all path age/hold limits must pass.

The ROS command is published immediately after calculation. VESC UART transmission
continues through the existing bridge worker; this is not a hard real-time guarantee.

For every valid centerline that reaches final servo/duty calculation, the node publishes
`/auto/detector_input_to_control_decision_ms`. It starts when the detector receives the
BEV image and ends immediately before actuator command publication. Camera-image transport
before detector input, actuator command publication, and VESC UART transmission are excluded;
the detector queue and `LaneResult` transport are included. The line-detector preview uses
these samples for its cumulative `control avg` time and reciprocal FPS.

1. Subscribe to `/line_detactor/result`; validate the source timestamp, frame,
   centerline validity, sample limit, point/support arrays, and metric scale.
2. Remove `padding_left` from result pixels and convert pixel centers to metres:
   `X = x_max - (v + 0.5) * mpp`, `Y = y_max - (u - padding_left + 0.5) * mpp`.
3. Preserve near-to-far point order, including sideways corners. Keep the first
   contiguous forward-X ROI segment; stop at an excessive adjacent-point gap.
   `path_minimum_span_m` measures distance along the path, not its X extent.
   Spatial smoothing and outer-boundary weighting happen in `line_detactor`.
4. Calculate Stanley cross-track error from the front-axle origin and heading
   at the closest path projection plus `stanley_heading_lookahead_m` along the path.
5. With corner speed control enabled, sample curvature along the selected path
   and preserve each point's local speed limit, including the corner entry.
   Propagate deceleration backward and
   acceleration forward over every sampled path point. The camera-horizon point
   is not treated as a zero-speed stop; stale or missing paths use the watchdog.
   Combine this curve plan with the independently ramped obstacle speed cap.
6. Build lower, nominal and upper speeds around every curve-constrained profile
   point. Inside this band the propulsion PID error is cleared and feed-forward
   maintains the present speed. Below it duty targets the nominal speed. Near or
   above the upper edge duty is reduced first.
7. For every future point, calculate the deceleration required to remain below
   its upper speed. Add brake current only after the duty-coast observation time
   when this requirement exceeds measured coast deceleration and the vehicle is
   above the corner brake-entry margin. Urgent infeasible approaches may bypass
   the observation time. Traffic-stop latching remains separate.

The shared `longitudinal_planning_deceleration_mps2` is the feasible service
deceleration profile for both traffic stops and corners. Duty-only coast
deceleration is measured separately and determines whether current braking is
needed to meet that profile.
`corner_planning_response_sec` reserves travel during camera, control and actuator
delay. `corner_planning_acceleration_mps2` limits the spatial exit profile and
frame-to-frame target recovery. `corner_confirm_frames` rejects a one-frame curve;
`corner_release_frames` prevents a one-frame miss from releasing the band.
`minimum_speed_mps` is also the floor for local curve speeds and the lower edge
of the corner band, including the held band during a turn. It does not raise an
obstacle cap, traffic-stop target or guard-stop command. Recovery from those lower
targets still respects the existing acceleration ramp and may temporarily remain
below the floor. This is a target-speed preference, not a guarantee of actual speed:
when the floor exceeds the curvature-derived limit it can require more than
`maximum_lateral_acceleration_mps2`. Tune the floor for the vehicle and track.
The legacy speed-to-duty mapping continues to use the same parameter; no separate
corner-minimum parameter is needed. Build and tests were not run for this change.

### 장애물 점진 감속과 코너 회전 중 목표 회복 보류

장애물 검출 시 `obstacle_slowdown_speed_mps`는 즉시 적용하는 상한이 아니라 감속 후
도달할 속도다. 목표는 `obstacle_planning_deceleration_mps2`(기본 0.35)로 감소하고,
검출 해제 후에는 `obstacle_planning_acceleration_mps2`(기본 0.35)로 증가한다.
1.4→1.0m/s의 목표 전환은 기본값에서 약 1.14초다. 이는 목표 변화 시간이며 실제 속도
추종 시간은 아니다. 코너 감속 OFF에서도 이 장애물 계획은 적용된다.

단계 제어에서는 `obstacle_coast_probe_sec`(0.20초) 동안 duty 감속을 관찰한다.
현재 계획 속도 초과량이 `obstacle_brake_entry_margin_mps`(0.10)를 넘고,
필요 감속도가 관측 coast 감속도보다 `obstacle_brake_deceleration_margin_mps2`(0.10)
이상 부족할 때 전류 제동을 추가한다. 해제 여유는 `obstacle_brake_exit_margin_mps`(0.03)다.
최종 도달 속도와의 차이로 검출 순간 전류 제동을 걸지 않는다.

코너의 구간 전체에 최저속도를 적용하지 않고, 각 지점의 곡률 상한까지 감속을 역산한다.
회전 중에는 활성 코너 속도 상한과 장애물 계획 속도의 상승을 보류한다.
근거리 곡률이 `corner_exit_curvature_per_m`(0.25) 이하이고, 경로 방향과 이전 조향각이
모두 `corner_exit_angle_deg`(8도) 이내인 상태를 `corner_exit_confirm_sec`(0.20초)
확인하면 회복을 허용한다. 경로 노이즈가 지속되면 회복이 늦어질 수 있다.
낮은 속도에서의 출발·속도 부족 보정은 계속 허용하므로 실제 속도의 단조 감소를 보장하지 않는다.

제동 해제 예측에는 전류 감소 시간, `longitudinal_brake_release_response_sec`(0.15초),
속도 필터 시간 및 시작 duty 회복 시간을 반영한다. 예상 속도가 계획보다 낮아지면 코너·장애물
제동을 미리 해제한다. 긴급 코너 감속, 기존 회피 검사 보호 정지, 입력 유실 및 신호등 정지는
이 장애물 목표 램프로 지연시키지 않는다. 장애물 검출 감속에는 거리 기반 충돌 판단을 새로
추가하지 않았으며, 중앙선 변형 모드의 검출 감속은 충돌 방지 보장이 아니다.

새 파라미터는 YAML의 `auto_control.ros__parameters`에 지정하고 재시작한다.
기존 외부 YAML에도 코드 기본값이 적용된다. `curvature_speed_control_enabled: false`인
외부 YAML에서는 곡률별 코너 감속이 계속 꺼져 있으므로 이를 사용하려면 true로 설정한다.
이번 변경은 빌드·테스트·실차 주행을 수행하지 않았다.

The optional `/vehicle/dynamics/duty_cycle` and `/vehicle/dynamics/motor_current_a`
topics veto contaminated duty-coast samples when fresh. The dynamics monitor
publishes them from VESC CAN status in `slcan` or `socketcan` mode; its default
`ros_topic` mode has only bridge ERPM. The controller retains ERPM-only feedback
when CAN telemetry is absent. Neither motor current nor ERPM alone establishes
tire-road braking capability, so the plan starts conservatively and uses closed
loop braking when the observed deceleration falls short.

Missing/short geometric paths beyond the bounded hold interval invalidate control.
Stale ERPM, VESC disconnect, disable and shutdown also stop drive commands. Steering
follows `stop_steering_hold_enabled`. During an active RED approach, recoverable
input failures keep braking; disconnected/disabled modes retain their motor behavior.
Existing freshness limits still apply; camera capture age includes ML inference time.
`stop_line_present` alone never requests a traffic stop: a fresh RED and a confirmed
distance must be paired before activating an approach.

The control result carries timestamps, lane state/geometry, and the ordered
centerline only. BGR/label/mask payloads are not serialized through the
latency-sensitive control topic. Use `/line_detactor/result_image` for visual
diagnostics; it is generated only while the preview or an image subscriber needs it.

Default BEV is 120x300, X=0..3m and Y=-0.6..0.6m. The integrated launch checks
that output size matches the static model and derives producer/consumer geometry
from the BEV YAML. Direct node launches must provide the same geometry and frame.

Vehicle conversion defaults: 109.5mm tires, two motor pole pairs, 13/54 motor
gearing, and 13/37 differential gearing. Speed, duty and brake defaults are listed
in `config/auto_control.yaml`; electrical braking defaults to disabled.

## Launch

The default autonomous launch shows only the measured left/right lanes and
yellow centerline, without the original camera image:

```bash
ros2 launch vehicle_bringup auto_drive.launch.py
```

Existing BEV/controller/camera YAML arguments and CAN arguments are supported.
Add `line_detactor_params_file:=/absolute/path/line_detactor_test.yaml` to tune
65cm lane width, yellow-line smoothing and outer-boundary weighting. See
[the full launch command](../vehicle_bringup/README.md#autonomous-driving).

Run without any GUI preview:

```bash
ros2 launch vehicle_bringup auto_drive.launch.py preview_enabled:=false
```

Start disarmed for actuator/diagnostic checks:

```bash
ros2 launch vehicle_bringup auto_drive.launch.py \
  preview_enabled:=false \
  auto_enabled:=false
```

Measure lateral and heading errors while pushing the vehicle by hand without
publishing duty, brake-current or servo actuator commands:

```bash
ros2 launch vehicle_bringup auto_drive.launch.py \
  auto_control_mode:=monitor_only
```

Use `auto_control_mode:=steering_only` to publish automatic servo commands while
leaving duty and brake-current topics entirely to a manual controller. Passive
modes publish zero motor-command diagnostics but do not publish suppressed VESC
actuator topics. `monitor_only` also computes path diagnostics without requiring
VESC connection or fresh ERPM.

Tune electrical braking without editing YAML:

```bash
ros2 launch vehicle_bringup auto_drive.launch.py \
  brake_entry_speed_error_mps:=0.12 \
  brake_exit_speed_error_mps:=0.04 \
  brake_maximum_current_amps:=3.0 \
  brake_current_gain_amps_per_mps:=6.0 \
  brake_current_rise_amps_per_sec:=6.0 \
  brake_current_fall_amps_per_sec:=16.0
```

Set `electrical_brake_enabled:=true` to enable the configured overspeed braking.

Enable or stop an already running node:

```bash
ros2 topic pub --once /auto/enabled std_msgs/msg/Bool "data: true"
ros2 topic pub --once /auto/enabled std_msgs/msg/Bool "data: false"
```

The ML preview does not provide the old BEV Space-key disable shortcut.
Use `/auto/enabled` to explicitly disable control.

The default launch is armed and starts when a valid centerline, VESC connection,
and fresh ERPM have all arrived. Lift the wheels for the first test and keep a
hardware power cutoff reachable.

## Main tuning parameters

The complete Korean symptom-based tuning guide is installed as
`share/auto_control/AUTO_CONTROL_PARAMETER_TUNING_KO.txt`.

- `stanley_gain`: larger values correct lateral displacement more strongly.
- `stanley_heading_lookahead_m`: distance along the path from the closest
  front-axle projection; larger values use a farther heading.
- `path_maximum_gap_m`: stop retaining points at a larger adjacent-point gap.
- `centerline_smoothing_*` in the **line_detactor YAML**: spatial smoothing.
  Legacy `path_local_smoothing_window_m` and `path_outlier_threshold_m` are unused.
- `path_geometry_window_m`: larger values make local heading and curvature less
  sensitive to centimetre-scale steps.
- `path_minimum_x_m`, `path_minimum_points`, `path_minimum_span_m`: minimum
  ML centerline coverage accepted by the controller. Smaller
  values keep tight, mostly lateral corners valid but reduce path confidence.
- `stanley_corner_heading_threshold_deg`: enables the corner direction guard
  above this absolute path heading.
- `stanley_corner_opposing_correction_ratio`: limits an opposing cross-track
  correction so it cannot reverse the path-heading steering direction.
- `steering_current_weight`: smaller values smooth steering more but add lag.
- `steering_servo_inverted`: reverses only the final servo output while keeping
  positive `/auto/steering_angle_rad` defined as a vehicle-left command.
- `maximum_lateral_acceleration_mps2`: smaller values lower the local curve speed.
- `curvature_percentile`: diagnostic path-curvature percentile only.
- `longitudinal_planning_deceleration_mps2`: shared planning deceleration for
  traffic stops and corners.
- `corner_planning_acceleration_mps2`: maximum target-speed rise after a limit ends.
- `corner_planning_response_sec`: travel-time allowance before a corner limit.
- `corner_speed_band_width_mps`: distance between the curve profile's lower and
  upper speed. The midpoint is used only when acceleration or coasting is needed.
- `corner_coast_entry_margin_mps`: starts duty reduction before the upper edge.
- `corner_brake_entry_margin_mps`, `corner_brake_exit_margin_mps`: separate
  corner-current hysteresis around the upper edge.
- `corner_brake_deceleration_margin_mps2`: required deceleration shortfall that
  duty coasting may not satisfy before current braking is allowed.
- `corner_coast_probe_sec`: duration to observe duty-only deceleration.
- `corner_confirm_frames`, `corner_release_frames`: curve-profile frame hysteresis.
- `speed_pid_kp`, `speed_pid_ki`, `speed_pid_kd`: measured-speed PID gains.
- `brake_entry_speed_error_mps`: overspeed required to enter electrical
  braking; increase it if braking triggers too often.
- `brake_exit_speed_error_mps`: lower hysteresis boundary used to release the
  brake; it must stay below the entry threshold.
- `brake_minimum_current_amps`, `brake_maximum_current_amps`: brake-current
  bounds while braking is requested.
- `brake_current_gain_amps_per_mps`: converts speed overshoot into amperes.
- `brake_current_rise_amps_per_sec`, `brake_current_fall_amps_per_sec`: limit
  brake application and release rates.
- `speed_scale_correction`: calibrates ERPM speed using measured travel distance.

Diagnostic topics are `/auto/current_duty`, `/auto/current_brake_current`,
`/auto/target_speed`, `/auto/current_speed`, `/auto/path_curvature`, and
`/auto/steering_angle_rad`. Stanley steering diagnostics are available at
`/auto/cross_track_error_m`, `/auto/heading_error_rad`,
`/auto/raw_steering_angle_rad`, and `/auto/current_servo_position`.
Diagnostic messages are serialized only while a subscriber is present. VESC
actuator command publication is unconditional in the corresponding active mode.

### 실제 주행 CSV 기록

사용 중인 `auto_control_test.yaml`의 `auto_control.ros__parameters` 아래에 추가한다.
기존 `auto_control_params_file:=.../auto_control_test.yaml` 실행 인자를 그대로 사용한다.

```yaml
driving_log_enabled: true
driving_log_rate_hz: 20.0
driving_log_directory: "driving_logs"
stop_steering_hold_enabled: true
```

이 기록 기능은 실제 drive 모드에서 동작한다.
실행 위치의 `driving_logs/drive_<시작시각 ns>.csv`와 같은 이름의 `.parameters.txt`에
각각 주행 상태와 실제 적용된 auto_control 파라미터를 저장한다. 시작 로그의
`Driving CSV:`에 절대 경로가 표시된다. 분석할 때 두 파일을 함께 전달한다.

CSV는 제어 명령 발행 시점의 상태를 최대 20Hz로 샘플링하고 제어 상태, 모터 모드,
수신 신호 색상, 조향 유지 상태, 종방향 제어 단계가 바뀌면 추가 기록한다. 따라서 짧은 정지 분기를
파악할 수 있으나 모든 센서 메시지나 모든 제어 주기를 보존하는 기록은 아니다.
스냅샷을 최대 256개 큐에 넣고 별도 스레드에서 CSV 변환/파일 쓰기 및 1초 간격
flush를 수행한다. 기록 스레드가 밀리면 제어를 기다리게 하지 않고 행을 누락하며
`logger_dropped_rows`와 경고에 누적 수를 남긴다. Ctrl+C 정상 종료 시 대기 중인
기록을 모두 저장한다. 강제 종료/전원 차단 시 마지막 미저장 데이터는 남지 않을 수 있다.

- `target_speed_mps`, `speed_mps`, `raw_speed_mps`, `measured_erpm`: 목표/필터 속도,
  필터 전 속도와 원본 ERPM. 시간차로 실제 감속 정도를 계산할 수 있다.
- `pid_effort`, `desired_duty`, `command_duty`, `brake_current_a`: PID 출력
  (-1..1), duty 변화율 제한 전 목표, 최종 duty 및 제동 전류 명령이다.
  단계 제어에서 제동 중에는 PID를 사용하지 않아 앞의 두 값은 빈 칸이다.
  실제 전류 측정값은 아니며 `control_mode`/`motor_mode`/연결 상태와 함께 해석한다.
- 스키마 8의 `longitudinal_phase`는 `start_drive`/`track_speed`/`reduce_duty`/`additional_brake`/
  `release_brake`/`recover_drive`/`recover_motion`/`stop_brake` 등 실제 구동 제어 단계를 기록한다.
  `traffic_phase`의 `range_braking`은 허용 정차 범위 진입 후 정지 확정, `position_hold`는 저속 유지 상태다.
  `start_drive`는 시작 duty를 1회 적용한 주기이며, 다음 주기부터 일반 PID 제어로 이어진다.
  `feedforward_duty`는 기본 duty, `requested_brake_current_a`는 변화율 제한 전 전류 요청이다.
  `required_deceleration_mps2`, `measured_deceleration_mps2`는 필요/측정 감속도다.
  `coast_deceleration_mps2`는 마지막 duty 감속 추정값으로 `coast_age_s`가 1초를 넘으면
  제어에서는 0으로 취급한다. `coast_probe_elapsed_s`는 제동 전 관찰 경과시간이다.
  `traction_recovery`는 별도 duty 회복률 사용, `terminal_tracking`은 근접 거리 추적 조건 충족을 뜻한다.
  `recover_motion`은 시작 duty까지 정체 방지 보조 상승을 적용한 주기다.
  `brake_delay_s`는 현재 전류를 반영한 제동 지연 추정,
  `brake_rise_limit_a_per_s`는 해당 주기에 계산한 전류 상승 한도다. 실측 전류/지연이 아니다.
  `corner_lower_speed_mps`/`corner_nominal_speed_mps`/`corner_upper_speed_mps`는 현재
  코너 속도 구간이며, `corner_speed_in_band`가 1이면 정확한 한 속도를 추종하지 않는다.
  `corner_constraint_m`과 `corner_required_deceleration_mps2`는 전방 포인트 전체에서
  가장 큰 감속 요구를 만든 거리와 그 감속도다.
- `obstacle_requested_speed_mps`는 장애물 검출 상태에 따른 최종 도달 속도,
  `obstacle_planned_speed_mps`는 변화율을 적용한 현재 계획 상한이다.
  `obstacle_profile_active`는 검출 해제 후의 회복 구간까지 포함하며,
  `obstacle_plan_deceleration_mps2`는 현재 계획 상한의 감소율이다.
  `turn_recovery_hold`는 회전 종료 확인 전 목표 상한 상승 보류,
  `predicted_release_speed_mps`는 제동 해제 후 예상 속도다. 실제 측정값이 아니다.
- `centerline_xy_m`: 제어에 사용한 경로 전체를 `x:y;x:y;...` 형식으로 저장한다.
  차량 기준 x 전방, y 좌측, 미터 단위이며 세계 좌표 궤적은 아니다.
  경로 유효 여부와 원본 순서는 `path_valid`, `path_point_count`, `lane_sequence`에 있다.
- `raw_steering_rad`, `steering_rad`, `servo_position`, `steering_held`: Stanley
  계산값, 최종 조향각, 서보 명령과 조향 유지 여부. 각도는 라디안이다.
- `stop_line_raw_m`: 마지막 LaneResult의 앞차축 기준 원본 정지선 거리.
  `stop_line_present` 및 lane 상태/경과시간을 함께 확인한다.
  `stop_line_corrected_m`은 범퍼/여유 거리/영상 지연 이동량을 뺀 최신 관측이며,
  `stop_remaining_m`은 연속 확인·거리 일치 검사·필터·이동량 보정 후 제어에 사용한 값이다.
  `stop_confirmed_age_s`, `stop_candidate_count`, `stop_line_rejections`로 미확인/유실/거부를 구분한다.
- `signal_state`, `signal_score`, `signal_age_s`, `signal_message_accepted`는 마지막
  수신 신호와 메시지 수용 여부다. 수용에는 UNKNOWN도 포함하며 정지 요청은
  `stop_active`와 `traffic_stop_enabled`로 구분한다. `red_latched`만으로는 거리 대기와
  실제 정지 접근을 구분할 수 없다. 메시지 유실 시 마지막 색상이
  남으므로 반드시 경과시간도 본다. 미측정/사용 불가 수치는 빈 칸이다.

RED 직후 정지 원인은 `control_state`/`traffic_phase`로 구분한다.
`waiting_for_red_line_pair`이면 아직 RED와 거리를 함께 확보하지 못해 정상 주행한다.
`pass_committed`이면 0.6m 초과 정지 예측에 따라 통과를 선택했다.
`predicted_stop_distance_m`/`predicted_overshoot_m`은 결정 시점의 값을 보존하며,
`red_from_green`, `red_age_s`, `stop_active`, `pass_committed`로 근거를 확인한다.
`path_held`는 잠깐 유효하지 않은 새 경로 대신 이전 유효 경로를 사용했음을 뜻한다.
`temporary_stop_line_unavailable`이면 이미 시작한 접근 중 정지선이 오래 유실되어
목표 속도를 0으로 만든 것이다. `temporary_stop_control_input`이면 차선/ERPM 등
제어 입력 실패로 제동 상한 명령을 내린 것이다. `approach`이면서 목표 속도는 남아
있는데 차가 먼저 멈춘다면 PID 출력, 제동 전류, duty 회복 속도를 함께 분석한다.
한 번 임시 정지를 거쳤다면 거리 복구 후에도 `slow_reapproach` 속도 제한이 유지된다.

## 장애물 검출을 포함한 수동 통합 확인

기존 `vehicle_bringup/auto_drive.launch.py manual_test:=true`로 차선·신호등·장애물 BEV 표시를
함께 실행한다. 장애물 검출은 이 패키지의 `ObstacleDetectorNode`에서 처리하며,
`auto_control_node`는 monitor_only로 자동 액추에이터 출력을 차단한다.
[실행·YAML·구조 설명](AUTO_CONTROL_OBSTACLES.md).

`avoidance_test_enabled:=true`를 추가하면 장애물 회피 후보/선택 경로와 권고 속도를
BEV에 표시한다. 기본 OFF이며 `/auto_obstacles`의 `obstacles.avoidance.enabled`로
실행 중 전환할 수 있다. 이 옵션만 켜면 화면 전용이다.
`avoidance_control_enabled:=true`는 장애물에 따라 변형한 중앙선을 실제 조향에 사용한다.
기본 `avoidance_deformation_only: true`에서는 차선 안 장애물의 반대쪽으로 중앙선만 휘게 하며,
변형 결과가 없으면 기존 중앙선과 일반 출발·속도 제어를 사용한다. 회피 실패로 출발을 차단하지 않는다.
기존 충돌/곡률 검사와 실패 시 제동은 `avoidance_deformation_only: false`에서만 적용한다.
주행 속도는 `maximum_speed_mps`로 설정한다. `avoidance_speed_cap_mps`와
`obstacles.avoidance.max_speed_mps`는 폐기되어 기존 YAML에 남아 있어도 무시한다.
차선 안 장애물 최초 검출 시 `obstacle_slowdown_speed_mps`(기본 1.2m/s)까지 목표를 점진적으로 낮추고,
마지막 검출 후 `obstacle_slowdown_clear_sec`(기본 1초)가 지나면 회전 종료 확인 후 서서히 회복한다.
`obstacle_slowdown_enabled: false`로 이 감속을 끌 수 있다.
기본 OFF이며 `manual_test:=true`에서는 여전히 액추에이터 출력이 없다.
[실제 적용 실행과 제한](AUTO_CONTROL_OBSTACLES.md)을 확인한다.


## 안전점검 창의 최대 속력 슬라이더

안전점검 창 하단에 최대 속력 가로 슬라이더와 Timings·Params·STOP 버튼을 표시합니다.
START 승인 후에도 점검 창이 유지됩니다. BEV 프리뷰에서 조작 패널을 분리했으므로
테마와 BEV 표시 배율에 관계없이 같은 창에서 조작합니다. 점검 창은 1020×731로,
기존보다 가로·세로 각각 15% 작게 표시하며 클릭 좌표도 같은 배율로 보정합니다.
변경한 `line_detactor`와 `auto_control`을 함께 빌드해야 합니다.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to vehicle_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch vehicle_bringup auto_drive.launch.py
```

기존 워크스페이스의 `depthai_DIR` 등 추가 빌드 인자는 동일하게 사용합니다.
테마를 켤 때는 기존 제한에 따라 장애물 오버레이를 꺼야 합니다. 회피 제어를
별도로 켜 둔 YAML이라면 그것도 꺼야 기존 launch 검사를 통과합니다.

```bash
ros2 launch vehicle_bringup auto_drive.launch.py \
  bev_theme_enable:=true obstacles_enabled:=false avoidance_control_enabled:=false
```

- **TARGET MAX SPEED**: 드래그 중 선택한 최대 속력과 단위 `m/s`.
- **Applied**: 제어 노드가 확인한 실제 적용 상한.
- **APPLYING... / LIVE**: 적용 확인 대기 / 확인 완료.
- **CONTROLLER OFFLINE**: 0.75초간 상태 수신 없음. 드래그가 비활성화됩니다.
- 적용 확인 없이 1.5초가 지나면 실제 적용값으로 되돌리고 잠시 `NOT APPLIED`를 표시합니다.

눈금과 범위는 제어 노드 시작 시 최종 적용된 `minimum_speed_mps`와
`maximum_speed_mps`를 사용합니다. 중간 값은 0.05 m/s 단위이고 양 끝점은
설정값 그대로 선택할 수 있습니다. 기본 설정 파일 기준 범위는 1.00–1.80 m/s지만,
다른 YAML/launch 인자를 사용하면 그 범위로 표시됩니다. 0은 정지 명령으로
사용하지 않습니다. 정지와 주행 활성화는 기존 제어를 사용합니다.

드래그 중 요청은 최대 10Hz이고 마우스를 놓으면 마지막 변경값을 바로 보냅니다.
실제 목표 속도는 다음 제어 계산부터 이 상한 이하로 제한됩니다. 실제 차량 속도가
즉시 바뀌는 것은 아니며 기존 PID·duty·제동 설정에 따라 감속합니다. 코너·장애물·신호등의
더 낮은 속도 제한과 정지 요청은 유지합니다. 기존 speed-to-duty 보정의 기준 상한은
시작 설정값으로 유지합니다.

변경값은 현재 실행에만 적용하고 YAML에 저장하지 않습니다. 재시작 시
`maximum_speed_mps` 설정값으로 돌아갑니다. `ros2 param get /auto_control maximum_speed_mps`는
슬라이더의 설정 상한을 반환하고, 현재 적용값은 아래 상태 토픽에서 확인합니다.
주행 CSV를 켜면 `runtime_maximum_speed_mps` 열에도 현재 적용값이 기록됩니다.

```bash
ros2 topic echo /auto/maximum_speed_state
```

통신 인터페이스: 요청 `/auto/maximum_speed_request` (`std_msgs/msg/Float64`),
상태 `/auto/maximum_speed_state` (`line_detactor/msg/SpeedLimitState`). 상태는 5Hz와
요청 처리 직후에 발행합니다. 시작 범위 밖 값과 NaN/Inf 요청은 거절합니다.
두 토픽은 reliable/volatile, depth 1입니다. 저장된 GUI 요청을 제어 노드 재시작 시
재발행하지 않습니다. 다른 ROS namespace 구성이 필요하면 점검 GUI와 제어 노드에서 같은 토픽으로 remap합니다.
GUI 입력은 [OpenCV HighGUI mouse callback](https://docs.opencv.org/4.13.0/d7/dfc/group__highgui.html)을 사용합니다.

이전 슬라이더 구현 검증 기록: ROS와 무관한 C++ 범위·UI 조작/확인/단절 테스트, 기존 C++ staged brake 테스트,
기존 Python 제어/제동 테스트 20개를 로컬에서 실행했습니다. ROS 2/CUDA가 없는 Mac에서는
전체 colcon 빌드와 실차 ROS 통신·마우스 동작을 확인하지 못했습니다.

조작 패널 이전과 안전점검 창 15% 축소 변경은 사용자 요청으로 빌드·테스트를 실행하지 않았습니다.

## 실행 중 도구와 처리 주기

일반 auto_drive는 RGB 촬영 및 카메라→BEV 전달 상한 70 FPS, 신호등 추론
20 FPS, 장애물 depth 30 FPS를 사용한다. 회피 계획 상한은 80 Hz다.
회피 worker는 처리 시작 간격으로 상한을 적용하고, 차선 또는 대응 장애물 결과가
바뀌었을 때만 다시 계획한다. 연산 시간이 길어져도 지난 입력을 몰아서 계산하지 않는다.

안전점검 창 하단의 `Timings`는 별도 연산량 창을 연다. 창이 열려 있을 때만
새 측정 표본과 평균·p95 통계를 수집한다. 기존 추론 백엔드/제어/프리뷰에서 사용하는
타이밍은 계속 유지한다. 창을 닫으면 측정이 해제되며, GUI 프로세스가 종료되어도
2초 heartbeat 만료 후 해제된다. 실제 완료 FPS와 `1000 / 평균 ms` 추정 FPS는 별도 열이다.
카메라 촬영 행은 수신 빈도만 표시하며 센서 자체의 연산 시간은 측정하지 않는다.
센서 입력이 없어 측정값이 갱신되지 않으면 STALE로 표시한다.

안전점검 창 하단의 `Params`는 노드·기능별 숫자 입력 창을 연다. 분류 선택으로 모든 탭에 접근하며
각 탭의 스크롤 영역에서 값을 편집한다. BOOL은 0/1이다. `Apply`는 해당 탭의
변경분을 노드 단위로 일괄 검증한다. 제어 설정은 기존 전체 검증과 PID/브레이크
구성 검증을 통과해야 적용되며, 거절된 변경은 기존 값으로 복원된다.
제어 설정을 적용하면 PID 적분 상태가 초기화된다. 최대 속도는 시작 설정의 속도
범위 안에서 변경한다. 센서 촬영 모드, 모델, 좌표계 등 재시작 설정은 읽기 전용이다.
BEV 입력 전달 상한(publish_fps), 신호등 추론 상한(inference_fps), 회피 계획 상한(max_fps)도 LIVE 입력으로 바꿀 수 있다.
`프리셋 저장` 버튼은 창에 연결된 모든 노드의 **현재 적용된 전체 파라미터**를
다시 조회해 하나의 ROS 파라미터 YAML로 저장한다. 숫자·BOOL뿐 아니라 문자열·배열과
읽기 전용 시작 설정도 포함한다. 저장 대상 노드는 버튼 옆에 표시되며, 실행되지 않아
창에 연결되지 않은 노드는 포함하지 않는다. 기본 저장 위치는
`~/.ros/auto_control/presets/`이고 저장 대화상자에서 파일명·경로를 선택할 수 있다.

입력만 하고 `Apply`하지 않은 값은 저장하지 않는다. 원하는 변경을 먼저 적용하고
저장한다. 적용 중에는 저장을, 저장 중에는 Apply를 제한한다. 노드별 조회는
비동기로 수행하며 노드 간 동시 시점의 스냅샷을 보장하지는 않는다. 조회 실패·10초
시간 초과 시 부분 파일을 남기지 않고 기존 프리셋을 유지한다. 성공하면 저장 경로를 표시한다.

저장 파일은 `auto_control.ros__parameters`, `camera_driver.ros__parameters` 등
노드별 섹션을 갖는다. 제어 설정은 다음 실행의 `auto_control_params_file`로 지정할 수 있다.
다른 노드까지 복원하려면 해당 노드의 설정 파일 인자에도 지정해야 한다.
`obstacle_params_file`은 `obstacles.*` 전용 형식을 검사하므로 전체 프리셋을 그대로
지정하지 말고 각 섹션에서 `obstacles.*` 항목만 추출해 사용한다. launch 인자의
명시적 덮어쓰기는 기존 우선순위를 따른다. 저장 버튼은 실행 중 설정을 변경하지 않는다.
별도 창에는 Python Tk(`python3-tk`)와 PyYAML(`python3-yaml`)이 필요하다.

안전점검 창의 START 옆 `STOP`은 `/auto_control/stop`
서비스를 호출한다. 자동 Duty 0, 설정된 `brake_maximum_current_amps` 제동,
기존 정지 조향 정책을 적용하고 재출발은 START 승인이 필요하다.
이 버튼은 auto_control의 자동 명령을 정지시키며 별도 수동 제어기의 명령을 제어하지 않는다.
