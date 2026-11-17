# 원격 프리뷰(성능 테스트) 모드

Jetson에 NoMachine 대신 SSH로 접속해 주행을 실행하고, 프리뷰는 같은 네트워크의
iPad/MacBook 브라우저에서 본다. Jetson은 영상 창·영상 토픽을 만들지 않고 숫자 정보만
30Hz로 보낸다. 프리뷰 그리기는 단말이 한다.

## 실행

기존 실행 명령 끝에 `remote_preview_enabled:=true`만 추가한다. 다른 YAML 인자(`*_params_file`)는
그대로 쓸 수 있다.

```bash
ros2 launch vehicle_bringup auto_drive.launch.py <기존 인자들> remote_preview_enabled:=true
```

브라우저에서 `http://<jetson-ip>:8765`를 연다(iPad Safari, macOS Safari/Chrome).

## 원격 프리뷰 YAML

`remote_preview_params_file:=<경로>`로 설정 파일을 바꿀 수 있다. 기본 파일은
`auto_control/config/remote_preview.yaml`이다.

```yaml
remote_preview:
  ros__parameters:
    port: 8765
    bind: "0.0.0.0"
    frame_rate_hz: 30.0
    lane_width_m: 0.0          # 0 = line_detactor YAML의 centerline_lane_width_m 사용
    link_loss_stop_sec: 1.0    # 0 = 끊김 STOP 끄기
```

## 원격 모드에서 바뀌는 것

| 항목 | 일반 실행 | `remote_preview_enabled:=true` |
|---|---|---|
| line_detactor 프리뷰 창 / result_image | 설정에 따름 | 끔 |
| `/bev_processor/startup_preview` 영상 | 발행 | 발행하지 않음(상태 문자열만 발행) |
| startup_inspection 창, runtime_tools | 실행 | 실행하지 않음 |
| remote_preview 노드 | 없음 | 실행(HTTP 8765) |
| START 승인 / 컨트롤러 점검 | 필요 | 동일하게 필요 |
| 수동 전환(X 버튼) | 유지 | 유지 |

## 연결 끊김 STOP

컨트롤러가 `STARTED`인 동안 단말 heartbeat(200ms 주기)가 `link_loss_stop_sec`보다 오래 끊기면
`/auto_control/stop`을 한 번 호출한다. 다시 연결되어도 자동으로 출발하지 않으며 START를 새로 눌러야 한다.
단말을 한 번도 연결하지 않고 출발하면 이 보호는 동작하지 않는다.
여러 단말이 동시에 접속할 수 있다. 그중 하나라도 heartbeat를 보내고 있으면 STOP하지 않는다.

## Jetson 부하

remote_preview는 영상·GPU를 쓰지 않는 Python 노드 하나다. 차량 수치·경로·단계 2~4는 C++ 컨트롤러가
30Hz 요약 메시지(`/auto/preview_snapshot`, 구독자가 있을 때만 생성) 하나로 보내므로 Python은 이것과
저빈도 상태 토픽만 받는다. 중앙선 경로는 컨트롤러가 실제로 채택한 경로다.
`top -p $(pgrep -f remote_preview)`로 확인할 수 있다.
단계별 측정(`/perf/*`)은 프레임당 시간 기록 한 번과 1초에 한 번 발행하는 것이 전부다.

## 보안

인증이 없다. 같은 Wi-Fi에 있는 누구나 접속해 START/STOP을 누를 수 있다.
차량 전용 네트워크에서만 사용한다. START를 눌러도 컨트롤러의 기존 점검을 통과해야 출발한다.
조이스틱 X 수동 전환은 항상 물리 백업으로 유지한다.

## 표시 항목

- 차량 정보: 속도, 목표 속도, duty, 서보, 조향, CTE, 차선 상태/소스, 정지선 거리, 신호등
- 중앙선 경로: Jetson이 미터 좌표로 바꿔 최대 24점으로 보내고, 단말이 도로 폭을 가정해 도로를 그림
- 터널 상태: ENTERING/INSIDE이면 1초에 걸쳐 어두워지고, EXITING/OUTSIDE이면 1초에 걸쳐 밝아짐
- 단계별 연산 시간: 구간별 연산 ms(최근 1초 평균)와 FPS(= 1000 ÷ ms, 기존 BEV HUD와 같은 기준)를 표시한다.
  실제 Hz도 함께 표시한다. 카메라 행은 수신 횟수, 2·3단계는 제어기가 받은 검출 결과 횟수, 4단계는 조향 계산 완료 횟수다. 유효 차선 수신은 시간·순서·중앙선 검증을 통과해 새 경로로 채택된 결과만 센다(이전 경로 재사용 제외). 합계 행의 Hz는 합산하지 않는다.
  | 구간 | 시작 → 끝 | 출처 |
  |---|---|---|
  | 카메라 수신 | Jetson이 OAK 프레임을 받은 횟수(Hz만) | `/perf/camera_stage` |
  | 1a | Jetson 수신 → IMU 안정화 출력 | `/perf/camera_stage` (camera_driver) |
  | 1b | BEV 입력 수신 → BEV 완료 | `/perf/bev_stage` (bev_processor) |
  | 2 | BEV 검출기 입력 → 차선 추론 출력 (대기+H2D/전처리+추론) | `/auto/preview_snapshot` (auto_control이 LaneResult 시간 필드로 집계) |
  | 3 | 추론 출력 → 중앙선 결과 생성 (라벨/후처리/기하/메시지) | `/auto/preview_snapshot` |
  | 4 | 중앙선 결과 → 조향 결정 (프로세스 간 전달 + 경로·가감속·코너 판단, 장애물 결과 반영) | `/auto/preview_snapshot` (auto_control) |
  | 5 | TCN 추론 1회(ms)와 실제 추론 Hz(기본 10Hz) | `/tunnel_tcn/result` |
  | 신호등 | 추론 ms와 결과 Hz | `/traffic_light/state` |
  - 합계 1a~4는 각 구간 평균의 합이다. 구간 사이의 전달 대기와 OAK→Jetson 전송은 포함하지 않는다.
  - 합계 FPS는 1a~4를 순서대로 한 프레임 처리할 때의 환산값(1000 ÷ 합계 ms)이다.
- 시스템: CPU/GPU %, 온도, 보드 입력 전력(INA3221)
- 장애물: 아직 표시하지 않음(프레임에 `obstacles` 키를 추가할 예정)
