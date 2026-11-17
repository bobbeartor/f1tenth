# 원격 프리뷰(성능 테스트) 모드 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Jetson은 SSH로 실행하면서 숫자 정보만 30Hz로 내보내고, 프리뷰 그리기·실행 점검 표시·START/STOP은 iPad/MacBook 브라우저가 담당한다.

**Architecture:** `auto_control` 패키지에 ROS 노드 `remote_preview` 하나를 추가한다. 이 노드는 기존 토픽을 구독해 30Hz JSON 프레임을 만들고, 표준 라이브러리 HTTP 서버로 정적 웹페이지(`GET /`)와 SSE 스트림(`GET /events`)을 제공한다. START/STOP은 `POST`를 받아 기존 `/auto_control/start`, `/auto_control/stop` Trigger 서비스로 전달한다. 원격 모드에서는 Jetson의 영상 생성·발행과 OpenCV 창을 모두 끈다. 기존 `performance_measurement_enabled` 모드는 삭제한다.

**Tech Stack:** ROS 2 Humble, Python 3 표준 라이브러리(`http.server`, `json`, `threading`)만 사용한다. Jetson에 `websockets`가 없어 추가 의존성 없이 SSE를 쓴다. 클라이언트는 단일 HTML 파일(Canvas 2D, 외부 라이브러리 없음)이다.

**Spec:** 이 문서의 「요구사항」 절(사용자 요청 2026-11-13 기준).

## 요구사항

1. Jetson 접속은 NoMachine이 아니라 SSH로 한다. DISPLAY가 없어도 모든 노드가 정상 실행되어야 한다.
2. 기존 auto_control 주행 파이프라인은 그대로 실행한다. 영상 정보는 발행하지 않거나 쓰지 않고, 숫자 정보만 단말로 보낸다.
3. 프리뷰 그리기는 단말(iPad Safari, macOS 브라우저)에서 한다.
4. 필수 정보: 차량 속도, duty, 조향 서보, 정지선 거리, 기능별 평균 FPS, 중앙선 경로. 단말은 가정한 도로 폭으로 중앙선 양쪽에 도로를 그린다.
5. 추가 정보: 데이터 나이, 정지/대기 사유, 목표 속도, 조향각, 오차, 차선 상태, 신호등, 터널, VESC 연결, Jetson CPU/GPU/온도/전력, 통신 상태.
6. **장애물 정보는 이번 범위에서 제외한다.** 나중에 프레임에 `obstacles` 키를 추가하는 방식으로 확장한다.
7. 프리뷰는 20~30FPS로 그린다. 프리뷰 옆에는 실행 점검 단계(CAMERA ATTITUDE, INFERENCE MODELS, DEPARTURE)와 START/STOP 버튼만 둔다. 파라미터·타이밍 버튼과 속도 슬라이더는 만들지 않는다.
8. `performance_measurement_enabled`는 폐기한다. 쓸 만한 부분(INA3221 전력 측정, BEV 파이프라인 FPS 집계)은 새 모드로 옮긴다.

## Global Constraints

- 런치 인자 이름: `remote_preview_enabled:=true|false`, 기본값 `false`.
- HTTP 포트 파라미터 `remote_preview_port`의 기본값은 `8765`, 바인드 주소 `remote_preview_bind`의 기본값은 `0.0.0.0`.
- 프레임 송신 주기는 30Hz. 시스템 지표(CPU/GPU/온도/전력)는 1Hz로 갱신해 매 프레임에 마지막 값을 넣는다.
- 중앙선은 미터 단위 `[x_forward_m, y_left_m]` 배열로 보내고, 최대 24점으로 줄인다. 소수점은 3자리로 반올림한다.
- 가정 도로 폭은 `line_detactor`의 `centerline_lane_width_m`(현재 0.65)과 같은 값을 파라미터 `remote_preview_lane_width_m`으로 받는다.
- 원격 모드에서 다음이 Jetson에 없어야 한다: OpenCV 창(`imshow`), `/line_detactor/result_image` 발행, `/bev_processor/startup_preview` 발행, `startup_inspection`·`runtime_tools` 프로세스.
- START 승인 경로는 바꾸지 않는다. 원격 노드는 Trigger 서비스만 호출하고 duty·enable 토픽은 절대 발행하지 않는다.
- 연결 끊김 시 STOP: 컨트롤러 상태가 `STARTED`인 동안 heartbeat가 `remote_link_loss_stop_sec`(기본 1.0초)보다 오래 없으면 `/auto_control/stop`을 1회 호출한다. 이 값을 `0`으로 두면 기능이 꺼진다.
- 새 Python 의존성은 추가하지 않는다.

## Review Focus

1. **Wi-Fi가 순간적으로 끊긴 뒤 재연결되는 경우:** 끊김 STOP은 1회만 호출되어야 하고, 재연결 후 자동으로 다시 출발하면 안 된다. START는 새로 눌러야 한다. 테스트는 Task 3에 있다.
2. **단말 두 대(iPad + Mac)가 동시에 접속한 경우:** 두 단말 모두 프레임을 받는다. heartbeat는 아무 단말에서든 최근 것을 인정한다. 둘 중 하나만 꺼져도 STOP하지 않는다. 테스트는 Task 3과 Task 4에 있다.
3. **LaneResult의 중앙선이 무효(`centerline_valid=false`)이거나 점이 1개 이하인 경우:** `points: []`로 보낸다. 단말은 이전 경로를 계속 그리지 않고 "NO PATH"를 표시한다. 테스트는 Task 2에 있다.
4. **토픽이 끊긴 경우(예: 신호등 노드 크래시):** 해당 값은 `null`과 함께 나이(`age_ms`)를 보낸다. 단말은 500ms 넘게 갱신이 없는 값을 회색 `--`로 표시하며, 0으로 보이면 안 된다. 테스트는 Task 2와 Task 5에 있다.
5. **느린 단말이나 백그라운드 탭 때문에 SSE 쓰기가 막히는 경우:** 송신 스레드가 막혀 ROS 콜백이 지연되면 안 된다. 각 클라이언트 큐는 최신 1개만 유지하고 오래된 프레임은 버린다. 테스트는 Task 4에 있다.

---

## File Structure

| 파일 | 책임 |
|---|---|
| `src/auto_control/auto_control_tunnel/remote_frame.py` (생성) | ROS와 무관한 순수 함수. 중앙선 px→m 변환과 축약, 1초 이동 평균(`RateWindow`), 프레임 dict 조립 |
| `src/auto_control/auto_control_tunnel/remote_link.py` (생성) | ROS와 무관한 heartbeat 감시(`LinkWatchdog`), 클라이언트별 최신 1개 큐(`LatestSlot`) |
| `src/auto_control/auto_control_tunnel/remote_http.py` (생성) | HTTP/SSE 서버. 백엔드 인터페이스로만 ROS 쪽과 통신 |
| `src/auto_control/auto_control_tunnel/remote_preview.py` (생성) | ROS 노드. 구독, 30Hz 타이머, 서비스 호출, 서버 기동 |
| `src/auto_control/auto_control_tunnel/web/remote_preview.html` (생성) | 단말 UI. Canvas BEV, 수치 HUD, 점검 단계, START/STOP, heartbeat |
| `src/auto_control/auto_control_tunnel/compute_status.py` (수정) | INA3221 전력 측정 추가(기존 C++ 측정 코드에서 이식) |
| `src/auto_control/scripts/remote_preview` (생성) | 실행 진입점 |
| `src/auto_control/auto_control_tunnel/launch_support.py` (수정) | `remote_preview_node()` 추가 |
| `src/vehicle_bringup/launch/auto_drive.launch.py` (수정) | `performance_measurement_*` 삭제, `remote_preview_enabled` 추가 |
| `src/bev_processor/...`, `src/camera_driver/...`, `src/auto_control/src/...` (수정/삭제) | 기존 측정 모드 삭제와 파이프라인 FPS 토픽화 |
| `src/auto_control/REMOTE_PREVIEW_KO.md` (생성) | 실행 방법(SSH, 접속 URL) |

---

### Task 1: 기존 performance_measurement 폐기 + 재사용 부분 이식

**Files:**
- Modify: `src/auto_control/auto_control_tunnel/compute_status.py` (`SystemProbe.sample`에 `power_w` 추가)
- Delete: `src/auto_control/src/performance_measurement.cpp`, `src/auto_control/include/auto_control/performance_measurement.hpp`, `src/auto_control/auto_control/performance_measurement.py`
- Modify: `src/auto_control/CMakeLists.txt:68`, `src/auto_control/src/auto_control_node.cpp`(`measurement_` 관련 48곳), `src/auto_control/auto_control/auto_control_node.py:33,168-200,341-355`, `src/auto_control/config/auto_control.yaml`
- Modify: `src/bev_processor/src/bev_processor_node.cpp:119,139,462,588,634,1785,1812,1910`, `src/bev_processor/launch/bev_processor.launch.py:109,147,207,258`, `src/bev_processor/config/bev_config.yaml`
- Modify: `src/camera_driver/src/camera_driver_node.cpp:311,457,702,1832,2572,3023,3154`, `src/camera_driver/config/camera_config.yaml`
- Modify: `src/ir_bev_line_detection/ir_bev_line_detection/settings.py:48`, `src/roi_vfh_test/config/reference/*.yaml`
- Modify: `src/vehicle_bringup/launch/auto_drive.launch.py:96,247,346-356,453,509,746,765-790,801-813,910-935`, `src/vehicle_bringup/test/test_handover_launch.py:38,68`
- Modify: README 4곳(`auto_control`, `bev_processor`, `camera_driver`, `line_detactor`, `vehicle_bringup`), `src/JETSON_RUN_COMMANDS.txt`
- Test: `src/auto_control/test/tunnel/test_compute_status.py`

**Interfaces:**
- Produces: `SystemProbe.sample()` 결과에 `power_w: float | None` 키 추가. INA3221 채널 합계(W)이며, 센서가 없으면 키를 넣지 않는다.
- Produces: `bev_processor` 파라미터 `startup_preview_image_enabled: bool`(기본 `true`). `false`이면 `~/startup_preview` 퍼블리셔를 만들지 않는다. `~/startup_status`는 계속 발행한다.
- Produces: `bev_processor` 토픽 `/bev_processor/pipeline_fps`(`std_msgs/String`, 1Hz, 내용 `{"stabilized_fps": float, "bev_ready_fps": float}`). 기존 `[PERF][PIPELINE]` 집계를 항상 켜고, 로그 대신 이 토픽으로 낸다.

- [ ] **Step 1: 전력 측정 실패 테스트 작성**

`test_compute_status.py`에 `test_ina3221_power_sums_channels_and_absent_sensor_omits_key`를 추가한다. 임시 `sys` 트리를 만든다. 첫 번째 경우는 `bus/i2c/drivers/ina3221/1-0040/hwmon/hwmon3/in1_input=5000`, `curr1_input=1000`, `in2_input=5000`, `curr2_input=400`이다(mV, mA 단위). 이때 `sample()['power_w'] == pytest.approx(7.0)`이어야 한다. 두 번째 경우는 hwmon 디렉터리가 없을 때이며, `'power_w' not in sample()`이어야 한다. `performance_measurement.cpp:177-225`의 두 번째 경로 형식(`in_power<N>_input`, mW 단위)도 같은 테스트에서 1건 확인한다.

- [ ] **Step 2: 실패 확인**

Run: `cd src/auto_control && python3 -m pytest test/tunnel/test_compute_status.py -k ina3221 -v`
Expected: FAIL (`KeyError: 'power_w'`)

- [ ] **Step 3: `SystemProbe._power_w() -> float | None` 구현**

`performance_measurement.cpp:177-225`의 탐색 규칙(hwmon `in*_input`×`curr*_input` 또는 iio `in_power*_input`)을 Python으로 옮긴다. `sample()`에서 값이 있을 때만 `result['power_w']`를 넣는다.

- [ ] **Step 4: 통과 확인**

Run: `python3 -m pytest test/tunnel/test_compute_status.py -v`
Expected: PASS(기존 테스트 포함)

- [ ] **Step 5: 측정 모드 코드 삭제**

위 Files 목록의 `performance_measurement*` 참조를 모두 지운다. 각 노드의 동작은 다음과 같이 정리한다.
- `camera_driver`: 측정 모드가 하던 `preview_enabled_=false`는 기존 `preview_enabled` 파라미터로 대체한다. 원격 모드에서는 Task 6의 런치가 이 값을 `false`로 넘긴다.
- `bev_processor`: `startup_inspection && !performance_measurement_enabled_` 조건을 `startup_inspection && startup_preview_image_enabled`로 바꾼다. `[PERF][PIPELINE]` 집계 블록(1785, 1812, 1910 부근)에서 측정 모드 조건을 빼고, 1초마다 `/bev_processor/pipeline_fps`로 발행하게 바꾼다.
- `auto_drive.launch.py`: `performance_shutdown_handler`, 측정 관련 `DeclareLaunchArgument` 6개와 `startup_inspection_active` 계산을 지운다. `startup_inspection_active`는 항상 `"true"`로 둔다.
- `test_handover_launch.py:68`: `{"performance_measurement_enabled": "true"}` 케이스를 지운다.

- [ ] **Step 6: 남은 참조가 없는지 확인**

Run: `git grep -n "performance_measurement" -- src ':!src/auto_control/tools'`
Expected: 출력 없음

- [ ] **Step 7: 빌드와 테스트 (Jetson 또는 ROS 환경)**

Run: `colcon build --packages-select auto_control bev_processor camera_driver vehicle_bringup ir_bev_line_detection && colcon test --packages-select auto_control vehicle_bringup && colcon test-result --verbose`
Expected: 빌드 성공, 테스트 0 failures

- [ ] **Step 8: Commit**

```bash
git add -A src/auto_control src/bev_processor src/camera_driver src/vehicle_bringup src/ir_bev_line_detection src/roi_vfh_test src/line_detactor/README.md src/JETSON_RUN_COMMANDS.txt
git commit -m "Remove performance measurement mode; keep power probe and BEV pipeline FPS topic"
```

---

### Task 2: 프레임 조립 순수 함수 (`remote_frame.py`)

**Files:**
- Create: `src/auto_control/auto_control_tunnel/remote_frame.py`
- Test: `src/auto_control/test/tunnel/test_remote_frame.py`

**Interfaces:**
- Produces:
  - `centerline_metric(points, source_width, source_height, padding_left, bev_width_m, bev_height_m, max_points=24) -> list[list[float]]`. `points`는 `(x, y)` 픽셀 튜플의 순서 있는 시퀀스다. 변환식은 컨트롤러(`auto_control_node.cpp:1019-1021`)와 같다. `x_m = bev_height_m - (v + 0.5) * (bev_height_m / source_height)`, `y_m = bev_width_m/2 - (u - padding_left + 0.5) * (bev_width_m / source_width)`. 점이 `max_points`보다 많으면 첫 점과 끝 점을 포함해 같은 간격으로 고른다. 입력이 2점 미만이거나 기하값이 0 이하·NaN이면 `[]`를 반환한다.
  - `class RateWindow(window_sec=1.0)`: `add(now: float, value: float | None = None)`, `fps(now) -> float | None`(창 안 이벤트 수 ÷ 실제 시간 폭, 2개 미만이면 `None`), `mean(now) -> float | None`(창 안 value 평균), `age_ms(now) -> float | None`(마지막 이벤트 이후 시간).
  - `build_frame(seq: int, now: float, snap: dict) -> dict`: 아래 스키마를 그대로 따른다. 키는 항상 존재하고, 값이 없거나 500ms보다 오래된 값은 `None`이다.

```text
{
  "v": 1, "seq": int, "t": float(Jetson monotonic s),
  "vehicle": {"speed_mps","target_speed_mps","duty","brake_current_a","servo",
              "steering_rad","raw_steering_rad","corner_state","cte_m","heading_rad",
              "curvature","vesc_connected","age_ms"},
  "lane": {"points":[[x,y],...],"valid":bool,"state":int|None,"source":str,
           "stop_line_m":float|None,"lane_width_m":float,"age_ms"},
  "fps": {"camera","bev","lane","inference_ms","postprocess_ms","control_ms",
          "traffic","tcn"},
  "traffic": {"state": "UNKNOWN"|"RED"|"GREEN"|None, "age_ms"},
  "tunnel": {"state": "OUTSIDE"|"ENTERING"|"INSIDE"|"EXITING"|None,
             "lines": [str,str,str] | None},
  "system": {"cpu_pct","gpu_pct","cpu_temp_c","gpu_temp_c","power_w"},
  "startup": {"camera": str, "models": {name: str}, "controller": str,
              "can_start": bool, "started": bool, "message": str},
  "link": {"clients": int, "heartbeat_age_ms": float|None, "loss_stop_sec": float}
}
```

- [ ] **Step 1: 실패 테스트 작성**

다음 테스트를 작성한다.
- `test_centerline_matches_controller_formula`: 입력은 `points=[(30.5,299.5),(30.5,0.5)]`, `source_width=120, source_height=300, padding_left=0, bev_width_m=1.2, bev_height_m=3.0`이다. 결과가 `[[0.0, 0.29], [2.99, 0.29]]`이어야 한다(`abs=1e-3`).
- `test_centerline_downsamples_keeping_endpoints`: 100점을 넣으면 길이 24이고, 첫 점과 끝 점이 원본 변환값과 같아야 한다.
- `test_centerline_invalid_returns_empty`: 1점 입력, `source_height=0`, `bev_width_m=float('nan')`이면 각각 `[]`이어야 한다.
- `test_rate_window_fps_and_mean`: t=0.0, 0.1, …, 0.9에 value=2.0을 10번 add했을 때, `fps(0.9) == approx(10.0, rel=0.12)`, `mean(0.9) == 2.0`, `age_ms(1.0) == approx(100)`이어야 한다. 1.95초 뒤에는 `fps(2.95) is None`이어야 한다.
- `test_build_frame_marks_stale_values_none`: `snap['vehicle']['speed_mps']=(1.2, at=now-0.6)`이면 `None`, `at=now-0.1`이면 `1.2`이어야 한다. 모든 최상위 키가 존재해야 한다.

- [ ] **Step 2: 실패 확인**

Run: `python3 -m pytest test/tunnel/test_remote_frame.py -v`
Expected: FAIL (`ModuleNotFoundError: auto_control_tunnel.remote_frame`)

- [ ] **Step 3: 구현**

`snap`에는 각 항목을 `(value, received_monotonic)` 튜플로 담는다. `build_frame`은 신선도를 판정하고 반올림만 한다(3자리). 계산은 하지 않는다.

- [ ] **Step 4: 통과 확인**

Run: `python3 -m pytest test/tunnel/test_remote_frame.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/auto_control/auto_control_tunnel/remote_frame.py src/auto_control/test/tunnel/test_remote_frame.py
git commit -m "Add remote preview frame builder"
```

---

### Task 3: 연결 감시와 클라이언트 큐 (`remote_link.py`)

**Files:**
- Create: `src/auto_control/auto_control_tunnel/remote_link.py`
- Test: `src/auto_control/test/tunnel/test_remote_link.py`

**Interfaces:**
- Produces:
  - `class LinkWatchdog(timeout_sec: float)`: `heartbeat(now)`, `heartbeat_age_ms(now) -> float | None`, `should_stop(now, started: bool) -> bool`. `True`는 손실 1회당 한 번만 반환한다. 다시 `True`를 반환할 수 있으려면 다음 heartbeat를 받은 뒤 다시 `started=True` 상태에서 손실이 일어나야 한다. `timeout_sec <= 0`이면 항상 `False`를 반환한다. 아직 heartbeat를 한 번도 받지 못했으면 `False`를 반환한다(단말 없이 출발한 경우 보호하지 않음; 문서에 명시).
  - `class LatestSlot`: `put(item)`(이전 항목 덮어쓰기), `get(timeout: float) -> item | None`, `close()`(이후 `get`은 즉시 `None`).

- [ ] **Step 1: 실패 테스트 작성**

다음 테스트를 작성한다.
- `test_watchdog_fires_once_per_loss`: heartbeat(0) 후 `should_stop(1.2, True)`는 True, `should_stop(1.5, True)`는 False여야 한다. heartbeat(2.0) 후 `should_stop(3.1, True)`는 다시 True여야 한다.
- `test_watchdog_ignores_when_not_started_or_disabled`: `should_stop(5, False)`는 False여야 한다. `LinkWatchdog(0).should_stop(99, True)`도 False여야 한다.
- `test_watchdog_no_heartbeat_yet_does_not_fire`: heartbeat 없이 `should_stop(10, True)`는 False여야 한다.
- `test_watchdog_any_client_heartbeat_counts`: 두 단말이 번갈아 0.6초 간격으로 heartbeat하면 3초 동안 False여야 한다.
- `test_latest_slot_drops_old_items`: put(1), put(2), put(3) 후 get은 3이고, 다음 `get(0.01)`은 None이어야 한다. close 후 get은 즉시 None이어야 한다.

- [ ] **Step 2: 실패 확인**

Run: `python3 -m pytest test/tunnel/test_remote_link.py -v`
Expected: FAIL (모듈 없음)

- [ ] **Step 3: 구현** (`LatestSlot`은 `threading.Condition` 하나로 만든다.)

- [ ] **Step 4: 통과 확인**

Run: `python3 -m pytest test/tunnel/test_remote_link.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/auto_control/auto_control_tunnel/remote_link.py src/auto_control/test/tunnel/test_remote_link.py
git commit -m "Add remote link watchdog and latest-only client slot"
```

---

### Task 4: HTTP/SSE 서버 (`remote_http.py`)

**Files:**
- Create: `src/auto_control/auto_control_tunnel/remote_http.py`
- Test: `src/auto_control/test/tunnel/test_remote_http.py`

**Interfaces:**
- Consumes: Task 3의 `LatestSlot`, `LinkWatchdog.heartbeat`
- Produces:
  - `class RemoteBackend(Protocol)`: `request_start() -> tuple[bool, str]`, `request_stop() -> tuple[bool, str]`, `heartbeat() -> None`. 두 request 메서드는 블로킹이며 서비스 응답까지 최대 2초 기다린다.
  - `class RemoteServer(bind: str, port: int, html_path: str, backend: RemoteBackend)`: `start()`(데몬 스레드에서 `ThreadingHTTPServer` 실행), `broadcast(frame: dict)`(각 클라이언트의 `LatestSlot`에 직렬화된 JSON을 put), `client_count() -> int`, `close()`.
  - 라우트:
    - `GET /`: HTML (`text/html; charset=utf-8`)
    - `GET /events`: `text/event-stream`, `Cache-Control: no-cache`. 각 메시지는 `data: <json>\n\n`. 2초 동안 프레임이 없으면 `: ping\n\n`을 보낸다.
    - `POST /start`, `POST /stop`: 응답 `{"ok": bool, "message": str}`
    - `POST /heartbeat`: 204
    - 그 외 경로: 404

- [ ] **Step 1: 실패 테스트 작성**

포트 0으로 띄우고 `http.client`로 검증한다.
- `test_index_serves_html`: 상태 200이고 본문에 `<canvas`가 있어야 한다.
- `test_events_stream_delivers_latest_frame_only`: 연결한 뒤 `broadcast({'seq':1})`, `broadcast({'seq':2})`를 연속 호출한다. 첫 수신 `data:`의 seq는 2여야 한다(1이 와도 그 뒤 2가 와야 한다. 최종 수신값이 2인지 확인).
- `test_slow_client_does_not_block_broadcast`: 읽지 않는 SSE 클라이언트를 연결한 상태에서 `broadcast`를 1000회 호출해도 0.5초 안에 끝나야 한다.
- `test_start_stop_forward_to_backend`: 가짜 backend가 `(False, 'waiting_for_camera')`를 반환하면 POST /start 응답 JSON이 `{"ok": false, "message": "waiting_for_camera"}`이어야 한다. POST /stop 1회에 backend 호출도 1회여야 한다.
- `test_heartbeat_calls_backend_and_two_clients_counted`: SSE 2개를 연결하면 `client_count() == 2`, 하나를 닫은 뒤 1초 안에 1이 되어야 한다.
- `test_unknown_path_404`

- [ ] **Step 2: 실패 확인**

Run: `python3 -m pytest test/tunnel/test_remote_http.py -v`
Expected: FAIL (모듈 없음)

- [ ] **Step 3: 구현**

핸들러 스레드마다 `LatestSlot`을 하나씩 등록한다. 쓰기가 실패하면(`BrokenPipeError`, `ConnectionResetError`) 해당 슬롯을 제거한다. `broadcast`는 JSON 직렬화를 1회만 하고 슬롯에 put만 한다.

- [ ] **Step 4: 통과 확인**

Run: `python3 -m pytest test/tunnel/test_remote_http.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/auto_control/auto_control_tunnel/remote_http.py src/auto_control/test/tunnel/test_remote_http.py
git commit -m "Add stdlib HTTP/SSE server for remote preview"
```

---

### Task 5: 단말 UI (`web/remote_preview.html`)

**Files:**
- Create: `src/auto_control/auto_control_tunnel/web/remote_preview.html`
- Create: `src/auto_control/test/tunnel/remote_preview_geometry.test.mjs`
- Modify: `src/auto_control/setup.py` 또는 `CMakeLists.txt`의 install 규칙(`web/*.html`을 패키지 데이터로 설치)

**Interfaces:**
- Consumes: Task 2의 프레임 스키마, Task 4의 라우트
- Produces: HTML 안 `<script type="module">`에서 export하는 순수 함수 두 개. 테스트는 이 블록을 추출해 import한다.
  - `laneEdges(points, widthM) -> {left: [[x,y]], right: [[x,y]]}`: 각 점에서 진행 방향의 법선 방향으로 ±widthM/2만큼 옮긴다. 진행 방향은 인접한 두 점의 차분으로 구하고, 끝 점은 한쪽 차분만 쓴다. 점이 2개 미만이면 `{left: [], right: []}`.
  - `display(value, digits, ageMs) -> string`: `value == null`이거나 `ageMs > 500`이면 `"--"`, 그 외에는 `value.toFixed(digits)`.
  - `tunnelTarget(state, current) -> 0|1|current`: `ENTERING`/`INSIDE`는 1(어두움), `EXITING`/`OUTSIDE`는 0(밝음), `null`이면 현재 값을 유지한다.
  - `approach(current, target, dtSec, tauSec=1/Math.log(20)) -> number`: 1차 지연으로 접근한다. 1.0초에 95% 도달하며, 프레임레이트와 무관하다.

화면 구성(가로 기준, iPad 세로 화면에서는 위아래로 쌓음):
- **왼쪽(약 65%) BEV Canvas**: 차량 기준으로 전방 3.0m, 좌우 ±0.6m를 그린다(`bev_x_max_m`, `bev_y_max_m`와 같은 범위). 도로 면은 `laneEdges`로 만든 다각형으로 채우고, 중앙선은 점선으로 그린다. 정지선 거리가 있으면 그 x 위치에 가로선을 그린다. 차체 사각형도 그린다. `requestAnimationFrame`으로 그리며 실제 그리기 FPS를 화면 구석에 표시한다. `points`가 비어 있으면 "NO PATH"를 표시한다.
- **BEV 위 수치 오버레이**: 속도(m/s), 목표 속도, duty, 서보, 조향각, 정지선 거리, 신호등 원.
- **오른쪽(약 35%) 패널**:
  1. 실행 점검 3단계(CAMERA ATTITUDE / INFERENCE MODELS(모델별 상태) / DEPARTURE). 각 단계는 PASS, 진행 중, FAIL 색으로 구분한다.
  2. 컨트롤러 상태 문자열(`startup.controller`)과 마지막 START/STOP 응답 메시지.
  3. START 버튼(`can_start`가 false이면 비활성)과 STOP 버튼(항상 활성, 크게).
  4. 기능별 FPS 표(camera, BEV, lane, inference ms, post ms, control ms, traffic, TCN, preview(로컬)).
  5. 시스템(CPU/GPU %, 온도, 전력 W)과 링크(데이터 나이 ms, 연결 단말 수).
- **터널 명암 전환**: `tunnel.state`에 따라 BEV 카드의 팔레트를 주간(DAY)과 터널(NIGHT) 사이에서 `approach`로 보간한다(배경, 도로, 가장자리, 중앙선, 차체, HUD 글자색). 어두운 정도에 비례해 다음을 함께 표시한다.
  - 가장자리 비네트
  - 양쪽 벽의 0.5m 간격 주황 조명. `speed_mps`를 단말에서 적분해 아래로 흘려보낸다.
  - 상단 `TUNNEL · IR` 배지
  - 오른쪽 패널은 밝게 유지한다. 시안은 `docs/superpowers/mockups/remote_preview_sample.html`이다.
- **heartbeat**: `setInterval`로 200ms마다 `POST /heartbeat`를 보낸다. `EventSource`가 끊기면 화면 전체를 "LINK LOST"로 덮고 자동 재연결한다.
- **스타일**: 기존 daylight 테마 색(흰 배경, 회색 잉크)과 맞춘다.

- [ ] **Step 1: 실패 테스트 작성**

다음 테스트를 작성한다.
- `laneEdges([[0,0],[1,0]], 0.6)`: left가 `[[0,0.3],[1,0.3]]`, right가 `[[0,-0.3],[1,-0.3]]`이어야 한다.
- `laneEdges([[0,0]], 0.6)`: left와 right가 모두 비어 있어야 한다.
- `tunnelTarget('ENTERING',0)===1`, `tunnelTarget('EXITING',1)===0`, `tunnelTarget(null,0.4)===0.4`이어야 한다. `approach(0,1,0.6)`은 `approx(0.8343, 1e-3)`, `approach(0,1,1.0)`은 `approx(0.95, 1e-3)`, `approach(0.5,0.5,1)===0.5`이어야 한다.
- `display(null,2,0)`는 `"--"`, `display(1.234,2,600)`은 `"--"`, `display(1.234,2,100)`은 `"1.23"`이어야 한다.

- [ ] **Step 2: 실패 확인**

Run: `node --test src/auto_control/test/tunnel/remote_preview_geometry.test.mjs`
Expected: FAIL (파일 없음)

- [ ] **Step 3: HTML 작성**

- [ ] **Step 4: 통과 확인**

Run: `node --test src/auto_control/test/tunnel/remote_preview_geometry.test.mjs`
Expected: PASS

- [ ] **Step 5: Mac에서 가짜 프레임으로 눈으로 확인**

Task 4의 `RemoteServer`를 가짜 backend와 1초 주기 S자 중앙선 프레임으로 띄우는 스크래치 스크립트를 실행한다. 브라우저에서 `http://localhost:8765`를 열고 다음을 확인한다.
- 그리기 FPS가 25 이상이다.
- 차선 면이 그려진다.
- 신호등 데이터를 멈추면 0.5초 뒤 `--`로 바뀐다.
- 서버를 끄면 LINK LOST가 나온다.
- 375px 폭에서도 START/STOP이 보인다.

- [ ] **Step 6: Commit**

```bash
git add src/auto_control/auto_control_tunnel/web src/auto_control/test/tunnel/remote_preview_geometry.test.mjs src/auto_control/setup.py src/auto_control/CMakeLists.txt
git commit -m "Add browser remote preview UI"
```

---

### Task 6: ROS 노드 + 런치 통합

**Files:**
- Create: `src/auto_control/auto_control_tunnel/remote_preview.py`, `src/auto_control/scripts/remote_preview`
- Modify: `src/auto_control/CMakeLists.txt`(scripts 설치), `src/auto_control/auto_control_tunnel/launch_support.py`, `src/vehicle_bringup/launch/auto_drive.launch.py`
- Create: `src/auto_control/REMOTE_PREVIEW_KO.md`; Modify: `src/JETSON_RUN_COMMANDS.txt`, `src/auto_control/STARTUP_INSPECTION.md`(원격 모드 1문단)
- Test: `src/auto_control/test/tunnel/test_launch.py`, `src/vehicle_bringup/test/test_handover_launch.py`

**Interfaces:**
- Consumes:
  - Task 2: `centerline_metric`, `RateWindow`, `build_frame`
  - Task 3: `LinkWatchdog`
  - Task 4: `RemoteServer`, `RemoteBackend`
  - Task 1: `SystemProbe`, `/bev_processor/pipeline_fps`
  - 기존: `InspectionState`(`inspection_state.py`). `update_model`, `update_controller`, `update_link`, `can_start()`, `checks_ready()`를 그대로 쓴다.
- Produces: `remote_preview_node(tunnel_enabled: bool, traffic_enabled: bool, connection_diagnostics_topic: str) -> Node`. `expected_nodes` 계산은 `startup_inspection_node`와 같은 규칙이며, 공통 헬퍼 `_expected_nodes(tunnel, traffic)`로 뽑아 두 함수가 함께 쓴다.

구독 목록(전부 기존 토픽):

| 프레임 필드 | 토픽 | 타입 |
|---|---|---|
| speed / target / duty / brake / servo | `/auto/current_speed`, `/auto/target_speed`, `/auto/current_duty`, `/auto/current_brake_current`, `/auto/current_servo_position` | Float32 |
| steering / raw / cte / heading / curvature | `/auto/steering_angle_rad`, `/auto/raw_steering_angle_rad`, `/auto/cross_track_error_m`, `/auto/heading_error_rad`, `/auto/path_curvature` | Float32 |
| corner_state | `/auto/corner_state` | UInt8 |
| vesc_connected | `/vesc/connected` | Bool |
| lane.*, fps.lane/inference/postprocess | `/line_detactor/result` | LaneResult |
| fps.control_ms | `/auto/detector_input_to_control_decision_ms` | Float32 |
| fps.camera / bev | `/bev_processor/pipeline_fps` | String(JSON) |
| traffic, fps.traffic | `/traffic_light/state` | TrafficLightState |
| tunnel.state / lines, fps.tcn | `/tunnel/preview_status` | String(3줄). 첫 줄 `Tunnel: <STATE>`에서 state 파싱 |
| startup.* | `/bev_processor/startup_status`, `/auto_control/startup_state`, `/<node>/model_status`, `/vesc/connected/diagnostics` | String |

FPS 정의(1초 `RateWindow` 기준):
- lane: LaneResult 도착 빈도
- inference_ms: `pure_inference_nanoseconds`의 평균
- postprocess_ms: `label_export + backend_postprocess + lane_geometry` 합의 평균
- control_ms: 위 토픽 값의 평균
- traffic: `source_generation`이 바뀐 메시지의 빈도
- tcn: `/tunnel/preview_status` 둘째 줄의 `TCN avg (x)FPS`를 그대로 파싱
- LaneResult 콜백에서는 숫자 필드와 축약된 중앙선만 저장한다. 메시지 전체는 보관하지 않는다.

런치(`remote_preview_enabled:=true`)가 바꾸는 값:
- `detector["preview_enabled"]=False`
- `preview_result_only_enabled="false"`
- camera_driver의 `preview_enabled=false`
- bev_processor의 `startup_preview_image_enabled=false`
- `startup_inspection` 노드와 `runtime_tools` 노드를 만들지 않는다.
- 대신 `remote_preview_node(...)`를 추가한다.
- `startup_inspection_active`와 컨트롤러의 `startup_safety_enabled`는 `true`로 유지한다. START 승인이 계속 필요하다.
- `manual_test:=true`와 함께 써도 된다. 이때 START 단계는 기존처럼 MANUAL 통과로 표시된다.

- [ ] **Step 1: 런치 실패 테스트 작성**

다음 테스트를 작성한다.
- `test_remote_preview_disables_jetson_rendering`: `remote_preview_enabled:=true`로 런치 설명을 생성한다. 그 결과에서 다음을 확인한다.
  - 노드 이름 `remote_preview`가 있다.
  - `startup_inspection`, `auto_runtime_tools`가 없다.
  - line_detactor 파라미터 `preview_enabled is False`다.
  - camera_driver 파라미터 `preview_enabled == 'false'`다.
  - bev_processor 파라미터 `startup_preview_image_enabled == 'false'`다.
  - auto_control 파라미터 `startup_safety_enabled`가 true다.
- `test_default_launch_keeps_local_inspection`: 인자를 주지 않으면 `startup_inspection`이 있고 `remote_preview`는 없어야 한다.

- [ ] **Step 2: 실패 확인**

Run: `colcon test --packages-select vehicle_bringup auto_control --pytest-args -k remote_preview && colcon test-result --verbose`
Expected: FAIL

- [ ] **Step 3: 노드 구현 `RemotePreviewNode(Node)`**

노드가 `RemoteBackend`를 구현한다. `request_start`와 `request_stop`은 `call_async` 뒤 `threading.Event`로 최대 2초 기다린다. HTTP 스레드에서 호출되므로 노드는 `MultiThreadedExecutor`로 spin한다. 30Hz 타이머에서 하는 일은 다음과 같다.
1. `build_frame`
2. `server.broadcast`
3. `watchdog.should_stop(now, started)`가 True이면 `request_stop`을 비동기로 호출하고 `startup.message`에 `"LINK LOST: STOP sent"`를 표시한다.

`started` 판정은 `startup_state`가 `STARTED`로 시작하는지로 한다. `SystemProbe`는 1Hz 별도 스레드에서 돈다(`ComputeMonitor` 방식). 파라미터는 다음과 같다: `remote_preview_port=8765`, `remote_preview_bind='0.0.0.0'`, `remote_preview_lane_width_m=0.65`, `remote_link_loss_stop_sec=1.0`, `expected_nodes`, `connection_diagnostics_topic`.

- [ ] **Step 4: 런치 연결 구현 후 통과 확인**

Run: Step 2와 같은 명령
Expected: PASS

- [ ] **Step 5: Jetson 실차 무부하 확인(바퀴를 띄운 상태)**

Jetson에서 SSH로 다음을 실행한다.

```bash
ssh <jetson> 'cd ~/Desktop/1030ML/f1tenth_project_repo && source install/setup.bash && ros2 launch vehicle_bringup auto_drive.launch.py remote_preview_enabled:=true'
```

Mac과 iPad에서 `http://<jetson-ip>:8765`를 열고 다음을 확인한다.
- 점검 3단계가 기존 창과 같은 순서로 PASS된다.
- START → STARTED가 표시된다.
- STOP → STOPPED가 표시된다.
- 주행 중 iPad Wi-Fi를 끄면 1초 안에 STOP 로그가 찍히고, Wi-Fi를 다시 켜도 출발하지 않는다.
- `ros2 topic list`에 `/line_detactor/result_image`와 `/bev_processor/startup_preview`가 없다.
- 기존 NoMachine+로컬 창 실행과 lane FPS·inference ms를 비교해 기록한다.

- [ ] **Step 6: 문서**

`REMOTE_PREVIEW_KO.md`에 다음을 쓴다.
- 실행 명령
- 접속 URL
- 끊김 STOP 동작과 0으로 끄는 방법
- 단말 없이 출발하면 끊김 보호가 없다는 점
- 같은 Wi-Fi의 누구나 접속할 수 있다는 점(차량 전용 네트워크 사용 권장)
- 장애물은 미구현(`obstacles` 키 예약)이라는 점

`JETSON_RUN_COMMANDS.txt`에는 명령 1줄을 추가한다.

- [ ] **Step 7: Commit**

```bash
git add src/auto_control src/vehicle_bringup src/JETSON_RUN_COMMANDS.txt
git commit -m "Add remote_preview mode: numeric telemetry over SSE with START/STOP"
```

---

## 나중에 추가할 것 (이번 범위 아님)

- 장애물: 프레임에 `"obstacles": [{"x_m","y_m","w_m","l_m","class","conf"}]` 키를 추가한다. 위치·크기 계산은 Jetson에서 `object_localization.hpp`를 써서 하고, 단말은 사각형만 그린다.
