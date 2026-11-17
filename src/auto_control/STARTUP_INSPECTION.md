# 출발 전 점검과 시작 승인

`vehicle_bringup auto_drive.launch.py`는 일반 실행에서 출발 전 점검 창을 자동으로 연다.
기존 YAML이나 `auto_enabled:=true`를 사용해도 점검과 사용자 승인 전에는 자율주행 duty를 허용하지 않는다.

## 표시 순서

1. CAMERA ATTITUDE: 처음 카메라가 비추는 측정 영상을 표시한다. 자동 지면 모드는
   깊이 영상과 빨간 측정 ROI를 보여 주고, 수동 높이 모드는 RGB 영상과 IMU 자세 측정을
   사용한다. 실제 측정 완료 후 높이/roll/pitch와 PASS를 표시한다.
2. INFERENCE MODELS: 현재 실행에서 필요한 RGB/IR/TCN/신호등 모델별 WAITING,
   LOADING, READY, FAILED를 표시한다. 엔진 생성·시험 추론을 완료해야 READY다.
3. DEPARTURE: 카메라 측정, 모델 준비 heartbeat, VESC 연결, 신선한 ERPM,
   유효한 최신 차선과 기존 제어 입력 조건이 모두 충족되면 START 버튼을 활성화한다.
   클릭 또는 Enter 입력 시 C++ 제어기가 그 시점의 조건을 다시 검사한다.
4. C++ 제어기의 승인이 확인되면 START 영역에 주행 상태를 표시하고 점검 창을 유지한다.
   창 하단의 STOP, Timings, Params와 최대 속력 가로 슬라이더를 주행 중에도 사용할 수 있다.
   Timings/Params는 기존 별도 도구 창을 열며 Params의 프리셋 저장도 그대로 사용한다.

점검 창은 기존 1200×860에서 가로·세로를 각각 15% 줄인 1020×731로 표시한다.
버튼과 슬라이더의 마우스 좌표는 축소된 표시 크기에서 원래 도면 좌표로 환산한다.
BEV 프리뷰에는 조작 패널을 붙이지 않는다. Q/Esc 또는 창 닫기는 점검·조작 창만 닫으며
별도의 STOP 요청이나 START 승인을 보내지 않는다.

측정이 끝난 카메라 영상은 마지막 영상으로 고정하고 frozen으로 표시한다.
점검 창은 별도 프로세스/메인 스레드에서 그리며 기존 BEV·모델 로딩을 막지 않는다.
UI 영상 출력은 관측용이며 측정 품질 기준이나 카메라 EEPROM은 바꾸지 않는다.

## duty 허용 조건

GUI는 duty 토픽이나 /auto/enabled를 발행하지 않는다. /auto_control/start 서비스에
시작 요청만 보내고, 자동 제어 노드가 자체 점검을 통과한 경우에만 승인한다.
최종 duty 발행 함수에도 승인 여부 검사를 적용한다. GUI를 닫거나 오류가 나도
승인이 생기지 않는다. 브레이크/긴급 정지와 기존 차선/장애물/신호등 조건은 유지한다.

카메라 재측정, 모델 준비 신호 중단/실패, VESC 연결 끊김, ERPM timeout,
자동 제어 disable 또는 수동 전환은 승인을 취소한다. 복구되어도 자동으로
다시 출발하지 않고 새 START 요청이 필요하다.
일시적인 차선 입력 누락/신호등 정지 등 기존 주행 조건은 기존 로직을 따른다.
auto_enabled:=false는 여전히 disable이며 START 버튼이 이를 덮어쓰지 않는다.

## 수동과 성능 측정

manual_test:=true는 기존처럼 자동 제어기를 monitor_only로 실행하고, 수동 컨트롤러의
duty 경로를 변경하지 않는다. X 버튼으로 수동 전환할 때는 기존 STOPPING 제동·정지 확인을
거쳐 MANUAL로 진입한다. 수동 상태에서는 시작 버튼 단계를 통과 처리하며 카메라/모델
표시도 통과하면 MANUAL 상태를 표시하고 조작 창을 유지한다. 별도의 manual_drive.launch.py는 점검 창을 시작하지 않는다.
`remote_preview_enabled:=true`(SSH 실행)는 점검 창 대신 브라우저 프리뷰가 같은 점검 단계와 START/STOP을 제공한다. START 승인 조건은 같다. `REMOTE_PREVIEW_KO.md` 참고.

## 반영과 빌드

OAK 공용 측정 헤더가 변경되므로 전체 ROS 워크스페이스를 재빌드한다.
DepthAI SDK 재설치는 필요 없다. 엔진 캐시가 있는 install/을 삭제하지 않는다.

```bash
cd ~/Desktop/1030ML/f1tenth_project_repo &&
git pull --ff-only origin 1030ML &&
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH &&
source /opt/ros/humble/setup.bash &&
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release &&
source install/setup.bash
```

GUI에는 ROS Python 환경의 cv2/numpy가 필요하다. 패키지 의존성은 python3-opencv,
sensor_msgs, std_srvs로 선언했다. 디스플레이가 없으면 GUI가 시작 승인을 내리지 않는다.
수동 서비스 요청도 C++의 같은 점검을 거친다.

진단 인터페이스:

```bash
ros2 topic echo /bev_processor/startup_status --qos-durability transient_local --once
ros2 topic echo /auto_control/startup_state --qos-durability transient_local --once
ros2 topic echo /perception/models_ready --qos-durability transient_local --once
```

## 검증 범위

ROS 없이 실제 C++ 정지 조건과 최종 duty guard를 컴파일해 승인 전 차단,
너무 이른 시작 거부, 연결/모델/속도 정보 소실 시 재승인 요구와 수동 모드를 검증한다.
Python 테스트는 UI 활성화/승인 응답 대기, 상태 freshness와 창 렌더링을 확인한다.
실제 OAK 영상, ROS 서비스·QoS 연결과 차량 작동은 Jetson에서 전체 빌드 후 확인한다.

## 모델 로딩 단계 표시

점검 창의 INFERENCE MODELS는 RGB 차선, IR 차선, 터널 TCN, 신호등 중 이번 실행에
필요한 모델별로 다음을 표시한다. 캐시가 유효하면 엔진 생성 단계는 건너뛴다.

- 실제 단계: 엔진 캐시 확인/읽기 → ONNX 해석 → TensorRT 엔진 생성 → 캐시 저장 →
  실행 컨텍스트·GPU 버퍼 준비 → 시험 추론 → READY 또는 FAILED와 실패 원인.
- Total: 로더 시작 이후 시간. Stage: 현재 단계에 머무른 시간.
  Last report: 마지막 로딩 상태 수신 후 시간. 전체 READY 모델 수도 표시한다.
- LOADING 중 로더와 별도의 보고 스레드가 약 1초마다 상태를 전송한다.
  5초 넘게 보고가 없으면 REPORT STALE로 표시한다. 아직 보고가 없는 모델은 WAITING이다.
  구성요소가 순서대로 로드되어 뒤쪽 모델이 대기할 수 있다.
- 하트비트는 상태 보고 스레드의 생존만 확인하며 TensorRT 내부 진행률이나 GPU의 정상 진행을
  증명하지 않는다. 같은 단계에서 60초 이상 지나면 progress unknown 안내를 표시한다.
  엔진 생성은 정상이어도 수분 걸릴 수 있으므로 이 안내를 실패 판정으로 사용하지 않는다.

완료율 %나 남은 시간을 추정해서 표시하지 않는다. READY는 기존처럼 실제 시험 추론이
통과한 뒤에만 발행한다. 로딩 표시와 타이머는 START 승인 조건을 바꾸지 않는다.

## 도구 창 실행과 응답

Timings/Params는 점검 창과 함께 시작한 `auto_runtime_tools`의 Trigger 서비스로 연다.
도구 프로세스는 BEV 원본 프리뷰의 `preview_enabled` 값과 무관하게 점검 창이 있는
일반 실행에서 시작하며, 성능 측정 모드에서는 시작하지 않는다.
도구가 창을 생성하고 앞으로 올린 뒤 응답해야 점검 창에 opened를 표시한다.
서비스가 없으면 Tools unavailable, 열기 실패 시 오류 원인, 10초간 응답이 없으면
시간 초과를 표시한다. Python Tk가 설치되어 있어야 한다(`python3-tk`).
이 변경은 사용자 요청에 따라 빌드·테스트를 실행하지 않았다.

파라미터 도구는 `rcl_interfaces`의 ListParameters/GetParameters/SetParametersAtomically
서비스를 직접 비동기 호출한다. `rclpy.parameter_client`가 없는 Humble 설치에서도
모델·제어 파라미터 조회, 일괄 검증·적용과 프리셋 저장을 사용하도록 구성했다.
Tk import/화면 연결 등 초기화 실패는 도구 서비스의 실패 응답으로 점검 창에 전달한다.
이 호환성 수정도 빌드·테스트를 실행하지 않았다.

## 재승인 대기 시 마지막 정지 원인

출발 승인 취소 시 카메라 자세, 모델 준비, VESC 연결, ERPM 수명, 수동 전환,
STOP/disable 중 원인을 보존한다. 통신이 복구되어 READY_TO_START가 된 뒤에도
last stop 표시를 유지하며 START가 실제 수락되면 지운다. VESC 진단 토픽에서 받은
UART write timeout 등의 원인과 복구 상태를 함께 표시한다. 연결 장애나 입력 수명
검사와 실제 승인 취소·정지 정책은 동일하다. 50ms 쓰기 제한과 계측·포트 독점 설정은
`vehicle_bringup/config/vesc_config.yaml` 및 `vesc_bridge` 문서를 참고한다.


## 컴퓨팅 상태 표시

안전점검창 하단 `COMPUTING STATUS`에 장치 전체의 CPU/GPU 사용률, 온라인 CPU 코어 수,
RAM 사용량/총량, Swap 사용량/총량, 루트 파일시스템 여유 공간, CPU/GPU/SoC 온도,
CPU 정책 중 최고 현재 클럭·GPU 현재 클럭, 1/5/15분 시스템 load average를 표시한다.
출발 승인 후에도 창을 닫지 않았다면 계속 갱신한다. 기존 버튼/슬라이더 위치는 유지한다.

별도 데몬 스레드에서 1초마다 Linux `/proc`와 Jetson `/sys` 파일을 읽는다.
추가 Python 패키지·sudo·상시 외부 명령 실행 없이 읽기만 하며 전력/클럭/팬 설정을 바꾸지 않는다.
CPU 사용률은 이전 표본 이후의 장치 전체 busy 비율(0~100%)이고 첫 표본은 `--`다.
RAM은 MemTotal−MemAvailable, GPU는 nvgpu load(0~1000)를 %로 변환한다.
Jetson은 공유 메모리 구조이므로 GPU 전용 VRAM 수치로 RAM 값을 중복 표기하지 않는다.
온도는 해당 종류 센서의 최고값이며 Tmargin(온도 여유)은 실제 온도에서 제외한다.
센서가 없거나 권한/커널 차이로 읽지 못하면 `--`를 표시한다. 3초 넘게 갱신이 없으면
`STALE`을 표시하고 이전 수치는 숨긴다. 이 값은 출발 허가 조건이나 과열/스로틀링 확정 판정이 아니다.

창의 논리 크기는 1200×1020, 실제 표시는 기존과 같은 85%(1020×867)다.
로컬 가상 proc/sys 파일로 CPU 차분·단위 변환·센서 유실·STALE을 테스트하고 실제 렌더링을 확인했다.
차량 JetPack에서의 센서 경로와 수치 갱신은 실기기 확인이 필요하다.

구현 참고: [Linux proc 문서](https://www.kernel.org/doc/html/latest/filesystems/proc.html),
[NVIDIA Jetson 전력·성능 문서](https://docs.nvidia.com/jetson/archives/r39.2/DeveloperGuide/SD/PlatformPowerAndPerformance/JetsonOrinNanoSeriesJetsonOrinNxSeriesAndJetsonAgxOrinSeries.html).
