# 기존 DepthAI SDK 경로 고정

차량 PC에서 확인한 기존 C++ SDK를 그대로 사용한다.

```text
CMake config: /usr/local/lib/cmake/depthai/depthaiConfig.cmake
공유 라이브러리: /usr/local/lib/libdepthai-core.so
```

`/opt/ros/humble`에도 별도 DepthAI가 있으므로 헤더·링크 라이브러리와 실행 시
로드하는 라이브러리가 서로 달라지지 않도록 두 경로를 함께 고정한다.
`undefined symbol` 오류는 이러한 불일치에서 발생할 수 있다. 실제 원인 확정에는
실행 바이너리의 `ldd` 결과를 확인한다.

## 날짜별 폴더 구성

```text
1030ML/
├── depthai-core/               # 기존에 복사해 둔 소스 폴더: 수정하지 않음
└── f1tenth_project_repo/
    ├── cmake/DepthaiWorkspace.cmake
    ├── src/
    ├── build/
    ├── install/
    └── log/
```

`depthai-core` 소스 폴더의 위치와 이미 설치된 C++ SDK의 위치는 별개다.
확인한 차량 PC에서는 SDK 공유 라이브러리가 `/usr/local/lib`에 있다.
날짜별로 `f1tenth_project_repo`와 소스 폴더를 복사해도 설치된 SDK는 같은 위치에
유지되므로 브랜치 날짜를 SDK 경로에 넣지 않는다.

ROS 패키지의 CMake는 기본 SDK prefix를 `/usr/local`로 고정하고, 이전
`depthai_DIR` 캐시를 해당 prefix의 config로 교정한다. ROS 패키지만 빌드하며
SDK 소스 재설정·재빌드·재설치, vcpkg 실행, 빌드 도구 설치를 수행하지 않는다.
이전에 추가했던 `scripts/install_depthai.sh`는 제거했다.
`../depthai-core/install` 폴더도 요구하지 않는다.

## 전체 ROS 패키지 빌드

실행 중인 ROS 노드를 종료한 뒤 새 터미널에서 실행한다.
이미 ROS의 `build/`, `install/`, `log/`를 삭제했다면 추가 삭제 없이 아래부터
진행하면 된다. `--cmake-clean-cache`로 남아 있을 수 있는 예전 SDK 선택도 비운다.

```bash
cd ~/Desktop/1030ML/f1tenth_project_repo &&
git switch 1030ML &&
git pull --ff-only origin 1030ML &&
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH &&
source /opt/ros/humble/setup.bash &&
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release &&
source install/setup.bash
```

기존 ROS 산출물을 전부 폐기하려는 경우에만, 위 빌드 전에 워크스페이스 루트에서
`rm -rf -- build install log`를 실행한다. 이 명령은 ROS 빌드·설치 산출물과 빌드 로그를
삭제하며 `src/`, 개인 YAML, `../depthai-core`, `/usr/local`은 변경하지 않는다.

패키지는 순차 빌드하고 각 CMake 빌드의 병렬 수는 2로 제한한다.
ROS 패키지 빌드에는 차량 PC에 원래 설치된 CMake를 사용하면 된다.
SDK 재빌드용 CMake 3.30.1 가상환경을 활성화할 필요가 없다.

## 실행

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/1030ML/f1tenth_project_repo/install/setup.bash
ros2 launch point_cloud point_cloud.launch.py
```

`source install/setup.bash`는 환경 훅으로 선택된 SDK 라이브러리 경로를
`LD_LIBRARY_PATH` 맨 앞에 배치한다. 반복 source 시 중복도 제거한다.
각 패키지의 RPATH와 `depthai_runtime_dir.txt` 역시 동일한 SDK를 기록한다.
`ros2 run`, standalone 노드, 컴포넌트 컨테이너에 같은 환경을 적용한다.
기존 launch의 `depthai_library_dir`는 기본값을 사용한다.

SDK가 기본 경로에 없거나 요구 버전(3.6+)과 다르면 자동 설치나 ROS SDK로의
자동 대체 없이 선택 경로와 오류를 표시한다. 다른 PC에서 **이미 설치된** SDK가
다른 위치에 있는 경우에만 모든 관련 패키지에 공통 prefix를 전달한다.

```bash
colcon build --cmake-args -DF1TENTH_DEPTHAI_PREFIX=/existing/sdk/prefix
```

## 실제 선택 확인

```bash
cat install/point_cloud/share/point_cloud/depthai_runtime_dir.txt
ldd install/point_cloud/lib/point_cloud/point_cloud_node | grep depthai
ldd install/oak_startup/lib/liboak_startup_measurement.so | grep depthai
```

확인한 차량 PC에서는 `/usr/local/lib`의 라이브러리가 선택되어야 한다.
CMake 로그의 `Workspace DepthAI config`와 `Workspace DepthAI library`에도
선택한 경로가 출력된다.

ROS 없이 SDK 선택과 실제 공유 라이브러리 충돌을 검증하려면:

```bash
python3 -m unittest discover -s tests -p 'test_depthai_workspace.py' -v
python3 src/bev_processor/test/test_depthai_runtime_launch.py
```
