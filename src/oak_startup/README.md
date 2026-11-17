# oak_startup

`bev_processor`에서 분리한 초기 OAK 자세·높이 측정 공용 라이브러리.
`oak_startup::measurement`를 링크하고 `oak_startup/oak_startup_measurement.hpp`를 포함한다.

정지 IMU와 CAM_A 정렬 stereo depth의 바닥 평면으로 roll/pitch/높이를 구한다.
자동 장치 calibration은 꺼 두며 EEPROM을 쓰지 않는다. 성공/실패 후 파이프라인과
장치를 닫는다. 반환값에는 재연결할 장치 ID가 포함된다. 선택적으로 중단 callback을
전달할 수 있다. 특정 장치는 config.device_id로 지정한다.

BEV의 기본 측정 기준·알고리즘은 유지하며 기존 bev_processor 헤더는 타입/함수 별칭을
제공한다. 카메라 공유·runtime IMU 보정·TF 발행은 이 라이브러리의 역할이 아니다.
