# HL_KU Autonomous Vehicle

This repository is provided for portfolio and research demonstration purposes.
No separate open-source reuse license is granted; third-party dependencies keep
their own terms.

자작자동차 대회를 위해 개발한 ROS 2 기반 실차 자율주행 소프트웨어의 공개용
staging snapshot이다. Henes T8 Sports 플랫폼에 NUCLEO-H743ZI2 차량 인터페이스를
연결하고, UM982 듀얼 안테나 RTK GNSS에서 얻은 위치·방향·속도를 local ENU 좌표로
변환했다. 기록한 전역 경로는 자연 cubic spline으로 다듬은 뒤 연속적인
`RouteFollower`와 Pure Pursuit가 추종하도록 구성했다.

## Overview

실차 구성은 **GNSS 수신 → local ENU pose → 경로 기록·준비 → spline smoothing →
연속 경로 추종 → 차량 제어**의 흐름으로 연결된다. ROS 2 노드, 팀 전용 메시지,
NUCLEO 펌웨어와 회귀 테스트를 한 저장소에서 확인할 수 있다.

이 공개본은 소스 구조와 재현 가능한 예시만 담은 검토용 snapshot이다. 실제 차량을
움직이려면 배포 환경에서 별도의 센서 장치 경로, 기준점, 경로, 조향·구동 보정,
네트워크 설정과 안전 점검을 작성해야 한다. 기본 설정은 하드웨어 출력을 비활성화한다.

## Vehicle Platform

- 차량 플랫폼: Henes T8 Sports
- 컴퓨팅/미들웨어: Linux, ROS 2
- 차량 인터페이스: NUCLEO-H743ZI2와 기존 모터·조향 구동부
- 위치·방향·속도: UM982 듀얼 안테나 RTK GNSS
- 인터페이스: ROS 2 명령을 NUCLEO UART 프로토콜로 전달

## Localization

`um982_serial_node.py`가 UM982의 NMEA 문장을 검증하고 위치·방향·속도를
`gnss_localizer_node.py`에 전달한다. 수신된 위도·경도·고도를 local ENU 좌표로
변환하고, 안테나 위치와 차량 기준점을 반영해 차량 pose를 구성한다. 기준점과
안테나 보정값은 공개본에 포함하지 않고 로컬 배포 설정에서 입력하도록 했다.

## Route Recording

실제 차량에서 GNSS 경로를 기록할 수 있도록 `route_recorder_node.py`와
`rtk_waypoint_recorder_node.py`를 제공한다. 기록 과정에서는 위치 품질과 샘플 간
간격을 확인하고, 경로를 차량 기준 좌표와 mission metadata가 포함된 CSV로 저장한다.
실제 대회장 좌표와 원본 로그는 공개하지 않았다. 형식만 확인할 수 있는
`ros2_ws/src/hl_ku_core/routes/example_route.csv`를 참고한다.

## Route Smoothing

`route_smoothing.py`는 기록된 경로를 자연 cubic spline으로 보간하고, 원래 경로와의
편차·곡률·샘플 간격을 검증한다. mission과 속도 변화가 있는 지점은 anchor로 남겨
경로의 의미를 유지한다. 제한을 만족하지 못하는 경로는 자동으로 완화하지 않고
거부하도록 구성했다.

## Continuous Route Tracking

`RouteFollower`는 차량 pose를 경로의 가까운 선분에 투영하고 진행도(progress)를
연속적으로 갱신한다. 이전 진행도 주변의 거리·방향 창을 사용해 교차 구간에서
뒤쪽 경로로 뛰지 않도록 했고, 진행도를 되돌리지 않는다. 속도에 따른 lookahead와
곡률을 바탕으로 연속적인 virtual target을 계산해 waypoint를 하나씩 건너뛰는 대신
경로를 따라 추종하도록 했다.

## Pure Pursuit Control

`control.py`와 `route_tracking.py`의 Pure Pursuit는 차량 좌표계의 lookahead target을
이용해 조향 명령을 계산한다. 조향 변화율, 최대 조향각, 횡가속도와 경로 이탈
조건을 함께 검사하고, pose·속도·방향 입력이 유효하지 않으면 정지 명령을 낸다.

## Vehicle Interface

`vehicle_controller_node.py`와 `nucleo_serial_bridge_node.py`가 상위 주행 명령을
안전 감독 이후의 액추에이터 명령으로 연결한다. NUCLEO 쪽에는 명령·조향 위치·ACK
프로토콜과 watchdog, fault latch를 포함한 하드웨어 독립 제어 모듈을 두었다.
장치 경로, ADC 조향 보정과 네트워크 주소는 공개본에서 제거했으며, 로컬 장비에
맞는 설정을 별도로 주입해야 한다.

## ROS 2 Packages

- `hl_ku_core`: GNSS 파싱·ENU 변환, 경로 기록·준비·spline smoothing, RouteFollower,
  Pure Pursuit, perception/mission 연결, 안전 감독, 차량 인터페이스
- `hl_ku_interfaces`: GNSS 상태, perception 상태, mission 상태, 주행·액추에이터
  명령과 차량 feedback 메시지
- `firmware/nucleo_h743zi2`: NUCLEO-H743ZI2에 이식하는 프로토콜·제어 모듈

## Example Launch

ROS 2 Humble과 의존 패키지를 설치한 뒤 다음처럼 공개 템플릿을 빌드할 수 있다.

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths ros2_ws/src --packages-select hl_ku_interfaces hl_ku_core
source install/setup.bash
ros2 launch hl_ku_core public_route_preview.launch.py
```

`public_route_preview.launch.py`는 synthetic route와 하드웨어 비활성 설정을
사용한다. 이 예시는 노드 연결을 확인하기 위한 것이며 실제 차량 주행을 승인하지
않는다. 실제 GNSS·차량 통합은 현장 안전 절차와 팀의 로컬 설정을 거쳐야 한다.

## Ongoing Work

현재 YOLO 기반 장애물 perception과 mission 시스템 통합은 진행 중인 작업이다.
공개본에는 이를 완료된 장애물 회피 기능이나 완성된 모델로 표현할 코드·가중치를
포함하지 않았다.

## Validation and Limitations

소스 수준에서는 NMEA 파서, ENU 변환, 경로 준비·smoothing, 연속 추종, mission,
프로토콜과 안전 점검에 대한 회귀 테스트를 제공한다. 원본 개발에서 확인된 완료
범위는 듀얼 RTK GNSS 기반 위치·방향 확보, 경로 기록, cubic spline smoothing,
RouteFollower + Pure Pursuit, NUCLEO 인터페이스와 실제 차량 경로 추종 검증이다.

실제 경로 좌표, 보정 기록, 센서 로그와 대용량 실험 자료는 공개 범위에서 제외했다.
따라서 이 저장소만으로 특정 대회 트랙이나 차량을 재현할 수 없으며, 공개 템플릿은
안전 점검을 통과하지 않도록 설계되어 있다.

## My Contribution

커밋 `2f88330`의 route smoothing·continuous tracking 변경은 `yunny22`가 작성한
개인 작업 증거로 확인된다. 차량 플랫폼·GNSS·펌웨어·perception·mission 통합은
팀 저장소에서 공동 개발된 범위로 분류했다. 이 구분은 공개본의 파일을 개인 단독
성과로 과장하지 않기 위한 것이다.

## Credits and Source

- 원본 팀 저장소: [yunny22/HL_KU](https://github.com/yunny22/HL_KU)
- 공개본 기준 source branch: `codex/smooth-route-tracking`
- 공개본 기준 source commit: `2f88330`

원본 팀 코드와 외부 의존성의 공개·라이선스 범위는 팀과 함께 최종 검토해야 한다.
공개본은 원본 Git history나 remote를 포함하지 않는 별도 staging snapshot이다.
