# KAIEV26 Decision

KAIEV26의 전역경로 판단, 미션 FSM, 경로 추종, 시험 TUI, Foxglove 화면과 

라바콘 제동 검차를 제공하는 ROS 2 Humble 저장소다.

## 문서

| 문서 | 내용 |
|---|---|
| [운용 Runbook](docs/DECISION_RUNBOOK.md) | 실차·Gazebo 실행, 기록, 문제 확인 |
| [Architecture](docs/ARCHITECTURE.md) | 패키지 경계와 데이터 흐름 |
| [Mission Scenarios](docs/MISSION_SCENARIOS.md) | 예선·본선 Zone, 신호, 정지 지점 |
| [Route Tracking](docs/ROUTE_TRACKING.md) | PP·Stanley·속도·정지 원리 |
| [Parameter Book](docs/PARAMETER_BOOK.md) | 주요 설정값과 변경 위치 |

## 패키지

| 패키지 | 역할 |
|---|---|
| [`kaiev26_decision`](kaiev26_decision/README.md) | Route·Zone, 정책, FSM, 시험 TUI |
| [`kaiev26_motion_control`](kaiev26_motion_control/README.md) | 목표 경로·속도를 차량 명령으로 변환 |
| [`kaiev26_decision_fox`](kaiev26_decision_fox/README.md) | Decision 전용 Foxglove 시각화 |
| [`kaiev26_aeb`](kaiev26_aeb/README.md) | 라바콘 경로 추종·빨간선 제동 검차 |

## 빌드

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  kaiev26_decision kaiev26_motion_control kaiev26_decision_fox kaiev26_aeb
source install/setup.bash
```

## 빠른 실행

실차 시험은 **Control과 GNSS-only Localization을 먼저 실행**한다. 

카메라와 인지 조건을 포함한 전체 순서는 [Runbook](docs/DECISION_RUNBOOK.md#실차-decision-시험)을 따른다.

```bash
ros2 run kaiev26_decision decision_test
```

Gazebo 예선 정지선 시험:

```bash
ros2 launch kaiev26_gazebo_bringup decision_test.launch.py \
  case:=2 speed_mps:=3.2 lateral_controller:=pure_pursuit
```

라바콘 제동 검차:

```bash
# 실차
ros2 run kaiev26_aeb cone_trial_tui

# Gazebo
ros2 launch kaiev26_aeb aeb_sim.launch.py scenario:=2
```

## 명령 흐름

```text
Perception + Localization + VehicleState
  -> Route·Zone + Mission FSM
  -> target_path + target_speed
  -> Motion Control
  -> /planning/command
  -> Control command_governor
  -> /vehicle/command
```

시험 TUI에서는 Motion Control 출력이 `/decision/raw_command`로 들어오고,

출발·AUTO 조건을 통과한 뒤 TUI가 `/planning/command`로 중계한다.

## 기본 원칙

- `/planning/command` 발행자는 하나만 둔다. Decision, AEB, SysID를 동시에 실행하지 않는다.
- 실제 조향 피드백은 진단용이다. Decision은 추종 오차만으로 감속하거나 정지하지 않는다.
- 정지선 위치와 제동거리는 저속 실차 시험으로 확인한 뒤 속도를 높인다.
- Foxglove는 모니터링 도구이며 MANUAL 전환이나 E-Stop을 대신하지 않는다.

## 테스트

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select \
  kaiev26_decision kaiev26_motion_control kaiev26_decision_fox kaiev26_aeb
colcon test-result --verbose
```
