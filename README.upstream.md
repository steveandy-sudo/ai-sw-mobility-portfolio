# KAIEV26 Decision

KAIEV26의 전역경로 추종, 예선·본선 미션 판단, 시험 TUI, 제동 검차와 시각화를 제공하는
ROS 2 Humble 저장소다.

## 문서

| 문서 | 내용 |
|---|---|
| [운용 Runbook](docs/DECISION_RUNBOOK.md) | 실차·경로 수집·회피 시험·Gazebo 실행과 문제 확인 |
| [Architecture](docs/ARCHITECTURE.md) | 패키지 경계와 데이터 흐름 |
| [Mission Scenarios](docs/MISSION_SCENARIOS.md) | 예선·본선 Stage, 정지·재출발과 신호 통과 조건 |
| [Route Tracking](docs/ROUTE_TRACKING.md) | PP+Stanley, 속도와 정지 |
| [Parameter Book](docs/PARAMETER_BOOK.md) | 현장 조정값과 기준 파일 |

## 패키지

| 패키지 | 역할 |
|---|---|
| [`kaiev26_decision`](kaiev26_decision/README.md) | Route·Zone, 정책, 정지점 기계, 장애물 행동, 시험 TUI |
| [`kaiev26_motion_control`](kaiev26_motion_control/README.md) | 목표 경로·속도를 차량 명령으로 변환 |
| [`kaiev26_aeb`](kaiev26_aeb/README.md) | 라바콘 중앙경로 추종과 빨간선 제동 검차 |
| [`kaiev26_decision_fox`](kaiev26_decision_fox/README.md) | 읽기 전용 Foxglove 화면 |

## 빌드

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  kaiev26_msgs kaiev26_maps pv26_perception cone_perception \
  kaiev26_decision kaiev26_motion_control kaiev26_decision_fox kaiev26_aeb
source install/setup.bash
```

## 실행

실차 경기·운용(녹화 없음):

```bash
ros2 launch kaiev26_decision decision.launch.py
```

기존 TUI와 같은 상태·출발 검사 화면에서 `P`로 예선·본선, `V`로 속도, `B`로 출발 모드를
선택한다.
기본속도는 예선 `7 m/s`, 본선 `10 m/s`이며 `R` 시작, `S` 중단, `Q` 종료다.
MCAP과 `records` 폴더는 생성하지 않는다. 아래 시험 TUI는 주행을 기록한다.

시나리오 시험:

```bash
ros2 run kaiev26_decision decision_test
```

실행 전 Control, GNSS Localization과 필요한 센서·인지 모듈을 준비한다. 경기 인지는
`pv26_perception`, 제동 검차 인지는 `cone_perception`을 사용한다. 전체 순서는
[Runbook](docs/DECISION_RUNBOOK.md)을 따른다.

Gazebo에서는 통합 실행기로 Gazebo를 띄운 뒤 시험 TUI를 `--sim-time`으로 실행한다.

```bash
ros2 run kaiev26_gazebo_bringup gazebo_launcher
ros2 run kaiev26_decision decision_test --sim-time
```

시뮬레이션 TUI에서는 예선·본선 미션 실행(내부 launch Case `3`, `7`)을 선택한다.

라바콘 제동 검차:

```bash
ros2 run kaiev26_aeb cone_trial_tui
```

## 데이터 흐름

```text
Perception + Localization + VehicleState
  -> Route·Zone + 판단(정지점 기계, 장애물 행동, 중재기)
  -> target_path + target_speed
  -> Motion Control
  -> /planning/command
  -> Control command_governor
  -> /vehicle/command
```

TUI에서는 Motion Control이 `/decision/raw_command`를 내고 TUI가 출발 조건을 확인한 뒤
`/planning/command`로 중계한다. 운용 화면은 코스별 `ALL`과 PP+Stanley를 사용한다.

## 운용 원칙

- `/planning/command` 발행자는 하나만 둔다. Decision, AEB, SysID를 동시에 실행하지 않는다.
- 실차에서는 `pv26_perception`을 `use_sim_time:=false`로 실행한다.
- 조향 추종 오차만으로 감속하거나 시험을 중단하지 않는다.
- 정지·AEB 수치는 저속 실차 시험으로 확인한 뒤 속도를 높인다.
- Foxglove는 읽기 전용이며 MANUAL 전환이나 물리 E-Stop을 대신하지 않는다.

## 테스트

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select \
  kaiev26_decision kaiev26_motion_control kaiev26_decision_fox kaiev26_aeb
for pkg in kaiev26_decision kaiev26_motion_control kaiev26_decision_fox kaiev26_aeb; do
  colcon test-result --test-result-base "build/$pkg"
done
```
