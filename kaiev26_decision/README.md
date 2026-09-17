# kaiev26_decision

전역경로 정합, Zone·Event 선택과 FSM을 실행해 목표 경로·속도를 만드는 ROS 2 패키지다.
최종 차량 명령은 [`kaiev26_motion_control`](../kaiev26_motion_control/README.md)이 만든다.

## 구성

```text
config/                    판단·코스·RViz 설정
launch/
  decision.launch.py      경기 판단
  decision_test.launch.py 시험 Case 1~8
  decision_rviz.launch.py RViz
kaiev26_decision/
  perception_gateway_node.py    인지 동기화
  route_zone_manager_node.py    Route 진행도·Stage·로컬 경로
  main_planning_engine_node.py  정책·FSM·우선순위
  test_tracking.py               Case 1·4 경로 추종 계획
  test_readiness.py              실차 출발 검사
  test_steering_step.py          Case 8 조향 스텝
  test_tui.py                    시험 TUI·명령 게이트·MCAP
  scenario_modules/              개별 FSM
waypoints/                 예선·본선 전역경로
```

## 데이터 흐름

```text
Perception -> /planning/scene_summary ─┐
Odometry  -> /planning/route_context  ─┼-> Policy/FSM
VehicleState                          ─┘
  -> /planning/target_path
  -> /planning/target_speed
  -> Motion Control
```

판단 우선순위는 `E-Stop/AEB -> 입력 이상 -> Finish -> Zone Event -> 일반 추종`이다.

## 설정

| 파일 | 역할 |
|---|---|
| `config/decision_pipeline.yaml` | 입력, 속도, 정지, 회피·복귀, AEB |
| `config/*_landmarks.yaml` | 정지선·신호등 지도 위치 |
| `config/*_policy.yaml` | Stage·Event·제동 waypoint |
| `waypoints/kcity_*_route.yaml` | 전역경로 |

## 빌드

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  kaiev26_decision kaiev26_motion_control
source install/setup.bash
```

## 실행

실차 시험 TUI:

```bash
ros2 run kaiev26_decision decision_test
```

경기 판단 직접 실행:

```bash
ros2 launch kaiev26_decision decision.launch.py \
  course:=qualifying use_sim_time:=false
```

본선은 `course:=final`을 사용한다. Control, Localization과 실제 인지는 이 launch에 포함되지
않는다. 현장 실행 순서는 [Decision Runbook](../docs/DECISION_RUNBOOK.md)을 따른다.

Gazebo 통합 시험:

```bash
ros2 launch kaiev26_gazebo_bringup decision_test.launch.py \
  case:=3 speed_mps:=3.2 lateral_controller:=pure_pursuit
```

RViz:

```bash
ros2 launch kaiev26_decision decision_rviz.launch.py mode:=spatial
ros2 launch kaiev26_decision decision_rviz.launch.py mode:=lidar
```

실차 RViz는 `use_sim_time:=false`를 추가한다.

## 시험 Case

| Case | 기능 |
|---:|---|
| `1·4` | 예선·본선 경로 추종 |
| `2·5` | 경로 추종 + 모든 정지점 3초 정지 |
| `3·7` | 예선·본선 실제 미션 전체 |
| `6` | 본선 회피 |
| `8` | 상수 속도 + R/L 조향 스텝 |

TUI 키는 `1~8` Case, `V` 속도, `L` 횡제어기, `R` Run, `S` Stop, `Q` 종료다.
Case 8은 `A` 진폭, `F` 주파수, `P` 방향 패턴을 추가로 설정한다.

## 주요 토픽

| 입력 | 출력 |
|---|---|
| `/localization/odometry` | `/planning/route_context` |
| `/perception/{road_segments,centerline,objects,traffic_lights}` | `/planning/scene_summary` |
| `/vehicle/state` | `/planning/target_path`, `/planning/target_speed` |
| `/planning/route_ready` | `/debug/*` |

## 규칙

- `/localization/odometry` 발행자는 하나만 둔다.
- `/planning/command`를 만드는 Decision·AEB·SysID를 동시에 실행하지 않는다.
- Mock 인지와 실제 인지를 동시에 실행하지 않는다.
- 코스 정의는 [Mission Scenarios](../docs/MISSION_SCENARIOS.md)를 기준으로 확인한다.

## 테스트

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select kaiev26_decision
colcon test-result --verbose
```
