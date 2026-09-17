# kaiev26_decision_fox

Decision·차량·센서 상태를 Foxglove에서 읽기 전용으로 표시한다. 차량 명령과 모드 전환은
수행하지 않는다.

## Mode

| mode | 동작 |
|---|---|
| `manual` | 실시간 궤적 노드, bridge, Foxglove Desktop 실행 |
| `auto` | 위 구성으로 Decision 상태까지 표시 |
| `record` | Desktop만 실행해 로컬 MCAP 열기 |

## 빌드

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select kaiev26_decision_fox
source install/setup.bash
```

## 실시간 화면

Control, 센서와 Localization을 먼저 실행한다. 자율주행은 Decision도 먼저 실행한다.

```bash
# 수동주행
ros2 launch kaiev26_decision_fox fox.launch.py mode:=manual

# 자율주행
ros2 launch kaiev26_decision_fox fox.launch.py mode:=auto
```

mode는 화면 구분용이며 차량의 MANUAL/AUTO 상태를 바꾸지 않는다. 이 패키지는 MCAP을
녹화하지 않는다.

여러 노트북이 동시에 볼 때는 이 launch 대신 차량 PC에서 Control의 공용 bridge 하나를
실행한다.

```bash
ros2 launch foxglove foxglove_bridge.launch.py
```

노트북은 `ws://<차량-PC-IP>:8765`에 연결한다. 자세한 주소는
[Runbook](../docs/DECISION_RUNBOOK.md#foxglove)을 참고한다.

## MCAP

```bash
ros2 launch kaiev26_decision_fox fox.launch.py mode:=record
```

Foxglove Desktop에서 `Open local file`을 선택하고 `.mcap` 파일을 연다. `ros2 bag play`와
bridge는 필요하지 않다.

## 사용자 패널

최초 설치 또는 `foxglove/src/` 변경 후 실행한다.

```bash
cd ~/KAI_ws/src/Decision/kaiev26_decision_fox/foxglove
npm ci
npm run local-install
```

## 표시 항목

- 전체 Route, 일반·제동 waypoint, 차량 궤적
- 출발 준비와 주행 상태
- 현재 Stage, FSM, 신호·정지 타이머
- 목표·실제 속도와 조향
- 좌우 카메라

## 문제 확인

```bash
ros2 topic hz /localization/odometry
ros2 topic hz /planning/route_context
ros2 topic hz /planning/readiness_status
ros2 topic hz /planning/command
ros2 topic hz /vehicle/command
ros2 node list | rg foxglove
```

MCAP 화면이 비면 해당 토픽이 기록됐는지 확인한다.

```bash
ros2 bag info /path/to/rosbag_directory
```

Foxglove는 E-Stop 대체 수단이 아니다. 같은 `8765` 포트의 bridge를 둘 이상 실행하지 않는다.

## 테스트

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select kaiev26_decision_fox
colcon test-result --verbose
```
