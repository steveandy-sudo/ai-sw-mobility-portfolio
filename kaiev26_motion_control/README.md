# kaiev26_motion_control

차량 기준 목표 경로와 목표속도를 `ActuatorCommand`로 변환하는 ROS 2 패키지다.

## 입출력

```text
/planning/target_path  ─┐
/planning/target_speed ─┼-> motion_control_node -> /planning/command
/vehicle/state         ─┘
```

`/planning/command`는 Control의 `command_governor`를 거쳐 `/vehicle/command`가 된다.
IMU는 직접 구독하지 않는다. 경로 위치·헤딩은 Route Manager가 변환한 로컬 경로에 반영된다.

## 처리 순서

1. PP+Stanley로 조향을 계산한다. 전역경로 직선에서는 속도별 Stanley 이득, 경로 지연 보상,
   직진 편향 적분을 쓴다(`docs/PARAMETER_BOOK.md`의 직선 추종 절).
2. 직진 오프셋과 엔코더 오차 기반 마찰 보상을 더한다(조향 보정 절).
3. 최대 조향 `25 deg`를 적용한다. 변화율 `60 deg/s`는 lateral_ctrl이 건다.
4. Planner의 목표속도에 가감속 제한과 계획 정지를 적용한다.
5. 정지 후 브레이크를 먼저 해제하고 `0.8초` 뒤 가속한다.
6. 목표 입력이 0.35초 넘게 오래되면 곧바로 속도 0을 브레이크 없이 요청하고, 1초 안에 돌아오지 않으면
   브레이크를 요청한다. 이 브레이크는 입력이 돌아와도 실제 정지(`0.05 m/s` 이하)까지 유지하고, 정지 뒤 입력이
   돌아오면 풀고 다시 달린다. 입력이 끊길 때 이미 걸려 있던 브레이크는 끊긴 동안 풀지 않고, 1초 안에 입력이
   돌아오면 돌아온 목표를 따른다. 마지막
   목표를 그대로 이어 쓰지 않는다(2026-10-16 결정 D2). 규칙은 `input_loss.InputLossLatch` 하나이고 판단과 시험 TUI도
   같은 클래스를 쓴다. 끊긴 동안 조향 목표는 측정 조향각이다.

경로 추종 중 실제 조향속도와 목표·실제 조향 오차는 진단만 한다. 목표각 예측, 선행 제한, 추종 오차
기반 감속에는 사용하지 않는다.

## 횡제어기

PP+Stanley 하나다. PP는 속도·전방 곡률 기반 적응형 Ld, Stanley는 앞차축 CTE와 경로 헤딩을
보정한다. PP 비율은 전역경로 단계별로 직선 `0`, 코너 접근 `0.65`, 회전 `0.35`, 복귀 `0.10`이며
순간 CTE로 제어기를 전환하지 않는다.

## 핵심값

설정 파일: [`config/motion_control.yaml`](config/motion_control.yaml)

| 항목 | 값 |
|---|---:|
| 제어 주기 | `50 Hz` |
| 축거 | `1.2991 m` |
| 최대 조향 / 변화율 | `25 deg` / 제한 없음(60 deg/s는 lateral_ctrl) |
| 직선 Ld | `clamp(4.0 + 0.9 * speed, 3.6, 9.5) m` |
| 강한 코너 Ld 하한 | `clamp(3.6 + 0.30 * speed, 3.6, 직선 Ld) m` |
| 곡률 미리보기 | `12.0 m` |
| 일반 가속 / 감속 | `1.0 / 2.0 m/s^2` |
| 정지선·신호 사전 감속 상한 | `10.0 m/s^2` |
| 비정지 장애물 감속 상한 | `10.0 m/s^2` |
| 브레이크 해제 대기 | `0.8 s` |

코스별 곡선 감속은 Decision이 전역경로 진행거리로 계산한다. Motion Control은 local path
곡률을 이용해 목표속도를 다시 제한하지 않는다.

브레이크를 요청하는 동안(`brake_engage=true`)도 유효 경로와 실제속도 약 `0.278 m/s` 이상이면
경로 조향을 계속 계산한다. 현재 Control도 `brake_steering_min_speed_kph=1.0` 이상에서
제동 중 조향을 허용한다. 정지 목표거리 0은 "지금 제동"이므로 제동 중 추종각을
마지막 2 m 조향 축소로 중립까지 줄이지 않는다. 축소는 목표속도가 남아 있는 계획 정지 접근에만 쓴다. 실제속도 약 0.278 m/s 미만에서는
피드백 각도를 유지하며, 0.1 m/s 이하 정지 확인 후에는 0.3 m/s 이상에서 경로 조향을 재개한다.
재출발은 `speed=0, brake=false`를 0.8초 유지한 뒤 가속하여 브레이크 서보와 구동 명령이
겹치지 않게 한다. 신호 재출발·통과와 3초 정지선 해제는 모두 `2.0 m/s^2`로 `5.0 m/s`까지
가속한 뒤 일반 `1.0 m/s^2` 가속으로 전환한다.
판단의 정지점 기계는 정지 뒤 출발을 `SIGNAL_RELEASE` 한 틱 다음 `COMMITTED` 확정(`SIGNAL_PASS_COMMITTED`)으로 내고, 둘 다 같은 부스트를 받는다.

정지선·신호의 제동 WP 사전 감속과 비정지 장애물 목표만 명령 감속 상한 `10.0 m/s^2`를
사용한다. 일반 추종·코너 감속 상한은 `2.0 m/s^2`이며, 명령 상한은 실제 감속도를 보장하지 않는다.

전체 값과 수식은 [Parameter Book](../docs/PARAMETER_BOOK.md)과
[Route Tracking](../docs/ROUTE_TRACKING.md)을 참고한다.

## 빌드·실행

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select kaiev26_motion_control
source install/setup.bash
```

일반적으로 `decision.launch.py` 또는 `decision_test.launch.py`가 자동 실행한다. 단독 실행은
상위 노드가 목표 경로·속도를 발행할 때만 사용한다.

```bash
ros2 run kaiev26_motion_control motion_control_node \
  --ros-args \
  --params-file ~/KAI_ws/src/Decision/kaiev26_motion_control/config/motion_control.yaml
```

## 확인

```bash
ros2 topic hz /planning/command
ros2 topic info -v /planning/command
ros2 topic echo --once /planning/target_speed
```

정상 발행 주기는 약 50 Hz이고 발행자는 하나여야 한다.

## 테스트

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select kaiev26_motion_control
colcon test-result --test-result-base build/kaiev26_motion_control
```
