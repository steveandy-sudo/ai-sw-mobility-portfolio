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

1. PP, Stanley, PP+Stanley 또는 FF+Stanley로 조향을 계산한다.
2. 전역경로 직선의 작은 오차에서는 조향을 완화한다.
3. 최대 조향 `25 deg`와 변화율 `60 deg/s`를 적용한다.
4. 순항·곡률·미션 속도 중 가장 낮은 값을 선택한다.
5. 가감속 제한과 계획 정지를 적용한다.
6. 목표 입력이 0.35초 이상 오래되면 속도 0과 브레이크를 요청한다.

실제 조향속도와 목표·실제 조향 오차는 진단만 한다. 목표각 예측, 선행 제한, 추종 오차
기반 감속에는 사용하지 않는다.

## 횡제어기

| 값 | 동작 |
|---|---|
| `pure_pursuit` | 속도·전방 곡률 기반 적응형 Ld |
| `stanley` | 앞차축 CTE와 경로 헤딩 보정 |
| `pp_stanley` | 전역경로 단계별 고정 비율 혼합 |
| `ff_stanley` | 경로 곡률 FF + Stanley 보정 |

PP+Stanley의 PP 비율은 직선 `0`, 코너 접근 `0.65`, 회전 `0.35`, 복귀 `0.10`이다.
순간 CTE로 제어기를 전환하지 않는다.

## 핵심값

설정 파일: [`config/motion_control.yaml`](config/motion_control.yaml)

| 항목 | 값 |
|---|---:|
| 제어 주기 | `50 Hz` |
| 축거 | `1.2991 m` |
| 최대 조향 / 변화율 | `25 deg / 60 deg/s` |
| 직선 Ld | `clamp(4.0 + 0.9 * speed, 3.6, 9.5) m` |
| 강한 코너 Ld 하한 | `clamp(3.6 + 0.30 * speed, 3.6, 직선 Ld) m` |
| 곡률 미리보기 | `12.0 m` |
| 강한 코너 속도 상한 | `3.2 m/s` |
| 일반 가속 / 감속 | `1.0 / 2.0 m/s^2` |

직선 허용 통로는 예상 횡오차 `0.08~0.25 m`, 헤딩 오차 `0.8~2 deg` 사이에서 조향을
연속 복구한다. 코너·회피·REJOIN에는 적용하지 않는다.

정지 접근 중 마지막 2 m에서는 조향을 줄인다. 실제속도 0.1 m/s 이하에서는 현재 조향을
유지하고, 재출발 후 0.3 m/s 이상에서 경로 조향을 다시 적용한다.

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
colcon test --packages-select kaiev26_motion_control
colcon test-result --verbose
```
