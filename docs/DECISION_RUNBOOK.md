# Decision Runbook

실차와 Gazebo에서 Decision 시험, 기록, AEB 검차를 실행하는 순서다.

> 실차에서는 먼저 MANUAL, 물리 E-Stop, 수동 제동을 확인한다.

## 공통 준비

모든 새 터미널에서 실행한다.

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
```

코드 변경 후 빌드:

```bash
colcon build --symlink-install --packages-select \
  kaiev26_msgs kaiev26_decision kaiev26_motion_control \
  kaiev26_decision_fox kaiev26_aeb
source install/setup.bash
```

## 시험 Case

| Case | 기능 |
|---:|---|
| `1` | 예선 경로 추종 |
| `2` | 예선 추종 + 모든 정지점 3초 정지 |
| `3` | 예선 실제 미션 전체 |
| `4` | 본선 경로 추종 |
| `5` | 본선 추종 + 모든 정지점 3초 정지 |
| `6` | 본선 추종 + 장애물 회피 |
| `7` | 본선 실제 미션 전체 |
| `8` | 상수 속도 + R/L 조향 스텝 |

속도 `0`은 **예선 `3.2`, 본선 `3.5`**, Case 8 `1.0 m/s`다. 

Case `1·4`의 입력은 **직선 목표속도**이고, 

Case `2·3·5·6·7`은 **곡률·미션이 낮출 수 있는 순항 상한**이다.

## 실차 Decision 시험

### 터미널 1: Control

```bash
ros2 launch control_launch vehicle.launch.py
```

```bash
ros2 topic hz /vehicle/state
ros2 topic info /vehicle/command
```

`/vehicle/command` 발행자는 `command_governor` 하나여야 한다.

### 터미널 2: GNSS-only Localization

```bash
ros2 launch eskf_localization kcity_gnss.launch.py
```

이 launch가 **EBIMU, UM980 20 Hz, GNSS-only Localization**을 함께 실행한다. 

별도 드라이버를 중복 실행하지 않는다.

```bash
ros2 topic hz /gnss/fix
ros2 topic hz /gnss/fix_velocity
ros2 topic hz /imu/data
ros2 topic hz /localization/odometry
```

### 터미널 3: 좌우 카메라

```bash
ros2 launch usb_cam camera.launch.py
```

Case `1·2·4·5`에서는 선택 사항이다. 

Case `3·6·7`은 카메라와 Perception 팀의 승인된 실차 runtime을 함께 실행한다. 

runtime 이름은 Perception 배포 상태에 맞춰 선택하고, Decision에서는 아래 네 출력만 확인한다.

```bash
ros2 topic hz /perception/road_segments
ros2 topic hz /perception/centerline
ros2 topic hz /perception/objects
ros2 topic hz /perception/traffic_lights
```

### 터미널 4: Decision TUI

```bash
ros2 run kaiev26_decision decision_test
```

| 키 | 기능 |
|---|---|
| `1`~`8` | Case 선택 |
| `V` | 속도 입력 |
| `L` | PP, Stanley, PP+Stanley, FF+Stanley 선택 |
| `R` | 준비 검사와 Run 시작 |
| `S` | 안전 정지 후 기록 종료 |
| `Q` | TUI 종료 |

### 출발

1. MANUAL, 전진 기어, E-Stop 해제, 정지 상태를 확인한다.
2. Case·속도·횡제어기를 선택하고 **`R`을 누른다.**
3. MANUAL로 경로 방향을 따라 `0.5 m/s` 이상 **짧게 주행해 GNSS 헤딩을 얻는다.**
4. 경로 중앙에서 차체와 앞바퀴를 경로와 평행하게 맞추고 **정지한다.**
5. **`PRE-FLIGHT READY`**와 올바른 progress·Stage를 확인한다.
6. **AUTO로 전환**하고 3초 뒤 `RUNNING`을 확인한다.

준비 검사는 GNSS `RTK FIX(status=2)`, 위치·속도 covariance, 필수 액추에이터 상태를 먼저 확인한다. 

핵심 한계는 실제속도 `0.2 m/s`, CTE `2.0 m`, 헤딩 오차 30도, 위치 variance X·Y 각각 `0.25 m^2`다. 

**모든 조건을 1초 유지**해야 한다. 트랙 중간에서도 같은 절차를 따른다.

### Case 8

1. `8`을 선택한다.
2. `V` 속도, `A` 진폭, `F` 주파수를 입력한다.
3. `P`를 누르고 `R`, `L` 패턴을 연속 입력한 뒤 `Enter`를 누른다.
4. `R`로 Run을 시작하고 AUTO로 전환한다.
5. 3초 직진 후 설정 패턴이 반복된다.

**Case 8은 경로·인지·GNSS를 출발 조건으로 쓰지 않는다.**

궤적·헤딩 분석이 필요하면 터미널 2를 실행한다. 

MCAP에는 명령, 차량·조향 상태, STM 진단, 선택적 GNSS·IMU를 기록한다.

카메라와 LiDAR는 기록하지 않는다.

## 자동 기록

TUI가 Run마다 MCAP을 저장한다. 별도 recorder를 실행하지 않는다.

```text
~/KAI_ws/records/<시각>_decision_case_<번호>_<제어기>_v<속도>/
├── scenario.yaml
├── console.log
└── rosbag/
```

`S`, MANUAL 전환 또는 안전 조건으로 Run이 끝나면 recorder도 종료된다.

## 경량 Export

TUI에 `MCAP 저장 완료`가 표시된 후 실행한다.

```bash
# 아직 변환하지 않은 모든 Decision 기록
ros2 run kaiev26_decision kaiev26_export

# 최근 기록만
ros2 run kaiev26_decision kaiev26_export latest
```

출력은 `~/KAI_ws/exports/kaiev26_tracking/`에 저장된다. 

노트북으로 전송:

```bash
mkdir -p ~/KAI_ws/bags/field_tracking
rsync -avhP \
  kai@192.168.6.9:/home/kai/KAI_ws/exports/kaiev26_tracking/ \
  ~/KAI_ws/bags/field_tracking/
```

## 수동주행 MCAP

Control, GNSS-only Localization과 필요한 센서를 먼저 실행한 뒤 사용한다.

```bash
ros2 launch lateral_ctrl mcap_logging.launch.py
```

MANUAL로 주행하고, 완전히 정지한 뒤 로깅 터미널에서 `Ctrl+C`를 한 번 누른다. 

Decision, AEB, SysID recorder와 동시에 실행하지 않는다.

## Gazebo Decision 시험

GUI:

```bash
ros2 launch kaiev26_gazebo_bringup decision_test.launch.py \
  case:=2 speed_mps:=3.2 lateral_controller:=pure_pursuit
```

이 launch가 Gazebo, 차량, Localization, Mock 인지, Control bridge와 Decision을 함께 실행한다.
**별도 Control·GNSS·Decision launch를 실행하지 않는다.**

## 경기 판단 직접 실행

TUI와 자동 기록 없이 판단 파이프라인만 실행한다. 

**Control, Localization, 카메라와 실제 인지**를 먼저 실행한다.

```bash
# 예선
ros2 launch kaiev26_decision decision.launch.py \
  course:=qualifying use_sim_time:=false

# 본선
ros2 launch kaiev26_decision decision.launch.py \
  course:=final use_sim_time:=false
```

## 라바콘 제동 검차

일반 Decision을 종료한 뒤 실행한다. 전체 절차는
[`kaiev26_aeb/README.md`](../kaiev26_aeb/README.md)를 따른다.

```bash
# 실차
ros2 run kaiev26_aeb cone_trial_tui

# Gazebo
ros2 launch kaiev26_aeb aeb_sim.launch.py scenario:=2
```

## Foxglove

차량 PC에서 공용 bridge 하나만 실행한다.

```bash
ros2 launch foxglove foxglove_bridge.launch.py
```

각 노트북의 Foxglove Desktop에서 `Open connection -> Foxglove WebSocket`을 선택한다.

| 연결 | 주소 |
|---|---|
| 차량 Wi-Fi/LAN | `ws://192.168.6.9:8765` |
| Tailscale | `ws://100.112.72.27:8765` |
| 차량 PC | `ws://127.0.0.1:8765` |

여러 노트북이 같은 bridge에 접속할 수 있다. 

같은 포트의 다른 bridge를 동시에 실행하지 않는다. 

Control 레이아웃은 다음 파일을 각 노트북에서 한 번 import한다.

```text
~/KAI_ws/src/Control/software/src/foxglove/control.json
```

Decision 전용 화면은 공용 bridge 대신 다음 명령을 사용한다.

```bash
ros2 launch kaiev26_decision_fox fox.launch.py mode:=auto
```

저장된 MCAP은 Foxglove Desktop의 `Open local file`에서 `.mcap` 파일을 직접 연다.

## AUTO인데 움직이지 않을 때

```bash
ros2 topic echo --once /planning/route_ready
ros2 topic echo --once /planning/route_context
ros2 topic echo --once /debug/active_behavior
ros2 topic echo --once /debug/fsm_state
ros2 topic echo --once /decision/raw_command
ros2 topic echo --once /planning/command
ros2 topic info /planning/command
ros2 topic info /vehicle/command
```

| 결과 | 확인 |
|---|---|
| `PREFLIGHT`, `WAIT_AUTO`, `COUNTDOWN` | TUI 출발 단계 완료 여부 |
| `route_ready=false` | GNSS, covariance, 헤딩, CTE |
| `route_projection_valid=false` | 현재 위치와 전역경로 정합 |
| raw 속도 0·브레이크 true | 현재 FSM 정지 이유 |
| raw는 주행, planning은 정지 | TUI 게이트 상태 |
| `/planning/command` 발행자 2개 이상 | 다른 Decision·AEB·SysID 종료 |
| KEYA/CAN fault | 즉시 MANUAL 후 Control 상태 확인 |

조향 추종 오차 경고만으로 Decision이 감속하거나 시험을 끝내지는 않는다. 

실제 Control fault, E-Stop, CAN timeout은 계속 유효하다.
