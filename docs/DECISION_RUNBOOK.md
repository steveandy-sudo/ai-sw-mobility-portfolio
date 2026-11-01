# Decision Runbook

실차 Decision, Gazebo 통합 시험과 라바콘 제동 검차의 실행 순서다.

> 실차에서는 먼저 MANUAL, 물리 E-Stop과 수동 제동을 확인한다.

## 공통 준비

모든 새 zsh 터미널에서 실행한다.

```zsh
cd ~/KAI_ws
source /opt/ros/humble/setup.zsh
source install/setup.zsh
export ROS_DOMAIN_ID=0
```

코드 변경 후 빌드:

```zsh
colcon build --symlink-install --packages-select \
  kaiev26_msgs kaiev26_maps pv26_perception cone_perception \
  kaiev26_decision kaiev26_motion_control kaiev26_decision_fox kaiev26_aeb
source install/setup.zsh
```

## 실차 시험

미션 실행·경로추종·수동 경로 수집·회피 시험은 같은 TUI를 사용한다.

### 1. Control

```zsh
ros2 launch control_launch vehicle.launch.py
```

### 2. Localization

```zsh
ros2 launch eskf_localization kcity_gnss.launch.py
```

### 3. Perception

- 미션·회피 시험은 좌우 카메라와 차량의 LiDAR driver 실행이 필요하다.
- 실차 인지는 기본값인 system time을 사용한다.
- 미션 실행은 `SceneSummary` 상태 `GOOD`을 요구한다. 경로추종·수동 경로 수집은 인지를 요구하지 않는다.
- 수집 경로 회피는 같은 frame의 최신 `/perception/objects`와 유효한 Route 정합으로도 출발할 수 있다.

좌우 카메라와 LiDAR, 각각 별도 터미널:

```zsh
ros2 launch usb_cam camera.launch.py
```

```zsh
ros2 launch hesai_ouster_compatible_ros driver.launch.py
```
```zsh
ros2 launch pv26_perception pv26_perception.launch.py
```

```zsh
ros2 topic hz /perception/road_segments
ros2 topic hz /perception/centerline
ros2 topic hz /perception/objects
ros2 topic hz /perception/traffic_lights
```


### 4. Decision TUI

녹화 없는 실차 운용:

```zsh
ros2 launch kaiev26_decision decision.launch.py
```

- 기존 TUI와 같은 상태·출발 검사 화면이다. `P` 예선·본선, `V` 속도, `B` 출발 모드, `T` 추종기, `R` 시작, `S` 중단, `Q` 종료만 사용한다.
- 경로 선택 시 예선 `7.0 m/s`, 본선 `10.0 m/s`로 설정된다. 실행 중에는 경로·속도를 바꿀 수 없다.
- ALL 미션·PP+Stanley·system time을 사용하며 신호 현시 추적기도 함께 실행한다.
- MCAP·`scenario.yaml`·`console.log`·`records` 폴더는 생성하지 않는다. ROS 기본 로그는 남는다.
- 아래 시험 TUI는 MCAP을 기록하며 같은 ROS Domain에서 운용 화면과 중복 실행하지 않는다.

```zsh
# 시험 Case TUI
ros2 run kaiev26_decision decision_test
```
- 경로 추종은 PP+Stanley로 고정된다.
- 출발 준비 검사는 운용 화면과 동일하다.
- 기본 속도는 예선 `7.0 m/s`, 본선 `10.0 m/s`, 수집 경로 `10.0 m/s`다.
- 추종기는 하나다. 직선 Stanley 이득을 속도별로 계산하고 경로 지연 보상, 직진 편향 적분, 엔코더 오차 기반 조향 보정, 곡선별 속도 상한을 쓴다(`PARAMETER_BOOK.md`의 직선 추종 절, 근거 `reports/261022_경로추종기_속도별_재검토`).
- 시험 TUI의 `V` 입력은 `P`로 경로를 바꿔도 유지된다. 녹화 없는 운용 화면은 경로 변경 시 기본속도로 돌아간다. `V=0`이면 선택 경로 기본속도를 사용한다.

| 키 | 기능 |
|---|---|
| `P` | 예선 → 본선 → 저장된 수집 경로 선택 |
| `1` | 미션 실행: 선택 코스 ALL |
| `2` | 전역경로 추종 |
| `3` | 수동주행 경로 수집·등록 |
| `4` | 장애물 회피: 수집 경로를 선택했을 때만 표시 |
| `V` | 목표속도 `m/s` |
| `B` | 출발 모드: `처음부터`(기본) / `이어서`. Case 3 제외 |
| `N` | Case 3 경로 이름 |
| `H` | Case 3·4 현장 메모 |
| `R` | 준비 검사와 Run 시작 |
| `S`, `Space` | Run 종료. Case 3은 기록만 종료하며 차량 제동 안 함 |
| `Q` | TUI 종료 |

- 예선·본선 경로는 `kaiev26_decision/waypoints/`, 수집 경로는 `kaiev26_maps/maps/midan/`에 있다.
- 수집 경로는 미션 Policy가 없으므로 Case `2~4`만 지원한다.
- 주행 Case의 MCAP에는 조향 토크·요청각·Control 파라미터 토픽(`/steering/torque_command_raw` 등, `FIELD_EXTRA_TOPICS`)도 기록한다.

### 5. 출발

1. MANUAL, 전진 기어, E-Stop 해제와 정지를 확인한다.
2. 운용 화면은 경로·속도를 정하고, 시험 TUI는 Case도 선택한다.
   - 출발점에서 시작하면 `처음부터`, 경로 중간에서 다시 시작하면 `B`로 `이어서`를 고른다.
   - `처음부터`는 출발 구간에서 위치만으로 진행도를 맞추므로 헤딩이 뒤집혀도 종점 구간에 붙지 않는다.
3. `R`을 누른다.
4. MANUAL로 경로 방향을 따라 `0.5 m/s` 이상 짧게 주행해 GNSS 헤딩을 얻는다.
5. 경로와 차체·앞바퀴를 평행하게 맞추고 정지한다.
6. `PRE-FLIGHT READY`, progress와 Stage를 확인한다.
7. AUTO로 전환하고 3초 뒤 `RUNNING`을 확인한다.

- 준비 검사는 RTK FIX, 최신 차량·액추에이터·Route, 위치 covariance, 속도 `0.2 m/s` 이하를 확인한다.
- 모든 조건은 1초 안정되어야 한다.
- 출발 전 헤딩오차 `45°` 또는 횡오차 `3 m` 초과는 준비 완료를 차단한다.
- AUTO 정상 주행 확인 후에는 헤딩 반전만으로 다른 경로 구간을 재선택하지 않는다.

## 자동 MCAP
```zsh
ros2 run kaiev26_decision kaiev26_export
ros2 run kaiev26_decision kaiev26_export latest
```
```text
~/KAI_ws/records/YYMMDD_HHMM_decision_case0#_<경로>_V<속도>/
├── scenario.yaml
├── console.log
└── rosbag/
```

- 예: `261021_1400_decision_case01_final_V10`이다.
- 예선·본선 경로명은 `qualifying`·`final`, 수집 경로는 입력한 이름을 쓴다.
- 속도는 `m/s`이며 소수점은 `p`로 표시한다(`V7p5`).
- Case 3도 같은 형식이며 경로명은 `N` 입력값, 속도는 수동 수집을 뜻하는 `V0`다.
- 같은 이름이 있으면 `_02`, `_03`을 붙여 기록을 구분한다.
- 정확한 시작 시각·모드·설정은 `scenario.yaml`에 보존한다.

## Gazebo 통합 시험

```zsh
ros2 run kaiev26_gazebo_bringup gazebo_launcher
ros2 run kaiev26_decision decision_test --sim-time
```

## 기본 운용 실행
```zsh
ros2 launch kaiev26_decision decision.launch.py
```
- 인라인 인자 없이 화면에서 코스·속도를 고른다. Control, Localization과 경기 인지는 먼저 실행한다.
- `R` → 준비 검사 → `PRE-FLIGHT READY` → 운전자 AUTO 전환 → 3초 후 명령 중계 순서다.
- `S`·MANUAL 전환·결승 정지로 Run을 끝내고 `Q`로 프로그램을 종료한다. Control 등 외부 실행 노드는 종료하지 않는다.
- launch는 키보드를 연결할 실제 터미널이 필요하다. SSH에서는 대화형 터미널에서 실행한다.

## 라바콘 제동 검차

Control, 좌우 카메라와 `/ouster/points`를 준비한다.

```zsh
ros2 run kaiev26_aeb cone_trial_tui
```
- 기본값은 `30 km/h`, 빨간선 선행 제동거리 `5.0 m`이며, 속도명령 증가율은 `1.0 m/s²`로 고정한다.
- 자세한 내용은 [`kaiev26_aeb/README.md`](../kaiev26_aeb/README.md)를 따른다.

## Foxglove

```zsh
ros2 launch foxglove foxglove_bridge.launch.py
```

## 움직이지 않을 때

| 현상 | 확인 |
|---|---|
| `route_ready=false` | GNSS, covariance, 액추에이터, 경로 투영 |
| 인지 미션이 `WAIT` | 인지 네 출력, 시각·frame, `perception_health` |
| raw는 주행, planning은 정지 | TUI 상태와 AUTO countdown |
| planning 발행자 중복 | 다른 Decision·AEB·SysID 종료 |
| 입력 stale | 속도 0, 1초 뒤 정지까지 브레이크. 입력이 돌아오고 서면 Run이 이어진다 |
| launch 종료 | 원인 복구 후 새 Run 시작 |
| KEYA·CAN fault | 즉시 MANUAL 후 Control 진단 |

조향 추종 오차 경고만으로 Decision은 감속하거나 Run을 끝내지 않는다.
