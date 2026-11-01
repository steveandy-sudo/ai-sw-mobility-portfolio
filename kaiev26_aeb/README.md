# kaiev26_aeb

인지팀의 컬러 콘 3D 위치로 중앙경로를 만들고 빨간 콘 진입선 전에 정지하는 제동 검차 패키지다.
일반 Decision이나 SysID와 동시에 실행하지 않는다.

## 횡제어기

TUI는 직선 검차용 Stanley로 고정한다(`scenario 2`, 기본 최고속도 `30 km/h`).
Pure Pursuit(`scenario 1`)는 `aeb_test.launch.py` 인자로만 남는다.
`cone_perception`의 `/perception/cone/detections`를 사용한다.

Stanley는 예선·본선 추종기(`kaiev26_motion_control`)와 같은 조향 규칙과 값을 쓴다.
값은 `kaiev26_motion_control/config/motion_control.yaml`에서 읽으므로 그쪽에서만 바꾼다.

- 직선 이득: `3 m/s` 이하 고정 `0.30/0.25`, `6 m/s` 이상 고유진동수·감쇠로 속도마다 계산
- 보정 한도: 헤딩 `0.10 rad`, 횡오차 `0.05 rad`
- 직선 적분: 앞바퀴 횡오차 `0.6 m` 이하·`4 m/s` 이상에서 시간상수 `4 s`, `±1.5°`, 빨간선 제동 중 유지
- 조향 보정: 영점 `−0.82°`, 엔코더가 명령에 못 미칠 때 속도별 마찰 밀기
- 경로·빨간선 움직임 보상은 직진 엔코더각 `−0.45°`를 뺀 조향으로 계산

```text
좌우 카메라 + LiDAR(`/ouster/points`)
  -> cone_perception: 색상·3D 위치
  -> 파랑/노랑/빨강 좌우 경계 독립 피팅
  -> /aeb/center_path
  -> Stanley (예선·본선 추종기 값)
  -> /planning/command

빨간 콘 양쪽 진입선 2프레임 확인
  -> 설정한 거리 전에 speed=0, brake_engage=true
  -> 실제속도 0.5 m/s 초과: 중앙경로 조향 계속 추종
  -> 실제속도 0.5 m/s 이하: 마지막 제한 조향각 고정
  -> 정지 latch 유지
```

속도명령 증가율은 `1.0 m/s²`로 고정하며, 앞범퍼 기준 선행 제동거리 기본값은 `5.0 m`다.
제동 중 조향 추종 종료 속도는 `0.5 m/s`다. 제동 latch와 조향 고정 상태는 인지가
사라지거나 AUTO를 유지해도 자동 해제되지 않는다.

## 준비

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cone_perception kaiev26_aeb
source install/setup.bash
test -f "$(ros2 pkg prefix cone_perception)/share/cone_perception/models/yolov8n-cone.engine"
```

- 차량을 콘 사이에 평행하게 놓고 앞바퀴를 중앙에 맞춘다.
- Control, 좌우 카메라와 현재 차량 LiDAR driver를 먼저 실행한다.
- GNSS는 필수가 아니다.
- TUI가 `cone_perception`을 시작하므로 별도로 실행하지 않는다.

입력 확인:

```bash
ros2 topic hz /vehicle/state
ros2 topic hz /ouster/points
ros2 topic hz /perception/camera/left/source/image_raw/compressed
ros2 topic hz /perception/camera/right/source/image_raw/compressed
ros2 topic echo --once /tf_static
```

## 실행

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run kaiev26_aeb cone_trial_tui
```

| 키 | 기능 |
|---|---|
| `V` | 최고 목표속도 `km/h` (기본 `30`) |
| `B` | 빨간선 선행 제동거리 `m` (기본 `5.0`, 앞범퍼가 빨간선 이 거리 앞에 오면 속도 0) |
| `E` | E-stop `NOT-USE`(기본): 빨간선에서 `brake_engage=true` / `USE`: 사람이 E-stop으로 정지. 빨간선은 인지·표시만 하고 속도·조향을 바꾸지 않으며, 추종기는 어떤 경우에도 브레이크를 체결하지 않는다 |
| `R` | 준비 검사 후 실행 |
| `S`, `Space` | 제동 요청 후 종료 |
| `Q` | TUI 종료 |

1. `V/B`와 E-stop `USE/NOT-USE`를 정한다.
2. `R`을 누르고 모든 준비 항목이 `GO`인지 확인한다.
3. `WAIT_AUTO`에서 AUTO로 전환한다.
4. 5초 뒤 `RUNNING`과 유효한 `/aeb/center_path`를 확인한다.
5. 빨간선에서 제동 latch와 실제 정지를 확인한다.
6. MANUAL로 전환한 뒤 `S`로 기록을 마감한다.

위험하면 TUI보다 물리 E-Stop 또는 MANUAL 전환을 먼저 사용한다. `30 km/h`, `1.0 m/s²`,
`5.0 m`는 시험 시작값이며 실차 제동거리를 보증하지 않는다.

## 준비 검사

| 항목 | 정상 조건 |
|---|---|
| 차량 | `/vehicle/state`, 전진 기어, E-Stop 해제 |
| 센서 | 좌우 영상·CameraInfo, `/ouster/points`, TF 수신 |
| 인지 | TensorRT 엔진 존재, 실행 후 `/perception/cone/detections` 수신 |
| 명령 | 기존 `/planning/command` 발행자 없음 |
| Control | `/vehicle/command` 발행자 하나와 governor 연결 |
| 기록 | 필수 토픽 recorder 연결 |

차량 상태가 만료되거나 E-stop·후진·비활성이면 속도 0과 브레이크를 요청한다.
콘 중앙경로가 비거나 `0.35 s`보다 늦으면 `0.5 s`(`path_gap_hold_s`)까지 직전 속도·조향을
유지하고 브레이크를 요청하지 않는다. 그보다 길면 속도 0과 브레이크를 요청한다.
추종 목표점만 잠시 사라질 때도 직전 속도·조향을 유지하며, 이 상태만 2초 지속되면
TUI가 시험을 종료한다. 정지 요청 뒤 추종이 돌아오면 속도 명령은 0이 아니라 실제 차속부터
다시 오른다. MCAP 기록 구독이 끊기면 출발 전에는 NO-GO, 카운트다운·주행 중에는 로그만 남기고
주행을 계속한다.

## 기록

```text
~/KAI_ws/records/cone_trials/<시각>_cone_scenario_<번호>_v<속도>/
├── scenario.yaml
├── console.log
└── rosbag/
```

TUI가 콘 검출, 경계·중앙경로, 빨간선, 명령, 차량·액추에이터 상태와 센서 입력을
`zstd_fast` MCAP으로 자동 기록한다. 추종 분석용으로 `/imu/data`, `/vehicle/wheel_twist`,
조향 상세(요청각·오차·조향률·토크·포화), `/rosout`, `/parameter_events`도 남긴다.
`scenario.yaml`의 `stanley_config`에는 실행에 쓴 Stanley 값과 읽은 `motion_control.yaml` 경로가
들어간다(읽지 못하면 `error`).

## 확인

```bash
ros2 topic echo --once /aeb/status
ros2 topic echo --once /aeb/tracking_status
ros2 topic echo --once /planning/command
ros2 topic info /vehicle/command
```

| 토픽 | 내용 |
|---|---|
| `/aeb/center_path` | 콘 중앙경로 |
| `/aeb/cones`, `/aeb/boundaries` | 콘과 좌우 경계 |
| `/aeb/red_gate` | 빨간 진입선 |
| `/aeb/status` | 경로 생성 상태 |
| `/aeb/tracking_status` | 추종·제동 latch 상태 |
| `/planning/command` | AEB 출력 명령 |
| `/vehicle/command` | Control 승인 명령 |

설정은 [`config/perception.yaml`](config/perception.yaml)과
[`config/tracking.yaml`](config/tracking.yaml)에 있다.

## 테스트

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select kaiev26_aeb
colcon test-result --test-result-base build/kaiev26_aeb
```
