# kaiev26_aeb

라바콘 중앙경로를 추종하고 빨간 콘 진입선에서 정지하는 제동 검차 패키지다.
일반 Decision, SysID와 동시에 실행하지 않는다.

## Scenario

| 번호 | 인지 | 횡제어기 | 센서 |
|---:|---|---|---|
| `1` | 카메라+LiDAR | Stanley | 좌우 카메라, Ouster |
| `2` | 카메라+LiDAR | Pure Pursuit | 좌우 카메라, Ouster |
| `3` | YOLO-only | Stanley | 좌우 카메라 |
| `4` | YOLO-only | Pure Pursuit | 좌우 카메라 |

기본값은 Scenario `2`, `5.0 km/h`다. AEB TUI 속도 단위는 `km/h`다.

## 동작

```text
카메라 + 선택적 LiDAR
  -> 파랑·노랑 경계와 중앙경로
  -> PP 또는 Stanley
  -> /planning/command

빨간 콘 쌍 2프레임 확인
  -> 앞범퍼가 진입선 통과
  -> speed=0, brake_engage=true latch
```

제동 latch는 인지가 사라지거나 AUTO를 유지해도 자동 해제되지 않는다.

## 준비

모델을 다음 위치에 둔다.

```text
~/KAI_ws/models/cone_yolo/yolov8n-cone.pt
```

모델은 `blue cone`, `yellow cone`, `red cone` 클래스를 포함해야 한다.

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select kaiev26_aeb
source install/setup.bash
test -f ~/KAI_ws/models/cone_yolo/yolov8n-cone.pt
```

차량은 콘 사이에 평행하게 놓고 앞바퀴를 중앙으로 맞춘다.
TUI Run 전에 위치 조정을 끝낸다.

## 실차 실행

모든 새 터미널에서 ROS와 `~/KAI_ws/install/setup.bash`를 source한다.

터미널 1, Control과 TF:

```bash
ros2 launch control_launch vehicle.launch.py
```

터미널 2, 좌우 카메라:

```bash
ros2 launch usb_cam camera.launch.py
```

Scenario `1·2`는 차량에 설치된 Ouster driver도 실행한다. Ouster launch는 이 저장소에 없으므로
실차에서 승인된 명령을 사용한다.

입력 확인:

```bash
ros2 topic hz /vehicle/state
ros2 topic hz /perception/camera/left/source/image_raw/compressed
ros2 topic hz /perception/camera/right/source/image_raw/compressed
ros2 topic echo --once /tf_static
```

Fusion은 `ros2 topic hz /ouster/points`도 확인한다.

터미널 3, AEB TUI:

```bash
ros2 run kaiev26_aeb cone_trial_tui
```

| 키 | 기능 |
|---|---|
| `1`~`4` | Scenario 선택 |
| `V` | 목표속도 `km/h` |
| `R` | 준비 검사, 파이프라인·MCAP 시작 |
| `S`, `Space` | 제동 요청 후 Run 종료 |
| `Q` | TUI 종료 |

1. 저속 Scenario를 선택하고 `R`을 누른다.
2. 모든 항목이 `GO`이고 `WAIT_AUTO`가 표시되는지 확인한다.
3. AUTO로 전환하고 5초 뒤 `RUNNING`을 확인한다.
4. `/aeb/center_path`가 유효한 상태에서만 진행한다.
5. 빨간선에서 `latched`, 브레이크 요청, 실제 정지를 확인한다.
6. MANUAL로 전환한 뒤 `S`로 기록을 마감한다.

위험하면 TUI보다 물리 E-Stop 또는 MANUAL을 먼저 사용한다.

## 준비 검사

| 검사 | 정상 조건 |
|---|---|
| 차량 | `/vehicle/state`, 전진 기어, E-Stop 해제 |
| 카메라 | 좌우 영상·CameraInfo와 TF 수신 |
| LiDAR | Fusion에서 point cloud와 TF 수신 |
| 모델 | 파일 존재와 로딩 성공 |
| 명령 | 기존 `/planning/command` 발행자 없음 |
| Control | `/vehicle/command` 발행자 하나 |
| 기록 | 필수 토픽 recorder 연결 |

입력·경로·차량 상태가 만료되면 속도 0과 브레이크를 요청한다.

## 기록

```text
~/KAI_ws/records/cone_trials/<시각>_cone_scenario_<번호>_v<속도>/
├── scenario.yaml
├── console.log
└── rosbag/
```

TUI가 카메라, 선택적 LiDAR, 콘·경계·경로, 빨간선, 명령, 차량 상태와 진단을
자동 기록한다.

## Gazebo

```bash
# GUI
ros2 launch kaiev26_aeb aeb_sim.launch.py scenario:=2

# GUI 없음
ros2 launch kaiev26_aeb aeb_sim.launch.py scenario:=2 headless:=true
```

Simulation `hwj`가 빌드되어 있어야 한다. 이 launch가 Gazebo, AEB, governor와 시험 기록을
실행하므로 다른 Gazebo·Decision launch를 함께 실행하지 않는다.

## 확인

```bash
ros2 topic echo --once /aeb/status
ros2 topic echo --once /aeb/tracking_status
ros2 topic echo --once /planning/command
ros2 topic info /planning/command
ros2 topic info /vehicle/command
```

| 토픽 | 내용 |
|---|---|
| `/aeb/center_path` | 콘 중앙경로 |
| `/aeb/cones`, `/aeb/boundaries` | 콘과 좌우 경계 |
| `/aeb/red_gate` | 빨간 진입선 |
| `/aeb/status` | 인지 상태·실패 이유 |
| `/aeb/tracking_status` | 추종·제동 latch 상태 |
| `/planning/command` | Decision 경계 명령 |
| `/vehicle/command` | Control 승인 명령 |

설정은 [`config/perception.yaml`](config/perception.yaml),
[`config/tracking.yaml`](config/tracking.yaml)을 사용한다.

## 테스트

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select kaiev26_aeb
colcon test-result --verbose
```
