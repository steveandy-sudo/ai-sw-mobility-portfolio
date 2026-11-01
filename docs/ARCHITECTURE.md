# Decision Architecture

Decision이 인지·측위·차량 상태를 받아 목표 경로와 속도를 만들고, Motion Control이 이를
차량 명령으로 바꾸는 구조다.

## 패키지 경계

| 패키지 | 책임 |
|---|---|
| `pv26_perception` | 경기 차선·중심선·장애물·신호등 인지. 모든 결과를 내고 고르지 않는다 |
| `kaiev26_decision` | Route 정합, Event, 정지점 기계, 장애물 행동, 목표 경로·속도 |
| `kaiev26_motion_control` | PP·Stanley, 가감속 제한, 제동 명령 |
| `kaiev26_aeb` | 별도 라바콘 제동 검차 |
| `kaiev26_decision_fox` | 읽기 전용 시각화 |
| Control | 명령 검증, STM·KEYA 구동, 차량 상태 합성 |

Decision은 목표값을 만들고 Control은 실제 구동 허용과 액추에이터 보호를 담당한다.

## 경기 데이터 흐름

```text
/localization/odometry
      -> route_zone_manager -> /planning/route_context

route_context + /vehicle/state
  + /perception/objects + /perception/traffic_lights + /planning/traffic_signal_phases
      -> main_planning_engine
      -> /planning/target_path + /planning/target_speed + /planning/behavior_decision
      -> motion_control -> /planning/command
      -> command_governor -> /vehicle/command

/perception/traffic_lights
      -> signal_phase_tracker -> /planning/traffic_signal_phases

pv26_perception
  /perception/{road_segments,centerline,objects,traffic_lights}
      -> perception_gateway -> /planning/scene_summary (TUI 준비 검사, 모니터)
```

- 세 노드(perception_gateway, route_zone_manager, main_planning_engine)는 따로 돈다(결정 D17).
- main_planning_engine은 `/planning/scene_summary`를 읽지 않는다. 신호등과 장애물은 인지 토픽을 직접 받는다.
- `cone_perception`의 `/perception/cone/detections`는 제동 검차 전용이다.

## main_planning_engine 구조

30 Hz 틱마다 아래 모듈이 각자 출력(`ModuleOutput`)을 내고 중재기가 하나로 합친다(결정 D18).

| 모듈 | 파일 | 하는 일 |
|---|---|---|
| 입력 상태 | `main_planning_engine_node.py` (`input_problems`) | 경로 정합과 차량 상태만 본다. 인지는 보지 않는다(D3). E-Stop |
| 정지점 기계 | `stop_point.py` | 신호, 시간 정지, 결승이 함께 쓰는 기계 하나와 Event 추적기 |
| 신호 입력 | `signal_schedule.py` | 자기 신호등의 정렬된 현시표를 신호 입력으로 쓴다. 카메라는 확실한 반대를 볼 때만 거부한다 |
| 허용 계산 | `event_permission.py` | 녹색 확인, 황색 딜레마, 3초 정지, UNKNOWN 탈출과 110초 대기 |
| Event 목록 | `mission_events.py` | 시작할 때 코스 Policy와 모드로 한 번 만든다 |
| 전역경로 추종 | `main_planning_engine_node.py` (`route_follow`) | Route 로컬 경로와 순항 상한 |
| 곡선 속도 | `curve_speed_application.py` (`curve_speed_cap`) | 곡선 구간 상한. 상한만 낮춘다 |
| 구간 속도 | `curve_speed_application.py` (`speed_zone_cap`) | 코스 Policy `speed_zones`(예선 어린이보호구역) 상한과 미리 감속. 상한만 낮춘다 |
| 장애물 세계 | `obstacle_world.py` | 추적 객체를 map 좌표로 기억, 구간 다각형, 지도 차선 경계 |
| 장애물 행동 | `obstacle_behavior.py`, `obstacle_sequence.py` | 구간 상한과 미리 감속, 회피 경로 확정과 유지 |
| 판단 AEB | `aeb_logic.py` | TTC 경고만 한다(D8) |
| 중재기 | `arbiter.py` | 상한은 최솟값, 정지 요청이 하나라도 있으면 정지, 거리는 최솟값 |

출력을 쓸 수 있는 모듈은 `arbiter.OUTPUT_OWNERSHIP`가 정한다. 표 밖에 쓰면 `OwnershipError`가 난다.

| 모듈 | 경로 | 정지 요청 | 속도 상한 |
|---|:-:|:-:|:-:|
| `input_state` | | O | O |
| `stop_point` | | O | O |
| `obstacle` | O | | O |
| `route_follow` | O | | O |
| `curve_speed` | | | O |
| `speed_zone` | | | O |
| `aeb` | | | |

- 경로는 장애물 행동이 회피 경로를 요청하는 동안(복귀점까지) 그 경로이고, 그 밖에는 전역경로 추종 경로다. 결승 제동이 복귀 전에 시작되어도 회피 경로를 유지한다.
- 장애물 행동은 정지를 요청할 수 없다.
- 정지 명령은 `common.stop_command()` 하나로 만든다. `need_stop=true`, 목표 0, 거리는 요청 거리다. 거리 0은 "지금 제동"이다(D1).
- `TargetSpeed`, `Centerline`, `BehaviorDecision`의 `header.stamp`는 그 틱의 시각이다.

### 정지점 기계

```text
FREE -> WATCH -> BRAKING -> HELD -> LAUNCH -> COMMITTED -> CLEARED
            \-> COMMITTED (8 m 안에서 허용 확인, 또는 황색 딜레마)
```

- `WATCH`: 정지선 접근 거리(보통 30 m) 안. 허용이 없으면 정지거리 지점에서 `BRAKING`, 제동 WP 8 m 전부터는 2.0 m/s 상한.
- `BRAKING`, `HELD`: 정지 요청(거리 0)을 낸다. 실제속도 0.1 m/s 이하에서 `HELD`.
- `LAUNCH`: 허용(녹색 확인, 3초, UNKNOWN 탈출, 110초)으로 출발한다. 다음 틱에 `COMMITTED`이며 같은 Event로 다시 제동하지 않는다(D4).
- `CLEARED`: 후륜축이 선을 지났다. 추적기는 `CLEARED`에서만 다음 Event로 넘어가므로, 선을 넘어 선 채 `HELD`인 Event를 떠나지 않는다.
- 입력이 끊긴 틱에는 기계를 진행하지 않고 지금 상태를 그대로 낸다. `BRAKING`, `HELD`의 제동은 끊김으로 풀리지 않는다.
- 결승은 허용을 받지 않는다. 20 m 안에서 0.6 m/s 상한, 앞범퍼가 종료선 뒤 5 m 안에 서도록 제동, 늦어도 마지막 1 m에서 정지한다. 정지한 뒤 `FINISH_HELD`를 붙이고 TUI는 이 값 하나로 완료를 정한다(D11).
- 접근 구간이 앞 Event의 정지선보다 앞에서 시작하면 노드가 시작하지 않는다.

### 판단 표시 순서

`active_behavior`는 아래 순서로 정한다. 표시일 뿐 출력은 중재기가 정한다.

```text
MANUAL_OVERRIDE -> EMERGENCY(E-Stop) -> RECOVERY(입력 끊김)
  -> 정지점 기계의 Event(TRAFFIC_LIGHT, STATIC_STOP, FINISH) -> OBSTACLE -> GLOBAL_ROUTE_FOLLOW
```

- MANUAL이면 경로만 내고 상한은 현재 속도다. MANUAL로 들어가고 나올 때 지금 Event는 `FREE`부터 다시 시작하고, 끝난 Event는 그대로 끝난 것으로 둔다.
- E-Stop은 정지 요청으로 중재기에 들어간다.
- Decision의 TTC AEB는 경고만 한다(로그와 `AEB_WARNING`, D8). 장애물 구간 안과 회피 중에는 경고도 끈다. 물리 E-Stop은 항상 우선한다.
- 일반 CTE·헤딩오차 자동 REJOIN은 없다. 회피 경로가 끝나면 전역경로로 돌아온다.

### fsm_state

`BehaviorDecision.fsm_state`와 `/debug/fsm_state`는 사람이 보는 이름이다(TUI, debug_monitor, Foxglove, 하네스 기록). 제어는 `active_behavior`와 constraint 태그만 읽는다. 이름은 종류 접두어와 공통 낱말(APPROACH, PRE_BRAKE, BRAKE, WAIT, PASS)로 짓는다.

| `active_behavior` | `fsm_state` | 뜻 |
|---|---|---|
| GLOBAL_ROUTE_FOLLOW | `ROUTE_FOLLOW` | 이벤트·장애물 없이 경로 추종 |
| TRAFFIC_LIGHT | `SIGNAL_APPROACH_GO` / `SIGNAL_APPROACH_STOP` | 접근 구간, 지금 통과 허용 / 불허 |
| TRAFFIC_LIGHT | `SIGNAL_PRE_BRAKE` / `SIGNAL_BRAKE` / `SIGNAL_WAIT` / `SIGNAL_PASS` | 제동 WP 앞 감속 / 제동 / 서서 허용 대기 / 통과 확정 |
| STATIC_STOP | `TIMED_STOP_APPROACH` / `_PRE_BRAKE` / `_BRAKE` / `_WAIT` / `_PASS` | 접근 / 감속 / 제동 / 정지 시간 채움 / 출발해 선을 넘는 중 |
| FINISH | `FINISH_APPROACH` / `FINISH_BRAKE` / `FINISH_STOPPED` | 결승 감속 / 제동 / 정지 완료(D11) |
| OBSTACLE | `OBSTACLE_APPROACH` / `_LANE_KEEP` | 구간 앞 감속 / 구간 안 차선 주행 |
| OBSTACLE | `OBSTACLE_BYPASS` / `_BYPASS_FORCED` / `_RETURN` / `_NO_PATH` | 회피 / 여유가 모자란 강제 S / 마지막 줄 뒤 복귀 / 회피 경로 없이 저속 |
| RECOVERY | `INPUT_LOSS_COAST` / `INPUT_LOSS_HOLD_BRAKE` / `INPUT_LOSS_BRAKE` | 브레이크 없이 속도 0 / 걸려 있던 브레이크 유지 / 1초 뒤 정지까지 제동 |
| EMERGENCY | `ESTOP` | 비상 정지 |
| MANUAL_OVERRIDE | `MANUAL` | 수동 모드 |

옛 기록은 다른 이름을 쓴다. 10/23 kgm 기록은 정지점 기계의 단계 이름이고, 10/22 이전 main 기록은 시나리오별 FSM 이름이다. main 이름은 FSM이 여럿이라 대략의 대응이다.

| 새 이름 | kgm (10/23) | main (10/22 이전) |
|---|---|---|
| `ROUTE_FOLLOW` | `GLOBAL_ROUTE_FOLLOW` | `GLOBAL_ROUTE_FOLLOW`, `LANE_GOOD`, `LANE_DEGRADED`, `LANE_LOST`, `ROUTE_REJOIN` |
| `SIGNAL_APPROACH_GO` | `WATCH` | `APPROACH` |
| `SIGNAL_APPROACH_STOP` | `WATCH` | `DECELERATE` |
| `SIGNAL_PRE_BRAKE`, `TIMED_STOP_PRE_BRAKE` | `WATCH` | `PRE_DECEL` |
| `SIGNAL_BRAKE`, `TIMED_STOP_BRAKE` | `BRAKING` | `STOP_CONFIRM` |
| `SIGNAL_WAIT` | `HELD` | `STOP_HOLD`, `WAIT_GREEN` |
| `SIGNAL_PASS` | `LAUNCH`, `COMMITTED` | `START`, `PASSING` |
| `TIMED_STOP_APPROACH` | `WATCH` | `DECELERATE` |
| `TIMED_STOP_WAIT` | `HELD` | `STOP_HOLD`, `HOLD_3S` |
| `TIMED_STOP_PASS` | `LAUNCH`, `COMMITTED` | `RELEASE`, `COMPLETE` |
| `FINISH_APPROACH` | `WATCH` | `APPROACH_FINISH`, `SLOW_DOWN` |
| `FINISH_BRAKE`, `FINISH_STOPPED` | `BRAKING`, `HELD` | `MISSION_COMPLETE` |
| `OBSTACLE_APPROACH` | `ZONE_PRE_SLOW` | `PRE_DECEL` |
| `OBSTACLE_LANE_KEEP` | `ZONE` | `CLEAR` |
| `OBSTACLE_BYPASS` | `BYPASS` | `BYPASS_READY`, `BYPASS_HOLD`, `BYPASS_CLOSE`, `BYPASS_COMMITTED` |
| `OBSTACLE_BYPASS_FORCED` | `BYPASS` | `FORCED_S_CONTINUE`, `FORCED_PF_CONTINUE` |
| `OBSTACLE_RETURN` | `BYPASS` | `RETURN` |
| `OBSTACLE_NO_PATH` | `FALLBACK_CONTINUE` | `FALLBACK_CONTINUE` |
| `INPUT_LOSS_COAST`, `INPUT_LOSS_HOLD_BRAKE` | `INPUT_STALE` | `STOP_SAFE` |
| `INPUT_LOSS_BRAKE` | `INPUT_LOSS_BRAKE` | `INPUT_LOSS_BRAKE` |
| `ESTOP` | `ESTOP` | `STOP` |
| `MANUAL` | `MANUAL_CONTROL` | `MANUAL_CONTROL` |

### 장애물

- 장애물 세계는 `/perception/objects`의 장애물과 라바콘을 map 좌표로 기억한다. `track_id`가 같으면 같은 객체이고, `track_id` 0은 1.0 m 안의 가장 가까운 기억 객체에 붙인다(D15). 기억 객체는 후륜축이 지나간 뒤에 잊는다.
- 본선 구간 다각형(`final_policy.yaml`의 `obstacle_zone.polygon`, 본선 WP 640부터 WP 670까지, 경로 639.85~669.84 m의 양 차로) 밖 객체는 버린다(D14). 구간 안의 차선 한계는 지도 차선 경계(`kcity_course_semantics.yaml`)에서 읽는다. 지도 선이 끊긴 곳은 양쪽 샘플을 이어 쓴다.
- 객체는 장애물이 켜진 Stage에서만 받는다. 본선 출발 구간도 같은 도로의 반대 차로라 다각형 안에 들기 때문이다.
- 구간 안 상한은 `3.0 m/s`, 구간 앞은 `0.5 m/s²`로 미리 낮춰 앞범퍼가 구간에 들어갈 때 3.0 m/s가 되게 한다(D10). 목표만 낮추고 제동하지 않는다. 구간 시작은 `obstacle_zone.start_route_s`(본선 WP 640, 경로 639.85 m)이고, 없으면 다각형 시작이다. 세 값 모두 `obstacle_zone`에 있다. 상한은 차가 구간 끝을 지나고 회피가 끝나면 풀리고, 그다음은 코스 속도다. 회피 끝은 후륜축이 마지막 열을 `obstacle_release_past_row_m`(3.0 m) 지나고 확정 경로가 그 자리에서 전역경로와 `obstacle_release_offset_m`(0.5 m) 안일 때다. 복귀점을 기다리지 않는다. 회피 경로 자체는 복귀점까지 유지한다.
- 회피 경로는 열을 보고 경로를 찾으면 한 번 확정한다. 새 열이 보이거나 기억 객체가 확정 경로의 여유를 침범할 때만 다시 계획한다. 인지가 끊겨도 확정 경로를 같은 상한으로 따른다. 열이 있는데 경로가 하나도 없을 때만 `1.6 m/s`로 이어 간다.
- 열의 앞면이 전이 길이(12 m) 안에 들어오면 그 열의 통과 방향을 고정한다. 이후 다시 계획해도 같은 쪽으로 지난다.
- 다시 계획한 경로는 차 위치에서 확정 경로의 오프셋, 기울기, 곡률로 시작한다. 차가 확정 경로에서 0.5 m 안이면 오프셋을 그대로 잇는다.
- 로컬 경로(장애물 Stage에서 45 m)는 전역경로보다 먼저 끝난다. 객체는 그 뒤로 복귀 길이(12 m)까지 로컬 경로 안에 들어온 뒤부터 계획에 넣는다. 복귀가 로컬 경로 끝에서 잘린 경로를 확정하지 않기 위해서다. 전역경로 끝이 로컬 경로 안에 있으면 그 끝에서 복귀한다.
- 중앙선 쪽은 어느 열이든 차선 한계보다 0.5 m 더 나갈 수 있고 연석 쪽은 0이다. 각 열은 열린 쪽으로만 지난다(D13).
- 다각형이 없는 코스(수집 경로 Case `4`)는 경로 통로(`obstacle_route_half_width_m`, 명목 폭 4.0 m) 기준으로 선택 경로 전체에서 계획하고 구간 상한은 없다.
- 시작할 때 구간 끝 + 복귀 길이가 결승 제동점보다 앞에 있는지 확인하고, 아니면 오류를 내고 시작하지 않는다(D18). 결승 제동점은 결승 정지점 기계의 제동 규칙을 구간 상한 속도에 적용해 구한다. K-City 본선은 구간 끝 669.8 m + 복귀 12 m = 681.8 m로 제동점 733.5 m(측량한 결승선을 안테나가 2 m 지난 곳)보다 앞이다.

## TUI 명령 경로

```text
Decision + Motion Control -> /decision/raw_command
TUI 출발·AUTO gate         -> /planning/command
command_governor           -> /vehicle/command
```

- TUI가 `/planning/command`의 유일한 발행자여야 한다. 
- CTE·헤딩·조향오차는 기록하지만 자동 중단 조건으로 쓰지 않는다. 설정은 Run 전에만 바꿀 수 있다.
- 시험 Case는 `1` 미션·`2` 추종·`3` 수동 수집·`4` 수집 경로 회피다.
- Case `3`은 차량 명령 없이 기록·경로 변환만 한다.
- Gazebo TUI는 `--sim-time`으로 Case `1`만 실행한다.
- `decision.launch.py` 운용 화면은 예선·본선, 속도, 출발 모드만 설정하며 녹화하지 않는다.
- 운용·시험 화면은 준비 검사·신호 현시 추적·명령 gate를 공유한다.
- 완료는 `/planning/behavior_decision`의 `FINISH_HELD`로 정한다(D11). MCAP 기록기가 죽어도 경고만 한다.

## 주요 입력과 실패 동작

| 입력 | 읽는 곳 | 누락·stale |
|---|---|---|
| `/planning/route_context` | 판단 입력 상태 | 0.5초 뒤 속도 0, 1초 더 끊기면 정지까지 제동. 장애물 구간이 있는 코스의 장애물 Stage에서는 1.0초까지 그대로 쓴다 |
| `/vehicle/state` | 판단 입력 상태 | 0.25초 뒤 속도 0, 1초 더 끊기면 정지까지 제동 |
| `/perception/traffic_lights` | 신호 Event의 자기 신호등 | 0.35초 넘으면 UNKNOWN |
| `/planning/traffic_signal_phases` | 신호 Event의 현시표 입력, 정지선 넘어 선 채 UNKNOWN일 때 | 0.5초 넘으면 쓰지 않고 카메라로 판단 |
| `/perception/objects` | 장애물 세계 | 0.5초 넘으면 새 계획 없이 확정 경로 유지 |
| `/planning/target_path`, `/planning/target_speed` | Motion Control | 0.35초 뒤 속도 0, 1초 더 끊기면 정지까지 제동 |
| `/decision/raw_command`, `/planning/scene_summary` | TUI | 0.25초, 0.5초 뒤 속도 0, 1초 더 끊기면 정지까지 제동 |

- 끊김 규칙은 Decision, Motion Control, TUI가 같다(D2). 끊기는 순간 브레이크 없이 속도 0을 내고, 1초 안에 돌아오지 않으면 브레이크를 요청한다. 그 요청은 입력이 돌아와도 실제 정지까지 유지하고, 정지 뒤 돌아온 입력으로 풀린다. 끊길 때 이미 걸린 브레이크는 끊긴 동안 풀지 않고, 1초 안에 입력이 돌아오면 돌아온 명령을 따른다. 세 곳 모두 `kaiev26_motion_control.input_loss.InputLossLatch`를 쓴다.
- 장애물 Stage에서 경로 입력이 0.5~1.0초 묵으면 끊김 규칙 대신 회피를 이어 가고 `ROUTE_CONTEXT_AGE_TOLERATED`와 경고(1초 간격)를 남긴다(`obstacle_route_stale_sec`). 경로 입력 나이는 `/debug/route_context_age`(Float32, 초, 미수신 -1)로 낸다.
- 인지 끊김은 판단의 입력 상태가 아니다(D3). 신호는 UNKNOWN으로 보고, 장애물 행동은 확정 경로를 유지하며, 차는 지도 경로를 계속 따른다.
- 입력이 끊긴 동안 경로는 직전 경로를 낸다. 직전 경로는 차량 기준 좌표라, 저장 시각과 지금의 odometry 자세로 현재 위치에 다시 옮겨 지금 시각을 붙인다. 두 자세가 없으면 옛 시각 그대로 내 하류의 경로 나이 검사가 보게 한다(hjh 10/23).
- 충돌 없는 후보가 없으면 강제 S자 후보를 사용할 수 있으며, 이는 무충돌을 보장하지 않는다.

## 기준 파일

| 파일 | 내용 |
|---|---|
| `waypoints/kcity_*_route.yaml` | 전역경로와 곡선 속도 구간 |
| `config/*_landmarks.yaml` | 이벤트 정지선과 신호등 id. 위치는 `map_semantics`가 가리키는 지도 semantics에서 읽는다 |
| `config/*_policy.yaml` | Stage·Event·제동 waypoint, 본선 장애물 구간 |
| `config/decision_pipeline.yaml` | 판단·회피·AEB 설정 |
| `kaiev26_motion_control/config/motion_control.yaml` | 추종·가감속·제동 설정 |

- 경로, landmark, policy는 같은 코스를 설명한다. 
- 하나를 바꾸면 waypoint 번호, `brake_trigger_route_s`, 신호 ID와 Zone 경계를 함께 확인한다.

## 메시지 계약

- 메시지 필드는 바꾸지 않는다(D1). `TargetSpeed`의 숫자 규칙이 Motion Control과의 계약이다.
- `/planning/traffic_signal_phases`의 발행 규격은 `kaiev26_msgs/TrafficSignalPhaseArray`다. main_planning_engine은 신호 Event의 자기 신호등 추정만 읽는다.
- `/vehicle/state`는 `kaiev26_msgs/VehicleState`, 명령은 `ActuatorCommand`를 사용한다.
- 메시지 변경 후 `kaiev26_msgs`, Perception, Decision, Control 소비 패키지를 함께 빌드한다.
- 실차에서는 Decision과 Control이 system time을 쓰며 `pv26_perception`도
  `use_sim_time:=false`로 실행한다.
