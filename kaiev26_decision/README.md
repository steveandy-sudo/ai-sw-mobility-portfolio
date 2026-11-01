# kaiev26_decision

전역경로 정합, Event 정지점 기계와 장애물 행동을 실행해 목표 경로·속도를 만드는 ROS 2 패키지다.
최종 차량 명령은 [`kaiev26_motion_control`](../kaiev26_motion_control/README.md)이 만든다.

## 구성

```text
config/                    판단·코스 설정
launch/                    경기, 시험 backend, RViz launch
waypoints/                 예선·본선 전역경로
kaiev26_decision/
  perception_gateway_node.py    인지 동기화와 SceneSummary(TUI, 모니터용)
  route_zone_manager_node.py    진행도·Stage·로컬 경로
  main_planning_engine_node.py  입력 상태, 모듈 호출, 중재, 발행
  stop_point.py                  신호·시간 정지·결승 정지점 기계와 Event 추적기
  signal_schedule.py             현시표 신호 입력, 카메라 거부
  event_permission.py            신호 허용, 3초 정지, UNKNOWN 탈출과 110초 대기
  mission_events.py              코스 Policy와 모드로 Event 목록 만들기
  arbiter.py                     출력 소유 표와 중재기
  obstacle_world.py              추적 객체 기억, 장애물 구간 다각형, 지도 차선 경계
  obstacle_behavior.py           구간 속도 상한, 회피 경로 확정과 유지
  obstacle_sequence.py           회피 후보 생성과 선택
  obstacle_geometry.py           회피 기하 계산
  obstacle_observation.py        차량 자세 기록, 직전 경로를 지금 자세로 옮기기
  signal_phase_tracker.py        지도 신호 프로그램의 주기 위치 추정
  signal_phase_tracker_node.py   모든 신호등의 현시 추정 발행(실차 UTC 시작값은 GNSS 시계 점검 뒤)
  test_tui.py                    실차 TUI·명령 gate·MCAP
  operating_tui.py               decision.launch.py 운용 화면(녹화 없음)
```

각 모듈은 속도 상한, 정지 요청, 경로 요청만 내고 중재기가 합친다. 누가 무엇을 쓸 수 있는지는
`arbiter.OUTPUT_OWNERSHIP`가 정한다. 구조는 [Architecture](../docs/ARCHITECTURE.md#main_planning_engine-구조)에 있다.

## 데이터 흐름

```text
Odometry         -> /planning/route_context ─┐
/vehicle/state                               ├-> main_planning_engine
/perception/objects, /perception/traffic_lights ┤
/planning/traffic_signal_phases              ─┘
  -> /planning/target_path + /planning/target_speed + /planning/behavior_decision
  -> Motion Control

pv26_perception -> perception_gateway -> /planning/scene_summary (TUI, 모니터)

/perception/traffic_lights
  -> signal_phase_tracker_node -> /planning/traffic_signal_phases
```

신호 현시 추정의 파라미터와 토픽은 [Parameter Book](../docs/PARAMETER_BOOK.md#신호-현시-추정)에 있다.

경기 인지는 다음 네 토픽을 같은 시각으로 발행한다.

```text
/perception/road_segments
/perception/centerline
/perception/objects
/perception/traffic_lights
```

현재 `pv26_perception`이 이 계약을 제공한다. 실차에서는 해당 launch를
`use_sim_time:=false`로 실행한다. `cone_perception` 출력은 제동 검차 전용이다.

## 실행

시나리오 시험 TUI:

```bash
ros2 run kaiev26_decision decision_test
```

시험 TUI는 준비 검사·수동 헤딩 정렬·AUTO 전환을 거쳐 실행하며 MCAP을 기록한다.

Gazebo에서는 `--sim-time`을 붙인다. TUI와 판단 노드가 `/clock`을 쓰고, launch는
`platform:=simulation`으로 실행된다. 이 경로는 route_readiness와 NTRIP 확인을 쓰지 않고
경로 정합 수신을 출발 조건으로 삼는다. TUI에서는 Case `1` 예선·본선 미션 실행만 지원하며,
내부 launch Case `3`, `6`, `7`은 TUI 번호와 구분한다.

```bash
ros2 run kaiev26_decision decision_test --sim-time
```

실차 경기·운용(녹화 없음):

```bash
ros2 launch kaiev26_decision decision.launch.py
```

기존 TUI와 같은 상태·출발 검사 화면을 사용한다. `P`로 예선·본선, `V`로 속도, `B`로 출발 모드를
정하고 `R`로 시작한다. 경로 변경 시 예선 `7 m/s`, 본선 `10 m/s`
기본속도로 돌아간다.
미션은 ALL·PP+Stanley 고정이며 MCAP·실행 기록 폴더를 생성하지 않는다.
AUTO 3초 대기, 명령 gate, 신호 현시 추적과 종료 처리는 미션 TUI와 같다.
Control·Localization·센서·인지는 별도 실행하며 실차 system time을 사용한다. 자세한 실차 순서는
[Runbook](../docs/DECISION_RUNBOOK.md)을 따른다.

## TUI Case

| TUI Case | 기능 | 내부 launch Case |
|---:|---|---|
| `1` | 미션 실행: 예선/본선 ALL | `3 / 7` |
| `2` | 전역경로 추종 | 예선 `1`, 본선 `4`, 수집 경로 `13` |
| `3` | 수동주행 경로 수집·등록 | launch 없이 MCAP 수집·변환 |
| `4` | 장애물 회피: 수집 경로에서만 표시 | `17` |

Case `1·2·4`는 같은 Main Planning Engine과 PP+Stanley를 사용하고 활성 미션만 다르다.
일반 오차 기반 자동 REJOIN은 없으며 장애물 회피 완료 뒤에만 REJOIN한다.
신호 재출발은 허용 신호 0.4초 연속 확인과 실제 정지가 필요하다. 정지 뒤 출발은 정지선까지
거리와 관계없이 그때 `COMMITTED`를 확정하고, 달리는 중의 허용 신호 확정은 8 m 안에서만 한다.

`P`로 경로를 선택하고 `V`로 목표속도를 입력한다. `B`로 출발 모드(`처음부터`/`이어서`)를 고른다.
Case `3`은 `N`으로 경로 이름을 입력한다. 예선·본선에는 회피 전용 Case를 표시하지 않는다.
수집 경로에는 미션 Policy가 없으므로 Case `1`은 실행하지 않는다.

## 장애물 회피

경기에서는 본선 `F_STAGE_6`에서만 활성화된다. 수집 경로의 Case `4`는 경로 전체에 적용한다.
수집 경로에서는 추종기와 목표속도만 정한다. 목표속도로 경로를 따라가다(곡선은 자동 곡선 상한) 인지가 드럼을 보면
장애물 FSM으로 넘어가 `3.0 m/s`로 줄이고 회피한 뒤 경로로 돌아와 다시 목표속도로 가속한다.
진입·회피·복귀·재가속을 끝까지 보도록 결승 구간(`finish_zone_distance_m`)을 `0 m`로 둔다. 경로 끝 `20 m`
앞에서 미리 서지 않고, 후륜축이 경로 끝에 닿을 때 브레이크를 건다. 앞범퍼 기준 정지는 경로 끝 뒤
약 `2 m + d_stop(v)`(3 m/s 약 5 m, 7 m/s 약 13 m)이므로 그만큼 공간을 두거나 운용자가 먼저 MANUAL로 받는다.
드럼 위치를 미리 모르므로 본선의 구간 앞 사전 감속은 없다. 드럼을 본 뒤(약 `25 m` 전) 감속을 시작한다.

1. 객체를 전역경로 `(s,d)`에 투영한다.
2. 비슷한 `s`의 객체를 세트별 점유 범위로 묶고 최근 7프레임 중앙값으로 안정화한다.
3. 첫 세트의 빈 통로, 두 번째 세트의 빈 통로와 복귀점을 연결한다.
4. 차체 외곽, 장애물, 차선, 조향 한계를 검사해 경로를 고른다.
5. 본선은 구간 앞에서 `0.5 m/s²`로 미리 낮춰 앞범퍼가 구간에 들어갈 때 `3.0 m/s`가 되게 한다. 곡률·조향률 제한이나 fallback은 더 낮은 목표를 만들 수 있다.
6. 구간 끝을 지나고 후륜축이 마지막 세트를 `3 m` 지났으며 회피 경로가 전역경로와 `0.5 m` 안이면 구간 상한을 푼다. 회피 경로는 복귀점까지 따른다.

객체는 `0.8 s`, 확인된 차선 경계는 최대 이동거리 `10 m`까지 기억한다. 장애물 간격과
좌우 순서는 고정하지 않지만, 앞면이 `12 m` 안에 든 세트의 통과 방향은 고정한다. 다시 계획한 경로는
확정 경로의 오프셋·기울기·곡률에서 시작한다. 여유가 `0.50 m`까지 가장 큰 경로를 고르며, 정상 후보가 없으면
`1.6 m/s` 강제 S자 또는 직전·전역경로 fallback을 쓴다. 이 fallback은 무충돌을 보장하지 않지만
새 정지 조건을 만들지 않는다. 물리 E-Stop과 MANUAL은 항상 우선한다.

비정지 회피 목표의 명령 감속 상한은 `10.0 m/s²`다. 일반 추종·코너의 `2.0 m/s²`와 구분한다.

수집 경로 Case `4`(미단 회피)는 실제 인지 차선을 제외하고, `midan_test_policy.yaml`의 고정 차선 구간
(`lane_offsets_m`: 오른쪽 `-1.96 m`, 왼쪽 `+2.14 m`, 본선 드럼 위치의 차선)으로 본선과 같은 차선 탐색을 쓴다.

## 기준 파일

| 파일 | 역할 |
|---|---|
| `config/decision_pipeline.yaml` | 입력, 정지, 회피, AEB |
| `config/*_landmarks.yaml` | 이벤트 정지선과 신호등 id. 위치는 지도 semantics에서 읽는다 |
| `config/*_policy.yaml` | Stage·Event·제동 waypoint |
| `waypoints/kcity_*_route.yaml` | 전역경로·곡선 속도 구간 |

## 규칙

- `/localization/odometry`와 `/planning/command` 발행자를 중복 실행하지 않는다.
- 실제 인지와 mock 인지를 동시에 실행하지 않는다.
- 신호·ALL TUI는 최신 `SceneSummary`와 `perception_health=GOOD`을 요구한다. 회피 전용은
  같은 frame의 최신 `/perception/objects`와 유효한 Route 정합으로도 출발할 수 있다.
- 조향오차, CTE와 헤딩오차만으로 Run을 중단하지 않는다.
- 코스 파일을 바꾸면 route, landmark, policy의 정합을 함께 검사한다.

## 테스트

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select kaiev26_decision
colcon test-result --test-result-base build/kaiev26_decision
```

### 시나리오 시험

`test/scenarios/*.yaml` 파일 하나가 상황 하나와 그 결과를 적는다. 시험기는 실제 노드
(route_zone_manager, perception_gateway, main_planning_engine, motion_control, 필요하면
signal_phase_tracker)를 `decision_test.launch.py`의 경기 Case(예선 3, 본선 7)와 같은 파라미터로
한 프로세스에 띄우고 토픽으로만 주고받는다. 차량은 명령을 따라가는 운동학 자전거 모델이고,
인지는 지도 정답으로 만든 pv26 메시지다. `/clock` 시뮬레이션 시간으로 돌기 때문에 결과가 매번
같고 실제 시간보다 빠르다.

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
cd src/Decision
python3 -m pytest -m scenario kaiev26_decision/test/test_scenarios.py
# 일부만 돌릴 때: -k "signal_red_stop and F2"
# 시나리오별 결과 파일과 요약표: --scenario-report <디렉터리>
```

- `-m scenario`를 주지 않으면 시나리오는 건너뛴다. 평소 단위 시험 시간은 늘지 않는다.
- 시나리오마다 프로세스를 따로 띄워 여러 개를 동시에 돌린다(`--scenario-workers`, 기본은
  CPU 수에서 2를 뺀 값, 최대 12). 각 프로세스는 아무도 띄우지 않은 Fast DDS discovery server의
  클라이언트로 띄우므로 다른 ROS 프로세스와 서로 보이지 않는다.
- 16스레드 PC에서 전체 목록이 6~7분 걸린다.

시나리오를 더할 때는 yaml 파일만 추가한다. Python은 고치지 않는다. 이름이 `_`로 시작하는
파일은 읽지 않는다.

- `course`, `speed_mps`(launch 속도), `event`(정책 이벤트 id), `start`(`distance_before_line_m`
  앞범퍼 기준 또는 `route_s`, `speed_mps`), `duration_s`, `vehicle`(예: `brake_decel_mps2`),
  `perception`(센서 범위), `launch`(추가 launch 인자)를 적는다.
- `world`에는 신호 시간표(`event_lights`, `other_lights`, `lights`), 토픽 끊김(`dropouts`),
  드럼 줄(`obstacle_rows`), 신호 현시 추정 시간표(`event_phases`)를 적는다. `event_phases`의
  단계는 `{aligned: true|false, permitted: true|false}`나 `unpublished: true`이고, 시험기가
  추적기 노드를 빼고 시나리오 Event의 자기 신호등 추정을 대신 발행한다. 판정의 시각으로 `{phase: event, step: k}`를
  쓸 수 있다. 추정을 끊으려면 `dropouts`에 `signal_phases`를 적는다. 시간표 단계는 `at_s`, `after_s`, `stopped_for_s`,
  `moving_for_s`, `front_gap_le`, `route_s_ge` 가운데 하나를 조건으로 갖는다.
- `expect`에는 `test/scenario_harness/checks.py`의 `CHECKS`에 있는 판정만 쓴다. 새 판정이
  꼭 필요하면 `checks.py`에 한 번 구현하고 모든 시나리오가 함께 쓴다.
- `matrix`의 목록은 모든 조합으로 펼쳐지고 `${ev.id}`처럼 값을 넣는다.
- 기대값은 지금 코드의 동작이 아니라 정해진 원하는 동작으로 적는다. 시험이 실패해도 기대값을
  낮추지 않는다.
