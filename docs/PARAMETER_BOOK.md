# Decision Parameter Book

현장에서 확인하거나 조정하는 값만 정리한다. 실제 기준은 각 YAML과 launch 파일이다.

## 기준 파일

| 파일 | 범위 |
|---|---|
| `kaiev26_decision/config/decision_pipeline.yaml` | 입력, Route, 미션, 회피, AEB |
| `kaiev26_decision/config/*_policy.yaml` | Stage, Event, 제동 waypoint |
| `kaiev26_decision/config/*_landmarks.yaml` | 이벤트 정지선과 신호등 id. 위치는 `map_semantics`가 가리키는 지도 semantics에서 읽는다 |
| `kaiev26_decision/waypoints/kcity_*_route.yaml` | 전역경로와 곡선 속도 구간 |
| `kaiev26_motion_control/config/motion_control.yaml` | 횡제어, 가감속, 제동 |
| `kaiev26_aeb/config/{perception,tracking}.yaml` | 라바콘 제동 검차 |

단위는 거리 `m`, 속도 `m/s`, 가속도 `m/s²`, 각도 `rad`, 시간 `s`다.

## 실행 기본값

| 실행 | 속도 | 횡제어기 |
|---|---:|---|
| 실차 TUI 예선 | `7.0 m/s` | PP+Stanley 고정 |
| 실차 TUI 본선 | `10.0 m/s` | PP+Stanley 고정 |
| 수집 경로 추종·회피 | `10.0 m/s` | PP+Stanley 고정 |
| `decision.launch.py` 운용 화면 | 예선 `7.0`, 본선 `10.0 m/s` | ALL·PP+Stanley 고정, 녹화 없음 |

## Perception·Route

| 파라미터 | 값 | 의미 |
|---|---:|---|
| `publish_rate_hz` | `30.0` | SceneSummary 주기 |
| `stale_data_sec` | `0.35` | 인지 stale 기준 |
| Route `publish_rate_hz` | `20.0` | RouteContext 주기 |
| `lookahead_distance_m` | `18.0` | 일반 로컬 경로 길이 |
| `obstacle_lookahead_distance_m` | `45.0` | 장애물 Stage 로컬 경로 길이. 2026-10-23 현장 임시 기준값(hjh), 기록된 여유·복귀 길이 검토 후 조정 |
| `local_path_spacing_m` | `1.0` | 로컬 경로 간격 |
| `max_projection_distance_m` | `15.0` | 경로 정합 최대 거리 |
| `pose_freeze_sec` | `0.5` | 차량 자세 스탬프가 이 시간 동안 바뀌지 않으면 측위가 멈춘 것으로 보고 RouteContext 발행을 멈춘다. 그러면 판단, motion_control, TUI가 끊김 규칙(속도 0, 1 s 뒤 브레이크)을 따른다. 스탬프끼리만 비교하므로 시계가 어긋나도 잘못 걸리지 않는다 |
| `progress_search_window_m` | `35.0` | 진행점 주변 검색 거리 |
| `start_mode` | `resume`, TUI 기본 `from_start` | 출발 모드: `from_start` 처음부터, `resume` 이어서 |
| `finish_zone_distance_m` | `30.0` | Route의 Finish Zone 거리(표시용). 약 `13.5 m/s`까지의 종료 제동점을 포함한다 |

| 코스 | waypoint | 길이 | 시작·종료 |
|---|---:|---:|---|
| 예선 | `597` | `596.13 m` | 공통 START, 종료는 공통 FINISH + `45 m` |
| 본선 | `774` | `772.39 m` | 공통 START, 종료는 공통 FINISH + `45 m` |

공통 START는 `(-66.689, -130.618)`, FINISH는 `(-71.284, -128.294)`다. 본선은 결승선 바로 앞에서 서던 문제로 2026-10-24에 같은 직선을 따라 `15 m`(WP 729~743) 늘렸고, 12~13 m/s에서 결승 제동이 여전히 결승선 앞에서 시작돼 2026-10-25에 `10 m`(WP 744~753), 측량한 결승선 뒤에서 제동하게 되면서 `20 m`(WP 754~773)를 더 늘려 `(-92.259, -168.106)`에서 끝난다. 예선은 같은 날 본선 WP 729~773과 같은 점으로 `45 m`(WP 552~596) 늘려 본선과 같은 점에서 끝난다(`straightening.finish_extension`).

실제 출발·결승선은 2026-10-25에 GNSS 안테나를 선 위에 두고 측량했다(RTK FIX, `records/start_line` 출발 차로 `37.238943406, 126.772968818`, `records/final_line` 결승 차로 `37.238965823, 126.772923934`). 두 점을 이은 선은 결승 직선과 `85.7°`로 만나고, 경로와의 교점은 본선 `s=731.93 m`, 예선 `s=555.67 m`다(옛 공통 FINISH보다 `4.54 m` 뒤). 출발 차로에서는 경로 시작점(`s=0`)보다 `5.02 m` 앞이다.

- `from_start`: 진행도 확정 전까지 경로 `s=0`부터 위치만으로 정합하고 헤딩은 쓰지 않는다.
- `from_start`의 검색 끝은 `start_snap_progress_m` 또는 직전 진행도 + 검색 거리 중 큰 값이다.
- `from_start`에서는 헤딩을 준비 검사가 따로 확인하고, 경로 중간 출발을 허용하지 않는다.
- `resume`: 경로 전체에서 헤딩을 함께 써서 정합한다(10/17 이전 동작).
- 10/17 08:50~08:57에 출발 헤딩이 약 180° 뒤집혀 5 m 옆 종점 구간에 정합된 뒤 풀리지 않았다.

- 게이트웨이는 장애물을 고르지 않는다(2026-10-16 결정 D16). 판단 노드가 `/perception/objects`를 직접 받는다.

## 판단 입력 상태

| 파라미터 | 값 | 의미 |
|---|---:|---|
| `planning_rate_hz` | `30.0` | 판단 틱 주기 |
| `route_stale_sec` | `0.5` | RouteContext가 이보다 오래되면 입력 끊김 |
| `obstacle_route_stale_sec` | `1.0` | 장애물 구간이 있는 코스의 장애물 Stage에서만 쓰는 같은 기준 |
| `debug_route_context_age_topic` | `/debug/route_context_age` | 매 판단 주기의 경로 입력 나이 [s], 미수신 `-1` |
| `vehicle_stale_sec` | `0.25` | VehicleState가 이보다 오래되면 입력 끊김 |
| `input_loss_brake_delay_sec` | `1.0` | 끊김이 이만큼 이어지면 실제 정지까지 브레이크(결정 D2) |

- 입력 끊김은 RouteContext(정합 포함)와 VehicleState만 본다. 인지는 보지 않는다(결정 D3).
- 나이는 메시지 stamp와 틱 시각의 차이다. 두 토픽은 큐 깊이 1로 받아 틱이 길어져도 밀린 옛 메시지를 읽지 않는다. 깊이 10이면 50 ms 주기 입력이 쌓여 틱마다 최대 0.5 s 묵은 입력을 읽고 끊김으로 본다(10/17 14:57 A'의 "route context stale" 55회, 토픽 자체는 끊김 없음).
- 장애물 Stage에서 `0.5~1.0 s`이면 입력 끊김 대신 회피를 이어 가고 `ROUTE_CONTEXT_AGE_TOLERATED`와 경고를 남긴다. `1.0 s`를 넘으면 다른 구간과 같다.
- 10/17 145703·114251 실차에서 회피 탐색 주기의 경로 나이가 `0.3~0.505 s`였고, `0.5 s`를 넘는 주기마다 입력 끊김으로 차가 섰다.
- 끊기는 순간 브레이크 없이 속도 0, 1초 뒤 브레이크를 요청하고 실제속도 `static_stop_stopped_speed_mps` 이하까지 유지한다. 이미 걸린 정지 요청은 끊긴 동안 풀지 않는다.

## TUI 준비 검사

| 파라미터 | 값 |
|---|---:|
| 입력 freshness | `1.0 s` |
| 조건 안정 시간 | `1.0 s` |
| 위치 variance X·Y | 각각 `<= 0.25 m²` |
| 속도 variance | `<= 1.0 m²/s²` |
| GNSS 헤딩 초기화 속도 | `>= 0.5 m/s` 1회 |
| AUTO 전환 전 속도 | `<= 0.2 m/s` |
| 출발 헤딩오차 | `maximum_start_heading_error_deg=45.0` 이하 |
| 출발 횡오차 | `maximum_start_cross_track_m=3.0` 이하 |
| 판단 노드 경로 입력 나이 | `maximum_start_route_context_age_sec=0.15` 이하 (`/debug/route_context_age`) |

- GNSS는 RTK FIX `status=2`여야 한다. 
- 판단 노드가 실제로 받는 경로 입력 나이(`/debug/route_context_age`)가 출발 전 `0.15 s`를 넘으면 READY를 주지 않는다. 10/23~10/24 실차에서 런치 4번에 1번꼴로 토픽은 20 Hz로 정상인데 판단 노드만 처음부터 `0.25~0.85 s` 늦게 받아 `ROUTE_CONTEXT_STALE`로 속도 0을 냈다(10/24 10:10 본선, 10/23 0944·1004·1005 등). 정상 런치는 `0.05 s` 이하다. 런치 전체가 이 상태로 굳으므로 TUI에 `판단 경로 입력` 지연이 보이면 Decision을 다시 실행한다. 출발 뒤에는 기록만 한다.
- 출발 전 CTE·헤딩오차는 위 허용 범위를 확인한다. 출발 뒤 CTE·헤딩오차와 조향오차는 기록만 하고 주행을 차단하지 않는다.
- TUI `B` 키로 출발 모드를 고른다: `처음부터`(기본, `from_start`) / `이어서`(`resume`).
- `처음부터`는 진행거리 `35 m` 이내 출발만 허용하고, `이어서`는 경로 중간 출발을 허용한다.
- 시험 TUI와 경기 TUI 모두 주행 중 인지 상태가 `GOOD`가 아니어도 run을 끝내지 않고 속도와 제동도 바꾸지 않는다(2026-10-16 결정 D3). 화면에 경고를 띄우고 상태가 바뀐 시각을 로그와 `scenario.yaml`의 `perception_health_changes`에 남긴다. 출발 전 점검에서는 여전히 `GOOD`을 요구한다.
- 주행 중 입력 끊김은 run을 끝내지 않는다(2026-10-16 결정 D2). 두 TUI 모두 SceneSummary가 `SCENE_RECEIVE_TIMEOUT_SEC=0.5 s` 넘게 오지 않거나 경로 준비가 풀릴 때, 판단 명령이 `RAW_COMMAND_TIMEOUT_SEC=0.25 s` 넘게 오지 않을 때를 끊김으로 본다. 장애물 허용 구간(본선 `F_STAGE_6`, 수집 경로 Case `4`)은 유효 경로가 있으면 인지 저하에도 중계를 유지하고, 이때도 판단 명령 freshness는 확인한다.
- 끊기면 곧바로 브레이크 없이 속도 0과 측정 조향각을 보낸다. `INPUT_LOSS_BRAKE_DELAY_SEC=1.0 s` 안에 돌아오지 않으면 브레이크를 걸고, 입력이 돌아와도 `0.1 m/s` 이하로 설 때까지 유지한다. 정지한 뒤 입력이 돌아오면 판단 명령 중계를 다시 시작한다. 끊길 때 판단 명령이 이미 브레이크를 걸고 있었으면 끊긴 동안 풀지 않는다.
- MCAP recorder가 죽어도 경고만 남기고 run을 이어 간다. Decision launch가 죽으면 지금처럼 run을 끝낸다.
- 인지 상태 변화 로그는 `PERCEPTION_HEALTH_LOG_INTERVAL_SEC=1.0 s`보다 자주 찍지 않고, 그 사이 변화 횟수를 다음 줄에 붙인다.

## 정지·신호

| 파라미터 | 값 | 의미 |
|---|---:|---|
| Policy `approach_distance_m` | `30.0`, F5만 `40.0` | 정지선 기준 Event 선택 시작 거리. F5는 여유 7.5 m라 10 m/s 제동점이 29.3 m다 |
| `pre_brake_decel_distance_m` | `8.0` | 제동 WP 전 감속 요청 시작 거리 |
| `brake_entry_speed_mps` | `2.0` | 사전 감속 구간에서 즉시 요청할 목표속도 |
| `static_stop_stopped_speed_mps` | `0.1` | 실제 정지 판정 |
| `test_stop_hold_sec` | `3.0` | TIMED_STOP 유지 |
| `traffic_release_confirm_sec` | `0.4` | 제동 중을 포함한 진행 허용 신호 연속 확인 |
| Policy `commit_distance_m` | `8.0` | 달리는 중 허용 신호 통과 확정 거리, 정지선 기준. 정지 뒤 출발은 거리와 관계없이 확정 |
| `intersection_escape_unknown_confirm_sec` | `1.0` | 선을 지난 정차 상태에서 자기 신호등 UNKNOWN 탈출. 신호 현시 추정이 정렬돼 있으면 그 추정을 먼저 쓴다 |
| `traffic_lights_stale_sec` | `0.35` | 선을 넘은 뒤 판단 노드가 자기 신호등을 직접 찾을 때 관측의 최대 나이 |
| `signal_phase_stale_sec` | `0.5` | 신호 현시 추정을 쓰는 최대 나이. 정지 중 출발 보류에도 같은 값을 쓴다 |
| `vehicle_front_overhang_m` | `1.98` | 후륜축에서 앞범퍼 거리 |
| `brake_delay_sec` | `0.70` | 정지거리 식의 제동 지연. 10/11 실차 제동 맞춤값 |
| `brake_decel_mps2` | `3.9` | 정지거리 식의 감속도. 10/11 실차 제동 맞춤값 |
| `stopline_brake_margin_m` | `1.0` | 앞범퍼와 정지선 사이가 `d_stop(v)`에 이 값을 더한 거리 안이면 제동. Policy 이벤트에 `brake_margin_m`이 없을 때만 쓴다 |
| Policy `brake_margin_m` | Q `2.75/2.80/2.85/3.20`, F `2.15/1.75/3.25/2.35/7.50 m` | 이벤트별 제동 여유. `0~10 m` 밖이면 Policy를 읽지 않는다 |
| `signal_unknown_after_permission_confirm_sec` | `0.3` | 진행 허용을 확인한 뒤 들어온 UNKNOWN, 화살표 이벤트에서 화살표만 꺼진 신호를 정지 신호로 보기까지의 시간 |
| `signal_unknown_before_permission_grace_sec` | `0.3` | 허용 확인(0.4초) 중 이 시간 이하의 UNKNOWN은 확인을 멈추기만 하고 처음부터 다시 세지 않는다. UNKNOWN 자체는 허용이 아니다. 화살표 꺼짐은 초기화(10/17 145703 F2) |
| `signal_unknown_wait_limit_sec` | `110.0` | 정지선에 선 채 UNKNOWN만 계속되면 이 시간 뒤 출발. `0`이면 끈다 |
| `signal_arrival_margin_sec` | `1.0` | 현시표 허가로 접근할 때 제동점(느린 차는 8 m 확정 거리가 먼저면 그곳)에서 한 번 통과와 정지를 정한다. 앞범퍼가 허가가 끝나기 이 시간 전에 정지선에 닿으면 통과, 아니면 제동점에서 선다. K-City 신호 시계가 UTC 100 s 격자에서 약 1 s 안이고 현시 추정 오차가 ±0.5 s라 1 s로 둔다 |

- 예선·본선은 각 Policy의 `approach_distance_m`을 사용한다. 이웃 Event의 접근 구간이 앞 정지선보다 앞에서 시작하면 판단 노드가 시작하지 않는다.

- 제동점은 기존 WP 중 예선 `204/249/296/340`, 본선 `120/165/302/356/454`다(0-based).
- 예선 정지선→제동 WP는 `14.13/9.99/14.64/11.87 m`다.
- 본선 정지선→제동 WP는 `4.91/14.26/14.53/17.08/21.50 m`다.
- 정지거리는 `d_stop(v) = v * brake_delay_sec + v² / (2 * brake_decel_mps2)`로 계산하고 앞범퍼 기준으로 잰다. `v`는 차량 실측 속도다.
- 신호 정지와 TIMED_STOP은 앞범퍼와 지도 정지선 사이가 `d_stop(v) + 여유` 이하가 되면 제동한다. 여유는 이벤트의 `brake_margin_m`, 없으면 `stopline_brake_margin_m`이다. 제동 WP는 사전 감속 시작점(WP 8 m 전)만 정한다. 차량 속도가 끊기면 입력 끊김 규칙이 정지를 맡는다.
- 속도로 정한 제동 시점은 한 번 닿으면 유지한다. 감속으로 `d_stop(v)`가 줄어도 브레이크를 풀지 않는다.
- 고정 제동 WP에서 지도상 앞범퍼 여유는 예선 약 `7.83~12.45 m`, 본선 약 `2.74~19.34 m`다. 정지 완료 후 여유가 아니다.
- 정지선의 `route_s`는 지도 semantics 정지선과 전역경로의 교차점이다. route_zone_manager가 기동할 때 계산하고, landmark 파일에는 좌표를 두지 않는다.
- 감속 8 m는 제동 WP까지, 신호 통과 확정 8 m는 지도상 정지선까지의 거리다.
- 사전 감속 구간에서는 목표 2.0 m/s를 즉시 요청한다.
- 최종 속도명령은 `stop_pre_decel_mps2=10.0` 상한으로 감소한다. 
- 현시표가 허가하는 중에 제동점이나 8 m 확정 거리에 처음 닿으면 그 틱에 통과와 정지를 정하고 다시 바꾸지 않는다. 통과면 8 m 밖이어도 그때 `COMMITTED`(`SIGNAL_ARRIVAL_PASS`), 정지면 적색처럼 제동점에서 제동한다(`SIGNAL_ARRIVAL_STOP`). 허가 끝 시각을 모르면(현시표 없음, 불신, 카메라 전용) 전처럼 8 m 확정 규칙을 따른다.
- 정지 후 허용 신호로 출발하면 8 m 밖이어도 그때 `COMMITTED`를 확정하고 다시 제동하지 않는다. `COMMITTED`와 UNKNOWN 탈출 조건은 [Mission Scenarios](MISSION_SCENARIOS.md#신호-판단)를 따른다.
- UNKNOWN 대기 한도를 넘겨 출발하면 경고 로그와 `/diagnostics`(`signal_diagnostics_topic`)에 Event, 정지선, 대기 시간을 남긴다.
- 신호 현시 추정이 신호 정지 중 출발을 막기 시작하면 경고 로그와 `/diagnostics`(`signal_diagnostics_topic`)에 Event와 정지선을 남긴다.

## 신호 현시 추정

`signal_phase_tracker_node`(노드 이름 `kaiev26_signal_phase_tracker`)는 지도의 신호 프로그램과 PV26 관측으로 지도에 있는 모든 신호등의 현재 등화와 남은 시간을 추정해 발행한다. 판단 노드는 신호 Event 자기 신호등의 정렬된 최신 추정(현시표)을 신호 입력으로 쓰고, 카메라는 현시표를 거부할 때만 쓴다(`signal_schedule.py`, 아래 [현시표 판단](#현시표-판단)). 정지선을 넘어 선 채 자기 신호등이 UNKNOWN일 때도 그 추정을 쓴다([Mission Scenarios](MISSION_SCENARIOS.md#신호-판단)). `decision_test.launch.py`는 예선과 본선 실행에서 이 노드를 함께 띄우고(`decision.launch.py` 운용 화면 포함), `enable_signal_phase_tracker:=false`면 띄우지 않는다.

| 파라미터 | 값 | 의미 |
|---|---:|---|
| `publish_rate_hz` | `10.0` | 추정 발행 주기 |
| `phases_topic` | `/planning/traffic_signal_phases` | `kaiev26_msgs/TrafficSignalPhaseArray` |
| `marker_topic` | `/planning/traffic_signal_phases/markers` | RViz `MarkerArray`. 추정 등화 색의 구, 켜진 화살표, 남은 시간 글자 |
| `evidence_topic` | `/planning/traffic_signal_phases/evidence` | 신호 그룹별 주기 위치, 색 전환 시각, 측정한 등화 길이를 담은 JSON. `evidence_publish_period_sec=1.0`마다 |
| `map_semantics_path` | `""` | 비우면 `kaiev26_maps` 설치본의 `maps/kcity/kcity_course_semantics.yaml` |
| `alignment_bin_sec` | `0.1` | 주기 위치를 나누는 간격 |
| `credible_mass` | `0.9` | 남은 시간 구간과 주기 위치 구간이 담는 확률 |
| `aligned_interval_sec` | `2.0` | 주기 위치 구간의 폭이 이 값 이하면 `aligned` |
| `observation_correlation_sec` | `0.5` | 같은 신호등의 프레임은 이 시간 안에서 합쳐 관측 1회로 센다 |
| `circle_error_probability` | `0.04` | 원형 등화 색을 잘못 읽을 확률 |
| `forgetting_time_constant_sec` | `180.0` | 오래된 관측의 비중을 줄이는 시정수. 그 신호 그룹을 관측하고 있던 시간만 센다 |
| `max_log_odds` | `40.0` | 가장 유력한 주기 위치에 견준 로그 우도의 하한. 잘못 맞춘 위치에서 다시 빠져나올 수 있게 한다 |
| `coupling` | `controller` | 지도 `controller_id`가 같은 신호등은 주기 위치 하나를 함께 추정한다. `program`은 신호 프로그램마다, `all`은 지도 전체를 하나로 본다 |
| `edge_stable_sec`, `edge_max_gap_sec` | `1.0`, `0.5` | 색 전환을 길이 측정에 쓰려면 앞뒤 색이 각각 이 시간 이상 이어지고 그 사이 공백이 이 값 이하여야 한다 |
| `duration_warn_sec` | `2.0` | 측정한 등화 길이나 주기가 지도 프로그램과 이 값보다 크게 다르면 경고 로그를 남긴다 |
| `wallclock_prior_enabled` | `true` | 실차 UTC 주기 시작값 사용 |
| `wallclock_prior_mean_sec`, `wallclock_prior_std_sec` | `0.0`, `0.5` | 주기 시작값의 평균·표준편차 |
| `gnss_status_topic` | `/gnss/status_verbose` | 시계 점검용 RMC 입력 |
| `clock_check_max_offset_sec`, `clock_check_samples` | `0.5`, `10` | PC-GNSS 시각 차 한도·점검 표본 수 |
| `sim_prior_enabled`, `sim_prior_mean_sec` | `false`, `0.0` | Gazebo 시작값, 시계 점검 없음 |

- 추정하는 값은 신호 그룹마다 주기 위치 하나다. 지도 프로그램은 모두 주기 `100 s`이므로 관측이 끊긴 동안에도 같은 주기로 시간을 따라 예측한다.
- 관측 전에는 `cycle_time_sec`가 NaN이고, 남은 시간은 가능한 모든 주기 위치를 평균한 값이다. 원형 등화의 색 전환을 한 번 보면 보통 `aligned`가 된다.
- 주기 위치는 원형 등화(적색, 황색, 녹색)로만 맞추고 푼다. 화살표 관측은 맞추는 데도 푸는 데도 쓰지 않는다. PV26이 적색만 켜진 신호등을 적색과 화살표로 읽으면(Gazebo 375, 실차 305) 추정이 화살표 구간으로 끌려가 정지 중 출발 보류가 풀렸기 때문이다(kgm 결정, 2026-10-16). 적색과 화살표 관측(`STATE_ARROW`)은 `lamps`에 적색 비트가 있으면 적색 관측으로 센다.
- 화살표 등화와 화살표 진행 허용(`permitted_now`)은 원형 등화로 맞춘 주기 위치에서 지도 프로그램이 정한 대로 낸다. 적색에서 적색과 좌회전 화살표로만 바뀌는 신호(375)는 그 전환으로는 맞출 수 없고 황색 전환이나 같은 제어기의 다른 신호등으로 맞춘다.
- `phase_remaining_sec`는 지금 켜진 등화 조합이 바뀌기까지 남은 시간이다. 녹색에 화살표가 더해지는 것도 바뀜으로 센다.
- 경로 신호(Policy의 `SIGNAL_OBEY` 이벤트)에는 진행 허용 여부와 그 확률, 허용 여부가 바뀌기까지 남은 시간도 채운다. 허용 판정은 판단의 `event_permission.light_allows`와 같다.
- 관측 stamp와 노드 시계가 `5 s` 넘게 다르면 경고한다. 가제보 기록을 재생하면서 `/clock`이나 `use_sim_time`을 맞추지 않으면 나온다.
- 원형 등화는 켜진 색 가운데 하나를 읽으면 맞게 읽은 것으로 본다. 실차 375는 좌회전 화살표가 끝난 뒤 3초 동안 적색과 황색을 함께 켜고, PV26은 이때 주로 적색으로 읽는다. 지금 지도 프로그램에는 두 색을 함께 켜는 구간이 없다.
- 실차 기록(9/05, 9/19, 9/20, 10/17)에서 K-City 신호의 주기 위치는 GNSS 시각(UTC epoch 초)을 100으로 나눈 나머지와 1초 안에서 같았다.
- 실차에서는 노드 시계가 GNSS RMC 시각과 `clock_check_samples`(10) 동안 `clock_check_max_offset_sec`(0.5 s) 안으로 맞은 뒤에만 UTC 100 s 격자를 주기 시작값으로 준다. 시작값이 있으면 관측 없이도 `aligned`가 된다.
- 시계 점검 전이나 실패 시에는 관측만으로 정렬한다. 그동안 현시표가 정렬되지 않으면 카메라가 판단한다. 시계 점검은 차를 세우지 않는다.

### 현시표 판단

| 파라미터 | 값 | 의미 |
|---|---:|---|
| `signal_schedule_transition_margin_sec` | `1.0` | 자기 신호등의 예정 변경 전후 이 시간 안은 카메라가 판단 |
| `signal_schedule_reject_frames`, `signal_schedule_reject_sec` | `10`, `0.5` | 현시표가 허용인데 카메라가 확실한 반대를 이만큼 읽으면 그 허용 동안 카메라가 판단 |
| `signal_schedule_distrust_opposite_sec`, `signal_schedule_distrust_go_sec` | `3.0`, `2.0` | 카메라가 이만큼 계속 반대하면 이번 접근에서 현시표를 버린다 |
| `signal_schedule_launch_min_probability` | `0.9` | 정지 중 현시표 허용으로 출발할 최소 허용 확률 |
| `signal_schedule_launch_min_remaining_sec` | `4.0` | 정지 중 현시표 허용으로 출발할 최소 허용 잔여시간. 이보다 짧게 남은 허용으로는 출발하지 않는다 |
| `signal_schedule_withhold_limit_sec` | `100.0` | 한 정지에서 카메라 허용을 현시표가 보류하는 한도. 넘으면 이번 접근은 카메라가 판단. `0`이면 무제한 |

- 확실한 반대는 현시표가 켜지 않은 원형 등화(적색·황색, 화살표 이벤트에서는 화살표 꺼진 원형)다. 화살표 누락이나 추가는 확실한 반대로 보지 않는다.
- 현시표는 판단의 입력만 고른다. 녹색 0.4초 확인 같은 허용 규칙은 그대로다.

## 종료

| 파라미터 | 값 | 의미 |
|---|---:|---|
| `finish_line_reference_forward_m` | `0.426` | 측량한 결승선(policy `finish_line_route_s`)에 올린 차량 기준점(GNSS 안테나, tf `base_link -> gps` x)이 후륜축보다 앞선 거리 |
| `finish_line_pass_margin_m` | `2.0` | 안테나가 결승선을 이만큼 지나면 제동. 그 전까지 감속 없이 코스 속도 유지. 제동 지점은 본선 `s=733.50 m`, 예선 `s=557.24 m`. 13 m/s면 약 `764 m`에서 선다(경로 끝 `772.39 m`) |
| `finish_slow_speed_mps` | `0.6` | Finish 구간 감속 목표속도 |
| `finish_stop_distance_m` | `1.0` | 후륜축이 종료점에서 이 거리 안이면 정지. 차량 속도가 없을 때도 쓴다 |
| `finish_overrun_m` | `5.0` | 앞범퍼가 종료선을 넘은 뒤 이 거리 안에 서도록 `d_stop(v)` 지점에서 제동 |
| `finish_approach_distance_m` | `20.0` | 종료 감시 시작 거리. Stage `course_speed_mps`가 있으면 그 속도의 제동점 + `2 m`까지 자동으로 늘린다(본선 13 m/s: `29.75 m`) |
| `standstill_brake_release_sec` | `2.0` | 계획 정지(신호·정지 미션·결승) `HELD`에서 정지 상태가 이만큼 이어지면 `STANDSTILL_BRAKE_RELEASE`를 붙여 motion_control이 브레이크 서보를 푼다. 목표속도는 계속 0, `0`이면 끈다 |

- 측량한 결승선이 있는 코스(K-City 예선·본선)는 위 `finish_line_*` 두 값만 쓴다. 속도와 상관없이 고정 지점에서 제동하고, 결승선 전 `0.6 m/s` 감속과 `d_stop(v)` 제동점은 없다. 아래 네 값은 측량선이 없는 경로(경로 끝 = 결승선)용이다.
- 제동 시점은 후륜축과 종료점 사이가 `d_stop(v) + vehicle_front_overhang_m - finish_overrun_m` 이하이거나 `finish_stop_distance_m` 이하일 때다.
- 결승 정지 뒤 속도가 `static_stop_stopped_speed_mps=0.1 m/s` 이하이면 `FINISH_HELD`를 낸다. 시험 TUI의 Run 완료는 이것만 본다(결정 D11).
- 10/11 제동값이면 앞범퍼는 종료선 `5.0 m` 뒤, 뒤범퍼는 `2.5 m` 뒤에 선다. 10/03처럼 제동이 강하면 `8.3 m/s` 넘게 들어올 때 앞범퍼가 선에 닿기 전에 설 수 있다.
- Finish 구간에서도 경로는 전역 로컬 경로를 따른다.
- 브레이크 해제(2026-10-24): 정지 요청이 stop_point 하나뿐이고 속도가 `0.1 m/s` 이하로 `2 s` 이어질 때만 붙는다. E-Stop·입력 끊김에서는 붙지 않는다. 차가 `0.1 m/s`를 넘게 구르면 motion_control이 그 주기에 바로 다시 브레이크를 걸고, 판단은 2초를 다시 센다. 해제된 상태에서 출발하면 서보 해제 대기(`brake_release_delay_sec`) 없이 바로 구동한다.

## 곡선 속도

- 목표속도: `순항속도 × speed_scale`이다.
- 2026-10-21 곡선 절대 속도 상한(`curve_speed_max_mps=5.0`)을 파라미터째 없앴다.
- `speed_scale=0.8`과 `approach_decel_mps2=1.4`는 각 `waypoints/kcity_*_route.yaml`의 `curve_speed_policy`에 있다.
- TUI로 수집한 경로에는 `curve_speed_policy`가 없다. 수집 때 찾은 회전 구간(`tracking_profile.corners`)마다 곡선 구간을 만들고 같은 `speed_scale=0.8`, 접근 감속 `1.4`를 쓴다(`captured_route_curve_policy`). 곡선 반경 상한(`curve_lateral_accel_mps2`)과 접근 감속(`curve_approach_decel_mps2`)도 그대로 적용된다.
- 이전 값 `5.0 m/s`는 10/03 선회 13회 모두 CTE `1.0 m` 이내인 최대 속도였다. 검토 이력은 상위 저장소의 `docs/reports/2026-10-15_decision_review/decision_fixes.md`다.

## 장애물 회피

| 파라미터 | 값 | 의미 |
|---|---:|---|
| `obstacle_fallback_speed_mps` | `1.6` | 확정 여유가 `obstacle_slow_clearance_m` 이하인 경로, 차선 한계를 벗어난 경로, 강제 S자 경로, 열은 있는데 경로가 하나도 없을 때 |
| `obstacle_slow_clearance_m` | `0.25` | 회피 속도가 이 여유에서 `obstacle_fallback_speed_mps`, `obstacle_clearance_m`에서 구간 속도이고 그 사이는 직선으로 잇는다 |
| `obstacle_clearance_m` | `0.50` | 차체와 장애물 사이 목표 여유. 탐색은 이 값까지 여유가 가장 큰 경로를 고른다 |
| `obstacle_transition_length_m` | `12.0` | 첫 통과점 연결 길이 |
| `obstacle_return_length_m` | `12.0` | 전역경로 복귀 길이 |
| `obstacle_lateral_accel_mps2` | `6.0` | 회피 경로 곡률 속도 상한 √(a/곡률). 10/24 본선 1241에서 `1.4`는 반경 4.9 m 경로를 2.6 m/s로 묶었다. `6.0`은 반경 4.2 m까지 5.0 m/s |
| `obstacle_curve_speed_window_sec` | `0.5` | 회피 경로 곡률 속도 상한을 이 시간 동안의 중앙값으로 쓴다. 프레임마다 곡률이 조금씩 달라 상한이 튀지 않게 한다. `0`이면 끈다 |
| `obstacle_max_steering_rate_deg_s` | `0` (없음) | 회피 경로 조향 변화로 거는 속도 한계. 0 이하면 걸지 않는다. 10/23 미단에서 60°/s가 횡가속 한계(약 2.6 m/s)보다 낮은 1~2 m/s로 묶었다 |
| `obstacle_steering_gradient_window_m` | `1.0` | 조향 변화량을 이 거리 동안의 변화로 잰다. 0.25 m 이웃 점 사이로 재면 곡률 잡음 때문에 2~3배 크게 나왔다 |
| `obstacle_row_memory_sec` | `0.8` | 객체 세트 관측 유지 |
| `obstacle_row_track_match_m` | `1.25` | 프레임 간 세트 연결 거리 |
| `obstacle_row_history_size` | `7` | 세트 형상 중앙값 표본 수 |
| `obstacle_row_pair_nominal_gap_m` | `4.4` | 두 드럼 줄의 명목 간격. 서로 반대쪽에 있는 쌍이 여럿이면 이 값에 가장 가까운 쌍을 고른다. `6.0`으로 두면 첫 줄을 `5.9 m` 떨어진 길가 봉과 짝지었다(2026-10-17) |
| `obstacle_route_half_width_m` | `2.0` | 지도 차선이 없는 곳의 명목 반폭(통로 4.0 m). 수집 경로 Case `4`는 선택 경로 전체에 쓴다 |
| `obstacle_centerline_departure_m` | `0.5` | 어느 열이든 중앙선 쪽으로 차선 한계보다 더 나갈 수 있는 거리. 연석 쪽은 0(결정 D13) |
| `obstacle_right_passage_margin_m` | `0.3` | 열의 오른쪽으로 지나는 통로의 상한을 이만큼 낮춰 열에서 더 떨어진다. 오른쪽 조향이 약하다 |
| `obstacle_association_m` | `1.0` | `track_id` 0 객체를 기억 객체에 붙이는 map 거리 |
| `obstacle_confirm_frames` | `10` | 이 프레임 수만큼 본 객체만 장애물로 쓴다(30 Hz에서 약 0.33 s). 10/23 미단 유령은 1~8프레임, 드럼은 400프레임 이상. 확인된 객체는 차가 지날 때까지 기억한다 |
| `obstacle_max_bottom_height_m` | `2.0` | 상자 바닥이 이 높이(base_footprint 기준)보다 위면 도로 위 구조물(표지판, 나뭇가지)로 보고 버린다. 10/23 미단 s 64: 바닥 3.5 m |
| `obstacle_min_top_height_m` | `0.2` | 상자 꼭대기가 이 높이보다 낮으면 노면 반사로 보고 버린다. 10/23 미단 드럼 뒤 군집: 꼭대기 −0.02~−0.08 m |
| `obstacle_unconfirmed_memory_sec` | `0.8` | 확인 전 객체가 이 시간 다시 안 보이면 버린다. 10/23 미단: 1~3프레임 유령이 끝까지 남아 없는 장애물을 피했다 |
| `objects_stale_sec` | `0.5` | 객체가 이보다 오래되면 새 계획 없이 확정 경로만 따른다 |
| `obstacle_zone.polygon` (`final_policy.yaml`) | 본선 WP 640 ~ WP 670 | 장애물 구간 다각형(map). 밖의 객체는 버린다. 경로 기준 `639.85~669.84 m`, 양 차로(결정 D14, 범위는 2026-10-16 kgm 결정). 끝 + `obstacle_return_length_m`이 결승 제동점(측량한 결승선을 안테나가 2 m 지난 733.5 m)보다 앞이 아니면 판단이 시작하지 않는다(D18) |
| `obstacle_zone.start_route_s` (`final_policy.yaml`) | `639.849483634` | 구간 속도 상한의 시작(본선 WP 640, 앞범퍼 기준). 없으면 다각형 시작 |
| `obstacle_zone.lane_boundaries` (`final_policy.yaml`) | 지도 차선 경계 id | 구간 안 차선 한계. 경로에서 `boundary_max_offset_m=3.0` 넘게 떨어진 점은 버린다 |
| `obstacle_zone.speed_cap_mps` (`final_policy.yaml`) | `5.0` | 장애물 구간 속도 상한. 인지가 끊겨 확정 경로를 따를 때도 같다. 차가 구간 끝을 지나고 회피가 끝나면 풀린다(아래 두 값). 브레이크는 걸지 않는다(결정 D10) |
| `obstacle_release_past_row_m` | `3.0` | 구간 끝 이후 상한 해제: 후륜축이 마지막 열을 지난 거리 |
| `obstacle_release_offset_m` | `0.5` | 구간 끝 이후 상한 해제: 확정 경로의 그 자리 전역경로 횡오프셋. 복귀점을 기다리지 않는다 |
| `obstacle_zone.pre_slow_decel_mps2` (`final_policy.yaml`) | `0.5` | 구간 앞에서 상한을 낮추는 감속. 구동과 브레이크 없이 실측한 값(0.4~0.7)이라 굴러가기만 해도 맞춘다 |
| `obstacle_forced_speed_mps` | `5.0` | 강제(best-effort) 회피 경로 속도. 간격이 `obstacle_slow_clearance_m` 이하인 경로도 같다. `-1`이면 `obstacle_fallback_speed_mps`. 간격 상한이 이 값에서 시작하므로 `obstacle_bypass_speed_mps`와 같은 `5.0`이면 간격으로 줄이지 않는다(10/24. 10/23 미단 회피 성공 값은 `3.0`) |
| `obstacle_bypass_speed_mps` | `5.0` | 회피 경로를 따르는 동안의 상한(main의 `obstacle_slow_speed_mps`). 본선은 구간 상한과 같다. 구간 상한이 없는 미단(`15.0`)에서는 드럼을 보고 회피 경로를 확정한 뒤부터 줄이고, 차체가 마지막 열을 지나면 푼다(복귀 경로는 곡률 속도만) |
| `obstacle_zone.lane_offsets_m` (`midan_test_policy.yaml`) | 오른쪽 `-1.96 m`, 왼쪽 `+2.14 m` | 다각형·지도 차선 대신 전역경로 기준 고정 차선. 본선 드럼 위치의 흰 가장자리선·노란 중앙선(지도와 10/17 인지 일치). 구간은 `start_route_s`부터 경로 끝까지, 모든 객체가 대상이며 D18 결승 점검은 하지 않는다 |
| `speed_zones[SCHOOL_ZONE].start_route_s` (`qualifying_policy.yaml`) | `331.5 m` | 예선 어린이보호구역, 앞범퍼 진입 시 상한 적용 |
| `speed_zones[SCHOOL_ZONE].end_route_s` (`qualifying_policy.yaml`) | `404.6 m` | 후륜축 통과 후 상한 해제 |
| `speed_zones[SCHOOL_ZONE].speed_cap_mps` (`qualifying_policy.yaml`) | `7.5` | `27 km/h`, 규정 `30 km/h`에 오버슈트 여유 |
| `speed_zones[SCHOOL_ZONE].pre_slow_decel_mps2` (`qualifying_policy.yaml`) | `0.5` | 장애물 구간과 같은 사전 감속 |

- 장애물은 map 좌표로 기억하고 매 틱 전역경로 `(s,d)`의 점유 범위로 처리한다. 
- 회피 경로는 한 번 확정하고, 새 열이 보이거나 기억 객체가 확정 경로 여유를 침범할 때만 다시 계획한다.
- 열의 앞면이 `obstacle_transition_length_m`(12 m) 안에 들어오면 그 열의 통과 방향을 고정한다. 이후 다시 계획해도 같은 쪽으로 지난다.
- 다시 계획한 경로는 차 위치에서 확정 경로의 오프셋, 기울기, 곡률로 시작한다. 차가 확정 경로에서 `0.5 m` 안이면 오프셋을 그대로 잇고, 아니면 차 위치에서 시작한다.
- 첫 세트의 빈 쪽, 두 번째 세트의 빈 쪽, 전역경로 복귀점을 연결한다. 
- 고정 좌우 순서에 의존하지 않는다. 명목 간격은 반대쪽 짝이 여럿일 때 고르는 기준일 뿐이어서(`obstacle_row_pair_nominal_gap_m`), 그와 다른 간격의 배치도 그대로 통과한다.
- 지도 차선(구간 밖에서는 `obstacle_route_half_width_m` 통로)에 걸치지 않는 객체는 장애물로 보지 않는다. 옆 차로의 길가 봉을 드럼 줄로 보지 않기 위해서다(2026-10-17).
- 경로 선택 기준은 하나다(2026-10-19). 조향률, 곡률, 차선 한계를 지키는 후보 가운데 차체와 장애물 사이 최소 여유가 가장 큰 경로를 고른다. 여유는 `obstacle_clearance_m`에서 자르고, `3 cm` 안의 차이는 같은 것으로 본다. 같은 여유라면 열의 오른쪽으로 지나는 통로가 `obstacle_right_passage_margin_m`을 더 지키는 경로가 먼저고, 그다음은 덜 꺾는 경로다(최대 곡률 `0.1 1/m`를 옆 이동 `1 m`로 친다). 여유 단계를 나눠 다시 찾는 탐색과 best-effort 탐색은 없앴다.
- 각 줄은 빈 쪽으로만 지나므로 좌우가 번갈이면 항상 S자다. 차선 한계 안의 후보가 하나도 없을 때만 한계를 벗어난 후보를 고르고(벗어난 거리가 작은 것 먼저), 이 경로는 `1.6 m/s`로 간다.
- 속도는 확정 경로의 여유를 따른다. 여유가 목표 이상이면 구간 속도, `obstacle_slow_clearance_m` 이하이면 `1.6 m/s`, 그 사이는 직선으로 잇는다. 차체가 마지막 줄을 지나면 풀린다. 장애물에 닿는 경로도 멈추지 않고 `1.6 m/s`로 간다.
- 탐색은 확정 경로가 없을 때, 확정 여유보다 `0.1 m` 넘게 줄었을 때(차선 한계를 확정 때보다 더 벗어날 때 포함), 새 줄이 보일 때만 한다. 나머지 틱은 확정 경로를 다시 재기만 한다(약 `2 ms`). 탐색 한 번은 후보 약 120개를 한꺼번에 재며 `25~40 ms`다. 찾지 못한 탐색은 같은 줄에 대해 `0.5 s` 동안 되풀이하지 않는다.
- 줄의 차선 한계가 없으면 다른 줄의 한계를, 그것도 없으면 `obstacle_route_half_width_m` 통로를 쓴다.
- 후보가 하나도 없으면 마지막 수단으로 `1.6 m/s` 고정 차선변 강제 S자 또는 기존 경로 fallback으로 계속 주행하며 새 정지 트리거를 만들지 않는다.

- 일반 주행 자동 REJOIN은 없다. 회피 경로는 `obstacle_return_length_m` 안에 전역경로로 돌아오고, 복귀점을 지나면 전역경로 추종으로 바뀐다. 구간 상한은 그보다 먼저 위 해제 조건에서 풀릴 수 있다.
- 수집 경로 Case `4`(launch `17`, 미단 회피)는 `midan_test_policy.yaml`의 고정 차선 구간으로 본선과 같은 지도 차선 탐색을 쓴다. 인지 차선은 쓰지 않는다.
- 미단 구간은 경로 `21 m`부터 끝까지(`start_route_s` `21.0 m`, 상한 `15.0 m/s` = 사실상 없음)다. 앞 `21 m`는 전역경로 추종이라 장애물 구간으로 바뀌는 순간이 기록에 남는다(10/23: `1.0 m`는 출발부터 구간 안이었다). 목표속도를 유지하다 회피 경로를 확정하면 `obstacle_bypass_speed_mps`(`5.0`)로 줄인다. 드럼 위치를 설정하지 않으므로 구간 앞 사전 감속과 다각형 거르기는 없다.
- Case `4`는 결승 구간 `finish_zone_distance_m`을 `0 m`, 판단의 `finish_approach_distance_m`을 `0.1 m`로 둔다. 회피·복귀·재가속을 끝까지 보고 후륜축이 경로 끝에 닿을 때 결승 브레이크를 건다. 수집 경로 추종(`13`)과 예선·본선은 `20 m` 그대로다.
- 미단 드럼은 10/17 본선처럼 전역경로 기준으로 놓는다: 줄당 3개, 줄 간격 `4.4 m`, 한 줄 `d +0.4 ~ +2.05 m`, 다른 줄 `d -1.7 ~ -0.05 m`.

## Decision AEB

| 파라미터 | 값 |
|---|---:|
| 객체 stale | `0.25 s` |
| 경로 반폭 | `1.35 m` |
| warning TTC / 거리 | `2.0 s / 5.0 m` |
| brake TTC / 거리 | `0.8 s / 1.4 m` |
| 해제 확인 | `8 ticks` |

- Decision AEB는 경고만 한다. brake 조건에 닿아도 로그와 `AEB_WARNING`만 남기고 제동하지 않는다(2026-10-16 결정 D8). `avoidance/all`에서 쓰고 장애물 구간 안과 회피 중에는 경고도 끈다.
- 물리 E-Stop은 별개이며 항상 우선한다.

## Motion Control

| 파라미터 | 값 |
|---|---:|
| 제어 주기 | `50 Hz` |
| 목표 stale | `0.35 s`, 넘으면 속도 0(브레이크 없음) |
| 끊김 브레이크 `target_stale_brake_delay_sec` | `1.0 s`, stale이 이만큼 이어지면 실제 정지까지 브레이크 |
| 축거 | `1.2991 m` |
| 최대 조향 / 변화율 | `25 deg` / 제한 없음(60 deg/s는 lateral_ctrl) |
| 일반 가속 / 감속 | `1.0 / 2.0 m/s²` |
| 정지선·신호 사전 감속 `stop_pre_decel_mps2` | `10.0 m/s²`, 해당 PRE_DECEL에 적용. |
| 장애물 감속 `obstacle_decel_mps2` | `10.0 m/s²`, 비정지 OBSTACLE 목표에 적용. 곡률·조향률 감속과 저속 fallback 포함 |
| 정지 후 brake 해제 대기 | `0.8 s` |
| 재출발 boost | `2.0 m/s²`, `5.0 m/s`까지 |

### PP·Stanley

| 항목 | 값 |
|---|---:|
| 직선 Ld | `clamp(4.0 + 0.9 * speed, 3.6, 9.5) m` |
| 곡선 Ld 하한 | `clamp(3.6 + 0.30 * speed, 3.6, 직선 Ld) m` |
| Stanley heading gain | 직선 `0.30`, 코너 `0.45` |
| Stanley CTE gain | 직선 `0.25`, 코너 `0.35` |
| PP 비율 | 직선 `0`, 접근 `0.65`, 회전 `0.35`, 복귀 `0.10` |
| `stanley_only_path_source_prefixes` | `sequential_bypass`, `forced_sequential_bypass`: 회피 경로를 따르는 동안 PP 비율 `0`(10/23 미단 1426, PP가 S의 첫 열 옆을 `0.3 m` 안쪽으로 잘랐다) |

- 경로 추종 중 실제 조향 피드백은 진단용이다. 초저속·정지 시에는 현재 조향각 유지에 사용한다.

### 직선 추종

| 항목 | 내용 |
|---|---|
| 직선 Stanley 이득 | `6 m/s` 이상은 고유진동수 `straight_natural_frequency_radps`·감쇠 `straight_damping_ratio`로 속도마다 계산. 그 아래는 `3 m/s`까지 위 표의 고정 이득으로 섞고, `3 m/s` 이하와 회피 경로는 고정 이득 |
| 곡선 Stanley 이득·PP 비율·Ld | 위 표 |
| 경로 지연 보상 | 전역경로 기반 지역경로만, 시각 이후 이동만큼 옮김(최대 `0.3 s`). 회피 경로는 제외 |
| 직진 편향 적분 | 전역경로 직선·`4 m/s` 이상·CTE `0.6 m` 이내에서 전륜 CTE 적분, 시간상수 `4 s`, `±1.5 deg` |
| 곡선 속도 | 순항×0.8과 √(`curve_lateral_accel_mps2`×곡선 최소 반경) 중 작은 값, 접근 감속 `curve_approach_decel_mps2` |

| 파라미터 (Motion Control) | 값 | 의미 |
|---|---:|---|
| `straight_natural_frequency_radps` | `1.5` | 직선 폐루프 고유진동수 |
| `straight_damping_ratio` | `1.1` | 직선 폐루프 감쇠. 2026-10-23 0.95에서 올림(루프 지연 0.42 s, 고속 헤딩 이득 확보) |
| `straight_gain_blend_start_mps` / `straight_gain_full_speed_mps` | `3.0` / `6.0` | `6 m/s` 이상은 계산 이득, 그 아래는 `3 m/s`까지 고정 이득으로 섞는다 |
| `straight_integral_time_s` / `straight_integral_limit_deg` | `4.0` / `1.5` | 직진 적분 시간상수·제한 |
| `straight_integral_min_speed_mps` | `4.0` | 적분하는 최저 속도 |
| `straight_integral_max_error_m` | `0.6` | 이보다 큰 전륜 CTE(출발 오프셋·회피 복귀)에서는 적분 유지 |
| `path_latency_max_s` | `0.3` | 지연 보상 최대 시간 |
| `straight_encoder_deg` | `-0.20` | 지연 보상 요레이트용 직진 엔코더각. 2026-10-25 앞바퀴 정렬 뒤 `-0.45`에서 바꿈 |

| 파라미터 (Planning, `decision_pipeline.yaml`) | 값 | 의미 |
|---|---:|---|
| `curve_lateral_accel_mps2` | `6.0`, `0` 끔 | 곡선별 속도 상한의 횡가속(약 0.6 G, 10/24에 `3.5`에서 올림. 실차는 횡 1 G를 버틴다) |
| `curve_approach_decel_mps2` | `0.5`, `-1` 경로 YAML | 곡선 접근 감속 기울기, 10/11 실측 감속 |

- 이득 식: `ke = ωn²·L·(v+3)/v²`, `kh = 2ζωn·L/v − ke·L/(v+3)`. 고정 이득의 ωn은 7 m/s `0.97`, 10 m/s `1.22`, 15 m/s `1.55`다.
- 적분은 곡선·회피·저속·정지에서 값을 유지하고 수동 전환에서 0이 된다.
- 진단: `/diagnostics` Motion Control의 `straight_integral_deg`, `path_latency_s`, `stanley_heading_gain`·`stanley_cross_track_gain`(계산된 이득).
- 근거: `reports/261022_경로추종기_속도별_재검토`.

### 조향 보정

Motion Control은 명령에 직진 오프셋을 더하고, 엔코더가 목표에 못 미치면 목표−엔코더 차이 방향으로 속도별 마찰만큼 더 민다. 보정은 25° 제한 앞에 걸린다.

| 파라미터 | 값 | 의미 |
|---|---:|---|
| `steering_neutral_offset_deg` | `-0.52` | 직진각 −0.20° + 액추에이터 좌편향 약 −0.32°. 2026-10-25 앞바퀴 정렬 뒤 `-0.82`에서 바꿈(`reports/261025_정렬후_좌우90도_추종기_재적합`) |
| `steering_friction_speed_mps` / `steering_friction_comp_deg` | `[3, 6.5, 12]` / `[1.0, 0.85, 0.8]` | 속도별 마찰 보상량, 구간 밖은 끝값. 2026-10-23 12 m/s 값을 0.6에서 올림 |
| `steering_error_start_deg` / `steering_error_full_deg` | `0.2` / `0.5` | 보상 시작·최대 오차 |
| `steering_error_ramp_s` | `0.1` | 보상 전환 완화 시간 |

- 근거: 09/19~10/17 자율주행 기록의 마찰띠 0.7~1.1°, 좌편향 +0.25~0.42°, 속도별 출발 문턱(`reports/261022_조향액추에이터_모델_직선흔들림`).
- 오프셋은 엔코더 보정(영점)이 바뀌면 다시 재야 한다(09/20 `e0cc6b9` 전후 직진각 −1.3° → −0.45°).
- 시나리오 하네스의 조향은 이상적이라 오프셋이 그대로 우측 편향이 된다. 비교는 보고서의 실측 조향 모델(`scripts/sim_worker.py`)로 한다.
- Control `kty`의 유격 보상(`torque_backlash_friction_raw`)과 함께 켜면 중복 보정이다.
- 진단: `/diagnostics` Motion Control의 `steering_compensation_deg`(더한 총량), `steering_friction_push_deg`.

## 라바콘 제동 검차

| 항목 | 값 |
|---|---:|
| 횡제어기 | TUI는 Stanley 고정(`scenario 2`). PP(`1`)는 launch 인자로만 |
| Stanley 조향 | 예선·본선 추종기와 같은 값을 `motion_control.yaml`에서 읽는다: 직선 고정·속도별 이득, 보정 한도, 직선 적분, 영점 −0.52°·마찰 밀기 |
| TUI 최고속도 | `30 km/h` |
| 속도명령 증가율 | `1.0 m/s²` 고정 |
| 빨간선 선행 제동거리 | `5.0 m` |
| E-stop | TUI `E`: `NOT-USE`(기본) 빨간선에서 차량 브레이크 체결, `USE` 빨간선에서 속도·조향 그대로, 브레이크 체결 없음 (launch `vehicle_brake`) |
| 빨간선 확인 | `2 frames` |
| 제동 중 경로 추종 종료 | 실제속도 `<= 0.5 m/s` |
| 경로 / 차량 timeout | `0.35 / 0.25 s` |
| 콘 경로 끊김 유지 | `path_gap_hold_s 0.5 s`: 빈 경로·늦은 경로에서 직전 속도·조향 유지, 넘으면 제동 |

- 단독 추종 노드의 YAML 속도는 `5 km/h`지만 TUI가 `30 km/h`와 입력한 `V/B`로 덮어쓴다.
- 기본 제동값은 시험 시작값이며 실차 정지거리를 보증하지 않는다.
