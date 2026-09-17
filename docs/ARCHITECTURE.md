# Decision Architecture

Decision이 인지·측위 입력에서 차량 명령을 만드는 구조다. 실행은
[DECISION_RUNBOOK.md](DECISION_RUNBOOK.md), 설정값은 [PARAMETER_BOOK.md](PARAMETER_BOOK.md)를
참고한다.

## 패키지 경계

| 패키지 | 책임 | 주요 출력 |
|---|---|---|
| `kaiev26_decision` | Route 정합, Zone·Event, FSM, 시험 TUI | 목표 경로·속도 |
| `kaiev26_motion_control` | PP·Stanley, 속도 계획, 최종 제동 | `/planning/command` |
| `kaiev26_decision_fox` | 실차·MCAP 시각화 | Foxglove 토픽·화면 |
| `kaiev26_aeb` | 라바콘 경로와 빨간선 제동 검차 | `/planning/command` |
| Control | 명령 검증과 STM·KEYA 구동 | `/vehicle/command` |

Decision은 목표값을 만든다. 실제 구동 허용과 액추에이터 보호는 Control 책임이다.

## 경기 판단 흐름

```mermaid
flowchart TD
    P[Perception] --> PG[perception_gateway]
    PG --> S[/planning/scene_summary]
    O[/localization/odometry] --> RZ[route_zone_manager]
    RZ --> R[/planning/route_context]
    V[/vehicle/state] --> PE[planning_engine]
    S --> PE
    R --> PE
    PE --> FSM[Policy + Priority + FSM]
    FSM --> TP[/planning/target_path]
    FSM --> TS[/planning/target_speed]
    TP --> MC[motion_control]
    TS --> MC
    V --> MC
    MC --> C[/planning/command]
    C --> GOV[command_governor]
    GOV --> VC[/vehicle/command]
    VC --> HW[STM_A / STM_B / KEYA]
```

| 단계 | 구현 | 역할 |
|---:|---|---|
| 1 | `perception_gateway_node.py` | 인지 4종을 동기화하고 대상 신호·장애물 선별 |
| 2 | `route_zone_manager_node.py` | Odometry를 경로에 투영하고 진행도·Stage·로컬 경로 생성 |
| 3 | `priority_selector.py`, `zone_policy.py` | 현재 Zone에서 실행할 Event 선택 |
| 4 | `scenario_modules/` | 추종, 신호, 정지, 회피, 복귀, 완주 FSM 실행 |
| 5 | `main_planning_engine_node.py` | 목표 경로·속도 발행 |
| 6 | `motion_control_node.py` | 조향·속도·브레이크 명령 계산 |
| 7 | Control | 명령 검증 후 하드웨어 전달 |

## 시험 TUI 흐름

시험 중에는 TUI가 명령 중계와 MCAP 기록을 관리한다.

```text
decision_test.launch.py
  -> Decision + Motion Control
  -> /decision/raw_command
  -> test_tui 출발·AUTO 게이트
  -> /planning/command
  -> command_governor
```

`/planning/command`에는 TUI만 발행한다. 다른 Decision·AEB·SysID 발행자가 있으면 출발을
막는다. 조향 추종 오차 경고는 기록하지만 TUI가 시험을 중단하지 않는다.

## 행동 우선순위

```text
E-Stop/AEB
  > 입력·Localization 이상
  > Finish
  > 현재 Zone의 장애물
  > 현재 Zone의 정지선·신호
  > 일반 경로 추종
```

진행 중인 미션 FSM은 완료까지 유지된다. Stage와 완료 Event는 같은 Run에서 뒤로 돌아가지
않는다.

## 시험 Mode

| mode | 기능 |
|---|---|
| `tracking` | 전역경로와 곡률 속도 계획만 사용 |
| `stopline` | 등록 정지점을 모두 3초 정지로 시험 |
| `avoidance` | 본선 회피만 사용 |
| `all` | 코스 Policy의 정지·신호·회피·완주 사용 |
| `steering_step` | 경로 없이 상수 속도와 R/L 조향 스텝 발행 |

Case 8은 경로·Localization·FSM·Decision AEB를 제어 조건으로 사용하지 않는다. GNSS·IMU는
궤적 분석용으로만 기록한다.

## 기준 파일

| 파일 | 내용 |
|---|---|
| `waypoints/kcity_*_route.yaml` | 전역경로와 종료점 |
| `config/*_landmarks.yaml` | 정지선·신호등 지도 위치 |
| `config/*_policy.yaml` | Stage, Event, 제동 waypoint, 허용 회전 |
| `config/decision_pipeline.yaml` | 입력, 미션, 회피·복귀, AEB 설정 |
| `kaiev26_motion_control/config/motion_control.yaml` | PP·Stanley, 조향·속도·정지 설정 |

경로, landmark, policy는 같은 코스를 설명하므로 하나를 바꾸면 서로의 정합을 확인한다.

## 주요 인터페이스

| 토픽 | 내용 | 미수신 시 |
|---|---|---|
| `/localization/odometry` | map 위치·헤딩 | 경로 정합 불가 |
| `/planning/route_context` | 진행도·Stage·정지선·로컬 경로 | 판단 불가 |
| `/planning/scene_summary` | 차선·장애물·신호 요약 | `all/avoidance` 판단 불가 |
| `/vehicle/state` | 속도·조향·모드·E-Stop | 제어·정지 확인 불가 |
| `/planning/target_path` | 차량 기준 목표 경로 | 0.35초 후 fail-safe 제동 |
| `/planning/target_speed` | 속도·정지 목표 | 0.35초 후 fail-safe 제동 |
| `/planning/command` | Decision 최종 명령 | governor 입력 단절 |
| `/vehicle/command` | Control 승인 명령 | 액추에이터 입력 단절 |

Motion Control은 실제 조향속도와 목표·실제 오차를 진단한다. 이 값으로 목표각을
예측하거나
속도를 낮추지 않는다.

## 라바콘 검차 흐름

```text
카메라 + 선택적 LiDAR
  -> 콘·경계·빨간선 인지
  -> /aeb/center_path
  -> PP 또는 Stanley
  -> 빨간선 latch
  -> /planning/command
```

`kaiev26_aeb`는 경기 Decision과 동시에 실행하지 않는다.

## 안전 경계

- Motion Control은 목표 입력이 0.35초 이상 오래되면 속도 0과 브레이크를 요청한다.
- `command_governor`는 stamp가 없거나 오래된 명령과 비정상 수치를 거부한다.
- Control의 AUTO·CAN·KEYA 조건은 Decision과 별도로 항상 적용된다.
- `brake_engage=true`는 브레이크 서보 요청이다. 실제 체결 여부는 하드웨어에서 확인한다.
- 실차 안전 기준은 현재 Control 소스와 `Control/docs/SAFETY.md`를 우선한다.
