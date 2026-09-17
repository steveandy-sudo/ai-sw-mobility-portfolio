# Route Tracking

전역 waypoint에서 조향·속도·브레이크 명령이 만들어지는 과정이다. 현재값은
[PARAMETER_BOOK.md](PARAMETER_BOOK.md)를 참고한다.

## 처리 순서

```text
/localization/odometry
  -> 전역경로 진행거리와 차량 기준 18 m 로컬 경로
  -> PP / Stanley / PP+Stanley / FF+Stanley
  -> 직선 허용 통로와 조향 제한
  -> 곡률·미션 속도 중 최솟값
  -> 가감속 제한과 정지
  -> /planning/command
```

## 경로 정합

`route_zone_manager_node`는 후륜축 중앙인 `base_footprint`를 전역경로에 투영한다. 주행 중에는
이전 진행점 주변만 검색해 가까운 반대 방향 차선으로 점프하는 것을 줄인다.

| 항목 | 값 |
|---|---:|
| 전역 waypoint 간격 | `1.0 m` |
| 로컬 경로 | 전방 `18.0 m`, 간격 `1.0 m` |
| 진행점 검색 범위 | `35.0 m` |
| 최대 경로 정합 거리 | `15.0 m` |

차량 뒤 `x < -0.2 m`인 로컬 경로점은 제외한다. 지정 waypoint 제동도
`base_footprint`가 해당 진행거리에 도달할 때 발생한다.

## Pure Pursuit

직선 Ld와 강한 코너 Ld 하한은 다음과 같다.

```text
straight_Ld = clamp(4.0 + 0.9 * speed, 3.6, 9.5) [m]
curve_floor = clamp(3.6 + 0.30 * speed, 3.6, straight_Ld) [m]
```

전방 12 m에서 같은 방향 곡률이 3점 이상 이어지면 코너로 판단한다. 상태는
`straight -> turn_approach -> turning/chained_turn -> turn_exit` 순서다. 진입 곡률은
`0.015 1/m`, 해제 곡률은 `0.010 1/m`다.

Ld 원과 경로 선분의 교점을 보간해 목표점 `(x, y)`를 고른다.

```text
curvature = 2 * y / (x^2 + y^2)
steering = atan(wheelbase * curvature)
```

`y > 0`은 좌조향, `y < 0`은 우조향이다.

## Stanley

앞차축을 경로에 투영해 헤딩 오차와 CTE를 더한다.

```text
heading_term = clamp(heading_gain * heading_error, heading_limit)
cte_term = clamp(atan(cross_track_gain * front_axle_CTE
                 / (abs(speed) + softening_speed)), cte_limit)
steering = heading_term + cte_term
```

| 항목 | 직선 | 강한 코너 |
|---|---:|---:|
| heading gain | `0.30` | `0.45` |
| CTE gain | `0.25` | `0.35` |
| heading 항 제한 | `0.10 rad` | `0.16 rad` |
| CTE 항 제한 | `0.05 rad` | `0.07 rad` |
| 접선 평활 거리 | `8.0 m` | `6.0 m` |

코너에서는 속도와 곡률 강도에 따라 최대 3.5 m 앞의 경로 형상을 반영한다. 실제 조향각을
예측하는 기능은 아니다.

## 혼합 제어기

PP+Stanley는 전역경로 형상 단계마다 고정 PP 비율을 사용한다.

| 단계 | PP / Stanley |
|---|---:|
| `straight` | `0 / 100%` |
| `turn_approach` | `65 / 35%` |
| `turning`, `chained_turn` | `35 / 65%` |
| `turn_exit` | `10 / 90%` |

단계가 바뀌면 혼합비를 최대 `6.0/s`로 이동한다. 순간 CTE로 제어기를 전환하지 않는다.

FF+Stanley는 전방 경로 곡률로 기본 조향을 만들고 Stanley 보정의 50%를 더한다.

```text
ff = atan(wheelbase * reference_curvature)
steering = ff + 0.50 * stanley
```

## 직선 허용 통로

전역경로 직선에서는 작은 GNSS·횡오차를 모두 추종하지 않는다.

```text
preview = clamp(speed * 1.2, 4.0, 9.0) [m]
predicted_error = path_y + preview * sin(path_heading_error)
```

예상 횡오차 `0.08 m`, 헤딩 `0.8 deg` 안에서는 미세 보정을 제거한다. 각각 `0.25 m`,
`2 deg`까지 정상 조향을 연속 복구한다. 코너·회피·REJOIN에는 적용하지 않는다.

모든 제어기의 최종 제한은 최대 조향 `25 deg`, 변화율 `60 deg/s`다. 실제 조향속도와
목표·실제 오차는 진단만 하며 목표각이나 속도를 바꾸지 않는다.

## 속도 계획

```text
target = min(cruise, curvature, Zone/FSM, avoidance/rejoin)
```

곡률 강도 `severity`가 0에서 1로 커질수록 코너 속도 상한을 `3.2 m/s`까지 낮춘다.

```text
curve_speed = cruise + (min(3.2, cruise) - cruise) * severity
```

| 항목 | 값 |
|---|---:|
| 일반 가속 | `1.0 m/s^2` |
| 일반 감속 | `2.0 m/s^2` |
| 신호 재출발 | `2.0 m/s^2`, `3.8 m/s`까지 |

## 미션 정지

정지가 필요한 Event는 제동 WP 6 m 전부터 `2.0 m/s` 방향으로 감속한다.

```text
entry = min(cruise, 2.0)
d > 6 m:       target = cruise
0 < d <= 6 m: target = entry + (cruise - entry) * d / 6.0
d <= 0:       target = 0, brake_engage = true
```

실제속도 `0.1 m/s` 이하부터 3초 정지 또는 신호 대기를 시작한다. 신호 진행은 0.4초 연속
확인 후 재출발한다.

| 코스 | 제동 waypoint |
|---|---|
| 예선 | `203, 244, 296, 337` |
| 본선 | `98, 153, 289, 346, 448` |

8.0 m/s에서 2.0 m/s까지 2.0 m/s²로 낮추는 이론 거리는 약 15 m다. 고속 시험 전에는
실제 제동거리로 감속 구간과 제동 waypoint를 다시 정한다.

## Fail-safe

| 조건 | 동작 |
|---|---|
| 목표 경로·속도 0.35초 초과 | 속도 0, 브레이크 요청 |
| 경로 2점 미만 또는 confidence 0 | 속도 0, 브레이크 요청 |
| Route projection 무효 | Recovery 감속·정지 |
| MANUAL | 자율 구동 대기, 현재 조향 동기화 |
| E-Stop·AEB | 즉시 정지 요청 |

## 진단

`/diagnostics`의 `kaiev26_motion_control/adaptive_lookahead`에서 확인한다.

| 값 | 의미 |
|---|---|
| `phase`, `lookahead_m` | 경로 단계와 적용 Ld |
| `preview_curvature_1pm`, `curve_severity` | 코너 형상과 감속 강도 |
| `curvature_speed_limit_mps` | 곡률 속도 상한 |
| `stanley_*` | CTE·헤딩·게인·보정항 |
| `hybrid_*` | 혼합 단계와 PP 비율 |
| `ff_*` | FF 곡률과 조향 |
| `straight_corridor_*` | 직선 완화 적용 여부와 비율 |
| `actual_steering_rate_degps` | 실제 조향속도 추정값, 진단 전용 |
| `steering_tracking_error_deg` | 목표·실제 조향 차이, 진단 전용 |

라바콘 경로는 별도 [`kaiev26_aeb`](../kaiev26_aeb/README.md)가 추종한다.
