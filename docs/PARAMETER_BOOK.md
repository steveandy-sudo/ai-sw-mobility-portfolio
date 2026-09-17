# Decision Parameter Book

현장에서 자주 확인하거나 조정하는 값만 정리한다. 전체 기본값은 각 YAML과 launch 파일이
기준이다.

## 기준 파일

| 파일 | 범위 |
|---|---|
| `kaiev26_decision/config/decision_pipeline.yaml` | 입력, Route, 미션, 회피·복귀, AEB |
| `kaiev26_decision/config/*_policy.yaml` | Stage, Event, 제동 waypoint |
| `kaiev26_decision/config/*_landmarks.yaml` | 정지선·신호등 지도 위치 |
| `kaiev26_decision/waypoints/kcity_*_route.yaml` | 전역경로 |
| `kaiev26_motion_control/config/motion_control.yaml` | 횡제어, 속도 계획, 최종 제동 |
| `kaiev26_aeb/config/perception.yaml` | 라바콘 인지 |
| `kaiev26_aeb/config/tracking.yaml` | 라바콘 추종·빨간선 제동 |

## 변경 원칙

- 거리 `m`, 속도 `m/s`, 가속도 `m/s^2`, 각도 `rad`, 시간 `s`를 사용한다.
- `wheelbase`, 차폭, 오버행, 최대 조향각은 실측값이다.
- 한 번에 한 종류만 바꾸고 동일 조건 MCAP으로 비교한다.
- 정지·AEB 값은 저속 실차 시험 후 올린다.

## Decision 입력·Route

### Perception Gateway

| 파라미터 | 값 | 의미 |
|---|---:|---|
| `publish_rate_hz` | `30.0` | SceneSummary 발행 주기 |
| `stale_data_sec` | `0.35` | 인지 stale 기준 |
| `obstacle_detect_distance_m` | `20.0` | 전방 장애물 범위 |
| `obstacle_path_half_width_m` | `1.4` | 경로 주변 장애물 폭 |
| `traffic_light_map_match_distance_m` | `2.0` | 지도 신호등 연결 거리 |

### Route Manager

| 파라미터 | 값 | 의미 |
|---|---:|---|
| `publish_rate_hz` | `20.0` | RouteContext 발행 주기 |
| `lookahead_distance_m` | `18.0` | 로컬 경로 길이 |
| `local_path_spacing_m` | `1.0` | 로컬 경로 간격 |
| `max_projection_distance_m` | `15.0` | 경로 정합 최대 거리 |
| `progress_search_window_m` | `35.0` | 이전 진행점 주변 검색 범위 |
| `start_snap_distance_m` | `8.0` | 시작 구간 위치 snap 범위 |
| `start_snap_heading_error_rad` | `0.8` | 시작 snap 헤딩 허용값 |
| `initial_projection_max_heading_error_rad` | `0.7` | 첫 경로 정합 헤딩 한계 |
| `finish_zone_distance_m` | `4.0` | 종료 Zone 거리 |

## 출발 준비 검사

`test_readiness.py`의 기본값이다. `decision_test.launch.py`는 중간 경로 시작을 허용한다.

| 파라미터 | 값 | 통과 조건 |
|---|---:|---|
| `freshness_timeout_sec` | `1.0` | 필수 입력이 1초 이내 |
| `stable_ready_sec` | `1.0` | 모든 조건 연속 유지 |
| `maximum_position_variance_m2` | `0.25` | 위치 covariance X·Y 각각 이하 |
| `maximum_velocity_variance_m2ps2` | `1.0` | 속도 covariance 이하 |
| `heading_initialization_speed_mps` | `0.5` | 이 속도 이상 헤딩 1회 획득 |
| `maximum_start_speed_mps` | `0.2` | AUTO 전환 전 정지 기준 |
| `maximum_cross_track_error_m` | `2.0` | 경로 횡오차 한계 |
| `maximum_heading_error_rad` | `0.524` | 경로 헤딩 오차 30도 이하 |
| `allow_midroute_start` | 시험 시 `true` | 코스 중간 재시작 허용 |

GNSS 위치는 `NavSatStatus.STATUS_GBAS_FIX(status=2)`여야 한다. `/drive/status`,
`/steering/status`, `/brake/status`, Route와 차량 상태도 최신이어야 한다.

## 미션·정지

| 파라미터 | 값 | 의미 |
|---|---:|---|
| `base_speed_mps` | 예선 `3.2`, 본선 `3.5` | 속도 입력 `0`의 코스 기본값 |
| `degraded_speed_mps` | 최대 `2.5` | 미션 degraded 상한 |
| `stopline_approach_distance_m` | `18.0` | 정지선 상태 진입 범위 |
| `pre_brake_decel_distance_m` | `6.0` | 제동 WP 전 감속 거리 |
| `brake_entry_speed_mps` | `2.0` | 제동 WP 접근 목표속도 |
| `static_stop_stopped_speed_mps` | `0.1` | 실제 정지 판정 |
| `test_stop_hold_sec` | `3.0` | 시험 정지 유지 시간 |
| `traffic_release_confirm_sec` | `0.4` | 진행 신호 연속 확인 시간 |
| `yellow_stop_decel_mps2` | `2.0` | 황색 정지 가능성 계산 감속도 |
| `yellow_reaction_time_sec` | `0.25` | 황색 반응시간 |
| `comfortable_decel_mps2` | `1.4` | 일반 정지 가능성 계산 감속도 |
| `vehicle_front_overhang_m` | `1.98` | 후륜축에서 앞범퍼 거리 |
| `stopline_clearance_m` | `1.5` | 정지선 여유거리 |

Policy의 모든 Event는 `approach_distance_m=30.0`이다. 신호 Event의
`commit_distance_m=6.0`이며, 실제 제동 위치는 `brake_trigger_waypoint`가 정한다.

| 코스 | 제동 waypoint |
|---|---|
| 예선 | `203, 244, 296, 337` |
| 본선 | `98, 153, 289, 346, 448` |

## 회피·REJOIN

| 파라미터 | 값 |
|---|---:|
| `obstacle_slow_speed_mps` | `2.2` |
| `obstacle_return_speed_mps` | `2.6` |
| `obstacle_clearance_m` | `0.30` |
| `obstacle_side_deadband_m` | `0.30` |
| `obstacle_preferred_side` | `left` |
| `obstacle_transition_length_m` | `10.0` |
| `obstacle_return_length_m` | `10.0` |
| `route_rejoin_cross_track_m` | `0.8` |
| `route_rejoin_heading_error_rad` | `0.45` |
| `route_rejoin_exit_cross_track_m` | `0.30` |
| `route_rejoin_exit_heading_error_rad` | `0.15` |
| `rejoin_speed_min_mps` / `max_mps` | `1.4 / 2.6` |

## Decision AEB

경기 `avoidance/all` mode의 객체 기반 AEB다. 라바콘 검차 AEB와 다르다.

| 파라미터 | 값 |
|---|---:|
| `aeb_stale_objects_sec` | `0.25` |
| `aeb_obstacle_path_half_width_m` | `1.35` |
| `aeb_warning_ttc_sec` | `2.0` |
| `aeb_brake_ttc_sec` | `0.8` |
| `aeb_warning_distance_m` | `5.0` |
| `aeb_emergency_distance_m` | `1.4` |
| `aeb_release_clear_ticks` | `8` |

## Motion Control 공통

| 파라미터 | 값 | 의미 |
|---|---:|---|
| `control_rate_hz` | `50.0` | 명령 계산 주기 |
| `target_stale_sec` | `0.35` | 목표 입력 timeout |
| `wheelbase_m` | `1.2991` | 축거 |
| `max_steering_rad` | `0.4363` | 최대 조향 약 25도 |
| `steering_rate_limit_radps` | `1.0472` | 조향 변화율 약 60도/s |
| `lateral_controller` | `pure_pursuit` | `pure_pursuit`, `stanley`, `pp_stanley`, `ff_stanley` |
| `curvature_speed_min_mps` | `3.2` | 강한 코너 속도 상한 |
| `max_accel_mps2` | `1.0` | 일반 가속 제한 |
| `max_decel_mps2` | `2.0` | 일반 감속 제한 |
| `signal_release_accel_mps2` | `2.0` | 신호 재출발 가속 제한 |
| `signal_release_boost_speed_mps` | `3.8` | 재출발 boost 종료 속도 |

### 직선 허용 통로

| 파라미터 | 값 |
|---|---:|
| `straight_corridor_preview_time_s` | `1.2` |
| `straight_corridor_preview_min_m` / `max_m` | `4.0 / 9.0` |
| `straight_corridor_deadband_m` / `full_m` | `0.08 / 0.25` |
| `straight_heading_deadband_rad` / `full_rad` | `0.01396 / 0.03491` (0.8/2도) |

실제 조향속도와 추종 오차는 진단 전용이다. 목표각 예측·선행 제한·오차 기반 감속에는
사용하지 않는다.

### Pure Pursuit

| 파라미터 | 값 |
|---|---:|
| `lookahead_base_m` / `gain_s` | `4.0 / 0.9` |
| `lookahead_min_m` / `max_m` | `3.6 / 9.5` |
| `lookahead_curve_start_1pm` / `exit_1pm` | `0.015 / 0.010` |
| `lookahead_curve_full_1pm` | `0.070` |
| `lookahead_curve_speed_gain_s` | `0.30` |
| `lookahead_curve_minimum_samples` | `3` |
| `lookahead_near_window_m` / `preview_distance_m` | `4.0 / 12.0` |
| `lookahead_shorten_rate_mps` / `lengthen_rate_mps` | `8.0 / 8.0` |

```text
straight_Ld = clamp(4.0 + 0.9 * speed, 3.6, 9.5)
curve_floor = clamp(3.6 + 0.30 * speed, 3.6, straight_Ld)
```

### Stanley·Hybrid·FF

| 파라미터 | 직선 / 코너 |
|---|---:|
| `stanley_heading_gain` | `0.30 / 0.45` |
| `stanley_cross_track_gain` | `0.25 / 0.35` |
| `stanley_heading_correction_limit_rad` | `0.10 / 0.16` |
| `stanley_cross_track_correction_limit_rad` | `0.05 / 0.07` |
| `stanley_heading_window_m` | `8.0 / 6.0` |

| 파라미터 | 값 |
|---|---:|
| `stanley_softening_speed_mps` | `3.0` |
| `stanley_curve_preview_time_s` / `max_m` | `0.18 / 3.5` |
| `hybrid_pp_straight_weight` | `0.0` |
| `hybrid_pp_approach_weight` | `0.65` |
| `hybrid_pp_turning_weight` | `0.35` |
| `hybrid_pp_exit_weight` | `0.10` |
| `hybrid_blend_rate_per_s` | `6.0` |
| `ff_stanley_feedback_scale` | `0.50` |
| `ff_preview_min_m` / `max_m` | `2.0 / 4.0` |
| `ff_curvature_limit_1pm` | `0.20` |
| `ff_steering_limit_rad` | `0.30` |

### 계획 정지

| 파라미터 | 값 |
|---|---:|
| `planned_stop_brake_distance_m` | `2.0` |
| `stop_speed_epsilon_mps` | `0.05` |
| `stopped_steering_hold_speed_mps` | `0.10` |
| `planned_stop_steering_align_distance_m` | `2.0` |
| `stop_release_steering_resume_speed_mps` | `0.30` |

## 라바콘 검차

| 항목 | PP | Stanley |
|---|---:|---:|
| 목표속도 | `5.0 km/h` | `5.0 km/h` |
| Ld | `7.0 m` | - |
| heading / CTE gain | - | `0.4 / 0.4` |
| 최대 조향 | `0.4363 rad` | `0.4363 rad` |
| 조향 변화율 | `1.0472 rad/s` | `1.0472 rad/s` |
| 가속도 | `2.0 m/s^2` | `2.0 m/s^2` |
| 경로 timeout | `0.35 s` | `0.35 s` |
| 차량 상태 timeout | `0.25 s` | `0.25 s` |
| 빨간선 확인 | `2 frames` | `2 frames` |

인지 공통값은 `10 Hz`, confidence `0.5`, image size `800`이다. Fusion은 전방 25 m,
YOLO-only는 15 m를 사용한다.

## Launch 입력

| Launch | 주요 인자 |
|---|---|
| `decision.launch.py` | `course`, `cruise_speed_mps`, `lateral_controller`, `use_sim_time` |
| `decision_test.launch.py` | `case`, `speed_mps`, `lateral_controller`, Case 8 스텝 설정 |
| Gazebo `decision_test.launch.py` | 위 값 + `headless`, driver·bridge 설정 |
| `aeb_sim.launch.py` | `scenario`, `target_speed_kph`, `model_path`, `headless` |

`speed_mps=0`은 예선 `3.2`, 본선 `3.5`, Case 8 `1.0 m/s`를 선택한다. 직접 입력 범위는
`0.1~15.0 m/s`다.

## 수정 후 확인

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  kaiev26_decision kaiev26_motion_control kaiev26_aeb
source install/setup.bash
```

`경로 정합 -> 곡률·Ld -> 조향 -> 속도 -> 정지` 순서로 확인한다.
