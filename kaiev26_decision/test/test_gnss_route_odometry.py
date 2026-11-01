from pathlib import Path
from types import SimpleNamespace
import math

from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Point
from kaiev26_decision.test_readiness import InputState, RouteReadinessNode
from kaiev26_decision.route_zone_manager_node import (
    PolylineRoute,
    RouteWaypoint,
    MAP_GEOMETRY_KEYS,
    RouteZoneManagerNode,
    load_landmark_stoplines,
    load_map_geometry,
    normalize_angle,
    parse_route_stoplines,
    resolve_package_path,
)
from kaiev26_decision.zone_policy import load_zone_policy
from kaiev26_msgs.msg import RouteContext, SteeringActuatorStatus, VehicleState
from std_msgs.msg import Float32
import pytest
import yaml


WAYPOINT_DIR = Path(__file__).resolve().parents[1] / 'waypoints'
CONFIG_DIR = Path(__file__).resolve().parents[1] / 'config'


@pytest.mark.parametrize(
    ('filename', 'route_name', 'source_bag', 'point_count'),
    [
        (
            'kcity_quali_route.yaml',
            'kcity_quali_20260905',
            '11_28_57_qualifying_global_route',
            597,
        ),
        (
            'kcity_final_route.yaml',
            'kcity_final_20260905',
            '11_56_27_final_global_route',
            774,
        ),
    ],
)
def test_official_kcity_route_contract(
    filename: str,
    route_name: str,
    source_bag: str,
    point_count: int,
) -> None:
    document = yaml.safe_load(
        (WAYPOINT_DIR / filename).read_text(encoding='utf-8')
    )
    waypoints = document['waypoints']

    assert document['route_name'] == route_name
    assert document['source_bag'] == source_bag
    assert document['reference_point'] == 'base_footprint'
    assert document['map_origin']['latitude_deg'] == pytest.approx(37.240157)
    assert document['map_origin']['longitude_deg'] == pytest.approx(126.773747)
    assert len(waypoints) == point_count
    assert all(waypoint['z'] == pytest.approx(0.0) for waypoint in waypoints)
    assert waypoints[-1]['zone'] == 'FINISH_ZONE'
    if filename == 'kcity_quali_route.yaml':
        assert document['finish_waypoint_index'] == 596


def test_official_routes_share_extended_start_and_finish() -> None:
    qualifying = yaml.safe_load(
        (WAYPOINT_DIR / 'kcity_quali_route.yaml').read_text(encoding='utf-8')
    )['waypoints']
    final = yaml.safe_load(
        (WAYPOINT_DIR / 'kcity_final_route.yaml').read_text(encoding='utf-8')
    )['waypoints']

    # Both go on 45 m past the shared finish (WP 728 of the final) to the same end point: the final
    # 15 m on 2026-10-24 and 30 m more on 2026-10-25, the qualifying on the final's points (finish_extension).
    shared_finish = final[728]
    assert qualifying[0]['x'] == pytest.approx(final[0]['x'])
    assert qualifying[0]['y'] == pytest.approx(final[0]['y'])
    assert qualifying[551]['x'] == pytest.approx(shared_finish['x'])
    assert qualifying[551]['y'] == pytest.approx(shared_finish['y'])
    assert qualifying[-1]['x'] == pytest.approx(final[-1]['x'])
    assert qualifying[-1]['y'] == pytest.approx(final[-1]['y'])
    assert math.hypot(
        qualifying[10]['x'] - qualifying[0]['x'],
        qualifying[10]['y'] - qualifying[0]['y'],
    ) == pytest.approx(10.0)
    assert math.hypot(
        shared_finish['x'] - final[708]['x'],
        shared_finish['y'] - final[708]['y'],
    ) == pytest.approx(19.993835771057487, abs=1.0e-8)
    assert math.hypot(
        final[-1]['x'] - shared_finish['x'],
        final[-1]['y'] - shared_finish['y'],
    ) == pytest.approx(45.0, abs=1.0e-8)


@pytest.mark.parametrize('course', ['qualifying', 'final'])
def test_mission_brake_triggers_match_extended_route_progress(course: str) -> None:
    route_name = 'kcity_quali_route.yaml' if course == 'qualifying' else 'kcity_final_route.yaml'
    waypoints = yaml.safe_load(
        (WAYPOINT_DIR / route_name).read_text(encoding='utf-8')
    )['waypoints']
    cumulative_s = [0.0]
    for start, end in zip(waypoints, waypoints[1:]):
        cumulative_s.append(
            cumulative_s[-1]
            + math.hypot(end['x'] - start['x'], end['y'] - start['y'])
        )
    policy = load_zone_policy(CONFIG_DIR / f'{course}_policy.yaml')

    for stage in policy.stages:
        event = stage.event
        if event.brake_trigger_waypoint is None:
            continue
        assert event.brake_trigger_route_s == pytest.approx(
            cumulative_s[event.brake_trigger_waypoint], abs=1.0e-6
        )


# 2026-10-25 survey (records/start_line, records/final_line): GNSS antenna on the painted
# start/finish line in the start lane and in the finish lane.
SURVEYED_FINISH_LINE = ((37.238943406, 126.772968818), (37.238965823, 126.772923934))


@pytest.mark.parametrize('course, line_s', [('qualifying', 555.67), ('final', 731.93)])
def test_finish_line_route_s_is_where_the_route_crosses_the_surveyed_line(course: str, line_s: float) -> None:
    route_name = 'kcity_quali_route.yaml' if course == 'qualifying' else 'kcity_final_route.yaml'
    document = yaml.safe_load((WAYPOINT_DIR / route_name).read_text(encoding='utf-8'))
    waypoints, origin = document['waypoints'], document['map_origin']
    finish = load_zone_policy(CONFIG_DIR / f'{course}_policy.yaml').stages[-1].event
    assert finish.event_type == 'FINISH'
    assert finish.finish_line_points == SURVEYED_FINISH_LINE

    radius_m = 6378137.0
    east_scale = radius_m * math.cos(math.radians(origin['latitude_deg']))
    (ax, ay), (bx, by) = (
        (math.radians(lon - origin['longitude_deg']) * east_scale,
         math.radians(lat - origin['latitude_deg']) * radius_m)
        for lat, lon in finish.finish_line_points
    )
    lx, ly = bx - ax, by - ay
    crossings = []
    s = 0.0
    for start, end in zip(waypoints, waypoints[1:]):
        dx, dy = end['x'] - start['x'], end['y'] - start['y']
        length = math.hypot(dx, dy)
        denominator = dx * ly - dy * lx
        t = ((ax - start['x']) * ly - (ay - start['y']) * lx) / denominator
        # u: position along the surveyed segment (0 start lane, 1 finish lane); the road only.
        u = ((ax - start['x']) * dy - (ay - start['y']) * dx) / denominator
        if 0.0 <= t < 1.0 and -1.0 <= u <= 2.0:
            crossings.append(s + t * length)
        s += length
    # The start lane crosses the line 5 m before the route start (s < 0); the finish lane once.
    assert crossings == [pytest.approx(finish.finish_line_route_s, abs=1.0e-4)]
    assert finish.finish_line_route_s == pytest.approx(line_s, abs=0.01)


@pytest.mark.parametrize('course', ['qualifying', 'final'])
def test_kcity_landmarks_take_their_geometry_from_the_map_semantics(course: str) -> None:
    route_name = 'kcity_quali_route.yaml' if course == 'qualifying' else 'kcity_final_route.yaml'
    document = yaml.safe_load((WAYPOINT_DIR / route_name).read_text(encoding='utf-8'))
    route = PolylineRoute([RouteWaypoint(x=p['x'], y=p['y']) for p in document['waypoints']])
    landmark_path = CONFIG_DIR / f'{course}_landmarks.yaml'
    data = yaml.safe_load(landmark_path.read_text(encoding='utf-8'))
    assert data['map_semantics'] == 'package://kaiev26_maps/maps/kcity/kcity_course_semantics.yaml'
    for item in data['landmarks']:
        assert not set(item) & set(MAP_GEOMETRY_KEYS)
    stoplines, semantics_path = load_landmark_stoplines(route, landmark_path)
    geometry = load_map_geometry(semantics_path)
    assert [item.landmark_id for item in stoplines] == [item['id'] for item in data['landmarks']]
    for stopline in stoplines:
        a, b = geometry.stop_lines[stopline.landmark_id][0], geometry.stop_lines[stopline.landmark_id][-1]
        x, y, _, _ = route.sample_at(stopline.progress_s)
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        # the route point lies on the painted line, inside its extent
        assert ((x - a[0]) * (b[1] - a[1]) - (y - a[1]) * (b[0] - a[0])) / length == pytest.approx(0.0, abs=1e-6)
        assert 0.0 <= ((x - a[0]) * (b[0] - a[0]) + (y - a[1]) * (b[1] - a[1])) / length**2 <= 1.0
        assert stopline.width_m == pytest.approx(length)
        assert stopline.traffic_light_positions == tuple(
            geometry.traffic_lights[light_id] for light_id in stopline.traffic_light_ids
        )


def test_landmarks_that_repeat_map_geometry_are_rejected() -> None:
    document = yaml.safe_load((WAYPOINT_DIR / 'kcity_quali_route.yaml').read_text(encoding='utf-8'))
    route = PolylineRoute([RouteWaypoint(x=p['x'], y=p['y']) for p in document['waypoints']])
    data = yaml.safe_load((CONFIG_DIR / 'qualifying_landmarks.yaml').read_text(encoding='utf-8'))
    geometry = load_map_geometry(resolve_package_path(data['map_semantics']))
    data['landmarks'][1]['traffic_light_positions'] = [{'x': 0.0, 'y': 0.0, 'z': 5.0}]
    with pytest.raises(ValueError, match='traffic_light_positions come from map_semantics'):
        parse_route_stoplines(route, data, map_geometry=geometry)
    data['landmarks'][1].pop('traffic_light_positions')
    data['landmarks'][1]['id'] = 'stop_line_manual_999'
    with pytest.raises(ValueError, match='not a stop line of map_semantics'):
        parse_route_stoplines(route, data, map_geometry=geometry)


@pytest.mark.parametrize(('course', 'expected_gaps'), [
    ('qualifying', [14.13159663, 9.989220753, 14.635398181, 11.866046618]),
    ('final', [4.907796477, 14.259762058, 14.52853385, 17.082100384, 21.495582441]),
])
def test_predecel_points_remain_inside_their_stage_and_event_window(
    course, expected_gaps,
) -> None:
    route_name = 'kcity_quali_route.yaml' if course == 'qualifying' else 'kcity_final_route.yaml'
    document = yaml.safe_load((WAYPOINT_DIR / route_name).read_text())
    route = PolylineRoute([
        RouteWaypoint(x=point['x'], y=point['y']) for point in document['waypoints']
    ])
    policy = load_zone_policy(CONFIG_DIR / f'{course}_policy.yaml')
    stoplines, _ = load_landmark_stoplines(route, CONFIG_DIR / f'{course}_landmarks.yaml')
    progress = {item.landmark_id: item.progress_s for item in stoplines}
    parameters = yaml.safe_load((CONFIG_DIR / 'decision_pipeline.yaml').read_text())[
        'kaiev26_main_planning_engine']['ros__parameters']
    distance = parameters['pre_brake_decel_distance_m']
    assert distance == 8.0
    stage_start = 0.0
    for stage, expected_gap in zip(policy.stages[:len(expected_gaps)], expected_gaps):
        event = stage.event
        line_s = progress[event.stop_line_id]
        onset_s = event.brake_trigger_route_s - distance
        assert line_s - event.brake_trigger_route_s == pytest.approx(expected_gap, abs=1e-6)
        assert stage_start < onset_s < event.brake_trigger_route_s < line_s
        assert line_s - onset_s < event.approach_distance_m
        if event.event_type == 'SIGNAL_OBEY':
            assert event.commit_distance_m == 8.0
        stage_start = line_s


# The painted lines of the map semantics (2026-10-15 camera survey) share their route
# intersection with the runtime landmarks; expected_margin is the travel from the brake waypoint
# until the front corner of the car reaches the 0.35 m wide paint.
@pytest.mark.parametrize(('course', 'stage_id', 'expected_margin'), [
    ('qualifying', 'Q_STAGE_1', 11.957119),
    ('qualifying', 'Q_STAGE_2', 7.833558),
    ('qualifying', 'Q_STAGE_3', 12.447740),
    ('qualifying', 'Q_STAGE_4', 9.700364),
    ('final', 'F_STAGE_1', 2.743631),
    ('final', 'F_STAGE_2', 12.100428),
    ('final', 'F_STAGE_3', 12.328836),
    ('final', 'F_STAGE_4', 14.914237),
    ('final', 'F_STAGE_5', 19.338086),
])
def test_existing_brake_waypoints_match_corrected_stopline_front_clearance(
    course, stage_id, expected_margin,
) -> None:
    filename = 'kcity_quali_route.yaml' if course == 'qualifying' else 'kcity_final_route.yaml'
    document = yaml.safe_load((WAYPOINT_DIR / filename).read_text(encoding='utf-8'))
    route = PolylineRoute([RouteWaypoint(x=p['x'], y=p['y']) for p in document['waypoints']])
    policy = load_zone_policy(CONFIG_DIR / f'{course}_policy.yaml')
    event = next(stage.event for stage in policy.stages if stage.stage_id == stage_id)
    parameters = yaml.safe_load((CONFIG_DIR / 'decision_pipeline.yaml').read_text())[
        'kaiev26_main_planning_engine']['ros__parameters']
    line = load_map_geometry(resolve_package_path(yaml.safe_load(
        (CONFIG_DIR / f'{course}_landmarks.yaml').read_text())['map_semantics'])).stop_lines[event.stop_line_id]
    a, b = line[0], line[-1]
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    nx, ny = -(b[1] - a[1]) / length, (b[0] - a[0]) / length
    heading = route.heading_at(event.brake_trigger_route_s)
    if nx * math.cos(heading) + ny * math.sin(heading) < 0:
        nx, ny = -nx, -ny

    landmarks, semantics_path = load_landmark_stoplines(route, CONFIG_DIR / f'{course}_landmarks.yaml')
    landmark = next(item for item in landmarks if item.landmark_id == event.stop_line_id)
    lx, ly, _, _ = route.sample_at(landmark.progress_s)
    assert (lx - a[0]) * nx + (ly - a[1]) * ny == pytest.approx(0.0, abs=1e-7)
    fraction = ((lx - a[0]) * (b[0] - a[0]) + (ly - a[1]) * (b[1] - a[1])) / length**2
    assert 0.0 <= fraction <= 1.0

    def signed_front_corner(travel):
        progress = event.brake_trigger_route_s + travel
        x, y, _, _ = route.sample_at(progress)
        yaw = route.heading_at(progress)
        dx, dy = math.cos(yaw), math.sin(yaw)
        front = parameters['vehicle_front_overhang_m']
        half_width = parameters['vehicle_width_m'] / 2.0
        return ((x + front * dx - a[0]) * nx + (y + front * dy - a[1]) * ny
                + half_width * abs(-dy * nx + dx * ny) + 0.35 / 2.0)

    assert expected_margin > 0.0
    assert signed_front_corner(0.0) < 0.0
    assert signed_front_corner(expected_margin) == pytest.approx(0.0, abs=0.001)
    assert signed_front_corner(expected_margin - 0.01) < 0.0
    assert signed_front_corner(expected_margin + 0.01) > 0.0


@pytest.mark.parametrize(
    ('course', 'expected_maneuvers'),
    [
        ('qualifying', {'Q_STAGE_2': 'STRAIGHT', 'Q_STAGE_4': 'STRAIGHT'}),
        (
            'final',
            {
                'F_STAGE_1': 'STRAIGHT',
                'F_STAGE_2': 'STRAIGHT',
                'F_STAGE_3': 'LEFT',
                'F_STAGE_4': 'LEFT',
                'F_STAGE_5': 'STRAIGHT',
            },
        ),
    ],
)
def test_recorded_route_preserves_signal_identity_position_and_maneuver(
    course: str,
    expected_maneuvers: dict[str, str],
) -> None:
    route_filename = (
        'kcity_quali_route.yaml'
        if course == 'qualifying'
        else 'kcity_final_route.yaml'
    )
    route_document = yaml.safe_load(
        (WAYPOINT_DIR / route_filename).read_text(encoding='utf-8')
    )
    route = PolylineRoute(
        [
            RouteWaypoint(
                x=float(point['x']),
                y=float(point['y']),
                z=float(point.get('z', 0.0)),
            )
            for point in route_document['waypoints']
        ]
    )
    policy = load_zone_policy(CONFIG_DIR / f'{course}_policy.yaml')
    landmarks, _ = load_landmark_stoplines(route, CONFIG_DIR / f'{course}_landmarks.yaml')
    route.set_zone_ranges(
        policy.zone_ranges(
            {item.landmark_id: item.progress_s for item in landmarks},
            route.total_length,
        )
    )
    node = object.__new__(RouteZoneManagerNode)
    node.route = route
    node.zone_policy = policy
    node.stoplines = landmarks
    node.validate_policy_controls()

    for stage_id, expected_maneuver in expected_maneuvers.items():
        stage = next(item for item in policy.stages if item.stage_id == stage_id)
        stopline = next(
            item
            for item in landmarks
            if item.landmark_id == stage.event.stop_line_id
        )
        stage_range = next(
            item for item in route.zone_spans if item.zone == stage_id
        )
        progress_s = 0.5 * (stage_range.start_s + stage_range.end_s)

        assert node.next_maneuver(progress_s) == expected_maneuver
        assert stopline.traffic_light_positions


class CapturePublisher:
    """Remember the most recently published test message."""

    def __init__(self) -> None:
        self.message = None

    def publish(self, message) -> None:
        self.message = message


def readiness_results(publisher_counts: dict[str, int]):
    now_ns = 2_000_000_000
    node = SimpleNamespace(
        inputs={
            topic: InputState(now_ns, True, 'healthy')
            for topic in RouteReadinessNode.INPUT_NAMES
        },
        freshness_timeout_ns=1_000_000_000,
        UNIQUE_PUBLISHER_TOPICS=RouteReadinessNode.UNIQUE_PUBLISHER_TOPICS,
        count_publishers=lambda topic: publisher_counts.get(topic, 1),
    )
    return RouteReadinessNode.current_results(node, now_ns)


def test_route_readiness_accepts_exactly_one_publisher_per_control_topic() -> None:
    results = readiness_results({})

    assert all(valid for valid, _detail in results.values())


class FixedClock:
    """Return a deterministic ROS timestamp for published diagnostics."""

    class FixedNow:
        @staticmethod
        def to_msg() -> Time:
            return Time(sec=12, nanosec=345)

    @staticmethod
    def now():
        return FixedClock.FixedNow()


def test_route_readiness_publishes_good_and_runtime_fault_diagnostics() -> None:
    status_pub = CapturePublisher()
    node = SimpleNamespace(
        preflight_passed=True,
        status_pub=status_pub,
        get_clock=lambda: FixedClock(),
    )
    results = {
        '/gnss/fix': (True, 'healthy'),
        '/steering/status': (False, 'waiting for fresh message'),
    }

    RouteReadinessNode.publish_status(node, results, runtime_ready=False)

    message = status_pub.message
    assert message.header.stamp.sec == 12
    assert message.status[0].name == 'PRE-FLIGHT'
    assert message.status[0].level == DiagnosticStatus.ERROR
    assert message.status[0].message == 'FAULT'
    assert message.status[1].level == DiagnosticStatus.OK
    assert message.status[2].level == DiagnosticStatus.ERROR
    assert message.status[2].values[0].value == 'waiting for fresh message'


def test_steering_tracking_fault_is_a_non_blocking_recording_warning() -> None:
    captured = {}
    node = SimpleNamespace(
        update_input=lambda name, valid, detail: captured.update(
            {name: (valid, detail)}
        ),
    )
    status = SteeringActuatorStatus()
    status.communication_ok = True
    status.feedback_valid = True
    status.tracking_error_fault = True

    RouteReadinessNode.on_steering(node, status)

    valid, detail = captured['/steering/status']
    assert valid is True
    assert '시험과 기록은 계속 진행' in detail


def test_estop_pause_does_not_fail_vehicle_or_steering_readiness() -> None:
    captured = {}
    node = SimpleNamespace(
        preflight_passed=False,
        maximum_start_speed_mps=0.2,
        update_input=lambda name, valid, detail: captured.update({name: (valid, detail)}),
    )
    RouteReadinessNode.on_vehicle(
        node, VehicleState(mode=VehicleState.MODE_MANUAL, estop_active=True))
    status = SteeringActuatorStatus()
    status.communication_ok = True
    status.feedback_valid = True
    status.estop_active = True
    RouteReadinessNode.on_steering(node, status)

    assert captured['/vehicle/state'] == (True, 'MANUAL, E-Stop PAUSE, 차량 상태 통과')
    assert captured['/steering/status'] == (True, 'E-Stop으로 조향 disable')

    status.communication_ok = False
    RouteReadinessNode.on_steering(node, status)
    assert captured['/steering/status'][0] is False


@pytest.mark.parametrize('publisher_count', [0, 2])
def test_route_readiness_rejects_missing_or_duplicate_planner(publisher_count) -> None:
    results = readiness_results({'/planning/command': publisher_count})

    assert results['/planning/command'][0] is False
    assert f'publishers={publisher_count}' in results['/planning/command'][1]


def test_route_readiness_allows_valid_midroute_restart_when_enabled() -> None:
    captured = {}
    node = SimpleNamespace(
        preflight_passed=False,
        allow_midroute_start=True,
        maximum_start_progress_m=35.0,
        update_input=lambda name, valid, detail: captured.update(
            {name: (valid, detail)}
        ),
    )
    route = RouteContext()
    route.route_projection_valid = True
    route.progress_s = 250.0
    route.cross_track_error = 0.2
    route.heading_error = 0.1
    route.local_path_points = [Point(x=1.0), Point(x=2.0)]

    RouteReadinessNode.on_route(node, route)

    assert captured['/planning/route_context'][0] is True


@pytest.mark.parametrize(
    ('cross_track_error_m', 'heading_error_deg'),
    [(2.1, 0.0), (0.0, 31.0), (5.0, 90.0)],
)
def test_route_readiness_records_large_route_error_without_blocking(
    cross_track_error_m: float,
    heading_error_deg: float,
) -> None:
    captured = {}
    node = SimpleNamespace(
        preflight_passed=True,
        allow_midroute_start=True,
        maximum_start_progress_m=35.0,
        update_input=lambda name, valid, detail: captured.update(
            {name: (valid, detail)}
        ),
    )
    route = RouteContext()
    route.route_projection_valid = True
    route.cross_track_error = cross_track_error_m
    route.heading_error = math.radians(heading_error_deg)
    route.local_path_points = [Point(x=1.0), Point(x=2.0)]

    RouteReadinessNode.on_route(node, route)

    valid, detail = captured['/planning/route_context']
    assert valid is True
    assert f'{cross_track_error_m:+.2f} m' in detail
    assert f'{heading_error_deg:+.1f}°' in detail


@pytest.mark.parametrize(
    ('cross_track_error_m', 'heading_error_deg', 'expected_reason'),
    [
        (9.89, -2.0, '출발 횡오차 초과'),
        (5.27, 179.6, '출발 헤딩 오차 초과'),
        (1.28, -18.3, None),
        (3.0, 45.0, None),
        (-3.01, 0.0, '출발 횡오차 초과'),
        (0.0, -45.01, '출발 헤딩 오차 초과'),
        (math.nan, 0.0, '출발 횡오차 초과'),
        (0.0, math.nan, '출발 헤딩 오차 초과'),
    ],
)
def test_route_readiness_blocks_preflight_off_the_route_or_facing_against_it(
    cross_track_error_m: float,
    heading_error_deg: float,
    expected_reason: str | None,
) -> None:
    # 2026-10-17 trials: READY at 9.9 m / 179.6 deg; the good first trial started at 1.3 m / 18 deg.
    captured = {}
    node = SimpleNamespace(
        preflight_passed=False,
        allow_midroute_start=True,
        maximum_start_progress_m=35.0,
        maximum_start_heading_error_rad=math.radians(45.0),
        maximum_start_cross_track_m=3.0,
        update_input=lambda name, valid, detail: captured.update(
            {name: (valid, detail)}
        ),
    )
    route = RouteContext()
    route.route_projection_valid = True
    route.progress_s = 5.0
    route.cross_track_error = cross_track_error_m
    route.heading_error = math.radians(heading_error_deg)
    route.local_path_points = [Point(x=1.0), Point(x=2.0)]

    RouteReadinessNode.on_route(node, route)

    valid, detail = captured['/planning/route_context']
    assert valid is (expected_reason is None)
    if expected_reason is not None:
        assert expected_reason in detail


@pytest.mark.parametrize(
    ('age_sec', 'preflight_passed', 'expected_valid', 'expected_text'),
    [
        (0.024, False, True, '0.02 s'),
        (0.15, False, True, '0.15 s'),
        # 2026-10-24 10:10 final: the planner saw 0.25-0.85 s from the start and coasted at 0.
        (0.51, False, False, 'Decision을 다시 실행'),
        (math.nan, False, False, '경로 입력 지연'),
        (-1.0, False, False, '아직 받지 못함'),
        (0.7, True, True, '출발 뒤 기록만'),
    ],
)
def test_route_readiness_blocks_preflight_while_the_planner_route_input_lags(
    age_sec: float,
    preflight_passed: bool,
    expected_valid: bool,
    expected_text: str,
) -> None:
    captured = {}
    node = SimpleNamespace(
        preflight_passed=preflight_passed,
        maximum_start_route_context_age_sec=0.15,
        update_input=lambda name, valid, detail: captured.update(
            {name: (valid, detail)}
        ),
    )

    RouteReadinessNode.on_route_age(node, Float32(data=age_sec))

    valid, detail = captured['/debug/route_context_age']
    assert valid is expected_valid
    assert expected_text in detail


def test_route_readiness_without_a_route_ignores_the_planner_route_age() -> None:
    captured = {}
    node = SimpleNamespace(
        require_route=False,
        update_input=lambda name, valid, detail: captured.update({name: (valid, detail)}),
    )

    RouteReadinessNode.on_route_age(node, Float32(data=0.9))

    assert captured == {}


@pytest.mark.parametrize(
    ('age_sec', 'preflight_passed', 'expected_valid', 'expected_text'),
    [
        (0.026, False, True, '0.03 s'),
        (0.15, False, True, '0.15 s'),
        # 2026-10-25 09:36 qualifying: the planner saw 0.14-0.64 s from the start and braked at AUTO.
        (0.435, False, False, 'Decision을 다시 실행'),
        (math.nan, False, False, '차량 상태 입력 지연'),
        (-1.0, False, False, '아직 받지 못함'),
        (0.7, True, True, '출발 뒤 기록만'),
    ],
)
def test_route_readiness_blocks_preflight_while_the_planner_vehicle_input_lags(
    age_sec: float,
    preflight_passed: bool,
    expected_valid: bool,
    expected_text: str,
) -> None:
    captured = {}
    node = SimpleNamespace(
        preflight_passed=preflight_passed,
        maximum_start_vehicle_state_age_sec=0.15,
        update_input=lambda name, valid, detail: captured.update(
            {name: (valid, detail)}
        ),
    )

    RouteReadinessNode.on_vehicle_age(node, Float32(data=age_sec))

    valid, detail = captured['/debug/vehicle_state_age']
    assert valid is expected_valid
    assert expected_text in detail


def test_route_readiness_without_a_route_ignores_the_planner_vehicle_age() -> None:
    captured = {}
    node = SimpleNamespace(
        require_route=False,
        update_input=lambda name, valid, detail: captured.update({name: (valid, detail)}),
    )

    RouteReadinessNode.on_vehicle_age(node, Float32(data=0.9))

    assert captured == {}


def out_and_back_route() -> PolylineRoute:
    # Out-and-back route whose return leg (s 100..200) runs 5 m beside the outbound leg, as the
    # K-City finish leg runs beside the start leg.
    return PolylineRoute(
        [RouteWaypoint(float(x), 0.0) for x in range(0, 101, 10)]
        + [RouteWaypoint(float(x), 5.0) for x in range(100, -1, -10)]
    )


def tracked_node(last_progress_s: float, committed: bool = False) -> RouteZoneManagerNode:
    node = object.__new__(RouteZoneManagerNode)
    node.route = out_and_back_route()
    node.last_progress_s = last_progress_s
    node.progress_search_window_m = 35.0
    node.max_projection_distance_m = 15.0
    node.local_projection_max_heading_error_rad = math.pi / 2.0
    node.initial_projection_max_heading_error_rad = 0.7
    node.initial_projection_heading_weight_m = 4.0
    node.projection_heading_min_speed_mps = 1.0
    node.progress_committed = committed
    node.vehicle_auto = False
    node.get_logger = lambda: SimpleNamespace(info=lambda *args, **kwargs: None)
    return node


def test_tracked_projection_leaves_a_route_leg_that_points_against_the_vehicle() -> None:
    # 2026-10-17 trials: the progress was latched on the return leg (s = 155) while the car faced
    # it; the car then U-turned at about 2 m/s onto the outbound leg at s = 50, outside the search
    # window, and must not stay matched to the reversed leg 5 m away.
    node = tracked_node(155.0)

    on_outbound_leg = RouteZoneManagerNode.project_route(node, 50.0, 0.0, 0.0, 2.0)
    still_on_return_leg = RouteZoneManagerNode.project_route(node, 50.0, 5.0, math.pi, 2.0)

    assert on_outbound_leg.progress_s == pytest.approx(50.0)
    assert still_on_return_leg.progress_s == pytest.approx(155.0)
    assert not RouteZoneManagerNode.projection_valid(
        node, still_on_return_leg, normalize_angle(still_on_return_leg.yaw - 0.0), 2.0
    )


@pytest.mark.parametrize(
    ('committed', 'speed_mps'),
    [
        # Standing, before or after the commit: the heading flips near standstill (09:28 at Q1).
        (False, 0.0),
        (True, 0.0),
        # Committed and pulling away: a 0.2 s flip at 2.2 m/s (17:16 at 38.9 s).
        (True, 2.2),
    ],
)
def test_tracked_projection_keeps_its_leg_when_the_heading_cannot_be_trusted(
    committed: bool,
    speed_mps: float,
) -> None:
    # The car drives the outbound leg at s = 50 and the localization heading reads 180 deg off.
    node = tracked_node(50.0, committed=committed)

    projection = RouteZoneManagerNode.project_route(node, 50.0, 0.0, math.pi, speed_mps)
    heading_error = normalize_angle(projection.yaw - math.pi)

    assert projection.progress_s == pytest.approx(50.0)
    assert abs(heading_error) == pytest.approx(math.pi)
    assert RouteZoneManagerNode.projection_valid(node, projection, heading_error, speed_mps)


def test_progress_commits_once_the_car_drives_in_auto_along_the_route() -> None:
    node = tracked_node(50.0)

    RouteZoneManagerNode.commit_progress(node, 0.0, 2.0)  # MANUAL
    node.vehicle_auto = True
    RouteZoneManagerNode.commit_progress(node, 0.0, 0.5)  # AUTO, still slow
    RouteZoneManagerNode.commit_progress(node, math.pi, 2.0)  # AUTO on a reversed leg
    assert not node.progress_committed

    RouteZoneManagerNode.commit_progress(node, 0.2, 1.2)
    assert node.progress_committed


# 2026-10-17 08:50-08:57: standing at the start with the heading about 180 deg off, the cold
# search matched the finish leg 5 m beside and the progress stayed there. situation: start
# mode, vehicle heading -> progress (m) and whether the projection is valid.
START_MODE_CASES = [
    pytest.param("from_start", math.pi, 2.0, True, id="from-start-ignores-a-flipped-heading"),
    pytest.param("from_start", 0.0, 2.0, True, id="from-start-aligned"),
    pytest.param("resume", math.pi, 203.0, True, id="resume-matches-the-finish-leg"),
    pytest.param("resume", 0.0, 2.0, True, id="resume-aligned"),
]


@pytest.mark.parametrize(("start_mode", "yaw", "progress_s", "valid"), START_MODE_CASES)
def test_start_mode_decides_the_first_projection(start_mode, yaw, progress_s, valid) -> None:
    node = tracked_node(None)
    node.start_mode = start_mode
    node.start_snap_distance_m = 8.0
    node.start_snap_heading_error_rad = 0.8
    node.start_snap_progress_m = 35.0

    projection = RouteZoneManagerNode.project_route(node, 2.0, 1.0, yaw, 0.0)
    heading_error = normalize_angle(projection.yaw - yaw)

    assert projection.progress_s == pytest.approx(progress_s, abs=0.5)
    assert RouteZoneManagerNode.projection_valid(node, projection, heading_error, 0.0) is valid


def test_from_start_follows_the_car_until_the_progress_commits() -> None:
    node = tracked_node(30.0)
    node.start_mode = "from_start"
    node.start_snap_progress_m = 35.0

    # Driven past the start range before the commit: still on the outbound leg.
    ahead = RouteZoneManagerNode.project_route(node, 60.0, 2.0, math.pi, 2.0)
    assert ahead.progress_s == pytest.approx(60.0)

    node.vehicle_auto = True
    node.last_progress_s = ahead.progress_s
    RouteZoneManagerNode.commit_progress(node, 0.0, 2.0)
    assert node.progress_committed
    assert not RouteZoneManagerNode.starting_from_route_start(node)


def test_cold_projection_uses_heading_to_separate_outbound_and_return_lanes() -> None:
    route = PolylineRoute(
        [
            RouteWaypoint(0.0, 0.0),
            RouteWaypoint(10.0, 0.0),
            RouteWaypoint(10.0, 1.0),
            RouteWaypoint(0.0, 1.0),
        ]
    )

    outbound = route.project_with_heading(5.0, 0.5, 0.0, 0.7, 4.0)
    returning = route.project_with_heading(5.0, 0.5, math.pi, 0.7, 4.0)

    assert outbound.progress_s < 10.0
    assert returning.progress_s > 11.0
