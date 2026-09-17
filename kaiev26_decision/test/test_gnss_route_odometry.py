from pathlib import Path
from types import SimpleNamespace
import math

from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Point
from kaiev26_decision.test_readiness import InputState, RouteReadinessNode
from kaiev26_decision.test_tracking import ConstantRoutePlanNode
from kaiev26_decision.route_zone_manager_node import (
    PolylineRoute,
    RouteWaypoint,
    RouteZoneManagerNode,
    parse_route_stoplines,
)
from kaiev26_decision.zone_policy import load_zone_policy
from kaiev26_msgs.msg import RouteContext, SteeringActuatorStatus
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
            516,
        ),
        (
            'kcity_final_route.yaml',
            'kcity_final_20260905',
            '11_56_27_final_global_route',
            684,
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
        assert document['finish_waypoint_index'] == 515


def test_qualifying_timed_stops_target_confirmed_waypoints() -> None:
    route_document = yaml.safe_load(
        (WAYPOINT_DIR / 'kcity_quali_route.yaml').read_text(encoding='utf-8')
    )
    landmark_document = yaml.safe_load(
        (CONFIG_DIR / 'qualifying_landmarks.yaml').read_text(encoding='utf-8')
    )
    waypoints = route_document['waypoints']
    landmarks = {
        landmark['stage']: landmark
        for landmark in landmark_document['landmarks']
    }

    for stage, waypoint_index in {'Q_STAGE_1': 203, 'Q_STAGE_3': 296}.items():
        landmark = landmarks[stage]
        waypoint = waypoints[waypoint_index]
        assert landmark['target_waypoint_index'] == waypoint_index
        assert landmark['x'] == pytest.approx(waypoint['x'])
        assert landmark['y'] == pytest.approx(waypoint['y'])


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
    landmarks = parse_route_stoplines(
        route,
        yaml.safe_load(
            (CONFIG_DIR / f'{course}_landmarks.yaml').read_text(encoding='utf-8')
        ),
    )
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


def route_plan_harness(speed_mps: float, ready: bool):
    return type(
        'RoutePlanHarness',
        (),
        {
            'constant_speed_mps': speed_mps,
            'route_ready': ready,
            'target_path_pub': CapturePublisher(),
            'target_speed_pub': CapturePublisher(),
        },
    )()


def test_constant_route_plan_preserves_cruise_speed_and_passes_finish_distance() -> None:
    route = RouteContext()
    route.route_projection_valid = True
    route.distance_to_finish = 100.0
    route.local_path_points = [Point(x=1.0), Point(x=2.0)]
    node = route_plan_harness(1.3, True)

    ConstantRoutePlanNode.on_route_context(node, route)

    speed = node.target_speed_pub.message
    assert speed.target_speed_mps == pytest.approx(1.3)
    assert speed.need_stop is True
    assert speed.stop_target_distance == pytest.approx(100.0)


def test_constant_route_plan_stops_while_preflight_is_not_ready() -> None:
    route = RouteContext()
    route.route_projection_valid = True
    route.local_path_points = [Point(x=1.0), Point(x=2.0)]
    node = route_plan_harness(3.0, False)

    ConstantRoutePlanNode.on_route_context(node, route)

    speed = node.target_speed_pub.message
    assert speed.target_speed_mps == pytest.approx(0.0)
    assert speed.need_stop is True


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
        maximum_cross_track_error_m=2.0,
        maximum_heading_error_rad=math.radians(30.0),
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
    ('heading_error_deg', 'expected_valid'),
    [(29.0, True), (31.0, False)],
)
def test_route_readiness_uses_thirty_degree_heading_limit(
    heading_error_deg: float,
    expected_valid: bool,
) -> None:
    captured = {}
    node = SimpleNamespace(
        preflight_passed=True,
        allow_midroute_start=True,
        maximum_cross_track_error_m=2.0,
        maximum_heading_error_rad=math.radians(30.0),
        maximum_start_progress_m=35.0,
        update_input=lambda name, valid, detail: captured.update(
            {name: (valid, detail)}
        ),
    )
    route = RouteContext()
    route.route_projection_valid = True
    route.cross_track_error = 0.2
    route.heading_error = math.radians(heading_error_deg)
    route.local_path_points = [Point(x=1.0), Point(x=2.0)]

    RouteReadinessNode.on_route(node, route)

    assert captured['/planning/route_context'][0] is expected_valid


@pytest.mark.parametrize('distance', [float('nan'), float('inf'), -1.0])
def test_constant_route_plan_rejects_invalid_finish_distance(distance) -> None:
    route = RouteContext()
    route.route_projection_valid = True
    route.distance_to_finish = distance
    route.local_path_points = [Point(x=1.0), Point(x=2.0)]
    node = route_plan_harness(3.0, True)

    ConstantRoutePlanNode.on_route_context(node, route)

    assert node.target_speed_pub.message.target_speed_mps == 0.0
    assert node.target_speed_pub.message.stop_target_distance == 0.0
    assert not node.target_path_pub.message.points


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
