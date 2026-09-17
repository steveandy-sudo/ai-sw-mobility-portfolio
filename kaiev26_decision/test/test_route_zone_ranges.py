from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from kaiev26_msgs.msg import RouteContext, SceneSummary
from nav_msgs.msg import Odometry
import pytest
from visualization_msgs.msg import Marker

from kaiev26_decision.common import DecisionCommand
from kaiev26_decision.planning_core import PlanningCore
from kaiev26_decision.priority_selector import PrioritySelector
from kaiev26_decision.route_zone_manager_node import (
    PolylineRoute,
    RouteMapPoint,
    RouteStopline,
    RouteWaypoint,
    RouteZoneManagerNode,
    merge_route_stoplines,
    odometry_position_covariance_usable,
    parse_route_stoplines,
)
from kaiev26_decision.zone_policy import EventPolicy, StagePolicy, ZonePolicy


def make_route() -> PolylineRoute:
    waypoints = [RouteWaypoint(x=float(index), y=0.0) for index in range(6)]
    return PolylineRoute(
        waypoints,
        zone_ranges=[
            {
                "zone": "INTERSECTION_ZONE",
                "start_waypoint": 2,
                "end_waypoint": 4,
            }
        ],
    )


def test_uninitialized_odometry_covariance_is_not_route_usable() -> None:
    odometry = Odometry()
    odometry.pose.covariance[0] = 1.0e6
    odometry.pose.covariance[7] = 1.0e6

    assert not odometry_position_covariance_usable(odometry, 15.0)

    odometry.pose.covariance[0] = 0.04
    odometry.pose.covariance[7] = 0.09
    assert odometry_position_covariance_usable(odometry, 15.0)


def test_zone_range_uses_one_based_inclusive_waypoint_numbers() -> None:
    route = make_route()

    assert route.zone_at(0.99) == "NORMAL_ZONE"
    assert route.zone_at(1.0) == "INTERSECTION_ZONE"
    assert route.zone_at(3.0) == "INTERSECTION_ZONE"
    assert route.zone_at(3.01) == "NORMAL_ZONE"
    assert route.distance_to_zone(0.0, "INTERSECTION_ZONE") == 1.0
    assert route.distance_to_zone(2.0, "INTERSECTION_ZONE") == 0.0


def test_policy_exposes_only_the_current_stage_mission_stopline() -> None:
    route = PolylineRoute(
        [RouteWaypoint(x=float(index * 20), y=0.0) for index in range(6)],
        zone_ranges=[
            {"zone": "Q_STAGE_1", "start_s": 0.0, "end_s": 60.0},
            {"zone": "Q_STAGE_2", "start_s": 60.0, "end_s": 100.0},
        ],
    )
    policy = ZonePolicy(
        course_id="test",
        ready=True,
        stages=(
            StagePolicy(
                "Q_STAGE_1",
                boundary_stop_line_id="mission_stop",
                event=EventPolicy("Z1_TIMED_STOP", "TIMED_STOP", "mission_stop"),
            ),
            StagePolicy("Q_STAGE_2"),
        ),
    )
    node = object.__new__(RouteZoneManagerNode)
    node.route = route
    node.zone_policy = policy
    node.stoplines = [
        RouteStopline("ignored_stop", 20.0),
        RouteStopline("mission_stop", 60.0),
    ]

    assert node.next_policy_stopline(10.0).landmark_id == "mission_stop"
    assert node.next_policy_stopline(61.0) is None


def test_policy_signal_mapping_is_validated_before_driving() -> None:
    event = EventPolicy(
        "Z2_SIGNAL_OBEY",
        "SIGNAL_OBEY",
        "signal_stop",
        ("expected_light",),
    )
    node = object.__new__(RouteZoneManagerNode)
    node.zone_policy = ZonePolicy(
        course_id="test",
        ready=True,
        stages=(StagePolicy("Q_STAGE_2", event=event),),
    )
    node.stoplines = [
        RouteStopline(
            "signal_stop",
            40.0,
            traffic_light_ids=("expected_light",),
        )
    ]
    node.validate_policy_controls()

    node.stoplines = [
        RouteStopline(
            "signal_stop",
            40.0,
            traffic_light_ids=("wrong_light",),
        )
    ]
    with pytest.raises(ValueError, match="traffic-light mapping is invalid"):
        node.validate_policy_controls()


def test_custom_stopline_can_define_directional_signal_identity() -> None:
    stopline = parse_route_stoplines(
        make_route(),
        {
            "landmarks": [
                {
                    "id": "signal_stop",
                    "type": "STOP_LINE",
                    "route_s": 2.0,
                    "rule_id": "route_direction_signal",
                    "traffic_light_ids": ["expected_light"],
                }
            ]
        },
    )[0]

    assert stopline.rule_id == "route_direction_signal"
    assert stopline.traffic_light_ids == ("expected_light",)


def test_custom_stopline_can_define_signal_map_position() -> None:
    stopline = parse_route_stoplines(
        make_route(),
        {
            "landmarks": [
                {
                    "id": "signal_stop",
                    "type": "STOP_LINE",
                    "route_s": 2.0,
                    "traffic_light_ids": ["expected_light"],
                    "traffic_light_positions": [
                        {"x": 10.0, "y": 20.0, "z": 4.8}
                    ],
                }
            ]
        },
    )[0]

    assert stopline.traffic_light_positions == (
        RouteMapPoint(10.0, 20.0, 4.8),
    )


def test_landmark_override_preserves_matching_signal_position() -> None:
    signal_position = RouteMapPoint(10.0, 20.0, 4.8)
    map_control = RouteStopline(
        "signal_stop",
        40.0,
        traffic_light_ids=("expected_light",),
        traffic_light_positions=(signal_position,),
    )
    landmark_override = RouteStopline(
        "signal_stop",
        40.0,
        rule_id="course_signal_rule",
        traffic_light_ids=("expected_light",),
    )

    merged = merge_route_stoplines([map_control, landmark_override])

    assert merged == [
        RouteStopline(
            "signal_stop",
            40.0,
            rule_id="course_signal_rule",
            traffic_light_ids=("expected_light",),
            traffic_light_positions=(signal_position,),
        )
    ]


def test_intersection_zone_is_descriptive_and_uses_lane_follow() -> None:
    scene = SceneSummary()
    route = RouteContext()
    route.route_projection_valid = True
    route.current_zone = "INTERSECTION_ZONE"
    route.local_path_points = [Point(x=0.0), Point(x=10.0)]

    selector = PrioritySelector(stopline_approach_distance_m=18.0)

    assert selector.choose(scene, route, None, False).behavior == "LANE_FOLLOW"


def test_dashboard_only_zone_does_not_apply_intersection_speed_cap() -> None:
    scene = SceneSummary()
    route = RouteContext()
    route.route_projection_valid = True
    route.current_zone = "INTERSECTION_ZONE"
    route.local_path_points = [Point(x=0.0), Point(x=10.0)]
    decision = DecisionCommand(
        active_behavior="GLOBAL_ROUTE_FOLLOW",
        target_speed_limit_mps=4.0,
        path_request="ROUTE_LOCAL_PATH",
    )
    core = PlanningCore(4.0, 0.8)
    constraints = []

    speed = core.resolve_speed_limit(decision, scene, route, constraints)

    assert speed == 4.0
    assert "INTERSECTION_ZONE_CAP" not in constraints


def test_mapped_unknown_signal_requires_stop_near_stopline() -> None:
    scene = SceneSummary()
    scene.traffic_light_state = SceneSummary.TRAFFIC_UNKNOWN
    scene.stopline_distance = 12.0
    route = RouteContext()
    route.route_projection_valid = True
    route.next_stop_line_id = "stop_line_manual_052"
    route.distance_to_stopline = 12.0

    selector = PrioritySelector(stopline_approach_distance_m=18.0)

    assert selector.choose(scene, route, None, False).behavior == "TRAFFIC_LIGHT"


def test_arrow_only_releases_left_turn() -> None:
    scene = SceneSummary()
    scene.traffic_light_state = SceneSummary.TRAFFIC_ARROW
    scene.stopline_distance = 8.0
    route = RouteContext()
    route.route_projection_valid = True
    route.next_stop_line_id = "stop_line_manual_052"
    route.distance_to_stopline = 8.0
    selector = PrioritySelector(stopline_approach_distance_m=18.0)

    route.next_maneuver = "RIGHT"
    assert selector.choose(scene, route, None, False).behavior == "TRAFFIC_LIGHT"

    route.next_maneuver = "LEFT"
    assert selector.choose(scene, route, None, False).behavior == "LANE_FOLLOW"


def test_hd_map_speed_limit_caps_planning_speed() -> None:
    scene = SceneSummary()
    route = RouteContext()
    route.route_projection_valid = True
    route.speed_limit_mps = 2.5
    decision = DecisionCommand(
        active_behavior="GLOBAL_ROUTE_FOLLOW",
        target_speed_limit_mps=4.0,
        path_request="ROUTE_LOCAL_PATH",
    )
    core = PlanningCore(4.0, 0.8)
    constraints = []

    speed = core.resolve_speed_limit(decision, scene, route, constraints)

    assert speed == 2.5
    assert "HD_MAP_SPEED_LIMIT" in constraints


def test_global_route_markers_are_static_sparse_and_zone_aware() -> None:
    route = PolylineRoute(
        [RouteWaypoint(x=float(index * 10), y=0.0) for index in range(7)],
        zone_ranges=[
            {
                "zone": "INTERSECTION_ZONE",
                "start_waypoint": 3,
                "end_waypoint": 5,
            }
        ],
    )

    class CapturePublisher:
        message = None

        def publish(self, message):
            self.message = message

    node = object.__new__(RouteZoneManagerNode)
    node.map_frame_id = "map"
    node.route = route
    node.zone_policy = None
    node.stoplines = []
    node.global_route_marker_pub = CapturePublisher()
    node.publish_global_route_markers(Time())

    markers = node.global_route_marker_pub.message.markers
    namespaces = {marker.ns for marker in markers}
    assert "global_route_underlay" in namespaces
    assert "global_route_line" in namespaces
    assert "route_zone_intersection_zone" in namespaces
    waypoint_marker = next(
        marker for marker in markers if marker.ns == "global_route_waypoints"
    )
    assert waypoint_marker.type == Marker.SPHERE_LIST
    assert len(waypoint_marker.points) == 7
    distance_labels = {
        marker.text
        for marker in markers
        if marker.ns == "global_route_distance_labels"
    }
    assert distance_labels == {"25 m", "50 m"}


def test_global_route_markers_distinguish_configured_brake_waypoints() -> None:
    route = PolylineRoute(
        [RouteWaypoint(x=float(index), y=0.0) for index in range(5)]
    )
    policy = ZonePolicy(
        course_id="test",
        ready=True,
        stages=(
            StagePolicy(
                stage_id="TEST_STAGE",
                event=EventPolicy(
                    event_id="STOP",
                    event_type="TIMED_STOP",
                    brake_trigger_waypoint=3,
                    brake_trigger_route_s=3.0,
                ),
            ),
        ),
    )

    class CapturePublisher:
        message = None

        def publish(self, message):
            self.message = message

    node = object.__new__(RouteZoneManagerNode)
    node.map_frame_id = "map"
    node.route = route
    node.zone_policy = policy
    node.stoplines = []
    node.global_route_marker_pub = CapturePublisher()
    node.publish_global_route_markers(Time())

    marker = next(
        item
        for item in node.global_route_marker_pub.message.markers
        if item.ns == "global_route_brake_waypoints"
    )
    assert marker.type == Marker.SPHERE_LIST
    assert len(marker.points) == 1
    assert marker.points[0].x == pytest.approx(3.0)
