from pathlib import Path
import sys

from kaiev26_msgs.msg import (
    Centerline,
    PerceptionObjectArray,
    RoadSegmentArray,
    RouteContext,
    SceneSummary,
    TrafficLightObservation,
    TrafficLightObservationArray,
)
from geometry_msgs.msg import Point
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy


PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR))

from kaiev26_decision.perception_gateway_node import (  # noqa: E402
    OBSERVATION_QOS,
    ObservationFrameJoiner,
    PerceptionGatewayNode,
)


def stamped(message, sec: int, nanosec: int = 0):
    message.header.stamp.sec = sec
    message.header.stamp.nanosec = nanosec
    message.header.frame_id = "base_footprint"
    return message


def messages_for_stamp(sec: int):
    return (
        stamped(RoadSegmentArray(), sec),
        stamped(Centerline(), sec),
        stamped(PerceptionObjectArray(), sec),
        stamped(TrafficLightObservationArray(), sec),
    )


def make_gateway() -> PerceptionGatewayNode:
    gateway = PerceptionGatewayNode.__new__(PerceptionGatewayNode)
    gateway.latest_road_segments = None
    gateway.latest_centerline = None
    gateway.latest_objects = None
    gateway.latest_traffic_lights = None
    gateway.latest_route_context = None
    gateway.observation_frames = ObservationFrameJoiner()
    gateway.traffic_light_map_match_distance_m = 2.0
    return gateway


def point(x: float, y: float) -> Point:
    value = Point()
    value.x = x
    value.y = y
    return value


def test_gateway_promotes_only_complete_same_stamp_observation_frame() -> None:
    gateway = make_gateway()
    road_1, center_1, objects_1, lights_1 = messages_for_stamp(1)

    gateway.on_road_segments(road_1)
    gateway.on_centerline(center_1)
    gateway.on_objects(objects_1)

    assert gateway.latest_road_segments is None
    assert gateway.latest_centerline is None
    assert gateway.latest_objects is None
    assert gateway.latest_traffic_lights is None

    road_2, center_2, objects_2, lights_2 = messages_for_stamp(2)
    gateway.on_road_segments(road_2)
    gateway.on_centerline(center_2)
    gateway.on_objects(objects_2)
    gateway.on_traffic_lights(lights_2)

    assert gateway.latest_road_segments is road_2
    assert gateway.latest_centerline is center_2
    assert gateway.latest_objects is objects_2
    assert gateway.latest_traffic_lights is lights_2

    gateway.on_traffic_lights(lights_1)
    assert gateway.latest_road_segments is road_2
    assert gateway.latest_centerline is center_2
    assert gateway.latest_objects is objects_2
    assert gateway.latest_traffic_lights is lights_2


def test_empty_observation_messages_still_complete_the_frame() -> None:
    joiner = ObservationFrameJoiner()
    road, center, objects, lights = messages_for_stamp(7)

    assert joiner.add("road_segments", road) is None
    assert joiner.add("centerline", center) is None
    assert joiner.add("objects", objects) is None
    complete = joiner.add("traffic_lights", lights)

    assert complete == (road, center, objects, lights)
    assert not road.segments
    assert not center.points
    assert not objects.objects
    assert not lights.lights


def test_partial_new_frame_does_not_replace_last_complete_frame() -> None:
    gateway = make_gateway()
    complete = messages_for_stamp(3)
    gateway.on_road_segments(complete[0])
    gateway.on_centerline(complete[1])
    gateway.on_objects(complete[2])
    gateway.on_traffic_lights(complete[3])

    partial = messages_for_stamp(4)
    gateway.on_road_segments(partial[0])
    gateway.on_objects(partial[2])

    assert gateway.latest_road_segments is complete[0]
    assert gateway.latest_centerline is complete[1]
    assert gateway.latest_objects is complete[2]
    assert gateway.latest_traffic_lights is complete[3]


def test_gateway_observation_qos_is_reliable_latest_only() -> None:
    assert OBSERVATION_QOS.history == HistoryPolicy.KEEP_LAST
    assert OBSERVATION_QOS.depth == 1
    assert OBSERVATION_QOS.reliability == ReliabilityPolicy.RELIABLE
    assert OBSERVATION_QOS.durability == DurabilityPolicy.VOLATILE


def mapped_route() -> RouteContext:
    route = RouteContext()
    route.route_projection_valid = True
    route.next_stop_line_id = "stop_line_manual_052"
    route.next_traffic_light_ids = ["traffic_light_manual_142"]
    route.next_traffic_light_positions = [point(14.0, 1.0)]
    route.distance_to_stopline = 8.0
    return route


def test_gateway_selects_observation_matching_mapped_signal() -> None:
    gateway = make_gateway()
    observations = TrafficLightObservationArray()

    off_route = TrafficLightObservation()
    off_route.position = point(14.0, 7.0)
    off_route.state = TrafficLightObservation.STATE_RED
    off_route.confidence = 1.0
    on_route = TrafficLightObservation()
    on_route.position = point(14.5, 1.0)
    on_route.state = TrafficLightObservation.STATE_GREEN
    on_route.confidence = 0.9
    observations.lights = [off_route, on_route]

    distance, state, confidence = gateway.summarize_traffic_control(
        observations, mapped_route()
    )

    assert distance == 8.0
    assert state == SceneSummary.TRAFFIC_GREEN
    assert confidence == 0.9


def test_gateway_preserves_green_plus_arrow_as_route_go_signal() -> None:
    gateway = make_gateway()
    observations = TrafficLightObservationArray()
    light = TrafficLightObservation()
    light.position = point(14.0, 1.0)
    light.state = TrafficLightObservation.STATE_ARROW
    light.green_score = 0.9
    light.arrow_score = 0.95
    light.confidence = 0.9
    observations.lights = [light]

    _, state, _ = gateway.summarize_traffic_control(observations, mapped_route())

    assert state == SceneSummary.TRAFFIC_GREEN


def test_gateway_maps_red_plus_arrow_to_zone_interpreted_arrow() -> None:
    gateway = make_gateway()
    observations = TrafficLightObservationArray()
    light = TrafficLightObservation()
    light.position = point(14.0, 1.0)
    light.state = TrafficLightObservation.STATE_RED
    light.red_score = 0.9
    light.arrow_score = 0.9
    light.confidence = 0.9
    observations.lights = [light]

    _, state, _ = gateway.summarize_traffic_control(observations, mapped_route())

    assert state == SceneSummary.TRAFFIC_ARROW


def test_gateway_keeps_yellow_as_stop_for_composite_signal() -> None:
    gateway = make_gateway()
    observations = TrafficLightObservationArray()
    light = TrafficLightObservation()
    light.position = point(14.0, 1.0)
    light.state = TrafficLightObservation.STATE_ARROW
    light.yellow_score = 0.9
    light.arrow_score = 0.9
    light.confidence = 0.9
    observations.lights = [light]

    _, state, _ = gateway.summarize_traffic_control(observations, mapped_route())

    assert state == SceneSummary.TRAFFIC_YELLOW


def test_gateway_preserves_mapped_stopline_when_signal_is_not_observed() -> None:
    gateway = make_gateway()
    distance, state, confidence = gateway.summarize_traffic_control(
        None, mapped_route()
    )

    assert distance == 8.0
    assert state == SceneSummary.TRAFFIC_UNKNOWN
    assert confidence == 0.0


def test_gateway_rejects_observation_far_from_mapped_signal() -> None:
    gateway = make_gateway()
    observations = TrafficLightObservationArray()
    light = TrafficLightObservation()
    light.position = point(14.0, 5.0)
    light.state = TrafficLightObservation.STATE_RED
    observations.lights = [light]

    distance, state, confidence = gateway.summarize_traffic_control(
        observations, mapped_route()
    )

    assert distance == 8.0
    assert state == SceneSummary.TRAFFIC_UNKNOWN
    assert confidence == 0.0


def test_gateway_has_no_control_without_mapped_regulatory_element() -> None:
    gateway = make_gateway()
    route = RouteContext()
    route.route_projection_valid = True

    distance, state, confidence = gateway.summarize_traffic_control(
        TrafficLightObservationArray(), route
    )

    assert distance == 1.0e6
    assert state == SceneSummary.TRAFFIC_UNKNOWN
    assert confidence == 0.0
