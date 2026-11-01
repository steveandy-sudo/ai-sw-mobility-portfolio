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
    light_numeric_id,
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


def observed(track_id: int, state: int, lamps: int = 0, confidence: float = 0.9) -> TrafficLightObservation:
    light = TrafficLightObservation()
    light.track_id = track_id
    light.state = state
    light.lamps = lamps
    light.confidence = confidence
    return light


def summarize(*lights: TrafficLightObservation):
    observations = TrafficLightObservationArray()
    observations.lights = list(lights)
    return make_gateway().summarize_traffic_control(observations, mapped_route())


RED, GREEN, ARROW, YELLOW = (
    TrafficLightObservation.STATE_RED,
    TrafficLightObservation.STATE_GREEN,
    TrafficLightObservation.STATE_ARROW,
    TrafficLightObservation.STATE_YELLOW,
)
LAMP_RED, LAMP_YELLOW, LAMP_GREEN, LAMP_ARROW = (
    TrafficLightObservation.LAMP_RED,
    TrafficLightObservation.LAMP_YELLOW,
    TrafficLightObservation.LAMP_GREEN,
    TrafficLightObservation.LAMP_ARROW,
)


def test_light_numeric_id_is_the_trailing_number_of_the_map_id() -> None:
    assert light_numeric_id("traffic_light_manual_305") == 305
    assert light_numeric_id("traffic_light_manual_142") == 142
    assert light_numeric_id("no_digits") is None


def test_gateway_reads_the_light_perception_names_for_the_route() -> None:
    off_route = observed(143, RED, LAMP_RED, confidence=1.0)
    on_route = observed(142, GREEN, LAMP_GREEN)

    distance, state, confidence, arrow_active = summarize(off_route, on_route)

    assert distance == 8.0
    assert state == SceneSummary.TRAFFIC_GREEN
    assert confidence == 0.9
    assert arrow_active is False


def test_gateway_takes_the_most_confident_observation_of_the_light() -> None:
    _, state, confidence, _ = summarize(observed(142, RED, LAMP_RED, 0.4), observed(142, GREEN, LAMP_GREEN, 0.8))
    assert state == SceneSummary.TRAFFIC_GREEN
    assert confidence == 0.8


def test_gateway_uses_perception_state_and_lamps_as_they_are() -> None:
    # Perception decides state and lamps; the gateway reads them without scores.
    for state, lamps, scene_state, arrow in (
        (GREEN, LAMP_GREEN | LAMP_ARROW, SceneSummary.TRAFFIC_GREEN, True),
        (ARROW, LAMP_RED | LAMP_ARROW, SceneSummary.TRAFFIC_ARROW, True),
        (YELLOW, LAMP_YELLOW | LAMP_ARROW, SceneSummary.TRAFFIC_YELLOW, True),
        (RED, LAMP_RED, SceneSummary.TRAFFIC_RED, False),
        (TrafficLightObservation.STATE_UNKNOWN, 0, SceneSummary.TRAFFIC_UNKNOWN, False),
    ):
        light = observed(142, state, lamps)
        light.red_score = light.green_score = 0.99  # ignored
        _, light_state, _, arrow_active = summarize(light)
        assert (light_state, arrow_active) == (scene_state, arrow)


def test_gateway_preserves_mapped_stopline_when_signal_is_not_observed() -> None:
    gateway = make_gateway()
    distance, state, confidence, arrow_active = gateway.summarize_traffic_control(
        None, mapped_route()
    )

    assert distance == 8.0
    assert state == SceneSummary.TRAFFIC_UNKNOWN
    assert confidence == 0.0
    assert arrow_active is False


def test_gateway_ignores_lights_perception_did_not_match_to_the_route() -> None:
    for track_id in (0, 143):  # unassociated, another map light
        distance, state, confidence, arrow_active = summarize(observed(track_id, RED, LAMP_RED))
        assert distance == 8.0
        assert state == SceneSummary.TRAFFIC_UNKNOWN
        assert confidence == 0.0
        assert arrow_active is False


def test_gateway_has_no_control_without_mapped_regulatory_element() -> None:
    gateway = make_gateway()
    route = RouteContext()
    route.route_projection_valid = True

    distance, state, confidence, arrow_active = gateway.summarize_traffic_control(
        TrafficLightObservationArray(), route
    )

    assert distance == 1.0e6
    assert state == SceneSummary.TRAFFIC_UNKNOWN
    assert confidence == 0.0
    assert arrow_active is False
