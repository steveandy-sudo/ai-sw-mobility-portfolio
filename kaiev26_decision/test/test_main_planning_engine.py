"""Main planning engine node, ticked by hand: situation -> published result tables."""
from pathlib import Path

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from kaiev26_msgs.msg import (
    RouteContext,
    TrafficLightObservation,
    TrafficLightObservationArray,
    TrafficSignalPhase,
    TrafficSignalPhaseArray,
    VehicleState,
)
import pytest
import rclpy
from rclpy.clock import ClockType
from rclpy.time import Time as RosTime

from kaiev26_decision.main_planning_engine_node import MainPlanningEngineNode
from kaiev26_decision.mission_events import obstacle_zone_finish_check


PACKAGE = Path(__file__).resolve().parents[1]
ROUTES = {"qualifying": "kcity_quali_route.yaml", "final": "kcity_final_route.yaml"}


def Time(seconds):
    return RosTime(seconds=seconds, clock_type=ClockType.ROS_TIME)


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture
def engine(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "43")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    nodes = []

    def make(course="final", mode="all", speed=8.0, **extra):
        parameters = {
            "route_config": str(PACKAGE / "waypoints" / ROUTES[course]),
            "mission_policy_config": "" if mode == "tracking" else str(PACKAGE / "config" / f"{course}_policy.yaml"),
            "landmark_config": "" if mode == "tracking" else str(PACKAGE / "config" / f"{course}_landmarks.yaml"),
            "mission_mode": mode,
            "base_speed_mps": speed,
            **extra,
        }
        args = ["--ros-args"] + [
            item for key, value in parameters.items()
            for item in ("-p", f"{key}:={value}" if value != "" else f"{key}:=''")
        ]
        rclpy.init(args=args)
        node = MainPlanningEngineNode()
        nodes.append(node)
        node.clock_sec = 100.0
        node.get_clock = lambda: type("Clock", (), {"now": lambda _self: Time(seconds=node.clock_sec)})()
        node.captured = {}
        for name in ("target_path_pub", "target_speed_pub", "behavior_decision_pub"):
            capture = Capture()
            node.captured[name] = capture
            setattr(node, name, capture)
        return node

    yield make
    for node in nodes:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def tick(node, now, progress, speed, *, fresh=True, mode=VehicleState.MODE_AUTO, estop=False,
         zone="", lights=None, phases=None):
    node.clock_sec = now
    stamp = Time(seconds=now if fresh else now - 5.0).to_msg()
    route = RouteContext()
    route.header.stamp = stamp
    route.header.frame_id = "base_footprint"
    route.route_projection_valid = True
    route.projection_confidence = 0.9
    route.progress_s = float(progress)
    route.distance_to_finish = node.route_geometry.total_length - float(progress)
    route.current_zone = zone
    route.local_path_points = [Point(x=float(x)) for x in range(20)]
    vehicle = VehicleState(mode=mode, speed_mps=float(speed), estop_active=estop)
    vehicle.header.stamp = Time(seconds=now).to_msg()
    node.latest_route_context = route
    node.latest_vehicle_state = vehicle
    if lights is not None:
        message = TrafficLightObservationArray()
        message.header.stamp = Time(seconds=now).to_msg()
        for track_id, state, lamps in lights:
            message.lights.append(TrafficLightObservation(track_id=track_id, state=state, lamps=lamps,
                                                          confidence=0.9))
        node.latest_lights = message
    if phases is not None:
        message = TrafficSignalPhaseArray()
        message.header.stamp = Time(seconds=now).to_msg()
        message.phases = phases
        node.latest_phases = message
    node.on_timer()
    target = node.captured["target_speed_pub"].messages[-1]
    decision = node.captured["behavior_decision_pub"].messages[-1]
    path = node.captured["target_path_pub"].messages[-1]
    return target, decision, path


@pytest.mark.parametrize("course, mode, kinds", [
    ("final", "all", ["SIGNAL_OBEY"] * 5 + ["FINISH"]),
    ("final", "avoidance", ["FINISH"]),
    ("qualifying", "stopline", ["TIMED_STOP"] * 4 + ["FINISH"]),
    ("qualifying", "tracking", ["FINISH"]),
])
def test_engine_builds_its_events_at_start(engine, course, mode, kinds) -> None:
    node = engine(course, mode)
    assert [event.kind for event in node.tracker.events] == kinds
    assert (node.obstacle_zone is not None) == (course == "final" and mode in {"all", "avoidance"})


def test_outputs_are_stamped_with_the_tick_time(engine) -> None:
    node = engine("final", "avoidance")
    target, decision, path = tick(node, 100.0, 10.0, 5.0)
    node.latest_route_context.header.stamp = Time(seconds=99.8).to_msg()
    node.clock_sec = 100.1
    node.on_timer()
    target = node.captured["target_speed_pub"].messages[-1]
    path = node.captured["target_path_pub"].messages[-1]
    assert RosTime.from_msg(target.header.stamp).nanoseconds == Time(seconds=100.1).nanoseconds
    assert RosTime.from_msg(path.header.stamp).nanoseconds == Time(seconds=100.1).nanoseconds


def odometry(now, x):
    message = Odometry()
    message.header.stamp = Time(seconds=now).to_msg()
    message.header.frame_id = "map"
    message.child_frame_id = "base_footprint"
    message.pose.pose.position.x = float(x)
    message.pose.pose.orientation.w = 1.0
    return message


def test_held_path_is_rebased_to_the_tick_pose_not_restamped(engine) -> None:
    node = engine("final", "avoidance", speed=5.0)
    node.odometry.add_odometry(odometry(100.0, 0.0))
    tick(node, 100.0, 10.0, 5.0)
    node.odometry.add_odometry(odometry(100.1, 0.5))
    _, decision, path = tick(node, 100.1, 10.0, 5.0, fresh=False)
    assert decision.active_behavior == "RECOVERY" and path.source == "previous_safe_path"
    assert RosTime.from_msg(path.header.stamp).nanoseconds == Time(seconds=100.1).nanoseconds
    assert path.points[0].x == pytest.approx(-0.5)
    # Without the pose at the tick the held path keeps its own stamp: its age stays visible.
    _, _, path = tick(node, 101.0, 10.0, 4.0, fresh=False)
    assert RosTime.from_msg(path.header.stamp).nanoseconds == Time(seconds=100.0).nanoseconds
    assert path.points[0].x == pytest.approx(0.0)


# (time, inputs fresh, speed, estop, MANUAL) -> (need_stop, target, active_behavior)
INPUT_SEQUENCES = {
    "estop_is_a_stop_request": [
        (100.0, True, 5.0, False, False, (False, 5.0, "GLOBAL_ROUTE_FOLLOW")),
        (100.1, True, 5.0, True, False, (True, 0.0, "EMERGENCY")),
        (100.2, True, 0.0, False, False, (False, 5.0, "GLOBAL_ROUTE_FOLLOW")),
    ],
    "stale_route_is_speed_zero_then_brake_until_stopped": [
        (100.0, True, 5.0, False, False, (False, 5.0, "GLOBAL_ROUTE_FOLLOW")),
        (100.1, False, 5.0, False, False, (False, 0.0, "RECOVERY")),
        (101.0, False, 4.0, False, False, (False, 0.0, "RECOVERY")),
        (101.2, False, 3.0, False, False, (True, 0.0, "RECOVERY")),
        (101.3, True, 2.0, False, False, (True, 0.0, "RECOVERY")),
        (101.4, True, 0.0, False, False, (False, 5.0, "GLOBAL_ROUTE_FOLLOW")),
    ],
    "manual_mode_suspends_without_a_stop": [
        (100.0, True, 3.2, False, True, (False, 3.2, "MANUAL_OVERRIDE")),
        (100.1, False, 3.2, False, True, (False, 3.2, "MANUAL_OVERRIDE")),
        (100.2, True, 3.0, False, False, (False, 5.0, "GLOBAL_ROUTE_FOLLOW")),
    ],
}


@pytest.mark.parametrize("name", sorted(INPUT_SEQUENCES))
def test_input_state(engine, name) -> None:
    node = engine("final", "avoidance", speed=5.0)
    for now, fresh, speed, estop, manual, expected in INPUT_SEQUENCES[name]:
        target, decision, path = tick(
            node, now, 10.0, speed, fresh=fresh, estop=estop,
            mode=VehicleState.MODE_MANUAL if manual else VehicleState.MODE_AUTO,
        )
        assert (target.need_stop, target.target_speed_mps, decision.active_behavior) == pytest.approx(
            expected), (name, now)
        assert len(path.points) >= 2  # the path is never dropped, also while stopping
        if target.need_stop:
            assert target.stop_target_distance == 0.0


def test_finish_hold_is_published_for_the_tui(engine) -> None:
    node = engine("final", "avoidance", speed=5.0)
    end = node.route_geometry.total_length
    tick(node, 100.0, end - 15.0, 0.6)
    target, decision, _ = tick(node, 100.1, end - 0.9, 0.6)
    assert target.need_stop and "FINISH_STOP" in decision.constraints
    target, decision, _ = tick(node, 100.2, end - 0.8, 0.0)
    assert decision.active_behavior == "FINISH" and "FINISH_HELD" in decision.constraints
    # D11: the finish brake also holds while the inputs are stale.
    target, decision, _ = tick(node, 101.5, end - 0.8, 0.0, fresh=False)
    assert target.need_stop and "FINISH_HELD" in decision.constraints


def own_light(event, state, arrow=False):
    from kaiev26_decision.perception_gateway_node import light_numeric_id
    lamps = TrafficLightObservation.LAMP_ARROW if arrow else 0
    return (light_numeric_id(event.light_ids[0]), state, lamps)


def signal_event(node, index):
    return [event for event in node.tracker.events if event.kind == "SIGNAL_OBEY"][index]


def test_signal_reads_only_its_own_light(engine) -> None:
    node = engine("qualifying", "all", speed=7.0)
    event = signal_event(node, 0)
    for tracker_event in node.tracker.events:
        if tracker_event.line_s < event.line_s:
            node.tracker.cleared_ids.append(tracker_event.event_id)
            node.tracker.index += 1
    node.tracker.machine = node.tracker._machine_for(node.tracker.index)
    other = (999, TrafficLightObservation.STATE_GREEN, 0)
    target, decision, _ = tick(node, 100.0, event.line_s - 3.5, 1.0,
                               lights=[other, own_light(event, TrafficLightObservation.STATE_RED)])
    assert target.need_stop and decision.active_behavior == "TRAFFIC_LIGHT"
    for now in (100.1, 100.6):
        target, decision, _ = tick(node, now, event.line_s - 3.0, 0.0,
                                   lights=[other, own_light(event, TrafficLightObservation.STATE_RED)])
        assert target.need_stop


@pytest.mark.parametrize("aligned, launches", [(True, True), (False, False)])
def test_phase_tracker_stands_in_stopped_past_the_line(engine, aligned, launches) -> None:
    node = engine("final", "all", speed=8.0)
    event = signal_event(node, 0)
    phase = TrafficSignalPhase(light_id=event.light_ids[0], on_route=True, route_event_id=event.event_id,
                               aligned=aligned, permitted_now=True,
                               # well before its end: one ending within 4 s never launches a stopped car
                               permission_change_sec=30.0)
    target, _, _ = tick(node, 100.0, event.line_s - 20.0, 1.0, phases=[phase])
    assert not target.need_stop
    target, _, _ = tick(node, 100.1, event.line_s - 4.5, 1.0, phases=[phase])
    assert target.need_stop  # the own light is UNKNOWN: the estimate is not used while moving
    # Stopped at 100.2 (BRAKING), HELD from 100.4: the 0.4 s confirmation counts from there.
    for now in (100.2, 100.4, 100.65, 100.85):
        target, decision, _ = tick(node, now, event.line_s + 0.3, 0.0, phases=[phase])
    assert (not target.need_stop) == launches
    assert ("SIGNAL_PHASE_TRACKER_USED" in decision.constraints) == aligned


@pytest.mark.parametrize("zone_exit, return_length, finish_brake, ok", [
    (700.0, 10.0, 726.4, True),
    (727.4, 10.0, 726.4, False),
])
def test_obstacle_zone_finish_check(zone_exit, return_length, finish_brake, ok) -> None:
    assert (obstacle_zone_finish_check(zone_exit, return_length, finish_brake) == "") == ok


# K-City final: zone end 669.84 m (WP 670), finish brake point 733.5 m: the antenna 2.0 m past the
# surveyed line at 731.93 m (0.426 m ahead of the rear axle), whatever the speed.
@pytest.mark.parametrize("return_length, starts", [(10.0, True), (63.5, True), (64.0, False)])
def test_final_refuses_to_start_when_the_bypass_return_reaches_the_finish_brake(
        engine, return_length, starts) -> None:
    if starts:
        assert engine(obstacle_return_length_m=return_length).obstacle_zone.exit_s == pytest.approx(669.84, abs=0.01)
    else:
        with pytest.raises(ValueError, match="finish brake point 733.5"):
            engine(obstacle_return_length_m=return_length)


# 2026-10-17 145703/114251: in the obstacle stage a route context up to 1.0 s old still drives
# the bypass. situation: stage, route age -> stale (input-loss rule) or tolerated.
@pytest.mark.parametrize("zone, age, stale", [
    ("F_STAGE_6", 0.8, False),
    ("F_STAGE_6", 1.2, True),
    ("F_STAGE_5", 0.8, True),
    ("F_STAGE_6", 0.3, None),
])
def test_obstacle_stage_tolerates_an_older_route_context(engine, zone, age, stale) -> None:
    node = engine("final", "avoidance", speed=5.0)
    node.debug_route_age_pub = Capture()
    tick(node, 100.0, 600.0, 5.0, zone=zone)
    node.clock_sec = 100.0 + age
    node.latest_vehicle_state.header.stamp = Time(seconds=100.0 + age).to_msg()
    node.on_timer()
    decision = node.captured["behavior_decision_pub"].messages[-1]
    assert ("ROUTE_CONTEXT_STALE" in decision.constraints) == bool(stale)
    assert ("ROUTE_CONTEXT_AGE_TOLERATED" in decision.constraints) == (stale is False)
    assert node.debug_route_age_pub.messages[-1].data == pytest.approx(age, abs=1e-3)


# 2026-10-24: F_STAGE_6 (F5 line to the finish, no signal) runs at its course speed in place of
# the TUI speed; the stages before it keep the TUI speed.
@pytest.mark.parametrize("zone, progress, cap, fsm", [
    ("F_STAGE_5", 440.0, 10.0, "ROUTE_FOLLOW"),
    ("F_STAGE_6", 700.0, 13.0, "FINAL_SPRINT"),
])
def test_final_stage_runs_at_its_course_speed(engine, zone, progress, cap, fsm) -> None:
    node = engine("final", "avoidance", speed=10.0)
    target, decision, _ = tick(node, 100.0, progress, cap, zone=zone)
    assert not target.need_stop
    assert target.speed_limit_mps == pytest.approx(cap)
    assert decision.fsm_state == fsm
    assert any(tag.startswith("STAGE_COURSE_SPEED:F_STAGE_6") for tag in decision.constraints) == (
        zone == "F_STAGE_6")


# 2026-10-25: the surveyed painted line crosses the route at 731.93 m (final) / 555.67 m
# (qualifying). The brake station is where the antenna, 0.426 m ahead of the rear axle, is 1.0 m
# past it; the watch starts 2 m before it (a tick).
@pytest.mark.parametrize("course, brake_s", [("final", 733.503), ("qualifying", 557.244)])
def test_finish_brakes_at_a_fixed_station_past_the_surveyed_line(engine, course, brake_s) -> None:
    node = engine(course, "avoidance", speed=8.0)
    finish = node.tracker.events[-1]
    end = node.route_geometry.total_length
    assert finish.kind == "FINISH"
    assert finish.line_s == pytest.approx(end)
    assert finish.brake_s == pytest.approx(brake_s, abs=0.001)
    assert finish.approach_m == pytest.approx(end - brake_s + 2.0, abs=0.001)
    assert finish.watch_cap_mps is None


def test_final_sprint_keeps_thirteen_mps_over_the_finish_line_then_brakes(engine) -> None:
    # 10/24-25: the d_stop(13) brake point and the 0.6 m/s watch cap braked about 11 m before the line.
    node = engine("final", "avoidance", speed=10.0)
    for now, s_rear in [(100.0, 731.0), (100.1, 733.45)]:
        target, decision, _ = tick(node, now, s_rear, 13.0, zone="F_STAGE_6")
        assert not target.need_stop, (s_rear, decision.constraints)
        assert target.speed_limit_mps == pytest.approx(13.0), (s_rear, decision.constraints)
    target, decision, _ = tick(node, 100.2, 733.55, 13.0, zone="F_STAGE_6")
    assert target.need_stop
    assert "FINISH_STOP" in decision.constraints


def test_planned_stop_releases_the_brake_after_two_seconds_at_standstill(engine) -> None:
    node = engine("final", "avoidance", speed=5.0)
    end = node.route_geometry.total_length
    tick(node, 100.0, end - 15.0, 0.6)
    tick(node, 100.1, end - 0.9, 0.6)
    target, decision, _ = tick(node, 100.2, end - 0.8, 0.0)
    assert "FINISH_HELD" in decision.constraints
    assert "STANDSTILL_BRAKE_RELEASE" not in decision.constraints
    target, decision, _ = tick(node, 102.0, end - 0.8, 0.0)
    assert "STANDSTILL_BRAKE_RELEASE" not in decision.constraints
    target, decision, _ = tick(node, 102.3, end - 0.8, 0.0)
    assert target.need_stop and "STANDSTILL_BRAKE_RELEASE" in target.constraints
    # Rolling restarts the count.
    target, decision, _ = tick(node, 102.4, end - 0.7, 0.3)
    assert target.need_stop and "STANDSTILL_BRAKE_RELEASE" not in target.constraints
    target, decision, _ = tick(node, 103.5, end - 0.7, 0.0)
    assert "STANDSTILL_BRAKE_RELEASE" not in target.constraints
    target, decision, _ = tick(node, 105.6, end - 0.7, 0.0)
    assert "STANDSTILL_BRAKE_RELEASE" in target.constraints
    # Never under E-stop.
    target, decision, _ = tick(node, 105.7, end - 0.7, 0.0, estop=True)
    assert target.need_stop and "STANDSTILL_BRAKE_RELEASE" not in target.constraints
