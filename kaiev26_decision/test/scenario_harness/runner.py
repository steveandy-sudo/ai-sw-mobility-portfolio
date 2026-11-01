"""Run one scenario: real Decision nodes, plant, scripted perception, simulated clock.

Step order every dt (sim time, deterministic): plant integrates to t with the last command ->
/clock t -> node timers run -> the plant and the perception script publish what they would at t
-> subscriptions run -> one sample is recorded.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import copy
import itertools
import math
from pathlib import Path
import re
import time

from geometry_msgs.msg import TransformStamped
from kaiev26_msgs.msg import (
    ActuatorCommand,
    BehaviorDecision,
    Centerline,
    PerceptionObjectArray,
    RoadSegmentArray,
    TargetSpeed,
    TrafficLightObservationArray,
    TrafficSignalPhase,
    TrafficSignalPhaseArray,
    VehicleState,
)
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage
import yaml

from .course import load_course
from .launch_nodes import launch_node_specs
from .perception import LAMP_BITS, PerceptionDriver, SensorParams
from .plant import Command, Plant, VehicleParams
from .stack import InProcessStack, time_msg
from .world import MOVING_MPS, STOPPED_MPS, Truth, make_world

# Executables the runner starts from the launch; the launch decides which of them exist.
STACK_EXECUTABLES = (
    "route_zone_manager_node",
    "perception_gateway_node",
    "main_planning_engine_node",
    "motion_control_node",
    "signal_phase_tracker_node",
)
CLOCK_ORIGIN_SEC = 1000.0
SCENARIO_KEYS = {
    "name", "description", "course", "speed_mps", "event", "start", "vehicle", "perception",
    "duration_s", "dt_s", "warmup_s", "launch", "world", "expect", "matrix",
}


# ---------------------------------------------------------------- scenario files
def _lookup(variables: dict, dotted: str):
    value = variables
    for part in dotted.split("."):
        value = value[part]
    return value


def _substitute(node, variables: dict):
    if isinstance(node, dict):
        return {key: _substitute(value, variables) for key, value in node.items()}
    if isinstance(node, list):
        return [_substitute(value, variables) for value in node]
    if isinstance(node, str):
        whole = re.fullmatch(r"\$\{([\w.]+)\}", node)
        if whole:
            return copy.deepcopy(_lookup(variables, whole.group(1)))
        return re.sub(r"\$\{([\w.]+)\}", lambda m: f"{_lookup(variables, m.group(1))}", node)
    return node


def expand_file(path: Path) -> list[dict]:
    """A file is one scenario, or a template with `matrix:` (cartesian product of its lists)."""
    document = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"{path}: a scenario file is a mapping")
    unknown = set(document) - SCENARIO_KEYS
    if unknown:
        raise ValueError(f"{path}: unknown keys {sorted(unknown)}")
    matrix = document.pop("matrix", None)
    if not matrix:
        return [document]
    names = list(matrix)
    scenarios = []
    for combination in itertools.product(*(matrix[name] for name in names)):
        variables = dict(zip(names, combination))
        scenario = copy.deepcopy(document)
        # Repeat so a reference built from another one ("${ev.${brake.window}}") resolves too.
        for _ in range(4):
            expanded = _substitute(scenario, variables)
            if expanded == scenario:
                break
            scenario = expanded
        scenarios.append(scenario)
    return scenarios


# ---------------------------------------------------------------- recording
@dataclass
class Samples:
    columns: dict[str, list] = field(default_factory=dict)

    def add(self, **values) -> None:
        for key, value in values.items():
            self.columns.setdefault(key, []).append(value)

    def __getitem__(self, key: str) -> list:
        return self.columns[key]

    def __len__(self) -> int:
        return len(self.columns.get("t", []))


@dataclass
class RunResult:
    name: str
    scenario: dict
    course: object
    world: object
    samples: Samples
    vehicle: VehicleParams
    wall_s: float
    sim_s: float


# ---------------------------------------------------------------- run
def start_pose(scenario: dict, course, vehicle: VehicleParams) -> tuple[float, float, float, float]:
    start = scenario.get("start") or {}
    if "route_s" in start:
        rear_s = float(start["route_s"])
    else:
        event = course.event(start.get("event", scenario.get("event")))
        rear_s = event.line_s - float(start["distance_before_line_m"]) - vehicle.front_overhang_m
    x, y, yaw = course.route.pose_at(rear_s, float(start.get("lateral_m", 0.0)))
    return x, y, yaw, rear_s


class _Inputs:
    """Latest Decision outputs the harness listens to."""

    def __init__(self) -> None:
        self.command: ActuatorCommand | None = None
        self.target: TargetSpeed | None = None
        self.behavior: BehaviorDecision | None = None


def run_scenario(scenario: dict) -> RunResult:
    wall_start = time.monotonic()
    course_name = scenario["course"]
    speed = float(scenario["speed_mps"])
    launch_args = dict(scenario.get("launch") or {})
    specs = [s for s in launch_node_specs(course_name, speed, launch_args) if s.executable in STACK_EXECUTABLES]
    rz = next(s for s in specs if s.executable == "route_zone_manager_node").parameters
    course = load_course(course_name, rz["route_config"], rz["mission_policy_config"], rz["landmark_config"])
    vehicle = VehicleParams.from_dict(scenario.get("vehicle"))
    sensors = SensorParams.from_dict(scenario.get("perception"))
    default_event = scenario.get("event")
    world = make_world(scenario.get("world") or {}, default_event, course)
    driver = PerceptionDriver(course, world, sensors)
    x, y, yaw, rear_s = start_pose(scenario, course, vehicle)
    v0 = float((scenario.get("start") or {}).get("speed_mps", 0.0))
    plant = Plant(vehicle, x, y, yaw, v0)
    dt = float(scenario.get("dt_s", 0.02))
    warmup = float(scenario.get("warmup_s", 2.0))
    duration = float(scenario["duration_s"])
    command_topic = next(s for s in specs if s.executable == "motion_control_node").parameters["command_topic"]
    planning = next(s for s in specs if s.executable == "main_planning_engine_node").parameters
    if world.phases is not None:
        # world.event_phases is published in place of the tracker node.
        specs = [s for s in specs if s.executable != "signal_phase_tracker_node"]

    stack = InProcessStack(specs, CLOCK_ORIGIN_SEC - warmup)
    inputs = _Inputs()
    harness = stack.harness
    reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
    harness.create_subscription(ActuatorCommand, command_topic, lambda m: setattr(inputs, "command", m), reliable)
    harness.create_subscription(TargetSpeed, "/planning/target_speed", lambda m: setattr(inputs, "target", m), reliable)
    harness.create_subscription(BehaviorDecision, "/planning/behavior_decision", lambda m: setattr(inputs, "behavior", m), reliable)
    pubs = {
        "odometry": harness.create_publisher(Odometry, "/localization/odometry", reliable),
        "tf": harness.create_publisher(TFMessage, "/tf", QoSProfile(depth=100)),
        "vehicle_state": harness.create_publisher(VehicleState, "/vehicle/state", reliable),
        "road_segments": harness.create_publisher(RoadSegmentArray, "/perception/road_segments", reliable),
        "centerline": harness.create_publisher(Centerline, "/perception/centerline", reliable),
        "objects": harness.create_publisher(PerceptionObjectArray, "/perception/objects", reliable),
        "traffic_lights": harness.create_publisher(TrafficLightObservationArray, "/perception/traffic_lights", reliable),
        "signal_phases": harness.create_publisher(
            TrafficSignalPhaseArray, planning.get("signal_phases_topic", "/planning/traffic_signal_phases"), reliable
        ),
    }
    samples = Samples()
    route_s = rear_s
    stopped_since = None if v0 > STOPPED_MPS else -warmup
    moving_since = -warmup if v0 > MOVING_MPS else None
    next_state_t = -warmup
    next_frame_t = -warmup
    next_phase_t = -warmup
    state_period = 0.02
    # The tracker node publishes at publish_rate_hz 10.
    phase_period = 0.1
    frame_period = 1.0 / sensors.rate_hz
    t = -warmup
    steps = int(round((warmup + duration) / dt))
    try:
        for step in range(steps + 1):
            t = -warmup + step * dt
            auto = t >= -1.0e-9
            if auto and step > 0:
                command = inputs.command
                if command is not None:
                    stamp = command.header.stamp.sec + command.header.stamp.nanosec * 1.0e-9 - CLOCK_ORIGIN_SEC
                    plant.set_command(Command(stamp, float(command.speed_target_mps),
                                              float(command.steering_target_rad), bool(command.brake_engage)))
                plant.step(t, dt)
            projection = course.route.project(plant.x, plant.y, route_s)
            route_s = projection.s
            fx, fy = plant.front_bumper()
            front_s = course.route.project(fx, fy, route_s + vehicle.front_overhang_m).s
            if plant.v <= STOPPED_MPS:
                stopped_since = t if stopped_since is None else stopped_since
            else:
                stopped_since = None
            if plant.v > MOVING_MPS:
                moving_since = t if moving_since is None else moving_since
            else:
                moving_since = None
            truth = Truth(t, plant.v, route_s, front_s, stopped_since, moving_since)
            world.update(truth)

            stack.advance_to(CLOCK_ORIGIN_SEC + t)
            stamp = time_msg(CLOCK_ORIGIN_SEC + t)
            pose = (plant.x, plant.y, plant.yaw)
            if t >= next_state_t - 1.0e-9:
                next_state_t += state_period
                _publish_vehicle(pubs, world, stamp, plant, auto)
            if t >= next_frame_t - 1.0e-9:
                next_frame_t += frame_period
                frame = {
                    "road_segments": driver.road_segments(stamp, pose),
                    "centerline": driver.centerline(stamp, pose, route_s),
                    "objects": driver.objects(stamp, pose),
                    "traffic_lights": driver.traffic_lights(stamp, pose, t),
                }
                for topic, message in frame.items():
                    if not world.dropped(topic):
                        pubs[topic].publish(message)
            if world.phases is not None and t >= next_phase_t - 1.0e-9:
                next_phase_t += phase_period
                if not world.dropped("signal_phases"):
                    _publish_phases(pubs, world.phases, stamp, t)
            stack.drain()

            command, target, behavior = inputs.command, inputs.target, inputs.behavior
            samples.add(
                t=t, auto=auto, x=plant.x, y=plant.y, yaw=plant.yaw, v=plant.v, steer=plant.steer,
                s=route_s, lateral=projection.lateral, front_s=front_s,
                cmd_speed=float(command.speed_target_mps) if command else math.nan,
                cmd_brake=bool(command.brake_engage) if command else False,
                cmd_steer=float(command.steering_target_rad) if command else math.nan,
                brake_applied=plant.brake_applied, drive_active=plant.drive_active,
                ts_need_stop=bool(target.need_stop) if target else False,
                ts_target=float(target.target_speed_mps) if target else math.nan,
                ts_distance=float(target.stop_target_distance) if target else math.nan,
                ts_source=target.source_behavior if target else "",
                behavior=behavior.active_behavior if behavior else "",
                fsm=behavior.fsm_state if behavior else "",
                finish_held=("FINISH_HELD" in list(behavior.constraints)) if behavior else False,
                lights={light_id: script.view(t) for light_id, script in world.lights.items()},
            )
    finally:
        stack.shutdown()
    return RunResult(scenario.get("name", "?"), scenario, course, world, samples, vehicle,
                     time.monotonic() - wall_start, duration)


def _publish_phases(pubs, script, stamp, t: float) -> None:
    view = script.view(t)
    if not view.published:
        return
    array = TrafficSignalPhaseArray()
    array.header.stamp = stamp
    array.header.frame_id = "map"
    event = script.event
    array.phases = [
        TrafficSignalPhase(
            light_id=light_id, on_route=True, route_event_id=event.event_id,
            stop_line_id=event.stop_line_id, aligned=view.aligned, permitted_now=view.permitted,
            phase_remaining_sec=view.until_change_s, permission_change_sec=view.until_change_s,
            permission_probability=view.probability,
            predicted_lamps=sum(LAMP_BITS[name] for name in view.lamps),
        )
        for light_id in event.light_ids
    ]
    pubs["signal_phases"].publish(array)


def _publish_vehicle(pubs, world, stamp, plant: Plant, auto: bool) -> None:
    flipped = world.heading_flipped()
    yaw = plant.yaw + (math.pi if flipped else 0.0)
    qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    if not world.dropped("odometry"):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = plant.x
        odom.pose.pose.position.y = plant.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        covariance = [0.0] * 36
        covariance[0] = covariance[7] = 0.01
        covariance[14] = 0.01
        covariance[21] = covariance[28] = covariance[35] = 0.001
        odom.pose.covariance = covariance
        odom.twist.twist.linear.x = -plant.v if flipped else plant.v
        odom.twist.twist.angular.z = plant.v * math.tan(plant.steer) / plant.p.wheelbase_m
        odom.twist.covariance = [0.01 if i % 7 == 0 else 0.0 for i in range(36)]
        pubs["odometry"].publish(odom)
    if not world.dropped("tf"):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = plant.x
        transform.transform.translation.y = plant.y
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        pubs["tf"].publish(TFMessage(transforms=[transform]))
    if not world.dropped("vehicle_state"):
        state = VehicleState()
        state.header.stamp = stamp
        state.speed_mps = float(plant.v)
        state.steering_rad = float(plant.steer)
        state.gear = VehicleState.GEAR_FORWARD
        state.mode = VehicleState.MODE_AUTO if auto else VehicleState.MODE_MANUAL
        state.estop_active = False
        pubs["vehicle_state"].publish(state)
