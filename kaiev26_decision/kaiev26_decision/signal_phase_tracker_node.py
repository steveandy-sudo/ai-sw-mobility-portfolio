#!/usr/bin/env python3
"""Publish the map-wide traffic-signal phase estimate.

The main planning engine reads an aligned estimate of the event's own light as the signal input
(signal_schedule.py); the camera then only rejects it on a sure opposite reading.

The wall-clock prior (signal_phase_tracker module docstring) assumes the node clock is UTC. On
the vehicle it is given only after the node clock has matched the GNSS RMC time within
clock_check_max_offset_sec; until then, or when the check fails, the estimate comes from
observations alone. The check never stops the car. Under use_sim_time the prior is off unless
sim_prior_enabled, without the GNSS check; in Gazebo the cycle runs on (sim time +
traffic_signal_cycle_offset_sec), so sim_prior_mean_sec is that offset (the launcher's
--signal-cycle-offset, 30 for "o30").
"""

from __future__ import annotations

import datetime as dt
import json
import math
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from kaiev26_msgs.msg import TrafficLightObservationArray, TrafficSignalPhase, TrafficSignalPhaseArray
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from um980_msgs.msg import StatusVerbose
from visualization_msgs.msg import Marker, MarkerArray

from kaiev26_decision.common import spin_until_shutdown
from kaiev26_decision.perception_gateway_node import PerceptionGatewayNode, lights_by_map_id
from kaiev26_decision.signal_phase_tracker import (
    LAMP_ARROW,
    PhaseEstimate,
    SignalPhaseTracker,
    TrackerConfig,
    lamps_circle,
    lamps_name,
    load_map_signals,
    route_signals_from_policy,
)
from kaiev26_decision.zone_policy import load_zone_policy


LAMP_COLORS = {
    "RED": (1.0, 0.1, 0.1),
    "YELLOW": (1.0, 0.8, 0.0),
    "GREEN": (0.1, 0.9, 0.2),
    None: (0.55, 0.55, 0.55),
}


def default_map_semantics_path() -> str:
    return os.path.join(
        get_package_share_directory("kaiev26_maps"),
        "maps",
        "kcity",
        "kcity_course_semantics.yaml",
    )


def stamp_sec(stamp) -> float:
    return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)


def rmc_utc_sec(sentence: str) -> float | None:
    """UTC epoch seconds of a valid NMEA RMC sentence, else None."""
    fields = str(sentence).strip().split(",")
    if len(fields) < 10 or not fields[0].endswith("RMC") or fields[2] != "A":
        return None
    clock, date = fields[1], fields[9]
    try:
        day = dt.datetime(2000 + int(date[4:6]), int(date[2:4]), int(date[0:2]), tzinfo=dt.timezone.utc)
        return day.timestamp() + int(clock[0:2]) * 3600 + int(clock[2:4]) * 60 + float(clock[4:])
    except (ValueError, IndexError):
        return None


def light_label(light_id: str) -> str:
    return light_id.rsplit("_", 1)[-1]


def phase_text(estimate: PhaseEstimate) -> str:
    label = light_label(estimate.light_id)
    if not estimate.program_id:
        observed = lamps_name(estimate.last_observed_lamps) if estimate.last_observed_lamps else "-"
        return f"{label} no program, seen {observed}"
    if not math.isfinite(estimate.cycle_time_sec):
        return f"{label} ?"
    lamps = lamps_name(estimate.predicted_lamps)
    following = lamps_name(estimate.next_lamps)
    if estimate.aligned:
        text = (
            f"{label} {lamps} {estimate.remaining_sec:.0f}s "
            f"[{estimate.remaining_min_sec:.0f}-{estimate.remaining_max_sec:.0f}] -> {following}"
        )
    else:
        text = f"{label} {lamps}? -> {following}"
    if estimate.on_route:
        event = estimate.route_event_id.split("_", 1)[0]
        action = "go" if estimate.permitted_now else "wait"
        timing = f" {estimate.permission_change_sec:.0f}s" if estimate.aligned else " ?"
        text += f"\n{event} {estimate.route_maneuver} {action}{timing}"
    return text


class SignalPhaseTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("kaiev26_signal_phase_tracker")
        defaults = TrackerConfig()
        self.declare_parameter("traffic_lights_topic", "/perception/traffic_lights")
        self.declare_parameter("phases_topic", "/planning/traffic_signal_phases")
        self.declare_parameter("marker_topic", "/planning/traffic_signal_phases/markers")
        self.declare_parameter("evidence_topic", "/planning/traffic_signal_phases/evidence")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("evidence_publish_period_sec", 1.0)
        self.declare_parameter("map_semantics_path", "")
        self.declare_parameter("mission_policy_config", "")
        self.declare_parameter("map_frame_id", "map")
        # Wall-clock prior on every controller's theta (K-City: cycles on the UTC 100 s grid).
        self.declare_parameter("wallclock_prior_enabled", True)
        self.declare_parameter("wallclock_prior_mean_sec", 0.0)
        self.declare_parameter("wallclock_prior_std_sec", 0.5)
        # Under use_sim_time the prior applies only with this; the mean is Gazebo's cycle offset.
        self.declare_parameter("sim_prior_enabled", False)
        self.declare_parameter("sim_prior_mean_sec", 0.0)
        self.declare_parameter("gnss_status_topic", "/gnss/status_verbose")
        self.declare_parameter("clock_check_max_offset_sec", 0.5)
        self.declare_parameter("clock_check_samples", 10)
        prior_fields = ("prior_mean_sec", "prior_std_sec")
        for name in TrackerConfig.__dataclass_fields__:
            if name not in prior_fields:
                self.declare_parameter(name, getattr(defaults, name))

        self.sim_time = bool(self.get_parameter("use_sim_time").value)
        prior_std = float(self.get_parameter("wallclock_prior_std_sec").value)
        if self.sim_time:
            prior_on = bool(self.get_parameter("sim_prior_enabled").value)
            self.prior_mean_sec = float(self.get_parameter("sim_prior_mean_sec").value)
        else:
            prior_on = bool(self.get_parameter("wallclock_prior_enabled").value)
            self.prior_mean_sec = float(self.get_parameter("wallclock_prior_mean_sec").value)
        self.prior_std_sec = prior_std if prior_on else 0.0
        config = TrackerConfig(
            **{
                name: type(getattr(defaults, name))(self.get_parameter(name).value)
                for name in TrackerConfig.__dataclass_fields__
                if name not in prior_fields
            },
            prior_mean_sec=self.prior_mean_sec,
            # On the vehicle the prior waits for the GNSS clock check (on_gnss_status).
            prior_std_sec=self.prior_std_sec if self.sim_time else 0.0,
        )
        map_path = str(self.get_parameter("map_semantics_path").value).strip() or default_map_semantics_path()
        policy_path = str(self.get_parameter("mission_policy_config").value).strip()
        policy = load_zone_policy(policy_path) if policy_path else None
        signals = load_map_signals(map_path)
        self.tracker = SignalPhaseTracker(signals, route_signals_from_policy(policy), config)
        for warning in self.tracker.warnings_at_build:
            self.get_logger().warning(warning)
        self.map_lights = list(signals.lights)
        self.map_frame_id = str(self.get_parameter("map_frame_id").value)
        self.evidence_period_sec = float(self.get_parameter("evidence_publish_period_sec").value)
        self.last_now_sec: float | None = None
        self.last_evidence_sec: float | None = None

        self.phases_pub = self.create_publisher(
            TrafficSignalPhaseArray, str(self.get_parameter("phases_topic").value), 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter("marker_topic").value), 10
        )
        self.evidence_pub = self.create_publisher(
            String, str(self.get_parameter("evidence_topic").value), 10
        )
        self.create_subscription(
            TrafficLightObservationArray,
            str(self.get_parameter("traffic_lights_topic").value),
            self.on_traffic_lights,
            10,
        )
        publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / publish_rate_hz, self.publish)

        self.clock_check_max_offset_sec = float(self.get_parameter("clock_check_max_offset_sec").value)
        self.clock_check_samples = max(1, int(self.get_parameter("clock_check_samples").value))
        self.clock_offsets: list[float] = []
        self.last_rmc = ""
        self.clock_checked = False
        if self.prior_std_sec > 0.0 and not self.sim_time:
            self.create_subscription(
                StatusVerbose,
                str(self.get_parameter("gnss_status_topic").value),
                self.on_gnss_status,
                10,
            )
        on_route = sorted(signal_id for signal_id in self.tracker.route)
        if self.tracker.prior_active:
            prior = f"theta prior {self.prior_mean_sec:g} +- {self.prior_std_sec:g} s"
        elif self.prior_std_sec > 0.0:
            prior = "theta prior waits for the GNSS clock check"
        else:
            prior = "no theta prior"
        self.get_logger().info(
            f"signal phase tracker: {len(self.map_lights)} lights, {len(self.tracker.groups)} "
            f"alignment groups ({config.coupling}), {prior}, route lights {on_route}, map {map_path}"
        )

    def on_gnss_status(self, msg: StatusVerbose) -> None:
        """Node clock against GNSS RMC time; the wall-clock prior is given only on a match."""
        if self.clock_checked or msg.latest_raw_rmc == self.last_rmc:
            return
        self.last_rmc = msg.latest_raw_rmc
        utc = rmc_utc_sec(msg.latest_raw_rmc)
        stamp = stamp_sec(msg.header.stamp)
        if utc is None or stamp <= 0.0:
            return
        # Header stamp (node clock at reception) minus the fix time; the smallest of several
        # leaves the least transport delay in it (2026-10-17: +0.035 to +0.084 s).
        self.clock_offsets.append(stamp - utc)
        if len(self.clock_offsets) < self.clock_check_samples:
            return
        self.clock_checked = True
        offset = min(self.clock_offsets)
        if abs(offset) > self.clock_check_max_offset_sec:
            self.get_logger().warning(
                f"system clock differs from GNSS RMC time by {offset:+.2f} s "
                f"(limit {self.clock_check_max_offset_sec:g} s); signal phase prior stays off, "
                "the estimate comes from observations only"
            )
            return
        self.tracker.set_prior(self.prior_mean_sec, self.prior_std_sec)
        self.get_logger().info(
            f"system clock matches GNSS RMC time ({offset:+.2f} s); "
            f"theta prior {self.prior_mean_sec:g} +- {self.prior_std_sec:g} s on"
        )

    def on_traffic_lights(self, msg: TrafficLightObservationArray) -> None:
        if not msg.lights:
            return
        stamp = stamp_sec(msg.header.stamp)
        clock_gap = self.get_clock().now().nanoseconds * 1.0e-9 - stamp
        if abs(clock_gap) > 5.0:
            # Predictions run on the node clock, so it must share the observation time base.
            self.get_logger().warning(
                f"traffic-light stamps are {clock_gap:.1f} s behind the node clock; check use_sim_time",
                throttle_duration_sec=5.0,
            )
        # Perception names the map light (track_id) and decides its state.
        for light_id, observation in lights_by_map_id(
            msg.lights, [light.light_id for light in self.map_lights]
        ).items():
            for warning in self.tracker.observe(
                light_id,
                stamp,
                PerceptionGatewayNode.map_light_observation(observation),
                PerceptionGatewayNode.map_light_arrow_active(observation),
                int(observation.lamps),
            ):
                self.get_logger().warning(f"signal duration evidence: {warning}")

    def publish(self) -> None:
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1.0e-9
        if self.last_now_sec is not None and now_sec < self.last_now_sec - 1.0:
            self.get_logger().warning("clock jumped back; resetting the signal phase estimate")
            self.tracker.reset()
            self.last_evidence_sec = None
        self.last_now_sec = now_sec
        estimates = self.tracker.estimate(now_sec)

        array = TrafficSignalPhaseArray()
        array.header.stamp = now.to_msg()
        array.header.frame_id = self.map_frame_id
        array.phases = [self.phase_message(estimate) for estimate in estimates]
        self.phases_pub.publish(array)
        self.marker_pub.publish(self.markers(estimates, array.header))

        if self.last_evidence_sec is None or now_sec - self.last_evidence_sec >= self.evidence_period_sec:
            self.last_evidence_sec = now_sec
            evidence = String()
            evidence.data = json.dumps({"stamp": round(now_sec, 3), **self.tracker.evidence()})
            self.evidence_pub.publish(evidence)

    @staticmethod
    def phase_message(estimate: PhaseEstimate) -> TrafficSignalPhase:
        msg = TrafficSignalPhase()
        msg.light_id = estimate.light_id
        msg.program_id = estimate.program_id
        msg.controller_id = estimate.controller_id
        msg.position = Point(x=estimate.x, y=estimate.y, z=estimate.z)
        msg.on_route = estimate.on_route
        msg.route_event_id = estimate.route_event_id
        msg.stop_line_id = estimate.stop_line_id
        msg.route_maneuver = estimate.route_maneuver
        msg.predicted_lamps = estimate.predicted_lamps
        msg.confidence = float(estimate.confidence)
        msg.phase_remaining_sec = float(estimate.remaining_sec)
        msg.phase_remaining_min_sec = float(estimate.remaining_min_sec)
        msg.phase_remaining_max_sec = float(estimate.remaining_max_sec)
        msg.next_lamps = estimate.next_lamps
        msg.aligned = estimate.aligned
        msg.alignment_width_sec = float(estimate.alignment_width_sec)
        msg.cycle_time_sec = float(estimate.cycle_time_sec)
        msg.period_sec = float(estimate.period_sec)
        msg.last_observed_lamps = estimate.last_observed_lamps
        msg.observation_age_sec = float(estimate.observation_age_sec)
        msg.permitted_now = estimate.permitted_now
        msg.permission_probability = float(estimate.permission_probability)
        msg.permission_change_sec = float(estimate.permission_change_sec)
        return msg

    @staticmethod
    def markers(estimates: list[PhaseEstimate], header) -> MarkerArray:
        markers = MarkerArray()
        for index, estimate in enumerate(estimates):
            alpha = 0.95 if estimate.on_route else 0.35
            size = 1.2 if estimate.on_route else 0.7
            known = math.isfinite(estimate.cycle_time_sec)
            rgb = LAMP_COLORS[lamps_circle(estimate.predicted_lamps) if known else None]

            lamp = Marker(header=header, ns="signal_phase_lamp", id=index, type=Marker.SPHERE)
            lamp.action = Marker.ADD
            lamp.pose.position = Point(x=estimate.x, y=estimate.y, z=estimate.z)
            lamp.pose.orientation.w = 1.0
            lamp.scale.x = lamp.scale.y = lamp.scale.z = size
            lamp.color.r, lamp.color.g, lamp.color.b = rgb
            lamp.color.a = alpha
            markers.markers.append(lamp)

            # Driver's left: the light yaw faces the approach, so left is yaw - pi/2.
            arrow = Marker(header=header, ns="signal_phase_arrow", id=index, type=Marker.ARROW)
            if known and estimate.predicted_lamps & LAMP_ARROW:
                arrow.action = Marker.ADD
                heading = estimate.yaw - 0.5 * math.pi
                arrow.points = [
                    Point(x=estimate.x, y=estimate.y, z=estimate.z - size),
                    Point(
                        x=estimate.x + 1.5 * size * math.cos(heading),
                        y=estimate.y + 1.5 * size * math.sin(heading),
                        z=estimate.z - size,
                    ),
                ]
                arrow.scale.x = 0.25 * size
                arrow.scale.y = 0.5 * size
                arrow.scale.z = 0.5 * size
                arrow.color.r, arrow.color.g, arrow.color.b = LAMP_COLORS["GREEN"]
                arrow.color.a = alpha
            else:
                arrow.action = Marker.DELETE
            markers.markers.append(arrow)

            text = Marker(header=header, ns="signal_phase_text", id=index, type=Marker.TEXT_VIEW_FACING)
            text.action = Marker.ADD
            text.pose.position = Point(x=estimate.x, y=estimate.y, z=estimate.z + 1.5 * size)
            text.pose.orientation.w = 1.0
            text.scale.z = 1.0 if estimate.on_route else 0.6
            text.color.r = text.color.g = text.color.b = 1.0 if estimate.on_route else 0.7
            text.color.a = 1.0 if estimate.on_route else 0.6
            text.text = phase_text(estimate)
            markers.markers.append(text)
        return markers


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SignalPhaseTrackerNode()
    spin_until_shutdown(node)


if __name__ == "__main__":
    main()
