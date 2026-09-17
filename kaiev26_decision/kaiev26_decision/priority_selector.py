from __future__ import annotations

from dataclasses import replace
import math

from kaiev26_msgs.msg import RouteContext, SceneSummary, VehicleState

from kaiev26_decision.common import ScenarioChoice
from kaiev26_decision.zone_policy import (
    EventPolicy,
    StagePolicy,
    ZonePolicy,
    ZonePolicyRuntime,
)


class PrioritySelector:
    def __init__(
        self,
        stopline_approach_distance_m: float,
        static_stop_enabled: bool = False,
        zone_policy: ZonePolicy | None = None,
        mission_mode: str = "all",
        test_stop_hold_sec: float = 3.0,
    ) -> None:
        self.stopline_approach_distance_m = stopline_approach_distance_m
        self.static_stop_enabled = static_stop_enabled
        self.policy_runtime = (
            ZonePolicyRuntime(zone_policy) if zone_policy is not None else None
        )
        if mission_mode not in {"all", "stopline", "avoidance"}:
            raise ValueError(f"unsupported mission_mode [{mission_mode}]")
        self.mission_mode = mission_mode
        self.test_stop_hold_sec = max(0.1, test_stop_hold_sec)
        self.committed_signal_event_ids: set[str] = set()

    def current_stage(self, route: RouteContext | None) -> StagePolicy | None:
        if self.policy_runtime is None or route is None:
            return None
        return self.policy_runtime.stage_for(route.current_zone)

    def active_event(self, route: RouteContext | None) -> EventPolicy | None:
        if self.policy_runtime is None or route is None:
            return None
        event = self.policy_runtime.event_for(
            route.current_zone,
            route.next_stop_line_id,
        )
        if event is None or self.mission_mode == "all":
            return event
        if self.mission_mode == "avoidance" or not event.stop_line_id:
            return None
        return replace(
            event,
            event_type="TIMED_STOP",
            traffic_light_ids=(),
            hold_sec=self.test_stop_hold_sec,
            arrow_maneuver="NONE",
        )

    def complete_event(self, event_id: str) -> None:
        if self.policy_runtime is not None:
            self.policy_runtime.complete(event_id)

    def reset_transient_state(self) -> None:
        self.committed_signal_event_ids.clear()

    def signal_control_enabled(self, route: RouteContext | None) -> bool:
        if self.mission_mode != "all":
            return False
        if self.policy_runtime is None:
            return True
        stage = self.current_stage(route)
        event = self.active_event(route)
        return (
            stage is not None
            and stage.traffic_light_enabled
            and event is not None
            and event.event_type == "SIGNAL_OBEY"
            and event.event_id not in self.committed_signal_event_ids
            and self.signal_identity_matches(event, route)
        )

    def obstacle_control_enabled(self, route: RouteContext | None) -> bool:
        if self.mission_mode == "stopline":
            return False
        if self.policy_runtime is None:
            return True
        stage = self.current_stage(route)
        return stage is not None and stage.obstacle_enabled

    @staticmethod
    def signal_identity_matches(
        event: EventPolicy,
        route: RouteContext | None,
    ) -> bool:
        if route is None or event.stop_line_id != route.next_stop_line_id:
            return False
        expected = set(event.traffic_light_ids)
        route_ids = set(route.next_traffic_light_ids)
        return bool(expected) and expected.issubset(route_ids)

    def choose(
        self,
        scene: SceneSummary,
        route: RouteContext | None,
        vehicle: VehicleState | None,
        estop_active: bool,
    ) -> ScenarioChoice:
        if estop_active or (vehicle is not None and vehicle.estop_active):
            return ScenarioChoice("EMERGENCY", "estop active")

        if (
            self.mission_mode != "stopline"
            and scene.perception_health in {"STALE", "LOST"}
        ):
            prefix = "zone policy " if self.policy_runtime is not None else ""
            return ScenarioChoice(
                "RECOVERY",
                f"{prefix}perception {scene.perception_health.lower()}",
            )

        if self.policy_runtime is not None and route is None:
            return ScenarioChoice("RECOVERY", "zone policy waiting for route context")

        if route is not None and not route.route_projection_valid:
            return ScenarioChoice("RECOVERY", "route projection invalid")

        stage = self.current_stage(route)
        stage_id = stage.stage_id if stage is not None else ""

        if route is not None and route.current_zone == "FINISH_ZONE":
            return ScenarioChoice("FINISH", "finish zone", stage_id=stage_id)

        if self.policy_runtime is not None and stage is None:
            return ScenarioChoice("RECOVERY", "zone policy stage unavailable")

        if scene.obstacle_on_path and self.obstacle_control_enabled(route):
            return ScenarioChoice(
                "OBSTACLE",
                "obstacle on ego path",
                stage_id=stage_id,
            )

        route_stopline_distance = (
            float(route.distance_to_stopline) if route is not None else math.inf
        )
        event = self.active_event(route)
        if event is not None and event.event_type == "TIMED_STOP":
            if 0.0 <= route_stopline_distance <= event.approach_distance_m:
                return ScenarioChoice(
                    "STATIC_STOP",
                    "timed stopline ahead",
                    stage_id=stage_id,
                    event_id=event.event_id,
                    stop_line_id=event.stop_line_id,
                    hold_sec=event.hold_sec,
                    brake_trigger_waypoint=event.brake_trigger_waypoint,
                    brake_trigger_route_s=event.brake_trigger_route_s,
                )
        elif (
            self.policy_runtime is None
            and self.static_stop_enabled
            and math.isfinite(route_stopline_distance)
            and 0.0 <= route_stopline_distance <= self.stopline_approach_distance_m
        ):
            return ScenarioChoice("STATIC_STOP", "configured test stopline ahead")

        if event is not None and event.event_type == "SIGNAL_OBEY":
            if 0.0 <= route_stopline_distance <= event.approach_distance_m:
                if not self.signal_identity_matches(event, route):
                    return ScenarioChoice(
                        "RECOVERY",
                        "zone policy signal identity mismatch",
                        stage_id=stage_id,
                        event_id=event.event_id,
                        stop_line_id=event.stop_line_id,
                    )
                if event.event_id in self.committed_signal_event_ids:
                    return ScenarioChoice(
                        "LANE_FOLLOW",
                        "signal pass already committed",
                        stage_id=stage_id,
                        event_id=event.event_id,
                        stop_line_id=event.stop_line_id,
                    )
                if (
                    self.light_allows_route(scene, route, event.arrow_maneuver)
                    and route_stopline_distance <= event.commit_distance_m
                ):
                    self.committed_signal_event_ids.add(event.event_id)
                    return ScenarioChoice(
                        "LANE_FOLLOW",
                        "signal pass committed near stopline",
                        stage_id=stage_id,
                        event_id=event.event_id,
                        stop_line_id=event.stop_line_id,
                        arrow_maneuver=event.arrow_maneuver,
                        brake_trigger_waypoint=event.brake_trigger_waypoint,
                        brake_trigger_route_s=event.brake_trigger_route_s,
                    )
                if self.light_requires_stop(
                    scene,
                    route,
                    mapped_signal_ahead=True,
                    arrow_maneuver=event.arrow_maneuver,
                ):
                    return ScenarioChoice(
                        "TRAFFIC_LIGHT",
                        "zone policy signal requires stop",
                        stage_id=stage_id,
                        event_id=event.event_id,
                        stop_line_id=event.stop_line_id,
                        arrow_maneuver=event.arrow_maneuver,
                        brake_trigger_waypoint=event.brake_trigger_waypoint,
                        brake_trigger_route_s=event.brake_trigger_route_s,
                    )
                return ScenarioChoice(
                    "LANE_FOLLOW",
                    "signal permits route maneuver",
                    stage_id=stage_id,
                    event_id=event.event_id,
                    stop_line_id=event.stop_line_id,
                    arrow_maneuver=event.arrow_maneuver,
                )
        elif self.policy_runtime is None:
            mapped_signal_ahead = (
                route is not None
                and bool(route.next_stop_line_id)
                and math.isfinite(float(route.distance_to_stopline))
            )
            if (
                self.light_requires_stop(scene, route, mapped_signal_ahead)
                and scene.stopline_distance <= self.stopline_approach_distance_m
            ):
                return ScenarioChoice("TRAFFIC_LIGHT", "mapped signal requires stop")

        return ScenarioChoice(
            "LANE_FOLLOW",
            "nominal lane follow",
            stage_id=stage_id,
        )

    @staticmethod
    def light_requires_stop(
        scene: SceneSummary,
        route: RouteContext | None,
        mapped_signal_ahead: bool,
        arrow_maneuver: str = "LEFT",
    ) -> bool:
        return scene.traffic_light_state in {
            SceneSummary.TRAFFIC_RED,
            SceneSummary.TRAFFIC_YELLOW,
        } or (
            mapped_signal_ahead
            and not PrioritySelector.light_allows_route(
                scene,
                route,
                arrow_maneuver,
            )
        )

    @staticmethod
    def light_allows_route(
        scene: SceneSummary,
        route: RouteContext | None,
        arrow_maneuver: str = "LEFT",
    ) -> bool:
        arrow_allows_maneuver = (
            scene.traffic_light_state == SceneSummary.TRAFFIC_ARROW
            and route is not None
            and arrow_maneuver in {"LEFT", "RIGHT"}
            and route.next_maneuver == arrow_maneuver
        )
        return (
            scene.traffic_light_state == SceneSummary.TRAFFIC_GREEN
            or arrow_allows_maneuver
        )
