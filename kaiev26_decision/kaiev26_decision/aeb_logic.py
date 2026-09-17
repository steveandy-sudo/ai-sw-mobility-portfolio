from __future__ import annotations

from kaiev26_msgs.msg import PerceptionObject, PerceptionObjectArray, VehicleState

from kaiev26_decision.common import INF_DISTANCE, stamp_age_sec


class AEBLogic:
    """Decision-local emergency braking decision.

    The result is consumed by the planner and leaves Decision only as
    ``ActuatorCommand.brake_engage``.
    """

    def __init__(
        self,
        *,
        enabled: bool,
        stale_objects_sec: float,
        obstacle_path_half_width_m: float,
        warning_ttc_sec: float,
        brake_ttc_sec: float,
        warning_distance_m: float,
        emergency_distance_m: float,
        bypass_brake_ttc_sec: float,
        bypass_emergency_distance_m: float,
        release_clear_ticks: int,
    ) -> None:
        self.enabled = enabled
        self.stale_objects_sec = stale_objects_sec
        self.obstacle_path_half_width_m = obstacle_path_half_width_m
        self.warning_ttc_sec = warning_ttc_sec
        self.brake_ttc_sec = brake_ttc_sec
        self.warning_distance_m = warning_distance_m
        self.emergency_distance_m = emergency_distance_m
        self.bypass_brake_ttc_sec = bypass_brake_ttc_sec
        self.bypass_emergency_distance_m = bypass_emergency_distance_m
        self.release_clear_ticks = release_clear_ticks
        self.state = "CLEAR"
        self.clear_ticks = 0

    def brake_required(
        self,
        now,
        objects: PerceptionObjectArray | None,
        vehicle_state: VehicleState | None,
        *,
        bypass_active: bool,
    ) -> bool:
        if not self.enabled:
            self.state = "DISABLED"
            self.clear_ticks = 0
            return False
        if objects is None or vehicle_state is None:
            self.state = "UNKNOWN"
            return False
        if stamp_age_sec(now, objects.header.stamp) > self.stale_objects_sec:
            self.state = "UNKNOWN"
            return False

        front_distance, ttc = self.compute_front_risk(objects, vehicle_state)
        brake_distance = (
            self.bypass_emergency_distance_m
            if bypass_active
            else self.emergency_distance_m
        )
        brake_ttc = self.bypass_brake_ttc_sec if bypass_active else self.brake_ttc_sec
        brake_condition = front_distance <= brake_distance or ttc <= brake_ttc
        warning_condition = (
            front_distance <= self.warning_distance_m or ttc <= self.warning_ttc_sec
        )

        if brake_condition:
            self.state = "BRAKE"
            self.clear_ticks = 0
            return True
        if self.state in {"BRAKE", "HOLD"} and self.clear_ticks < self.release_clear_ticks:
            self.state = "HOLD"
            self.clear_ticks += 1
            return True
        if warning_condition:
            self.state = "WARNING"
            self.clear_ticks = 0
            return False

        self.state = "RELEASE" if self.clear_ticks >= self.release_clear_ticks else "CLEAR"
        self.clear_ticks += 1
        return False

    def compute_front_risk(
        self,
        objects: PerceptionObjectArray,
        vehicle_state: VehicleState,
    ) -> tuple[float, float]:
        obstacle_classes = {
            PerceptionObject.CLASS_VEHICLE,
            PerceptionObject.CLASS_BIKE,
            PerceptionObject.CLASS_PEDESTRIAN,
            PerceptionObject.CLASS_TRAFFIC_CONE,
            PerceptionObject.CLASS_OBSTACLE,
        }
        nearest_distance = INF_DISTANCE
        nearest_ttc = INF_DISTANCE
        ego_speed = max(0.0, float(vehicle_state.speed_mps))

        for obj in objects.objects:
            if obj.class_id not in obstacle_classes:
                continue
            x = float(obj.pose.position.x)
            if x < 0.0:
                continue
            lateral_limit = self.obstacle_path_half_width_m + max(
                0.1, float(obj.size.y) * 0.5
            )
            if abs(float(obj.pose.position.y)) > lateral_limit:
                continue

            distance = max(0.0, x - max(0.0, float(obj.size.x) * 0.5))
            closing_speed = ego_speed - float(obj.twist.linear.x)
            ttc = distance / closing_speed if closing_speed > 0.05 else INF_DISTANCE
            nearest_distance = min(nearest_distance, distance)
            nearest_ttc = min(nearest_ttc, ttc)

        return nearest_distance, nearest_ttc
