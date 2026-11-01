"""Slow planning ticks must not make the planner read old route contexts and vehicle states.

10/17 14:57 (A' 0260a02) went to RECOVERY "zone policy route context stale" 55 times in the
obstacle zone while the route context reached the car's recorder without a gap (largest 69 ms,
stamp to receipt 3 ms p99). An rclpy executor pass runs the overdue timer and then takes one
message per subscription, the oldest queued. With ticks longer than the 50 ms input period a
depth-10 queue fills, every tick plans on a message up to 10 x 50 ms old, and the stamp check
reports it stale. B' read RouteContext and VehicleState with depth 10 too (2026-10-18 replay of
an earlier B': VEHICLE_STATE_STALE RECOVERY ten times in the obstacle zone). Depth 1 makes every
tick read the newest message, so the age it checks is the real age of its input.

Wall clock, real executor: the planner ticks take 80 ms (a loaded vehicle PC), the route context
and vehicle state come at 20 Hz from another thread, as on the car.
"""
from pathlib import Path
import threading
import time

from geometry_msgs.msg import Point
from kaiev26_msgs.msg import BehaviorDecision, RouteContext, VehicleState
import rclpy
from rclpy.executors import SingleThreadedExecutor

from kaiev26_decision.main_planning_engine_node import MainPlanningEngineNode


PACKAGE = Path(__file__).resolve().parents[1]
TICK_COST_SEC = 0.08
INPUT_PERIOD_SEC = 0.05
RUN_SEC = 3.0


class SlowPlanner(MainPlanningEngineNode):
    def on_timer(self) -> None:
        super().on_timer()
        time.sleep(TICK_COST_SEC)


def test_slow_ticks_read_the_newest_inputs(monkeypatch) -> None:
    monkeypatch.setenv("ROS_DOMAIN_ID", "46")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    route_config = PACKAGE / "waypoints" / "kcity_final_route.yaml"
    args = ["--ros-args", "-p", f"route_config:={route_config}",
            "-p", f"mission_policy_config:={PACKAGE / 'config' / 'final_policy.yaml'}",
            "-p", f"landmark_config:={PACKAGE / 'config' / 'final_landmarks.yaml'}",
            "-p", "mission_mode:=avoidance", "-p", "base_speed_mps:=5.0"]
    rclpy.init(args=args)
    planner = SlowPlanner()
    feeder = rclpy.create_node("backlog_feeder", use_global_arguments=False)
    decisions: list[BehaviorDecision] = []
    feeder.create_subscription(BehaviorDecision, "/planning/behavior_decision", decisions.append, 100)
    route_pub = feeder.create_publisher(RouteContext, "/planning/route_context", 10)
    vehicle_pub = feeder.create_publisher(VehicleState, "/vehicle/state", 10)
    length = planner.route_geometry.total_length

    def feed() -> None:
        stamp = feeder.get_clock().now().to_msg()
        route = RouteContext()
        route.header.stamp = stamp
        route.header.frame_id = "base_footprint"
        route.route_projection_valid = True
        route.projection_confidence = 0.9
        route.progress_s = 10.0
        route.distance_to_finish = length - 10.0
        route.local_path_points = [Point(x=float(x)) for x in range(20)]
        route_pub.publish(route)
        vehicle = VehicleState(mode=VehicleState.MODE_AUTO, speed_mps=0.0)
        vehicle.header.stamp = stamp
        vehicle_pub.publish(vehicle)

    feeder.create_timer(INPUT_PERIOD_SEC, feed)
    feeder_executor = SingleThreadedExecutor()
    feeder_executor.add_node(feeder)
    planner_executor = SingleThreadedExecutor()
    planner_executor.add_node(planner)
    stop = threading.Event()

    def spin(executor) -> None:
        while not stop.is_set():
            executor.spin_once(timeout_sec=0.01)

    threads = [threading.Thread(target=spin, args=(executor,)) for executor in (feeder_executor, planner_executor)]
    try:
        for thread in threads:
            thread.start()
        time.sleep(RUN_SEC)
    finally:
        stop.set()
        for thread in threads:
            thread.join(timeout=5.0)
        planner.destroy_node()
        feeder.destroy_node()
        rclpy.shutdown()

    # The first second lets discovery settle; then every tick must have fresh inputs.
    settled = decisions[len(decisions) // 3:]
    assert len(settled) >= 5, f"only {len(decisions)} decisions in {RUN_SEC} s"
    stale = [(d.fsm_state, [c for c in d.constraints if "STALE" in c or "MISSING" in c]) for d in settled
             if d.active_behavior == "RECOVERY"]
    assert not stale, f"{len(stale)}/{len(settled)} ticks in RECOVERY: {stale[:3]}"
