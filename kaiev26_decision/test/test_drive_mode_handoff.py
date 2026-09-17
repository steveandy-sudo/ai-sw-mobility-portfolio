from geometry_msgs.msg import Point
from kaiev26_msgs.msg import RouteContext, SceneSummary, VehicleState

from kaiev26_decision.main_planning_engine_node import MainPlanningEngineNode


class Resettable:
    def __init__(self) -> None:
        self.reset_count = 0

    def reset(self) -> None:
        self.reset_count += 1


class LaneResettable(Resettable):
    def __init__(self) -> None:
        super().__init__()
        self.rejoin_requested = False

    def request_route_rejoin(self) -> None:
        self.rejoin_requested = True


class SelectorResettable:
    def __init__(self) -> None:
        self.reset_count = 0

    def reset_transient_state(self) -> None:
        self.reset_count += 1


class PlanningCoreResettable:
    def __init__(self) -> None:
        self.reset_count = 0

    def reset_transient_state(self) -> None:
        self.reset_count += 1


def make_node() -> MainPlanningEngineNode:
    node = MainPlanningEngineNode.__new__(MainPlanningEngineNode)
    node.manual_override_active = False
    node.latched_mission_choice = None
    node.selector = SelectorResettable()
    node.lane_fsm = LaneResettable()
    node.traffic_light_fsm = Resettable()
    node.static_stop_fsm = Resettable()
    node.obstacle_fsm = Resettable()
    node.planning_core = PlanningCoreResettable()
    return node


def make_route() -> RouteContext:
    route = RouteContext()
    route.route_projection_valid = True
    route.local_path_points = [Point(x=0.0), Point(x=10.0)]
    return route


def test_manual_mode_suspends_autonomy_without_requesting_a_stop() -> None:
    node = make_node()
    node.latest_route_context = make_route()
    node.latest_vehicle_state = VehicleState()
    node.latest_vehicle_state.mode = VehicleState.MODE_MANUAL
    node.latest_vehicle_state.speed_mps = 2.7

    command = node.run_planning_tick(SceneSummary())

    assert command.active_behavior == "MANUAL_OVERRIDE"
    assert command.fsm_state == "MANUAL_CONTROL"
    assert command.target_speed_limit_mps == 2.7
    assert command.path_request == "ROUTE_LOCAL_PATH"
    assert not command.need_stop
    assert node.selector.reset_count == 1


def test_auto_resume_clears_transient_fsms_and_requests_route_rejoin() -> None:
    node = make_node()
    route = make_route()
    manual = VehicleState()
    manual.mode = VehicleState.MODE_MANUAL
    autonomous = VehicleState()
    autonomous.mode = VehicleState.MODE_AUTO

    assert node.update_drive_mode(route, manual) is not None
    assert node.update_drive_mode(route, autonomous) is None

    assert node.selector.reset_count == 2
    assert node.traffic_light_fsm.reset_count == 2
    assert node.static_stop_fsm.reset_count == 2
    assert node.obstacle_fsm.reset_count == 2
    assert node.planning_core.reset_count == 2
    assert node.lane_fsm.rejoin_requested
    assert not node.manual_override_active
