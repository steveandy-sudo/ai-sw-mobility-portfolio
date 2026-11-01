"""Obstacle zone, obstacle world and obstacle behavior: situation -> result tables."""
from functools import lru_cache
import math
from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import Point
from kaiev26_msgs.msg import PerceptionObject, PerceptionObjectArray, RouteContext
import pytest

from kaiev26_decision.obstacle_behavior import (
    BYPASS, BYPASS_FORCED, FALLBACK, IDLE, IN_ZONE, PRE_SLOW, RETURN, ObstacleBehavior,
)
from kaiev26_decision.obstacle_sequence import ObstacleSequencePlanner, Row
from kaiev26_decision.obstacle_world import ObstacleWorld, ObstacleZone, Pose2D
from kaiev26_decision.route_zone_manager_node import PolylineRoute, RouteWaypoint, load_polyline_route
from kaiev26_decision.zone_policy import load_zone_policy


PACKAGE = Path(__file__).resolve().parents[1]


@lru_cache(maxsize=None)
def final_zone():
    route, _ = load_polyline_route(str(PACKAGE / "waypoints" / "kcity_final_route.yaml"))
    policy = load_zone_policy(PACKAGE / "config" / "final_policy.yaml")
    return route, ObstacleZone(policy.obstacle_zone, route)


def test_final_zone_runs_from_wp_640_to_wp_670_before_the_finish_brake() -> None:
    route, zone = final_zone()
    assert zone.entry_s == pytest.approx(639.849483634)  # WP 640, the speed cap entry
    assert zone.polygon_entry_s == pytest.approx(639.849, abs=0.01)
    assert zone.exit_s == pytest.approx(669.840, abs=0.01)  # WP 670
    # Zone end + 12 m return before the finish brake point (route end - 1 m), D18.
    assert zone.exit_s + 12.0 < route.total_length - 1.0


@pytest.mark.parametrize("progress", [600.0, 650.0, 700.0])
def test_final_zone_lane_lines_come_from_the_map_on_both_sides(progress) -> None:
    _, zone = final_zone()
    right, left = zone.lane_lines(progress, ahead_m=20.0)
    # The car's own painted lane (4.10 m, notes/30): right edge line and centre line.
    assert all(-2.6 < d < -1.5 for _, d in right)
    assert all(1.5 < d < 2.6 for _, d in left)


@pytest.mark.parametrize("progress", [625.0, 632.0, 640.0])
def test_gaps_in_the_painted_lines_are_bridged(progress) -> None:
    # The map has no centre-line samples between about 622 and 646 m on the final.
    _, zone = final_zone()
    lines = zone.lane_lines(progress, ahead_m=20.0)
    assert len(lines) == 2
    for line in lines:
        assert line[0][0] <= 0.0 and line[-1][0] >= 20.0


def test_midan_zone_is_the_fixed_final_lane_from_its_start_to_the_route_end() -> None:
    route = straight_route(112.0)
    zone = ObstacleZone(load_zone_policy(PACKAGE / "config" / "midan_test_policy.yaml").obstacle_zone, route)
    assert (zone.fixed_lane, zone.entry_s, zone.exit_s) == (True, 21.0, pytest.approx(route.total_length))
    right, left = zone.lane_lines(50.0, ahead_m=20.0)
    assert {d for _, d in right} == {-1.96} and {d for _, d in left} == {2.14}
    assert right[0][0] <= -5.0 and right[-1][0] >= 20.0
    # No polygon: every object counts, wherever it is.
    world = ObstacleWorld(route, zone)
    world.observe(frame(drum(30.0, -8.0, 1), drum(60.0, 1.0, 2)), Pose2D(0.0, 0.0, 0.0), 1.0)
    assert len(world.kept) == 2


def drum(x, y, track_id=0, size=0.5):
    obj = PerceptionObject()
    obj.class_id = PerceptionObject.CLASS_OBSTACLE
    obj.track_id = track_id
    obj.pose.position.x, obj.pose.position.y = float(x), float(y)
    obj.pose.orientation.w = 1.0
    obj.size.x = obj.size.y = float(size)
    return obj


def frame(*objects):
    message = PerceptionObjectArray()
    message.header.frame_id = "map"
    message.objects = list(objects)
    return message


def straight_route(length=200.0):
    return PolylineRoute([RouteWaypoint(x=float(x), y=0.0) for x in range(0, int(length) + 1, 5)])


class Box:
    """A zone polygon on the straight route: x 50..150, y -3..6."""

    polygon = ((50.0, -3.0), (150.0, -3.0), (150.0, 6.0), (50.0, 6.0))

    def contains(self, x, y):
        return 50.0 <= x <= 150.0 and -3.0 <= y <= 6.0


# frames of (time, objects) -> kept (route s, lateral) positions, sorted
WORLD_CASES = {
    "tracked_object_keeps_one_entry_and_its_latest_position": (
        [(1.0, [drum(60.0, -1.0, 7)]), (1.1, [drum(60.2, -1.0, 7)])],
        [(60.2, -1.0)],
    ),
    "objects_out_of_view_are_kept": (
        [(1.0, [drum(60.0, -1.0, 7), drum(66.0, 1.0, 8)]), (1.1, [drum(66.0, 1.0, 8)])],
        [(60.0, -1.0), (66.0, 1.0)],
    ),
    "untracked_object_joins_the_nearest_kept_one_within_one_metre": (
        [(1.0, [drum(60.0, -1.0)]), (1.1, [drum(60.6, -1.0)])],
        [(60.6, -1.0)],
    ),
    "untracked_object_far_from_any_kept_one_is_new": (
        [(1.0, [drum(60.0, -1.0)]), (1.1, [drum(61.5, -1.0)])],
        [(60.0, -1.0), (61.5, -1.0)],
    ),
    "objects_outside_the_zone_are_ignored": (
        [(1.0, [drum(40.0, 0.0, 1), drum(60.0, -5.0, 2), drum(60.0, 5.0, 3)])],
        [(60.0, 5.0)],
    ),
    "fragments_inside_a_later_merged_row_are_dropped": (
        [(1.0, [drum(80.0, -0.6, 11), drum(80.0, -1.4, 12)]),
         (1.1, [drum(80.0, -1.0, 13, size=1.6)])],
        [(80.0, -1.0)],
    ),
    "an_older_frame_is_ignored": (
        [(1.0, [drum(60.0, -1.0, 7)]), (0.9, [drum(70.0, -1.0, 9)])],
        [(60.0, -1.0)],
    ),
}


@pytest.mark.parametrize("name", sorted(WORLD_CASES))
def test_obstacle_world(name) -> None:
    frames, expected = WORLD_CASES[name]
    world = ObstacleWorld(straight_route(), Box(), association_m=1.0)
    for stamp, objects in frames:
        world.observe(frame(*objects), Pose2D(0.0, 0.0, 0.0), stamp)
    kept = sorted((round(item.route_s, 3), round(item.y, 3)) for item in world.kept.values())
    assert kept == expected


@pytest.mark.parametrize("rear_s, kept", [(60.0, 1), (61.5 + 0.62 + 1.5 - 0.01, 1), (61.5 + 0.62 + 1.5 + 0.01, 0)])
def test_objects_are_forgotten_only_after_the_car_passed_them(rear_s, kept) -> None:
    world = ObstacleWorld(straight_route(), Box())
    world.observe(frame(drum(61.5, -1.0, 4)), Pose2D(0.0, 0.0, 0.0), 1.0)
    world.forget_passed(rear_s)
    assert len(world.kept) == kept


def test_kept_objects_are_given_in_the_route_context_frame() -> None:
    world = ObstacleWorld(straight_route(), Box())
    world.observe(frame(drum(70.0, 1.0, 4)), Pose2D(0.0, 0.0, 0.0), 1.0)
    header = RouteContext().header
    local = world.local_objects(Pose2D(60.0, 0.5, math.pi / 2), header).objects[0]
    assert (local.pose.position.x, local.pose.position.y) == pytest.approx((0.5, -10.0))


# Obstacle behavior ---------------------------------------------------------------------------

def planner(**overrides):
    return ObstacleSequencePlanner(
        vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.5,
        route_half_width_m=1.625, centerline_departure_m=0.5, **overrides,
    )


def route_at(progress, length=36):
    route = RouteContext()
    route.route_projection_valid = True
    route.projection_confidence = 0.9
    route.progress_s = float(progress)
    route.local_path_points = [Point(x=float(x), y=0.0) for x in range(length)]
    return route


def rows_ahead(progress, first_s, second_s=None, side=-1):
    """Local objects of rows at route stations first_s and second_s (first row on `side`)."""
    message = PerceptionObjectArray()
    for station, y_sign in ((first_s, side), (second_s, -side)):
        if station is None:
            continue
        for y in (0.8, 1.3):
            message.objects.append(drum(station - progress, y_sign * y))
    return message


def plan(sequence, objects):
    """The path the planner searches for these objects with the car at the route start."""
    scene = sequence.observe(route_at(0.0).local_path_points, objects)
    return None if scene is None else sequence.search(scene)


def tick(behavior, progress, objects, *, perception_ok=True, now=None):
    return behavior.update(
        route_at(progress), mode_enabled=True, stage_enabled=True,
        objects=objects, perception_ok=perception_ok,
        now_sec=now if now is not None else 10.0 + 0.1 * progress,
    )


def test_bypass_is_committed_once_and_kept_while_perception_is_lost() -> None:
    behavior = ObstacleBehavior(planner())
    first = tick(behavior, 100.0, rows_ahead(100.0, 112.0, 118.0))
    assert first.path is not None and "OBSTACLE_BYPASS_COMMITTED" in first.tags
    assert behavior.state == BYPASS
    assert first.cap_mps is not None and first.cap_mps > 1.0  # path speed, not the fallback

    same = tick(behavior, 101.0, rows_ahead(101.0, 112.0, 118.0))
    assert same.path is not None and "OBSTACLE_BYPASS_COMMITTED" not in same.tags

    lost = tick(behavior, 102.0, None, perception_ok=False)
    assert lost.path is not None and "OBSTACLE_PERCEPTION_LOST_KEEP_PATH" in lost.tags
    assert lost.cap_mps == pytest.approx(same.cap_mps, rel=0.05)
    assert lost.stop_distance_m is None


def test_a_zone_without_its_own_cap_slows_to_the_bypass_speed_once_drums_are_seen() -> None:
    # Midan: the zone cap is 15 m/s, so the course speed holds until a bypass path is followed.
    zone = SimpleNamespace(entry_s=1.0, exit_s=500.0)
    behavior = ObstacleBehavior(planner(), zone=zone, zone_speed_mps=15.0, bypass_speed_mps=3.0)
    clear = tick(behavior, 100.0, PerceptionObjectArray())
    assert clear.cap_mps == pytest.approx(15.0) and clear.path is None
    bypass = tick(behavior, 101.0, rows_ahead(101.0, 112.0, 118.0))
    assert bypass.path is not None and bypass.cap_mps <= 3.0


def test_a_new_row_replans_the_committed_bypass() -> None:
    behavior = ObstacleBehavior(planner())
    tick(behavior, 100.0, rows_ahead(100.0, 114.0))
    stations = behavior.committed.row_stations
    second = tick(behavior, 101.0, rows_ahead(101.0, 114.0, 120.0))
    assert "OBSTACLE_BYPASS_COMMITTED" in second.tags
    assert len(behavior.committed.row_stations) > len(stations)


def test_bypass_is_released_at_its_return_point() -> None:
    behavior = ObstacleBehavior(planner())
    tick(behavior, 100.0, rows_ahead(100.0, 112.0, 118.0))
    back = behavior.committed.return_s
    # Past the last row the bypass path leads back to the route.
    tick(behavior, behavior.committed.row_stations[-1] + behavior.planner.rear + 1.0, PerceptionObjectArray())
    assert behavior.state == RETURN
    output = tick(behavior, back + 0.1, PerceptionObjectArray())
    assert "OBSTACLE_BYPASS_RETURNED" in output.tags
    assert output.path is None and behavior.committed is None


def test_the_bypass_cap_ends_midway_between_the_two_rows() -> None:
    # 2026-10-24: accelerate out from between the drum rows, not once past the last one.
    behavior = ObstacleBehavior(planner(), bypass_speed_mps=1.0)
    tick(behavior, 100.0, rows_ahead(100.0, 112.0, 118.0))
    first, last = behavior.committed.row_stations
    midway = behavior.committed.release_s
    assert midway == pytest.approx(0.5 * (first + last))
    before = tick(behavior, midway - 0.2, PerceptionObjectArray(), perception_ok=False)
    assert behavior.state == BYPASS and before.cap_mps <= 1.0
    after = tick(behavior, midway + 0.1, PerceptionObjectArray(), perception_ok=False)
    assert behavior.state == RETURN and after.cap_mps > 1.0  # the path curvature alone
    assert after.path is not None and behavior.bypass_active


def test_a_replan_past_the_first_row_keeps_the_midpoint() -> None:
    behavior = ObstacleBehavior(planner())
    tick(behavior, 100.0, rows_ahead(100.0, 112.0, 118.0))
    midway = behavior.committed.release_s
    last = behavior.committed.row_stations[-1]
    assert behavior.release_station((last + 0.3,)) == midway
    assert behavior.release_station((last + 10.0,)) == math.inf


def test_a_row_is_planned_once_its_full_return_fits_the_local_route() -> None:
    # Local route 35 m, return 10 m: a row whose far end is more than 25 m ahead is not planned
    # yet, so no return cut off at the route end is committed and there is no fallback speed.
    behavior = ObstacleBehavior(planner())
    far = tick(behavior, 100.0, rows_ahead(100.0, 128.0))
    assert far.path is None and far.cap_mps is None and behavior.committed is None
    near = tick(behavior, 104.0, rows_ahead(104.0, 128.0))
    assert "OBSTACLE_BYPASS_COMMITTED" in near.tags
    assert behavior.committed.return_s >= 128.25 + 10.0 - 1.0e-6


def test_a_row_near_the_route_end_returns_at_the_route_end() -> None:
    behavior = ObstacleBehavior(planner(), route_length_m=130.0)
    output = tick(behavior, 100.0, rows_ahead(100.0, 128.0))
    assert output.path is not None and behavior.committed.return_s == pytest.approx(135.0)


def test_blocked_lane_drives_on_slowly_and_never_stops() -> None:
    behavior = ObstacleBehavior(planner())
    blocked = PerceptionObjectArray(objects=[drum(12.0, y) for y in (-1.4, -0.7, 0.0, 0.7, 1.4, 2.1)])
    output = tick(behavior, 100.0, blocked)
    assert behavior.state == BYPASS_FORCED  # the fixed lane-edge S
    assert output.cap_mps is not None and output.cap_mps <= 1.0
    assert output.stop_distance_m is None


# Committed gap [m], forced -> clearance speed cap; zone 3.0, fallback 1.6, slow 0.25, target 0.5.
CLEARANCE_SPEED_CASES = [
    (0.50, False, None),
    (0.62, False, None),
    (0.375, False, 2.3),
    (0.25, False, 1.6),
    (0.10, False, 1.6),
    (0.50, True, 1.6),
    (math.nan, False, 1.6),
]


@pytest.mark.parametrize("gap, forced, cap", CLEARANCE_SPEED_CASES)
def test_bypass_speed_follows_the_committed_clearance(gap, forced, cap) -> None:
    behavior = ObstacleBehavior(planner(), zone_speed_mps=3.0, fallback_speed_mps=1.6, slow_clearance_m=0.25)
    assert behavior.clearance_speed(gap, forced) == (pytest.approx(cap) if cap is not None else None)


def recorded_search(behavior, finds_nothing=False):
    calls = []
    search = behavior.planner.search

    def recorded(*args, **kwargs):
        calls.append(kwargs)
        return None if finds_nothing else search(*args, **kwargs)

    behavior.planner.search = recorded
    return calls


def test_the_planner_searches_only_when_the_committed_path_needs_replacing() -> None:
    behavior = ObstacleBehavior(planner())
    calls = recorded_search(behavior)
    for i in range(10):
        progress = 100.0 + 0.3 * i
        output = tick(behavior, progress, rows_ahead(progress, 114.0), now=10.0 + 0.05 * i)
        assert output.path is not None
    assert len(calls) == 1
    output = tick(behavior, 103.0, rows_ahead(103.0, 114.0, 120.0), now=10.6)
    assert "OBSTACLE_REPLAN_NEW_ROW" in output.tags and len(calls) == 2
    # The new search scores continuity against the committed path.
    assert calls[-1]["previous_knots"] is not None


def test_a_failed_search_waits_before_it_is_repeated() -> None:
    behavior = ObstacleBehavior(planner(), search_retry_sec=0.5)
    calls = recorded_search(behavior, finds_nothing=True)
    for i in range(12):
        output = tick(behavior, 100.0, rows_ahead(100.0, 114.0), now=10.0 + 0.1 * i)
        assert behavior.state == FALLBACK and output.cap_mps == pytest.approx(1.0)
    assert len(calls) == 2  # at 10.0 and 10.6 s


# Path curve speeds per tick -> zone-free cap per tick; 0.5 s window, ticks 0.1 s apart.
CURVE_MEDIAN_CASES = [
    (0.5, [3.0, 1.0, 3.0, 3.0], [3.0, 2.0, 3.0, 3.0]),   # one low frame does not drop the cap
    (0.5, [3.0, 1.0, 1.0, 1.0], [3.0, 2.0, 1.0, 1.0]),   # a lasting drop follows within the window
    (0.0, [3.0, 1.0, 3.0, 3.0], [3.0, 1.0, 3.0, 3.0]),   # zero window: every frame as computed
]


@pytest.mark.parametrize("window, speeds, caps", CURVE_MEDIAN_CASES)
def test_bypass_curve_speed_is_the_median_over_the_window(window, speeds, caps) -> None:
    behavior = ObstacleBehavior(planner(), curve_speed_window_sec=window)
    given = iter(speeds)
    behavior.path_speed = lambda path: next(given)
    got = [tick(behavior, 100.0 + i, rows_ahead(100.0 + i, 112.0, 118.0), now=10.0 + 0.1 * i).cap_mps
           for i in range(len(speeds))]
    assert got == pytest.approx(caps)


def test_curve_speeds_older_than_the_window_or_before_a_reset_are_dropped() -> None:
    behavior = ObstacleBehavior(planner(), curve_speed_window_sec=0.5)
    assert behavior.filtered_path_speed(1.0, 10.0) == 1.0
    assert behavior.filtered_path_speed(3.0, 10.6) == 3.0
    behavior.filtered_path_speed(1.0, 10.7)
    behavior.reset()
    assert behavior.filtered_path_speed(3.0, 10.8) == 3.0
    assert behavior.filtered_path_speed(1.0, None) == 1.0


def test_disabled_mode_writes_nothing() -> None:
    behavior = ObstacleBehavior(planner())
    output = behavior.update(route_at(100.0), mode_enabled=False, stage_enabled=True,
                             objects=rows_ahead(100.0, 112.0, 118.0), perception_ok=True)
    assert output.writes() == set()


ZONE = SimpleNamespace(entry_s=200.0, exit_s=300.0)
# rear-axle s -> (cap, state): pre-slow reaches 3 m/s at the entry rolling at 0.5 m/s^2
ZONE_CAP_CASES = [
    (100.0, math.sqrt(9.0 + 2.0 * 0.5 * (200.0 - 101.98)), PRE_SLOW),
    (198.02, 3.0, IN_ZONE),
    (250.0, 3.0, IN_ZONE),
    (300.0, 3.0, IN_ZONE),
    (300.1, None, IDLE),
]


@pytest.mark.parametrize("rear_s, cap, state", ZONE_CAP_CASES)
def test_zone_speed_cap_and_pre_slow(rear_s, cap, state) -> None:
    behavior = ObstacleBehavior(planner(), zone=ZONE)
    output = behavior.update(route_at(rear_s), mode_enabled=True, stage_enabled=False,
                             objects=None, perception_ok=False)
    assert output.cap_mps == (pytest.approx(cap) if cap is not None else None)
    assert behavior.state == state


# D13: every row is passed on its open side only; the centre line side may be crossed by
# centerline_departure_m, the curb side never.
@pytest.mark.parametrize("row_center, sign", [(-1.05, 1), (1.05, -1)])
def test_rows_are_passed_only_on_their_open_side(row_center, sign) -> None:
    sequence = planner()
    row = Row(10.0, row_center - 0.45, row_center + 0.45, 9.75, 10.25, 2)
    left_limit = 1.625 + sequence.centerline_departure
    options = sequence.row_offsets(row, (-1.625, left_limit))
    assert options and all(option * sign > 0.0 for option in options)
    assert max(options) <= left_limit - sequence.half_width - 0.05 + 1e-9
    assert min(options) >= -1.625 + sequence.half_width + 0.05 - 1e-9


def row_objects(*rows):
    """rows of (station, (y, ...)) drums of 0.5 m in the route frame."""
    return PerceptionObjectArray(objects=[drum(station, y) for station, ys in rows for y in ys])


WIDE_RIGHT = (-0.5, 0.05)   # row centre right of the route; passing left needs the allowance
NORMAL_LEFT = (0.8, 1.3)
NORMAL_RIGHT = (-0.8, -1.3)
WIDE_LEFT = (0.5, -0.05)    # row centre left; passing right would need the curb side


@pytest.mark.parametrize("rows, row_index", [
    (((12.0, WIDE_RIGHT), (18.0, NORMAL_LEFT)), 0),
    (((12.0, NORMAL_LEFT), (18.0, WIDE_RIGHT)), 1),
])
def test_centre_line_allowance_applies_to_any_row(rows, row_index) -> None:
    lane_only = 1.625 - 0.59 - 0.05
    with_allowance = plan(ObstacleSequencePlanner(
        vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.5,
        route_half_width_m=1.625, centerline_departure_m=0.5,
    ), row_objects(*rows))
    assert with_allowance is not None and not with_allowance.forced
    assert with_allowance.passage_offsets[row_index] > lane_only
    without = plan(ObstacleSequencePlanner(
        vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.5,
        route_half_width_m=1.625, centerline_departure_m=0.0,
    ), row_objects(*rows))
    assert without is None or without.forced


def test_the_curb_side_gets_no_allowance() -> None:
    path = plan(planner(), row_objects((12.0, NORMAL_RIGHT), (18.0, WIDE_LEFT)))
    assert path is None or path.forced


# Right steering is weaker: a right-side passage wants right_passage_margin_m more than the target
# clearance where that costs no clearance elsewhere.
@pytest.mark.parametrize("row_center, margin_applies", [(1.05, True), (-1.05, False)])
def test_only_a_right_side_passage_keeps_the_right_margin(row_center, margin_applies) -> None:
    ys = (row_center - 0.25, row_center + 0.25)
    objects = row_objects((14.0, ys))
    paths = [plan(ObstacleSequencePlanner(
        vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.5, route_half_width_m=2.5,
        right_passage_margin_m=margin,
    ), objects) for margin in (0.0, 0.3)]
    plain, margined = paths
    assert plain is not None and margined is not None
    assert plain.body_clearance_m == pytest.approx(0.5, abs=0.03)
    assert margined.body_clearance_m == pytest.approx(0.5, abs=0.03)
    if margin_applies:
        # Passed on its right: the row edge is at row_center - 0.5 and the body half width 0.59.
        assert margined.passage_offsets[0] <= row_center - 0.5 - 0.59 - 0.8 + 0.03
        assert margined.passage_offsets[0] < plain.passage_offsets[0] - 0.1
    else:
        assert margined.passage_offsets == pytest.approx(plain.passage_offsets)


def test_the_right_margin_never_costs_clearance() -> None:
    # A 3.25 m corridor: the margin cannot be had, so the path keeps the most clearance there is,
    # the same as without the margin.
    rows = row_objects((12.0, NORMAL_LEFT), (18.0, NORMAL_RIGHT))
    plain = plan(planner(), rows)
    margined = plan(planner(right_passage_margin_m=0.3), rows)
    assert margined is not None and not margined.forced
    assert margined.body_clearance_m == pytest.approx(plain.body_clearance_m, abs=0.03)
    assert margined.passage_offsets[0] < 0.0 < margined.passage_offsets[1]


def committed_bypass_behavior(**options):
    """A committed two-row bypass: rows at s 656 and 664, the path back on the route at s 676
    and its return point at s 688 (like 2026-10-17 143332, last knot ~688)."""
    from kaiev26_decision.obstacle_behavior import CommittedBypass

    _, zone = final_zone()
    behavior = ObstacleBehavior(planner(), zone=zone, **options)
    knots = ObstacleSequencePlanner.make_knots(
        [(644.0, 0.0), (656.0, 1.4), (664.0, -1.4), (676.0, 0.0), (688.0, 0.0)])
    behavior.committed = CommittedBypass(tuple(knots), (656.0, 664.0), 688.0, 0.5, 0.4, 0.05, False, 2)
    return behavior


# situation: rear-axle progress, distance required past the last row -> the zone cap still holds.
# The path offset is -1.04 m at s 671, -0.36 m at s 674 and 0 from s 676.
BYPASS_RELEASE_CASES = [
    pytest.param(668.0, 3.0, True, id="inside-the-zone-capped"),
    pytest.param(671.0, 3.0, True, id="past-the-zone-path-still-offset"),
    pytest.param(674.0, 3.0, False, id="back-within-0.5m-released-before-the-return-point"),
    pytest.param(687.0, 3.0, False, id="near-the-return-point-released"),
    pytest.param(674.0, 12.0, True, id="not-far-enough-past-the-last-row"),
]


@pytest.mark.parametrize("progress,past_row,capped", BYPASS_RELEASE_CASES)
def test_zone_cap_ends_when_the_bypass_is_finished(progress, past_row, capped) -> None:
    behavior = committed_bypass_behavior(release_past_row_m=past_row)
    cap, tag = behavior.zone_cap(progress)
    assert (tag == "OBSTACLE_ZONE_CAP") == capped
    # The path stays the bypass until its return point either way.
    assert behavior.bypass_active


def test_zone_cap_ends_midway_between_the_rows_inside_the_zone() -> None:
    from dataclasses import replace

    behavior = committed_bypass_behavior()
    behavior.committed = replace(behavior.committed, release_s=660.0)
    assert behavior.zone_cap(659.9)[1] == "OBSTACLE_ZONE_CAP"
    assert behavior.zone_cap(660.0) == (None, "")


def test_a_flickering_cluster_is_never_a_row_and_is_dropped() -> None:
    # 10/23 Midan: clusters seen in 1-3 frames 35 m ahead were kept to the end as rows.
    world = ObstacleWorld(straight_route(), Box(), confirm_frames=5, unconfirmed_memory_sec=0.8)
    for stamp in (1.0, 1.033):
        world.observe(frame(drum(80.0, 0.0, 9)), Pose2D(0.0, 0.0, 0.0), stamp)
    assert not world.local_objects(Pose2D(0.0, 0.0, 0.0), RouteContext().header).objects
    world.observe(frame(), Pose2D(0.0, 0.0, 0.0), 1.9)
    assert not world.kept


def test_a_confirmed_drum_is_kept_out_of_view_until_passed() -> None:
    world = ObstacleWorld(straight_route(), Box(), confirm_frames=5, unconfirmed_memory_sec=0.8)
    for k in range(5):
        world.observe(frame(drum(60.0, -1.0, 7)), Pose2D(0.0, 0.0, 0.0), 1.0 + 0.033 * k)
    world.observe(frame(), Pose2D(0.0, 0.0, 0.0), 5.0)
    assert len(world.local_objects(Pose2D(0.0, 0.0, 0.0), RouteContext().header).objects) == 1
    world.forget_passed(65.0)
    assert not world.kept


def test_a_box_over_the_road_is_not_an_obstacle() -> None:
    # 10/23 Midan s 64: a sign cluster with its bottom 3.5 m up (top 6.2 m); drums sit on the road.
    world = ObstacleWorld(straight_route(), Box(), max_bottom_height_m=2.0)
    sign, barrel = drum(64.0, 0.0, 1), drum(70.0, 1.0, 2)
    sign.pose.position.z, sign.size.z = 4.87, 2.7
    barrel.pose.position.z, barrel.size.z = 0.45, 0.8
    world.observe(frame(sign, barrel), Pose2D(0.0, 0.0, 0.0), 1.0)
    assert sorted(world.kept) == [2]


def test_a_road_surface_return_is_not_an_obstacle() -> None:
    world = ObstacleWorld(straight_route(), Box(), min_top_height_m=0.2)
    ground = drum(57.0, 1.0, 1)
    ground.pose.position.z, ground.size.z = -0.13, 0.22  # 10/23 Midan: top -0.02 m
    world.observe(frame(ground), Pose2D(0.0, 0.0, 0.0), 1.0)
    assert not world.kept


def test_the_bypass_cap_ends_once_the_body_is_past_the_last_row() -> None:
    # Midan: no zone cap of its own; 3 m/s while passing, the path speed right after the rows.
    zone = SimpleNamespace(entry_s=1.0, exit_s=500.0)
    behavior = ObstacleBehavior(planner(), zone=zone, zone_speed_mps=15.0, bypass_speed_mps=3.0)
    passing = tick(behavior, 100.0, rows_ahead(100.0, 112.0, 118.0))
    assert passing.cap_mps <= 3.0
    last = behavior.committed.row_stations[-1]
    after = tick(behavior, last + behavior.planner.rear + 1.0, PerceptionObjectArray())
    assert after.path is not None and after.cap_mps > 3.0


@pytest.mark.parametrize("gap, forced, expected", [
    (0.1, True, 3.0), (0.1, False, 3.0), (0.375, False, 4.0), (0.6, False, None)])
def test_midan_bypass_speeds(gap, forced, expected) -> None:
    # Defaults (10/23 Midan): forced path 3 m/s, a valid one up to 5 m/s by its clearance.
    behavior = ObstacleBehavior(planner(), zone_speed_mps=15.0, bypass_speed_mps=5.0,
                                fallback_speed_mps=1.6, forced_speed_mps=3.0)
    assert behavior.clearance_speed(gap, forced) == (None if expected is None else pytest.approx(expected))


def test_a_committed_forced_path_is_searched_again_and_replaced_by_a_valid_one() -> None:
    # 10/23 Midan: a forced path committed for a short fragment was kept past both rows.
    from dataclasses import replace
    behavior = ObstacleBehavior(planner(), forced_replan_sec=0.5)
    tick(behavior, 100.0, rows_ahead(100.0, 112.0, 118.0), now=10.0)
    behavior.committed = replace(behavior.committed, forced=True, gap_m=-0.1, min_gap_m=-math.inf)
    behavior.forced_searched_sec = 10.0
    early = tick(behavior, 100.2, rows_ahead(100.2, 112.0, 118.0), now=10.2)
    assert "OBSTACLE_REPLAN_FORCED_RETRY" not in early.tags and behavior.committed.forced
    retry = tick(behavior, 100.6, rows_ahead(100.6, 112.0, 118.0), now=10.6)
    assert "OBSTACLE_REPLAN_FORCED_RETRY" in retry.tags and not behavior.committed.forced
