from dataclasses import replace
import math

import pytest

from geometry_msgs.msg import Point
from kaiev26_msgs.msg import PerceptionObject, PerceptionObjectArray

from kaiev26_decision.obstacle_sequence import ObstacleSequencePlanner, PathKnot


def point(x, y):
    return Point(x=float(x), y=float(y))


def observations(first_x=8.0, gap=4.0, radius=0.225, side=-1, lane_half_width=2.5):
    """Two alternating drum rows and lane lines (s, d) from the car to 20 m."""
    objects = PerceptionObjectArray()
    for x, y in (
        (first_x, side * 0.8),
        (first_x, side * 1.3),
        (first_x + gap, -side * 0.8),
        (first_x + gap, -side * 1.3),
    ):
        obj = PerceptionObject()
        obj.class_id = PerceptionObject.CLASS_OBSTACLE
        obj.pose.position = point(x, y)
        obj.size.x = obj.size.y = 2 * radius
        objects.objects.append(obj)
    lanes = [[(0.0, y), (20.0, y)] for y in (-lane_half_width, lane_half_width)]
    return objects, lanes


def plan(planner, route, objects, lane_lines=None, *, steering_rad=None, **observe):
    """observe and search in one call."""
    scene = planner.observe(route, objects, lane_lines=lane_lines, **observe)
    return None if scene is None else planner.search(scene, steering_rad=steering_rad)


@pytest.mark.parametrize('side', [-1, 1])
def test_early_approach_adds_aligned_placements_without_removing_the_normal_ones(side):
    from kaiev26_decision.obstacle_sequence import Row
    route = [point(x, 0) for x in range(36)]
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98,
                                      clearance_m=0.5)
    low, high = sorted((side*.2, side*1.6))
    rows = (Row(22, low, high, 21.8, 22.2, 3),)
    scene = planner.observe(route, None, track=False)
    scene = replace(scene, rows=rows, limits_by_row=((-2.5, 2.5),))
    placements = planner.placements(rows)
    early = [placement for placement in placements if placement.early]
    assert early
    offset = -side*0.9
    for placement in early:
        knots = planner.knots_for(scene, (offset,), placement, None)
        assert knots is not None
        aligned = [k for k in knots[1:] if k.d == offset and k.slope == 0.0 and k.second_derivative == 0.0]
        assert aligned and aligned[0].s < rows[0].front
        assert planner.offset(10.0, knots)*side < -0.1
        assert abs(knots[-1].d) < 1e-9


def test_first_row_only_can_transition_to_two_rows_after_early_preparation():
    route = [point(x, 0) for x in range(36)]
    objects, lanes = observations(first_x=20.0, gap=6.0, side=1)
    lanes = [[line[0], (35.0, line[-1][1])] for line in lanes]
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98,
                                      clearance_m=0.5)
    first_only = PerceptionObjectArray(objects=objects.objects[:2])
    first = plan(planner, route, first_only, lanes, route_progress_s=100.0, observation_time_sec=10.0)
    both = plan(planner, route, objects, lanes, route_progress_s=100.0, observation_time_sec=10.1)
    assert first is not None and first.rows == 1
    assert both is not None and both.rows == 2
    assert first.passage_offsets[0] < 0 and both.passage_offsets[0] < 0


@pytest.mark.parametrize('side', [-1, 1])
def test_near_rows_get_no_early_approach(side):
    from kaiev26_decision.obstacle_sequence import Row
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98,
                                      clearance_m=0.5)
    low, high = sorted((side*.2, side*1.6))
    rows = (Row(8, low, high, 7.8, 8.2, 3),)
    assert not any(placement.early for placement in planner.placements(rows))


def measured(gap, corridor=0.0, right_gap=math.inf):
    from kaiev26_decision.obstacle_sequence import Measured
    return Measured(None, 0.0, 0.0, gap, corridor, right_gap)


# (cost, gap, corridor, right gap) candidates -> order picked by the one ranking rule. Target
# clearance 0.5, right margin 0.3 (right gap capped at 0.8), gap tolerance 0.03.
RANKING_CASES = [
    # The largest gap wins over smoothness.
    ([(0.0, 0.20, 0.0, 0.8), (5.0, 0.40, 0.0, 0.8)], [1, 0]),
    # Within the tolerance the smoother one wins, also at the target.
    ([(5.0, 0.40, 0.0, 0.8), (0.0, 0.38, 0.0, 0.8)], [1, 0]),
    ([(5.0, 0.50, 0.0, 0.8), (0.0, 0.48, 0.0, 0.8)], [1, 0]),
    ([(5.0, 0.50, 0.0, 0.8), (0.0, 0.46, 0.0, 0.8)], [0, 1]),
    # Inside the lane before any outside candidate, whatever its gap.
    ([(0.0, 0.50, 0.10, 0.8), (5.0, 0.05, 0.0, 0.8)], [1, 0]),
    # Outside only: least excess first.
    ([(0.0, 0.50, 0.30, 0.8), (5.0, 0.10, 0.05, 0.8)], [1, 0]),
    # Equal gaps: the larger right-passage gap (within the tolerance), then smoothness.
    ([(0.0, 0.50, 0.0, 0.5), (5.0, 0.50, 0.0, 0.6), (1.0, 0.50, 0.0, 0.58)], [2, 1, 0]),
    ([(0.0, 0.50, 0.0, 0.5), (5.0, 0.50, 0.0, 0.8), (1.0, 0.50, 0.0, 0.70)], [1, 2, 0]),
    # The right gap never outranks the gap itself.
    ([(0.0, 0.30, 0.0, 0.8), (0.0, 0.45, 0.0, 0.45)], [1, 0]),
]


@pytest.mark.parametrize('candidates, order', RANKING_CASES)
def test_one_ranking_rule_orders_candidates(candidates, order):
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98,
                                      clearance_m=0.5, right_passage_margin_m=0.3)
    entries = [(cost, i, measured(gap, corridor, right)) for i, (cost, gap, corridor, right)
               in enumerate(candidates)]
    assert [entry[1] for entry in planner.ranked(entries)] == order


def test_search_returns_the_best_ranked_candidate_it_measured(monkeypatch):
    route = [point(x, 0) for x in range(31)]
    objects, lanes = observations(first_x=10, gap=6)
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.5)
    scene = planner.observe(route, objects, lane_lines=lanes, observation_time_sec=10.0)
    seen = []
    ranked = planner.ranked

    def record(candidates):
        seen.append(list(candidates))
        return ranked(candidates)

    monkeypatch.setattr(planner, 'ranked', record)
    path = planner.search(scene)
    inside = [c for c in seen[-1] if c[2].corridor <= 0.0]
    assert path is not None and not path.forced
    assert planner.last_search_candidates >= len(seen[-1]) > 20
    assert path.body_clearance_m >= max(c[2].gap for c in inside) - 0.03


def test_measured_gap_is_capped_at_the_target_and_the_lane_edge_s_is_unchecked():
    route = [point(x, 0) for x in range(31)]
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98,
                                      clearance_m=0.5, route_half_width_m=2.0)
    scene = planner.observe(route, PerceptionObjectArray(), observation_time_sec=10.0)
    straight = planner.measure(scene, [PathKnot(0.0, 0.0, 0.0), PathKnot(30.0, 0.0, 0.0)])
    assert straight.gap == 0.5 and straight.corridor == 0.0
    objects, lanes = observations(first_x=10, gap=6)
    scene = planner.observe(route, objects, lane_lines=lanes, observation_time_sec=10.1)
    forced = planner.lane_edge_path(scene, None)
    assert forced is not None and forced.forced
    assert math.isnan(forced.body_clearance_m) and forced.clearance_m == -math.inf


@pytest.mark.parametrize("side", [-1, 1])
@pytest.mark.parametrize("first_x,gap,radius", [(8.0, 4.0, 0.225), (9.0, 5.82, 0.25)])
def test_sequence_uses_observed_order_and_geometry(side, first_x, gap, radius):
    route = [point(x, 0) for x in range(21)]
    objects, lanes = observations(first_x, gap, radius, side)
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.3)

    path = plan(planner, route, objects, lanes)

    assert path is not None
    assert path.forced is False
    assert path.rows == 2
    assert path.curvature <= planner.max_curvature
    first = min(path.points, key=lambda p: abs(p.x - first_x))
    second = min(path.points, key=lambda p: abs(p.x - first_x - gap))
    assert first.y * side < 0
    assert second.y * side > 0
    assert abs(path.points[-1].y) < 1.0e-6
    assert path.row_stations == pytest.approx((first_x, first_x + gap))
    assert path.passage_offsets[0] * side < 0
    assert path.passage_offsets[1] * side > 0


def test_sequence_clusters_shuffled_three_barrel_rows_and_rejects_unknown_third_row():
    route = [point(x, 0) for x in range(31)]
    lanes = [[(0.0, y), (30.0, y)] for y in (-2.5, 2.5)]

    objects = PerceptionObjectArray()
    detections = [
        (15.92, 1.65), (9.90, -1.15), (23.0, 0.0),
        (15.72, 0.75), (10.12, -1.60), (15.82, 1.20), (10.0, -0.70),
    ]
    for x, y in detections:
        obj = PerceptionObject()
        obj.class_id = PerceptionObject.CLASS_OBSTACLE
        obj.pose.position = point(x, y)
        obj.size.x = obj.size.y = 0.45
        objects.objects.append(obj)

    planner = ObstacleSequencePlanner(
        vehicle_width_m=1.18,
        front_overhang_m=1.98,
        clearance_m=0.3,
    )
    stations = planner.route_samples(route)
    visible = planner.visible_obstacles(route, stations, objects, lanes)
    rows = planner.select_rows(planner.rows(visible))
    assert len(rows) == 2
    assert [row.members for row in rows] == [3, 3]
    assert rows[1].s - rows[0].s == pytest.approx(5.813, abs=0.03)
    assert rows[0].low < -1.8 and rows[0].high < 0.0
    assert rows[1].low > 0.0 and rows[1].high > 1.8
    path_with_third = plan(planner, route, objects, lanes)
    assert path_with_third is not None
    assert path_with_third.forced is False
    # A later return can now clear the third object without selecting it as a row.
    from kaiev26_decision.obstacle_geometry import polygon_clearance, rectangle_corners
    for i, p in enumerate(path_with_third.points):
        a = path_with_third.points[max(0, i-1)]
        b = path_with_third.points[min(len(path_with_third.points)-1, i+1)]
        body = rectangle_corners(p.x, p.y, math.atan2(b.y-a.y, b.x-a.x),
                                 -planner.rear, planner.front, planner.half_width)
        assert all(polygon_clearance(body, obj.corners) > 0.0 for obj in visible)

    objects.objects = [obj for obj in objects.objects if obj.pose.position.x < 20.0]
    path = plan(planner, route, objects, lanes)
    assert path is not None
    assert path.rows == 2
    assert path.passage_offsets[0] > 0.0
    assert path.passage_offsets[1] < 0.0
    assert abs(path.points[-1].y) < 1.0e-6


def test_narrow_lane_keeps_the_largest_clearance_it_has_instead_of_forcing():
    route = [point(x, 0) for x in range(21)]
    objects, lanes = observations(lane_half_width=1.0)
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.3)

    path = plan(planner, route, objects, lanes)
    assert path is not None
    assert path.forced is False and path.corridor_excess_m == 0.0
    assert 0.05 < path.body_clearance_m < 0.3
    # Committed, it must keep its own gap less the recheck tolerance.
    assert path.clearance_m == pytest.approx(path.body_clearance_m - planner.recheck_tolerance)
    assert path.passage_offsets[0] * path.passage_offsets[1] < 0.0


def test_sequence_tracks_rows_across_jitter_and_brief_detection_dropout():
    route = [point(x, 0) for x in range(36)]
    planner = ObstacleSequencePlanner(
        vehicle_width_m=1.18,
        front_overhang_m=1.98,
        clearance_m=0.5,
        row_memory_sec=0.8,
    )

    first_objects, lanes = observations(first_x=9.0, gap=5.82, radius=0.25)
    first = plan(
        planner,
        route,
        first_objects,
        lanes,
        route_progress_s=100.0,
        observation_time_sec=10.0,
    )
    assert first is not None

    jittered_objects, jittered_lanes = observations(
        first_x=8.25,
        gap=5.95,
        radius=0.30,
    )
    jittered = plan(
        planner,
        route,
        jittered_objects,
        jittered_lanes,
        route_progress_s=101.0,
        observation_time_sec=10.1,
    )
    assert jittered is not None
    assert jittered.rows == 2
    assert jittered.row_stations[0] == pytest.approx(8.125, abs=0.05)

    remembered = plan(
        planner,
        route,
        None,
        None,
        route_progress_s=102.0,
        observation_time_sec=10.6,
    )
    assert remembered is not None
    assert remembered.rows == 2
    assert remembered.passage_offsets[0] * remembered.passage_offsets[1] < 0.0

    expired = plan(
        planner,
        route,
        None,
        None,
        route_progress_s=103.0,
        observation_time_sec=11.1,
    )
    assert expired is None


@pytest.mark.parametrize("side", [-1, 1])
def test_sequence_updates_first_row_path_when_second_row_becomes_visible(side):
    route = [point(x, 0) for x in range(36)]
    planner = ObstacleSequencePlanner(
        vehicle_width_m=1.18,
        front_overhang_m=1.98,
        clearance_m=0.5,
    )
    first_objects, lanes = observations(
        first_x=9.0,
        gap=5.82,
        radius=0.25,
        side=side,
    )
    first_objects.objects = first_objects.objects[:2]

    first_path = plan(
        planner,
        route,
        first_objects,
        lanes,
        route_progress_s=100.0,
        observation_time_sec=10.0,
    )
    assert first_path is not None
    assert first_path.rows == 1
    assert first_path.passage_offsets[0] * side < 0.0

    both_objects, both_lanes = observations(
        first_x=8.5,
        gap=5.82,
        radius=0.25,
        side=side,
    )
    s_path = plan(
        planner,
        route,
        both_objects,
        both_lanes,
        route_progress_s=100.5,
        observation_time_sec=10.1,
    )
    assert s_path is not None
    assert s_path.rows == 2
    assert s_path.passage_offsets[0] * side < 0.0
    assert s_path.passage_offsets[1] * side > 0.0


def test_sequence_ignores_objects_outside_observed_lane():
    route = [point(x, 0) for x in range(21)]
    objects, lanes = observations()
    extra = PerceptionObject()
    extra.class_id = PerceptionObject.CLASS_OBSTACLE
    extra.pose.position = point(7, 7)
    extra.size.x = extra.size.y = 0.5
    objects.objects.append(extra)
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.3)

    assert plan(planner, route, objects, lanes) is not None


def test_oriented_row_projects_box_corners_without_inflating_longitudinal_extent():
    route = [point(x, 0) for x in range(31)]
    obj = PerceptionObject()
    obj.class_id = PerceptionObject.CLASS_OBSTACLE
    obj.pose.position = point(10, 0.8)
    obj.pose.orientation.z = math.sin(math.pi / 4)
    obj.pose.orientation.w = math.cos(math.pi / 4)
    obj.size.x, obj.size.y = 1.6, 0.3
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.5)
    selected = planner.visible_obstacles(route, planner.route_samples(route), PerceptionObjectArray(objects=[obj]), [])
    row = planner.rows(selected)[0]
    assert (row.front, row.rear, row.low, row.high) == pytest.approx((9.85, 10.15, 0, 1.6))


def test_long_roadside_cluster_does_not_enter_corridor_by_circle_radius():
    route = [point(x, 0) for x in range(31)]
    objects, lanes = observations()
    obj = PerceptionObject()
    obj.class_id = PerceptionObject.CLASS_OBSTACLE
    obj.pose.position = point(10, 4)
    obj.size.x, obj.size.y = 9.0, 0.3
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.5)
    stations = planner.route_samples(route)
    assert not planner.visible_obstacles(route, stations, PerceptionObjectArray(objects=[obj]), lanes)


@pytest.mark.parametrize('side', [-1, 1])
def test_lidar_only_plan_uses_explicit_configured_corridor(side):
    route = [point(x, 0) for x in range(36)]
    objects, _ = observations(first_x=9, gap=6, side=side)
    planner = ObstacleSequencePlanner(
        vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.5,
        route_half_width_m=1.625, centerline_departure_m=0.5,
    )
    path = plan(planner, route, objects, None)
    assert path is not None and path.rows == 2
    assert path.boundary_source == 'configured_route_corridor'
    assert path.passage_offsets[0] * side < 0
    assert path.passage_offsets[1] * side > 0
    assert abs(path.passage_offsets[0]) <= 1.625 - planner.half_width


def test_lidar_only_fallback_can_be_disabled():
    route = [point(x, 0) for x in range(36)]
    objects, _ = observations()
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.5)
    assert plan(planner, route, objects, None) is None


def test_single_lane_boundary_does_not_disable_configured_corridor():
    route = [point(x, 0) for x in range(36)]
    objects, lanes = observations()
    lanes = lanes[:1]
    planner = ObstacleSequencePlanner(
        vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.5,
        route_half_width_m=1.625,
    )
    assert plan(planner, route, objects, lanes).boundary_source == 'configured_route_corridor'


def test_same_side_clutter_is_not_selected_as_an_alternating_pair():
    from kaiev26_decision.obstacle_sequence import Row
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.5)
    first = Row(10, 0.1, 1.6, 9.8, 10.2, 1)
    same_side = Row(16, 0.2, 1.7, 15.8, 16.2, 1)
    assert planner.select_rows([first, same_side]) == [first]


def test_cached_projection_and_bisected_pose_preserve_polyline_geometry():
    route = [point(0, 0), point(10, 0), point(10, 10)]
    stations = ObstacleSequencePlanner.route_samples(route)
    segments = ObstacleSequencePlanner.projection_segments(route, stations)
    for xy, expected in [((3, 2), (3, 2)), ((12, 7), (17, -2))]:
        assert ObstacleSequencePlanner.project(route, stations, *xy) == pytest.approx(expected)
        assert ObstacleSequencePlanner.project(route, stations, *xy, segments=segments) == pytest.approx(expected)
    assert ObstacleSequencePlanner.route_pose(route, stations, 10) == pytest.approx((10, 0, 0))
    assert ObstacleSequencePlanner.route_pose(route, stations, 11) == pytest.approx((10, 1, math.pi/2))


def test_obstacle_clearance_is_not_also_required_at_the_road_edge():
    from kaiev26_decision.obstacle_sequence import Row
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.5)
    row = Row(10, 0.2, 1.6, 9.8, 10.2, 1)
    offsets = planner.row_offsets(row, (-1.625, 1.625))
    # Passed on the right, at most at the lane edge less the body and 5 cm, and at least the
    # target clearance from the row where the lane allows it.
    assert offsets and all(-0.985 - 1e-9 <= d < 0.2 - 0.59 for d in offsets)
    assert min(offsets) == pytest.approx(-0.985)
    assert any(d <= 0.2 - 0.59 - 0.5 + 1e-9 for d in offsets)


def test_row_tracking_does_not_merge_opposite_sides_at_same_station():
    from kaiev26_decision.obstacle_sequence import Row
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.5)
    left = Row(10, 0.3, 1.5, 9.8, 10.2, 1)
    right = Row(10, -1.5, -0.3, 9.8, 10.2, 1)
    planner.tracked_rows([left], 100, 1.0, 30)
    rows = planner.tracked_rows([right], 100, 1.1, 30)
    assert len(rows) == 2
    assert sorted((r.low, r.high) for r in rows) == [(-1.5, -0.3), (0.3, 1.5)]


def test_selected_pair_does_not_jump_to_a_new_better_scoring_pair():
    from kaiev26_decision.obstacle_sequence import Row
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.5)
    first = Row(8, 0.3, 1.5, 7.8, 8.2, 1)
    second = Row(13, -1.5, -0.3, 12.8, 13.2, 1)
    new = Row(19, 0.3, 1.5, 18.8, 19.2, 5)
    planner.first_row_station, planner.second_row_station = 108, 113
    planner.selected_rows = tuple(planner.global_row(r, 100) for r in (first, second))
    assert planner.select_rows([first, second, new], route_progress_s=100) == [first, second]


def test_outside_and_longitudinal_objects_remain_collision_objects_not_rows():
    route = [point(x, 0) for x in range(31)]
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.5)
    objects = PerceptionObjectArray()
    for x, y, length, width in [(10, 2.0, 0.4, 0.4), (10, 1.0, 8.0, 0.4)]:
        obj = PerceptionObject()
        obj.class_id = PerceptionObject.CLASS_OBSTACLE
        obj.pose.position = point(x, y)
        obj.size.x, obj.size.y = length, width
        objects.objects.append(obj)
    visible = planner.visible_obstacles(route, planner.route_samples(route), objects, [])
    assert len(visible) == 2
    lines = [[(0, y), (30, y)] for y in (-1.625, 1.625)]
    assert planner.row_obstacles(visible, lines) == []


def test_passed_objects_do_not_project_back_to_the_route_start():
    route = [point(x, 0) for x in range(31)]
    objects, _ = observations(first_x=-5, gap=1)
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.5)
    assert planner.visible_obstacles(route, planner.route_samples(route), objects, []) == []


def test_missing_row_footprint_is_kept_with_unrelated_live_detections():
    from kaiev26_decision.obstacle_sequence import Row
    route = [point(x, 0) for x in range(31)]
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.5)
    missing = Row(8, 0.2, 1.5, 7.8, 8.2, 1)
    unrelated = Row(15, -1.5, -0.2, 14.8, 15.2, 1)
    boxes = planner.missing_row_obstacles(route, planner.route_samples(route),
                                         [missing, unrelated], [unrelated])
    assert len(boxes) == 1
    assert boxes[0].s == pytest.approx(8.0)
    assert boxes[0].footprint_sd == ((7.8, 0.2), (8.2, 0.2), (8.2, 1.5), (7.8, 1.5))


@pytest.mark.parametrize('heading', [-0.12, 0.12])
def test_path_starts_at_current_vehicle_pose_and_heading(heading):
    route = [point(x*math.cos(heading), x*math.sin(heading)) for x in range(31)]
    objects, _ = observations(first_x=10, gap=6)
    for obj in objects.objects:
        x, y = obj.pose.position.x, obj.pose.position.y
        obj.pose.position = point(x*math.cos(heading)-y*math.sin(heading),
                                  x*math.sin(heading)+y*math.cos(heading))
        obj.pose.orientation.z, obj.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98,
                                      clearance_m=0.3, route_half_width_m=2.5)
    path = plan(planner, route, objects, None, steering_rad=0.02)
    assert path is not None and not path.forced
    assert math.hypot(path.points[0].x, path.points[0].y) < 1.0e-6
    assert abs(math.atan2(path.points[1].y, path.points[1].x)) < 0.015


def test_nonfinite_steering_falls_back_to_geometry_without_stopping():
    route = [point(x, 0) for x in range(31)]
    objects, lanes = observations(first_x=10, gap=6)
    paths = []
    for steering in (None, float('nan')):
        planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.3)
        paths.append(plan(planner, route, objects, lanes, steering_rad=steering))
    assert paths[0] == paths[1]


def test_left_right_mirrored_inputs_produce_mirrored_paths():
    route = [point(x, 0) for x in range(31)]
    paths = []
    for side in (-1, 1):
        objects, lanes = observations(first_x=10, gap=6, side=side)
        planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.3)
        paths.append(plan(planner, route, objects, lanes, steering_rad=side*0.03))
    assert paths[0] is not None and paths[1] is not None
    assert [(p.x, p.y) for p in paths[0].points] == pytest.approx(
        [(p.x, -p.y) for p in paths[1].points])


def test_candidates_past_the_steering_geometry_are_not_measured():
    route = [point(x, 0) for x in range(21)]
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.3,
                                      route_half_width_m=5.0)
    scene = planner.observe(route, PerceptionObjectArray(), observation_time_sec=10.0)
    knots = planner.make_knots([(0, 0), (1, 2), (2, -2), (20, 0)])
    assert planner.measure(scene, knots) is None
    assert planner.follow(scene, knots, min_gap=-math.inf, corridor_allowance=math.inf) is None


def test_reset_clears_the_row_tracks_and_the_selection():
    route = [point(x, 0) for x in range(31)]
    objects, lanes = observations(first_x=10, gap=6)
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.3)
    assert not plan(planner, route, objects, lanes, observation_time_sec=10).forced
    assert planner.row_tracks and planner.selected_rows and planner.last_search_candidates
    planner.reset()
    assert not planner.row_tracks and not planner.selected_rows
    assert planner.last_search_candidates == 0


def test_committed_path_is_followed_until_it_loses_its_gap():
    route = [point(x, 0) for x in range(31)]
    objects, lanes = observations(first_x=10, gap=6)
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.5)
    path = plan(planner, route, objects, lanes, observation_time_sec=10.0)
    knots = tuple(PathKnot(k.s + 100.0, k.d, k.slope, k.second_derivative) for k in path.knots)
    # The same objects seen from 1 m further on: the committed path still keeps its gap.
    moved = [point(x - 1.0, 0) for x in range(31)]
    shifted = PerceptionObjectArray(objects=[
        PerceptionObject(class_id=obj.class_id, size=obj.size) for obj in objects.objects])
    for new, old in zip(shifted.objects, objects.objects):
        new.pose.position = point(old.pose.position.x - 1.0, old.pose.position.y)
    scene = planner.observe(moved, shifted, lane_lines=lanes, route_progress_s=101.0, observation_time_sec=10.1)
    kept = planner.follow(scene, knots, min_gap=path.clearance_m, corridor_allowance=0.05)
    assert kept is not None and kept.body_clearance_m >= path.clearance_m
    assert kept.points[0].y == pytest.approx(planner.offset(1.0, path.knots), abs=0.02)
    # A drum moved into the path: the gap is lost and the committed path is dropped.
    shifted.objects[0].pose.position.y = planner.offset(9.0, path.knots)
    scene = planner.observe(moved, shifted, lane_lines=lanes, route_progress_s=101.0, observation_time_sec=10.2)
    assert planner.follow(scene, knots, min_gap=path.clearance_m, corridor_allowance=0.05) is None


def test_vectorized_projection_matches_scalar_projection():
    import numpy as np
    route = [point(0, 0), point(10, 0), point(10, 10), point(15, 15)]
    stations = ObstacleSequencePlanner.route_samples(route)
    segments = ObstacleSequencePlanner.projection_segments(route, stations)
    samples = np.random.default_rng(42).uniform(-5, 20, (100, 2))
    s, d = ObstacleSequencePlanner.project_many(samples, np.asarray(segments))
    expected = [ObstacleSequencePlanner.project(route, stations, *p) for p in samples]
    np.testing.assert_allclose(np.column_stack((s, d)), expected)


def test_projection_merges_collinear_segments_but_keeps_turns_and_reversals():
    route = [point(x, 0) for x in range(21)]+[point(20, 1), point(20, 2), point(20, 1)]
    stations = ObstacleSequencePlanner.route_samples(route)
    segments = ObstacleSequencePlanner.projection_segments(route, stations)
    assert len(segments) == 3
    assert [segment[-1] for segment in segments] == pytest.approx([0, 20, 22])
    assert ObstacleSequencePlanner.project(route, stations, 10, 1) == pytest.approx((10, 1))


def test_batched_polygon_gaps_match_scalar_geometry():
    import numpy as np
    from kaiev26_decision.obstacle_geometry import polygon_clearance, rectangle_corners
    obstacle = rectangle_corners(0, 0, 0.6, -0.25, 0.25, 0.8)
    rng = np.random.default_rng(9)
    bodies = [rectangle_corners(*rng.uniform(-4, 4, 3), -0.62, 1.98, 0.59) for _ in range(200)]
    # One batch holds overlapping and separate bodies; each keeps its own exact gap.
    actual = ObstacleSequencePlanner.polygon_gaps(np.asarray(bodies), np.asarray(obstacle))
    expected = [polygon_clearance(body, obstacle) for body in bodies]
    assert any(e <= 0.0 for e in expected) and any(e > 0.0 for e in expected)
    for a, e in zip(actual, expected):
        if e <= 0.0:
            assert a <= 0.0
        else:
            assert a == pytest.approx(e, abs=1e-10)
    # Overlap is negative by the penetration depth: deeper overlaps are more negative.
    deep = ObstacleSequencePlanner.polygon_gaps(
        np.asarray([rectangle_corners(x, 0, 0, -0.62, 1.98, 0.59) for x in (-2.2, -1.9, -1.0)]),
        np.asarray(obstacle))
    assert deep[0] > deep[1] > deep[2] and deep[1] < 0.0


def test_row_keeps_measured_lane_limits_while_the_camera_misses_the_near_lane():
    route = [point(x, 0) for x in range(36)]
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.5)
    objects, lanes = observations(first_x=9.0, gap=6.0)
    assert plan(planner, route, objects, lanes, route_progress_s=100.0, observation_time_sec=10.0)
    # The camera sees the lane only from 5 m ahead; the first row stays nearer for longer
    # than the row history holds samples.
    for step in range(1, 12):
        first_x = 5.0 - 0.1 * step
        objects, lanes = observations(first_x=first_x, gap=6.0)
        lanes = [[(5.0, line[0][1]), (20.0, line[0][1])] for line in lanes]
        path = plan(planner, route, objects, lanes, route_progress_s=104.0 + 0.1 * step,
                            observation_time_sec=10.0 + 0.05 * step)
        assert path is not None and path.rows == 2


def test_vectorized_lane_limits_match_the_scalar_lookup():
    import random
    rng = random.Random(3)
    for _ in range(500):
        lines = []
        for _ in range(rng.randint(0, 4)):
            stations = sorted(rng.uniform(-3, 30) for _ in range(rng.randint(1, 8)))
            if len(stations) > 2 and rng.random() < 0.3:
                stations[1] = stations[2]
            base = rng.choice([-1.9, 2.1, 0.1, -0.31, 0.31])
            lines.append([(s, base + rng.uniform(-0.4, 0.4)) for s in stations])
        queries = [rng.uniform(-5, 35) for _ in range(20)] + [s for line in lines for s, _ in line]
        right, left = ObstacleSequencePlanner.lane_limits_many(lines, queries)
        for index, station in enumerate(queries):
            expected = ObstacleSequencePlanner.lane_limits(lines, station)
            actual = None if math.isnan(right[index]) else (float(right[index]), float(left[index]))
            assert actual == expected


# Best-effort passage (kty 7fab8bb, 2026-10-17 final): the same cases as on main.
def short_lane_observations(first_x, gap, line_end_x, lane_half_width=2.0):
    """Drum rows on opposite halves of the lane; lane lines end at line_end_x (pv26 range)."""
    objects, _ = observations(first_x=first_x, gap=gap, side=1, lane_half_width=lane_half_width)
    return objects, [[(0.0, y), (float(line_end_x), y)] for y in (-lane_half_width, lane_half_width)]


def test_far_row_without_lane_lines_borrows_the_near_rows_limits():
    # 2026-10-17 final: the second row lay beyond the perceived lane lines and the planner
    # returned nothing until 4.8 m before the first row.
    route = [point(x, 0) for x in range(36)]
    objects, lanes = short_lane_observations(first_x=12, gap=6, line_end_x=15)
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.3)

    path = plan(planner, route, objects, lanes, observation_time_sec=10.0)

    assert path is not None
    assert path.passage_offsets[0] < 0.0 < path.passage_offsets[1]  # S: right of row 1, left of row 2


MAP_LANES = [[(-1.0, -2.0), (40.0, -2.0)], [(-1.0, 2.0), (40.0, 2.0)]]


def test_the_s_keeps_the_most_clearance_there_is_when_the_target_cannot_be_kept():
    # Rows 4.4 m apart on opposite halves of the lane and a target no S can keep: the planner
    # still drives an S inside the lane with the largest gap it has, within the steering limit,
    # instead of nothing or one swerve around both.
    route = [point(x, 0) for x in range(36)]
    objects, _ = observations(first_x=12, gap=4.4, side=1, lane_half_width=2.0)
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=1.5)

    path = plan(planner, route, objects, MAP_LANES, observation_time_sec=10.0)

    assert path is not None and not path.forced and path.corridor_excess_m == 0.0
    assert path.passage_offsets[0] < 0.0 < path.passage_offsets[1]
    assert path.curvature <= planner.max_curvature + 1.0e-9
    assert 0.3 < path.body_clearance_m < 1.5
    # The committed path is one of the candidates of the next search: it is kept.
    scene = planner.observe(route, objects, lane_lines=MAP_LANES, observation_time_sec=10.1)
    again = planner.search(scene, previous_knots=path.knots)
    assert again.passage_offsets == pytest.approx(path.passage_offsets, abs=1.0e-6)


def test_local_route_starts_at_the_car_when_the_points_behind_it_were_dropped():
    # The route manager drops points with local x < -0.2; yawed against the route, the first
    # point left lies ahead of the car's projection.
    import math as m
    yaw = 0.3
    route = [point((x + 1.0) * m.cos(yaw), -(x + 1.0) * m.sin(yaw) + 0.3) for x in range(30)]
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.5)
    points, stations = planner.local_route(route, 100.0)
    s, d = planner.project(points, stations, 0.0, 0.0)
    assert s == pytest.approx(0.0, abs=1e-6)
    # The first point kept is 1 - 0.3 sin(yaw) ahead of the projection.
    assert stations[1] == pytest.approx(1.0 - 0.3 * m.sin(yaw))
    assert len(points) == len(route) + 1


def test_a_row_is_never_crossed_to_its_closed_side():
    from kaiev26_decision.obstacle_sequence import Row
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.3)
    left_row = Row(10.0, 0.4, 1.9, 9.8, 10.2, 3)
    right_row = Row(14.4, -1.7, -0.1, 14.2, 14.6, 3)
    limits = (-2.0, 2.5)
    assert planner.passage_side(left_row, limits) == -1.0
    assert planner.passage_side(right_row, limits) == 1.0


def test_a_row_keeps_its_passage_side_once_it_is_near():
    # 2026-10-17 171607: a row centre read near the route centre flipped the first row's side.
    from kaiev26_decision.obstacle_sequence import Row
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.3,
                                      transition_length_m=12.0)
    limits = (-2.0, 2.5)
    planner.progress_s = 100.0
    far = Row(20.0, 0.05, 1.0, 19.8, 20.2, 3)  # left of the route: passed on its right
    planner.lock_row_sides([far], [limits])
    assert planner.row_sides == []  # still 19.8 m ahead, past the 12 m lock distance
    near = Row(10.0, 0.05, 1.0, 9.8, 10.2, 3)
    planner.lock_row_sides([near], [limits])
    assert planner.passage_side(near, limits) == -1.0
    # The same row read slightly right of the route from 1 m further on keeps its side.
    planner.progress_s = 101.0
    flipped = Row(9.0, -1.0, -0.05, 8.8, 9.2, 3)
    assert planner.passage_side(flipped, limits) == -1.0
    assert planner.forced_passage_offset(flipped, limits) == pytest.approx(-2.0 + 0.59 + 0.05)
    planner.reset()
    assert planner.passage_side(flipped, limits) == 1.0


# situation: the car's offset from the committed path at its station -> where the new path starts.
@pytest.mark.parametrize("current_d,keeps_offset", [(0.62, True), (1.5, False)])
def test_a_replan_starts_on_the_committed_path(current_d, keeps_offset):
    # 171607 at 369.1 s: a replan moved the near path 0.72 m and the steering 9.2 deg in 0.3 s.
    from kaiev26_decision.obstacle_sequence import Scene
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.5)
    followed = ObstacleSequencePlanner.make_knots([(90.0, 0.0), (104.0, 1.2), (118.0, 0.0)])
    planner._followed_knots = tuple(followed)
    scene = Scene([], [], [], "map_lane", [], (), None, current_d, 0.0, 100.0)
    new = planner.continue_start(scene, ObstacleSequencePlanner.make_knots(
        [(0.0, current_d), (10.0, -1.0), (20.0, 0.0)]))
    d0 = ObstacleSequencePlanner.offset(100.0, list(followed))
    slope = (ObstacleSequencePlanner.offset(100.1, list(followed))
             - ObstacleSequencePlanner.offset(99.9, list(followed))) / 0.2
    assert new[0].d == pytest.approx(d0 if keeps_offset else current_d)
    assert new[0].slope == pytest.approx(slope, abs=1e-6)
    assert new[0].s == 0.0
    planner._followed_knots = None
    plain = ObstacleSequencePlanner.make_knots([(0.0, current_d), (10.0, -1.0), (20.0, 0.0)])
    assert planner.continue_start(scene, plain) == plain


def test_steering_gradient_is_taken_over_a_window_and_the_rate_limit_can_be_off():
    # 10/23 Midan: sample-to-sample steering change read 2-3x the smooth gradient; a 1 m window
    # gives the smooth one, and a rate limit <= 0 sets no steering-rate speed limit.
    from kaiev26_decision.obstacle_sequence import ObstacleSequencePlanner as Planner
    common = dict(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.5)
    assert Planner(**common).steering_gradient_window_m == 1.0
    assert math.isinf(Planner(**common, max_steering_rate_deg_s=0.0).max_steering_rate)
