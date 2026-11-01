import math
from pathlib import Path

import pytest
import yaml


WAYPOINTS = Path(__file__).resolve().parents[1] / 'waypoints'
RANGES = {
    'quali': [(10, 80), (154, 215), (340, 394), (476, 551)],
    'final': [(20, 205), (351, 370), (476, 525), (551, 572), (600, 773)],
}


def document(course):
    return yaml.safe_load(
        (WAYPOINTS / f'kcity_{course}_route.yaml').read_text(encoding='utf-8')
    )


@pytest.mark.parametrize('course', ['quali', 'final'])
def test_straightened_cores_are_collinear_and_forward_ordered(course):
    route = document(course)
    for section in route['straightening']['ranges']:
        start = section['straight_start_waypoint']
        end = section['straight_end_waypoint']
        assert section['start_waypoint'] < start < end <= section['end_waypoint']
        if end == section['end_waypoint']:
            assert end == len(route['waypoints']) - 1
            assert section['end_transition_m'] == 0.0
        points = route['waypoints'][start:end + 1]
        dx = points[-1]['x'] - points[0]['x']
        dy = points[-1]['y'] - points[0]['y']
        length = math.hypot(dx, dy)
        previous_progress = -math.inf
        for point in points:
            px = point['x'] - points[0]['x']
            py = point['y'] - points[0]['y']
            assert abs(dx * py - dy * px) / length < 1.0e-8
            progress = (dx * px + dy * py) / length
            assert progress > previous_progress
            previous_progress = progress


@pytest.mark.parametrize('course', ['quali', 'final'])
def test_geodetic_waypoints_match_localization_planar_enu(course):
    route = document(course)
    origin = route['map_origin']
    radius_m = 6378137.0
    east_scale = radius_m * math.cos(math.radians(origin['latitude_deg']))
    for point in route['waypoints']:
        east = math.radians(point['longitude_deg'] - origin['longitude_deg']) * east_scale
        north = math.radians(point['latitude_deg'] - origin['latitude_deg']) * radius_m
        assert east == pytest.approx(point['x'], abs=1.0e-6, rel=0.0)
        assert north == pytest.approx(point['y'], abs=1.0e-6, rel=0.0)


def test_qualifying_connection_is_explicitly_excluded():
    assert document('quali')['straightening']['excluded_ranges'] == [
        {'start_waypoint': 241, 'end_waypoint': 303}
    ]


def test_final_finish_is_on_the_incoming_straight_without_an_end_join():
    route = document('final')
    alignment = route['straightening']['finish_alignment']
    assert alignment == {
        'method': 'projection_onto_terminal_straight',
        'start_waypoint': 699,
        'end_waypoint': 728,
        'reference_start_waypoint': 609,
        'reference_end_waypoint': 699,
    }
    points = route['waypoints']
    first, reference = points[609], points[699]
    dx, dy = reference['x'] - first['x'], reference['y'] - first['y']
    length = math.hypot(dx, dy)
    for point in points[609:]:
        px, py = point['x'] - first['x'], point['y'] - first['y']
        assert abs(dx * py - dy * px) / length < 1.0e-8
    assert points[728]['x'] == pytest.approx(-71.28397602862954, abs=1.0e-9, rel=0.0)
    assert points[728]['y'] == pytest.approx(-128.29369640508042, abs=1.0e-9, rel=0.0)
    # The route goes on 45 m past the old finish on the same straight (15 m 2026-10-24, 30 m 2026-10-25).
    assert route['straightening']['finish_extension'] == {
        'start_waypoint': 729, 'end_waypoint': 773, 'length_m': 45.0,
    }
    assert len(points) == 774
    for first_point, second_point in zip(points[728:], points[729:]):
        assert math.hypot(
            second_point['x'] - first_point['x'], second_point['y'] - first_point['y'],
        ) == pytest.approx(1.0, abs=1.0e-9)


def test_qualifying_finish_alignment_has_no_abrupt_waypoint_turn():
    route = document('quali')
    assert route['straightening']['finish_alignment'] == {
        'method': 'shared_terminal_straight_with_quintic_join',
        'start_waypoint': 492,
        'straight_start_waypoint': 502,
        'end_waypoint': 551,
        'transition_m': 9.999490058,
        'shared_finish_route': 'kcity_final_route.yaml',
        'reference_start_waypoint': 609,
        'reference_end_waypoint': 699,
    }
    points = route['waypoints'][491:]
    headings = [
        math.atan2(b['y'] - a['y'], b['x'] - a['x'])
        for a, b in zip(points, points[1:])
    ]
    for first, second in zip(headings, headings[1:]):
        turn = math.atan2(math.sin(second - first), math.cos(second - first))
        assert abs(turn) < math.radians(1.0)


def test_qualifying_last_fifty_metres_share_the_final_straight():
    qualifying = document('quali')['waypoints']
    final = document('final')['waypoints']
    first, reference = final[609], final[699]
    dx, dy = reference['x'] - first['x'], reference['y'] - first['y']
    length = math.hypot(dx, dy)
    previous_progress = -math.inf
    for point in qualifying[502:]:
        px, py = point['x'] - first['x'], point['y'] - first['y']
        assert abs(dx * py - dy * px) / length < 1.0e-8
        progress = (dx * px + dy * py) / length
        assert progress > previous_progress
        previous_progress = progress
    assert qualifying[551]['x'] == final[728]['x']
    assert qualifying[551]['y'] == final[728]['y']
    # 2026-10-25: the qualifying goes on 45 m on the final's own extension points.
    assert document('quali')['straightening']['finish_extension'] == {
        'start_waypoint': 552, 'end_waypoint': 596, 'length_m': 45.0,
    }
    for index in range(552, 597):
        assert (qualifying[index]['x'], qualifying[index]['y']) == pytest.approx(
            (final[index + 177]['x'], final[index + 177]['y']), abs=1.0e-9, rel=0.0)
