import math
from pathlib import Path

from geometry_msgs.msg import Point
from kaiev26_msgs.msg import RouteContext
import pytest

from kaiev26_decision.curve_speed_application import curve_speed_cap
from kaiev26_decision.curve_speed_policy import (
    load_curve_speed_policy,
    resolve_curve_speed_scale_override,
    validate_curve_speed_scale,
)


PACKAGE = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize(
    ("name", "zone_ids"),
    [
        ("kcity_quali_route.yaml", ("Q1", "Q2", "Q3", "Q4", "Q5", "Q6")),
        ("kcity_final_route.yaml", ("F1", "F2", "F3", "F4")),
    ],
)
def test_course_policy_loads_expected_fixed_zones(name, zone_ids) -> None:
    policy = load_curve_speed_policy(str(PACKAGE / "waypoints" / name))

    assert policy is not None
    assert policy.speed_scale == pytest.approx(0.8)
    assert policy.approach_decel_mps2 == pytest.approx(1.4)
    assert tuple(zone.zone_id for zone in policy.zones) == zone_ids


def test_curve_zone_scales_original_cruise_target() -> None:
    policy = load_curve_speed_policy(
        str(PACKAGE / "waypoints" / "kcity_quali_route.yaml")
    )

    first_zone = policy.zones[0]
    limit = policy.limit_for_progress(
        0.5 * (first_zone.start_s + first_zone.end_s),
        8.0,
    )

    assert limit is not None
    assert limit.zone_id == "Q1"
    assert limit.phase == "zone"
    assert limit.speed_limit_mps == pytest.approx(6.4)


def test_approach_envelope_starts_only_when_it_is_below_cruise() -> None:
    policy = load_curve_speed_policy(
        str(PACKAGE / "waypoints" / "kcity_quali_route.yaml")
    )

    first_zone = policy.zones[0]
    assert policy.limit_for_progress(first_zone.start_s - 10.0, 8.0) is None
    limit = policy.limit_for_progress(first_zone.start_s - 8.0, 8.0)
    assert limit is not None
    assert limit.zone_id == "Q1"
    assert limit.phase == "approach"
    assert 6.4 < limit.speed_limit_mps < 8.0


def test_curve_policy_is_a_cap_with_its_zone_tag() -> None:
    policy = load_curve_speed_policy(
        str(PACKAGE / "waypoints" / "kcity_quali_route.yaml")
    )
    route = RouteContext()
    route.route_projection_valid = True
    first_zone = policy.zones[0]
    route.progress_s = 0.5 * (first_zone.start_s + first_zone.end_s)
    route.distance_to_finish = policy.route_length_m - route.progress_s
    route.local_path_points = [Point(x=0.0), Point(x=10.0)]

    cap, tags = curve_speed_cap(route, policy, 8.0)

    assert cap == pytest.approx(6.4)
    assert "CURVE_SPEED_ZONE:Q1" in tags


@pytest.mark.parametrize("route", [None, RouteContext()])
def test_unlocated_route_gives_zero_cap_without_a_stop(route) -> None:
    policy = load_curve_speed_policy(
        str(PACKAGE / "waypoints" / "kcity_quali_route.yaml")
    )

    cap, tags = curve_speed_cap(route, policy, 8.0)

    assert cap == 0.0
    assert tags == ["CURVE_SPEED_ROUTE_INVALID"]


@pytest.mark.parametrize("value", [0.0, -0.5, 1.01, float("nan"), True, "0.8"])
def test_invalid_explicit_scale_is_rejected(value) -> None:
    with pytest.raises(ValueError):
        validate_curve_speed_scale(value)


def test_launch_sentinel_uses_yaml_scale() -> None:
    assert resolve_curve_speed_scale_override(-1.0) is None
    assert resolve_curve_speed_scale_override(0.6) == pytest.approx(0.6)


# No absolute corner cap (removed 2026-10-21, was 5 m/s): the corner speed is cruise x 0.8.
@pytest.mark.parametrize(("cruise", "expected"), [(10.0, 8.0), (8.0, 6.4), (5.0, 4.0)])
def test_corner_speed_is_the_cruise_times_the_route_scale(cruise, expected) -> None:
    policy = load_curve_speed_policy(str(PACKAGE / "waypoints" / "kcity_final_route.yaml"))
    zone = policy.zones[0]

    limit = policy.limit_for_progress(0.5 * (zone.start_s + zone.end_s), cruise)
    approach = policy.limit_for_progress(zone.start_s - 2.0, cruise)

    assert limit.speed_limit_mps == pytest.approx(expected)
    assert approach.speed_limit_mps == pytest.approx(
        min(cruise, (expected ** 2 + 2.0 * policy.approach_decel_mps2 * 2.0) ** 0.5)
    )




def test_only_the_final_and_midan_policies_have_an_obstacle_zone() -> None:
    from kaiev26_decision.zone_policy import load_zone_policy

    assert load_zone_policy(PACKAGE / "config" / "qualifying_policy.yaml").obstacle_zone is None
    midan = load_zone_policy(PACKAGE / "config" / "midan_test_policy.yaml").obstacle_zone
    assert midan.lane_offsets_m == pytest.approx((-1.96, 2.14))
    # No drum range: whole route, no fixed cap (15 m/s, the launch maximum).
    assert (midan.start_route_s, midan.speed_cap_mps, midan.polygon) == (21.0, pytest.approx(15.0), ())
    zone = load_zone_policy(PACKAGE / "config" / "final_policy.yaml").obstacle_zone
    assert zone.speed_cap_mps == pytest.approx(5.0)
    assert zone.pre_slow_decel_mps2 == pytest.approx(0.5)
SCHOOL_START = 331.5
SCHOOL_END = 404.6
FRONT = 1.98


def school_route(progress: float) -> RouteContext:
    route = RouteContext()
    route.route_projection_valid = True
    route.progress_s = progress
    return route


def school_zones():
    from kaiev26_decision.zone_policy import load_zone_policy

    return load_zone_policy(PACKAGE / "config" / "qualifying_policy.yaml").speed_zones


# situation: rear-axle progress, cruise, route state -> cap and tag. The cap holds from the front
# bumper at 331.5 to the rear axle at 404.6 and only lowers the cruise; it never stops the car.
SCHOOL_ZONE_CASES = [
    pytest.param(SCHOOL_START - FRONT - 30.0, 8.0, "ok", None, None, id="far-before-untouched"),
    pytest.param(SCHOOL_START - FRONT - 5.0, 8.0, "ok", (7.5 ** 2 + 2.0 * 0.5 * 5.0) ** 0.5,
                 "SPEED_ZONE_PRE_SLOW:SCHOOL_ZONE", id="pre-slow-envelope"),
    pytest.param(SCHOOL_START - FRONT, 8.0, "ok", 7.5, "SPEED_ZONE_CAP:SCHOOL_ZONE",
                 id="front-bumper-at-start-capped"),
    pytest.param(351.8, 8.0, "ok", 7.5, "SPEED_ZONE_CAP:SCHOOL_ZONE", id="signal-line-capped"),
    pytest.param(SCHOOL_END, 8.0, "ok", 7.5, "SPEED_ZONE_CAP:SCHOOL_ZONE",
                 id="rear-axle-at-end-capped"),
    pytest.param(SCHOOL_END + 0.1, 8.0, "ok", None, None, id="rear-axle-past-end-released"),
    pytest.param(360.0, 7.0, "ok", None, None, id="qualifying-7mps-untouched"),
    pytest.param(360.0, 8.0, "stale", None, None, id="stale-route-left-to-input-loss"),
    pytest.param(360.0, 8.0, "unlocated", None, None, id="unlocated-route-left-to-input-loss"),
]


@pytest.mark.parametrize("progress,cruise,state,expected,tag", SCHOOL_ZONE_CASES)
def test_school_zone_caps_speed_without_braking(progress, cruise, state, expected, tag) -> None:
    from kaiev26_decision.arbiter import ModuleOutput, arbitrate
    from kaiev26_decision.curve_speed_application import speed_zone_cap

    route = school_route(progress)
    route.route_projection_valid = state != "unlocated"

    cap, tags = speed_zone_cap(route, school_zones(), cruise, FRONT, route_is_fresh=state != "stale")

    assert cap == (None if expected is None else pytest.approx(expected))
    assert tags == ([tag] if tag else [])
    decision = arbitrate([
        ModuleOutput("route_follow", cap_mps=cruise),
        ModuleOutput("speed_zone", cap_mps=cap, tags=tags),
    ])
    assert decision.need_stop is False
    assert decision.target_speed_mps == pytest.approx(cruise if expected is None else expected)


def test_school_zone_keeps_a_lower_stop_point_cap() -> None:
    from kaiev26_decision.arbiter import ModuleOutput, arbitrate
    from kaiev26_decision.curve_speed_application import speed_zone_cap

    cap, tags = speed_zone_cap(school_route(345.0), school_zones(), 8.0, FRONT)
    decision = arbitrate([
        ModuleOutput("route_follow", cap_mps=8.0),
        ModuleOutput("stop_point", cap_mps=2.0, stop_distance_m=4.0),
        ModuleOutput("speed_zone", cap_mps=cap, tags=tags),
    ])

    assert decision.speed_limit_mps == pytest.approx(2.0)
    assert (decision.need_stop, decision.stop_distance_m) == (True, pytest.approx(4.0))


def test_only_the_qualifying_policy_has_the_school_zone() -> None:
    from kaiev26_decision.zone_policy import load_zone_policy

    for name in ("final_policy.yaml", "midan_test_policy.yaml"):
        assert load_zone_policy(PACKAGE / "config" / name).speed_zones == ()
    (zone,) = load_zone_policy(PACKAGE / "config" / "qualifying_policy.yaml").speed_zones
    assert zone.zone_id == "SCHOOL_ZONE"
    assert (zone.start_route_s, zone.end_route_s) == pytest.approx((SCHOOL_START, SCHOOL_END))
    assert zone.speed_cap_mps * 3.6 == pytest.approx(27.0)
    assert zone.start_route_s < 351.8 < zone.end_route_s  # signal stop line 048 inside


@pytest.mark.parametrize(
    "item,message",
    [
        pytest.param({"start_route_s": 10.0, "end_route_s": 20.0, "speed_cap_mps": 7.5,
                      "pre_slow_decel_mps2": 0.5}, "requires [id]", id="missing-id"),
        pytest.param({"id": "Z", "start_route_s": 20.0, "end_route_s": 10.0,
                      "speed_cap_mps": 7.5, "pre_slow_decel_mps2": 0.5},
                     "end_route_s", id="end-before-start"),
        pytest.param({"id": "Z", "start_route_s": 10.0, "end_route_s": 20.0,
                      "speed_cap_mps": 0.0, "pre_slow_decel_mps2": 0.5},
                     "speed_cap_mps", id="zero-cap"),
        pytest.param({"id": "Z", "start_route_s": 10.0, "end_route_s": 20.0,
                      "speed_cap_mps": 7.5}, "pre_slow_decel_mps2", id="missing-decel"),
    ],
)
def test_invalid_speed_zone_is_rejected(item, message) -> None:
    from kaiev26_decision.zone_policy import _parse_speed_zones

    with pytest.raises(ValueError, match=message.replace("[", r"\[").replace("]", r"\]")):
        _parse_speed_zones([item])


def test_duplicate_speed_zone_ids_are_rejected() -> None:
    from kaiev26_decision.zone_policy import _parse_speed_zones

    item = {"id": "Z", "start_route_s": 10.0, "end_route_s": 20.0, "speed_cap_mps": 7.5,
            "pre_slow_decel_mps2": 0.5}
    with pytest.raises(ValueError, match="unique"):
        _parse_speed_zones([item, dict(item)])


# ---- curve cap from the zone radius, approach at the measured decel ----
from kaiev26_decision.curve_speed_policy import (  # noqa: E402
    resolve_curve_approach_decel_override,
    resolve_curve_lateral_accel,
)


def test_radius_cap_slows_tight_curves_below_the_cruise_scale():
    route = str(PACKAGE / "waypoints" / "kcity_final_route.yaml")
    plain = load_curve_speed_policy(route)
    capped = load_curve_speed_policy(route, lateral_accel_mps2=4.0, approach_decel_override_mps2=0.5)
    assert capped.approach_decel_mps2 == 0.5
    radii = {zone.zone_id: zone.radius_m for zone in capped.zones}
    assert 8.0 < radii["F3"] < 9.0 and 12.5 < radii["F1"] < 13.5
    assert all(math.isinf(zone.radius_m) for zone in plain.zones)
    f3 = next(zone for zone in capped.zones if zone.zone_id == "F3")
    inside = (f3.start_s + f3.end_s) / 2.0
    limit = capped.limit_for_progress(inside, 15.0)
    assert limit.speed_limit_mps == pytest.approx(math.sqrt(4.0 * radii["F3"]))
    assert plain.limit_for_progress(inside, 15.0).speed_limit_mps == pytest.approx(12.0)
    # At 7 m/s the 0.8 scale (5.6 m/s) is already below the F1 radius cap.
    f1 = next(zone for zone in capped.zones if zone.zone_id == "F1")
    assert capped.limit_for_progress(f1.start_s + 1.0, 7.0).speed_limit_mps == pytest.approx(5.6)


def test_radius_cap_slows_down_earlier_at_the_measured_decel():
    route = str(PACKAGE / "waypoints" / "kcity_final_route.yaml")
    capped = load_curve_speed_policy(route, lateral_accel_mps2=4.0, approach_decel_override_mps2=0.5)
    f1 = capped.zones[0]
    corner = math.sqrt(4.0 * f1.radius_m)
    before = capped.limit_for_progress(f1.start_s - 50.0, 15.0)
    assert before.phase == "approach"
    assert before.speed_limit_mps == pytest.approx(math.sqrt(corner ** 2 + 2.0 * 0.5 * 50.0))


@pytest.mark.parametrize("value", [-1.0, 0.5, 11.0, float("nan")])
def test_bad_lateral_accel_is_rejected(value):
    with pytest.raises(ValueError):
        resolve_curve_lateral_accel(value)
    assert resolve_curve_lateral_accel(0.0) == 0.0


def test_approach_decel_override_keeps_the_yaml_at_minus_one():
    assert resolve_curve_approach_decel_override(-1.0) is None
    assert resolve_curve_approach_decel_override(0.5) == 0.5
    with pytest.raises(ValueError):
        resolve_curve_approach_decel_override(0.0)


def captured_route(tmp_path, radius=10.0):
    """A route captured in the test TUI: 40 m straight, a 90 deg left turn, 40 m straight."""
    import yaml
    from kaiev26_decision.route_capture import make_tracking_profile
    points = [(float(x), 0.0) for x in range(41)]
    for k in range(1, 16):
        angle = k / 15 * math.pi / 2
        points.append((40.0 + radius * math.sin(angle), radius * (1 - math.cos(angle))))
    points += [(40.0 + radius, radius + float(y)) for y in range(1, 41)]
    document = dict(route_name='curve', route_id='run', capture_mode='manual_tui', frame_id='map',
                    reference_point='base_footprint', map_origin={'x': 0.0, 'y': 0.0}, waypoints=[dict(x=x, y=y) for x, y in points])
    document['tracking_profile'] = make_tracking_profile(document)
    path = tmp_path / 'route.yaml'
    path.write_text(yaml.safe_dump(document))
    return str(path), document


def test_captured_route_gets_curve_zones_from_its_corners(tmp_path) -> None:
    from kaiev26_decision.curve_speed_policy import load_curve_speed_policy
    path, document = captured_route(tmp_path)
    corners = document['tracking_profile']['corners']
    assert corners
    policy = load_curve_speed_policy(path, lateral_accel_mps2=3.5, approach_decel_override_mps2=0.5)
    assert policy is not None and len(policy.zones) == len(corners)
    zone = policy.zones[0]
    assert (zone.start_s, zone.end_s) == pytest.approx((corners[0]['start_s_m'], corners[0]['end_s_m']))
    assert 8.0 < zone.radius_m < 12.0  # the 10 m arc
    assert policy.speed_scale == pytest.approx(0.8)
    assert policy.approach_decel_mps2 == pytest.approx(0.5)


def test_straight_captured_route_has_no_curve_policy(tmp_path) -> None:
    import yaml
    from kaiev26_decision.curve_speed_policy import load_curve_speed_policy
    from kaiev26_decision.route_capture import make_tracking_profile
    document = dict(route_name='s', route_id='run', capture_mode='manual_tui', frame_id='map',
                    reference_point='base_footprint', map_origin={'x': 0.0, 'y': 0.0}, waypoints=[dict(x=float(x), y=0.0) for x in range(60)])
    document['tracking_profile'] = make_tracking_profile(document)
    path = tmp_path / 'straight.yaml'
    path.write_text(yaml.safe_dump(document))
    assert load_curve_speed_policy(str(path)) is None
