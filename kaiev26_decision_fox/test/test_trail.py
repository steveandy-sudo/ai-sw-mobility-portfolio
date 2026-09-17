from kaiev26_decision_fox.trail import TrailBuffer, TrailPoint


def point(x: float, stamp_ns: int) -> TrailPoint:
    return TrailPoint(x=x, y=0.0, z=0.0, stamp_ns=stamp_ns)


def test_trail_respects_spacing_and_bound() -> None:
    trail = TrailBuffer(max_points=3, min_spacing_m=0.5)
    assert trail.add(point(0.0, 1))
    assert not trail.add(point(0.2, 2))
    assert trail.add(point(0.6, 3))
    assert trail.add(point(1.2, 4))
    assert trail.add(point(1.8, 5))
    assert [item.x for item in trail.points] == [0.6, 1.2, 1.8]


def test_trail_clears_when_time_moves_backwards() -> None:
    trail = TrailBuffer(max_points=10, min_spacing_m=0.0)
    trail.add(point(0.0, 100))
    trail.add(point(1.0, 200))
    assert trail.add(point(5.0, 50))
    assert [item.x for item in trail.points] == [5.0]


def test_trail_clears_after_localization_jump() -> None:
    trail = TrailBuffer(max_points=10, min_spacing_m=0.0, max_jump_m=5.0)
    trail.add(point(0.0, 100))
    trail.add(point(1.0, 200))
    trail.add(point(30.0, 300))
    assert [item.x for item in trail.points] == [30.0]
