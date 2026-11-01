"""Frozen localization pose stops the route context (decisions D2, D3)."""

from types import SimpleNamespace

import pytest

from kaiev26_decision.route_zone_manager_node import RouteZoneManagerNode

SEC = 1_000_000_000

# (pose stamp ns, node clock ns, frozen?) fed in order to one node.
FREEZE_SEQUENCES = [
    pytest.param([(1 * SEC, 10 * SEC, False), (2 * SEC, 10 * SEC + SEC // 20, False)],
                 id="advancing-stamp"),
    pytest.param([(1 * SEC, 10 * SEC, False), (1 * SEC, 10 * SEC + 4 * SEC // 10, False),
                  (1 * SEC, 10 * SEC + 6 * SEC // 10, True)], id="same-stamp-over-0.5s"),
    pytest.param([(1 * SEC, 10 * SEC, False), (1 * SEC, 10 * SEC + 6 * SEC // 10, True),
                  (2 * SEC, 10 * SEC + 7 * SEC // 10, False)], id="recovers-when-stamp-advances"),
    pytest.param([(0, 10 * SEC, False), (0, 20 * SEC, False)], id="zero-stamp-never-frozen"),
    pytest.param([(500 * SEC, 10 * SEC, False), (500 * SEC + SEC // 10, 10 * SEC + SEC // 10, False)],
                 id="clock-offset-ignored"),
]


@pytest.mark.parametrize("sequence", FREEZE_SEQUENCES)
def test_pose_freeze(sequence) -> None:
    node = SimpleNamespace(pose_freeze_sec=0.5, last_pose_stamp_ns=None, last_pose_change_ns=None)
    for stamp_ns, now_ns, frozen in sequence:
        assert RouteZoneManagerNode.pose_frozen(node, stamp_ns, now_ns) is frozen
