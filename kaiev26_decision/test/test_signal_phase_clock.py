"""GNSS RMC time and the wall-clock prior check of the signal phase tracker node."""

from types import SimpleNamespace

import pytest

from kaiev26_decision.signal_phase_tracker_node import SignalPhaseTrackerNode, rmc_utc_sec


# 2026-10-17 F4 wait (143332): RMC 05:33:33.60 UTC.
RMC = "$GNRMC,053333.60,A,3714.33669,N,12646.37732,E,0.01,0.0,171026,,,D*7A"
RMC_UTC = 1792215213.6


@pytest.mark.parametrize(
    "sentence,expected",
    [
        pytest.param(RMC, RMC_UTC, id="valid"),
        pytest.param(RMC.replace(",A,", ",V,", 1), None, id="void-fix"),
        pytest.param("$GNGGA,053333.60,3714.33669,N", None, id="not-rmc"),
        pytest.param("$GNRMC,05xx33.60,A,,,,,,,171026,,,", None, id="bad-time"),
        pytest.param("", None, id="empty"),
    ],
)
def test_rmc_utc_sec(sentence, expected):
    value = rmc_utc_sec(sentence)
    if expected is None:
        assert value is None
    else:
        assert value == pytest.approx(expected, abs=1.0e-3)


class FakeTracker:
    def __init__(self):
        self.prior = None

    def set_prior(self, mean_sec, std_sec):
        self.prior = (mean_sec, std_sec)


def checking_node(samples=3, limit=0.5):
    node = SimpleNamespace(
        clock_checked=False, last_rmc="", clock_offsets=[],
        clock_check_samples=samples, clock_check_max_offset_sec=limit,
        prior_mean_sec=0.0, prior_std_sec=0.5, tracker=FakeTracker(),
    )
    node.get_logger = lambda: SimpleNamespace(info=lambda *_: None, warning=lambda *_: None)
    return node


def status(sentence, stamp_sec):
    sec = int(stamp_sec)
    return SimpleNamespace(
        latest_raw_rmc=sentence,
        header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=int((stamp_sec - sec) * 1e9))),
    )


def feed(node, offsets, first=0):
    for i, offset in enumerate(offsets, start=first):
        sentence = RMC.replace("053333.60", f"0533{34 + i:02d}.60")
        SignalPhaseTrackerNode.on_gnss_status(node, status(sentence, RMC_UTC + 1.0 + i + offset))


# situation: node clock minus RMC time per sample -> the prior is given.
CLOCK_CASES = [
    pytest.param([0.08, 0.04, 0.06], True, id="10-17-like-offsets-turn-the-prior-on"),
    pytest.param([0.9, 0.8, 0.7], False, id="clock-off-by-0.7-s-keeps-it-off"),
    pytest.param([0.9, 0.3, 0.8], True, id="smallest-offset-is-used"),
]


@pytest.mark.parametrize("offsets,prior_on", CLOCK_CASES)
def test_prior_is_given_only_after_the_clock_matches_gnss(offsets, prior_on):
    node = checking_node()

    feed(node, offsets[:-1])
    assert node.tracker.prior is None and not node.clock_checked
    feed(node, offsets[-1:], first=len(offsets) - 1)

    assert node.clock_checked
    assert (node.tracker.prior == (0.0, 0.5)) == prior_on
    if not prior_on:
        assert node.tracker.prior is None


def test_repeated_rmc_counts_once():
    node = checking_node(samples=2)
    message = status(RMC, RMC_UTC + 0.05)
    SignalPhaseTrackerNode.on_gnss_status(node, message)
    SignalPhaseTrackerNode.on_gnss_status(node, message)

    assert node.clock_offsets == [pytest.approx(0.05, abs=1.0e-6)]
    assert not node.clock_checked
