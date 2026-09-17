from collections import deque
import threading

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from kaiev26_msgs.msg import SteeringActuatorStatus, VehicleState
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
import pytest

from kaiev26_decision.test_tui import (
    CONTROLLER_NAMES,
    IMPLEMENTED_CONTROLLERS,
    DecisionTestNode,
    READINESS_LABELS,
    RECORD_TOPICS,
    REQUIRED_EXTERNAL_TOPICS,
    STEERING_STEP_RECORD_TOPICS,
    STEERING_STEP_REQUIRED_TOPICS,
    TestSettings as DecisionTestSettings,
    render,
)


@pytest.mark.parametrize(
    ("case", "course", "mode"),
    [
        (1, "qualifying", "tracking"),
        (2, "qualifying", "stopline"),
        (3, "qualifying", "all"),
        (4, "final", "tracking"),
        (5, "final", "stopline"),
        (6, "final", "avoidance"),
        (7, "final", "all"),
        (8, "steering_step", "steering_step"),
    ],
)
def test_decision_test_case_contract(case: int, course: str, mode: str) -> None:
    settings = DecisionTestSettings(case=case)

    assert settings.course == course
    assert settings.mode == mode


def test_course_defaults_follow_manual_drive_recordings() -> None:
    assert DecisionTestSettings(case=1).effective_speed_mps == pytest.approx(3.2)
    assert DecisionTestSettings(case=3).effective_speed_mps == pytest.approx(3.2)
    assert DecisionTestSettings(case=4).effective_speed_mps == pytest.approx(3.5)
    assert DecisionTestSettings(case=7).effective_speed_mps == pytest.approx(3.5)
    assert DecisionTestSettings(case=8).effective_speed_mps == pytest.approx(1.0)


def test_explicit_speed_overrides_course_default_for_every_mode() -> None:
    for case in range(1, 9):
        settings = DecisionTestSettings(case=case, speed_mps=2.4)
        assert settings.effective_speed_mps == pytest.approx(2.4)


def test_zero_is_the_only_course_default_sentinel() -> None:
    assert DecisionTestSettings(speed_mps=0.0).effective_speed_mps == pytest.approx(3.2)
    with pytest.raises(ValueError):
        DecisionTestSettings(speed_mps=0.05)


def test_all_selectable_lateral_controllers_are_implemented() -> None:
    assert list(CONTROLLER_NAMES) == [
        "pure_pursuit",
        "stanley",
        "pp_stanley",
        "ff_stanley",
    ]
    assert IMPLEMENTED_CONTROLLERS == {
        "pure_pursuit",
        "stanley",
        "pp_stanley",
        "ff_stanley",
    }
    assert DecisionTestSettings(controller="pure_pursuit").implemented
    assert DecisionTestSettings(controller="stanley").implemented
    assert DecisionTestSettings(controller="pp_stanley").implemented
    assert DecisionTestSettings(controller="ff_stanley").implemented


def test_decision_mcap_topic_list_has_no_duplicates() -> None:
    assert len(RECORD_TOPICS) == len(set(RECORD_TOPICS))
    assert "/planning/command" in RECORD_TOPICS
    assert "/decision/raw_command" in RECORD_TOPICS
    assert "/planning/route_context" in RECORD_TOPICS
    assert "/planning/readiness_status" in RECORD_TOPICS
    assert "/perception/camera/left/source/image_raw/compressed" in RECORD_TOPICS
    assert "/perception/camera/right/source/image_raw/compressed" in RECORD_TOPICS
    assert "/perception/camera/left/source/camera_info" in RECORD_TOPICS
    assert "/perception/camera/right/source/camera_info" in RECORD_TOPICS


def test_external_localization_is_required_before_starting_a_vehicle_run() -> None:
    assert "/gnss/fix" in REQUIRED_EXTERNAL_TOPICS
    assert "/gnss/fix_velocity" in REQUIRED_EXTERNAL_TOPICS
    assert "/localization/odometry" in REQUIRED_EXTERNAL_TOPICS


def test_steering_step_uses_compact_dynamics_recording() -> None:
    settings = DecisionTestSettings(
        case=8,
        speed_mps=1.5,
        steering_amplitude_deg=4.0,
        steering_frequency_hz=0.5,
        steering_pattern="rlr",
    )

    assert settings.steering_pattern == "RLR"
    assert settings.record_topics == STEERING_STEP_RECORD_TOPICS
    assert len(settings.record_topics) == len(set(settings.record_topics))
    assert "/steering/status" in settings.record_topics
    assert "/drive/status" in settings.record_topics
    assert "/decision/steering_step_state" in settings.record_topics
    assert {
        "/gnss/fix",
        "/gnss/fix_velocity",
        "/gnss/status",
        "/gnss/status_verbose",
        "/imu/data",
        "/imu/data_raw",
        "/ebimu/status",
        "/localization/odometry",
        "/localization/odometry_gnss",
        "/stm_a/runtime_stats",
        "/stm_b/runtime_stats",
        "/tf",
        "/tf_static",
    }.issubset(settings.record_topics)
    assert not any("camera" in topic or "lidar" in topic for topic in settings.record_topics)
    document = settings.document()
    assert document["steering_step"]["pattern"] == "RLR"
    assert document["steering_step"]["pulse_duty_ratio"] == pytest.approx(0.5)


def test_steering_step_does_not_require_gnss_or_route() -> None:
    assert STEERING_STEP_REQUIRED_TOPICS == (
        "/vehicle/state",
        "/drive/status",
        "/steering/status",
    )


def test_steering_step_selection_renders_pattern_editor(tui) -> None:
    settings = DecisionTestSettings(case=8, steering_pattern="RLRL")
    tui.settings = settings

    table = render(
        tui.snapshot(),
        settings,
        ("steering_pattern", "RLR"),
        "",
    )

    assert table is not None


@pytest.fixture
def tui():
    # Exercise the display consumer without creating vehicle command publishers.
    node = DecisionTestNode.__new__(DecisionTestNode)
    node.lock = threading.RLock()
    node.received = {}
    node.settings = DecisionTestSettings()
    node.state = 'PREFLIGHT'
    node.graph_issues = []
    node.logs = deque()
    node.run_dir = ''
    node.finish_reason = ''
    node.now = lambda: 10.0
    return node


def test_received_gnss_uses_actual_preflight_failure(tui) -> None:
    fix = NavSatFix()
    fix.status.status = 0
    tui.receive('/gnss/fix', fix)
    tui.receive('/planning/route_ready', Bool(data=False))
    diagnostics = DiagnosticArray(status=[
        DiagnosticStatus(
            name=topic,
            level=DiagnosticStatus.WARN if topic == '/gnss/fix' else DiagnosticStatus.OK,
            values=[KeyValue(key='detail', value='RTK FIX 미확보' if topic == '/gnss/fix' else '통과')],
        )
        for topic in READINESS_LABELS
    ])
    tui.receive('/planning/readiness_status', diagnostics)

    snapshot = tui.snapshot()
    rows = {topic: (received, state, detail) for topic, received, state, detail in snapshot['readiness']}
    assert rows['/gnss/fix'][0] == '수신'
    assert rows['/gnss/fix'][1] == 'WAIT'
    assert rows['/gnss/fix'][2] == diagnostics.status[0].values[0].value
    assert rows['/gnss/fix_velocity'][1] == 'GOOD'
    assert not snapshot['ready']


def test_received_inputs_without_diagnostics_are_not_declared_good(tui) -> None:
    for topic in REQUIRED_EXTERNAL_TOPICS:
        tui.receive(topic, object())

    assert all(state != 'GOOD' for _topic, _received, state, _detail in tui.readiness_rows())


def test_stale_readiness_does_not_keep_good_or_start_permission(tui) -> None:
    diagnostics = DiagnosticArray(status=[
        DiagnosticStatus(name=topic, level=DiagnosticStatus.OK)
        for topic in READINESS_LABELS
    ])
    tui.receive('/planning/readiness_status', diagnostics)
    tui.receive('/planning/route_ready', Bool(data=True))
    assert tui.snapshot()['ready']

    tui.now = lambda: 11.1
    snapshot = tui.snapshot()

    assert not snapshot['ready']
    assert all(state != 'GOOD' for _topic, _received, state, _detail in snapshot['readiness'])


def test_missing_perception_is_visible_alongside_other_blockers(tui) -> None:
    tui.settings = DecisionTestSettings(case=3)

    rows = {topic: state for topic, _received, state, _detail in tui.readiness_rows()}

    assert rows['/gnss/fix'] == 'WAIT'
    assert rows['/planning/scene_summary'] == 'WAIT'


def test_steering_tracking_fault_warns_without_blocking_run(tui) -> None:
    for topic in REQUIRED_EXTERNAL_TOPICS:
        tui.receive(topic, object())
    vehicle = VehicleState()
    vehicle.mode = VehicleState.MODE_MANUAL
    tui.receive('/vehicle/state', vehicle)
    steering = SteeringActuatorStatus()
    steering.communication_ok = True
    steering.feedback_valid = True
    steering.tracking_error_fault = True
    steering.tracking_error_rad = 0.08
    tui.receive('/steering/status', steering)

    assert 'KEYA 조향 통신/피드백 상태 확인' not in tui.external_issues()
    assert any('시험·MCAP 기록 계속' in item for item in tui.runtime_warnings())
