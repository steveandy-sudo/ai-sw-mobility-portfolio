from collections import deque
from datetime import datetime
from types import SimpleNamespace
import threading
from zoneinfo import ZoneInfo

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from kaiev26_msgs.msg import ActuatorCommand, BehaviorDecision, PerceptionObjectArray, RouteContext, SceneSummary, SteeringActuatorStatus, VehicleState
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
import pytest

from kaiev26_decision.test_tui import (
    COURSE_NAMES, DecisionTestNode, SettingsEditor,
    READINESS_LABELS, RECORD_TOPICS, REQUIRED_EXTERNAL_TOPICS,
    TestSettings as DecisionTestSettings, create_recording_directory, render, supported_cases,
)


@pytest.mark.parametrize('settings,expected', [
    (DecisionTestSettings(), '261021_1400_decision_case01_qualifying_V7'),
    (DecisionTestSettings(course='final'), '261021_1400_decision_case01_final_V10'),
    (DecisionTestSettings(case=2, speed_mps=7.5), '261021_1400_decision_case02_qualifying_V7p5'),
    (DecisionTestSettings(case=3, route_name='미단_직선'), '261021_1400_decision_case03_미단_직선_V0'),
    (DecisionTestSettings(case=4, course='custom:old_run_id', route_name='미단_직선',
                          route_config='/tmp/route.yaml'), '261021_1400_decision_case04_미단_직선_V10'),
])
def test_recording_names_use_case_route_and_speed(tmp_path, settings, expected):
    stamp = datetime(2026, 10, 21, 14, 0, 59, 123456, tzinfo=ZoneInfo('Asia/Seoul'))
    directory = create_recording_directory(tmp_path, settings, stamp)
    assert directory.name == expected
    assert directory.is_dir()


def test_same_minute_recordings_never_overwrite_existing_data(tmp_path):
    stamp = datetime(2026, 10, 21, 14, 0, tzinfo=ZoneInfo('Asia/Seoul'))
    first = create_recording_directory(tmp_path, DecisionTestSettings(), stamp)
    (first / 'original.mcap').write_bytes(b'keep')
    second = create_recording_directory(tmp_path, DecisionTestSettings(), stamp)
    third = create_recording_directory(tmp_path, DecisionTestSettings(), stamp)
    assert second.name == first.name + '_02'
    assert third.name == first.name + '_03'
    assert (first / 'original.mcap').read_bytes() == b'keep'


def test_recording_route_name_cannot_create_nested_paths(tmp_path):
    settings = DecisionTestSettings(case=3, route_name='../직선 / 왕복')
    stamp = datetime(2026, 10, 21, 14, 0, tzinfo=ZoneInfo('Asia/Seoul'))
    directory = create_recording_directory(tmp_path, settings, stamp)
    assert directory.parent == tmp_path
    assert '직선' in directory.name
    assert '/' not in directory.name


def test_recorder_uses_compact_directory_and_retains_precise_metadata(tmp_path, monkeypatch):
    import yaml
    from kaiev26_decision import test_tui

    stamp = datetime(2026, 10, 21, 14, 0, 59, 123456, tzinfo=ZoneInfo('Asia/Seoul'))
    class FixedDateTime:
        @staticmethod
        def now(timezone):
            return stamp

    monkeypatch.setattr(test_tui, 'datetime', FixedDateTime)
    monkeypatch.setattr(test_tui, 'workspace_root', lambda: tmp_path)
    monkeypatch.setattr(test_tui, 'OwnedProcess',
                        lambda name, argv, log: SimpleNamespace(argv=argv))
    node = SimpleNamespace(settings=DecisionTestSettings(case=2), log=lambda text: None)
    DecisionTestNode.start_recording(node)
    directory = tmp_path / 'records/261021_1400_decision_case02_qualifying_V7'
    document = yaml.safe_load((directory / 'scenario.yaml').read_text())
    assert node.run_dir == str(directory)
    assert str(directory / 'rosbag') in node.recorder_process.argv
    assert document['started_at'] == stamp.isoformat()
    assert document['mode'] == 'tracking'
    assert document['speed_mps'] == 7.0


@pytest.mark.parametrize("case,course,mode,launch_case", [
    (1, 'qualifying', 'all', 3), (1, 'final', 'all', 7),
    (2, 'qualifying', 'tracking', 1), (2, 'final', 'tracking', 4),
    (3, 'qualifying', 'route_capture', None),
    (2, 'custom:run', 'tracking', 13), (4, 'custom:run', 'avoidance', 17),
    (3, 'custom:run', 'route_capture', None),
])
def test_decision_test_case_contract(case, course, mode, launch_case):
    settings = DecisionTestSettings(case=case, course=course, route_config='/tmp/route.yaml')
    assert settings.mode == mode
    assert settings.launch_case == launch_case
    assert settings.document()['case_schema_version'] == 2
    assert settings.document()['route_used'] == (case != 3)
    if case == 3:
        assert settings.launch_arguments() == []
        assert settings.document()['lateral_controller'] is None
    else:
        assert f'case:={launch_case}' in settings.launch_arguments()


def test_only_four_cases_and_competition_routes_are_predefined():
    assert tuple(COURSE_NAMES) == ('qualifying', 'final')
    assert supported_cases('qualifying') == (1, 2, 3)
    assert supported_cases('final') == (1, 2, 3)
    assert supported_cases('custom:run') == (2, 3, 4)


@pytest.mark.parametrize('case,course', [(4, 'qualifying'), (4, 'final'), (1, 'custom:run')])
def test_cases_require_their_registered_mission_policy(case, course):
    with pytest.raises(ValueError, match='지원하지 않는 Case'):
        DecisionTestSettings(case=case, course=course, route_config='/tmp/route.yaml')


@pytest.mark.parametrize('case,course,expected', [
    (1, 'qualifying', 7.), (2, 'qualifying', 7.),
    (1, 'final', 10.), (2, 'final', 10.),
    (2, 'custom:run', 10.), (4, 'custom:run', 10.),
])
def test_course_defaults_and_explicit_speed(case, course, expected):
    settings = DecisionTestSettings(case=case, course=course, route_config='/tmp/route.yaml')
    assert settings.effective_speed_mps == expected
    assert DecisionTestSettings(case=case, course=course, route_config='/tmp/route.yaml',
                                speed_mps=2.4).effective_speed_mps == 2.4


@pytest.mark.parametrize('speed', [-1., .05, 15.1, float('inf'), float('nan')])
def test_speed_must_be_finite_and_in_range(speed):
    with pytest.raises(ValueError):
        DecisionTestSettings(speed_mps=speed)


def test_tracking_accepts_only_speed_and_course_controls():
    editor = SettingsEditor(selection=DecisionTestSettings(case=2))
    for key in '56789mafwlnh':
        assert not editor.handle_key(key, 'IDLE')
        assert editor.edit_field is None
    for char in 'v1e1\n':
        assert editor.handle_key(char, 'IDLE')
    assert editor.selection.speed_mps == 10.
    assert editor.selection.launch_arguments() == [
        'case:=1', 'speed_mps:=10.0', 'platform:=vehicle',
        'command_topic:=/decision/raw_command', 'start_mode:=from_start',
    ]


@pytest.mark.parametrize('case,course', [(1, 'qualifying'), (2, 'final'), (4, 'custom:run')])
def test_b_toggles_the_start_mode_for_driving_cases(case, course):
    editor = SettingsEditor(selection=DecisionTestSettings(
        case=case, course=course, route_config='/tmp/route.yaml'))
    assert 'start_mode:=from_start' in editor.selection.launch_arguments()
    assert editor.handle_key('b', 'IDLE')
    assert 'start_mode:=resume' in editor.selection.launch_arguments()
    assert editor.handle_key('b', 'IDLE')
    assert editor.selection.start_mode == 'from_start'


def test_route_capture_has_no_start_mode():
    editor = SettingsEditor(selection=DecisionTestSettings(case=3))
    assert not editor.handle_key('b', 'IDLE')
    assert editor.selection.launch_arguments() == []


def test_start_mode_must_be_known():
    with pytest.raises(ValueError):
        DecisionTestSettings(start_mode='middle')


def test_settings_editor_locks_configuration_during_a_run():
    editor = SettingsEditor()
    original = editor.selection
    assert editor.handle_key('4', 'RUNNING')
    assert editor.selection == original and '실행 중' in editor.message


def test_settings_editor_keeps_stop_and_quit_available_while_editing():
    editor = SettingsEditor()
    assert editor.handle_key('v', 'IDLE')
    assert not editor.handle_key('s', 'IDLE')
    assert editor.edit_field is None


def test_p_cycles_routes_and_preserves_a_supported_case():
    editor = SettingsEditor()
    assert editor.handle_key('p', 'IDLE')
    assert editor.selection.course == 'final'
    assert editor.handle_key('3', 'IDLE')
    assert editor.selection.launch_case is None
    assert editor.handle_key('p', 'IDLE')
    assert editor.selection.course == 'qualifying' and editor.selection.case == 3


def test_new_route_can_be_selected_for_tracking_or_avoidance():
    editor = SettingsEditor(selection=DecisionTestSettings(course='final'))
    editor.custom_routes = {'custom:run': ('새 경로', '/tmp/route.yaml')}
    assert editor.handle_key('p', 'IDLE')
    assert editor.selection.course == 'custom:run' and editor.selection.case == 2
    assert editor.selection.launch_case == 13
    assert editor.handle_key('4', 'IDLE')
    assert editor.selection.launch_case == 17
    assert editor.handle_key('p', 'IDLE')
    assert editor.selection.course == 'qualifying' and editor.selection.case == 2


def test_capture_name_accepts_unicode_space_and_command_letters():
    editor = SettingsEditor()
    editor.handle_key('3', 'IDLE')
    for char in 'n미단 수동 s q road\n':
        assert editor.handle_key(char, 'IDLE')
    assert editor.selection.route_name == '미단 수동 s q road'
    assert editor.selection.launch_arguments() == []
    assert not editor.handle_key('v', 'IDLE')


@pytest.mark.parametrize('state', ['IDLE', 'CAPTURE_PREP', 'WAIT_MOVEMENT', 'CAPTURING',
                                  'STOPPING', 'CLOSING', 'CONVERTING', 'COMPLETE', 'ERROR'])
def test_capture_never_publishes_vehicle_command_or_run_active(tui, state):
    tui.settings = DecisionTestSettings(case=3)
    tui.state = state
    tui.publish_command(True, True)
    tui.publish_run_active()


def test_failed_capture_metadata_write_does_not_leave_converting(tui, tmp_path, monkeypatch):
    from pathlib import Path
    tui.run_dir = str(tmp_path)
    tui.state = 'CONVERTING'
    tui.capture = {'invalid_reason': 'test interruption'}
    (tmp_path / 'scenario.yaml').write_text('{}')
    def fail_write(*args, **kwargs):
        raise OSError('disk full')
    monkeypatch.setattr(Path, 'write_text', fail_write)
    tui.convert_capture()
    assert tui.state == 'ERROR' and 'disk full' in tui.finish_reason


def test_capture_recording_keeps_dynamics_without_cameras():
    topics = DecisionTestSettings(case=3).record_topics
    assert '/drive/longitudinal_status' in topics and '/stm_b/runtime_stats' in topics
    assert not any('/image' in t for t in topics)
    assert '/ouster/points' not in topics


@pytest.mark.parametrize('case,course', [(4, 'custom:run'), (1, 'final')])
def test_obstacle_cases_record_sensor_inputs_and_perception_outputs(case, course):
    settings = DecisionTestSettings(case=case, course=course, route_config='/tmp/route.yaml')
    expected = {
        '/perception/camera/left/source/image_raw/compressed',
        '/perception/camera/right/source/image_raw/compressed',
        '/perception/camera/left/source/camera_info', '/perception/camera/right/source/camera_info',
        '/ouster/points', '/perception/road_segments', '/perception/centerline',
        '/perception/objects', '/perception/traffic_lights', '/perception/occupancy_grid',
        '/pv26_perception/left/overlay/compressed', '/pv26_perception/right/overlay/compressed',
        '/pv26_perception/traffic_light_detections', '/pv26_perception/markers',
        '/pv26_perception/map_alignment', '/tf', '/tf_static',
    }
    assert expected <= set(settings.record_topics)
    assert len(settings.record_topics) == len(set(settings.record_topics))


@pytest.mark.parametrize('course', ['qualifying', 'final'])
def test_signal_runs_record_the_signal_phase_estimate(course):
    settings = DecisionTestSettings(case=1, course=course)
    assert '/planning/traffic_signal_phases' in settings.record_topics
    assert '/planning/traffic_signal_phases/evidence' in settings.record_topics


@pytest.mark.parametrize('case,course', [(2, 'final'), (1, 'qualifying'), (3, 'qualifying')])
def test_other_cases_do_not_add_obstacle_recording_topics(case, course):
    topics = DecisionTestSettings(case=case, course=course).record_topics
    assert '/ouster/points' not in topics
    assert not any(t.startswith('/pv26_perception/') for t in topics)


@pytest.mark.parametrize('case,course', [(1, 'qualifying'), (2, 'final'), (3, 'qualifying'), (4, 'custom:run')])
@pytest.mark.parametrize('width', [80, 120])
def test_screen_shows_the_editors_of_the_case(tui, case, course, width):
    from io import StringIO
    from rich.console import Console
    settings = DecisionTestSettings(case=case, course=course, route_config='/tmp/route.yaml')
    tui.settings = settings
    output = StringIO()
    Console(file=output, width=width, color_system=None).print(render(tui.snapshot(), settings, None, ''))
    screen = output.getvalue()
    for label in ('전역경로 추종', '수동주행 경로'):
        assert label in screen
    assert ('장애물 회피' in screen) == course.startswith('custom:')
    assert ('목표속도:' in screen) == (case != 3)
    assert ('출발: 처음부터' in screen) == (case != 3)
    assert ('경로 이름:' in screen) == (case == 3)


def test_decision_mcap_topic_list_has_no_duplicates():
    assert len(RECORD_TOPICS) == len(set(RECORD_TOPICS))
    assert {'/planning/command', '/decision/raw_command', '/vehicle/command',
            '/planning/route_context', '/planning/readiness_status'} <= set(RECORD_TOPICS)


@pytest.mark.parametrize('case,course', [(1, 'final'), (1, 'qualifying'), (4, 'custom:run')])
def test_runs_record_the_route_context_age(case, course):
    settings = DecisionTestSettings(case=case, course=course, route_config='/tmp/route.yaml')
    assert '/debug/route_context_age' in settings.record_topics
    assert '/debug/vehicle_state_age' in settings.record_topics


def test_external_localization_is_required_before_starting_a_vehicle_run():
    assert {'/gnss/fix', '/gnss/fix_velocity', '/localization/odometry'} <= set(REQUIRED_EXTERNAL_TOPICS)


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


# Finished is Decision's finish stop only (decision D11): FINISH_HELD in a fresh
# /planning/behavior_decision. Steps are (time s, constraints or None for no message,
# rear axle distance to finish, raw brake) -> Run state after the tick.
FINISH_CASES = [
    pytest.param("qualifying", [
        (10.0, ["FINISH_STOP"], 0.8, True, 'RUNNING'),
        (10.1, ["FINISH_STOP", "FINISH_HELD"], 0.8, True, 'STOPPING'),
    ], id="qualifying-finish-held-completes"),
    pytest.param("final", [
        (10.0, ["FINISH_STOP", "FINISH_HELD", "STOP_BRAKE_HELD"], 3.5, True, 'STOPPING'),
    ], id="held-through-recovery-completes-wherever-it-stopped"),
    pytest.param("qualifying", [
        (10.0, ["FINISH_DECEL"], 0.5, True, 'RUNNING'),
        (11.5, ["FINISH_DECEL"], 0.5, True, 'RUNNING'),
    ], id="old-rule-endpoint-brake-alone-does-not-complete"),
    pytest.param("final", [
        (10.0, ["FINISH_STOP", "FINISH_HELD"], 0.8, True, 'STOPPING'),
    ], id="final-finish-held-completes"),
]


@pytest.mark.parametrize("course,timeline", FINISH_CASES)
def test_run_completes_only_on_decision_finish_held(tui, course, timeline) -> None:
    tui.settings = DecisionTestSettings(course=course)
    tui.state = 'RUNNING'
    tui.launch_process = None
    tui.recorder_process = None
    tui.publish_run_active = lambda: None
    tui.publish_command = lambda *args, **kwargs: None
    tui.publish_label = lambda: None
    for now, constraints, distance, brake, state in timeline:
        tui.now = lambda: now
        tui.receive('/vehicle/state', VehicleState(mode=VehicleState.MODE_AUTO, speed_mps=0.0))
        tui.receive('/planning/route_ready', Bool(data=True))
        tui.receive('/planning/route_context', RouteContext(distance_to_finish=distance))
        tui.receive('/decision/raw_command', ActuatorCommand(brake_engage=brake))
        tui.receive('/planning/behavior_decision', BehaviorDecision(constraints=constraints))
        tui.tick()
        assert tui.state == state, now
    if state == 'STOPPING':
        assert tui.requested_terminal == 'COMPLETE'


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
    tui.settings = DecisionTestSettings(case=2)
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
    tui.settings = DecisionTestSettings(case=1)

    rows = {topic: state for topic, _received, state, _detail in tui.readiness_rows()}

    assert rows['/gnss/fix'] == 'WAIT'
    assert rows['/planning/scene_summary'] == 'WAIT'


@pytest.mark.parametrize('settings', [
    DecisionTestSettings(case=1),
    DecisionTestSettings(case=4, course='custom:run', route_config='/tmp/route.yaml'),
    DecisionTestSettings(case=1, course='final'),
])
def test_perception_dependent_cases_require_good_perception_before_go(
    tui, settings: DecisionTestSettings
) -> None:
    tui.settings = settings
    tui.receive('/planning/route_ready', Bool(data=True))

    scene = SceneSummary()
    scene.perception_health = 'DEGRADED'
    tui.receive('/planning/scene_summary', scene)
    assert not tui.snapshot()['ready']

    scene.perception_health = 'GOOD'
    tui.receive('/planning/scene_summary', scene)
    assert tui.snapshot()['ready']


def test_tracking_case_does_not_use_the_perception_gate(tui) -> None:
    tui.settings = DecisionTestSettings(case=2)
    tui.receive('/planning/route_ready', Bool(data=True))

    assert tui.snapshot()['ready']
    assert all(
        topic != '/planning/scene_summary'
        for topic, _received, _state, _detail in tui.readiness_rows()
    )


def test_avoidance_can_start_with_fresh_lidar_and_valid_route_without_lanes(tui):
    tui.settings = DecisionTestSettings(case=4, course='custom:run', route_config='/tmp/route.yaml')
    route = RouteContext(route_projection_valid=True)
    route.header.frame_id = 'base_footprint'
    objects = PerceptionObjectArray()
    objects.header.frame_id = 'base_footprint'
    tui.receive('/planning/route_context', route)
    tui.receive('/perception/objects', objects)
    tui.receive('/planning/route_ready', Bool(data=True))
    assert tui.snapshot()['ready']
    tui.now = lambda: 10.6
    assert not tui.perception_gate()[0]


@pytest.mark.parametrize('course', ['qualifying', 'final'])
def test_lidar_only_does_not_bypass_signal_perception_readiness(tui, course):
    tui.settings = DecisionTestSettings(case=1, course=course)
    route = RouteContext(route_projection_valid=True)
    route.header.frame_id = 'base_footprint'
    objects = PerceptionObjectArray()
    objects.header.frame_id = 'base_footprint'
    tui.receive('/planning/route_context', route)
    tui.receive('/perception/objects', objects)
    assert not tui.perception_gate()[0]


def test_lidar_only_readiness_rejects_incompatible_frame(tui):
    tui.settings = DecisionTestSettings(case=4, course='custom:run', route_config='/tmp/route.yaml')
    route = RouteContext(route_projection_valid=True)
    route.header.frame_id = 'base_footprint'
    objects = PerceptionObjectArray()
    objects.header.frame_id = 'hesai_lidar'
    tui.receive('/planning/route_context', route)
    tui.receive('/perception/objects', objects)
    assert not tui.perception_gate()[0]


@pytest.mark.parametrize('settings', [
    DecisionTestSettings(case=4, course='custom:run', route_config='/tmp/route.yaml'),
    DecisionTestSettings(case=1, course='final'),
])
def test_obstacle_zone_keeps_runtime_relay_available_after_preflight(
    tui, settings: DecisionTestSettings
) -> None:
    tui.settings = settings
    route = RouteContext()
    route.current_zone = 'MIDAN_TEST' if settings.course.startswith('custom:') else 'F_STAGE_6'
    tui.receive('/planning/route_context', route)

    assert tui.obstacle_zone_runtime_continue()


def test_non_obstacle_case_does_not_bypass_runtime_gate(tui) -> None:
    tui.settings = DecisionTestSettings(case=2, course='final')
    route = RouteContext()
    route.current_zone = 'F_STAGE_6'
    tui.receive('/planning/route_context', route)

    assert not tui.obstacle_zone_runtime_continue()


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


def test_sim_time_uses_simulation_launch_and_route_context_gate(tui) -> None:
    from kaiev26_decision.test_tui import SIMULATION_LAUNCH_CASES
    for course in ("qualifying", "final"):
        settings = DecisionTestSettings(case=1, course=course)
        assert settings.launch_case in SIMULATION_LAUNCH_CASES
        assert "platform:=simulation" in settings.launch_arguments(simulation=True)
        assert "platform:=vehicle" in settings.launch_arguments()
    tui.sim_time = True
    tui.state = 'IDLE'
    tui.request_run(DecisionTestSettings(case=2, course="qualifying"))
    assert tui.state == 'IDLE' and "시뮬레이션에서는" in tui.logs[-1]
    assert not tui.route_gate()
    tui.received['/planning/route_context'] = (10.0, RouteContext())
    assert tui.route_gate()


def running_tui(tui):
    tui.settings = DecisionTestSettings(case=1, course='qualifying')
    tui.state = 'RUNNING'
    tui.finish_since = None
    tui.launch_process = None
    tui.recorder_process = None
    tui.publish_run_active = lambda: None
    tui.publish_command = lambda *args, **kwargs: None
    tui.publish_label = lambda: None
    tui.perception_health = None
    tui.perception_health_changes = []
    tui.perception_health_logged_at = None
    tui.perception_health_unlogged = 0
    return tui


def drive_tick(tui, now, health=None):
    tui.now = lambda: now
    tui.receive('/vehicle/state', VehicleState(mode=VehicleState.MODE_AUTO, speed_mps=5.0))
    tui.receive('/planning/route_ready', Bool(data=True))
    route = RouteContext(route_projection_valid=True, distance_to_finish=200.0)
    route.current_zone = 'Q_STAGE_2'
    tui.receive('/planning/route_context', route)
    tui.receive('/decision/raw_command', ActuatorCommand())
    if health is not None:
        tui.receive('/planning/scene_summary', SceneSummary(perception_health=health))
    tui.tick()


def estop_tick(tui, now, *, auto, estop):
    tui.now = lambda: now
    tui.receive('/vehicle/state', VehicleState(
        mode=VehicleState.MODE_AUTO if auto else VehicleState.MODE_MANUAL, estop_active=estop))
    tui.receive('/planning/route_ready', Bool(data=True))
    tui.receive('/planning/route_context', RouteContext(route_projection_valid=True, distance_to_finish=200.0))
    tui.receive('/planning/scene_summary', SceneSummary(perception_health='GOOD'))
    tui.receive('/decision/raw_command', ActuatorCommand())
    tui.tick()


def test_estop_pause_does_not_block_the_run_request(tui) -> None:
    for topic in REQUIRED_EXTERNAL_TOPICS:
        tui.receive(topic, object())
    tui.receive('/vehicle/state', VehicleState(mode=VehicleState.MODE_MANUAL, estop_active=True))
    tui.receive('/steering/status', SteeringActuatorStatus(
        communication_ok=True, feedback_valid=True, estop_active=True))

    assert tui.external_issues() == []


def test_rulebook_start_waits_in_estop_pause_and_goes_on_run(tui) -> None:
    """2.17.2 사: PAUSE로 대기, AUTO 전환, RUN에 출발. AUTO에서 바로 RUNNING이고 차는 PAUSE가 잡는다."""
    running_tui(tui)
    tui.state = 'WAIT_AUTO'

    estop_tick(tui, 10.0, auto=False, estop=True)
    assert tui.state == 'WAIT_AUTO'
    estop_tick(tui, 11.0, auto=True, estop=True)
    assert tui.state == 'RUNNING'
    estop_tick(tui, 20.0, auto=True, estop=False)
    assert tui.state == 'RUNNING'
    estop_tick(tui, 30.0, auto=True, estop=True)
    estop_tick(tui, 31.0, auto=True, estop=False)
    assert tui.state == 'RUNNING'

    assert [(c['state'], c['to']) for c in tui.estop_changes] == [
        ('WAIT_AUTO', 'PAUSE'), ('RUNNING', 'RUN'), ('RUNNING', 'PAUSE'), ('RUNNING', 'RUN')]
    assert sum('E-Stop' in line for line in tui.logs) == 4


def test_released_estop_at_run_start_is_not_recorded(tui) -> None:
    running_tui(tui)
    estop_tick(tui, 10.0, auto=True, estop=False)
    assert getattr(tui, 'estop_changes', []) == []


@pytest.mark.parametrize('state, shown', [
    ('WAIT_AUTO', True), ('RUNNING', True), ('IDLE', False)])
def test_screen_shows_estop_pause_guidance(tui, state, shown) -> None:
    from io import StringIO
    from rich.console import Console
    tui.state = state
    tui.receive('/vehicle/state', VehicleState(mode=VehicleState.MODE_AUTO, estop_active=True))
    output = StringIO()
    Console(file=output, width=160, color_system=None).print(
        render(tui.snapshot(), tui.settings, None, ''))
    screen = output.getvalue()
    assert 'E-Stop PAUSE' in screen
    assert ('RUN을 주면' in screen) == shown


def test_run_warns_but_continues_when_perception_is_not_good(tui) -> None:
    running_tui(tui)
    drive_tick(tui, 10.0, 'GOOD')
    drive_tick(tui, 10.1, 'DEGRADED')
    drive_tick(tui, 10.2, 'STALE')
    drive_tick(tui, 11.5, 'GOOD')

    assert tui.state == 'RUNNING'
    assert [(item['from'], item['to']) for item in tui.perception_health_changes] == [
        ('GOOD', 'DEGRADED'), ('DEGRADED', 'STALE'), ('STALE', 'GOOD'),
    ]
    assert all(item['at'] for item in tui.perception_health_changes)
    logged = [line for line in tui.logs if '인지 상태' in line]
    assert len(logged) == 2  # the 0.1 s flicker is counted in the next line
    assert '변화 1회' in logged[-1]

    drive_tick(tui, 11.6, 'DEGRADED')
    assert any('인지 상태 DEGRADED' in item for item in tui.runtime_warnings())


# Input loss while running (decision D2): the Run never ends because of it.
# Steps are (time s, inputs, measured speed, raw brake) -> (relayed speed, brake).
# inputs: "ok" all fresh, "no_scene" scene missing, "no_raw" raw command missing,
# "degraded" scene DEGRADED (a warning only, D3).
RELAY = 4.0
INPUT_LOSS_CASES = {
    "scene_loss_brakes_after_1s_until_stopped": [
        (10.0, "ok", 5.0, False, (RELAY, False)),
        (10.6, "no_scene", 5.0, False, (0.0, False)),
        (11.2, "no_scene", 4.5, False, (0.0, False)),
        (11.6, "no_scene", 4.0, False, (0.0, True)),
        (11.7, "ok", 3.0, False, (0.0, True)),
        (12.5, "ok", 0.0, False, (RELAY, False)),
    ],
    "decision_loss_brakes_after_1s_until_stopped": [
        (10.0, "ok", 5.0, False, (RELAY, False)),
        (10.6, "no_raw", 5.0, False, (0.0, False)),
        (11.6, "no_raw", 4.0, False, (0.0, True)),
        (11.7, "ok", 3.0, False, (0.0, True)),
        (12.5, "ok", 0.0, False, (RELAY, False)),
    ],
    "short_raw_gap_relays_again": [
        (10.0, "ok", 5.0, False, (RELAY, False)),
        (10.3, "no_raw", 5.0, False, (0.0, False)),
        (10.7, "ok", 4.8, False, (RELAY, False)),
    ],
    "engaged_brake_stays_through_the_loss": [
        (10.0, "ok", 2.0, True, (RELAY, True)),
        (10.3, "no_raw", 1.0, True, (0.0, True)),
        (10.8, "no_raw", 0.5, True, (0.0, True)),
    ],
    "not_good_perception_only_warns": [
        (10.0, "ok", 5.0, False, (RELAY, False)),
        (10.1, "degraded", 5.0, False, (RELAY, False)),
        (11.1, "degraded", 4.0, False, (RELAY, False)),
        (12.0, "ok", 4.0, False, (RELAY, False)),
    ],
}


@pytest.mark.parametrize("case", sorted(INPUT_LOSS_CASES))
def test_input_loss_never_ends_the_run(tui, case) -> None:
    running_tui(tui)
    published = []
    tui.publish_command = DecisionTestNode.publish_command.__get__(tui)
    tui.command_pub = type('Pub', (), {'publish': lambda _self, msg: published.append(msg)})()
    tui.get_clock = lambda: type('Clock', (), {'now': lambda _self: type(
        'Now', (), {'to_msg': lambda _self: ActuatorCommand().header.stamp})()})()
    tui.input_loss = None  # track_input_loss builds a fresh latch
    tui.input_loss_active = False
    tui.input_loss_state = None
    for now, inputs, speed, raw_brake, expected in INPUT_LOSS_CASES[case]:
        tui.now = lambda: now
        tui.receive('/vehicle/state', VehicleState(mode=VehicleState.MODE_AUTO, speed_mps=speed))
        tui.receive('/planning/route_ready', Bool(data=True))
        route = RouteContext(route_projection_valid=True, distance_to_finish=200.0)
        route.current_zone = 'Q_STAGE_2'
        tui.receive('/planning/route_context', route)
        if inputs != "no_raw":
            tui.receive('/decision/raw_command',
                        ActuatorCommand(speed_target_mps=RELAY, brake_engage=raw_brake))
        if inputs != "no_scene":
            health = 'DEGRADED' if inputs == "degraded" else 'GOOD'
            tui.receive('/planning/scene_summary', SceneSummary(perception_health=health))
        tui.tick()
        assert tui.state == 'RUNNING', now
        assert (published[-1].speed_target_mps, published[-1].brake_engage) == expected, now


def test_recorder_death_only_warns(tui) -> None:
    running_tui(tui)
    tui.recorder_failure_reported = False
    tui.recorder_process = type('Proc', (), {'process': type('P', (), {'poll': lambda _self: 1})()})()
    drive_tick(tui, 10.0, 'GOOD')
    drive_tick(tui, 10.1, 'GOOD')

    assert tui.state == 'RUNNING'
    assert sum('MCAP recorder 종료' in line for line in tui.logs) == 1
    assert any('MCAP recorder 종료' in item for item in tui.runtime_warnings())


def test_vehicle_run_warns_but_continues_when_readiness_drops(tui) -> None:
    running_tui(tui)
    tui.route_readiness_ok = True
    drive_tick(tui, 10.0, 'GOOD')
    original = tui.receive

    def readiness_lost(topic, message):
        if topic == '/planning/route_ready':
            message = Bool(data=False)
        original(topic, message)

    tui.receive = readiness_lost
    drive_tick(tui, 10.1, 'GOOD')
    drive_tick(tui, 10.2, 'GOOD')

    assert tui.state == 'RUNNING'
    assert sum('출발 점검 경고' in line for line in tui.logs) == 1
    assert any('출발 점검 항목 이상' in item for item in tui.runtime_warnings())

    tui.receive = original
    drive_tick(tui, 10.3, 'GOOD')
    assert tui.state == 'RUNNING'
    assert '출발 점검 정상 복귀' in tui.logs[-1]


@pytest.mark.parametrize('state, brake', [
    ('PREFLIGHT', False), ('WAIT_AUTO', False),
    ('STOPPING', True), ('COMPLETE', True), ('ERROR', True),
])
def test_tui_brakes_in_auto_only_after_the_run(tui, state, brake) -> None:
    published = []
    tui.command_pub = type('Pub', (), {'publish': lambda _self, msg: published.append(msg)})()
    tui.get_clock = lambda: type('Clock', (), {'now': lambda _self: type(
        'Now', (), {'to_msg': lambda _self: ActuatorCommand().header.stamp})()})()
    tui.state = state
    tui.receive('/vehicle/state', VehicleState(mode=VehicleState.MODE_AUTO))

    tui.publish_command(False, True)
    assert published[-1].brake_engage is brake
    assert published[-1].speed_target_mps == 0.0

    tui.publish_command(False, False)
    assert published[-1].brake_engage is False


def _closed_obstacle_run(tui, tmp_path, case=4):
    import yaml
    tui.settings = DecisionTestSettings(case=case, course='custom:run', route_config='/tmp/route.yaml')
    tui.run_dir = str(tmp_path)
    tui.launch_process = None
    tui.recorder_process = None
    tui.requested_terminal = 'COMPLETE'
    (tmp_path / 'rosbag').mkdir()
    (tmp_path / 'rosbag' / 'run_0.mcap').write_bytes(b'mcap')
    (tmp_path / 'scenario.yaml').write_text(yaml.safe_dump({'name': 'run'}))
    tui.close_processes('경로 종료점 정지 완료')
    return yaml


def test_obstacle_case_asks_before_keeping_mcap(tui, tmp_path):
    yaml = _closed_obstacle_run(tui, tmp_path)
    assert tui.state == 'SAVE_PROMPT'
    tui.request_stop('again')
    assert tui.state == 'SAVE_PROMPT'
    tui.resolve_mcap_save(True)
    assert tui.state == 'COMPLETE'
    assert (tmp_path / 'rosbag' / 'run_0.mcap').is_file()
    assert yaml.safe_load((tmp_path / 'scenario.yaml').read_text())['mcap_kept'] is True


def test_obstacle_case_discard_removes_only_the_bag(tui, tmp_path):
    yaml = _closed_obstacle_run(tui, tmp_path)
    tui.resolve_mcap_save(False)
    assert tui.state == 'COMPLETE'
    assert not (tmp_path / 'rosbag').exists()
    document = yaml.safe_load((tmp_path / 'scenario.yaml').read_text())
    assert document['mcap_kept'] is False
    assert (tmp_path / 'console.log').is_file()


@pytest.mark.parametrize('case', [2])
def test_other_cases_keep_mcap_without_prompt(tui, tmp_path, case):
    _closed_obstacle_run(tui, tmp_path, case=case)
    assert tui.state == 'COMPLETE'
    assert (tmp_path / 'rosbag').is_dir()


def test_mission_case_keeps_mcap_without_prompt(tui, tmp_path):
    import yaml
    tui.settings = DecisionTestSettings(case=1, course='final')
    tui.run_dir = str(tmp_path)
    tui.launch_process = None
    tui.recorder_process = None
    tui.requested_terminal = 'COMPLETE'
    (tmp_path / 'rosbag').mkdir()
    (tmp_path / 'scenario.yaml').write_text(yaml.safe_dump({'name': 'run'}))
    tui.close_processes('경로 종료점 정지 완료')
    assert tui.state == 'COMPLETE'
    assert (tmp_path / 'rosbag').is_dir()


@pytest.mark.parametrize('case,course', [(1, 'qualifying'), (1, 'final'), (2, 'qualifying')])
def test_driving_runs_record_the_steering_torque_and_control_parameters(case, course):
    topics = DecisionTestSettings(case=case, course=course).record_topics
    for topic in ('/steering/status', '/steering/requested_angle', '/steering/angle_error',
                  '/steering/torque_command_raw', '/steering/torque_saturated',
                  '/sysid/control_parameters', '/vehicle/state', '/imu/data', '/gnss/fix',
                  '/gnss/status_verbose', '/planning/command', '/planning/route_context',
                  '/diagnostics'):
        assert topic in topics
    assert len(topics) == len(set(topics))
