from collections import deque
from io import StringIO
import sys
import threading
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from rich.console import Console
from kaiev26_msgs.msg import ActuatorCommand, BehaviorDecision, SceneSummary, VehicleState

from kaiev26_decision import operating_tui, test_tui
from kaiev26_decision.test_tui import (
    DecisionTestNode, SettingsEditor, TestSettings as Settings, render,
)


@pytest.fixture
def operator():
    node = DecisionTestNode.__new__(DecisionTestNode)
    node.operating_mode = True
    node.lock = threading.RLock()
    node.received = {}
    node.settings = Settings()
    node.state = 'IDLE'
    node.graph_issues = []
    node.logs = deque()
    node.run_dir = ''
    node.finish_reason = ''
    node.launch_process = None
    node.recorder_process = None
    node.now = lambda: 10.
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time))
    return node


def test_operator_has_only_course_and_speed_settings():
    editor = SettingsEditor(operating_mode=True)
    editor.custom_routes = {'custom:ignored': ('ignored', '/tmp/ignored.yaml')}
    for key in '1234nh':
        assert editor.handle_key(key, 'IDLE')
        assert editor.selection == Settings()
        assert editor.edit_field is None
    for key in 'v6.5\n':
        assert editor.handle_key(key, 'IDLE')
    assert editor.selection.effective_speed_mps == 6.5
    assert editor.handle_key('p', 'IDLE')
    assert editor.selection == Settings(course='final')
    assert editor.selection.effective_speed_mps == 10.
    assert editor.handle_key('p', 'IDLE')
    assert editor.selection == Settings(course='qualifying')
    assert editor.selection.effective_speed_mps == 7.
    assert not editor.handle_key('r', 'IDLE')
    assert not editor.handle_key('s', 'RUNNING')
    assert not editor.handle_key('q', 'RUNNING')


def test_operator_selects_the_start_mode_and_keeps_it_across_courses():
    editor = SettingsEditor(operating_mode=True)
    assert editor.selection.start_mode == 'from_start'
    assert editor.handle_key('b', 'IDLE')
    assert editor.selection.start_mode == 'resume'
    assert editor.handle_key('p', 'IDLE')
    assert editor.selection == Settings(course='final', start_mode='resume')
    assert 'start_mode:=resume' in editor.selection.launch_arguments()
    assert editor.handle_key('b', 'RUNNING')
    assert editor.selection.start_mode == 'resume'


@pytest.mark.parametrize('state', ['STARTING', 'PREFLIGHT', 'WAIT_AUTO', 'RUNNING', 'STOPPING'])
def test_operator_cannot_change_course_or_speed_during_run(state):
    editor = SettingsEditor(operating_mode=True)
    for key in 'pv':
        assert editor.handle_key(key, state)
        assert editor.selection == Settings()
        assert editor.edit_field is None


@pytest.mark.parametrize('course,launch_case,speed', [('qualifying', 3, 7.), ('final', 7, 10.)])
def test_operator_starts_same_mission_backend_without_recorder_or_files(
    operator, tmp_path, monkeypatch, course, launch_case, speed,
):
    operator.settings = Settings(course=course)
    children = []

    def child(name, argv, log):
        value = SimpleNamespace(name=name, argv=argv, stop=lambda: 0)
        children.append(value)
        return value

    def forbidden(*args, **kwargs):
        pytest.fail('Operating mode must not create a recorder or records directory')

    monkeypatch.setattr(test_tui, 'OwnedProcess', child)
    monkeypatch.setattr(test_tui, 'workspace_root', forbidden)
    monkeypatch.setattr(operator, 'start_recording', forbidden)
    operator.start_processes()
    assert len(children) == 1 and children[0].name == 'Decision'
    assert children[0].argv == [
        'ros2', 'launch', 'kaiev26_decision', 'decision_test.launch.py',
        f'case:={launch_case}', f'speed_mps:={speed}', 'platform:=vehicle',
        'command_topic:=/decision/raw_command', 'start_mode:=from_start',
    ]
    assert operator.recorder_process is None and operator.run_dir == ''
    operator.requested_terminal = 'COMPLETE'
    operator.close_processes('finish')
    assert operator.state == 'COMPLETE' and operator.launch_process is None
    assert not list(tmp_path.iterdir())
    assert not any('MCAP' in line for line in operator.logs)


@pytest.mark.parametrize('settings', [Settings(case=2), Settings(case=3),
    Settings(case=4, course='custom:route', route_config='/tmp/route.yaml')])
def test_operator_rejects_non_mission_settings(operator, settings):
    operator.request_run(settings)
    assert operator.state == 'IDLE'
    assert operator.launch_process is None and operator.recorder_process is None


def test_operator_keeps_preflight_auto_relay_and_manual_stop(operator):
    emitted = []
    operator.command_pub = SimpleNamespace(publish=emitted.append)
    operator.publish_run_active = lambda: None
    operator.publish_label = lambda: None
    operator.route_gate = lambda: True
    operator.perception_gate = lambda: (True, 'ready')
    operator.ready_since = None
    operator.state = 'PREFLIGHT'
    operator.input_loss = None  # track_input_loss builds a fresh latch
    operator.input_loss_active = False
    operator.input_loss_state = None

    def tick(now, mode):
        operator.now = lambda: now
        operator.receive('/vehicle/state', VehicleState(mode=mode, speed_mps=0.))
        operator.receive('/decision/raw_command', ActuatorCommand(
            speed_target_mps=7., steering_target_rad=.1))
        operator.receive('/planning/scene_summary', SceneSummary(perception_health='GOOD'))
        operator.tick()

    tick(10., VehicleState.MODE_MANUAL)
    assert operator.state == 'PREFLIGHT' and emitted[-1].speed_target_mps == 0.
    tick(10.6, VehicleState.MODE_MANUAL)
    assert operator.state == 'WAIT_AUTO' and emitted[-1].speed_target_mps == 0.
    tick(10.7, VehicleState.MODE_AUTO)
    assert operator.state == 'RUNNING'
    assert emitted[-1].speed_target_mps == 7. and emitted[-1].steering_target_rad == .1
    tick(13.9, VehicleState.MODE_MANUAL)
    assert operator.state == 'STOPPING'
    assert emitted[-1].speed_target_mps == 0. and not emitted[-1].brake_engage


def test_operator_finishes_on_finish_held_without_recording(operator):
    operator.state = 'RUNNING'
    operator.publish_run_active = lambda: None
    operator.publish_command = lambda *args, **kwargs: None
    operator.publish_label = lambda: None
    operator.input_loss = None  # track_input_loss builds a fresh latch
    operator.input_loss_active = False
    operator.input_loss_state = None
    operator.route_gate = lambda: True
    operator.receive('/vehicle/state', VehicleState(mode=VehicleState.MODE_AUTO, speed_mps=0.))
    operator.receive('/decision/raw_command', ActuatorCommand(brake_engage=True))
    operator.receive('/planning/scene_summary', SceneSummary(perception_health='GOOD'))
    operator.receive('/planning/behavior_decision', BehaviorDecision(constraints=['FINISH_HELD']))
    operator.tick()
    assert operator.state == 'STOPPING' and operator.requested_terminal == 'COMPLETE'
    operator.close_processes('finish')
    assert operator.state == 'COMPLETE' and not operator.run_dir


@pytest.mark.parametrize('width', [80, 120])
@pytest.mark.parametrize('course', ['qualifying', 'final'])
def test_operator_reuses_status_panels_without_case_or_recording_controls(operator, width, course):
    operator.settings = Settings(course=course)
    output = StringIO()
    Console(file=output, width=width, color_system=None).print(render(
        operator.snapshot(), operator.settings, None, '',
        operating_mode=True,
    ))
    screen = output.getvalue()
    for label in ('Decision Operation', 'GNSS / RTK', '출발 조건', '목표속도', '출발: 처음부터', 'Runtime'):
        assert label in screen
    for removed in ('MCAP', 'record:', 'records', 'Case Select', 'Stop and save',
                    '수동주행 경로', '장애물 회피', '현장 메모'):
        assert removed not in screen
    assert ('7 m/s' if course == 'qualifying' else '10 m/s') in screen


def test_terminal_binding_restores_streams_on_failure(monkeypatch):
    previous = sys.stdin, sys.stdout, sys.stderr
    streams = []
    descriptors = []

    class Terminal(StringIO):
        def fileno(self):
            return 10

    def terminal(*args, **kwargs):
        value = Terminal()
        streams.append(value)
        return value

    monkeypatch.setattr('builtins.open', terminal)
    monkeypatch.setattr(operating_tui.os, 'dup', lambda fd: 42)
    monkeypatch.setattr(operating_tui.os, 'dup2', lambda source, target: descriptors.append((source, target)))
    monkeypatch.setattr(operating_tui.os, 'close', lambda fd: descriptors.append(('close', fd)))
    with pytest.raises(RuntimeError):
        with operating_tui.controlling_terminal():
            assert sys.stdin is streams[0]
            assert sys.stdout is sys.stderr is streams[1]
            raise RuntimeError('test')
    assert (sys.stdin, sys.stdout, sys.stderr) == previous
    assert all(stream.closed for stream in streams)
    assert descriptors == [(10, 0), (42, 0), ('close', 42)]


def test_noninteractive_launch_does_not_create_ros_node(monkeypatch):
    def no_terminal(*args, **kwargs):
        raise OSError('no controlling terminal')

    def forbidden(**kwargs):
        pytest.fail('ROS nodes must not start without a terminal')

    monkeypatch.setattr('builtins.open', no_terminal)
    monkeypatch.setattr(operating_tui, 'run_tui', forbidden)
    assert operating_tui.main() == 2
