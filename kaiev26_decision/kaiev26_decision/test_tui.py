"""Field Decision test selector, safety gate, and one-run MCAP recorder."""

from __future__ import annotations

__test__ = False

from collections import deque
from contextlib import contextmanager
from dataclasses import asdict, dataclass, field, replace
from datetime import datetime
import fcntl
import codecs
import hashlib
import json
import math
import os
from pathlib import Path
import select
import shutil
import signal
import subprocess
import sys
import termios
import threading
import time
import tty
from zoneinfo import ZoneInfo

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import TwistWithCovarianceStamped
from kaiev26_motion_control.input_loss import InputLossLatch
from kaiev26_msgs.msg import (
    ActuatorCommand,
    BehaviorDecision,
    BrakeActuatorStatus,
    DriveActuatorStatus,
    PerceptionObjectArray,
    RouteContext,
    SceneSummary,
    SteeringActuatorStatus,
    VehicleState,
)
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import Parameter, parameter_value_to_python
from rich.console import Console
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32, String
from um980_msgs.msg import Status as GnssStatus
import yaml

from kaiev26_decision.route_capture import (
    LOCALIZATION_PARAMETERS, capture_from_bag, discover_routes,
    load_custom_route, origin_from_parameters, route_directory, stamp_ns,
    verify_origin, workspace_root, recording_quality,
)

CASE_NAMES = {
    1: "미션 실행",
    2: "전역경로 추종",
    3: "수동주행 경로 수집·등록",
    4: "장애물 회피",
}
COURSE_NAMES = {
    "qualifying": "예선",
    "final": "본선",
}
COURSE_DEFAULT_SPEED_MPS = {
    "qualifying": 7.0,
    "final": 10.0,
}
# 수집 경로(custom:) 기본 목표속도.
CUSTOM_DEFAULT_SPEED_MPS = 10.0
# 출발 모드: decision_test.launch.py start_mode. 처음부터는 출발점 구간에서 위치만으로 진행도를 맞춘다.
START_MODE_NAMES = {
    "from_start": "처음부터",
    "resume": "이어서",
}
MISSION_CASE = 1
TRACKING_CASE = 2
ROUTE_CAPTURE_CASE = 3
OBSTACLE_CASE = 4
COMPETITION_CASE = MISSION_CASE
# 인지 장면 허용 수신 간격. 넘으면 주행 중 입력 끊김으로 본다.
SCENE_RECEIVE_TIMEOUT_SEC = 0.5
# 판단 명령 허용 수신 간격.
RAW_COMMAND_TIMEOUT_SEC = 0.25
# 입력이 끊기면 곧바로 속도 0(제동 없음), 이 시간 안에 돌아오지 않으면 제동하고 실제 정지까지 유지한다.
INPUT_LOSS_BRAKE_DELAY_SEC = 1.0
INPUT_LOSS_STOPPED_SPEED_MPS = 0.1
# 판단의 종료 정지(종료선을 지나 정지 유지) 표시. 이 상태를 받으면 Run을 COMPLETE로 끝낸다.
FINISH_HELD_CONSTRAINT = "FINISH_HELD"
BEHAVIOR_DECISION_TIMEOUT_SEC = 0.5
# 경기 모드의 인지 상태 변화 로그 최소 간격. 변화 시각은 scenario.yaml에 모두 남긴다.
PERCEPTION_HEALTH_LOG_INTERVAL_SEC = 1.0
# 출발 전 상태. 이때 TUI는 브레이크를 걸지 않는다.
PRE_RUN_STATES = frozenset({"IDLE", "STARTING", "PREFLIGHT", "WAIT_AUTO"})
# decision_test.launch.py의 platform:=simulation이 받는 launch case와 같다.
SIMULATION_LAUNCH_CASES = frozenset({3, 6, 7})
EDITABLE_STATES = {"IDLE", "COMPLETE", "ABORTED", "ERROR"}
# 규정 2.17.2 사: E-Stop PAUSE(AS-Emergency)로 대기하다 RUN으로 출발한다. 제동, 구동 차단,
# 조향 disable은 STM_B와 조향 노드가 하고 TUI 상태는 바꾸지 않는다. 화면 안내만 띄운다.
ESTOP_PAUSE_HINTS = {
    "PREFLIGHT": "E-Stop PAUSE: 출발 점검은 그대로 진행",
    "WAIT_AUTO": "E-Stop PAUSE 대기: AUTO로 올린 뒤 RUN을 주면 출발",
    "RUNNING": "E-Stop PAUSE: 차량 정지, RUN을 주면 이어서 주행",
}
REQUIRED_EXTERNAL_TOPICS = (
    "/gnss/fix",
    "/gnss/fix_velocity",
    "/localization/odometry",
    "/vehicle/state",
    "/drive/status",
    "/steering/status",
    "/brake/status",
)
READINESS_LABELS = {
    '/gnss/fix': 'GNSS 위치',
    '/gnss/fix_velocity': 'GNSS 헤딩',
    '/localization/odometry': '자차 위치',
    '/vehicle/state': '차량 상태',
    '/drive/status': '구동기',
    '/steering/status': '조향',
    '/brake/status': '브레이크',
    '/planning/route_context': '경로 정합',
    '/debug/route_context_age': '판단 경로 입력',
    '/debug/vehicle_state_age': '판단 차량 입력',
}
RECORD_TOPICS = tuple(
    dict.fromkeys(
        (
            "/planning/command",
            "/decision/raw_command",
            "/vehicle/command",
            "/decision/scenario",
            "/drive/status",
            "/steering/status",
            "/steering/angle",
            "/brake/status",
            "/vehicle/state",
            "/vehicle/wheel_twist",
            "/gnss/fix",
            "/gnss/fix_velocity",
            "/gnss/status",
            "/gnss/status_verbose",
            "/imu/data",
            "/imu/data_raw",
            "/ebimu/status",
            "/localization/odometry",
            "/localization/odometry_gnss",
            "/planning/route_ready",
            "/planning/readiness_status",
            "/planning/route_context",
            "/planning/scene_summary",
            "/planning/events",
            "/planning/traffic_signal_phases",
            "/planning/traffic_signal_phases/evidence",
            "/planning/behavior_decision",
            "/planning/target_path",
            "/planning/target_speed",
            "/perception/road_segments",
            "/perception/centerline",
            "/perception/objects",
            "/perception/traffic_lights",
            "/perception/camera/left/source/image_raw/compressed",
            "/perception/camera/right/source/image_raw/compressed",
            "/perception/camera/left/source/camera_info",
            "/perception/camera/right/source/camera_info",
            "/debug/active_behavior",
            "/debug/fsm_state",
            "/debug/planning_latency",
            "/debug/route_context_age",
            "/debug/vehicle_state_age",
            "/debug/perception_status",
            "/debug/global_route_markers",
            "/debug/route_state_markers",
            "/debug/autonomous_markers",
            "/tf",
            "/tf_static",
            "/diagnostics",
            "/clock",
            "/parameter_events",
        )
    )
)
FIELD_EXTRA_TOPICS = (
    '/decision/test_run_active', '/drive/longitudinal_status',
    '/drive/longitudinal_config', '/sysid/control_parameters',
    '/stm_a/runtime_stats', '/stm_b/runtime_stats', '/drive/can_rx_diagnostics', '/rosout',
    '/steering/actuation_mode', '/steering/requested_angle', '/steering/angle_error',
    '/steering/rate', '/steering/torque_command_raw', '/steering/torque_current_limit_command_a',
    '/steering/torque_saturated', '/steering/torque_peak_limited', '/steering/torque_safety',
)
OBSTACLE_RECORD_TOPICS = (
    '/ouster/points', '/perception/occupancy_grid',
    '/pv26_perception/left/overlay/compressed',
    '/pv26_perception/right/overlay/compressed',
    '/pv26_perception/traffic_light_detections', '/pv26_perception/markers',
    '/pv26_perception/map_alignment',
)


def supported_cases(course: str) -> tuple[int, ...]:
    if course.startswith('custom:'):
        return (TRACKING_CASE, ROUTE_CAPTURE_CASE, OBSTACLE_CASE)
    return (MISSION_CASE, TRACKING_CASE, ROUTE_CAPTURE_CASE)


def field_record_topics(case: int) -> tuple[str, ...]:
    topics = [t for t in RECORD_TOPICS
              if case == OBSTACLE_CASE or not t.startswith(('/perception/', '/debug/'))]
    topics.extend(FIELD_EXTRA_TOPICS)
    if case == OBSTACLE_CASE:
        topics.extend(OBSTACLE_RECORD_TOPICS)
    return tuple(dict.fromkeys(topics))
RECORDER_CODE = """
import fcntl, json, os, sys
import rosbag2_py
lock_path = f'/tmp/kaiev26_mcap_{os.environ.get("ROS_DOMAIN_ID", "0")}.lock'
lock_file = open(lock_path, 'w')
try:
    fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError:
    raise RuntimeError(f'another MCAP recorder holds {lock_path}')
storage = rosbag2_py.StorageOptions(
    uri=sys.argv[1], storage_id='mcap', storage_preset_profile='zstd_fast')
options = rosbag2_py.RecordOptions()
options.topics = json.loads(sys.argv[2])
options.rmw_serialization_format = 'cdr'
try:
    rosbag2_py.Recorder().record(storage, options)
except KeyboardInterrupt:
    pass
"""


@dataclass(frozen=True)
class TestSettings:
    case: int = MISSION_CASE
    course: str = "qualifying"
    speed_mps: float = 0.0
    route_name: str = ''
    route_config: str = ''
    note: str = ''
    start_mode: str = 'from_start'

    def __post_init__(self) -> None:
        if self.case not in CASE_NAMES:
            raise ValueError("case는 1부터 4까지입니다")
        if self.course not in COURSE_NAMES and not self.course.startswith('custom:'):
            raise ValueError("지원하지 않는 코스입니다")
        if self.case not in supported_cases(self.course):
            raise ValueError("선택 코스에서 지원하지 않는 Case입니다")
        if self.start_mode not in START_MODE_NAMES:
            raise ValueError("출발 모드는 처음부터 또는 이어서입니다")
        if self.course.startswith('custom:') and not Path(self.route_config).is_absolute():
            raise ValueError('수집 경로 YAML 절대경로가 필요합니다')
        if not math.isfinite(self.speed_mps) or not (
            self.speed_mps == 0.0 or 0.1 <= self.speed_mps <= 15.0
        ):
            raise ValueError("속도는 코스 기본값 0 또는 0.1~15.0 m/s여야 합니다")

    @property
    def mode(self) -> str:
        return {MISSION_CASE: 'all', TRACKING_CASE: 'tracking',
                OBSTACLE_CASE: 'avoidance', ROUTE_CAPTURE_CASE: 'route_capture'}[self.case]

    @property
    def launch_case(self) -> int | None:
        if self.case == ROUTE_CAPTURE_CASE:
            return None
        if self.course.startswith('custom:'):
            return {TRACKING_CASE: 13, OBSTACLE_CASE: 17}[self.case]
        if self.case == TRACKING_CASE:
            return 1 if self.course == "qualifying" else 4
        return 3 if self.course == "qualifying" else 7

    @property
    def requires_perception(self) -> bool:
        return self.case in {MISSION_CASE, OBSTACLE_CASE}

    @property
    def obstacle_enabled(self) -> bool:
        return self.case == OBSTACLE_CASE or (
            self.course == 'final' and self.case == MISSION_CASE)

    @property
    def effective_speed_mps(self) -> float:
        if self.speed_mps > 0.0:
            return self.speed_mps
        if self.course.startswith('custom:'):
            return CUSTOM_DEFAULT_SPEED_MPS
        return COURSE_DEFAULT_SPEED_MPS[self.course]

    @property
    def record_topics(self) -> tuple[str, ...]:
        if self.case in {OBSTACLE_CASE, ROUTE_CAPTURE_CASE}:
            return field_record_topics(self.case)
        # Steering torque, requested angle and the Control parameters, for the actuator analysis.
        topics = tuple(dict.fromkeys((*RECORD_TOPICS, *FIELD_EXTRA_TOPICS)))
        if self.obstacle_enabled:
            return tuple(dict.fromkeys((*topics, *OBSTACLE_RECORD_TOPICS)))
        return topics

    def launch_arguments(self, *, simulation: bool = False) -> list[str]:
        if self.case == ROUTE_CAPTURE_CASE:
            return []
        arguments = [
            f"case:={self.launch_case}",
            f"speed_mps:={self.effective_speed_mps}",
            "platform:=simulation" if simulation else "platform:=vehicle",
            "command_topic:=/decision/raw_command",
            f"start_mode:={self.start_mode}",
        ]
        if self.course.startswith('custom:'):
            arguments.append(f'route_config:={self.route_config}')
        return arguments

    def document(self) -> dict:
        value = asdict(self)
        value['case_schema_version'] = 2
        value["requested_speed_mps"] = value.pop("speed_mps")
        value.update(
            {
                "name": CASE_NAMES[self.case],
                "course": self.course,
                "mode": self.mode,
                "launch_case": self.launch_case,
                "speed_mps": self.effective_speed_mps,
                "speed_source": (
                    "launch_argument" if self.speed_mps > 0.0 else "course_default"
                ),
                "speed_control": (
                    "tracking_target" if self.case == TRACKING_CASE
                    else "manual" if self.case == ROUTE_CAPTURE_CASE else "decision_policy"
                ),
                "lateral_controller": None if self.case == ROUTE_CAPTURE_CASE else "pp_stanley",
                'route_used': self.case != ROUTE_CAPTURE_CASE,
                "record_topics": list(self.record_topics),
            }
        )
        return value


def create_recording_directory(
    records_root: Path, settings: TestSettings, started_at: datetime
) -> Path:
    route = settings.course
    if settings.case == ROUTE_CAPTURE_CASE or route.startswith('custom:'):
        route = settings.route_name.strip() or route.removeprefix('custom:')
    route = ''.join(char if char.isalnum() or char in '-_.' else '_' for char in route)
    route = route.strip('._') or 'route'
    speed = 0.0 if settings.case == ROUTE_CAPTURE_CASE else settings.effective_speed_mps
    speed_text = f'{speed:g}'.replace('.', 'p')
    name = f'{started_at:%y%m%d_%H%M}_decision_case{settings.case:02d}_{route}_V{speed_text}'
    attempt = 1
    while True:
        suffix = '' if attempt == 1 else f'_{attempt:02d}'
        directory = records_root / (name + suffix)
        try:
            directory.mkdir(parents=True, exist_ok=False)
        except FileExistsError:
            attempt += 1
        else:
            return directory


@dataclass
class SettingsEditor:
    """Keyboard-only settings state that cannot alter an active run."""

    selection: TestSettings = field(default_factory=TestSettings)
    edit_field: str | None = None
    buffer: str = ""
    message: str = ""
    custom_routes: dict = field(default_factory=dict)
    operating_mode: bool = False

    def handle_key(self, char: str, state: str) -> bool:
        if not char:
            return False
        key = char.lower()
        if self.edit_field is not None:
            if key in {"q", "s", " "} and self.edit_field not in {'route_name', 'note'}:
                self.edit_field = None
                self.buffer = ""
                self.message = ""
                return False
            if state not in EDITABLE_STATES:
                self.edit_field = None
                self.buffer = ""
                self.message = "실행 중에는 설정을 변경할 수 없습니다. Stop 후 변경하세요."
                return True
            if char in "\r\n":
                try:
                    value = (
                        self.buffer
                        if self.edit_field in {'route_name', 'note'}
                        else float(self.buffer)
                    )
                    self.selection = replace(
                        self.selection, **{self.edit_field: value}
                    )
                except ValueError as error:
                    self.message = str(error)
                else:
                    self.message = "설정 저장: 다음 Run부터 적용됩니다."
                    self.edit_field = None
                    self.buffer = ""
            elif char == "\x1b":
                self.edit_field = None
                self.buffer = ""
                self.message = "입력을 취소했습니다. 기존 설정을 유지합니다."
            elif char in ("\x7f", "\b"):
                self.buffer = self.buffer[:-1]
            elif self.edit_field in {'route_name', 'note'} and char.isprintable():
                self.buffer += char
            elif char in "0123456789.eE+-":
                self.buffer += char
            return True

        configurable = {"p", "v", "b"} if self.operating_mode else set('1234pvnhb')
        if self.operating_mode and key in '1234nh':
            self.message = "운용 모드에서는 코스·속도·출발만 설정합니다."
            return True
        if key not in configurable:
            return False
        if state not in EDITABLE_STATES:
            self.message = "실행 중에는 설정을 변경할 수 없습니다. Stop 후 변경하세요."
            return True

        self.message = ""
        if key in "1234":
            if int(key) not in supported_cases(self.selection.course):
                self.message = "선택 경로에서 지원하지 않습니다. P로 다른 경로를 선택하세요."
                return True
            self.selection = replace(self.selection, case=int(key))
        elif key == "p":
            if self.operating_mode:
                course = 'final' if self.selection.course == 'qualifying' else 'qualifying'
                self.selection = TestSettings(
                    case=MISSION_CASE, course=course, start_mode=self.selection.start_mode)
                return True
            courses = (*COURSE_NAMES, *self.custom_routes)
            if self.selection.course not in courses:
                self.selection = replace(self.selection, course='qualifying', case=MISSION_CASE)
            course = courses[(courses.index(self.selection.course) + 1) % len(courses)]
            case = self.selection.case if self.selection.case in supported_cases(course) else TRACKING_CASE
            name, path = self.custom_routes.get(course, ('', ''))
            self.selection = replace(self.selection, course=course, case=case,
                                     route_config=path, route_name=name)
        elif key == 'b':
            if self.selection.case == ROUTE_CAPTURE_CASE:
                return False
            start_mode = 'resume' if self.selection.start_mode == 'from_start' else 'from_start'
            self.selection = replace(self.selection, start_mode=start_mode)
        elif key == 'n' and self.selection.case != ROUTE_CAPTURE_CASE:
            return False
        elif key == 'v' and self.selection.case == ROUTE_CAPTURE_CASE:
            return False
        elif key == 'h' and self.selection.case not in {OBSTACLE_CASE, ROUTE_CAPTURE_CASE}:
            return False
        else:
            self.edit_field = {
                "v": 'speed_mps', 'n': 'route_name', 'h': 'note',
            }[key]
            self.buffer = ""
        return True


def process_group_members(pgid: int) -> list[int]:
    members = []
    for path in Path("/proc").glob("[0-9]*/stat"):
        try:
            fields = path.read_text().rsplit(")", 1)[1].split()
            if int(fields[2]) == pgid and fields[0] != "Z":
                members.append(int(path.parent.name))
        except (OSError, ValueError, IndexError):
            continue
    return members


class OwnedProcess:
    def __init__(self, name: str, argv: list[str], log) -> None:
        self.name = name
        self.log = log
        self.process = subprocess.Popen(
            argv,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            start_new_session=True,
            bufsize=1,
        )
        self.pgid = self.process.pid
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self) -> None:
        assert self.process.stdout is not None
        for line in self.process.stdout:
            self.log(f"{self.name}: {line.rstrip()}")
        self.process.stdout.close()

    def stop(self) -> int:
        if process_group_members(self.pgid):
            try:
                self.process.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass
            deadline = time.monotonic() + 7.0
            while time.monotonic() < deadline:
                if not process_group_members(self.pgid):
                    break
                time.sleep(0.05)
        for sig, timeout in (
            (signal.SIGTERM, 2.0),
            (signal.SIGKILL, 1.0),
        ):
            if not process_group_members(self.pgid):
                break
            try:
                os.killpg(self.pgid, sig)
            except ProcessLookupError:
                break
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if not process_group_members(self.pgid):
                    break
                time.sleep(0.05)
        self.process.wait(timeout=2.0)
        self.reader.join(timeout=1.0)
        return int(self.process.returncode or 0)


class DecisionTestNode(Node):
    sim_time = False
    operating_mode = False

    def __init__(
        self,
        initial_settings: TestSettings | None = None,
        sim_time: bool = False,
        operating_mode: bool = False,
    ) -> None:
        super().__init__(
            "decision_test",
            parameter_overrides=[Parameter("use_sim_time", value=sim_time)],
        )
        # Gazebo에서는 route_readiness 없이 시뮬레이션 launch 경로를 쓴다.
        self.sim_time = sim_time
        self.operating_mode = operating_mode
        self.lock = threading.RLock()
        self.logs = deque(maxlen=100)
        self.received: dict[str, tuple[float, object]] = {}
        self.settings = initial_settings or TestSettings()
        self.state = "IDLE"
        self.run_dir = ""
        self.finish_reason = ""
        self.requested_terminal = "ABORTED"
        self.launch_process: OwnedProcess | None = None
        self.recorder_process: OwnedProcess | None = None
        self.stop_started = 0.0
        self.ready_since: float | None = None
        self.input_loss = InputLossLatch(INPUT_LOSS_BRAKE_DELAY_SEC, INPUT_LOSS_STOPPED_SPEED_MPS)
        self.input_loss_active = False
        self.last_brake_engage = False
        self.input_loss_state = None
        self.recorder_failure_reported = False
        self.graph_issues: list[str] = ["ROS 그래프 확인 중"]
        self.declare_parameter('route_dir', str(route_directory()))
        self.route_dir = Path(self.get_parameter('route_dir').value).expanduser()
        self.runtime_parameters = {}
        self.parameter_clients = []
        self.parameter_requests = []
        self.capture = {}
        self.capture_worker = None
        self.custom_origin_checked = False

        self.command_pub = self.create_publisher(
            ActuatorCommand, "/planning/command", 10
        )
        self.scenario_pub = self.create_publisher(String, "/decision/scenario", 10)
        self.run_active_pub = self.create_publisher(
            Bool, "/decision/test_run_active", 10
        )
        subscriptions = {
            "/gnss/fix": NavSatFix,
            "/gnss/status": GnssStatus,
            "/gnss/fix_velocity": TwistWithCovarianceStamped,
            "/localization/odometry": Odometry,
            "/vehicle/state": VehicleState,
            "/drive/status": DriveActuatorStatus,
            "/steering/status": SteeringActuatorStatus,
            "/brake/status": BrakeActuatorStatus,
            "/planning/route_context": RouteContext,
            "/debug/route_context_age": Float32,
            "/debug/vehicle_state_age": Float32,
            "/planning/route_ready": Bool,
            "/planning/readiness_status": DiagnosticArray,
            "/planning/scene_summary": SceneSummary,
            "/perception/objects": PerceptionObjectArray,
            "/decision/raw_command": ActuatorCommand,
            "/planning/behavior_decision": BehaviorDecision,
            "/vehicle/command": ActuatorCommand,
            "/debug/active_behavior": String,
            "/debug/fsm_state": String,
        }
        self.subscriptions_owned = [
            self.create_subscription(
                message_type,
                topic,
                lambda message, topic=topic: self.receive(topic, message),
                qos_profile_sensor_data,
            )
            for topic, message_type in subscriptions.items()
        ]
        self.create_timer(0.05, self.tick)
        self.create_timer(1.0, self.scan_graph)

    def now(self) -> float:
        return time.monotonic()

    def log(self, message: str) -> None:
        line = datetime.now().astimezone().isoformat(timespec="milliseconds")
        with self.lock:
            self.logs.append(f"{line} {message}")

    def receive(self, topic: str, message: object) -> None:
        with self.lock:
            self.received[topic] = (self.now(), message)

    def fresh(self, topic: str, timeout: float = 1.0) -> bool:
        sample = self.received.get(topic)
        return sample is not None and self.now() - sample[0] <= timeout

    def message(self, topic: str):
        sample = self.received.get(topic)
        return sample[1] if sample else None

    def perception_gate(self) -> tuple[bool, str]:
        if not self.settings.requires_perception:
            return True, "인지 출발 조건 미사용"
        if self.settings.mode == "avoidance" and self.fresh("/perception/objects", 0.5):
            objects = self.message("/perception/objects")
            route = self.message("/planning/route_context")
            if route is not None and route.route_projection_valid and objects.header.frame_id == route.header.frame_id:
                return True, "라이다 장애물 수신 및 전역경로 정합: 설정 통로 사용 가능"
        if not self.fresh("/planning/scene_summary", SCENE_RECEIVE_TIMEOUT_SEC):
            return False, f"인지 장면 미수신: 허용 수신 간격 {SCENE_RECEIVE_TIMEOUT_SEC:g}초"
        scene = self.message("/planning/scene_summary")
        health = str(scene.perception_health).strip().upper()
        if health != "GOOD":
            return False, f"인지 상태 {health or 'UNKNOWN'}: GOOD 필요"
        return True, "인지 장면 수신 및 상태 GOOD"

    def perception_health_continue(self, route_ready: bool) -> bool:
        """주행 중에는 인지 상태가 GOOD이 아니어도 장면이 들어오는 동안 이어 간다(D3).

        시험 모드와 경기 모드가 같다. 인지 상태는 경고와 로그만 남기고 속도와 제동에 쓰지 않는다.
        """
        if not self.settings.requires_perception:
            return False
        return route_ready and self.fresh("/planning/scene_summary", SCENE_RECEIVE_TIMEOUT_SEC)

    def track_route_readiness(self, ready: bool) -> None:
        """주행 중 출발 점검 상태가 바뀔 때만 로그를 남긴다."""
        if ready == getattr(self, 'route_readiness_ok', True):
            return
        self.route_readiness_ok = ready
        if ready:
            self.log("출발 점검 정상 복귀")
        else:
            self.log("출발 점검 경고, 주행 계속: " + "; ".join(self.readiness_blockers()))

    def scene_health(self) -> str:
        if not self.fresh("/planning/scene_summary", SCENE_RECEIVE_TIMEOUT_SEC):
            return "NO_SCENE"
        scene = self.message("/planning/scene_summary")
        return str(scene.perception_health).strip().upper() or "UNKNOWN"

    def track_perception_health(self, now: float) -> None:
        """주행 중 인지 상태가 바뀐 시각을 남긴다."""
        health = self.scene_health()
        previous = getattr(self, 'perception_health', None)
        if health == previous:
            return
        self.perception_health = health
        if previous is None:
            if health == "GOOD":
                return
            previous = "RUN_START"
        changes = getattr(self, 'perception_health_changes', None)
        if changes is None:
            changes = self.perception_health_changes = []
        changes.append({
            'at': datetime.now(ZoneInfo("Asia/Seoul")).isoformat(timespec='milliseconds'),
            'from': previous,
            'to': health,
        })
        unlogged = getattr(self, 'perception_health_unlogged', 0)
        logged_at = getattr(self, 'perception_health_logged_at', None)
        if logged_at is not None and now - logged_at < PERCEPTION_HEALTH_LOG_INTERVAL_SEC:
            self.perception_health_unlogged = unlogged + 1
            return
        suffix = f", 직전 로그 뒤 변화 {unlogged}회" if unlogged else ""
        action = "경고만, 주행 계속" if health != "GOOD" else "정상 복귀"
        self.log(f"인지 상태 {previous} -> {health}: {action}{suffix}")
        self.perception_health_logged_at = now
        self.perception_health_unlogged = 0

    def track_estop(self, vehicle) -> None:
        """E-Stop PAUSE/RUN이 바뀐 시각을 남긴다. Run 상태는 바꾸지 않는다."""
        if vehicle is None:
            return
        active = bool(vehicle.estop_active)
        previous = getattr(self, 'last_estop_active', None)
        if active == previous:
            return
        self.last_estop_active = active
        if previous is None and not active:
            return
        changes = getattr(self, 'estop_changes', None)
        if changes is None:
            changes = self.estop_changes = []
        changes.append({
            'at': datetime.now(ZoneInfo("Asia/Seoul")).isoformat(timespec='milliseconds'),
            'state': self.state,
            'to': 'PAUSE' if active else 'RUN',
        })
        if active:
            self.log(f"E-Stop PAUSE ({self.state}): 제동, 구동 차단, 조향 disable")
        elif self.state == "RUNNING":
            self.log("E-Stop RUN: 판단 명령대로 주행")
        else:
            self.log(f"E-Stop RUN ({self.state}): RUNNING 전환 뒤 출발")

    def obstacle_zone_runtime_continue(self) -> bool:
        if not self.settings.obstacle_enabled:
            return False
        route = self.message("/planning/route_context")
        zone = 'MIDAN_TEST' if self.settings.case == OBSTACLE_CASE and self.settings.course.startswith('custom:') else 'F_STAGE_6'
        return route is not None and str(route.current_zone) == zone

    def read_runtime_parameters(self) -> None:
        for client in self.parameter_clients:
            self.destroy_client(client)
        self.parameter_clients = []
        self.runtime_parameters = {}
        self.parameter_requests = []
        requests = {'/eskf_localization': LOCALIZATION_PARAMETERS,
                    '/command_governor': ('deceleration_slew_mps2', 'acceleration_slew_mps2', 'input_timeout_ms')}
        for node_name, names in requests.items():
            client = self.create_client(GetParameters, node_name + '/get_parameters')
            self.parameter_clients.append(client)
            self.runtime_parameters[node_name] = {'status': 'unknown'}
            self.parameter_requests.append([client, node_name, names, False])
        self.poll_parameter_reads()

    def poll_parameter_reads(self):
        for request in getattr(self, 'parameter_requests', []):
            client, node_name, names, sent = request
            if sent or not client.service_is_ready():
                continue
            request[3] = True
            future = client.call_async(GetParameters.Request(names=list(names)))
            def done(result, key=node_name, keys=names, snapshot=self.runtime_parameters):
                if snapshot is not self.runtime_parameters:
                    return
                try:
                    values = result.result().values
                    if len(values) != len(keys):
                        raise ValueError('parameter service returned an incomplete snapshot')
                    parameters = dict(zip(keys, (parameter_value_to_python(v) for v in values)))
                    self.runtime_parameters[key] = {'status': 'readback', 'parameters': parameters,
                                                    'ros_stamp_ns': self.get_clock().now().nanoseconds}
                except Exception as error:
                    self.runtime_parameters[key] = {'status': 'unknown', 'error': str(error)}
            future.add_done_callback(done)

    def localization_parameters(self) -> dict:
        return self.runtime_parameters.get('/eskf_localization', {}).get('parameters', {})

    def capture_tick(self):
        if self.state not in {'CAPTURE_PREP', 'WAIT_MOVEMENT', 'CAPTURING'}:
            return
        if self.recorder_process is None or self.recorder_process.process.poll() is not None:
            self.request_stop('MCAP recorder 종료', 'ERROR')
            return
        vehicle = self.message('/vehicle/state')
        odom = self.message('/localization/odometry')
        if not self.fresh('/vehicle/state', 1.) or not self.fresh('/localization/odometry', 1.):
            if self.state != 'CAPTURE_PREP':
                self.capture['invalid_reason'] = '차량/측위 입력 유실'
            return
        if vehicle.mode != VehicleState.MODE_MANUAL:
            self.capture['invalid_reason'] = '수집 중 AUTO 전환'
            return
        if self.state == 'CAPTURE_PREP':
            required = ('/vehicle/state', '/localization/odometry', '/gnss/fix', '/tf_static')
            subscribed = all(any(i.node_name == 'rosbag2_recorder'
                                 for i in self.get_subscriptions_info_by_topic(t)) for t in required)
            try:
                origin_from_parameters(self.localization_parameters())
            except ValueError:
                return
            if not subscribed:
                return
            if abs(vehicle.speed_mps) > .2:
                self.capture['invalid_reason'] = 'recorder 준비 완료 전 출발'
                return
            self.state = 'WAIT_MOVEMENT'
            self.log('수집 준비 완료: 원하는 경로로 MANUAL 출발, 끝에서 S')
        if self.state == 'WAIT_MOVEMENT':
            if abs(vehicle.speed_mps) <= .2:
                self.capture['anchor_ns'] = stamp_ns(odom)
                self.capture.pop('moving_since', None)
            else:
                self.capture.setdefault('moving_since', self.now())
                if self.now() - self.capture['moving_since'] >= .3:
                    if 'anchor_ns' not in self.capture:
                        self.capture['invalid_reason'] = '출발 직전 정지 위치 없음'
                    else:
                        self.capture['start_ns'] = self.capture['anchor_ns']
                        self.state = 'CAPTURING'
                        self.log('경로 수집 중')

    def scan_graph(self) -> None:
        self.poll_parameter_reads()
        issues = []
        planning_publishers = self.get_publishers_info_by_topic("/planning/command")
        if len(planning_publishers) != 1 or planning_publishers[0].node_name != self.get_name():
            issues.append("/planning/command에 다른 발행자가 있음")
        vehicle_publishers = self.get_publishers_info_by_topic("/vehicle/command")
        if len(vehicle_publishers) != 1:
            issues.append("/vehicle/command 발행자는 정확히 1개여야 함")
        localization_publishers = self.get_publishers_info_by_topic(
            "/localization/odometry"
        )
        if len(localization_publishers) > 1:
            issues.append("/localization/odometry 발행자가 중복됨")
        recorder_nodes = [
            name
            for name, _namespace in self.get_node_names_and_namespaces()
            if name == "rosbag2_recorder"
        ]
        if recorder_nodes and self.recorder_process is None and not self.operating_mode:
            issues.append("다른 rosbag2_recorder가 실행 중")
        with self.lock:
            self.graph_issues = issues

    def external_issues(self) -> list[str]:
        if self.settings.case == ROUTE_CAPTURE_CASE:
            issues = []
            for topic in ('/vehicle/state', '/localization/odometry', '/gnss/fix'):
                if not self.fresh(topic):
                    issues.append(f'{topic} 미수신')
            vehicle = self.message('/vehicle/state')
            if vehicle is not None and (vehicle.mode != VehicleState.MODE_MANUAL or abs(vehicle.speed_mps) > .2):
                issues.append('수집 시작 위치에서 MANUAL 정지 필요')
            if not self.settings.route_name.strip():
                issues.append('N으로 경로 이름 입력')
            return issues
        issues = list(self.graph_issues)
        required_topics = REQUIRED_EXTERNAL_TOPICS
        for topic in required_topics:
            if not self.fresh(topic):
                issues.append(f"{topic} 미수신")
        vehicle = self.message("/vehicle/state")
        if vehicle is not None:
            if vehicle.mode != VehicleState.MODE_MANUAL:
                issues.append("Run 전 차량을 MANUAL로 전환")
            if abs(float(vehicle.speed_mps)) > 0.2:
                issues.append("Run 전 차량 정지 필요")
        steering = self.message("/steering/status")
        if steering is not None and (
            not steering.communication_ok
            or not steering.feedback_valid
            or steering.can_timeout
        ):
            issues.append("KEYA 조향 통신/피드백 상태 확인")
        return issues

    def runtime_warnings(self) -> list[str]:
        warnings = []
        if self.state == "RUNNING" and self.settings.requires_perception:
            health = self.scene_health()
            if health != "GOOD":
                warnings.append(f"인지 상태 {health}: 경고만, run을 끝내지 않고 주행 계속")
        if (
            self.state == "RUNNING"
            and not self.sim_time
            and not getattr(self, 'route_readiness_ok', True)
        ):
            warnings.append("출발 점검 항목 이상: run을 끝내지 않고 주행 계속, 원인은 로그 확인")
        loss = getattr(self, "input_loss_state", None)
        if self.state == "RUNNING" and loss is not None and loss.active:
            warnings.append("입력 끊김: 속도 0" + (", 정지까지 브레이크" if loss.latched else ""))
        if getattr(self, 'recorder_failure_reported', False):
            warnings.append("MCAP recorder 종료: 기록 없이 주행 계속")
        steering = self.message("/steering/status")
        if steering is not None and steering.tracking_error_fault:
            tracking_error_deg = math.degrees(float(steering.tracking_error_rad))
            warnings.append(
                f"조향 추종 오차 fault {tracking_error_deg:+.1f} deg: 시험·MCAP 기록 계속"
            )
        return warnings

    def request_run(self, settings: TestSettings) -> None:
        if self.state not in {"IDLE", "COMPLETE", "ABORTED", "ERROR"}:
            self.log("현재 Run을 먼저 종료하세요")
            return
        if self.operating_mode and (
            settings.case != MISSION_CASE or settings.course not in COURSE_NAMES
        ):
            self.log("운용 모드는 예선·본선 미션 실행만 지원합니다")
            return
        if self.sim_time and settings.launch_case not in SIMULATION_LAUNCH_CASES:
            self.log("시뮬레이션에서는 예선·본선 미션 실행만 선택할 수 있습니다")
            return
        self.settings = settings
        issues = self.external_issues()
        if issues:
            self.log("RUN NO-GO: " + "; ".join(issues))
            return
        self.state = "STARTING"
        self.finish_reason = ""
        self.requested_terminal = "ABORTED"
        self.ready_since = None
        self.input_loss = InputLossLatch(INPUT_LOSS_BRAKE_DELAY_SEC, INPUT_LOSS_STOPPED_SPEED_MPS)
        self.input_loss_active = False
        self.last_brake_engage = False
        self.input_loss_state = None
        self.recorder_failure_reported = False
        self.custom_origin_checked = False
        self.capture = {}
        self.perception_health = None
        self.perception_health_changes = []
        self.perception_health_logged_at = None
        self.perception_health_unlogged = 0
        self.last_estop_active = None
        self.estop_changes = []
        if settings.case in {OBSTACLE_CASE, ROUTE_CAPTURE_CASE} or settings.course.startswith('custom:'):
            self.read_runtime_parameters()
        for topic in (
            "/localization/odometry",
            "/planning/route_context",
            "/planning/route_ready",
            "/planning/readiness_status",
            "/decision/raw_command",
        ):
            self.received.pop(topic, None)
        try:
            self.start_processes()
        except Exception as error:
            self.state = "ERROR"
            self.finish_reason = str(error)
            self.log(f"시작 실패: {error}")
            self.close_processes("start_failed")
            return
        self.state = 'CAPTURE_PREP' if settings.case == ROUTE_CAPTURE_CASE else "PREFLIGHT"
        self.log(
            f"{CASE_NAMES[settings.case]} 시작, "
            f"course={settings.course}, mode={settings.mode}"
        )

    def start_processes(self) -> None:
        if self.operating_mode:
            if self.settings.case != MISSION_CASE or self.settings.course not in COURSE_NAMES:
                raise ValueError("운용 모드는 예선·본선 미션 실행만 지원합니다")
            self.run_dir = ""
        else:
            self.start_recording()
        if self.settings.case == ROUTE_CAPTURE_CASE:
            return
        launch_arguments = [
            "ros2",
            "launch",
            "kaiev26_decision",
            "decision_test.launch.py",
            *self.settings.launch_arguments(simulation=self.sim_time),
        ]
        self.launch_process = OwnedProcess("Decision", launch_arguments, self.log)

    def start_recording(self) -> None:
        started_at = datetime.now(ZoneInfo("Asia/Seoul"))
        run_dir = create_recording_directory(
            workspace_root() / 'records', self.settings, started_at
        )
        self.run_dir = str(run_dir)
        document = self.settings.document()
        document["started_at"] = started_at.isoformat()
        if self.settings.case in {OBSTACLE_CASE, ROUTE_CAPTURE_CASE} or self.settings.course.startswith('custom:'):
            document['source_versions'] = {}
            for repo in ('Decision', 'Control', 'Perception'):
                folder = workspace_root() / 'src' / repo
                try:
                    commit = subprocess.check_output(['git', '-C', str(folder), 'rev-parse', 'HEAD'], text=True, stderr=subprocess.DEVNULL).strip()
                    status = subprocess.check_output(['git', '-C', str(folder), 'status', '--porcelain'], text=True, stderr=subprocess.DEVNULL)
                    document['source_versions'][repo] = dict(commit=commit, dirty=bool(status))
                except (OSError, subprocess.CalledProcessError):
                    document['source_versions'][repo] = 'unknown'
            from ament_index_python.packages import get_package_share_directory
            document['configuration_snapshots'] = {}
            for package, filename in (
                ('kaiev26_motion_control', 'motion_control.yaml'),
                ('kaiev26_decision', 'decision_pipeline.yaml'),
                ('kaiev26_decision', 'midan_test_policy.yaml'),
            ):
                source = Path(get_package_share_directory(package)) / 'config' / filename
                if source.is_file():
                    document['configuration_snapshots'][filename] = yaml.safe_load(source.read_text())
            if self.settings.route_config and self.settings.case != ROUTE_CAPTURE_CASE:
                source = Path(self.settings.route_config)
                document['route_snapshot'] = load_custom_route(source)
                document['route_yaml_sha256'] = hashlib.sha256(source.read_bytes()).hexdigest()
            document['hardware_firmware_and_mechanical_changes'] = 'unknown; operator note is not readback'
        (run_dir / "scenario.yaml").write_text(
            yaml.safe_dump(document, sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )
        self.recorder_process = OwnedProcess(
            "MCAP",
            [
                sys.executable,
                "-u",
                "-c",
                RECORDER_CODE,
                str(run_dir / "rosbag"),
                json.dumps(self.settings.record_topics),
            ],
            self.log,
        )

    def request_stop(self, reason: str = "사용자 Stop", terminal: str = "ABORTED") -> None:
        if self.state in {"IDLE", "COMPLETE", "ABORTED", "ERROR", "CLOSING", 'CONVERTING', 'STOPPING', 'SAVE_PROMPT'}:
            return
        self.finish_reason = reason
        self.requested_terminal = terminal
        self.state = "STOPPING"
        self.stop_started = self.now()
        if self.settings.case == ROUTE_CAPTURE_CASE:
            self.capture.setdefault('end_ns', self.get_clock().now().nanoseconds)
            self.requested_terminal = 'COMPLETE' if terminal != 'ERROR' and 'start_ns' in self.capture else terminal
        self.log(reason)

    def route_gate(self) -> bool:
        if self.sim_time:
            # 시뮬레이션 launch는 route_readiness를 띄우지 않는다. 경로 정합 수신으로 대신한다.
            return self.fresh('/planning/route_context')
        ready_msg = self.message("/planning/route_ready")
        return self.fresh('/planning/route_ready') and bool(ready_msg.data)

    def tick(self) -> None:
        with self.lock:
            if self.settings.case == ROUTE_CAPTURE_CASE:
                self.capture_tick()
                self.publish_label()
                return
            now = self.now()
            vehicle = self.message("/vehicle/state")
            if self.state not in EDITABLE_STATES:
                self.track_estop(vehicle)
            route_ready = self.route_gate()
            if self.state == "RUNNING" and not self.sim_time:
                # 출발 점검은 출발만 막는다. 주행 중에 나빠지면 경고만 남기고 계속 간다.
                self.track_route_readiness(route_ready)
                route_ready = True
            perception_ready, _perception_detail = self.perception_gate()
            pipeline_ready = route_ready and perception_ready
            if self.settings.course.startswith('custom:'):
                if not getattr(self, 'custom_origin_checked', False) and self.state == 'PREFLIGHT':
                    try:
                        verify_origin(load_custom_route(self.settings.route_config), self.localization_parameters())
                        self.custom_origin_checked = True
                    except ValueError as error:
                        if self.localization_parameters():
                            self.request_stop(str(error), 'ERROR')
                pipeline_ready = pipeline_ready and getattr(self, 'custom_origin_checked', False)
            obstacle_continue = self.obstacle_zone_runtime_continue()
            health_continue = self.perception_health_continue(route_ready)
            runtime_pipeline_ready = pipeline_ready or (
                self.state == "RUNNING" and (obstacle_continue or health_continue)
            )
            if self.state == "RUNNING" and self.settings.requires_perception:
                self.track_perception_health(now)
            auto = vehicle is not None and vehicle.mode == VehicleState.MODE_AUTO

            if self.state == "PREFLIGHT" and pipeline_ready:
                if self.ready_since is None:
                    self.ready_since = now
                elif now - self.ready_since >= 0.5:
                    self.state = "WAIT_AUTO"
                    self.log(
                        "PRE-FLIGHT READY: 안전요원이 AUTO로 전환할 수 있습니다"
                    )
            elif self.state == "PREFLIGHT":
                self.ready_since = None

            # 출발 대기는 E-Stop PAUSE가 맡는다(규정 2.17.2 사). AUTO 전환 즉시 판단 명령을 전달한다.
            if self.state == "WAIT_AUTO" and auto:
                self.state = "RUNNING"
                self.route_readiness_ok = True
                self.log("AUTO 확인, RUNNING")

            if self.state == "RUNNING":
                if not auto:
                    self.request_stop("MANUAL 전환으로 Run 종료")
                else:
                    self.track_input_loss(
                        now, runtime_pipeline_ready, vehicle,
                    )
                    if self.decision_finished():
                        self.request_stop("경로 종료점 정지 완료", "COMPLETE")

            if self.launch_process is not None and self.launch_process.process.poll() is not None:
                self.request_stop("Decision launch가 예기치 않게 종료됨", "ERROR")
            if (
                self.recorder_process is not None
                and self.recorder_process.process.poll() is not None
                and not getattr(self, "recorder_failure_reported", False)
            ):
                # 기록이 끊겨도 Run은 끝내지 않는다. 경고만 남긴다.
                self.log("MCAP recorder 종료: 경고만, 주행 계속")
                self.recorder_failure_reported = True

            self.publish_run_active()
            self.publish_command(runtime_pipeline_ready, auto)
            self.publish_label()

    def track_input_loss(self, now: float, pipeline_ready: bool, vehicle) -> None:
        """주행 중 입력 끊김. 속도 0으로 내리고 1초 뒤 제동하며, Run은 끝내지 않는다."""
        inputs_ok = pipeline_ready and self.fresh("/decision/raw_command", RAW_COMMAND_TIMEOUT_SEC)
        speed = abs(float(vehicle.speed_mps)) if vehicle is not None else None
        latch = getattr(self, "input_loss", None)
        if latch is None:
            latch = self.input_loss = InputLossLatch(
                INPUT_LOSS_BRAKE_DELAY_SEC, INPUT_LOSS_STOPPED_SPEED_MPS
            )
        loss = latch.update(
            now, inputs_ok, speed, brake_engaged=getattr(self, "last_brake_engage", False)
        )
        previous = getattr(self, "input_loss_state", None)
        was_stale = previous is not None and previous.stale
        was_latched = previous is not None and previous.latched
        if loss.stale and not was_stale:
            cause = (
                '판단 명령 수신 중단'
                if not self.fresh("/decision/raw_command", RAW_COMMAND_TIMEOUT_SEC)
                else '; '.join(self.readiness_blockers())
            )
            self.log(f"입력 끊김, 속도 0: {cause}")
        if loss.latched and not was_latched:
            self.log(f"입력 끊김 {INPUT_LOSS_BRAKE_DELAY_SEC:g}초: 정지까지 브레이크")
        if was_stale and not loss.stale and not loss.latched and not was_latched:
            self.log("입력 복귀: 판단 명령 중계 재개")
        if was_latched and not loss.latched:
            self.log("입력 복귀와 정지 확인: 판단 명령 중계 재개")
        self.input_loss_active = loss.active
        self.input_loss_state = loss

    def decision_finished(self) -> bool:
        """판단이 종료선 뒤 정지를 유지하고 있는지. TUI는 자체 종료 판정을 두지 않는다."""
        if not self.fresh("/planning/behavior_decision", BEHAVIOR_DECISION_TIMEOUT_SEC):
            return False
        decision = self.message("/planning/behavior_decision")
        return FINISH_HELD_CONSTRAINT in list(decision.constraints)

    def publish_run_active(self) -> None:
        if self.settings.case == ROUTE_CAPTURE_CASE:
            return
        message = Bool()
        message.data = self.state == "RUNNING"
        self.run_active_pub.publish(message)

    def publish_command(
        self,
        route_ready: bool,
        auto: bool,
        *,
        raw_command_timeout: float = RAW_COMMAND_TIMEOUT_SEC,
    ) -> None:
        if self.settings.case == ROUTE_CAPTURE_CASE:
            return
        command = ActuatorCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        raw = self.message("/decision/raw_command")
        vehicle = self.message("/vehicle/state")
        loss = getattr(self, "input_loss_state", None) if self.state == "RUNNING" else None
        # loss는 이번 tick의 track_input_loss 결과다. 출발 전과 Run 종료 뒤에는 쓰지 않는다.
        can_relay = (
            self.state == "RUNNING"
            and route_ready
            and auto
            and raw is not None
            and self.fresh("/decision/raw_command", RAW_COMMAND_TIMEOUT_SEC)
            and (loss is None or not loss.active)
        )
        if can_relay:
            command.steering_target_rad = float(raw.steering_target_rad)
            command.speed_target_mps = float(raw.speed_target_mps)
            command.brake_engage = bool(raw.brake_engage)
        else:
            command.steering_target_rad = (
                float(vehicle.steering_rad) if vehicle is not None else 0.0
            )
            command.speed_target_mps = 0.0
            if self.state == "RUNNING" and loss is not None:
                # 입력 끊김: 1초 동안은 제동 없이 속도 0, 그 뒤 실제 정지까지 제동.
                command.brake_engage = bool(auto and loss.brake)
            else:
                # 출발 전에는 브레이크를 걸지 않는다. Run이 끝난 뒤에는 AUTO인 동안 잡고,
                # MANUAL로 바꾸면 풀린다(STM_B는 AUTO일 때만 명령 제동을 건다).
                command.brake_engage = bool(auto and self.state not in PRE_RUN_STATES)
        self.last_brake_engage = bool(command.brake_engage)
        self.command_pub.publish(command)

    def publish_label(self) -> None:
        label = String()
        label.data = (
            f"case={self.settings.case} course={self.settings.course} "
            f"mode={self.settings.mode} "
            f"controller={'manual' if self.settings.case == ROUTE_CAPTURE_CASE else 'pp_stanley'} "
            f"state={self.state}"
        )
        self.scenario_pub.publish(label)

    def should_close(self) -> bool:
        delay = 0.0 if self.settings.case == ROUTE_CAPTURE_CASE else 2.0
        return self.state == "STOPPING" and self.now() - self.stop_started >= delay

    def close_processes(self, reason: str) -> None:
        self.state = "CLOSING"
        codes = {}
        process_order = ('recorder_process', 'launch_process') if self.settings.case == ROUTE_CAPTURE_CASE else ('launch_process', 'recorder_process')
        for name in process_order:
            child = getattr(self, name)
            setattr(self, name, None)
            if child is not None:
                try:
                    codes[child.name] = child.stop()
                except Exception as error:
                    codes[child.name] = str(error)
                    self.log(f"{child.name} 종료 오류: {error}")
        if self.run_dir:
            path = Path(self.run_dir) / "scenario.yaml"
            try:
                document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
                document.update(
                    {
                        "finished_at": datetime.now(ZoneInfo("Asia/Seoul")).isoformat(),
                        "finish_reason": reason,
                        "process_exit": codes,
                        "estop_changes": getattr(self, "estop_changes", []),
                    }
                )
                if self.settings.case in {OBSTACLE_CASE, ROUTE_CAPTURE_CASE} or self.settings.course.startswith('custom:'):
                    document['runtime_parameters'] = getattr(self, 'runtime_parameters', {})
                    document['terminal_state'] = self.requested_terminal
                    document['capture'] = getattr(self, 'capture', {})
                    metadata = Path(self.run_dir) / 'rosbag/metadata.yaml'
                    if metadata.is_file():
                        info = yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
                        counts = {v['topic_metadata']['name']: dict(
                            count=v['message_count'], type=v['topic_metadata']['type'])
                            for v in info['topics_with_message_count']}
                        document['recorded_topics'] = counts
                        document['missing_topics'] = [t for t in self.settings.record_topics
                                                      if counts.get(t, {}).get('count', 0) == 0]
                        try:
                            document['dynamics_topic_timing'] = recording_quality(
                                Path(self.run_dir) / 'rosbag', self.settings.record_topics)
                        except Exception as error:
                            document['recording_quality_error'] = str(error)
                        essential = ('/vehicle/state', '/localization/odometry')
                        if self.settings.case != ROUTE_CAPTURE_CASE:
                            essential += ('/planning/command', '/vehicle/command')
                        document['measurement_data_complete'] = all(counts.get(t, {}).get('count', 0) > 0 for t in essential)
                path.write_text(
                    yaml.safe_dump(document, sort_keys=False, allow_unicode=True),
                    encoding="utf-8",
                )
                (Path(self.run_dir) / "console.log").write_text(
                    "\n".join(self.logs) + "\n", encoding="utf-8"
                )
            except OSError as error:
                self.log(f"실행 기록 저장 오류: {error}")
        self.state = self.requested_terminal
        if self.settings.case == OBSTACLE_CASE and self.run_dir and (Path(self.run_dir) / 'rosbag').is_dir():
            # Obstacle trials: the operator decides whether this MCAP is worth keeping.
            self.save_terminal = self.state
            self.state = 'SAVE_PROMPT'
            self.log(f"MCAP 저장 여부 선택: Y 보존 / N 삭제 ({self.run_dir})")
            return
        self.log(f"MCAP 저장 완료: {self.run_dir}" if not self.operating_mode
                 else f"Run 종료: {reason}")
        if self.settings.case == ROUTE_CAPTURE_CASE and self.requested_terminal == 'COMPLETE':
            self.state = 'CONVERTING'
            self.capture_worker = threading.Thread(target=self.convert_capture, daemon=False)
            self.capture_worker.start()

    def resolve_mcap_save(self, keep: bool) -> None:
        if self.state != 'SAVE_PROMPT':
            return
        run_dir = Path(self.run_dir)
        bag = run_dir / 'rosbag'
        error_text = ''
        if not keep:
            try:
                shutil.rmtree(bag)
            except OSError as error:
                error_text = str(error)
                self.log(f"MCAP 삭제 오류: {error}")
        path = run_dir / 'scenario.yaml'
        try:
            document = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
            document['mcap_kept'] = bag.exists()
            if error_text:
                document['mcap_delete_error'] = error_text
            path.write_text(yaml.safe_dump(document, sort_keys=False, allow_unicode=True),
                            encoding='utf-8')
        except OSError as error:
            self.log(f"실행 기록 저장 오류: {error}")
        self.state = self.save_terminal
        self.log(f"MCAP 저장 완료: {run_dir}" if bag.exists()
                 else f"MCAP 삭제 완료 (scenario.yaml·console.log만 보존): {run_dir}")

    def convert_capture(self):
        path = Path(self.run_dir) / 'scenario.yaml'
        document = {}
        terminal = 'ERROR'
        try:
            document = yaml.safe_load(path.read_text()) or {}
            if not isinstance(document, dict):
                document = {}
                raise ValueError('Invalid scenario metadata')
            if self.capture.get('invalid_reason'):
                raise ValueError(self.capture['invalid_reason'])
            if document.get('process_exit', {}).get('MCAP') not in (0, -2):
                raise ValueError('MCAP did not close normally')
            result = capture_from_bag(Path(self.run_dir) / 'rosbag', self.capture['start_ns'],
                                      self.capture['end_ns'], self.localization_parameters(),
                                      self.settings.route_name, Path(self.run_dir).name, self.route_dir,
                                      document.get('configuration_snapshots'))
            document['generated_route'] = result
            self.log(f"경로 등록: {result['name']} / {result['points']}점 / {result['length_m']:.1f} m / {result['path']}")
            terminal = 'COMPLETE'
        except Exception as error:
            document['route_generation_error'] = str(error)
            self.finish_reason = f'경로 생성 실패, MCAP 보존: {error}'
            self.log(self.finish_reason)
        document['terminal_state'] = terminal
        try:
            path.write_text(yaml.safe_dump(document, allow_unicode=True, sort_keys=False), encoding='utf-8')
        except OSError as error:
            terminal = 'ERROR'
            self.finish_reason = f'경로 변환 기록 저장 실패, MCAP 보존: {error}'
            self.log(self.finish_reason)
        finally:
            self.state = terminal

    def readiness_rows(self) -> list[tuple[str, str, str, str]]:
        if self.settings.case == ROUTE_CAPTURE_CASE:
            return [(t, '수신' if self.fresh(t) else '미수신', 'GOOD' if self.fresh(t) else 'WAIT',
                     '수동 경로 수집 입력') for t in ('/vehicle/state', '/localization/odometry', '/gnss/fix')]
        rows = self.simulation_rows() if self.sim_time else self.diagnostic_rows()
        if self.settings.requires_perception:
            perception_ready, perception_detail = self.perception_gate()
            perception_received = self.fresh('/planning/scene_summary', 0.5)
            rows.append((
                '/planning/scene_summary', '수신' if perception_received else '미수신',
                'GOOD' if perception_ready else 'WAIT', perception_detail,
            ))
        return rows

    def simulation_rows(self) -> list[tuple[str, str, str, str]]:
        rows = []
        for topic in READINESS_LABELS:
            fresh = self.fresh(topic)
            rows.append((
                topic, '수신' if fresh else '미수신', 'GOOD' if fresh else 'WAIT',
                '시뮬레이션: 수신만 확인' if fresh else '미수신',
            ))
        return rows

    def diagnostic_rows(self) -> list[tuple[str, str, str, str]]:
        diagnostics = self.message('/planning/readiness_status')
        statuses = (
            {item.name: item for item in diagnostics.status if item.name != 'PRE-FLIGHT'}
            if self.fresh('/planning/readiness_status') else {}
        )
        topics = list(READINESS_LABELS)
        topics.extend(
            name for name, item in statuses.items()
            if name not in READINESS_LABELS and item.level != DiagnosticStatus.OK
        )
        rows = []
        for topic in topics:
            item = statuses.get(topic)
            reception = '수신' if self.fresh(topic) else '미수신'
            if topic not in READINESS_LABELS:
                reception = '발행자'
            if item is None:
                state = 'WAIT'
                detail = (
                    'Run 후 출발 조건 판정'
                    if self.state in {'IDLE', 'COMPLETE', 'ABORTED', 'ERROR'}
                    else '출발 진단 미수신: route_readiness 노드 확인'
                )
            else:
                state = (
                    'GOOD' if item.level == DiagnosticStatus.OK
                    else 'FAULT' if item.level in {DiagnosticStatus.ERROR, DiagnosticStatus.STALE}
                    else 'WAIT'
                )
                detail = next((v.value for v in item.values if v.key == 'detail'), item.message)
            rows.append((topic, reception, state, detail))
        return rows

    def readiness_blockers(self) -> list[str]:
        if self.settings.case == ROUTE_CAPTURE_CASE:
            return self.external_issues()
        if self.sim_time:
            reasons = [
                f'{READINESS_LABELS.get(topic, "인지 장면")}: {detail}'
                for topic, _reception, state, detail in self.readiness_rows()
                if state != 'GOOD'
            ]
            return reasons or ['출발 조건 안정화 대기']
        reasons = []
        if not self.fresh('/planning/route_ready'):
            reasons.append('출발 허가 신호 미수신: route_readiness 노드 확인')
        if not self.fresh('/planning/readiness_status'):
            reasons.append('출발 진단 미수신: 차단 원인 확인 불가')
        else:
            reasons.extend(
                f'{READINESS_LABELS.get(topic, topic)}: {detail}'
                for topic, _reception, state, detail in self.readiness_rows()
                if state != 'GOOD'
            )
        return reasons or [
            '출발 허가 해제됨: 상세 진단 갱신 대기'
            if self.state == 'RUNNING' else '출발 조건 안정화 대기'
        ]

    def gnss_detail(self) -> str:
        if self.sim_time:
            fix = self.message('/gnss/fix')
            return (
                '시뮬레이션: RTK 보정(NTRIP) 확인 생략 | '
                + (f'/gnss/fix status={fix.status.status}' if self.fresh('/gnss/fix')
                   else '/gnss/fix 미수신')
            )
        if not self.fresh('/gnss/status', 2.0):
            return 'GNSS 상세 상태 미수신: /gnss/status에서 RTK·NTRIP 상태 확인 필요'
        message = self.message('/gnss/status')
        detail = (
            f'위치해: {message.fix_mode} | 수평 오차 추정 {message.horizontal_accuracy_m:.2f} m | '
            f'NTRIP {"연결됨" if message.ntrip_connected else "연결 안 됨"} | '
            f'RTCM {message.rtcm_bytes_per_sec:g} B/s'
        )
        if message.ntrip_last_error:
            detail += f'\nNTRIP 오류: {message.ntrip_last_error}'
        if not message.ntrip_connected:
            error = message.ntrip_last_error.lower()
            if 'name resolution' in error or 'name or service not known' in error:
                detail += (
                    f'\n조치: 보정 서버 {message.ntrip_host}의 DNS 이름 해석 실패. '
                    '차량 PC의 인터넷·DNS 연결 확인 필요'
                )
            else:
                detail += '\n조치: 보정 서버 연결 설정과 위 NTRIP 오류 확인 필요'
        elif message.rtcm_bytes_per_sec == 0.0:
            detail += '\n보정 데이터 수신 0: NTRIP 보정 스트림 확인 필요'
        return detail

    def snapshot(self) -> dict:
        with self.lock:
            route = self.message("/planning/route_context")
            vehicle = self.message("/vehicle/state")
            behavior = self.message("/debug/active_behavior")
            fsm = self.message("/debug/fsm_state")
            return {
                "sim_time": self.sim_time,
                "state": self.state,
                "speed": float(vehicle.speed_mps) if vehicle is not None else None,
                "estop": (
                    "PAUSE" if vehicle.estop_active else "RUN"
                ) if vehicle is not None else "-",
                "mode": (
                    "AUTO"
                    if vehicle is not None and vehicle.mode == VehicleState.MODE_AUTO
                    else "MANUAL"
                    if vehicle is not None
                    else "WAIT"
                ),
                "ready": (
                    self.state in {'WAIT_MOVEMENT', 'CAPTURING'} and not self.capture.get('invalid_reason')
                    if self.settings.case == ROUTE_CAPTURE_CASE
                    else self.route_gate() and self.perception_gate()[0]
                ),
                "progress": float(route.progress_s) if route is not None else None,
                "cte": float(route.cross_track_error) if route is not None else None,
                "heading": (
                    math.degrees(float(route.heading_error)) if route is not None else None
                ),
                "zone": route.current_zone if route is not None else "-",
                "behavior": behavior.data if behavior is not None else "-",
                "fsm": fsm.data if fsm is not None else "-",
                'capture': getattr(self, 'capture', {}),
                "readiness": self.readiness_rows(),
                "blockers": self.readiness_blockers(),
                "gnss": self.gnss_detail(),
                "issues": (
                    self.external_issues()
                    if self.state in {'IDLE', 'COMPLETE', 'ABORTED', 'ERROR'}
                    else list(self.graph_issues)
                ),
                "warnings": self.runtime_warnings(),
                "logs": list(self.logs)[-6:],
                "run_dir": self.run_dir,
                "recording_enabled": not self.operating_mode,
                "finish_reason": self.finish_reason,
            }


@contextmanager
def keyboard():
    original = termios.tcgetattr(sys.stdin.fileno())
    try:
        tty.setcbreak(sys.stdin.fileno())
        yield
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, original)


def render(
    snapshot: dict,
    selection: TestSettings,
    edit: tuple[str, str] | None,
    message: str,
    operating_mode: bool = False,
) -> Table:
    recording = snapshot.get("recording_enabled", True)
    root = Table.grid(expand=True)
    status = Table.grid(expand=True)
    status.add_column(ratio=1)
    status.add_column(ratio=1)
    if selection.case == ROUTE_CAPTURE_CASE:
        route_text = (f"경로 이름: {selection.route_name or '미입력'} | 차량 명령 미발행\n"
                      + snapshot.get('capture', {}).get('invalid_reason', ''))
    else:
        route_text = (
            "route waiting"
            if snapshot["progress"] is None
            else (
                f"s={snapshot['progress']:.1f} m  CTE={snapshot['cte']:+.2f} m  "
                f"heading={snapshot['heading']:+.1f} deg  zone={snapshot['zone']}"
            )
        )
    speed_text = f"{snapshot['speed']:.2f}" if snapshot['speed'] is not None else '--'
    status.add_row(
        Text(
            f"state {snapshot['state']}  mode {snapshot['mode']}  "
            f"E-Stop {snapshot.get('estop', '-')}  speed {speed_text} m/s\n"
            + ("S 중지·저장 | Q 종료" if recording else "S 중지 | Q 종료")
        ),
        Text(
            f"preflight {'READY' if snapshot['ready'] else 'WAIT'}\n{route_text}",
            style="green" if snapshot["ready"] else "yellow",
        ),
    )
    root.add_row(
        Panel(
            status,
            title=(
                ("Decision Operation" if operating_mode else "Decision Field Test")
                + (" · SIM TIME" if snapshot.get("sim_time") else "")
            ),
        )
    )
    if snapshot.get('estop') == 'PAUSE' and snapshot['state'] in ESTOP_PAUSE_HINTS:
        root.add_row(Text(ESTOP_PAUSE_HINTS[snapshot['state']], style='bold black on yellow'))
    if snapshot['finish_reason']:
        root.add_row(Text('종료 사유: ' + snapshot['finish_reason'], style='yellow'))
    if snapshot['state'] == 'SAVE_PROMPT':
        root.add_row(Text('MCAP을 저장할까요?  Y 보존 / N 삭제  (Q는 보존 후 종료)',
                          style='bold black on yellow'))

    root.add_row(Panel(Text(snapshot['gnss']), title='GNSS / RTK 보정'))
    gates = Table(expand=True)
    gates.add_column('입력', no_wrap=True)
    gates.add_column('수신', no_wrap=True)
    gates.add_column('출발 조건', no_wrap=True)
    gates.add_column('현재 판정 이유', ratio=1)
    for topic, reception, state, detail in sorted(
        snapshot['readiness'], key=lambda row: row[2] == 'GOOD'
    ):
        gates.add_row(
            '인지 장면' if topic == '/planning/scene_summary' else READINESS_LABELS.get(topic, topic), reception,
            Text(state, style='green' if state == 'GOOD' else 'red' if state == 'FAULT' else 'yellow'),
            Text(detail),
        )
    root.add_row(Panel(gates, title='출발 조건 — 미충족 항목 우선'))
    if not snapshot['ready'] and all(row[2] == 'GOOD' for row in snapshot['readiness']):
        root.add_row(Text('출발 불가: ' + '; '.join(snapshot['blockers']), style='yellow'))
    if snapshot['issues']:
        root.add_row(Text('Run 조건: ' + '; '.join(snapshot['issues']), style='yellow'))
    if snapshot['warnings']:
        root.add_row(Text(('기록 경고: ' if recording else '운용 경고: ')
                          + '; '.join(snapshot['warnings']), style='yellow'))

    choices = Table(expand=True)
    choices.add_column("Key", width=4)
    choices.add_column("Operation" if operating_mode else "Case")
    if operating_mode:
        choices.add_row("P", f"코스: {COURSE_NAMES[selection.course]}")
        choices.add_row("", "모드: ALL (고정)")
    else:
        choices.add_row("P", f"코스: {COURSE_NAMES.get(selection.course, '수집 경로: ' + selection.route_name)}")
        for case, name in CASE_NAMES.items():
            if case not in supported_cases(selection.course):
                continue
            marker = ">" if selection.case == case else " "
            choices.add_row(str(case), f"{marker} {name}")
    speed_source = "코스 기본" if selection.speed_mps == 0.0 else "직접 지정"
    speed_label = f"{selection.effective_speed_mps:g} m/s ({speed_source})"
    if selection.case != ROUTE_CAPTURE_CASE:
        choices.add_row("V", f"목표속도: {speed_label}")
        choices.add_row("B", f"출발: {START_MODE_NAMES[selection.start_mode]}")
    else:
        choices.add_row('N', f'경로 이름: {selection.route_name or "미입력"}')
    if selection.case in {OBSTACLE_CASE, ROUTE_CAPTURE_CASE}:
        choices.add_row('H', f'현장 메모: {selection.note or "미입력"}')
    if snapshot["state"] not in EDITABLE_STATES:
        choices.add_row(
            "", "설정 잠금: 실행 중에는 변경할 수 없습니다. Stop 후 변경하세요."
        )
    choices.add_row("R", "Run")
    choices.add_row("S", "Stop and save" if recording else "Stop")
    choices.add_row("Q", "Quit")
    if edit is not None:
        edit_field, edit_value = edit
        edit_labels = {
            "speed_mps": "속도",
            'route_name': '경로 이름', 'note': '현장 메모',
        }
        choices.add_row(
            "",
            f"{edit_labels[edit_field]} 입력: {edit_value}_  "
            "(Enter 적용 / Esc 취소 / Backspace 삭제)",
        )
    if message:
        choices.add_row("", message)
    root.add_row(
        Panel(choices, title="Operation" if operating_mode else "Case Select")
    )

    root.add_row(
        Panel(
            Text(
                f"behavior: {snapshot['behavior']}\n"
                f"fsm: {snapshot['fsm']}\n"
                + (f"record: {snapshot['run_dir'] or '~/KAI_ws/records'}\n" if recording else "")
                + "\n".join(snapshot["logs"])
            ),
            title="Runtime / MCAP" if recording else "Runtime",
        )
    )
    return root


def run_tui(
    initial_settings: TestSettings | None = None,
    *,
    command_hint: str = "ros2 run kaiev26_decision decision_test",
    sim_time: bool = False,
    operating_mode: bool = False,
) -> int:
    console = Console()
    if not sys.stdin.isatty():
        console.print(f"실제 터미널에서 {command_hint}를 실행하세요.")
        return 2

    lock_path = Path("/tmp") / (
        f"decision_test_{os.getuid()}_{os.environ.get('ROS_DOMAIN_ID', '0')}.lock"
    )
    lock_fd = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o600)
    try:
        fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        os.close(lock_fd)
        console.print("같은 ROS domain에서 Decision test가 이미 실행 중입니다.")
        return 2

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = DecisionTestNode(
        initial_settings=initial_settings,
        sim_time=sim_time,
        operating_mode=operating_mode,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    editor = SettingsEditor(
        selection=node.settings,
        operating_mode=operating_mode,
    )
    decoder = codecs.getincrementaldecoder('utf-8')()
    editor.custom_routes, _route_errors = ({}, []) if operating_mode else discover_routes(node.route_dir)
    quitting = False
    try:
        with keyboard(), Live(console=console, screen=True, auto_refresh=False) as live:
            while not quitting:
                if node.should_close():
                    node.close_processes(node.finish_reason or "Run 종료")
                if node.state in EDITABLE_STATES:
                    with node.lock:
                        node.settings = editor.selection
                live.update(
                    render(
                        node.snapshot(),
                        editor.selection,
                        (
                            (editor.edit_field, editor.buffer)
                            if editor.edit_field is not None
                            else None
                        ),
                        editor.message,
                        operating_mode,
                    ),
                    refresh=True,
                )
                readable, _, _ = select.select([sys.stdin], [], [], 0.15)
                if not readable:
                    continue
                raw = os.read(sys.stdin.fileno(), 1)
                if not raw:
                    break
                char = decoder.decode(raw)
                if not char:
                    continue
                if node.state == 'SAVE_PROMPT':
                    if char.lower() in {'y', 'n', 'q'}:
                        node.resolve_mcap_save(char.lower() != 'n')
                    if char.lower() != 'q':
                        continue
                if (not operating_mode and char.lower() == 'p' and editor.edit_field is None
                        and node.state in EDITABLE_STATES):
                    editor.custom_routes, _route_errors = discover_routes(node.route_dir)
                if editor.handle_key(char, node.state):
                    if char.lower() == 'p' and _route_errors:
                        editor.message = '등록 제외 경로: ' + '; '.join(_route_errors)
                    continue
                if char.lower() == "q":
                    if node.state == 'CONVERTING':
                        node.capture_worker.join()
                    if node.state not in EDITABLE_STATES:
                        node.request_stop("사용자 종료")
                        while not node.should_close():
                            time.sleep(0.05)
                        node.close_processes(node.finish_reason)
                    quitting = True
                elif char.lower() == "s" or char == " ":
                    node.request_stop()
                elif char.lower() == "r":
                    node.request_run(editor.selection)
    except (KeyboardInterrupt, OSError):
        node.request_stop("터미널 종료")
        if node.launch_process is not None or node.recorder_process is not None:
            node.close_processes(node.finish_reason)
    finally:
        if node.capture_worker is not None and node.capture_worker.is_alive():
            node.capture_worker.join()
        if node.launch_process is not None or node.recorder_process is not None:
            node.close_processes("application_exit")
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        os.close(lock_fd)
    console.print("Decision 운용 종료" if operating_mode else f"기록: {node.run_dir or '생성되지 않음'}")
    return 0


def main(args: list[str] | None = None) -> int:
    options = sys.argv[1:] if args is None else args
    if options not in ([], ["--sim-time"]):
        Console().print(
            "사용법: ros2 run kaiev26_decision decision_test [--sim-time]\n"
            "--sim-time: Gazebo /clock을 쓰고 시뮬레이션 launch 경로로 실행 "
            "(ALL, 정지선+신호등, 장애물 회피)"
        )
        return 0 if options in (["-h"], ["--help"]) else 2
    return run_tui(
        command_hint="ros2 run kaiev26_decision decision_test",
        sim_time=options == ["--sim-time"],
    )


if __name__ == "__main__":
    raise SystemExit(main())
