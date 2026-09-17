"""Field Decision test selector, safety gate, and one-run MCAP recorder."""

from __future__ import annotations

from collections import deque
from contextlib import contextmanager
from dataclasses import asdict, dataclass, replace
from datetime import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import select
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
from kaiev26_msgs.msg import (
    ActuatorCommand,
    BrakeActuatorStatus,
    DriveActuatorStatus,
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
from rich.console import Console
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String
from um980_msgs.msg import Status as GnssStatus
import yaml


CASE_NAMES = {
    1: "예선 - 전역경로 추종",
    2: "예선 - 추종 + 정지선",
    3: "예선 - ALL",
    4: "본선 - 전역경로 추종",
    5: "본선 - 추종 + 정지선",
    6: "본선 - 추종 + 장애물 회피",
    7: "본선 - ALL",
    8: "조향 스텝 - 모터 토크 데이터",
}
CONTROLLER_NAMES = {
    "pure_pursuit": "Pure Pursuit",
    "stanley": "Stanley",
    "pp_stanley": "PP + Stanley",
    "ff_stanley": "FF + Stanley",
}
IMPLEMENTED_CONTROLLERS = {
    "pure_pursuit",
    "stanley",
    "pp_stanley",
    "ff_stanley",
}
TRACKING_CASES = {1, 4}
COURSE_DEFAULT_SPEED_MPS = {
    "qualifying": 3.2,
    "final": 3.5,
    "steering_step": 1.0,
}
STEERING_STEP_CASE = 8
REQUIRED_EXTERNAL_TOPICS = (
    "/gnss/fix",
    "/gnss/fix_velocity",
    "/localization/odometry",
    "/vehicle/state",
    "/drive/status",
    "/steering/status",
    "/brake/status",
)
STEERING_STEP_REQUIRED_TOPICS = (
    "/vehicle/state",
    "/drive/status",
    "/steering/status",
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
STEERING_STEP_RECORD_TOPICS = (
    "/planning/command",
    "/decision/raw_command",
    "/vehicle/command",
    "/decision/scenario",
    "/decision/test_run_active",
    "/decision/steering_step_state",
    "/drive/status",
    "/steering/status",
    "/steering/angle",
    "/steering/enable_commanded",
    "/steering/enabled",
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
    "/stm_a/runtime_stats",
    "/stm_b/runtime_stats",
    "/tf",
    "/tf_static",
    "/diagnostics",
)
RECORDER_CODE = """
import fcntl, json, os, sys
import rosbag2_py
lock_path = f'/tmp/kaiev26_mcap_{os.environ.get("ROS_DOMAIN_ID", "0")}.lock'
lock_file = open(lock_path, 'w')
try:
    fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError:
    raise RuntimeError(f'another MCAP recorder holds {lock_path}')
storage = rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap')
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
    case: int = 1
    speed_mps: float = 0.0
    controller: str = "pure_pursuit"
    steering_amplitude_deg: float = 3.0
    steering_frequency_hz: float = 0.25
    steering_pattern: str = "RLR"
    steering_initial_straight_s: float = 3.0

    def __post_init__(self) -> None:
        if self.case not in CASE_NAMES:
            raise ValueError("case는 1부터 8까지입니다")
        if not math.isfinite(self.speed_mps) or not (
            self.speed_mps == 0.0 or 0.1 <= self.speed_mps <= 15.0
        ):
            raise ValueError("속도는 코스 기본값 0 또는 0.1~15.0 m/s여야 합니다")
        if self.controller not in CONTROLLER_NAMES:
            raise ValueError("지원하지 않는 횡제어기입니다")

        if not math.isfinite(self.steering_amplitude_deg) or not (
            0.0 <= self.steering_amplitude_deg <= 25.0
        ):
            raise ValueError("steering amplitude must be 0..25 deg")
        if not math.isfinite(self.steering_frequency_hz) or not (
            0.01 <= self.steering_frequency_hz <= 10.0
        ):
            raise ValueError("steering frequency must be 0.01..10 Hz")
        pattern = "".join(self.steering_pattern.upper().split())
        if not pattern or any(direction not in {"R", "L"} for direction in pattern):
            raise ValueError("steering pattern must contain only R and L")
        object.__setattr__(self, "steering_pattern", pattern)
        if not math.isfinite(self.steering_initial_straight_s) or (
            self.steering_initial_straight_s < 0.0
        ):
            raise ValueError("initial straight duration must be non-negative")

    @property
    def course(self) -> str:
        if self.case == STEERING_STEP_CASE:
            return "steering_step"
        return "qualifying" if self.case <= 3 else "final"

    @property
    def mode(self) -> str:
        return {
            1: "tracking",
            2: "stopline",
            3: "all",
            4: "tracking",
            5: "stopline",
            6: "avoidance",
            7: "all",
            8: "steering_step",
        }[self.case]

    @property
    def implemented(self) -> bool:
        return self.case == STEERING_STEP_CASE or self.controller in IMPLEMENTED_CONTROLLERS

    @property
    def effective_speed_mps(self) -> float:
        if self.speed_mps > 0.0:
            return self.speed_mps
        return COURSE_DEFAULT_SPEED_MPS[self.course]

    @property
    def record_topics(self) -> tuple[str, ...]:
        return (
            STEERING_STEP_RECORD_TOPICS
            if self.case == STEERING_STEP_CASE
            else RECORD_TOPICS
        )

    def document(self) -> dict:
        value = asdict(self)
        step_configuration = {
            "amplitude_deg": value.pop("steering_amplitude_deg"),
            "frequency_hz": value.pop("steering_frequency_hz"),
            "pattern": value.pop("steering_pattern"),
            "initial_straight_s": value.pop("steering_initial_straight_s"),
            "pulse_duty_ratio": 0.5,
        }
        value["requested_speed_mps"] = value.pop("speed_mps")
        value.update(
            {
                "name": CASE_NAMES[self.case],
                "course": self.course,
                "mode": self.mode,
                "speed_mps": self.effective_speed_mps,
                "speed_source": (
                    "launch_argument" if self.speed_mps > 0.0 else "course_default"
                ),
                "speed_control": (
                    "constant_speed_and_open_loop_steering_step"
                    if self.case == STEERING_STEP_CASE
                    else "adaptive_curvature_ceiling"
                    if self.case in TRACKING_CASES
                    else "decision_policy_and_curvature_ceiling"
                ),
                "record_topics": list(self.record_topics),
            }
        )
        if self.case == STEERING_STEP_CASE:
            value["steering_step"] = step_configuration
        return value


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
    def __init__(self) -> None:
        super().__init__("decision_test")
        self.lock = threading.RLock()
        self.logs = deque(maxlen=100)
        self.received: dict[str, tuple[float, object]] = {}
        self.settings = TestSettings()
        self.state = "IDLE"
        self.run_dir = ""
        self.finish_reason = ""
        self.requested_terminal = "ABORTED"
        self.launch_process: OwnedProcess | None = None
        self.recorder_process: OwnedProcess | None = None
        self.stop_started = 0.0
        self.ready_since: float | None = None
        self.auto_since: float | None = None
        self.finish_since: float | None = None
        self.graph_issues: list[str] = ["ROS 그래프 확인 중"]

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
            "/planning/route_ready": Bool,
            "/planning/readiness_status": DiagnosticArray,
            "/planning/scene_summary": SceneSummary,
            "/decision/raw_command": ActuatorCommand,
            "/vehicle/command": ActuatorCommand,
            "/debug/active_behavior": String,
            "/debug/fsm_state": String,
            "/decision/steering_step_state": String,
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

    def scan_graph(self) -> None:
        issues = []
        planning_publishers = self.get_publishers_info_by_topic("/planning/command")
        if len(planning_publishers) != 1 or planning_publishers[0].node_name != self.get_name():
            issues.append("/planning/command에 다른 발행자가 있음")
        vehicle_publishers = self.get_publishers_info_by_topic("/vehicle/command")
        if len(vehicle_publishers) != 1:
            issues.append("/vehicle/command 발행자는 정확히 1개여야 함")
        if self.settings.case != STEERING_STEP_CASE:
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
        if recorder_nodes and self.recorder_process is None:
            issues.append("다른 rosbag2_recorder가 실행 중")
        with self.lock:
            self.graph_issues = issues

    def external_issues(self) -> list[str]:
        issues = list(self.graph_issues)
        required_topics = (
            STEERING_STEP_REQUIRED_TOPICS
            if self.settings.case == STEERING_STEP_CASE
            else REQUIRED_EXTERNAL_TOPICS
        )
        for topic in required_topics:
            if not self.fresh(topic):
                issues.append(f"{topic} 미수신")
        vehicle = self.message("/vehicle/state")
        if vehicle is not None:
            if vehicle.mode != VehicleState.MODE_MANUAL:
                issues.append("Run 전 차량을 MANUAL로 전환")
            if vehicle.estop_active:
                issues.append("물리 E-Stop 해제 필요")
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
        if not settings.implemented:
            self.log(
                f"{CONTROLLER_NAMES[settings.controller]} "
                "제어기는 구현되어 있지 않습니다"
            )
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
        self.auto_since = None
        self.finish_since = None
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
        self.state = "PREFLIGHT"
        self.log(
            f"{CASE_NAMES[settings.case]} 시작, "
            + (
                f"pattern={settings.steering_pattern}, "
                f"amplitude={settings.steering_amplitude_deg:g} deg, "
                f"frequency={settings.steering_frequency_hz:g} Hz"
                if settings.case == STEERING_STEP_CASE
                else f"controller={settings.controller}"
            )
        )

    def start_processes(self) -> None:
        stamp = datetime.now(ZoneInfo("Asia/Seoul")).strftime(
            "%Y%m%d_%H%M%S_%f"
        )
        speed = f"{self.settings.effective_speed_mps:g}".replace(".", "p")
        if self.settings.case == STEERING_STEP_CASE:
            amplitude = f"{self.settings.steering_amplitude_deg:g}".replace(".", "p")
            frequency = f"{self.settings.steering_frequency_hz:g}".replace(".", "p")
            run_name = (
                f"{stamp}_decision_case_08_steering_step_v{speed}_"
                f"a{amplitude}_f{frequency}_{self.settings.steering_pattern}"
            )
        else:
            run_name = (
                f"{stamp}_decision_case_{self.settings.case:02d}_"
                f"{self.settings.controller}_v{speed}"
            )
        run_dir = (
            Path.home()
            / "KAI_ws"
            / "records"
            / run_name
        )
        run_dir.mkdir(parents=True, exist_ok=False)
        self.run_dir = str(run_dir)
        document = self.settings.document()
        document["started_at"] = datetime.now(ZoneInfo("Asia/Seoul")).isoformat()
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
        launch_arguments = [
            "ros2",
            "launch",
            "kaiev26_decision",
            "decision_test.launch.py",
            f"case:={self.settings.case}",
            f"speed_mps:={self.settings.effective_speed_mps:g}",
            "platform:=vehicle",
            "command_topic:=/decision/raw_command",
        ]
        if self.settings.case == STEERING_STEP_CASE:
            launch_arguments.extend(
                [
                    f"steering_amplitude_deg:={self.settings.steering_amplitude_deg:g}",
                    f"steering_frequency_hz:={self.settings.steering_frequency_hz:g}",
                    f"steering_pattern:={self.settings.steering_pattern}",
                    f"steering_initial_straight_s:={self.settings.steering_initial_straight_s:g}",
                    "step_wait_for_run_enable:=true",
                ]
            )
        else:
            launch_arguments.append(
                f"lateral_controller:={self.settings.controller}"
            )
        self.launch_process = OwnedProcess(
            "Decision",
            launch_arguments,
            self.log,
        )

    def request_stop(self, reason: str = "사용자 Stop", terminal: str = "ABORTED") -> None:
        if self.state in {"IDLE", "COMPLETE", "ABORTED", "ERROR", "CLOSING"}:
            return
        self.finish_reason = reason
        self.requested_terminal = terminal
        self.state = "STOPPING"
        self.stop_started = self.now()
        self.log(reason)

    def tick(self) -> None:
        with self.lock:
            now = self.now()
            vehicle = self.message("/vehicle/state")
            ready_msg = self.message("/planning/route_ready")
            route_ready = (
                self.fresh('/planning/route_ready') and bool(ready_msg.data)
            )
            perception_ready = (
                self.settings.mode not in {"all", "avoidance"}
                or self.fresh("/planning/scene_summary", 0.5)
            )
            step_case = self.settings.case == STEERING_STEP_CASE
            pipeline_ready = (
                self.fresh("/decision/raw_command", 0.25)
                if step_case
                else route_ready and perception_ready
            )
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

            if self.state == "WAIT_AUTO" and auto:
                self.state = "COUNTDOWN"
                self.auto_since = now
                self.log("AUTO 확인, 3초 후 판단 명령 전달")
            elif self.state == "COUNTDOWN" and not auto:
                self.state = "WAIT_AUTO"
                self.auto_since = None
                self.log("MANUAL 복귀, AUTO 대기")
            elif (
                self.state == "COUNTDOWN"
                and self.auto_since is not None
                and now - self.auto_since >= 3.0
            ):
                self.state = "RUNNING"
                self.log("RUNNING")

            if self.state == "RUNNING":
                if not auto:
                    self.request_stop("MANUAL 전환으로 Run 종료")
                elif not pipeline_ready:
                    self.request_stop('; '.join(self.readiness_blockers()), "ERROR")
                elif not self.fresh("/decision/raw_command", 0.25):
                    self.request_stop("판단 명령 수신 중단", "ERROR")
                else:
                    route = self.message("/planning/route_context")
                    raw = self.message("/decision/raw_command")
                    stopped = abs(float(vehicle.speed_mps)) <= 0.1
                    finished = (
                        not step_case
                        and route is not None
                        and route.distance_to_finish <= 1.0
                        and raw is not None
                        and raw.brake_engage
                        and stopped
                    )
                    if finished:
                        if self.finish_since is None:
                            self.finish_since = now
                        elif now - self.finish_since >= 1.0:
                            self.request_stop("경로 종료점 정지 완료", "COMPLETE")
                    else:
                        self.finish_since = None

            if self.launch_process is not None and self.launch_process.process.poll() is not None:
                self.request_stop("Decision launch가 예기치 않게 종료됨", "ERROR")
            if (
                self.recorder_process is not None
                and self.recorder_process.process.poll() is not None
            ):
                self.request_stop("MCAP recorder가 예기치 않게 종료됨", "ERROR")

            self.publish_run_active()
            self.publish_command(pipeline_ready, auto)
            self.publish_label()

    def publish_run_active(self) -> None:
        message = Bool()
        message.data = self.state == "RUNNING"
        self.run_active_pub.publish(message)

    def publish_command(self, route_ready: bool, auto: bool) -> None:
        command = ActuatorCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        raw = self.message("/decision/raw_command")
        vehicle = self.message("/vehicle/state")
        can_relay = (
            self.state == "RUNNING"
            and route_ready
            and auto
            and raw is not None
            and self.fresh("/decision/raw_command", 0.25)
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
            command.brake_engage = bool(auto and self.state != "IDLE")
        self.command_pub.publish(command)

    def publish_label(self) -> None:
        label = String()
        if self.settings.case == STEERING_STEP_CASE:
            label.data = (
                f"case=8 mode=steering_step speed={self.settings.effective_speed_mps:g} "
                f"amplitude_deg={self.settings.steering_amplitude_deg:g} "
                f"frequency_hz={self.settings.steering_frequency_hz:g} "
                f"pattern={self.settings.steering_pattern} state={self.state}"
            )
        else:
            label.data = (
                f"case={self.settings.case} course={self.settings.course} "
                f"mode={self.settings.mode} controller={self.settings.controller} "
                f"state={self.state}"
            )
        self.scenario_pub.publish(label)

    def should_close(self) -> bool:
        return self.state == "STOPPING" and self.now() - self.stop_started >= 2.0

    def close_processes(self, reason: str) -> None:
        self.state = "CLOSING"
        codes = {}
        for name in ("launch_process", "recorder_process"):
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
                    }
                )
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
        self.log(f"MCAP 저장 완료: {self.run_dir}")

    def readiness_rows(self) -> list[tuple[str, str, str, str]]:
        if self.settings.case == STEERING_STEP_CASE:
            rows = []
            for topic in STEERING_STEP_REQUIRED_TOPICS:
                fresh = self.fresh(topic)
                state = "GOOD" if fresh else "WAIT"
                detail = (
                    "스텝 시험 입력 정상"
                    if fresh
                    else "스텝 시험 입력 미수신"
                )
                if topic == "/steering/status" and fresh:
                    steering = self.message(topic)
                    if not steering.communication_ok or not steering.feedback_valid:
                        state = "FAULT"
                        detail = "KEYA 조향 통신/피드백 확인"
                rows.append((topic, "수신" if fresh else "미수신", state, detail))
            return rows
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
        if self.settings.mode in {'all', 'avoidance'}:
            perception_ready = self.fresh('/planning/scene_summary', 0.5)
            rows.append((
                '/planning/scene_summary', '수신' if perception_ready else '미수신',
                'GOOD' if perception_ready else 'WAIT',
                '인지 장면 수신' if perception_ready else '인지 장면 미수신: 허용 수신 간격 0.5초',
            ))
        return rows

    def readiness_blockers(self) -> list[str]:
        if self.settings.case == STEERING_STEP_CASE:
            reasons = [
                f"{READINESS_LABELS.get(topic, topic)}: {detail}"
                for topic, _reception, state, detail in self.readiness_rows()
                if state != "GOOD"
            ]
            if not self.fresh("/decision/raw_command", 0.25):
                reasons.append("조향 스텝 명령 미수신")
            return reasons or ["조향 스텝 시작 대기"]
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
        if self.settings.case == STEERING_STEP_CASE:
            return "Case 8은 GNSS·Localization·전역경로를 사용하지 않음"
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
            ready = self.message("/planning/route_ready")
            behavior = self.message("/debug/active_behavior")
            fsm = self.message("/debug/fsm_state")
            step_state = {}
            step_message = self.message("/decision/steering_step_state")
            if step_message is not None:
                try:
                    step_state = json.loads(step_message.data)
                except (TypeError, ValueError, json.JSONDecodeError):
                    step_state = {}
            return {
                "state": self.state,
                "speed": float(vehicle.speed_mps) if vehicle is not None else None,
                "mode": (
                    "AUTO"
                    if vehicle is not None and vehicle.mode == VehicleState.MODE_AUTO
                    else "MANUAL"
                    if vehicle is not None
                    else "WAIT"
                ),
                "ready": (
                    self.fresh("/decision/raw_command", 0.25)
                    if self.settings.case == STEERING_STEP_CASE
                    else bool(ready.data)
                    if self.fresh('/planning/route_ready')
                    and (
                        self.settings.mode not in {"all", "avoidance"}
                        or self.fresh("/planning/scene_summary", 0.5)
                    )
                    else False
                ),
                "progress": float(route.progress_s) if route is not None else None,
                "cte": float(route.cross_track_error) if route is not None else None,
                "heading": (
                    math.degrees(float(route.heading_error)) if route is not None else None
                ),
                "zone": route.current_zone if route is not None else "-",
                "behavior": behavior.data if behavior is not None else "-",
                "fsm": fsm.data if fsm is not None else "-",
                "step_state": step_state,
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
) -> Table:
    root = Table.grid(expand=True)
    status = Table.grid(expand=True)
    status.add_column(ratio=1)
    status.add_column(ratio=1)
    if selection.case == STEERING_STEP_CASE:
        step = snapshot["step_state"]
        route_text = (
            f"step={step.get('phase', 'WAIT')}  "
            f"target={step.get('steering_deg', 0.0):+.2f} deg  "
            f"cycle={step.get('pattern_cycle', 0)}"
        )
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
            f"speed {speed_text} m/s\nS 중지·저장 | Q 종료"
        ),
        Text(
            f"preflight {'READY' if snapshot['ready'] else 'WAIT'}\n{route_text}",
            style="green" if snapshot["ready"] else "yellow",
        ),
    )
    root.add_row(Panel(status, title="Decision Field Test"))
    if snapshot['finish_reason']:
        root.add_row(Text('종료 사유: ' + snapshot['finish_reason'], style='yellow'))

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
        root.add_row(Text('기록 경고: ' + '; '.join(snapshot['warnings']), style='yellow'))

    choices = Table(expand=True)
    choices.add_column("Key", width=4)
    choices.add_column("Case")
    for case, name in CASE_NAMES.items():
        marker = ">" if selection.case == case else " "
        choices.add_row(str(case), f"{marker} {name}")
    controller = CONTROLLER_NAMES[selection.controller]
    speed_kind = (
        "상수 목표속도"
        if selection.case == STEERING_STEP_CASE
        else "직선 목표속도"
        if selection.case in TRACKING_CASES
        else "순항 상한"
    )
    speed_source = "코스 기본" if selection.speed_mps == 0.0 else "직접 지정"
    speed_label = f"{selection.effective_speed_mps:g} m/s ({speed_source})"
    if selection.case != STEERING_STEP_CASE:
        choices.add_row("L", f"횡제어기: {controller}")
    choices.add_row("V", f"{speed_kind}: {speed_label}")
    if selection.case == STEERING_STEP_CASE:
        choices.add_row("A", f"조향 진폭: {selection.steering_amplitude_deg:g} deg")
        choices.add_row("F", f"스텝 주파수: {selection.steering_frequency_hz:g} Hz")
        choices.add_row("P", f"방향 패턴: {selection.steering_pattern}")
    choices.add_row("R", "Run")
    choices.add_row("S", "Stop and save")
    choices.add_row("Q", "Quit")
    if edit is not None:
        edit_field, edit_value = edit
        edit_labels = {
            "speed_mps": "속도",
            "steering_amplitude_deg": "조향 진폭(deg)",
            "steering_frequency_hz": "스텝 주파수(Hz)",
            "steering_pattern": "방향 패턴(R/L 연속 입력)",
        }
        choices.add_row(
            "",
            f"{edit_labels[edit_field]} 입력: {edit_value}_  "
            "(Enter 적용 / Esc 취소 / Backspace 삭제)",
        )
    if message:
        choices.add_row("", message)
    root.add_row(Panel(choices, title="Case Select"))

    root.add_row(
        Panel(
            Text(
                f"behavior: {snapshot['behavior']}\n"
                f"fsm: {snapshot['fsm']}\n"
                f"record: {snapshot['run_dir'] or '~/KAI_ws/records'}\n"
                + "\n".join(snapshot["logs"])
            ),
            title="Runtime / MCAP",
        )
    )
    return root


def main(args: list[str] | None = None) -> int:
    del args
    console = Console()
    if not sys.stdin.isatty():
        console.print(
            "실제 터미널에서 ros2 run kaiev26_decision decision_test를 실행하세요."
        )
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
    node = DecisionTestNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    selection = TestSettings()
    edit_field: str | None = None
    edit_value = ""
    message = ""
    quitting = False
    try:
        with keyboard(), Live(console=console, screen=True, auto_refresh=False) as live:
            while not quitting:
                if node.should_close():
                    node.close_processes(node.finish_reason or "Run 종료")
                if node.state in {"IDLE", "COMPLETE", "ABORTED", "ERROR"}:
                    with node.lock:
                        node.settings = selection
                live.update(
                    render(
                        node.snapshot(),
                        selection,
                        (edit_field, edit_value) if edit_field is not None else None,
                        message,
                    ),
                    refresh=True,
                )
                readable, _, _ = select.select([sys.stdin], [], [], 0.15)
                if not readable:
                    continue
                raw = os.read(sys.stdin.fileno(), 1)
                if not raw:
                    break
                char = raw.decode("ascii", errors="ignore")
                if edit_field is not None:
                    if char in "\r\n":
                        try:
                            value = (
                                edit_value
                                if edit_field == "steering_pattern"
                                else float(edit_value)
                            )
                            selection = replace(selection, **{edit_field: value})
                            message = ""
                            edit_field = None
                            edit_value = ""
                        except ValueError as error:
                            message = str(error)
                    elif char == "\x1b":
                        edit_field = None
                        edit_value = ""
                    elif char in ("\x7f", "\b"):
                        edit_value = edit_value[:-1]
                    elif edit_field == "steering_pattern" and char.upper() in {"R", "L"}:
                        edit_value += char.upper()
                    elif edit_field != "steering_pattern" and char in "0123456789.":
                        edit_value += char
                    continue
                if char.lower() == "q":
                    if node.state not in {"IDLE", "COMPLETE", "ABORTED", "ERROR"}:
                        node.request_stop("사용자 종료")
                        while not node.should_close():
                            time.sleep(0.05)
                        node.close_processes(node.finish_reason)
                    quitting = True
                elif char.lower() == "s" or char == " ":
                    node.request_stop()
                elif char in "12345678" and node.state in {
                    "IDLE",
                    "COMPLETE",
                    "ABORTED",
                    "ERROR",
                }:
                    selection = replace(selection, case=int(char))
                    message = ""
                elif char.lower() == "l" and node.state in {
                    "IDLE",
                    "COMPLETE",
                    "ABORTED",
                    "ERROR",
                } and selection.case != STEERING_STEP_CASE:
                    values = list(CONTROLLER_NAMES)
                    selected = values[(values.index(selection.controller) + 1) % len(values)]
                    selection = replace(selection, controller=selected)
                    message = ""
                elif char.lower() == "v" and node.state in {
                    "IDLE",
                    "COMPLETE",
                    "ABORTED",
                    "ERROR",
                }:
                    edit_field = "speed_mps"
                    edit_value = ""
                elif (
                    char.lower() in {"a", "f", "p"}
                    and selection.case == STEERING_STEP_CASE
                    and node.state in {"IDLE", "COMPLETE", "ABORTED", "ERROR"}
                ):
                    edit_field = {
                        "a": "steering_amplitude_deg",
                        "f": "steering_frequency_hz",
                        "p": "steering_pattern",
                    }[char.lower()]
                    edit_value = ""
                elif char.lower() == "r":
                    node.request_run(selection)
    except (KeyboardInterrupt, OSError):
        node.request_stop("터미널 종료")
        if node.launch_process is not None or node.recorder_process is not None:
            node.close_processes(node.finish_reason)
    finally:
        if node.launch_process is not None or node.recorder_process is not None:
            node.close_processes("application_exit")
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        os.close(lock_fd)
    console.print(f"기록: {node.run_dir or '생성되지 않음'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
