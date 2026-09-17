"""ROS state and safety gates for the interactive cone-course trial."""

from collections import deque
import json
import math
from pathlib import Path
from queue import SimpleQueue
import threading
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseArray
from kaiev26_msgs.msg import ActuatorCommand, VehicleState
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage

from .trial_profiles import COUNTDOWN_SEC


TOPICS = {
    '/vehicle/state': VehicleState,
    '/ouster/points': PointCloud2,
    '/perception/camera/left/source/image_raw/compressed': CompressedImage,
    '/perception/camera/right/source/image_raw/compressed': CompressedImage,
    '/perception/camera/left/source/camera_info': CameraInfo,
    '/perception/camera/right/source/camera_info': CameraInfo,
    '/tf_static': TFMessage,
    '/aeb/status': String,
    '/aeb/center_path': PathMessage,
    '/aeb/red_gate': PoseArray,
    '/aeb/tracking_status': String,
    '/planning/command': ActuatorCommand,
    '/vehicle/command': ActuatorCommand,
}

BASE_EXTERNAL_REQUIRED = (
    '/vehicle/state',
    '/perception/camera/left/source/image_raw/compressed',
    '/perception/camera/right/source/image_raw/compressed',
    '/perception/camera/left/source/camera_info',
    '/perception/camera/right/source/camera_info', '/tf_static',
)

STATE_LABELS = {
    'IDLE': '시험 선택',
    'STARTING': '인지·제어 준비',
    'WAIT_AUTO': 'AUTO 전환 대기',
    'COUNTDOWN': '출발 카운트다운',
    'ENABLING': '추종 활성화',
    'RUNNING': '시험 주행 중',
    'STOPPING': '제동·기록 저장',
    'COMPLETE': '시험 종료',
    'ABORTED': '시험 중단',
    'ERROR': '오류 종료',
}

BUSY_STATES = {
    'STARTING', 'WAIT_AUTO', 'COUNTDOWN', 'ENABLING', 'RUNNING', 'STOPPING',
}


class TrialNode(Node):
    def __init__(self, launch_dir=None, clock=time.monotonic):
        super().__init__('cone_trial_tui')
        self.clock = clock
        self.launch_dir = Path(
            launch_dir or Path(get_package_share_directory('kaiev26_aeb')) / 'launch')
        self.lock = threading.RLock()
        self.commands = SimpleQueue()
        self.received = {}
        self.arrivals = {topic: deque(maxlen=200) for topic in TOPICS}
        self.json_status = {}
        self.logs = deque(maxlen=200)
        self.state = 'IDLE'
        self.settings = None
        self.manager = None
        self.run_dir = ''
        self.finish_reason = ''
        self.countdown = None
        self.phase_started = self.clock()
        self.auto_seen = False
        self.invalid_since = None
        self.stop_deadline = None
        self.manager_stop_requested = False
        self.terminal_target = 'COMPLETE'
        self.graph_scan = {
            'planning': [], 'vehicle': [], 'governor_connected': False,
            'recorder_connected': False,
        }
        self.subscriptions_owned = []
        for topic, message_type in TOPICS.items():
            qos = (QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                   if topic == '/tf_static' else qos_profile_sensor_data)
            self.subscriptions_owned.append(self.create_subscription(
                message_type, topic,
                lambda message, topic=topic: self._receive(topic, message), qos))
        self.enable_client = self.create_client(SetBool, '/aeb/set_tracking_enabled')
        self.create_timer(0.1, self._tick)
        self.create_timer(1.0, self._scan_graph)

    @property
    def busy(self):
        return self.state in BUSY_STATES

    def attach_manager(self, manager):
        self.manager = manager

    def log(self, message):
        line = time.strftime('%H:%M:%S') + ' ' + str(message)
        with self.lock:
            self.logs.append(line)

    def process_event(self, event):
        self.commands.put(event)

    def request_run(self, settings):
        self.commands.put(('run', settings))

    def request_stop(self, reason='사용자 Stop', terminal='ABORTED'):
        self.commands.put(('stop', (reason, terminal)))

    def _receive(self, topic, message):
        now = self.clock()
        with self.lock:
            self.received[topic] = (now, message)
            times = self.arrivals[topic]
            times.append(now)
            while times and times[0] < now - 2.0:
                times.popleft()
            if topic in ('/aeb/status', '/aeb/tracking_status'):
                try:
                    self.json_status[topic] = json.loads(message.data)
                except (TypeError, ValueError):
                    self.json_status[topic] = {'valid': False, 'reason': 'invalid_json'}

    def _scan_graph(self):
        planning = self.get_publishers_info_by_topic('/planning/command')
        vehicle = self.get_publishers_info_by_topic('/vehicle/command')
        subscribers = self.get_subscriptions_info_by_topic('/planning/command')
        connected = bool(vehicle) and any(
            item.node_name == vehicle[0].node_name
            and item.node_namespace == vehicle[0].node_namespace
            for item in subscribers)
        recorder_connected = all(any(
            item.node_name == 'rosbag2_recorder'
            for item in self.get_subscriptions_info_by_topic(topic))
            for topic in ('/vehicle/state', '/aeb/status', '/planning/command'))
        with self.lock:
            self.graph_scan = {
                'planning': [(p.node_name, p.node_namespace) for p in planning],
                'vehicle': [(p.node_name, p.node_namespace) for p in vehicle],
                'governor_connected': connected,
                'recorder_connected': recorder_connected,
            }

    def _topic_issue(self, topic, now, timeout=1.0):
        sample = self.received.get(topic)
        if sample is None:
            return f'{topic} 미수신'
        if topic != '/tf_static' and now - sample[0] > timeout:
            return f'{topic} {now - sample[0]:.1f}초 수신 중단'
        return ''

    def _vehicle_issue(self, now):
        issue = self._topic_issue('/vehicle/state', now)
        if issue:
            return issue
        state = self.received['/vehicle/state'][1]
        if state.estop_active:
            return '차량 E-stop 활성'
        if state.gear != VehicleState.GEAR_FORWARD:
            return '전진 기어가 아님'
        if not math.isfinite(state.speed_mps) or not math.isfinite(state.steering_rad):
            return '차량 상태 값이 유효하지 않음'
        return ''

    def availability(self, settings):
        return settings.profile.missing_launches(self.launch_dir)

    @staticmethod
    def external_required(settings):
        topics = list(BASE_EXTERNAL_REQUIRED)
        if settings.profile.perception == 'fusion':
            topics.insert(1, '/ouster/points')
        return tuple(topics)

    def command_flow_issue(self, active):
        graph = self.graph_scan
        planning = graph['planning']
        vehicle = graph['vehicle']
        if active:
            expected = self.settings.profile.controller_node if self.settings else ''
            if len(planning) != 1:
                return f'/planning/command 발행자 {len(planning)}개'
            if planning[0][0] != expected:
                return f'/planning/command의 발행자가 {planning[0][0]}임'
        elif planning:
            return '/planning/command에 기존 발행자가 있음'
        if len(vehicle) != 1:
            return f'/vehicle/command 발행자 {len(vehicle)}개'
        if not graph['governor_connected']:
            return '/vehicle/command 발행자가 /planning/command를 구독하지 않음'
        return ''

    def preflight_issues(self, settings, now=None):
        now = self.clock() if now is None else now
        issues = []
        missing = self.availability(settings)
        if missing:
            issues.append('미구현 launch: ' + ', '.join(missing))
        if not Path(settings.model_path).is_file():
            issues.append('YOLO 모델 파일 없음: ' + settings.model_path)
        for topic in self.external_required(settings):
            issue = self._topic_issue(topic, now)
            if issue:
                issues.append(issue)
        vehicle = self._vehicle_issue(now)
        if vehicle and vehicle not in issues:
            issues.append(vehicle)
        graph = self.command_flow_issue(active=False)
        if graph:
            issues.append(graph)
        return issues

    def pipeline_issues(self, now=None):
        now = self.clock() if now is None else now
        issues = []
        for topic in self.external_required(self.settings):
            issue = self._topic_issue(topic, now)
            if issue:
                issues.append(issue)
        vehicle = self._vehicle_issue(now)
        if vehicle and vehicle not in issues:
            issues.append(vehicle)
        graph = self.command_flow_issue(active=True)
        if graph:
            issues.append(graph)
        for topic in ('/aeb/status', '/aeb/center_path', '/aeb/tracking_status',
                      '/planning/command', '/vehicle/command'):
            issue = self._topic_issue(topic, now, timeout=0.8)
            if issue:
                issues.append(issue)
        perception = self.json_status.get('/aeb/status', {})
        if not perception.get('valid', False):
            issues.append('인지 경로: ' + str(perception.get('reason', '준비 대기')))
        path = self.received.get('/aeb/center_path')
        if path and len(path[1].poses) < 2:
            issues.append('중앙 경로점 부족')
        if not self.enable_client.service_is_ready():
            issues.append('추종 활성화 서비스 대기')
        if not self.graph_scan['recorder_connected']:
            issues.append('MCAP 필수 토픽 구독 대기')
        return issues

    def _call_enabled(self, enabled):
        if not self.enable_client.service_is_ready():
            self.commands.put(('service_result', (enabled, False, '서비스 미연결')))
            return
        request = SetBool.Request()
        request.data = enabled
        future = self.enable_client.call_async(request)

        def done(result):
            try:
                response = result.result()
                event = (enabled, bool(response.success), str(response.message))
            except Exception as error:
                event = (enabled, False, str(error))
            self.commands.put(('service_result', event))

        future.add_done_callback(done)

    def _begin_stop(self, reason, terminal):
        if self.state == 'STOPPING':
            return
        self.finish_reason = reason
        self.terminal_target = terminal
        self.state = 'STOPPING'
        self.stop_deadline = self.clock() + 0.5
        self.manager_stop_requested = False
        self.log(reason + ' · 제동 요청')
        self._call_enabled(False)

    def _actions(self, now):
        while not self.commands.empty():
            action, value = self.commands.get()
            if action == 'run':
                if self.busy:
                    self.log('현재 시험 종료 후 다시 실행하세요')
                    continue
                issues = self.preflight_issues(value, now)
                if issues:
                    self.log('Run NO-GO: ' + '; '.join(issues))
                    continue
                if self.manager is None:
                    self.log('프로세스 관리자가 연결되지 않았습니다')
                    continue
                self.settings = value
                self.run_dir = ''
                self.finish_reason = ''
                self.countdown = None
                self.auto_seen = False
                self.invalid_since = None
                self.manager_stop_requested = False
                for topic in ('/aeb/status', '/aeb/center_path', '/aeb/red_gate',
                              '/aeb/tracking_status', '/planning/command'):
                    self.received.pop(topic, None)
                    self.json_status.pop(topic, None)
                self.state = 'STARTING'
                self.phase_started = now
                self.manager.request_start(value)
                self.log(f'{value.profile.name} 파이프라인 시작')
            elif action == 'stop':
                reason, terminal = value
                if self.busy:
                    self._begin_stop(reason, terminal)
            elif action == 'process_started':
                self.run_dir = value
                self.log('MCAP 기록: ' + value)
            elif action == 'process_failed':
                self._begin_stop('프로세스 오류: ' + value, 'ERROR')
            elif action == 'process_stopped':
                self.state = self.terminal_target
                self.stop_deadline = None
                self.manager_stop_requested = False
                self.log('기록 저장 완료')
            elif action == 'service_result':
                enabled, success, message = value
                if enabled and self.state == 'ENABLING':
                    if success:
                        self.state = 'RUNNING'
                        self.phase_started = now
                        self.log('추종 활성화 · 시험 시작')
                    else:
                        self._begin_stop('추종 활성화 실패: ' + message, 'ERROR')
                elif not enabled and not success and self.state == 'STOPPING':
                    self.log('추종 비활성화 응답 실패: ' + message)

    def _tick(self):
        with self.lock:
            now = self.clock()
            self._actions(now)
            if self.state == 'STARTING':
                if not self.pipeline_issues(now):
                    self.state = 'WAIT_AUTO'
                    self.log('인지 경로·제어 명령 준비 완료 · AUTO 대기')
            if self.state in ('WAIT_AUTO', 'COUNTDOWN', 'ENABLING', 'RUNNING'):
                vehicle_issue = self._vehicle_issue(now)
                state = self.received.get('/vehicle/state')
                auto = bool(state and state[1].mode == VehicleState.MODE_AUTO)
                if self.auto_seen and (vehicle_issue or not auto):
                    self._begin_stop('MANUAL 전환 또는 차량 안전상태 변경', 'ABORTED')
                elif self.state == 'WAIT_AUTO' and not vehicle_issue and auto:
                    self.auto_seen = True
                    self.state = 'COUNTDOWN'
                    self.phase_started = now
                    self.log('AUTO 확인 · 5초 후 출발')
            if self.state == 'COUNTDOWN':
                issues = self.pipeline_issues(now)
                if issues:
                    self._begin_stop('출발 취소: ' + '; '.join(issues), 'ERROR')
                else:
                    remaining = max(0, math.ceil(COUNTDOWN_SEC - (now - self.phase_started)))
                    self.countdown = remaining
                    if now - self.phase_started >= COUNTDOWN_SEC:
                        self.state = 'ENABLING'
                        self.countdown = None
                        self._call_enabled(True)
            if self.state == 'RUNNING':
                issues = self.pipeline_issues(now)
                if issues:
                    if self.invalid_since is None:
                        self.invalid_since = now
                    elif now - self.invalid_since >= 1.0:
                        self._begin_stop('주행 입력 단절: ' + '; '.join(issues), 'ERROR')
                else:
                    self.invalid_since = None
            if (self.state == 'STOPPING' and self.stop_deadline is not None
                    and now >= self.stop_deadline and not self.manager_stop_requested):
                self.manager_stop_requested = True
                if self.manager:
                    self.manager.request_stop(self.finish_reason)

    def gate_rows(self, settings, now=None):
        now = self.clock() if now is None else now
        rows = []
        missing = self.availability(settings)
        rows.append(('시나리오 구현', not missing,
                     '실행 가능' if not missing else '미구현: ' + ', '.join(missing)))
        model_ok = Path(settings.model_path).is_file()
        rows.append(('YOLO 모델', model_ok,
                     settings.model_path if model_ok else '파일 없음: ' + settings.model_path))
        for topic in self.external_required(settings):
            issue = self._topic_issue(topic, now)
            rows.append((topic, not issue, issue or '정상'))
        graph = self.command_flow_issue(active=self.busy and self.state != 'STOPPING')
        rows.append(('명령 발행·중계', not graph, graph or '정상'))
        if self.busy:
            recording = self.graph_scan['recorder_connected']
            rows.append(('MCAP 연결', recording,
                         '필수 토픽 구독 완료' if recording else '구독 연결 대기'))
        return rows

    def snapshot(self, selection):
        with self.lock:
            now = self.clock()
            vehicle = self.received.get('/vehicle/state')
            fresh_vehicle = vehicle and now - vehicle[0] <= 1.0
            mode = ('미수신' if not fresh_vehicle else
                    'AUTO' if vehicle[1].mode == VehicleState.MODE_AUTO else 'MANUAL')
            speed = vehicle[1].speed_mps * 3.6 if fresh_vehicle else None
            perception = self.json_status.get('/aeb/status', {})
            tracking = self.json_status.get('/aeb/tracking_status', {})
            return {
                'state': self.state,
                'label': STATE_LABELS[self.state],
                'mode': mode,
                'speed': speed,
                'countdown': self.countdown,
                'gates': self.gate_rows(selection, now),
                'perception_reason': perception.get('reason', '미수신'),
                'path_points': perception.get('path_points', 0),
                'tracking_reason': tracking.get('reason', '미수신'),
                'red_state': tracking.get('red_state', '미수신'),
                'logs': list(self.logs)[-7:],
                'run_dir': self.run_dir,
                'finish_reason': self.finish_reason,
            }
