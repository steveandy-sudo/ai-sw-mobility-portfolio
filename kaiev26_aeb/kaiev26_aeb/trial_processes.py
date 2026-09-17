"""Own one cone-trial launch and its MCAP recorder."""

from __future__ import annotations

from datetime import datetime
import json
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
from zoneinfo import ZoneInfo

import yaml


DEFAULT_RECORDS_ROOT = next((
    ancestor / 'records' / 'cone_trials'
    for ancestor in Path(__file__).resolve().parents
    if (ancestor / 'src/Decision/kaiev26_aeb/package.xml').is_file()
), Path.home() / 'KAI_ws/records/cone_trials')

RECORD_TOPICS = (
    '/planning/command', '/vehicle/command', '/vehicle/state',
    '/drive/status', '/steering/status', '/steering/angle', '/brake/status',
    '/aeb/center_path', '/aeb/cones', '/aeb/boundaries', '/aeb/status',
    '/aeb/red_gate', '/aeb/control_command', '/aeb/tracking_status',
    '/aeb/pursuit_target', '/aeb/predicted_path',
    '/aeb/left/image/compressed', '/aeb/right/image/compressed', '/aeb/points',
    '/perception/camera/left/source/camera_info',
    '/perception/camera/left/source/image_raw/compressed',
    '/perception/camera/right/source/camera_info',
    '/perception/camera/right/source/image_raw/compressed',
    '/ouster/imu_packets', '/ouster/lidar_packets', '/ouster/metadata',
    '/tf', '/tf_static', '/diagnostics',
)

RECORDER_CODE = """
import json, sys
import rosbag2_py
storage = rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap')
options = rosbag2_py.RecordOptions()
options.topics = json.loads(sys.argv[2])
options.rmw_serialization_format = 'cdr'
try:
    rosbag2_py.Recorder().record(storage, options)
except KeyboardInterrupt:
    pass
"""


class OwnedProcess:
    def __init__(self, name: str, argv: list[str], emit):
        self.name = name
        self.emit = emit
        self.last_output = ''
        self.process = subprocess.Popen(
            argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, encoding='utf-8',
            errors='replace', start_new_session=True, bufsize=1)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        for line in self.process.stdout:
            self.last_output = line.rstrip()
            self.emit(f'{self.name}: {self.last_output}')
        self.process.stdout.close()

    def stop(self) -> int:
        for sig, timeout in ((signal.SIGINT, 5.0), (signal.SIGTERM, 2.0),
                             (signal.SIGKILL, 1.0)):
            if self.process.poll() is not None:
                break
            try:
                os.killpg(self.process.pid, sig)
            except ProcessLookupError:
                break
            try:
                self.process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                continue
        self.reader.join(timeout=1.0)
        return self.process.returncode if self.process.returncode is not None else -1


class ProcessManager:
    """Serialize subprocess lifecycle work away from the TUI/ROS executor."""

    def __init__(self, event_sink, log_sink, records_root=DEFAULT_RECORDS_ROOT):
        self.event_sink = event_sink
        self.log_sink = log_sink
        self.records_root = Path(records_root)
        self.actions = queue.SimpleQueue()
        self.stop_event = threading.Event()
        self.launch = None
        self.recorder = None
        self.settings = None
        self.run_dir = None
        self.document = None
        self.log_file = None
        self.cleanup_errors = []
        self.thread = threading.Thread(target=self._work, name='cone-trial-processes')

    @staticmethod
    def launch_argv(settings) -> list[str]:
        return [
            'ros2', 'launch', 'kaiev26_aeb', 'aeb_test.launch.py',
            f'scenario:={settings.scenario}', 'use_sim_time:=false',
            'enabled:=false', 'command_topic:=/planning/command',
            f'target_speed_kph:={settings.target_speed_kph:g}',
            f'model_path:={settings.model_path}',
        ]

    def start(self):
        self.thread.start()

    def request_start(self, settings):
        self.actions.put(('start', settings))

    def request_stop(self, reason):
        self.actions.put(('stop', str(reason)))

    def _log(self, message):
        self.log_sink(message)
        if self.log_file:
            self.log_file.write(
                datetime.now().astimezone().isoformat(timespec='milliseconds')
                + ' ' + message + '\n')

    def _save_document(self):
        if self.run_dir and self.document is not None:
            (self.run_dir / 'scenario.yaml').write_text(
                yaml.safe_dump(self.document, sort_keys=False, allow_unicode=True),
                encoding='utf-8')

    def _start_run(self, settings):
        if self.launch or self.recorder:
            raise RuntimeError('이미 실행 중인 콘 시험 프로세스가 있습니다')
        stamp = datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S_%f')
        speed = f'{settings.target_speed_kph:g}'.replace('.', 'p')
        self.run_dir = self.records_root / (
            f'{stamp}_cone_scenario_{settings.scenario:02d}_v{speed}')
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.log_file = (self.run_dir / 'console.log').open(
            'a', encoding='utf-8', buffering=1)
        self.settings = settings
        self.document = settings.document()
        self.document.update({
            'started_at': datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),
            'topics': list(RECORD_TOPICS),
        })
        self._save_document()
        self.recorder = OwnedProcess('MCAP', [
            sys.executable, '-u', '-c', RECORDER_CODE,
            str(self.run_dir / 'rosbag'), json.dumps(RECORD_TOPICS),
        ], self._log)
        try:
            self.launch = OwnedProcess('주행 파이프라인', self.launch_argv(settings), self._log)
        except Exception:
            self.recorder.stop()
            self.recorder = None
            raise
        self._log(f'{settings.profile.name} 준비 · 최고 목표 {settings.target_speed_kph:g} km/h')
        self.event_sink(('process_started', str(self.run_dir)))

    def _metadata(self):
        if not self.run_dir:
            return
        try:
            import rosbag2_py
            metadata = rosbag2_py.Info().read_metadata(str(self.run_dir / 'rosbag'), 'mcap')
            self.document['recorded_messages'] = {
                item.topic_metadata.name: item.message_count
                for item in metadata.topics_with_message_count
            }
        except Exception as error:
            self.document['recording_error'] = str(error)

    def _stop_run(self, reason):
        launch_code = self.launch.stop() if self.launch else None
        self.launch = None
        recorder_code = self.recorder.stop() if self.recorder else None
        self.recorder = None
        if self.document is not None:
            self.document.update({
                'finished_at': datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),
                'finish_reason': reason,
                'launch_exit_code': launch_code,
                'recorder_exit_code': recorder_code,
            })
            self._metadata()
            self._save_document()
        if self.log_file:
            self.log_file.close()
            self.log_file = None
        self.event_sink(('process_stopped', reason))

    def _unexpected_exit(self):
        if self.launch and self.launch.process.poll() is not None:
            detail = self.launch.last_output or f'exit code {self.launch.process.returncode}'
            self._log(f'주행 파이프라인 비정상 종료: {detail}')
            self.event_sink(('process_failed', detail))
            self._stop_run('pipeline_failed')
        elif self.recorder and self.recorder.process.poll() is not None:
            detail = self.recorder.last_output or f'exit code {self.recorder.process.returncode}'
            self._log(f'MCAP 비정상 종료: {detail}')
            self.event_sink(('process_failed', detail))
            self._stop_run('recorder_failed')

    def _work(self):
        try:
            while not self.stop_event.wait(0.1):
                while not self.actions.empty():
                    action, value = self.actions.get()
                    try:
                        if action == 'start':
                            self._start_run(value)
                        elif action == 'stop':
                            self._stop_run(value)
                    except Exception as error:
                        self._log(f'프로세스 관리 오류: {error}')
                        self.event_sink(('process_failed', str(error)))
                        try:
                            self._stop_run('process_error')
                        except Exception as cleanup_error:
                            self.cleanup_errors.append(str(cleanup_error))
                self._unexpected_exit()
        finally:
            if self.launch or self.recorder:
                try:
                    self._stop_run('application_exit')
                except Exception as error:
                    self.cleanup_errors.append(str(error))

    def close(self):
        self.stop_event.set()
        if self.thread.ident is not None:
            self.thread.join()
        return list(self.cleanup_errors)
