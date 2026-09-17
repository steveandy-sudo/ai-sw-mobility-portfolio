"""Rich terminal UI for the four K-City cone-course experiments."""

import argparse
from contextlib import contextmanager
import fcntl
import os
from pathlib import Path
import select
import signal
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rich.console import Console
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from .trial_processes import DEFAULT_RECORDS_ROOT, ProcessManager
from .trial_profiles import PROFILES, TrialSettings, default_model_path
from .trial_runtime import TrialNode


@contextmanager
def keyboard():
    original = termios.tcgetattr(sys.stdin.fileno())
    try:
        tty.setcbreak(sys.stdin.fileno())
        yield
    finally:
        try:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, original)
        except (termios.error, OSError):
            pass


def _dashboard(snapshot, selection, launch_dir, speed_text=None, message='',
               records_root=DEFAULT_RECORDS_ROOT):
    speed = '—' if snapshot['speed'] is None else f"{snapshot['speed']:.1f} km/h"
    countdown = (f" · {snapshot['countdown']}초" if snapshot['countdown'] is not None
                 else '')
    title = Text(
        f"🚧 K-CITY CONE TRIAL   |   {snapshot['mode']}   |   "
        f"{snapshot['label']}{countdown}\n"
        f"차속 {speed}   인지 {snapshot['perception_reason']} "
        f"({snapshot['path_points']} points)   추종 {snapshot['tracking_reason']}   "
        f"빨간선 {snapshot['red_state']}", style='bold')

    choices = []
    for number, profile in PROFILES.items():
        missing = profile.missing_launches(launch_dir)
        availability = '실행 가능' if not missing else '미구현'
        marker = '▶ ' if number == selection.scenario else '  '
        choices.append(f'{marker}[{number}] {profile.name}  [{availability}]')
    settings = Text(
        '\n'.join(choices) + '\n'
        + f'최고 목표속도 {selection.target_speed_kph:g} km/h\n'
        + '1–4 선택   V 최고속도   R Run   S / Space Stop   Q 종료')
    if speed_text is not None:
        settings.append(
            '\n최고 목표속도 입력: ' + speed_text + '▌  (Enter 적용 / Esc 취소)',
            style='bold cyan')
    if message:
        settings.append('\n' + message, style='yellow')
    if snapshot['finish_reason']:
        settings.append('\n종료 사유: ' + snapshot['finish_reason'], style='yellow')

    gates = Table(expand=True, box=None)
    gates.add_column('실행 조건', ratio=3)
    gates.add_column('판정', ratio=4)
    all_go = True
    for name, go, detail in snapshot['gates']:
        all_go &= go
        gates.add_row(name, Text(
            f"{'🟢 GO' if go else '🔴 NO-GO'}  {detail}",
            style='green' if go else 'bold red'))

    recording = snapshot['run_dir'] or str(records_root)
    layout = Table.grid(expand=True)
    layout.add_row(Panel(title, border_style='cyan'))
    layout.add_row(Panel(settings, title='시험 선택'))
    layout.add_row(Panel(
        gates, title='RUN GO' if all_go else 'RUN NO-GO',
        border_style='green' if all_go else 'red'))
    layout.add_row(Panel(Text(recording), title='MCAP 기록'))
    layout.add_row(Panel(Text('\n'.join(snapshot['logs'])), title='실행 로그'))
    return layout


def _parser():
    parser = argparse.ArgumentParser(
        description='K-City 라바콘 인지/제어 4조합 실차 시험 TUI')
    parser.add_argument(
        '--model-path',
        default=default_model_path(),
        help='YOLO best.pt absolute path (or set KAIEV26_CONE_MODEL)')
    parser.add_argument('--records-root', type=Path, default=DEFAULT_RECORDS_ROOT)
    return parser


def main(model_path, records_root=DEFAULT_RECORDS_ROOT):
    console = Console()
    if not sys.stdin.isatty():
        console.print('대화형 터미널에서 cone_trial_tui를 실행하세요.')
        return 2
    lock_path = Path('/tmp') / (
        f'auto_cone_trial_{os.getuid()}_{os.environ.get("ROS_DOMAIN_ID", "0")}.lock')
    lock_fd = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o600)
    try:
        fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        os.close(lock_fd)
        console.print('같은 ROS domain에서 콘 시험 TUI가 이미 실행 중입니다.')
        return 2

    node = manager = executor = spin_thread = None
    cleanup_errors = []
    quitting = threading.Event()
    old_handlers = {
        sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    signal.signal(signal.SIGTERM, lambda *_: quitting.set())
    signal.signal(signal.SIGHUP, lambda *_: quitting.set())
    exit_code = 0
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = TrialNode()
        manager = ProcessManager(
            node.process_event, node.log, records_root=records_root)
        node.attach_manager(manager)
        manager.start()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, name='cone-trial-ros')
        spin_thread.start()

        selection = TrialSettings(2, 5.0, str(Path(model_path).expanduser()))
        speed_text = None
        message = ''
        with keyboard(), Live(console=console, screen=True, auto_refresh=False) as live:
            while not quitting.is_set():
                live.update(_dashboard(
                    node.snapshot(selection), selection, node.launch_dir,
                    speed_text, message, records_root), refresh=True)
                readable, _, _ = select.select([sys.stdin], [], [], 0.2)
                if not readable:
                    continue
                raw = os.read(sys.stdin.fileno(), 1)
                if not raw:
                    break
                char = raw.decode('ascii', errors='ignore')
                if not char:
                    continue
                if char.lower() == 'q':
                    break
                if char.lower() == 's' or char == ' ':
                    node.request_stop()
                    continue
                if speed_text is not None:
                    if char in '\r\n':
                        try:
                            selection = TrialSettings(
                                selection.scenario, float(speed_text), selection.model_path)
                            speed_text = None
                            message = ''
                        except ValueError as error:
                            message = str(error)
                    elif char == '\x1b':
                        speed_text = None
                    elif char in ('\x7f', '\b'):
                        speed_text = speed_text[:-1]
                    elif char in '0123456789.':
                        speed_text += char
                    continue
                if node.busy:
                    message = '실행·저장이 끝난 뒤 설정을 바꿀 수 있습니다.'
                    continue
                if char in '1234':
                    selection = TrialSettings(
                        int(char), selection.target_speed_kph, selection.model_path)
                    missing = selection.profile.missing_launches(node.launch_dir)
                    message = ('현재 미구현: ' + ', '.join(missing)) if missing else ''
                elif char.lower() == 'v':
                    speed_text = ''
                elif char.lower() == 'r':
                    node.request_run(selection)
                    message = ''
    except KeyboardInterrupt:
        pass
    except Exception as error:
        exit_code = 1
        if node:
            node.log(f'TUI 오류: {error}')
        else:
            console.print(f'TUI 오류: {error}')
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node and node.busy and rclpy.ok():
            node.request_stop('TUI 종료', 'ABORTED')
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline and node.busy:
                time.sleep(0.05)
        if manager:
            cleanup_errors = manager.close()
            if cleanup_errors:
                exit_code = 1
        if executor:
            executor.shutdown()
        if spin_thread and spin_thread.ident is not None:
            spin_thread.join(timeout=2.0)
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        os.close(lock_fd)
        try:
            console.print('기록을 저장하고 종료했습니다.')
            for error in cleanup_errors:
                console.print(Text('정리 오류: ' + error, style='red'))
        except OSError:
            pass
    return exit_code


def cli(argv=None):
    args = _parser().parse_args(argv)
    return main(args.model_path, args.records_root)


if __name__ == '__main__':
    raise SystemExit(cli())
