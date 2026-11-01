"""Rich terminal UI for the K-City cone-course experiments."""

import argparse
from contextlib import contextmanager
from dataclasses import replace
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
from .trial_profiles import TrialSettings
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


def _dashboard(snapshot, selection, launch_dir, edit_field=None, edit_text='', message='',
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

    missing = selection.profile.missing_launches(launch_dir)
    availability = '실행 가능' if not missing else '미구현'
    settings = Text(f'{selection.profile.name} (고정)  [{availability}]\n'
                    + f'최고 목표속도 {selection.target_speed_kph:g} km/h'
                    + f'   속도명령 증가율 {selection.acceleration_mps2:.1f} m/s² (고정)'
                    + f'   빨간선 선행 제동 {selection.red_brake_lead_m:g} m\n'
                    + ('E-stop: USE (사람이 E-stop으로 정지, 빨간선에서 감속·브레이크 없음)'
                       if selection.estop_use else
                       'E-stop: NOT-USE (빨간선에서 차량 브레이크 자동 체결)'))
    settings.append('\nV 최고속도   B 제동거리   E E-stop USE/NOT-USE'
                    '   R Run   S / Space Stop   Q 종료')
    if edit_field is not None:
        label = {'target_speed_kph': '최고 목표속도',
                 'red_brake_lead_m': '빨간선 선행 제동거리'}[edit_field]
        settings.append(
            '\n' + label + ' 입력: ' + edit_text + '▌  (Enter 적용 / Esc 취소)',
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
        description='K-City 라바콘 인지/제어 실차 시험 TUI')
    parser.add_argument('--records-root', type=Path, default=DEFAULT_RECORDS_ROOT)
    return parser


def main(records_root=DEFAULT_RECORDS_ROOT):
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

        selection = TrialSettings()
        edit_field = None
        edit_text = ''
        message = ''
        with keyboard(), Live(console=console, screen=True, auto_refresh=False) as live:
            while not quitting.is_set():
                live.update(_dashboard(
                    node.snapshot(selection), selection, node.launch_dir,
                    edit_field, edit_text, message, records_root), refresh=True)
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
                if edit_field is not None:
                    if char in '\r\n':
                        try:
                            selection = replace(selection, **{edit_field: float(edit_text)})
                            edit_field = None
                            edit_text = ''
                            message = ''
                        except ValueError as error:
                            message = str(error)
                    elif char == '\x1b':
                        edit_field = None
                        edit_text = ''
                    elif char in ('\x7f', '\b'):
                        edit_text = edit_text[:-1]
                    elif char in '0123456789.':
                        edit_text += char
                    continue
                if node.busy:
                    message = '실행·저장이 끝난 뒤 설정을 바꿀 수 있습니다.'
                    continue
                if char.lower() == 'v':
                    edit_field, edit_text = 'target_speed_kph', ''
                elif char.lower() == 'b':
                    edit_field, edit_text = 'red_brake_lead_m', ''
                elif char.lower() == 'e':
                    selection = replace(selection, estop_use=not selection.estop_use)
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
    return main(args.records_root)


if __name__ == '__main__':
    raise SystemExit(cli())
