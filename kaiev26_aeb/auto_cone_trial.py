#!/usr/bin/env python3
"""Start the K-City cone trial TUI from a sourced shell or ROS workspace."""

from pathlib import Path
import os
import shlex
import sys


def _overlay_environment():
    env = os.environ.copy()
    for name in ('PYTHONPATH', 'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH',
                 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH',
                 'COLCON_CURRENT_PREFIX', 'AMENT_CURRENT_PREFIX'):
        env.pop(name, None)
    return env


def main():
    here = Path(__file__).resolve()
    if os.environ.get('AUTO_CONE_TRIAL_ROS_ENV') != '1':
        setups = []
        humble = Path('/opt/ros/humble/setup.bash')
        if humble.is_file():
            setups.append(humble)
        for ancestor in here.parents:
            local = ancestor / 'install/local_setup.bash'
            if local.is_file():
                setups.append(local)
                break
        if not setups:
            print('ROS 2 Humble / workspace install을 찾지 못했습니다.', file=sys.stderr)
            return 1
        env = _overlay_environment()
        env['AUTO_CONE_TRIAL_ROS_ENV'] = '1'
        command = '\n'.join(
            'source ' + shlex.quote(str(path)) for path in dict.fromkeys(setups))
        argv = ' '.join(shlex.quote(value) for value in [str(here), *sys.argv[1:]])
        command += '\nexec /usr/bin/python3 ' + argv
        os.execvpe('/bin/bash', ['bash', '-c', command], env)

    sys.path.insert(0, str(here.parent))
    from kaiev26_aeb.trial_tui import cli
    return cli(sys.argv[1:])


if __name__ == '__main__':
    raise SystemExit(main())
