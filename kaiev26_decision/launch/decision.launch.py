"""Open the vehicle mission TUI without inline settings or MCAP recording."""

import sys

from launch import LaunchDescription
from launch.actions import ExecuteProcess, Shutdown


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[sys.executable, "-m", "kaiev26_decision.operating_tui"],
            name="decision_operator",
            output="log",
            sigterm_timeout="20",
            sigkill_timeout="5",
            on_exit=[Shutdown(reason="Decision operator exited")],
        ),
    ])
