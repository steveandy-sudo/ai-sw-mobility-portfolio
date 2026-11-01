"""Attach the shared mission TUI to the launch terminal, not its output pipe."""

from contextlib import contextmanager
import os
import sys

from kaiev26_decision.test_tui import run_tui


@contextmanager
def controlling_terminal():
    # ros2 launch pipes child output and does not forward keyboard input.
    with open("/dev/tty", "r", encoding="utf-8") as reader, open(
        "/dev/tty", "w", encoding="utf-8", buffering=1
    ) as writer:
        previous = sys.stdin, sys.stdout, sys.stderr
        previous_stdin_fd = os.dup(0)
        try:
            # Rich reads terminal dimensions from fd 0/1/2, not sys.stdout.fileno().
            os.dup2(reader.fileno(), 0)
            sys.stdin, sys.stdout, sys.stderr = reader, writer, writer
            yield
        finally:
            sys.stdin, sys.stdout, sys.stderr = previous
            os.dup2(previous_stdin_fd, 0)
            os.close(previous_stdin_fd)


def main() -> int:
    try:
        with controlling_terminal():
            return run_tui(
                operating_mode=True,
                command_hint="ros2 launch kaiev26_decision decision.launch.py",
            )
    except OSError as error:
        print(f"Decision requires an interactive terminal: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
