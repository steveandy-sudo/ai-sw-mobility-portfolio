"""One scenario per process: read {id, scenario} as JSON on stdin, print the outcome as one line."""
from __future__ import annotations

import json
import math
import sys
import traceback

from .checks import evaluate
from .runner import run_scenario
from .catalog import RESULT_PREFIX


def trace(result) -> list[str]:
    """Rows at every change of behaviour, FSM state or brake command, plus one per second."""
    samples = result.samples
    event = result.scenario.get("event")
    line = result.course.event(event).line_s if event else None
    rows = []
    previous = None
    next_tick = 0.0
    for i in range(len(samples)):
        t = samples["t"][i]
        if t < -1.0e-9:
            continue
        key = (samples["behavior"][i], samples["fsm"][i], samples["cmd_brake"][i], samples["brake_applied"][i])
        if key == previous and t < next_tick - 1.0e-9 and i != len(samples) - 1:
            continue
        previous = key
        next_tick = math.floor(t) + 1.0
        gap = "" if line is None else f" gap={line - samples['front_s'][i]:7.2f}"
        rows.append(
            f"t={t:6.2f} v={samples['v'][i]:5.2f} s={samples['s'][i]:7.2f}{gap} lat={samples['lateral'][i]:+5.2f}"
            f" cmd={samples['cmd_speed'][i]:5.2f} brk={int(samples['cmd_brake'][i])}{int(samples['brake_applied'][i])}"
            f" {samples['behavior'][i]}:{samples['fsm'][i]}"
        )
    return rows


def main() -> int:
    request = json.loads(sys.stdin.read())
    outcome = {"id": request["id"], "checks": []}
    try:
        result = run_scenario(request["scenario"])
        outcome["checks"] = [vars(check) for check in evaluate(result)]
        outcome["wall_s"] = round(result.wall_s, 2)
        outcome["sim_s"] = result.sim_s
        outcome["trace"] = trace(result)
    except Exception:  # noqa: BLE001 - reported to the test as an error
        outcome["error"] = traceback.format_exc()
    sys.stdout.write("\n" + RESULT_PREFIX + json.dumps(outcome) + "\n")
    sys.stdout.flush()
    return 0


if __name__ == "__main__":
    sys.exit(main())
