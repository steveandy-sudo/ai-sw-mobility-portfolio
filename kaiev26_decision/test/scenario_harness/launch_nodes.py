"""Read the node list and parameters the real Decision test launch would start.

The runner never builds node parameters itself: it evaluates decision_test.launch.py with the
same launch arguments the TUI passes, then takes each Node action's package, executable, name and
merged parameters. A change to the launch or the shipped yaml therefore reaches the scenarios.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import runpy

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node as LaunchNode
from launch_ros.utilities import evaluate_parameters
import yaml


# Course name -> launch case of the full competition pipeline (decision_test.launch.py CASES).
COURSE_CASES = {"qualifying": "3", "final": "7"}


@dataclass
class NodeSpec:
    package: str
    executable: str
    name: str
    parameters: dict = field(default_factory=dict)


def test_launch_path() -> Path:
    return Path(get_package_share_directory("kaiev26_decision")) / "launch" / "decision_test.launch.py"


def _yaml_parameters(path: Path, node_name: str) -> dict:
    document = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    merged: dict = {}
    for key in ("/**", node_name, f"/{node_name}"):
        section = document.get(key)
        if isinstance(section, dict):
            merged.update(section.get("ros__parameters", {}) or {})
    return merged


def _launch_nodes(entities, context) -> list[LaunchNode]:
    """Node actions of the description, expanding OpaqueFunctions with the launch context."""
    found = []
    for action in entities:
        if isinstance(action, LaunchNode):
            found.append(action)
        elif isinstance(action, OpaqueFunction):
            found.extend(_launch_nodes(action.execute(context) or [], context))
    return found


def launch_node_specs(course: str, speed_mps: float, extra_arguments: dict | None = None) -> list[NodeSpec]:
    """Node specs of the simulation competition case for one course."""
    if course not in COURSE_CASES:
        raise ValueError(f"course must be one of {sorted(COURSE_CASES)}, got [{course}]")
    arguments = {
        "case": COURSE_CASES[course],
        "platform": "simulation",
        "speed_mps": f"{float(speed_mps):g}",
        # The TUI relays this topic to /planning/command; the plant reads it directly.
        "command_topic": "/decision/raw_command",
    }
    arguments.update(extra_arguments or {})
    return node_specs(arguments)


def node_specs(arguments: dict) -> list[NodeSpec]:
    """Node specs decision_test.launch.py starts with these launch arguments (others default)."""
    module = runpy.run_path(str(test_launch_path()))
    context = LaunchContext()
    entities = list(module["generate_launch_description"]().entities)
    for action in entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update({key: str(value) for key, value in arguments.items()})
    specs = []
    for action in _launch_nodes(entities, context):
        name = perform_substitutions(context, normalize_to_list_of_substitutions(action._Node__node_name))
        parameters: dict = {}
        for item in evaluate_parameters(context, action._Node__parameters):
            if isinstance(item, dict):
                parameters.update(item)
            else:
                parameters.update(_yaml_parameters(Path(str(item)), name))
        specs.append(NodeSpec(action.node_package, action.node_executable, name, parameters))
    return specs
