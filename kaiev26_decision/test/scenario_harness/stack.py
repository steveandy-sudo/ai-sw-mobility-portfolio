"""Real Decision nodes in one process on a simulated clock.

Each launch Node is resolved through its console_scripts entry point to the module, and the one
rclpy Node class that the entry point's main() builds is instantiated in the default rclpy context.
The parameters go in through a params file keyed by node name, the same way ros2 launch passes
them. Nothing else of the nodes is touched: inputs and outputs are topics only.
"""
from __future__ import annotations

from importlib import import_module
from importlib.metadata import entry_points
import inspect
import os
import tempfile

from builtin_interfaces.msg import Time
import rclpy
from rclpy.executors import SingleThreadedExecutor, TimeoutException
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import yaml

from .launch_nodes import NodeSpec


def _console_scripts() -> dict[str, str]:
    found = {}
    for entry in entry_points().get("console_scripts", []):
        found.setdefault(entry.name, entry.value)
    return found


def node_class_for(spec: NodeSpec):
    target = _console_scripts().get(spec.executable)
    if target is None:
        raise RuntimeError(f"no console_scripts entry point for {spec.package}/{spec.executable}")
    module_name, _, function_name = target.partition(":")
    module = import_module(module_name)
    classes = [
        value for value in vars(module).values()
        if inspect.isclass(value) and issubclass(value, Node) and value is not Node
        and value.__module__ == module.__name__
    ]
    if len(classes) > 1:
        main_source = inspect.getsource(getattr(module, function_name or "main"))
        classes = [value for value in classes if f"{value.__name__}(" in main_source]
    if len(classes) != 1:
        raise RuntimeError(f"cannot pick the node class of {spec.executable}: {classes}")
    return classes[0]


def time_msg(t_sec: float) -> Time:
    stamp = Time()
    total_ns = int(round(t_sec * 1.0e9))
    stamp.sec = total_ns // 1_000_000_000
    stamp.nanosec = total_ns % 1_000_000_000
    return stamp


class InProcessStack:
    """The Decision nodes, a harness node and one executor, all on the scenario's sim time."""

    def __init__(self, specs: list[NodeSpec], start_time_sec: float) -> None:
        # The node classes take no arguments, so they live in the default context; one scenario
        # runs at a time and shuts the context down at the end.
        if rclpy.ok():
            raise RuntimeError("an rclpy context is already running in this process")
        self.context = rclpy.utilities.get_default_context()
        params = {spec.name: {"ros__parameters": {**spec.parameters, "use_sim_time": True}} for spec in specs}
        handle, params_path = tempfile.mkstemp(prefix="decision_scenario_", suffix=".yaml")
        try:
            with os.fdopen(handle, "w", encoding="utf-8") as stream:
                yaml.safe_dump(params, stream)
            rclpy.init(args=["--ros-args", "--params-file", params_path], context=self.context)
            self.harness = Node(
                "decision_scenario_harness", context=self.context, use_global_arguments=False,
                start_parameter_services=False,
            )
            self.clock_pub = self.harness.create_publisher(
                Clock, "/clock",
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST),
            )
            self.now_sec = float(start_time_sec)
            self.publish_clock()
            self.nodes = [node_class_for(spec)() for spec in specs]
        finally:
            os.unlink(params_path)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.harness)
        for node in self.nodes:
            self.executor.add_node(node)
        self.publish_clock()
        self.drain()

    def publish_clock(self) -> None:
        message = Clock()
        message.clock = time_msg(self.now_sec)
        self.clock_pub.publish(message)

    def drain(self, max_callbacks: int = 100000) -> int:
        count = 0
        while count < max_callbacks:
            try:
                handler, _entity, _node = self.executor.wait_for_ready_callbacks(timeout_sec=0.0)
            except TimeoutException:
                return count
            handler()
            if handler.exception() is not None:
                raise handler.exception()
            count += 1
        raise RuntimeError("executor did not go idle")

    def advance_to(self, t_sec: float) -> None:
        self.now_sec = float(t_sec)
        self.publish_clock()
        self.drain()

    def shutdown(self) -> None:
        try:
            self.executor.shutdown(timeout_sec=1.0)
        except Exception:  # noqa: BLE001 - best effort teardown
            pass
        for node in [*self.nodes, self.harness]:
            try:
                node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
        try:
            rclpy.shutdown(context=self.context)
        except Exception:  # noqa: BLE001
            pass
