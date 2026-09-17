import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _actions(context, config: str):
    mode = LaunchConfiguration("mode").perform(context)
    desktop = ExecuteProcess(cmd=["foxglove-studio"], output="screen")
    if mode == "record":
        return [
            LogInfo(msg="Foxglove record mode: open the recorded MCAP as a local file."),
            desktop,
        ]

    return [
        LogInfo(
            msg=(
                f"Foxglove {mode} monitor: vehicle, sensors, localization, and "
                "the driving stack must already be running."
            )
        ),
        Node(
            package="kaiev26_decision_fox",
            executable="fox_node",
            name="kaiev26_decision_fox",
            output="screen",
            parameters=[
                config,
                {
                    "use_sim_time": False,
                    "odometry_topic": "/localization/odometry",
                },
            ],
        ),
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[{"port": 8765, "address": "0.0.0.0"}],
        ),
        desktop,
    ]


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("kaiev26_decision_fox"),
        "config",
        "fox.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="manual",
                choices=["manual", "auto", "record"],
            ),
            OpaqueFunction(function=_actions, args=[config]),
        ]
    )
