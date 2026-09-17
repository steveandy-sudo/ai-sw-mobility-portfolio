import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _rviz_node(context):
    mode = LaunchConfiguration("mode").perform(context)
    share = get_package_share_directory("kaiev26_decision")
    config = os.path.join(share, "config", f"driving_{mode}.rviz")

    return [
        LogInfo(msg=f"Decision RViz mode: {mode}"),
        Node(
            package="rviz2",
            executable="rviz2",
            name="decision_rviz",
            output="screen",
            arguments=["-d", config],
            parameters=[
                {
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"),
                        value_type=bool,
                    )
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="spatial",
                choices=["spatial", "lidar"],
                description="RViz layout: spatial overview or LiDAR-focused view.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                choices=["true", "false"],
            ),
            OpaqueFunction(function=_rviz_node),
        ]
    )
