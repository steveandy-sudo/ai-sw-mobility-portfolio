"""Run one complete AEB trial on the current workspace Gazebo map."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    OpaqueFunction, TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from kaiev26_aeb.trial_profiles import default_model_path, profile_for


def _build(context):
    try:
        profile_for(int(LaunchConfiguration('scenario').perform(context)))
    except ValueError as error:
        raise RuntimeError('scenario must be an integer from 1 to 4') from error

    from kaiev26_gazebo_bringup.simulation_launch import (
        build_simulation_actions, simulation_launch_spec,
    )

    share = get_package_share_directory('kaiev26_aeb')
    actions = build_simulation_actions(simulation_launch_spec(
        headless=LaunchConfiguration('headless').perform(context),
        driver_input='emulated', driver_initial_mode='autonomous'))
    actions.extend([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'aeb_test.launch.py')),
            launch_arguments={
                'scenario': LaunchConfiguration('scenario'),
                'use_sim_time': 'true',
                'enabled': 'false',
                'command_topic': '/planning/command',
                'target_speed_kph': LaunchConfiguration('target_speed_kph'),
                'model_path': LaunchConfiguration('model_path'),
                'cloud_time_offset_s': '0.0',
            }.items()),
        Node(
            package='command_governor', executable='command_governor_node',
            name='command_governor', output='screen',
            parameters=[{'use_sim_time': True}]),
        TimerAction(
            period=2.0,
            actions=[ExecuteProcess(
                cmd=['ros2', 'run', 'kaiev26_aeb', 'sim_trial'],
                output='screen',
                condition=IfCondition(LaunchConfiguration('run_trial')),
            )]),
    ])
    return actions


def generate_launch_description():
    defaults = {
        'scenario': '2',
        'target_speed_kph': '5.0',
        'model_path': default_model_path(),
        'headless': 'false',
        'run_trial': 'true',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value)
          for name, value in defaults.items()],
        OpaqueFunction(function=_build),
    ])
