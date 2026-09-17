"""Select one AEB perception/tracking pair for a real-vehicle trial."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from kaiev26_aeb.trial_profiles import default_model_path, profile_for


def _build(context):
    try:
        profile = profile_for(int(LaunchConfiguration('scenario').perform(context)))
    except ValueError as error:
        raise RuntimeError('scenario must be an integer from 1 to 4') from error

    share = get_package_share_directory('kaiev26_aeb')
    perception_config = os.path.join(share, 'config', 'perception.yaml')
    tracking_config = os.path.join(share, 'config', 'tracking.yaml')
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)

    perception_parameters = {
        'use_sim_time': use_sim_time,
        'model_path': ParameterValue(
            LaunchConfiguration('model_path'), value_type=str),
        'device': ParameterValue(LaunchConfiguration('device'), value_type=str),
    }
    if profile.perception == 'fusion':
        perception_parameters.update({
            'points_topic': ParameterValue(
                LaunchConfiguration('points_topic'), value_type=str),
            'cloud_time_offset_s': ParameterValue(
                LaunchConfiguration('cloud_time_offset_s'), value_type=float),
        })

    tracking_parameters = {
        'use_sim_time': use_sim_time,
        'enabled': ParameterValue(LaunchConfiguration('enabled'), value_type=bool),
        'command_topic': ParameterValue(
            LaunchConfiguration('command_topic'), value_type=str),
        'target_speed_kph': ParameterValue(
            LaunchConfiguration('target_speed_kph'), value_type=float),
    }

    return [
        Node(
            package='kaiev26_aeb', executable=profile.perception_node,
            name=profile.perception_node, output='screen',
            parameters=[perception_config, perception_parameters]),
        Node(
            package='kaiev26_aeb', executable=profile.controller_node,
            name=profile.controller_node, output='screen',
            parameters=[tracking_config, tracking_parameters]),
    ]


def generate_launch_description():
    defaults = {
        'scenario': '2',
        'use_sim_time': 'false',
        'enabled': 'false',
        'command_topic': '',
        'target_speed_kph': '5.0',
        'model_path': default_model_path(),
        'device': 'auto',
        'points_topic': '/ouster/points',
        'cloud_time_offset_s': '0.025',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value)
          for name, value in defaults.items()],
        OpaqueFunction(function=_build),
    ])
