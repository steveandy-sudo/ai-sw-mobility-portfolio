"""Select one AEB perception/tracking pair for a real-vehicle trial."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from kaiev26_aeb.trial_profiles import (
    TRIAL_ACCELERATION_MPS2, default_target_speed_kph, profile_for)


def _build(context):
    try:
        profile = profile_for(int(LaunchConfiguration('scenario').perform(context)))
    except ValueError as error:
        raise RuntimeError('scenario must be 1 or 2') from error

    share = get_package_share_directory('kaiev26_aeb')
    perception_config = os.path.join(share, 'config', 'perception.yaml')
    tracking_config = os.path.join(share, 'config', 'tracking.yaml')
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)

    perception_node = Node(
        package='cone_perception', executable='cone_node',
        name='cone_perception', output='screen',
        parameters=[os.path.join(get_package_share_directory('cone_perception'),
                                 'config', 'cone_perception.yaml'),
                    {'use_sim_time': use_sim_time}],
    )

    tracking_parameters = {
        'use_sim_time': use_sim_time,
        'enabled': ParameterValue(LaunchConfiguration('enabled'), value_type=bool),
        'command_topic': ParameterValue(
            LaunchConfiguration('command_topic'), value_type=str),
        'target_speed_kph': float(
            LaunchConfiguration('target_speed_kph').perform(context)
            or default_target_speed_kph(profile.scenario)),
        'acceleration_mps2': TRIAL_ACCELERATION_MPS2,
        'red_brake_lead_m': ParameterValue(
            LaunchConfiguration('red_brake_lead_m'), value_type=float),
        'vehicle_brake': ParameterValue(
            LaunchConfiguration('vehicle_brake'), value_type=bool),
    }

    actions = [
        perception_node,
        Node(
            package='kaiev26_aeb', executable=profile.perception_node,
            name=profile.perception_node, output='screen', parameters=[
                perception_config, {'use_sim_time': use_sim_time}]),
        Node(
            package='kaiev26_aeb', executable=profile.controller_node,
            name=profile.controller_node, output='screen',
            parameters=[tracking_config, tracking_parameters]),
    ]
    return actions


def generate_launch_description():
    defaults = {
        'scenario': '2',
        'use_sim_time': 'false',
        'enabled': 'false',
        'command_topic': '',
        'target_speed_kph': '',
        'red_brake_lead_m': '5.0',
        # false: TUI E-stop USE, the controller never engages the vehicle brake.
        'vehicle_brake': 'true',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value)
          for name, value in defaults.items()],
        OpaqueFunction(function=_build),
    ])
