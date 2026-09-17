"""Run one field Decision test case over externally launched localization."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


CASES = {
    1: ("qualifying", "tracking"),
    2: ("qualifying", "stopline"),
    3: ("qualifying", "all"),
    4: ("final", "tracking"),
    5: ("final", "stopline"),
    6: ("final", "avoidance"),
    7: ("final", "all"),
    8: ("steering_step", "steering_step"),
}
COURSE_DEFAULT_SPEED_MPS = {
    "qualifying": 3.2,
    "final": 3.5,
    "steering_step": 1.0,
}
DEGRADED_SPEED_LIMIT_MPS = 2.5


def _nodes(context):
    try:
        case = int(LaunchConfiguration("case").perform(context))
    except ValueError as error:
        raise RuntimeError("case must be an integer from 1 to 8") from error
    if case not in CASES:
        raise RuntimeError("case must be an integer from 1 to 8")

    course, mode = CASES[case]
    platform = LaunchConfiguration("platform").perform(context).strip().lower()
    if platform not in {"vehicle", "simulation"}:
        raise RuntimeError("platform must be vehicle or simulation")

    speed_text = LaunchConfiguration("speed_mps").perform(context).strip()
    try:
        requested_speed_mps = float(speed_text)
    except ValueError as error:
        raise RuntimeError("speed_mps must be a number in m/s") from error
    if requested_speed_mps == 0.0:
        speed_mps = COURSE_DEFAULT_SPEED_MPS[course]
    elif 0.1 <= requested_speed_mps <= 15.0:
        speed_mps = requested_speed_mps
    else:
        raise RuntimeError(
            "speed_mps must be 0 (course default) or between 0.1 and 15.0 m/s"
        )

    decision_share = get_package_share_directory("kaiev26_decision")
    decision_config = os.path.join(
        decision_share, "config", "decision_pipeline.yaml"
    )
    motion_config = os.path.join(
        get_package_share_directory("kaiev26_motion_control"),
        "config",
        "motion_control.yaml",
    )
    route_name = (
        "kcity_quali_route.yaml"
        if course == "qualifying"
        else "kcity_final_route.yaml"
    )
    route_config = os.path.join(decision_share, "waypoints", route_name)
    policy_config = os.path.join(decision_share, "config", f"{course}_policy.yaml")
    landmark_config = os.path.join(
        decision_share, "config", f"{course}_landmarks.yaml"
    )
    use_sim_time = platform == "simulation"
    require_readiness = platform == "vehicle"
    command_topic = LaunchConfiguration("command_topic")
    controller = ParameterValue(
        LaunchConfiguration("lateral_controller"), value_type=str
    )

    if mode == "steering_step":
        return [
            Node(
                package="kaiev26_decision",
                executable="steering_step_test_node",
                name="kaiev26_steering_step_test",
                output="screen",
                parameters=[
                    {
                        "command_topic": command_topic,
                        "constant_speed_mps": speed_mps,
                        "steering_amplitude_deg": ParameterValue(
                            LaunchConfiguration("steering_amplitude_deg"),
                            value_type=float,
                        ),
                        "steering_frequency_hz": ParameterValue(
                            LaunchConfiguration("steering_frequency_hz"),
                            value_type=float,
                        ),
                        "steering_pattern": ParameterValue(
                            LaunchConfiguration("steering_pattern"),
                            value_type=str,
                        ),
                        "initial_straight_s": ParameterValue(
                            LaunchConfiguration("steering_initial_straight_s"),
                            value_type=float,
                        ),
                        "wait_for_run_enable": ParameterValue(
                            LaunchConfiguration("step_wait_for_run_enable"),
                            value_type=bool,
                        ),
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            LogInfo(
                msg=(
                    f"Decision test case 8: steering_step, speed={speed_mps:g} m/s"
                )
            ),
        ]

    nodes = []
    if platform == "vehicle":
        nodes.append(
            Node(
                package="kaiev26_decision",
                executable="route_readiness_node",
                name="route_readiness",
                output="screen",
                parameters=[
                    {
                        "allow_midroute_start": True,
                        "use_sim_time": False,
                    }
                ],
            )
        )

    route_parameters = {
        "route_config": route_config,
        "landmark_config": landmark_config if mode != "tracking" else "",
        "mission_policy_config": policy_config if mode != "tracking" else "",
        "odometry_topic": "/localization/odometry",
        "use_sim_time": use_sim_time,
    }
    nodes.append(
        Node(
            package="kaiev26_decision",
            executable="route_zone_manager_node",
            name="kaiev26_route_zone_manager",
            output="screen",
            parameters=[decision_config, route_parameters],
        )
    )

    if mode == "tracking":
        nodes.append(
            Node(
                package="kaiev26_decision",
                executable="constant_route_plan_node",
                name="kaiev26_constant_route_plan",
                output="screen",
                parameters=[
                    {
                        "constant_speed_mps": speed_mps,
                        "require_readiness": require_readiness,
                        "use_sim_time": use_sim_time,
                    }
                ],
            )
        )
    else:
        if mode in {"all", "avoidance"}:
            nodes.append(
                Node(
                    package="kaiev26_decision",
                    executable="perception_gateway_node",
                    name="kaiev26_perception_gateway",
                    output="screen",
                    parameters=[decision_config, {"use_sim_time": use_sim_time}],
                )
            )
        nodes.append(
            Node(
                package="kaiev26_decision",
                executable="main_planning_engine_node",
                name="kaiev26_main_planning_engine",
                output="screen",
                parameters=[
                    decision_config,
                    {
                        "mission_policy_config": policy_config,
                        "mission_mode": mode,
                        "base_speed_mps": speed_mps,
                        "degraded_speed_mps": min(
                            speed_mps, DEGRADED_SPEED_LIMIT_MPS
                        ),
                        "require_readiness": require_readiness,
                        "use_sim_time": use_sim_time,
                    },
                ],
            )
        )

    nodes.extend(
        [
            Node(
                package="kaiev26_motion_control",
                executable="motion_control_node",
                name="kaiev26_motion_control",
                output="screen",
                parameters=[
                    motion_config,
                    {
                        "command_topic": command_topic,
                        "lateral_controller": controller,
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
            Node(
                package="kaiev26_decision",
                executable="debug_monitor_node",
                name="kaiev26_debug_monitor",
                output="screen",
                parameters=[
                    decision_config,
                    {
                        "command_topic": command_topic,
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
            LogInfo(
                msg=(
                    f"Decision test case {case}: course={course}, mode={mode}, "
                    f"speed={'straight_target' if mode == 'tracking' else 'cruise_ceiling'}="
                    f"{speed_mps:g} m/s"
                )
            ),
        ]
    )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("case", default_value="1"),
            DeclareLaunchArgument(
                "platform",
                default_value="vehicle",
                choices=["vehicle", "simulation"],
            ),
            DeclareLaunchArgument(
                "speed_mps",
                default_value="0.0",
                description=(
                    "Tracking speed or mission cruise ceiling in m/s. Use 0 for "
                    "qualifying=3.2, final=3.5, and steering step=1.0."
                ),
            ),
            DeclareLaunchArgument(
                "lateral_controller",
                default_value="pure_pursuit",
                choices=["pure_pursuit", "stanley", "pp_stanley", "ff_stanley"],
            ),
            DeclareLaunchArgument(
                "command_topic", default_value="/planning/command"
            ),
            DeclareLaunchArgument(
                "steering_amplitude_deg",
                default_value="3.0",
                description="Case 8 central-equivalent steering amplitude in degrees.",
            ),
            DeclareLaunchArgument(
                "steering_frequency_hz",
                default_value="0.25",
                description="Case 8 direction-plus-center pulse frequency in Hz.",
            ),
            DeclareLaunchArgument(
                "steering_pattern",
                default_value="RLR",
                description="Case 8 repeated direction sequence containing only R and L.",
            ),
            DeclareLaunchArgument(
                "steering_initial_straight_s", default_value="3.0"
            ),
            DeclareLaunchArgument(
                "step_wait_for_run_enable",
                default_value="false",
                choices=["true", "false"],
            ),
            OpaqueFunction(function=_nodes),
        ]
    )
