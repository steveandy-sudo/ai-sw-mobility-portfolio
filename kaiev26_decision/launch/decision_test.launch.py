"""Run one field Decision test case over externally launched localization."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from kaiev26_decision.curve_speed_policy import resolve_curve_speed_scale_override
from kaiev26_decision.route_capture import load_custom_route, profile_parameters

CASES = {
    1: ("qualifying", "tracking"),
    3: ("qualifying", "all"),
    4: ("final", "tracking"),
    6: ("final", "avoidance"),
    7: ("final", "all"),
    13: ("custom", "tracking"),
    17: ("custom", "avoidance"),
}
SIMULATION_CASES = frozenset({3, 6, 7})
COURSE_DEFAULT_SPEED_MPS = {
    "qualifying": 4.0,
    "final": 4.0,
    "custom": 2.0,
}


def _nodes(context):
    try:
        case = int(LaunchConfiguration("case").perform(context))
    except ValueError as error:
        raise RuntimeError("case must be one of: " + ", ".join(map(str, CASES))) from error
    if case not in CASES:
        raise RuntimeError("case must be one of: " + ", ".join(map(str, CASES)))

    course, mode = CASES[case]
    platform = LaunchConfiguration("platform").perform(context).strip().lower()
    if platform not in {"vehicle", "simulation"}:
        raise RuntimeError("platform must be vehicle or simulation")
    if platform == "simulation" and case not in SIMULATION_CASES:
        raise RuntimeError("simulation platform supports only cases 3, 6, and 7")

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
    command_topic = LaunchConfiguration("command_topic")
    route_name = (
        "kcity_quali_route.yaml"
        if course == "qualifying"
        else "kcity_final_route.yaml"
    )
    route_config = os.path.join(decision_share, "waypoints", route_name)
    custom_phases = {}
    if course == 'custom':
        route_config = LaunchConfiguration('route_config').perform(context)
        if not Path(route_config).is_absolute():
            raise RuntimeError('custom route_config must be an absolute YAML path')
        custom_phases = profile_parameters(load_custom_route(route_config))
    policy_config = os.path.join(decision_share, "config", f"{course}_policy.yaml")
    landmark_config = os.path.join(
        decision_share, "config", f"{course}_landmarks.yaml"
    )
    if course == 'custom':
        landmark_config = ''
        policy_config = os.path.join(decision_share, 'config', 'midan_test_policy.yaml')
    start_mode = LaunchConfiguration("start_mode").perform(context).strip().lower()
    if start_mode not in {"from_start", "resume"}:
        raise RuntimeError("start_mode must be from_start or resume")
    use_sim_time = platform == "simulation"
    require_readiness = platform == "vehicle"
    command_topic = LaunchConfiguration("command_topic")
    try:
        curve_speed_scale = float(
            LaunchConfiguration("curve_speed_scale").perform(context)
        )
        resolve_curve_speed_scale_override(curve_speed_scale)
    except ValueError as error:
        raise RuntimeError(
            "curve_speed_scale must be -1 (route YAML) or 0 < scale <= 1"
        ) from error

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
                        "allow_midroute_start": course != 'custom' and start_mode == "resume",
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
        "start_mode": start_mode,
        "use_sim_time": use_sim_time,
    }
    if case == 17:
        # Captured-route avoidance shows approach, bypass, return and speed-up: no finish zone
        # takes over 20 m early; the finish stop only brakes once the route end is reached.
        route_parameters["finish_zone_distance_m"] = 0.0
    nodes.append(
        Node(
            package="kaiev26_decision",
            executable="route_zone_manager_node",
            name="kaiev26_route_zone_manager",
            output="screen",
            parameters=[decision_config, route_parameters],
        )
    )

    if mode in {"all", "avoidance"}:
        nodes.append(
            Node(
                package="kaiev26_decision",
                executable="perception_gateway_node",
                name="kaiev26_perception_gateway",
                output="screen",
                parameters=[decision_config, {"use_sim_time": use_sim_time,
                    **({'road_segments_topic': '/decision/midan_unused/road_segments',
                        'centerline_topic': '/decision/midan_unused/centerline'} if case == 17 else {})}],
            )
        )
        if course in {"qualifying", "final"}:
            # Map-wide signal phase estimate. The main planning engine drives on an aligned
            # estimate of the event's own light; the camera rejects it (signal_schedule.py).
            # In Gazebo, signal_cycle_offset_sec (the launcher's --signal-cycle-offset) turns
            # the theta prior on with that mean; on the vehicle the prior is on by default.
            cycle_offset = LaunchConfiguration("signal_cycle_offset_sec").perform(context).strip()
            sim_prior = (
                {"sim_prior_enabled": True, "sim_prior_mean_sec": float(cycle_offset)}
                if cycle_offset
                else {}
            )
            nodes.append(
                Node(
                    package="kaiev26_decision",
                    executable="signal_phase_tracker_node",
                    name="kaiev26_signal_phase_tracker",
                    output="screen",
                    parameters=[
                        decision_config,
                        {
                            "map_semantics_path": os.path.join(
                                get_package_share_directory("kaiev26_maps"),
                                "maps", "kcity", "kcity_course_semantics.yaml",
                            ),
                            "mission_policy_config": policy_config,
                            "use_sim_time": use_sim_time,
                            **sim_prior,
                        },
                    ],
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
                    "mission_policy_config": (
                        "" if mode == "tracking" else policy_config
                    ),
                    "mission_mode": mode,
                    "base_speed_mps": speed_mps,
                    "route_config": route_config,
                    "landmark_config": landmark_config if mode != "tracking" else "",
                    "curve_speed_scale": curve_speed_scale,
                    "require_readiness": require_readiness,
                    "use_sim_time": use_sim_time,
                    # Captured-route avoidance: the finish stop does not slow down 20 m early
                    # either; it brakes once the rear axle reaches the route end (0.1 m minimum).
                    **({"finish_approach_distance_m": 0.1} if case == 17 else {}),
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
                        "use_sim_time": use_sim_time,
                        **custom_phases,
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
                    f"speed=cruise_target={speed_mps:g} m/s, start={start_mode}"
                )
            ),
        ]
    )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("case", default_value="1"),
            DeclareLaunchArgument('route_config', default_value=''),
            DeclareLaunchArgument(
                "platform",
                default_value="vehicle",
                choices=["vehicle", "simulation"],
            ),
            DeclareLaunchArgument(
                "speed_mps",
                default_value="0.0",
                description=(
                    "Tracking or mission cruise target in m/s. Use 0 for "
                    "qualifying=4.0, final=4.0, custom=2.0."
                ),
            ),
            DeclareLaunchArgument(
                "start_mode",
                default_value="resume",
                choices=["from_start", "resume"],
                description=(
                    "from_start: the run starts at the route start; the progress is matched "
                    "there by position only. resume: heading-aware search over the whole route."
                ),
            ),
            DeclareLaunchArgument(
                "curve_speed_scale",
                default_value="-1.0",
                description=(
                    "Startup curve speed multiplier: 0 < scale <= 1; "
                    "-1 keeps route YAML."
                ),
            ),
            DeclareLaunchArgument(
                "command_topic", default_value="/planning/command"
            ),
            DeclareLaunchArgument(
                "signal_cycle_offset_sec",
                default_value="",
                description=(
                    "Gazebo only: the traffic_signal_cycle_offset_sec the simulation runs with "
                    "(30 for o30). Given, the tracker's theta prior is on with this mean under "
                    "sim time. Empty keeps the prior off in simulation."
                ),
            ),
            OpaqueFunction(function=_nodes),
        ]
    )
