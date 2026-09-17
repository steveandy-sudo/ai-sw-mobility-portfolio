import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from kaiev26_decision.zone_policy import load_zone_policy


COURSE_DEFAULT_SPEED_MPS = {
    "qualifying": 3.2,
    "final": 3.5,
}
DEGRADED_SPEED_LIMIT_MPS = 2.5


def _required_path(context, argument_name):
    value = LaunchConfiguration(argument_name).perform(context).strip()
    if not value:
        raise RuntimeError(f"course:=custom requires {argument_name}:=<absolute_path>")
    return os.path.abspath(os.path.expanduser(value))


def _course_files(context, package_share):
    course = LaunchConfiguration("course").perform(context).strip().lower()
    supplied_route = LaunchConfiguration("route_config").perform(context).strip()
    supplied_policy = LaunchConfiguration("mission_policy_config").perform(context).strip()
    supplied_landmarks = LaunchConfiguration("landmark_config").perform(context).strip()

    qualifying_route = os.path.join(
        package_share,
        "waypoints",
        "kcity_quali_route.yaml",
    )
    qualifying_policy = os.path.join(package_share, "config", "qualifying_policy.yaml")
    qualifying_landmarks = os.path.join(
        package_share,
        "config",
        "qualifying_landmarks.yaml",
    )

    if course == "qualifying":
        return (
            supplied_route or qualifying_route,
            supplied_policy or qualifying_policy,
            supplied_landmarks or qualifying_landmarks,
        )
    if course == "final":
        final_policy = supplied_policy or os.path.join(
            package_share,
            "config",
            "final_policy.yaml",
        )
        load_zone_policy(final_policy)
        return (
            supplied_route
            or os.path.join(
                package_share,
                "waypoints",
                "kcity_final_route.yaml",
            ),
            final_policy,
            supplied_landmarks
            or os.path.join(package_share, "config", "final_landmarks.yaml"),
        )
    if course == "custom":
        return (
            _required_path(context, "route_config"),
            _required_path(context, "mission_policy_config"),
            _required_path(context, "landmark_config"),
        )
    raise RuntimeError(f"unsupported course [{course}]")


def _launch_nodes(context):
    package_share = get_package_share_directory("kaiev26_decision")
    decision_config = os.path.join(package_share, "config", "decision_pipeline.yaml")
    motion_config = os.path.join(
        get_package_share_directory("kaiev26_motion_control"),
        "config",
        "motion_control.yaml",
    )
    route_config, mission_policy, landmark_config = _course_files(
        context,
        package_share,
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    odometry_topic = LaunchConfiguration("odometry_topic")
    course = LaunchConfiguration("course").perform(context).strip().lower()
    speed_text = LaunchConfiguration("cruise_speed_mps").perform(context).strip()
    try:
        requested_speed_mps = float(speed_text)
    except ValueError as error:
        raise RuntimeError("cruise_speed_mps must be a number in m/s") from error
    if requested_speed_mps == 0.0:
        if course not in COURSE_DEFAULT_SPEED_MPS:
            raise RuntimeError(
                "course:=custom requires cruise_speed_mps:=<0.1..15.0>"
            )
        cruise_speed_mps = COURSE_DEFAULT_SPEED_MPS[course]
        speed_source = "course_default"
    elif 0.1 <= requested_speed_mps <= 15.0:
        cruise_speed_mps = requested_speed_mps
        speed_source = "launch_argument"
    else:
        raise RuntimeError(
            "cruise_speed_mps must be 0 (course default) or between 0.1 and 15.0 m/s"
        )
    degraded_speed_mps = min(cruise_speed_mps, DEGRADED_SPEED_LIMIT_MPS)
    lateral_controller = ParameterValue(
        LaunchConfiguration("lateral_controller"),
        value_type=str,
    )

    return [
        Node(
            package="kaiev26_decision",
            executable="perception_gateway_node",
            name="kaiev26_perception_gateway",
            output="screen",
            parameters=[decision_config, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="kaiev26_decision",
            executable="route_zone_manager_node",
            name="kaiev26_route_zone_manager",
            output="screen",
            parameters=[
                decision_config,
                {
                    "route_config": route_config,
                    "landmark_config": landmark_config,
                    "mission_policy_config": mission_policy,
                    "odometry_topic": odometry_topic,
                    "use_sim_time": use_sim_time,
                },
            ],
        ),
        Node(
            package="kaiev26_decision",
            executable="main_planning_engine_node",
            name="kaiev26_main_planning_engine",
            output="screen",
            parameters=[
                decision_config,
                {
                    "mission_policy_config": mission_policy,
                    "base_speed_mps": cruise_speed_mps,
                    "degraded_speed_mps": degraded_speed_mps,
                    "use_sim_time": use_sim_time,
                },
            ],
        ),
        Node(
            package="kaiev26_decision",
            executable="debug_monitor_node",
            name="kaiev26_debug_monitor",
            output="screen",
            parameters=[decision_config, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="kaiev26_motion_control",
            executable="motion_control_node",
            name="kaiev26_motion_control",
            output="screen",
            parameters=[
                motion_config,
                {
                    "lateral_controller": lateral_controller,
                    "use_sim_time": use_sim_time,
                },
            ],
        ),
        LogInfo(
            msg=(
                f"Decision speed: course={course}, cruise_limit={cruise_speed_mps:g} "
                f"m/s, degraded_limit={degraded_speed_mps:g} m/s, source={speed_source}"
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "course",
                default_value="qualifying",
                choices=["qualifying", "final", "custom"],
                description="Mission profile for the qualifying, final, or custom course.",
            ),
            DeclareLaunchArgument(
                "odometry_topic",
                default_value="/localization/odometry",
            ),
            DeclareLaunchArgument(
                "lateral_controller",
                default_value="pure_pursuit",
                choices=["pure_pursuit", "stanley", "pp_stanley", "ff_stanley"],
                description=(
                    "Lateral controller: pure_pursuit, stanley, pp_stanley, or ff_stanley."
                ),
            ),
            DeclareLaunchArgument(
                "cruise_speed_mps",
                default_value="0.0",
                description=(
                    "Cruise speed ceiling in m/s. Use 0 for the recorded manual-drive "
                    "default: qualifying=3.2, final=3.5."
                ),
            ),
            DeclareLaunchArgument("route_config", default_value=""),
            DeclareLaunchArgument("mission_policy_config", default_value=""),
            DeclareLaunchArgument("landmark_config", default_value=""),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
