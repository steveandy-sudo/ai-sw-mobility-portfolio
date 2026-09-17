from glob import glob
from setuptools import setup


package_name = "kaiev26_decision"

setup(
    name=package_name,
    version="0.1.0",
    packages=[
        package_name,
        f"{package_name}.scenario_modules",
        "kaiev26_export",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/config", glob("config/*.rviz")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/waypoints", glob("waypoints/*.yaml")),
    ],
    install_requires=["setuptools", "rich"],
    zip_safe=True,
    maintainer="user1",
    maintainer_email="user@example.com",
    description="Autonomous decision and route planning pipeline for KAIEV26.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perception_gateway_node = kaiev26_decision.perception_gateway_node:main",
            "route_zone_manager_node = kaiev26_decision.route_zone_manager_node:main",
            "main_planning_engine_node = kaiev26_decision.main_planning_engine_node:main",
            "debug_monitor_node = kaiev26_decision.debug_monitor_node:main",
            "constant_route_plan_node = kaiev26_decision.test_tracking:main",
            "route_readiness_node = kaiev26_decision.test_readiness:main",
            "steering_step_test_node = kaiev26_decision.test_steering_step:main",
            "decision_test = kaiev26_decision.test_tui:main",
            "kaiev26_export = kaiev26_export.cli:main",
        ],
    },
)
