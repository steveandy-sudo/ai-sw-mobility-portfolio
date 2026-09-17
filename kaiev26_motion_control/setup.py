from glob import glob
from setuptools import setup


package_name = "kaiev26_motion_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="user1",
    maintainer_email="user@example.com",
    description="Path following and speed planning for KAIEV26.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "motion_control_node = kaiev26_motion_control.motion_control_node:main",
        ],
    },
)
