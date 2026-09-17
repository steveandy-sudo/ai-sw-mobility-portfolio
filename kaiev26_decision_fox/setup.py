from glob import glob
import os

from setuptools import find_packages, setup


package_name = "kaiev26_decision_fox"


def foxglove_files():
    files = []
    for pattern in ("foxglove/*.json", "foxglove/*.md", "foxglove/*.foxe"):
        files.extend(glob(pattern))
    return files


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "foxglove"), foxglove_files()),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Team KAI",
    maintainer_email="user@example.com",
    description="Read-only Foxglove visualization support for KAIEV26 decision data.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fox_node = kaiev26_decision_fox.fox_node:main",
        ],
    },
)
