from glob import glob
import os

from setuptools import find_packages, setup


package_name = "rc_safe_corridor"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml") + glob("config/*.rviz"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="RC Car Project",
    maintainer_email="phibui2112@gmail.com",
    description=(
        "LiDAR obstacle tracking, Frenet safe corridor planning, "
        "behavior FSM and safety monitor"
    ),
    license="BSD-2-Clause",
    entry_points={
        "console_scripts": [
            "obstacle_detector = rc_safe_corridor.obstacle_detector_node:main",
            "safe_corridor_planner = "
            "rc_safe_corridor.safe_corridor_planner_node:main",
            "safety_monitor = rc_safe_corridor.safety_monitor_node:main",
            "corridor_offline_demo = rc_safe_corridor.offline_demo:main",
            "sim_world = rc_safe_corridor.sim_world_node:main",
        ],
    },
)
