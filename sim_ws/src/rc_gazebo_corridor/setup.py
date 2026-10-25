from glob import glob
import os

from setuptools import find_packages, setup

package_name = "rc_gazebo_corridor"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RC Car Project",
    maintainer_email="phibui2112@gmail.com",
    description="Gazebo Classic demo of the Safe Corridor stack",
    license="BSD-2-Clause",
    entry_points={
        "console_scripts": [
            "gazebo_corridor_bridge = rc_gazebo_corridor.gazebo_bridge_node:main",
            "build_world = rc_gazebo_corridor.world_builder:main",
        ],
    },
)
