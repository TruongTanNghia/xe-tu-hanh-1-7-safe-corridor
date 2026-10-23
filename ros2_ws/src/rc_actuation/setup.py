from glob import glob
import os

from setuptools import find_packages, setup


package_name = "rc_actuation"

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
            glob("config/*.yaml"),
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
    description="Safety-gated typed command to micro-ROS PWM adapter",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "actuation_manager = rc_actuation.actuation_manager:main",
            "esc_step_source = rc_actuation.esc_step_source:main",
            "rolling_step_source = rc_actuation.rolling_step_source:main",
        ],
    },
)
