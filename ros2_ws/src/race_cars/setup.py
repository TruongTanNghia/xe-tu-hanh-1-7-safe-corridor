from glob import glob
import os

from setuptools import find_packages, setup


package_name = "race_cars"

setup(
    name=package_name,
    version="0.2.0",
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
    description="Track-relative NMPC with hardware safety gates",
    license="BSD-2-Clause",
    entry_points={
        "console_scripts": [
            "lane_reference_adapter = race_cars.lane_reference_adapter:main",
            "racecar_nmpc_node = race_cars.nmpc_node:main",
            "benchmark_solver = race_cars.benchmark_solver:main",
        ],
    },
)
