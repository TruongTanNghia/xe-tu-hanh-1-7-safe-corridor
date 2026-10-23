from glob import glob
import os

from setuptools import find_packages, setup


package_name = "rc_localization"

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
    description="RF2O velocity conditioning and hardware EKF bringup",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "bno055_safe_node = rc_localization.bno055_safe_node:main",
            "velocity_conditioner = rc_localization.velocity_conditioner:main",
        ],
    },
)
