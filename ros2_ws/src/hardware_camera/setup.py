from glob import glob
import os

from setuptools import find_packages, setup


package_name = "hardware_camera"

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
    zip_safe=True,
    maintainer="RC Car Project",
    maintainer_email="phibui2112@gmail.com",
    description="Native Picamera2 adapter for the Pi 5 IMX708 Wide camera",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "camera_node = hardware_camera.camera_node:main",
            "save_image = hardware_camera.save_image:main",
            "web_viewer = hardware_camera.web_viewer:main",
        ],
    },
)
