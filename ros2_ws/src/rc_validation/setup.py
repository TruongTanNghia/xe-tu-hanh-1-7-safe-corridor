from setuptools import find_packages, setup


package_name = "rc_validation"

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
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="RC Car Project",
    maintainer_email="phibui2112@gmail.com",
    description="Validation and SSH-friendly sensor viewers for the RC car",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lidar_web_viewer = rc_validation.lidar_web_viewer:main",
            "lidar_scan_snapshot = rc_validation.lidar_scan_snapshot:main",
            "velocity_evaluator = rc_validation.velocity_evaluator:main",
            "evaluate_velocity_bag = rc_validation.velocity_bag_evaluator:main",
            "check_imu_axis_bag = rc_validation.imu_axis_check:main",
            "longitudinal_recorder = rc_validation.longitudinal_recorder:main",
            "fit_longitudinal_model = rc_validation.longitudinal_fit:main",
        ],
    },
)
