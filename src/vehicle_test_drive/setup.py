from glob import glob
import os

from setuptools import find_packages, setup


package_name = "vehicle_test_drive"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ohslo",
    maintainer_email="ohslo@example.com",
    description="Direct steering servo and ERPM test sequence for AutoDrive.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "servo_angle_calibrator = "
            "vehicle_test_drive.servo_angle_calibrator_node:main",
            "vehicle_test_drive = vehicle_test_drive.vehicle_test_drive_node:main",
            "vehicle_test_drive_node = vehicle_test_drive.vehicle_test_drive_node:main",
        ],
    },
)
