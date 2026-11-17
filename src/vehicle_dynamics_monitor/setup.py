from glob import glob
import os

from setuptools import find_packages, setup


package_name = "vehicle_dynamics_monitor"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "python-can>=4.0", "pyserial>=3.5"],
    zip_safe=True,
    maintainer="ohslo",
    maintainer_email="ohslo@example.com",
    description="Receive-only VESC telemetry and vehicle dynamics estimation.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "vehicle_dynamics_node = "
            "vehicle_dynamics_monitor.vehicle_dynamics_node:main",
        ],
    },
)
