from glob import glob
import os

from setuptools import find_packages, setup


package_name = "jetson_resource_monitor"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ohslo",
    maintainer_email="ohslo@example.com",
    description="Board-wide Jetson and per-process ROS 2 resource monitor.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "resource_monitor_node = "
            "jetson_resource_monitor.resource_monitor_node:main",
        ],
    },
)

