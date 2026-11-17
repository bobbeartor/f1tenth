from glob import glob
import os

from setuptools import find_packages, setup


package_name = "auto_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (
            f"share/{package_name}",
            ["package.xml", "AUTO_CONTROL_PARAMETER_TUNING_KO.txt", "AUTO_CONTROL_TUNNEL.md"],
        ),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ohslo",
    maintainer_email="ohslo@example.com",
    description="Stanley lateral and curvature-aware speed control.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "auto_control_node = auto_control.auto_control_node:main",
            "tunnel_lane_selector = auto_control_tunnel.node:main",
        ],
    },
)
