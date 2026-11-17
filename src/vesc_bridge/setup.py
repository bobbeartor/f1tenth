from setuptools import find_packages, setup


package_name = "vesc_bridge"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ohslo",
    maintainer_email="ohslo@example.com",
    description="ROS 2 bridge for VESC serial command and telemetry communication.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "vesc_bridge_node = vesc_bridge.vesc_bridge_node:main",
        ],
    },
)
