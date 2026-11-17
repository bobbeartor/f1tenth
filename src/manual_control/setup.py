from setuptools import find_packages, setup


package_name = "manual_control"

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
    description="SDL-standard Bluetooth manual controller input for AutoDrive.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "joy_params_converter_node = manual_control.joy_params_converter_node:main",
            "actuator_commander_node = manual_control.actuator_commander_node:main",
            "auto_handover_node = manual_control.auto_handover_node:main",
            "vfh_handover_node = manual_control.vfh_handover_node:main",
        ],
    },
)
