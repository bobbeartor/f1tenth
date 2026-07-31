from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    keymap_path = PathJoinSubstitution(
        [FindPackageShare("vehicle_config"), "config", "controller_keymap.yaml"]
    )
    actuator_commander_config = PathJoinSubstitution(
        [FindPackageShare("vehicle_config"), "config", "manual_vesc_config.yaml"]
    )
    vesc_config = PathJoinSubstitution(
        [FindPackageShare("vehicle_config"), "config", "vesc_config.yaml"]
    )
    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare("joy_initializer"), "launch", "joy.launch.py"]
    )

    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joy_launch_path),
        launch_arguments={
            "device_id": "0",
            "device_name": "",
            "deadzone": "0.05",
            "autorepeat_rate": "50.0",
            "sticky_buttons": "false",
            "coalesce_interval_ms": "1",
        }.items(),
    )

    joy_params_converter_node = Node(
        package="manual_control",
        executable="joy_params_converter_node",
        name="joy_params_converter_node",
        output="screen",
        parameters=[
            actuator_commander_config,
            {
                "keymap_path": keymap_path,
                "joy_topic": "/joy",
                "accelerator_topic": "/manual/accelerator",
                "brake_topic": "/manual/brake",
                "steering_topic": "/manual/steering",
                "gear_toggle_topic": "/manual/gear_toggle",
                "debug_topic": "/manual/controller_debug",
                "publish_debug": True,
            }
        ],
    )

    actuator_commander_node = Node(
        package="manual_control",
        executable="actuator_commander_node",
        name="actuator_commander_node",
        output="screen",
        parameters=[actuator_commander_config],
    )

    vesc_initialize_node = Node(
        package="vesc_initializer",
        executable="vesc_initialize_node",
        name="vesc_initialize_node",
        output="screen",
        parameters=[vesc_config],
    )

    return LaunchDescription(
        [
            joy_launch,
            joy_params_converter_node,
            actuator_commander_node,
            vesc_initialize_node,
        ]
    )
