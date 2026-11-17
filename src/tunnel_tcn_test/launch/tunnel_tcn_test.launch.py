import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    share = Path(get_package_share_directory('tunnel_tcn_test'))
    camera_share = Path(get_package_share_directory('ir_camera_driver'))
    arg = lambda name: LaunchConfiguration(name).perform(context)
    threshold, warmup = float(arg('threshold')), float(arg('warmup_seconds'))
    initial = arg('initial_state')
    if not (0 <= threshold <= 1 and warmup >= 0 and initial in ('unknown', 'outside', 'inside')):
        raise ValueError('Invalid threshold, warmup_seconds or initial_state')
    library = (camera_share / 'depthai_runtime_dir.txt').read_text().strip()
    camera = Node(
        package='ir_camera_driver', executable='ir_camera_driver_node', name='ir_camera_driver', output='screen',
        additional_env={'LD_LIBRARY_PATH': os.pathsep.join(filter(None, [library, context.environment.get('LD_LIBRARY_PATH', '')]))},
        parameters=[str(camera_share / 'config/ir_camera_config.yaml'), {
            'device_id': arg('device_id'), 'reprojection_enabled': True,
            'virtual_camera_position_ratio': .5, 'width': 1280, 'height': 800,
            'reprojection_fps': 30., 'single_camera_fps': 30., 'preview_max_fps': 30.,
            'ir_enabled': False, 'ir_dot_projector_intensity': 0., 'ir_flood_light_intensity': 0.,
            'rgb_enabled': True, 'rgb_width': 1280, 'rgb_height': 800, 'rgb_fps': 30.,
            'rgb_undistort_enabled': True, 'rgb_topic': '/camera/image_rect',
            'bev_processing_enabled': False, 'use_sensor_timestamps': True,
            'stereo_preview_enabled': False, 'rgb_preview_enabled': False,
            'bev_preview_enabled': False, 'controls_preview_enabled': False,
            'bev.startup_measurement_enabled': False, 'imu_stabilization_enabled': False,
            'measurement_roi_preview_enabled': False,
        }])
    inference = Node(package='tunnel_tcn_test', executable='tunnel_tcn_node',
                     name='tunnel_tcn_test', output='screen',
                     parameters=[str(share / 'config/tunnel_tcn.yaml'), {
                         'threshold': threshold, 'warmup_seconds': warmup, 'initial_state': initial,
                     }])
    return [
        RegisterEventHandler(OnProcessExit(target_action=camera, on_exit=[EmitEvent(event=Shutdown(reason='Camera closed'))])),
        RegisterEventHandler(OnProcessExit(target_action=inference, on_exit=[EmitEvent(event=Shutdown(reason='Inference closed'))])),
        camera, inference,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('threshold', default_value='0.5'),
        DeclareLaunchArgument('warmup_seconds', default_value='2.5'),
        DeclareLaunchArgument('initial_state', default_value='unknown', description='unknown, outside or inside'),
        DeclareLaunchArgument('device_id', default_value='', description='Optional OAK device MXID'),
        OpaqueFunction(function=start),
    ])
