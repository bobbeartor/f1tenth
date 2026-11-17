"""Merge production defaults with test overrides, without importing ROS."""
import math
from pathlib import Path
import yaml


def parameters(path, node):
    document = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    return dict(document[node]['ros__parameters'])


def build_parameters(shares, config_path, speed):
    speed = float(speed)
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError('speed_mps must be a finite positive number')
    specs = {
        'ir_camera_driver': ('ir_camera_driver', 'ir_camera_config.yaml'),
        'line_detactor': ('line_detactor', 'line_detactor.yaml'),
        'auto_control': ('auto_control', 'auto_control.yaml'),
    }
    result = {}
    for node, (package, filename) in specs.items():
        values = parameters(Path(shares[package]) / 'config' / filename, node)
        values.update(parameters(config_path, node))
        result[node] = values
    camera, detector, controller = (result[key] for key in specs)
    expected = {
        'bev.x_min_m': 0., 'bev.x_max_m': 3.,
        'bev.y_min_m': -.6, 'bev.y_max_m': .6,
        'bev.meter_per_pixel': .01, 'bev.output_width': 120, 'bev.output_height': 300,
    }
    for name, value in expected.items():
        if not math.isclose(float(camera[name]), value, abs_tol=1e-9):
            raise ValueError(f'Trained IR BEV geometry requires {name}={value}')
    if camera['bev_frame_id'] != 'front_axle_bev':
        raise ValueError('IR BEV must use front_axle_bev')
    camera['bev_processing_enabled'] = True
    detector.update(
        direct_bev_input_enabled=False, model_input_width=120, model_input_height=300,
        connection_enabled=True, centerline_enabled=True, result_publish_enabled=True,
        centerline_bev_width_m=1.2, centerline_bev_height_m=3.,
    )
    # Geometry must agree across camera, lane fitting and pixel-to-metre control.
    controller.update(
        minimum_speed_mps=speed, maximum_speed_mps=speed,
        bev_x_max_m=3., bev_y_max_m=.6, bev_meter_per_pixel=.01,
        lane_result_topic=detector['result_topic'], lane_result_frame_id='front_axle_bev',
        longitudinal_pid_enabled=True,
        traffic_stop_enabled=False, avoidance_control_enabled=False,
        obstacle_slowdown_enabled=False,
    )
    return result
