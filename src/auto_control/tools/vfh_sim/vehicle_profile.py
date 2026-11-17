"""Read the scalar ROS policy settings used by roi_vfh_test, without ROS/PyYAML."""
from pathlib import Path
import hashlib
import json
import math
import shlex

HERE = Path(__file__).resolve().parent
SRC = HERE.parents[2]
REFERENCE = SRC / 'roi_vfh_test/config/reference'


def parameters(path, node):
    # These ROS files use flat scalar ros__parameters. Complex YAML is never
    # interpreted as a value for a required numeric/bool field: conversion fails.
    result, selected, inside = {}, False, False
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        indent = len(line) - len(line.lstrip())
        if indent == 0:
            selected = line.rstrip().rstrip(':') == node
            inside = False
        elif selected and indent == 2:
            inside = line.strip() == 'ros__parameters:'
        elif selected and inside and indent == 4 and ':' in line:
            key, raw = line.strip().split(':', 1)
            lexer = shlex.shlex(raw, posix=True)
            lexer.whitespace_split, lexer.commenters = True, '#'
            tokens = list(lexer)
            if len(tokens) != 1:
                continue
            value = tokens[0]
            if value.lower() in ('true', 'false'):
                value = value.lower() == 'true'
            else:
                try:
                    value = float(value)
                except ValueError:
                    pass
            result[key] = value
    if not result:
        raise ValueError(f'{path}: missing scalar {node}.ros__parameters')
    return result


# Simulator field -> Config member -> ROS preview parameter.
EDITABLE = {
    'speed': ('speed', 'nominal_speed_mps'),
    'horizon': ('horizon', 'horizon_m'),
    'sampleStep': ('step', 'sample_step_m'),
    'margin': ('margin', 'safety_margin_m'),
    'laneLookahead': ('lane_lookahead', 'lane_lookahead_m'),
    'referenceDistance': ('reference_distance', 'reference_distance_m'),
    'lowThreshold': ('hist_low', 'histogram_low'),
    'highThreshold': ('hist_high', 'histogram_high'),
    'deceleration': ('deceleration', 'deceleration_mps2'),
    'responseDelay': ('delay', 'response_delay_sec'),
    'laneMemoryEnabled': ('lane_memory_enabled', 'lane_memory_enabled'),
    'hazardEnabled': ('hazard_filter_enabled', 'hazard_filter_enabled'),
    'hazardFrames': ('hazard_confirm_frames', 'hazard_confirm_frames'),
    'hazardSmall': ('hazard_small_max_cells', 'hazard_small_max_cells'),
    'hazardRadius': ('hazard_match_radius_cells', 'hazard_match_radius_cells'),
    'hazardGap': ('hazard_max_gap_sec', 'hazard_max_gap_sec'),
    'extraCurves': ('avoidance_curve_enabled', 'avoidance_curve_enabled'),
    'curveSamples': ('avoidance_curve_samples', 'avoidance_curve_samples'),
    'lengthSlack': ('avoidance_length_slack_m', 'avoidance_length_slack_m'),
    'laneWeight': ('avoidance_lane_weight', 'avoidance_lane_weight'),
    'steeringWeight': ('avoidance_steering_weight', 'avoidance_steering_weight'),
    'clearanceWeight': ('clearance_weight', 'avoidance_clearance_weight'),
    'uncertainty': ('uncertainty', 'obstacle_uncertainty_m'),
    'hazardHold': ('hazard_hold_sec', 'hazard_hold_sec'),
    'obstacleMemoryEnabled': ('obstacle_memory_enabled', 'obstacle_memory_enabled'),
    'obstacleMemorySeconds': ('obstacle_memory_sec', 'obstacle_memory_sec'),
    'obstacleMemoryDistance': ('obstacle_memory_distance_m', 'obstacle_memory_distance_m'),
    'obstacleMemoryClearance': ('obstacle_memory_clearance_m', 'obstacle_memory_clearance_m'),
    'targetAcceleration': ('target_recovery_acceleration', 'target_recovery_acceleration_mps2'),
}


def load_profile(params_file=None, reference_dir=None):
    ref = Path(reference_dir) if reference_dir else REFERENCE
    main = Path(params_file) if params_file else SRC / 'roi_vfh_test/config/preview.yaml'
    sources = [main, *(ref / name for name in (
        'auto_control_test.yaml', 'obstacles_test.yaml', 'camera_config_test.yaml',
        'bev_config_test.yaml', 'line_detactor_test.yaml', 'roi_lidar_test.yaml')),
        SRC / 'roi_lidar/config/roi_lidar.yaml']
    preview = parameters(main, 'roi_vfh_test')
    control = parameters(ref / 'auto_control_test.yaml', 'auto_control')
    obstacle = parameters(ref / 'obstacles_test.yaml', 'auto_obstacles')
    depth = parameters(ref / 'obstacles_test.yaml', 'camera_driver')
    camera = parameters(ref / 'camera_config_test.yaml', 'camera_driver')
    bev = parameters(ref / 'bev_config_test.yaml', 'bev_processor')
    lane = parameters(ref / 'line_detactor_test.yaml', 'line_detactor')
    roi = parameters(SRC / 'roi_lidar/config/roi_lidar.yaml', 'roi_lidar')
    roi.update(parameters(ref / 'roi_lidar_test.yaml', 'roi_lidar'))
    defaults, cpp = {}, {}
    introduced = {'obstacle_uncertainty_m': obstacle['obstacles.avoidance.unknown_extent_m'],
                  'hazard_hold_sec': .30, 'obstacle_memory_enabled': 1., 'obstacle_memory_sec': 2.,
                  'obstacle_memory_distance_m': 1., 'obstacle_memory_clearance_m': .10,
                  'target_recovery_acceleration_mps2': control['corner_planning_acceleration_mps2']}
    for key, (member, parameter) in EDITABLE.items():
        value = preview[parameter] if parameter in preview else introduced[parameter]
        defaults[key] = cpp[member] = float(value)
    speed_limit = float(control['maximum_speed_mps'])
    deceleration_limit = min(control['longitudinal_planning_deceleration_mps2'],
                             obstacle['obstacles.avoidance.deceleration_mps2'])
    defaults['speed'] = cpp['speed'] = min(defaults['speed'], speed_limit)
    defaults['deceleration'] = cpp['deceleration'] = min(defaults['deceleration'], deceleration_limit)
    for key, member, parameter, fallback in (
        ('width', 'width', 'vehicle_width_m', .30),
        ('wheelbase', 'wheelbase', 'wheelbase_m', control['avoidance_wheelbase_m']),
        ('frontOverhang', 'front', 'front_overhang_m', .05),
        ('rearExtent', 'rear', 'rear_extent_m', .45),
    ):
        defaults[key] = cpp[member] = float(preview.get(parameter, fallback))
    defaults['length'] = defaults['frontOverhang'] + defaults['rearExtent']
    defaults['maxSteer'] = preview.get('max_steering_deg', control['maximum_steering_angle_deg'])
    defaults['steerRate'] = preview.get('steering_rate_deg_per_sec', control['steering_rate_limit_deg_per_sec'])
    defaults['laneWidth'] = cpp['lane_width'] = lane['centerline_lane_width_m']
    cpp.update(max_steer=math.radians(defaults['maxSteer']), steer_rate=math.radians(defaults['steerRate']),
               uncertainty=cpp['uncertainty'],
               max_curvature=preview.get('max_curvature_per_m', obstacle['obstacles.avoidance.max_curvature_per_m']),
               lane_lateral_acceleration=control['maximum_lateral_acceleration_mps2'],
               avoidance_lateral_acceleration=obstacle['obstacles.avoidance.lateral_acceleration_mps2'],
               curvature_speed_control=control['curvature_speed_control_enabled'],
               stanley_gain=preview.get('stanley_gain', control['stanley_gain']),
               stanley_softening=preview.get('stanley_softening_speed_mps', control['stanley_softening_speed_mps']),
               stanley_corner_heading=math.radians(preview.get('stanley_corner_heading_threshold_deg', control['stanley_corner_heading_threshold_deg'])),
               stanley_opposing=preview.get('stanley_corner_opposing_correction_ratio', control['stanley_corner_opposing_correction_ratio']),
               steering_weight=preview.get('steering_current_weight', control['steering_current_weight']), control_rate=control['control_rate_hz'],
               minimum_path_length=control['path_minimum_span_m'], geometry_window=control['path_geometry_window_m'])
    rgb_rates = [camera['sensor_fps'], camera['publish_fps']]
    if bev['publish_max_fps'] > 0:
        rgb_rates.append(bev['publish_max_fps'])
    defaults.update(planHz=obstacle['obstacles.avoidance.max_fps'], previewHz=bev['preview_max_fps'], rgbHz=min(rgb_rates),
                    depthHz=depth['obstacles.camera.fps'], controlHz=control['control_rate_hz'],
                    imuHz=camera['imu_rate_hz'], steeringCurrentWeight=cpp['steering_weight'],
                    stanleyGain=cpp['stanley_gain'], stanleySoftening=cpp['stanley_softening'],
                    maxCurvature=cpp['max_curvature'], cornerHeading=math.degrees(cpp['stanley_corner_heading']),
                    cornerOpposing=cpp['stanley_opposing'],
                    terrainWidth=roi['preview.width_m'], terrainForward=roi['preview.forward_m'],
                    terrainCell=roi['terrain.cell_m'], laneVisibleLength=bev['x_max_m'],
                    acceleration=control['corner_planning_acceleration_mps2'],
                    laneMinX=control['path_minimum_x_m'], laneMaxX=control['path_maximum_x_m'],
                    laneBevHalfWidth=(bev['y_max_m']-bev['y_min_m'])/2)
    runtime = dict(speed_ceiling=speed_limit, deceleration_ceiling=deceleration_limit,
                   max_age=preview['max_age_sec'], slowdown=control['obstacle_slowdown_enabled'],
                   slowdown_cap=control['obstacle_slowdown_speed_mps'],
                   clear_sec=control['obstacle_slowdown_clear_sec'],
                   recovery_acceleration=defaults['targetAcceleration'],
                   steering_ceiling_deg=control['maximum_steering_angle_deg'],
                   steering_rate_ceiling_deg_per_sec=control['steering_rate_limit_deg_per_sec'])
    for group in (defaults,cpp,runtime):
        if any(not isinstance(value,(int,float,bool)) or not math.isfinite(value) for value in group.values()):
            raise ValueError('required vehicle policy values must be finite scalars')
    return dict(defaults=defaults, cpp=cpp, runtime=runtime,
                sources=[str(p.resolve()) for p in sources])


def fingerprint(profile):
    paths = [*(Path(p) for p in profile['sources']), *HERE.glob('*.js'),
             HERE/'native_planner.cpp', HERE/'serve.py', HERE/'vehicle_profile.py', HERE/'view.html',
             * (SRC/'roi_vfh_test/include').rglob('*.hpp'),
             SRC/'auto_control/src/control_core.cpp', SRC/'auto_control/include/auto_control/control_core.hpp',
             SRC/'roi_vfh_test/src/preview_node.cpp', SRC/'roi_vfh_test/launch/preview.launch.py']
    digest = hashlib.sha256()
    for path in sorted(set(paths)):
        digest.update(str(path).encode()); digest.update(path.read_bytes())
    return digest.hexdigest()[:16]


def write_header(path, profile):
    def literal(value):
        return ('true' if value else 'false') if isinstance(value, bool) else repr(float(value))
    lines = ['#pragma once', 'inline SimulationProfile simulationProfile() {', '  SimulationProfile p;']
    for key, value in profile['cpp'].items():
        lines.append(f'  p.config.{key}={literal(value)};')
    for key, value in profile['runtime'].items():
        lines.append(f'  p.{key}={literal(value)};')
    lines += [f'  p.fingerprint={json.dumps(profile["fingerprint"])};', '  return p;', '}']
    Path(path).write_text('\n'.join(lines)+'\n')
