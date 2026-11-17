"""Verify real launch assembly against transport doubles and installed-style files."""
import ast
import importlib.util
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

SOURCE = Path(__file__).resolve().parents[3]


class LaunchTests(unittest.TestCase):
    def helper(self):
        modules = {name: ModuleType(name) for name in (
            'ament_index_python', 'ament_index_python.packages',
            'launch_ros', 'launch_ros.actions', 'launch_ros.descriptions')}
        modules['ament_index_python.packages'].get_package_share_directory = lambda name: str(SOURCE / name)
        modules['launch_ros.actions'].Node = lambda **kw: kw
        modules['launch_ros.descriptions'].ComposableNode = lambda **kw: kw
        path = SOURCE / 'auto_control/auto_control_tunnel/launch_support.py'
        spec = importlib.util.spec_from_file_location('auto_control_tunnel._launch_test', path)
        module = importlib.util.module_from_spec(spec)
        with patch.dict('sys.modules', modules):
            spec.loader.exec_module(module)
        return module

    def test_shared_camera_integration_adds_no_camera_owner_or_actuators(self):
        rgb = dict(preview_enabled=False, result_topic='/custom/rgb_result')
        context = NS(launch_configurations=dict(path_minimum_points='12'))
        components, processes = self.helper().integrated_nodes(rgb, {}, context)
        self.assertEqual(rgb['inference_max_fps'], 70.)
        self.assertTrue(rgb['tunnel.enabled'])
        self.assertEqual([c['plugin'] for c in components],
                         ['ir_camera_driver::SharedIrBevNode', 'line_detactor::LineDetactorNode',
                          'auto_control::TunnelLaneSelector'])
        self.assertEqual(components[0]['parameters'][0], dict(
            validation_mode=True, standby_fps=0., active_fps=30., model_input_topic='/ir_bev_line_detection/image_bgr'))
        ir = components[1]['parameters'][0]
        self.assertFalse(ir['direct_bev_input_enabled'])
        self.assertEqual(ir['inference_max_fps'], 0.)
        self.assertIn('ir_stereo_bev_120x300.onnx', ir['model_path'])
        self.assertFalse(ir['preview_enabled'])
        self.assertFalse(ir['shared_ir_preview_output_enabled'])
        self.assertEqual([p['executable'] for p in processes], ['tunnel_tcn_node'])
        selector = components[-1]
        self.assertEqual(selector['package'], 'auto_control')
        self.assertEqual(selector['parameters'][-1]['path_minimum_points'], 12)
        self.assertEqual(selector['parameters'][-1]['rgb_result_topic'], '/custom/rgb_result')
        self.assertEqual(processes[0]['parameters'][-1]['initial_state'], 'outside')
        self.assertFalse(processes[0]['parameters'][-1]['preview_enabled'])

    def test_validation_mode_is_default_and_guarded_mode_is_selectable(self):
        for flag, expected in ((None, True), ('true', True), ('false', False)):
            config = {} if flag is None else dict(tunnel_validation_mode=flag)
            components, processes = self.helper().integrated_nodes(
                dict(preview_enabled=False, result_topic='/line_detactor/result'), {}, NS(launch_configurations=config))
            self.assertIs(components[0]['parameters'][0]['validation_mode'], expected)
            self.assertIs(components[-1]['parameters'][-1]['validation_mode'], expected)
        with self.assertRaises(ValueError):
            self.helper().integrated_nodes({}, {}, NS(launch_configurations=dict(tunnel_validation_mode='invalid')))

    def test_ir_length_exit_default_off_and_cli_overrides(self):
        _, nodes = self.helper().integrated_nodes(
            dict(preview_enabled=False, result_topic='/rgb'), {}, NS(launch_configurations={}))
        self.assertIs(nodes[-1]['parameters'][0]['ir_length_exit_enabled'], False)
        self.assertEqual(nodes[-1]['parameters'][0]['ir_length_exit_ratio'], .15)
        config = dict(tunnel_ir_length_exit_enabled='true', tunnel_ir_length_exit_ratio='.25',
                      tunnel_ir_length_exit_confirm_frames='4', tunnel_ir_length_exit_minimum_reference_m='.8')
        _, nodes = self.helper().integrated_nodes(
            dict(preview_enabled=False, result_topic='/rgb'), {}, NS(launch_configurations=config))
        params = nodes[-1]['parameters'][0]
        self.assertIs(params['ir_length_exit_enabled'], True)
        self.assertEqual(params['ir_length_exit_ratio'], .25)
        self.assertEqual(params['ir_length_exit_confirm_frames'], 4)
        self.assertEqual(params['ir_length_exit_minimum_reference_m'], .8)
        for key, value in (('tunnel_ir_length_exit_enabled', 'maybe'),
                           ('tunnel_ir_length_exit_ratio','nan'),
                           ('tunnel_ir_length_exit_confirm_frames','0')):
            with self.assertRaises(ValueError):
                self.helper().integrated_nodes({}, {}, NS(launch_configurations={key:value}))
        tree = ast.parse((SOURCE/'vehicle_bringup/launch/auto_drive.launch.py').read_text())
        declared = {n.args[0].value: next(k.value.value for k in n.keywords if k.arg=='default_value')
                    for n in ast.walk(tree) if isinstance(n,ast.Call) and
                    isinstance(n.func,ast.Name) and n.func.id=='DeclareLaunchArgument' and
                    n.args and isinstance(n.args[0],ast.Constant) and
                    str(n.args[0].value).startswith('tunnel_ir_length_exit_')}
        self.assertEqual(declared['tunnel_ir_length_exit_enabled'],'false')
        self.assertEqual(declared['tunnel_ir_length_exit_ratio'],'0.15')

    def test_selector_inherits_effective_controller_geometry(self):
        controller = dict(path_minimum_points=12, path_minimum_span_m=.2, path_maximum_x_m=1.8)
        context = NS(launch_configurations=dict(path_minimum_points='16', path_maximum_gap_m='.1'))
        _, processes = self.helper().integrated_nodes(
            dict(preview_enabled=False, result_topic='/line_detactor/result'), controller, context)
        parameters = processes[-1]['parameters'][-1]
        self.assertEqual(parameters['path_minimum_points'], 16)
        self.assertEqual(parameters['path_minimum_span_m'], .2)
        self.assertEqual(parameters['path_maximum_x_m'], 1.8)
        self.assertEqual(parameters['path_maximum_gap_m'], .1)

    def test_validation_flag_reaches_camera_included_launch(self):
        bringup = ast.parse((SOURCE/'vehicle_bringup/launch/auto_drive.launch.py').read_text())
        bev = ast.parse((SOURCE/'bev_processor/launch/bev_processor.launch.py').read_text())
        # Exercise the actual launch declarations/parameter expressions with doubles.
        for tree in (bringup, bev):
            declaration = next(n for n in ast.walk(tree) if isinstance(n, ast.Call)
                and isinstance(n.func, ast.Name) and n.func.id == 'DeclareLaunchArgument'
                and n.args and isinstance(n.args[0], ast.Constant) and n.args[0].value == 'tunnel_validation_mode')
            self.assertEqual(ast.literal_eval(next(k.value for k in declaration.keywords
                                                  if k.arg == 'default_value')), 'true')
        config = next(n for n in ast.walk(bev) if isinstance(n, ast.Dict)
                      and any(isinstance(k, ast.Constant) and k.value == 'tunnel.validation_mode' for k in n.keys))
        value = next(v for k, v in zip(config.keys, config.values)
                     if isinstance(k, ast.Constant) and k.value == 'tunnel.validation_mode')
        self.assertEqual(eval(compile(ast.Expression(value), '<launch parameter>', 'eval'), dict(
            ParameterValue=lambda value, value_type: value_type(value),
            LaunchConfiguration=lambda key: {'tunnel_validation_mode': True}[key])), True)

    def test_tunnel_ir_imu_switch_reaches_camera_and_preserves_rgb_setting(self):
        bev = ast.parse((SOURCE/'bev_processor/launch/bev_processor.launch.py').read_text())
        bringup = ast.parse((SOURCE/'vehicle_bringup/launch/auto_drive.launch.py').read_text())
        for tree in (bev, bringup):
            declaration = next(n for n in ast.walk(tree) if isinstance(n, ast.Call)
                and isinstance(n.func, ast.Name) and n.func.id == 'DeclareLaunchArgument'
                and n.args and isinstance(n.args[0], ast.Constant) and n.args[0].value == 'tunnel_ir_imu_enabled')
            self.assertEqual(ast.literal_eval(next(k.value for k in declaration.keywords
                                                  if k.arg == 'default_value')), 'true')
        config = next(n for n in ast.walk(bev) if isinstance(n, ast.Dict)
                      and any(isinstance(k, ast.Constant) and k.value == 'tunnel.ir_imu_enabled' for k in n.keys))
        value = next(v for k, v in zip(config.keys, config.values)
                     if isinstance(k, ast.Constant) and k.value == 'tunnel.ir_imu_enabled')
        for enabled in (True, False):
            self.assertIs(eval(compile(ast.Expression(value), '<IR IMU parameter>', 'eval'), dict(
                ParameterValue=lambda value, value_type: value_type(value),
                LaunchConfiguration=lambda key: {'tunnel_ir_imu_enabled': enabled}[key])), enabled)
        forwarded = next(n for n in ast.walk(bringup) if isinstance(n, ast.Dict)
                         and any(isinstance(k, ast.Constant) and k.value == 'tunnel_ir_imu_enabled' for k in n.keys))
        expression = next(v for k, v in zip(forwarded.keys, forwarded.values)
                          if isinstance(k, ast.Constant) and k.value == 'tunnel_ir_imu_enabled')
        self.assertEqual(ast.literal_eval(expression.args[0]), 'tunnel_ir_imu_enabled')

    def test_one_window_and_same_result_only_selection_for_rgb_and_ir(self):
        for result_only in (True, False):
            with self.subTest(result_only=result_only):
                rgb = dict(preview_enabled=True, preview_result_only_enabled=result_only,
                           bev_theme_enable=False, result_topic='/line_detactor/result')
                components, _ = self.helper().integrated_nodes(rgb, {}, NS(launch_configurations={}))
                ir = components[1]['parameters'][0]
                self.assertEqual(sum(p['preview_enabled'] for p in (rgb, ir)), 1)
                self.assertFalse(ir['bev_theme_enable'])
                self.assertTrue(ir['shared_ir_preview_output_enabled'])
                self.assertEqual(ir['preview_result_only_enabled'], result_only)

    def test_model_check_tracks_only_enabled_inference_nodes(self):
        helper = self.helper()
        for tunnel in (False, True):
            for traffic in (False, True):
                expected = ['line_detactor']
                if tunnel:
                    expected += ['ir_line_detactor', 'tunnel_tcn_test']
                if traffic:
                    expected += ['traffic_light_detector']
                node = helper.model_check_node(tunnel, traffic)
                self.assertEqual(node['parameters'][0]['expected_nodes'], expected)
                self.assertEqual(node['executable'], 'model_startup_check')
                inspection = helper.startup_inspection_node(tunnel, traffic)
                self.assertEqual(inspection['parameters'][0]['expected_nodes'], expected)
                self.assertEqual(inspection['executable'], 'startup_inspection')

    def test_ir_model_input_contract_matches_working_standalone_test(self):
        path = SOURCE/'ir_bev_line_detection/ir_bev_line_detection/settings.py'
        spec = importlib.util.spec_from_file_location('_ir_standalone_settings', path)
        standalone = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(standalone)
        shares = {name: SOURCE/name for name in ('ir_camera_driver', 'line_detactor', 'auto_control')}
        settings = standalone.build_parameters(shares,
            SOURCE/'ir_bev_line_detection/config/ir_bev_test.yaml', '.3')
        components, _ = self.helper().integrated_nodes(
            dict(preview_enabled=True, result_topic='/line_detactor/result'), {}, NS(launch_configurations={}))
        integrated = components[1]['parameters'][0]
        for key in ('engine_precision', 'model_input_width', 'model_input_height', 'mask_threshold',
                    'connection_enabled', 'centerline_enabled', 'result_publish_enabled',
                    'centerline_bev_width_m', 'centerline_bev_height_m', 'stop_line_mask_threshold',
                    'stop_line_distance_enabled'):
            self.assertEqual(integrated[key], settings['line_detactor'][key], key)

    def test_auto_drive_requires_model_gate(self):
        path = SOURCE / 'vehicle_bringup/launch/auto_drive.launch.py'
        tree = ast.parse(path.read_text())
        override = next(n for n in ast.walk(tree) if isinstance(n, ast.Dict)
                        and any(isinstance(k, ast.Constant) and k.value == 'model_startup_check_required'
                                for k in n.keys))
        value = next(v for k, v in zip(override.keys, override.values)
                     if isinstance(k, ast.Constant) and k.value == 'model_startup_check_required')
        self.assertIs(ast.literal_eval(value), True)

    def speed_override(self, speed):
        path = SOURCE / 'vehicle_bringup/launch/auto_drive.launch.py'
        function = next(n for n in ast.parse(path.read_text()).body
                        if isinstance(n, ast.FunctionDef) and n.name == '_apply_speed_override')
        context = NS(launch_configurations=dict(auto_enabled='false'))
        import math
        namespace = dict(math=math)
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), 'exec'), namespace)
        namespace['_apply_speed_override'](context, speed)
        return context.launch_configurations

    def test_requested_speed_has_no_point_three_ceiling_and_preserves_enable_choice(self):
        args = self.speed_override('1.5')
        self.assertEqual(args['minimum_speed_mps'], '1.5')
        self.assertEqual(args['maximum_speed_mps'], '1.5')
        self.assertEqual(args['auto_enabled'], 'false')
        self.assertNotIn('maximum_speed_mps', self.speed_override('inherit'))
        for value in ('0', '-1', 'nan', 'inf'):
            with self.assertRaises(ValueError):
                self.speed_override(value)


if __name__ == '__main__':
    unittest.main()
