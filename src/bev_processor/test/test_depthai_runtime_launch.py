"""Exercise runtime SDK precedence without ROS, CUDA or a connected camera."""
import ast
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
from types import SimpleNamespace as NS
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / 'launch/bev_processor.launch.py'


class DepthaiRuntimeTests(unittest.TestCase):
    def setUp(self):
        tree = ast.parse(LAUNCH.read_text())
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                        and n.name == '_configure_depthai_runtime')
        namespace = dict(os=os, Path=Path,
                         LaunchConfiguration=lambda name: NS(perform=lambda ctx: ctx.launch_configurations[name]),
                         SetLaunchConfiguration=lambda name, value: ('set', name, value),
                         LogInfo=lambda **kw: ('log', kw['msg']))
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(LAUNCH), 'exec'), namespace)
        self.configure = namespace['_configure_depthai_runtime']

    def test_build_sdk_precedes_incompatible_environment_without_mutating_parent(self):
        with tempfile.TemporaryDirectory() as directory:
            share = Path(directory)
            library = share / 'matching SDK/lib'
            library.mkdir(parents=True)
            (share / 'depthai_runtime_dir.txt').write_text(str(library)+'\n')
            for previous in ('', '/other/depthai/lib:/opt/ros/humble/lib'):
                context = NS(launch_configurations={'depthai_library_dir': ''},
                             environment={'LD_LIBRARY_PATH': previous})
                actions = self.configure(context, str(share))
                expected = str(library) + (os.pathsep+previous if previous else '')
                self.assertEqual(actions[0], ('set', 'depthai_runtime_library_path', expected))
                self.assertEqual(context.environment['LD_LIBRARY_PATH'], previous)

    def test_explicit_library_directory_works_without_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            context = NS(launch_configurations={'depthai_library_dir': directory}, environment={})
            self.assertEqual(self.configure(context, directory)[0][2], directory)

    def test_missing_empty_or_deleted_build_sdk_fails_with_actionable_error(self):
        with tempfile.TemporaryDirectory() as directory:
            context = NS(launch_configurations={'depthai_library_dir': ''}, environment={})
            with self.assertRaisesRegex(RuntimeError, 'rebuild bev_processor'):
                self.configure(context, directory)
            metadata = Path(directory)/'depthai_runtime_dir.txt'
            metadata.write_text('\n')
            with self.assertRaisesRegex(RuntimeError, 'empty'):
                self.configure(context, directory)
            metadata.write_text(str(Path(directory)/'deleted'))
            with self.assertRaisesRegex(RuntimeError, 'does not exist'):
                self.configure(context, directory)

    def test_container_receives_resolved_environment_before_loading_components(self):
        tree = ast.parse(LAUNCH.read_text())
        calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
        configure = next(n for n in calls if isinstance(n.func, ast.Name) and n.func.id == 'OpaqueFunction'
                         and any(k.arg == 'function' and isinstance(k.value, ast.Name)
                                 and k.value.id == '_configure_depthai_runtime' for k in n.keywords))
        container = next(n for n in calls if isinstance(n.func, ast.Name)
                         and n.func.id == 'ComposableNodeContainer')
        self.assertLess(configure.lineno, container.lineno)
        environment = next(k.value for k in container.keywords if k.arg == 'additional_env')
        self.assertEqual(ast.literal_eval(environment.keys[0]), 'LD_LIBRARY_PATH')
        self.assertEqual(ast.literal_eval(environment.values[0].args[0]), 'depthai_runtime_library_path')

    def test_real_cmake_records_imported_sdk_directory(self):
        cmake = shutil.which('cmake')
        if not cmake:
            self.skipTest('CMake unavailable')
        source = (PACKAGE.parents[1]/'cmake/DepthaiWorkspace.cmake').read_text()
        start = source.index('file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/depthai_runtime_dir.txt"')
        block = source[start:source.index('# Also cover ros2 run', start)]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            sdk = root/'external SDK/lib'
            sdk.mkdir(parents=True)
            (root/'CMakeLists.txt').write_text(
                'cmake_minimum_required(VERSION 3.18)\nproject(probe NONE)\n'
                'add_library(depthai::core SHARED IMPORTED)\n'
                f'set_target_properties(depthai::core PROPERTIES IMPORTED_LOCATION "{sdk}/libdepthai-core.so")\n'
                +block)
            result = subprocess.run([cmake, '-S', str(root), '-B', str(root/'build')],
                                    text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            self.assertEqual((root/'build/depthai_runtime_dir.txt').read_text(), str(sdk)+'\n')


if __name__ == '__main__':
    unittest.main()
