"""Validate SDK selection and runtime loading without ROS or an OAK device."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


REPOSITORY = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which("cmake"), "CMake unavailable")
class DepthaiWorkspaceTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="depthai workspace ")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.workspace = self.root / "f1tenth_project_repo"
        shutil.copytree(REPOSITORY / "cmake", self.workspace / "cmake")
        self.sdk = self.root / "system" / "usr" / "local"
        self.library_dir = self.sdk / "lib"
        self.library_dir.mkdir(parents=True)
        self.library = self.library_dir / (
            "libdepthai-core.dylib" if sys.platform == "darwin" else "libdepthai-core.so"
        )
        self.library.touch()
        self.config = self.sdk / "lib/cmake/depthai"
        self.config.mkdir(parents=True)
        self.write_sdk_config()
        (self.config / "depthaiConfigVersion.cmake").write_text(
            'set(PACKAGE_VERSION "3.6.1")\n'
            'if(PACKAGE_FIND_VERSION VERSION_GREATER PACKAGE_VERSION)\n'
            '  set(PACKAGE_VERSION_COMPATIBLE FALSE)\n'
            'else()\n'
            '  set(PACKAGE_VERSION_COMPATIBLE TRUE)\n'
            'endif()\n'
        )
        self.source = self.workspace / "src/probe"
        self.source.mkdir(parents=True)
        self.build = self.root / "probe build"
        self.write_project()

    def write_sdk_config(self, kind="SHARED"):
        (self.config / "depthaiConfig.cmake").write_text(
            f'add_library(depthai::core {kind} IMPORTED)\n'
            'set_target_properties(depthai::core PROPERTIES\n'
            '  IMPORTED_CONFIGURATIONS RELEASE\n'
            f'  IMPORTED_LOCATION_RELEASE "{self.library}")\n'
        )

    def write_project(self, *, executable=False):
        (self.source / "CMakeLists.txt").write_text(
            'cmake_minimum_required(VERSION 3.16)\n'
            f'project(probe {"CXX" if executable else "NONE"})\n'
            'function(ament_environment_hooks template)\n'
            '  configure_file("${template}" "${CMAKE_CURRENT_BINARY_DIR}/sdk.sh" @ONLY)\n'
            'endfunction()\n'
            'include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/DepthaiWorkspace.cmake")\n'
            'file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/selection.txt"\n'
            '  "${depthai_DIR}\\n${CMAKE_INSTALL_RPATH}\\n")\n'
            + ('add_executable(probe main.cpp)\n'
               'target_link_libraries(probe PRIVATE depthai::core)\n'
               'install(TARGETS probe DESTINATION bin)\n' if executable else '')
        )

    def configure(self, *arguments, success=True):
        result = subprocess.run(
            ["cmake", "-S", str(self.source), "-B", str(self.build),
             "-DCMAKE_BUILD_TYPE=Release", f"-DF1TENTH_DEPTHAI_PREFIX={self.sdk}", *arguments],
            text=True, capture_output=True,
        )
        if success:
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        else:
            self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        return result

    def test_ignores_stale_cache_and_other_prefixes(self):
        stale = self.root / "previous SDK"
        stale.mkdir()
        (stale / "depthaiConfig.cmake").write_text('message(FATAL_ERROR "WRONG SDK")\n')
        self.configure(f"-Ddepthai_DIR={stale}", f"-DCMAKE_PREFIX_PATH={stale}")
        self.assertEqual(
            (self.build / "selection.txt").read_text().splitlines(),
            [str(self.config), str(self.library_dir)],
        )
        self.assertEqual(
            (self.build / "depthai_runtime_dir.txt").read_text(),
            str(self.library_dir) + "\n",
        )
        self.assertIn(
            f"depthai_DIR:PATH={self.config}",
            (self.build / "CMakeCache.txt").read_text(),
        )

    def test_missing_selected_sdk_does_not_fall_back_to_another_install(self):
        other_sdk = self.root / "other SDK"
        self.sdk.rename(other_sdk)
        result = self.configure(
            f"-Ddepthai_DIR={other_sdk / 'lib/cmake/depthai'}",
            f"-DCMAKE_PREFIX_PATH={other_sdk}", success=False,
        )
        self.assertIn("existing DepthAI C++ SDK config was not found", result.stderr)
        self.assertNotIn("scripts/install_depthai.sh", result.stderr)

    def test_missing_library_reports_the_selected_sdk_without_installing(self):
        self.library.unlink()
        result = self.configure(success=False)
        self.assertIn("no SDK build or installation is performed", " ".join(result.stderr.split()))

    def test_static_sdk_is_rejected(self):
        self.write_sdk_config(kind="STATIC")
        result = self.configure(success=False)
        self.assertIn("not a shared DepthAI SDK", result.stderr)

    def test_sdk_export_cannot_point_to_an_external_library(self):
        self.library = self.root / "external-depthai.so"
        self.library.touch()
        self.write_sdk_config()
        result = self.configure(success=False)
        self.assertIn("outside the selected SDK prefix", result.stderr)

    def test_copied_source_is_ignored_and_existing_sdk_is_unchanged(self):
        copied = self.workspace.parent / "depthai-core" / "build"
        copied.mkdir(parents=True)
        (copied / "depthaiConfig.cmake").write_text('message(FATAL_ERROR "WRONG SDK")\n')
        (copied.parent / "CMakeLists.txt").write_text('message(FATAL_ERROR "DO NOT BUILD SDK")\n')
        paths = list(self.sdk.rglob("*")) + list(copied.parent.rglob("*"))
        before = {str(p): (p.read_bytes(), p.stat().st_mtime_ns) for p in paths if p.is_file()}
        self.configure(f"-Ddepthai_DIR={copied}", f"-DCMAKE_PREFIX_PATH={copied}")
        after = {str(p): (p.read_bytes(), p.stat().st_mtime_ns) for p in paths if p.is_file()}
        self.assertEqual(before, after)
        self.assertEqual(
            (self.build / "selection.txt").read_text().splitlines()[0], str(self.config)
        )

    def test_moving_dated_workspace_keeps_the_same_existing_sdk(self):
        self.configure()
        first = (self.build / "selection.txt").read_text()
        moved = self.workspace.with_name("different-date-workspace")
        self.workspace.rename(moved)
        self.workspace = moved
        self.source = moved / "src/probe"
        self.build = self.root / "new ROS build"
        self.configure()
        self.assertEqual((self.build / "selection.txt").read_text(), first)

    def test_hook_moves_sdk_ahead_of_conflicting_paths_on_every_source(self):
        self.configure()
        variable = "DYLD_LIBRARY_PATH" if sys.platform == "darwin" else "LD_LIBRARY_PATH"
        previous = str(self.root / "other SDK/lib")
        environment = dict(os.environ)
        environment[variable] = f"{previous}:{self.library_dir}:{self.library_dir}"
        environment["CMAKE_PREFIX_PATH"] = f"{previous}:{self.sdk}:{self.sdk}"
        script = (
            # macOS strips DYLD_* before starting /bin/bash (SIP), so set
            # the inherited value inside the shell before exercising the hook.
            f'export {variable}="$2"\n'
            '. "$1"\n. "$1"\n'
            f'printf "%s\\n" "${{{variable}}}" "$CMAKE_PREFIX_PATH"\n'
        )
        result = subprocess.run(
            ["bash", "-c", script, "hook-test", str(self.build / "sdk.sh"),
             environment[variable]],
            env=environment, text=True, capture_output=True, check=True,
        )
        self.assertEqual(result.stdout.splitlines(), [
            f"{self.library_dir}:{previous}", f"{self.sdk}:{previous}",
        ])

    @unittest.skipUnless(shutil.which("c++"), "C++ compiler unavailable")
    def test_real_executable_uses_workspace_sdk_with_conflicting_runtime_library(self):
        # Two actual shared libraries have the same SONAME/install name but
        # incompatible behavior, reproducing a wrong runtime SDK selection.
        self.library.unlink()
        fixture = self.root / "fixture"
        fixture.mkdir()
        (fixture / "sdk.cpp").write_text('extern "C" int sdk_generation() { return SDK_VERSION; }\n')
        conflict = self.root / "old SDK/lib"
        (fixture / "CMakeLists.txt").write_text(
            'cmake_minimum_required(VERSION 3.16)\nproject(fixture CXX)\n'
            'foreach(name current old)\n'
            '  add_library(${name} SHARED sdk.cpp)\n'
            '  set_target_properties(${name} PROPERTIES OUTPUT_NAME depthai-core\n'
            '    INSTALL_NAME_DIR "@rpath" BUILD_WITH_INSTALL_NAME_DIR ON)\n'
            'endforeach()\n'
            f'set_target_properties(current PROPERTIES LIBRARY_OUTPUT_DIRECTORY "{self.library_dir}")\n'
            f'set_target_properties(old PROPERTIES LIBRARY_OUTPUT_DIRECTORY "{conflict}")\n'
            'target_compile_definitions(current PRIVATE SDK_VERSION=3)\n'
            'target_compile_definitions(old PRIVATE SDK_VERSION=2)\n'
        )
        for command in (
            ["cmake", "-S", str(fixture), "-B", str(fixture / "build")],
            ["cmake", "--build", str(fixture / "build")],
        ):
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.write_project(executable=True)
        (self.source / "main.cpp").write_text(
            'extern "C" int sdk_generation();\n'
            'int main() { return sdk_generation() == 3 ? 0 : 9; }\n'
        )
        install = self.root / "probe install"
        self.configure(f"-DCMAKE_INSTALL_PREFIX={install}")
        for command in (
            ["cmake", "--build", str(self.build)],
            ["cmake", "--install", str(self.build)],
        ):
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        executable = install / "bin/probe"
        self.assertEqual(subprocess.run([str(executable)]).returncode, 0)
        variable = "DYLD_LIBRARY_PATH" if sys.platform == "darwin" else "LD_LIBRARY_PATH"
        environment = dict(os.environ, **{variable: str(conflict)})
        self.assertEqual(subprocess.run([str(executable)], env=environment).returncode, 9)
        environment[variable] += os.pathsep + str(self.library_dir)
        result = subprocess.run(
            ["bash", "-c", f'export {variable}="$3"; . "$1"; exec "$2"',
             "runtime-test", str(self.build / "sdk.sh"), str(executable),
             environment[variable]],
            env=environment, capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
