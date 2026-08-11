import re
from unittest.mock import patch

from jetson_resource_monitor.monitor_core import (
    extract_remap,
    merge_jetson_metrics,
    parse_installed_executable,
    SystemMonitor,
    TegrastatsReader,
)


def test_parse_installed_python_executable():
    package, executable = parse_installed_executable(
        [
            "/usr/bin/python3",
            "/work/install/example_pkg/lib/example_pkg/example_node",
        ],
        "/usr/bin/python3",
    )
    assert package == "example_pkg"
    assert executable == "example_node"


def test_parse_opt_ros_executable():
    package, executable = parse_installed_executable(
        ["/opt/ros/humble/lib/joy/joy_node"], "/opt/ros/humble/lib/joy/joy_node"
    )
    assert package == "joy"
    assert executable == "joy_node"


def test_extract_ros_remaps():
    args = ["node", "--ros-args", "-r", "__node:=camera", "-r", "__ns:=/front"]
    assert extract_remap(args, "__node") == "camera"
    assert extract_remap(args, "__ns") == "/front"


def test_merge_tegrastats_overrides_gpu():
    result = merge_jetson_metrics(
        {"gpu_percent": 10.0, "cpu_percent": 20.0},
        {"tegrastats_available": True, "gpu_percent": 42.0},
    )
    assert result["cpu_percent"] == 20.0
    assert result["gpu_percent"] == 42.0


def test_parse_legacy_tegrastats_power():
    reader = TegrastatsReader(1000)
    reader._latest_line = (
        "RAM 3200/7620MB GR3D_FREQ 71% cpu@45.5C POM_5V_IN 12890/12000"
    )
    result = reader.sample()
    reader.close()
    assert result["gpu_percent"] == 71.0
    assert result["power_mw"] == 12890
    assert result["power_average_mw"] == 12000


def test_filter_example():
    pattern = re.compile("camera|lane")
    assert pattern.search("/camera_driver_node camera_driver")
    assert pattern.search("/lane_mask lane_detect")
    assert not pattern.search("/joy_node joy")


def test_temperature_read_error_skips_only_the_failed_zone():
    zones = [
        "/sys/class/thermal/thermal_zone0",
        "/sys/class/thermal/thermal_zone1",
    ]

    def fake_read_text(path, encoding="utf-8"):
        del encoding
        text_path = str(path)
        if text_path.endswith("thermal_zone0/type"):
            return "cpu-thermal\n"
        if text_path.endswith("thermal_zone0/temp"):
            raise TypeError("can't concat NoneType to bytes")
        if text_path.endswith("thermal_zone1/type"):
            return "gpu-thermal\n"
        if text_path.endswith("thermal_zone1/temp"):
            return "45000\n"
        raise AssertionError(text_path)

    with patch(
        "jetson_resource_monitor.monitor_core.glob.glob",
        return_value=zones,
    ), patch(
        "jetson_resource_monitor.monitor_core.Path.read_text",
        autospec=True,
        side_effect=fake_read_text,
    ):
        result = SystemMonitor._temperatures()

    assert result == {"gpu-thermal": 45.0}
