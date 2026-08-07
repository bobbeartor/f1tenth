"""Linux and Jetson metric collection without third-party Python packages."""

from __future__ import annotations

from dataclasses import dataclass
import glob
import os
from pathlib import Path
import re
import shutil
import subprocess
import threading
import time
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


_NODE_REMAP_PATTERN = re.compile(r"(?:^|:)__node:=(.+)$")
_NAMESPACE_REMAP_PATTERN = re.compile(r"(?:^|:)__ns:=(.+)$")


@dataclass(frozen=True)
class ProcStat:
    """Values from one /proc/<pid>/stat sample."""

    cpu_ticks: int
    num_threads: int
    start_ticks: int
    rss_pages: int


@dataclass(frozen=True)
class ProcessIdentity:
    """Best-effort identity for one ROS process."""

    pid: int
    package: str
    executable: str
    node_name: str
    namespace: str
    command: str

    @property
    def full_node_name(self) -> str:
        namespace = self.namespace.strip("/")
        node_name = self.node_name.strip("/")
        return f"/{namespace}/{node_name}" if namespace else f"/{node_name}"


def read_proc_stat(pid: int) -> ProcStat:
    """Read fields from proc stat while allowing spaces in the process name."""
    raw = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
    closing_paren = raw.rfind(")")
    if closing_paren < 0:
        raise ValueError(f"Malformed /proc/{pid}/stat")
    fields = raw[closing_paren + 2 :].split()
    return ProcStat(
        cpu_ticks=int(fields[11]) + int(fields[12]),
        num_threads=int(fields[17]),
        start_ticks=int(fields[19]),
        rss_pages=int(fields[21]),
    )


def read_cmdline(pid: int) -> List[str]:
    raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    return [
        value.decode("utf-8", errors="replace")
        for value in raw.rstrip(b"\0").split(b"\0")
        if value
    ]


def read_environ(pid: int) -> Dict[str, str]:
    try:
        raw = Path(f"/proc/{pid}/environ").read_bytes()
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return {}
    result = {}
    for item in raw.rstrip(b"\0").split(b"\0"):
        if b"=" not in item:
            continue
        key, value = item.split(b"=", 1)
        result[key.decode(errors="replace")] = value.decode(errors="replace")
    return result


def parse_installed_executable(
    cmdline: Sequence[str], exe_path: str
) -> Tuple[str, str]:
    """Return package and executable for install/lib/<pkg>/<executable> paths."""
    candidates = list(cmdline[:2]) + [exe_path]
    for candidate in candidates:
        parts = Path(candidate).parts
        for index in range(len(parts) - 2):
            if parts[index] == "lib" and index + 2 < len(parts):
                return parts[index + 1], parts[index + 2]
    executable = Path(cmdline[0] if cmdline else exe_path).name
    if executable.startswith("python") and len(cmdline) > 1:
        executable = Path(cmdline[1]).name
    return "", executable


def extract_remap(args: Sequence[str], key: str) -> str:
    pattern = _NODE_REMAP_PATTERN if key == "__node" else _NAMESPACE_REMAP_PATTERN
    for arg in args:
        match = pattern.search(arg)
        if match:
            return match.group(1)
    return ""


def identify_ros_process(
    pid: int, include_non_ros: bool = False
) -> Optional[ProcessIdentity]:
    """Identify a ROS process using its command, install path, and environment."""
    try:
        cmdline = read_cmdline(pid)
        exe_path = os.readlink(f"/proc/{pid}/exe")
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return None
    if not cmdline:
        return None

    package, executable = parse_installed_executable(cmdline, exe_path)
    environment = read_environ(pid)
    has_ros_args = "--ros-args" in cmdline or any(
        "__node:=" in arg or "__ns:=" in arg for arg in cmdline
    )
    installed_ros_executable = bool(package) and (
        "/install/" in exe_path
        or "/install/" in " ".join(cmdline[:2])
        or "/opt/ros/" in exe_path
        or "/opt/ros/" in " ".join(cmdline[:2])
    )
    source_node = any(
        Path(arg).name.endswith(("_node.py", "_node")) for arg in cmdline[:2]
    )
    ros_environment = environment.get("ROS_VERSION") == "2"
    is_ros = (
        has_ros_args
        or installed_ros_executable
        or (ros_environment and source_node)
    )
    if not (is_ros or include_non_ros):
        return None

    node_name = extract_remap(cmdline, "__node")
    namespace = extract_remap(cmdline, "__ns") or "/"
    if not node_name:
        node_name = Path(executable).stem
        if node_name.endswith(".py"):
            node_name = node_name[:-3]

    return ProcessIdentity(
        pid=pid,
        package=package or "-",
        executable=executable or Path(exe_path).name,
        node_name=node_name,
        namespace=namespace,
        command=" ".join(cmdline),
    )


class ProcessMonitor:
    """Track CPU deltas and memory for matching Linux processes."""

    def __init__(self) -> None:
        self._clock_ticks = os.sysconf("SC_CLK_TCK")
        self._page_size = os.sysconf("SC_PAGE_SIZE")
        self._previous: Dict[Tuple[int, int], Tuple[int, float]] = {}

    def sample(
        self,
        exclude_pids: Iterable[int] = (),
        include_non_ros: bool = False,
        name_pattern: Optional[re.Pattern] = None,
    ) -> List[dict]:
        now = time.monotonic()
        excluded = set(exclude_pids)
        samples = []
        current_keys = set()

        for proc_path in glob.glob("/proc/[0-9]*"):
            pid = int(Path(proc_path).name)
            if pid in excluded:
                continue
            identity = identify_ros_process(pid, include_non_ros=include_non_ros)
            if identity is None:
                continue
            searchable = " ".join(
                (
                    identity.full_node_name,
                    identity.package,
                    identity.executable,
                    identity.command,
                )
            )
            if name_pattern and not name_pattern.search(searchable):
                continue
            try:
                stat = read_proc_stat(pid)
            except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError):
                continue

            key = (pid, stat.start_ticks)
            current_keys.add(key)
            previous = self._previous.get(key)
            cpu_percent = 0.0
            if previous:
                previous_ticks, previous_time = previous
                elapsed = now - previous_time
                if elapsed > 0.0:
                    cpu_seconds = (
                        stat.cpu_ticks - previous_ticks
                    ) / self._clock_ticks
                    cpu_percent = max(0.0, 100.0 * cpu_seconds / elapsed)
            self._previous[key] = (stat.cpu_ticks, now)

            samples.append(
                {
                    "pid": pid,
                    "package": identity.package,
                    "executable": identity.executable,
                    "node_name": identity.full_node_name,
                    "cpu_percent": round(cpu_percent, 2),
                    "cpu_cores": round(cpu_percent / 100.0, 3),
                    "memory_mb": round(
                        stat.rss_pages * self._page_size / (1024.0 * 1024.0), 2
                    ),
                    "threads": stat.num_threads,
                    "command": identity.command,
                }
            )

        self._previous = {
            key: value for key, value in self._previous.items() if key in current_keys
        }
        return sorted(
            samples, key=lambda item: (-item["cpu_percent"], -item["memory_mb"])
        )


class SystemMonitor:
    """Collect portable Linux system metrics and Jetson sysfs values."""

    def __init__(self) -> None:
        self._previous_cpu: Optional[Tuple[int, int]] = None

    @staticmethod
    def _cpu_counters() -> Tuple[int, int]:
        fields = Path("/proc/stat").read_text(encoding="utf-8").splitlines()[0].split()
        values = [int(value) for value in fields[1:]]
        total = sum(values)
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        return total, idle

    def _cpu_percent(self) -> float:
        current = self._cpu_counters()
        result = 0.0
        if self._previous_cpu:
            total_delta = current[0] - self._previous_cpu[0]
            idle_delta = current[1] - self._previous_cpu[1]
            if total_delta > 0:
                result = 100.0 * (total_delta - idle_delta) / total_delta
        self._previous_cpu = current
        return round(max(0.0, result), 2)

    @staticmethod
    def _memory() -> dict:
        values = {}
        for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
            key, value = line.split(":", 1)
            values[key] = int(value.strip().split()[0])
        total_kb = values["MemTotal"]
        available_kb = values.get("MemAvailable", values.get("MemFree", 0))
        used_kb = total_kb - available_kb
        return {
            "ram_used_mb": round(used_kb / 1024.0, 2),
            "ram_total_mb": round(total_kb / 1024.0, 2),
            "ram_percent": round(100.0 * used_kb / total_kb, 2),
            "swap_used_mb": round(
                (values.get("SwapTotal", 0) - values.get("SwapFree", 0)) / 1024.0,
                2,
            ),
        }

    @staticmethod
    def _gpu_percent() -> Optional[float]:
        candidates = [
            "/sys/devices/gpu.0/load",
            "/sys/devices/platform/17000000.gpu/load",
            "/sys/class/devfreq/17000000.ga10b/load",
        ]
        candidates.extend(glob.glob("/sys/class/devfreq/*gpu*/load"))
        for path in candidates:
            try:
                value = float(Path(path).read_text(encoding="utf-8").strip())
            except (FileNotFoundError, PermissionError, ValueError):
                continue
            return round(value / 10.0 if value > 100.0 else value, 2)
        return None

    @staticmethod
    def _temperatures() -> Dict[str, float]:
        result = {}
        for zone_path in glob.glob("/sys/class/thermal/thermal_zone*"):
            try:
                name = Path(zone_path, "type").read_text(encoding="utf-8").strip()
                value = float(
                    Path(zone_path, "temp").read_text(encoding="utf-8").strip()
                )
            except (FileNotFoundError, PermissionError, ValueError):
                continue
            if abs(value) > 1000.0:
                value /= 1000.0
            result[name] = round(value, 1)
        return result

    @staticmethod
    def _cpu_frequency_mhz() -> Optional[float]:
        frequencies = []
        for path in glob.glob(
            "/sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_cur_freq"
        ):
            try:
                frequencies.append(
                    float(Path(path).read_text(encoding="utf-8").strip()) / 1000.0
                )
            except (FileNotFoundError, PermissionError, ValueError):
                continue
        return round(sum(frequencies) / len(frequencies), 1) if frequencies else None

    def sample(self) -> dict:
        load_1m, load_5m, load_15m = os.getloadavg()
        result = {
            "cpu_percent": self._cpu_percent(),
            "cpu_count": os.cpu_count() or 1,
            "cpu_frequency_mhz": self._cpu_frequency_mhz(),
            "load_1m": round(load_1m, 2),
            "load_5m": round(load_5m, 2),
            "load_15m": round(load_15m, 2),
            "gpu_percent": self._gpu_percent(),
            "temperatures_c": self._temperatures(),
        }
        result.update(self._memory())
        return result


class TegrastatsReader:
    """Keep the latest line from NVIDIA tegrastats, when it is available."""

    _GPU_PATTERN = re.compile(r"GR3D_FREQ\s+(\d+(?:\.\d+)?)%")
    _RAM_PATTERN = re.compile(r"RAM\s+(\d+)/(\d+)MB")
    _TEMP_PATTERN = re.compile(r"([A-Za-z0-9_]+)@(\d+(?:\.\d+)?)C")
    _POWER_PATTERN = re.compile(
        r"(?:VDD_IN|POM_5V_IN)\s+(\d+)(?:mW)?"
        r"(?:/(\d+)(?:mW)?)?"
    )

    def __init__(self, interval_ms: int) -> None:
        self._latest_line = ""
        self._lock = threading.Lock()
        self._process: Optional[subprocess.Popen] = None
        executable = shutil.which("tegrastats")
        if not executable:
            return
        try:
            self._process = subprocess.Popen(
                [executable, "--interval", str(interval_ms)],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                bufsize=1,
            )
        except OSError:
            self._process = None
            return
        threading.Thread(target=self._read_loop, daemon=True).start()

    @property
    def available(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def _read_loop(self) -> None:
        if self._process is None or self._process.stdout is None:
            return
        for line in self._process.stdout:
            with self._lock:
                self._latest_line = line.strip()

    def sample(self) -> dict:
        with self._lock:
            line = self._latest_line
        if not line:
            return {"tegrastats_available": self.available}

        result = {"tegrastats_available": True, "tegrastats_raw": line}
        gpu_match = self._GPU_PATTERN.search(line)
        ram_match = self._RAM_PATTERN.search(line)
        power_match = self._POWER_PATTERN.search(line)
        if gpu_match:
            result["gpu_percent"] = float(gpu_match.group(1))
        if ram_match:
            result["tegrastats_ram_used_mb"] = int(ram_match.group(1))
            result["tegrastats_ram_total_mb"] = int(ram_match.group(2))
        if power_match:
            result["power_mw"] = int(power_match.group(1))
            if power_match.group(2):
                result["power_average_mw"] = int(power_match.group(2))
        temperatures = {
            name: float(value)
            for name, value in self._TEMP_PATTERN.findall(line)
        }
        if temperatures:
            result["tegrastats_temperatures_c"] = temperatures
        return result

    def close(self) -> None:
        if self._process is None or self._process.poll() is not None:
            return
        self._process.terminate()
        try:
            self._process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            self._process.kill()
            self._process.wait(timeout=1.0)


def merge_jetson_metrics(system: dict, tegrastats: dict) -> dict:
    """Prefer tegrastats for Jetson-specific values while retaining raw data."""
    result = dict(system)
    result.update(tegrastats)
    return result
