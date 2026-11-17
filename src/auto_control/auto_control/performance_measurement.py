from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import json
import math
from pathlib import Path
import re
import time
from typing import Any


def _safe_name(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip().lower())
    return cleaned or "unknown"


def _percentile(sorted_values: list[float], percentile: float) -> float:
    if not sorted_values:
        return 0.0
    position = (len(sorted_values) - 1) * percentile / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return sorted_values[lower]
    fraction = position - lower
    return (
        sorted_values[lower] * (1.0 - fraction)
        + sorted_values[upper] * fraction
    )


def _metric_summary(values: list[float]) -> dict[str, float | int]:
    if not values:
        return {
            "count": 0,
            "average_ms": 0.0,
            "minimum_ms": 0.0,
            "p50_ms": 0.0,
            "p95_ms": 0.0,
            "maximum_ms": 0.0,
            "equivalent_fps_from_average": 0.0,
        }
    ordered = sorted(values)
    average = sum(values) / len(values)
    return {
        "count": len(values),
        "average_ms": average,
        "minimum_ms": ordered[0],
        "p50_ms": _percentile(ordered, 50.0),
        "p95_ms": _percentile(ordered, 95.0),
        "maximum_ms": ordered[-1],
        "equivalent_fps_from_average": 1000.0 / average if average > 0.0 else 0.0,
    }


@dataclass(frozen=True)
class PowerSource:
    rail_name: str
    source: str
    voltage_path: Path | None = None
    current_path: Path | None = None
    power_path: Path | None = None
    power_divisor: float = 1.0

    def read_watts(self) -> float:
        if self.power_path is not None:
            return float(self.power_path.read_text(encoding="utf-8").strip()) / self.power_divisor
        if self.voltage_path is None or self.current_path is None:
            raise RuntimeError("power source has no readable measurement files")
        millivolts = float(self.voltage_path.read_text(encoding="utf-8").strip())
        milliamps = float(self.current_path.read_text(encoding="utf-8").strip())
        return millivolts * milliamps / 1_000_000.0


class JetsonPowerMonitor:
    """Read the Jetson module input rail without changing any sysfs values."""

    _RAIL_PRIORITY = {
        "VDD_IN": 0,
        "5V_IN": 1,
        "POM_5V_IN": 2,
        "VIN_SYS_5V0": 3,
    }

    def __init__(self) -> None:
        self.source = self._discover()
        self.samples: list[dict[str, float]] = []
        self.read_errors = 0

    @classmethod
    def _discover(cls) -> PowerSource | None:
        candidates: list[tuple[int, PowerSource]] = []
        roots = tuple(Path("/sys/bus/i2c/drivers").glob("ina3221*"))
        for root in roots:
            for label_path in root.glob("*/hwmon/hwmon*/in*_label"):
                match = re.fullmatch(r"in(\d+)_label", label_path.name)
                if match is None:
                    continue
                try:
                    rail_name = label_path.read_text(encoding="utf-8").strip()
                except OSError:
                    continue
                priority = cls._RAIL_PRIORITY.get(rail_name.upper())
                if priority is None:
                    continue
                channel = match.group(1)
                voltage_path = label_path.with_name(f"in{channel}_input")
                current_path = label_path.with_name(f"curr{channel}_input")
                if voltage_path.is_file() and current_path.is_file():
                    candidates.append(
                        (
                            priority,
                            PowerSource(
                                rail_name=rail_name,
                                source=f"{voltage_path} * {current_path}",
                                voltage_path=voltage_path,
                                current_path=current_path,
                            ),
                        )
                    )
        for root in roots:
            for label_path in root.glob("*/iio:device*/rail_name_*"):
                match = re.fullmatch(r"rail_name_(\d+)", label_path.name)
                if match is None:
                    continue
                try:
                    rail_name = label_path.read_text(encoding="utf-8").strip()
                except OSError:
                    continue
                priority = cls._RAIL_PRIORITY.get(rail_name.upper())
                if priority is None:
                    continue
                power_path = label_path.with_name(
                    f"in_power{match.group(1)}_input"
                )
                if power_path.is_file():
                    candidates.append(
                        (
                            priority,
                            PowerSource(
                                rail_name=rail_name,
                                source=str(power_path),
                                power_path=power_path,
                                power_divisor=1000.0,
                            ),
                        )
                    )
        return min(candidates, key=lambda candidate: candidate[0])[1] if candidates else None

    def sample(self, elapsed_sec: float) -> float | None:
        if self.source is None:
            return None
        try:
            watts = self.source.read_watts()
        except (OSError, ValueError, RuntimeError):
            self.read_errors += 1
            return None
        if not math.isfinite(watts) or watts < 0.0:
            self.read_errors += 1
            return None
        self.samples.append({"elapsed_sec": elapsed_sec, "watts": watts})
        return watts

    def report(self, duration_sec: float) -> dict[str, Any]:
        watts = [sample["watts"] for sample in self.samples]
        return {
            "available": self.source is not None,
            "rail_name": self.source.rail_name if self.source else None,
            "source": self.source.source if self.source else None,
            "sample_count": len(watts),
            "read_error_count": self.read_errors,
            "average_watts": sum(watts) / len(watts) if watts else None,
            "minimum_watts": min(watts) if watts else None,
            "maximum_watts": max(watts) if watts else None,
            "estimated_energy_wh": (
                (sum(watts) / len(watts)) * duration_sec / 3600.0
                if watts else None
            ),
            "samples": self.samples,
        }


class PerformanceMeasurement:
    def __init__(
        self,
        *,
        duration_sec: float,
        startup_timeout_sec: float,
        log_directory: str,
        engine_precision: str,
        model_path: str,
    ) -> None:
        self.duration_sec = duration_sec
        self.startup_timeout_sec = startup_timeout_sec
        self.log_directory = Path(log_directory).expanduser().resolve()
        self.engine_precision = _safe_name(engine_precision)
        self.model_path = model_path
        self.created_datetime = datetime.now().astimezone()
        self.created_monotonic_ns = time.perf_counter_ns()
        self.started_datetime: datetime | None = None
        self.started_monotonic_ns: int | None = None
        self.frames: list[dict[str, Any]] = []
        self.power = JetsonPowerMonitor()
        self.output_path: Path | None = None

    @property
    def active(self) -> bool:
        return self.started_monotonic_ns is not None

    def start(self, now_monotonic_ns: int) -> None:
        if self.active:
            return
        self.started_monotonic_ns = now_monotonic_ns
        self.started_datetime = datetime.now().astimezone()

    def elapsed_sec(self, now_monotonic_ns: int | None = None) -> float:
        if self.started_monotonic_ns is None:
            return 0.0
        end = time.perf_counter_ns() if now_monotonic_ns is None else now_monotonic_ns
        return max(0.0, (end - self.started_monotonic_ns) / 1_000_000_000.0)

    def startup_elapsed_sec(self) -> float:
        return max(
            0.0,
            (time.perf_counter_ns() - self.created_monotonic_ns) / 1_000_000_000.0,
        )

    def add_frame(self, frame: dict[str, Any]) -> None:
        self.frames.append(frame)

    def sample_power(self) -> float | None:
        if not self.active:
            return None
        return self.power.sample(self.elapsed_sec())

    def due_to_finish(self) -> bool:
        return self.active and self.elapsed_sec() >= self.duration_sec

    def startup_timed_out(self) -> bool:
        return not self.active and self.startup_elapsed_sec() >= self.startup_timeout_sec

    def write(self, status: str) -> Path:
        completed_datetime = datetime.now().astimezone()
        actual_duration_sec = self.elapsed_sec()
        metric_names = (
            "h2d_preprocess_ms",
            "pure_inference_ms",
            "label_export_ms",
            "backend_postprocess_ms",
            "lane_geometry_ms",
            "result_message_build_ms",
            "lane_postprocess_total_ms",
            "detector_queue_ms",
            "detector_total_compute_ms",
            "lane_result_transport_ms",
            "source_to_detector_input_ms",
            "auto_control_compute_ms",
            "compute_only_total_ms",
            "detector_input_to_control_complete_ms",
            "source_capture_to_control_complete_ms",
        )
        metrics = {
            name: _metric_summary(
                [
                    float(frame[name])
                    for frame in self.frames
                    if frame.get(name) is not None
                    and math.isfinite(float(frame[name]))
                    and float(frame[name]) >= 0.0
                ]
            )
            for name in metric_names
        }
        valid_count = sum(bool(frame.get("valid_centerline")) for frame in self.frames)
        control_count = sum(bool(frame.get("servo_position_calculated")) for frame in self.frames)
        observed_precisions = sorted({
            str(frame.get("actual_engine_precision", "unknown"))
            for frame in self.frames
        })
        payload = {
            "schema_version": 1,
            "status": status,
            "title": f"auto_drive_{self.engine_precision}_{self.created_datetime.isoformat()}",
            "engine_precision": self.engine_precision,
            "observed_engine_precisions": observed_precisions,
            "model_path": self.model_path,
            "safety_mode": "monitor_only (no duty/brake/servo actuator output)",
            "created_at": self.created_datetime.isoformat(),
            "measurement_started_at": (
                self.started_datetime.isoformat() if self.started_datetime else None
            ),
            "completed_at": completed_datetime.isoformat(),
            "target_duration_sec": self.duration_sec,
            "actual_duration_sec": actual_duration_sec,
            "startup_timeout_sec": self.startup_timeout_sec,
            "frame_count": len(self.frames),
            "valid_centerline_count": valid_count,
            "servo_position_calculated_count": control_count,
            "valid_centerline_ratio": (
                valid_count / len(self.frames) if self.frames else 0.0
            ),
            "lane_result_throughput_fps_excluding_source_transport_delay": (
                len(self.frames) / actual_duration_sec
                if actual_duration_sec > 0.0 else 0.0
            ),
            "servo_position_calculation_throughput_fps": (
                control_count / actual_duration_sec
                if actual_duration_sec > 0.0 else 0.0
            ),
            "metrics": metrics,
            "power": self.power.report(actual_duration_sec),
            "frames": self.frames,
        }
        self.log_directory.mkdir(parents=True, exist_ok=True)
        timestamp = self.created_datetime.strftime("%Y%m%d_%H%M%S_%f%z")
        base = self.log_directory / (
            f"auto_drive_benchmark_{self.engine_precision}_{timestamp}.json"
        )
        candidate = base
        suffix = 1
        while True:
            try:
                with candidate.open("x", encoding="utf-8") as output:
                    json.dump(payload, output, ensure_ascii=False, indent=2)
                    output.write("\n")
                break
            except FileExistsError:
                candidate = base.with_name(f"{base.stem}_{suffix}{base.suffix}")
                suffix += 1
        self.output_path = candidate
        return candidate
