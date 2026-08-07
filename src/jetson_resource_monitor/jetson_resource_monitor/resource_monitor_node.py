"""ROS 2 node that publishes Jetson and per-process resource metrics as JSON."""

from __future__ import annotations

import csv
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .monitor_core import (
    METRIC_READ_ERRORS,
    merge_jetson_metrics,
    ProcessMonitor,
    SystemMonitor,
    TegrastatsReader,
)


class ResourceMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("jetson_resource_monitor")
        self.declare_parameter("interval_sec", 1.0)
        self.declare_parameter("console_output", True)
        self.declare_parameter("console_every_n", 1)
        self.declare_parameter("csv_path", "")
        self.declare_parameter("process_filter", "")
        self.declare_parameter("include_non_ros_processes", False)

        interval_sec = max(
            0.1, float(self.get_parameter("interval_sec").value)
        )
        self._console_output = bool(self.get_parameter("console_output").value)
        self._console_every_n = max(
            1, int(self.get_parameter("console_every_n").value)
        )
        self._include_non_ros = bool(
            self.get_parameter("include_non_ros_processes").value
        )
        filter_text = str(self.get_parameter("process_filter").value)
        try:
            self._name_pattern = re.compile(filter_text) if filter_text else None
        except re.error as error:
            self.get_logger().warning(
                "process_filter 정규식이 올바르지 않아 무시합니다: "
                f"{error}"
            )
            self._name_pattern = None

        self._system_monitor = SystemMonitor()
        self._process_monitor = ProcessMonitor()
        self._tegrastats = TegrastatsReader(int(interval_sec * 1000))
        self._sample_count = 0
        self._csv_file = None
        self._csv_writer: Optional[csv.DictWriter] = None
        self._open_csv(str(self.get_parameter("csv_path").value))

        self._system_publisher = self.create_publisher(
            String, "/jetson_monitor/system", 10
        )
        self._process_publisher = self.create_publisher(
            String, "/jetson_monitor/processes", 10
        )
        self._metrics_publisher = self.create_publisher(
            String, "/jetson_monitor/metrics", 10
        )
        self._timer = self.create_timer(interval_sec, self._collect)

        source = "tegrastats + /proc" if self._tegrastats.available else "/proc + sysfs"
        self.get_logger().info(
            f"Jetson resource monitor 시작: {interval_sec:.2f}초 주기, "
            f"측정 소스={source}"
        )

    def _open_csv(self, path_text: str) -> None:
        if not path_text:
            return
        path = Path(path_text).expanduser()
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            self._csv_file = path.open("a", newline="", encoding="utf-8")
            fieldnames = [
                "timestamp",
                "pid",
                "package",
                "executable",
                "node_name",
                "process_cpu_percent",
                "process_cpu_cores",
                "process_memory_mb",
                "threads",
                "system_cpu_percent",
                "system_ram_percent",
                "system_gpu_percent",
                "power_mw",
            ]
            self._csv_writer = csv.DictWriter(
                self._csv_file, fieldnames=fieldnames
            )
            if self._csv_file.tell() == 0:
                self._csv_writer.writeheader()
                self._csv_file.flush()
            self.get_logger().info(f"CSV 기록 경로: {path}")
        except OSError as error:
            self.get_logger().warning(f"CSV 파일을 열 수 없습니다: {error}")
            self._csv_file = None
            self._csv_writer = None

    def _collect(self) -> None:
        timestamp = datetime.now(timezone.utc).isoformat()
        try:
            system = merge_jetson_metrics(
                self._system_monitor.sample(), self._tegrastats.sample()
            )
            processes = self._process_monitor.sample(
                exclude_pids=(os.getpid(),),
                include_non_ros=self._include_non_ros,
                name_pattern=self._name_pattern,
            )
        except METRIC_READ_ERRORS as error:
            self.get_logger().warning(
                "리소스 파일을 일시적으로 읽지 못해 이번 측정을 "
                f"건너뜁니다: {error}"
            )
            return

        graph_names = {
            f"{namespace.rstrip('/')}/{name}"
            if namespace != "/"
            else f"/{name}"
            for name, namespace in self.get_node_names_and_namespaces()
        }
        for process in processes:
            process["graph_name_match"] = process["node_name"] in graph_names

        system_message = {"timestamp": timestamp, **system}
        process_message = {"timestamp": timestamp, "processes": processes}
        combined_message = {
            "timestamp": timestamp,
            "system": system,
            "processes": processes,
            "ros_graph_nodes": sorted(graph_names),
        }
        self._publish_json(self._system_publisher, system_message)
        self._publish_json(self._process_publisher, process_message)
        self._publish_json(self._metrics_publisher, combined_message)

        self._write_csv(timestamp, system, processes)
        self._sample_count += 1
        if self._console_output and self._sample_count % self._console_every_n == 0:
            self.get_logger().info(self._format_console(system, processes))

    @staticmethod
    def _publish_json(publisher, payload: dict) -> None:
        message = String()
        message.data = json.dumps(
            payload, ensure_ascii=False, separators=(",", ":"), allow_nan=False
        )
        publisher.publish(message)

    def _write_csv(self, timestamp: str, system: dict, processes: list) -> None:
        if self._csv_writer is None or self._csv_file is None:
            return
        rows = processes if processes else [None]
        for process in rows:
            self._csv_writer.writerow(
                {
                    "timestamp": timestamp,
                    "pid": process["pid"] if process else "",
                    "package": process["package"] if process else "",
                    "executable": process["executable"] if process else "",
                    "node_name": process["node_name"] if process else "",
                    "process_cpu_percent": (
                        process["cpu_percent"] if process else ""
                    ),
                    "process_cpu_cores": (
                        process["cpu_cores"] if process else ""
                    ),
                    "process_memory_mb": (
                        process["memory_mb"] if process else ""
                    ),
                    "threads": process["threads"] if process else "",
                    "system_cpu_percent": system["cpu_percent"],
                    "system_ram_percent": system["ram_percent"],
                    "system_gpu_percent": system.get("gpu_percent"),
                    "power_mw": system.get("power_mw"),
                }
            )
        self._csv_file.flush()

    @staticmethod
    def _format_console(system: dict, processes: list) -> str:
        gpu = system.get("gpu_percent")
        gpu_text = f"{gpu:.1f}%" if gpu is not None else "N/A"
        power = system.get("power_mw")
        power_text = f"{power}mW" if power is not None else "N/A"
        header = (
            "\n[Jetson] "
            f"CPU {system['cpu_percent']:.1f}% | "
            f"RAM {system['ram_used_mb']:.0f}/{system['ram_total_mb']:.0f}MB "
            f"({system['ram_percent']:.1f}%) | GPU {gpu_text} | Power {power_text}"
            "\n"
            f"{'PID':>7}  {'CPU%':>7}  {'RAM(MB)':>8}  {'THR':>4}  "
            f"{'PACKAGE':<22}  NODE/PROCESS"
        )
        rows = [
            f"{item['pid']:>7}  {item['cpu_percent']:>7.1f}  "
            f"{item['memory_mb']:>8.1f}  {item['threads']:>4}  "
            f"{item['package'][:22]:<22}  {item['node_name']}"
            for item in processes
        ]
        if not rows:
            rows = ["(감지된 ROS 2 프로세스가 없습니다.)"]
        return header + "\n" + "\n".join(rows)

    def destroy_node(self) -> bool:
        self._tegrastats.close()
        if self._csv_file is not None:
            self._csv_file.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ResourceMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
