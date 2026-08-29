"""Bounded experiment logging with configuration manifests and optional rosbag2."""

from __future__ import annotations

from functools import partial
import hashlib
import json
from math import isfinite
from pathlib import Path
import shutil
import subprocess
import time

from crazyflie_interfaces.msg import Status
from diagnostic_msgs.msg import DiagnosticArray
from motion_capture_tracking_interfaces.msg import NamedPoseArray
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from .config import load_fleet, load_safety


class ExperimentLogger(Node):
    def __init__(self) -> None:
        super().__init__("crazyfly_experiment_logger")
        self.declare_parameter("fleet_config_file", "")
        self.declare_parameter("safety_config_file", "")
        self.declare_parameter("output_root", "experiments")
        self.declare_parameter("record_rosbag", False)
        self.declare_parameter("maximum_duration_s", 300.0)

        fleet_path = str(self.get_parameter("fleet_config_file").value)
        safety_path = str(self.get_parameter("safety_config_file").value)
        if not fleet_path or not safety_path:
            raise RuntimeError("fleet_config_file and safety_config_file are required")
        fleet = load_fleet(fleet_path)
        load_safety(safety_path)

        try:
            self.maximum_duration_s = float(
                self.get_parameter("maximum_duration_s").value
            )
        except (TypeError, ValueError) as exc:
            raise RuntimeError("maximum_duration_s must be finite") from exc
        if (
            not isfinite(self.maximum_duration_s)
            or not 0 < self.maximum_duration_s <= 3600.0
        ):
            raise RuntimeError("maximum_duration_s must be between 0 and 3600")
        record_rosbag = bool(self.get_parameter("record_rosbag").value)
        output_root = Path(str(self.get_parameter("output_root").value)).resolve()
        output_root.mkdir(parents=True, exist_ok=True)
        if record_rosbag and shutil.disk_usage(output_root).free < 1_000_000_000:
            raise RuntimeError(
                "less than 1 GB is free; refusing to start rosbag recording"
            )

        run_id = (
            time.strftime("%Y%m%dT%H%M%S", time.gmtime())
            + f"-{time.time_ns() % 1_000_000_000:09d}"
        )
        self.run_directory = output_root / run_id
        self.run_directory.mkdir(exist_ok=False)
        self.event_file = (self.run_directory / "events.jsonl").open(
            "w", encoding="utf-8"
        )
        self.started_at = time.monotonic()
        self.bag_process: subprocess.Popen[str] | None = None

        manifest = {
            "run_id": run_id,
            "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "robots": sorted(fleet.enabled),
            "configuration": {
                "fleet": self._file_record(fleet_path),
                "safety": self._file_record(safety_path),
            },
            "record_rosbag": record_rosbag,
            "maximum_duration_s": self.maximum_duration_s,
        }
        (self.run_directory / "manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

        self.create_subscription(
            NamedPoseArray, "/poses", self._poses_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            String, "/crazyfly/safety/state", self._state_callback, 10
        )
        self.create_subscription(
            String, "/crazyfly/commands", self._command_callback, 10
        )
        self.create_subscription(
            DiagnosticArray,
            "/crazyfly/safety/diagnostics",
            self._diagnostic_callback,
            10,
        )
        for name in fleet.enabled:
            self.create_subscription(
                Status,
                f"/{name}/status",
                partial(self._status_callback, name),
                10,
            )
        self.create_timer(1.0, self._duration_guard)

        if record_rosbag:
            topics = [
                "/poses",
                "/crazyfly/commands",
                "/crazyfly/safety/state",
                "/crazyfly/safety/diagnostics",
            ]
            topics.extend(f"/{name}/status" for name in fleet.enabled)
            self.bag_process = subprocess.Popen(
                [
                    "ros2",
                    "bag",
                    "record",
                    "-o",
                    str(self.run_directory / "bag"),
                    *topics,
                ],
                text=True,
            )
        self.get_logger().info(f"logging experiment to {self.run_directory}")

    @staticmethod
    def _file_record(path: str) -> dict[str, str]:
        resolved = Path(path).resolve()
        content = resolved.read_bytes()
        return {"path": str(resolved), "sha256": hashlib.sha256(content).hexdigest()}

    def _write(self, event: str, data: object) -> None:
        record = {
            "monotonic_s": time.monotonic(),
            "ros_time_ns": self.get_clock().now().nanoseconds,
            "event": event,
            "data": data,
        }
        self.event_file.write(json.dumps(record, separators=(",", ":")) + "\n")
        self.event_file.flush()

    def _poses_callback(self, message: NamedPoseArray) -> None:
        self._write(
            "poses",
            {
                pose.name: [
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                ]
                for pose in message.poses
            },
        )

    def _state_callback(self, message: String) -> None:
        self._write("safety_state", message.data)

    def _command_callback(self, message: String) -> None:
        try:
            data: object = json.loads(message.data)
        except json.JSONDecodeError:
            data = {"raw": message.data}
        self._write("command", data)

    def _diagnostic_callback(self, message: DiagnosticArray) -> None:
        self._write(
            "diagnostics",
            [
                {
                    "name": status.name,
                    "level": (
                        status.level[0]
                        if isinstance(status.level, (bytes, bytearray))
                        else int(status.level)
                    ),
                    "message": status.message,
                    "values": {item.key: item.value for item in status.values},
                }
                for status in message.status
            ],
        )

    def _status_callback(self, name: str, message: Status) -> None:
        self._write(
            "status",
            {
                "robot": name,
                "battery_voltage": message.battery_voltage,
                "supervisor_info": message.supervisor_info,
                "rssi": message.rssi,
                "latency_unicast_ms": message.latency_unicast,
            },
        )

    def _duration_guard(self) -> None:
        if time.monotonic() - self.started_at >= self.maximum_duration_s:
            self.get_logger().warning("maximum logging duration reached")
            rclpy.shutdown()

    def stop(self) -> None:
        if self.bag_process is not None and self.bag_process.poll() is None:
            self.bag_process.terminate()
            try:
                self.bag_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.bag_process.kill()
        if not self.event_file.closed:
            self.event_file.close()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: ExperimentLogger | None = None
    try:
        node = ExperimentLogger()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
