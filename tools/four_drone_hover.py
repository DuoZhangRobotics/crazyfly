#!/usr/bin/env python3
"""Run one guarded selected-fleet Crazyflie takeoff, hover, and landing."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from contextlib import contextmanager, suppress
from pathlib import Path

import yaml
from crazyradio_guard import describe_status, ensure_crazyradio_free

from crazyfly.config import ConfigError
from crazyfly.config_validator import validate

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_FLEET = ROOT / "config" / "local" / "crazyflies.yaml"
DEFAULT_SAFETY = ROOT / "config" / "local" / "safety.yaml"
DEFAULT_MOTION_CAPTURE = ROOT / "config" / "local" / "motion_capture.yaml"
DEFAULT_SERVER = ROOT / "config" / "server.yaml"
EXPECTED_ROBOTS = ("cf1", "cf2", "cf3", "cf4")


@contextmanager
def _ignore_cleanup_interrupts():
    previous_handlers = {}
    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.signal(signum, signal.SIG_IGN)
    try:
        yield
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Synchronized 0.30 m takeoff over 3 s, 5 s hover, and landing "
            "for the selected robots. Defaults to a hardware-free dry run."
        )
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--fleet", default=str(DEFAULT_FLEET))
    parser.add_argument("--safety", default=str(DEFAULT_SAFETY))
    parser.add_argument("--motion-capture", default=str(DEFAULT_MOTION_CAPTURE))
    parser.add_argument("--server", default=str(DEFAULT_SERVER))
    return parser


def _enabled_safety_copy(source: str, directory: str) -> Path:
    source_path = Path(source).expanduser().resolve()
    data = yaml.safe_load(source_path.read_text(encoding="utf-8"))
    data["crazyfly_safety"]["flight_enabled"] = True
    destination = Path(directory) / "safety.flight-enabled.yaml"
    destination.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return destination


def _stop_launch(process: subprocess.Popen | None) -> None:
    if process is None:
        return
    with _ignore_cleanup_interrupts():
        if process.poll() is None:
            with suppress(ProcessLookupError):
                os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=6.0)
            except subprocess.TimeoutExpired:
                with suppress(ProcessLookupError):
                    os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3.0)
        with suppress(RuntimeError):
            ensure_crazyradio_free()


def _spin_for(node, seconds: float, safety_stopped) -> None:
    import rclpy

    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and not safety_stopped():
        rclpy.spin_once(node, timeout_sec=0.05)


def _call(node, client, request, timeout_s: float = 5.0):
    import rclpy

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    if not future.done():
        raise RuntimeError("gateway service call timed out")
    return future.result()


def _run_sequence(launch_process: subprocess.Popen) -> None:
    import rclpy
    from builtin_interfaces.msg import Duration
    from crazyflie_interfaces.srv import Land, Stop, Takeoff
    from std_msgs.msg import String
    from std_srvs.srv import SetBool

    rclpy.init()
    node = rclpy.create_node("managed_swarm_hover")
    safety_action = False
    accepted_takeoffs: set[str] = set()
    rejected_takeoffs: set[str] = set()
    accepted_landings: set[str] = set()

    def event_callback(message) -> None:
        nonlocal safety_action
        event = json.loads(message.data)
        command = event.get("command", "")
        status = event.get("status", "")
        robot = event.get("robot", "")
        details = event.get("details", "")
        print(f"EVENT {command} {status} {robot}: {details}", flush=True)
        if command == "takeoff" and status == "accepted":
            accepted_takeoffs.add(robot)
        elif command.startswith("takeoff/") and status == "rejected":
            rejected_takeoffs.add(command.split("/", 1)[1])
        elif command == "land" and status == "accepted":
            accepted_landings.add(robot)
        elif command in {"safety_land", "emergency"}:
            safety_action = True

    node.create_subscription(String, "/crazyfly/commands", event_callback, 10)
    enable = node.create_client(SetBool, "/crazyfly/enable")
    takeoff_clients = {
        name: node.create_client(Takeoff, f"/crazyfly/{name}/takeoff")
        for name in EXPECTED_ROBOTS
    }
    land_clients = {
        name: node.create_client(Land, f"/crazyfly/{name}/land")
        for name in EXPECTED_ROBOTS
    }
    emergency = node.create_client(Stop, "/crazyfly/emergency")

    try:
        for name, client in (
            [("enable", enable), ("emergency", emergency)]
            + [(f"takeoff/{name}", client) for name, client in takeoff_clients.items()]
            + [(f"land/{name}", client) for name, client in land_clients.items()]
        ):
            if launch_process.poll() is not None:
                raise RuntimeError("ROS launch exited before services became ready")
            if not client.wait_for_service(timeout_sec=20.0):
                raise RuntimeError(f"gateway service unavailable: {name}")

        # Allow radio connections, status logs, and the gateway recovery
        # interval to stabilize. Enable performs the actual health check and is
        # retried for transient firmware/log startup states.
        hard_failures = (
            "locked:",
            "tumbled:",
            "crashed:",
            "hard geofence",
            "separation violation",
            "battery below threshold",
        )
        startup_deadline = time.monotonic() + 60.0
        last_reason = ""
        while True:
            _spin_for(node, 1.0, lambda: safety_action)
            response = _call(node, enable, SetBool.Request(data=True))
            print(
                f"ENABLE success={response.success}: {response.message}",
                flush=True,
            )
            if response.success:
                break
            last_reason = response.message
            if any(reason in last_reason for reason in hard_failures):
                raise RuntimeError(f"gateway refused enable: {last_reason}")
            if time.monotonic() >= startup_deadline:
                raise RuntimeError(
                    f"gateway did not become ready within 60 s: {last_reason}"
                )

        takeoff_futures = []
        for name, client in takeoff_clients.items():
            request = Takeoff.Request()
            request.group_mask = 0
            request.height = 0.30
            request.duration = Duration(sec=3)
            takeoff_futures.append((name, client.call_async(request)))
        deadline = time.monotonic() + 3.0
        while (
            any(not future.done() for _, future in takeoff_futures)
            or len(accepted_takeoffs | rejected_takeoffs) < len(EXPECTED_ROBOTS)
        ) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if accepted_takeoffs != set(EXPECTED_ROBOTS):
            raise RuntimeError(
                "not all takeoffs were accepted; accepted="
                f"{sorted(accepted_takeoffs)}, rejected={sorted(rejected_takeoffs)}"
            )

        print("All selected takeoffs accepted; 3 s climb + 5 s hover", flush=True)
        _spin_for(node, 8.0, lambda: safety_action)
        if safety_action:
            print("Gateway ended the flight; skipping duplicate landing", flush=True)
            _spin_for(node, 1.5, lambda: False)
        else:
            land_futures = []
            for name, client in land_clients.items():
                request = Land.Request()
                request.group_mask = 0
                request.height = 0.04
                request.duration = Duration(sec=1)
                land_futures.append((name, client.call_async(request)))
            deadline = time.monotonic() + 3.0
            while (
                any(not future.done() for _, future in land_futures)
                or len(accepted_landings) < len(EXPECTED_ROBOTS)
            ) and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            if accepted_landings != set(EXPECTED_ROBOTS):
                if emergency.service_is_ready():
                    emergency.call_async(Stop.Request())
                raise RuntimeError(
                    "not all landings were accepted; emergency requested; accepted="
                    f"{sorted(accepted_landings)}"
                )
            print("All selected landings accepted", flush=True)
            _spin_for(node, 1.5, lambda: False)
    finally:
        with _ignore_cleanup_interrupts():
            if enable.service_is_ready():
                with suppress(Exception):
                    response = _call(node, enable, SetBool.Request(data=False), 2.0)
                    print(
                        f"DISABLE success={response.success}: {response.message}",
                        flush=True,
                    )
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    launch_process = None
    try:
        fleet, safety = validate(
            args.fleet,
            args.safety,
            args.motion_capture,
            require_motive=True,
        )
        enabled = tuple(sorted(fleet.enabled))
        if enabled != EXPECTED_ROBOTS:
            raise ConfigError(
                f"exactly {EXPECTED_ROBOTS} must be enabled; found {enabled}"
            )
        if safety.flight_enabled:
            raise ConfigError(
                "persistent safety file must remain flight_enabled: false; "
                "this tool uses a temporary enabled copy"
            )
        print("PASS: selected-fleet configuration is valid")
        for name in EXPECTED_ROBOTS:
            robot = fleet.enabled[name]
            print(f"  {name}: {robot.uri}, start={robot.initial_position}")
        print("Sequence: synchronized 0.30 m takeoff over 3 s, 5 s hover, 1 s landing")
        device, holders, conflicts = describe_status()
        if holders or conflicts:
            print(
                f"Crazyradio {device} is busy; execute mode will safely release "
                f"known processes {sorted(set(holders) | set(conflicts))}"
            )
        else:
            print(f"Crazyradio is free: {device}")
        if not args.execute:
            print("DRY RUN: no ROS process, radio connection, or command was started")
            return 0
        if shutil.which("ros2") is None:
            raise RuntimeError("ros2 is unavailable; source tools/activate_ros.sh")
        ensure_crazyradio_free()

        with tempfile.TemporaryDirectory(prefix="crazyfly-hover-") as temporary:
            enabled_safety = _enabled_safety_copy(args.safety, temporary)
            command = [
                "ros2",
                "launch",
                "crazyfly",
                "swarm.launch.py",
                "allow_hardware:=true",
                f"fleet_config_file:={Path(args.fleet).resolve()}",
                f"safety_config_file:={enabled_safety}",
                f"motion_capture_yaml_file:={Path(args.motion_capture).resolve()}",
                f"server_config_file:={Path(args.server).resolve()}",
            ]
            launch_process = subprocess.Popen(command, start_new_session=True)
            _run_sequence(launch_process)
        return 0
    except KeyboardInterrupt:
        print(
            "Interrupted; disabling and stopping the complete ROS launch tree",
            file=sys.stderr,
        )
        return 130
    except (ConfigError, RuntimeError, OSError, yaml.YAMLError) as exc:
        print(f"FAILED: {exc}", file=sys.stderr)
        return 2
    finally:
        _stop_launch(launch_process)


if __name__ == "__main__":
    raise SystemExit(main())
