#!/usr/bin/env python3
"""Compile and run one safety-gated synchronized polynomial mission."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import statistics
import subprocess
import sys
import tempfile
import time
from contextlib import suppress
from itertools import combinations
from pathlib import Path

import yaml

try:
    from . import all_drone_hover as hover
    from .crazyradio_guard import describe_status, ensure_crazyradio_free
except ImportError:  # Direct execution from the tools directory.
    import all_drone_hover as hover
    from crazyradio_guard import describe_status, ensure_crazyradio_free

from crazyfly.config import (
    ConfigError,
    SafetyConfig,
    load_fleet,
    load_safety,
    point_in_geofence_frame,
)
from crazyfly.config_validator import validate
from crazyfly.safety import continuous_minimum_separation
from crazyfly.trajectory import (
    TrajectoryPlan,
    evaluate_plan,
    load_trajectory,
    trajectory_from_payload,
    trajectory_payload,
)


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_FLEET = ROOT / "config" / "local" / "crazyflies.yaml"
DEFAULT_SAFETY = ROOT / "config" / "local" / "safety.yaml"
DEFAULT_MOTION_CAPTURE = ROOT / "config" / "local" / "motion_capture.yaml"
DEFAULT_SERVER = ROOT / "config" / "server.yaml"
DEFAULT_OUTPUT_ROOT = ROOT / "experiments"
MOCK_FLEET = ROOT / "config" / "mock_crazyflies.yaml"
MOCK_SAFETY = ROOT / "config" / "mock_safety.yaml"
RETURN_BASE_ALTITUDE_M = 0.20
RETURN_ALTITUDE_SPACING_M = 0.20
RETURN_SPEED_M_S = 0.20


def bounded_batch_id(mission_id: str, label: str, sequence: int) -> str:
    """Return one readable, deterministic gateway batch ID of at most 64 characters."""
    raw = f"{mission_id}-{label}-{sequence}"
    if len(raw) <= 64:
        return raw
    digest = hashlib.sha256(raw.encode("utf-8")).hexdigest()[:8]
    return f"{raw[:55]}-{digest}"


def staged_return_goals(
    current: dict[str, tuple[float, float, float]],
    anchors: dict[str, tuple[float, float, float]],
    safety: SafetyConfig,
) -> tuple[tuple[str, dict[str, tuple[float, float, float]]], ...]:
    """Build vertical-separation, horizontal-return, and descent stages."""
    if set(current) != set(anchors) or not current:
        raise ConfigError("staged return requires matching current poses and anchors")
    names = tuple(sorted(current))
    for first, second in combinations(names, 2):
        if math.dist(anchors[first], anchors[second]) < safety.minimum_separation_m:
            raise ConfigError(
                f"captured anchors for {first} and {second} are closer than "
                f"the {safety.minimum_separation_m:.2f} m live separation limit"
            )

    spacing = max(
        RETURN_ALTITUDE_SPACING_M,
        safety.minimum_separation_m + 0.05,
    )
    lower = RETURN_BASE_ALTITUDE_M
    upper = float("inf")
    if safety.has_geofence:
        assert safety.geofence_min is not None
        assert safety.geofence_max is not None
        lower = max(
            lower,
            safety.geofence_min[2] + safety.soft_geofence_margin_m,
        )
        upper = safety.geofence_max[2] - safety.soft_geofence_margin_m
    altitudes = {
        name: lower + index * spacing for index, name in enumerate(names)
    }
    if max(altitudes.values()) > upper:
        raise ConfigError("not enough geofence height for separated return lanes")

    separate = {
        name: (current[name][0], current[name][1], altitudes[name])
        for name in names
    }
    over_anchors = {
        name: (anchors[name][0], anchors[name][1], altitudes[name])
        for name in names
    }
    stages = (
        ("separate-altitudes", separate),
        ("horizontal-to-anchors", over_anchors),
    )
    if safety.has_geofence:
        assert safety.geofence_min is not None
        assert safety.geofence_max is not None
        for label, goals in stages:
            for name, point in goals.items():
                for axis, value in enumerate(point):
                    minimum = (
                        safety.geofence_min[axis]
                        + safety.soft_geofence_margin_m
                    )
                    maximum = (
                        safety.geofence_max[axis]
                        - safety.soft_geofence_margin_m
                    )
                    if not minimum <= value <= maximum:
                        raise ConfigError(
                            f"{label} goal for {name} is outside the effective geofence"
                        )
    for label, goals in stages:
        for first, second in combinations(names, 2):
            if math.dist(goals[first], goals[second]) < safety.minimum_separation_m:
                raise ConfigError(
                    f"{label} violates live separation for {first} and {second}"
                )
    return stages


def staged_preposition_goals(
    current: dict[str, tuple[float, float, float]],
    starts: dict[str, tuple[float, float, float]],
    safety: SafetyConfig,
) -> tuple[tuple[str, dict[str, tuple[float, float, float]]], ...]:
    """Build separated altitude lanes ending at exact trajectory starts."""
    return_stages = staged_return_goals(current, starts, safety)
    for name, point in starts.items():
        if safety.has_geofence:
            assert safety.geofence_min is not None
            assert safety.geofence_max is not None
            for axis, value in enumerate(point):
                minimum = (
                    safety.geofence_min[axis]
                    + safety.soft_geofence_margin_m
                )
                maximum = (
                    safety.geofence_max[axis]
                    - safety.soft_geofence_margin_m
                )
                if not minimum <= value <= maximum:
                    raise ConfigError(
                        f"trajectory start for {name} is outside the effective geofence"
                    )
    return (
        ("separate-altitudes", return_stages[0][1]),
        ("horizontal-to-starts", return_stages[1][1]),
        ("align-start-altitudes", dict(starts)),
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Compile a version-2 UR-base trajectory and optionally execute the "
            "complete takeoff-to-start, synchronized flight, return, and landing."
        )
    )
    parser.add_argument("trajectory", nargs="?")
    parser.add_argument("--trajectory", dest="trajectory_option")
    parser.add_argument(
        "--compiled-payload",
        help=(
            "exact compiled polynomial JSON from pRRTC; coefficients are "
            "validated and uploaded without waypoint recompilation"
        ),
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--wait-for-start", action="store_true")
    parser.add_argument("--start-timeout-s", type=float, default=30.0)
    parser.add_argument("--wait-for-return", action="store_true")
    parser.add_argument("--return-timeout-s", type=float, default=10.0)
    parser.add_argument(
        "--playback-timescale",
        type=float,
        default=1.0,
        help="duration multiplier; values above 1.0 slow the trajectory",
    )
    parser.add_argument(
        "--trajectory-id",
        type=int,
        help="waypoint missions only; compiled payloads carry their own ID",
    )
    parser.add_argument("--fleet")
    parser.add_argument("--safety")
    parser.add_argument("--motion-capture", default=str(DEFAULT_MOTION_CAPTURE))
    parser.add_argument("--server", default=str(DEFAULT_SERVER))
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT))
    return parser


def validate_scheduled_start(
    payload: object,
    *,
    mission_id: str,
    mission_ready: bool,
    already_scheduled: bool,
    now_ns: int,
) -> int:
    if not isinstance(payload, dict):
        raise ConfigError("mission schedule must contain a JSON object")
    if payload.get("mission_id") != mission_id:
        raise ConfigError("mission schedule ID does not match the prepared mission")
    if not mission_ready:
        raise ConfigError("mission is not ready")
    if already_scheduled:
        raise ConfigError("mission start is already scheduled")
    start_ns = payload.get("start_monotonic_ns")
    if isinstance(start_ns, bool) or not isinstance(start_ns, int):
        raise ConfigError("start_monotonic_ns must be an integer")
    lead_ns = start_ns - now_ns
    if lead_ns < 1_000_000_000:
        raise ConfigError("scheduled start must be at least one second in the future")
    if lead_ns > 10_000_000_000:
        raise ConfigError("scheduled start must be no more than ten seconds ahead")
    return start_ns


def _load_compiled_payload(
    path: str | Path,
    robot_names: tuple[str, ...],
    safety: SafetyConfig,
) -> tuple[dict[str, object], str, int, TrajectoryPlan]:
    source = Path(path).expanduser().resolve()
    try:
        payload: object = json.loads(source.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ConfigError(f"compiled payload is not valid JSON: {exc}") from exc
    if not isinstance(payload, dict):
        raise ConfigError("compiled payload must contain a JSON object")
    mission_id, trajectory_id, plan = trajectory_from_payload(
        payload, robot_names, safety
    )
    canonical = trajectory_payload(
        plan, mission_id=mission_id, trajectory_id=trajectory_id
    )
    if payload != canonical:
        raise ConfigError(
            "compiled payload changes during normalization; refusing to alter coefficients"
        )
    return payload, mission_id, trajectory_id, plan


def _trajectory_record(
    source: Path,
    *,
    kind: str,
    mission_id: str | None,
    trajectory_id: int,
    plan: TrajectoryPlan,
) -> dict[str, object]:
    assert plan.metrics is not None
    return {
        "kind": kind,
        "path": str(source),
        "sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "mission_id": mission_id,
        "trajectory_id": trajectory_id,
        "duration_s": plan.duration_s,
        "robots": sorted(plan.trajectories),
        "start_positions_base_m": plan.start_positions,
        "end_positions_base_m": plan.end_positions,
        "metrics": {
            "maximum_speed_m_s": plan.metrics.maximum_speed_m_s,
            "maximum_acceleration_m_s2": (
                plan.metrics.maximum_acceleration_m_s2
            ),
            "maximum_jerk_m_s3": plan.metrics.maximum_jerk_m_s3,
            "minimum_separation_m": plan.metrics.minimum_separation_m,
        },
    }


def _publish_event(publisher, command: str, status: str, details: object) -> None:
    from std_msgs.msg import String

    message = String()
    message.data = json.dumps(
        {
            "monotonic_s": time.monotonic(),
            "command": command,
            "status": status,
            "robot": "",
            "details": (
                details
                if isinstance(details, str)
                else json.dumps(details, separators=(",", ":"))
            ),
        },
        separators=(",", ":"),
    )
    publisher.publish(message)


def _percentile(values: list[float], fraction: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def mission_report(
    plan: TrajectoryPlan,
    started_at: float,
    pose_samples: list[tuple[float, dict[str, tuple[float, float, float]]]],
    battery_minima: dict[str, float],
    playback_timescale: float = 1.0,
) -> dict[str, object]:
    errors = {name: [] for name in plan.trajectories}
    minimum_separation: float | None = None
    motion_started_at: dict[str, float] = {}
    measured_starts: dict[str, tuple[float, float, float]] | None = None
    for timestamp, positions in pose_samples:
        physical_elapsed = timestamp - started_at
        elapsed = physical_elapsed / playback_timescale
        if not 0.0 <= elapsed <= plan.duration_s or set(positions) != set(
            plan.trajectories
        ):
            continue
        expected = evaluate_plan(plan, elapsed)
        if measured_starts is None:
            measured_starts = dict(positions)
        for name in plan.trajectories:
            error = math.dist(positions[name], expected[name])
            errors[name].append(error)
            if (
                name not in motion_started_at
                and math.dist(positions[name], measured_starts[name]) >= 0.01
            ):
                motion_started_at[name] = physical_elapsed
        for first, second in combinations(sorted(positions), 2):
            separation = math.dist(positions[first], positions[second])
            minimum_separation = (
                separation
                if minimum_separation is None
                else min(minimum_separation, separation)
            )
    per_robot = {}
    for name, values in errors.items():
        if values:
            per_robot[name] = {
                "mean_m": statistics.fmean(values),
                "rms_m": math.sqrt(statistics.fmean(value * value for value in values)),
                "p95_m": _percentile(values, 0.95),
                "maximum_m": max(values),
            }
    onset_values = list(motion_started_at.values())
    onset_is_observable = (
        plan.metrics is not None and plan.metrics.maximum_speed_m_s >= 0.10
    )
    return {
        "duration_s": plan.duration_s * playback_timescale,
        "source_duration_s": plan.duration_s,
        "playback_timescale": playback_timescale,
        "tracking_error": per_robot,
        "minimum_separation_m": minimum_separation,
        "motion_start_skew_s": (
            max(onset_values) - min(onset_values)
            if onset_is_observable and len(onset_values) == len(plan.trajectories)
            else None
        ),
        "battery_minimum_v": battery_minima,
    }


def _run_mission(
    launch_process: subprocess.Popen,
    *,
    fleet,
    safety: SafetyConfig,
    plan: TrajectoryPlan,
    mission_id: str,
    trajectory_id: int,
    wait_for_start: bool,
    start_timeout_s: float,
    wait_for_return: bool,
    return_timeout_s: float,
    playback_timescale: float,
    skip_estimator_gate: bool,
    trajectory_record: dict[str, object],
) -> dict[str, object]:
    import rclpy
    from builtin_interfaces.msg import Duration
    from crazyflie_interfaces.msg import LogDataGeneric, Status
    from crazyflie_interfaces.srv import Land, Stop, Takeoff
    from motion_capture_tracking_interfaces.msg import NamedPoseArray
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from std_srvs.srv import SetBool, Trigger

    robot_names = tuple(sorted(fleet.enabled))
    rclpy.init()
    node = rclpy.create_node("crazyfly_trajectory_mission")
    command_publisher = node.create_publisher(String, "/crazyfly/commands", 10)
    upload_publisher = node.create_publisher(
        String, "/crazyfly/trajectory_upload_requests", 10
    )
    start_publisher = node.create_publisher(
        String, "/crazyfly/trajectory_start_requests", 10
    )
    batch_publisher = node.create_publisher(
        String, "/crazyfly/batch_go_to_requests", 10
    )
    results: dict[tuple[str, str], tuple[str, dict[str, object]]] = {}
    accepted_takeoffs: set[str] = set()
    accepted_landings: set[str] = set()
    safety_action = False
    mission_ready = False
    endpoint_ready = False
    external_start_requested = False
    scheduled_start_ns: int | None = None
    return_requested = False
    abort_requested = False
    positions_base: dict[str, tuple[float, float, float]] = {}
    position_received_at: dict[str, float] = {}
    pose_samples: list[tuple[float, dict[str, tuple[float, float, float]]]] = []
    estimator_variances = {name: [] for name in robot_names}
    battery_minima: dict[str, float] = {}
    trajectory_started_at: float | None = None
    flight_active = False
    launch_positions: dict[str, tuple[float, float, float]] | None = None

    def event_callback(message: String) -> None:
        nonlocal safety_action, trajectory_started_at
        event = json.loads(message.data)
        command = str(event.get("command", ""))
        status = str(event.get("status", ""))
        robot = str(event.get("robot", ""))
        raw_details = event.get("details", "")
        print(f"EVENT {command} {status} {robot}: {raw_details}", flush=True)
        try:
            details = json.loads(raw_details) if raw_details else {}
        except (TypeError, json.JSONDecodeError):
            details = {"reason": raw_details}
        key = ""
        if command == "batch_go_to":
            key = str(details.get("batch_id", ""))
        elif command in {"trajectory_upload", "trajectory_start"}:
            key = str(details.get("mission_id", ""))
        if key and status in {"accepted", "rejected"}:
            results[(command, key)] = (status, details)
        if command == "takeoff" and status == "accepted":
            accepted_takeoffs.add(robot)
        elif command == "land" and status == "accepted":
            accepted_landings.add(robot)
        elif command in {"safety_land", "emergency"}:
            safety_action = True
        elif command == "trajectory_start" and status == "accepted":
            trajectory_started_at = float(event.get("monotonic_s", time.monotonic()))

    def pose_callback(message: NamedPoseArray) -> None:
        received_at = time.monotonic()
        for named_pose in message.poses:
            if named_pose.name in robot_names:
                point = named_pose.pose.position
                positions_base[named_pose.name] = point_in_geofence_frame(
                    (point.x, point.y, point.z), safety
                )
                position_received_at[named_pose.name] = received_at
        if set(positions_base) == set(robot_names):
            pose_samples.append((received_at, dict(positions_base)))

    def start_service_callback(_request, response):
        nonlocal external_start_requested, scheduled_start_ns
        if not mission_ready:
            response.success = False
            response.message = "mission is not ready"
        elif external_start_requested:
            response.success = False
            response.message = "start was already requested"
        else:
            external_start_requested = True
            scheduled_start_ns = time.monotonic_ns()
            response.success = True
            response.message = f"start accepted for {mission_id}"
        return response

    def schedule_callback(message: String) -> None:
        nonlocal external_start_requested, scheduled_start_ns
        try:
            payload = json.loads(message.data)
            scheduled_start_ns = validate_scheduled_start(
                payload,
                mission_id=mission_id,
                mission_ready=mission_ready,
                already_scheduled=scheduled_start_ns is not None,
                now_ns=time.monotonic_ns(),
            )
            external_start_requested = True
            _publish_event(
                command_publisher,
                "mission_schedule",
                "accepted",
                {
                    "mission_id": mission_id,
                    "start_monotonic_ns": scheduled_start_ns,
                },
            )
        except (ConfigError, json.JSONDecodeError) as exc:
            _publish_event(
                command_publisher,
                "mission_schedule",
                "rejected",
                {"mission_id": mission_id, "reason": str(exc)},
            )

    def return_service_callback(_request, response):
        nonlocal return_requested
        if not endpoint_ready:
            response.success = False
            response.message = "mission endpoint is not ready"
        elif return_requested:
            response.success = False
            response.message = "return was already requested"
        else:
            return_requested = True
            response.success = True
            response.message = f"return accepted for {mission_id}"
        return response

    def abort_service_callback(_request, response):
        nonlocal abort_requested
        if abort_requested:
            response.success = False
            response.message = "abort was already requested"
        else:
            abort_requested = True
            response.success = True
            response.message = f"land-in-place accepted for {mission_id}"
        return response

    node.create_subscription(String, "/crazyfly/commands", event_callback, 10)
    node.create_subscription(
        NamedPoseArray, "/poses", pose_callback, qos_profile_sensor_data
    )
    if wait_for_start:
        node.create_service(
            Trigger, "/crazyfly/mission/start", start_service_callback
        )
        node.create_subscription(
            String,
            "/crazyfly/mission/schedule_requests",
            schedule_callback,
            10,
        )
    if wait_for_return:
        node.create_service(
            Trigger, "/crazyfly/mission/return", return_service_callback
        )
        node.create_service(
            Trigger, "/crazyfly/mission/abort", abort_service_callback
        )
    for name in robot_names:
        def estimator_callback(
            message: LogDataGeneric, robot: str = name
        ) -> None:
            if len(message.values) >= 3:
                estimator_variances[robot].append(tuple(message.values[:3]))

        def status_callback(message: Status, robot: str = name) -> None:
            voltage = float(message.battery_voltage)
            battery_minima[robot] = min(battery_minima.get(robot, voltage), voltage)

        node.create_subscription(
            LogDataGeneric,
            f"/{name}/estimator_debug",
            estimator_callback,
            10,
        )
        node.create_subscription(Status, f"/{name}/status", status_callback, 10)

    enable = node.create_client(SetBool, "/crazyfly/enable")
    emergency = node.create_client(Stop, "/crazyfly/emergency")
    takeoff_clients = {
        name: node.create_client(Takeoff, f"/crazyfly/{name}/takeoff")
        for name in robot_names
    }
    land_clients = {
        name: node.create_client(Land, f"/crazyfly/{name}/land")
        for name in robot_names
    }

    def spin_for(seconds: float) -> None:
        hover._spin_for(node, seconds, lambda: safety_action or abort_requested)

    def current_positions() -> dict[str, tuple[float, float, float]]:
        now = time.monotonic()
        if set(positions_base) != set(robot_names):
            raise RuntimeError("not all base-frame poses are available")
        stale = [
            name
            for name in robot_names
            if now - position_received_at.get(name, 0.0) >= safety.pose_reject_age_s
        ]
        if stale:
            raise RuntimeError(f"base-frame poses are stale for: {', '.join(stale)}")
        return dict(positions_base)

    def fresh_positions(timeout_s: float = 1.0) -> dict[str, tuple[float, float, float]]:
        deadline = time.monotonic() + timeout_s
        last_error = "poses did not become fresh"
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            try:
                return current_positions()
            except RuntimeError as exc:
                last_error = str(exc)
        raise RuntimeError(last_error)

    def wait_result(command: str, key: str, timeout_s: float) -> dict[str, object]:
        deadline = time.monotonic() + timeout_s
        while (command, key) not in results and time.monotonic() < deadline:
            if launch_process.poll() is not None:
                raise RuntimeError("ROS launch exited while waiting for acknowledgement")
            rclpy.spin_once(node, timeout_sec=0.05)
        if (command, key) not in results:
            raise RuntimeError(f"{command} was not acknowledged")
        status, details = results[(command, key)]
        if status != "accepted":
            raise RuntimeError(
                f"{command} rejected: {details.get('reason', details)}"
            )
        return details

    def publish_request(publisher, payload: dict[str, object]) -> None:
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        publisher.publish(message)

    def wait_for_services() -> None:
        clients = [("enable", enable), ("emergency", emergency)]
        clients.extend((f"takeoff/{name}", client) for name, client in takeoff_clients.items())
        clients.extend((f"land/{name}", client) for name, client in land_clients.items())
        for label, client in clients:
            if not client.wait_for_service(timeout_sec=20.0):
                raise RuntimeError(f"gateway service unavailable: {label}")
        spin_for(0.5)

    def wait_for_estimators() -> None:
        if skip_estimator_gate:
            return
        print("Waiting for stable Kalman estimates: " + ", ".join(robot_names))
        deadline = time.monotonic() + 20.0
        while not all(
            hover._estimator_variance_stable(estimator_variances[name])
            for name in robot_names
        ):
            if time.monotonic() >= deadline:
                counts = {name: len(estimator_variances[name]) for name in robot_names}
                raise RuntimeError(
                    f"startup telemetry unavailable: estimator samples {counts}"
                )
            rclpy.spin_once(node, timeout_sec=0.10)

    def enable_when_ready() -> None:
        wait_for_estimators()
        deadline = time.monotonic() + 60.0
        while True:
            spin_for(1.0)
            response = hover._call(node, enable, SetBool.Request(data=True))
            print(f"ENABLE success={response.success}: {response.message}")
            if response.success:
                return
            if any(reason in response.message for reason in hover.HARD_ENABLE_FAILURES):
                raise RuntimeError(f"gateway refused enable: {response.message}")
            if time.monotonic() >= deadline:
                raise RuntimeError(f"gateway did not become ready: {response.message}")

    def takeoff() -> None:
        futures = []
        for name in robot_names:
            request = Takeoff.Request()
            request.group_mask = 0
            request.height = 0.30
            request.duration = Duration(sec=3)
            futures.append(takeoff_clients[name].call_async(request))
        deadline = time.monotonic() + 3.0
        while (
            any(not future.done() for future in futures)
            or set(robot_names) - accepted_takeoffs
        ) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if set(robot_names) - accepted_takeoffs:
            raise RuntimeError("takeoff was not accepted for every drone")

    def land(*, from_return_lanes: bool = False) -> float:
        accepted_landings.clear()
        landing_positions = fresh_positions() if from_return_lanes else {}
        maximum_duration_s = 1.0
        for name in robot_names:
            duration_s = (
                max(
                    1.0,
                    (landing_positions[name][2] - 0.04) / RETURN_SPEED_M_S,
                )
                if from_return_lanes
                else 1.0
            )
            maximum_duration_s = max(maximum_duration_s, duration_s)
            request = Land.Request()
            request.group_mask = 0
            request.height = 0.04
            whole_seconds = int(duration_s)
            request.duration = Duration(
                sec=whole_seconds,
                nanosec=int((duration_s - whole_seconds) * 1_000_000_000),
            )
            land_clients[name].call_async(request)
        deadline = time.monotonic() + 3.0
        while set(robot_names) - accepted_landings and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if set(robot_names) - accepted_landings:
            raise RuntimeError("landing was not accepted for every drone")
        return maximum_duration_s

    def wait_settled(
        goals: dict[str, tuple[float, float, float]],
        tolerance_m: float,
        stable_s: float,
        timeout_s: float,
    ) -> None:
        stable_since = None
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if safety_action or abort_requested:
                raise RuntimeError("safety gateway stopped the mission")
            actual = current_positions()
            if all(math.dist(actual[name], goals[name]) <= tolerance_m for name in robot_names):
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= stable_s:
                    return
            else:
                stable_since = None
        errors = {
            name: math.dist(current_positions()[name], goals[name])
            for name in robot_names
        }
        raise RuntimeError(f"robots did not settle at targets: {errors}")

    batch_sequence = 0

    def send_batch(
        label: str,
        goals: dict[str, tuple[float, float, float]],
        duration_s: float,
        tolerance_m: float,
    ) -> None:
        nonlocal batch_sequence
        batch_sequence += 1
        batch_id = bounded_batch_id(mission_id, label, batch_sequence)
        publish_request(
            batch_publisher,
            {
                "batch_id": batch_id,
                "frame": "base",
                "duration_s": duration_s,
                "yaw_rad": 0.0,
                "goals": goals,
            },
        )
        wait_result("batch_go_to", batch_id, 3.0)
        spin_for(duration_s)
        wait_settled(goals, tolerance_m, 0.5, 3.0)

    def return_to_launch(label: str) -> None:
        if launch_positions is None:
            raise RuntimeError("captured launch positions are unavailable")
        errors = []
        for attempt in range(1, 4):
            if safety_action:
                raise RuntimeError("safety gateway stopped return-to-launch")
            try:
                actual = fresh_positions()
                stages = staged_return_goals(actual, launch_positions, safety)
                stage_start = actual
                for stage_label, goals in stages:
                    distance_m = max(
                        math.dist(stage_start[name], goals[name])
                        for name in robot_names
                    )
                    duration_s = max(2.0, distance_m / RETURN_SPEED_M_S)
                    send_batch(
                        f"{label}-{stage_label}-attempt-{attempt}",
                        goals,
                        duration_s,
                        0.08,
                    )
                    stage_start = goals
                return
            except RuntimeError as exc:
                errors.append(str(exc))
                spin_for(1.0)
        raise RuntimeError("return-to-launch failed: " + "; ".join(errors))

    try:
        wait_for_services()
        _publish_event(
            command_publisher,
            "mission_plan",
            "accepted",
            {
                "mission_id": mission_id,
                "trajectory_id": trajectory_id,
                "trajectory": trajectory_record,
                "duration_s": plan.duration_s,
                "frame": plan.frame,
            },
        )
        table_positions = {
            name: point_in_geofence_frame(robot.initial_position, safety)
            for name, robot in fleet.enabled.items()
        }
        deadline = time.monotonic() + 5.0
        while set(positions_base) != set(robot_names) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        table_errors = {
            name: math.dist(current_positions()[name], table_positions[name])
            for name in robot_names
        }
        if any(error > 0.08 for error in table_errors.values()):
            raise RuntimeError(
                f"drones are not at their labeled table starts: {table_errors}"
            )

        publish_request(
            upload_publisher,
            trajectory_payload(
                plan, mission_id=mission_id, trajectory_id=trajectory_id
            ),
        )
        wait_result("trajectory_upload", mission_id, 15.0)
        enable_when_ready()
        takeoff()
        flight_active = True
        spin_for(3.5)
        if safety_action:
            raise RuntimeError("safety gateway stopped takeoff")
        launch_positions = fresh_positions()
        start_positions = plan.start_positions
        preposition_minimum, _pair, _fraction = continuous_minimum_separation(
            launch_positions, start_positions
        )
        if preposition_minimum >= safety.minimum_separation_m:
            preposition_distance = max(
                math.dist(launch_positions[name], start_positions[name])
                for name in robot_names
            )
            preposition_duration = max(2.0, preposition_distance / 0.20)
            send_batch(
                "preposition-direct",
                start_positions,
                preposition_duration,
                0.05,
            )
        else:
            stage_start = launch_positions
            for stage_label, goals in staged_preposition_goals(
                launch_positions, start_positions, safety
            ):
                distance_m = max(
                    math.dist(stage_start[name], goals[name])
                    for name in robot_names
                )
                duration_s = max(2.0, distance_m / RETURN_SPEED_M_S)
                send_batch(
                    f"preposition-{stage_label}",
                    goals,
                    duration_s,
                    0.05 if stage_label == "align-start-altitudes" else 0.08,
                )
                stage_start = goals

        mission_ready = True
        _publish_event(
            command_publisher,
            "mission_ready",
            "accepted",
            {
                "mission_id": mission_id,
                "start_service": "/crazyfly/mission/start" if wait_for_start else None,
            },
        )
        if wait_for_start:
            print(
                "Mission ready; waiting for /crazyfly/mission/start",
                flush=True,
            )
            deadline = time.monotonic() + start_timeout_s
            while not external_start_requested and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if safety_action:
                    raise RuntimeError("safety gateway stopped while waiting")
            if not external_start_requested:
                raise RuntimeError("external mission start timed out")
            assert scheduled_start_ns is not None
            while time.monotonic_ns() < scheduled_start_ns:
                rclpy.spin_once(node, timeout_sec=0.01)
                if safety_action or abort_requested:
                    raise RuntimeError("mission stopped before scheduled start")

        publish_request(
            start_publisher,
            {
                "mission_id": mission_id,
                "trajectory_id": trajectory_id,
                "timescale": playback_timescale,
            },
        )
        wait_result("trajectory_start", mission_id, 3.0)
        if trajectory_started_at is None:
            trajectory_started_at = time.monotonic()
        spin_for(plan.duration_s * playback_timescale + 0.5)
        if safety_action or abort_requested:
            raise RuntimeError("safety gateway stopped trajectory execution")
        wait_settled(plan.end_positions, 0.08, 0.3, 2.0)
        _publish_event(
            command_publisher,
            "trajectory_complete",
            "accepted",
            {
                "mission_id": mission_id,
                "duration_s": plan.duration_s * playback_timescale,
                "source_duration_s": plan.duration_s,
                "playback_timescale": playback_timescale,
            },
        )

        endpoint_ready = True
        if wait_for_return:
            _publish_event(
                command_publisher,
                "mission_endpoint_ready",
                "accepted",
                {
                    "mission_id": mission_id,
                    "return_service": "/crazyfly/mission/return",
                    "abort_service": "/crazyfly/mission/abort",
                },
            )
            deadline = time.monotonic() + return_timeout_s
            while (
                not return_requested
                and not abort_requested
                and time.monotonic() < deadline
            ):
                rclpy.spin_once(node, timeout_sec=0.05)
                if safety_action:
                    raise RuntimeError("safety gateway stopped endpoint hold")
            if abort_requested or not return_requested:
                landing_duration_s = land()
                spin_for(landing_duration_s + 0.5)
                flight_active = False
                reason = (
                    "coordinator requested land-in-place"
                    if abort_requested
                    else "arm park acknowledgement timed out"
                )
                raise RuntimeError(reason)

        return_to_launch("return")
        landing_duration_s = land(from_return_lanes=True)
        spin_for(landing_duration_s + 0.5)
        flight_active = False
        report = mission_report(
            plan,
            trajectory_started_at,
            pose_samples,
            battery_minima,
            playback_timescale,
        )
        completed = dict(report)
        completed["mission_id"] = mission_id
        completed["returned_to_launch"] = True
        _publish_event(
            command_publisher, "mission_complete", "accepted", completed
        )
        return report
    except BaseException as exc:
        _publish_event(
            command_publisher,
            "mission_incomplete",
            "rejected",
            {
                "mission_id": mission_id,
                "reason": str(exc),
                "identity_recheck_required": flight_active,
            },
        )
        if flight_active and not safety_action:
            with suppress(Exception):
                if wait_for_return:
                    landing_duration_s = land()
                    spin_for(landing_duration_s + 0.5)
                else:
                    return_to_launch("abort-return")
                    landing_duration_s = land(from_return_lanes=True)
                    spin_for(landing_duration_s + 0.5)
        raise
    finally:
        with hover._ignore_cleanup_interrupts():
            if enable.service_is_ready():
                with suppress(Exception):
                    hover._call(node, enable, SetBool.Request(data=False), 2.0)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def _print_plan(plan: TrajectoryPlan) -> None:
    assert plan.metrics is not None
    print(f"PASS: mission {plan.name!r} is continuously valid")
    print(f"  frame:        {plan.frame}")
    print(f"  robots:       {', '.join(sorted(plan.trajectories))}")
    print(f"  duration:     {plan.duration_s:.2f} s")
    print(f"  max speed:    {plan.metrics.maximum_speed_m_s:.3f} m/s")
    print(
        f"  max accel:    {plan.metrics.maximum_acceleration_m_s2:.3f} m/s^2"
    )
    print(f"  max jerk:     {plan.metrics.maximum_jerk_m_s3:.3f} m/s^3")
    if plan.metrics.minimum_separation_m is not None:
        print(f"  min spacing:  {plan.metrics.minimum_separation_m:.3f} m")


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    launch_process = None
    try:
        sources = [
            value
            for value in (
                args.trajectory,
                args.trajectory_option,
                args.compiled_payload,
            )
            if value is not None
        ]
        if not sources:
            raise ConfigError("a trajectory or compiled payload file is required")
        if len(sources) != 1:
            raise ConfigError(
                "provide exactly one positional trajectory, --trajectory, or "
                "--compiled-payload"
            )
        if args.compiled_payload is not None and args.trajectory_id is not None:
            raise ConfigError("--trajectory-id cannot be used with --compiled-payload")
        if not math.isfinite(args.start_timeout_s) or args.start_timeout_s <= 0:
            raise ConfigError("--start-timeout-s must be positive and finite")
        if not math.isfinite(args.return_timeout_s) or args.return_timeout_s <= 0:
            raise ConfigError("--return-timeout-s must be positive and finite")
        if (
            not math.isfinite(args.playback_timescale)
            or args.playback_timescale < 1.0
        ):
            raise ConfigError("--playback-timescale must be finite and at least 1.0")
        if args.trajectory_id is not None and not 0 <= args.trajectory_id <= 255:
            raise ConfigError("--trajectory-id must be from 0 to 255")
        fleet_path = Path(
            args.fleet or (MOCK_FLEET if args.mock else DEFAULT_FLEET)
        ).resolve()
        safety_path = Path(
            args.safety or (MOCK_SAFETY if args.mock else DEFAULT_SAFETY)
        ).resolve()
        if args.mock:
            fleet = load_fleet(fleet_path)
            safety = load_safety(safety_path)
        else:
            fleet, safety = validate(
                str(fleet_path),
                str(safety_path),
                args.motion_capture,
                require_motive=args.execute,
            )
        embedded_mission_id: str | None = None
        if args.compiled_payload is not None:
            trajectory_source = Path(args.compiled_payload).expanduser().resolve()
            _, embedded_mission_id, trajectory_id, plan = _load_compiled_payload(
                trajectory_source,
                tuple(fleet.enabled),
                safety,
            )
            source_kind = "compiled_payload"
        else:
            trajectory_source = Path(sources[0]).expanduser().resolve()
            plan = load_trajectory(trajectory_source, fleet, safety)
            trajectory_id = 1 if args.trajectory_id is None else args.trajectory_id
            source_kind = "waypoints"
        trajectory_record = _trajectory_record(
            trajectory_source,
            kind=source_kind,
            mission_id=embedded_mission_id,
            trajectory_id=trajectory_id,
            plan=plan,
        )
        if (
            plan.duration_s * args.playback_timescale
            > safety.maximum_trajectory_duration_s
        ):
            raise ConfigError(
                "scaled trajectory duration exceeds the configured limit"
            )
        _print_plan(plan)
        if not args.execute:
            print("DRY RUN: no ROS process, radio connection, or command was started")
            return 0
        if shutil.which("ros2") is None:
            raise RuntimeError("ros2 is unavailable; source tools/activate_ros.sh")
        mission_id = embedded_mission_id or (
            f"{plan.name[:40]}-{time.time_ns() % 1_000_000_000:09d}"
        )
        if args.mock:
            command = [
                "ros2",
                "launch",
                "crazyfly",
                "mock_stack.launch.py",
                f"fleet_config_file:={fleet_path}",
                f"safety_config_file:={safety_path}",
            ]
            launch_process = subprocess.Popen(command, start_new_session=True)
            report = _run_mission(
                launch_process,
                fleet=fleet,
                safety=safety,
                plan=plan,
                mission_id=mission_id,
                trajectory_id=trajectory_id,
                wait_for_start=args.wait_for_start,
                start_timeout_s=args.start_timeout_s,
                wait_for_return=args.wait_for_return,
                return_timeout_s=args.return_timeout_s,
                playback_timescale=args.playback_timescale,
                skip_estimator_gate=True,
                trajectory_record=trajectory_record,
            )
        else:
            if safety.flight_enabled:
                raise ConfigError(
                    "persistent safety file must remain flight_enabled: false"
                )
            device, holders, conflicts = describe_status()
            if holders or conflicts:
                print(
                    f"Crazyradio {device} is busy; safely releasing known idle processes"
                )
            ensure_crazyradio_free()
            with tempfile.TemporaryDirectory(prefix="crazyfly-mission-") as temporary:
                enabled_safety = hover._enabled_safety_copy(
                    str(safety_path), temporary
                )
                command = [
                    "ros2",
                    "launch",
                    "crazyfly",
                    "swarm.launch.py",
                    "allow_hardware:=true",
                    f"fleet_config_file:={fleet_path}",
                    f"safety_config_file:={enabled_safety}",
                    f"motion_capture_yaml_file:={Path(args.motion_capture).resolve()}",
                    f"server_config_file:={Path(args.server).resolve()}",
                    f"experiment_output_root:={Path(args.output_root).resolve()}",
                ]
                launch_process = subprocess.Popen(command, start_new_session=True)
                report = _run_mission(
                    launch_process,
                    fleet=fleet,
                    safety=safety,
                    plan=plan,
                    mission_id=mission_id,
                    trajectory_id=trajectory_id,
                    wait_for_start=args.wait_for_start,
                    start_timeout_s=args.start_timeout_s,
                    wait_for_return=args.wait_for_return,
                    return_timeout_s=args.return_timeout_s,
                    playback_timescale=args.playback_timescale,
                    skip_estimator_gate=False,
                    trajectory_record=trajectory_record,
                )
        print("MISSION COMPLETE")
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0
    except KeyboardInterrupt:
        print("Interrupted; mission cleanup requested", file=sys.stderr)
        return 130
    except (ConfigError, RuntimeError, OSError, yaml.YAMLError) as exc:
        print(f"FAILED: {exc}", file=sys.stderr)
        return 2
    finally:
        hover._stop_launch(launch_process)


if __name__ == "__main__":
    raise SystemExit(main())
