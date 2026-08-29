"""Command-line validation for fleet, safety, Motive, and trajectory files."""

from __future__ import annotations

import argparse
from copy import deepcopy
from math import dist, isfinite
import sys
from typing import Any, Mapping

from .config import (
    ConfigError,
    FleetConfig,
    SafetyConfig,
    load_fleet,
    load_safety,
    load_yaml,
    point_inside_geofence,
)
from .trajectory import load_trajectory


def validate_motion_capture(
    path: str, require_motive: bool = False
) -> dict[str, Any]:
    motion = load_yaml(path)
    node = motion.get("/motion_capture_tracking")
    if not isinstance(node, Mapping):
        raise ConfigError("motion configuration must define /motion_capture_tracking")
    parameters = node.get("ros__parameters")
    if not isinstance(parameters, dict):
        raise ConfigError("motion configuration must define ros__parameters")
    backend = parameters.get("type")
    if backend not in {"optitrack", "optitrack_closed_source"}:
        raise ConfigError("the implementation requires an OptiTrack backend")

    hostname = str(parameters.get("hostname", "")).strip()
    unsafe_hosts = {"", "localhost", "127.0.0.1", "0.0.0.0", "::1"}
    if require_motive and (
        hostname.lower() in unsafe_hosts or "required" in hostname.lower()
    ):
        raise ConfigError("a real Motive hostname or IP is required")

    topics = parameters.get("topics")
    if not isinstance(topics, Mapping):
        raise ConfigError("motion configuration must define topics")
    frame_id = topics.get("frame_id")
    if not isinstance(frame_id, str) or not frame_id:
        raise ConfigError("motion topics.frame_id must be non-empty")
    poses = topics.get("poses")
    qos = poses.get("qos") if isinstance(poses, Mapping) else None
    if not isinstance(qos, Mapping):
        raise ConfigError("motion topics.poses.qos must be configured")
    if qos.get("mode") not in {"none", "sensor"}:
        raise ConfigError("motion pose QoS mode must be none or sensor")
    try:
        deadline = float(qos.get("deadline"))
    except (TypeError, ValueError) as exc:
        raise ConfigError("motion pose deadline must be a positive finite rate") from exc
    if not isfinite(deadline) or deadline <= 0:
        raise ConfigError("motion pose deadline must be a positive finite rate")

    transform = topics.get("tf")
    if not isinstance(transform, Mapping):
        raise ConfigError("motion topics.tf must be configured")
    if not isinstance(transform.get("reference_frame"), str) or not transform[
        "reference_frame"
    ]:
        raise ConfigError("motion TF reference_frame must be non-empty")
    child_format = transform.get("child_frame_fmt")
    if not isinstance(child_format, str) or "%s" not in child_format:
        raise ConfigError("motion TF child_frame_fmt must contain %s")
    return parameters


def prepare_motion_capture_parameters(
    motion_capture_path: str,
    fleet_path: str,
    require_motive: bool = False,
) -> dict[str, Any]:
    """Build mocap parameters, including locally tracked enabled robots."""
    parameters = deepcopy(
        validate_motion_capture(motion_capture_path, require_motive=require_motive)
    )
    fleet = load_fleet(fleet_path)
    fleet_data = load_yaml(fleet_path)
    robot_types = fleet_data["robot_types"]
    marker_configurations = parameters.get("marker_configurations")
    dynamics_configurations = parameters.get("dynamics_configurations")

    rigid_bodies: dict[str, Any] = {}
    for name, robot in fleet.enabled.items():
        motion_capture = robot_types[robot.robot_type]["motion_capture"]
        if motion_capture["tracking"] != "librigidbodytracker":
            continue

        marker = motion_capture["marker"]
        dynamics = motion_capture["dynamics"]
        if (
            not isinstance(marker_configurations, Mapping)
            or marker not in marker_configurations
        ):
            raise ConfigError(
                f"robot {name} references unknown marker configuration {marker!r}"
            )
        if (
            not isinstance(dynamics_configurations, Mapping)
            or dynamics not in dynamics_configurations
        ):
            raise ConfigError(
                f"robot {name} references unknown dynamics configuration {dynamics!r}"
            )
        rigid_bodies[name] = {
            "initial_position": list(robot.initial_position),
            "marker": marker,
            "dynamics": dynamics,
        }

    parameters["rigid_bodies"] = rigid_bodies
    return parameters


def validate(
    fleet_path: str,
    safety_path: str,
    motion_capture_path: str | None = None,
    require_motive: bool = False,
) -> tuple[FleetConfig, SafetyConfig]:
    fleet = load_fleet(fleet_path)
    safety = load_safety(safety_path)
    enabled = fleet.enabled
    if safety.flight_enabled and not enabled:
        raise ConfigError("flight_enabled requires at least one enabled robot")
    for name, robot in enabled.items():
        if not point_inside_geofence(robot.initial_position, safety):
            raise ConfigError(f"initial position for {name} lies outside the geofence")
    enabled_items = list(enabled.items())
    for index, (name, robot) in enumerate(enabled_items):
        for other_name, other in enabled_items[index + 1 :]:
            if (
                dist(robot.initial_position, other.initial_position)
                < safety.minimum_separation_m
            ):
                raise ConfigError(
                    f"initial positions violate separation: {name}/{other_name}"
                )
    if motion_capture_path:
        prepare_motion_capture_parameters(
            motion_capture_path,
            fleet_path,
            require_motive=require_motive,
        )
    return fleet, safety


def validate_hardware_configuration(
    fleet_path: str, safety_path: str, motion_capture_path: str
) -> tuple[FleetConfig, SafetyConfig]:
    fleet, safety = validate(
        fleet_path,
        safety_path,
        motion_capture_path,
        require_motive=True,
    )
    if not safety.flight_enabled:
        raise ConfigError(
            "hardware launch requires flight_enabled: true in a reviewed local safety file"
        )
    return fleet, safety


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate Crazyfly ROS configuration without accessing hardware."
    )
    parser.add_argument(
        "--fleet", required=True, help="Crazyswarm2 crazyflies.yaml file"
    )
    parser.add_argument("--safety", required=True, help="Crazyfly safety.yaml file")
    parser.add_argument("--motion-capture", help="motion_capture.yaml file")
    parser.add_argument("--trajectory", help="optional synchronized waypoint YAML file")
    parser.add_argument(
        "--require-motive",
        action="store_true",
        help="reject placeholder Motive addresses",
    )
    args = parser.parse_args()
    try:
        fleet, safety = validate(
            args.fleet, args.safety, args.motion_capture, args.require_motive
        )
        if args.trajectory:
            load_trajectory(args.trajectory, fleet, safety)
        print(f"PASS: configuration is valid ({len(fleet.enabled)} enabled robot(s))")
        return 0
    except ConfigError as exc:
        print(f"FAILED: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
