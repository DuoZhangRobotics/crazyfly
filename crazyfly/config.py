"""Configuration loading and validation with no ROS runtime dependency."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from pathlib import Path
import re
from typing import Any, Mapping, Sequence

import yaml


RADIO_URI = re.compile(
    r"^radio://(?P<radio>\d+)/(?P<channel>\d+)/"
    r"(?P<rate>250K|1M|2M)/(?P<address>[0-9A-Fa-f]{10})$"
)
ROBOT_NAME = re.compile(r"^cf\d+$")


class ConfigError(ValueError):
    """Raised when a configuration is unsafe or structurally invalid."""


@dataclass(frozen=True)
class RobotConfig:
    name: str
    enabled: bool
    uri: str
    initial_position: tuple[float, float, float]
    robot_type: str


@dataclass(frozen=True)
class FleetConfig:
    robots: dict[str, RobotConfig]
    fileversion: int

    @property
    def enabled(self) -> dict[str, RobotConfig]:
        return {name: robot for name, robot in self.robots.items() if robot.enabled}


@dataclass(frozen=True)
class SafetyConfig:
    flight_enabled: bool
    pose_reject_age_s: float
    pose_land_age_s: float
    pose_emergency_age_s: float
    status_stale_age_s: float
    recovery_time_s: float
    battery_warning_v: float
    battery_critical_v: float
    maximum_takeoff_height_m: float
    maximum_command_speed_m_s: float
    minimum_separation_m: float
    soft_geofence_margin_m: float
    geofence_min: tuple[float, float, float] | None
    geofence_max: tuple[float, float, float] | None

    @property
    def has_geofence(self) -> bool:
        return self.geofence_min is not None and self.geofence_max is not None


def load_yaml(path: str | Path) -> dict[str, Any]:
    config_path = Path(path).expanduser().resolve()
    if not config_path.is_file():
        raise ConfigError(f"configuration file does not exist: {config_path}")
    data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ConfigError(f"configuration must contain a YAML mapping: {config_path}")
    return data


def _finite_number(value: Any, field: str) -> float:
    if isinstance(value, bool):
        raise ConfigError(f"{field} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{field} must be a finite number") from exc
    if not isfinite(result):
        raise ConfigError(f"{field} must be a finite number")
    return result


def _vector3(value: Any, field: str) -> tuple[float, float, float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 3:
        raise ConfigError(f"{field} must contain exactly three finite numbers")
    result = tuple(_finite_number(component, field) for component in value)
    return result  # type: ignore[return-value]


def _validate_robot_type(robot_types: Mapping[Any, Any], name: str) -> None:
    raw_type = robot_types[name]
    if not isinstance(raw_type, Mapping):
        raise ConfigError(f"robot type {name!r} must be a mapping")
    motion_capture = raw_type.get("motion_capture")
    if not isinstance(motion_capture, Mapping):
        raise ConfigError(f"robot type {name!r} must define motion_capture")
    tracking = motion_capture.get("tracking")
    if tracking not in {"vendor", "librigidbodytracker"}:
        raise ConfigError(f"robot type {name!r} has invalid motion_capture.tracking")
    if tracking == "librigidbodytracker":
        for field in ("marker", "dynamics"):
            if not isinstance(motion_capture.get(field), str) or not motion_capture[field]:
                raise ConfigError(f"robot type {name!r} requires motion_capture.{field}")


def load_fleet(path: str | Path) -> FleetConfig:
    data = load_yaml(path)
    fileversion = data.get("fileversion")
    if fileversion != 3:
        raise ConfigError("crazyflies.yaml must use Crazyswarm2 fileversion 3")
    raw_robots = data.get("robots")
    robot_types = data.get("robot_types")
    if not isinstance(raw_robots, Mapping) or not raw_robots:
        raise ConfigError("crazyflies.yaml must define at least one robot")
    if not isinstance(robot_types, Mapping):
        raise ConfigError("crazyflies.yaml must define robot_types")

    robots: dict[str, RobotConfig] = {}
    seen_uris: set[str] = set()
    validated_types: set[str] = set()
    for name, raw in raw_robots.items():
        if not isinstance(name, str) or not ROBOT_NAME.fullmatch(name):
            raise ConfigError(f"invalid robot name {name!r}; expected cf followed by digits")
        if not isinstance(raw, Mapping):
            raise ConfigError(f"robot {name} must be a mapping")
        uri = str(raw.get("uri", ""))
        uri_match = RADIO_URI.fullmatch(uri)
        if uri_match is None:
            raise ConfigError(f"robot {name} has invalid radio URI: {uri!r}")
        if int(uri_match.group("channel")) > 125:
            raise ConfigError(f"robot {name} radio channel must be between 0 and 125")
        if uri in seen_uris:
            raise ConfigError(f"radio URI is duplicated: {uri}")
        seen_uris.add(uri)

        robot_type = str(raw.get("type", ""))
        if robot_type not in robot_types:
            raise ConfigError(f"robot {name} references unknown type {robot_type!r}")
        if robot_type not in validated_types:
            _validate_robot_type(robot_types, robot_type)
            validated_types.add(robot_type)

        enabled = raw.get("enabled", False)
        if not isinstance(enabled, bool):
            raise ConfigError(f"robots.{name}.enabled must be true or false")
        if enabled and uri_match.group("rate") != "2M":
            raise ConfigError(f"enabled robot {name} must use a 2M radio URI")
        robots[name] = RobotConfig(
            name=name,
            enabled=enabled,
            uri=uri,
            initial_position=_vector3(
                raw.get("initial_position"), f"robots.{name}.initial_position"
            ),
            robot_type=robot_type,
        )
    return FleetConfig(robots=robots, fileversion=fileversion)


def load_safety(path: str | Path) -> SafetyConfig:
    root = load_yaml(path)
    data = root.get("crazyfly_safety")
    if not isinstance(data, Mapping):
        raise ConfigError("safety configuration must define crazyfly_safety")
    timeouts = data.get("timeouts", {})
    battery = data.get("battery", {})
    limits = data.get("limits", {})
    fence = data.get("geofence", {})
    for name, value in (
        ("timeouts", timeouts),
        ("battery", battery),
        ("limits", limits),
        ("geofence", fence),
    ):
        if not isinstance(value, Mapping):
            raise ConfigError(f"crazyfly_safety.{name} must be a mapping")

    flight_enabled = data.get("flight_enabled", False)
    if not isinstance(flight_enabled, bool):
        raise ConfigError("crazyfly_safety.flight_enabled must be true or false")

    fence_min = fence.get("min")
    fence_max = fence.get("max")
    geofence_min = None if fence_min is None else _vector3(fence_min, "geofence.min")
    geofence_max = None if fence_max is None else _vector3(fence_max, "geofence.max")
    if (geofence_min is None) != (geofence_max is None):
        raise ConfigError("geofence min and max must either both be set or both be omitted")
    if geofence_min is not None and any(
        low >= high for low, high in zip(geofence_min, geofence_max)
    ):
        raise ConfigError("every geofence minimum must be less than its maximum")

    config = SafetyConfig(
        flight_enabled=flight_enabled,
        pose_reject_age_s=_finite_number(
            timeouts.get("pose_reject_s", 0.100), "timeouts.pose_reject_s"
        ),
        pose_land_age_s=_finite_number(
            timeouts.get("pose_land_s", 0.250), "timeouts.pose_land_s"
        ),
        pose_emergency_age_s=_finite_number(
            timeouts.get("pose_emergency_s", 1.000), "timeouts.pose_emergency_s"
        ),
        status_stale_age_s=_finite_number(
            timeouts.get("status_stale_s", 2.500), "timeouts.status_stale_s"
        ),
        recovery_time_s=_finite_number(
            timeouts.get("recovery_s", 1.000), "timeouts.recovery_s"
        ),
        battery_warning_v=_finite_number(
            battery.get("warning_v", 3.8), "battery.warning_v"
        ),
        battery_critical_v=_finite_number(
            battery.get("critical_v", 3.7), "battery.critical_v"
        ),
        maximum_takeoff_height_m=_finite_number(
            limits.get("maximum_takeoff_height_m", 0.5),
            "limits.maximum_takeoff_height_m",
        ),
        maximum_command_speed_m_s=_finite_number(
            limits.get("maximum_command_speed_m_s", 0.25),
            "limits.maximum_command_speed_m_s",
        ),
        minimum_separation_m=_finite_number(
            limits.get("minimum_separation_m", 0.4),
            "limits.minimum_separation_m",
        ),
        soft_geofence_margin_m=_finite_number(
            limits.get("soft_geofence_margin_m", 0.15),
            "limits.soft_geofence_margin_m",
        ),
        geofence_min=geofence_min,
        geofence_max=geofence_max,
    )
    if not 0 < config.pose_reject_age_s < config.pose_land_age_s < config.pose_emergency_age_s:
        raise ConfigError("pose timeouts must satisfy reject < land < emergency")
    if not 0 < config.battery_critical_v <= config.battery_warning_v:
        raise ConfigError("battery thresholds must satisfy 0 < critical <= warning")
    if min(
        config.status_stale_age_s,
        config.recovery_time_s,
        config.maximum_takeoff_height_m,
        config.maximum_command_speed_m_s,
        config.minimum_separation_m,
    ) <= 0:
        raise ConfigError("safety timeouts and limits must be positive")
    if config.soft_geofence_margin_m < 0:
        raise ConfigError("soft geofence margin must be non-negative")
    if config.flight_enabled and not config.has_geofence:
        raise ConfigError("flight_enabled requires explicit geofence bounds")
    if config.has_geofence:
        assert config.geofence_min is not None and config.geofence_max is not None
        if any(
            2 * config.soft_geofence_margin_m >= high - low
            for low, high in zip(config.geofence_min, config.geofence_max)
        ):
            raise ConfigError("soft geofence margin leaves no usable flight volume")
    return config


def point_inside_geofence(
    point: Sequence[float], safety: SafetyConfig, margin: float = 0.0
) -> bool:
    if not safety.has_geofence:
        return False
    assert safety.geofence_min is not None and safety.geofence_max is not None
    return all(
        low + margin <= value <= high - margin
        for value, low, high in zip(point, safety.geofence_min, safety.geofence_max)
    )
