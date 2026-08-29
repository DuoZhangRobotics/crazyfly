"""Pure safety state machine used by the ROS gateway and unit tests."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from math import dist, isfinite
from typing import Iterable, Sequence

from .config import SafetyConfig, point_inside_geofence


CAN_BE_ARMED = 1
IS_ARMED = 2
CAN_FLY = 8
IS_FLYING = 16
IS_TUMBLED = 32
IS_LOCKED = 64


class SafetyState(str, Enum):
    DISABLED = "DISABLED"
    PREFLIGHT = "PREFLIGHT"
    READY = "READY"
    FLYING = "FLYING"
    LANDING = "LANDING"
    EMERGENCY = "EMERGENCY"


class SafetyAction(str, Enum):
    NONE = "NONE"
    LAND = "LAND"
    EMERGENCY = "EMERGENCY"


@dataclass
class RobotHealth:
    pose_received_at: float | None = None
    status_received_at: float | None = None
    position: tuple[float, float, float] | None = None
    battery_voltage: float | None = None
    supervisor_info: int = 0


@dataclass(frozen=True)
class Evaluation:
    state: SafetyState
    action: SafetyAction
    reasons: tuple[str, ...]
    commands_allowed: bool


@dataclass
class SafetyMachine:
    config: SafetyConfig
    robot_names: tuple[str, ...]
    state: SafetyState = SafetyState.DISABLED
    health: dict[str, RobotHealth] = field(default_factory=dict)
    operator_enabled: bool = False
    healthy_since: float | None = None
    _land_requested: bool = False
    _takeoff_requested: set[str] = field(default_factory=set)

    def __post_init__(self) -> None:
        self.health = {name: RobotHealth() for name in self.robot_names}

    def record_pose(
        self, name: str, position: Sequence[float], received_at: float
    ) -> None:
        if name in self.health:
            values = tuple(float(value) for value in position)
            self.health[name].pose_received_at = received_at
            self.health[name].position = values  # type: ignore[assignment]

    def record_status(
        self, name: str, voltage: float, supervisor_info: int, received_at: float
    ) -> None:
        if name in self.health:
            item = self.health[name]
            item.status_received_at = received_at
            item.battery_voltage = float(voltage)
            item.supervisor_info = int(supervisor_info)

    def reset(self) -> None:
        self.operator_enabled = False
        self.state = SafetyState.DISABLED
        self.healthy_since = None
        self._land_requested = False
        self._takeoff_requested.clear()

    def preflight(self, now: float) -> tuple[bool, tuple[str, ...]]:
        reasons = self._health_reasons(now, preflight=True)
        if not self.config.flight_enabled:
            reasons.append("flight is disabled in safety configuration")
        if not self.config.has_geofence:
            reasons.append("geofence is not configured")
        if not self.robot_names:
            reasons.append("no robots are enabled")
        if (
            self.healthy_since is None
            or now - self.healthy_since < self.config.recovery_time_s
        ):
            reasons.append("healthy recovery interval has not completed")
        self.state = SafetyState.PREFLIGHT if reasons else SafetyState.READY
        return not reasons, tuple(reasons)

    def enable(self, now: float) -> tuple[bool, tuple[str, ...]]:
        success, reasons = self.preflight(now)
        self.operator_enabled = success
        if success:
            self.state = SafetyState.READY
            self._land_requested = False
            self._takeoff_requested.clear()
        return success, reasons

    def disable(self) -> None:
        self.operator_enabled = False
        self.state = SafetyState.DISABLED
        self._land_requested = False
        self._takeoff_requested.clear()

    def mark_flying(self, name: str | None = None) -> None:
        if self.operator_enabled and self.state in {
            SafetyState.READY,
            SafetyState.FLYING,
        }:
            if name is not None:
                self._takeoff_requested.add(name)
            self.state = SafetyState.FLYING

    def mark_landing(self) -> None:
        if self.state != SafetyState.EMERGENCY:
            self.state = SafetyState.LANDING

    def mark_emergency(self) -> None:
        self.operator_enabled = False
        self.state = SafetyState.EMERGENCY

    def evaluate(self, now: float) -> Evaluation:
        reasons = self._health_reasons(now, preflight=False)
        hard_faults = [
            reason
            for reason in reasons
            if reason.startswith(
                (
                    "tumbled",
                    "locked",
                    "cannot fly",
                    "invalid position",
                    "hard geofence",
                    "separation",
                )
            )
        ]
        if not reasons:
            if self.healthy_since is None:
                self.healthy_since = now
        else:
            self.healthy_since = None

        if hard_faults and self.state in {
            SafetyState.FLYING,
            SafetyState.LANDING,
            SafetyState.READY,
        }:
            self.mark_emergency()
            return Evaluation(
                self.state, SafetyAction.EMERGENCY, tuple(hard_faults), False
            )

        if not self.operator_enabled:
            return Evaluation(self.state, SafetyAction.NONE, tuple(reasons), False)

        maximum_pose_age = max(
            self._ages(now, "pose_received_at"), default=float("inf")
        )
        maximum_status_age = max(
            self._ages(now, "status_received_at"), default=float("inf")
        )

        if self.state in {SafetyState.FLYING, SafetyState.LANDING}:
            if maximum_pose_age >= self.config.pose_emergency_age_s:
                self.mark_emergency()
                return Evaluation(
                    self.state,
                    SafetyAction.EMERGENCY,
                    ("tracking lost beyond emergency timeout",),
                    False,
                )
            should_land = (
                maximum_pose_age >= self.config.pose_land_age_s
                or maximum_status_age >= self.config.status_stale_age_s
                or any(reason.startswith("soft geofence") for reason in reasons)
            )
            if should_land:
                self.state = SafetyState.LANDING
                if not self._land_requested:
                    self._land_requested = True
                    return Evaluation(
                        self.state, SafetyAction.LAND, tuple(reasons), False
                    )
            battery_fault = any(
                item.battery_voltage is None
                or not isfinite(item.battery_voltage)
                or item.battery_voltage <= self.config.battery_critical_v
                for item in self.health.values()
            )
            if battery_fault:
                self.state = SafetyState.LANDING
                if not self._land_requested:
                    self._land_requested = True
                    return Evaluation(
                        self.state, SafetyAction.LAND, ("critical battery",), False
                    )

        commands_allowed = (
            self.state in {SafetyState.READY, SafetyState.FLYING}
            and self.healthy_since is not None
            and now - self.healthy_since >= self.config.recovery_time_s
            and maximum_pose_age < self.config.pose_reject_age_s
            and maximum_status_age < self.config.status_stale_age_s
            and not reasons
        )
        return Evaluation(
            self.state, SafetyAction.NONE, tuple(reasons), commands_allowed
        )

    def validate_takeoff(
        self, name: str, height: float, duration_s: float
    ) -> tuple[bool, str]:
        if (
            self.state not in {SafetyState.READY, SafetyState.FLYING}
            or not self.operator_enabled
        ):
            return False, "safety gateway is not ready for takeoff"
        if name in self._takeoff_requested:
            return False, f"takeoff was already requested for {name}"
        if not isfinite(height) or not isfinite(duration_s):
            return False, "takeoff height and duration must be finite"
        current = self.health.get(name, RobotHealth()).position
        if current is None:
            return False, f"no current pose for {name}"
        target = (current[0], current[1], float(height))
        if not point_inside_geofence(
            target, self.config, margin=self.config.soft_geofence_margin_m
        ):
            return False, "takeoff target violates the soft geofence"
        if not 0 < height <= self.config.maximum_takeoff_height_m:
            return False, "takeoff height exceeds the configured limit"
        if (
            duration_s <= 0
            or abs(height - current[2]) / duration_s
            > self.config.maximum_command_speed_m_s
        ):
            return False, "takeoff command exceeds the configured speed limit"
        for other_name, other in self.health.items():
            if (
                other_name != name
                and other.position is not None
                and dist(target, other.position) < self.config.minimum_separation_m
            ):
                return False, f"takeoff target violates separation from {other_name}"
        return True, "accepted"

    def validate_goto(
        self,
        name: str,
        goal: Sequence[float],
        duration_s: float,
        relative: bool = False,
        yaw_deg: float = 0.0,
    ) -> tuple[bool, str, tuple[float, float, float] | None]:
        if self.state != SafetyState.FLYING or not self.operator_enabled:
            return False, "safety gateway is not FLYING", None
        try:
            requested_goal = tuple(float(value) for value in goal)
        except (TypeError, ValueError) as exc:
            return False, f"go_to goal is invalid: {exc}", None
        if (
            len(requested_goal) != 3
            or not isfinite(duration_s)
            or not isfinite(yaw_deg)
            or not all(isfinite(value) for value in requested_goal)
        ):
            return False, "go_to goal, yaw, and duration must be finite", None
        current = self.health.get(name, RobotHealth()).position
        if current is None:
            return False, f"no current pose for {name}", None
        target = tuple(
            current[index] + requested_goal[index]
            if relative
            else requested_goal[index]
            for index in range(3)
        )
        if not point_inside_geofence(
            target, self.config, margin=self.config.soft_geofence_margin_m
        ):
            return False, "goal violates the soft geofence", None
        if (
            duration_s <= 0
            or dist(current, target) / duration_s
            > self.config.maximum_command_speed_m_s
        ):
            return False, "go_to command exceeds the configured speed limit", None
        for other_name, other in self.health.items():
            if (
                other_name != name
                and other.position is not None
                and dist(target, other.position) < self.config.minimum_separation_m
            ):
                return False, f"goal violates separation from {other_name}", None
        return True, "accepted", target

    def _ages(self, now: float, attribute: str) -> list[float]:
        return [
            float("inf")
            if getattr(item, attribute) is None
            else now - getattr(item, attribute)
            for item in self.health.values()
        ]

    def _health_reasons(self, now: float, preflight: bool) -> list[str]:
        reasons: list[str] = []
        positions: list[tuple[str, tuple[float, float, float]]] = []
        for name, item in self.health.items():
            if item.pose_received_at is None:
                reasons.append(f"missing pose for {name}")
            elif now - item.pose_received_at >= self.config.pose_reject_age_s:
                reasons.append(f"stale pose for {name}")
            if item.status_received_at is None:
                reasons.append(f"missing status for {name}")
            elif now - item.status_received_at >= self.config.status_stale_age_s:
                reasons.append(f"stale status for {name}")

            if item.battery_voltage is None:
                reasons.append(f"missing battery for {name}")
            elif not isfinite(item.battery_voltage):
                reasons.append(f"invalid battery for {name}")
            elif item.battery_voltage <= self.config.battery_warning_v:
                reasons.append(f"battery below threshold for {name}")

            if item.supervisor_info & IS_TUMBLED:
                reasons.append(f"tumbled: {name}")
            if item.supervisor_info & IS_LOCKED:
                reasons.append(f"locked: {name}")
            if preflight and not item.supervisor_info & CAN_BE_ARMED:
                reasons.append(f"cannot be armed: {name}")
            if not item.supervisor_info & CAN_FLY:
                reasons.append(f"cannot fly: {name}")

            if item.position is not None:
                if not all(isfinite(value) for value in item.position):
                    reasons.append(f"invalid position: {name}")
                    continue
                positions.append((name, item.position))
                if self.config.has_geofence and not point_inside_geofence(
                    item.position, self.config
                ):
                    reasons.append(f"hard geofence breach: {name}")
                elif self.state == SafetyState.FLYING and not self._inside_live_soft_fence(
                    item.position
                ):
                    reasons.append(f"soft geofence margin: {name}")

        for index, (name, position) in enumerate(positions):
            for other_name, other_position in positions[index + 1 :]:
                if dist(position, other_position) < self.config.minimum_separation_m:
                    reasons.append(f"separation violation: {name}/{other_name}")
        return reasons

    def _inside_live_soft_fence(self, point: Sequence[float]) -> bool:
        if not self.config.has_geofence:
            return False
        assert self.config.geofence_min is not None
        assert self.config.geofence_max is not None
        margin = self.config.soft_geofence_margin_m
        return (
            self.config.geofence_min[0] + margin
            <= point[0]
            <= self.config.geofence_max[0] - margin
            and self.config.geofence_min[1] + margin
            <= point[1]
            <= self.config.geofence_max[1] - margin
            and self.config.geofence_min[2]
            <= point[2]
            <= self.config.geofence_max[2] - margin
        )


def minimum_pairwise_separation(points: Iterable[Sequence[float]]) -> float:
    values = [tuple(float(value) for value in point) for point in points]
    if len(values) < 2:
        return float("inf")
    return min(
        dist(first, second)
        for index, first in enumerate(values)
        for second in values[index + 1 :]
    )
