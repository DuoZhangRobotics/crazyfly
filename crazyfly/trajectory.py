"""Declarative synchronized waypoint trajectory parsing and validation."""

from __future__ import annotations

from dataclasses import dataclass
from math import dist, isfinite
from pathlib import Path
from typing import Mapping

from .config import ConfigError, FleetConfig, SafetyConfig, load_yaml, point_inside_geofence
from .safety import minimum_pairwise_separation


@dataclass(frozen=True)
class Goal:
    position: tuple[float, float, float]
    yaw_deg: float


@dataclass(frozen=True)
class TrajectoryStep:
    duration_s: float
    goals: dict[str, Goal]


@dataclass(frozen=True)
class TrajectoryPlan:
    steps: tuple[TrajectoryStep, ...]


def load_trajectory(
    path: str | Path, fleet: FleetConfig, safety: SafetyConfig
) -> TrajectoryPlan:
    data = load_yaml(path)
    if data.get("version") != 1:
        raise ConfigError("trajectory must use version 1")
    raw_steps = data.get("steps")
    if not isinstance(raw_steps, list) or not raw_steps:
        raise ConfigError("trajectory must define a non-empty steps list")
    enabled = fleet.enabled
    if not enabled:
        raise ConfigError("trajectory validation requires at least one enabled robot")
    current = {name: robot.initial_position for name, robot in enabled.items()}
    steps: list[TrajectoryStep] = []
    for step_index, raw_step in enumerate(raw_steps):
        if not isinstance(raw_step, Mapping):
            raise ConfigError(f"trajectory step {step_index} must be a mapping")
        try:
            duration = float(raw_step.get("duration_s", 0.0))
        except (TypeError, ValueError) as exc:
            raise ConfigError(
                f"trajectory step {step_index} duration must be finite"
            ) from exc
        if not isfinite(duration) or duration <= 0:
            raise ConfigError(
                f"trajectory step {step_index} duration must be positive and finite"
            )
        raw_goals = raw_step.get("goals")
        if not isinstance(raw_goals, Mapping) or set(raw_goals) != set(enabled):
            raise ConfigError(
                f"trajectory step {step_index} must contain exactly the enabled robots"
            )
        goals: dict[str, Goal] = {}
        for name, raw_goal in raw_goals.items():
            if not isinstance(raw_goal, Mapping):
                raise ConfigError(
                    f"trajectory goal {step_index}/{name} must be a mapping"
                )
            raw_position = raw_goal.get("position")
            if not isinstance(raw_position, list) or len(raw_position) != 3:
                raise ConfigError(
                    f"trajectory goal {step_index}/{name} requires a 3D position"
                )
            try:
                position = tuple(float(value) for value in raw_position)
            except (TypeError, ValueError) as exc:
                raise ConfigError(
                    f"trajectory goal {step_index}/{name} requires finite numbers"
                ) from exc
            if not all(isfinite(value) for value in position):
                raise ConfigError(
                    f"trajectory goal {step_index}/{name} requires finite numbers"
                )
            if not point_inside_geofence(
                position, safety, margin=safety.soft_geofence_margin_m
            ):
                raise ConfigError(
                    f"trajectory goal {step_index}/{name} violates the soft geofence"
                )
            if dist(current[name], position) / duration > safety.maximum_command_speed_m_s:
                raise ConfigError(
                    f"trajectory goal {step_index}/{name} exceeds the speed limit"
                )
            try:
                yaw_deg = float(raw_goal.get("yaw_deg", 0.0))
            except (TypeError, ValueError) as exc:
                raise ConfigError(
                    f"trajectory goal {step_index}/{name} yaw must be finite"
                ) from exc
            if not isfinite(yaw_deg):
                raise ConfigError(
                    f"trajectory goal {step_index}/{name} yaw must be finite"
                )
            goals[name] = Goal(position=position, yaw_deg=yaw_deg)
        if (
            minimum_pairwise_separation(goal.position for goal in goals.values())
            < safety.minimum_separation_m
        ):
            raise ConfigError(
                f"trajectory step {step_index} violates minimum separation"
            )
        current = {name: goal.position for name, goal in goals.items()}
        steps.append(TrajectoryStep(duration_s=duration, goals=goals))
    return TrajectoryPlan(steps=tuple(steps))
