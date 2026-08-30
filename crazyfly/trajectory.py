"""Version-2 timed waypoint compilation and continuous trajectory validation."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace
from itertools import combinations
from math import factorial, isfinite
from pathlib import Path
from typing import Any

import numpy as np

from .config import (
    ConfigError,
    FleetConfig,
    SafetyConfig,
    load_yaml,
    point_from_geofence_frame,
)


COEFFICIENT_COUNT = 8
NUMERICAL_TOLERANCE = 1e-7
DENSE_VALIDATION_STEP_S = 0.01


@dataclass(frozen=True)
class Goal:
    position: tuple[float, float, float]
    yaw_deg: float = 0.0


@dataclass(frozen=True)
class Waypoint:
    time_s: float
    goals: dict[str, Goal]


@dataclass(frozen=True)
class PolynomialPiece:
    duration_s: float
    poly_x: tuple[float, ...]
    poly_y: tuple[float, ...]
    poly_z: tuple[float, ...]
    poly_yaw: tuple[float, ...]


@dataclass(frozen=True)
class RobotTrajectory:
    pieces: tuple[PolynomialPiece, ...]


@dataclass(frozen=True)
class TrajectoryMetrics:
    maximum_speed_m_s: float
    maximum_acceleration_m_s2: float
    maximum_jerk_m_s3: float
    minimum_separation_m: float | None


@dataclass(frozen=True)
class TrajectoryPlan:
    name: str
    frame: str
    trajectories: dict[str, RobotTrajectory]
    duration_s: float
    waypoints: tuple[Waypoint, ...] = ()
    metrics: TrajectoryMetrics | None = None

    @property
    def start_positions(self) -> dict[str, tuple[float, float, float]]:
        return {
            name: evaluate_piece(trajectory.pieces[0], 0.0)
            for name, trajectory in self.trajectories.items()
        }

    @property
    def end_positions(self) -> dict[str, tuple[float, float, float]]:
        return {
            name: evaluate_piece(
                trajectory.pieces[-1], trajectory.pieces[-1].duration_s
            )
            for name, trajectory in self.trajectories.items()
        }


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
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or len(value) != 3
    ):
        raise ConfigError(f"{field} must contain three finite numbers")
    return tuple(_finite_number(item, field) for item in value)  # type: ignore[return-value]


def _derivative_basis(order: int, time_s: float) -> np.ndarray:
    result = np.zeros(COEFFICIENT_COUNT)
    for power in range(order, COEFFICIENT_COUNT):
        result[power] = (
            factorial(power)
            / factorial(power - order)
            * time_s ** (power - order)
        )
    return result


def _minimum_snap_coefficients(
    times_s: Sequence[float], values: Sequence[float]
) -> tuple[tuple[float, ...], ...]:
    segment_count = len(times_s) - 1
    variable_count = segment_count * COEFFICIENT_COUNT
    durations = [times_s[index + 1] - times_s[index] for index in range(segment_count)]
    cost = np.zeros((variable_count, variable_count))
    for segment, duration in enumerate(durations):
        offset = segment * COEFFICIENT_COUNT
        for first in range(4, COEFFICIENT_COUNT):
            for second in range(4, COEFFICIENT_COUNT):
                cost[offset + first, offset + second] = (
                    factorial(first)
                    / factorial(first - 4)
                    * factorial(second)
                    / factorial(second - 4)
                    * duration ** (first + second - 7)
                    / (first + second - 7)
                )

    rows: list[np.ndarray] = []
    targets: list[float] = []

    def constrain(segment: int, order: int, time_s: float, target: float) -> None:
        row = np.zeros(variable_count)
        offset = segment * COEFFICIENT_COUNT
        row[offset : offset + COEFFICIENT_COUNT] = _derivative_basis(order, time_s)
        rows.append(row)
        targets.append(target)

    for segment, duration in enumerate(durations):
        constrain(segment, 0, 0.0, values[segment])
        constrain(segment, 0, duration, values[segment + 1])
    for order in range(1, 4):
        constrain(0, order, 0.0, 0.0)
        constrain(segment_count - 1, order, durations[-1], 0.0)
    for segment in range(segment_count - 1):
        for order in range(1, 4):
            row = np.zeros(variable_count)
            first = segment * COEFFICIENT_COUNT
            second = (segment + 1) * COEFFICIENT_COUNT
            row[first : first + COEFFICIENT_COUNT] = _derivative_basis(
                order, durations[segment]
            )
            row[second : second + COEFFICIENT_COUNT] = -_derivative_basis(
                order, 0.0
            )
            rows.append(row)
            targets.append(0.0)

    constraints = np.vstack(rows)
    zeros = np.zeros((constraints.shape[0], constraints.shape[0]))
    system = np.block([[cost, constraints.T], [constraints, zeros]])
    right_hand_side = np.concatenate(
        [np.zeros(variable_count), np.asarray(targets, dtype=float)]
    )
    try:
        solution = np.linalg.solve(system, right_hand_side)[:variable_count]
    except np.linalg.LinAlgError as exc:
        raise ConfigError("minimum-snap trajectory constraints are degenerate") from exc
    return tuple(
        tuple(
            float(value)
            for value in solution[
                index * COEFFICIENT_COUNT : (index + 1) * COEFFICIENT_COUNT
            ]
        )
        for index in range(segment_count)
    )


def _compile_robot(
    times_s: Sequence[float], positions: Sequence[tuple[float, float, float]]
) -> RobotTrajectory:
    axes = [
        _minimum_snap_coefficients(times_s, [position[axis] for position in positions])
        for axis in range(3)
    ]
    pieces = []
    for index in range(len(times_s) - 1):
        pieces.append(
            PolynomialPiece(
                duration_s=times_s[index + 1] - times_s[index],
                poly_x=axes[0][index],
                poly_y=axes[1][index],
                poly_z=axes[2][index],
                poly_yaw=(0.0,) * COEFFICIENT_COUNT,
            )
        )
    return RobotTrajectory(pieces=tuple(pieces))


def load_trajectory(
    path: str | Path, fleet: FleetConfig, safety: SafetyConfig
) -> TrajectoryPlan:
    data = load_yaml(path)
    if data.get("version") != 2:
        raise ConfigError("trajectory must use version 2")
    name = data.get("name")
    if not isinstance(name, str) or not name.strip() or len(name) > 64:
        raise ConfigError("trajectory name must be a non-empty string up to 64 characters")
    if data.get("frame") != "base" or safety.geofence_frame != "base":
        raise ConfigError("trajectory and safety geofence must both use UR base frame")
    enabled = tuple(sorted(fleet.enabled))
    if not enabled:
        raise ConfigError("trajectory validation requires enabled robots")
    raw_waypoints = data.get("waypoints")
    if not isinstance(raw_waypoints, list) or len(raw_waypoints) < 2:
        raise ConfigError("trajectory must define at least two timed waypoints")

    waypoints = []
    previous_time = -1.0
    for index, raw_waypoint in enumerate(raw_waypoints):
        if not isinstance(raw_waypoint, Mapping):
            raise ConfigError(f"waypoint {index} must be a mapping")
        time_s = _finite_number(raw_waypoint.get("time_s"), f"waypoint {index} time_s")
        if index == 0 and abs(time_s) > NUMERICAL_TOLERANCE:
            raise ConfigError("first waypoint time_s must be zero")
        if time_s <= previous_time:
            raise ConfigError("waypoint times must be strictly increasing")
        previous_time = time_s
        raw_goals = raw_waypoint.get("goals")
        if not isinstance(raw_goals, Mapping) or set(raw_goals) != set(enabled):
            raise ConfigError(
                f"waypoint {index} must contain exactly the enabled robots"
            )
        goals = {}
        for robot in enabled:
            raw_goal = raw_goals[robot]
            if not isinstance(raw_goal, Mapping):
                raise ConfigError(f"waypoint {index}/{robot} must be a mapping")
            position = _vector3(
                raw_goal.get("position"), f"waypoint {index}/{robot} position"
            )
            yaw_deg = _finite_number(
                raw_goal.get("yaw_deg", 0.0), f"waypoint {index}/{robot} yaw_deg"
            )
            if abs(yaw_deg) > NUMERICAL_TOLERANCE:
                raise ConfigError("version-2 single-marker trajectories require yaw_deg 0")
            goals[robot] = Goal(position=position, yaw_deg=0.0)
        waypoints.append(Waypoint(time_s=time_s, goals=goals))

    if waypoints[-1].time_s > safety.maximum_trajectory_duration_s:
        raise ConfigError("trajectory exceeds the configured maximum duration")
    times_s = [waypoint.time_s for waypoint in waypoints]
    trajectories = {
        robot: _compile_robot(
            times_s, [waypoint.goals[robot].position for waypoint in waypoints]
        )
        for robot in enabled
    }
    plan = TrajectoryPlan(
        name=name.strip(),
        frame="base",
        trajectories=trajectories,
        duration_s=waypoints[-1].time_s,
        waypoints=tuple(waypoints),
    )
    return replace(plan, metrics=validate_compiled_trajectory(plan, safety))


def derivative_coefficients(
    coefficients: Sequence[float], order: int = 1
) -> tuple[float, ...]:
    result = np.asarray(coefficients, dtype=float)
    for _ in range(order):
        result = np.polynomial.polynomial.polyder(result)
    return tuple(float(value) for value in result)


def evaluate_coefficients(coefficients: Sequence[float], time_s: float) -> float:
    return float(np.polynomial.polynomial.polyval(time_s, coefficients))


def evaluate_piece(
    piece: PolynomialPiece, time_s: float, derivative: int = 0
) -> tuple[float, float, float]:
    clamped = min(max(float(time_s), 0.0), piece.duration_s)
    return tuple(
        evaluate_coefficients(derivative_coefficients(coefficients, derivative), clamped)
        for coefficients in (piece.poly_x, piece.poly_y, piece.poly_z)
    )  # type: ignore[return-value]


def evaluate_plan(
    plan: TrajectoryPlan, time_s: float
) -> dict[str, tuple[float, float, float]]:
    elapsed = min(max(float(time_s), 0.0), plan.duration_s)
    result = {}
    for robot, trajectory in plan.trajectories.items():
        remaining = elapsed
        selected = trajectory.pieces[-1]
        local_time = selected.duration_s
        for piece in trajectory.pieces:
            if remaining <= piece.duration_s:
                selected = piece
                local_time = remaining
                break
            remaining -= piece.duration_s
        result[robot] = evaluate_piece(selected, local_time)
    return result


def _real_roots(coefficients: Sequence[float], duration_s: float) -> list[float]:
    values = np.trim_zeros(np.asarray(coefficients, dtype=float), trim="b")
    if len(values) <= 1:
        return []
    result = []
    for root in np.polynomial.polynomial.polyroots(values):
        if abs(float(root.imag)) <= NUMERICAL_TOLERANCE:
            value = float(root.real)
            if -NUMERICAL_TOLERANCE <= value <= duration_s + NUMERICAL_TOLERANCE:
                result.append(min(max(value, 0.0), duration_s))
    return result


def _dense_times(duration_s: float) -> list[float]:
    count = max(2, int(np.ceil(duration_s / DENSE_VALIDATION_STEP_S)) + 1)
    return [float(value) for value in np.linspace(0.0, duration_s, count)]


def _sum_squared_polynomials(polynomials: Sequence[Sequence[float]]) -> np.ndarray:
    result = np.zeros(1)
    for polynomial in polynomials:
        result = np.polynomial.polynomial.polyadd(
            result, np.polynomial.polynomial.polymul(polynomial, polynomial)
        )
    return result


def _norm_extreme(
    polynomials: Sequence[Sequence[float]], duration_s: float, *, minimum: bool
) -> float:
    squared = _sum_squared_polynomials(polynomials)
    candidates = _dense_times(duration_s)
    candidates.extend(
        _real_roots(np.polynomial.polynomial.polyder(squared), duration_s)
    )
    values = [max(0.0, evaluate_coefficients(squared, time_s)) for time_s in candidates]
    selected = min(values) if minimum else max(values)
    return float(np.sqrt(selected))


def _validate_piece_shape(piece: PolynomialPiece, robot: str, index: int) -> None:
    if not isfinite(piece.duration_s) or piece.duration_s <= 0:
        raise ConfigError(f"trajectory {robot} piece {index} has invalid duration")
    for field, coefficients in (
        ("x", piece.poly_x),
        ("y", piece.poly_y),
        ("z", piece.poly_z),
        ("yaw", piece.poly_yaw),
    ):
        if len(coefficients) != COEFFICIENT_COUNT or not all(
            isfinite(value) for value in coefficients
        ):
            raise ConfigError(
                f"trajectory {robot} piece {index} {field} requires eight finite coefficients"
            )
    if any(abs(value) > NUMERICAL_TOLERANCE for value in piece.poly_yaw):
        raise ConfigError("version-2 single-marker trajectories require zero yaw")


def validate_compiled_trajectory(
    plan: TrajectoryPlan, safety: SafetyConfig
) -> TrajectoryMetrics:
    if plan.frame != "base" or safety.geofence_frame != "base":
        raise ConfigError("compiled trajectory must use UR base frame")
    if set(plan.trajectories) == set() or not safety.has_geofence:
        raise ConfigError("compiled trajectory requires robots and a configured geofence")
    piece_counts = {len(trajectory.pieces) for trajectory in plan.trajectories.values()}
    if len(piece_counts) != 1 or not piece_counts or next(iter(piece_counts)) < 1:
        raise ConfigError("every robot trajectory must have the same non-zero piece count")
    piece_count = next(iter(piece_counts))
    reference_durations = tuple(
        piece.duration_s for piece in next(iter(plan.trajectories.values())).pieces
    )
    for robot, trajectory in plan.trajectories.items():
        for index, piece in enumerate(trajectory.pieces):
            _validate_piece_shape(piece, robot, index)
            if abs(piece.duration_s - reference_durations[index]) > NUMERICAL_TOLERANCE:
                raise ConfigError("all robot trajectories must share piece durations")
    duration_s = sum(reference_durations)
    if duration_s > safety.maximum_trajectory_duration_s + NUMERICAL_TOLERANCE:
        raise ConfigError("trajectory exceeds the configured maximum duration")
    if abs(duration_s - plan.duration_s) > NUMERICAL_TOLERANCE:
        raise ConfigError("trajectory duration does not match its polynomial pieces")

    assert safety.geofence_min is not None and safety.geofence_max is not None
    lower = tuple(value + safety.soft_geofence_margin_m for value in safety.geofence_min)
    upper = tuple(value - safety.soft_geofence_margin_m for value in safety.geofence_max)
    maximum_speed = 0.0
    maximum_acceleration = 0.0
    maximum_jerk = 0.0
    minimum_separation: float | None = None

    for robot, trajectory in plan.trajectories.items():
        for index, piece in enumerate(trajectory.pieces):
            position_polynomials = (piece.poly_x, piece.poly_y, piece.poly_z)
            for axis, coefficients in enumerate(position_polynomials):
                candidates = _dense_times(piece.duration_s)
                candidates.extend(
                    _real_roots(derivative_coefficients(coefficients), piece.duration_s)
                )
                values = [evaluate_coefficients(coefficients, time_s) for time_s in candidates]
                if min(values) < lower[axis] - NUMERICAL_TOLERANCE or max(values) > upper[
                    axis
                ] + NUMERICAL_TOLERANCE:
                    raise ConfigError(
                        f"trajectory {robot} piece {index} violates the soft geofence"
                    )
            speed = _norm_extreme(
                [derivative_coefficients(values, 1) for values in position_polynomials],
                piece.duration_s,
                minimum=False,
            )
            acceleration = _norm_extreme(
                [derivative_coefficients(values, 2) for values in position_polynomials],
                piece.duration_s,
                minimum=False,
            )
            jerk = _norm_extreme(
                [derivative_coefficients(values, 3) for values in position_polynomials],
                piece.duration_s,
                minimum=False,
            )
            maximum_speed = max(maximum_speed, speed)
            maximum_acceleration = max(maximum_acceleration, acceleration)
            maximum_jerk = max(maximum_jerk, jerk)

    if maximum_speed > safety.maximum_command_speed_m_s + NUMERICAL_TOLERANCE:
        raise ConfigError(
            f"trajectory speed {maximum_speed:.3f} m/s exceeds the configured limit"
        )
    if (
        maximum_acceleration
        > safety.maximum_trajectory_acceleration_m_s2 + NUMERICAL_TOLERANCE
    ):
        raise ConfigError(
            f"trajectory acceleration {maximum_acceleration:.3f} m/s^2 exceeds the configured limit"
        )
    if maximum_jerk > safety.maximum_trajectory_jerk_m_s3 + NUMERICAL_TOLERANCE:
        raise ConfigError(
            f"trajectory jerk {maximum_jerk:.3f} m/s^3 exceeds the configured limit"
        )

    for first, second in combinations(sorted(plan.trajectories), 2):
        for index in range(piece_count):
            first_piece = plan.trajectories[first].pieces[index]
            second_piece = plan.trajectories[second].pieces[index]
            differences = [
                np.polynomial.polynomial.polysub(left, right)
                for left, right in zip(
                    (first_piece.poly_x, first_piece.poly_y, first_piece.poly_z),
                    (second_piece.poly_x, second_piece.poly_y, second_piece.poly_z),
                )
            ]
            separation = _norm_extreme(
                differences, first_piece.duration_s, minimum=True
            )
            minimum_separation = (
                separation
                if minimum_separation is None
                else min(minimum_separation, separation)
            )
    if (
        minimum_separation is not None
        and minimum_separation
        < safety.trajectory_minimum_separation_m - NUMERICAL_TOLERANCE
    ):
        raise ConfigError(
            f"trajectory separation {minimum_separation:.3f} m is below "
            "the configured planning minimum"
        )
    return TrajectoryMetrics(
        maximum_speed_m_s=maximum_speed,
        maximum_acceleration_m_s2=maximum_acceleration,
        maximum_jerk_m_s3=maximum_jerk,
        minimum_separation_m=minimum_separation,
    )


def trajectory_payload(
    plan: TrajectoryPlan, *, mission_id: str, trajectory_id: int
) -> dict[str, object]:
    if not isinstance(mission_id, str) or not mission_id or len(mission_id) > 64:
        raise ConfigError("mission_id must be a non-empty string up to 64 characters")
    if isinstance(trajectory_id, bool) or not 0 <= trajectory_id <= 255:
        raise ConfigError("trajectory_id must be an integer from 0 to 255")
    return {
        "mission_id": mission_id,
        "trajectory_id": trajectory_id,
        "frame": plan.frame,
        "duration_s": plan.duration_s,
        "trajectories": {
            robot: {
                "pieces": [
                    {
                        "duration_s": piece.duration_s,
                        "poly_x": list(piece.poly_x),
                        "poly_y": list(piece.poly_y),
                        "poly_z": list(piece.poly_z),
                        "poly_yaw": list(piece.poly_yaw),
                    }
                    for piece in trajectory.pieces
                ]
            }
            for robot, trajectory in plan.trajectories.items()
        },
    }


def trajectory_from_payload(
    payload: Mapping[str, object], robot_names: Sequence[str], safety: SafetyConfig
) -> tuple[str, int, TrajectoryPlan]:
    mission_id = payload.get("mission_id")
    if not isinstance(mission_id, str) or not mission_id or len(mission_id) > 64:
        raise ConfigError("mission_id must be a non-empty string up to 64 characters")
    trajectory_id = payload.get("trajectory_id")
    if (
        isinstance(trajectory_id, bool)
        or not isinstance(trajectory_id, int)
        or not 0 <= trajectory_id <= 255
    ):
        raise ConfigError("trajectory_id must be an integer from 0 to 255")
    if payload.get("frame") != "base":
        raise ConfigError("trajectory payload frame must be base")
    duration_s = _finite_number(payload.get("duration_s"), "duration_s")
    raw_trajectories = payload.get("trajectories")
    if not isinstance(raw_trajectories, Mapping) or set(raw_trajectories) != set(
        robot_names
    ):
        raise ConfigError("trajectory payload must contain exactly the enabled robots")
    trajectories = {}
    for robot in robot_names:
        raw_trajectory = raw_trajectories[robot]
        if not isinstance(raw_trajectory, Mapping):
            raise ConfigError(f"trajectory payload {robot} must be a mapping")
        raw_pieces = raw_trajectory.get("pieces")
        if not isinstance(raw_pieces, list) or not raw_pieces:
            raise ConfigError(f"trajectory payload {robot} requires polynomial pieces")
        pieces = []
        for index, raw_piece in enumerate(raw_pieces):
            if not isinstance(raw_piece, Mapping):
                raise ConfigError(f"trajectory payload {robot} piece {index} is invalid")
            pieces.append(
                PolynomialPiece(
                    duration_s=_finite_number(
                        raw_piece.get("duration_s"),
                        f"trajectory payload {robot} piece {index} duration",
                    ),
                    poly_x=tuple(
                        _finite_number(value, "poly_x")
                        for value in raw_piece.get("poly_x", [])
                    ),
                    poly_y=tuple(
                        _finite_number(value, "poly_y")
                        for value in raw_piece.get("poly_y", [])
                    ),
                    poly_z=tuple(
                        _finite_number(value, "poly_z")
                        for value in raw_piece.get("poly_z", [])
                    ),
                    poly_yaw=tuple(
                        _finite_number(value, "poly_yaw")
                        for value in raw_piece.get("poly_yaw", [])
                    ),
                )
            )
        trajectories[robot] = RobotTrajectory(pieces=tuple(pieces))
    plan = TrajectoryPlan(
        name=mission_id,
        frame="base",
        trajectories=trajectories,
        duration_s=duration_s,
    )
    return mission_id, trajectory_id, replace(
        plan, metrics=validate_compiled_trajectory(plan, safety)
    )


def trajectory_in_world(
    plan: TrajectoryPlan, safety: SafetyConfig
) -> dict[str, RobotTrajectory]:
    if safety.geofence_frame != "base" or safety.geofence_from_world is None:
        raise ConfigError("trajectory conversion requires an accepted base-from-world transform")
    rotation = np.asarray(
        [row[:3] for row in safety.geofence_from_world[:3]], dtype=float
    )
    result = {}
    for robot, trajectory in plan.trajectories.items():
        pieces = []
        for piece in trajectory.pieces:
            base_axes = (piece.poly_x, piece.poly_y, piece.poly_z)
            world_coefficients = [[], [], []]
            for power in range(COEFFICIENT_COUNT):
                vector = np.asarray([axis[power] for axis in base_axes], dtype=float)
                if power == 0:
                    transformed = np.asarray(
                        point_from_geofence_frame(vector, safety), dtype=float
                    )
                else:
                    transformed = rotation.T @ vector
                for axis in range(3):
                    world_coefficients[axis].append(float(transformed[axis]))
            pieces.append(
                PolynomialPiece(
                    duration_s=piece.duration_s,
                    poly_x=tuple(world_coefficients[0]),
                    poly_y=tuple(world_coefficients[1]),
                    poly_z=tuple(world_coefficients[2]),
                    poly_yaw=piece.poly_yaw,
                )
            )
        result[robot] = RobotTrajectory(pieces=tuple(pieces))
    return result
