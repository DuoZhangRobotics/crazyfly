"""Rigid transform loading for frame-aware safety geofences."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from math import isfinite
from pathlib import Path
from typing import Any

import yaml


class GeofenceTransformError(ValueError):
    pass


Matrix4 = tuple[
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
]


def _matrix4(value: Any, field: str) -> Matrix4:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or len(value) != 4
    ):
        raise GeofenceTransformError(f"{field} must be a finite 4x4 matrix")
    rows: list[tuple[float, float, float, float]] = []
    for row in value:
        if (
            not isinstance(row, Sequence)
            or isinstance(row, (str, bytes))
            or len(row) != 4
        ):
            raise GeofenceTransformError(f"{field} must be a finite 4x4 matrix")
        try:
            converted = tuple(float(item) for item in row)
        except (TypeError, ValueError) as exc:
            raise GeofenceTransformError(
                f"{field} must be a finite 4x4 matrix"
            ) from exc
        if not all(isfinite(item) for item in converted):
            raise GeofenceTransformError(f"{field} must be a finite 4x4 matrix")
        rows.append(converted)  # type: ignore[arg-type]
    matrix = tuple(rows)
    if any(
        abs(actual - expected) > 1e-8
        for actual, expected in zip(matrix[3], (0, 0, 0, 1))
    ):
        raise GeofenceTransformError(
            f"{field} must have homogeneous bottom row [0, 0, 0, 1]"
        )
    rotation = [row[:3] for row in matrix[:3]]
    for first in range(3):
        for second in range(3):
            dot = sum(rotation[row][first] * rotation[row][second] for row in range(3))
            expected = 1.0 if first == second else 0.0
            if abs(dot - expected) > 1e-5:
                raise GeofenceTransformError(f"{field} rotation must be orthonormal")
    determinant = (
        rotation[0][0]
        * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1]
        * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2]
        * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )
    if abs(determinant - 1.0) > 1e-5:
        raise GeofenceTransformError(f"{field} rotation must have determinant +1")
    return matrix  # type: ignore[return-value]


def load_base_from_world(path: str | Path) -> Matrix4:
    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise GeofenceTransformError(
            f"geofence transform file does not exist: {source}"
        )
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(raw, Mapping) or raw.get("schema_version") != 1:
        raise GeofenceTransformError(
            "geofence transform file must use schema_version 1"
        )
    if raw.get("accepted") is not True:
        raise GeofenceTransformError(
            "geofence transform requires an accepted calibration"
        )
    frames = raw.get("frames")
    transforms = raw.get("transforms")
    if not isinstance(frames, Mapping) or not isinstance(transforms, Mapping):
        raise GeofenceTransformError(
            "geofence transform file must define frames and transforms"
        )
    if frames.get("base") != "base" or frames.get("mocap") != "world":
        raise GeofenceTransformError(
            "geofence transform frames must map Motive world into UR base"
        )
    return _matrix4(transforms.get("base_from_mocap"), "base_from_mocap")


def transform_point(
    matrix: Matrix4, point: Sequence[float]
) -> tuple[float, float, float]:
    if len(point) != 3:
        raise GeofenceTransformError("point must contain three values")
    values = tuple(float(value) for value in point)
    if not all(isfinite(value) for value in values):
        raise GeofenceTransformError("point must contain three finite values")
    return tuple(
        sum(matrix[row][column] * values[column] for column in range(3))
        + matrix[row][3]
        for row in range(3)
    )  # type: ignore[return-value]
