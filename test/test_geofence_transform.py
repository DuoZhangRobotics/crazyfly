from pathlib import Path

import pytest
import yaml

from crazyfly.config import (
    ConfigError,
    load_safety,
    point_from_geofence_frame,
    point_in_geofence_frame,
)
from crazyfly.safety import SafetyMachine

ROOT = Path(__file__).resolve().parents[1]


def _write_transform(path: Path, *, accepted: bool = True) -> None:
    path.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "accepted": accepted,
                "frames": {"base": "base", "mocap": "world"},
                "transforms": {
                    "base_from_mocap": [
                        [1.0, 0.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0, -1.0],
                        [0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0],
                    ]
                },
            }
        )
    )


def _write_safety(path: Path, transform_name: str) -> None:
    source = yaml.safe_load((ROOT / "config" / "mock_safety.yaml").read_text())
    source["crazyfly_safety"]["geofence"] = {
        "frame": "base",
        "transform_file": transform_name,
        "min": [-1.0, -2.0, 0.0],
        "max": [1.0, 0.0, 1.0],
    }
    path.write_text(yaml.safe_dump(source))


def test_base_geofence_transforms_world_pose_before_checks(tmp_path: Path) -> None:
    transform_path = tmp_path / "transform.yaml"
    safety_path = tmp_path / "safety.yaml"
    _write_transform(transform_path)
    _write_safety(safety_path, transform_path.name)

    safety = load_safety(safety_path)

    assert safety.geofence_frame == "base"
    assert point_in_geofence_frame((0.25, 0.0, 0.5), safety) == pytest.approx(
        (0.25, -1.0, 0.5)
    )
    assert point_from_geofence_frame((0.25, -1.0, 0.5), safety) == pytest.approx(
        (0.25, 0.0, 0.5)
    )
    machine = SafetyMachine(safety, ("cf1",))
    assert machine._inside_live_soft_fence((0.25, 0.0, 0.5))
    assert not machine._inside_live_soft_fence((0.90, 0.0, 0.5))
    assert not machine._inside_live_soft_fence((0.25, 0.90, 0.5))


def test_unaccepted_base_transform_is_rejected(tmp_path: Path) -> None:
    transform_path = tmp_path / "transform.yaml"
    safety_path = tmp_path / "safety.yaml"
    _write_transform(transform_path, accepted=False)
    _write_safety(safety_path, transform_path.name)

    with pytest.raises(ConfigError, match="accepted"):
        load_safety(safety_path)


def test_world_geofence_rejects_transform_file(tmp_path: Path) -> None:
    transform_path = tmp_path / "transform.yaml"
    safety_path = tmp_path / "safety.yaml"
    _write_transform(transform_path)
    source = yaml.safe_load((ROOT / "config" / "mock_safety.yaml").read_text())
    source["crazyfly_safety"]["geofence"]["frame"] = "world"
    source["crazyfly_safety"]["geofence"]["transform_file"] = str(transform_path)
    safety_path.write_text(yaml.safe_dump(source))

    with pytest.raises(ConfigError, match="only valid"):
        load_safety(safety_path)
