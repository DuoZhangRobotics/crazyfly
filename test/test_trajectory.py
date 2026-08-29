from pathlib import Path

import pytest
import yaml

from crazyfly.config import ConfigError, load_fleet, load_safety
from crazyfly.trajectory import load_trajectory


ROOT = Path(__file__).resolve().parents[1]


def _inputs():
    return (
        load_fleet(ROOT / "config" / "mock_crazyflies.yaml"),
        load_safety(ROOT / "config" / "mock_safety.yaml"),
    )


def test_mock_square_is_valid() -> None:
    plan = load_trajectory(ROOT / "config" / "trajectories" / "mock_square.yaml", *_inputs())

    assert len(plan.steps) == 4
    assert all(set(step.goals) == {"cf1"} for step in plan.steps)


def test_trajectory_speed_limit_is_enforced(tmp_path: Path) -> None:
    source = yaml.safe_load((ROOT / "config" / "trajectories" / "mock_square.yaml").read_text())
    source["steps"][0]["duration_s"] = 0.01
    path = tmp_path / "fast.yaml"
    path.write_text(yaml.safe_dump(source))

    with pytest.raises(ConfigError, match="speed limit"):
        load_trajectory(path, *_inputs())


def test_trajectory_geofence_is_enforced(tmp_path: Path) -> None:
    source = yaml.safe_load((ROOT / "config" / "trajectories" / "mock_square.yaml").read_text())
    source["steps"][0]["goals"]["cf1"]["position"] = [3.0, 0.0, 0.3]
    source["steps"][0]["duration_s"] = 20.0
    path = tmp_path / "outside.yaml"
    path.write_text(yaml.safe_dump(source))

    with pytest.raises(ConfigError, match="geofence"):
        load_trajectory(path, *_inputs())


def test_trajectory_requires_every_enabled_robot(tmp_path: Path) -> None:
    source = yaml.safe_load((ROOT / "config" / "trajectories" / "mock_square.yaml").read_text())
    source["steps"][0]["goals"] = {}
    path = tmp_path / "missing.yaml"
    path.write_text(yaml.safe_dump(source))

    with pytest.raises(ConfigError, match="exactly the enabled robots"):
        load_trajectory(path, *_inputs())


def test_trajectory_soft_geofence_margin_is_enforced(tmp_path: Path) -> None:
    source = yaml.safe_load((ROOT / "config" / "trajectories" / "mock_square.yaml").read_text())
    source["steps"][0]["goals"]["cf1"]["position"] = [1.9, 0.0, 0.3]
    source["steps"][0]["duration_s"] = 20.0
    path = tmp_path / "soft_boundary.yaml"
    path.write_text(yaml.safe_dump(source))

    with pytest.raises(ConfigError, match="soft geofence"):
        load_trajectory(path, *_inputs())


def test_trajectory_non_finite_duration_is_rejected(tmp_path: Path) -> None:
    source = yaml.safe_load((ROOT / "config" / "trajectories" / "mock_square.yaml").read_text())
    source["steps"][0]["duration_s"] = float("nan")
    path = tmp_path / "nan.yaml"
    path.write_text(yaml.safe_dump(source))

    with pytest.raises(ConfigError, match="finite"):
        load_trajectory(path, *_inputs())
