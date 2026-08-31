import hashlib
import json
from pathlib import Path

import pytest

from crazyfly.config import ConfigError
from crazyfly.prrtc_bundle import (
    JOINT_NAMES,
    load_execution_bundle,
    validate_physical_metadata,
)


def _write(path: Path, value) -> None:
    path.write_text(json.dumps(value, sort_keys=True), encoding="utf-8")


def _joint_payload(samples):
    return {
        "schema_version": 1,
        "frame": "base",
        "joint_names": list(JOINT_NAMES),
        "samples": [
            {"time_s": time_s, "positions_rad": positions}
            for time_s, positions in samples
        ],
    }


def write_bundle(root: Path, *, dirty: bool = False) -> Path:
    root.mkdir()
    _write(
        root / "crazyfly_trajectory_payload.json",
        {
            "mission_id": "bundle-1",
            "trajectory_id": 1,
            "frame": "base",
            "duration_s": 4.0,
            "trajectories": {
                "cf1": {
                    "pieces": [
                        {
                            "duration_s": 4.0,
                            "poly_x": [0.0] * 8,
                            "poly_y": [-0.5] + [0.0] * 7,
                            "poly_z": [0.3] + [0.0] * 7,
                            "poly_yaw": [0.0] * 8,
                        }
                    ]
                }
            },
        },
    )
    _write(
        root / "ur5e_trajectory.json",
        _joint_payload(
            [
                (0.0, [0.0] * 6),
                (2.0, [0.2] * 6),
                (4.0, [0.4] * 6),
            ]
        ),
    )
    _write(
        root / "ur5e_park_trajectory.json",
        _joint_payload([(0.0, [0.4] * 6), (2.0, [0.2] * 6)]),
    )
    _write(
        root / "validation.json",
        {
            "offline_export_eligible": True,
            "post_goal": {"goal_hold": {"collision_free": True}},
            "arm_park": {"collision_free": True},
            "drone_return": {"collision_free": True},
            "abort_land_in_place": {"collision_free": True},
        },
    )
    files = {}
    for name in (
        "crazyfly_trajectory_payload.json",
        "ur5e_trajectory.json",
        "ur5e_park_trajectory.json",
        "validation.json",
    ):
        files[name] = {
            "sha256": hashlib.sha256((root / name).read_bytes()).hexdigest()
        }
    _write(
        root / "manifest.json",
        {
            "schema_version": 1,
            "bundle_id": "bundle-1",
            "T_goal_s": 4.0,
            "pRRTC": {"commit": "abc", "dirty": dirty},
            "files": files,
        },
    )
    return root


def test_bundle_loads_and_interpolates_exact_joint_path(tmp_path: Path) -> None:
    bundle = load_execution_bundle(write_bundle(tmp_path / "bundle"))

    assert bundle.robot_names == ("cf1",)
    assert bundle.main.evaluate(1.0) == pytest.approx((0.1,) * 6)
    assert bundle.main.evaluate(5.0) == pytest.approx((0.4,) * 6)
    assert bundle.park.duration_s == 2.0


def test_bundle_hash_mismatch_is_rejected(tmp_path: Path) -> None:
    root = write_bundle(tmp_path / "bundle")
    (root / "ur5e_trajectory.json").write_text("{}", encoding="utf-8")

    with pytest.raises(ConfigError, match="hash mismatch"):
        load_execution_bundle(root)


def test_physical_bundle_requires_clean_prrtc_state(tmp_path: Path) -> None:
    root = write_bundle(tmp_path / "bundle", dirty=True)

    with pytest.raises(ConfigError, match="clean pRRTC"):
        load_execution_bundle(root, require_clean=True)


def test_bundle_requires_park_and_abort_validations(tmp_path: Path) -> None:
    root = write_bundle(tmp_path / "bundle")
    validation = json.loads((root / "validation.json").read_text())
    validation["arm_park"]["collision_free"] = False
    _write(root / "validation.json", validation)
    manifest = json.loads((root / "manifest.json").read_text())
    manifest["files"]["validation.json"]["sha256"] = hashlib.sha256(
        (root / "validation.json").read_bytes()
    ).hexdigest()
    _write(root / "manifest.json", manifest)

    with pytest.raises(ConfigError, match="arm_park"):
        load_execution_bundle(root)


def test_bundle_rejects_joint_speed_violation(tmp_path: Path) -> None:
    root = write_bundle(tmp_path / "bundle")
    payload = _joint_payload([(0.0, [0.0] * 6), (0.1, [0.4] * 6)])
    _write(root / "ur5e_trajectory.json", payload)
    manifest = json.loads((root / "manifest.json").read_text())
    manifest["T_goal_s"] = 0.1
    manifest["files"]["ur5e_trajectory.json"]["sha256"] = hashlib.sha256(
        (root / "ur5e_trajectory.json").read_bytes()
    ).hexdigest()
    _write(root / "manifest.json", manifest)

    with pytest.raises(ConfigError, match="joint speed"):
        load_execution_bundle(root)


def test_requested_global_joint_limits_accept_circle_one_dynamics(
    tmp_path: Path,
) -> None:
    root = write_bundle(tmp_path / "bundle")
    payload = _joint_payload(
        [
            (0.0, [0.0] * 6),
            (0.5, [0.7] * 6),
            (1.0, [0.1] * 6),
        ]
    )
    _write(root / "ur5e_trajectory.json", payload)
    manifest = json.loads((root / "manifest.json").read_text())
    manifest["T_goal_s"] = 1.0
    manifest["files"]["ur5e_trajectory.json"]["sha256"] = hashlib.sha256(
        (root / "ur5e_trajectory.json").read_bytes()
    ).hexdigest()
    park_path = root / "ur5e_park_trajectory.json"
    _write(
        park_path,
        _joint_payload([(0.0, [0.1] * 6), (2.0, [0.0] * 6)]),
    )
    manifest["files"]["ur5e_park_trajectory.json"]["sha256"] = hashlib.sha256(
        park_path.read_bytes()
    ).hexdigest()
    _write(root / "manifest.json", manifest)

    bundle = load_execution_bundle(root)

    assert bundle.main.duration_s == 1.0


def test_prrtc_v2_bundle_normalizes_names_and_absolute_park_time(
    tmp_path: Path,
) -> None:
    root = write_bundle(tmp_path / "bundle")
    park_path = root / "ur5e_park_trajectory.json"
    park = json.loads(park_path.read_text())
    park.pop("frame")
    for sample in park["samples"]:
        sample["time_s"] += 4.0
    _write(park_path, park)

    validation_path = root / "validation.json"
    _write(validation_path, {
        "offline_export_eligible": True,
        "goal_hold_through_cf1_brake": {"collision_free": True},
        "park_path_with_cf1_held": {"collision_free": True},
        "cf1_return_after_park": {"collision_free": True},
        "vertical_land_abort_corridors": {"collision_free": True},
    })
    hashes = {
        name: {"sha256": hashlib.sha256((root / name).read_bytes()).hexdigest()}
        for name in (
            "crazyfly_trajectory_payload.json",
            "ur5e_trajectory.json",
            "ur5e_park_trajectory.json",
            "validation.json",
        )
    }
    _write(root / "manifest.json", {
        "schema_version": 2,
        "bundle_id": "bundle-v2",
        "artifact_hashes": hashes,
        "durations_s": {"main_arm": 4.0},
        "frames": {"payload_to_planner": {
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            "translation_m": [0.0, 0.0, 0.0],
        }},
        "sources": {
            "planner_scene": {"repository_path": "scene.json"},
            "planner_result": {"path": "result.json"},
        },
        "name_mapping": {"one": "cf1"},
        "pRRTC": {"planner": {"commit": "abc", "dirty": False}},
        "physical_execution_eligible": True,
    })

    bundle = load_execution_bundle(root, require_clean=True)

    assert bundle.bundle_id == "bundle-v2"
    assert bundle.park.duration_s == 2.0
    assert bundle.park.samples[0].time_s == 0.0
    assert bundle.manifest["name_mapping"] == {"cf1": "one"}


def test_physical_metadata_requires_measured_calibration(tmp_path: Path) -> None:
    bundle = load_execution_bundle(write_bundle(tmp_path / "bundle"))

    with pytest.raises(ConfigError, match="name_mapping"):
        validate_physical_metadata(bundle)
