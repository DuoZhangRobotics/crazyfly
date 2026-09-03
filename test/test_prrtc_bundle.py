import hashlib
import json
from math import pi
from pathlib import Path

import pytest

from crazyfly.config import ConfigError
from crazyfly.prrtc_bundle import (
    JOINT_NAMES,
    JointSample,
    JointTrajectory,
    load_execution_bundle,
    maximum_joint_speed,
    scale_trajectory_time,
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


def add_preposition(root: Path, *, schema_version: int = 1) -> Path:
    preposition_path = root / "ur5e_preposition_trajectory.json"
    payload = _joint_payload(
        [(0.0, [0.2] * 6), (1.0, [0.1] * 6), (2.0, [0.0] * 6)]
    )
    payload["schema_version"] = schema_version
    if schema_version == 3:
        payload.pop("frame")
    _write(preposition_path, payload)
    validation_path = root / "validation.json"
    validation = json.loads(validation_path.read_text())
    validation["arm_preposition"] = {"collision_free": True}
    _write(validation_path, validation)
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["files"]["ur5e_preposition_trajectory.json"] = {
        "sha256": hashlib.sha256(preposition_path.read_bytes()).hexdigest()
    }
    manifest["files"]["validation.json"]["sha256"] = hashlib.sha256(
        validation_path.read_bytes()
    ).hexdigest()
    _write(manifest_path, manifest)
    return root


def write_continuous_bundle(root: Path) -> Path:
    root = write_bundle(root)
    main_path = root / "ur5e_trajectory.json"
    main = _joint_payload(
        [
            (0.0, [0.0] * 6),
            (2.0, [0.2] * 6),
            (4.0, [0.4] * 6),
        ]
    )
    main["schema_version"] = 77
    _write(main_path, main)

    park_path = root / "ur5e_park_trajectory.json"
    park = _joint_payload([(4.0, [0.4] * 6), (6.0, [0.0] * 6)])
    park["schema_version"] = 88
    park.pop("frame")
    _write(park_path, park)

    complete_path = root / "ur5e_complete_trajectory.json"
    complete = _joint_payload(
        [
            (0.0, [0.0] * 6),
            (2.0, [0.2] * 6),
            (4.0, [0.4] * 6),
            (6.0, [0.0] * 6),
        ]
    )
    complete["schema_version"] = 99
    _write(complete_path, complete)

    preposition_path = root / "ur5e_preposition_trajectory.json"
    _write(
        preposition_path,
        {
            "schema_version": 123,
            "required": False,
            "duration_s": 0.0,
            "joint_names": list(JOINT_NAMES),
            "samples": [{"time_s": 0.0, "positions_rad": [0.0] * 6}],
        },
    )

    payload_path = root / "crazyfly_trajectory_payload.json"
    payload = json.loads(payload_path.read_text())
    payload["duration_s"] = 8.0
    payload["trajectories"]["cf1"]["pieces"][0]["duration_s"] = 8.0
    _write(payload_path, payload)

    validation_path = root / "validation.json"
    _write(
        validation_path,
        {
            "offline_export_eligible": True,
            "drone_brake_applied": False,
            "complete_robot_mission": {"collision_free": True},
            "return_home": {"collision_free": True},
            "home_hold_through_drone_completion": {"collision_free": True},
            "robot_boundaries": {"passed": True},
            "arm_motion_limits": {"passed": True},
        },
    )

    names = (
        "crazyfly_trajectory_payload.json",
        "ur5e_trajectory.json",
        "ur5e_park_trajectory.json",
        "ur5e_complete_trajectory.json",
        "ur5e_preposition_trajectory.json",
        "validation.json",
    )
    hashes = {
        name: {"sha256": hashlib.sha256((root / name).read_bytes()).hexdigest()}
        for name in names
    }
    _write(
        root / "manifest.json",
        {
            "schema_version": 456,
            "bundle_id": "continuous-bundle",
            "mission_mode": "continuous_ordered_stop",
            "drone_brake_applied": False,
            "goal_arrival_times_s": [4.0],
            "artifact_hashes": hashes,
            "frames": {
                "payload_to_planner": {
                    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                    "translation_m": [0.0, 0.0, 0.0],
                }
            },
            "sources": {
                "scene": {"snapshot_artifact": "planner_scene.json"},
            },
            "name_mapping": {"role": "cf1"},
            "pRRTC": {"planner": {"commit": "abc"}},
        },
    )
    return root


def test_bundle_loads_and_interpolates_exact_joint_path(tmp_path: Path) -> None:
    bundle = load_execution_bundle(write_bundle(tmp_path / "bundle"))

    assert bundle.robot_names == ("cf1",)
    assert bundle.main.evaluate(1.0) == pytest.approx((0.1,) * 6)
    assert bundle.main.evaluate(5.0) == pytest.approx((0.4,) * 6)
    assert bundle.park.duration_s == 2.0


def test_bundle_applies_physical_first_joint_offset(tmp_path: Path) -> None:
    bundle = load_execution_bundle(
        write_bundle(tmp_path / "bundle"),
        first_joint_offset_rad=pi / 2.0,
    )

    assert bundle.main.start[0] == pytest.approx(pi / 2.0)
    assert bundle.main.end[0] == pytest.approx(0.4 + pi / 2.0)
    assert bundle.park.start[0] == pytest.approx(0.4 + pi / 2.0)
    assert bundle.manifest["physical_first_joint_offset_rad"] == pytest.approx(
        pi / 2.0
    )


@pytest.mark.parametrize("schema_version", [1, 3])
def test_bundle_loads_validated_home_first_preposition(
    tmp_path: Path, schema_version: int
) -> None:
    root = add_preposition(
        write_bundle(tmp_path / "bundle"), schema_version=schema_version
    )

    bundle = load_execution_bundle(root, first_joint_offset_rad=pi / 2.0)

    assert bundle.preposition is not None
    assert bundle.preposition.start[0] == pytest.approx(0.2 + pi / 2.0)
    assert bundle.preposition.end == pytest.approx(bundle.main.start)
    assert bundle.preposition.start == pytest.approx(bundle.park.end)


def test_preposition_requires_hash_and_endpoint_continuity(tmp_path: Path) -> None:
    root = write_bundle(tmp_path / "missing_hash")
    _write(
        root / "ur5e_preposition_trajectory.json",
        _joint_payload([(0.0, [0.2] * 6), (1.0, [0.0] * 6)]),
    )
    with pytest.raises(ConfigError, match="both ur5e_preposition"):
        load_execution_bundle(root)

    root = add_preposition(write_bundle(tmp_path / "bad_endpoint"))
    preposition_path = root / "ur5e_preposition_trajectory.json"
    payload = json.loads(preposition_path.read_text())
    payload["samples"][-1]["positions_rad"] = [0.05] * 6
    _write(preposition_path, payload)
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["files"]["ur5e_preposition_trajectory.json"]["sha256"] = (
        hashlib.sha256(preposition_path.read_bytes()).hexdigest()
    )
    _write(manifest_path, manifest)
    with pytest.raises(ConfigError, match="end at the main-path start"):
        load_execution_bundle(root)


def test_joint_trajectory_timescale_reduces_speed() -> None:
    trajectory = JointTrajectory(
        "scaled",
        (
            JointSample(0.0, (0.0,) * 6),
            JointSample(1.0, (1.5,) * 6),
        ),
    )

    scaled = scale_trajectory_time(trajectory, 3.0)

    assert scaled.duration_s == 3.0
    assert maximum_joint_speed(trajectory) == pytest.approx(1.5)
    assert maximum_joint_speed(scaled) == pytest.approx(0.5)


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


@pytest.mark.parametrize(
    ("goal_key", "park_key", "return_key"),
    [
        (
            "goal_hold_through_cf1_brake",
            "park_path_with_cf1_held",
            "cf1_return_after_park",
        ),
        (
            "goal_hold_through_drone_brake",
            "park_path_with_drones_held",
            "drone_return_after_park",
        ),
    ],
)
def test_prrtc_v2_bundle_normalizes_names_and_absolute_park_time(
    tmp_path: Path,
    goal_key: str,
    park_key: str,
    return_key: str,
) -> None:
    root = write_bundle(tmp_path / "bundle")
    park_path = root / "ur5e_park_trajectory.json"
    park = json.loads(park_path.read_text())
    park["schema_version"] = 2
    park.pop("frame")
    for sample in park["samples"]:
        sample["time_s"] += 4.0
    _write(park_path, park)

    validation_path = root / "validation.json"
    _write(validation_path, {
        "offline_export_eligible": True,
        goal_key: {"collision_free": True},
        park_key: {"collision_free": True},
        return_key: {"collision_free": True},
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


def test_continuous_bundle_uses_structure_not_schema_number(
    tmp_path: Path,
) -> None:
    bundle = load_execution_bundle(
        write_continuous_bundle(tmp_path / "continuous"),
        accept_missing_physical_evidence=True,
    )

    assert bundle.return_during_drone_motion is True
    assert bundle.preposition is None
    assert bundle.main.duration_s == pytest.approx(4.0)
    assert bundle.park.duration_s == pytest.approx(2.0)
    assert bundle.park.start == pytest.approx(bundle.main.end)
    assert bundle.park.end == pytest.approx(bundle.main.start)
    assert bundle.manifest["source_schema_version"] == 456
    assert bundle.manifest["name_mapping"] == {"cf1": "role"}


def test_continuous_bundle_rejects_braked_drone_payload(tmp_path: Path) -> None:
    root = write_continuous_bundle(tmp_path / "continuous")
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["drone_brake_applied"] = True
    _write(manifest_path, manifest)

    with pytest.raises(ConfigError, match="unbraked drone motion"):
        load_execution_bundle(root, accept_missing_physical_evidence=True)


def test_continuous_bundle_requires_authoritative_split_match(
    tmp_path: Path,
) -> None:
    root = write_continuous_bundle(tmp_path / "continuous")
    complete_path = root / "ur5e_complete_trajectory.json"
    complete = json.loads(complete_path.read_text())
    complete["samples"][-1]["positions_rad"][0] = 0.1
    _write(complete_path, complete)
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["artifact_hashes"]["ur5e_complete_trajectory.json"]["sha256"] = (
        hashlib.sha256(complete_path.read_bytes()).hexdigest()
    )
    _write(manifest_path, manifest)

    with pytest.raises(ConfigError, match="authoritative complete"):
        load_execution_bundle(root, accept_missing_physical_evidence=True)


def test_return_home_has_no_fixed_duration_gate(tmp_path: Path) -> None:
    root = write_bundle(tmp_path / "bundle")
    park_path = root / "ur5e_park_trajectory.json"
    _write(
        park_path,
        _joint_payload([(0.0, [0.4] * 6), (20.0, [0.2] * 6)]),
    )
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["files"]["ur5e_park_trajectory.json"]["sha256"] = hashlib.sha256(
        park_path.read_bytes()
    ).hexdigest()
    _write(manifest_path, manifest)

    bundle = load_execution_bundle(root)

    assert bundle.park.duration_s == pytest.approx(20.0)


def test_continuous_return_must_finish_before_drone_endpoint(
    tmp_path: Path,
) -> None:
    root = write_continuous_bundle(tmp_path / "continuous")
    payload_path = root / "crazyfly_trajectory_payload.json"
    payload = json.loads(payload_path.read_text())
    payload["duration_s"] = 5.0
    payload["trajectories"]["cf1"]["pieces"][0]["duration_s"] = 5.0
    _write(payload_path, payload)
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["artifact_hashes"]["crazyfly_trajectory_payload.json"]["sha256"] = (
        hashlib.sha256(payload_path.read_bytes()).hexdigest()
    )
    _write(manifest_path, manifest)

    with pytest.raises(ConfigError, match="arm must reach home"):
        load_execution_bundle(root, accept_missing_physical_evidence=True)
