from dataclasses import replace
import hashlib
import json
from pathlib import Path

import pytest
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from crazyfly.config import load_safety
from crazyfly.experiment_logger import (
    calibration_file_record,
    copy_trajectory_source,
    raw_marker_event_data,
    trajectory_record_from_command,
)


ROOT = Path(__file__).resolve().parents[1]
BASE_FROM_WORLD = (
    (1.0, 0.0, 0.0, 1.0),
    (0.0, 1.0, 0.0, -2.0),
    (0.0, 0.0, 1.0, 0.5),
    (0.0, 0.0, 0.0, 1.0),
)


def _base_safety():
    return replace(
        load_safety(ROOT / "config" / "mock_safety.yaml"),
        expected_raw_marker_count=4,
        geofence_frame="base",
        geofence_from_world=BASE_FROM_WORLD,
    )


def test_raw_marker_mismatch_records_every_position_in_ur_base() -> None:
    message = point_cloud2.create_cloud_xyz32(
        Header(frame_id="world"),
        [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
            (0.25, 0.50, 0.75),
        ],
    )

    data = raw_marker_event_data(message, _base_safety())

    assert data["source_frame"] == "world"
    assert data["frame"] == "base"
    assert data["count"] == 5
    assert data["valid_position_count"] == 5
    expected = [
        (1.0, -2.0, 0.5),
        (2.0, -2.0, 0.5),
        (1.0, -1.0, 0.5),
        (1.0, -2.0, 1.5),
        (1.25, -1.50, 1.25),
    ]
    for actual, target in zip(data["positions"], expected):
        assert actual == pytest.approx(target)


def test_expected_marker_count_omits_redundant_positions() -> None:
    message = point_cloud2.create_cloud_xyz32(
        Header(frame_id="world"),
        [(0.0, 0.0, 0.0)] * 4,
    )

    data = raw_marker_event_data(message, _base_safety())

    assert data == {
        "source_frame": "world",
        "frame": "base",
        "count": 4,
    }


def test_trajectory_source_is_copied_byte_for_byte(tmp_path: Path) -> None:
    source = tmp_path / "payload.json"
    content = b'{"mission_id":"exact","coefficient":-0.0}\n'
    source.write_bytes(content)
    digest = hashlib.sha256(content).hexdigest()
    command = {
        "command": "mission_plan",
        "status": "accepted",
        "details": json.dumps(
            {
                "trajectory": {
                    "kind": "compiled_payload",
                    "path": str(source),
                    "sha256": digest,
                    "mission_id": "exact",
                }
            }
        ),
    }
    run_directory = tmp_path / "run"
    run_directory.mkdir()

    record = trajectory_record_from_command(command)
    assert record is not None
    copied = copy_trajectory_source(run_directory, record)

    destination = run_directory / "trajectory_source.json"
    assert destination.read_bytes() == content
    assert copied["sha256"] == digest
    assert copied["copied_path"] == str(destination)


def test_trajectory_source_rejects_hash_mismatch(tmp_path: Path) -> None:
    source = tmp_path / "payload.json"
    source.write_text("{}", encoding="utf-8")
    run_directory = tmp_path / "run"
    run_directory.mkdir()

    with pytest.raises(RuntimeError, match="SHA-256"):
        copy_trajectory_source(
            run_directory,
            {"path": str(source), "sha256": "0" * 64},
        )


def test_calibration_file_is_hashed_from_relative_safety_path(
    tmp_path: Path,
) -> None:
    calibration = tmp_path / "calibration.yaml"
    calibration.write_text("accepted: true\n", encoding="utf-8")
    safety = tmp_path / "safety.yaml"
    safety.write_text(
        "crazyfly_safety:\n"
        "  geofence:\n"
        "    frame: base\n"
        "    transform_file: calibration.yaml\n",
        encoding="utf-8",
    )

    record = calibration_file_record(safety)

    assert record is not None
    assert record["path"] == str(calibration)
    assert record["sha256"] == hashlib.sha256(calibration.read_bytes()).hexdigest()
