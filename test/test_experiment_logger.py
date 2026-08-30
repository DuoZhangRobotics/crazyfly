from dataclasses import replace
from pathlib import Path

import pytest
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from crazyfly.config import load_safety
from crazyfly.experiment_logger import raw_marker_event_data


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
