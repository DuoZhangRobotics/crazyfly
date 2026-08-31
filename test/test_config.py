from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from crazyfly.config import ConfigError, load_fleet, load_safety
from crazyfly.config_validator import (
    load_server_parameters,
    prepare_motion_capture_parameters,
    validate,
    validate_hardware_configuration,
    validate_motion_capture,
)


ROOT = Path(__file__).resolve().parents[1]


def _mock_safety_source():
    source = yaml.safe_load((ROOT / "config" / "mock_safety.yaml").read_text())
    source["crazyfly_safety"]["geofence"]["transform_file"] = str(
        ROOT / "config" / "mock_base_from_world.yaml"
    )
    return source


def test_hardware_defaults_cannot_fly() -> None:
    fleet = load_fleet(ROOT / "config" / "crazyflies.yaml")
    safety = load_safety(ROOT / "config" / "safety.yaml")

    assert not fleet.enabled
    assert not safety.flight_enabled
    assert not safety.has_geofence


def test_mock_configuration_is_explicitly_bounded() -> None:
    fleet, safety = validate(
        str(ROOT / "config" / "mock_crazyflies.yaml"),
        str(ROOT / "config" / "mock_safety.yaml"),
    )

    assert set(fleet.enabled) == {"cf1"}
    assert safety.flight_enabled
    assert safety.geofence_min == (-2.0, -2.0, 0.0)
    assert safety.geofence_max == (2.0, 2.0, 1.0)


def test_server_warning_range_accepts_nominal_120_hz_mocap() -> None:
    parameters = load_server_parameters(str(ROOT / "config" / "server.yaml"))

    assert parameters["warnings"]["motion_capture"][
        "warning_if_rate_outside"
    ] == [80.0, 180.0]


def test_invalid_server_warning_range_is_rejected(tmp_path: Path) -> None:
    server = yaml.safe_load((ROOT / "config" / "server.yaml").read_text())
    server["/crazyflie_server"]["ros__parameters"]["warnings"]["motion_capture"][
        "warning_if_rate_outside"
    ] = [180.0, 80.0]
    path = tmp_path / "server.yaml"
    path.write_text(yaml.safe_dump(server))

    with pytest.raises(ConfigError, match="positive and increasing"):
        load_server_parameters(str(path))


def test_real_motive_address_is_required_for_hardware() -> None:
    with pytest.raises(ConfigError, match="real Motive"):
        validate(
            str(ROOT / "config" / "crazyflies.yaml"),
            str(ROOT / "config" / "safety.yaml"),
            str(ROOT / "config" / "motion_capture.yaml"),
            require_motive=True,
        )


def test_hardware_configuration_requires_explicit_flight_enable(
    tmp_path: Path,
) -> None:
    motion = yaml.safe_load((ROOT / "config" / "motion_capture.yaml").read_text())
    motion["/motion_capture_tracking"]["ros__parameters"]["hostname"] = "192.0.2.1"
    motion_path = tmp_path / "motion.yaml"
    motion_path.write_text(yaml.safe_dump(motion))

    with pytest.raises(ConfigError, match="flight_enabled"):
        validate_hardware_configuration(
            str(ROOT / "config" / "crazyflies.yaml"),
            str(ROOT / "config" / "safety.yaml"),
            str(motion_path),
        )


def test_hardware_configuration_requires_raw_marker_count(
    tmp_path: Path,
) -> None:
    motion = yaml.safe_load((ROOT / "config" / "motion_capture.yaml").read_text())
    motion["/motion_capture_tracking"]["ros__parameters"]["hostname"] = "192.0.2.1"
    motion_path = tmp_path / "motion.yaml"
    motion_path.write_text(yaml.safe_dump(motion))

    with pytest.raises(ConfigError, match="expected_raw_marker_count"):
        validate_hardware_configuration(
            str(ROOT / "config" / "mock_crazyflies.yaml"),
            str(ROOT / "config" / "mock_safety.yaml"),
            str(motion_path),
        )


def test_official_optitrack_backend_is_supported(tmp_path: Path) -> None:
    motion = yaml.safe_load((ROOT / "config" / "motion_capture.yaml").read_text())
    parameters = motion["/motion_capture_tracking"]["ros__parameters"]
    parameters["type"] = "optitrack_closed_source"
    parameters["hostname"] = "192.0.2.1"
    motion_path = tmp_path / "motion.yaml"
    motion_path.write_text(yaml.safe_dump(motion))

    validated = validate_motion_capture(str(motion_path), require_motive=True)

    assert validated["type"] == "optitrack_closed_source"


def test_single_marker_tracking_is_prepared_for_mocap_only(tmp_path: Path) -> None:
    fleet = yaml.safe_load((ROOT / "config" / "crazyflies.yaml").read_text())
    fleet["robots"]["cf1"]["enabled"] = True
    fleet["robots"]["cf1"]["initial_position"] = [1.0, 2.0, 0.1]
    fleet_path = tmp_path / "fleet.yaml"
    fleet_path.write_text(yaml.safe_dump(fleet))

    parameters = prepare_motion_capture_parameters(
        str(ROOT / "config" / "motion_capture.yaml"),
        str(fleet_path),
    )

    assert parameters["rigid_bodies"] == {
        "cf1": {
            "initial_position": [1.0, 2.0, 0.1],
            "marker": "default_single_marker",
            "dynamics": "default",
        }
    }


def test_unknown_single_marker_configuration_is_rejected(tmp_path: Path) -> None:
    fleet = yaml.safe_load((ROOT / "config" / "crazyflies.yaml").read_text())
    fleet["robots"]["cf1"]["enabled"] = True
    fleet["robot_types"]["cf21"]["motion_capture"]["marker"] = "missing"
    fleet_path = tmp_path / "fleet.yaml"
    fleet_path.write_text(yaml.safe_dump(fleet))

    with pytest.raises(ConfigError, match="unknown marker configuration"):
        prepare_motion_capture_parameters(
            str(ROOT / "config" / "motion_capture.yaml"),
            str(fleet_path),
        )


def test_duplicate_radio_uri_is_rejected(tmp_path: Path) -> None:
    source = yaml.safe_load((ROOT / "config" / "crazyflies.yaml").read_text())
    bad = deepcopy(source)
    bad["robots"]["cf2"]["uri"] = bad["robots"]["cf1"]["uri"]
    path = tmp_path / "duplicate.yaml"
    path.write_text(yaml.safe_dump(bad))

    with pytest.raises(ConfigError, match="duplicated"):
        load_fleet(path)


def test_flight_enabled_without_geofence_is_rejected(tmp_path: Path) -> None:
    source = _mock_safety_source()
    source["crazyfly_safety"]["geofence"] = {}
    path = tmp_path / "unbounded.yaml"
    path.write_text(yaml.safe_dump(source))

    with pytest.raises(ConfigError, match="explicit geofence"):
        load_safety(path)


def test_quoted_enabled_boolean_is_rejected(tmp_path: Path) -> None:
    source = yaml.safe_load((ROOT / "config" / "mock_crazyflies.yaml").read_text())
    source["robots"]["cf1"]["enabled"] = "false"
    path = tmp_path / "quoted_boolean.yaml"
    path.write_text(yaml.safe_dump(source))

    with pytest.raises(ConfigError, match="true or false"):
        load_fleet(path)


def test_non_finite_safety_limit_is_rejected(tmp_path: Path) -> None:
    source = _mock_safety_source()
    source["crazyfly_safety"]["limits"]["maximum_command_speed_m_s"] = float("nan")
    path = tmp_path / "nan_limit.yaml"
    path.write_text(yaml.safe_dump(source))

    with pytest.raises(ConfigError, match="finite number"):
        load_safety(path)


def test_invalid_raw_marker_count_is_rejected(tmp_path: Path) -> None:
    source = _mock_safety_source()
    source["crazyfly_safety"]["tracking"]["expected_raw_marker_count"] = 0
    path = tmp_path / "bad_marker_count.yaml"
    path.write_text(yaml.safe_dump(source))

    with pytest.raises(ConfigError, match="positive integer"):
        load_safety(path)


def test_identity_timeout_cannot_precede_pose_rejection(tmp_path: Path) -> None:
    source = _mock_safety_source()
    source["crazyfly_safety"]["tracking"]["pose_identity_emergency_s"] = 0.05
    path = tmp_path / "bad_identity_timeout.yaml"
    path.write_text(yaml.safe_dump(source))

    with pytest.raises(ConfigError, match="between pose reject"):
        load_safety(path)


def test_invalid_pose_speed_action_is_rejected(tmp_path: Path) -> None:
    source = _mock_safety_source()
    source["crazyfly_safety"]["tracking"]["pose_speed_action"] = "ignore"
    path = tmp_path / "bad_pose_speed_action.yaml"
    path.write_text(yaml.safe_dump(source))

    with pytest.raises(ConfigError, match="land or emergency"):
        load_safety(path)


@pytest.mark.parametrize("window_s", [0.0, 0.1])
def test_pose_speed_window_must_be_shorter_than_pose_rejection(
    tmp_path: Path, window_s: float
) -> None:
    source = _mock_safety_source()
    source["crazyfly_safety"]["tracking"]["pose_speed_window_s"] = window_s
    path = tmp_path / "bad_pose_speed_window.yaml"
    path.write_text(yaml.safe_dump(source))

    with pytest.raises(ConfigError, match="pose_speed_window_s"):
        load_safety(path)


def test_trajectory_separation_cannot_be_below_live_limit(tmp_path: Path) -> None:
    source = _mock_safety_source()
    source["crazyfly_safety"]["limits"][
        "trajectory_minimum_separation_m"
    ] = 0.2
    path = tmp_path / "unsafe_trajectory_separation.yaml"
    path.write_text(yaml.safe_dump(source))

    with pytest.raises(ConfigError, match="must not be below"):
        load_safety(path)


def test_invalid_radio_channel_is_rejected(tmp_path: Path) -> None:
    source = yaml.safe_load((ROOT / "config" / "mock_crazyflies.yaml").read_text())
    source["robots"]["cf1"]["uri"] = "radio://0/126/2M/E7E7E7E701"
    path = tmp_path / "bad_channel.yaml"
    path.write_text(yaml.safe_dump(source))

    with pytest.raises(ConfigError, match="between 0 and 125"):
        load_fleet(path)
