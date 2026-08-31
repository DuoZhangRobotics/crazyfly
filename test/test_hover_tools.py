import json
import signal
import sys
import logging
from dataclasses import replace
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import crazyradio_guard  # noqa: E402
import all_drone_hover  # noqa: E402
import fleet_battery  # noqa: E402
import one_drone_hover  # noqa: E402
import payload_mission  # noqa: E402
import trajectory_mission  # noqa: E402

from crazyfly.config import ConfigError, load_fleet, load_safety  # noqa: E402
from crazyfly.trajectory import (  # noqa: E402
    evaluate_plan,
    load_trajectory,
    trajectory_payload,
)


def test_crazyradio_holder_parsing_is_pid_only() -> None:
    assert crazyradio_guard.parse_fuser_pids(" 1234  5678c\n") == {1234, 5678}


def test_only_known_lab_processes_are_eligible_for_release() -> None:
    assert crazyradio_guard.is_known_crazyflie_process(
        "/opt/ros/jazzy/lib/crazyflie/crazyflie_server --ros-args"
    )
    assert crazyradio_guard.is_known_crazyflie_process("python -m cfclient.gui")
    assert not crazyradio_guard.is_known_crazyflie_process("python important_job.py")


@pytest.mark.parametrize(
    ("voltage", "expected"),
    [
        (4.1, "READY"),
        (3.6, "LOW"),
        (3.2, "LOW"),
        (3.0, "CRITICAL"),
        (float("nan"), "INVALID"),
    ],
)
def test_fleet_battery_classification(voltage: float, expected: str) -> None:
    assert fleet_battery.classify_voltage(voltage, 3.6, 3.0) == expected


def test_fleet_battery_table_preserves_offline_drones() -> None:
    readings = [
        fleet_battery.BatteryReading("cf1", "radio://0/80/2M/E7E7E7E701", 4.1),
        fleet_battery.BatteryReading(
            "cf2", "radio://0/80/2M/E7E7E7E702", error="connection timed out"
        ),
    ]

    table = fleet_battery.render_table(readings, 3.6, 3.0)

    assert "cf1" in table and "4.10 V" in table and "READY" in table
    assert "cf2" in table and "OFFLINE" in table and "connection timed out" in table


def test_fleet_battery_hides_only_stale_log_entry_warnings() -> None:
    message_filter = fleet_battery.StaleLogEntryFilter()

    stale = logging.LogRecord(
        "cflib.crazyflie.log", logging.WARNING, "", 0,
        "Error no LogEntry to handle id=%d", (1,), None
    )
    useful = logging.LogRecord(
        "cflib.crazyflie.log", logging.WARNING, "", 0,
        "radio disconnected", (), None
    )

    assert not message_filter.filter(stale)
    assert message_filter.filter(useful)


def test_trajectory_mission_is_a_hardware_free_dry_run_by_default(capsys) -> None:
    result = trajectory_mission.main(
        [str(ROOT / "config" / "trajectories" / "mock_square.yaml"), "--mock"]
    )

    assert result == 0
    output = capsys.readouterr().out
    assert "continuously valid" in output
    assert "DRY RUN" in output


def test_legacy_trajectory_option_is_still_accepted(capsys) -> None:
    result = trajectory_mission.main(
        [
            "--trajectory",
            str(ROOT / "config" / "trajectories" / "mock_square.yaml"),
            "--mock",
        ]
    )

    assert result == 0
    assert "mock_square" in capsys.readouterr().out


def _mock_compiled_payload(tmp_path: Path) -> Path:
    fleet = load_fleet(ROOT / "config" / "mock_crazyflies.yaml")
    safety = load_safety(ROOT / "config" / "mock_safety.yaml")
    plan = load_trajectory(
        ROOT / "config" / "trajectories" / "mock_square.yaml",
        fleet,
        safety,
    )
    path = tmp_path / "compiled.json"
    path.write_text(
        json.dumps(
            trajectory_payload(plan, mission_id="prrtc-seed-1", trajectory_id=9)
        ),
        encoding="utf-8",
    )
    return path


def test_compiled_payload_is_a_hardware_free_dry_run_by_default(
    tmp_path: Path, capsys, monkeypatch
) -> None:
    path = _mock_compiled_payload(tmp_path)
    monkeypatch.setattr(
        trajectory_mission.subprocess,
        "Popen",
        lambda *_args, **_kwargs: pytest.fail("dry run started a process"),
    )

    result = trajectory_mission.main(
        ["--compiled-payload", str(path), "--mock"]
    )

    assert result == 0
    output = capsys.readouterr().out
    assert "prrtc-seed-1" in output
    assert "DRY RUN" in output


def test_compiled_payload_rejects_conflicting_sources(
    tmp_path: Path, capsys
) -> None:
    path = _mock_compiled_payload(tmp_path)

    result = trajectory_mission.main(
        [
            str(ROOT / "config" / "trajectories" / "mock_square.yaml"),
            "--compiled-payload",
            str(path),
            "--mock",
        ]
    )

    assert result == 2
    assert "provide exactly one" in capsys.readouterr().err


def test_compiled_payload_owns_trajectory_id(tmp_path: Path, capsys) -> None:
    path = _mock_compiled_payload(tmp_path)

    result = trajectory_mission.main(
        [
            "--compiled-payload",
            str(path),
            "--trajectory-id",
            "3",
            "--mock",
        ]
    )

    assert result == 2
    assert "cannot be used" in capsys.readouterr().err


def test_compiled_payload_rejects_invalid_json(tmp_path: Path, capsys) -> None:
    path = tmp_path / "compiled.json"
    path.write_text("{broken", encoding="utf-8")

    result = trajectory_mission.main(
        ["--compiled-payload", str(path), "--mock"]
    )

    assert result == 2
    assert "not valid JSON" in capsys.readouterr().err


def test_compiled_payload_rejects_normalization_changes(tmp_path: Path) -> None:
    path = _mock_compiled_payload(tmp_path)
    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["unreviewed_metadata"] = True
    path.write_text(json.dumps(payload), encoding="utf-8")
    fleet = load_fleet(ROOT / "config" / "mock_crazyflies.yaml")
    safety = load_safety(ROOT / "config" / "mock_safety.yaml")

    with pytest.raises(ConfigError, match="changes during normalization"):
        trajectory_mission._load_compiled_payload(
            path, tuple(fleet.enabled), safety
        )


def test_scheduled_start_accepts_one_to_ten_second_lead() -> None:
    start_ns = trajectory_mission.validate_scheduled_start(
        {"mission_id": "mission-1", "start_monotonic_ns": 3_000_000_000},
        mission_id="mission-1",
        mission_ready=True,
        already_scheduled=False,
        now_ns=1_000_000_000,
    )

    assert start_ns == 3_000_000_000


@pytest.mark.parametrize(
    ("payload", "ready", "scheduled", "message"),
    [
        (
            {"mission_id": "wrong", "start_monotonic_ns": 3_000_000_000},
            True,
            False,
            "does not match",
        ),
        (
            {"mission_id": "mission-1", "start_monotonic_ns": 3_000_000_000},
            False,
            False,
            "not ready",
        ),
        (
            {"mission_id": "mission-1", "start_monotonic_ns": 3_000_000_000},
            True,
            True,
            "already scheduled",
        ),
        (
            {"mission_id": "mission-1", "start_monotonic_ns": 1_500_000_000},
            True,
            False,
            "at least one second",
        ),
        (
            {"mission_id": "mission-1", "start_monotonic_ns": 12_000_000_000},
            True,
            False,
            "no more than ten seconds",
        ),
    ],
)
def test_scheduled_start_rejects_invalid_requests(
    payload, ready, scheduled, message
) -> None:
    with pytest.raises(ConfigError, match=message):
        trajectory_mission.validate_scheduled_start(
            payload,
            mission_id="mission-1",
            mission_ready=ready,
            already_scheduled=scheduled,
            now_ns=1_000_000_000,
        )


def test_combined_parser_exposes_endpoint_return_gate() -> None:
    args = trajectory_mission._parser().parse_args(
        ["trajectory.json", "--wait-for-start", "--wait-for-return"]
    )

    assert args.wait_for_start
    assert args.wait_for_return
    assert args.return_timeout_s == 10.0


def test_trajectory_mission_report_uses_timed_reference() -> None:
    fleet = load_fleet(ROOT / "config" / "mock_crazyflies.yaml")
    safety = load_safety(ROOT / "config" / "mock_safety.yaml")
    plan = load_trajectory(
        ROOT / "config" / "trajectories" / "mock_square.yaml", fleet, safety
    )
    started_at = 100.0
    samples = [
        (started_at + elapsed, evaluate_plan(plan, elapsed))
        for elapsed in (0.0, 4.0, 8.0, 12.0, 16.0)
    ]

    report = trajectory_mission.mission_report(
        plan, started_at, samples, {"cf1": 3.9}
    )

    assert report["tracking_error"]["cf1"]["maximum_m"] == pytest.approx(0.0)
    assert report["motion_start_skew_s"] == 0.0
    assert report["battery_minimum_v"] == {"cf1": 3.9}


def test_trajectory_mission_rejects_invalid_start_timeout(capsys) -> None:
    result = trajectory_mission.main(
        [
            str(ROOT / "config" / "trajectories" / "mock_square.yaml"),
            "--mock",
            "--start-timeout-s",
            "0",
        ]
    )

    assert result == 2
    assert "positive and finite" in capsys.readouterr().err


@pytest.mark.parametrize(
    ("drone_index", "expected_robot"), [("03", "cf3"), ("05", "cf5")]
)
def test_drone_index_selects_only_matching_fleet_entry(
    tmp_path: Path, monkeypatch, drone_index: str, expected_robot: str
) -> None:
    source = yaml.safe_load((ROOT / "config" / "local" / "crazyflies.yaml").read_text())
    fleet_path = tmp_path / "fleet.yaml"
    fleet_path.write_text(yaml.safe_dump(source, sort_keys=False))
    monkeypatch.setattr(one_drone_hover, "FLEET", fleet_path)

    def fake_hover_main(arguments):
        selected_path = Path(arguments[arguments.index("--fleet") + 1])
        selected = yaml.safe_load(selected_path.read_text())
        safety_path = Path(arguments[arguments.index("--safety") + 1])
        safety = yaml.safe_load(safety_path.read_text())
        enabled = {
            name for name, robot in selected["robots"].items() if robot["enabled"]
        }
        assert enabled == {expected_robot}
        assert (
            safety["crazyfly_safety"]["tracking"]["expected_raw_marker_count"]
            == 1
        )
        tracking = safety["crazyfly_safety"]["tracking"]
        assert tracking["maximum_pose_speed_m_s"] is None
        assert tracking["pose_identity_emergency_s"] == 1.0
        assert tracking["marker_count_grace_s"] == 1.0
        assert safety["crazyfly_safety"]["timeouts"]["recovery_s"] == 1.0
        topics = selected["all"]["firmware_logging"]["custom_topics"]
        assert topics["estimator_debug"]["frequency"] == 2
        assert topics["estimator_debug"]["vars"] == [
            "kalman.varPX",
            "kalman.varPY",
            "kalman.varPZ",
        ]
        return 0

    monkeypatch.setattr(all_drone_hover, "main", fake_hover_main)

    assert one_drone_hover.main([drone_index]) == 0


def test_drone_index_must_keep_leading_zero() -> None:
    with pytest.raises(SystemExit):
        one_drone_hover.main(["3"])


def test_payload_mission_builds_exact_selected_fleet(
    tmp_path: Path, monkeypatch
) -> None:
    payload = tmp_path / "payload.json"
    payload.write_text("{}", encoding="utf-8")
    captured = {}

    def fake_main(arguments):
        fleet_path = Path(arguments[arguments.index("--fleet") + 1])
        safety_path = Path(arguments[arguments.index("--safety") + 1])
        fleet = yaml.safe_load(fleet_path.read_text(encoding="utf-8"))
        safety = yaml.safe_load(safety_path.read_text(encoding="utf-8"))
        captured["enabled"] = {
            name for name, robot in fleet["robots"].items() if robot["enabled"]
        }
        captured["marker_count"] = safety["crazyfly_safety"]["tracking"][
            "expected_raw_marker_count"
        ]
        captured["flight_enabled"] = safety["crazyfly_safety"]["flight_enabled"]
        captured["payload"] = arguments[arguments.index("--compiled-payload") + 1]
        captured["execute"] = "--execute" in arguments
        return 0

    monkeypatch.setattr(payload_mission.trajectory_mission, "main", fake_main)

    assert payload_mission.main(
        [str(payload), "--robots", "cf1,cf2,cf3", "--execute"]
    ) == 0
    assert captured == {
        "enabled": {"cf1", "cf2", "cf3"},
        "marker_count": 3,
        "flight_enabled": False,
        "payload": str(payload),
        "execute": True,
    }


def test_payload_mission_enables_only_mock_execution(
    tmp_path: Path, monkeypatch
) -> None:
    payload = tmp_path / "payload.json"
    payload.write_text("{}", encoding="utf-8")
    enabled = []

    def fake_main(arguments):
        safety_path = Path(arguments[arguments.index("--safety") + 1])
        safety = yaml.safe_load(safety_path.read_text(encoding="utf-8"))
        enabled.append(safety["crazyfly_safety"]["flight_enabled"])
        return 0

    monkeypatch.setattr(payload_mission.trajectory_mission, "main", fake_main)

    assert payload_mission.main(
        [str(payload), "--robots", "cf1", "--mock", "--execute"]
    ) == 0
    assert enabled == [True]


def test_payload_mission_rejects_unknown_robot(tmp_path: Path) -> None:
    payload = tmp_path / "payload.json"
    payload.write_text("{}", encoding="utf-8")

    with pytest.raises(SystemExit, match="unknown robot IDs"):
        payload_mission.main(
            [str(payload), "--robots", "cf1,cf99"]
        )


def _five_drone_configuration(tmp_path: Path) -> tuple[Path, Path]:
    fleet = yaml.safe_load(
        (ROOT / "config" / "local" / "crazyflies.yaml").read_text()
    )
    for index, robot in enumerate(fleet["robots"].values()):
        robot["enabled"] = True
        robot["initial_position"] = [-0.8 + 0.4 * index, 0.0, 0.04]
    fleet_path = tmp_path / "five-drone-fleet.yaml"
    fleet_path.write_text(yaml.safe_dump(fleet, sort_keys=False))

    safety = yaml.safe_load(
        (ROOT / "config" / "local" / "safety.yaml").read_text()
    )
    safety["crazyfly_safety"]["tracking"]["expected_raw_marker_count"] = 5
    safety_path = tmp_path / "five-drone-safety.yaml"
    safety_path.write_text(yaml.safe_dump(safety, sort_keys=False))
    return fleet_path, safety_path


def test_selected_fleet_hover_accepts_five_drones(
    tmp_path: Path, monkeypatch, capsys
) -> None:
    fleet_path, safety_path = _five_drone_configuration(tmp_path)
    monkeypatch.setattr(
        all_drone_hover,
        "describe_status",
        lambda: (Path("/dev/bus/usb/001/004"), set(), {}),
    )

    result = all_drone_hover.main(
        ["--fleet", str(fleet_path), "--safety", str(safety_path)]
    )

    assert result == 0
    output = capsys.readouterr().out
    assert "cf5: radio://0/80/2M/E7E7E7E705" in output
    assert "Sequence: staged cf1, cf2, cf3, cf4, cf5" in output


def test_selected_fleet_hover_requires_matching_marker_count(
    tmp_path: Path, capsys
) -> None:
    fleet_path, safety_path = _five_drone_configuration(tmp_path)
    safety = yaml.safe_load(safety_path.read_text())
    safety["crazyfly_safety"]["tracking"]["expected_raw_marker_count"] = 4
    safety_path.write_text(yaml.safe_dump(safety, sort_keys=False))

    result = all_drone_hover.main(
        ["--fleet", str(fleet_path), "--safety", str(safety_path)]
    )

    assert result == 2
    assert "must match the enabled fleet size (5)" in capsys.readouterr().err


def test_all_drone_execution_defaults_to_staged_mode() -> None:
    args = all_drone_hover._parser().parse_args(["--execute"])
    assert args.execute
    assert not args.synchronized


def test_synchronized_mode_requires_an_explicit_flag() -> None:
    args = all_drone_hover._parser().parse_args(["--execute", "--synchronized"])
    assert args.execute
    assert args.synchronized


def test_five_drone_cycle_advances_once_and_returns_on_step_five() -> None:
    names = ("cf1", "cf2", "cf3", "cf4", "cf5")
    original = {
        name: (float(index), 0.0, 0.3)
        for index, name in enumerate(names)
    }

    first = all_drone_hover.cycle_goals(original, names, 1)
    fifth = all_drone_hover.cycle_goals(original, names, 5)

    assert first == {
        "cf1": original["cf2"],
        "cf2": original["cf3"],
        "cf3": original["cf4"],
        "cf4": original["cf5"],
        "cf5": original["cf1"],
    }
    assert fifth == original


def test_cycle_duration_uses_live_distance_with_speed_headroom() -> None:
    starts = {"cf1": (0.0, 0.0, 0.3), "cf2": (0.7, 0.0, 0.3)}
    goals = {"cf1": (0.7, 0.0, 0.3), "cf2": (0.0, 0.0, 0.3)}

    duration_s = all_drone_hover.coordinated_duration(
        starts,
        goals,
        maximum_speed_m_s=0.25,
    )

    assert duration_s == pytest.approx(3.5)
    assert 0.7 / duration_s == pytest.approx(0.20)


def test_staged_return_uses_separate_altitudes_then_anchors() -> None:
    safety = replace(
        load_safety(ROOT / "config" / "mock_safety.yaml"),
        minimum_separation_m=0.15,
        soft_geofence_margin_m=0.01,
        geofence_max=(2.0, 2.0, 1.5),
    )
    current = {
        f"cf{index}": (-0.8 + 0.3 * index, -0.5, 0.23)
        for index in range(1, 6)
    }
    anchors = {
        f"cf{index}": (-0.8 + 0.3 * index, -1.0, 0.30)
        for index in range(1, 6)
    }

    stages = trajectory_mission.staged_return_goals(
        current, anchors, safety
    )

    assert [label for label, _goals in stages] == [
        "separate-altitudes",
        "horizontal-to-anchors",
    ]
    assert [
        stages[0][1][f"cf{index}"][2] for index in range(1, 6)
    ] == pytest.approx([
        0.2, 0.4, 0.6, 0.8, 1.0,
    ])
    assert stages[1][1]["cf3"] == pytest.approx(
        (anchors["cf3"][0], anchors["cf3"][1], 0.6)
    )


def test_staged_return_rejects_anchors_inside_live_separation() -> None:
    safety = replace(
        load_safety(ROOT / "config" / "mock_safety.yaml"),
        minimum_separation_m=0.15,
    )
    current = {"cf1": (0.0, -0.5, 0.3), "cf2": (0.3, -0.5, 0.3)}
    anchors = {"cf1": (0.0, -1.0, 0.3), "cf2": (0.1, -1.0, 0.3)}

    with pytest.raises(ConfigError, match="captured anchors"):
        trajectory_mission.staged_return_goals(current, anchors, safety)


def test_touchdown_speed_is_recoverable_between_staged_flights() -> None:
    assert all(
        "implausible pose speed" not in reason
        for reason in all_drone_hover.HARD_ENABLE_FAILURES
    )


def test_missing_startup_telemetry_is_retryable() -> None:
    error = RuntimeError(
        "startup telemetry unavailable: missing status for cf3; "
        "missing battery for cf3"
    )
    assert all_drone_hover._is_retryable_telemetry_startup_failure(error)
    estimator_error = RuntimeError(
        "startup telemetry unavailable: estimator variance received only "
        "0 of 10 samples"
    )
    assert all_drone_hover._is_retryable_telemetry_startup_failure(
        estimator_error
    )


def test_locked_drone_is_not_automatically_restarted() -> None:
    error = RuntimeError("gateway refused enable: locked: cf3")
    assert not all_drone_hover._is_retryable_telemetry_startup_failure(error)


def test_estimator_variance_requires_ten_stable_samples() -> None:
    stable = [(0.0005, 0.0006, 0.0007)] * 10
    assert not all_drone_hover._estimator_variance_stable(stable[:9])
    assert all_drone_hover._estimator_variance_stable(stable)


def test_estimator_variance_rejects_range_and_non_finite_values() -> None:
    unstable = [(0.0005, 0.0006, 0.0007)] * 9 + [(0.002, 0.0006, 0.0007)]
    non_finite = [(0.0005, 0.0006, 0.0007)] * 9 + [
        (float("nan"), 0.0006, 0.0007)
    ]
    assert not all_drone_hover._estimator_variance_stable(unstable)
    assert not all_drone_hover._estimator_variance_stable(non_finite)


def test_cleanup_temporarily_ignores_repeated_termination_signals() -> None:
    previous_int = signal.getsignal(signal.SIGINT)
    previous_term = signal.getsignal(signal.SIGTERM)
    with all_drone_hover._ignore_cleanup_interrupts():
        assert signal.getsignal(signal.SIGINT) is signal.SIG_IGN
        assert signal.getsignal(signal.SIGTERM) is signal.SIG_IGN
    assert signal.getsignal(signal.SIGINT) is previous_int
    assert signal.getsignal(signal.SIGTERM) is previous_term


def test_stop_launch_checks_radio_even_if_launch_parent_already_exited(
    monkeypatch,
) -> None:
    calls = []

    class FakeProcess:
        def poll(self):
            return 0

    monkeypatch.setattr(
        all_drone_hover,
        "ensure_crazyradio_free",
        lambda: calls.append("checked"),
    )
    all_drone_hover._stop_launch(FakeProcess())
    assert calls == ["checked"]
