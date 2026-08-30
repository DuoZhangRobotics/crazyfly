import signal
import sys
import logging
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import crazyradio_guard  # noqa: E402
import fleet_battery  # noqa: E402
import four_drone_hover  # noqa: E402
import one_drone_hover  # noqa: E402
import trajectory_mission  # noqa: E402

from crazyfly.config import load_fleet, load_safety  # noqa: E402
from crazyfly.trajectory import evaluate_plan, load_trajectory  # noqa: E402


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


def test_drone_index_selects_only_matching_fleet_entry(
    tmp_path: Path, monkeypatch
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
        assert enabled == {"cf3"}
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
        assert four_drone_hover.EXPECTED_ROBOTS == ("cf3",)
        return 0

    monkeypatch.setattr(four_drone_hover, "main", fake_hover_main)

    assert one_drone_hover.main(["03"]) == 0


def test_drone_index_must_keep_leading_zero() -> None:
    with pytest.raises(SystemExit):
        one_drone_hover.main(["3"])


def test_four_drone_execution_defaults_to_staged_mode() -> None:
    args = four_drone_hover._parser().parse_args(["--execute"])
    assert args.execute
    assert not args.synchronized


def test_synchronized_mode_requires_an_explicit_flag() -> None:
    args = four_drone_hover._parser().parse_args(["--execute", "--synchronized"])
    assert args.execute
    assert args.synchronized


def test_touchdown_speed_is_recoverable_between_staged_flights() -> None:
    assert all(
        "implausible pose speed" not in reason
        for reason in four_drone_hover.HARD_ENABLE_FAILURES
    )


def test_missing_startup_telemetry_is_retryable() -> None:
    error = RuntimeError(
        "startup telemetry unavailable: missing status for cf3; "
        "missing battery for cf3"
    )
    assert four_drone_hover._is_retryable_telemetry_startup_failure(error)
    estimator_error = RuntimeError(
        "startup telemetry unavailable: estimator variance received only "
        "0 of 10 samples"
    )
    assert four_drone_hover._is_retryable_telemetry_startup_failure(
        estimator_error
    )


def test_locked_drone_is_not_automatically_restarted() -> None:
    error = RuntimeError("gateway refused enable: locked: cf3")
    assert not four_drone_hover._is_retryable_telemetry_startup_failure(error)


def test_estimator_variance_requires_ten_stable_samples() -> None:
    stable = [(0.0005, 0.0006, 0.0007)] * 10
    assert not four_drone_hover._estimator_variance_stable(stable[:9])
    assert four_drone_hover._estimator_variance_stable(stable)


def test_estimator_variance_rejects_range_and_non_finite_values() -> None:
    unstable = [(0.0005, 0.0006, 0.0007)] * 9 + [(0.002, 0.0006, 0.0007)]
    non_finite = [(0.0005, 0.0006, 0.0007)] * 9 + [
        (float("nan"), 0.0006, 0.0007)
    ]
    assert not four_drone_hover._estimator_variance_stable(unstable)
    assert not four_drone_hover._estimator_variance_stable(non_finite)


def test_cleanup_temporarily_ignores_repeated_termination_signals() -> None:
    previous_int = signal.getsignal(signal.SIGINT)
    previous_term = signal.getsignal(signal.SIGTERM)
    with four_drone_hover._ignore_cleanup_interrupts():
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
        four_drone_hover,
        "ensure_crazyradio_free",
        lambda: calls.append("checked"),
    )
    four_drone_hover._stop_launch(FakeProcess())
    assert calls == ["checked"]
