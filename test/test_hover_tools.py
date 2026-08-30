import signal
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import crazyradio_guard
import four_drone_hover
import one_drone_hover


def test_crazyradio_holder_parsing_is_pid_only() -> None:
    assert crazyradio_guard.parse_fuser_pids(" 1234  5678c\n") == {1234, 5678}


def test_only_known_lab_processes_are_eligible_for_release() -> None:
    assert crazyradio_guard.is_known_crazyflie_process(
        "/opt/ros/jazzy/lib/crazyflie/crazyflie_server --ros-args"
    )
    assert crazyradio_guard.is_known_crazyflie_process("python -m cfclient.gui")
    assert not crazyradio_guard.is_known_crazyflie_process("python important_job.py")


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
