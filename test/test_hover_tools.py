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
        enabled = {
            name for name, robot in selected["robots"].items() if robot["enabled"]
        }
        assert enabled == {"cf3"}
        assert four_drone_hover.EXPECTED_ROBOTS == ("cf3",)
        return 0

    monkeypatch.setattr(four_drone_hover, "main", fake_hover_main)

    assert one_drone_hover.main(["03"]) == 0


def test_drone_index_must_keep_leading_zero() -> None:
    with pytest.raises(SystemExit):
        one_drone_hover.main(["3"])


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
