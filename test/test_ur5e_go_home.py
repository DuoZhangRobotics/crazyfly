import json
from math import pi
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import ur5e_go_home  # noqa: E402


def test_load_physical_home_applies_installation_offset(tmp_path: Path) -> None:
    path = tmp_path / "home.json"
    path.write_text(
        json.dumps({"positions_rad": [0.2, -1.0, 1.2, -2.0, -1.5, 0.0]})
    )

    result = ur5e_go_home.load_physical_home(path)

    assert result == pytest.approx(
        (0.2 + pi / 2.0, -1.0, 1.2, -2.0, -1.5, pi / 2.0)
    )


def test_default_home_configuration_is_owned_by_crazyfly() -> None:
    assert ur5e_go_home.DEFAULT_HOME_CONFIG == (
        ROOT / "config" / "ur5e_home_configuration.json"
    )
    assert ur5e_go_home.load_physical_home(
        ur5e_go_home.DEFAULT_HOME_CONFIG
    ) == pytest.approx(
        (
            1.5708118677139282 + pi / 2.0,
            -2.2,
            1.9,
            -1.383,
            -1.5700505415545862,
            pi / 2.0,
        )
    )


def test_home_command_is_hardware_free_by_default(capsys) -> None:
    result = ur5e_go_home.main([])

    assert result == 0
    assert "DRY RUN" in capsys.readouterr().out


def test_execute_requires_matching_robot_confirmation(capsys) -> None:
    result = ur5e_go_home.main(["--execute"])

    assert result == 2
    assert "confirm-robot-ip" in capsys.readouterr().err
