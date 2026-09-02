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

    assert result == pytest.approx((0.2 + pi / 2.0, -1.0, 1.2, -2.0, -1.5, 0.0))


def test_home_command_is_hardware_free_by_default(capsys) -> None:
    result = ur5e_go_home.main([])

    assert result == 0
    assert "DRY RUN" in capsys.readouterr().out


def test_execute_requires_matching_robot_confirmation(capsys) -> None:
    result = ur5e_go_home.main(["--execute"])

    assert result == 2
    assert "confirm-robot-ip" in capsys.readouterr().err
