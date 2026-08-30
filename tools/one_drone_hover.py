#!/usr/bin/env python3
"""Run the managed hover helper for one selected Crazyflie."""

from __future__ import annotations

import argparse
import tempfile
from pathlib import Path

import four_drone_hover
import yaml

ROOT = Path(__file__).resolve().parents[1]
FLEET = ROOT / "config" / "local" / "crazyflies.yaml"
DRONE_INDICES = ("01", "02", "03", "04")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run the managed 0.30 m hover for one drone selected by address "
            "suffix 01, 02, 03, or 04."
        )
    )
    parser.add_argument("drone_index", choices=DRONE_INDICES)
    parser.add_argument("--execute", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args, hover_arguments = _parser().parse_known_args(argv)
    if args.execute:
        hover_arguments.append("--execute")
    robot_name = f"cf{int(args.drone_index)}"
    data = yaml.safe_load(FLEET.read_text(encoding="utf-8"))
    for name, robot in data["robots"].items():
        robot["enabled"] = name == robot_name
    with tempfile.TemporaryDirectory(prefix="crazyfly-one-hover-") as temporary:
        selected_fleet = Path(temporary) / f"crazyflies.{robot_name}.yaml"
        selected_fleet.write_text(
            yaml.safe_dump(data, sort_keys=False), encoding="utf-8"
        )
        four_drone_hover.EXPECTED_ROBOTS = (robot_name,)
        return four_drone_hover.main(["--fleet", str(selected_fleet), *hover_arguments])


if __name__ == "__main__":
    raise SystemExit(main())
