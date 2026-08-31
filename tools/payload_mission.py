#!/usr/bin/env python3
"""Run one compiled payload with an exact selected subset of the lab fleet."""

from __future__ import annotations

import argparse
import tempfile
from pathlib import Path

import yaml

try:
    from . import trajectory_mission
except ImportError:  # Direct execution from the tools directory.
    import trajectory_mission


ROOT = Path(__file__).resolve().parents[1]
FLEET = ROOT / "config" / "local" / "crazyflies.yaml"
SAFETY = ROOT / "config" / "local" / "safety.yaml"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Validate or run an exact compiled payload using only the selected "
            "Crazyflies. Remaining arguments are forwarded to trajectory_mission."
        )
    )
    parser.add_argument("compiled_payload")
    parser.add_argument(
        "--robots",
        required=True,
        help="comma-separated robot IDs, for example cf1 or cf1,cf2,cf3",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args, forwarded = _parser().parse_known_args(argv)
    selected = tuple(
        name.strip() for name in args.robots.split(",") if name.strip()
    )
    if not selected or len(set(selected)) != len(selected):
        raise SystemExit("--robots must contain unique robot IDs")

    fleet = yaml.safe_load(FLEET.read_text(encoding="utf-8"))
    available = set(fleet["robots"])
    unknown = set(selected) - available
    if unknown:
        raise SystemExit(f"unknown robot IDs: {', '.join(sorted(unknown))}")
    for name, robot in fleet["robots"].items():
        robot["enabled"] = name in selected

    safety = yaml.safe_load(SAFETY.read_text(encoding="utf-8"))
    safety["crazyfly_safety"]["tracking"][
        "expected_raw_marker_count"
    ] = len(selected)
    if "--mock" in forwarded and "--execute" in forwarded:
        safety["crazyfly_safety"]["flight_enabled"] = True

    with tempfile.TemporaryDirectory(prefix="crazyfly-payload-fleet-") as temporary:
        directory = Path(temporary)
        fleet_path = directory / "crazyflies.yaml"
        safety_path = directory / "safety.yaml"
        fleet_path.write_text(
            yaml.safe_dump(fleet, sort_keys=False), encoding="utf-8"
        )
        safety_path.write_text(
            yaml.safe_dump(safety, sort_keys=False), encoding="utf-8"
        )
        return trajectory_mission.main(
            [
                *forwarded,
                "--compiled-payload",
                str(Path(args.compiled_payload).expanduser().resolve()),
                "--fleet",
                str(fleet_path),
                "--safety",
                str(safety_path),
            ]
        )


if __name__ == "__main__":
    raise SystemExit(main())
