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
SAFETY = ROOT / "config" / "local" / "safety.yaml"
DRONE_INDICES = ("01", "02", "03", "04")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run the managed 0.30 m hover for one drone selected by address "
            "suffix 01, 02, 03, or 04."
        )
    )
    parser.add_argument("drone_index", choices=DRONE_INDICES)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true")
    mode.add_argument("--ros-only", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args, hover_arguments = _parser().parse_known_args(argv)
    if args.execute:
        hover_arguments.append("--execute")
    elif args.ros_only:
        hover_arguments.append("--ros-only")
    robot_name = f"cf{int(args.drone_index)}"
    data = yaml.safe_load(FLEET.read_text(encoding="utf-8"))
    for name, robot in data["robots"].items():
        robot["enabled"] = name == robot_name
    data["all"]["firmware_logging"]["custom_topics"] = {
        "estimator_debug": {
            "frequency": 2,
            "vars": ["kalman.varPX", "kalman.varPY", "kalman.varPZ"],
        },
        "flight_debug": {
            "frequency": 25,
            "vars": [
                "stateEstimate.vx",
                "stateEstimate.vy",
                "stabilizer.roll",
                "stabilizer.pitch",
                "stabilizer.yaw",
                "ctrlMel.i_err_x",
            ],
        },
        "actuator_debug": {
            "frequency": 25,
            "vars": [
                "ctrlMel.cmd_roll",
                "ctrlMel.cmd_pitch",
                "ctrlMel.cmd_yaw",
                "ctrlMel.cmd_thrust",
                "motor.m1",
                "motor.m2",
                "motor.m3",
                "motor.m4",
            ],
        },
    }
    with tempfile.TemporaryDirectory(prefix="crazyfly-one-hover-") as temporary:
        selected_fleet = Path(temporary) / f"crazyflies.{robot_name}.yaml"
        selected_fleet.write_text(
            yaml.safe_dump(data, sort_keys=False), encoding="utf-8"
        )
        safety = yaml.safe_load(SAFETY.read_text(encoding="utf-8"))
        tracking = safety["crazyfly_safety"]["tracking"]
        tracking["expected_raw_marker_count"] = 1
        # Preserve the known-working single-drone flight behavior. With only
        # one marker, a fast pose cannot be an inter-drone identity swap.
        tracking["maximum_pose_speed_m_s"] = None
        tracking["pose_identity_emergency_s"] = 1.0
        tracking["marker_count_grace_s"] = 1.0
        safety["crazyfly_safety"]["timeouts"]["recovery_s"] = 1.0
        selected_safety = Path(temporary) / "safety.one-marker.yaml"
        selected_safety.write_text(
            yaml.safe_dump(safety, sort_keys=False), encoding="utf-8"
        )
        four_drone_hover.EXPECTED_ROBOTS = (robot_name,)
        return four_drone_hover.main(
            [
                "--fleet",
                str(selected_fleet),
                "--safety",
                str(selected_safety),
                *hover_arguments,
            ]
        )


if __name__ == "__main__":
    raise SystemExit(main())
