#!/usr/bin/env python3
"""Safely command the lab UR5e to its reviewed physical home configuration."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys


DEFAULT_ROBOT_IP = "172.16.90.197"
DEFAULT_HOME_CONFIG = Path(
    "/home/duo/pRRTC/dataset/ur5e_crazyflie_experiment/"
    "hardware_inputs/ur5e_home_configuration.json"
)


def load_physical_home(
    path: str | Path,
    first_joint_offset_rad: float = math.pi / 2.0,
    last_joint_offset_rad: float = math.pi / 2.0,
) -> tuple[float, float, float, float, float, float]:
    source = Path(path).expanduser().resolve()
    data = json.loads(source.read_text(encoding="utf-8"))
    positions = data.get("positions_rad")
    if (
        not isinstance(positions, list)
        or len(positions) != 6
        or any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            for value in positions
        )
    ):
        raise ValueError("home configuration requires six finite joint positions")
    result = [float(value) for value in positions]
    result[0] += first_joint_offset_rad
    result[5] += last_joint_offset_rad
    return tuple(result)  # type: ignore[return-value]


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--robot-ip", default=DEFAULT_ROBOT_IP)
    parser.add_argument("--confirm-robot-ip")
    parser.add_argument("--home-config", default=str(DEFAULT_HOME_CONFIG))
    parser.add_argument("--first-joint-offset-rad", type=float, default=math.pi / 2.0)
    parser.add_argument("--last-joint-offset-rad", type=float, default=math.pi / 2.0)
    parser.add_argument("--speed-rad-s", type=float, default=0.15)
    parser.add_argument("--acceleration-rad-s2", type=float, default=0.15)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    receive = None
    control = None
    try:
        for value, label in (
            (args.first_joint_offset_rad, "first-joint offset"),
            (args.last_joint_offset_rad, "last-joint offset"),
            (args.speed_rad_s, "speed"),
            (args.acceleration_rad_s2, "acceleration"),
        ):
            if not math.isfinite(value):
                raise ValueError(f"{label} must be finite")
        if args.speed_rad_s <= 0 or args.acceleration_rad_s2 <= 0:
            raise ValueError("speed and acceleration must be positive")
        home = load_physical_home(
            args.home_config,
            args.first_joint_offset_rad,
            args.last_joint_offset_rad,
        )
        print(f"Robot: {args.robot_ip}")
        print(f"Physical home: {[round(value, 9) for value in home]}")
        print(
            f"Motion: {args.speed_rad_s:.3f} rad/s, "
            f"{args.acceleration_rad_s2:.3f} rad/s^2"
        )
        if not args.execute:
            print("DRY RUN: no RTDE connection or robot command was started")
            return 0
        if args.confirm_robot_ip != args.robot_ip:
            raise ValueError("--confirm-robot-ip must match --robot-ip")

        import rtde_control
        import rtde_receive

        receive = rtde_receive.RTDEReceiveInterface(args.robot_ip)
        if receive.getRobotMode() != 7:
            raise RuntimeError("UR5e robot mode is not RUNNING")
        if receive.getSafetyMode() != 1:
            raise RuntimeError("UR5e safety mode is not NORMAL")
        tcp = tuple(float(value) for value in receive.getActualTCPPose())
        if len(tcp) != 6 or not all(math.isfinite(value) for value in tcp):
            raise RuntimeError("UR5e returned an invalid TCP pose")
        print(f"Current TCP: {[round(value, 6) for value in tcp]}")

        control = rtde_control.RTDEControlInterface(args.robot_ip)
        if not control.moveJ(
            list(home), args.speed_rad_s, args.acceleration_rad_s2
        ):
            raise RuntimeError("moveJ to reviewed home failed")
        actual = tuple(float(value) for value in receive.getActualQ())
        error = max(abs(value - target) for value, target in zip(actual, home))
        if error > 0.03:
            raise RuntimeError(
                f"UR5e did not settle at home; maximum joint error {error:.4f} rad"
            )
        print(f"UR5e is home; maximum joint error {error:.6f} rad")
        return 0
    except KeyboardInterrupt:
        if control is not None:
            try:
                control.stopJ(1.0)
            except Exception:
                pass
        print("Interrupted; robot stop requested", file=sys.stderr)
        return 130
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as exc:
        print(f"FAILED: {exc}", file=sys.stderr)
        return 2
    finally:
        if control is not None:
            try:
                control.stopScript()
            except Exception:
                pass
            control.disconnect()
        if receive is not None:
            receive.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
