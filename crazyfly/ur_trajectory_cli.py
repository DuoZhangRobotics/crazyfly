"""Validate or execute the UR5e portions of a pRRTC execution bundle."""

from __future__ import annotations

import argparse
from contextlib import suppress
import math
from math import pi
import sys
import time

from .config import ConfigError
from .prrtc_bundle import load_execution_bundle
from .ur_executor import TimedURExecutor, URExecutionConfig, connect_ur5e


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--robot-ip", default="172.16.90.197")
    parser.add_argument("--confirm-robot-ip")
    parser.add_argument("--first-joint-offset-rad", type=float, default=pi / 2.0)
    parser.add_argument("--servo-lookahead-s", type=float, default=0.03)
    parser.add_argument("--servo-gain", type=float, default=1000.0)
    parser.add_argument("--maximum-joint-error-rad", type=float, default=0.20)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    executor = None
    try:
        if not 0.03 <= args.servo_lookahead_s <= 0.20:
            raise ConfigError("servo lookahead must be from 0.03 to 0.20 seconds")
        if not 100 <= args.servo_gain <= 2000:
            raise ConfigError("servo gain must be from 100 to 2000")
        if (
            not math.isfinite(args.maximum_joint_error_rad)
            or args.maximum_joint_error_rad <= 0
        ):
            raise ConfigError("maximum joint error must be positive and finite")
        bundle = load_execution_bundle(
            args.bundle,
            require_clean=args.execute,
            first_joint_offset_rad=args.first_joint_offset_rad,
            accept_missing_physical_evidence=True,
        )
        print(
            f"PASS: UR5e main {bundle.main.duration_s:.3f} s, "
            f"park {bundle.park.duration_s:.3f} s"
        )
        if not args.execute:
            print("DRY RUN: no RTDE connection or robot command was started")
            return 0
        if args.confirm_robot_ip != args.robot_ip:
            raise ConfigError("--confirm-robot-ip must match the UR5e robot IP")
        control, receive = connect_ur5e(args.robot_ip)
        executor = TimedURExecutor(
            control,
            receive,
            URExecutionConfig(
                servo_lookahead_s=args.servo_lookahead_s,
                servo_gain=args.servo_gain,
                maximum_joint_error_rad=args.maximum_joint_error_rad,
            ),
        )
        executor.validate_robot_ready()
        executor.preposition(bundle.main.start)
        samples = list(
            executor.execute(bundle.main, time.monotonic() + 2.0)
        )
        samples.extend(
            executor.execute(bundle.park, time.monotonic() + 0.02)
        )
        executor.stop()
        errors = [sample.maximum_error_rad for sample in samples]
        maximum_error = max(errors)
        rms_error = math.sqrt(
            sum(error * error for error in errors) / len(errors)
        )
        print("UR5E TRAJECTORY COMPLETE")
        print(f"Maximum joint error: {maximum_error:.6f} rad")
        print(f"RMS joint error:     {rms_error:.6f} rad")
        return 0
    except KeyboardInterrupt:
        return 130
    except (ConfigError, RuntimeError, OSError) as exc:
        print(f"FAILED: {exc}", file=sys.stderr)
        return 2
    finally:
        if executor is not None:
            with suppress(Exception):
                executor.stop()
            with suppress(Exception):
                executor.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
