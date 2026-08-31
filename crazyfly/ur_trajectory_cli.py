"""Validate or execute the UR5e portions of a pRRTC execution bundle."""

from __future__ import annotations

import argparse
from contextlib import suppress
import sys
import time

from .config import ConfigError
from .prrtc_bundle import load_execution_bundle
from .ur_executor import TimedURExecutor, connect_ur5e


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--robot-ip", default="172.16.90.197")
    parser.add_argument("--confirm-robot-ip")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    executor = None
    try:
        bundle = load_execution_bundle(args.bundle, require_clean=args.execute)
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
        executor = TimedURExecutor(control, receive)
        executor.validate_robot_ready()
        executor.preposition(bundle.main.start)
        executor.execute(bundle.main, time.monotonic() + 2.0)
        executor.execute(bundle.park, time.monotonic() + 0.02)
        executor.stop()
        print("UR5E TRAJECTORY COMPLETE")
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
