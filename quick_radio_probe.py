#!/usr/bin/env python3
"""Quickly check whether one Crazyflie radio URI acknowledges packets."""

from __future__ import annotations

import argparse
import sys

import cflib.crtp


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Probe one Crazyflie URI without downloading logs or parameters."
    )
    parser.add_argument("--uri", required=True)
    args = parser.parse_args()

    cflib.crtp.init_drivers(enable_debug_driver=False)
    link = None
    try:
        link = cflib.crtp.get_link_driver(args.uri)
        found = link.scan_selected((args.uri,))
        if found:
            print(f"RESPONDING: {args.uri}")
            return 0
        print(f"NO RESPONSE: {args.uri}")
        return 1
    except Exception as exc:
        print(f"FAILED: {exc}", file=sys.stderr)
        return 1
    finally:
        if link is not None:
            link.close()


if __name__ == "__main__":
    raise SystemExit(main())
