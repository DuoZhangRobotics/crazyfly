#!/usr/bin/env python3
"""Short hover test for a Crazyflie fitted with a Flow deck.

Run cf_diagnostics.py and complete a manual first flight before using this.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

from cf_diagnostics import active_decks, scan_for_crazyflies


CACHE_DIR = Path(__file__).resolve().parent / ".cache"
FLOW_DECK_PARAMETERS = {"bcFlow", "bcFlow2"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Take off to 0.30 m, hover for 3 seconds, and land. Flow deck required."
    )
    parser.add_argument("--uri", help="Crazyflie radio URI; scans when omitted")
    parser.add_argument(
        "--confirm",
        metavar="FLY",
        help="Required acknowledgement that the flight area is safe",
    )
    args = parser.parse_args()
    if args.confirm != "FLY":
        parser.error(
            "flight not authorized; clear a 2 m indoor area, wear eye protection, "
            "keep people away, then add: --confirm FLY"
        )
    return args


def run_hover(uri: str) -> None:
    cf = Crazyflie(rw_cache=str(CACHE_DIR))
    print(f"Connecting to {uri} ...")
    with SyncCrazyflie(uri, cf=cf) as scf:
        scf.wait_for_params()
        decks = set(active_decks(scf.cf))
        flow_decks = decks & FLOW_DECK_PARAMETERS
        if not flow_decks:
            raise RuntimeError(
                "No Flow deck was detected. MotionCommander is not safe for a stock "
                "Crazyflie 2.0; use cfclient with a gamepad for the first flight."
            )

        print(f"Flow deck detected: {', '.join(sorted(flow_decks))}")
        print("Arming; taking off to 0.30 m in 3 seconds...")
        for seconds in (3, 2, 1):
            print(f"  {seconds}")
            time.sleep(1)

        try:
            scf.cf.supervisor.send_arming_request(True)
            time.sleep(1)
            with MotionCommander(scf, default_height=0.30):
                print("Hovering for 3 seconds. Press Ctrl-C to land early.")
                time.sleep(3)
            print("Landed.")
        finally:
            # Ensure no old setpoint remains active and leave the supervisor disarmed.
            scf.cf.commander.send_stop_setpoint()
            scf.cf.supervisor.send_arming_request(False)


def main() -> int:
    args = parse_args()
    try:
        cflib.crtp.init_drivers(enable_debug_driver=False)
        uri = args.uri or scan_for_crazyflies()[0]
        run_hover(uri)
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted; landing/stopping.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"\nFAILED: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
