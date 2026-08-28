#!/usr/bin/env python3
"""Read-only Crazyflie radio and telemetry diagnostics.

This program never arms the Crazyflie and never sends motor setpoints.
"""

from __future__ import annotations

import argparse
import statistics
import sys
from pathlib import Path

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger


CACHE_DIR = Path(__file__).resolve().parent / ".cache"


def scan_for_crazyflies() -> list[str]:
    """Return the radio URIs reported by cflib."""
    print("Scanning for Crazyflies (the aircraft must be powered on)...")
    interfaces = cflib.crtp.scan_interfaces()
    uris = [uri for uri, _description in interfaces if uri.startswith("radio://")]

    if not uris:
        raise RuntimeError(
            "No Crazyflie was found. Put it level, switch it on, wait for sensor "
            "calibration, and make sure cfclient is not using the radio."
        )

    print("Found:")
    for uri in uris:
        print(f"  {uri}")
    return uris


def active_decks(cf: Crazyflie) -> list[str]:
    """Return deck parameters whose value indicates an attached deck."""
    detected: list[str] = []
    for name, value in cf.param.values.get("deck", {}).items():
        try:
            if int(value) == 1:
                detected.append(name)
        except (TypeError, ValueError):
            continue
    return sorted(detected)


def read_telemetry(uri: str, sample_count: int) -> None:
    """Connect without arming and summarize a few flight-health values."""
    log_config = LogConfig(name="FirstFlightCheck", period_in_ms=100)
    log_config.add_variable("pm.vbat", "float")
    log_config.add_variable("stabilizer.roll", "float")
    log_config.add_variable("stabilizer.pitch", "float")
    log_config.add_variable("stabilizer.yaw", "float")
    log_config.add_variable("sys.canfly", "uint8_t")

    print(f"Connecting to {uri} ...")
    cf = Crazyflie(rw_cache=str(CACHE_DIR))
    with SyncCrazyflie(uri, cf=cf) as scf:
        scf.wait_for_params()
        print("Connected. Motors remain disarmed.")

        decks = active_decks(scf.cf)
        if decks:
            print(f"Detected deck(s): {', '.join(decks)}")
        else:
            print("Detected deck(s): none")

        samples: list[dict[str, float]] = []
        with SyncLogger(scf, log_config) as logger:
            for _timestamp, data, _log_block in logger:
                samples.append(data)
                if len(samples) >= sample_count:
                    break

    battery = statistics.fmean(sample["pm.vbat"] for sample in samples)
    roll = statistics.fmean(sample["stabilizer.roll"] for sample in samples)
    pitch = statistics.fmean(sample["stabilizer.pitch"] for sample in samples)
    can_fly = bool(samples[-1]["sys.canfly"])

    print("\nDiagnostic summary")
    print(f"  Battery:       {battery:.2f} V")
    print(f"  Mean attitude: roll {roll:+.2f} deg, pitch {pitch:+.2f} deg")
    print(f"  Firmware says flight is possible: {'yes' if can_fly else 'NO'}")

    if abs(roll) > 5 or abs(pitch) > 5:
        print(
            "  Warning: attitude is not close to level. Repeat the test on a flat, "
            "motionless surface before flying."
        )
    if not can_fly:
        raise RuntimeError(
            "The Crazyflie reports that it cannot fly. Check the cfclient Console "
            "tab for self-test, sensor-calibration, battery, or firmware errors."
        )

    print("\nPASS: radio link and basic telemetry are working.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Safely scan and read Crazyflie telemetry without starting motors."
    )
    parser.add_argument(
        "--uri",
        help="Use a known radio URI instead of scanning, for example radio://0/80/2M/E7E7E7E7E7",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=20,
        help="Number of 10 Hz samples to collect (default: 20)",
    )
    args = parser.parse_args()
    if args.samples < 1:
        parser.error("--samples must be at least 1")
    return args


def main() -> int:
    args = parse_args()
    try:
        cflib.crtp.init_drivers(enable_debug_driver=False)
        uri = args.uri or scan_for_crazyflies()[0]
        read_telemetry(uri, args.samples)
        return 0
    except KeyboardInterrupt:
        print("\nCancelled.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"\nFAILED: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
