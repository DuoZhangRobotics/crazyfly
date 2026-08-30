#!/usr/bin/env python3
"""Read resting battery voltage from every enabled Crazyflie without arming."""

from __future__ import annotations

import argparse
import logging
import statistics
import sys
from dataclasses import dataclass
from math import isfinite
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from crazyfly.config import ConfigError, load_fleet, load_safety  # noqa: E402
from crazyradio_guard import ensure_crazyradio_free  # noqa: E402


DEFAULT_FLEET = ROOT / "config" / "local" / "crazyflies.yaml"
DEFAULT_SAFETY = ROOT / "config" / "local" / "safety.yaml"
CACHE_DIR = ROOT / ".cache" / "cflib"


@dataclass(frozen=True)
class BatteryReading:
    name: str
    uri: str
    voltage: float | None = None
    error: str | None = None


class StaleLogEntryFilter(logging.Filter):
    def filter(self, record: logging.LogRecord) -> bool:  # noqa: A003
        message = record.getMessage()
        return not (
            message.startswith("Error no LogEntry to handle")
            or message.startswith("No LogEntry to assign block")
        )


def classify_voltage(voltage: float, warning_v: float, critical_v: float) -> str:
    if not isfinite(voltage):
        return "INVALID"
    if voltage <= critical_v:
        return "CRITICAL"
    if voltage <= warning_v:
        return "LOW"
    return "READY"


def read_voltage(uri: str, sample_count: int) -> float:
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.crazyflie.syncLogger import SyncLogger

    log_config = LogConfig(name="FleetBattery", period_in_ms=100)
    log_config.add_variable("pm.vbat", "float")
    samples: list[float] = []
    crazyflie = Crazyflie(rw_cache=str(CACHE_DIR))
    with SyncCrazyflie(uri, cf=crazyflie) as connected:
        with SyncLogger(connected, log_config) as logger:
            for _timestamp, data, _log_block in logger:
                samples.append(float(data["pm.vbat"]))
                if len(samples) >= sample_count:
                    break
    if not samples:
        raise RuntimeError("no battery samples received")
    return statistics.fmean(samples)


def render_table(
    readings: list[BatteryReading], warning_v: float, critical_v: float
) -> str:
    rows = []
    for reading in readings:
        address = reading.uri.rsplit("/", 1)[-1][-2:]
        if reading.error is not None:
            rows.append((reading.name, address, "--", "OFFLINE", reading.error))
        else:
            assert reading.voltage is not None
            rows.append(
                (
                    reading.name,
                    address,
                    f"{reading.voltage:.2f} V",
                    classify_voltage(reading.voltage, warning_v, critical_v),
                    "",
                )
            )
    headers = ("Drone", "Address", "Voltage", "Status", "Details")
    widths = [
        max(len(headers[index]), *(len(row[index]) for row in rows))
        for index in range(len(headers))
    ]

    def format_row(row) -> str:
        return "  ".join(
            value.ljust(widths[index]) for index, value in enumerate(row)
        ).rstrip()

    separator = tuple("-" * width for width in widths)
    return "\n".join(
        [format_row(headers), format_row(separator), *(format_row(row) for row in rows)]
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read all enabled Crazyflie batteries sequentially. Motors remain "
            "disarmed."
        )
    )
    parser.add_argument("--fleet", default=str(DEFAULT_FLEET))
    parser.add_argument("--safety", default=str(DEFAULT_SAFETY))
    parser.add_argument(
        "--samples",
        type=int,
        default=5,
        help="number of 10 Hz voltage samples per drone (default: 5)",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    if args.samples < 1:
        parser.error("--samples must be at least 1")

    try:
        fleet = load_fleet(args.fleet)
        safety = load_safety(args.safety)
        robots = sorted(
            fleet.enabled.values(), key=lambda robot: int(robot.name.removeprefix("cf"))
        )
        if not robots:
            raise ConfigError("fleet configuration has no enabled drones")
        ensure_crazyradio_free()
        CACHE_DIR.mkdir(parents=True, exist_ok=True)
        import cflib.crtp

        logging.getLogger("cflib.crazyflie.log").addFilter(StaleLogEntryFilter())
        cflib.crtp.init_drivers(enable_debug_driver=False)

        readings = []
        for robot in robots:
            print(f"Reading {robot.name} ({robot.uri}) ...", flush=True)
            try:
                voltage = read_voltage(robot.uri, args.samples)
                readings.append(BatteryReading(robot.name, robot.uri, voltage=voltage))
            except Exception as exc:
                readings.append(BatteryReading(robot.name, robot.uri, error=str(exc)))

        print()
        print(render_table(readings, safety.battery_warning_v, safety.battery_critical_v))
        print(
            f"\nResting thresholds: READY > {safety.battery_warning_v:.2f} V, "
            f"CRITICAL <= {safety.battery_critical_v:.2f} V"
        )
        print("Motors were never armed.")
        return 1 if any(reading.error is not None for reading in readings) else 0
    except KeyboardInterrupt:
        print("\nCancelled; motors were never armed.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"FAILED: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
