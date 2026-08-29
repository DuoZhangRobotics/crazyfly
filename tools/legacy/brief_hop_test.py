#!/usr/bin/env python3
"""Guarded open-loop hop for a stock Crazyflie 2.x.

This cannot control height precisely. It sends one short, capped thrust pulse,
then sends repeated zero/stop commands and disarms.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger


CACHE_DIR = Path(__file__).resolve().parent / ".cache"
DEFAULT_URI = "radio://0/80/2M"
MAX_THRUST = 50_000
CONTROL_PERIOD_S = 0.01


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Attempt one very brief vertical hop.")
    parser.add_argument("--uri", default=DEFAULT_URI)
    parser.add_argument("--confirm", metavar="HOP")
    args = parser.parse_args()
    if args.confirm != "HOP":
        parser.error("add --confirm HOP only after the flight area is clear")
    return args


def preflight(scf: SyncCrazyflie) -> tuple[float, float, float, float]:
    config = LogConfig(name="HopPreflight", period_in_ms=100)
    config.add_variable("pm.vbat", "float")
    config.add_variable("stabilizer.roll", "float")
    config.add_variable("stabilizer.pitch", "float")
    config.add_variable("stateEstimate.z", "float")
    config.add_variable("sys.canfly", "uint8_t")

    samples: list[dict[str, float]] = []
    with SyncLogger(scf, config) as logger:
        for _timestamp, data, _block in logger:
            samples.append(data)
            if len(samples) == 10:
                break

    battery = statistics.fmean(x["pm.vbat"] for x in samples)
    roll = statistics.fmean(x["stabilizer.roll"] for x in samples)
    pitch = statistics.fmean(x["stabilizer.pitch"] for x in samples)
    baseline_z = statistics.fmean(x["stateEstimate.z"] for x in samples)
    can_fly = bool(samples[-1]["sys.canfly"])

    if not can_fly:
        raise RuntimeError("firmware reports that flight is not possible")
    if battery < 3.55:
        raise RuntimeError(f"battery is too low for this test ({battery:.2f} V)")
    if abs(roll) > 8 or abs(pitch) > 8:
        raise RuntimeError(
            f"aircraft is not level (roll {roll:+.1f}, pitch {pitch:+.1f} degrees)"
        )
    return battery, roll, pitch, baseline_z


def send_ramp(cf: Crazyflie, start: int, stop: int, duration: float, abort: Event) -> None:
    started = time.monotonic()
    while True:
        elapsed = time.monotonic() - started
        if elapsed >= duration or abort.is_set():
            break
        fraction = elapsed / duration
        thrust = round(start + (stop - start) * fraction)
        cf.commander.send_setpoint(0.0, 0.0, 0.0, thrust)
        time.sleep(CONTROL_PERIOD_S)


def send_constant(cf: Crazyflie, thrust: int, duration: float, abort: Event) -> None:
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline and not abort.is_set():
        cf.commander.send_setpoint(0.0, 0.0, 0.0, thrust)
        time.sleep(CONTROL_PERIOD_S)


def run_hop(uri: str) -> None:
    cf = Crazyflie(rw_cache=str(CACHE_DIR))
    with SyncCrazyflie(uri, cf=cf) as scf:
        scf.wait_for_params()
        battery, roll, pitch, baseline_z = preflight(scf)
        print(
            f"Preflight passed: {battery:.2f} V, "
            f"roll {roll:+.1f} deg, pitch {pitch:+.1f} deg"
        )

        abort = Event()
        observed_z: list[float] = []
        observed_tilt: list[float] = []
        observed_battery: list[float] = []
        abort_reasons: set[str] = set()
        monitor = LogConfig(name="HopMonitor", period_in_ms=10)
        monitor.add_variable("stateEstimate.z", "float")
        monitor.add_variable("stabilizer.roll", "float")
        monitor.add_variable("stabilizer.pitch", "float")
        monitor.add_variable("pm.vbat", "float")

        def monitor_callback(_timestamp: int, data: dict[str, float], _block: object) -> None:
            z_delta = data["stateEstimate.z"] - baseline_z
            observed_z.append(z_delta)
            observed_tilt.append(
                max(abs(data["stabilizer.roll"]), abs(data["stabilizer.pitch"]))
            )
            observed_battery.append(data["pm.vbat"])
            # The stock barometer can jump by meters from pressure changes and
            # prop wash, so it is recorded but is not a usable hop-height cutoff.
            if abs(data["stabilizer.roll"]) > 20 or abs(data["stabilizer.pitch"]) > 20:
                abort_reasons.add("tilt exceeded 20 degrees")
                abort.set()
            if data["pm.vbat"] < 3.30:
                abort_reasons.add("battery fell below 3.30 V")
                abort.set()

        scf.cf.log.add_config(monitor)
        monitor.data_received_cb.add_callback(monitor_callback)
        monitor.start()

        print("Motors will start after this countdown:")
        for seconds in (5, 4, 3, 2, 1):
            print(f"  {seconds}", flush=True)
            time.sleep(1)

        try:
            # Unlock the low-level commander before applying any thrust.
            scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
            scf.cf.supervisor.send_arming_request(True)
            time.sleep(0.5)

            # Cross the likely lift threshold only briefly. Height is not closed-loop.
            send_ramp(scf.cf, 0, MAX_THRUST, 0.65, abort)
            send_constant(scf.cf, MAX_THRUST, 0.35, abort)
            send_ramp(scf.cf, MAX_THRUST, 0, 0.15, abort)
        finally:
            # Repeated zeros are intentional: they make packet loss less consequential.
            for _ in range(20):
                try:
                    scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
                except Exception:
                    break
                time.sleep(0.02)
            try:
                scf.cf.commander.send_stop_setpoint()
            except Exception:
                pass
            try:
                scf.cf.supervisor.send_arming_request(False)
            except Exception:
                pass
            try:
                monitor.stop()
                monitor.delete()
            except Exception:
                pass

        if abort.is_set():
            reason = ", ".join(sorted(abort_reasons)) or "unknown monitor condition"
            print(f"Safety cutoff triggered ({reason}); motors stopped and aircraft disarmed.")
        else:
            print("Pulse complete; motors stopped and aircraft disarmed.")
        if observed_z:
            print(
                "Estimated peak height change (rough without a range sensor): "
                f"{max(observed_z) * 100:.1f} cm"
            )
        if observed_tilt:
            print(f"Peak measured tilt: {max(observed_tilt):.1f} degrees")
        if observed_battery:
            print(f"Lowest measured battery: {min(observed_battery):.2f} V")


def main() -> int:
    args = parse_args()
    try:
        cflib.crtp.init_drivers(enable_debug_driver=False)
        run_hop(args.uri)
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted; stop/disarm sequence requested.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"\nFAILED: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
