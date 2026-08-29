#!/usr/bin/env python3
"""Five-second, below-liftoff Crazyflie motor spin test."""

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
SPIN_THRUST = 30_000
SPIN_SECONDS = 5.0
CONTROL_PERIOD = 0.01


def send_for(cf: Crazyflie, thrust: int, seconds: float, abort: Event) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and not abort.is_set():
        cf.commander.send_setpoint(0.0, 0.0, 0.0, thrust)
        time.sleep(CONTROL_PERIOD)


def ramp(cf: Crazyflie, start: int, stop: int, seconds: float, abort: Event) -> None:
    started = time.monotonic()
    while not abort.is_set():
        elapsed = time.monotonic() - started
        if elapsed >= seconds:
            break
        fraction = elapsed / seconds
        thrust = round(start + (stop - start) * fraction)
        cf.commander.send_setpoint(0.0, 0.0, 0.0, thrust)
        time.sleep(CONTROL_PERIOD)


def run(uri: str) -> None:
    cf = Crazyflie(rw_cache=str(CACHE_DIR))
    with SyncCrazyflie(uri, cf=cf) as scf:
        scf.wait_for_params()

        preflight = LogConfig(name="SpinPreflight", period_in_ms=100)
        preflight.add_variable("pm.vbat", "float")
        preflight.add_variable("stabilizer.roll", "float")
        preflight.add_variable("stabilizer.pitch", "float")
        preflight.add_variable("sys.canfly", "uint8_t")
        samples = []
        with SyncLogger(scf, preflight) as logger:
            for _timestamp, data, _block in logger:
                samples.append(data)
                if len(samples) == 10:
                    break

        battery = statistics.fmean(sample["pm.vbat"] for sample in samples)
        roll = statistics.fmean(sample["stabilizer.roll"] for sample in samples)
        pitch = statistics.fmean(sample["stabilizer.pitch"] for sample in samples)
        if not samples[-1]["sys.canfly"]:
            raise RuntimeError("firmware reports that motor operation is unavailable")
        if battery < 3.50:
            raise RuntimeError(f"resting battery voltage is too low ({battery:.2f} V)")
        if abs(roll) > 8 or abs(pitch) > 8:
            raise RuntimeError("Crazyflie is not level")

        abort = Event()
        reasons: set[str] = set()
        batteries: list[float] = []
        tilts: list[float] = []
        monitor = LogConfig(name="SpinMonitor", period_in_ms=20)
        monitor.add_variable("pm.vbat", "float")
        monitor.add_variable("stabilizer.roll", "float")
        monitor.add_variable("stabilizer.pitch", "float")

        def monitor_cb(_timestamp: int, data: dict[str, float], _block: object) -> None:
            batteries.append(data["pm.vbat"])
            tilt = max(abs(data["stabilizer.roll"]), abs(data["stabilizer.pitch"]))
            tilts.append(tilt)
            if data["pm.vbat"] < 3.25:
                reasons.add("battery below 3.25 V")
                abort.set()
            if tilt > 20:
                reasons.add("tilt above 20 degrees")
                abort.set()

        scf.cf.log.add_config(monitor)
        monitor.data_received_cb.add_callback(monitor_cb)
        monitor.start()

        print(f"Preflight passed: {battery:.2f} V; motors start in 3 seconds", flush=True)
        time.sleep(3)
        try:
            scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
            scf.cf.supervisor.send_arming_request(True)
            time.sleep(0.25)
            ramp(scf.cf, 0, SPIN_THRUST, 0.5, abort)
            send_for(scf.cf, SPIN_THRUST, SPIN_SECONDS, abort)
            ramp(scf.cf, SPIN_THRUST, 0, 0.3, abort)
        finally:
            for _ in range(20):
                try:
                    scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
                except Exception:
                    break
                time.sleep(0.02)
            try:
                scf.cf.commander.send_stop_setpoint()
                scf.cf.supervisor.send_arming_request(False)
            except Exception:
                pass
            try:
                monitor.stop()
                monitor.delete()
            except Exception:
                pass

        if abort.is_set():
            print(f"Emergency stop: {', '.join(sorted(reasons))}")
        else:
            print("Five-second spin completed; motors stopped and disarmed")
        if batteries:
            print(f"Lowest battery: {min(batteries):.2f} V")
        if tilts:
            print(f"Peak tilt: {max(tilts):.1f} degrees")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--uri", required=True)
    parser.add_argument("--confirm")
    args = parser.parse_args()
    if args.confirm != "SPIN":
        parser.error("add --confirm SPIN after clearing the propeller area")
    try:
        cflib.crtp.init_drivers(enable_debug_driver=False)
        run(args.uri)
        return 0
    except KeyboardInterrupt:
        print("Interrupted; stop sequence requested", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"FAILED: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
