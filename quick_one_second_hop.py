#!/usr/bin/env python3
"""Guarded one-second open-loop hop without downloading firmware tables."""

from __future__ import annotations

import argparse
import struct
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crtp.crtpstack import CRTPPacket, CRTPPort


TAKEOFF_THRUST = 35_000
FLIGHT_SECONDS = 1.0
CONTROL_PERIOD = 0.01


def send_packet(link, port: int, channel: int, data: bytes | tuple[int, ...]) -> None:
    packet = CRTPPacket()
    packet.set_header(port, channel)
    packet.data = data
    if not link.send_packet(packet):
        raise RuntimeError("the radio transmit queue is unavailable")


def send_thrust(link, thrust: int) -> None:
    # Level roll/pitch and zero yaw rate; the onboard stabilizer remains active.
    data = struct.pack("<fffH", 0.0, 0.0, 0.0, thrust)
    send_packet(link, CRTPPort.COMMANDER, 0, data)


def send_arming(link, armed: bool) -> None:
    # Send both forms so the cleanup works with old and new firmware.
    send_packet(link, CRTPPort.PLATFORM, 0, (1, int(armed)))
    send_packet(link, CRTPPort.SUPERVISOR, 1, (1, int(armed)))


def ramp(link, start: int, stop: int, seconds: float, abort: Event) -> None:
    started = time.monotonic()
    while not abort.is_set():
        elapsed = time.monotonic() - started
        if elapsed >= seconds:
            return
        fraction = elapsed / seconds
        thrust = round(start + (stop - start) * fraction)
        send_thrust(link, thrust)
        time.sleep(CONTROL_PERIOD)


def hold(link, thrust: int, seconds: float, abort: Event) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and not abort.is_set():
        send_thrust(link, thrust)
        time.sleep(CONTROL_PERIOD)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Attempt one one-second, open-loop Crazyflie hop."
    )
    parser.add_argument("--uri", required=True)
    parser.add_argument("--confirm")
    args = parser.parse_args()
    if args.confirm != "HOP":
        parser.error("add --confirm HOP only after clearing the flight area")

    abort = Event()
    errors: list[str] = []

    def link_error(message: str) -> None:
        errors.append(message)
        abort.set()

    cflib.crtp.init_drivers(enable_debug_driver=False)
    link = None
    try:
        link = cflib.crtp.get_link_driver(args.uri, link_error_callback=link_error)
        if not link.scan_selected((args.uri,)):
            raise RuntimeError("the Crazyflie did not answer at this URI")

        print("Radio responded. Hop starts after this countdown:", flush=True)
        for seconds in (5, 4, 3, 2, 1):
            print(f"  {seconds}", flush=True)
            time.sleep(1)

        send_thrust(link, 0)
        send_arming(link, True)
        time.sleep(0.25)
        ramp(link, 0, TAKEOFF_THRUST, 0.45, abort)
        hold(link, TAKEOFF_THRUST, FLIGHT_SECONDS, abort)
        ramp(link, TAKEOFF_THRUST, 0, 0.20, abort)

        if abort.is_set():
            raise RuntimeError(errors[-1] if errors else "radio link was interrupted")
        print("One-second hop completed; motors stopped and disarmed")
        return 0
    except KeyboardInterrupt:
        print("Interrupted; stop sequence requested", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"FAILED: {exc}", file=sys.stderr)
        return 1
    finally:
        if link is not None:
            for _ in range(30):
                try:
                    send_thrust(link, 0)
                except Exception:
                    break
                time.sleep(0.02)
            try:
                # Generic STOP, followed by legacy and current disarm requests.
                send_packet(link, CRTPPort.COMMANDER_GENERIC, 0, (0,))
                send_arming(link, False)
            except Exception:
                pass
            link.close()


if __name__ == "__main__":
    raise SystemExit(main())
