#!/usr/bin/env python3
"""Short motor spin that skips the slow Crazyflie parameter/memory setup."""

from __future__ import annotations

import argparse
import struct
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crtp.crtpstack import CRTPPacket, CRTPPort


SPIN_THRUST = 20_000
SPIN_SECONDS = 1.5
CONTROL_PERIOD = 0.02


def send_packet(link, port: int, channel: int, data: bytes | tuple[int, ...]) -> None:
    packet = CRTPPacket()
    packet.set_header(port, channel)
    packet.data = data
    if not link.send_packet(packet):
        raise RuntimeError("the radio transmit queue is unavailable")


def send_thrust(link, thrust: int) -> None:
    send_packet(link, CRTPPort.COMMANDER, 0, struct.pack("<fffH", 0.0, 0.0, 0.0, thrust))


def send_arming(link, armed: bool) -> None:
    # Send both forms: old firmware uses PLATFORM, new firmware uses SUPERVISOR.
    send_packet(link, CRTPPort.PLATFORM, 0, (1, int(armed)))
    send_packet(link, CRTPPort.SUPERVISOR, 1, (1, int(armed)))


def ramp(link, start: int, stop: int, seconds: float, abort: Event) -> None:
    started = time.monotonic()
    while not abort.is_set():
        elapsed = time.monotonic() - started
        if elapsed >= seconds:
            return
        fraction = elapsed / seconds
        send_thrust(link, round(start + (stop - start) * fraction))
        time.sleep(CONTROL_PERIOD)


def hold(link, thrust: int, seconds: float, abort: Event) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and not abort.is_set():
        send_thrust(link, thrust)
        time.sleep(CONTROL_PERIOD)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Spin all four motors briefly without downloading firmware tables."
    )
    parser.add_argument("--uri", required=True)
    parser.add_argument("--confirm")
    args = parser.parse_args()
    if args.confirm != "SPIN":
        parser.error("add --confirm SPIN after clearing the propeller area")

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

        print("Radio responded. Motors start in 3 seconds.", flush=True)
        time.sleep(3)
        send_thrust(link, 0)
        send_arming(link, True)
        time.sleep(0.25)
        ramp(link, 0, SPIN_THRUST, 0.3, abort)
        hold(link, SPIN_THRUST, SPIN_SECONDS, abort)
        ramp(link, SPIN_THRUST, 0, 0.2, abort)

        if abort.is_set():
            raise RuntimeError(errors[-1] if errors else "radio link was interrupted")
        print("Spin completed; motors stopped and disarmed")
        return 0
    except KeyboardInterrupt:
        print("Interrupted; stop sequence requested", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"FAILED: {exc}", file=sys.stderr)
        return 1
    finally:
        if link is not None:
            for _ in range(20):
                try:
                    send_thrust(link, 0)
                except Exception:
                    break
                time.sleep(0.02)
            try:
                # Generic STOP command, followed by both disarming formats.
                send_packet(link, CRTPPort.COMMANDER_GENERIC, 0, (0,))
                send_arming(link, False)
            except Exception:
                pass
            link.close()


if __name__ == "__main__":
    raise SystemExit(main())
