#!/usr/bin/env python3
"""Read the Crazyflie configuration EEPROM over USB without modifying it."""

from __future__ import annotations

import sys
import threading

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.mem.memory_element import MemoryElement
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


RADIO_SPEEDS = {
    0: "250K",
    1: "1M",
    2: "2M",
}


def main() -> int:
    cflib.crtp.init_drivers(enable_debug_driver=False)
    cf = Crazyflie(rw_cache="./.cache")

    print("Connecting read-only to usb://0 ...")
    with SyncCrazyflie("usb://0", cf=cf) as scf:
        scf.wait_for_params()

        memories = scf.cf.mem.get_mems(MemoryElement.TYPE_I2C)
        if not memories:
            print("No configuration EEPROM was exposed by this firmware.", file=sys.stderr)
            return 1

        config = memories[0]
        finished = threading.Event()
        config.update(lambda _memory: finished.set())
        if not finished.wait(timeout=5.0):
            print("Timed out while reading the configuration EEPROM.", file=sys.stderr)
            return 1
        if not config.valid:
            print("The configuration EEPROM did not contain a valid checksum.", file=sys.stderr)
            return 1

        values = config.elements
        channel = int(values["radio_channel"])
        speed_number = int(values["radio_speed"])
        speed = RADIO_SPEEDS.get(speed_number, f"unknown({speed_number})")
        address = values.get("radio_address")

        print("Stored radio configuration")
        print(f"  EEPROM version: {values['version']}")
        print(f"  Channel:        {channel}")
        print(f"  Data rate:      {speed}")
        if address is None:
            print("  Address:        default (E7E7E7E7E7)")
            address_text = "E7E7E7E7E7"
        else:
            address_text = f"{int(address):010X}"
            print(f"  Address:        {address_text}")
        print(f"  Radio URI:      radio://0/{channel}/{speed}/{address_text}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
