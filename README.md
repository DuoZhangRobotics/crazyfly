# Crazyflie 2.0 first-flight checks on macOS

The safe order is:

1. Verify startup, radio, telemetry, battery, and motor/propeller health.
2. Fly manually with the official Crazyflie client and a four-axis gamepad.
3. Only use the scripted hover if a Flow deck is fitted and detected.

The stock Crazyflie 2.0 has attitude stabilization, but no horizontal position
sensor. A scripted `MotionCommander` hover therefore requires a Flow deck (V1
or V2) or another suitable positioning system.

## What has already been checked

This Mac is Apple Silicon, runs macOS 15.3, and reports the connected USB device
as `Crazyradio PA USB Dongle` (Bitcraze, USB vendor `0x1915`, product `0x7777`).

## 1. Install the Python environment

This Mac's Python is 3.14. The current `cflib` release documents support through
Python 3.13, so the commands below deliberately use Python 3.13 through `uv`.

```sh
uv sync --python 3.13
```

The installed `cflib` includes its USB runtime. If macOS still reports a USB
backend error, install the system fallback with `brew install libusb`. Do not
use `sudo pip`.

## 2. Power-on check

- Remove the propellers for connection and telemetry work.
- Connect a charged battery and place the Crazyflie flat and absolutely still.
- Switch it on and wait for calibration.
- Normal ready indication is both blue LEDs continuously on and the front-right
  red LED blinking twice per second. Five short red pulses repeating indicates a
  failed self-test.

## 3. Run the no-motor diagnostic

Make sure the Crazyflie client is closed so it does not hold the radio, then run:

```sh
uv run python cf_diagnostics.py
```

The script scans for the aircraft, connects, reports attached decks, samples the
battery and attitude, and checks `sys.canfly`. It never arms or starts motors.
Save the URI that it prints; it may look like:

```text
radio://0/80/2M/E7E7E7E7E7
```

To bypass scanning later:

```sh
uv run python cf_diagnostics.py --uri radio://0/80/2M/E7E7E7E7E7
```

## 4. Check motors and propellers in the official client

Start the current official client with:

```sh
uvx --python 3.13 cfclient
```

Then:

1. Press **Scan**, select the URI, and connect.
2. Open the **Console** tab and resolve any self-test, calibration, deck, or
   battery errors.
3. With the aircraft restrained as directed by the client and the area clear,
   use the client's **Propeller test**. Do not improvise a thrust-ramp script.
4. Confirm each propeller is undamaged, fully seated, in the correct numbered
   position, and produces downward airflow.

## 5. First real flight (recommended)

Use a gamepad with at least four analog axes. The client is not designed for
keyboard piloting.

- Choose a large, uncluttered indoor area with no fans, pets, or people nearby.
- Wear eye protection and keep fingers and faces away from the propellers.
- Verify the controller mapping before connecting and make certain thrust reads
  zero.
- Refit the propellers, put the Crazyflie level on the floor, scan, connect, and
  start with a very small, brief lift. Keep the emergency-stop control ready.

## 6. Optional scripted hover (Flow deck only)

Do this only after the diagnostic and a successful manual flight. Use a clean,
textured, well-lit floor; the downward-facing optical-flow sensor performs poorly
over glossy, uniform, transparent, or very dark surfaces.

```sh
uv run python flow_hover_test.py --confirm FLY
```

The program refuses to fly unless firmware reports `bcFlow` or `bcFlow2`. It
takes off to 0.30 m, hovers for 3 seconds, lands, sends a stop setpoint, and
disarms. Press **Ctrl-C** to land early.

Do not run this script on a stock Crazyflie 2.0 without a Flow deck.

## Guarded open-loop hop (no positioning deck)

`brief_hop_test.py` is intentionally separate from the controlled hover test.
It cannot guarantee height: it applies one short, capped thrust pulse and then
sends repeated zero/stop commands and disarms. It aborts its pulse if measured
tilt exceeds 20 degrees or battery voltage falls below 3.30 V. The stock
barometer is recorded but is too sensitive to pressure changes and prop wash to
act as a short-hop height cutoff. These checks are not a substitute for a
positioning deck.

```sh
uv run python brief_hop_test.py --confirm HOP
```

## Troubleshooting

- **Dongle is visible but scan finds nothing:** power-cycle the Crazyflie on a
  level surface, close any other client using the radio, move it within 1 m, and
  scan again with the default address `0xE7E7E7E7E7`.
- **USB/backend error:** verify `libusb` is installed, unplug/replug the
  Crazyradio PA, and rerun the diagnostic.
- **`sys.canfly` is false:** inspect the client Console tab. Do not try to bypass
  the supervisor check.
- **Old firmware or connection oddities:** use **Connect > Bootloader** in the
  client and flash the current official release for platform `cf2`. Keep the
  battery connected and do not interrupt flashing.
