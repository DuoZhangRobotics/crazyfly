# Crazyflie + OptiTrack Swarm: Status and Implementation Plan

Last updated: 2026-08-28  
Repository: <https://github.com/DuoZhangRobotics/crazyfly>  
Target: Control five or more Crazyflie 2.0 drones using OptiTrack feedback and Crazyswarm2.

## 1. Objective

Build a reproducible research codebase that:

- receives Crazyflie poses from OptiTrack/Motive over NatNet;
- forwards external poses to the Crazyflie state estimators;
- supports safe takeoff, landing, `goTo`, and synchronized trajectories;
- controls at least five Crazyflies through one or more Crazyradio PA dongles;
- records pose, command, battery, link, and experiment data;
- provides emergency stop, tracking-loss handling, geofencing, and staged validation.

## 2. Selected architecture

The selected production architecture is:

```text
OptiTrack cameras
        |
        v
Motive on Windows
        |
        | NatNet/UDP over wired Ethernet
        v
Crazyswarm2 + ROS 2 on native Ubuntu 24.04
        |
        | USB
        v
Crazyradio PA
        |
        | CRTP radio, preferably 2M with unique addresses
        v
Crazyflie 2.0 fleet
```

Use ROS 2 Jazzy on Ubuntu 24.04. Do not use the Windows Motive machine as the
flight-control computer. Avoid WSL and virtual machines for physical flight due
to USB and latency complications.

## 3. Available systems

- **Development machine:** Apple Silicon MacBook.
- **Motion capture:** OptiTrack is available and Motive runs on Windows.
- **Flight-control machine:** A native Ubuntu 24.04 machine is available. A
  personal user account still needs to be created and validated.
- **Radio:** Crazyradio PA is available.
- **Aircraft:** Multiple Crazyflie 2.0 units are available.
- **Positioning decks:** No Flow deck was detected on the units tested so far.

## 4. Repository status

The repository has been initialized and pushed to GitHub.

- Branch: `main`
- Last known pushed project commit: `3169c66`
- Commit message: `Add Crazyflie diagnostics and flight test tools`
- Python syntax checks passed before that push.
- `.venv`, `.cache`, `__pycache__`, and bytecode are ignored.

Current scripts:

| File | Purpose | Production status |
| --- | --- | --- |
| `cf_diagnostics.py` | Read-only battery, attitude, deck, and `sys.canfly` checks | Bring-up tool |
| `inspect_usb_radio.py` | Reads channel, data rate, and address from configuration EEPROM over USB | Bring-up tool |
| `quick_radio_probe.py` | Fast acknowledgment check without downloading firmware tables | Bring-up tool |
| `flow_hover_test.py` | Short hover only when a Flow deck is detected | Experimental |
| `motor_spin_test.py` | Telemetry-guarded, below-liftoff motor test | Experimental |
| `quick_motor_spin.py` | Direct low-level motor spin that bypasses telemetry setup | Experimental/high risk |
| `brief_hop_test.py` | Short open-loop thrust pulse with telemetry cutoffs | Experimental/high risk |
| `quick_one_second_hop.py` | Direct one-second open-loop hop without telemetry | Experimental/high risk |

The existing flight scripts are hardware experiments. They are **not** the
production OptiTrack controller and must not be used as the basis for swarm
flight.

## 5. Verified hardware findings

### Unit using address `E7E7E7E707`

- URI: `radio://0/80/2M/E7E7E7E707`
- Radio and basic telemetry passed.
- No expansion deck detected.
- One tested battery suffered severe voltage sag under motor load and must not
  be used for flight.

### Unit using the default address

- Stored URI: `radio://0/80/250K/E7E7E7E7E7`
- The fast radio probe confirmed the unit responds at this URI.
- A full `cflib` connection at 250K stalled or emitted:
  `Address did not match when adding data to read request!`
- A direct open-loop hop at thrust `46000` reached the ceiling. The user found
  that approximately `35000` produced a much smaller hop.
- The vehicle drifted sideways because the open-loop script had no external
  position feedback. This is expected and is the reason OptiTrack feedback is
  required.

## 6. Firmware status and requirement

The tested Crazyflies reported CRTP protocol version `4`. Current supervisor
features use protocol `12+`; the library only operated through legacy fallbacks.

Before production Crazyswarm2 flight:

1. Update one spare/test Crazyflie first using the latest official CFclient.
2. Use **Connect -> Bootloader** and select the latest official release for
   platform `cf2`.
3. Flash through the Crazyradio, not through the Crazyflie USB connection.
4. Use a fully charged battery and do not interrupt the update.
5. Record the existing URI before flashing.
6. After flashing, power-cycle and verify LEDs, console output, telemetry, and
   supervisor state.
7. Give every Crazyflie a unique address and configure `2M` for swarm use.
8. Only after the first unit passes should the procedure be repeated for the
   remaining aircraft.
9. Update the Crazyradio PA firmware as recommended by the current Crazyswarm2
   installation documentation.

Suggested fleet addresses:

```text
cf1: radio://0/80/2M/E7E7E7E701
cf2: radio://0/80/2M/E7E7E7E702
cf3: radio://0/80/2M/E7E7E7E703
cf4: radio://0/80/2M/E7E7E7E704
cf5: radio://0/80/2M/E7E7E7E705
```

Do not change all aircraft at once. Maintain an inventory mapping the physical
label, Crazyflie address, OptiTrack name, firmware version, and battery ID.

## 7. OptiTrack/Motive plan

Windows is a dedicated Motive data server. Ubuntu receives tracking frames over
wired Ethernet.

Motive settings:

- enable NatNet streaming;
- start with multicast transmission;
- select the Ethernet interface connected to the Ubuntu machine;
- use Z-up coordinates;
- stream rigid bodies;
- stream unlabeled markers if using Crazyswarm2 frame-to-frame tracking;
- keep default NatNet ports: UDP 1510 command and UDP 1511 data;
- allow Motive/NatNet through Windows Firewall.

For the first implementation, prefer unique rigid bodies named exactly like the
Crazyswarm2 robot entries:

```text
cf1
cf2
cf3
cf4
cf5
```

Verify every rigid body's position and orientation in RViz with motors disabled
before forwarding pose data to any flight controller.

## 8. Ubuntu bootstrap plan

Create a normal personal Ubuntu account with sudo access. Do not run the flight
stack as root.

First collect:

```bash
whoami
lsb_release -ds
uname -m
sudo -v
ip -br address
```

Expected platform: Ubuntu 24.04, preferably `x86_64`.

Then:

1. Install ROS 2 Jazzy from the official ROS apt repository.
2. Install `ros-jazzy-desktop` and `ros-dev-tools`.
3. Add `/opt/ros/jazzy/setup.bash` to the user's shell startup.
4. Verify ROS using the standard talker/listener demo.
5. Install Crazyswarm2 and `motion_capture_tracking`.
6. Configure Crazyradio Linux USB permissions/udev rules.
7. Clone this repository under `~/ros2_ws/src/crazyfly`.
8. Turn this repository into a custom `ament_python` ROS 2 package rather than
   editing Crazyswarm2 itself.

## 9. Planned repository structure

The server implementation should move toward:

```text
crazyfly/
  config/
    crazyflies.yaml
    motion_capture.yaml
    server.yaml
    safety.yaml
  launch/
    swarm.launch.py
    mocap_only.launch.py
  crazyfly/
    __init__.py
    safety_monitor.py
    trajectory_runner.py
    experiment_logger.py
  scripts/
    takeoff_land.py
    single_drone_goto.py
    five_drone_trajectory.py
  tools/
    existing bring-up and experimental scripts
  test/
    configuration and trajectory tests
  package.xml
  setup.py
  setup.cfg
  README.md
```

Keep hardware-specific IP addresses and local interface names in untracked local
configuration or environment files. Commit example configuration files with safe
placeholders.

## 10. Milestones and acceptance criteria

### Milestone A: Ubuntu and ROS 2

- Ubuntu account works and has sudo access.
- ROS 2 Jazzy talker/listener works.
- Repository is cloned into the ROS workspace.

### Milestone B: OptiTrack data only

- Ubuntu can ping the Motive PC over wired Ethernet.
- `motion_capture_tracking` receives NatNet data.
- `/poses` publishes at a stable rate.
- `cf1` through `cf5` have correct positions, identities, axes, and orientations
  in RViz.
- Tracking loss is detectable before any motors are enabled.

### Milestone C: One updated Crazyflie

- One test unit has current `cf2` firmware.
- It uses a unique `2M` URI.
- Crazyswarm2 connects and reads battery/status.
- OptiTrack external pose agrees with the onboard estimate while the vehicle is
  moved by hand with propellers removed.

### Milestone D: Single-drone controlled flight

- Emergency stop works before takeoff.
- The flight volume and maximum altitude are configured.
- One drone takes off to 0.3-0.5 m, holds position, executes a small `goTo`, and
  lands.
- Tracking dropout and low-battery behavior are tested safely.

### Milestone E: Five-drone swarm

- All five aircraft have unique URIs, current firmware, healthy batteries, and
  correct rigid-body associations.
- All five can connect simultaneously without excessive packet loss.
- Sequential takeoff/landing passes before synchronized flight.
- A low-speed, well-separated formation trajectory passes.
- Experiment logs contain commanded and measured poses, battery, link state,
  tracking state, and emergency events.

## 11. Safety constraints for all future agents

- Do not run `quick_one_second_hop.py` indoors as a production test; it has
  already sent a Crazyflie into the ceiling.
- Do not disable emergency stop, link-loss handling, tracking-loss handling, or
  battery thresholds.
- Do not fly before OptiTrack identity and coordinate axes are verified in RViz.
- Use propellers-off tests for radio, firmware, pose, configuration, and estimator
  validation whenever possible.
- Start with one aircraft. Add additional aircraft only after the prior milestone
  passes.
- Keep people clear of the flight volume and establish a physical emergency-stop
  operator during initial tests.
- Inspect propellers and motor shafts after any ceiling or wall contact.
- Never assume a battery is healthy based only on resting voltage; validate sag
  under a controlled health test.

## 12. Information still needed

The server Codex should request or discover:

- Ubuntu username and whether sudo access is available;
- Ubuntu CPU architecture;
- Ubuntu Ethernet interface and IP;
- Motive PC IP and Motive/NatNet version;
- whether multicast is permitted on the lab switch;
- the selected tracking mode: Motive vendor rigid bodies or
  `librigidbodytracker`;
- the physical inventory of Crazyflies and batteries;
- current firmware and radio settings for every Crazyflie;
- Crazyradio PA firmware version;
- dimensions and coordinate origin of the approved flight volume.

## 13. Immediate next action

On the Ubuntu machine, create the user account and collect the five system-check
outputs in Section 8. Do not install or modify firmware until those results are
reviewed. After ROS 2 is verified, install Crazyswarm2 and validate OptiTrack data
with motors disabled before working on flight commands.
