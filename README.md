# Crazyfly OptiTrack swarm controller

This repository is now a ROS 2 Jazzy package for controlling Crazyflie 2.x
vehicles through Crazyswarm2 with OptiTrack/Motive feedback. Every project flight
command passes through a safety gateway; the committed physical-flight settings
are deliberately disabled.

Current state on this Ubuntu machine:

- ROS 2 Jazzy is installed system-wide.
- Crazyswarm2 1.0.3 and motion_capture_tracking 1.0.6 are installed system-wide
  under `/opt/ros/jazzy`.
- Project-only Python packages live in the uv environment at
  `/home/duo/ros2_ws/.venv`; nothing is installed into system Python.
- The package is installed in `/home/duo/ros2_ws/install`; all automated tests
  pass.
- The hardware-free mock stack passes preflight, enable, takeoff, controlled
  landing on tracking loss, and emergency stop.
- No Crazyradio is currently visible. No physical aircraft was contacted,
  armed, or flown during implementation.

See `STATUS_AND_PLAN.md` for the lab bring-up checklist and remaining work.

## Safety model

The gateway starts `DISABLED`. It must observe fresh OptiTrack poses, fresh
Crazyflie status, healthy battery/supervisor state, valid separation, and a
continuous one-second recovery interval before it will enable commands.

| Guard | Default |
| --- | ---: |
| Reject commands when pose age reaches | 0.10 s |
| Request controlled landing when pose age reaches | 0.25 s |
| Emergency stop when pose age reaches | 1.00 s |
| Status stale timeout | 2.50 s |
| Battery warning / critical | 3.8 V / 3.7 V |
| Maximum takeoff height | 0.50 m |
| Maximum commanded speed | 0.25 m/s |
| Minimum robot separation | 0.40 m |

The warning voltage blocks preflight. Once a flight has started, crossing the
warning voltage does not reject subsequent motion commands; reaching the
critical voltage requests a controlled landing.

A tumble, supervisor lock, hard geofence breach, or live separation violation
causes an immediate emergency stop. The committed `config/crazyflies.yaml` has
all robots disabled, `config/safety.yaml` has `flight_enabled: false`, and
`config/motion_capture.yaml` contains a non-routable placeholder.

The upstream Crazyswarm2 services still exist when its server is running. Code
in this repository must use only `/crazyfly/...` command services; directly
calling `/cf1/...` or `/all/...` bypasses the project gateway.

## Repository layout

- `crazyfly/`: configuration validation, pure safety state machine, ROS gateway,
  mock stack, trajectory runner, and experiment logger.
- `config/`: fail-closed production templates plus isolated mock settings.
- `launch/mock_stack.launch.py`: hardware-free integration environment.
- `launch/mocap_only.launch.py`: OptiTrack reception only; never starts a radio
  server.
- `launch/swarm.launch.py`: hardware-gated Crazyswarm2 and safety gateway.
- `test/`: mock-only tests for configuration, command limits, trajectories, and
  staged fault handling.
- `tools/legacy/`: archived direct-cflib bring-up and flight experiments. These
  are not production controllers.

## Activate the installed ROS environment

Every new shell can load ROS 2, the system Crazyswarm2 packages, the uv
environment, and the installed project package with one command:

```sh
source /home/duo/crazyfly/tools/activate_ros.sh
```

Read the resting battery voltage of every enabled drone with one safe command:

```sh
/home/duo/crazyfly/tools/battery_status
```

The command reads the configured radio addresses sequentially, never arms a
drone, reports offline aircraft without hiding the others, and uses the warning
and critical thresholds from the reviewed local safety profile. Its dedicated
environment can be reproduced without touching system Python using:

```sh
uv pip install --python /home/duo/crazyfly/.venv/bin/python \
  -r /home/duo/crazyfly/tools/battery-requirements.txt
```

The helper activates `/home/duo/ros2_ws/.venv` and routes `colcon` through that
interpreter, ensuring rebuilt project executables keep a virtual-environment
shebang. To use another uv environment:

```sh
export CRAZYFLY_VENV=/absolute/path/to/.venv
source /path/to/crazyfly/tools/activate_ros.sh
```

The former no-sudo Crazyswarm2 overlay is inactive and retained only as an
automatic recovery fallback if the system packages are unavailable.

## Build in a ROS workspace

Place this repository under a workspace `src` directory, then build it:

```sh
cd /path/to/ros2_ws
source src/crazyfly/tools/activate_ros.sh
colcon build --symlink-install --packages-select crazyfly
source install/setup.sh
```

Run the tests from the repository at any time:

```sh
python -m pytest -q
```

## Hardware-free workflow

Start the mock OptiTrack publisher, mock Crazyswarm2 services, and real safety
gateway:

```sh
ros2 launch crazyfly mock_stack.launch.py
```

In a second activated shell, wait at least one second for the health recovery
interval and then exercise only the mock vehicle:

```sh
ros2 service call /crazyfly/preflight std_srvs/srv/Trigger '{}'
ros2 service call /crazyfly/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /crazyfly/cf1/takeoff crazyflie_interfaces/srv/Takeoff \
  '{group_mask: 0, height: 0.3, duration: {sec: 2, nanosec: 0}}'
```

Simulate tracking loss:

```sh
ros2 service call /mock/drop_tracking std_srvs/srv/SetBool '{data: true}'
```

The gateway requests a controlled landing after 0.25 seconds and calls the
emergency service after 1.0 second. Restore the mock and disable the gateway:

```sh
ros2 service call /mock/drop_tracking std_srvs/srv/SetBool '{data: false}'
ros2 service call /crazyfly/enable std_srvs/srv/SetBool '{data: false}'
```

Compile and continuously validate the version-2 timed waypoint example without
starting ROS or sending commands:

```sh
python tools/trajectory_mission.py config/trajectories/mock_square.yaml --mock
```

Run the complete takeoff, preposition, onboard polynomial, return, and landing
against the hardware-free mock stack:

```sh
python tools/trajectory_mission.py config/trajectories/mock_square.yaml \
  --mock --execute
```

The installed `crazyfly_trajectory` and `crazyfly_trajectory_mission` commands
are aliases for the same CLI. The legacy `--trajectory FILE` spelling remains
accepted.

pRRTC may instead provide an exact compiled polynomial payload. Validate it
without ROS, radio access, or waypoint recompilation with:

```sh
python tools/trajectory_mission.py \
  --compiled-payload /home/duo/pRRTC/dataset/ur5e_crazyflie_experiment/crazyfly_payloads/circle_5_seed_1.json \
  --fleet /absolute/path/to/five_drone_crazyflies.yaml \
  --safety config/local/safety.yaml
```

The payload's mission ID, trajectory ID, shared segment durations, and eight
ascending-power coefficients are authoritative. The enabled fleet must exactly
match the payload robot set. `--trajectory-id` is therefore accepted only for
waypoint missions. Hardware execution still requires the separate `--execute`
flag and every normal safety gate.

### Unified pRRTC UR5e + Crazyflie demo

Install the sibling `ur_tools` package and measured-clearance dependency into
the same ROS-aware uv environment:

```sh
uv pip install --python /home/duo/ros2_ws/.venv/bin/python \
  -r tools/ur-demo-requirements.txt
```

The combined bundle must contain exact Crazyflie polynomials, timed UR5e main
and park paths, validation evidence, and a manifest with hashes, clean pRRTC
state, hardware name mapping, source scene/result paths, and positive measured
calibration/error/latency fields. Dry validation starts no ROS or RTDE process:

```sh
crazyfly_prrtc_demo /absolute/path/to/execution_bundle
```

Physical execution is separately gated by the robot IP and clean `crazyfly`,
`ur_tools`, and pRRTC provenance:

```sh
crazyfly_prrtc_demo /absolute/path/to/execution_bundle \
  --execute --confirm-robot-ip 172.16.90.197
```

Slow both systems together by supplying either a duration multiplier or an arm
speed cap. Never slow only one side of a synchronized pRRTC plan:

```sh
crazyfly_prrtc_demo /absolute/path/to/execution_bundle \
  --maximum-arm-speed-rad-s 0.5

crazyfly_prrtc_demo /absolute/path/to/execution_bundle \
  --playback-timescale 2.0
```

The current circle-one bundle resolves the 0.5 rad/s cap to a 2.905464x
duration multiplier: 20.048 s of synchronized main motion, 22.227 s through
the drone braking endpoint, and an 11.622 s arm park. Only timescales at least
1.0 are allowed; this interface cannot speed a validated plan up.

The UR trajectory validator uses the lab-wide limits `pi rad/s` and
`40 rad/s^2`; the RTDE executor uses `servoJ` at 100 Hz with lookahead 0.03 s
and gain 1000. The physical UR5e installation adds `+pi/2` to the first
(`shoulder_pan_joint`) coordinate of every main and park sample; the clearance
model removes the same offset before evaluating planner-frame geometry. These
values can still be overridden explicitly on the command line. The combined
demo accepts missing offline return/abort corridors and
measured arm/latency evidence by default, records that policy in the experiment
manifest, and keeps all live flight and RTDE checks enabled.

The coordinator owns the Crazyflie subprocess and RTDE connection. It
prepositions both systems, schedules one monotonic start two seconds ahead,
holds the arm goal through drone completion, executes the validated arm park
path, and only then releases drone return. Normal return assigns sorted drones
to 0.2, 0.4, 0.6, 0.8, and 1.0 m lanes, moves them horizontally over their
captured anchors, and lands directly with descent durations capped at 0.2 m/s.
The route
is rejected if the effective geofence cannot contain the lanes or the captured
anchors violate live separation. Coordinated abort stops the arm and lands
drones vertically in place. Arm-only validation/playback is available as
`crazyfly_ur_trajectory`; both commands are dry-run by default.

Combined logs under `combined_experiments/` freeze the bundle and record
repository/calibration hashes, scheduled and measured starts, UR joint/TCP
samples, drone command/measurement samples, URDF-based measured clearance, and
the pRRTC hardware audit result.

Version-2 files use shared absolute times and UR-base goals for every enabled
drone. The compiler creates degree-7 minimum-snap pieces with zero endpoint
velocity, acceleration, and jerk and continuous derivatives through fly-through
waypoints. Yaw must remain zero while single-marker tracking is used.

The gateway re-parses and continuously validates the compiled polynomials before
uploading them. The configured limits are 0.25 m/s speed, 0.50 m/s² acceleration,
2.0 m/s³ jerk, 60 seconds duration, the soft geofence, and a 0.40 m planned
separation in the generic profile. The reviewed local lab profile currently
uses 0.15 m for both planned and live separation. Position extrema and pairwise
separation are checked between waypoints as well as at them.

For a future arm coordinator, prepare a mission with:

```sh
python tools/trajectory_mission.py TRAJECTORY.yaml --execute --wait-for-start
```

After `mission_ready`, release it from another activated process with:

```sh
ros2 service call /crazyfly/mission/start std_srvs/srv/Trigger '{}'
```

The mission waits at the trajectory starts, broadcasts one absolute
`/all/start_trajectory` only after the Trigger, then returns every drone to its
captured launch position before landing. The conservative reviewed four-drone
dry-run example is `config/trajectories/four_drone_box.yaml`: it first assembles
a 35 cm-spaced line, executes a smooth 4 cm box as a formation, returns to the
line, then returns to launch. Its first physical execution passed on 2026-08-30;
repeatability testing remains required before involving the UR5e.

Five-drone software-validation counterparts are provided as
`config/trajectories/five_drone_box.yaml` and
`config/trajectories/five_drone_figure8.yaml`. They include `cf5` in the same
35 cm-spaced formation. Do not execute them physically until `cf5` has a
measured local starting position, five identities have passed staged bring-up,
and the reviewed safety profile expects exactly five raw markers.

## OptiTrack-only workflow

Copy `config/motion_capture.yaml` and `config/crazyflies.yaml` to ignored
`config/local/`. Replace `MOTIVE_PC_IP_REQUIRED` with the reviewed Motive PC
address, enable only the marker being tested, and set its measured starting
position. Keep motors and radio disconnected. The default launch performs no
network access:

```sh
ros2 launch crazyfly mocap_only.launch.py
```

Start NatNet reception only after reviewing the local file:

```sh
ros2 launch crazyfly mocap_only.launch.py \
  allow_network:=true \
  fleet_config_file:=/absolute/path/to/config/local/crazyflies.yaml \
  motion_capture_yaml_file:=/absolute/path/to/config/local/motion_capture.yaml
```

Verify names, axes, position update rate, and tracking-loss behavior in RViz
before proceeding to estimator or motor work. Single-marker poses intentionally
have no external orientation.

## Physical-hardware gate

Physical launch requires all of these explicit local changes:

1. A local fleet file with reviewed, unique 2M URIs and only the intended robot
   enabled.
2. A local safety file with `flight_enabled: true` and measured geofence bounds.
3. A local motion-capture file with the real Motive address.
4. The command-line gate `allow_hardware:=true`.
5. An exact reviewed `tracking.expected_raw_marker_count` for the current
   single-marker volume.

The launch validates all three files before starting Crazyswarm2. Use absolute
paths so there is no ambiguity:

```sh
ros2 launch crazyfly swarm.launch.py \
  allow_hardware:=true \
  fleet_config_file:=/absolute/path/to/config/local/crazyflies.yaml \
  safety_config_file:=/absolute/path/to/config/local/safety.yaml \
  motion_capture_yaml_file:=/absolute/path/to/config/local/motion_capture.yaml \
  server_config_file:=/absolute/path/to/config/server.yaml
```

A stationary one-aircraft launch has been validated with live OptiTrack and radio
telemetry. No enable, takeoff, or motor command was sent. Keep the local safety
file at `flight_enabled: false` between approved tests. The project server
configuration accepts mocap arrival rates from 80-180 Hz to accommodate the
nominal 120 Hz stream. Before the first flight, repeat preflight and use a clear
controlled volume with a dedicated emergency-stop operator.

Single-marker identities are initialized from configured starting positions;
the markers do not contain radio-address identity. Keep only the expected
unlabeled markers in the volume. A persistent raw-marker count change, a
missing named pose, or an implausible pose jump is treated as an identity fault
before another marker can be reassigned to that drone.

Pose-jump speed uses `tracking.pose_speed_window_s` seconds of source-stamped
Motive history instead of two callback arrival times. Monotonic receipt time is
still used independently for stale-pose timeouts. The local profile uses a
50 ms window, which filters ROS scheduling jitter while retaining rapid marker
swap detection.

The one-drone helper accepts address suffixes `01` through `05` and requires
exactly one visible marker:

```sh
source tools/activate_ros.sh
python tools/one_drone_hover.py 05 --execute
```

The all-drone helper derives its robot set from the enabled entries in the fleet
file, including `cf5`. For more than one enabled aircraft it defaults to staged
validation with all selected trackers active but only one aircraft flying at a
time. The safety profile's
`tracking.expected_raw_marker_count` must equal the enabled fleet size:

```sh
source tools/activate_ros.sh
python tools/all_drone_hover.py --execute
```

Only after the staged run and its logs pass, synchronized takeoff can be
requested explicitly:

```sh
python tools/all_drone_hover.py --execute --synchronized
```

The first synchronized translation test moves the full formation 8 cm along
UR-base +X over two seconds, dwells for 0.5 seconds, returns, and lands:

```sh
python tools/all_drone_hover.py --execute --synchronized --movement translate
```

After that passes, the cyclic permutation moves each drone to the next drone's
captured takeoff position over 2.5 seconds, dwells for 0.5 seconds, repeats once
per enabled drone, and lands:

```sh
python tools/all_drone_hover.py --execute --synchronized --movement cycle
```

Movement requests are submitted to the safety gateway as one atomic
base-frame batch. The gateway converts targets to Motive world, validates
geofence and speed limits, computes exact continuous pairwise separation for
all linear paths, checks every upstream service, and only then dispatches the
full batch. Every step verifies that all drones settled within 8 cm.

Successful execution now ends with an explicit `PASS: hover sequence completed`
line. `Cleanup complete; Crazyradio is free` is also a normal successful cleanup
message; it means the stopped ROS launch no longer owns the USB radio.

Polynomial missions use `/crazyfly/trajectory_upload_requests` and
`/crazyfly/trajectory_start_requests`. Upload is permitted only while disarmed;
the gateway prepares a mission only after every per-drone upload future
completes. Start requires a matching mission ID, a flying/healthy gateway, and
every drone within 5 cm of its compiled starting point.

## Experiment logging

The logger writes newline-delimited pose, status, safety, diagnostic, and command
events. Its manifest records absolute configuration paths and SHA-256 hashes.
Raw point-cloud events normally store only the marker count. Whenever that count
differs from the reviewed expected count, the event also stores every valid raw
marker position transformed from Motive `world` into the configured UR `base`
frame. This makes intermittent reflections spatially diagnosable without
inflating every normal frame.
Rosbag recording is optional and is refused when less than 1 GB is free:

```sh
ros2 run crazyfly crazyfly_experiment_logger --ros-args \
  -p fleet_config_file:=/absolute/path/to/crazyflies.yaml \
  -p safety_config_file:=/absolute/path/to/safety.yaml \
  -p output_root:=/absolute/path/to/experiments \
  -p maximum_duration_s:=300.0 \
  -p record_rosbag:=false
```

Generated `experiments/`, `log/`, `build/`, and `install/` data is ignored by
Git.

Physical swarm launches start this bounded logger automatically. Named mocap
positions, onboard position estimates, raw marker counts, safety state, status,
and command events are preserved under `experiments/`. Positions are converted
to the configured geofence frame, which is `base` in the local UR5e profile.
Mission events preserve the trajectory path and SHA-256, readiness/start times,
completion or identity-recheck status, mean/RMS/95th-percentile/maximum tracking
error, measured separation, battery minima, and inferred motion-start skew. The
logger also copies the original waypoint or compiled-payload bytes into the run
directory and records fleet, safety, accepted calibration, and trajectory
SHA-256 hashes in the manifest.
