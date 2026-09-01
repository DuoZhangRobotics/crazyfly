# Crazyflie + OptiTrack Swarm: Status and Next Plan

Last updated: 2026-08-30
Repository: <https://github.com/DuoZhangRobotics/crazyfly>  
Target: five or more Crazyflie 2.x vehicles using OptiTrack feedback and
Crazyswarm2 on ROS 2 Jazzy.

## 1. Current outcome

The repository safety and development foundation is implemented. The first
stationary physical Crazyswarm2 launch has now passed with one Crazyflie, the
Crazyradio PA, and live single-marker OptiTrack feedback. The safety gateway
reached `READY`, while `operator_enabled` and `commands_allowed` remained false.
No enable, takeoff, or motor command was sent, and nothing was flown.

Subsequent one-aircraft hover tests passed for radio addresses 01-05. The first
synchronized four-aircraft attempt failed when multiple single-marker tracks
were lost and two aircraft diverged laterally. A motors-off identity test then
confirmed the physical 01-04 mappings after swapping the positions of 03 and
04, and reproduced the tracker failure mode: after an occlusion, a surviving
marker can migrate to another rigid-body name unless the tracker is restarted.

The hardware gateway now requires the exact raw marker count, detects
implausible name-position jumps, and emergency-stops on a 0.10-second identity
loss. Four-aircraft execution defaults to staged one-at-a-time takeoffs, and
physical launches automatically preserve base-frame telemetry. Synchronized
takeoff requires an additional explicit flag.

Experiment logs preserve all raw marker coordinates in UR `base` whenever the
observed marker count differs from the reviewed expected count, allowing an
extra reflection or missing reconstruction to be localized after a run.

The pose-jump detector now measures displacement across a 50 ms Motive-stamped
history window. Callback arrival time remains dedicated to tracking-age
timeouts, preventing short ROS scheduling intervals from producing false
instantaneous-speed emergencies.

Unified pRRTC arm/drone coordination is implemented in this repository. The
one-command coordinator validates hashed main/park/drone artifacts, schedules a
shared monotonic start, executes the UR5e at 100 Hz, holds through drone
completion, parks the arm before drone return, and couples arm stop to drone
land-in-place abort. A complete simulated RTDE plus ROS/Crazyflie execution has
passed through return, landing, audit-ready logging, and cleanup. Physical
combined execution remains locked until clean repositories and a newly
replanned, fully inflated execution bundle are available.

The current combined-demo profile accepts planned UR joint speed up to pi rad/s
and piecewise acceleration up to 40 rad/s^2, with `servoJ` lookahead 0.03 s and
gain 1000. After the
physical installation correction, every arm sample receives `+pi/2` on the
first joint and planner-frame clearance checks subtract that same offset. After the
arm returns to its park/home configuration, normal drone return uses sorted
altitude lanes at 0.2 m intervals from 0.2 through 1.0 m, flies horizontally
over captured anchors, and lands at no more than 0.2 m/s. The one-drone circle schema-v2
bundle passes dry validation and the complete simulated run with a 9.6 ms start
skew. Missing offline recovery/measurement evidence requires an explicit,
logged combined-demo policy acknowledgement; live safety gates remain active.

Coordinated playback slowdown is available through either
`--playback-timescale` or `--maximum-arm-speed-rad-s`. The resolved duration
multiplier is applied to the UR main/park timestamps, Crazyflie onboard
trajectory timescale, endpoint-hold timeout, reference tracking, and mock
clock. The circle-one 0.5 rad/s cap resolves to 2.905464x duration (20.048 s
main arm motion and 22.227 s through the drone brake).

The synchronized hover and the all-four staged sequence have subsequently
passed. Atomic coordinated movement is implemented with two modes: an 8 cm
formation translation-and-return and a four-step cyclic position permutation.
Both use UR-base targets, exact continuous-path separation validation,
all-estimator readiness, 0.5-second dwell, target-error verification, and
automatic landing on failure.

The synchronized 8 cm hardware translation-and-return passed on 2026-08-30.
Both atomic batches were accepted, outbound and return target errors were at
most 2.5 cm and 3.1 cm, respectively, and measured separation remained at
least 30.7 cm. All four landings and disarms completed with no safety event.
The first attempt exposed and safely stopped on a battery-warning logic error:
the warning threshold had blocked a new motion command after takeoff. Battery
warning is now a preflight gate only; the configured critical threshold remains
the in-flight controlled-landing threshold.

The four-step cyclic permutation also passed after restoring each physical
radio to its configured single-marker table position. It exposed the restart
identity constraint: after an interrupted permutation, restarting the tracker
without restoring physical 01-04 causes marker names to be reassigned by
location. Successful missions therefore return to captured launch positions,
and incomplete mission logs require an identity recheck.

Version-2 timed trajectory missions are implemented. Shared-time UR-base
waypoints compile to continuously validated degree-7 minimum-snap polynomials,
upload while disarmed, move atomically to their starting points, broadcast one
absolute start, report tracking metrics, return to launch, and land. Automatic
and external Trigger start modes both pass complete hardware-free executions.

The first physical four-drone polynomial mission passed on 2026-08-30. All four
drones took off, assembled a 35 cm-spaced line, executed the 12-second smooth
4 cm formation box, returned to their captured launch positions, landed, and
disarmed without a safety event. Mean tracking error was 1.0-1.7 cm, maximum
error was 3.0-3.5 cm, and measured minimum separation was 32.9 cm. All 3,509 raw
point-cloud samples contained exactly four markers. Minimum loaded battery
voltage was 3.39 V. Motion-start skew is intentionally reported as unavailable
for this 0.029 m/s path because onset cannot be distinguished reliably from
position noise at that speed.

A larger 30 cm by 16 cm, 16-second figure-eight then passed cleanly after fixing
the mission ordering so tracking reports are computed only after return and
landing. Any non-safety failure now reacquires fresh poses and makes up to three
atomic return-to-launch attempts before fallback landing. The clean run returned
all drones to the tabletop and landed without a safety event; mean tracking
error was 1.4-2.3 cm, maximum error was 3.4-4.8 cm, and minimum separation was
32.7 cm. Five transient extra-marker samples were recorded, all about 4.6-4.7 cm
from cf2, indicating a local reflection or split reconstruction.

pRRTC compiled-polynomial payload ingestion is now implemented without
waypoint refitting. The mission runner preserves the embedded mission and
trajectory IDs, reuses the gateway's exact continuous validation, and remains
dry-run by default. Experiment manifests hash the fleet, safety profile,
accepted calibration, and original payload, whose bytes are copied into the run
directory. The exact one-drone pRRTC circle payload completed the hardware-free
mock takeoff, preposition, synchronized trajectory, return, landing, and cleanup
workflow. The unchanged five-drone payload is deliberately blocked by the
reviewed negative-Y geofence and is not ready for physical execution.

The selected-fleet hover path now derives every subscription, service client,
staged step, synchronized batch, and movement cycle from the enabled fleet, so
it can include `cf5`. Separate five-drone box and figure-eight examples compile
with 35 cm planned spacing. Five-aircraft physical execution still requires a
measured `cf5` starting position, an exact five-marker safety profile, and
staged identity validation.

## 2. Installed environment

- Host: Ubuntu 24.04, x86_64.
- ROS 2 Jazzy: installed under `/opt/ros/jazzy`.
- Crazyswarm2 1.0.3 and `motion_capture_tracking` 1.0.6 are installed as
  official ROS Jazzy packages under `/opt/ros/jazzy`.
- Fast-CDR, Fast-DDS, ROSIDL, and RMW packages were upgraded to the compatible
  versions supplied by the current ROS repository.
- uv 0.12.7 manages `/home/duo/ros2_ws/.venv`, based on Python 3.12 with
  read-only access to system ROS packages. Rowan 1.3.2 is installed only there.
- The project is symlink-built under `/home/duo/ros2_ws`; its installed Python
  executables use `/home/duo/ros2_ws/.venv/bin/python`.
- The official Bitcraze udev rule is installed, and the user is in `plugdev`.
- Activation helper: `tools/activate_ros.sh`; it loads ROS 2, uv, Crazyswarm2,
  and the installed project package. The old userspace overlay is inactive and
  retained only as a recovery fallback.
- Free disk space after installation: approximately 2.9 GB.

This environment is functional for development but remains machine-specific.
Additional free disk space is recommended before broad system upgrades or
building the optional upstream simulator. That simulator still lacks
`cffirmware`; the repository's mock stack does not require it.

## 3. Implemented repository

### ROS package and configuration

- `package.xml`, `setup.py`, `setup.cfg`, and the ament resource marker.
- Production fleet template for `cf1` through `cf5`; every robot is disabled.
- OptiTrack/Motive template with a required hostname placeholder.
- Production safety template with physical flight disabled and no implicit
  geofence.
- Separate, clearly named mock fleet and mock safety files.
- Declarative synchronized waypoint format and example mock square.

### Safety gateway

The gateway owns project takeoff, landing, and `go_to` requests. It validates
fresh tracking/status, battery, supervisor state, geofence, command speed,
takeoff height, and robot separation. It starts disabled and requires a stable
one-second healthy interval before enabling.

Tracking-loss behavior is staged:

1. At 0.10 s, reject new commands.
2. At 0.25 s, request a controlled landing.
3. At 1.00 s, invoke the upstream emergency stop.

Tumble, crash, supervisor lock, hard geofence breach, and live separation violation
invoke the emergency stop immediately.

Public project interfaces:

- `/crazyfly/preflight` (`std_srvs/Trigger`)
- `/crazyfly/enable` (`std_srvs/SetBool`)
- `/crazyfly/emergency` (`crazyflie_interfaces/Stop`)
- `/crazyfly/<robot>/takeoff`
- `/crazyfly/<robot>/land`
- `/crazyfly/<robot>/go_to`
- `/crazyfly/batch_go_to_requests`
- `/crazyfly/trajectory_upload_requests`
- `/crazyfly/trajectory_start_requests`
- `/crazyfly/mission/start` (available while a mission waits externally)
- `/crazyfly/commands`
- `/crazyfly/safety/state`
- `/crazyfly/safety/diagnostics`

### Launch separation

- `mock_stack.launch.py` starts only the hardware-free mock and safety gateway.
- `mocap_only.launch.py` starts no node unless `allow_network:=true`; it never
  starts Crazyswarm2 or a radio server.
- `swarm.launch.py` defaults `allow_hardware:=false`. When enabled, it validates
  the fleet, safety bounds, initial separation, 2M URIs, and real Motive address
  before the upstream server starts. A reviewed local file must also explicitly
  set `flight_enabled: true`.

### Experiment tooling

- Trajectories are dry-run only unless `--execute` is supplied.
- Exact pRRTC degree-7 JSON payloads are accepted through
  `--compiled-payload` without waypoint recompilation.
- The runner sends commands only to `/crazyfly/...` services.
- The logger writes an event stream, copies the original trajectory source, and
  records fleet, safety, calibration, and trajectory SHA-256 hashes.
- Accepted, rejected, safety-land, and emergency actions are published on the
  command event topic and included in JSONL/rosbag output.
- Optional rosbag recording refuses to start below 1 GB free and has a maximum
  duration.
- Old direct-cflib scripts moved to `tools/legacy/` with risk documentation.

## 4. Verification completed

The following passed on this machine:

- Python compilation for application, launch, and test modules.
- 33 automated tests covering safe defaults, invalid configurations, Motive
  placeholders, duplicate radios, recovery time, battery, tumble, command
  limits, geofence, separation, trajectories, and architecture boundaries.
- Clean `colcon` build in an isolated workspace.
- ROS package discovery and parsing of all three launch descriptions.
- Live mock preflight and enable.
- Live mock takeoff to 0.30 m over two seconds.
- Simulated tracking loss: controlled-land request at 0.25 s and emergency stop
  at 1.00 s.
- Fail-closed launch behavior remains intact with committed files and with the
  ignored local safety file restored to `flight_enabled: false`.
- Motive 2.0 at `172.16.90.213` publishes NatNet 3.0 over wired multicast. The
  single marker is assigned to `cf1`, axes are correct, and static noise is below
  0.02 mm per axis at approximately 120 Hz.
- A brief 42 ms marker dropout recovered. An occlusion longer than about 1.1 s
  required restarting the tracker; automatic reacquisition remains a known
  single-marker limitation.
- Crazyflie firmware 2026.08 (`54f31e243a0b`, CLEAN) and the 2M radio URI were
  validated. A 30-second radio test and a motor-free estimator injection test
  passed; the settled position error was approximately 2.5 mm or less per axis.
- The stationary full stack connected with battery 3.77 V, live pose and status,
  and supervisor flags `526`. Read-only preflight passed after correcting the
  auto-arm compatibility check; the command gate remained disabled.
- The project-owned Crazyswarm2 server configuration accepts 80-180 Hz, covering
  the nominal 120 Hz stream and measured network-arrival jitter.
- `motion_capture_tracking` still exits with a Boost interrupted-system-call
  error on Ctrl+C, after the other nodes shut down cleanly.

## 5. Selected physical architecture

```text
OptiTrack cameras
        |
        v
Motive on Windows
        |
        | NatNet/UDP over wired Ethernet
        v
Crazyswarm2 + safety gateway on native Ubuntu 24.04
        |
        | USB
        v
Crazyradio PA
        |
        | CRTP, 2M, unique address per vehicle
        v
Crazyflie fleet
```

Motive remains a tracking server. Physical control stays on native Ubuntu; do
not move the flight stack into WSL or a virtual machine.

## 6. Physical information still required

Confirmed for `cf1`: Motive and Ubuntu addressing, wired multicast, Z-up frame,
measured geofence, marker assignment, initial position, current firmware, 2M
radio URI, Crazyradio visibility, battery telemetry, and stationary estimator
convergence.

Still required before scaling beyond one aircraft:

- Label, unique radio URI, marker mount, measured initial position, firmware, and
  battery inventory for each additional vehicle.
- A repeatable procedure for restarting single-marker tracking after a long
  occlusion.
- Battery validation under controlled load; resting voltage alone is not enough.
- UR5e-to-Motive frame calibration before coordinated robot-drone work.

## 7. Next physical bring-up plan

### Stage A: host setup completed

The official Crazyswarm2 and motion-capture packages, matching middleware,
Bitcraze udev rule, uv environment, permanent workspace build, and automated
tests are installed. Free additional disk space before any broad OS upgrade or
optional simulator build.

### Stage B: OptiTrack-only validation completed

The wired NatNet stream, Z-up world frame, geofence, single-marker assignment,
identity, rate, static noise, hand motion, and dropout behavior were measured. No
Crazyflie server ran during the isolated motion-capture tests. Long occlusion
reacquisition remains procedural: restart the tracker after restoring visibility.

### Stage C: one stationary aircraft completed

The firmware, 2M radio, telemetry, external-position injection, estimator
convergence, full ROS stack, and fail-closed safety gateway were validated on
`cf1`. Props remained installed, so no motor, enable, takeoff, land, or emergency
command was issued. Read-only preflight passed and the command gate stayed off.

### Stage D: one controlled flight

1. Verify the 80-180 Hz diagnostic range clears the mocap-rate warning.
2. Use a fully charged, validated battery and repeat stationary preflight.
3. Use one vehicle, low speed, 0.30 m takeoff, and a clear netted volume.
4. Assign a dedicated physical emergency-stop operator.
5. Pass takeoff/hover/short `go_to`/land before testing fault behavior.
6. Review logs and battery sag after every run.

### Stage E: scale to five

Add one aircraft at a time. Require a unique URI, current firmware, correct
single-marker identity, healthy battery, stable simultaneous links, and
sequential takeoff/landing before synchronized trajectories. Begin well above
0.40 m separation and commands no faster than 0.25 m/s.

### Stage F: onboard polynomial trajectories

1. Repeat `four_drone_box.yaml` twice more with measured separation at least
   20 cm, mean error at most 3 cm, maximum error at most 8 cm, and no safety
   event.
2. Repeat with the powered UR5e stationary outside the reviewed flight paths.
3. Only then add an RTDE coordinator that releases `/crazyfly/mission/start`.

## 8. Non-negotiable constraints

- Never bypass `/crazyfly/...` with project flight code.
- Never enable all aircraft at once during bring-up.
- Never fly before Motive identity and axes are verified.
- Prefer propellers-off tests for firmware, radio, pose, estimator, and fault
  validation.
- Do not weaken tracking-loss, battery, geofence, separation, or supervisor
  checks to make a test pass.
- Do not use the archived open-loop hop scripts indoors.
- Keep people clear, wear eye protection, and inspect vehicles after contact.
- Validate batteries under controlled load; resting voltage is insufficient.

## 9. Known boundary

The gateway is an application safety boundary, not a ROS security policy. An
operator can still call upstream Crazyswarm2 services directly if they choose
to bypass it. Repository tests prevent application modules from importing the
direct Crazyswarm API, and all documented commands use the gateway. A lab
operating procedure must enforce the same boundary for interactive commands.
