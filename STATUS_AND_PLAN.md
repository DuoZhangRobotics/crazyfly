# Crazyflie + OptiTrack Swarm: Status and Next Plan

Last updated: 2026-08-28  
Repository: <https://github.com/DuoZhangRobotics/crazyfly>  
Target: five or more Crazyflie 2.x vehicles using OptiTrack feedback and
Crazyswarm2 on ROS 2 Jazzy.

## 1. Current outcome

The phone/no-sudo development phase is implemented. The repository is now an
`ament_python` ROS 2 package with a mandatory project safety gateway, isolated
mock environment, fail-closed physical launch, configuration and trajectory
validation, and bounded experiment logging.

Nothing physical was armed or flown. A Crazyradio is not currently visible on
this Ubuntu machine.

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

Tumble, supervisor lock, hard geofence breach, and live separation violation
invoke the emergency stop immediately.

Public project interfaces:

- `/crazyfly/preflight` (`std_srvs/Trigger`)
- `/crazyfly/enable` (`std_srvs/SetBool`)
- `/crazyfly/emergency` (`crazyflie_interfaces/Stop`)
- `/crazyfly/<robot>/takeoff`
- `/crazyfly/<robot>/land`
- `/crazyfly/<robot>/go_to`
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
- The runner sends commands only to `/crazyfly/...` services.
- The logger writes an event stream and a manifest with configuration paths and
  SHA-256 hashes.
- Accepted, rejected, safety-land, and emergency actions are published on the
  command event topic and included in JSONL/rosbag output.
- Optional rosbag recording refuses to start below 1 GB free and has a maximum
  duration.
- Old direct-cflib scripts moved to `tools/legacy/` with risk documentation.

## 4. Verification completed

The following passed on this machine:

- Python compilation for application, launch, and test modules.
- 29 automated tests covering safe defaults, invalid configurations, Motive
  placeholders, duplicate radios, recovery time, battery, tumble, command
  limits, geofence, separation, trajectories, and architecture boundaries.
- Clean `colcon` build in an isolated workspace.
- ROS package discovery and parsing of all three launch descriptions.
- Live mock preflight and enable.
- Live mock takeoff to 0.30 m over two seconds.
- Simulated tracking loss: controlled-land request at 0.25 s and emergency stop
  at 1.00 s.
- The physical launch with committed files fails before upstream hardware nodes
  start.
- Wired multicast NatNet reception from Motive 2.0 at `172.16.90.213` is stable
  at approximately 120 Hz; Motive reports NatNet 3.0.
- The latest live check produced an empty unlabeled point cloud and no `/poses`.
  A visible free marker is the remaining physical prerequisite for assignment.
- The mocap-only launch now loads an enabled local fleet and generates named
  single-marker tracker entries without starting Crazyswarm2 or a radio server.

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

Confirmed network: Motive 2.0/NatNet 3.0 is `172.16.90.213`; Ubuntu is
`172.16.90.195/27`; wired multicast is working. Still required:

- Measured flight-volume minimum and maximum in the Motive world frame.
- Final marker mount offset and measured initial position for each vehicle.
- Physical label, radio URI, firmware, and battery ID for each vehicle.
- Crazyradio PA firmware version and USB visibility after udev setup.

Previously tested aircraft reported CRTP protocol version 4, so firmware must be
reviewed before Crazyswarm2 flight. One battery showed severe voltage sag and
must remain quarantined. A previous open-loop hop reached the ceiling; those
legacy scripts are not acceptable production tests.

## 7. Next physical bring-up plan

### Stage A: host setup completed

The official Crazyswarm2 and motion-capture packages, matching middleware,
Bitcraze udev rule, uv environment, permanent workspace build, and automated
tests are installed. Free additional disk space before any broad OS upgrade or
optional simulator build.

### Stage B: OptiTrack only, motors disconnected

1. Create ignored local motion-capture configuration with the real Motive IP.
2. Enable NatNet streaming on the correct Windows Ethernet interface.
3. Remove or disable unrelated Motive assets and stream unlabeled markers.
4. Launch only `mocap_only.launch.py` with a local fleet file containing measured
   initial marker positions.
5. Confirm `/poses` rate, identity, world axes, position, and dropout timing.
6. Move each marker by hand and verify there are no swaps. Single-marker tracking
   intentionally provides no external orientation.

Acceptance: stable 80-120 Hz data and correct `cf1`-`cf5` identity with no
Crazyflie server running.

### Stage C: one aircraft, propellers removed

1. Inventory and label one test Crazyflie and battery.
2. Update firmware through the official procedure if required.
3. Assign a unique 2M URI and update only the ignored local fleet file.
4. Connect through Crazyswarm2 and check battery, supervisor, link quality, and
   emergency service.
5. Compare OptiTrack pose with the onboard estimate while moving it by hand.
6. Test tracking dropout and gateway command rejection without propellers.

Acceptance: one correctly identified aircraft, current firmware, healthy
battery, stable link, matching pose, and verified stop path.

### Stage D: one controlled flight

1. Measure and review the local geofence.
2. Use one vehicle, low speed, 0.30 m takeoff, and a clear netted volume.
3. Assign a dedicated physical emergency-stop operator.
4. Pass takeoff/hover/short `go_to`/land before testing fault behavior.
5. Review logs and battery sag after every run.

### Stage E: scale to five

Add one aircraft at a time. Require a unique URI, current firmware, correct
single-marker identity, healthy battery, stable simultaneous links, and
sequential takeoff/landing before synchronized trajectories. Begin well above
0.40 m separation and commands no faster than 0.25 m/s.

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
