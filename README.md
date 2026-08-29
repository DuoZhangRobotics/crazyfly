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

Validate the example trajectory without sending commands:

```sh
ros2 run crazyfly crazyfly_trajectory \
  --fleet config/mock_crazyflies.yaml \
  --safety config/mock_safety.yaml \
  --trajectory config/trajectories/mock_square.yaml
```

Execution requires the separately running and enabled mock gateway plus the
explicit `--execute` flag.

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

The launch validates all three files before starting Crazyswarm2. Use absolute
paths so there is no ambiguity:

```sh
ros2 launch crazyfly swarm.launch.py \
  allow_hardware:=true \
  fleet_config_file:=/absolute/path/to/config/local/crazyflies.yaml \
  safety_config_file:=/absolute/path/to/config/local/safety.yaml \
  motion_capture_yaml_file:=/absolute/path/to/config/local/motion_capture.yaml
```

Do not run this yet. Radio USB permissions, current Crazyflie firmware, unique
radio addresses, single-marker identity, and a propellers-off
single-aircraft check still need physical validation.

## Experiment logging

The logger writes newline-delimited pose, status, safety, diagnostic, and command
events. Its manifest records absolute configuration paths and SHA-256 hashes.
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
