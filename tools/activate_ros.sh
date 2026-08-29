#!/bin/sh
# Source this file before local Crazyswarm2 development commands.

if [ ! -f /opt/ros/jazzy/setup.sh ]; then
  echo "ROS 2 Jazzy is not installed at /opt/ros/jazzy" >&2
  return 1 2>/dev/null || exit 1
fi

. /opt/ros/jazzy/setup.sh

CRAZYFLY_VENV=${CRAZYFLY_VENV:-/home/duo/ros2_ws/.venv}
if [ ! -f "$CRAZYFLY_VENV/bin/activate" ]; then
  echo "Crazyfly uv environment is missing at $CRAZYFLY_VENV" >&2
  return 1 2>/dev/null || exit 1
fi

# Prefer the apt-installed ROS packages. Keep the extracted overlay only as a
# recovery fallback when the system packages are unavailable.
if [ ! -d /opt/ros/jazzy/share/crazyflie ] || \
   [ ! -d /opt/ros/jazzy/share/motion_capture_tracking ]; then
  CRAZYSWARM_LOCAL_ROOT=${CRAZYSWARM_LOCAL_ROOT:-/home/duo/ros2_ws/local_overlay}
  CRAZYSWARM_LOCAL_PREFIX="$CRAZYSWARM_LOCAL_ROOT/opt/ros/jazzy"
else
  CRAZYSWARM_LOCAL_ROOT=
  CRAZYSWARM_LOCAL_PREFIX=
fi

if [ -n "$CRAZYSWARM_LOCAL_PREFIX" ] && [ -d "$CRAZYSWARM_LOCAL_PREFIX" ]; then
  # Extracted Debian package hooks contain /opt paths, so register the relocated
  # prefix directly when the recovery fallback is needed.
  case ":${AMENT_PREFIX_PATH:-}:" in
    *:"$CRAZYSWARM_LOCAL_PREFIX":*) ;;
    *) AMENT_PREFIX_PATH="$CRAZYSWARM_LOCAL_PREFIX${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}" ;;
  esac
  case ":${CMAKE_PREFIX_PATH:-}:" in
    *:"$CRAZYSWARM_LOCAL_PREFIX":*) ;;
    *) CMAKE_PREFIX_PATH="$CRAZYSWARM_LOCAL_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}" ;;
  esac
  case ":${LD_LIBRARY_PATH:-}:" in
    *:"$CRAZYSWARM_LOCAL_PREFIX/lib":*) ;;
    *) LD_LIBRARY_PATH="$CRAZYSWARM_LOCAL_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;;
  esac
  case ":${PATH:-}:" in
    *:"$CRAZYSWARM_LOCAL_PREFIX/bin":*) ;;
    *) PATH="$CRAZYSWARM_LOCAL_PREFIX/bin${PATH:+:$PATH}" ;;
  esac
  PYTHONPATH="$CRAZYSWARM_LOCAL_ROOT/python:$CRAZYSWARM_LOCAL_PREFIX/lib/python3.12/site-packages:$CRAZYSWARM_LOCAL_ROOT/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
  export AMENT_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PATH PYTHONPATH
fi

. "$CRAZYFLY_VENV/bin/activate"

# Ubuntu's colcon launcher has a /usr/bin/python3 shebang. Route interactive
# builds through the active uv interpreter so installed project scripts retain
# the virtual-environment shebang.
colcon() {
  "$VIRTUAL_ENV/bin/python" /usr/bin/colcon "$@"
}

CRAZYFLY_WORKSPACE_INSTALL=${CRAZYFLY_WORKSPACE_INSTALL:-/home/duo/ros2_ws/install}
if [ -f "$CRAZYFLY_WORKSPACE_INSTALL/local_setup.sh" ]; then
  . "$CRAZYFLY_WORKSPACE_INSTALL/local_setup.sh"
fi

unset CRAZYFLY_VENV CRAZYFLY_WORKSPACE_INSTALL CRAZYSWARM_LOCAL_PREFIX CRAZYSWARM_LOCAL_ROOT
