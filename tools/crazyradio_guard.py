"""Detect and safely release a Crazyradio held by stale lab processes."""

from __future__ import annotations

import os
import re
import signal
import subprocess
import time
from contextlib import suppress
from pathlib import Path

VENDOR_ID = "1915"
PRODUCT_ID = "7777"
KNOWN_COMMAND_MARKERS = (
    "crazyflie_server",
    "crazyfly_safety_gateway",
    "motion_capture_tracking_node",
    "ros2 launch crazyfly",
    "cfclient",
)
ACTIVE_STATES = {"FLYING", "LANDING"}


def _read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8").strip()


def crazyradio_device() -> Path:
    for vendor_path in Path("/sys/bus/usb/devices").glob("*/idVendor"):
        device_directory = vendor_path.parent
        product_path = device_directory / "idProduct"
        try:
            if (
                _read_text(vendor_path).lower() == VENDOR_ID
                and _read_text(product_path).lower() == PRODUCT_ID
            ):
                bus = int(_read_text(device_directory / "busnum"))
                device = int(_read_text(device_directory / "devnum"))
                path = Path(f"/dev/bus/usb/{bus:03d}/{device:03d}")
                if path.exists():
                    return path
        except (FileNotFoundError, PermissionError, ValueError):
            continue
    raise RuntimeError("Crazyradio 1915:7777 is not detected by Linux")


def parse_fuser_pids(output: str) -> set[int]:
    return {int(value) for value in re.findall(r"\d+", output)}


def device_holders(device: Path) -> set[int]:
    result = subprocess.run(
        ["fuser", str(device)],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    return parse_fuser_pids(result.stdout)


def process_command(pid: int) -> str:
    try:
        return (
            (Path("/proc") / str(pid) / "cmdline")
            .read_bytes()
            .replace(b"\0", b" ")
            .decode(errors="replace")
            .strip()
        )
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return ""


def is_known_crazyflie_process(command: str) -> bool:
    return any(marker in command for marker in KNOWN_COMMAND_MARKERS)


def _ancestor_pids() -> set[int]:
    result: set[int] = set()
    pid = os.getpid()
    while pid > 1 and pid not in result:
        result.add(pid)
        try:
            fields = (Path("/proc") / str(pid) / "stat").read_text().split()
            pid = int(fields[3])
        except (FileNotFoundError, PermissionError, ValueError, IndexError):
            break
    return result


def known_conflicts() -> dict[int, str]:
    excluded = _ancestor_pids()
    conflicts: dict[int, str] = {}
    for process_path in Path("/proc").glob("[0-9]*"):
        try:
            pid = int(process_path.name)
            if pid in excluded or process_path.stat().st_uid != os.getuid():
                continue
        except (FileNotFoundError, PermissionError, ValueError):
            continue
        command = process_command(pid)
        if is_known_crazyflie_process(command):
            conflicts[pid] = command
    return conflicts


def existing_safety_state() -> str | None:
    try:
        result = subprocess.run(
            [
                "ros2",
                "topic",
                "echo",
                "--once",
                "/crazyfly/safety/state",
            ],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=2.0,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None
    match = re.search(r"data:\s*['\"]?([A-Z_]+)", result.stdout)
    return match.group(1) if match else None


def _request_disable() -> None:
    try:
        result = subprocess.run(
            [
                "ros2",
                "service",
                "call",
                "/crazyfly/enable",
                "std_srvs/srv/SetBool",
                "{data: false}",
            ],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=4.0,
        )
        if "success=True" in result.stdout:
            print("Existing gateway accepted disable/disarm")
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass


def _terminate_processes(pids: set[int]) -> None:
    for pid in sorted(pids):
        with suppress(ProcessLookupError, PermissionError):
            os.kill(pid, signal.SIGTERM)
    deadline = time.monotonic() + 4.0
    remaining = set(pids)
    while remaining and time.monotonic() < deadline:
        remaining = {pid for pid in remaining if Path(f"/proc/{pid}").exists()}
        if remaining:
            time.sleep(0.05)
    for pid in sorted(remaining):
        with suppress(ProcessLookupError, PermissionError):
            os.kill(pid, signal.SIGKILL)


def describe_status() -> tuple[Path, set[int], dict[int, str]]:
    device = crazyradio_device()
    return device, device_holders(device), known_conflicts()


def ensure_crazyradio_free() -> Path:
    device, holders, conflicts = describe_status()
    if not holders and not conflicts:
        print(f"Crazyradio is free: {device}")
        return device

    unknown_holders = {
        pid: process_command(pid)
        for pid in holders
        if pid not in conflicts and not is_known_crazyflie_process(process_command(pid))
    }
    if unknown_holders:
        descriptions = ", ".join(
            f"{pid} ({command or 'unknown command'})"
            for pid, command in sorted(unknown_holders.items())
        )
        raise RuntimeError(
            f"Crazyradio is held by an unknown process; refusing to terminate: {descriptions}"
        )

    state = existing_safety_state()
    if state in ACTIVE_STATES:
        raise RuntimeError(
            f"an existing gateway reports {state}; refusing to terminate an active flight"
        )

    targets = set(conflicts) | holders
    print("Releasing stale Crazyradio processes:")
    for pid in sorted(targets):
        print(f"  {pid}: {process_command(pid) or conflicts.get(pid, 'unknown')}")
    if any("crazyfly_safety_gateway" in command for command in conflicts.values()):
        _request_disable()
    _terminate_processes(targets)

    device = crazyradio_device()
    remaining_holders = device_holders(device)
    remaining_conflicts = known_conflicts()
    if remaining_holders or remaining_conflicts:
        raise RuntimeError(
            "Crazyradio cleanup did not complete; holders="
            f"{sorted(remaining_holders)}, processes={sorted(remaining_conflicts)}"
        )
    print(f"Crazyradio released: {device}")
    return device
