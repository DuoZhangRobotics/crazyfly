"""Validate trajectories by default and execute only through the safety gateway."""

from __future__ import annotations

import argparse
import sys
import time

from crazyflie_interfaces.srv import GoTo
import rclpy
from rclpy.node import Node

from .config import ConfigError, load_fleet, load_safety
from .safety_gateway import duration_message
from .trajectory import load_trajectory


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate or execute a safety-gated trajectory.")
    parser.add_argument("--fleet", required=True)
    parser.add_argument("--safety", required=True)
    parser.add_argument("--trajectory", required=True)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="send go_to requests to a separately enabled safety gateway",
    )
    args = parser.parse_args()
    try:
        fleet = load_fleet(args.fleet)
        safety = load_safety(args.safety)
        plan = load_trajectory(args.trajectory, fleet, safety)
    except ConfigError as exc:
        print(f"FAILED: {exc}", file=sys.stderr)
        return 1
    print(f"PASS: {len(plan.steps)} synchronized trajectory step(s) are valid")
    if not args.execute:
        print("DRY RUN: no ROS commands were sent")
        return 0

    rclpy.init()
    node = Node("crazyfly_trajectory_runner")
    try:
        clients = {
            name: node.create_client(GoTo, f"/crazyfly/{name}/go_to")
            for name in fleet.enabled
        }
        for name, client in clients.items():
            if not client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(f"safety gateway service unavailable for {name}")
        for index, step in enumerate(plan.steps):
            futures = []
            for name, goal in step.goals.items():
                request = GoTo.Request()
                request.group_mask = 0
                request.relative = False
                request.goal.x, request.goal.y, request.goal.z = goal.position
                request.yaw = goal.yaw_deg
                request.duration = duration_message(step.duration_s)
                futures.append(clients[name].call_async(request))
            deadline = time.monotonic() + 2.0
            while any(not future.done() for future in futures) and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            if any(not future.done() for future in futures):
                raise RuntimeError(f"gateway did not acknowledge trajectory step {index}")
            time.sleep(step.duration_s)
        return 0
    except RuntimeError as exc:
        print(f"FAILED: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
