"""ROS 2 safety gateway between project scripts and Crazyswarm2 services."""

from __future__ import annotations

import json
import time
from functools import partial
from math import isfinite
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration as DurationMessage
from crazyflie_interfaces.msg import Status
from crazyflie_interfaces.srv import Arm, GoTo, Land, Stop, Takeoff
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool, Trigger

from .config import (
    ConfigError,
    load_fleet,
    load_safety,
    point_from_geofence_frame,
)
from .safety import Evaluation, SafetyAction, SafetyMachine, SafetyState


def duration_seconds(message: DurationMessage) -> float:
    return float(message.sec) + float(message.nanosec) / 1_000_000_000.0


def duration_message(seconds: float) -> DurationMessage:
    whole = int(seconds)
    return DurationMessage(sec=whole, nanosec=int((seconds - whole) * 1_000_000_000))


class SafetyGateway(Node):
    """Enforces configuration, health, tracking, and command limits."""

    def __init__(self) -> None:
        super().__init__("crazyfly_safety_gateway")
        share = Path(get_package_share_directory("crazyfly"))
        self.declare_parameter(
            "fleet_config_file", str(share / "config" / "crazyflies.yaml")
        )
        self.declare_parameter(
            "safety_config_file", str(share / "config" / "safety.yaml")
        )
        fleet_path = self.get_parameter("fleet_config_file").value
        safety_path = self.get_parameter("safety_config_file").value
        try:
            self.fleet = load_fleet(fleet_path)
            self.safety = load_safety(safety_path)
        except ConfigError as exc:
            raise RuntimeError(f"unsafe Crazyfly configuration: {exc}") from exc

        self.robot_names = tuple(sorted(self.fleet.enabled))
        self.machine = SafetyMachine(self.safety, self.robot_names)
        self.last_evaluation = Evaluation(
            SafetyState.DISABLED, SafetyAction.NONE, (), False
        )
        self._landing_disarm_at: float | None = None
        self._landing_robots: set[str] = set()
        self._batch_end_at: float | None = None
        self._last_rejection = ""

        self.state_publisher = self.create_publisher(
            String, "/crazyfly/safety/state", 10
        )
        self.command_publisher = self.create_publisher(String, "/crazyfly/commands", 10)
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "/crazyfly/safety/diagnostics", 10
        )
        self.create_subscription(
            NamedPoseArray, "/poses", self._pose_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2,
            "/pointCloud",
            self._point_cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            "/crazyfly/batch_go_to_requests",
            self._batch_goto_callback,
            10,
        )

        self.upstream_clients: dict[str, dict[str, object]] = {}
        self._service_handles: list[object] = []
        for name in self.robot_names:
            self.create_subscription(
                Status, f"/{name}/status", partial(self._status_callback, name), 10
            )
            self.upstream_clients[name] = {
                "arm": self.create_client(Arm, f"/{name}/arm"),
                "takeoff": self.create_client(Takeoff, f"/{name}/takeoff"),
                "land": self.create_client(Land, f"/{name}/land"),
                "go_to": self.create_client(GoTo, f"/{name}/go_to"),
            }
            self._service_handles.extend(
                [
                    self.create_service(
                        Takeoff,
                        f"/crazyfly/{name}/takeoff",
                        partial(self._takeoff_callback, name),
                    ),
                    self.create_service(
                        Land,
                        f"/crazyfly/{name}/land",
                        partial(self._land_callback, name),
                    ),
                    self.create_service(
                        GoTo,
                        f"/crazyfly/{name}/go_to",
                        partial(self._goto_callback, name),
                    ),
                ]
            )

        self.emergency_client = self.create_client(Empty, "/all/emergency")
        self._service_handles.extend(
            [
                self.create_service(
                    Trigger, "/crazyfly/preflight", self._preflight_callback
                ),
                self.create_service(SetBool, "/crazyfly/enable", self._enable_callback),
                self.create_service(
                    Stop, "/crazyfly/emergency", self._emergency_callback
                ),
            ]
        )
        self.create_timer(0.02, self._timer_callback)
        self.get_logger().info(
            f"Safety gateway started DISABLED for {len(self.robot_names)} robot(s): "
            f"{', '.join(self.robot_names) or 'none'}"
        )

    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def _pose_callback(self, message: NamedPoseArray) -> None:
        received_at = self._now()
        for named_pose in message.poses:
            position = named_pose.pose.position
            self.machine.record_pose(
                named_pose.name, (position.x, position.y, position.z), received_at
            )

    def _point_cloud_callback(self, message: PointCloud2) -> None:
        self.machine.record_raw_marker_count(message.width * message.height, self._now())

    def _status_callback(self, name: str, message: Status) -> None:
        self.machine.record_status(
            name, message.battery_voltage, message.supervisor_info, self._now()
        )

    def _preflight_callback(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        success, reasons = self.machine.preflight(self._now())
        response.success = success
        response.message = "preflight passed" if success else "; ".join(reasons)
        return response

    def _enable_callback(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        if not request.data:
            self._send_arm(False)
            self._landing_disarm_at = None
            self._landing_robots.clear()
            self._batch_end_at = None
            self.machine.disable()
            response.success = True
            self._publish_command(
                "enable", "accepted", details="disabled and disarm requested"
            )
            response.message = "flight gateway disabled and disarm requested"
            return response
        success, reasons = self.machine.enable(self._now())
        response.success = success
        if not success:
            self._reject("enable", "; ".join(reasons))
            response.message = "; ".join(reasons)
            return response
        unavailable = self._unavailable_clients("arm")
        if unavailable:
            self._reject(
                "enable", f"arm service unavailable for: {', '.join(unavailable)}"
            )
            self.machine.disable()
            response.success = False
            response.message = f"arm service unavailable for: {', '.join(unavailable)}"
            return response
        self._send_arm(True)
        self._landing_robots.clear()
        self.last_evaluation = self.machine.evaluate(self._now())
        self._publish_command("enable", "accepted", details="arm requests sent")
        response.message = "preflight passed; arm requests sent"
        return response

    def _takeoff_callback(
        self, name: str, request: Takeoff.Request, response: Takeoff.Response
    ) -> Takeoff.Response:
        if not self.last_evaluation.commands_allowed:
            reason = "; ".join(self.last_evaluation.reasons) or (
                f"commands are blocked in {self.machine.state.value}"
            )
            self._reject(f"takeoff/{name}", reason)
            return response
        accepted, reason = self.machine.validate_takeoff(
            name, request.height, duration_seconds(request.duration)
        )
        if not accepted:
            self._reject(f"takeoff/{name}", reason)
            return response
        if not self._forward(name, "takeoff", request):
            return response
        self.machine.mark_flying(name)
        self._publish_command(
            "takeoff", "accepted", name, f"height={request.height:.3f}"
        )
        self.get_logger().info(f"accepted takeoff for {name} to {request.height:.2f} m")
        return response

    def _land_callback(
        self, name: str, request: Land.Request, response: Land.Response
    ) -> Land.Response:
        duration_s = duration_seconds(request.duration)
        if not self.machine.operator_enabled or self.machine.state in {
            SafetyState.DISABLED,
            SafetyState.EMERGENCY,
        }:
            self._reject(
                f"land/{name}", f"commands are blocked in {self.machine.state.value}"
            )
            return response
        if self.machine.state is SafetyState.LANDING and name in self._landing_robots:
            self._reject(f"land/{name}", f"landing is already in progress for {name}")
            return response
        if (
            not isfinite(duration_s)
            or not isfinite(request.height)
            or duration_s <= 0
            or not 0 <= request.height <= 0.10
        ):
            self._reject(
                f"land/{name}", "landing height must be 0-0.10 m with positive duration"
            )
            return response
        if self._forward(name, "land", request):
            self._landing_robots.add(name)
            self.machine.mark_landing()
            requested_disarm_at = self._now() + duration_s + 0.25
            self._landing_disarm_at = max(
                self._landing_disarm_at or requested_disarm_at,
                requested_disarm_at,
            )
            self._publish_command(
                "land", "accepted", name, f"height={request.height:.3f}"
            )
        return response

    def _goto_callback(
        self, name: str, request: GoTo.Request, response: GoTo.Response
    ) -> GoTo.Response:
        if self._batch_end_at is not None and self._now() < self._batch_end_at:
            self._reject(f"go_to/{name}", "coordinated motion is in progress")
            return response
        goal = (request.goal.x, request.goal.y, request.goal.z)
        if not self.last_evaluation.commands_allowed:
            reason = "; ".join(self.last_evaluation.reasons) or (
                f"commands are blocked in {self.machine.state.value}"
            )
            self._reject(f"go_to/{name}", reason)
            return response
        accepted, reason, _target = self.machine.validate_goto(
            name,
            goal,
            duration_seconds(request.duration),
            request.relative,
            request.yaw,
        )
        if not accepted:
            self._reject(f"go_to/{name}", reason)
            return response
        if not self._forward(name, "go_to", request):
            return response
        self._publish_command("go_to", "accepted", name, f"goal={goal}")
        self.get_logger().info(f"accepted go_to for {name}")
        return response

    def _batch_goto_callback(self, message: String) -> None:
        batch_id = "invalid"
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("request must be a JSON object")
            raw_batch_id = payload.get("batch_id")
            if (
                not isinstance(raw_batch_id, str)
                or not raw_batch_id
                or len(raw_batch_id) > 64
            ):
                raise ValueError("batch_id must be a non-empty string")
            batch_id = raw_batch_id
            if payload.get("frame") != self.safety.geofence_frame:
                raise ValueError(
                    "batch frame must match the configured geofence frame"
                )
            duration_s = float(payload.get("duration_s"))
            yaw_rad = float(payload.get("yaw_rad", 0.0))
            if not isfinite(duration_s) or not isfinite(yaw_rad):
                raise ValueError("duration and yaw must be finite")
            raw_goals = payload.get("goals")
            if not isinstance(raw_goals, dict):
                raise ValueError("goals must be a JSON object")
            goals_in_frame: dict[str, tuple[float, float, float]] = {}
            goals_world: dict[str, tuple[float, float, float]] = {}
            for name, raw_goal in raw_goals.items():
                if (
                    not isinstance(name, str)
                    or not isinstance(raw_goal, list)
                    or len(raw_goal) != 3
                ):
                    raise ValueError("each goal must contain three coordinates")
                goal = tuple(float(value) for value in raw_goal)
                if not all(isfinite(value) for value in goal):
                    raise ValueError("goal coordinates must be finite")
                goals_in_frame[name] = goal  # type: ignore[assignment]
                goals_world[name] = point_from_geofence_frame(goal, self.safety)
        except (ConfigError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self._reject_batch(batch_id, str(exc))
            return

        now = self._now()
        if self._batch_end_at is not None and now < self._batch_end_at:
            self._reject_batch(batch_id, "coordinated motion is already in progress")
            return
        if not self.last_evaluation.commands_allowed:
            reason = "; ".join(self.last_evaluation.reasons) or (
                f"commands are blocked in {self.machine.state.value}"
            )
            self._reject_batch(batch_id, reason)
            return
        unavailable = self._unavailable_clients("go_to")
        if unavailable:
            self._reject_batch(
                batch_id,
                f"go_to service unavailable for: {', '.join(unavailable)}",
            )
            return
        accepted, reason, minimum = self.machine.validate_coordinated_goto(
            goals_world, duration_s
        )
        if not accepted:
            self._reject_batch(batch_id, reason)
            return

        requests: list[tuple[str, GoTo.Request]] = []
        for name in sorted(goals_world):
            request = GoTo.Request()
            request.group_mask = 0
            request.relative = False
            (
                request.goal.x,
                request.goal.y,
                request.goal.z,
            ) = goals_world[name]
            request.yaw = yaw_rad
            request.duration = duration_message(duration_s)
            requests.append((name, request))
        for name, request in requests:
            self.upstream_clients[name]["go_to"].call_async(request)

        self._batch_end_at = now + duration_s
        details = json.dumps(
            {
                "batch_id": batch_id,
                "frame": self.safety.geofence_frame,
                "goals": goals_in_frame,
                "duration_s": duration_s,
                "minimum_separation_m": minimum,
            },
            separators=(",", ":"),
        )
        self._publish_command("batch_go_to", "accepted", details=details)
        self.get_logger().info(
            f"accepted coordinated batch {batch_id} "
            f"(minimum separation {minimum:.3f} m)"
        )

    def _emergency_callback(
        self, _request: Stop.Request, response: Stop.Response
    ) -> Stop.Response:
        self._issue_emergency("operator emergency request")
        return response

    def _timer_callback(self) -> None:
        now = self._now()
        self.last_evaluation = self.machine.evaluate(now)
        if self.last_evaluation.action == SafetyAction.LAND:
            self._issue_land(
                "; ".join(self.last_evaluation.reasons) or "safety landing"
            )
        elif self.last_evaluation.action == SafetyAction.EMERGENCY:
            self._issue_emergency("; ".join(self.last_evaluation.reasons))
        if (
            self.machine.state == SafetyState.LANDING
            and self._landing_disarm_at is not None
            and now >= self._landing_disarm_at
        ):
            self._send_arm(False)
            self.machine.disable()
            self._landing_disarm_at = None
            self._landing_robots.clear()
        if self._batch_end_at is not None and now >= self._batch_end_at:
            self._batch_end_at = None
        self._publish_health()

    def _issue_land(self, reason: str) -> None:
        self._batch_end_at = None
        self.get_logger().error(f"safety landing: {reason}")
        self._publish_command("safety_land", "requested", details=reason)
        request = Land.Request()
        request.group_mask = 0
        request.height = 0.05
        request.duration = duration_message(1.0)
        for name in self.robot_names:
            self._forward(name, "land", request)
        self.machine.mark_landing()
        self._landing_robots = set(self.robot_names)
        self._landing_disarm_at = self._now() + 1.25

    def _issue_emergency(self, reason: str) -> None:
        self._batch_end_at = None
        self._landing_disarm_at = None
        self._landing_robots.clear()
        if self.machine.state != SafetyState.EMERGENCY:
            self.machine.mark_emergency()
        self.get_logger().fatal(f"EMERGENCY STOP: {reason}")
        self._publish_command("emergency", "requested", details=reason)
        self._send_arm(False)
        if self.emergency_client.service_is_ready():
            self.emergency_client.call_async(Empty.Request())
        else:
            self.get_logger().fatal("upstream /all/emergency service is unavailable")

    def _send_arm(self, armed: bool) -> None:
        for name in self.robot_names:
            client = self.upstream_clients[name]["arm"]
            if client.service_is_ready():
                request = Arm.Request()
                request.arm = armed
                client.call_async(request)

    def _unavailable_clients(self, kind: str) -> list[str]:
        return [
            name
            for name in self.robot_names
            if not self.upstream_clients[name][kind].service_is_ready()
        ]

    def _forward(self, name: str, kind: str, request: object) -> bool:
        client = self.upstream_clients[name][kind]
        if not client.service_is_ready():
            self._reject(f"{kind}/{name}", "upstream service is unavailable")
            return False
        client.call_async(request)
        return True

    def _publish_command(
        self, command: str, status: str, robot: str = "", details: str = ""
    ) -> None:
        message = String()
        message.data = json.dumps(
            {
                "monotonic_s": self._now(),
                "command": command,
                "status": status,
                "robot": robot,
                "details": details,
            },
            separators=(",", ":"),
        )
        self.command_publisher.publish(message)

    def _reject(self, command: str, reason: str) -> None:
        self._last_rejection = f"{command}: {reason}"
        self._publish_command(command, "rejected", details=reason)
        self.get_logger().warning(f"rejected {self._last_rejection}")

    def _reject_batch(self, batch_id: str, reason: str) -> None:
        details = json.dumps(
            {"batch_id": batch_id, "reason": reason},
            separators=(",", ":"),
        )
        self._last_rejection = f"batch_go_to/{batch_id}: {reason}"
        self._publish_command("batch_go_to", "rejected", details=details)
        self.get_logger().warning(f"rejected {self._last_rejection}")

    def _publish_health(self) -> None:
        state_message = String()
        state_message.data = self.machine.state.value
        self.state_publisher.publish(state_message)

        diagnostic = DiagnosticStatus()
        diagnostic.name = "crazyfly/safety_gateway"
        diagnostic.hardware_id = "crazyfly"
        if self.machine.state == SafetyState.EMERGENCY:
            diagnostic.level = DiagnosticStatus.ERROR
        elif self.last_evaluation.reasons:
            diagnostic.level = DiagnosticStatus.WARN
        else:
            diagnostic.level = DiagnosticStatus.OK
        diagnostic.message = self.machine.state.value
        diagnostic.values = [
            KeyValue(key="operator_enabled", value=str(self.machine.operator_enabled)),
            KeyValue(
                key="commands_allowed", value=str(self.last_evaluation.commands_allowed)
            ),
            KeyValue(key="reasons", value=json.dumps(self.last_evaluation.reasons)),
            KeyValue(key="last_rejection", value=self._last_rejection),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [diagnostic]
        self.diagnostic_publisher.publish(array)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: SafetyGateway | None = None
    try:
        node = SafetyGateway()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
