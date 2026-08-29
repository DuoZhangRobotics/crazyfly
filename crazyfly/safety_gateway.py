"""ROS 2 safety gateway between project scripts and Crazyswarm2 services."""

from __future__ import annotations

from functools import partial
from math import isfinite
import json
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration as DurationMessage
from crazyflie_interfaces.msg import Status
from crazyflie_interfaces.srv import Arm, GoTo, Land, Stop, Takeoff
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from motion_capture_tracking_interfaces.msg import NamedPoseArray
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool, Trigger

from .config import ConfigError, load_fleet, load_safety
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
        self.declare_parameter("fleet_config_file", str(share / "config" / "crazyflies.yaml"))
        self.declare_parameter("safety_config_file", str(share / "config" / "safety.yaml"))
        fleet_path = self.get_parameter("fleet_config_file").value
        safety_path = self.get_parameter("safety_config_file").value
        try:
            self.fleet = load_fleet(fleet_path)
            self.safety = load_safety(safety_path)
        except ConfigError as exc:
            raise RuntimeError(f"unsafe Crazyfly configuration: {exc}") from exc

        self.robot_names = tuple(sorted(self.fleet.enabled))
        self.machine = SafetyMachine(self.safety, self.robot_names)
        self.last_evaluation = Evaluation(SafetyState.DISABLED, SafetyAction.NONE, (), False)
        self._landing_started_at: float | None = None
        self._last_rejection = ""

        self.state_publisher = self.create_publisher(String, "/crazyfly/safety/state", 10)
        self.command_publisher = self.create_publisher(String, "/crazyfly/commands", 10)
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "/crazyfly/safety/diagnostics", 10
        )
        self.create_subscription(
            NamedPoseArray, "/poses", self._pose_callback, qos_profile_sensor_data
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
                self.create_service(Trigger, "/crazyfly/preflight", self._preflight_callback),
                self.create_service(SetBool, "/crazyfly/enable", self._enable_callback),
                self.create_service(Stop, "/crazyfly/emergency", self._emergency_callback),
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
            self.machine.disable()
            response.success = True
            self._publish_command("enable", "accepted", details="disabled and disarm requested")
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
            self._reject("enable", f"arm service unavailable for: {', '.join(unavailable)}")
            self.machine.disable()
            response.success = False
            response.message = f"arm service unavailable for: {', '.join(unavailable)}"
            return response
        self._send_arm(True)
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
        self._publish_command("takeoff", "accepted", name, f"height={request.height:.3f}")
        self.get_logger().info(f"accepted takeoff for {name} to {request.height:.2f} m")
        return response

    def _land_callback(
        self, name: str, request: Land.Request, response: Land.Response
    ) -> Land.Response:
        duration_s = duration_seconds(request.duration)
        if (
            not isfinite(duration_s)
            or not isfinite(request.height)
            or duration_s <= 0
            or not 0 <= request.height <= 0.10
        ):
            self._reject(f"land/{name}", "landing height must be 0-0.10 m with positive duration")
            return response
        if self._forward(name, "land", request):
            self.machine.mark_landing()
            self._landing_started_at = self._now()
            self._publish_command("land", "accepted", name, f"height={request.height:.3f}")
        return response

    def _goto_callback(
        self, name: str, request: GoTo.Request, response: GoTo.Response
    ) -> GoTo.Response:
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

    def _emergency_callback(
        self, _request: Stop.Request, response: Stop.Response
    ) -> Stop.Response:
        self._issue_emergency("operator emergency request")
        return response

    def _timer_callback(self) -> None:
        now = self._now()
        self.last_evaluation = self.machine.evaluate(now)
        if self.last_evaluation.action == SafetyAction.LAND:
            self._issue_land("; ".join(self.last_evaluation.reasons) or "safety landing")
        elif self.last_evaluation.action == SafetyAction.EMERGENCY:
            self._issue_emergency("; ".join(self.last_evaluation.reasons))
        if (
            self.machine.state == SafetyState.LANDING
            and self._landing_started_at is not None
            and now - self._landing_started_at >= 1.25
        ):
            self._send_arm(False)
            self.machine.disable()
            self._landing_started_at = None
        self._publish_health()

    def _issue_land(self, reason: str) -> None:
        self.get_logger().error(f"safety landing: {reason}")
        self._publish_command("safety_land", "requested", details=reason)
        request = Land.Request()
        request.group_mask = 0
        request.height = 0.05
        request.duration = duration_message(1.0)
        for name in self.robot_names:
            self._forward(name, "land", request)
        self.machine.mark_landing()
        self._landing_started_at = self._now()

    def _issue_emergency(self, reason: str) -> None:
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
            KeyValue(key="commands_allowed", value=str(self.last_evaluation.commands_allowed)),
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
