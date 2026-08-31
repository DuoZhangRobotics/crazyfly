"""Mock OptiTrack and Crazyswarm2 services for hardware-free integration tests."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from functools import partial
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from crazyflie_interfaces.msg import Status
from crazyflie_interfaces.srv import (
    Arm,
    GoTo,
    Land,
    StartTrajectory,
    Takeoff,
    UploadTrajectory,
)
from geometry_msgs.msg import PoseStamped
from motion_capture_tracking_interfaces.msg import NamedPose, NamedPoseArray
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty, SetBool

from .config import load_fleet
from .safety import CAN_BE_ARMED, CAN_FLY, IS_ARMED, IS_FLYING, IS_TUMBLED
from .trajectory import evaluate_coefficients


@dataclass
class MockMotion:
    start: tuple[float, float, float]
    target: tuple[float, float, float]
    started_at: float
    duration_s: float
    flying_after: bool | None = None


@dataclass
class MockRobot:
    position: list[float]
    battery_voltage: float = 4.1
    armed: bool = False
    flying: bool = False
    tumbled: bool = False
    trajectories: dict[int, list[object]] = field(default_factory=dict)
    active_trajectory_id: int | None = None
    trajectory_started_at: float | None = None
    active_motion: MockMotion | None = None


class MockStack(Node):
    def __init__(self) -> None:
        super().__init__("crazyfly_mock_stack")
        default = Path(get_package_share_directory("crazyfly")) / "config" / "mock_crazyflies.yaml"
        self.declare_parameter("fleet_config_file", str(default))
        fleet = load_fleet(self.get_parameter("fleet_config_file").value)
        self.robots = {
            name: MockRobot(position=list(robot.initial_position))
            for name, robot in fleet.enabled.items()
        }
        self.drop_tracking = False
        self.pose_publisher = self.create_publisher(
            NamedPoseArray, "/poses", qos_profile_sensor_data
        )
        self.point_cloud_publisher = self.create_publisher(
            PointCloud2, "/pointCloud", qos_profile_sensor_data
        )
        self.onboard_pose_publishers = {
            name: self.create_publisher(PoseStamped, f"/{name}/pose", 10)
            for name in self.robots
        }
        self.status_publishers = {
            name: self.create_publisher(Status, f"/{name}/status", 10)
            for name in self.robots
        }
        self._services: list[object] = [
            self.create_service(Empty, "/all/emergency", self._emergency_callback),
            self.create_service(
                StartTrajectory,
                "/all/start_trajectory",
                self._start_trajectory_callback,
            ),
            self.create_service(SetBool, "/mock/drop_tracking", self._drop_tracking_callback),
            self.create_service(SetBool, "/mock/tumble", self._tumble_callback),
        ]
        for name in self.robots:
            self._services.extend(
                [
                    self.create_service(Arm, f"/{name}/arm", partial(self._arm_callback, name)),
                    self.create_service(
                        Takeoff, f"/{name}/takeoff", partial(self._takeoff_callback, name)
                    ),
                    self.create_service(Land, f"/{name}/land", partial(self._land_callback, name)),
                    self.create_service(GoTo, f"/{name}/go_to", partial(self._goto_callback, name)),
                    self.create_service(
                        UploadTrajectory,
                        f"/{name}/upload_trajectory",
                        partial(self._upload_trajectory_callback, name),
                    ),
                ]
            )
        self.create_timer(0.01, self._publish_poses)
        self.create_timer(0.10, self._publish_status)
        self.get_logger().info(f"mock stack ready for: {', '.join(self.robots) or 'none'}")

    def _publish_poses(self) -> None:
        if self.drop_tracking:
            return
        self._update_motions()
        self._update_trajectories()
        message = NamedPoseArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "world"
        for name, robot in self.robots.items():
            pose = NamedPose()
            pose.name = name
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = robot.position
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
            onboard = PoseStamped()
            onboard.header = message.header
            onboard.pose = pose.pose
            self.onboard_pose_publishers[name].publish(onboard)
        self.pose_publisher.publish(message)
        point_cloud = PointCloud2()
        point_cloud.header = message.header
        point_cloud.height = 1
        point_cloud.width = len(self.robots)
        self.point_cloud_publisher.publish(point_cloud)

    def _publish_status(self) -> None:
        for name, robot in self.robots.items():
            message = Status()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = name
            message.battery_voltage = robot.battery_voltage
            message.rssi = 40
            info = CAN_BE_ARMED | CAN_FLY
            if robot.armed:
                info |= IS_ARMED
            if robot.flying:
                info |= IS_FLYING
            if robot.tumbled:
                info |= IS_TUMBLED
            message.supervisor_info = info
            self.status_publishers[name].publish(message)

    def _arm_callback(
        self, name: str, request: Arm.Request, response: Arm.Response
    ) -> Arm.Response:
        self.robots[name].armed = request.arm
        if not request.arm:
            self.robots[name].flying = False
            self.robots[name].active_motion = None
            self.robots[name].active_trajectory_id = None
            self.robots[name].trajectory_started_at = None
        return response

    def _takeoff_callback(
        self, name: str, request: Takeoff.Request, response: Takeoff.Response
    ) -> Takeoff.Response:
        robot = self.robots[name]
        if robot.armed:
            robot.flying = True
            target = (robot.position[0], robot.position[1], float(request.height))
            self._start_motion(
                robot,
                target,
                self._duration_seconds(request.duration),
                flying_after=True,
            )
        return response

    def _land_callback(
        self, name: str, request: Land.Request, response: Land.Response
    ) -> Land.Response:
        robot = self.robots[name]
        target = (robot.position[0], robot.position[1], float(request.height))
        self._start_motion(
            robot,
            target,
            self._duration_seconds(request.duration),
            flying_after=False,
        )
        robot.active_trajectory_id = None
        robot.trajectory_started_at = None
        return response

    def _goto_callback(
        self, name: str, request: GoTo.Request, response: GoTo.Response
    ) -> GoTo.Response:
        robot = self.robots[name]
        target = [request.goal.x, request.goal.y, request.goal.z]
        if request.relative:
            target = [current + delta for current, delta in zip(robot.position, target)]
        self._start_motion(
            robot,
            tuple(float(value) for value in target),
            self._duration_seconds(request.duration),
        )
        return response

    def _upload_trajectory_callback(
        self,
        name: str,
        request: UploadTrajectory.Request,
        response: UploadTrajectory.Response,
    ) -> UploadTrajectory.Response:
        self.robots[name].trajectories[int(request.trajectory_id)] = list(request.pieces)
        return response

    def _start_trajectory_callback(
        self,
        request: StartTrajectory.Request,
        response: StartTrajectory.Response,
    ) -> StartTrajectory.Response:
        started_at = time.monotonic()
        for robot in self.robots.values():
            trajectory_id = int(request.trajectory_id)
            if trajectory_id in robot.trajectories:
                robot.active_motion = None
                robot.active_trajectory_id = trajectory_id
                robot.trajectory_started_at = started_at
        return response

    @staticmethod
    def _duration_seconds(message) -> float:
        return float(message.sec) + float(message.nanosec) / 1_000_000_000.0

    @staticmethod
    def _start_motion(
        robot: MockRobot,
        target: tuple[float, float, float],
        duration_s: float,
        *,
        flying_after: bool | None = None,
    ) -> None:
        robot.active_motion = MockMotion(
            start=tuple(robot.position),
            target=target,
            started_at=time.monotonic(),
            duration_s=max(0.001, duration_s),
            flying_after=flying_after,
        )

    def _update_motions(self) -> None:
        now = time.monotonic()
        for robot in self.robots.values():
            motion = robot.active_motion
            if motion is None:
                continue
            fraction = min(1.0, max(0.0, now - motion.started_at) / motion.duration_s)
            robot.position[:] = [
                start + fraction * (target - start)
                for start, target in zip(motion.start, motion.target)
            ]
            if fraction >= 1.0:
                if motion.flying_after is not None:
                    robot.flying = motion.flying_after
                robot.active_motion = None

    def _update_trajectories(self) -> None:
        now = time.monotonic()
        for robot in self.robots.values():
            if (
                robot.active_trajectory_id is None
                or robot.trajectory_started_at is None
            ):
                continue
            pieces = robot.trajectories[robot.active_trajectory_id]
            elapsed = max(0.0, now - robot.trajectory_started_at)
            total_duration = sum(
                self._duration_seconds(piece.duration) for piece in pieces
            )
            selected = pieces[-1]
            local_time = self._duration_seconds(selected.duration)
            remaining = elapsed
            for piece in pieces:
                duration_s = self._duration_seconds(piece.duration)
                if remaining <= duration_s:
                    selected = piece
                    local_time = remaining
                    break
                remaining -= duration_s
            robot.position[:] = [
                evaluate_coefficients(coefficients, local_time)
                for coefficients in (
                    selected.poly_x,
                    selected.poly_y,
                    selected.poly_z,
                )
            ]
            if elapsed >= total_duration:
                robot.active_trajectory_id = None
                robot.trajectory_started_at = None

    def _emergency_callback(
        self, _request: Empty.Request, response: Empty.Response
    ) -> Empty.Response:
        for robot in self.robots.values():
            robot.armed = False
            robot.flying = False
            robot.active_motion = None
            robot.active_trajectory_id = None
            robot.trajectory_started_at = None
        return response

    def _drop_tracking_callback(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        self.drop_tracking = request.data
        response.success = True
        response.message = f"drop_tracking={self.drop_tracking}"
        return response

    def _tumble_callback(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        for robot in self.robots.values():
            robot.tumbled = request.data
        response.success = True
        response.message = f"tumbled={request.data}"
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MockStack()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
