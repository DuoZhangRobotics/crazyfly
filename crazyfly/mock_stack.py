"""Mock OptiTrack and Crazyswarm2 services for hardware-free integration tests."""

from __future__ import annotations

from dataclasses import dataclass
from functools import partial
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from crazyflie_interfaces.msg import Status
from crazyflie_interfaces.srv import Arm, GoTo, Land, Takeoff
from geometry_msgs.msg import PoseStamped
from motion_capture_tracking_interfaces.msg import NamedPose, NamedPoseArray
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty, SetBool

from .config import load_fleet
from .safety import CAN_BE_ARMED, CAN_FLY, IS_ARMED, IS_FLYING, IS_TUMBLED


@dataclass
class MockRobot:
    position: list[float]
    battery_voltage: float = 4.1
    armed: bool = False
    flying: bool = False
    tumbled: bool = False


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
                ]
            )
        self.create_timer(0.01, self._publish_poses)
        self.create_timer(0.10, self._publish_status)
        self.get_logger().info(f"mock stack ready for: {', '.join(self.robots) or 'none'}")

    def _publish_poses(self) -> None:
        if self.drop_tracking:
            return
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
        return response

    def _takeoff_callback(
        self, name: str, request: Takeoff.Request, response: Takeoff.Response
    ) -> Takeoff.Response:
        robot = self.robots[name]
        if robot.armed:
            robot.position[2] = float(request.height)
            robot.flying = True
        return response

    def _land_callback(
        self, name: str, request: Land.Request, response: Land.Response
    ) -> Land.Response:
        robot = self.robots[name]
        robot.position[2] = float(request.height)
        robot.flying = False
        return response

    def _goto_callback(
        self, name: str, request: GoTo.Request, response: GoTo.Response
    ) -> GoTo.Response:
        robot = self.robots[name]
        target = [request.goal.x, request.goal.y, request.goal.z]
        if request.relative:
            target = [current + delta for current, delta in zip(robot.position, target)]
        robot.position[:] = target
        return response

    def _emergency_callback(
        self, _request: Empty.Request, response: Empty.Response
    ) -> Empty.Response:
        for robot in self.robots.values():
            robot.armed = False
            robot.flying = False
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
