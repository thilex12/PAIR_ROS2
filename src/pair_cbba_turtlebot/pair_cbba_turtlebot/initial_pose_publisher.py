from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from rclpy.node import Node


def _quaternion_from_yaw(yaw: float) -> Quaternion:
    half_yaw = yaw * 0.5
    quaternion = Quaternion()
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = math.sin(half_yaw)
    quaternion.w = math.cos(half_yaw)
    return quaternion


class InitialPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__('initial_pose_publisher')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic', 'initialpose')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('publish_period', 1.0)
        self.declare_parameter('publish_attempts', 5)

        self._frame_id = str(self.get_parameter('frame_id').value)
        self._topic = str(self.get_parameter('topic').value)
        self._x = float(self.get_parameter('x').value)
        self._y = float(self.get_parameter('y').value)
        self._yaw = float(self.get_parameter('yaw').value)
        self._publish_attempts = max(1, int(self.get_parameter('publish_attempts').value))
        self._attempts = 0

        self._publisher = self.create_publisher(PoseWithCovarianceStamped, self._topic, 10)
        period = max(0.2, float(self.get_parameter('publish_period').value))
        self._timer = self.create_timer(period, self._publish_initial_pose)

    def _publish_initial_pose(self) -> None:
        if self._attempts >= self._publish_attempts:
            self.destroy_timer(self._timer)
            rclpy.shutdown()
            return

        message = PoseWithCovarianceStamped()
        message.header.frame_id = self._frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = self._x
        message.pose.pose.position.y = self._y
        message.pose.pose.position.z = 0.0
        message.pose.pose.orientation = _quaternion_from_yaw(self._yaw)

        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.06853891909122467

        self._publisher.publish(message)
        self._attempts += 1


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()