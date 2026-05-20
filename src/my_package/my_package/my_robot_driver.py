import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

from math import atan2, cos, sin

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__robot_node = self.__robot.getSelf()
        self.__robot_name = self.__robot.getName()

        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__target_twist = Twist()
        self.__pose_publisher = None

        if not rclpy.ok():
            rclpy.init(args=None)
        
        self.__node = rclpy.create_node(f'{self.__robot_name}_driver', namespace=self.__robot_name)
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__pose_publisher = self.__node.create_publisher(PoseStamped, 'pose', 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        pose_message = PoseStamped()
        pose_message.header.stamp = self.__node.get_clock().now().to_msg()
        pose_message.header.frame_id = 'map'

        position = self.__robot_node.getPosition()
        orientation = self.__robot_node.getOrientation()
        yaw = atan2(orientation[3], orientation[0])

        pose_message.pose.position.x = position[0]
        pose_message.pose.position.y = position[1]
        pose_message.pose.position.z = position[2]
        pose_message.pose.orientation.z = sin(yaw / 2.0)
        pose_message.pose.orientation.w = cos(yaw / 2.0)
        self.__pose_publisher.publish(pose_message)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)