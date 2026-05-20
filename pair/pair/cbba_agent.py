# pair/pair/cbba_agent.py
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025


class CbbaAgent:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')
        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)
        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__target_twist = Twist()

        # Init rclpy si pas encore fait
        if not rclpy.ok():
            rclpy.init(args=None)

        robot_name = self.__robot.getName()
        self.__node = Node(f'cbba_agent_{robot_name}')
        self.__node.create_subscription(
            Twist,
            f'/{robot_name}/cmd_vel',
            self.__cmd_vel_callback,
            1,
        )

        self.__thread = threading.Thread(target=rclpy.spin, args=(self.__node,), daemon=True)
        self.__thread.start()

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)