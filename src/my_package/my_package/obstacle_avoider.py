"""Eviteur d'obstacles reactif simple pour la simulation TurtleBot."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist


MAX_RANGE = 0.15


class ObstacleAvoider(Node):
    """Avance et tourne lorsque l'un des capteurs avant detecte un obstacle."""

    def __init__(self):
        """Installe les abonnements aux capteurs et le publish de vitesse."""

        super().__init__('obstacle_avoider')

        self.__left_sensor_value = MAX_RANGE
        self.__right_sensor_value = MAX_RANGE

        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        self.create_subscription(Range, 'left_sensor', self.__left_sensor_callback, 1)
        self.create_subscription(Range, 'right_sensor', self.__right_sensor_callback, 1)

    def __left_sensor_callback(self, message):
        """Stocke la derniere mesure du capteur gauche et recalcule la commande."""

        self.__left_sensor_value = message.range
        self.__publish_command()

    def __right_sensor_callback(self, message):
        """Stocke la derniere mesure du capteur droit et recalcule la commande."""

        self.__right_sensor_value = message.range
        self.__publish_command()

    def __publish_command(self):
        """Publie une commande simple en avant avec rotation si un obstacle est proche."""

        command_message = Twist()

        command_message.linear.x = 0.1

        if self.__left_sensor_value < 0.9 * MAX_RANGE or self.__right_sensor_value < 0.9 * MAX_RANGE:
            command_message.angular.z = -2.0

        self.__publisher.publish(command_message)


def main(args=None):
    """Lance le noeud d'evitement d'obstacles."""

    rclpy.init(args=args)
    avoider = ObstacleAvoider()
    rclpy.spin(avoider)
    # Destruction explicite du noeud.
    # Sinon, Python le liberera automatiquement plus tard.
    avoider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()