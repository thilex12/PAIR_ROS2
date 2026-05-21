import json
import math
import time
import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

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
        self.__node.create_subscription(String, '/cbba/state', self.__cbba_state_callback, 10)
        self.__node.create_subscription(String, 'cbba_path', self.__path_callback, 10)
        self.__pose_publisher = self.__node.create_publisher(PoseStamped, 'pose', 1)
        self.__cbba_completed: set[str] = set()
        self.__cbba_inprogress: set[str] = set()
        self.__cbba_known_tasks: set[str] = set()
        self.__last_path = None
        self.__last_path_time = 0.0

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def __path_callback(self, message: String):
        # Rate-limit and safely draw a single line per robot
        now = time.time()
        if now - self.__last_path_time < 0.6:
            return

        try:
            payload = json.loads(message.data)
        except Exception:
            return

        robot_name = payload.get('robot_name')
        if robot_name != self.__robot_name:
            return

        path = payload.get('path', [])
        # sanitize: keep only finite numeric pairs and limit length
        clean = []
        for p in path:
            try:
                x = float(p[0])
                y = float(p[1])
                if not (math.isfinite(x) and math.isfinite(y)):
                    continue
                clean.append((x, y))
            except Exception:
                continue
        if not clean:
            return
        MAX_POINTS = 50
        if len(clean) > MAX_POINTS:
            clean = clean[:MAX_POINTS]

        # avoid re-creating the line if path didn't change
        if self.__last_path == clean:
            return
        self.__last_path = clean
        self.__last_path_time = now
        try:
            root = self.__robot.getRoot()
            children = root.getField('children')
        except Exception:
            return
        # lightweight: remove previous line marker and draw a single IndexedLineSet for the path
        line_def = f'PATH_LINE_{self.__robot_name}'
        old = self.__robot.getFromDef(line_def)
        if old:
            try:
                old.remove()
            except Exception:
                pass

        if not clean:
            return

        # build coordinate list and coordIndex
        coord_points = []
        for (x, y) in clean:
            coord_points.append(f'{float(x):.6f} {float(y):.6f} 0.03')

        coord_str = ' '.join(coord_points)
        index_list = ', '.join(str(i) for i in range(len(clean)))

        # use Appearance/Material (simpler, more compatible) for stability
        node_str = (
            f'DEF {line_def} Transform {{ children [ '
            f'Shape {{ appearance Appearance {{ material Material {{ diffuseColor 0 0.6 1 }} }} '
            f'geometry IndexedLineSet {{ coord Coordinate {{ point [ {coord_str} ] }} coordIndex [ {index_list} -1 ] }} }} ] }}'
        )

        try:
            children.importMFNodeFromString(-1, node_str)
        except Exception:
            pass

    def step(self):
        try:
            rclpy.spin_once(self.__node, timeout_sec=0)

            pose_message = PoseStamped()
            pose_message.header.stamp = self.__node.get_clock().now().to_msg()
            pose_message.header.frame_id = 'map'

            position = self.__robot_node.getPosition()
            orientation = self.__robot_node.getOrientation()
            # orientation may be a rotation or quaternion-like array; guard against unexpected shapes
            try:
                yaw = atan2(orientation[3], orientation[0])
            except Exception:
                yaw = 0.0

            pose_message.pose.position.x = position[0]
            pose_message.pose.position.y = position[1]
            pose_message.pose.position.z = position[2]
            pose_message.pose.orientation.z = sin(yaw / 2.0)
            pose_message.pose.orientation.w = cos(yaw / 2.0)
            try:
                self.__pose_publisher.publish(pose_message)
            except Exception:
                pass

            forward_speed = self.__target_twist.linear.x
            angular_speed = self.__target_twist.angular.z

            command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
            command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

            try:
                self.__left_motor.setVelocity(command_motor_left)
            except Exception:
                pass
            try:
                self.__right_motor.setVelocity(command_motor_right)
            except Exception:
                pass
        except Exception as e:
            try:
                self.__node.get_logger().error(f'Unhandled exception in driver step: {e}')
            except Exception:
                pass

    def __cbba_state_callback(self, message: String):
        try:
            payload = json.loads(message.data)
        except Exception:
            return

        completed = set(payload.get('completed_tasks', []))
        bundle = set(payload.get('bundle', []))
        winner_keys = set(payload.get('winner_table', {}).keys())

        # update known tasks
        self.__cbba_known_tasks.update(completed)
        self.__cbba_known_tasks.update(bundle)
        self.__cbba_known_tasks.update(winner_keys)

        # update completed set (cumulative)
        self.__cbba_completed.update(completed)

        # update in-progress: any task currently in a bundle from any agent and not completed
        # remove completed tasks from inprogress
        self.__cbba_inprogress.difference_update(self.__cbba_completed)
        for t in bundle:
            if t not in self.__cbba_completed:
                self.__cbba_inprogress.add(t)

        # recolor according to precedence: completed > in-progress > not-done
        for task_id in list(self.__cbba_known_tasks):
            # only two states: red = not done, green = done
            if task_id in self.__cbba_completed:
                self.__set_task_color(task_id, 0.0, 0.8, 0.0)  # green
            else:
                self.__set_task_color(task_id, 0.85, 0.15, 0.15)  # red

    def __set_task_color(self, task_id: str, r: float, g: float, b: float) -> None:
        try:
            # DEF name produced by world builder: TASK_<id>
            def_name = f'TASK_{task_id}'
            node = self.__robot.getFromDef(def_name)
            if node is None:
                return

            children_field = node.getField('children')
            if children_field is None:
                return

            try:
                shape_node = children_field.getMFNode(0)
            except Exception:
                return

            appearance_field = shape_node.getField('appearance')
            if appearance_field is None:
                return

            try:
                app_node = appearance_field.getSFNode()
            except Exception:
                return

            base_color_field = app_node.getField('baseColor')
            if base_color_field is None:
                return

            # setSFColor expects a list of three floats
            base_color_field.setSFColor([r, g, b])
        except Exception as e:
            try:
                self.__node.get_logger().warning(f'Failed to set color for {task_id}: {e}')
            except Exception:
                pass