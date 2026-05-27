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

# Must match PATH_SPHERE_COUNT in cbba_launch.py.
PATH_SPHERE_COUNT = 40

# Minimum seconds between full path redraws.
PATH_REDRAW_INTERVAL = 0.8

# Park position for unused spheres (far outside the arena).
HIDDEN_POS = [0.0, 999.0, 0.0]


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

        if not rclpy.ok():
            rclpy.init(args=None)

        self.__node = rclpy.create_node(
            f'{self.__robot_name}_driver', namespace=self.__robot_name
        )
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.create_subscription(String, '/cbba/state', self.__cbba_state_callback, 10)
        self.__node.create_subscription(String, 'cbba_path', self.__path_callback, 10)
        self.__pose_publisher = self.__node.create_publisher(PoseStamped, 'pose', 1)

        self.__cbba_completed: set = set()
        self.__cbba_inprogress: set = set()
        self.__cbba_known_tasks: set = set()

        # Path visualisation state.
        self.__pending_path: list | None = None
        self.__path_dirty: bool = False
        self.__drawn_path: list | None = None
        self.__last_redraw_time: float = 0.0

        self.__sphere_translation_fields: list = []

    # -------------------------------------------------------------------------
    # ROS callbacks — must NOT touch Webots APIs
    # -------------------------------------------------------------------------

    def __cmd_vel_callback(self, twist: Twist) -> None:
        self.__target_twist = twist

    def __path_callback(self, message: String) -> None:
        """Buffer the latest path for drawing in step(); never touch Webots here."""
        try:
            payload = json.loads(message.data)
        except Exception:
            return

        if payload.get('robot_name') != self.__robot_name:
            return

        clean = []
        for point in payload.get('path', []):
            try:
                x, y = float(point[0]), float(point[1])
                if math.isfinite(x) and math.isfinite(y):
                    clean.append((x, y))
            except Exception:
                continue

        # Allow empty path through — it signals that all spheres should be parked.
        # Downsample to sphere budget only for non-empty paths.
        if clean and len(clean) > PATH_SPHERE_COUNT:
            step = max(1, len(clean) // PATH_SPHERE_COUNT)
            downsampled = [clean[i] for i in range(0, len(clean), step)]
            if downsampled[-1] != clean[-1]:
                downsampled.append(clean[-1])
            clean = downsampled[:PATH_SPHERE_COUNT]

        if clean == self.__drawn_path:
            return  # nothing changed, skip redraw

        self.__pending_path = clean
        self.__path_dirty = True

    def __cbba_state_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except Exception:
            return

        completed = set(payload.get('completed_tasks', []))
        bundle = set(payload.get('bundle', []))
        winner_keys = set(payload.get('winner_table', {}).keys())

        self.__cbba_known_tasks.update(completed | bundle | winner_keys)
        self.__cbba_completed.update(completed)
        self.__cbba_inprogress.difference_update(self.__cbba_completed)
        for task_id in bundle:
            if task_id not in self.__cbba_completed:
                self.__cbba_inprogress.add(task_id)

        for task_id in self.__cbba_known_tasks:
            if task_id in self.__cbba_completed:
                self.__set_task_color(task_id, 0.0, 0.8, 0.0)
            else:
                self.__set_task_color(task_id, 0.85, 0.15, 0.15)

    # -------------------------------------------------------------------------
    # Webots step — the only place that may call Webots APIs
    # -------------------------------------------------------------------------

    def step(self) -> None:
        try:
            rclpy.spin_once(self.__node, timeout_sec=0)
            self.__publish_pose()
            self.__apply_motors()

            if not self.__sphere_translation_fields:
                self.__cache_sphere_fields()

            # Redraw path if changed; empty path parks all spheres immediately
            # (no throttle so the clear is instantaneous).
            now = time.time()
            if self.__path_dirty and self.__pending_path is not None:
                is_clear = len(self.__pending_path) == 0
                if is_clear or now - self.__last_redraw_time >= PATH_REDRAW_INTERVAL:
                    self.__redraw_path(self.__pending_path)
                    self.__drawn_path = self.__pending_path
                    self.__path_dirty = False
                    if not is_clear:
                        self.__last_redraw_time = now

        except Exception as exc:
            try:
                self.__node.get_logger().error(f'step() error: {exc}')
            except Exception:
                pass

    # -------------------------------------------------------------------------
    # Internal helpers — all Webots API calls live here
    # -------------------------------------------------------------------------

    def __cache_sphere_fields(self) -> None:
        fields = []
        for i in range(PATH_SPHERE_COUNT):
            def_name = f'PATH_{self.__robot_name}_{i}'
            try:
                node = self.__robot.getFromDef(def_name)
                if node is not None:
                    field = node.getField('translation')
                    if field is not None:
                        fields.append(field)
            except Exception:
                pass
        self.__sphere_translation_fields = fields

    def __redraw_path(self, path: list) -> None:
        """Move pre-allocated spheres to path positions; park unused ones."""
        fields = self.__sphere_translation_fields
        if not fields:
            return

        for i, field in enumerate(fields):
            try:
                if i < len(path):
                    x, y = path[i]
                    field.setSFVec3f([x, y, 0.03])
                else:
                    field.setSFVec3f(HIDDEN_POS)
            except Exception:
                pass

    def __publish_pose(self) -> None:
        try:
            msg = PoseStamped()
            msg.header.stamp = self.__node.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            position = self.__robot_node.getPosition()
            orientation = self.__robot_node.getOrientation()
            try:
                yaw = atan2(orientation[3], orientation[0])
            except Exception:
                yaw = 0.0
            msg.pose.position.x = position[0]
            msg.pose.position.y = position[1]
            msg.pose.position.z = position[2]
            msg.pose.orientation.z = sin(yaw / 2.0)
            msg.pose.orientation.w = cos(yaw / 2.0)
            self.__pose_publisher.publish(msg)
        except Exception:
            pass

    def __apply_motors(self) -> None:
        fwd = self.__target_twist.linear.x
        ang = self.__target_twist.angular.z
        left = (fwd - ang * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        right = (fwd + ang * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        try:
            self.__left_motor.setVelocity(left)
        except Exception:
            pass
        try:
            self.__right_motor.setVelocity(right)
        except Exception:
            pass

    def __set_task_color(self, task_id: str, r: float, g: float, b: float) -> None:
        try:
            node = self.__robot.getFromDef(f'TASK_{task_id}')
            if node is None:
                return
            children = node.getField('children')
            if children is None:
                return
            shape = children.getMFNode(0)
            app_field = shape.getField('appearance')
            if app_field is None:
                return
            app_node = app_field.getSFNode()
            color_field = app_node.getField('baseColor')
            if color_field is None:
                return
            color_field.setSFColor([r, g, b])
        except Exception:
            pass