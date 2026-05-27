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

# Maximum number of path points rendered in Webots.
# Keeping this low avoids overloading the scene-tree insertion.
MAX_PATH_POINTS = 40


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

        self.__node = rclpy.create_node(f'{self.__robot_name}_driver', namespace=self.__robot_name)
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.create_subscription(String, '/cbba/state', self.__cbba_state_callback, 10)
        self.__node.create_subscription(String, 'cbba_path', self.__path_callback, 10)
        self.__pose_publisher = self.__node.create_publisher(PoseStamped, 'pose', 1)

        self.__cbba_completed: set[str] = set()
        self.__cbba_inprogress: set[str] = set()
        self.__cbba_known_tasks: set[str] = set()

        # Path drawing state ---------------------------------------------------
        # __pending_path holds the latest path received from the agent.
        # It is written by __path_callback (ROS thread) and consumed by step()
        # (Webots thread).  We use a simple replace-on-update approach: step()
        # checks __path_dirty and redraws only when something changed.
        self.__pending_path: list[tuple[float, float]] | None = None
        self.__path_dirty: bool = False
        self.__drawn_path: list[tuple[float, float]] | None = None
        self.__last_path_time: float = 0.0
        # Minimum seconds between path redraws to avoid flooding the scene tree.
        self.__path_redraw_interval: float = 1.0

    # -------------------------------------------------------------------------
    # ROS callbacks (called from rclpy.spin_once inside step())
    # -------------------------------------------------------------------------

    def __cmd_vel_callback(self, twist: Twist) -> None:
        self.__target_twist = twist

    def __path_callback(self, message: String) -> None:
        """Receive a path from the CBBA agent and store it for drawing in step().

        We intentionally do NOT touch the Webots scene tree here — that must
        only happen inside step() to avoid thread-safety crashes.
        """
        try:
            payload = json.loads(message.data)
        except Exception:
            return

        # Only process paths meant for this robot.
        if payload.get('robot_name') != self.__robot_name:
            return

        raw_path = payload.get('path', [])

        # Validate and sanitise each point.
        clean: list[tuple[float, float]] = []
        for point in raw_path:
            try:
                x = float(point[0])
                y = float(point[1])
                if math.isfinite(x) and math.isfinite(y):
                    clean.append((x, y))
            except Exception:
                continue

        if not clean:
            return

        # Downsample to avoid inserting an excessively long coordinate string.
        if len(clean) > MAX_PATH_POINTS:
            step = max(1, len(clean) // MAX_PATH_POINTS)
            downsampled = [clean[i] for i in range(0, len(clean), step)]
            # Always keep the last point (the goal).
            if downsampled[-1] != clean[-1]:
                downsampled.append(clean[-1])
            clean = downsampled[:MAX_PATH_POINTS]

        # Skip redraw if the path has not changed.
        if clean == self.__drawn_path:
            return

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

        self.__cbba_known_tasks.update(completed)
        self.__cbba_known_tasks.update(bundle)
        self.__cbba_known_tasks.update(winner_keys)

        self.__cbba_completed.update(completed)

        # Remove completed tasks from in-progress set.
        self.__cbba_inprogress.difference_update(self.__cbba_completed)
        for task_id in bundle:
            if task_id not in self.__cbba_completed:
                self.__cbba_inprogress.add(task_id)

        # Recolor task markers: green = done, red = not done.
        for task_id in list(self.__cbba_known_tasks):
            if task_id in self.__cbba_completed:
                self.__set_task_color(task_id, 0.0, 0.8, 0.0)
            else:
                self.__set_task_color(task_id, 0.85, 0.15, 0.15)

    # -------------------------------------------------------------------------
    # Webots step — runs in the Webots simulation thread
    # -------------------------------------------------------------------------

    def step(self) -> None:
        try:
            # Process all pending ROS messages.
            rclpy.spin_once(self.__node, timeout_sec=0)

            # Publish the robot's current pose.
            self.__publish_pose()

            # Apply motor velocities from the latest cmd_vel.
            forward_speed = self.__target_twist.linear.x
            angular_speed = self.__target_twist.angular.z
            left_vel = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
            right_vel = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
            self.__safe_set_velocity(self.__left_motor, left_vel)
            self.__safe_set_velocity(self.__right_motor, right_vel)

            # Redraw the planned path in the scene tree if it changed and enough
            # time has passed since the last redraw.
            now = time.time()
            if self.__path_dirty and self.__pending_path is not None:
                if now - self.__last_path_time >= self.__path_redraw_interval:
                    self.__redraw_path(self.__pending_path)
                    self.__drawn_path = self.__pending_path
                    self.__path_dirty = False
                    self.__last_path_time = now

        except Exception as exc:
            try:
                self.__node.get_logger().error(f'Unhandled exception in driver step: {exc}')
            except Exception:
                pass

    # -------------------------------------------------------------------------
    # Internal helpers
    # -------------------------------------------------------------------------

    def __publish_pose(self) -> None:
        """Read the robot's ground-truth pose from Webots and publish it."""
        try:
            pose_message = PoseStamped()
            pose_message.header.stamp = self.__node.get_clock().now().to_msg()
            pose_message.header.frame_id = 'map'

            position = self.__robot_node.getPosition()
            orientation = self.__robot_node.getOrientation()

            try:
                yaw = atan2(orientation[3], orientation[0])
            except Exception:
                yaw = 0.0

            pose_message.pose.position.x = position[0]
            pose_message.pose.position.y = position[1]
            pose_message.pose.position.z = position[2]
            pose_message.pose.orientation.z = sin(yaw / 2.0)
            pose_message.pose.orientation.w = cos(yaw / 2.0)
            self.__pose_publisher.publish(pose_message)
        except Exception:
            pass

    def __safe_set_velocity(self, motor, velocity: float) -> None:
        try:
            motor.setVelocity(velocity)
        except Exception:
            pass

    def __redraw_path(self, path: list[tuple[float, float]]) -> None:
        """Remove the previous path line and insert a new one into the scene tree.

        This must only be called from step() (the Webots simulation thread).
        """
        line_def = f'PATH_LINE_{self.__robot_name}'

        # Remove the previous line node if it exists.
        try:
            old_node = self.__robot.getFromDef(line_def)
            if old_node is not None:
                old_node.remove()
        except Exception:
            pass

        if not path:
            return

        # Build the VRML coordinate string.
        coord_parts = [f'{x:.4f} {y:.4f} 0.03' for x, y in path]
        coord_str = ' '.join(coord_parts)
        # coordIndex: 0 1 2 … N-1 -1  (single polyline, -1 terminates it)
        index_str = ' '.join(str(i) for i in range(len(path))) + ' -1'

        node_str = (
            f'DEF {line_def} Transform {{ children [ '
            f'Shape {{ '
            f'appearance Appearance {{ material Material {{ diffuseColor 0 0.6 1 emissiveColor 0 0.3 0.5 }} }} '
            f'geometry IndexedLineSet {{ '
            f'coord Coordinate {{ point [ {coord_str} ] }} '
            f'coordIndex [ {index_str} ] '
            f'}} }} ] }}'
        )

        try:
            root = self.__robot.getRoot()
            children = root.getField('children')
            children.importMFNodeFromString(-1, node_str)
        except Exception as exc:
            try:
                self.__node.get_logger().warning(f'Path draw failed for {self.__robot_name}: {exc}')
            except Exception:
                pass

    def __set_task_color(self, task_id: str, r: float, g: float, b: float) -> None:
        """Update the baseColor of a task marker DEF node in the Webots scene."""
        try:
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

            base_color_field.setSFColor([r, g, b])
        except Exception as exc:
            try:
                self.__node.get_logger().warning(f'Failed to set color for {task_id}: {exc}')
            except Exception:
                pass