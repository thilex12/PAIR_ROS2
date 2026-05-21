# pair/pair/cbba_agent.py
import math
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

# ── Robot physical constants ──────────────────────────────────────────────────
HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

# ── Navigation constants ──────────────────────────────────────────────────────
LINEAR_SPEED = 0.06           # m/s forward speed
ANGULAR_GAIN = 1.0            # proportional gain for heading correction
WAYPOINT_REACHED_DIST = 0.08  # m — how close counts as "reached"
MAX_WHEEL_VELOCITY = 9.0      # safely below Webots' limit of 10

# ── Avoidance constants ───────────────────────────────────────────────────────
AVOIDANCE_DIST = 0.25         # m — trigger avoidance below this distance
AVOIDANCE_TURN = -2.5         # rad/s — turn right to avoid

# ── CBBA constants ────────────────────────────────────────────────────────────
CBBA_PUBLISH_RATE = 0.5       # seconds between bid publications


class CbbaAgent:

    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        # ── Webots devices ────────────────────────────────────────────────────
        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')
        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)
        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__gps = self.__robot.getDevice('gps')
        self.__gps.enable(int(self.__robot.getBasicTimeStep()))

        self.__compass = self.__robot.getDevice('compass')
        self.__compass.enable(int(self.__robot.getBasicTimeStep()))

        # ── Identity ──────────────────────────────────────────────────────────
        self.__robot_id = self.__robot.getName()
        self.__robot_idx = int(self.__robot_id.split('_')[1])

        # ── Waypoints from launch parameter ───────────────────────────────────
        waypoints_str = properties.get('waypoints', '')
        if waypoints_str:
            flat = [float(v) for v in waypoints_str.split(',')]
            self.__waypoints = [(flat[k], flat[k + 1]) for k in range(0, len(flat), 2)]
        else:
            # Fallback default waypoints
            self.__waypoints = [
                ( 0.0,  0.0),
                ( 0.5,  0.0),
                (-0.5,  0.0),
                ( 0.0,  0.5),
                ( 0.0, -0.5),
                ( 0.35,  0.35),
                (-0.35,  0.35),
                ( 0.35, -0.35),
                (-0.35, -0.35),
            ]

        # ── State ─────────────────────────────────────────────────────────────
        self.__pos_x = 0.0
        self.__pos_y = 0.0
        self.__heading = 0.0

        # CBBA tables (indexed by waypoint id)
        n = len(self.__waypoints)
        self.__my_bids = [-1.0] * n
        self.__my_assignment = -1
        self.__winners = [''] * n
        self.__winning_bids = [-1.0] * n

        # Other robots' positions for avoidance {robot_id: (x, y)}
        self.__other_positions = {}
        self.__cbba_lock = threading.Lock()

        # ── ROS 2 node ────────────────────────────────────────────────────────
        if not rclpy.ok():
            rclpy.init(args=None)

        self.__node = Node(f'cbba_agent_{self.__robot_id}')

        self.__node.create_subscription(
            Float64MultiArray,
            '/cbba/bids',
            self.__cbba_callback,
            10,
        )

        self.__bid_publisher = self.__node.create_publisher(
            Float64MultiArray,
            '/cbba/bids',
            10,
        )

        self.__node.create_timer(CBBA_PUBLISH_RATE, self.__publish_bids)

        self.__thread = threading.Thread(
            target=rclpy.spin, args=(self.__node,), daemon=True
        )
        self.__thread.start()

        self.__node.get_logger().info(
            f'{self.__robot_id} initialized with {len(self.__waypoints)} waypoints'
        )

    # ── ROS 2 callbacks ───────────────────────────────────────────────────────

    def __publish_bids(self):
        msg = Float64MultiArray()
        with self.__cbba_lock:
            # Recompute bids from current position
            for i, (wx, wy) in enumerate(self.__waypoints):
                dist = math.hypot(wx - self.__pos_x, wy - self.__pos_y)
                self.__my_bids[i] = 1.0 / (dist + 0.01)

            self.__run_consensus()
            bids = list(self.__my_bids)

        msg.data = [float(self.__robot_idx), self.__pos_x, self.__pos_y] + bids
        self.__bid_publisher.publish(msg)

    def __cbba_callback(self, msg):
        data = msg.data
        sender_idx = int(data[0])
        if sender_idx == self.__robot_idx:
            return

        sender_id = f'robot_{sender_idx}'
        pos_x, pos_y = data[1], data[2]
        scores = data[3:]

        with self.__cbba_lock:
            self.__other_positions[sender_id] = (pos_x, pos_y)
            for wid, score in enumerate(scores):
                if wid >= len(self.__waypoints):
                    break
                if score > self.__winning_bids[wid]:
                    self.__winning_bids[wid] = score
                    self.__winners[wid] = sender_id
            self.__run_consensus()

    def __run_consensus(self):
        """
        CBBA consensus phase (called under lock).
        Each robot wins its highest-scoring waypoint that no one outbids it on.
        Bids only increase → guaranteed convergence.
        """
        # Assert my own bids into the winning table
        for i in range(len(self.__waypoints)):
            my_score = self.__my_bids[i]
            if my_score < 0:
                continue
            if my_score > self.__winning_bids[i]:
                self.__winning_bids[i] = my_score
                self.__winners[i] = self.__robot_id

        # Find best waypoint I am currently winning
        best_wp = -1
        best_score = -1.0
        for i in range(len(self.__waypoints)):
            if self.__winners[i] != self.__robot_id:
                continue
            if self.__my_bids[i] > best_score:
                best_score = self.__my_bids[i]
                best_wp = i

        self.__my_assignment = best_wp

    # ── Navigation helpers ────────────────────────────────────────────────────

    def __update_position(self):
        """Read GPS for position, compass for heading."""
        values = self.__gps.getValues()
        self.__pos_x = values[0]
        self.__pos_y = values[1]

        north = self.__compass.getValues()
        self.__heading = math.atan2(north[0], north[2])

    def __navigate_to(self, wx, wy):
        dx = wx - self.__pos_x
        dy = wy - self.__pos_y
        dist = math.hypot(dx, dy)

        if dist < WAYPOINT_REACHED_DIST:
            with self.__cbba_lock:
                if self.__my_assignment >= 0:
                    wid = self.__my_assignment
                    self.__winning_bids[wid] = -1.0
                    self.__winners[wid] = ''
                    self.__my_bids[wid] = -1.0
                    self.__my_assignment = -1
            return 0.0, 0.0

        target_heading = math.atan2(dy, dx)
        heading_error = target_heading - self.__heading

        # Normalize to [-pi, pi]
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        angular = ANGULAR_GAIN * heading_error
        linear = LINEAR_SPEED * max(0.2, 1.0 - abs(heading_error) / math.pi)

        left_vel = (linear - angular * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        right_vel = (linear + angular * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        left_vel = max(-MAX_WHEEL_VELOCITY, min(MAX_WHEEL_VELOCITY, left_vel))
        right_vel = max(-MAX_WHEEL_VELOCITY, min(MAX_WHEEL_VELOCITY, right_vel))

        return left_vel, right_vel

    def __check_avoidance(self):
        with self.__cbba_lock:
            others = dict(self.__other_positions)

        for rid, (ox, oy) in others.items():
            dist = math.hypot(ox - self.__pos_x, oy - self.__pos_y)
            if dist < AVOIDANCE_DIST:
                angular = AVOIDANCE_TURN
                linear = 0.0
                left_vel = (linear - angular * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
                right_vel = (linear + angular * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
                left_vel = max(-MAX_WHEEL_VELOCITY, min(MAX_WHEEL_VELOCITY, left_vel))
                right_vel = max(-MAX_WHEEL_VELOCITY, min(MAX_WHEEL_VELOCITY, right_vel))
                return left_vel, right_vel

        return None

    # ── Main Webots step ──────────────────────────────────────────────────────

    def step(self):
        # 1. Update position and heading from sensors
        self.__update_position()

        # 2. Reactive avoidance takes priority over CBBA
        avoidance = self.__check_avoidance()
        if avoidance is not None:
            left_vel, right_vel = avoidance
            self.__left_motor.setVelocity(left_vel)
            self.__right_motor.setVelocity(right_vel)
            return

        # 3. Navigate to CBBA-assigned waypoint
        with self.__cbba_lock:
            assignment = self.__my_assignment

        if assignment >= 0:
            wx, wy = self.__waypoints[assignment]
            left_vel, right_vel = self.__navigate_to(wx, wy)
        else:
            # No assignment yet — spin slowly while waiting for consensus
            left_vel = 0.3
            right_vel = -0.3

        self.__left_motor.setVelocity(left_vel)
        self.__right_motor.setVelocity(right_vel)