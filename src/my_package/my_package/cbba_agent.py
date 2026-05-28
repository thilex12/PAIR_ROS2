# src\my_package\my_package\cbba_agent.py
import json
import heapq
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
import time
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Range
from std_msgs.msg import String
import yaml


@dataclass(frozen=True)
class Task:
    task_id: str
    x: float
    y: float
    reward: float


@dataclass(frozen=True)
class StaticObstacle:
    x: float
    y: float
    radius: float


@dataclass(frozen=True)
class WallSegment:
    x1: float
    y1: float
    x2: float
    y2: float
    thickness: float


@dataclass
class WinnerEntry:
    robot: str
    score: float


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _distance(x_0: float, y_0: float, x_1: float, y_1: float) -> float:
    return math.hypot(x_1 - x_0, y_1 - y_0)


class CbbaAgent(Node):
    def __init__(self) -> None:
        super().__init__('cbba_agent')

        # parameters
        self.declare_parameter('robot_name', '')
        self.declare_parameter('tasks_config', '')
        self.declare_parameter('bundle_size',3)
        self.declare_parameter('goal_tolerance', 0.08)
        self.declare_parameter('max_linear_speed', 0.12)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('linear_gain', 0.9)
        self.declare_parameter('angular_gain', 2.5)
        self.declare_parameter('clearance_threshold', 0.09)
        self.declare_parameter('path_resolution', 0.05)
        self.declare_parameter('arena_radius')
        self.declare_parameter('robot_avoidance_radius', 0.12)
        self.declare_parameter('obstacles_config', '')
        self.declare_parameter('path_publish_interval', 0.6)
        self.declare_parameter('max_path_points', 50)
        self.declare_parameter('enable_path_visualization', True)
        self.declare_parameter('walls_json', '[]')
        self.declare_parameter('robot_count', 1)
        # Multiplier applied to travel cost in the bid score: score = reward - distance_weight * travel_cost.
        # A value of 1.0 (default) means raw path length.
        # Increase (e.g. 5.0–10.0) to make proximity dominate over reward differences,
        # so the closest robot almost always wins regardless of reward magnitude.
        self.declare_parameter('distance_weight', 5.0)

        self.__robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        if not self.__robot_name:
            self.__robot_name = self.get_namespace().strip('/') or 'robot'

        tasks_config = self.get_parameter('tasks_config').get_parameter_value().string_value
        self.__bundle_size = max(1, self.get_parameter('bundle_size').get_parameter_value().integer_value)
        self.__goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.__max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.__max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.__linear_gain = self.get_parameter('linear_gain').get_parameter_value().double_value
        self.__angular_gain = self.get_parameter('angular_gain').get_parameter_value().double_value
        self.__clearance_threshold = self.get_parameter('clearance_threshold').get_parameter_value().double_value
        self.__path_resolution = self.get_parameter('path_resolution').get_parameter_value().double_value
        arena_radius_param = self.get_parameter('arena_radius').get_parameter_value()
        self.__arena_radius = (
            arena_radius_param.double_value
            if arena_radius_param.type == Parameter.Type.DOUBLE
            else 0.75
        )
        self.__robot_avoidance_radius = self.get_parameter('robot_avoidance_radius').get_parameter_value().double_value
        self.__path_publish_interval = max(0.0, self.get_parameter('path_publish_interval').get_parameter_value().double_value)
        self.__max_path_points = max(1, self.get_parameter('max_path_points').get_parameter_value().integer_value)
        self.__enable_path_visualization = self.get_parameter('enable_path_visualization').get_parameter_value().bool_value
        # Total number of robots in the simulation (including self).
        # Used to delay the first CBBA bid until all peers have been heard from,
        # preventing spurious path displays before consensus is reached.
        self.__robot_count = max(1, self.get_parameter('robot_count').get_parameter_value().integer_value)
        self.__distance_weight = max(0.1, self.get_parameter('distance_weight').get_parameter_value().double_value)

        self.__last_path_pub_time = 0.0
        self.__last_published_path: list[tuple[float, float]] | None = None

        self.__tasks = self.__load_tasks(tasks_config)

        obstacles_config = self.get_parameter('obstacles_config').get_parameter_value().string_value
        self.__static_obstacles, self.__walls = self.__load_static_obstacles(obstacles_config)

        walls_json = self.get_parameter('walls_json').get_parameter_value().string_value
        self.__walls = self.__walls + self.__parse_walls_json(walls_json)

        self.__pose: tuple[float, float, float] | None = None
        self.__left_range = float('inf')
        self.__right_range = float('inf')
        self.__bundle: list[str] = []
        self.__completed_tasks: set[str] = set()
        self.__abandoned_tasks: set[str] = set()
        self.__winner_table: dict[str, WinnerEntry] = {}
        self.__peer_state: dict[str, dict[str, Any]] = {}

        # Cache for A* path lengths used during bid scoring.
        # Key: (from_x_cell, from_y_cell, to_x_cell, to_y_cell)
        # Value: float path length in world units
        # Cleared at the start of every __rebuild_bundle call.
        self.__path_length_cache: dict[tuple[int, int, int, int], float] = {}

        # subscriptions
        self.create_subscription(PoseStamped, 'pose', self.__pose_callback, 1)
        self.create_subscription(Range, 'left_sensor', self.__left_sensor_callback, 1)
        self.create_subscription(Range, 'right_sensor', self.__right_sensor_callback, 1)
        self.create_subscription(String, '/cbba/state', self.__state_callback, 10)

        # publishers
        self.__cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.__state_publisher = self.create_publisher(String, '/cbba/state', 10)
        self.__assignment_publisher = self.create_publisher(String, '/cbba/assignment', 10)
        self.__path_publisher = self.create_publisher(String, 'cbba_path', 10)

        self.create_timer(0.2, self.__timer_callback)

        self.get_logger().info(
            f'{self.__robot_name}: A* planner loaded {len(self.__walls)} wall segment(s).'
        )

    def __parse_walls_json(self, walls_json: str) -> list[WallSegment]:
        segments: list[WallSegment] = []
        try:
            raw = json.loads(walls_json)
            for entry in raw:
                segments.append(WallSegment(
                    x1=float(entry['x1']),
                    y1=float(entry['y1']),
                    x2=float(entry['x2']),
                    y2=float(entry['y2']),
                    thickness=float(entry.get('thickness', 0.04)),
                ))
        except Exception as exc:
            self.get_logger().warning(f'Failed to parse walls_json: {exc}')
        return segments

    def __load_tasks(self, tasks_config: str) -> list[Task]:
        if not tasks_config:
            return []
        config_path = Path(tasks_config)
        if not config_path.is_file():
            self.get_logger().warning(f'Tasks config not found: {tasks_config}')
            return []
        with config_path.open('r', encoding='utf-8') as tasks_file:
            configuration = yaml.safe_load(tasks_file) or {}
        tasks = []
        for task_data in configuration.get('tasks', []):
            tasks.append(Task(
                task_id=str(task_data['id']),
                x=float(task_data['x']),
                y=float(task_data['y']),
                reward=float(task_data.get('reward', 1.0)),
            ))
        return tasks

    def __load_static_obstacles(self, obstacles_config: str) -> tuple[list[StaticObstacle], list[WallSegment]]:
        if not obstacles_config:
            return [], []
        config_path = Path(obstacles_config)
        if not config_path.is_file():
            self.get_logger().warning(f'Obstacles config not found: {obstacles_config}')
            return [], []
        with config_path.open('r', encoding='utf-8') as obstacles_file:
            configuration = yaml.safe_load(obstacles_file) or {}
        obstacles: list[StaticObstacle] = []
        for obstacle_data in configuration.get('static_obstacles', []):
            obstacles.append(StaticObstacle(
                x=float(obstacle_data['x']),
                y=float(obstacle_data['y']),
                radius=float(obstacle_data.get('radius', 0.08)),
            ))
        walls: list[WallSegment] = []
        for wall_data in configuration.get('walls', []):
            walls.append(WallSegment(
                x1=float(wall_data['x1']),
                y1=float(wall_data['y1']),
                x2=float(wall_data['x2']),
                y2=float(wall_data['y2']),
                thickness=float(wall_data.get('thickness', 0.04)),
            ))
        return obstacles, walls

    def __pose_callback(self, message: PoseStamped) -> None:
        quaternion_z = message.pose.orientation.z
        quaternion_w = message.pose.orientation.w
        yaw = math.atan2(2.0 * quaternion_w * quaternion_z, 1.0 - 2.0 * quaternion_z * quaternion_z)
        self.__pose = (message.pose.position.x, message.pose.position.y, yaw)

    def __left_sensor_callback(self, message: Range) -> None:
        self.__left_range = message.range

    def __right_sensor_callback(self, message: Range) -> None:
        self.__right_range = message.range

    def __state_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        sender = payload.get('robot_name')
        if not sender or sender == self.__robot_name:
            return
        self.__peer_state[sender] = payload

    def __peers_ready(self) -> bool:
        """Return True once we have heard from every other robot at least once.

        With robot_count == 1 there are no peers, so we are always ready.
        This prevents the agent from bidding (and drawing a path) before CBBA
        consensus can even start, which was causing the spurious early path display.
        """
        expected_peers = self.__robot_count - 1
        return len(self.__peer_state) >= expected_peers

    def __timer_callback(self) -> None:
        if self.__pose is None or not self.__tasks:
            self.__publish_idle_state()
            return

        # Publish our state every tick so peers can discover us quickly,
        # but don't bid or navigate until all peers have checked in.
        self.__publish_state()

        if not self.__peers_ready():
            self.__publish_idle_state()
            return

        self.__merge_peer_winners()
        self.__merge_peer_completions()
        if self.__all_tasks_completed():
            self.__bundle.clear()
            self.__publish_idle_state()
            self.__publish_state()
            self.__publish_assignment_summary()
            return
        self.__rebuild_bundle()
        self.__publish_command()
        self.__publish_state()
        self.__publish_assignment_summary()

    def __merge_peer_winners(self) -> None:
        for peer_state in self.__peer_state.values():
            peer_winners = peer_state.get('winner_table', {})
            for task_id, winner_data in peer_winners.items():
                peer_entry = WinnerEntry(
                    robot=str(winner_data.get('robot', '')),
                    score=float(winner_data.get('score', 0.0)),
                )
                local_entry = self.__winner_table.get(task_id)
                if local_entry is None or self.__is_better_winner(peer_entry, local_entry):
                    if local_entry is not None and local_entry.robot == self.__robot_name and task_id in self.__bundle:
                        self.__abandon_from_task(task_id, peer_entry)
                    self.__winner_table[task_id] = peer_entry

    def __merge_peer_completions(self) -> None:
        peer_completed_tasks: set[str] = set()
        for peer_state in self.__peer_state.values():
            peer_completed_tasks.update(str(task_id) for task_id in peer_state.get('completed_tasks', []))
        for task_id in peer_completed_tasks:
            if task_id in self.__completed_tasks:
                continue
            self.__completed_tasks.add(task_id)
            self.__abandoned_tasks.discard(task_id)
            self.__winner_table.pop(task_id, None)
            if task_id in self.__bundle:
                self.__bundle.remove(task_id)

    def __all_tasks_completed(self) -> bool:
        if not self.__tasks:
            return True
        completed_task_ids = set(self.__completed_tasks)
        for peer_state in self.__peer_state.values():
            completed_task_ids.update(str(task_id) for task_id in peer_state.get('completed_tasks', []))
        return all(task.task_id in completed_task_ids for task in self.__tasks)

    def __abandon_from_task(self, task_id: str, winning_peer: WinnerEntry | None = None) -> None:
        if task_id not in self.__bundle:
            return
        abandon_index = self.__bundle.index(task_id)
        abandoned_bundle = self.__bundle[abandon_index:]
        for abandoned_task_id in abandoned_bundle:
            self.__abandoned_tasks.add(abandoned_task_id)
            if abandoned_task_id not in self.__completed_tasks:
                local_entry = self.__winner_table.get(abandoned_task_id)
                if local_entry is not None and local_entry.robot == self.__robot_name:
                    self.__winner_table.pop(abandoned_task_id, None)
        self.__bundle = self.__bundle[:abandon_index]
        if winning_peer is not None:
            self.get_logger().info(
                f'Robot {self.__robot_name} abandons task {task_id} after losing to '
                f'{winning_peer.robot} with score {winning_peer.score:.3f}'
            )
        else:
            self.get_logger().info(f'Robot {self.__robot_name} abandons task {task_id}')

    def __is_better_winner(self, candidate: WinnerEntry, incumbent: WinnerEntry) -> bool:
        if candidate.score != incumbent.score:
            return candidate.score > incumbent.score
        return candidate.robot < incumbent.robot

    # -- A* utilities -------------------------------------------------

    def __world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (int(round(x / self.__path_resolution)), int(round(y / self.__path_resolution)))

    def __cell_to_world(self, cx: int, cy: int) -> tuple[float, float]:
        return (cx * self.__path_resolution, cy * self.__path_resolution)

    def __distance_to_segment(self, px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
        dx = x2 - x1
        dy = y2 - y1
        if dx == 0 and dy == 0:
            return _distance(px, py, x1, y1)
        t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
        t = max(0.0, min(1.0, t))
        qx = x1 + t * dx
        qy = y1 + t * dy
        return _distance(px, py, qx, qy)

    def __cell_is_blocked_static(self, cx: int, cy: int, start_cell: tuple[int, int], goal_cell: tuple[int, int]) -> bool:
        """Check only static obstacles (arena boundary, circular obstacles, walls).

        Used by __astar_path_length during CBBA bid scoring so that peer robot
        positions — which change every tick — do not pollute the cached cost
        estimates and cause unstable bid oscillations.
        """
        if (cx, cy) == start_cell or (cx, cy) == goal_cell:
            return False
        wx, wy = self.__cell_to_world(cx, cy)
        if math.hypot(wx, wy) > self.__arena_radius:
            return True
        for obs in self.__static_obstacles:
            if _distance(wx, wy, obs.x, obs.y) <= obs.radius + self.__path_resolution * 0.5:
                return True
        _ROBOT_RADIUS = 0.045
        _WALL_SAFETY_MARGIN = 0.06
        for w in self.__walls:
            clearance = w.thickness / 2.0 + _ROBOT_RADIUS + _WALL_SAFETY_MARGIN
            if self.__distance_to_segment(wx, wy, w.x1, w.y1, w.x2, w.y2) <= clearance:
                return True
        return False

    def __cell_is_blocked(self, cx: int, cy: int, start_cell: tuple[int, int], goal_cell: tuple[int, int]) -> bool:
        """Full check including dynamic peer positions. Used during navigation."""
        if self.__cell_is_blocked_static(cx, cy, start_cell, goal_cell):
            return True
        for peer_state in self.__peer_state.values():
            pose = peer_state.get('pose', {})
            try:
                px = float(pose.get('x'))
                py = float(pose.get('y'))
            except Exception:
                continue
            wx, wy = self.__cell_to_world(cx, cy)
            if _distance(wx, wy, px, py) <= self.__robot_avoidance_radius:
                return True
        return False

    def __astar_path(self, sx: float, sy: float, gx: float, gy: float, static_only: bool = False) -> list[tuple[float, float]]:
        start = self.__world_to_cell(sx, sy)
        goal = self.__world_to_cell(gx, gy)
        if start == goal:
            return [(gx, gy)]

        blocked = self.__cell_is_blocked_static if static_only else self.__cell_is_blocked

        open_heap: list[tuple[float, int, int]] = []
        heapq.heappush(open_heap, (0.0, start[0], start[1]))
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        gscore: dict[tuple[int, int], float] = {start: 0.0}
        closed: set[tuple[int, int]] = set()
        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

        while open_heap:
            _, cx, cy = heapq.heappop(open_heap)
            current = (cx, cy)
            if current in closed:
                continue
            if current == goal:
                path_cells = []
                cur = current
                while cur != start:
                    path_cells.append(cur)
                    cur = came_from[cur]
                path_cells.reverse()
                path = [self.__cell_to_world(px, py) for px, py in path_cells]
                if not path or _distance(path[-1][0], path[-1][1], gx, gy) > self.__path_resolution * 0.6:
                    path.append((gx, gy))
                return path
            closed.add(current)
            for dx, dy in neighbors:
                nx, ny = cx + dx, cy + dy
                neighbor = (nx, ny)
                if blocked(nx, ny, start, goal):
                    continue
                tentative_g = gscore.get(current, float('inf')) + (math.sqrt(2) if dx != 0 and dy != 0 else 1.0)
                if tentative_g < gscore.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g
                    heuristic = math.hypot(goal[0] - nx, goal[1] - ny)
                    heapq.heappush(open_heap, (tentative_g + heuristic, nx, ny))

        return [(gx, gy)]

    def __astar_path_length(self, sx: float, sy: float, gx: float, gy: float) -> float:
        """Return the A* path length in world units, with per-rebuild caching.

        The cache key is based on grid cells so nearby positions hash to the
        same entry and avoid redundant searches.  Peer-robot positions are NOT
        included in the key — they change every timer tick but we only need a
        stable cost estimate for the bid; the actual navigation path is
        recomputed fresh in __publish_command.
        """
        sc = self.__world_to_cell(sx, sy)
        gc = self.__world_to_cell(gx, gy)
        cache_key = (sc[0], sc[1], gc[0], gc[1])

        if cache_key in self.__path_length_cache:
            return self.__path_length_cache[cache_key]

        path = self.__astar_path(sx, sy, gx, gy, static_only=True)

        # Sum Euclidean distances between consecutive waypoints.
        length = 0.0
        prev = (sx, sy)
        for wp in path:
            length += _distance(prev[0], prev[1], wp[0], wp[1])
            prev = wp

        euclidean = _distance(sx, sy, gx, gy)
        is_fallback = len(path) == 1 and path[0] == (gx, gy)
        penalised = False

        # If A* only returned the goal (fallback), use straight-line distance
        # but apply a large penalty so robots behind walls bid less.
        if is_fallback:
            if length > euclidean * 1.05:
                pass  # genuine detour path (shouldn't happen with fallback, but be safe)
            else:
                # Fallback: penalise heavily to discourage the blocked robot
                length = euclidean * 3.0
                penalised = True

        self.get_logger().debug(
            f'[A*] ({sx:.2f},{sy:.2f})->({gx:.2f},{gy:.2f}): '
            f'eucl={euclidean:.3f} astar={length:.3f} '
            f'waypoints={len(path)} fallback={is_fallback} penalised={penalised}'
        )
        if penalised:
            self.get_logger().warn(
                f'[A* PENALTY] {self.__robot_name}: no path found from '
                f'({sx:.2f},{sy:.2f}) to ({gx:.2f},{gy:.2f}) — '
                f'applying x3 penalty (eucl={euclidean:.3f} -> cost={length:.3f})'
            )

        self.__path_length_cache[cache_key] = length
        return length

    # -- CBBA core ----------------------------------------------------

    def __rebuild_bundle(self) -> None:
        # Clear the path-length cache at the start of each rebuild so costs
        # reflect the current world state (completed tasks, peer positions).
        self.__path_length_cache.clear()

        bundle_before = list(self.__bundle)

        for task_id in list(self.__bundle):
            winner = self.__winner_table.get(task_id)
            if winner is not None and winner.robot != self.__robot_name:
                self.__abandon_from_task(task_id, winner)
                break

        current_x, current_y, _ = self.__pose
        if self.__bundle:
            last_task = self.__task_by_id(self.__bundle[-1])
            current_x = last_task.x
            current_y = last_task.y

        while len(self.__bundle) < self.__bundle_size:
            selected_task: Task | None = None
            selected_score = float('-inf')

            for task in self.__tasks:
                if task.task_id in self.__completed_tasks or task.task_id in self.__bundle:
                    continue

                score = self.__score_for_task(task, current_x, current_y)
                winner = self.__winner_table.get(task.task_id)
                if winner is not None and winner.robot not in ('', self.__robot_name):
                    if winner.score >= score:
                        continue

                if score > selected_score:
                    selected_score = score
                    selected_task = task

            if selected_task is None:
                break

            self.__bundle.append(selected_task.task_id)
            self.__winner_table[selected_task.task_id] = WinnerEntry(robot=self.__robot_name, score=selected_score)
            current_x = selected_task.x
            current_y = selected_task.y

        if self.__bundle != bundle_before:
            # Log the full bundle with per-task travel cost + score breakdown.
            cx, cy, _ = self.__pose
            parts = []
            for tid in self.__bundle:
                t = self.__task_by_id(tid)
                cost = self.__astar_path_length(cx, cy, t.x, t.y)
                score = t.reward - self.__distance_weight * cost
                penalised = ''
                # Re-detect fallback to flag it in the log.
                raw_path = self.__astar_path(cx, cy, t.x, t.y, static_only=True)
                eucl = _distance(cx, cy, t.x, t.y)
                if len(raw_path) == 1 and raw_path[0] == (t.x, t.y) and cost >= eucl * 2.9:
                    penalised = ' [PENALISED]'
                parts.append('%s(cost=%.2f score=%.2f%s)' % (tid, cost, score, penalised))
                cx, cy = t.x, t.y
            self.get_logger().info(
                '[BUNDLE] %s: %s' % (self.__robot_name, ' -> '.join(parts) if parts else '(empty)')
            )

    def __score_for_task(self, task: Task, from_x: float, from_y: float) -> float:
        """Score = reward - distance_weight * A*_travel_cost.

        distance_weight > 1 makes proximity dominate over reward differences,
        ensuring the closest robot wins even when rewards vary significantly.
        Default is 5.0 so a 1m difference in path length outweighs a 2-point
        reward difference on the typical arena scale.
        """
        travel_cost = self.__astar_path_length(from_x, from_y, task.x, task.y)
        return task.reward - self.__distance_weight * travel_cost

    def __task_by_id(self, task_id: str) -> Task:
        for task in self.__tasks:
            if task.task_id == task_id:
                return task
        raise KeyError(task_id)

    def __publish_state(self) -> None:
        assert self.__pose is not None
        payload = {
            'robot_name': self.__robot_name,
            'pose': {
                'x': self.__pose[0],
                'y': self.__pose[1],
                'yaw': self.__pose[2],
            },
            'bundle': self.__bundle,
            'completed_tasks': sorted(self.__completed_tasks),
            'abandoned_tasks': sorted(self.__abandoned_tasks),
            'winner_table': {
                task_id: {'robot': winner.robot, 'score': winner.score}
                for task_id, winner in self.__winner_table.items()
            },
        }
        message = String()
        message.data = json.dumps(payload)
        self.__state_publisher.publish(message)

    def __publish_assignment_summary(self) -> None:
        assigned_tasks = {task_id: winner.robot for task_id, winner in self.__winner_table.items()}
        payload = {
            'robot_name': self.__robot_name,
            'bundle': list(self.__bundle),
            'assigned_tasks': assigned_tasks,
            'completed_tasks': sorted(self.__completed_tasks),
            'abandoned_tasks': sorted(self.__abandoned_tasks),
        }
        message = String()
        message.data = json.dumps(payload)
        self.__assignment_publisher.publish(message)

    def __publish_idle_state(self) -> None:
        self.__cmd_vel_publisher.publish(Twist())

    def __clear_path_visualization(self) -> None:
        """Publish an empty path so the driver parks all spheres back at y=999."""
        if not self.__enable_path_visualization:
            return
        payload = {'robot_name': self.__robot_name, 'task_id': '', 'path': []}
        msg = String()
        msg.data = json.dumps(payload)
        self.__path_publisher.publish(msg)
        self.__last_published_path = None

    def __publish_command(self) -> None:
        command_message = Twist()
        if not self.__bundle:
            self.__cmd_vel_publisher.publish(command_message)
            self.__clear_path_visualization()
            return

        target_task = self.__task_by_id(self.__bundle[0])
        current_x, current_y, current_yaw = self.__pose
        distance = _distance(current_x, current_y, target_task.x, target_task.y)

        if distance <= self.__goal_tolerance:
            self.__completed_tasks.add(target_task.task_id)
            self.__bundle.pop(0)
            self.__winner_table.pop(target_task.task_id, None)
            self.__abandoned_tasks.discard(target_task.task_id)
            self.__cmd_vel_publisher.publish(command_message)
            return

        path = self.__astar_path(current_x, current_y, target_task.x, target_task.y)

        if len(path) > self.__max_path_points:
            step = max(1, len(path) // self.__max_path_points)
            trimmed = [path[i] for i in range(0, len(path), step)][:self.__max_path_points]
            if trimmed[-1] != path[-1]:
                trimmed[-1] = path[-1]
            path_to_publish = trimmed
        else:
            path_to_publish = path

        if self.__enable_path_visualization:
            now = time.time()
            try:
                should_publish = False
                if now - self.__last_path_pub_time >= self.__path_publish_interval:
                    if self.__last_published_path != path_to_publish:
                        should_publish = True
                if should_publish:
                    payload = {'robot_name': self.__robot_name, 'task_id': target_task.task_id, 'path': [[float(x), float(y)] for x, y in path_to_publish]}
                    msg = String()
                    msg.data = json.dumps(payload)
                    self.__path_publisher.publish(msg)
                    self.__last_path_pub_time = now
                    self.__last_published_path = list(path_to_publish)
            except Exception:
                pass

        nav_x, nav_y = path[0] if path else (target_task.x, target_task.y)
        heading = math.atan2(nav_y - current_y, nav_x - current_x)
        heading_error = _normalize_angle(heading - current_yaw)

        nav_distance = _distance(current_x, current_y, nav_x, nav_y)
        linear_speed = min(self.__max_linear_speed, self.__linear_gain * nav_distance)
        linear_speed *= max(0.0, 1.0 - abs(heading_error) / math.pi)
        angular_speed = max(-self.__max_angular_speed, min(self.__max_angular_speed, self.__angular_gain * heading_error))

        if self.__left_range < self.__clearance_threshold or self.__right_range < self.__clearance_threshold:
            command_message.linear.x = 0.0
            command_message.angular.z = -angular_speed if self.__left_range < self.__right_range else angular_speed
        else:
            command_message.linear.x = linear_speed
            command_message.angular.z = angular_speed

        self.__cmd_vel_publisher.publish(command_message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CbbaAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()