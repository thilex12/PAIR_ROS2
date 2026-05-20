import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String
import yaml


@dataclass(frozen=True)
class Task:
    task_id: str
    x: float
    y: float
    reward: float


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

        self.declare_parameter('robot_name', '')
        self.declare_parameter('tasks_config', '')
        self.declare_parameter('bundle_size', 2)
        self.declare_parameter('goal_tolerance', 0.08)
        self.declare_parameter('max_linear_speed', 0.12)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('linear_gain', 0.9)
        self.declare_parameter('angular_gain', 2.5)
        self.declare_parameter('clearance_threshold', 0.09)

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

        self.__tasks = self.__load_tasks(tasks_config)
        self.__pose: tuple[float, float, float] | None = None
        self.__left_range = float('inf')
        self.__right_range = float('inf')
        self.__bundle: list[str] = []
        self.__completed_tasks: set[str] = set()
        self.__abandoned_tasks: set[str] = set()
        self.__winner_table: dict[str, WinnerEntry] = {}
        self.__peer_state: dict[str, dict[str, Any]] = {}

        self.create_subscription(PoseStamped, 'pose', self.__pose_callback, 1)
        self.create_subscription(Range, 'left_sensor', self.__left_sensor_callback, 1)
        self.create_subscription(Range, 'right_sensor', self.__right_sensor_callback, 1)
        self.create_subscription(String, '/cbba/state', self.__state_callback, 10)

        self.__cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.__state_publisher = self.create_publisher(String, '/cbba/state', 10)
        self.__assignment_publisher = self.create_publisher(String, '/cbba/assignment', 10)

        self.create_timer(0.2, self.__timer_callback)

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
            tasks.append(
                Task(
                    task_id=str(task_data['id']),
                    x=float(task_data['x']),
                    y=float(task_data['y']),
                    reward=float(task_data.get('reward', 1.0)),
                )
            )

        return tasks

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

    def __timer_callback(self) -> None:
        if self.__pose is None or not self.__tasks:
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

    def __rebuild_bundle(self) -> None:
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

            if selected_task is None or selected_score <= 0.0:
                break

            self.__bundle.append(selected_task.task_id)
            self.__winner_table[selected_task.task_id] = WinnerEntry(robot=self.__robot_name, score=selected_score)
            current_x = selected_task.x
            current_y = selected_task.y

    def __score_for_task(self, task: Task, from_x: float, from_y: float) -> float:
        travel_cost = _distance(from_x, from_y, task.x, task.y)
        return task.reward - travel_cost

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
                task_id: {
                    'robot': winner.robot,
                    'score': winner.score,
                }
                for task_id, winner in self.__winner_table.items()
            },
        }

        message = String()
        message.data = json.dumps(payload)
        self.__state_publisher.publish(message)

    def __publish_assignment_summary(self) -> None:
        assigned_tasks = {
            task_id: winner.robot
            for task_id, winner in self.__winner_table.items()
        }

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

    def __publish_command(self) -> None:
        command_message = Twist()

        if not self.__bundle:
            self.__cmd_vel_publisher.publish(command_message)
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

        heading = math.atan2(target_task.y - current_y, target_task.x - current_x)
        heading_error = _normalize_angle(heading - current_yaw)

        linear_speed = min(self.__max_linear_speed, self.__linear_gain * distance)
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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()