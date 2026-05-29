from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


@dataclass(frozen=True)
class Task:
    """Navigation task to bid on."""

    task_id: str
    x: float
    y: float
    reward: float


@dataclass(frozen=True)
class WinnerEntry:
    """Winning robot and its bid score for a task."""

    robot: str
    score: float


def _distance(x_0: float, y_0: float, x_1: float, y_1: float) -> float:
    return math.hypot(x_1 - x_0, y_1 - y_0)


def _quaternion_from_yaw(yaw: float) -> Quaternion:
    half_yaw = yaw * 0.5
    quaternion = Quaternion()
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = math.sin(half_yaw)
    quaternion.w = math.cos(half_yaw)
    return quaternion


class CbbaNode(Node):
    """Distributed CBBA node that sends winning tasks to Nav2 goals."""

    def __init__(self) -> None:
        super().__init__('cbba_node')

        self.declare_parameter('robot_name', '')
        self.declare_parameter('robot_count', 2)
        self.declare_parameter('bundle_size', 1)
        self.declare_parameter('tasks_config', '')
        self.declare_parameter('pose_topic', 'amcl_pose')
        self.declare_parameter('state_topic', '/cbba/state')
        self.declare_parameter('assignment_topic', '/cbba/assignment')
        self.declare_parameter('navigation_action', 'navigate_to_pose')
        self.declare_parameter('goal_tolerance', 0.25)
        self.declare_parameter('distance_weight', 5.0)
        self.declare_parameter('state_publish_period', 0.5)

        self._robot_name = self.get_parameter('robot_name').value or self.get_namespace().strip('/') or 'robot'
        self._robot_count = max(1, int(self.get_parameter('robot_count').value))
        self._bundle_size = max(1, int(self.get_parameter('bundle_size').value))
        self._goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self._distance_weight = max(0.1, float(self.get_parameter('distance_weight').value))
        self._state_publish_period = max(0.1, float(self.get_parameter('state_publish_period').value))
        self._navigation_action = str(self.get_parameter('navigation_action').value).strip() or 'navigate_to_pose'

        self._tasks = self._load_tasks(str(self.get_parameter('tasks_config').value))
        self._pose: tuple[float, float, float] | None = None
        self._peer_state: dict[str, dict[str, Any]] = {}
        self._winner_table: dict[str, WinnerEntry] = {}
        self._bundle: list[str] = []
        self._completed_tasks: set[str] = set()
        self._abandoned_tasks: set[str] = set()
        self._active_goal_task: str | None = None
        self._goal_handle: Any | None = None

        self._state_topic = str(self.get_parameter('state_topic').value)
        self._assignment_topic = str(self.get_parameter('assignment_topic').value)

        self._state_publisher = self.create_publisher(String, self._state_topic, 10)
        self._assignment_publisher = self.create_publisher(String, self._assignment_topic, 10)
        self._pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter('pose_topic').value),
            self._pose_callback,
            10,
        )
        self._state_subscription = self.create_subscription(String, self._state_topic, self._state_callback, 10)

        self._navigate_client = ActionClient(self, NavigateToPose, self._navigation_action)
        self._timer = self.create_timer(self._state_publish_period, self._timer_callback)

        self.get_logger().info(
            f'{self._robot_name}: loaded {len(self._tasks)} task(s) with bundle size {self._bundle_size}'
        )

    def _load_tasks(self, tasks_config: str) -> list[Task]:
        if not tasks_config:
            self.get_logger().warning('No tasks_config parameter provided.')
            return []

        config_path = Path(tasks_config)
        if not config_path.is_file():
            self.get_logger().warning(f'Tasks config not found: {tasks_config}')
            return []

        with config_path.open('r', encoding='utf-8') as tasks_file:
            configuration = yaml.safe_load(tasks_file) or {}

        tasks: list[Task] = []
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

    def _pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z),
            1.0 - 2.0 * (orientation.z * orientation.z),
        )
        self._pose = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            yaw,
        )

    def _state_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return

        sender = str(payload.get('robot_name', ''))
        if not sender or sender == self._robot_name:
            return

        self._peer_state[sender] = payload

    def _peers_ready(self) -> bool:
        expected_peers = self._robot_count - 1
        return len(self._peer_state) >= expected_peers

    def _task_by_id(self, task_id: str) -> Task | None:
        for task in self._tasks:
            if task.task_id == task_id:
                return task
        return None

    def _state_snapshot(self) -> dict[str, Any]:
        winner_table = {
            task_id: {'robot': entry.robot, 'score': entry.score}
            for task_id, entry in self._winner_table.items()
        }
        return {
            'robot_name': self._robot_name,
            'bundle': list(self._bundle),
            'completed_tasks': sorted(self._completed_tasks),
            'abandoned_tasks': sorted(self._abandoned_tasks),
            'winner_table': winner_table,
            'active_goal_task': self._active_goal_task,
            'pose': list(self._pose) if self._pose is not None else None,
        }

    def _publish_state(self) -> None:
        message = String()
        message.data = json.dumps(self._state_snapshot())
        self._state_publisher.publish(message)

    def _publish_assignment_summary(self) -> None:
        assigned = {
            task_id: entry.robot
            for task_id, entry in self._winner_table.items()
            if task_id in self._bundle and entry.robot == self._robot_name
        }
        message = String()
        message.data = json.dumps(
            {
                'robot_name': self._robot_name,
                'bundle': list(self._bundle),
                'assigned_tasks': assigned,
            }
        )
        self._assignment_publisher.publish(message)

    def _all_completed_task_ids(self) -> set[str]:
        completed = set(self._completed_tasks)
        for peer_state in self._peer_state.values():
            completed.update(str(task_id) for task_id in peer_state.get('completed_tasks', []))
        return completed

    def _all_abandoned_task_ids(self) -> set[str]:
        abandoned = set(self._abandoned_tasks)
        for peer_state in self._peer_state.values():
            abandoned.update(str(task_id) for task_id in peer_state.get('abandoned_tasks', []))
        return abandoned

    def _all_states(self) -> list[dict[str, Any]]:
        states = [self._state_snapshot()]
        states.extend(self._peer_state.values())
        return states

    def _is_better(self, candidate: WinnerEntry, incumbent: WinnerEntry, stable_robot: str | None = None) -> bool:
        if candidate.score > incumbent.score:
            return True
        if math.isclose(candidate.score, incumbent.score):
            if stable_robot is not None:
                if candidate.robot == stable_robot and incumbent.robot != stable_robot:
                    return True
                if incumbent.robot == stable_robot and candidate.robot != stable_robot:
                    return False
            return candidate.robot < incumbent.robot
        return False

    def _merge_winner_tables(self) -> None:
        for task in self._tasks:
            stable_robot = self._winner_table.get(task.task_id).robot if task.task_id in self._winner_table else None
            best_entry: WinnerEntry | None = None
            for state in self._all_states():
                raw_entry = state.get('winner_table', {}).get(task.task_id)
                if not raw_entry:
                    continue
                entry = WinnerEntry(
                    robot=str(raw_entry.get('robot', '')),
                    score=float(raw_entry.get('score', 0.0)),
                )
                if best_entry is None or self._is_better(entry, best_entry, stable_robot=stable_robot):
                    best_entry = entry

            if best_entry is None:
                continue

            local_previous = self._winner_table.get(task.task_id)
            self._winner_table[task.task_id] = best_entry

            if local_previous is not None and local_previous.robot == self._robot_name and best_entry.robot != self._robot_name:
                self._abandon_from_task(task.task_id)

    def _merge_completed_tasks(self) -> None:
        completed_task_ids = self._all_completed_task_ids()
        for task_id in completed_task_ids:
            self._completed_tasks.add(task_id)
            self._abandoned_tasks.discard(task_id)
            self._winner_table.pop(task_id, None)
            if task_id in self._bundle:
                self._bundle.remove(task_id)

    def _task_score(self, task: Task) -> float:
        if self._pose is None:
            return float('-inf')
        distance = _distance(self._pose[0], self._pose[1], task.x, task.y)
        return task.reward - (self._distance_weight * distance)

    def _best_unclaimed_task(self) -> tuple[Task | None, float]:
        best_task: Task | None = None
        best_score = float('-inf')
        abandoned_ids = self._all_abandoned_task_ids()
        completed_ids = self._all_completed_task_ids()

        for task in self._tasks:
            if task.task_id in self._bundle:
                continue
            if task.task_id in completed_ids or task.task_id in abandoned_ids:
                continue

            score = self._task_score(task) - (0.25 * len(self._bundle))
            current_entry = self._winner_table.get(task.task_id)
            if current_entry is not None:
                if current_entry.robot != self._robot_name and score <= current_entry.score:
                    continue
                if current_entry.robot == self._robot_name and score < current_entry.score:
                    continue

            if score > best_score:
                best_score = score
                best_task = task

        return best_task, best_score

    def _rebuild_bundle(self) -> None:
        completed_ids = self._all_completed_task_ids()
        abandoned_ids = self._all_abandoned_task_ids()

        self._bundle = [
            task_id
            for task_id in self._bundle
            if task_id not in completed_ids and task_id not in abandoned_ids
        ]

        while len(self._bundle) < self._bundle_size:
            task, score = self._best_unclaimed_task()
            if task is None:
                break

            self._bundle.append(task.task_id)
            self._winner_table[task.task_id] = WinnerEntry(robot=self._robot_name, score=score)

    def _abandon_from_task(self, task_id: str) -> None:
        if task_id not in self._bundle:
            return

        index = self._bundle.index(task_id)
        removed_tasks = self._bundle[index:]
        self._bundle = self._bundle[:index]

        for removed_task_id in removed_tasks:
            self._abandoned_tasks.add(removed_task_id)
            self._winner_table.pop(removed_task_id, None)

        if self._active_goal_task in removed_tasks:
            self._cancel_active_goal()

    def _dispatch_next_goal(self) -> None:
        if self._active_goal_task is not None or not self._bundle:
            return

        next_task = self._task_by_id(self._bundle[0])
        if next_task is None:
            self._bundle.pop(0)
            return

        if not self._navigate_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warning('Nav2 action server is not ready yet.')
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = next_task.x
        goal.pose.pose.position.y = next_task.y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = _quaternion_from_yaw(0.0)

        self._active_goal_task = next_task.task_id
        self.get_logger().info(
            f'{self._robot_name}: sending goal for {next_task.task_id} at ({next_task.x:.2f}, {next_task.y:.2f})'
        )

        send_future = self._navigate_client.send_goal_async(goal, feedback_callback=self._feedback_callback)
        send_future.add_done_callback(lambda future: self._goal_response_callback(future, next_task.task_id))

    def _feedback_callback(self, feedback_message: Any) -> None:
        if self._active_goal_task is None or self._pose is None:
            return

        task = self._task_by_id(self._active_goal_task)
        if task is None:
            return

        distance = _distance(self._pose[0], self._pose[1], task.x, task.y)
        if distance <= self._goal_tolerance:
            self._completed_tasks.add(task.task_id)

    def _goal_response_callback(self, future: Any, task_id: str) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning(f'{self._robot_name}: goal rejected for {task_id}')
            self._active_goal_task = None
            if self._bundle and self._bundle[0] == task_id:
                self._bundle.pop(0)
            self._abandoned_tasks.add(task_id)
            self._winner_table.pop(task_id, None)
            return

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda future_result: self._result_callback(future_result, task_id))

    def _result_callback(self, future: Any, task_id: str) -> None:
        status = future.result().status
        if status == 4:
            self.get_logger().warning(f'{self._robot_name}: goal aborted for {task_id}')
            self._abandoned_tasks.add(task_id)
        else:
            self._completed_tasks.add(task_id)
            self.get_logger().info(f'{self._robot_name}: completed task {task_id}')

        if self._bundle and self._bundle[0] == task_id:
            self._bundle.pop(0)

        self._winner_table.pop(task_id, None)
        self._active_goal_task = None
        self._goal_handle = None

    def _cancel_active_goal(self) -> None:
        if self._goal_handle is None:
            self._active_goal_task = None
            return

        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(lambda _: None)
        self._goal_handle = None
        self._active_goal_task = None

    def _timer_callback(self) -> None:
        if self._pose is None or not self._tasks:
            self._publish_state()
            self._publish_assignment_summary()
            return

        self._publish_state()

        if not self._peers_ready():
            self._publish_assignment_summary()
            return

        self._merge_completed_tasks()
        self._merge_winner_tables()

        if len(self._all_completed_task_ids()) >= len(self._tasks):
            self._cancel_active_goal()
            self._bundle.clear()
            self._publish_state()
            self._publish_assignment_summary()
            return

        if self._active_goal_task is None:
            self._rebuild_bundle()
            self._dispatch_next_goal()

        self._publish_state()
        self._publish_assignment_summary()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CbbaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
