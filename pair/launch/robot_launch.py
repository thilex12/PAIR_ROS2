# pair\launch\robot_launch.py
import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from pair.generate_world import generate_world
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController



def load_robot_count(config_file):
    if not os.path.exists(config_file):
        return 2
    with open(config_file, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            if ':' not in line:
                continue
            key, value = line.split(':', 1)
            if key.strip() == 'robot_count':
                return int(value.strip())
    return 2


def generate_launch_description():
    package_dir = get_package_share_directory('pair')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')
    config_path = os.path.join(package_dir, 'config', 'robot_count.yaml')
    num_robots = load_robot_count(config_path)
    world_path = os.path.join(package_dir, 'worlds', 'my_world.wbt')

    generate_world(num_robots=num_robots, output_file=world_path)

    with open(robot_description_path, 'r') as f:
        robot_description = f.read()

    webots = WebotsLauncher(
        world=world_path
    )

    robot_controllers = []
    obstacle_avoiders = []
    for i in range(num_robots):
        robot_name = f'robot_{i}'
        robot_controllers.append(
            WebotsController(
                robot_name=robot_name,
                parameters=[
                    {'robot_description': robot_description},
                ],
                namespace=robot_name
            )
        )
        obstacle_avoiders.append(
            Node(
                package='pair',
                executable='obstacle_avoider',
                name='obstacle_avoider',       # same base name is fine...
                namespace=robot_name,          # ...namespace makes /robot_0/obstacle_avoider unique
                output='screen',
                arguments=['--ros-args', '--log-level', 'warn'],  # reduce noise
            )
        )

    launch_description_entities = [webots] + robot_controllers + obstacle_avoiders

    return LaunchDescription(launch_description_entities + [
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])