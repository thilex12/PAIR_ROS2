# pair/launch/robot_launch.py
import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from pair.generate_world import load_config, generate_world
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('pair')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')
    config_path = os.path.join(package_dir, 'config', 'robot_count.yaml')
    world_path = os.path.join(package_dir, 'worlds', 'my_world.wbt')

    num_robots, waypoints = load_config(config_path)

    generate_world(num_robots=num_robots, waypoints=waypoints, output_file=world_path)

    with open(robot_description_path, 'r') as f:
        robot_description = f.read()

    # Encode waypoints as flat comma-separated string for the agent
    waypoints_param = ','.join(f'{x},{y}' for x, y in waypoints)

    webots = WebotsLauncher(world=world_path)

    robot_controllers = []
    obstacle_avoiders = []
    for i in range(num_robots):
        robot_name = f'robot_{i}'
        robot_controllers.append(
            WebotsController(
                robot_name=robot_name,
                parameters=[
                    {'robot_description': robot_description},
                    {'waypoints': waypoints_param},
                ],
                namespace=robot_name,
            )
        )
        obstacle_avoiders.append(
            Node(
                package='pair',
                executable='obstacle_avoider',
                name='obstacle_avoider',
                namespace=robot_name,
                output='screen',
                arguments=['--ros-args', '--log-level', 'warn'],
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