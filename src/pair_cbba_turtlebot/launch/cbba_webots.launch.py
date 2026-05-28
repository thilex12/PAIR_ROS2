from __future__ import annotations

import os
import tempfile
from math import cos, pi, sin, tau
from pathlib import Path
from string import Template

import launch
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


ROBOT_NAME_PREFIX = 'tb3_'
TASK_MARKER_PREFIX = 'TASK_'
CIRCLE_ARENA_CENTER_X = 0.0
CIRCLE_ARENA_CENTER_Y = 0.0
CIRCLE_ARENA_DEFAULT_RADIUS = 1.8
CIRCLE_ARENA_WALL_THICKNESS = 0.08
CIRCLE_ARENA_WALL_HEIGHT = 0.25
CIRCLE_ARENA_FLOOR_THICKNESS = 0.02
CIRCLE_ARENA_MAP_RESOLUTION = 0.05
CIRCLE_ARENA_MAP_MARGIN = 0.6
CIRCLE_ARENA_WALL_SEGMENTS = 36

WORLD_HEADER_TEMPLATE = Template(
    """#VRML_SIM R2023b utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/develop/projects/robots/robotis/turtlebot/protos/TurtleBot3Burger.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/develop/projects/devices/robotis/protos/RobotisLds01.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/develop/projects/objects/backgrounds/protos/TexturedBackground.proto"

WorldInfo {
  basicTimeStep 20
}
Viewpoint {
  orientation -0.2878951570967451 -0.16506369377382343 0.9433293992651676 4.177335774513749
  position 9.141595121588473 4.8923306199482335 3.8476210861955575
  follow "${follow_robot}"
}
TexturedBackground {
  texture "empty_office"
  skybox FALSE
}
"""
)

ARENA_WALL_TEMPLATE = Template(
        """Solid {
    translation ${x} ${y} ${z}
    rotation 0 0 1 ${yaw}
    children [
        Shape {
            appearance PBRAppearance {
                baseColor 0.35 0.35 0.35
                roughness 1
                metalness 0
            }
            geometry Box {
                size ${thickness} ${length} ${height}
            }
        }
    ]
    boundingObject Box {
        size ${thickness} ${length} ${height}
    }
    physics Physics {
        mass 0
    }
}
"""
)

TASK_MARKER_TEMPLATE = Template(
    """DEF ${def_name} Transform {
  translation ${x} ${y} 0.02
  children [
    Shape {
      appearance PBRAppearance {
        baseColor ${red} ${green} ${blue}
        roughness 1
        metalness 0
      }
      geometry Cylinder {
        height 0.02
        radius 0.05
      }
    }
  ]
}
"""
)

ROBOT_TEMPLATE = Template(
    """TurtleBot3Burger {
  translation ${x} ${y} 0
  rotation 0 0 1 ${yaw}
  controller "<extern>"
    controllerArgs [
        ""
    ]
  name "${robot_name}"
  extensionSlot [
    Solid {
      name "imu_link"
    }
    GPS {
    }
    InertialUnit {
      name "inertial_unit"
    }
    RobotisLds01 {
    }
  ]
}
"""
)


def _load_package_yaml(package_dir: str, file_name: str) -> dict:
    config_path = Path(package_dir) / 'config' / file_name
    with config_path.open('r', encoding='utf-8') as config_file:
        return yaml.safe_load(config_file) or {}


def _load_tasks(package_dir: str) -> tuple[int, int, list[dict[str, float | str]]]:
    configuration = _load_package_yaml(package_dir, 'tasks.yaml')
    robot_count = max(1, int(configuration.get('robot_count', 2)))
    bundle_size = max(1, int(configuration.get('bundle_size', 1)))
    tasks = list(configuration.get('tasks', []))
    return robot_count, bundle_size, tasks


def _load_arena_settings(package_dir: str) -> tuple[float, float]:
    configuration = _load_package_yaml(package_dir, 'tasks.yaml')
    arena_radius = float(configuration.get('arena_radius', CIRCLE_ARENA_DEFAULT_RADIUS))
    wall_thickness = float(configuration.get('arena_wall_thickness', CIRCLE_ARENA_WALL_THICKNESS))
    return arena_radius, wall_thickness


def _rewrite_nav2_params(source_params_path: str) -> str:
    with open(source_params_path, 'r', encoding='utf-8') as params_file:
        configuration = yaml.safe_load(params_file) or {}

    def _rewrite(node: object) -> object:
        if isinstance(node, dict):
            rewritten: dict[str, object] = {}
            for key, value in node.items():
                if key == 'use_sim_time':
                    rewritten[key] = True
                elif key == 'odom_topic' and value == '/odom':
                    rewritten[key] = 'odom'
                elif key == 'topic' and value == '/scan':
                    rewritten[key] = 'scan'
                elif key in ('robot_base_frame', 'base_frame_id') and value in ('base_link', 'base_footprint'):
                    rewritten[key] = 'base_footprint'
                else:
                    rewritten[key] = _rewrite(value)
            return rewritten
        if isinstance(node, list):
            return [_rewrite(item) for item in node]
        return node

    rewritten_configuration = _rewrite(configuration)
    temp_dir = tempfile.mkdtemp(prefix='pair_cbba_turtlebot_nav2_')
    params_path = os.path.join(temp_dir, 'nav2_params_ns.yaml')
    with open(params_path, 'w', encoding='utf-8') as params_file:
        yaml.safe_dump(rewritten_configuration, params_file, sort_keys=False)
    return params_path


def _build_robot_names(robot_count: int) -> list[str]:
    return [f'{ROBOT_NAME_PREFIX}{index + 1}' for index in range(robot_count)]


def _build_robot_pose(index: int, robot_count: int, arena_radius: float) -> tuple[float, float, float]:
    spawn_radius = max(0.45, min(arena_radius - 0.55, arena_radius * 0.55))
    if robot_count == 1:
        return spawn_radius, 0.0, pi

    angle = tau * index / robot_count
    x = CIRCLE_ARENA_CENTER_X + spawn_radius * cos(angle)
    y = CIRCLE_ARENA_CENTER_Y + spawn_radius * sin(angle)
    yaw = angle + pi
    return x, y, yaw


def _generate_circle_arena_map(arena_radius: float, wall_thickness: float) -> str:
    half_extent = arena_radius + CIRCLE_ARENA_MAP_MARGIN
    size = int((half_extent * 2.0) / CIRCLE_ARENA_MAP_RESOLUTION) + 1
    center = size // 2
    inner_radius = max(0.1, arena_radius - (wall_thickness * 0.5))
    wall_inner = inner_radius
    wall_outer = arena_radius + (wall_thickness * 0.5)

    image_lines = ['P2', f'{size} {size}', '255']
    for row in range(size):
        values = []
        for col in range(size):
            dx = (col - center) * CIRCLE_ARENA_MAP_RESOLUTION
            dy = (center - row) * CIRCLE_ARENA_MAP_RESOLUTION
            distance = (dx * dx + dy * dy) ** 0.5
            if distance <= wall_inner:
                values.append('254')
            elif distance <= wall_outer:
                values.append('0')
            else:
                values.append('0')
        image_lines.append(' '.join(values))

    temp_dir = tempfile.mkdtemp(prefix='pair_cbba_turtlebot_map_')
    image_path = os.path.join(temp_dir, 'circle_arena.pgm')
    yaml_path = os.path.join(temp_dir, 'circle_arena.yaml')

    with open(image_path, 'w', encoding='utf-8') as image_file:
        image_file.write('\n'.join(image_lines))

    with open(yaml_path, 'w', encoding='utf-8') as yaml_file:
        yaml_file.write(
            '\n'.join(
                [
                    f'image: {image_path}',
                    f'resolution: {CIRCLE_ARENA_MAP_RESOLUTION}',
                    f'origin: [{-half_extent:.3f}, {-half_extent:.3f}, 0.0]',
                    'occupied_thresh: 0.65',
                    'free_thresh: 0.25',
                    'negate: 0',
                ]
            )
        )

    return yaml_path


def _generate_circle_arena_world(robot_names: list[str], tasks: list[dict[str, float | str]], arena_radius: float, wall_thickness: float) -> str:
    base_world_path = Path(get_package_share_directory('webots_ros2_turtlebot')) / 'worlds' / 'turtlebot3_burger_example.wbt'
    world_text = base_world_path.read_text(encoding='utf-8')

    robot_token = 'TurtleBot3Burger {'
    robot_start = world_text.find(robot_token)
    if robot_start == -1:
        raise RuntimeError(f'Could not find TurtleBot3Burger block in {base_world_path}')

    brace_start = world_text.find('{', robot_start)
    if brace_start == -1:
        raise RuntimeError(f'Could not parse TurtleBot3Burger block in {base_world_path}')

    brace_depth = 0
    robot_end = None
    for index in range(brace_start, len(world_text)):
        character = world_text[index]
        if character == '{':
            brace_depth += 1
        elif character == '}':
            brace_depth -= 1
            if brace_depth == 0:
                robot_end = index + 1
                break

    if robot_end is None:
        raise RuntimeError(f'Could not close TurtleBot3Burger block in {base_world_path}')

    robot_blocks = []
    for index, robot_name in enumerate(robot_names):
        x, y, yaw = _build_robot_pose(index, len(robot_names), arena_radius)
        robot_blocks.append(
            ROBOT_TEMPLATE.substitute(
                robot_name=robot_name,
                x=f'{x:.6f}',
                y=f'{y:.6f}',
                yaw=f'{yaw:.6f}',
            )
        )

    task_blocks = []
    task_colors = [
        (0.95, 0.35, 0.25),
        (0.25, 0.75, 0.35),
        (0.25, 0.45, 0.95),
        (0.85, 0.75, 0.25),
        (0.75, 0.25, 0.85),
    ]
    for index, task in enumerate(tasks):
        red, green, blue = task_colors[index % len(task_colors)]
        task_blocks.append(
            TASK_MARKER_TEMPLATE.substitute(
                def_name=f'{TASK_MARKER_PREFIX}{task["id"]}',
                x=f'{float(task["x"]):.6f}',
                y=f'{float(task["y"]):.6f}',
                red=f'{red:.3f}',
                green=f'{green:.3f}',
                blue=f'{blue:.3f}',
            )
        )

    world_content = [
        world_text[:robot_start],
        '\n'.join(robot_blocks),
        world_text[robot_end:],
        '\n'.join(task_blocks),
    ]

    temp_dir = tempfile.mkdtemp(prefix='pair_cbba_turtlebot_world_')
    world_path = os.path.join(temp_dir, 'generated_circle_arena.wbt')
    with open(world_path, 'w', encoding='utf-8') as world_file:
        world_file.write(''.join(world_content))

    return world_path


def _initial_pose_node(robot_name: str, x: float, y: float, yaw: float) -> Node:
    return Node(
        package='pair_cbba_turtlebot',
        executable='initial_pose_publisher',
        namespace=robot_name,
        output='screen',
        parameters=[
            {
                'frame_id': 'map',
                'topic': 'initialpose',
                'x': x,
                'y': y,
                'yaw': yaw,
                'publish_period': 1.0,
                'publish_attempts': 5,
            }
        ],
    )


def _generate_world_file(robot_names: list[str], tasks: list[dict[str, float | str]]) -> str:
    world_content = [WORLD_HEADER_TEMPLATE.substitute(follow_robot=robot_names[0])]

    for index, robot_name in enumerate(robot_names):
        x, y, yaw = _build_robot_pose(index, len(robot_names), CIRCLE_ARENA_DEFAULT_RADIUS)
        world_content.append(
            ROBOT_TEMPLATE.substitute(
                robot_name=robot_name,
                x=f'{x:.6f}',
                y=f'{y:.6f}',
                yaw=f'{yaw:.6f}',
            )
        )

    task_colors = [
        (0.95, 0.35, 0.25),
        (0.25, 0.75, 0.35),
        (0.25, 0.45, 0.95),
        (0.85, 0.75, 0.25),
        (0.75, 0.25, 0.85),
    ]
    for index, task in enumerate(tasks):
        red, green, blue = task_colors[index % len(task_colors)]
        world_content.append(
            TASK_MARKER_TEMPLATE.substitute(
                def_name=f'{TASK_MARKER_PREFIX}{task["id"]}',
                x=f'{float(task["x"]):.6f}',
                y=f'{float(task["y"]):.6f}',
                red=f'{red:.3f}',
                green=f'{green:.3f}',
                blue=f'{blue:.3f}',
            )
        )

    temp_dir = tempfile.mkdtemp(prefix='pair_cbba_turtlebot_world_')
    world_path = os.path.join(temp_dir, 'generated_world.wbt')
    with open(world_path, 'w', encoding='utf-8') as world_file:
        world_file.write('\n'.join(world_content))

    return world_path


def _controller_mapping() -> list[tuple[str, str]]:
    use_twist_stamped = 'ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] in ['rolling', 'jazzy', 'kilted']
    if use_twist_stamped:
        return [('/diffdrive_controller/cmd_vel', 'cmd_vel'), ('/diffdrive_controller/odom', 'odom')]
    return [('/diffdrive_controller/cmd_vel_unstamped', 'cmd_vel'), ('/diffdrive_controller/odom', 'odom')]


def generate_launch_description() -> LaunchDescription:
    package_dir = get_package_share_directory('pair_cbba_turtlebot')
    turtlebot_dir = get_package_share_directory('webots_ros2_turtlebot')

    robot_count_from_config, bundle_size_from_config, tasks = _load_tasks(package_dir)
    arena_radius, wall_thickness = _load_arena_settings(package_dir)

    robot_count = LaunchConfiguration('robot_count')
    bundle_size = LaunchConfiguration('bundle_size')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_names = _build_robot_names(robot_count_from_config)
    world_path = _generate_circle_arena_world(robot_names, tasks, arena_radius, wall_thickness)

    webots = WebotsLauncher(world=world_path, ros2_supervisor=True)
    robot_description_path = os.path.join(turtlebot_dir, 'resource', 'turtlebot_webots.urdf')
    ros2_control_params = os.path.join(turtlebot_dir, 'resource', 'ros2control.yml')
    nav2_map = _generate_circle_arena_map(arena_radius, wall_thickness)
    nav2_params = _rewrite_nav2_params(os.path.join(turtlebot_dir, 'resource', 'nav2_params.yaml'))

    robot_actions = []
    for index, robot_name in enumerate(robot_names):
        robot_x, robot_y, robot_yaw = _build_robot_pose(index, len(robot_names), arena_radius)
        turtlebot_driver = WebotsController(
            robot_name=robot_name,
            namespace=robot_name,
            parameters=[
                {
                    'robot_description': robot_description_path,
                    'use_sim_time': use_sim_time,
                    'set_robot_state_publisher': True,
                },
                ros2_control_params,
            ],
            remappings=_controller_mapping(),
            respawn=True,
        )

        joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            namespace=robot_name,
            output='screen',
            arguments=['joint_state_broadcaster', '--controller-manager-timeout', '50'],
        )

        diffdrive_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            namespace=robot_name,
            output='screen',
            arguments=['diffdrive_controller', '--controller-manager-timeout', '50'],
        )

        footprint_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace=robot_name,
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        )

        nav2_stack = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'namespace': robot_name,
                'slam': 'False',
                'map': nav2_map,
                'use_sim_time': use_sim_time,
                'params_file': nav2_params,
                'autostart': 'true',
                'use_composition': 'False',
                'use_respawn': 'False',
                'use_localization': 'True',
                'use_keepout_zones': 'False',
                'use_speed_zones': 'False',
            }.items(),
            condition=launch.conditions.IfCondition(use_nav2),
        )

        cbba_node = Node(
            package='pair_cbba_turtlebot',
            executable='cbba_node',
            namespace=robot_name,
            output='screen',
            parameters=[
                {
                    'robot_name': robot_name,
                    'robot_count': robot_count,
                    'bundle_size': bundle_size,
                    'tasks_config': os.path.join(package_dir, 'config', 'tasks.yaml'),
                    'pose_topic': 'amcl_pose',
                    'state_topic': '/cbba/state',
                    'assignment_topic': '/cbba/assignment',
                    'navigation_action': 'navigate_to_pose',
                }
            ],
        )

        initial_pose_node = _initial_pose_node(robot_name, robot_x, robot_y, robot_yaw)

        robot_actions.append(
            WaitForControllerConnection(
                target_driver=turtlebot_driver,
                nodes_to_start=[joint_state_broadcaster_spawner, diffdrive_controller_spawner, footprint_publisher, nav2_stack, initial_pose_node, cbba_node],
            )
        )

    launch_description = [
        DeclareLaunchArgument('robot_count', default_value=str(robot_count_from_config)),
        DeclareLaunchArgument('bundle_size', default_value=str(bundle_size_from_config)),
        DeclareLaunchArgument('use_nav2', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        webots,
        webots._supervisor,
        *robot_actions,
        RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ]

    return LaunchDescription(launch_description)
