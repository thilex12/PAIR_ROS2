"""Lance la simulation Webots multi-robot avec allocation de taches CBBA."""

import json
import os
import tempfile
from math import cos, sin, tau
from string import Template
from pathlib import Path

import yaml
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


ROBOT_NAME_PREFIX = 'auto_robot_'

PATH_SPHERE_COUNT = 40

# Une couleur par robot, utilisee pour le corps et les spheres du chemin.
ROBOT_COLORS = [
    [0, 0, 1],
    [0, 1, 0],
    [1, 0, 0],
    [1, 1, 0],
    [1, 0, 1],
    [0, 1, 1],
    [1, 0.5, 0],
    [0.5, 0, 1],
    [1, 0.5, 0.5],
    [0.5, 1, 0],
    [0.3, 0.3, 1],
    [0.3, 1, 0.3],
    [1, 0.3, 0.3],
    [0.7, 0.7, 0],
    [0.7, 0, 0.7],
    [0, 0.7, 0.7],
    [1, 0.7, 0.3],
    [0.3, 0.7, 1],
    [0.7, 1, 0.3],
    [1, 0.3, 0.7],
]

WORLD_HEADER_TEMPLATE = Template("""#VRML_SIM R2022b utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2022b/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2022b/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2022b/projects/objects/floors/protos/CircleArena.proto"

WorldInfo {
}
Viewpoint {
    orientation -0.33185733874619844 -0.09874274160469809 0.9381474178937331 3.686018050088086
    position 1.700313773507203 1.0549607538959629 1.4846240848267684
    follow "auto_robot_1"
}
TexturedBackground {
}
TexturedBackgroundLight {
}
CircleArena {
    radius $arena_radius
}
""")

WORLD_FOOTER = '\n'

WALL_TEMPLATE = Template("""Solid {
    translation $x $y $z
    children [
        Shape {
            appearance PBRAppearance {
                baseColor $red $green $blue
                roughness 1
                metalness 0
            }
            geometry Box {
                size $width $depth $height
            }
        }
    ]
    boundingObject Box {
        size $width $depth $height
    }
    physics Physics {
    }
    static TRUE
}
""")

TASK_MARKER_TEMPLATE = Template("""DEF TASK_$task_name Transform {
    translation $x $y 0.01
    children [
        Shape {
            appearance PBRAppearance {
                baseColor $red $green $blue
                roughness 1
                metalness 0
            }
            geometry Cylinder {
                height 0.02
                radius 0.03
            }
        }
    ]
}
""")

PATH_SPHERE_TEMPLATE = Template("""DEF ${def_name} Transform {
    translation 0 999 0
    children [
        Shape {
            appearance PBRAppearance {
                baseColor $red $green $blue
                roughness 1
                metalness 0
            }
            geometry Sphere {
                radius 0.012
                subdivision 1
            }
        }
    ]
}
""")

ROBOT_TEMPLATE = Template("""Robot {
    translation $translation_x $translation_y 0
    rotation 0 0 1 $rotation
    children [
        HingeJoint {
            jointParameters HingeJointParameters {
                axis 0 1 0
                anchor 0 0 0.025
            }
            device [
                RotationalMotor {
                    name "left wheel motor"
                }
            ]
            endPoint Solid {
                translation 0 $left_wheel_offset 0.025
                children [
                    DEF WHEEL Transform {
                        rotation 1 0 0 1.5707996938995747
                        children [
                            Shape {
                                appearance PBRAppearance {
                                    baseColor 1 0 0
                                    roughness 1
                                    metalness 0
                                }
                                geometry Cylinder {
                                    height 0.01
                                    radius 0.025
                                }
                            }
                        ]
                    }
                ]
                name "left wheel"
                boundingObject USE WHEEL
                physics Physics {
                }
            }
        }
        HingeJoint {
            jointParameters HingeJointParameters {
                axis 0 1 0
                anchor 0 0 0.025
            }
            device [
                RotationalMotor {
                    name "right wheel motor"
                }
            ]
            endPoint Solid {
                translation 0 -0.045 0.025
                children [
                    USE WHEEL
                ]
                name "right wheel"
                boundingObject USE WHEEL
                physics Physics {
                }
            }
        }
        Transform {
            translation 0 0 0.0415
            children [
                Shape {
                    appearance PBRAppearance {
                        baseColor $body_red $body_green $body_blue
                        roughness 1
                        metalness 0
                    }
                    geometry DEF BODY Cylinder {
                        height 0.08
                        radius 0.045
                    }
                }
            ]
        }
        DistanceSensor {
            translation 0.042 0.02 0.063
            rotation 0 0 1 0.5236003061004253
            children [
                DEF SENSOR Transform {
                    rotation 0 1 0 1.5708
                    children [
                        Shape {
                            appearance PBRAppearance {
                                baseColor 1 1 0
                                roughness 1
                                metalness 0
                            }
                            geometry Cylinder {
                                height 0.004
                                radius 0.008
                            }
                        }
                    ]
                }
            ]
            name "ds0"
            lookupTable [
                0 1020 0
                0.05 1020 0
                0.15 0 0
            ]
            numberOfRays 2
            aperture 1
        }
        DistanceSensor {
            translation 0.042 -0.02 0.063
            rotation 0 0 1 -0.5235996938995747
            children [
                USE SENSOR
            ]
            name "ds1"
            lookupTable [
                0 1020 0
                0.05 1020 0
                0.15 0 0
            ]
            numberOfRays 2
            aperture 1
        }
    ]
    boundingObject Transform {
        translation 0 0 0.0415
        children [
            USE BODY
        ]
    }
    physics Physics {
    }
    supervisor TRUE
    controller "<extern>"
    name "$robot_name"
}
""")


def _load_robot_configuration(package_dir):
    """Charge la configuration robot depuis le YAML du package source ou installe."""

    source_config_path = Path(package_dir).resolve().parents[3] / 'src' / 'my_package' / 'config' / 'robots.yaml'
    installed_config_path = Path(package_dir) / 'config' / 'robots.yaml'
    config_path = source_config_path if source_config_path.exists() else installed_config_path

    if not os.path.exists(config_path):
        return 1

    with open(config_path, 'r', encoding='utf-8') as f:
        configuration = yaml.safe_load(f) or {}

    arena_radius = float(configuration.get('arena_radius', 0.75))
    arena_radius = max(0.25, min(arena_radius, 2.0))

    return {
        'robot_count': max(1, int(configuration.get('robot_count', 1))),
        'display_paths': bool(configuration.get('display_paths', False)),
        'max_linear_speed': float(configuration.get('max_linear_speed', 0.12)),
        'max_angular_speed': float(configuration.get('max_angular_speed', 2.0)),
        'arena_radius': arena_radius,
        'arena_walls': configuration.get('arena_walls', []),
        'distance_weight': float(configuration.get('distance_weight', 5.0)),
    }


def _load_cbba_config(tasks_config_path: str) -> dict:
    """Charge la configuration specifique a CBBA depuis le fichier des taches."""

    if not os.path.exists(tasks_config_path):
        return {}
    with open(tasks_config_path, 'r', encoding='utf-8') as f:
        configuration = yaml.safe_load(f) or {}
    return {
        'bundle_size': max(1, int(configuration.get('bundle_size', 2))),
    }


def _build_robot_names(robot_count):
    """Construit les noms de robots auto_robot_1, auto_robot_2, etc."""

    return [f'{ROBOT_NAME_PREFIX}{i + 1}' for i in range(robot_count)]


def _build_robot_pose(index, robot_count, arena_radius):
    """Place les robots sur un cercle autour de l'origine."""

    if robot_count == 1:
        return 0.0, 0.0, 0.0
    pose_radius = min(arena_radius * 0.75, max(0.35, 0.18 * robot_count))
    angle = tau * index / robot_count
    return pose_radius * cos(angle), pose_radius * sin(angle), angle + (tau / 2)


def _load_task_markers(tasks_config_path):
    """Transforme les taches YAML en marqueurs visuels Webots."""

    if not os.path.exists(tasks_config_path):
        return []
    with open(tasks_config_path, 'r', encoding='utf-8') as f:
        configuration = yaml.safe_load(f) or {}
    markers = []
    for index, task_data in enumerate(configuration.get('tasks', [])):
        markers.append(TASK_MARKER_TEMPLATE.substitute(
            task_name=str(task_data.get('id', f'task_{index + 1}')),
            x=f"{float(task_data['x']):.6f}",
            y=f"{float(task_data['y']):.6f}",
            red='0.850', green='0.150', blue='0.150',
        ))
    return markers


def _build_arena_walls(walls_config):
    """Construit les murs de l'arene en texte VRML."""

    walls = []
    for wall in walls_config:
        x = float(wall.get('x', 0.0))
        y = float(wall.get('y', 0.0))
        width = float(wall.get('width', 0.02))
        depth = float(wall.get('depth', 0.50))
        height = float(wall.get('height', 0.16))
        color = wall.get('color', [0.5, 0.5, 0.5])
        walls.append(WALL_TEMPLATE.substitute(
            x=f'{x:.6f}', y=f'{y:.6f}', z=f'{height / 2.0:.6f}',
            width=f'{width:.6f}', depth=f'{depth:.6f}', height=f'{height:.6f}',
            red=f'{float(color[0]):.3f}',
            green=f'{float(color[1]):.3f}',
            blue=f'{float(color[2]):.3f}',
        ))
    return walls


def _walls_config_to_segments(walls_config: list) -> list:
    """Convertit les murs en boites en segments utilises par le planificateur A*."""
    segments = []
    for wall in walls_config:
        x = float(wall.get('x', 0.0))
        y = float(wall.get('y', 0.0))
        width = float(wall.get('width', 0.02))
        depth = float(wall.get('depth', 0.50))
        if depth >= width:
            segments.append({'x1': x, 'y1': y - depth / 2.0,
                             'x2': x, 'y2': y + depth / 2.0, 'thickness': width})
        else:
            segments.append({'x1': x - width / 2.0, 'y1': y,
                             'x2': x + width / 2.0, 'y2': y, 'thickness': depth})
    return segments


def _build_path_spheres(robot_names: list, display_paths: bool) -> list:
    """Pre-alloue les spheres de chemin pour chaque robot avec la meme couleur."""
    if not display_paths:
        return []

    nodes = []
    for robot_index, robot_name in enumerate(robot_names):
        r, g, b = ROBOT_COLORS[robot_index % len(ROBOT_COLORS)]
        for sphere_index in range(PATH_SPHERE_COUNT):
            nodes.append(PATH_SPHERE_TEMPLATE.substitute(
                def_name=f'PATH_{robot_name}_{sphere_index}',
                red=f'{r:.3f}', green=f'{g:.3f}', blue=f'{b:.3f}',
            ))
    return nodes


def _generate_world_file(robot_names, task_markers, arena_radius, arena_walls, path_spheres):
    """Genere le fichier monde Webots temporaire pour cette simulation."""

    world_content = [WORLD_HEADER_TEMPLATE.substitute(arena_radius=f'{arena_radius:.6f}')]
    world_content.extend(arena_walls)
    world_content.extend(task_markers)
    world_content.extend(path_spheres)

    for index, robot_name in enumerate(robot_names):
        tx, ty, rot = _build_robot_pose(index, len(robot_names), arena_radius)
        r, g, b = ROBOT_COLORS[index % len(ROBOT_COLORS)]
        world_content.append(ROBOT_TEMPLATE.substitute(
            robot_name=robot_name,
            translation_x=f'{tx:.6f}', translation_y=f'{ty:.6f}',
            rotation=f'{rot:.6f}',
            left_wheel_offset='0.045',
            body_red=f'{r:.3f}', body_green=f'{g:.3f}', body_blue=f'{b:.3f}',
        ))

    world_content.append(WORLD_FOOTER)

    temp_dir = tempfile.mkdtemp(prefix='my_package_world_')
    world_path = os.path.join(temp_dir, 'generated_world.wbt')
    with open(world_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(world_content))
    return world_path

def generate_launch_description():
    """Construit la description de lancement pour Webots, les robots et les noeuds CBBA."""

    package_dir = get_package_share_directory('my_package')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')
    tasks_config_path = os.path.join(package_dir, 'config', 'cbba_tasks.yaml')
    cbba_config = _load_cbba_config(tasks_config_path)
    robot_config = _load_robot_configuration(package_dir)
    robot_count = robot_config['robot_count']
    display_paths = robot_config['display_paths']
    robot_names = _build_robot_names(robot_count)
    task_markers = _load_task_markers(tasks_config_path)
    arena_walls = _build_arena_walls(robot_config['arena_walls'])
    path_spheres = _build_path_spheres(robot_names, display_paths)
    world_path = _generate_world_file(
        robot_names, task_markers, robot_config['arena_radius'], arena_walls, path_spheres
    )
    walls_json = json.dumps(_walls_config_to_segments(robot_config['arena_walls']))

    webots = WebotsLauncher(world=world_path)
    robot_actions = []

    for robot_name in robot_names:
        robot_actions.append(WebotsController(
            robot_name=robot_name,
            namespace=robot_name,
            parameters=[{'robot_description': robot_description_path}],
            respawn=True,
        ))
        robot_actions.append(Node(
            package='my_package',
            executable='cbba_agent',
            namespace=robot_name,
            name=f'{robot_name}_cbba_agent',
            parameters=[{
                'robot_name': robot_name,
                'tasks_config': tasks_config_path,
                'bundle_size': cbba_config.get('bundle_size', 2),
                'max_linear_speed': robot_config['max_linear_speed'],
                'max_angular_speed': robot_config['max_angular_speed'],
                'enable_path_visualization': display_paths,
                'arena_radius': robot_config['arena_radius'],
                'walls_json': walls_json,
                'robot_count': robot_count,
                'distance_weight': float(robot_config.get('distance_weight', 5.0)),
            }],
        ))

    return LaunchDescription([
        webots,
        *robot_actions,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])