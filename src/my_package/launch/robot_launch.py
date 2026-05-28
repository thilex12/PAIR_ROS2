# src\my_package\launch\robot_launch.py
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
WORLD_HEADER = '''#VRML_SIM R2022b utf8

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
}
'''
WORLD_FOOTER = '''
'''
ROBOT_TEMPLATE = Template('''Robot {
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
                        baseColor 0 0 1
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
    controller "<extern>"
    name "$robot_name"
}
''')


def _load_robot_configuration(package_dir):
    source_config_path = Path(package_dir).resolve().parents[3] / 'src' / 'my_package' / 'config' / 'robots.yaml'
    installed_config_path = Path(package_dir) / 'config' / 'robots.yaml'

    config_path = source_config_path if source_config_path.exists() else installed_config_path

    if not os.path.exists(config_path):
        return 1

    with open(config_path, 'r', encoding='utf-8') as config_file:
        configuration = yaml.safe_load(config_file) or {}

    return max(1, int(configuration.get('robot_count', 1)))


def _build_robot_names(robot_count):
    return [f'{ROBOT_NAME_PREFIX}{index + 1}' for index in range(robot_count)]


def _build_robot_pose(index, robot_count):
    if robot_count == 1:
        return 0.0, 0.0, 0.0

    radius = min(0.75, max(0.35, 0.18 * robot_count))
    angle = tau * index / robot_count
    x = radius * cos(angle)
    y = radius * sin(angle)
    rotation = angle + (tau / 2)
    return x, y, rotation


def _generate_world_file(robot_names):
    world_content = [WORLD_HEADER]

    for index, robot_name in enumerate(robot_names):
        translation_x, translation_y, rotation = _build_robot_pose(index, len(robot_names))
        world_content.append(
            ROBOT_TEMPLATE.substitute(
                robot_name=robot_name,
                translation_x=f'{translation_x:.6f}',
                translation_y=f'{translation_y:.6f}',
                rotation=f'{rotation:.6f}',
                left_wheel_offset='0.045',
            )
        )

    world_content.append(WORLD_FOOTER)

    temp_dir = tempfile.mkdtemp(prefix='my_package_world_')
    world_path = os.path.join(temp_dir, 'generated_world.wbt')

    with open(world_path, 'w', encoding='utf-8') as world_file:
        world_file.write('\n'.join(world_content))

    return world_path


def generate_launch_description():
    package_dir = get_package_share_directory('my_package')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')
    robot_count = _load_robot_configuration(package_dir)
    robot_names = _build_robot_names(robot_count)
    world_path = _generate_world_file(robot_names)

    webots = WebotsLauncher(
        world=world_path
    )

    robot_actions = []

    for robot_name in robot_names:
        robot_actions.append(
            WebotsController(
                robot_name=robot_name,
                namespace=robot_name,
                parameters=[
                    {'robot_description': robot_description_path},
                ],
                respawn=True,
            )
        )
        robot_actions.append(
            Node(
                package='my_package',
                executable='obstacle_avoider',
                namespace=robot_name,
                name=f'{robot_name}_obstacle_avoider',
            )
        )

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