# pair/pair/generate_world.py
import math
import os


def load_config(config_file):
    """Parse robot_count and waypoints from YAML manually (no PyYAML dependency)."""
    robot_count = 2
    waypoints = []

    if not os.path.exists(config_file):
        return robot_count, waypoints

    with open(config_file, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    in_waypoints = False
    for line in lines:
        stripped = line.strip()
        if not stripped or stripped.startswith('#'):
            continue

        if stripped == 'waypoints:':
            in_waypoints = True
            continue

        if in_waypoints:
            if stripped.startswith('- ['):
                inner = stripped[3:-1]  # strip "- [" and "]"
                x, y = [float(v.strip()) for v in inner.split(',')]
                waypoints.append((x, y))
            elif not stripped.startswith('-'):
                in_waypoints = False

        if ':' in stripped and not in_waypoints:
            key, value = stripped.split(':', 1)
            if key.strip() == 'robot_count':
                robot_count = int(value.strip())

    return robot_count, waypoints


def generate_world(num_robots=2, waypoints=None, output_file='worlds/my_world.wbt'):
    """Generate a Webots world file with num_robots robots and waypoint markers."""

    if waypoints is None:
        waypoints = []

    world_header = """#VRML_SIM R2022b utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2022b/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2022b/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2022b/projects/objects/floors/protos/CircleArena.proto"

WorldInfo {
}
Viewpoint {
  orientation -0.33185733874619844 -0.09874274160469809 0.9381474178937331 3.686018050088086
  position 1.700313773507203 1.0549607538959629 1.4846240848267684
  follow "robot_0"
}
TexturedBackground {
}
TexturedBackgroundLight {
}
CircleArena {
}
"""

    colors = [
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

    world_content = world_header

    # ── Waypoint markers ──────────────────────────────────────────────────────
    for j, (wx, wy) in enumerate(waypoints):
        world_content += f"""Solid {{
  translation {wx} {wy} 0.005
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 1 1 0
        roughness 0.5
        metalness 0
        emissiveColor 1 1 0
      }}
      geometry Cylinder {{
        height 0.01
        radius 0.03
      }}
    }}
  ]
  name "waypoint_{j}"
}}
"""

    # ── Robots ────────────────────────────────────────────────────────────────
    spawn_radius = 0.7

    for i in range(num_robots):
        angle = 2 * math.pi * i / num_robots
        x = spawn_radius * math.cos(angle)
        y = spawn_radius * math.sin(angle)
        rotation_z = angle + math.pi

        color = colors[i % len(colors)]

        robot_template = f"""Robot {{
  translation {x:.4f} {y:.4f} 0
  rotation 0 0 1 {rotation_z:.4f}
  children [
    GPS {{
      name "gps"
    }}
    Compass {{
      name "compass"
    }}
    HingeJoint {{
      jointParameters HingeJointParameters {{
        axis 0 1 0
        anchor 0 0 0.025
      }}
      device [
        RotationalMotor {{
          name \"left wheel motor\"
        }}
      ]
      endPoint Solid {{
        translation 0 0.045 0.025
        children [
          DEF WHEEL_{i} Transform {{
            rotation 1 0 0 1.5707996938995747
            children [
              Shape {{
                appearance PBRAppearance {{
                  baseColor 1 0 0
                  roughness 1
                  metalness 0
                }}
                geometry Cylinder {{
                  height 0.01
                  radius 0.025
                }}
              }}
            ]
          }}
        ]
        name \"left wheel\"
        boundingObject USE WHEEL_{i}
        physics Physics {{
        }}
      }}
    }}
    HingeJoint {{
      jointParameters HingeJointParameters {{
        axis 0 1 0
        anchor 0 0 0.025
      }}
      device [
        RotationalMotor {{
          name \"right wheel motor\"
        }}
      ]
      endPoint Solid {{
        translation 0 -0.045 0.025
        children [
          USE WHEEL_{i}
        ]
        name \"right wheel\"
        boundingObject USE WHEEL_{i}
        physics Physics {{
        }}
      }}
    }}
    Transform {{
      translation 0 0 0.0415
      children [
        Shape {{
          appearance PBRAppearance {{
            baseColor {color[0]} {color[1]} {color[2]}
            roughness 1
            metalness 0
          }}
          geometry DEF BODY_{i} Cylinder {{
            height 0.08
            radius 0.045
          }}
        }}
      ]
    }}
    DistanceSensor {{
      translation 0.042 0.02 0.063
      rotation 0 0 1 0.5236003061004253
      children [
        DEF SENSOR_{i} Transform {{
          rotation 0 1 0 1.5708
          children [
            Shape {{
              appearance PBRAppearance {{
                baseColor 1 1 0
                roughness 1
                metalness 0
              }}
              geometry Cylinder {{
                height 0.004
                radius 0.008
              }}
            }}
          ]
        }}
      ]
      name \"ds0\"
      lookupTable [
        0 1020 0
        0.05 1020 0
        0.15 0 0
      ]
      numberOfRays 2
      aperture 1
    }}
    DistanceSensor {{
      translation 0.042 -0.02 0.063
      rotation 0 0 1 -0.5235996938995747
      children [
        USE SENSOR_{i}
      ]
      name \"ds1\"
      lookupTable [
        0 1020 0
        0.05 1020 0
        0.15 0 0
      ]
      numberOfRays 2
      aperture 1
    }}
  ]
  boundingObject Transform {{
    translation 0 0 0.0415
    children [
      USE BODY_{i}
    ]
  }}
  physics Physics {{
  }}
  controller \"<extern>\"
  name \"robot_{i}\"
}}
"""
        world_content += robot_template

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(world_content)