# PAIR_ROS2

This workspace contains the ROS2 package `pair` with a lightweight CBBA-style agent node for 2D simulation.

## What is implemented

- One ROS2 node per robot: `cbba_agent`
- YAML-based configuration per agent
- JSON messages over `std_msgs/String` for tasks, bids, and allocations
- Extensible task support through Python plugins
- A packaged default config and launch file

## Run

From the workspace root:

```bash
colcon build
source install/setup.bash
ros2 launch pair cbba_agent.launch.py
ros2 launch pair cbba_mvsim.launch.py agent_names:=robot_1,robot_2
ros2 launch pair cbba_mvsim_demo.launch.py agent_names:=robot_1,robot_2,robot_3
```

## Demo commands

If you want a quick 2-agent demo with MVSIM already running, use:

```bash
source /opt/ros/kilted/setup.bash
cd /home/alexandre/PAIR_ROS2
colcon build --packages-select pair
source install/setup.bash
ros2 launch pair cbba_mvsim_demo.launch.py agent_names:=robot_1,robot_2
```

For 3 agents, use the same command with:

```bash
ros2 launch pair cbba_mvsim_demo.launch.py agent_names:=robot_1,robot_2,robot_3
```

## Full MVSIM demo (recommended)

This launches MVSIM, CBBA agents, and demo tasks in one command:

```bash
source /opt/ros/kilted/setup.bash
cd /home/alexandre/PAIR_ROS2
colcon build --packages-select pair
source install/setup.bash
ros2 launch pair cbba_mvsim_full_demo.launch.py agent_names:=robot1,robot2
```

For RViz2 visualization (if needed):

```bash
ros2 launch pair cbba_mvsim_full_demo.launch.py agent_names:=robot1,robot2 use_rviz:=true
```

Note: The demo uses `robot1` and `robot2` (matching MVSIM warehouse robots) by default, not `robot_1` and `robot_2`.

## MVSIM

For MVSIM, the recommended setup is one `cbba_agent` node per robot namespace. The default MVSIM template uses `nav_msgs/msg/Odometry` on the relative topic `odom`, so a robot named `robot_1` listens on `/robot_1/odom`.

The MVSIM launch file is [src/pair/launch/cbba_mvsim.launch.py](src/pair/launch/cbba_mvsim.launch.py) and the template config is [src/pair/config/cbba_mvsim.yaml](src/pair/config/cbba_mvsim.yaml).

The demo launch is [src/pair/launch/cbba_mvsim_demo.launch.py](src/pair/launch/cbba_mvsim_demo.launch.py) and it also starts a demo task publisher from [src/pair/pair/demo_tasks.py](src/pair/pair/demo_tasks.py).

## Webots

For Webots integration with ROS2, use the `cbba_webots_demo.launch.py` file. This launch starts:

1. CBBA agents (one per robot namespace)
2. Demo task publisher

**Important:** Webots must be running separately before launching CBBA agents. Start Webots in one terminal, then launch the agents in another.

Quick start:

```bash
# Terminal 1: Start Webots
export WEBOTS_HOME=/mnt/c/Program\ Files/Webots
/mnt/c/Program\ Files/Webots/webots.exe /path/to/your/world.wbt

# Terminal 2: Start CBBA agents
source /opt/ros/kilted/setup.bash
cd /home/alexandre/PAIR_ROS2
colcon build --packages-select pair
source install/setup.bash
ros2 launch pair cbba_webots_demo.launch.py agent_names:=robot1,robot2
```

Verify Webots topics before starting CBBA:

```bash
ros2 topic list | grep odom
ros2 topic echo /robot1/odom --max-msgs=1
```

The Webots launch file is [src/pair/launch/cbba_webots_demo.launch.py](src/pair/launch/cbba_webots_demo.launch.py). Robot pose is read from `nav_msgs/msg/Odometry` on the `/robotX/odom` topic, same as MVSIM.

## Configuration

The default config lives in [src/pair/config/cbba_agent.yaml](src/pair/config/cbba_agent.yaml).

The main keys are:

- `agent_id`: unique robot identifier
- `tasks_topic`: topic used to receive tasks in JSON
- `bids_topic`: topic used to exchange bids in JSON
- `allocations_topic`: topic used to publish chosen tasks in JSON
- `plugin_modules`: list of Python modules that expose `TaskPlugin` implementations
- `cbba.max_bundle_size`: maximum number of tasks kept in the local bundle
- `cbba.travel_cost_weight`: penalty used when scoring a task

## Task plugins

Plugins are regular Python modules. Each plugin must implement `TaskPlugin` from [src/pair/pair/plugins/base.py](src/pair/pair/plugins/base.py).

The built-in example plugin is [src/pair/pair/plugins/generic.py](src/pair/pair/plugins/generic.py). Add a new task type by creating a new module and listing it in `plugin_modules` without changing the core allocator.

## Message format

Tasks are received as a JSON array of objects. A minimal task looks like this:

```json
{
	"id": "task_1",
	"type": "generic",
	"utility": 5.0
}
```

The node publishes local bids and allocations as JSON objects on the configured topics.

Build et sourcing
```bash
colcon build --packages-select my_package --symlink-install && source install/local_setup.bash
```

Lancement
```bash
ros2 launch my_package cbba_launch.py
```