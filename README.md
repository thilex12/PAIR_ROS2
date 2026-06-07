# PAIR_ROS2

This workspace contains the ROS2 package `pair` with a lightweight CBBA-style agent node for 2D simulation.

## What is implemented

- One ROS2 node per robot: `cbba_agent`
- YAML-based configuration per agent
- JSON messages over `std_msgs/String` for tasks, bids, and allocations
- Extensible task support through Python plugins
- A packaged default config and launch file

## Pre-requisites

In order to use this project, first follow the WSL installation on your WSL :
https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html
Ans then : https://docs.ros.org/en/kilted/Tutorials/Advanced/Simulators/Webots/Installation-Windows.html
(you can add source /opt/ros/kilted/setup.bash to your bashrc)

You also may need to download Webot if your wsl don't do it automatically


## Configuration

- In src\my_package\config\robots.yaml, you can configure the number of robots, arena size, and the walls
- In src\my_package\config\cbba_tasks.yaml, you can configure bundle size and taks

## Run

for the cbba working, you can do on your wsl ubuntu : 

```bash
colcon build --packages-select my_package --symlink-install && source install/local_setup.bash
ros2 launch my_package cbba_launch.py
```

While it's runnin, on another wsl ubuntu, you can check topics where robots are communicating

```bash
ros2 topic echo cbba/state
```

Or run rqt_graph to see relations between topics and nodes 

```bash
ros2 run rqt_graph rqt_graph
```

You can do the same workflow (build, source and launch) for the turtlebot package. But it is not working, and need more time to develop it.