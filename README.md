# PAIR_ROS2

In order to use this project, first follow the WSL installation on your WSL :  
https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html  
Ans then : https://docs.ros.org/en/kilted/Tutorials/Advanced/Simulators/Webots/Installation-Windows.html  
(you can add source /opt/ros/kilted/setup.bash to your bashrc)

Then to install the workspace : https://docs.ros.org/en/kilted/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html  

Then create your first package : https://docs.ros.org/en/kilted/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html

simple hello word publisher/listener : https://docs.ros.org/en/kilted/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html  

webot first example : https://docs.ros.org/en/kilted/Tutorials/Advanced/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.html  


To try the first example, go in the root folder and do : 
```bash
colcon build
source install/local_setup.bash
ros2 launch pair robot_launch.py
```


