# The cp15_auxiliary_files repository

## Description

This respository contains files required by the checkpoint Nr 15 project.
Students must clone this respository into their ros2_ws workspace as instructed by the project.

## Howto

```
cd ~/ros2_ws && colcon build && source install/setup.bash
ros2 launch rb1_ros2_description rb1_ros2_xacro.launch.py
ros2 control list_hardware_interfaces --controller-manager /rb1_robot/controller_manager
ros2 control list_controllers --controller-manager /rb1_robot/controller_manager
ros2 topic pub --rate 10 /rb1_robot/rb1_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0, z: 0.0}, angular: {x: 0.0,y: 0.0, z: 0.2}}"
```
