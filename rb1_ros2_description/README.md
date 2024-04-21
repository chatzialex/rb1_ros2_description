# The rb1_ros2_description package

## Introduction

This package contains a ROS2 robot model description for Robotnik's RB-1 mobile base.
The package also includes the configuration of a `diff_drive_controller/DiffDriveController` for controlling the base of the robot.
Due to issues with Gazebo, a controller for the elevator is not included. Instead, instructions are provided below for controlling the elevator with the `gazebo_ros_force_system` Gazebo plugin.

## Installation

To build and source the package:

```
cd ~/ros2_ws && colcon build && source install/setup.bash
```

## Usage

### Launching the simulation

After installing the package, use:

```
ros2 launch rb1_ros2_description rb1_ros2_xacro.launch.py
```

This will launch the simulation, spawn the robot, and launch the robot controller in the `rb1_robot` namespace.

This might fail the first time. If it does, simply press CTRL+C and run the above command again.

After everything has launched, you should be able to see the robot in Gazebo. Additionally, the hardware interfaces of the robot should be loaded and claimed. The `joint_state_broadcaster` and `rb1_base_controller` nodes should be running. To verify, run:

```
ros2 control list_hardware_interfaces --controller-manager /rb1_robot/controller_manager
ros2 control list_controllers --controller-manager /rb1_robot/controller_manager
```

### Controlling the base of the robot

Moving the base of the robot is accomplished
- by publishing a `geometry_msgs/msg/Twist` on the `/rb1_robot/rb1_base_controller/cmd_vel_unstamped` topic. For example:

```
ros2 topic pub --rate 10 /rb1_robot/rb1_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0, z: 0.0}, angular: {x: 0.0,y: 0.0, z: 0.2}}"
```

- with a `teleop_twist_keyboard` node
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/rb1_robot/rb1_base_controller/cmd_vel_unstamped
```

### Controlling the elevator  

To move the elevator up:

```
ros2 service call /apply_joint_effort gazebo_msgs/srv/ApplyJointEffort '{joint_name: "robot_elevator_platform_joint", effort: 5.0, start_time: {sec: 0, nanosec: 0}, duration: {sec: 2000, nanosec: 0} }'
```

To move the elevator down:
```
ros2 service call /clear_joint_efforts gazebo_msgs/srv/JointRequest "{joint_name: robot_elevator_platform_joint}"
```

## Disclaimer:  
This package only modifies/adapts files from these repositories/packages:  
- [RobotnikAutomation/rb1_base_sim](https://github.com/RobotnikAutomation/rb1_base_sim) licensed under the BSD 2-Clause "Simplified" License
- [RobotnikAutomation/rb1_base_common/rb1_base_description](https://github.com/RobotnikAutomation/rb1_base_common/tree/melodic-devel/rb1_base_description), licensed under the BSD License
- [RobotnikAutomation/robotnik_sensors],(https://github.com/RobotnikAutomation/robotnik_sensors) licensed under the BSD License