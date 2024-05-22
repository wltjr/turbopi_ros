# TurboPi ROS

This project aims to get all functionality of the TurboPi robot running on
[ROS 2 Iron](https://docs.ros.org/en/iron/), along with an RPLidar for SLAM.

![TurboPi in Gazebo](https://github.com/wltjr/turbopi_ros/assets/12835340/5c2e2cf6-8c80-49ee-b0a9-a5a6e4211558)

Mesh files have been generously provided by Hiwonder and are their property.
Mesh files are copyright Hiwonder. All Rights Reserved.

## Environment Preparation
The following assumes you have installed all the necessary ROS 2 Iron packages, and have sourced the installation before running any `ros2` commands.
```bash
source /opt/ros/iron/setup.bash
```

You may want to have your development user environment do this on login via `~/.bashrc` file; add that command to the end of that file.

Its recommended to have a ros2 workspace in a user directory for development purposes, to build this project, etc; ex `~/ros2_ws/`. The following will refer to that directory, and directories created within.

## Download
Download and unpack or clone this repositories contents into your ros2 workspace; ex `~/ros2_ws/src/turbopi_ros`.


## Build and Install
Building is done using colcon which will invoke cmake and run the necessary commands. Run the following command in your ros2 workspace; ex `~/ros2_ws/`.
```bash
colcon build --symlink-install --packages-select  turbopi_ros
```

### Source install
Make sure to run the following command after install and login. Run the following command in your ros2 workspace; ex `~/ros2_ws/`.
```bash
source install/setup.bash
```

You may want to have your development user environment do this on login via `~/.bashrc` file; add the following to the end of that file.
```bash
source ~/ros2_ws/install/setup.bash
```

## Run
To run on the actual hardware run the following command in your ros2 workspace; ex `~/ros2_ws/`.
```bash
ros2 launch turbopi_ros turbopi_ros.launch.py
```

## Robot Human Controllers
The primary way to control the robot is using telop_turbopi which is intended to
be used with a [DUALSHOCK™4](https://www.playstation.com/en-us/accessories/dualshock-4-wireless-controller/) wireless controller.


### DUALSHOCK™4
Run the following command to invoke the controller for the DUALSHOCK™4 wireless
controller.
```bash
ros2 launch turbopi_ros gamepad.launch.py
```

#### Button Layout
The primary buttons are the left joystick for driving/movement and the right
joystick for the attached camera. All other buttons are not mapped or in use
at this time.

<img align="left" alt="Drawing of DUALSHOCK™4" src="https://manuals.playstation.net/document/imgps4/other_basic_018.jpg" />

| Button | Action |
| ------------- | ------------- |
| A | Unused  |
| B | Unused  |
| C | Unused  |
| D | Unused  |
| E | Unused  |
| F | Unused  |
| G | Unused  |
| H | Camera - tilt up/down, pan left/right |
| I | Unused  |
| J | Unused  |
| K | Driving - forward/backward, turn left/right |

### Alternatives
There are presently alternative two ways to control the robot using [teleop twist joy](https://github.com/ros2/teleop_twist_joy) and [keyboard](https://github.com/ros2/teleop_twist_keyboard). However, they only support robot movement and do not control camera or other peripherals, they just use `/cmd_vel` topic.

#### Gamepad
Run the following command to invoke the controller for the gamepad. Sample 
command is  using a Logitech F310, which works with the `xbox` configuration. 
```bash
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```

#### Keyboard
Run the following command to invoke the controller for the keyboard, which will present a interface for controlling the robot in the same terminal the command is run within.
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


## Hardware
- [Hiwonder TurboPi](https://www.hiwonder.com/products/turbopi?variant=40112905388119) - 
  [Amazon](https://www.amazon.com/dp/B0BTTH8WD2)

  ![Picture of Hiwonder TurboPi](https://github.com/wltjr/turbopi_ros/assets/12835340/81dd585b-5b98-43b2-b532-ddd4233721ce)

- [Slamtec RPlidar A1](https://www.slamtec.ai/home/rplidar_a1/) -
  [Amazon](https://www.amazon.com/dp/B07TJW5SXF/)

  ![Picture of Slamtec RPlidar A1](https://github.com/wltjr/turbopi_ros/assets/12835340/9f7b9688-b600-42d9-8b1b-c3a834252112)



## Credits
Credits and thanks for resources used in this repository, some code and/or project structure, go to the following:

- Articulated Robotics - 
  [Making a Mobile Robot with ROS](https://articulatedrobotics.xyz/mobile-robot-full-list/)
- ROS 2 Control Demos -
  [example 2](https://github.com/ros-controls/ros2_control_demos)
- Slate Robotics - 
  [How to implement ros_control on a custom robot](https://slaterobotics.medium.com/how-to-implement-ros-control-on-a-custom-robot-748b52751f2e)
