# TurboPi ROS

This project aims to get all functionality of the TurboPi robot running on
[ROS 2 Iron](https://docs.ros.org/en/iron/), along with an RPLidar for SLAM.

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
There are presently two ways to control the robot using [teleop twist joy](https://github.com/ros2/teleop_twist_joy) and [keyboard](https://github.com/ros2/teleop_twist_keyboard)

### Gamepad
Run the following command to invoke the controller for the gamepad. Presently using a Logitech F310, which works with the `xbox` configuration. 
```bash
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```

#### Button Layout
Movement is done using the D-Pad and the X and Right Trigger button combinations. Press either X or Right Trigger, and then use the D-Pad to move forward, backward, and turn left and right.
![Picture of Logitech F310](https://gm0.org/en/latest/_images/logitech-f310.png)

### Keyboard
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
