# TurboPi ROS
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg?style=plastic)](https://github.com/wltjr/turbopi_ros/blob/master/LICENSE.txt)
![Build Status](https://github.com/wltjr/turbopi_ros/actions/workflows/docker_build.yml/badge.svg)
[![Code Quality](https://sonarcloud.io/api/project_badges/measure?project=wltjr_turbopi_ros&metric=alert_status)](https://sonarcloud.io/dashboard?id=wltjr_turbopi_ros)

This project aims to get all functionality of the TurboPi robot running on
[ROS 2 Jazzy](https://docs.ros.org/en/jazzy/), along with an RPLidar for SLAM.
The project is a work in progress, used to learn ROS 2, as well as for upcoming
research work into SLAM, navigation without GPS or Compass, dynamic path planning
with encountering unknown newly discovered obstacles, and other research topics.

Work is underway to customize the default TurboPi, replacing the 2DOF camera
with a Orbbec Astra S 3D Depth Camera. This will be done in a manner that
supports both, and work has already be done for simulation of both. A 2D
360&#176; RP LiDAR A1 has been added to both.

<table style="padding:10px">
  <tr>
    <td> 
      <img src="https://github.com/user-attachments/assets/7a3572b8-a915-4015-9751-5b90ae2336e1" alt="TurboPi in Gazebo" width="265px" height="255px" >
    </td>
    <td>
      <img src="https://github.com/user-attachments/assets/92f3f31c-802f-4783-b9ac-52aefce8a10d" alt="Custom TurboPi in Gazebo" width="265px" height="255px" >
    </td>
  </tr>
</table>

Mesh files have been generously provided by Hiwonder and are their property.
Mesh files are copyright Hiwonder. All Rights Reserved.

## Environment Preparation
The following assumes you have installed all the necessary ROS 2 Jazzy packages,
and have sourced the installation before running any `ros2` commands.
```bash
source /opt/ros/jazzy/setup.bash
```

You may want to have your development user environment do this on login via
`~/.bashrc` file; add that command to the end of that file.

Its recommended to have a ros2 workspace in a user directory for development
purposes, to build this project, etc; ex `~/ros2_ws/`. The following will refer
to that directory, and directories created within.

## Download
Download and unpack or clone this repositories contents into your ros2
workspace; ex `~/ros2_ws/src/turbopi_ros`.


## Build and Install
Building is done using colcon which will invoke cmake and run the necessary
commands. Run the following command in your ros2 workspace; ex `~/ros2_ws/`.
```bash
colcon build --symlink-install --packages-select  turbopi_ros
```

### Source install
Make sure to run the following command after install and login. Run the
following command in your ros2 workspace; ex `~/ros2_ws/`.
```bash
source install/setup.bash
```

You may want to have your development user environment do this on login via
`~/.bashrc` file; add the following to the end of that file.
```bash
source ~/ros2_ws/install/setup.bash
```

## Run
There are several launchers that are used to run parts of the project, some are
used together, some stand-alone, some for simulation and the robot. They are all
run from your ros2 workspace; ex `~/ros2_ws/`.
```bash
ros2 launch turbopi_ros turbopi_ros.launch.py
```

- [gamepad.launch.py](https://github.com/wltjr/turbopi_ros/blob/main/launch2/gamepad.launch.py) -
Start the gamepad node for remote operation, teleop; run in container, local, or
remote.
- [ign_gazebo.launch.py](https://github.com/wltjr/turbopi_ros/blob/main/launch2/ign_gazebo.launch.py) -
Start a simulated TurboPi in Gazebo; run in container or desktop/laptop.
- [nav2.launch.py](https://github.com/wltjr/turbopi_ros/blob/main/launch2/nav2.launch.py) -
Start the Nav 2 stack, used with both hardware and simulation.
- [turbopi_ros.launch.py](https://github.com/wltjr/turbopi_ros/blob/main/launch2/turbopi_ros.launch.py) -
Start ROS 2 with hardware support for TurboPi on robot hardware.

### Docker Containers
Three docker containers have been made to aid primarily in development, but the
first can be used on actual hardware.

- [docker-ros2-jazzy](https://github.com/UNF-Robotics/docker-ros2-jazzy) -
Base headless container used in CI/CD and can be used on hardware with slight
overhead.
- [docker-ros2-jazzy-gz-rviz2](https://github.com/UNF-Robotics/docker-ros2-jazzy-gz-rviz2) -
Base X11 graphical container used for simulation has Gazebo and RViz2, intended
for desktop/laptop; contains prior.
- [docker-ros2-jazzy-gz-rviz2](https://github.com/wltjr/docker-ros2-jazzy-gz-rviz2-turbopi) -
Main development container used for development, simulation, etc. intended for
desktop/laptop; contains prior two.

The first container is primarily used for CI/CD. The second one is not directly
used. For most purposes, the last container is the primary one to use, outside
of running on the actual robot. Which is advised to do outside of a docker
container to avoid the minimal overhead.


## Robot Human Controllers
The primary way to control the robot is using telop_turbopi which is intended to
be used with a
[DUALSHOCK™4](https://www.playstation.com/en-us/accessories/dualshock-4-wireless-controller/)
wireless controller.


### DUALSHOCK™4
Run the following command to invoke the controller for the DUALSHOCK™4 wireless
controller. This can be done within the robot, or on a remote system running a
docker container or locally installed.
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
There are presently alternative two ways to control the robot using
[teleop twist joy](https://github.com/ros2/teleop_twist_joy) and
[keyboard](https://github.com/ros2/teleop_twist_keyboard). However, they only
support robot movement and do not control camera or other peripherals, they just
 use `/cmd_vel` topic.

#### Gamepad
Run the following command to invoke the controller for the gamepad. Sample 
command is  using a Logitech F310, which works with the `xbox` configuration. 
```bash
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```

#### Keyboard
Run the following command to invoke the controller for the keyboard, which will
present a interface for controlling the robot in the same terminal the command
is run within.
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


## Hardware
- [Hiwonder TurboPi](https://www.hiwonder.com/products/turbopi?variant=40112905388119) - 
  [Amazon](https://www.amazon.com/dp/B0BTTH8WD2)

  ![Picture of Hiwonder TurboPi](https://github.com/wltjr/turbopi_ros/assets/12835340/81dd585b-5b98-43b2-b532-ddd4233721ce)

- [Orbbec Astra S](https://www.orbbec.com/products/structured-light-camera/astra-series/) -
  [Amazon](https://www.amazon.com/gp/product/B0C2H4QL5F/)

  ![Picture of Orbbec Astra S](https://github.com/user-attachments/assets/08dc402d-9d05-453e-9a04-c889ad56a590)

- [Slamtec RPlidar A1](https://www.slamtec.ai/home/rplidar_a1/) -
  [Amazon](https://www.amazon.com/dp/B07TJW5SXF/)

  ![Picture of Slamtec RPlidar A1](https://github.com/wltjr/turbopi_ros/assets/12835340/9f7b9688-b600-42d9-8b1b-c3a834252112)



## Credits
Credits and thanks for resources used in this repository, some code and/or
project structure, go to the following:

- Articulated Robotics - 
  [Making a Mobile Robot with ROS](https://articulatedrobotics.xyz/tutorials/)
- Linux I2C - [Implementing I2C device drivers in userspace](https://www.kernel.org/doc/html/latest/i2c/dev-interface.html)
- ROS 2 Control Demos -
  [example 2](https://github.com/ros-controls/ros2_control_demos)
- Slate Robotics - 
  [How to implement ros_control on a custom robot](https://slaterobotics.medium.com/how-to-implement-ros-control-on-a-custom-robot-748b52751f2e)
