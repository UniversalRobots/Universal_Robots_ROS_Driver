[![Build Status](https://travis-ci.org/UniversalRobots/Universal_Robots_ROS_Driver.svg?branch=master)](https://travis-ci.org/UniversalRobots/Universal_Robots_ROS_Driver)

# Universal_Robots_ROS_Driver
Universal Robots have become a dominant supplier of lightweight, robotic manipulators for industry, as well as for scientific research and education. The Robot Operating System (ROS) has developed from a community-centered movement to a mature framework and quasi standard, providing a rich set of powerful tools for robot engineers and researchers, working in many different domains.

<center><img src="ur_robot_driver/doc/initial_setup_images/e-Series.png" alt="Universal Robot e-Series family" style="width: 45%;"/></center>

With the release of UR’s new e-Series, the demand for a ROS driver that supports the new manipulators and the newest ROS releases and paradigms like ROS-control has increased further. The goal of this driver is to provide a stable and sustainable interface between UR robots and ROS that strongly benefit all parties.

It is the core value of Universal Robots, to empower people to achieve any goal within automation. The success criteria of this driver release is to follow this vision, by providing the ROS community with an easy to use, stable and powerful driver, that empowers the community to reach their goals in research and automation without struggling with unimportant technical challenges, instability or lacking features.

### Acknowledgement
This driver is forked from the [ur_modern_driver](https://github.com/ros-industrial/ur_modern_driver).

<!-- 
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" 
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" 
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287. 

It was developed in collaboration between [Universal Robots](https://www.universal-robots.com/) and
the [FZI Research Center for Information Technology](https://www.fzi.de).


## How to report an issue
Create an issue on the [Issue Board](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/new) using the default template.

## How to get help
If you need help using this driver, please see the ROS-category in the [UR+ Developer Forum](https://forum.universal-robots.com/c/ros). 

## Features
 * Works for all **CB3 (with software version >= 3.7) and e-Series (software >= 5.1)** robots and uses the RTDE interface for communication, whenever possible.
 * **Factory calibration** of the robot inside ROS to reach Cartesian
   targets precisely.
 * **Realtime-enabled** communication structure to robustly cope with the 2ms cycle time of the
   e-Series. To use this, compile and run it on a kernel with the `PREEMPT_RT` patch enabled. (See
   the [Real-time setup guide](ur_robot_driver/doc/real_time.md) on how to achieve this)
 * Transparent **integration of the teach-pendant**. Using the URCaps system, a program is running
   on the robot that handles control commands sent from ROS side. With this, the robot can be
   **paused**, **stopped** and **resumed** without restarting the ROS driver.
   This will in the future also enable the usage of ROS-components as part of a more complex UR-program
   on the teach pendant. ROS-control of the robot can be quit using a service call to continue
   program execution on the TP.
 * Use the robot's **speed-scaling**. When speed scaling is active due to safety constraints or the
   speed slider is used, this gets correctly handled on the ROS side, as well slowing down
   trajectory execution accordingly.<br/>
   **Note**: Other ros-controllers based on a position interface
   can be used with this driver, but may behave wrong if the speed slider isn't set to 100% or if
   speed scaling slows down the robot. Also, the pausing function can only be used if the default
   scaled trajectory controller is used.
 * **ROS-Service-based replacement of most every-day TP-interactions** offer using UR robots without
   interacting with the teach pendant at all, if desired. The robot can be started, stopped and even
   recovery from safety events can be done using ROS service- and action calls. See the driver's
   [dashboard services](ur_robot_driver/doc/ROS_INTERFACE.md#ur_robot_driver_node) and the
   [robot_state_helper node](ur_robot_driver/doc/ROS_INTERFACE.md#robot_state_helper) for details.

Please see the external [feature list](ur_robot_driver/doc/features.md) for a listing of all features supported by this driver.

## Contents
This repository contains the new **ur_robot_driver** and a couple of helper packages, such as:

  * **controller_stopper**: A small external tool that stops and restarts ros-controllers based on
    the robot's state. This can be helpful when the robot is in a state where it won't accept
    commands sent from ROS.
  * **ur_calibration**: Package around extracting and converting a robot's factory calibration
    information to make it usable by the robot_description.
  * **ur_controllers**: Controllers introduced with this driver, such as speed-scaling-aware
    controllers.
  * **ur_robot_driver**: The actual driver package.

## Requirements
This driver requires a system setup with ROS. It is recommended to use **Ubuntu 18.04 with ROS
melodic**, however using Ubuntu 16.04 with ROS kinetic should also work.

To make sure that robot control isn't affected by system latencies, it is highly recommended to use
a real-time kernel with the system. See the [real-time setup guide](ur_robot_driver/doc/real_time.md)
on information how to set this up.

## Building

```bash
# source global ros
$ source /opt/ros/<your_ros_version>/setup.bash

# create a catkin workspace
$ mkdir -p catkin_ws/src && cd catkin_ws

# clone the driver
$ git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

# clone fork of the description to use the calibration feature
$ git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot

# install dependencies
$ sudo apt update -qq
$ rosdep update
$ rosdep install --from-path src --ignore-src -y

# build the workspace
$ catkin_make

# activate the workspace (ie: source it)
$ source devel/setup.bash
```

## Setting up a UR robot for ur_robot_driver
### Prepare the robot
For using the *ur_robot_driver* with a real robot you need to install the
**externalcontrol-1.0.urcap** which can be found inside the **resources** folder of this driver.

**Note**: For installing this URCap a minimal PolyScope version of 3.7 or 5.1 (in case of e-Series) is
necessary.

For installing the necessary URCap and creating a program, please see the individual tutorials on
how to [setup a CB3 robot](ur_robot_driver/doc/install_urcap_cb3.md) or how to [setup an e-Series
robot](ur_robot_driver/doc/install_urcap_e_series.md).

To setup the tool communication on an e-Series robot, please consider the [tool communication setup
guide](ur_robot_driver/doc/setup_tool_communication.md).

### Prepare the ROS PC
For using the driver make sure it is installed (either by the debian package or built from source
inside a catkin workspace).

#### Extract calibration information
Each UR robot is calibrated inside the factory giving exact forward and inverse kinematics. To also
make use of this in ROS, you first have to extract the calibration information from the robot.

Though this step is not necessary to control the robot using this driver, it is highly recommended
to do so, as otherwise endeffector positions might be off in the magnitude of centimeters.


For this, there exists a helper script:

    $ roslaunch ur_calibration calibration_correction.launch \
      robot_ip:=<robot_ip> target_filename:="${HOME}/my_robot_calibration.yaml"

For the parameter `robot_ip` insert the IP address on which the ROS pc can reach the robot. As
`target_filename` provide an absolute path where the result will be saved to.

We recommend keeping calibrations for all robots in your organization in a common package. See the
[package's documentation](ur_calibration/README.md) for details.

#### Quick start
Once the driver is built and the **externalcontrol** URCap is installed and running on the robot, you are good
to go ahead starting the driver. (**Note**: We do recommend, though, to calibrate your robot first.)

To actually start the robot driver use one of the existing launch files

    $ roslaunch ur_robot_driver <robot_type>_bringup.launch robot_ip:=192.168.56.101

where **<robot_type>** is one of *ur3, ur5, ur10, ur3e, ur5e, ur10e*. Note that in this example we
load the calibration parameters for the robot "ur10_example".

If you calibrated your robot before, pass that calibration to the launch file:

    $ roslaunch ur_robot_driver <robot_type>_bringup.launch robot_ip:=192.168.56.101 \
      kinematics_config:=$(rospack find ur_calibration)/etc/ur10_example_calibration.yaml

If the parameters in that file don't match the ones reported from the robot, the driver will output
an error during startup, but will remain usable.

For more information on the launch file's parameters see its own documentation.

Once the robot driver is started, load the previously generated program on the robot panel and
execute it. From that moment on the robot is fully functional. You can make use of the pause
function or even stop the program. Simply press the play button again and the ROS driver will
reconnect.

To control the robot using ROS, use the action server on

```bash
/scaled_pos_traj_controller/follow_joint_trajectory
```

Use this with any client interface such as [MoveIt!](https://moveit.ros.org/) or simply the
`rqt_joint_trajectory_controller` gui:

```
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

