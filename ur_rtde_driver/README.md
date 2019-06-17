# ur_rtde_driver 

This driver is forked from the [ur_modern_driver](https://github.com/ros-industrial/ur_modern_driver).

It works for all CB3 and eSeries robots and uses the RTDE interface for communication, whenever possible.

## Features
 * **Factory calibration** of the robot inside ROS to reach Cartesian
   targets precisely.
 * **Realtime-enabled** communication structure to robustly cope with the 2ms cycle time of the eSeries. To use this, compile and run it on a kernel with the `PREEMPT_RT` patch enabled. (TODO: Write tutorial on how to compile a realtime kernel for Ubuntu)
 * Transparent **integration of the teach-pendant**. Using the URCaps system, a program is running
   on the robot that handles control commands sent from ROS side. With this, the robot can be
   **paused**, **stopped** and **resumed** without restarting the ROS driver.
   This will in the future also enable, the usage of ROS-components as a part of a more complex UR-program
   on the teach pendant. This is currently not yet supported, as we are still missing to exit
   control from ROS side. Expect this to come in future releases.
 * Use the robot's **speed-scaling**. When speed scaling is active due to safety constraints or the
   speed slider is used, this gets correctly handled on the ROS side, as well slowing down
   trajectory execution accordingly. **Note**: Due to the speed scaling interface, other controllers
   than the scaled controllers provided, currently cannot be used. We plan to change this in
   upcoming releases.

## Building

```bash
# source global ros
$ source /opt/ros/<your_ros_version>/setup.bash
# create a catkin workspace
$ mkdir -p catkin_ws/src && cd catkin_ws
$ catkin_make
$ cd src
# clone the driver
$ git clone <this_repository_url>
# clone fork of the description to use the calibration feature
$ git clone -b calibration_devel https://github.com/fmauch/universal_robot.git
# install dependencies
$ rosdep install --from-path . -y --ignore-src
# build the driver
$ cd ..
$ catkin_make
# source the workspace
$ source devel/setup.bash
```

## Initial robot setup
To setup a new robot with this driver, please see the [initial setup tutorial](doc/initial_setup.md)
