# UR_RTDE_Driver
This repository contains the new **ur_rtde_driver** and a couple of helper packages, such as:

  * **controller_stopper**: A small external tool that stops and restarts ros-controllers based on
    the robot's state. This can be helpful, when the robot is in a state, where it won't accept
    commands sent from ROS.
  * **ur_calibration**: Package around extracting and converting a robot's factory calibration
    information to make it usable by the robot_description.
  * **ur_controllers**: Controllers introduced with this driver, such as speed-scaling-aware
    controllers.
  * **ur_rtde_driver**: The actual driver package.

Please see the individual packages for further information. Especially the [README of the
ur_rtde_driver](ur_rtde_driver/README.md) serves as an entry point to get everything running.
