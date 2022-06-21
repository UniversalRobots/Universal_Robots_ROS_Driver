Requirements
============

For using the ``ur_robot_driver`` your setup should fulfill a couple of requirements:

 * ROS should be running as close to real-time as possible. If you use the driver in a VM setup,
   expect occasional disconnection problems. For the best experience, please install a real-time
   kernel as explained in the :ref:`real-time setup guide <real-time-kernel>`.
 * Please use a direct cable connection between the ROS machine and the robot. Using a switch with
   other network participants connected might introduce problems on the communication between the
   driver and the robot. Especially in cases of high load on the network things will go wrong.
   Especially, please **do not use a WIFI connection** for controlling the robot. This will not be
   stable and responsive enough to do high-frequency control.
 * For this driver to work, please use PolyScope software version >= 3.7 for a CB3 robot and
   software >= 5.1 for an e-Series model.
 * This driver and all of its resources should work under ROS melodic on Ubuntu 18.04 and ROS noetic
   on Ubuntu 20.04.
