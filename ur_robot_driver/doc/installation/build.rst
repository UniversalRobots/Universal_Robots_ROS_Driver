Building
========

**Note:** The driver consists of a `C++
library <https://github.com/UniversalRobots/Universal_Robots_Client_Library>`_ that abstracts the
robot's interfaces and a ROS driver on top of that. As the library can be built without ROS support,
it is not a catkin package and therefore requires a different treatment when being built inside the
workspace. See The alternative build method below if you'd like to build the library from source.

If you don't want to build the library from source, it is available as a binary package through the
ROS distribution of ROS melodic and noetic. It will be installed automatically if you
follow the steps below. If you'd like to also build the library from source, please follow the steps
explained in the [next section](#alternative-all-source-build).

.. code:: bash

   # source global ros
   $ source /opt/ros/<your_ros_version>/setup.bash

   # create a catkin workspace
   $ mkdir -p catkin_ws/src && cd catkin_ws

   # clone the driver
   $ git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

   # clone fork of the description. This is currently necessary, until the changes are merged upstream.
   $ git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot

   # install dependencies
   $ sudo apt update -qq
   $ rosdep update
   $ rosdep install --from-paths src --ignore-src -y

   # build the workspace
   $ catkin_make

   # activate the workspace (ie: source it)
   $ source devel/setup.bash


Alternative: All-source build
-----------------------------

If you would like to also build the library from source, clone the library into your workspace, as
well and build it using either ``catkin_make_isolated`` or `catkin
build <https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html>`_.

.. code:: bash

   $ source /opt/ros/<your_ros_version>/setup.bash
   $ mkdir -p catkin_ws/src && cd catkin_ws
   $ git clone -b boost https://github.com/UniversalRobots/Universal_Robots_Client_Library.git src/Universal_Robots_Client_Library
   $ git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver
   $ git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot
   $ sudo apt update -qq
   $ rosdep update
   $ rosdep install --from-paths src --ignore-src -y
   $ catkin_make_isolated
   $ source devel_isolated/setup.bash


