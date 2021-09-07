^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur_dashboard_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2021-09-07)
------------------
* Initial release
* Fix package dependencies `#132 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/132>`_ from UniversalRobots/fix_dependencies
* Use SPDX license identifiers. (`#145 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/145>`_)
  From https://spdx.org
* Fix all dependencies except yaml-cpp
* Merge pull request `#1 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/1>`_ from UniversalRobots/master
  update our master
* dashboard_msgs: fix typo in manifest description
* Changed my name in every occurence
* Merge branch 'robot_status'
  Propagating the robot's status (robot mode and safety mode) to the user
  so she can act accordingly (e.g. unlock after a protective stop or power
  on the robot if required)
* Updated documentation
* Added functionality to automatically restart the running program after recovery
* Implemented setMode action to bring the robot into a desired mode (e.g. RUNNING)
* Add a dashboard client to the driver `#16 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/16>`_ from UniversalRobots/dashboard_client
* Also publish robot mode
* Added more dashboard services
* Contributors: Felix Exner, Felix Mauch, G.A. vd. Hoorn, Tom Queen
