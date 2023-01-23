2.1.2 (2023-01-23)
------------------

2.1.1 (2023-01-23)
------------------
* Move controller_stopper to ur_robot_driver
  Since a standalone package with that name was declined during the release
  process, we decided to move the controller_stopper package over to the driver.
* Update minimal required polyscope version in docs
* Contributors: Felix Exner

2.1.0 (2022-12-08)
------------------
* delete ros_control.urscript (`#593 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/593>`_)
  We've been using the script from the library for a while now
* Use the RobotMode message inside the SetMode action (`#381 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/381>`_)
  This way we can make use of the predefined constants inside the RobotMode message.
* Make several members of hw_interface atomic, for thread safety (`#448 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/448>`_)
  Co-authored-by: Felix Exner (fexner) <exner@fzi.de>
* Updated transformForceTorque to handle wheter it is a cb3 or an e-Series robot (`#566 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/566>`_)
  The force torque is returned at the tool flange on e-series robots and at the tcp for CB3, this is now handled correctly, so that all force/torque measurements will be relative to the active TCP
* Updated set payload, zero ftsensor and set tool voltage to use the ur… (`#567 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/567>`_)
  * Updated set payload, zero ftsensor and set tool voltage to use the urdriver
  This makes it possible to call the commands when the robot is in local control if the external control script is running on the robot.
  * Update ROS interface docs
  * Fix argument passing in include instruction
  Co-authored-by: Miguel Prada <miguel.prada@tecnalia.com>
* Remove URCap installation files from driver and replace references (`#580 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/580>`_)
  Link to the respective Github release pages instead.
* Drop old C++ compiler flags (`#577 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/577>`_)
  Co-authored-by: Jochen Sprickerhof <git@jochen.sprickerhof.de>
* Fix MoveIt! command in Example (`#575 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/575>`_)
* Allow empty stopped_controllers argument. (`#572 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/572>`_)
* Dashboard service to query whether the robot is in remote control (`#561 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/561>`_)
  * new dashboard msg to check remote control
* test_move: Load controller only if it is not already loaded (`#552 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/552>`_)
* Add optional topic rename for speed scaling factor (`#544 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/544>`_)
  * Add optional topic rename for speed scaling factor
  * Update ROS_INTERFACE.md
* Wait for controller action server in test_move. (`#535 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/535>`_)
* Minor update to display robot_ip parameter without _ (`#521 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/521>`_)
* Make hw_interface-node required-argument optional (`#450 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/450>`_)
  The UR-robot is only one part of our roslaunch-setup, so I would like to be able to have the rest of the system (non-UR) continue to run even if the ur_hardware_interface-node dies.
* Fix test move python3 (`#492 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/492>`_)
  * make test_move work with python2 and python3
  As suggested in http://wiki.ros.org/UsingPython3/SourceCodeChanges#Changing_shebangs
  - Use a version-independent shebang
  - Use catkin_install_python to install the test_move script
* Update feature list (`#490 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/490>`_)
  Some minor formatting and reducing the wrench features to one features to make it more clear.
* Contributors: Adam Heins, AndyZe, Felix Exner, Felix Exner (fexner), Johnson, Mads Holm Peters, Michael Görner, Mingu Kwon, steinmn, teapfw, williamnavaraj

2.0.0 (2021-09-07)
------------------
* Merge pull request `#408 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/408>`_ from UniversalRobots/staging
  Adds new features to the driver:
  - Cartesian position-based control
  - Cartesian twist-based control
  - Trajectory forwarding for execution on robot
  - More documentation and examples
* Merge branch 'master' into staging
* Merge pull request `#437 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/437>`_ from UniversalRobots/test_move
  Add test_move script
* Move test_move files into one
* Added a full guide for starting with the driver
* Add more documentation to new features (`#423 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/423>`_)
  * Added a description for all controllers
  * Link repositories of URCap repos in feature list
  * Added note about blending with pass_through_controllers
  * Apply suggestions from code review
  Thanks @stefanscherzinger
  Co-authored-by: Stefan Scherzinger <scherzin@fzi.de>
  Co-authored-by: Stefan Scherzinger <scherzin@fzi.de>
* Add missing license header to tool_communication
* Added Cartesian test_move script
* Added choice of trajectory controller and require confirmation
* Fix default controller in common launchfile
  As mentioned in `#206 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/206>`_ this lists a non-existing controller
* Added test_move script
* Added log handler for handling log messages from client library with … (`#398 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/398>`_)
  * Added log handler for handling log messages from client library with ROS logging
* Merge pull request `#420 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/420>`_ from fmauch/update_feature_list
  Update feature list
* Added new control modes
* Removed comparison to ur_modern driver
  no point in doing this anymore.
* Updated ROS2 notice
* Added "On behalf of Universal Robots A/S" notice (`#416 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/416>`_)
  Removed copyright notice from LICENSE file, as the license file itself isn't
  copyrighted by FZI
* Merge pull request `#413 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/413>`_ from fmauch/cartesian_control
  Cartesian control
* Fix wait_for_server calls
  Rospy action_client.wait_for_server returns false instead of raising an exception
* Removed unneeded debug output
* Made twist controller an exec_depend
* Added Test for twist interface (`#2 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/2>`_)
* Added test for pose based cartesian trajectory execution
* added tcp pose of robot to cartesian pose interface read
* Adapt to new controller namespaces
  The namespaces have been moved upstream and we shall adapt to that
* added interface for cartesian pose streaming
* Added new controllers to controller configuration
* added twist interface for cartesian velocity control of the robot
  co-authored by Tristan Schnell <schnell@fzi.de>
* Merge pull request `#396 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/396>`_ from fmauch/trajectory_interface
  Adds an interface for trajectory forwarding
  Complete trajectories (joint-based and Cartesian) can be forwarded to the robot controller for interpolation and execution.
* Added test for pose based cartesian trajectory execution
* Add trajectory_port to hw-interface config
* Fix execution states
* Adapt to pass_though_controllers refactoring
* Register DoneCallback to trajectory passthrough
* added comment about angle representation
  Co-authored-by: Stefan Scherzinger <scherzin@fzi.de>
* changed trajectory action parameter for trajectory control messages to enum
* added feedback output to cartesian and joint-based trajectory forwarding
* controller config and launchfile updates for pass-through controllers
* added interface for cartesian trajectory forwarding
* added interfaces for joint trajectory forwarding controller
* Call calibration check in ROS driver (`#366 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/366>`_)
  Mandatory check in the client library has been deprecated.
* add reverse_ip argument (`#412 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/412>`_)
  Co-authored-by: JS00000 <winyangyuxin22@hotmail.com>
* Use urscript file from client library by default
* Merge pull request `#400 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/400>`_ from fmauch/external_scaling_interface
  Use speed scaling interface from external package and remove ur_contr…
* Update ur_robot_driver/README.md
  Co-authored-by: Stefan Scherzinger <scherzin@fzi.de>
* use the changed namespace of the scaling interface
* Use speed scaling interface from external package and remove ur_controllers
* Add partner logos to README (`#393 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/393>`_)
* Merge pull request `#389 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/389>`_ from fmauch/run_trajectory_test
  rename test method to be actually run by unittest
* power cycle robot before trajectory test to make sure the controller is running
* rename test method to be actually run by unittest
* Fix heading level for set_payload service
* Fixes reading has_realtime property
  Reading this from system may end up in undefined behavior. (https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/306)
* ensure extractToolPose always returns a valid transform (`#372 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/372>`_)
  Quaternion() returns 0,0,0,0 which leads to an invalid transform.
* Merge pull request `#97 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/97>`_ from fmauch/description_dev
  Use new ur_description_model
* Merge pull request `#382 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/382>`_ from fmauch/trajectory_tests
  Trajectory tests
* Run all integration tests in one test
* Make trajectory test monolithic
  Otherwise the trajectory tests might be running in parallel.
* Added note about the kinematics_config file
* Adapted to changed payload service
* Removed double yaml
* Updated default kinematics filename and removed ur_e_description
* Adapt to renamed description launchfiles
* Added ur16 support
* Use new description package with unified xacro arguments
* add arg for servoj_gain servoj_lookahead_time in ur_control.launch (`#354 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/354>`_)
  * add arg for servoj_gain servoj_lookahead_time in ur_control.launch
  * add default
* Added robot_ip and robot_type argument for integrations test
* Added headless mode to the feature list
* Remove manual keepalive message from script
  This is actually not properly checked by the driver, as keepalive signals
  won't be sent when the program is paused.
* Merge pull request `#342 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/342>`_ from fmauch/urcap_1.0.5
  Use external_control urcap version 1.0.5
* Added header and control loop definitions.
* Update external_control urcap to version 1.0.5
* Replace two logging macros with ROS logging macros. (`#330 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/330>`_)
  Inside the driver we want to use plain ROS logging instead of the library's logging macros.
* Use catkin_install_python macro for python files (`#318 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/318>`_)
  This macro works just like the normal `install` macro, but it also
  automatically changes the shebang line in the python file to `python2`
  or `python3`, depending which version is used.
  See: http://wiki.ros.org/UsingPython3/SourceCodeChanges#Changing_shebangs
  What this means is that this package can be used with Python3 without
  any further changes, for example in ROS Noetic.
* Wait for reverse socket response (`#288 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/288>`_)
  * Remove timeout and wait for response on reverse socket read
  Co-authored-by: Tom Queen <tom_q@hotmail.co.uk>
* Merge pull request `#266 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/266>`_ from UniversalRobots/separation
  Use ur_client_library package for building this driver
* Merge pull request `#270 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/270>`_ from UniversalRobots/ur16e
  Ur16e
* Replaced image by a version containing all 4 e-Series robots
* Add launchfile for ur16e
* Updated externalcontrol to v1.0.3 (`#245 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/245>`_)
  * Updated externalcontrol to v1.0.3
  * Updated externalcontrol to v1.0.4
* remove check_urls job
  This is done in the upstream library now
* Removed rtde_client test
  That moved to the library and makes more sense there.
* Moved files out of redundant "ros" subfolder
  Before, we had library compnents in other subfolders, but they got moved out.
* Use namespace urcl instead of ur_driver
* Renamed library
* Make tests use separate library, as well.
  Ultimately, this test should be moved to the library itself.
* Made library fully independent
* use ur_lib from separate package
* Disable trajectory test for now (`#264 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/264>`_)
  The trajectory test seems to not work anymore since a
  couple of weeks. Running those locally (also with a
  ursim running inside a docker container) works perfectly
  fine, but running it inside the github action not.
  As this is blocking many merges currently, I suggest
  to disable this temporarily while opening an issue to fix it.
* Use Robot_hw_nh node handle for joints. (`#227 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/227>`_)
  modified hardware interface to look for joints parameter under the robot_hw node handle
* Correct name of e-series in README
* Specify container IP addresses for testing purposes
  Before, the default Docker network in the range 172.17.0.0/16 was used. Since a specific IP cannot be chosen/guaranteed within this range, a network is now created with range 192.168.0.0/16, which allows for assigning specific IPs to the containers.
  Co-authored-by: Emil Vincent Ancker <emva@universal-robots.com>
* Added a service to setup the active payload (`#50 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/50>`_)
  * Added a service to setup the active payload
* Add prefix to wrench hw interface (`#217 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/217>`_)
  Use a parameter to set the wrench name
  This name will be picked up by the `force_torque_sensor_controller` in order to name the respective topic.
  Co-authored-by: carebare47 <tom@shadowrobot.com>
* Merge pull request `#209 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/209>`_ from fmauch/testing_scripts
  Add integration tests for automated testing
* Install resources directory (`#225 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/225>`_)
* Tests: Update the name of the trajectory controller
* Merge remote-tracking branch 'origin/master' into testing_scripts
* Install resources directory
* Add a gtest for RTDE client only
* Throw an exception when the recipe file cannot be read
* Added a running member to actually join the RTDEWriter thread
* Use a remap for the controller topic
* Use a test_depend for rostest
* replaced legacy package name
* specifically initialize robot before trajectory test
* Use enum identifier instead of hard coded value
* Added test for explicitly scaled trajectory execution
* Add a failing test
  I want to see whether the tests actually fail
* Moved everything to rostests
  Run docker ursim externally in GH action
* Renamed *_traj_controllers to *_joint_traj_controller (`#214 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/214>`_)
* driver: use default rate for JTC goal monitor. (`#221 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/221>`_)
  The old values overrode the default of 20 Hz, which is low and leads to a worst-case delay of approx 100 ms between a goal state change and action clients being notified of that change.
  This restores the rate to the default of 20 Hz.
  If a higher update-rate would be desirable for a particular application, users should change it in their own configuration of the controllers.
* Prefixing ExternalControl to log messages (`#222 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/222>`_)
  Co-authored-by: kut <kut@ubuntu.p52.ipa>
* Updated packaged externalcontrol urcap to v1.0.2 (`#208 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/208>`_)
* added basic action node for an IO integration test
* added basic action client node for a trajectory following integration test
* Export hardware interface library in CMakeLists (`#202 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/202>`_)
  Usage of the driver in a combined_robot_hw requires this change, as
  there will otherwise be undefined symbols from hardware_interface.cpp.
* Fix variable type checking in rtde_client (`#203 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/203>`_)
* Merge pull request `#193 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/193>`_ from UniversalRobots/add_documentation_link
  Add actual documentation link into calibration checker output
* robot_driver: use pass_all_args to reduce verbosity. (`#197 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/197>`_)
  The wrapper launch files essentially only provide defaults, and the common launch file requires all arguments, so we can just forward them.
* Draft for checking URLs
* Add actual documentation link into calibration checker output
  The output was generated when we didn't have the final repository available.
  However, updating the output got lost over time...
* Retry reading RTDE interface when unexpected messages appear during s… (`#186 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/186>`_)
  * Retry reading RTDE interface when unexpected messages appear during startup
  At startup we make a couple of requests to the RTDE interface. If the interface
  publishes messages by itself, a simple read() from the interface might grab another
  message than the answer.
  This change checks whether an answer to our requests was received and reports
  a warning otherwise while retrying.
* Merge pull request `#177 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/177>`_ from UniversalRobots/fix_robot_state_helper
  Make robot_state helper wait for a first status from robot before advertising the set_mode action.
* Merge pull request `#179 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/179>`_ from UniversalRobots/improve_docs
  Improve documentation
* replaced ros references that shouldn't be there (`#178 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/178>`_)
  We want to keep the pure driver part ros-independent
* Added a short section about remote-control and headless mode
* Added additional waitForService for dashboard service
* Add initialization routine for first messages
* Initialize member variables
  It can happen that the action gets triggered before the mode callback got triggered
  While this changes stops the helper from crashing when this happens, it might
  not be the best idea to do so as the question remains, what we should do
  if we haven't even received a current status from the robot.
  With the changes introduced inside this commit, the helper would trigger the
  respective state changes, which might lead to wrong requests if we aren't entirely
  sure what to do.
  One solution would be to reject goals as long as no status was received,
  but that would break such scenarios where you want to activate the robot automatically
  during startup.
  Another idea would be to delay actually starting the action server until we
  received both, robot mode and safety mode. But I am not entirely sure whether
  this will scale well.
* get effort feedback in joint_states (`#160 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/160>`_)
  Add joint currents as efforts in joint_state
  Co-authored-by: tonkei0361 <tonkei0361@gmail.com>
* Merge pull request `#166 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/166>`_ from UniversalRobots/packaget
  Use the package type and not the header type as template parameter for communication
* Implemented consuming for all primary types
  Also removed unused datatypes
* Added documentation
* Added an abstract primary consumer that can serve as a base for the visitor pattern
* Template all comm objects with the actual package type, not the header type
  When designing this driver we wanted to have all communication objects inherit
  from one common `Package` class.
  As we want to serve two different protocols (RTDE and Primary/Secondary), we
  had this Package class templated with a header type which is different in the
  two protocols. With this design decision we could have one common communication
  structure (Streams, Pipelines, Producers, Parsers, Consumers) without rewriting
  code.
  As the thing distinguishing the different protocols was the Header, we
  decided to template all the communication objects using HeaderT.
  However, as I recently realized, this destroys the possibility to easily create
  consumers using the visitor pattern as being done in the `ur_modern_driver`.
  With this, there would have to be one root consumer providing abstract methods
  for all packages available (over all interfaces).
  By templating the communication layer with the type of the actual package
  (In terms of RTDEPackage or PrimaryPackage) we can establish a visitor pattern
  at protocol level.
* Merge pull request `#141 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/141>`_ from isys-vision/robot_status
  Robot status topic via controller
* Merge pull request `#2 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/2>`_ from fmauch/robot_status
  set motion_possible to true only of robot can be actually moved
* Merge pull request `#156 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/156>`_ from UniversalRobots/ros_documentation
  Use section commands for each individual topic/service/parameter url
* Added missing doc string in launch file
* Only reflect RobotMode::RUNNING in motion_possible
* Code formatting
* set motion_possible to true only of robot can be actually moved
* Merge remote-tracking branch 'origin/robot_status' into robot_status
* Robot status: motion possible depends on error bits instead of robot mode
* Updated documentation
* Updated comments in source code
* removed temporary diff file
* Use section commands for each individual topic/service/parameter url
* Fix bug overwriting msg\_.analog_input2 variable
* Remove 2xbringup.launch
  This launchfile was created for local testing in the past and slipped through.
* RTDE handshake verification
  Throw an exception if the RTDE handshake could not be established correctly.
* Fixes controller switches to only act if necessary
  all control communication was set to false when a switch was called. This
  is not correct, as we might e.g. only start a reading controller such as
  the FTS measurements.
  Second, controllers were never checked for matching joints in this HW interface
  which is problematic in combined-hw cases.
* Merge pull request `#132 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/132>`_ from UniversalRobots/fix_dependencies
  Fix package dependencies
* Robot status: fixed in_error state
  Co-Authored-By: Felix Exner <felix_mauch@web.de>
* Added a comment about controller reset
* Reset the controller also when non-blocking read is used
  I don't see a reason why this should not happen there, as well.
* Require a controller reset when reading data from RTDE fails
  Otherwise the joint_state_controller will continue publishing old joint data
* Use SPDX license identifiers. (`#145 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/145>`_)
  From https://spdx.org
* Reduce bitset tests for in_error state
* Robot status: in_error considers several error bits
* Use scoped enums
* Added robot status controller to all configs
* Robot status: in_error considers emergency stopped flag
* Robot/safety status bits: Replaced comments by enums
* Initialize address length for accept() call (`#148 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/148>`_)
* real_time.md improvements (`#139 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/139>`_)
  When unzipping the patch file xz -d patch-4.14.139-rt66.patch.xz the xz -d command extracts the file but removes the original compressed file patch-4.14.139-rt66.patch.xz file. In a later step the patch is applied using the xz file xzcat ../patch-4.14.139-rt66.patch.xz | patch -p1. As you can see this command expects the patch-4.14.139-rt66.patch.xz file to be present in the directory. However, the file is not present because of the earlier xz -d command. Adding the -k option to the xz command extracts the file but also leaves the original compressed file in place.
  When going through the process the process failed (during make oldconfig I think) because flex and bison were not installed. Installing these packages during the apt-get install step allows make oldconfig to execute without failing due to missing packages.
* Fix typo in ur3_bringup.launch section (`#126 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/126>`_)
  The description for the ur3_bringup.launch section used the term ur5
* Adjusted dependencies and formatting
* clang formatting
* Added robot_status_controller to consistent_controllers
  fixes problem that no messages are published if robot program is not running
* Added robot_status topic via industrial robot status controller
* Merge pull request `#1 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/1>`_ from UniversalRobots/master
  Update from upstream repo
* Fix all dependencies except yaml-cpp
* Add missing package dependency (`#123 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/123>`_)
* velocity_interface is now available (`#120 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/120>`_)
* Merge pull request `#1 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/1>`_ from UniversalRobots/velocity_interface
  Adds a velocity interface to the driver.
* Updated scaled velocity controller for all models
* increase stop deceleration
  Otherwise the robot would move for too long when handing back control
  in the middle of a motion
* join move thread at script end
* Added scaled vel traj controller
  Do it for all robots
* Renamed the urscript as it is now general purpose ros_control
* Use a longer speedj time to avoid oscillations in the control cycle.
  Otherwise speed will return to 0 before a new command gets executed.
* Cleaned up launch files
* Send control type from hardware interface
  TODO:
  - Documentation of function members
  - Using enums for control modes
* added speed controllers to all robots and added ur10e_speed launchfile
* add support for speedj
* Always go through updateRobotState function in goal callback (`#99 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/99>`_)
  When robot is already in the target mode (safety- and robot mode) and the set_mode
  action is called with requesting to start the program afterwards, the program
  did not start as the robot already was at the desired state.
  However, e.g. after a protective stop that is resolved by hand (e.g. when driving
  into joint limits) users expected to call that action to restart the robot
  again.
  With this change, we do the usual check whether to start the program again.
  This way, this action can always be used to make sure the robot is running with
  the program correctly.
* Merge branch 'pr/86' 'Adding non blocking read'
* Update ROS_INTERFACE.md
* Merge pull request `#93 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/93>`_ from UniversalRobots/fix_sockets_close
  Close all closable sockets
* Merge branch 'master' into adding_non_blocking_read
* Added a comment about explicitly calling ReverseInterface's destructor
* Close all closable sockets
  Sockets do not necessarily have to be in state connected when they should be
  closed. Before, only connected sockets got closed leading to a "socket leak"
  if a socket was disconnected before a close request was processed.
  With this fix all sockets with a valid file descriptor get closed when close()
  is being called.
* Parameterising gains (`#88 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/88>`_)
  * added parameters for servoj_gain and servoj_lookahead_time
  * changing to ros_error_stream
  * lint
  * added documentation
* Merge pull request `#6 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/6>`_ from fmauch/adding_non_blocking_read
  Added documentation for non_blocking_read parameter
* config: use yaml anchor to reduce magic nrs. (`#89 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/89>`_)
  Users can still customise the publish_rate by removing the alias and specifying a custom rate.
  By default all controllers will publish at the controller's native rate.
* Added documentation for non_blocking_read parameter
* Update hardware_interface.cpp
* Update ur_driver.cpp
* Update hardware_interface.cpp
* lint
* lint
* add non-blocking-read for combined_robot_hw
* Merge pull request `#1 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/1>`_ from UniversalRobots/master
  update our master
* fixed duplicated service advertisements (`#75 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/75>`_)
* 'reverse_port' and 'script_sender_port' parameters (`#57 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/57>`_)
  Adds parameters for reverse_port and script_sender_port.
  This was implemented by @khssnv Thanks!
* Fix spelling of "actual_main_voltage"
* robot_driver: update tracker and repo urls.
  Copy-pasta from ur_modern_driver.
* Merge pull request `#48 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/48>`_ from UniversalRobots/tare_sensor
  Added a service to zero the robot's ftsensor
* Deny taring the TF sensor when major version is < 5
* Added a service to zero the robot's ftsensor
* Changed my name in every occurence
* Fix faulty 1MBaud rate
  It actually had a 0 too much. We use scientific notation to make this more clearly visible in future.
* Merge pull request `#49 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/49>`_ from UniversalRobots/end_script_command
  Always end script commands with a newline
* Add documentation why we append a newline.
  Co-Authored-By: G.A. vd. Hoorn <g.a.vanderhoorn@tudelft.nl>
* Always end script commands with a newline
  Otherwise script will not be interpreted by the robot which might be counter-intuitive.
  Changing the behavior as such will also be the same as in the ur_modern_driver
  so migrating will be easier.
  I decided to change the function's interface to copy the string in order add a
  trailing '\n' if necessary.
* Merge pull request `#34 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/34>`_ from tecnalia-medical-robotics/combined_hw
  Support for combined robot hardware
* Use a spawner to load stopped controllers to avoid confusion about finished nodes
  Before, we used the controller_manager/controller_manager node to load unstarted
  controllers, which logged a "finished cleanly" after loading the controllers.
  This led to confusion as actually you don't expect something to exit when
  starting the driver.
* Separate ROS related sources from ur_robot_driver library
* Avoid same source files to be built and linked in several places
* Add Missing dashboard client source file
* Minimum changes to add support for combined hardware interface
* Updated ROS interface documentation
* robot_driver: remove industrial_msgs dependency.
  It's not actually used (yet).
* Merge branch 'robot_status'
  Propagating the robot's status (robot mode and safety mode) to the user
  so she can act accordingly (e.g. unlock after a protective stop or power
  on the robot if required)
* Updated documentation
* Updated documentation regarding the full headless mode
* Start robot_state_helper together with driver from launchfile
* Do not specify hw-interface's namespace explicitly
* Added ROS interface documentation for state helper
* Added code documentation
* Added functionality to automatically restart the running program after recovery
* Implemented setMode action to bring the robot into a desired mode (e.g. RUNNING)
* Added a separate helper node that will handle robot and safety mode changes
* Create a common datatypes.h file for UR enums
  The enumerators are used through different interfaces which is why I think
  it is beneficial to pull them out into a separate header file.
* Publish robot mode and safety mode from RTDE
* Merge pull request `#16 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/16>`_ from UniversalRobots/dashboard_client
  Add a dashboard client to the driver
* Renaming source files for DashboardClientROS
* Removed leftover code fragments
* Explicitly delete default constructor of DashboardClient and DashboardClientROS
* Added more comments
* Added ur_dashbaord_msgs to the dependency list
* Updated service documentation
* Also publish robot mode
* Added the ability to reconnect to the dashboard server
* Use a timeout for dashboard server
  When the timeout is exceeded, a TimoutException is thrown causing the service
  to fail.
  All dashboard services return (almost) immediately, so actions do not really
  make sense here. The only exception is when there is a problem with the dasboard
  connection, which is why we introduce the timeout.
  This way, service calls will not block forever, when connection to the
  dashboard server got lost or if the server isn't answering due to any other
  reason.
* Added more dashboard services
* Simplify service advertisements
  For advertising the services I use a combination of a MACRO and a lambda,
  as suggested by @gavanderhoorn. I'm currently not completely happy with this,
  as I don't like using macros, but a "double" lambda seemed not to work.
* Added documentation to dashboard server
* Add a dashboard client to the hardware interface
* Moved dashboard functionality completely out of client
  The client itself should only be an abstraction of the actual interface
  which is sending strings and receiving strings as answers.
  All interpretation of those answers is now moved to the ROS module.
* Renamed the standalone dashboard server node
* return server response to caller
* Added first version of dashboard client
* Merge pull request `#18 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/18>`_ from UniversalRobots/fix_init_timing
  Fix init timing.
  Before pipeline overflows could happen at startup
* When no controller is active, set the current point as setpoint.
* Fixed a comment
* Merge branch 'formatting' into fix_init_timing
* Merge pull request `#21 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/21>`_ from UniversalRobots/formatting
  Formatting
* Removed spaces before ::
  How can I get clang-format-6 to do that? I only managed to get this working
  using clang-format 3.9
* Mark all producer methods as overrides
* Corrected typo in log message
* Start rtde client specifically
* Added more log output on errors
* Refactoring of RTDE client initialization
* Make pipeline stop- and restartable
* pass tcp_port parameter as string
* Renamed the driver to ur_robot_driver
* Contributors: Alisher A. Khassanov, Axel, Christian Jülg, Collin Avidano, Emil Ancker, Felix Exner, Felix Mauch, G.A. vd. Hoorn, Gyan Tatiya, Hongzhuo Liang, Krzysztof Stężała, Mads Holm Peters, Martin Günther, Mingu Kwon, Niels Hvid, RobertWilbrandt, Tejas Kumar Shastha, Tom Queen, Tristan Schnell, asier, axelschroth, carebare47, gavanderhoorn, giusebar, mahp, sharpe, steinmn, t-schnell, urrsk

0.0.3 (2019-08-09)
------------------
* Added a service to end ROS control from ROS side
* Publish IO state on ROS topics
* Added write channel through RTDE with speed slider and IO services
* Added subscriber to send arbitrary URScript commands to the robot

0.0.2 (2019-07-03)
------------------
* Fixed dependencies and installation
* Updated README
* Fixed passing parameters through launch files
* Added support for correctly switching controllers during runtime and using the standard
  joint_trajectory_controller
* Updated externalcontrol URCap to version 1.0.2
  + Fixed Script timeout when running the URCap inside of a looping tree
  + Fixed a couple of typos
* Increased minimal required UR software version to 3.7/5.1

0.0.1 (2019-06-28)
------------------
Initial release
