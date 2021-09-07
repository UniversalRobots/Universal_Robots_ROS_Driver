2.0.0 (2021-09-07)
------------------
* Add new features to the driver:
  - Cartesian position-based control
  - Cartesian twist-based control
  - Trajectory forwarding for execution on robot
  - More documentation and examples
* Fix package name in calibration example
  The example was referencing the library package where it should have been
  referencing the driver package.
* Added log handler for handling log messages from client library with … (`#398 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/398>`_)
* Added "On behalf of Universal Robots A/S" notice (`#416 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/416>`_)
  Removed copyright notice from LICENSE file, as the license file itself isn't
  copyrighted by FZI
* Replace two logging macros with ROS logging macros. (`#330 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/330>`_)
  Inside the driver we want to use plain ROS logging instead of the library's logging macros.
* Use ur_client_library package for building this driver `#266 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/266>`_ from UniversalRobots/separation
* Use namespace urcl instead of ur_driver
* Renamed library
* Made calibration use separated library, as well
* Merge pull request `#166 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/166>`_ from UniversalRobots/packaget
  Use the package type and not the header type as template parameter for communication
* Update ur_calibration to new template structure
* Use SPDX license identifiers. (`#145 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/145>`_)
  From https://spdx.org
* Merge pull request `#1 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/1>`_ from UniversalRobots/master
  Update from upstream repo
* catch all exceptions to bring them to output. (`#98 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/98>`_)
* Changed my name in every occurence
* Merge pull request `#42 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/42>`_ from gavanderhoorn/ur_calib_fix_yamlcpp_dep
  Fixup yaml-cpp linking and include paths in ur_calibration
* calibration: fix yaml-cpp include paths and linking.
* calibration: top-level pkgs don't export include dirs or libraries.
* calibration: remove boilerplate comments from build script.
* Renamed the driver to ur_robot_driver
* Simplified calibration interface
  When showing this to collegues it turned out that this was quite complicated.
  I reduced the target definition to a simple filename, while documenting
  the package approach separately.
* Updated calibration instructions
* Contributors: Felix Exner, Felix Mauch, G.A. vd. Hoorn, Lea Steffen, Mads Holm Peters, axelschroth, gavanderhoorn

0.0.2 (2019-07-03)
------------------
* Catch exception and log error when no connection can be established

0.0.1 (2019-06-28)
------------------
Initial release
