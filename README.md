# ur_moden_driver
ur_modern_driver
======
The new driver for the UR3/UR5/UR10 robot arms from universal robots

__Installation__
Just clone the repository into your catkin working directory and make it with ```catkin_make```.

Note that this package depends on ur_msgs, so it cannot directly be used with ROS versions prior to hydro

---

__Usage__
The driver is designed to be a drop-in replacement of the ur\_driver package. It _won't_ overwrite your current driver though, so you can use and test this package without risking to break your current setup.

Just use the modified launch files included in this package instead of those in ur\_bringup. Everything else should work as usual.


 
