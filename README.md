# ur_moden_driver

The new driver for the UR3/UR5/UR10 robot arms from universal robots

__Installation__

Just clone the repository into your catkin working directory and make it with ```catkin_make```.

Note that this package depends on ur_msgs, so it cannot directly be used with ROS versions prior to hydro

---

__Usage__

The driver is designed to be a drop-in replacement of the ur\_driver package. It _won't_ overwrite your current driver though, so you can use and test this package without risking to break your current setup.

Just use the modified launch files included in this package instead of those in ur\_bringup. Everything else should work as usual.

---

__Improvements__


The driver exposes the same functionality as the previous ur\_driver:

*Action interface on _/follow\_joint\_trajectory_ for seamless integration with MoveIt

*Publishes robot joint state on _/joint\_states_

*Publishes TCP force on state on _/wrench_

*Publishes IO states state on _/io\_states_

*Service call to set outputs and payload (Note: I am not sure if setting the payload actually works, as the robot GUI does not update. This is also true for the old ur\_driver  )


Besides this, the driver subscribes to two new topics:

*/ur\_driver/URScript : takes messages of type _std\_msgs/String_ and directly forwards it to the robot. Note that no control is done on the input, so use at your own risk! Inteded for sending movel/movej commands directly to the robot.

*/joint\_speed : takes messages of type trajectory\_msgs/JointTrajectory, parses the first JointTracetoryPoint and sends the specified joint speeds and accelerations to the robot. This interface is intended for doing visual servoing and other kind of control that requires speed control rather than position control of the robot. Remember to set values for all 6 joints. Ignores the field joint\_names, so set the values in the correct order.


This driver is written in c++, which should make it possible to integrate it with ros_control. If you fell like undertaking this task, please dive right in. I don't have the posibility to do this.

No script is sent to the robot. This means that the teach pendant can be used to move the robot around while the driver is running.

---
Should be compatible with all robots and control boxes with the newest firmware.
Tested with:

*Real UR10 with CB2 running 1.8.14035

*Real UR5 with CB2 running 1.8.14035


*Simulated UR3 running 3.1.18024

*Simulated UR5 running 3.0.16471

*Simulated UR5 running 1.8.16941

*Simulated UR5 running 1.7.10857

*Simulated UR5 running 1.6.08725

 
