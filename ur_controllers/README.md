# ur_controllers

This package contains controllers and hardware interface for ROS control that are special to the UR
robot family. Currently this contains

  * A **speed_scaling_interface** to read the value of the current speed scaling into controllers.
  * A **scaled_joint_command_interface** that provides access to joint values and commands in 
  combination with the speed scaling value.
  * A **speed_scaling_state_controller** that publishes the current value of the speed scaling
  to a topic interface.
  * A **scaled_joint_trajectory_controller** that is similar to the *joint_trajectory_controller*,
  but it uses the speed scaling reported by the robot to reduce progress in the trajectory.
