# ROS-Controllers available for the ur_robot_driver
This help page describes the different controllers available for the `ur_robot_driver`. This should
help users finding the right controller for their specific use case.

## Where are controllers defined?
Controllers are defined in the `config/ur<model>_controllers.yml` files, e.g.
[`ur10e_controllers.yml`](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/config/ur10e_controllers.yaml)
for the UR10e robot.

## How do controllers get loaded and started?
As this driver uses ROS-control all controllers are managed by the
[`controller_manager`](http://wiki.ros.org/controller_manager). During startup, a default set of
running controllers is loaded and started, another set is loaded in stopped mode. Stopped
controllers won't be usable right away, but their ROS interfaces are still visible on the command line e.g. by
`rostopic list`.

For switching controllers, use the methods offered by the
[`controller_manager`](http://wiki.ros.org/controller_manager) e.g. to switch from the default
`scaled_pos_joint_traj_controller` to the `joint_group_vel_controller` :

```bash
rosservice call /controller_manager/switch_controller "start_controllers: ['joint_group_vel_controller']
stop_controllers: ['scaled_pos_joint_traj_controller']
strictness: 2
start_asap: false
timeout: 0.0"
```

For more information on that topic, please refer to the
[`controller_manager`](http://wiki.ros.org/controller_manager)'s documentation.

## Read-Only controllers
These "controllers" are read-only. They read states from the robot and publish them on a ROS topic.
As they are read-only, they don't claim any resources and can be combined freely. By default, they
are all started and running. Those controllers do not require the robot to have the
`external_control` script running.

### `joint_state_controller`
**Type:** [`joint_state_controller/JointStateController`](http://wiki.ros.org/joint_state_controller)

Publishes all joints' positions, velocities, and motor currents as
[`sensor_msgs/JointState`](http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html) on the
`/joint_states` topic. Note that the `effort` field contains the currents reported by the joints and
not the actual *efforts* in a physical sense.

### `robot_status_controller`
**Type:** [`industrial_robot_status_controller/IndustrialRobotStatusController`](http://wiki.ros.org/industrial_robot_status_controller)

Controller that publishes robot/controller state (e-stopped, in motion, etc) as
[`industrial_msgs/RobotStatus`](http://docs.ros.org/en/melodic/api/industrial_msgs/html/msg/RobotStatus.html)
messages on the `/robot_status` topic.

### `force_torque_sensor_controller`
**Type:** [`force_torque_sensor_controller/ForceTorqueSensorController`](http://wiki.ros.org/force_torque_sensor_controller)

This controller publishes the wrench measured from the robot at its TCP as
[`geometry_msgs/WrenchStamped`](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/WrenchStamped.html)
on the topic `/wrench`. Note that the wrench can be tared to 0 using the
`zero_ftsensor`(https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/ROS_INTERFACE.md#zero_ftsensor-std_srvstrigger)
service.

### `speed_scaling_state_controller`
**Type:** [`scaled_controllers/SpeedScalingStateController`](http://wiki.ros.org/speed_scaling_state_controller)

Reports the current value of the speed multiplier used from the robot. This is the product of the
speed slider value as it is on the teach pendant and any active speed scaling mechanism. In other
words, this represents the factor of the real slowdown relative to the commanded motion.

## Commanding controllers
The commanding controllers control the robot's motions. They all claim all joints and therefore only
one of the following can be active at one time. You can define your own controllers with only a
subset of all joints, but this won't be covered here.

### `scaled_pos_joint_traj_controller` (Default motion controller)
**Type:** [`position_controllers/ScaledJointTrajectoryController`](http://wiki.ros.org/scaled_joint_trajectory_controller)

Similar to the non-scaled version it implements the
[`control_msgs/FollowJointTrajectory`](http://docs.ros.org/en/latest/api/control_msgs/html/action/FollowJointTrajectory.html)
interface. For controlling the robot it will send joint position commands.

In contrast to the non-scaled version, the speed slider position or any other speed-modifying action
such as pausing, EM-Stop or speed scaling due to limits configured on the robot will be respected by
this controller.  If a trajectory is given with time constraints and the trajectory is slowed down
during execution, the action fails. If no time constraints are set and the motion is slowed down
during execution, the actual action call will succeed.

### `joint_group_vel_controller`
**Type:** [`velocity_controllers/JointGroupVelocityController`](http://wiki.ros.org/forward_command_controller)

Directly command joint velocities though a topic interface. This controller is useful when doing
things like servoing in joint space.

### `twist_controller`
**Type:** [`ros_controllers_cartesian/TwistController`](http://wiki.ros.org/twist_controller)

Similar to the `joint_group_vel_controller` this controller allows directly sending speeds via a
topic interface to the robot. However, this controller expects a Cartesian TCP twist (linear and
angular velocity). This is useful in Cartesian servoing applications such as visual servoing or
teleoperation.

Note that this controller does not check for configuration changes in the robot's kinematics, twist
commands are sent for execution to the robot. Therefore, commanded motions can result in protective
stops, e.g. when leaving the robot's operation space or forcing it to change its configuration.

---
Controllers below this line are probably less useful for the majority of applications, but can be
used nevertheless.

### `pos_joint_traj_controller`
**Type:** [`position_controllers/JointTrajectoryController`](http://wiki.ros.org/joint_trajectory_controller)

This is the most commonly used ROS controller implementing the
[`control_msgs/FollowJointTrajectory`](http://docs.ros.org/en/latest/api/control_msgs/html/action/FollowJointTrajectory.html)
interface. For controlling the robot it will send joint position commands.

**We do not recommend using this controller, as it does not respect the robot's speed scaling value.
Setting the speed slider at another position than 100% or pausing the robot will lead to unexpected
side-effects! Use the scaled version of this controller instead.**

### `vel_joint_traj_controller`
**Type:** [`velocity_controllers/JointTrajectoryController`](http://wiki.ros.org/joint_trajectory_controller)

Implements the
[`control_msgs/FollowJointTrajectory`](http://docs.ros.org/en/latest/api/control_msgs/html/action/FollowJointTrajectory.html)
interface. For controlling the robot it will send joint velocity commands.

As this controller uses closed-loop velocity control, it requires tuned PID parameters. Currently,
they are not optimized, see
[#119](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/119).

### `scaled_vel_joint_traj_controller`
**Type:** [`velocity_controllers/ScaledJointTrajectoryController`](http://wiki.ros.org/scaled_joint_trajectory_controller)

Similar to the non-scaled version it implements the
[`control_msgs/FollowJointTrajectory`](http://docs.ros.org/en/latest/api/control_msgs/html/action/FollowJointTrajectory.html)
interface. For controlling the robot it will send joint velocity commands.

In contrast to the non-scaled version, the speed slider position or any other speed-modifying action
such as pausing, EM-Stop or speed scaling due to limits configured on the robot will be respected by
this controller.  If a trajectory is given with time constraints and the trajectory is slowed down
during execution, the action fails. If no time constraints are set and the motion is slowed down
during execution, the actual action call will succeed.

### `forward_joint_traj_controller`
**Type:** [`pass_through_controllers/JointTrajectoryController`](http://wiki.ros.org/pass_through_controllers)

This controller implements the
[`control_msgs/FollowJointTrajectory`](http://docs.ros.org/en/latest/api/control_msgs/html/action/FollowJointTrajectory.html)
interface. Instead of interpolating the trajectory on ROS side, this forwards the complete
trajectory to the robot for execution leaving the robot in charge of calculating accelerations and
speeds. Therefore, this will lead to motions most similar to what you could configure on the teach
pendant.

As this uses the robot controller's trajectory execution, trajectories will be slightly different to
the streaming controllers such as the `scaled_pos_joint_traj_controller`. Each setpoint will be
blended with a (currently [hard coded]()https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/125f4f7a48b299b07e298cf479e3ba6882c3af3a/include/ur_client_library/ur/ur_driver.h#L203) blend radius.

### `forward_cartesian_traj_controller`
**Type:** [`pass_through_controllers/JointTrajectoryController`](http://wiki.ros.org/pass_through_controllers)

This controller implements the
[`cartesian_control_msgs/FollowCartesianTrajectory`](http://docs.ros.org/en/noetic/api/cartesian_control_msgs/html/action/FollowCartesianTrajectory.html)
interface. Instead of interpolating the trajectory on ROS side, this forwards the complete
trajectory to the robot for execution leaving the robot in charge of calculating accelerations and
speeds. Therefore, this will lead to motions most similar to what you could configure on the teach
pendant.

Note that this controller does not check for configuration changes in the robot's kinematics, so
trajectories executed with this controller can potentially result in a protective stop.

As this uses the robot controller's trajectory execution, trajectories will be slightly different to
the streaming controllers such as the `scaled_pos_joint_traj_controller`. Each setpoint will be
blended with a (currently [hard coded]()https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/125f4f7a48b299b07e298cf479e3ba6882c3af3a/include/ur_client_library/ur/ur_driver.h#L203) blend radius.

### `pose_based_cartesian_traj_controller`
**Type:** [`pose_controllers/CartesianTrajectoryController`](http://wiki.ros.org/ros_controllers_cartesian)

This controller implements the
[`cartesian_control_msgs/FollowCartesianTrajectory`](http://docs.ros.org/en/noetic/api/cartesian_control_msgs/html/action/FollowCartesianTrajectory.html)
interface. Cartesian motion commands are continuously streamed to the robot hardware for execution,
where it uses the robot's own inverse kinematics for calculating the joint commands.

### `joint_based_cartesian_traj_controller`
**Type:** [`position_controllers/CartesianTrajectoryController`](http://wiki.ros.org/ros_controllers_cartesian)

This controller implements the
[`cartesian_control_msgs/FollowCartesianTrajectory`](http://docs.ros.org/en/noetic/api/cartesian_control_msgs/html/action/FollowCartesianTrajectory.html)
interface. The continuously generated Cartesian motion commands are transformed to joint commands
using an inverse kinematics method on the ROS side. The robot itself receives joint commands. The IK
method used is interchangeable, see the controller's documentation for details.
