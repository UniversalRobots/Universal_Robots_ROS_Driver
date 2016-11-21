# ur_modern_driver

A new driver for the UR3/UR5/UR10 robot arms from Universal Robots. It is designed to replace the old driver transparently, while solving some issues, improving usability as well as enabling compatibility  of ros_control. 

## Improvements

* A script is only running on the robot while a trajectory is actually executing. This means that the teach pendant can be used to move the robot around while the driver is connected.

* The driver exposes the same functionality as the previous ur\_driver:

  * Action interface on */follow\_joint\_trajectory* for seamless integration with MoveIt

  * Publishes robot joint state on */joint\_states*

  * Publishes TCP force on */wrench*

  * Publishes IO state on */ur\_driver/io\_states* (Note that the string */ur\_driver* has been prepended compared to the old driver)

  * Service call to set outputs and payload - Again, the string */ur\_driver* has been prepended compared to the old driver (Note: I am not sure if setting the payload actually works, as the robot GUI does not update. This is also true for the old ur\_driver  )


* Besides this, the driver subscribes to two new topics:

  * */ur\_driver/URScript* : Takes messages of type _std\_msgs/String_ and directly forwards it to the robot. Note that no control is done on the input, so use at your own risk! Inteded for sending movel/movej commands directly to the robot, conveyor tracking and the like.

  * */joint\_speed* : Takes messages of type _trajectory\_msgs/JointTrajectory_. Parses the first JointTracetoryPoint and sends the specified joint speeds and accelerations to the robot. This interface is intended for doing visual servoing and other kind of control that requires speed control rather than position control of the robot. Remember to set values for all 6 joints. Ignores the field joint\_names, so set the values in the correct order.

* Added support for ros_control. 
  * As ros_control wants to have control over the robot at all times, ros_control compatibility is set via a parameter at launch-time. 
  * With ros_control active, the driver doesn't open the action_lib interface nor publish joint_states or wrench msgs. This is handled by ros_control instead.
  * Currently two controllers are available, both controlling the joint position of the robot, useable for trajectroy execution
    * The velocity based controller sends joint speed commands to the robot, using the speedj command
    * The position based controller sends joint position commands to the robot, using the servoj command
    * I have so far only used the velocity based controller, but which one is optimal depends on the application.
  * As ros_control continuesly controls the robot, using the teach pendant while a controller is running will cause the controller **on the robot** to crash, as it obviously can't handle conflicting control input from two sources. Thus be sure to stop the running controller **before** moving the robot via the teach pendant:
    * A list of the loaded and running controllers can be found by a call to the controller_manager ```rosservice call /controller_manager/list_controllers {} ```
    * The running position trajectory controller can be stopped with a call to  ```rosservice call /universal_robot/controller_manager/switch_controller "start_controllers: - '' stop_controllers: - 'pos_based_pos_traj_controller' strictness: 1" ``` (Remember you can use tab-completion for this)
   

## Installation

Just clone the repository into your catkin working directory and make it with ```catkin_make```.

Note that this package depends on ur_msgs, hardware_interface, and controller_manager so it cannot directly be used with ROS versions prior to hydro.

## Usage

The driver is designed to be a drop-in replacement of the ur\_driver package. It _won't_ overwrite your current driver though, so you can use and test this package without risking to break your current setup.

If you want to test it in your current setup, just use the modified launch files included in this package instead of those in ur\_bringup. Everything else should work as usual.

---
If you would like to use the ros\_control-based approach, use the launch files urXX\_ros\_control.launch, where XX is '5' or '10' depending on your robot.

**Note:** If you are using the ros\_control-based approach you will need 2 packages that can be found in the ur\_driver package. If you do not have the ur\_driver package in your workspace simply copy these packages into your workspace /src folder:
 * urXX_moveit_config
 * ur_description

The driver currently supports two position trajectory controllers; a position based and a velocity based. They are both loaded via the launch file, but only one of them can be running at the same time. By default the velocity based controller is started. You can switch controller by calling the appropriate service:
```
rosservice call /universal_robot/controller_manager/switch_controller "start_controllers:
- 'vel_based_pos_traj_controller'
stop_controllers:
- 'pos_based_pos_traj_controller'
strictness: 1"
```
Be sure to stop the currently running controller **either before or in the same call** as you start a new one, otherwise it will fail.

The position based controller *should* stay closer to the commanded path, while the velocity based react faster (trajectory execution start within 50-70 ms, while it is in the 150-180ms range for the position_based. Usage without ros_control as well as the old driver is also in the 170ms range, as mentioned at my lightning talk @ ROSCon 2013).

**Note** that the PID values are not optimally tweaked as of this moment.

To use ros_control together with MoveIt, be sure to add the desired controller to the ```controllers.yaml``` in the urXX_moveit_config/config folder. Add the following 
```
controller_list:
 - name: /vel_based_pos_traj_controller #or /pos_based_pos_traj_controller
   action_ns: follow_joint_trajectory
   type: FollowJointTrajectory
   default: true
   joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
```

## Using the tool0_controller frame

Each robot from UR is calibrated individually, so there is a small error (in the order of millimeters) between the end-effector reported by the URDF models in https://github.com/ros-industrial/universal_robot/tree/indigo-devel/ur_description and
the end-effector as reported by the controller itself.

This driver broadcasts a transformation between the base link and the end-effector as reported by the UR. The default frame names are: *base* and *tool0_controller*. 

To use the *tool0_controller* frame in a URDF, there needs to be a link with that name connected to *base*. For example:

```
<!-- Connect tool0_controller to base using floating joint -->
<link name="tool0_controller"/>
<joint name="base-tool0_controller_floating_joint" type="floating">
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <parent link=base"/>
  <child link="tool0_controller"/>
</joint>
```

Now, the actual transform between *base* and *tool0_controller* will not be published by the *robot_state_publisher* but will be taken from this driver via */tf*.

NOTE: You need an up-to-date version of *robot_state_publisher* that is able to deal with floating joints, see: https://github.com/ros/robot_state_publisher/pull/32

## Compatibility
Should be compatible with all robots and control boxes with the newest firmware.

###Tested with:

*Real UR10 with CB2 running 1.8.14035

*Real UR5 with CB2 running 1.8.14035


*Simulated UR3 running 3.1.18024

*Simulated UR5 running 3.0.16471

*Simulated UR5 running 1.8.16941

*Simulated UR5 running 1.7.10857

*Simulated UR5 running 1.6.08725

 
#Credits
Please cite the following report if using this driver

@techreport{andersen2015optimizing, 
  title = {Optimizing the Universal Robots ROS driver.}, 
  institution = {Technical University of Denmark, Department of Electrical Engineering}, 
  author = {Andersen, Thomas Timm}, 
  year = {2015}, 
  url = {http://orbit.dtu.dk/en/publications/optimizing-the-universal-robots-ros-driver(20dde139-7e87-4552-8658-dbf2cdaab24b).html} 
  }


The report can be downloaded from http://orbit.dtu.dk/en/publications/optimizing-the-universal-robots-ros-driver(20dde139-7e87-4552-8658-dbf2cdaab24b).html
