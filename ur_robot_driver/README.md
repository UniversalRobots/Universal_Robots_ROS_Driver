# ur_robot_driver 

This package contains the actual driver for UR robots. It is part of the *universal_robots_driver*
repository and requires other packages from that repository. Also, see the [main repository's
README](../README.md) for information on how to install and startup this driver.

## ROS-API
The ROS API is documented in a [standalone document](doc/ROS_INTERFACE.md). It is auto-generated
using [catkin_doc](https://github.com/fzi-forschungszentrum-informatik/catkin_doc).

## Technical details
The following image shows a very coarse overview of the driver's architecture.

![Architecture overview](doc/architecture_coarse.svg "Architecture overview")

Upon connection to the primary interface the robot sends version and calibration information which
is consumed by the *calibration_check*. If the calibration reported by the robot doesn't match the
one configured (See [calibration guide](../ur_calibration/README.md)) an error will be printed to Roslog.

Real-time data from the robot is read through the RTDE interface. This is done automatically as soon
as a connection to the robot could be established. Thus joint states and IO data will be immediately
available.

To actually control the robot, a program node from the **External Control** URCap must be running on
the robot interpreting commands sent from an external source. When this program is not running, no
controllers moving the robot around will be available, which is handled by the
[controller_stopper](../controller_stopper/README.md). Please see the [initial setup
guide](../README.md) on how to install and start this on the robot.

The URScript that will be running on the robot is requested by the **External Control** program node
from the remote ROS PC. The robot *ur_control.launch* file has a parameter called `urscript_file` to
select a different program than the default one that will be sent as a response to a program
request.

**Custom script snippets** can be sent to the robot on a topic basis. By default, they will
interrupt other programs (such as the one controlling the robot). For a certain subset of functions,
it is however possible to send them as secondary programs. See [UR
documentation](https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/secondary-program-17257/)
on details.
<br/>
**Note to e-Series users:**
The robot won't accept script code from a remote source unless the robot is put into
*remote_control-mode*. However, if put into *remote_control-mode*, the program containing the
**External Control** program node can't be started from the panel.
For this purpose, please use the **dashboard** services to load, start and stop the main program
running on the robot. See the [ROS-API documentation](doc/ROS_INTERFACE.md) for details on the
dashboard services.

For using the **tool communication interface** on e-Series robots, a `socat` script is prepared to
forward the robot's tool communication interface to a local device on the ROS PC. See [the tool
communication setup guide](doc/setup_tool_communication.md) for details.

This driver is using [ROS-Control](https://wiki.ros.org/ros_control) for any control statements.
Therefor, it can be used with all position-based controllers available in ROS-Control. However, we
recommend using the controllers from the `ur_controllers` package. See it's
[documentation](../ur_controllers/README.md) for details. **Note: Speed scaling support will only be
available using the controllers from `ur_controllers`**
