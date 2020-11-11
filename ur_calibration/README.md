# ur_calibration

Package for extracting the factory calibration from a UR robot and changing it to be used by `ur_description` to gain a correct URDF model.

Each UR robot is calibrated inside the factory giving exact forward and inverse kinematics. To also
make use of this in ROS, you first have to extract the calibration information from the robot.

Though this step is not necessary, to control the robot using this driver, it is highly recommended
to do so, as end effector positions might be off in the magnitude of centimeters.

## Nodes
### calibration_correction
This node extracts calibration information directly from a robot, calculates the URDF correction and
saves it into a .yaml file.

In the launch folder of the ur_calibration package is a helper script:

```bash
$ roslaunch ur_calibration calibration_correction.launch \
robot_ip:=<robot_ip> target_filename:="${HOME}/my_robot_calibration.yaml"
```

For the parameter `robot_ip` insert the IP address on which the ROS pc can reach the robot. As
`target_filename` provide an absolute path where the result will be saved to.
    
## Creating a calibration / launch package for all local robots
When dealing with multiple robots in one organization it might make sense to store calibration data
into a package dedicated to that purpose only. To do so, create a new package (if it doesn't already
exist)

```bash
# Replace your actual catkin_ws folder
$ cd <catkin_ws>/src
$ catkin_create_pkg example_organization_ur_launch ur_client_library \
-D "Package containing calibrations and launch files for our UR robots."
# Create a skeleton package
$ mkdir -p example_organization_ur_launch/etc
$ mkdir -p example_organization_ur_launch/launch
```

We can use the new package to store the calibration data in that package. We recommend naming each
robot individually, e.g. *ex-ur10-1*.

```bash
$ roslaunch ur_calibration calibration_correction.launch \
robot_ip:=<robot_ip> \
target_filename:="$(rospack find example_organization_ur_launch)/etc/ex-ur10-1_calibration.yaml"
```

To make life easier, we create a launchfile for this particular robot. We base it upon the
respective launchfile in the driver:

```bash
# Replace your actual catkin_ws folder
$ cd <catkin_ws>/src/example_organization_ur_launch/launch
$ roscp ur_client_library ur10_bringup.launch ex-ur10-1.launch
```

Next, modify the parameter section of the new launchfile to match your actual calibration:

```xml
<!-- Note: Only the relevant lines are printed here-->
  <arg name="robot_ip" default="192.168.0.101"/> <!-- if there is a default IP scheme for your
  robots -->
  <arg name="kinematics_config" default="$(find example_organization_ur_launch)/etc/ex-ur10-1_calibration.yaml"/>
```

Then, anybody cloning this repository can startup the robot simply by launching

```bash
$ roslaunch example_organization_ur_launch ex-ur10-1.launch
```
