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

With the parameters explained below calibration will be saved inside
```bash
<output_package_name>/<subfolder>/<robot_name>_calibration.yaml
```


#### Helper script
In the launch folder of the ur_calibration package is a helper script:

    $ roslaunch ur_calibration calibration_correction.launch \
    robot_ip:=<robot_ip> \
    robot_name:=<robot_name> \
    output_package_name:=ur_calibration \
    subfolder_name:=etc

For the parameter **<robot_ip>** insert the ip on which the ROS pc can reach the robot. The
**<robot_name>** is an arbitrary name you can give to the robot. It is recommended, to choose a unique
name that can be easily matched to the physical robot.

As soon as you see the output
    
    [ INFO] [1560953586.352160902]: Calibration correction done

you can exit the roslaunch by pressing `CTRL+C`.
    
    
#### Prerequisites for binary installation
In the example above, we use the **ur_calibration** package from this repository. This won't work,
if you use a binary installation of this driver. In that case please create a new ROS package 
by going to your catkin_workspace's src folder and calling:

    catkin_create_pkg my_calibrations

It is recommended to adapt the new package's *package.xml* with a meaningful description.
