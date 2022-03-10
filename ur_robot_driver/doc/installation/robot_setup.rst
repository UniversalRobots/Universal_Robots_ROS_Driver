Setting up a UR robot for ur_robot_driver
=========================================

Prepare the robot
-----------------

For using the *ur_robot_driver* with a real robot you need to install the
**externalcontrol-1.0.5.urcap** which can be found inside the **resources** folder of this driver.

**Note**: For installing this URCap a minimal PolyScope version of 3.7 or 5.1 (in case of e-Series) is
necessary.

For installing the necessary URCap and creating a program, please see the individual tutorials on
how to :ref:`setup a cb3 robot <install-urcap-cb3>` or how to
:ref:`setup an e-Series robot <install-urcap-e-series>`.

To setup the tool communication on an e-Series robot, please consider the :ref:`tool communication setup
guide <setup-tool-communication>`.

Prepare the ROS PC
------------------

For using the driver make sure it is installed (either by the debian package or built from source
inside a catkin workspace).

Extract calibration information
-------------------------------

Each UR robot is calibrated inside the factory giving exact forward and inverse kinematics. To also
make use of this in ROS, you first have to extract the calibration information from the robot.

Though this step is not necessary to control the robot using this driver, it is highly recommended
to do so, as otherwise endeffector positions might be off in the magnitude of centimeters.


For this, there exists a helper script:

.. code:: bash

   $ roslaunch ur_calibration calibration_correction.launch \
     robot_ip:=<robot_ip> target_filename:="${HOME}/my_robot_calibration.yaml"

For the parameter ``robot_ip`` insert the IP address on which the ROS pc can reach the robot. As
``target_filename`` provide an absolute path where the result will be saved to.

We recommend keeping calibrations for all robots in your organization in a common package. See the
`package's documentation <ur_calibration/README.md>`_ for details.

