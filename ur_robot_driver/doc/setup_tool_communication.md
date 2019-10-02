# Setting up the tool communication on an e-Series robot
The Universal Robots e-Series provides an rs485 based interface at the tool flange that can be used
to attach an rs485-based device to the robot's tcp without the need to wire a separate cable along
the robot.

This driver enables forwarding this tool communication interface to an external machine for example
to start a device's ROS driver on a remote PC.

This document will guide you through installing the URCap needed for this and setting up your ROS
launch files to utilize the robot's tool communication.

## Robot setup
For setting up the robot, please install the **rs485-1.0.urcap** found in the **resources** folder.
Installing a URCap is explained in the [setup guide](install_urcap_e_series.md) for the **external-control** URCap.

After installing the URCap the robot will expose its tool communication device to the network.

## Setup the ROS side
In order to use the tool communication in ROS, simply pass the correct parameters to the bringup
launch files:

```bash
$ roslaunch ur_robot_driver ur<3|5|10>e_bringup.launch \
  use_tool_communication:=true \
  tool_voltage:=24 \ # can be 0, 12 or 24
  tool_parity:=0 \ # 0: none, 1: odd, 2: even
  tool_baud_rate:=115200 \
  tool_stop_bits:=1 \
  tool_rx_idle_chars:=1.5 \
  tool_tx_idle_chars:=3.5 \
  tool_device_name:=/tmp/ttyUR  # remember that your user needs to have the rights to write that file handle
```

The `tool_device_name` is an arbitrary name for the device file at which the device will be
accessible in the local file system. Most ROS drivers for rs485 devices accept an argument to
specify the device file path. With the example above you could run the `rs485_node` from the package
`imaginary_drivers` using the following command:

```bash
$ rosrun imaginary_drivers rs485_node device:=/tmp/ttyUR

```

You can basically choose any device name, but your user has to have the correct rights to actually
create a new file handle inside this directory. Therefore, we didn't use the `/dev` folder in the
example, as users usually don't have the access rights to create new files there.

For all the other tool parameters seen above, please refer to the Universal Robots user manual.
