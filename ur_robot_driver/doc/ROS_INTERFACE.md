# ur_robot_driver

The new driver for Universal Robots UR3, UR5 and UR10 robots with CB3 controllers and the e-series.

## Launchfiles

### ur3e_bringup.launch

Standalone launchfile to startup a ur3e. This requires a robot reachable via a network connection.

#### Arguments

##### controller_config_file (default: "$(find ur_robot_driver)/config/ur3e_controllers.yaml")

Config file used for defining the ROS-Control controllers.

##### controllers (default: "joint_state_controller scaled_pos_joint_traj_controller speed_scaling_state_controller force_torque_sensor_controller")

Controllers that are activated by default.

##### debug (default: "false")

Debug flag that will get passed on to ur_common.launch

##### headless_mode (default: "false")

Automatically send URScript to robot to execute. On e-Series this does require the robot to be in 'remote-control' mode. With this, the URCap is not needed on the robot.

##### kinematics_config (default: "$(find ur_e_description)/config/ur3e_default.yaml")

Kinematics config file used for calibration correction. This will be used to verify the robot's calibration is matching the robot_description.

##### limited (default: "false")

Use the description in limited mode (Every axis rotates from -PI to PI)

##### reverse_port (default: "50001")

Port that will be opened by the driver to allow direct communication between the driver and the robot controller.

##### robot_description_file (default: "$(find ur_e_description)/launch/ur3e_upload.launch")

Robot description launch file.

##### robot_ip (Required)

IP address by which the robot can be reached.

##### script_sender_port (default: "50002")

The driver will offer an interface to receive the program's URScript on this port. If the robot cannot connect to this port, `External Control` will stop immediately.

##### stopped_controllers (default: "pos_joint_traj_controller joint_group_vel_controller")

Controllers that are initally loaded, but not started.

##### tf_prefix (default: "")

tf_prefix used for the robot.

##### wrench_frame_id (default: "wrench")

Parameter to set the id of the wrench frame, required if using multiple robots

##### tool_baud_rate (default: "115200")

Baud rate used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_device_name (default: "/tmp/ttyUR")

Local device name used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_parity (default: "0")

Parity configuration used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_rx_idle_chars (default: "1.5")

Number of idle chars in RX channel used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_stop_bits (default: "1")

Number of stop bits used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_tcp_port (default: "54321")

Port on which the robot controller publishes the tool comm interface. Only used, when `use_tool_communication` is set to true.

##### tool_tx_idle_chars (default: "3.5")

Number of idle chars in TX channel used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_voltage (default: "0")

Tool voltage set at the beginning of the UR program. Only used, when `use_tool_communication` is set to true.

##### use_tool_communication (default: "false")

On e-Series robots tool communication can be enabled with this argument

### ur10_bringup.launch

Standalone launchfile to startup a ur10 robot. This requires a robot reachable via a network connection.

#### Arguments

##### controller_config_file (default: "$(find ur_robot_driver)/config/ur10_controllers.yaml")

Config file used for defining the ROS-Control controllers.

##### controllers (default: "joint_state_controller scaled_pos_joint_traj_controller speed_scaling_state_controller force_torque_sensor_controller")

Controllers that are activated by default.

##### debug (default: "false")

Debug flag that will get passed on to ur_common.launch

##### headless_mode (default: "false")

Automatically send URScript to robot to execute. On e-Series this does require the robot to be in 'remote-control' mode. With this, the URCap is not needed on the robot.

##### kinematics_config (default: "$(find ur_description)/config/ur10_default.yaml")

Kinematics config file used for calibration correction. This will be used to verify the robot's calibration is matching the robot_description.

##### limited (default: "false")

Use the description in limited mode (Every axis rotates from -PI to PI)

##### reverse_port (default: "50001")

Port that will be opened by the driver to allow direct communication between the driver and the robot controller.

##### robot_description_file (default: "$(find ur_description)/launch/ur10_upload.launch")

Robot description launch file.

##### robot_ip (Required)

IP address by which the robot can be reached.

##### script_sender_port (default: "50002")

The driver will offer an interface to receive the program's URScript on this port. If the robot cannot connect to this port, `External Control` will stop immediately.

##### stopped_controllers (default: "pos_joint_traj_controller joint_group_vel_controller")

Controllers that are initally loaded, but not started.

##### tf_prefix (default: "")

tf_prefix used for the robot.

### ur_control.launch

Robot bringup launchfile without the robot description. Include this, if you want to include robot control into a larger launchfile structure.

#### Arguments

##### controller_config_file (Required)

Config file used for defining the ROS-Control controllers.

##### controllers (default: "joint_state_controller vel_based_pos_joint_traj_controller force_torque_sensor_controller")

Controllers that are activated by default.

##### debug (default: "false")

If set to true, will start the driver inside gdb

##### headless_mode (default: "false")

Automatically send URScript to robot to execute. On e-Series this does require the robot to be in 'remote-control' mode. With this, the URCap is not needed on the robot.

##### kinematics_config (Required)

Kinematics config file used for calibration correction. This will be used to verify the robot's calibration is matching the robot_description. Pass the same config file that is passed to the robot_description.

##### launch_prefix (Required)

Please add description. See file "launch/ur_control.launch".

##### reverse_port (default: "50001")

Port that will be opened by the driver to allow direct communication between the driver and the robot controller.

##### robot_ip (Required)

IP address by which the robot can be reached.

##### rtde_input_recipe_file (default: "$(find ur_robot_driver)/resources/rtde_input_recipe.txt")

Recipe file used for the RTDE-inputs. Only change this if you know what you're doing.

##### rtde_output_recipe_file (default: "$(find ur_robot_driver)/resources/rtde_output_recipe.txt")

Recipe file used for the RTDE-outputs. Only change this if you know what you're doing.

##### script_sender_port (default: "50002")

The driver will offer an interface to receive the program's URScript on this port. If the robot cannot connect to this port, `External Control` will stop immediately.

##### stopped_controllers (default: "joint_group_vel_controller")

Controllers that are initally loaded, but not started.

##### tf_prefix (default: "")

tf_prefix used for the robot.

##### tool_baud_rate (default: "115200")

Baud rate used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_device_name (default: "/tmp/ttyUR")

Local device name used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_parity (default: "0")

Parity configuration used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_rx_idle_chars (default: "1.5")

Number of idle chars in RX channel used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_stop_bits (default: "1")

Number of stop bits used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_tcp_port (default: "54321")

Port on which the robot controller publishes the tool comm interface. Only used, when `use_tool_communication` is set to true.

##### tool_tx_idle_chars (default: "3.5")

Number of idle chars in TX channel used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_voltage (default: "0")

Tool voltage set at the beginning of the UR program. Only used, when `use_tool_communication` is set to true.

##### urscript_file (default: "$(find ur_robot_driver)/resources/ros_control.urscript")

Path to URScript that will be sent to the robot and that forms the main control program.

##### use_tool_communication (Required)

On e-Series robots tool communication can be enabled with this argument

### ur_common.launch

Launchfile that starts a robot description with robot_state publisher and the driver for a given robot. It is recommended to use the individual launch files instead such as `ur10_bringup.launch`. Additionally, this launchfile can be used as a template to include this driver into a larger launch file structure.

#### Arguments

##### controller_config_file (Required)

Config file used for defining the ROS-Control controllers.

##### controllers (default: "joint_state_controller scaled_pos_joint_traj_controller speed_scaling_state_controller force_torque_sensor_controller")

Controllers that are activated by default.

##### debug (default: "false")

Debug flag that will get passed on to ur_control.launch

##### headless_mode (default: "false")

Automatically send URScript to robot to execute. On e-Series this does require the robot to be in 'remote-control' mode. With this, the URCap is not needed on the robot.

##### kinematics_config (Required)

Kinematics config file used for calibration correction. This will be used to verify the robot's calibration is matching the robot_description.

##### limited (default: "false")

Use the description in limited mode (Every axis rotates from -PI to PI)

##### reverse_port (default: "50001")

Port that will be opened by the driver to allow direct communication between the driver and the robot controller.

##### robot_description_file (Required)

Robot description launch file.

##### robot_ip (Required)

IP address by which the robot can be reached.

##### script_sender_port (default: "50002")

The driver will offer an interface to receive the program's URScript on this port. If the robot cannot connect to this port, `External Control` will stop immediately.

##### stopped_controllers (default: "pos_joint_traj_controller joint_group_vel_controller")

Controllers that are initally loaded, but not started.

##### tf_prefix (default: "")

tf_prefix used for the robot.

##### tool_baud_rate (default: "115200")

Baud rate used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_device_name (default: "/tmp/ttyUR")

Local device name used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_parity (default: "0")

Parity configuration used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_rx_idle_chars (default: "1.5")

Number of idle chars in RX channel used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_stop_bits (default: "1")

Number of stop bits used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_tcp_port (default: "54321")

Port on which the robot controller publishes the tool comm interface. Only used, when `use_tool_communication` is set to true.

##### tool_tx_idle_chars (default: "3.5")

Number of idle chars in TX channel used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_voltage (default: "0")

Tool voltage set at the beginning of the UR program. Only used, when `use_tool_communication` is set to true.

##### use_tool_communication (Required)

On e-Series robots tool communication can be enabled with this argument

### ur5_bringup.launch

Standalone launchfile to startup a ur5 robot. This requires a robot reachable via a network connection.

#### Arguments

##### controller_config_file (default: "$(find ur_robot_driver)/config/ur5_controllers.yaml")

Config file used for defining the ROS-Control controllers.

##### controllers (default: "joint_state_controller scaled_pos_joint_traj_controller speed_scaling_state_controller force_torque_sensor_controller")

Controllers that are activated by default.

##### debug (default: "false")

Debug flag that will get passed on to ur_common.launch

##### headless_mode (default: "false")

Automatically send URScript to robot to execute. On e-Series this does require the robot to be in 'remote-control' mode. With this, the URCap is not needed on the robot.

##### kinematics_config (default: "$(find ur_description)/config/ur5_default.yaml")

Kinematics config file used for calibration correction. This will be used to verify the robot's calibration is matching the robot_description.

##### limited (default: "false")

Use the description in limited mode (Every axis rotates from -PI to PI)

##### reverse_port (default: "50001")

Port that will be opened by the driver to allow direct communication between the driver and the robot controller.

##### robot_description_file (default: "$(find ur_description)/launch/ur5_upload.launch")

Robot description launch file.

##### robot_ip (Required)

IP address by which the robot can be reached.

##### script_sender_port (default: "50002")

The driver will offer an interface to receive the program's URScript on this port. If the robot cannot connect to this port, `External Control` will stop immediately.

##### stopped_controllers (default: "pos_joint_traj_controller joint_group_vel_controller")

Controllers that are initally loaded, but not started.

##### tf_prefix (default: "")

tf_prefix used for the robot.

### ur5e_bringup.launch

Standalone launchfile to startup a ur5e robot. This requires a robot reachable via a network connection.

#### Arguments

##### controller_config_file (default: "$(find ur_robot_driver)/config/ur5e_controllers.yaml")

Config file used for defining the ROS-Control controllers.

##### controllers (default: "joint_state_controller scaled_pos_joint_traj_controller speed_scaling_state_controller force_torque_sensor_controller")

Controllers that are activated by default.

##### debug (default: "false")

Debug flag that will get passed on to ur_common.launch

##### headless_mode (default: "false")

Automatically send URScript to robot to execute. On e-Series this does require the robot to be in 'remote-control' mode. With this, the URCap is not needed on the robot.

##### kinematics_config (default: "$(find ur_e_description)/config/ur5e_default.yaml")

Kinematics config file used for calibration correction. This will be used to verify the robot's calibration is matching the robot_description.

##### limited (default: "false")

Use the description in limited mode (Every axis rotates from -PI to PI)

##### reverse_port (default: "50001")

Port that will be opened by the driver to allow direct communication between the driver and the robot controller.

##### robot_description_file (default: "$(find ur_e_description)/launch/ur5e_upload.launch")

Robot description launch file.

##### robot_ip (Required)

IP address by which the robot can be reached.

##### script_sender_port (default: "50002")

The driver will offer an interface to receive the program's URScript on this port. If the robot cannot connect to this port, `External Control` will stop immediately.

##### stopped_controllers (default: "pos_joint_traj_controller joint_group_vel_controller")

Controllers that are initally loaded, but not started.

##### tf_prefix (default: "")

tf_prefix used for the robot.

##### tool_baud_rate (default: "115200")

Baud rate used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_device_name (default: "/tmp/ttyUR")

Local device name used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_parity (default: "0")

Parity configuration used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_rx_idle_chars (default: "1.5")

Number of idle chars in RX channel used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_stop_bits (default: "1")

Number of stop bits used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_tcp_port (default: "54321")

Port on which the robot controller publishes the tool comm interface. Only used, when `use_tool_communication` is set to true.

##### tool_tx_idle_chars (default: "3.5")

Number of idle chars in TX channel used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_voltage (default: "0")

Tool voltage set at the beginning of the UR program. Only used, when `use_tool_communication` is set to true.

##### use_tool_communication (default: "false")

On e-Series robots tool communication can be enabled with this argument

### ur3_bringup.launch

Standalone launchfile to startup a ur3 robot. This requires a robot reachable via a network connection.

#### Arguments

##### controller_config_file (default: "$(find ur_robot_driver)/config/ur3_controllers.yaml")

Config file used for defining the ROS-Control controllers.

##### controllers (default: "joint_state_controller scaled_pos_joint_traj_controller speed_scaling_state_controller force_torque_sensor_controller")

Controllers that are activated by default.

##### debug (default: "false")

Debug flag that will get passed on to ur_common.launch

##### headless_mode (default: "false")

Automatically send URScript to robot to execute. On e-Series this does require the robot to be in 'remote-control' mode. With this, the URCap is not needed on the robot.

##### kinematics_config (default: "$(find ur_description)/config/ur3_default.yaml")

Kinematics config file used for calibration correction. This will be used to verify the robot's calibration is matching the robot_description.

##### limited (default: "false")

Use the description in limited mode (Every axis rotates from -PI to PI)

##### reverse_port (default: "50001")

Port that will be opened by the driver to allow direct communication between the driver and the robot controller.

##### robot_description_file (default: "$(find ur_description)/launch/ur3_upload.launch")

Robot description launch file.

##### robot_ip (Required)

IP address by which the robot can be reached.

##### script_sender_port (default: "50002")

The driver will offer an interface to receive the program's URScript on this port. If the robot cannot connect to this port, `External Control` will stop immediately.

##### stopped_controllers (default: "pos_joint_traj_controller joint_group_vel_controller")

Controllers that are initally loaded, but not started.

##### tf_prefix (default: "")

tf_prefix used for the robot.

### ur10e_bringup.launch

Standalone launchfile to startup a ur10e robot. This requires a robot reachable via a network connection.

#### Arguments

##### controller_config_file (default: "$(find ur_robot_driver)/config/ur10e_controllers.yaml")

Config file used for defining the ROS-Control controllers.

##### controllers (default: "joint_state_controller scaled_pos_joint_traj_controller speed_scaling_state_controller force_torque_sensor_controller")

Controllers that are activated by default.

##### debug (default: "false")

Debug flag that will get passed on to ur_common.launch

##### headless_mode (default: "false")

Automatically send URScript to robot to execute. On e-Series this does require the robot to be in 'remote-control' mode. With this, the URCap is not needed on the robot.

##### kinematics_config (default: "$(find ur_e_description)/config/ur10e_default.yaml")

Kinematics config file used for calibration correction. This will be used to verify the robot's calibration is matching the robot_description.

##### limited (default: "false")

Use the description in limited mode (Every axis rotates from -PI to PI)

##### reverse_port (default: "50001")

Port that will be opened by the driver to allow direct communication between the driver and the robot controller.

##### robot_description_file (default: "$(find ur_e_description)/launch/ur10e_upload.launch")

Robot description launch file.

##### robot_ip (Required)

IP address by which the robot can be reached.

##### script_sender_port (default: "50002")

The driver will offer an interface to receive the program's URScript on this port. If the robot cannot connect to this port, `External Control` will stop immediately.

##### stopped_controllers (default: "pos_joint_traj_controller joint_group_vel_controller")

Controllers that are initally loaded, but not started.

##### tf_prefix (default: "")

tf_prefix used for the robot.

##### tool_baud_rate (default: "115200")

Baud rate used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_device_name (default: "/tmp/ttyUR")

Local device name used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_parity (default: "0")

Parity configuration used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_rx_idle_chars (default: "1.5")

Number of idle chars in RX channel used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_stop_bits (default: "1")

Number of stop bits used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_tcp_port (default: "54321")

Port on which the robot controller publishes the tool comm interface. Only used, when `use_tool_communication` is set to true.

##### tool_tx_idle_chars (default: "3.5")

Number of idle chars in TX channel used for tool communication. Only used, when `use_tool_communication` is set to true.

##### tool_voltage (default: "0")

Tool voltage set at the beginning of the UR program. Only used, when `use_tool_communication` is set to true.

##### use_tool_communication (default: "false")

On e-Series robots tool communication can be enabled with this argument

## Nodes

### ur_robot_driver_node

This is the actual driver node containing the ROS-Control stack. Interfaces documented here refer to the robot's hardware interface. Controller-specific API elements might be present for the individual controllers outside of this package.

#### Advertised Services

##### dashboard/add_to_log ([ur_dashboard_msgs/AddToLog](http://docs.ros.org/api/ur_dashboard_msgs/html/srv/AddToLog.html))

Service to add a message to the robot's log

##### dashboard/brake_release ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Service to release the brakes. If the robot is currently powered off, it will get powered on on the fly.

##### dashboard/clear_operational_mode ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

If this service is called the operational mode can again be changed from PolyScope, and the user password is enabled.

##### dashboard/close_popup ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Close a (non-safety) popup on the teach pendant.

##### dashboard/close_safety_popup ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Close a safety popup on the teach pendant.

##### dashboard/connect ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Service to reconnect to the dashboard server

##### dashboard/get_loaded_program ([ur_dashboard_msgs/GetLoadedProgram](http://docs.ros.org/api/ur_dashboard_msgs/html/srv/GetLoadedProgram.html))

Load a robot installation from a file

##### dashboard/get_robot_mode ([ur_dashboard_msgs/GetRobotMode](http://docs.ros.org/api/ur_dashboard_msgs/html/srv/GetRobotMode.html))

Service to query the current robot mode

##### dashboard/get_safety_mode ([ur_dashboard_msgs/GetSafetyMode](http://docs.ros.org/api/ur_dashboard_msgs/html/srv/GetSafetyMode.html))

Service to query the current safety mode

##### dashboard/load_installation ([ur_dashboard_msgs/Load](http://docs.ros.org/api/ur_dashboard_msgs/html/srv/Load.html))

Load a robot installation from a file

##### dashboard/load_program ([ur_dashboard_msgs/Load](http://docs.ros.org/api/ur_dashboard_msgs/html/srv/Load.html))

Load a robot program from a file

##### dashboard/pause ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Pause a running program.

##### dashboard/play ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Start execution of a previously loaded program

##### dashboard/popup ([ur_dashboard_msgs/Popup](http://docs.ros.org/api/ur_dashboard_msgs/html/srv/Popup.html))

Service to show a popup on the UR Teach pendant.

##### dashboard/power_off ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Power off the robot motors

##### dashboard/power_on ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Power on the robot motors. To fully start the robot, call 'brake_release' afterwards.

##### dashboard/program_running ([ur_dashboard_msgs/IsProgramRunning](http://docs.ros.org/api/ur_dashboard_msgs/html/srv/IsProgramRunning.html))

Query whether there is currently a program running

##### dashboard/program_saved ([ur_dashboard_msgs/IsProgramSaved](http://docs.ros.org/api/ur_dashboard_msgs/html/srv/IsProgramSaved.html))

Query whether the current program is saved

##### dashboard/program_state ([ur_dashboard_msgs/GetProgramState](http://docs.ros.org/api/ur_dashboard_msgs/html/srv/GetProgramState.html))

Service to query the current program state

##### dashboard/quit ([ur_dashboard_msgs/GetLoadedProgram](http://docs.ros.org/api/ur_dashboard_msgs/html/srv/GetLoadedProgram.html))

Disconnect from the dashboard service.

##### dashboard/raw_request ([ur_dashboard_msgs/RawRequest](http://docs.ros.org/api/ur_dashboard_msgs/html/srv/RawRequest.html))

General purpose service to send arbitrary messages to the dashboard server

##### dashboard/restart_safety ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Used when robot gets a safety fault or violation to restart the safety. After safety has been rebooted the robot will be in Power Off. NOTE: You should always ensure it is okay to restart the system. It is highly recommended to check the error log before using this command (either via PolyScope or e.g. ssh connection).

##### dashboard/shutdown ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Shutdown the robot controller

##### dashboard/stop ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Stop program execution on the robot

##### dashboard/unlock_protective_stop ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Dismiss a protective stop to continue robot movements. NOTE: It is the responsibility of the user to ensure the cause of the protective stop is resolved before calling this service.

##### hand_back_control ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Calling this service will make the "External Control" program node on the UR-Program return.

##### resend_robot_program ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

When in headless mode, this sends the URScript program to the robot for execution. Use this after the program has been interrupted, e.g. by a protective- or EM-stop.

##### set_io (ur_msgs/SetIO)

Service to set any of the robot's IOs

##### set_speed_slider (ur_msgs/SetSpeedSliderFraction)

Set the speed slider fraction used by the robot's execution. Values should be between 0 and 1. Only set this smaller than 1 if you are using the scaled controllers (as by default) or you know what you're doing. Using this with other controllers might lead to unexpected behaviors.
 
#### set_payload (ur_msgs/SetPayload)

Setup the mounted payload through a ROS service

##### zero_ftsensor ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Calling this service will zero the robot's ftsensor. Note: On e-Series robots this will only work when the robot is in remote-control mode.

#### Parameters

##### dashboard/receive_timeout (Required)

Timeout after which a call to the dashboard server will be considered failure if no answer has been received.

##### hardware_interface/joints (Required)

Names of the joints. Usually, this is given in the controller config file.

##### headless_mode (Required)

Start robot in headless mode. This does not require the 'External Control' URCap to be running on the robot, but this will send the URScript to the robot directly. On e-Series robots this requires the robot to run in 'remote-control' mode.

##### input_recipe_file (Required)

Path to the file containing the recipe used for requesting RTDE inputs.

##### kinematics/hash (Required)

Hash of the calibration reported by the robot. This is used for validating the robot description is using the correct calibration. If the robot's calibration doesn't match this hash, an error will be printed. You can use the robot as usual, however Cartesian poses of the endeffector might be inaccurate. See the "ur_calibration" package on help how to generate your own hash matching your actual robot.

##### non_blocking_read (default: "false")

Enables non_blocking_read mode. Should only be used with combined_robot_hw. Disables error generated when read returns without any data, sets the read timeout to zero, and synchronises read/write operations. Enabling this when not used with combined_robot_hw can suppress important errors and affect real-time performance.

##### output_recipe_file (Required)

Path to the file containing the recipe used for requesting RTDE outputs.

##### reverse_port (Required)

Port that will be opened to communicate between the driver and the robot controller.

##### robot_ip (Required)

The robot's IP address.

##### script_file (Required)

Path to the urscript code that will be sent to the robot.

##### script_sender_port (Required)

The driver will offer an interface to receive the program's URScript on this port.

##### servoj_gain (Required)

Specify gain for servoing to position in joint space. A higher gain can sharpen the trajectory.

##### servoj_lookahead_time (Required)

Specify lookahead time for servoing to position in joint space. A longer lookahead time can smooth the trajectory.

##### tf_prefix (default: "")

When the robot's URDF is being loaded with a prefix, we need to know it here, as well, in order to publish correct frame names for frames reported by the robot directly.


##### tool_baud_rate (Required)

Baud rate used for tool communication. Will be set as soon as the UR-Program on the robot is started. See UR documentation for valid baud rates.  Note: This parameter is only evaluated, when the parameter "use_tool_communication" is set to TRUE.  Then, this parameter is required.

##### tool_parity (Required)

Parity used for tool communication. Will be set as soon as the UR-Program on the robot is started. Can be 0 (None), 1 (odd) and 2 (even).  Note: This parameter is only evaluated, when the parameter "use_tool_communication" is set to TRUE.  Then, this parameter is required.

##### tool_rx_idle_chars (Required)

Number of idle chars for the RX unit used for tool communication. Will be set as soon as the UR-Program on the robot is started. Valid values: min=1.0, max=40.0  Note: This parameter is only evaluated, when the parameter "use_tool_communication" is set to TRUE.  Then, this parameter is required.

##### tool_stop_bits (Required)

Number of stop bits used for tool communication. Will be set as soon as the UR-Program on the robot is started. Can be 1 or 2.  Note: This parameter is only evaluated, when the parameter "use_tool_communication" is set to TRUE.  Then, this parameter is required.

##### tool_tx_idle_chars (Required)

Number of idle chars for the TX unit used for tool communication. Will be set as soon as the UR-Program on the robot is started. Valid values: min=0.0, max=40.0  Note: This parameter is only evaluated, when the parameter "use_tool_communication" is set to TRUE.  Then, this parameter is required.

##### tool_voltage (Required)

Tool voltage that will be set as soon as the UR-Program on the robot is started. Note: This parameter is only evaluated, when the parameter "use_tool_communication" is set to TRUE. Then, this parameter is required.

##### use_tool_communication (Required)

Should the tool's RS485 interface be forwarded to the ROS machine? This is only available on e-Series models. Setting this parameter to TRUE requires multiple other parameters to be set,as well.

#### Published topics

##### robot_program_running ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))

Whenever the runtime state of the "External Control" program node in the UR-program changes, a message gets published here. So this is equivalent to the information whether the robot accepts commands from ROS side.

#### Subscribed topics

##### script_command ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))

Send arbitrary script commands to this topic. Note: On e-Series the robot has to be in remote-control mode.  Sending scripts to this will stop program execution unless wrapped in a secondary program:  sec myProgram(): set_digital_out(0, True) end

### dashboard_client



#### Advertised Services

##### add_to_log ([ur_dashboard_msgs/AddToLog](http://docs.ros.org/api/ur_dashboard_msgs/html/srv/AddToLog.html))

Service to add a message to the robot's log

##### brake_release ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Service to release the brakes. If the robot is currently powered off, it will get powered on on the fly.

##### clear_operational_mode ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

If this service is called the operational mode can again be changed from PolyScope, and the user password is enabled.

##### close_popup ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Close a (non-safety) popup on the teach pendant.

##### close_safety_popup ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Close a safety popup on the teach pendant.

##### connect ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Service to reconnect to the dashboard server

##### get_loaded_program ([ur_dashboard_msgs/GetLoadedProgram](http://docs.ros.org/api/ur_dashboard_msgs/html/srv/GetLoadedProgram.html))

Load a robot installation from a file

##### get_robot_mode ([ur_dashboard_msgs/GetRobotMode](http://docs.ros.org/api/ur_dashboard_msgs/html/srv/GetRobotMode.html))

Service to query the current robot mode

##### get_safety_mode ([ur_dashboard_msgs/GetSafetyMode](http://docs.ros.org/api/ur_dashboard_msgs/html/srv/GetSafetyMode.html))

Service to query the current safety mode

##### load_installation ([ur_dashboard_msgs/Load](http://docs.ros.org/api/ur_dashboard_msgs/html/srv/Load.html))

Load a robot installation from a file

##### load_program ([ur_dashboard_msgs/Load](http://docs.ros.org/api/ur_dashboard_msgs/html/srv/Load.html))

Load a robot program from a file

##### pause ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Pause a running program.

##### play ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Start execution of a previously loaded program

##### popup ([ur_dashboard_msgs/Popup](http://docs.ros.org/api/ur_dashboard_msgs/html/srv/Popup.html))

Service to show a popup on the UR Teach pendant.

##### power_off ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Power off the robot motors

##### power_on ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Power on the robot motors. To fully start the robot, call 'brake_release' afterwards.

##### program_running ([ur_dashboard_msgs/IsProgramRunning](http://docs.ros.org/api/ur_dashboard_msgs/html/srv/IsProgramRunning.html))

Query whether there is currently a program running

##### program_saved ([ur_dashboard_msgs/IsProgramSaved](http://docs.ros.org/api/ur_dashboard_msgs/html/srv/IsProgramSaved.html))

Query whether the current program is saved

##### program_state ([ur_dashboard_msgs/GetProgramState](http://docs.ros.org/api/ur_dashboard_msgs/html/srv/GetProgramState.html))

Service to query the current program state

##### quit ([ur_dashboard_msgs/GetLoadedProgram](http://docs.ros.org/api/ur_dashboard_msgs/html/srv/GetLoadedProgram.html))

Disconnect from the dashboard service.

##### raw_request ([ur_dashboard_msgs/RawRequest](http://docs.ros.org/api/ur_dashboard_msgs/html/srv/RawRequest.html))

General purpose service to send arbitrary messages to the dashboard server

##### restart_safety ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Used when robot gets a safety fault or violation to restart the safety. After safety has been rebooted the robot will be in Power Off. NOTE: You should always ensure it is okay to restart the system. It is highly recommended to check the error log before using this command (either via PolyScope or e.g. ssh connection).

##### shutdown ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Shutdown the robot controller

##### stop ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Stop program execution on the robot

##### unlock_protective_stop ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Dismiss a protective stop to continue robot movements. NOTE: It is the responsibility of the user to ensure the cause of the protective stop is resolved before calling this service.

#### Parameters

##### receive_timeout (Required)

Timeout after which a call to the dashboard server will be considered failure if no answer has been received.

##### robot_ip (Required)

The IP address under which the robot is reachable.

### robot_state_helper

This node prints the robot- and safety mode to ROS logging and offers an action to set the robot to a specific mode (e.g. for initial startup or recovery after a protective stop or EM-Stop).  It should best be started inside the hardware interface's namespace

#### Service Clients

##### dashboard/brake_release ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Service to release the robot's brakes

##### dashboard/play ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Service to start UR program execution on the robot

##### dashboard/power_off ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Service to power off the robot

##### dashboard/power_on ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Service to power on the robot

##### dashboard/restart_safety ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Service to restart safety

##### dashboard/stop ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Service to stop UR program execution on the robot

##### dashboard/unlock_protective_stop ([std_srvs/Trigger](http://docs.ros.org/api/std_srvs/html/srv/Trigger.html))

Service to unlock protective stop

#### Subscribed topics

##### robot_mode ([ur_dashboard_msgs/RobotMode](http://docs.ros.org/api/ur_dashboard_msgs/html/msg/RobotMode.html))

Topic on which the robot_mode is published by the driver

##### safety_mode ([ur_dashboard_msgs/SafetyMode](http://docs.ros.org/api/ur_dashboard_msgs/html/msg/SafetyMode.html))

Topic on which the safety is published by the driver

### tool_communication

This node is used to start the RS485 tunneling interface on the ROS machine. This requires that the RS485 daemon is running on the robot controller and tool communication is enabled on the robot.

#### Parameters

##### ~device_name (Required)

By default, socat will create a pty in /dev/pts/N with n being an increasing number. Additionally, a symlink at the given location will be created. Use an absolute path here.

##### ~robot_ip (Required)

IP address of the robot

