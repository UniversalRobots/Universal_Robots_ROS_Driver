# Quick feature overview

| Feature                                               | this_driver           |
| ---                                                   | ---                   |
| joint-position-based control                          | yes                   |
| scaled joint-position-based control                   | yes                   |
| joint-velocity-based control                          | yes                   |
| Cartesian position-based control                      | yes                   |
| Cartesian twist-based control                         | yes                   |
| Trajectory forwarding for execution on robot          | yes                   |
| reporting of tcp wrench                               | yes                   |
| pausing of programs                                   | yes                   |
| continue trajectories after EM-Stop resume            | yes                   |
| continue trajectories after protective stop           | yes                   |
| panel interaction in between possible                 | yes                   |
| get and set IO states                                 | yes                   |
| use tool communication on e-series                    | yes<sup>1</sup>       |
| use the driver without a teach pendant necessary      | yes               |
| support of CB1 and CB2 robots                                 | no                     |
| trajectory extrapolation on robot on missing packages | yes                   |
| use ROS as drop-in for TP-programs                    | yes<sup>2</sup>       |
| headless mode                                         | yes   |
| extract calibration from robot                        | yes                   |
| send custom script commands to robot                  | yes                   |
| ROS 2 support                                         | (yes)<sup>3</sup>      |
| Reconnect on a disconnected robot                     | yes           |

<sup>1</sup> Requires URCap (included in
[resources](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master/ur_robot_driver/resources)):
[https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap](https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap)

<sup>2</sup> Requires URCap (included in
[resources](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master/ur_robot_driver/resources)):
[https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap)

<sup>3</sup> The ROS2 driver lives in its own repository:
[https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

