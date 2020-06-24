# Feature comparison and roadmap

| Feature                                               | ur_modern_driver | this_driver           |
| ---                                                   | ---              | ---                   |
| position-based control                                | yes              | yes                   |
| scaled position-based control                         | -                | yes                   |
| velocity-based control                                | yes              | yes                   |
| reporting of tcp wrench                               | yes              | yes                   |
| reporting of tcp wrench in tcp link                   | -                | yes                   |
| pausing of programs                                   | -                | yes                   |
| continue trajectories after EM-Stop resume            | -                | yes                   |
| continue trajectories after protective stop           | -                | yes                   |
| panel interaction in between possible                 | no<sup>1</sup>   | yes                   |
| get and set IO states                                 | yes              | yes               |
| use tool communication on e-series                    | -                | yes                   |
| use the driver without a teach pendant necessary      | -                | yes               |
| support of CB2 robots                                 | yes              | -                     |
| trajectory extrapolation on robot on missing packages | no<sup>2</sup>   | yes                   |
| use ROS as drop-in for TP-programs                    | -                | yes   |
| extract calibration from robot                        | -                | yes                   |
| send custom script commands to robot                  | yes              | yes               |
| ROS 2 support                                         | ?                | (planned)<sup>3</sup> |
| Reconnect on a disconnected robot                     | yes              | yes           |

<sup>1</sup> Depending on the mode the driver is running the panel won't react or using the panel
will stop the program without notifying the ROS user.

<sup>2</sup> In velocity mode this is implicitly given.

<sup>3</sup> There is no specific plan to do this inside of the first driver development. However,
it is structured in a way so that a ROS2 driver should be developed as easy as possible by keeping
as much as possible in a ros-independent library.


