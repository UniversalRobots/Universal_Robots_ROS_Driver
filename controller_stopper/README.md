# controller_stopper

A small helper node that stops and restarts ROS controllers based on a boolean status topic. When the status goes to `false`, all running controllers except a set of predefined *consistent_controllers* get stopped. If status returns to `true` the stopped controllers are restarted.

## Nodes

### controller_stopper_node



#### Parameters

##### consistent_controllers (default: ["joint_state_controller"])

Consistent controllers will not be stopped when the robot stops. Defaults to ["joint_state_controller"]

#### Service Clients

##### controller_manager/list_controllers ([controller_manager_msgs/ListControllers](http://docs.ros.org/api/controller_manager_msgs/html/srv/ListControllers.html))

Controller manager service to list controllers

##### controller_manager/switch_controller ([controller_manager_msgs/SwitchController](http://docs.ros.org/api/controller_manager_msgs/html/srv/SwitchController.html))

Controller manager service to switch controllers

#### Subscribed topics

##### robot_running ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))

Subscribes to a robot's running state topic. Ideally this topic is latched and only publishes on changes. However, this node only reacts on state changes, so a state published each cycle would also be fine.

