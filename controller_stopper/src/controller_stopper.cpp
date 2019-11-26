// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-06-12
 *
 */
//----------------------------------------------------------------------

#include <controller_stopper/controller_stopper.h>

#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>

#include <ios>

ControllerStopper::ControllerStopper(const ros::NodeHandle& nh) : nh_(nh), priv_nh_("~"), robot_running_(true)
{
  // Subscribes to a robot's running state topic. Ideally this topic is latched and only publishes
  // on changes. However, this node only reacts on state changes, so a state published each cycle
  // would also be fine.
  robot_running_sub_ = nh_.subscribe("robot_running", 1, &ControllerStopper::robotRunningCallback, this);

  // Controller manager service to switch controllers
  controller_manager_srv_ = nh_.serviceClient<controller_manager_msgs::SwitchController>("controller_manager/"
                                                                                         "switch_controller");
  // Controller manager service to list controllers
  controller_list_srv_ = nh_.serviceClient<controller_manager_msgs::ListControllers>("controller_manager/"
                                                                                     "list_controllers");
  ROS_INFO_STREAM("Waiting for controller manager service to come up on " << nh_.resolveName("controller_manager/"
                                                                                             "switch_controller"));
  controller_manager_srv_.waitForExistence();
  ROS_INFO_STREAM("Service available.");
  ROS_INFO_STREAM("Waiting for controller list service to come up on " << nh_.resolveName("controller_manager/"
                                                                                          "list_controllers"));
  controller_list_srv_.waitForExistence();
  ROS_INFO_STREAM("Service available.");

  // Consistent controllers will not be stopped when the robot stops. Defaults to
  // ["joint_state_controller"]
  if (!priv_nh_.getParam("consistent_controllers", consistent_controllers_))
  {
    consistent_controllers_.push_back("joint_state_controller");
  }

  ROS_DEBUG("Waiting for running controllers");
  // Before we can work properly, we need to know which controllers there are
  while (stopped_controllers_.empty())
  {
    findStoppableControllers();
    ros::Duration(1).sleep();
  }
  ROS_DEBUG("Initialization finished");
}

void ControllerStopper::findStoppableControllers()
{
  controller_manager_msgs::ListControllers list_srv;
  controller_list_srv_.call(list_srv);
  stopped_controllers_.clear();
  for (auto& controller : list_srv.response.controller)
  {
    // Check if in consistent_controllers
    // Else:
    //   Add to stopped_controllers
    if (controller.state == "running")
    {
      auto it = std::find(consistent_controllers_.begin(), consistent_controllers_.end(), controller.name);
      if (it == consistent_controllers_.end())
      {
        stopped_controllers_.push_back(controller.name);
      }
    }
  }
}

void ControllerStopper::robotRunningCallback(const std_msgs::BoolConstPtr& msg)
{
  ROS_DEBUG_STREAM("robotRunningCallback with data " << std::boolalpha << msg->data);
  if (msg->data && !robot_running_)
  {
    ROS_DEBUG_STREAM("Starting controllers");
    controller_manager_msgs::SwitchController srv;
    srv.request.strictness = srv.request.STRICT;
    srv.request.start_controllers = stopped_controllers_;
    if (!controller_manager_srv_.call(srv))
    {
      ROS_ERROR_STREAM("Could not activate requested controllers");
    }
  }
  else if (!msg->data && robot_running_)
  {
    ROS_DEBUG_STREAM("Stopping controllers");
    //   stop all controllers except the once in consistent_controllers_
    findStoppableControllers();
    controller_manager_msgs::SwitchController srv;
    srv.request.strictness = srv.request.STRICT;
    srv.request.stop_controllers = stopped_controllers_;
    if (!controller_manager_srv_.call(srv))
    {
      ROS_ERROR_STREAM("Could not stop requested controllers");
    }
  }
  robot_running_ = msg->data;
}
