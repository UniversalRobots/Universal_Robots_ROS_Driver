// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
// Created on behalf of Universal Robots A/S
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
#ifndef CONTROLLER_STOPPER_CONTROLLER_STOPPER_H_INCLUDED
#define CONTROLLER_STOPPER_CONTROLLER_STOPPER_H_INCLUDED

#include <ros/ros.h>
#include <std_msgs/Bool.h>

class ControllerStopper
{
public:
  ControllerStopper() = delete;
  ControllerStopper(const ros::NodeHandle& nh);
  virtual ~ControllerStopper() = default;

private:
  void robotRunningCallback(const std_msgs::BoolConstPtr& msg);

  /*!
   * \brief Queries running stoppable controllers.
   *
   * Queries the controller manager for running controllers and compares the result with the
   * consistent_controllers_. The remaining running controllers are stored in stopped_controllers_
   */
  void findStoppableControllers();

  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  ros::Subscriber robot_running_sub_;
  ros::ServiceClient controller_manager_srv_;
  ros::ServiceClient controller_list_srv_;

  std::vector<std::string> consistent_controllers_;
  std::vector<std::string> stopped_controllers_;

  bool robot_running_;
};
#endif  // ifndef CONTROLLER_STOPPER_CONTROLLER_STOPPER_H_INCLUDED
