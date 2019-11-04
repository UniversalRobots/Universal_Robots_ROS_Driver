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
 * \author  Felix Mauch mauch@fzi.de
 * \date    2019-11-04
 *
 */
//----------------------------------------------------------------------

#include <ur_robot_driver/ros/robot_state_helper.h>
namespace ur_driver
{
RobotStateHelper::RobotStateHelper(const ros::NodeHandle& nh) : nh_(nh)
{
  robot_mode_sub_ = nh_.subscribe("robot_mode", 1, &RobotStateHelper::robotModeCallback, this);
  safety_mode_sub_ = nh_.subscribe("safety_mode", 1, &RobotStateHelper::safetyModeCallback, this);
}

void RobotStateHelper::robotModeCallback(const ur_dashboard_msgs::RobotMode& msg)
{
  robot_mode_ = RobotMode(msg.mode);
  ROS_INFO_STREAM("Robot mode is now " << robotModeString(robot_mode_));
}

void RobotStateHelper::safetyModeCallback(const ur_dashboard_msgs::SafetyMode& msg)
{
  safety_mode_ = SafetyMode(msg.mode);
  ROS_INFO_STREAM("Robot's safety mode is now " << safetyModeString(safety_mode_));
}
}  // namespace ur_driver
