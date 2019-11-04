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
#ifndef UR_ROBOT_DRIVER_ROS_ROBOT_STATE_HELPER_INCLUDED
#define UR_ROBOT_DRIVER_ROS_ROBOT_STATE_HELPER_INCLUDED
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <ur_robot_driver/ur/datatypes.h>
#include <ur_dashboard_msgs/RobotMode.h>
#include <ur_dashboard_msgs/SafetyMode.h>
#include <ur_dashboard_msgs/SetModeAction.h>

namespace ur_driver
{
class RobotStateHelper
{
public:
  RobotStateHelper() = delete;
  RobotStateHelper(const ros::NodeHandle& nh);
  virtual ~RobotStateHelper() = default;

private:
  void robotModeCallback(const ur_dashboard_msgs::RobotMode& msg);
  void safetyModeCallback(const ur_dashboard_msgs::SafetyMode& msg);
  void updateRobotState();
  void doTransition();

  void safeDashboardTrigger(ros::ServiceClient* srv);

  void setModeGoalCallback();
  void setModePreemptCallback();

  ros::NodeHandle nh_;
  RobotMode robot_mode_;
  SafetyMode safety_mode_;

  ros::Subscriber robot_mode_sub_;
  ros::Subscriber safety_mode_sub_;
  ros::ServiceClient unlock_protective_stop_srv_;
  ros::ServiceClient restart_safety_srv_;
  ros::ServiceClient power_on_srv_;
  ros::ServiceClient power_off_srv_;
  ros::ServiceClient brake_release_srv_;
  actionlib::SimpleActionServer<ur_dashboard_msgs::SetModeAction> set_mode_as_;

  ur_dashboard_msgs::SetModeGoalConstPtr goal_;
  ur_dashboard_msgs::SetModeFeedback feedback_;
  ur_dashboard_msgs::SetModeResult result_;
};
}  // namespace ur_driver

#endif  // ifndef UR_ROBOT_DRIVER_ROS_ROBOT_STATE_HELPER_INCLUDED
