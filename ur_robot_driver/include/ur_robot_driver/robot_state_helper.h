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
 * \date    2019-11-04
 *
 */
//----------------------------------------------------------------------
#ifndef UR_ROBOT_DRIVER_ROS_ROBOT_STATE_HELPER_INCLUDED
#define UR_ROBOT_DRIVER_ROS_ROBOT_STATE_HELPER_INCLUDED
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <ur_client_library/ur/datatypes.h>
#include <ur_dashboard_msgs/RobotMode.h>
#include <ur_dashboard_msgs/SafetyMode.h>
#include <ur_dashboard_msgs/SetModeAction.h>

namespace ur_driver
{
/*!
 * \brief A small helper class around the robot state (constisting of 'robot_mode' and
 * 'safety_mode')
 *
 * This class logs any changes of the robot state and provides an action interface bring the robot
 * into a required state. For modifying the robot mode it uses the dashboard server interface
 * provided by the ur_robot_driver.
 */
class RobotStateHelper
{
public:
  /*!
   * \brief Constructor that should be used by default
   *
   * \param nh Node handle that should be used. The handle's namespace should be the same as used
   * for the hardware interface. Otherwise remapping will be necessary to access the hardware
   * interface's topics and sercices.
   */
  RobotStateHelper(const ros::NodeHandle& nh);
  RobotStateHelper() = delete;
  virtual ~RobotStateHelper() = default;

private:
  ros::NodeHandle nh_;
  void robotModeCallback(const ur_dashboard_msgs::RobotMode& msg);
  void safetyModeCallback(const ur_dashboard_msgs::SafetyMode& msg);

  /*!
   * \brief Updates action feedback and triggers next transition if necessary
   */
  void updateRobotState();

  /*!
   * \brief Performs the transition required by the current mode to get to the next mode.
   */
  void doTransition();

  /*!
   * \brief Small wrapper function to call a trigger service. The trigger's response message will be
   * loged to INFO output.
   *
   * \param srv Pointer to service client that shall be used.
   *
   * \returns service response's success field.
   */
  bool safeDashboardTrigger(ros::ServiceClient* srv);

  // Action server functions
  void setModeGoalCallback();
  void setModePreemptCallback();
  void startActionServer();
  bool is_started_;

  urcl::RobotMode robot_mode_;
  urcl::SafetyMode safety_mode_;

  ros::Subscriber robot_mode_sub_;
  ros::Subscriber safety_mode_sub_;
  ros::ServiceClient unlock_protective_stop_srv_;
  ros::ServiceClient restart_safety_srv_;
  ros::ServiceClient power_on_srv_;
  ros::ServiceClient power_off_srv_;
  ros::ServiceClient brake_release_srv_;
  ros::ServiceClient stop_program_srv_;
  ros::ServiceClient play_program_srv_;
  actionlib::SimpleActionServer<ur_dashboard_msgs::SetModeAction> set_mode_as_;

  ur_dashboard_msgs::SetModeGoalConstPtr goal_;
  ur_dashboard_msgs::SetModeFeedback feedback_;
  ur_dashboard_msgs::SetModeResult result_;
};
}  // namespace ur_driver

#endif  // ifndef UR_ROBOT_DRIVER_ROS_ROBOT_STATE_HELPER_INCLUDED
