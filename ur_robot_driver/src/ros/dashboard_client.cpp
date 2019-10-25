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
 * \date    2019-10-21
 *
 */
//----------------------------------------------------------------------

#include <ur_robot_driver/ros/dashboard_client.h>

namespace ur_driver
{
DashboardClientROS::DashboardClientROS(const ros::NodeHandle& nh, const std::string& robot_ip)
  : nh_(nh), client_(robot_ip)
{
  client_.connect();

  // Service to release the brakes. If the robot is currently powered off, it will get powered on on the fly.
  // brake_release_service_ = create_dashboard_trigger_srv("brake_release", "brake release\n", "Brake releasing");
  brake_release_service_ = create_dashboard_trigger_srv("brake_release", "brake release\n", "Brake releasing");

  // If this service is called the operational mode can again be changed from PolyScope, and the user password is
  // enabled.
  clear_operational_mode_service_ = create_dashboard_trigger_srv("clear_operational_mode", "clear operational mode\n",
                                                                 "No longer controlling the operational mode\\. "
                                                                 "Current "
                                                                 "operational mode: "
                                                                 "'(MANUAL|AUTOMATIC)'\\.");

  // Close a (non-safety) popup on the teach pendant.
  close_popup_service_ = create_dashboard_trigger_srv("close_popup", "close popup\n", "closing popup");

  // Close a safety popup on the teach pendant.
  close_safety_popup_service_ =
      create_dashboard_trigger_srv("close_safety_popup", "close safety popup\n", "closing safety popup");

  // Pause a running program.
  pause_service_ = create_dashboard_trigger_srv("pause", "pause\n", "Pausing program");

  // Start execution of a previously loaded program
  play_service_ = create_dashboard_trigger_srv("play", "play\n", "Starting program");

  // Power off the robot motors
  power_off_service_ = create_dashboard_trigger_srv("power_off", "power off\n", "Powering off");

  // Power on the robot motors. To fully start the robot, call 'brake_release' afterwards.
  power_on_service_ = create_dashboard_trigger_srv("power_on", "power on\n", "Powering on");

  // Disconnect from the dashboard service. Currently, there's no way of reconnecting.
  quit_service_ = create_dashboard_trigger_srv("quit", "quit\n", "Disconnected");

  // Used when robot gets a safety fault or violation to restart the safety. After safety has been rebooted the robot
  // will be in Power Off. NOTE: You should always ensure it is okay to restart the system. It is highly recommended to
  // check the error log before using this command (either via PolyScope or e.g. ssh connection).
  restart_safety_service_ = create_dashboard_trigger_srv("restart_safety", "restart safety\n", "Restarting safety");

  // Shutdown the robot controller
  shutdown_service_ = create_dashboard_trigger_srv("shutdown", "shutdown\n", "Shutting down");

  // Stop program execution on the robot
  stop_service_ = create_dashboard_trigger_srv("stop", "stop\n", "Stopped");

  // Dismiss a protective stop to continue robot movements. NOTE: It is the responsibility of the user to ensure the
  // cause of the protective stop is resolved before calling this service.
  unlock_protective_stop_service_ =
      create_dashboard_trigger_srv("unlock_protective_stop", "unlock protective stop\n", "Protective stop releasing");
}

}  // namespace ur_driver
