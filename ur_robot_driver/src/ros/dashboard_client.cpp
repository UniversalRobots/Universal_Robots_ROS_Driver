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
  brake_release_service_ = nh_.advertiseService("brake_release", &DashboardClientROS::brakeRelease, this);
  clear_operational_mode_service_ =
      nh_.advertiseService("clear_operational_mode", &DashboardClientROS::clearOperationalMode, this);
  close_popup_service_ = nh_.advertiseService("close_popup", &DashboardClientROS::closePopup, this);
  close_safety_popup_service_ = nh_.advertiseService("close_safety_popup", &DashboardClientROS::closeSafetyPopup, this);
  pause_service_ = nh_.advertiseService("pause", &DashboardClientROS::pause, this);
  play_service_ = nh_.advertiseService("play", &DashboardClientROS::play, this);
  power_off_service_ = nh_.advertiseService("power_off", &DashboardClientROS::powerOff, this);
  power_on_service_ = nh_.advertiseService("power_on", &DashboardClientROS::powerOn, this);
  quit_service_ = nh_.advertiseService("quit", &DashboardClientROS::quit, this);
  restart_safety_service_ = nh_.advertiseService("restart_safety", &DashboardClientROS::restartSafety, this);
  shutdown_service_ = nh_.advertiseService("shutdown", &DashboardClientROS::shutdown, this);
  stop_service_ = nh_.advertiseService("stop", &DashboardClientROS::stop, this);
  unlock_protective_stop_service_ =
      nh_.advertiseService("unlock_protective_stop", &DashboardClientROS::unlockProtectiveStop, this);
}

bool DashboardClientROS::brakeRelease(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  resp.message = client_.sendAndReceive("brake release\n");
  resp.success = std::regex_match(resp.message, std::regex("Brake releasing"));
  return true;
}

bool DashboardClientROS::clearOperationalMode(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  resp.message = client_.sendAndReceive("clear operational mode\n");
  resp.success = std::regex_match(resp.message, std::regex("No longer controlling the operational mode\\. Current operational mode: '(MANUAL|AUTOMATIC)'\\."));
  return true;
}

bool DashboardClientROS::closePopup(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  resp.message = client_.sendAndReceive("close popup\n");
  resp.success = std::regex_match(resp.message, std::regex("closing popup"));
  return true;
}

bool DashboardClientROS::closeSafetyPopup(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  resp.message = client_.sendAndReceive("close safety popup\n");
  resp.success = std::regex_match(resp.message, std::regex("closing safety popup"));
  return true;
}

bool DashboardClientROS::pause(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  resp.message = client_.sendAndReceive("pause\n");
  resp.success = std::regex_match(resp.message, std::regex("Pausing program"));
  return true;
}

bool DashboardClientROS::play(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  resp.message = client_.sendAndReceive("play\n");
  resp.success = std::regex_match(resp.message, std::regex("Starting program"));
  return true;
}

bool DashboardClientROS::powerOff(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  resp.message = client_.sendAndReceive("power off\n");
  resp.success = std::regex_match(resp.message, std::regex("Powering off"));
  return true;
}

bool DashboardClientROS::powerOn(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  resp.message = client_.sendAndReceive("power on\n");
  resp.success = std::regex_match(resp.message, std::regex("Powering on"));
  return true;
}

bool DashboardClientROS::quit(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  resp.message = client_.sendAndReceive("quit\n");
  resp.success = std::regex_match(resp.message, std::regex("Disconnected"));
  return true;
}

bool DashboardClientROS::restartSafety(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  resp.message = client_.sendAndReceive("restart safety\n");
  resp.success = std::regex_match(resp.message, std::regex("Restarting safety"));
  return true;
}

bool DashboardClientROS::shutdown(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  resp.message = client_.sendAndReceive("shutdown\n");
  resp.success = std::regex_match(resp.message, std::regex("Shutting down"));
  return true;
}

bool DashboardClientROS::stop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  resp.message = client_.sendAndReceive("stop\n");
  resp.success = std::regex_match(resp.message, std::regex("Stopped"));
  return true;
}

bool DashboardClientROS::unlockProtectiveStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  resp.message = client_.sendAndReceive("unlock protective stop\n");
  resp.success = std::regex_match(resp.message, std::regex("Protective stop releasing"));
  return true;
}

}  // namespace ur_driver
