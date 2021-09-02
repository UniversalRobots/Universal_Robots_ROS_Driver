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
 * \date    2019-10-21
 *
 */
//----------------------------------------------------------------------

#include <ur_robot_driver/dashboard_client_ros.h>

namespace ur_driver
{
DashboardClientROS::DashboardClientROS(const ros::NodeHandle& nh, const std::string& robot_ip)
  : nh_(nh), client_(robot_ip)
{
  connect();

  // Service to release the brakes. If the robot is currently powered off, it will get powered on on the fly.
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

  // Query whether there is currently a program running
  running_service_ = nh_.advertiseService("program_running", &DashboardClientROS::handleRunningQuery, this);

  // Load a robot installation from a file
  get_loaded_program_service_ =
      nh_.advertiseService<ur_dashboard_msgs::GetLoadedProgram::Request, ur_dashboard_msgs::GetLoadedProgram::Response>(
          "get_loaded_program",
          [&](ur_dashboard_msgs::GetLoadedProgram::Request& req, ur_dashboard_msgs::GetLoadedProgram::Response& resp) {
            resp.answer = this->client_.sendAndReceive("get loaded program\n");
            std::smatch match;
            std::regex expected("Loaded program: (.+)");
            resp.success = std::regex_match(resp.answer, match, expected);
            if (resp.success)
            {
              resp.program_name = match[1];
            }
            return true;
          });

  // Load a robot installation from a file
  load_installation_service_ =
      nh_.advertiseService<ur_dashboard_msgs::Load::Request, ur_dashboard_msgs::Load::Response>(
          "load_installation", [&](ur_dashboard_msgs::Load::Request& req, ur_dashboard_msgs::Load::Response& resp) {
            resp.answer = this->client_.sendAndReceive("load installation " + req.filename + "\n");
            resp.success = std::regex_match(resp.answer, std::regex("Loading installation: .+"));
            return true;
          });

  // Load a robot program from a file
  load_program_service_ = nh_.advertiseService<ur_dashboard_msgs::Load::Request, ur_dashboard_msgs::Load::Response>(
      "load_program", [&](ur_dashboard_msgs::Load::Request& req, ur_dashboard_msgs::Load::Response& resp) {
        resp.answer = this->client_.sendAndReceive("load " + req.filename + "\n");
        resp.success = std::regex_match(resp.answer, std::regex("Loading program: .+"));
        return true;
      });

  // Query whether the current program is saved
  is_program_saved_service_ = nh_.advertiseService("program_saved", &DashboardClientROS::handleSavedQuery, this);

  // Service to show a popup on the UR Teach pendant.
  popup_service_ = nh_.advertiseService<ur_dashboard_msgs::Popup::Request, ur_dashboard_msgs::Popup::Response>(
      "popup", [&](ur_dashboard_msgs::Popup::Request& req, ur_dashboard_msgs::Popup::Response& resp) {
        resp.answer = this->client_.sendAndReceive("popup " + req.message + "\n");
        resp.success = std::regex_match(resp.answer, std::regex("showing popup"));

        return true;
      });

  // Service to query the current program state
  program_state_service_ =
      nh_.advertiseService<ur_dashboard_msgs::GetProgramState::Request, ur_dashboard_msgs::GetProgramState::Response>(
          "program_state",
          [&](ur_dashboard_msgs::GetProgramState::Request& req, ur_dashboard_msgs::GetProgramState::Response& resp) {
            resp.answer = this->client_.sendAndReceive("programState\n");
            std::smatch match;
            std::regex expected("(STOPPED|PLAYING|PAUSED) (.+)");
            resp.success = std::regex_match(resp.answer, match, expected);
            if (resp.success)
            {
              resp.state.state = match[1];
              resp.program_name = match[2];
            }
            return true;
          });

  // Service to query the current safety mode
  safety_mode_service_ = nh_.advertiseService("get_safety_mode", &DashboardClientROS::handleSafetyModeQuery, this);

  // Service to query the current robot mode
  robot_mode_service_ = nh_.advertiseService("get_robot_mode", &DashboardClientROS::handleRobotModeQuery, this);

  // Service to add a message to the robot's log
  add_to_log_service_ =
      nh_.advertiseService<ur_dashboard_msgs::AddToLog::Request, ur_dashboard_msgs::AddToLog::Response>(
          "add_to_log", [&](ur_dashboard_msgs::AddToLog::Request& req, ur_dashboard_msgs::AddToLog::Response& resp) {
            resp.answer = this->client_.sendAndReceive("addToLog " + req.message + "\n");
            resp.success = std::regex_match(resp.answer, std::regex("(Added log message|No log message to add)"));

            return true;
          });

  // General purpose service to send arbitrary messages to the dashboard server
  raw_request_service_ =
      nh_.advertiseService<ur_dashboard_msgs::RawRequest::Request, ur_dashboard_msgs::RawRequest::Response>(
          "raw_request",
          [&](ur_dashboard_msgs::RawRequest::Request& req, ur_dashboard_msgs::RawRequest::Response& resp) {
            resp.answer = this->client_.sendAndReceive(req.query + "\n");
            return true;
          });

  // Service to reconnect to the dashboard server
  reconnect_service_ = nh_.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
      "connect", [&](std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp) {
        resp.success = connect();
        return true;
      });

  // Disconnect from the dashboard service.
  quit_service_ = nh_.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
      "quit", [&](std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp) {
        resp.message = this->client_.sendAndReceive("quit\n");
        resp.success = std::regex_match(resp.message, std::regex("Disconnected"));
        client_.disconnect();
        return true;
      });
}

bool DashboardClientROS::handleRunningQuery(ur_dashboard_msgs::IsProgramRunning::Request& req,
                                            ur_dashboard_msgs::IsProgramRunning::Response& resp)
{
  resp.answer = this->client_.sendAndReceive("running\n");
  std::regex expected("Program running: (true|false)");
  std::smatch match;
  resp.success = std::regex_match(resp.answer, match, expected);

  if (resp.success)
  {
    resp.program_running = (match[1] == "true");
  }

  return true;
}

bool DashboardClientROS::handleSavedQuery(ur_dashboard_msgs::IsProgramSaved::Request& req,
                                          ur_dashboard_msgs::IsProgramSaved::Response& resp)
{
  resp.answer = this->client_.sendAndReceive("isProgramSaved\n");
  std::regex expected("(true|false) ([^\\s]+)");
  std::smatch match;
  resp.success = std::regex_match(resp.answer, match, expected);

  if (resp.success)
  {
    resp.program_saved = (match[1] == "true");
    resp.program_name = match[2];
  }

  return true;
}

bool DashboardClientROS::handleSafetyModeQuery(ur_dashboard_msgs::GetSafetyMode::Request& req,
                                               ur_dashboard_msgs::GetSafetyMode::Response& resp)
{
  resp.answer = this->client_.sendAndReceive("safetymode\n");
  std::smatch match;
  std::regex expected("Safetymode: (.+)");
  resp.success = std::regex_match(resp.answer, match, expected);
  if (resp.success)
  {
    if (match[1] == "NORMAL")
    {
      resp.safety_mode.mode = ur_dashboard_msgs::SafetyMode::NORMAL;
    }
    else if (match[1] == "REDUCED")
    {
      resp.safety_mode.mode = ur_dashboard_msgs::SafetyMode::REDUCED;
    }
    else if (match[1] == "PROTECTIVE_STOP")
    {
      resp.safety_mode.mode = ur_dashboard_msgs::SafetyMode::PROTECTIVE_STOP;
    }
    else if (match[1] == "RECOVERY")
    {
      resp.safety_mode.mode = ur_dashboard_msgs::SafetyMode::RECOVERY;
    }
    else if (match[1] == "SAFEGUARD_STOP")
    {
      resp.safety_mode.mode = ur_dashboard_msgs::SafetyMode::SAFEGUARD_STOP;
    }
    else if (match[1] == "SYSTEM_EMERGENCY_STOP")
    {
      resp.safety_mode.mode = ur_dashboard_msgs::SafetyMode::SYSTEM_EMERGENCY_STOP;
    }
    else if (match[1] == "ROBOT_EMERGENCY_STOP")
    {
      resp.safety_mode.mode = ur_dashboard_msgs::SafetyMode::ROBOT_EMERGENCY_STOP;
    }
    else if (match[1] == "VIOLATION")
    {
      resp.safety_mode.mode = ur_dashboard_msgs::SafetyMode::VIOLATION;
    }
    else if (match[1] == "FAULT")
    {
      resp.safety_mode.mode = ur_dashboard_msgs::SafetyMode::FAULT;
    }
    // The following are only available in SafetyStatus from 5.5 on
    // else if (match[1] == "AUTOMATIC_MODE_SAFEGUARD_STOP")
    //{
    // resp.safety_mode.mode = ur_dashboard_msgs::SafetyMode::AUTOMATIC_MODE_SAFEGUARD_STOP;
    //}
    // else if (match[1] == "SYSTEM_THREE_POSITION_ENABLING_STOP")
    //{
    // resp.safety_mode.mode = ur_dashboard_msgs::SafetyMode::SYSTEM_THREE_POSITION_ENABLING_STOP;
    //}
  }
  return true;
}

bool DashboardClientROS::handleRobotModeQuery(ur_dashboard_msgs::GetRobotMode::Request& req,
                                              ur_dashboard_msgs::GetRobotMode::Response& resp)
{
  resp.answer = this->client_.sendAndReceive("robotmode\n");
  std::smatch match;
  std::regex expected("Robotmode: (.+)");
  resp.success = std::regex_match(resp.answer, match, expected);
  if (resp.success)
  {
    if (match[1] == "NO_CONTROLLER")
    {
      resp.robot_mode.mode = ur_dashboard_msgs::RobotMode::NO_CONTROLLER;
    }
    else if (match[1] == "DISCONNECTED")
    {
      resp.robot_mode.mode = ur_dashboard_msgs::RobotMode::DISCONNECTED;
    }
    else if (match[1] == "CONFIRM_SAFETY")
    {
      resp.robot_mode.mode = ur_dashboard_msgs::RobotMode::CONFIRM_SAFETY;
    }
    else if (match[1] == "BOOTING")
    {
      resp.robot_mode.mode = ur_dashboard_msgs::RobotMode::BOOTING;
    }
    else if (match[1] == "POWER_OFF")
    {
      resp.robot_mode.mode = ur_dashboard_msgs::RobotMode::POWER_OFF;
    }
    else if (match[1] == "POWER_ON")
    {
      resp.robot_mode.mode = ur_dashboard_msgs::RobotMode::POWER_ON;
    }
    else if (match[1] == "IDLE")
    {
      resp.robot_mode.mode = ur_dashboard_msgs::RobotMode::IDLE;
    }
    else if (match[1] == "BACKDRIVE")
    {
      resp.robot_mode.mode = ur_dashboard_msgs::RobotMode::BACKDRIVE;
    }
    else if (match[1] == "RUNNING")
    {
      resp.robot_mode.mode = ur_dashboard_msgs::RobotMode::RUNNING;
    }
    else if (match[1] == "UPDATING_FIRMWARE")
    {
      resp.robot_mode.mode = ur_dashboard_msgs::RobotMode::UPDATING_FIRMWARE;
    }
  }
  return true;
}

bool DashboardClientROS::connect()
{
  timeval tv;
  // Timeout after which a call to the dashboard server will be considered failure if no answer has
  // been received.
  tv.tv_sec = nh_.param("receive_timeout", 1);
  tv.tv_usec = 0;
  client_.setReceiveTimeout(tv);
  return client_.connect();
}
}  // namespace ur_driver
