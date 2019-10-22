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

#include <regex>
#include <ur_robot_driver/log.h>
#include <ur_robot_driver/ur/dashboard_client.h>

namespace ur_driver
{
DashboardClient::DashboardClient(const std::string& host, int port) : host_(host), port_(port)
{
}

bool DashboardClient::connect()
{
  bool ret_val = false;
  if (TCPSocket::setup(host_, port_))
  {
    LOG_INFO("%s", read().c_str());
    ret_val = true;
  }
  return ret_val;
  ;
}

void DashboardClient::disconnect()
{
  LOG_DEBUG("Disconnecting from %s:%d", host_.c_str(), port_);
  TCPSocket::close();
}

bool DashboardClient::send(const std::string& text)
{
  size_t len = text.size();
  const uint8_t* data = reinterpret_cast<const uint8_t*>(text.c_str());
  size_t written;
  return TCPSocket::write(data, len, written);
}

std::string DashboardClient::read()
{
  std::stringstream result;
  char character;
  size_t read_chars = 99;
  while (read_chars > 0)
  {
    TCPSocket::read((uint8_t*)&character, 1, read_chars);
    result << character;
    if (character == '\n')
    {
      break;
    }
  }
  return result.str();
}

std::string DashboardClient::sendAndReceive(const std::string& text)
{
  std::lock_guard<std::mutex> lock(write_mutex_);
  if (send(text))
  {
    return read();
  }

  return "ERROR";
}

bool DashboardClient::addToLog(const std::string& text, std::string& response)
{
  response = sendAndReceive("addToLog " + text + "\n");
  rtrim(response);
  if (response != "Added log message")
  {
    return false;
  }
  return true;
}

bool DashboardClient::brakeRelease(std::string& response)
{
  response = sendAndReceive("brake release\n");
  rtrim(response);
  if (response != "Brake releasing")
  {
    return false;
  }
  return true;
}

bool DashboardClient::clearOperationalMode(std::string& response)
{
  response = sendAndReceive("clear operational mode\n");
  rtrim(response);
  std::regex re("No longer controlling the operational mode\\. Current operational mode: '(MANUAL|AUTOMATIC)'\\.");
  if (std::regex_match(response, re))
  {
    return true;
  }
  return false;
}

bool DashboardClient::closePopup(std::string& response)
{
  response = sendAndReceive("close popup\n");
  rtrim(response);
  if (response != "closing popup")
  {
    return false;
  }
  return true;
}

bool DashboardClient::closeSafetyPopup(std::string& response)
{
  response = sendAndReceive("close safety popup\n");
  rtrim(response);
  if (response != "closing safety popup")
  {
    return false;
  }
  return true;
}

bool DashboardClient::loadInstallation(const std::string& installation_name, std::string& response)
{
  response = sendAndReceive("load installation " + installation_name + "\n");
  rtrim(response);
  if (response != "Loading installation: " + installation_name)
  {
    return false;
  }
  return true;
}

bool DashboardClient::loadProgram(const std::string& program_name, std::string& response)
{
  response = sendAndReceive("load " + program_name + "\n");
  rtrim(response);
  if (response != "Loading program: " + program_name)
  {
    return false;
  }
  return true;
}

bool DashboardClient::pause(std::string& response)
{
  response = sendAndReceive("pause\n");
  rtrim(response);
  if (response != "Pausing program")
  {
    return false;
  }
  return true;
}

bool DashboardClient::play(std::string& response)
{
  response = sendAndReceive("play\n");
  rtrim(response);
  if (response != "Starting program")
  {
    return false;
  }
  return true;
}

bool DashboardClient::popup(const std::string& text, std::string& response)
{
  response = sendAndReceive("popup " + text + "\n");
  rtrim(response);
  if (response != "showing popup")
  {
    return false;
  }
  return true;
}

bool DashboardClient::powerOff(std::string& response)
{
  response = sendAndReceive("power off\n");
  rtrim(response);
  if (response != "Powering off")
  {
    return false;
  }
  return true;
}

bool DashboardClient::powerOn(std::string& response)
{
  response = sendAndReceive("power on\n");
  rtrim(response);
  if (response != "Powering on")
  {
    return false;
  }
  return true;
}

bool DashboardClient::quit(std::string& response)
{
  response = sendAndReceive("quit\n");
  rtrim(response);
  if (response != "Disconnected")
  {
    return false;
  }
  return true;
}

bool DashboardClient::restartSafety(std::string& response)
{
  response = sendAndReceive("restart safety\n");
  rtrim(response);
  if (response != "Restarting safety")
  {
    return false;
  }
  return true;
}

bool DashboardClient::setOperationalMode(const OperationalMode mode, std::string& response)
{
  std::string mode_string;
  switch (mode)
  {
    case OperationalMode::MANUAL:
      mode_string = "manual";
      break;
    case OperationalMode::AUTOMATIC:
      mode_string = "automatic";
      break;
    default:
      throw std::runtime_error("Enum value not known.");
  }
  response = sendAndReceive("set operational mode " + mode_string + "\n");
  rtrim(response);
  if (response != "Setting operational mode: " + mode_string)
  {
    return false;
  }
  return true;
}

bool DashboardClient::shutdown(std::string& response)
{
  response = sendAndReceive("shutdown\n");
  rtrim(response);
  if (response != "Shutting down")
  {
    return false;
  }
  return true;
}

bool DashboardClient::stop(std::string& response)
{
  response = sendAndReceive("stop\n");
  rtrim(response);
  if (response != "Stopped")
  {
    return false;
  }
  return true;
}

bool DashboardClient::unlockProtectiveStop(std::string& response)
{
  response = sendAndReceive("unlock protective stop\n");
  rtrim(response);
  if (response != "Protective stop releasing")
  {
    return false;
  }
  return true;
}


bool DashboardClient::running()
{
  std::string answer = sendAndReceive("isrunning\n");
  if (answer == "Program running: True\n")
  {
    return true;
  }
  return false;
}

std::string DashboardClient::robotMode()
{
  std::string answer = sendAndReceive("robotmode\n");
  return answer;
}

std::string DashboardClient::getLoadedProgram()
{
  std::string answer = sendAndReceive("get loaded program\n");
  if (answer == "No program loaded\n")
  {
    return "";
  }
  const std::string replacement = "Loaded program: ";
  return answer.erase(0, replacement.length());
}

bool DashboardClient::isProgramSaved()
{
  std::string answer = sendAndReceive("isProgramSaved\n");
  if (answer == "True\n")
  {
    return true;
  }
  return false;
}

std::string DashboardClient::programState()
{
  std::string answer = sendAndReceive("programState\n");
  return answer;
}

std::string DashboardClient::polyscopeVersion()
{
  std::string answer = sendAndReceive("PolyscpoeVersion\n");
  return answer;
}

std::string DashboardClient::safetyMode()
{
  std::string answer = sendAndReceive("safetymode\n");
  const std::string replacement = "Safetymode: ";
  return answer.erase(0, replacement.length());
}

std::string DashboardClient::safetyStatus()
{
  std::string answer = sendAndReceive("safetystatus\n");
  const std::string replacement = "Safetystatus: ";
  return answer.erase(0, replacement.length());
}

}  // namespace ur_driver
