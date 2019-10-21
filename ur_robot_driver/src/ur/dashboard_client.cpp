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

bool DashboardClient::addToLog(const std::string& text)
{
  std::string answer = sendAndReceive("addToLog " + text + "\n");
  if (answer != "Aded log message")
  {
    return false;
  }
  return true;
}

bool DashboardClient::brakeRelease()
{
  std::string answer = sendAndReceive("brake release\n");
  if (answer != "Brake releasing")
  {
    return false;
  }
  return true;
}

bool DashboardClient::clearOperationalMode()
{
  std::string answer = sendAndReceive("clear operational mode\n");
  if (answer != "operational mode is no longer controlled by Dashboard Server")
  {
    return false;
  }
  return true;
}

bool DashboardClient::closePopup()
{
  std::string answer = sendAndReceive("close popup\n");
  if (answer != "closing popup")
  {
    return false;
  }
  return true;
}

bool DashboardClient::closeSafetyPopup()
{
  std::string answer = sendAndReceive("close safety popup\n");
  if (answer != "closing safety popup")
  {
    return false;
  }
  return true;
}

bool DashboardClient::loadInstallation(const std::string& installation_name)
{
  std::string answer = sendAndReceive("load installation " + installation_name + "\n");
  if (answer != "Loading installation: " + installation_name)
  {
    return false;
  }
  return true;
}

bool DashboardClient::loadProgram(const std::string& program_name)
{
  std::string answer = sendAndReceive("load " + program_name + "\n");
  if (answer != "Loading program: " + program_name + "\n")
  {
    return false;
  }
  return true;
}

bool DashboardClient::pause()
{
  std::string answer = sendAndReceive("pause");
  if (answer != "Pausing program\n")
  {
    return false;
  }
  return true;
}

bool DashboardClient::play()
{
  std::string answer = sendAndReceive("play\n");
  if (answer != "Starting program\n")
  {
    return false;
  }
  return true;
}

bool DashboardClient::popup(const std::string& text)
{
  std::string answer = sendAndReceive("popup " + text + "\n");
  if (answer != "showing popup\n")
  {
    return false;
  }
  return true;
}

bool DashboardClient::powerOff()
{
  std::string answer = sendAndReceive("power off\n");
  if (answer != "Powering off\n")
  {
    return false;
  }
  return true;
}

bool DashboardClient::powerOn()
{
  std::string answer = sendAndReceive("power on\n");
  if (answer != "Powering on\n")
  {
    return false;
  }
  return true;
}

bool DashboardClient::quit()
{
  std::string answer = sendAndReceive("quit\n");
  if (answer != "Disconnected\n")
  {
    return false;
  }
  return true;
}

bool DashboardClient::restartSafety()
{
  std::string answer = sendAndReceive("restart safety\n");
  if (answer != "Restarting safety\n")
  {
    return false;
  }
  return true;
}

bool DashboardClient::setOperationalMode(const OperationalMode mode)
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
  std::string answer = sendAndReceive("set operational mode " + mode_string + "\n");
  if (answer != "Setting operational mode: " + mode_string + "\n")
  {
    return false;
  }
  return true;
}

bool DashboardClient::shutdown()
{
  std::string answer = sendAndReceive("shutdown\n");
  if (answer != "Shutting down\n")
  {
    return false;
  }
  return true;
}

bool DashboardClient::stop()
{
  std::string answer = sendAndReceive("stop\n");
  if (answer != "Stopped\n")
  {
    return false;
  }
  return true;
}

bool DashboardClient::unlockProtectiveStop()
{
  std::string answer = sendAndReceive("unlock protective stop\n");
  if (answer != "Protective stop releasing\n")
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
