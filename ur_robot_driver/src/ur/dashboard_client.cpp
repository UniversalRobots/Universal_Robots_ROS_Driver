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
DashboardClient::DashboardClient(const std::string& host) : host_(host), port_(DASHBOARD_SERVER_PORT)
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
  std::string response = "ERROR";
  std::lock_guard<std::mutex> lock(write_mutex_);
  if (send(text))
  {
    response = read();
  }
  rtrim(response);

  return response;
}

}  // namespace ur_driver