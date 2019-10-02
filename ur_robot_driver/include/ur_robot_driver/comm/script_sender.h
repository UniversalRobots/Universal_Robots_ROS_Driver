
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
 * \author  Tristan Schnell schnell@fzi.de
 * \date    2019-05-22
 *
 */
//----------------------------------------------------------------------

#ifndef UR_RTDE_DRIVER_SCRIPT_SENDER_H_INCLUDED
#define UR_RTDE_DRIVER_SCRIPT_SENDER_H_INCLUDED

#include "ur_robot_driver/comm/server.h"
#include "ur_robot_driver/log.h"

namespace ur_driver
{
namespace comm
{
/*!
 * \brief The ScriptSender class starts a URServer for a robot to connect to and waits for a
 * request to receive a program. This program is then delivered to the requesting robot.
 */
class ScriptSender
{
public:
  ScriptSender() = delete;
  /*!
   * \brief Creates a ScriptSender object, including a new URServer and not yet started thread.
   *
   * \param port Port to start the server on
   * \param program Program to send to the robot upon request
   */
  ScriptSender(uint32_t port, const std::string& program) : server_(port), script_thread_(), program_(program)
  {
    if (!server_.bind())
    {
      throw std::runtime_error("Could not bind to server");
    }
  }

  /*!
   * \brief Starts the thread that handles program requests by a robot.
   */
  void start()
  {
    script_thread_ = std::thread(&ScriptSender::runScriptSender, this);
  }

private:
  URServer server_;
  std::thread script_thread_;
  std::string program_;

  const std::string PROGRAM_REQUEST_ = std::string("request_program\n");

  void runScriptSender()
  {
    while (true)
    {
      if (!server_.accept())
      {
        throw std::runtime_error("Failed to accept robot connection");
      }
      if (requestRead())
      {
        LOG_INFO("Robot requested program");
        sendProgram();
      }
      server_.disconnectClient();
    }
  }

  bool requestRead()
  {
    size_t buf_len = 1024;
    char buffer[buf_len];

    bool read_successful = server_.readLine(buffer, buf_len);

    if (read_successful)
    {
      if (std::string(buffer) == PROGRAM_REQUEST_)
      {
        return true;
      }
      else
      {
        LOG_WARN("Received unexpected message on script request port ");
      }
    }
    else
    {
      LOG_WARN("Could not read on script request port");
    }
    return false;
  }

  void sendProgram()
  {
    size_t len = program_.size();
    const uint8_t* data = reinterpret_cast<const uint8_t*>(program_.c_str());
    size_t written;

    if (server_.write(data, len, written))
    {
      LOG_INFO("Sent program to robot");
    }
    else
    {
      LOG_ERROR("Could not send program to robot");
    }
  }
};

}  // namespace comm
}  // namespace ur_driver

#endif  // UR_RTDE_DRIVER_SCRIPT_SENDER_H_INCLUDED
