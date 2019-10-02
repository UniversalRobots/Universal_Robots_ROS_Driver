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
 * \date    2019-04-11
 *
 */
//----------------------------------------------------------------------

#ifndef UR_RTDE_DRIVER_REVERSE_INTERFACE_H_INCLUDED
#define UR_RTDE_DRIVER_REVERSE_INTERFACE_H_INCLUDED

#include "ur_robot_driver/comm/server.h"
#include "ur_robot_driver/types.h"
#include <cstring>
#include <endian.h>

namespace ur_driver
{
namespace comm
{
/*!
 * \brief The ReverseInterface class handles communication to the robot. It starts a server and
 * waits for the robot to connect via its URCaps program.
 */
class ReverseInterface
{
public:
  ReverseInterface() = delete;
  /*!
   * \brief Creates a ReverseInterface object including a URServer.
   *
   * \param port Port the Server is started on
   */
  ReverseInterface(uint32_t port) : server_(port)
  {
    if (!server_.bind())
    {
      throw std::runtime_error("Could not bind to server");
    }
    if (!server_.accept())
    {
      throw std::runtime_error("Failed to accept robot connection");
    }
  }
  /*!
   * \brief Disconnects possible clients so the reverse interface object can be safely destroyed.
   */
  ~ReverseInterface()
  {
    server_.disconnectClient();
  }

  /*!
   * \brief Writes needed information to the robot to be read by the URCaps program.
   *
   * \param positions A vector of joint position targets for the robot
   * \param type An additional integer used to command the program to end when needed
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  bool write(const vector6d_t* positions, const int32_t type = 2)
  {
    uint8_t buffer[sizeof(uint32_t) * 7];
    uint8_t* b_pos = buffer;

    int32_t val = htobe32(type);
    b_pos += append(b_pos, val);

    if (positions != nullptr)
    {
      for (auto const& pos : *positions)
      {
        int32_t val = static_cast<int32_t>(pos * MULT_JOINTSTATE);
        val = htobe32(val);
        b_pos += append(b_pos, val);
      }
    }

    size_t written;

    return server_.write(buffer, sizeof(buffer), written);
  }

  /*!
   * \brief Reads a keepalive signal from the robot.
   *
   * \returns The received keepalive string or the empty string, if nothing was received
   */
  std::string readKeepalive()
  {
    size_t buf_len = 16;
    char buffer[buf_len];

    bool read_successful = server_.readLine(buffer, buf_len);

    if (read_successful)
    {
      return std::string(buffer);
    }
    else
    {
      return std::string("");
    }
  }

private:
  URServer server_;
  static const int32_t MULT_JOINTSTATE = 1000000;

  template <typename T>
  size_t append(uint8_t* buffer, T& val)
  {
    size_t s = sizeof(T);
    std::memcpy(buffer, &val, s);
    return s;
  }
};

}  // namespace comm
}  // namespace ur_driver

#endif  // UR_RTDE_DRIVER_REVERSE_INTERFACE_H_INCLUDED
