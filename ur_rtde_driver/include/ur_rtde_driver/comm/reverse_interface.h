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

#include "ur_rtde_driver/comm/server.h"
#include "ur_rtde_driver/types.h"
#include <cstring>
#include <endian.h>

namespace ur_driver
{
namespace comm
{
class ReverseInterface
{
public:
  ReverseInterface() = delete;
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
  ~ReverseInterface()
  {
    server_.disconnectClient();
  }

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

  template <typename T>
  size_t append(uint8_t* buffer, T& val)
  {
    size_t s = sizeof(T);
    std::memcpy(buffer, &val, s);
    return s;
  }

private:
  URServer server_;
  static const int32_t MULT_JOINTSTATE = 1000000;
};

}  // namespace comm
}  // namespace ur_driver

#endif  // UR_RTDE_DRIVER_REVERSE_INTERFACE_H_INCLUDED
