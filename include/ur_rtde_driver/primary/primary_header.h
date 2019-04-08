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
 * \author  Lea Steffen steffen@fzi.de
 * \date    2019-04-01
 *
 */
//----------------------------------------------------------------------

#ifndef UR_RTDE_DRIVER_PRIMARY_INTERFACE_H_INCLUDED
#define UR_RTDE_DRIVER_PRIMARY_INTERFACE_H_INCLUDED

#include <inttypes.h>
#include <cstddef>
#include <endian.h>
#include "ur_rtde_driver/types.h"

namespace ur_driver
{
namespace primary
{
enum class message_type : int8_t
{
  DISCONNECT = -1,
  ROBOT_STATE = 16,
  ROBOT_MESSAGE = 20,
  HMC_MESSAGE = 22,
  MODBUS_INFO_MESSAGE = 5,
  SAFETY_SETUP_BROADCAST_MESSAGE = 23,
  SAFETY_COMPLIANCE_TOLERANCES_MESSAGE = 24,
  PROGRAM_STATE_MESSAGE = 25
};

enum class robot_state_type : uint8_t
{
  ROBOT_MODE_DATA = 0,
  JOINT_DATA = 1,
  TOOL_DATA = 2,
  MASTERBOARD_DATA = 3,
  CARTESIAN_INFO = 4,
  KINEMATICS_INFO = 5,
  CONFIGURATION_DATA = 6,
  FORCE_MODE_DATA = 7,
  ADDITIONAL_INFO = 8,
  CALIBRATION_DATA = 9
};

enum class robot_message_type : uint8_t
{
  ROBOT_MESSAGE_TEXT = 0,
  ROBOT_MESSAGE_PROGRAM_LABEL = 1,
  PROGRAM_STATE_MESSAGE_VARIABLE_UPDATE = 2,
  ROBOT_MESSAGE_VERSION = 3,
  ROBOT_MESSAGE_SAFETY_MODE = 5,
  ROBOT_MESSAGE_ERROR_CODE = 6,
  ROBOT_MESSAGE_KEY = 7,
  ROBOT_MESSAGE_REQUEST_VALUE = 9,
  ROBOT_MESSAGE_RUNTIME_EXCEPTION = 10
};


class Header
{
public:
  Header() = default;
  virtual ~Header() = default;

  using _package_size_type = int32_t;
  static size_t getPackageLength(uint8_t* buf)
  {
    return be32toh(*(reinterpret_cast<int32_t*>(buf)));
  }

  static size_t getPackageSize()
  {
    return sizeof(package_size_);
  }

  

private:
  int32_t package_size_;
  int8_t package_type_;
};
}  // namespace primary_interface
}  // namespace ur_driver

#endif  // ifndef UR_RTDE_DRIVER_PRIMARY_INTERFACE_H_INCLUDED
