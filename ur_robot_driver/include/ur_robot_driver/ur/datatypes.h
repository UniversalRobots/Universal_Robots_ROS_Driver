// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
// Copyright 2015, 2016 Thomas Timm Andersen (original version)
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
 * This file contains enums for internal mode representations.
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-11-04
 *
 */
//----------------------------------------------------------------------

#include <ur_robot_driver/types.h>

namespace ur_driver
{
enum class RobotMode : int8_t
{
  UNKNOWN = -128,  // This is not defined by UR but only inside this driver
  NO_CONTROLLER = -1,
  DISCONNECTED = 0,
  CONFIRM_SAFETY = 1,
  BOOTING = 2,
  POWER_OFF = 3,
  POWER_ON = 4,
  IDLE = 5,
  BACKDRIVE = 6,
  RUNNING = 7,
  UPDATING_FIRMWARE = 8
};

enum class SafetyMode : int8_t
{
  NORMAL = 1,
  REDUCED = 2,
  PROTECTIVE_STOP = 3,
  RECOVERY = 4,
  SAFEGUARD_STOP = 5,
  SYSTEM_EMERGENCY_STOP = 6,
  ROBOT_EMERGENCY_STOP = 7,
  VIOLATION = 8,
  FAULT = 9,
  VALIDATE_JOINT_ID = 10,
  UNDEFINED_SAFETY_MODE = 11
};

enum class SafetyStatus : int8_t  // Only available on 3.10/5.4
{
  NORMAL = 1,
  REDUCED = 2,
  PROTECTIVE_STOP = 3,
  RECOVERY = 4,
  SAFEGUARD_STOP = 5,
  SYSTEM_EMERGENCY_STOP = 6,
  ROBOT_EMERGENCY_STOP = 7,
  VIOLATION = 8,
  FAULT = 9,
  VALIDATE_JOINT_ID = 10,
  UNDEFINED_SAFETY_MODE = 11,
  AUTOMATIC_MODE_SAFEGUARD_STOP = 12,
  SYSTEM_THREE_POSITION_ENABLING_STOP = 13
};

inline std::string robotModeString(const RobotMode& mode)
{
  switch (mode)
  {
    case RobotMode::NO_CONTROLLER:
      return "NO_CONTROLLER";
    case RobotMode::DISCONNECTED:
      return "DISCONNECTED";
    case RobotMode::CONFIRM_SAFETY:
      return "CONFIRM_SAFETY";
    case RobotMode::BOOTING:
      return "BOOTING";
    case RobotMode::POWER_OFF:
      return "POWER_OFF";
    case RobotMode::POWER_ON:
      return "POWER_ON";
    case RobotMode::IDLE:
      return "IDLE";
    case RobotMode::BACKDRIVE:
      return "BACKDRIVE";
    case RobotMode::RUNNING:
      return "RUNNING";
    case RobotMode::UPDATING_FIRMWARE:
      return "UPDATING_FIRMWARE";
    default:
      std::stringstream ss;
      ss << "Unknown robot mode: " << static_cast<int>(mode);
      throw std::invalid_argument(ss.str());
  }
}

inline std::string safetyModeString(const SafetyMode& mode)
{
  switch (mode)
  {
    case SafetyMode::NORMAL:
      return "NORMAL";
    case SafetyMode::REDUCED:
      return "REDUCED";
    case SafetyMode::PROTECTIVE_STOP:
      return "PROTECTIVE_STOP";
    case SafetyMode::RECOVERY:
      return "RECOVERY";
    case SafetyMode::SAFEGUARD_STOP:
      return "SAFEGUARD_STOP";
    case SafetyMode::SYSTEM_EMERGENCY_STOP:
      return "SYSTEM_EMERGENCY_STOP";
    case SafetyMode::ROBOT_EMERGENCY_STOP:
      return "ROBOT_EMERGENCY_STOP";
    case SafetyMode::VIOLATION:
      return "VIOLATION";
    case SafetyMode::FAULT:
      return "FAULT";
    case SafetyMode::VALIDATE_JOINT_ID:
      return "VALIDATE_JOINT_ID";
    case SafetyMode::UNDEFINED_SAFETY_MODE:
      return "UNDEFINED_SAFETY_MODE";
    default:
      std::stringstream ss;
      ss << "Unknown safety mode: " << static_cast<int>(mode);
      throw std::invalid_argument(ss.str());
  }
}

inline std::string safetyStatusString(const SafetyStatus& status)
{
  switch (status)
  {
    case SafetyStatus::NORMAL:
      return "NORMAL";
    case SafetyStatus::REDUCED:
      return "REDUCED";
    case SafetyStatus::PROTECTIVE_STOP:
      return "PROTECTIVE_STOP";
    case SafetyStatus::RECOVERY:
      return "RECOVERY";
    case SafetyStatus::SAFEGUARD_STOP:
      return "SAFEGUARD_STOP";
    case SafetyStatus::SYSTEM_EMERGENCY_STOP:
      return "SYSTEM_EMERGENCY_STOP";
    case SafetyStatus::ROBOT_EMERGENCY_STOP:
      return "ROBOT_EMERGENCY_STOP";
    case SafetyStatus::VIOLATION:
      return "VIOLATION";
    case SafetyStatus::FAULT:
      return "FAULT";
    case SafetyStatus::VALIDATE_JOINT_ID:
      return "VALIDATE_JOINT_ID";
    case SafetyStatus::UNDEFINED_SAFETY_MODE:
      return "UNDEFINED_SAFETY_MODE";
    case SafetyStatus::AUTOMATIC_MODE_SAFEGUARD_STOP:
      return "AUTOMATIC_MODE_SAFEGUARD_STOP";
    case SafetyStatus::SYSTEM_THREE_POSITION_ENABLING_STOP:
      return "SYSTEM_THREE_POSITION_ENABLING_STOP";
    default:
      std::stringstream ss;
      ss << "Unknown safety status: " << static_cast<int>(status);
      throw std::invalid_argument(ss.str());
  }
}
}  // namespace ur_driver
