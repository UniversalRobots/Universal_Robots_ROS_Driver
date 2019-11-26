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
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-04-08
 *
 */
//----------------------------------------------------------------------

#ifndef UR_RTDE_DRIBVER_ROBOT_MODE_DATA_H_INCLUDED
#define UR_RTDE_DRIBVER_ROBOT_MODE_DATA_H_INCLUDED

#include "ur_robot_driver/primary/robot_state.h"
#include <ur_robot_driver/ur/datatypes.h>

namespace ur_driver
{
namespace primary_interface
{
class SharedRobotModeData
{
public:
  virtual bool parseWith(comm::BinParser& bp);

  uint64_t timestamp;
  bool physical_robot_connected;
  bool real_robot_enabled;
  bool robot_power_on;
  bool emergency_stopped;
  bool protective_stopped;  // AKA security_stopped
  bool program_running;
  bool program_paused;

  static const size_t SIZE = sizeof(uint64_t) + sizeof(uint8_t) * 6;
};

enum class robot_control_mode_V3_X : uint8_t
{
  POSITION = 0,
  TEACH = 1,
  FORCE = 2,
  TORQUE = 3
};

class RobotModeData_V3_0__1 : public SharedRobotModeData, public RobotState
{
public:
  virtual bool parseWith(comm::BinParser& bp);

  RobotMode robot_mode;
  robot_control_mode_V3_X control_mode;

  double target_speed_fraction;
  double speed_scaling;

  static const size_t SIZE = SharedRobotModeData::SIZE + sizeof(uint8_t) + sizeof(RobotMode) +
                             sizeof(robot_control_mode_V3_X) + sizeof(double) + sizeof(double);

  static_assert(RobotModeData_V3_0__1::SIZE == 33, "RobotModeData_V3_0__1 has missmatched size");
};

class RobotModeData_V3_2 : public RobotModeData_V3_0__1
{
public:
  virtual bool parseWith(comm::BinParser& bp);
  // virtual bool consumeWith(URStatePacketConsumer& consumer);

  double target_speed_fraction_limit;

  static const size_t SIZE = RobotModeData_V3_0__1::SIZE + sizeof(double);

  static_assert(RobotModeData_V3_2::SIZE == 41, "RobotModeData_V3_2 has missmatched size");
};

class RobotModeData_V3_5 : public RobotModeData_V3_2
{
public:
  virtual bool parseWith(comm::BinParser& bp);
  // virtual bool consumeWith(URStatePacketConsumer& consumer);

  unsigned char unknown_internal_use;

  static const size_t SIZE = RobotModeData_V3_2::SIZE + sizeof(unsigned char);

  static_assert(RobotModeData_V3_5::SIZE == 42, "RobotModeData_V3_5 has missmatched size");
};
}  // namespace primary_interface
}  // namespace ur_driver
#endif  // ifndef UR_RTDE_DRIBVER_ROBOT_MODE_DATA_H_INCLUDED
