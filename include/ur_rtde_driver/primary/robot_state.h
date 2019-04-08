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
 * \author  Lea Steffen steffen@fzi.de
 * \date    2019-04-01
 *
 */
//----------------------------------------------------------------------

#ifndef UR_RTDE_DRIVER_ROBOT_STATE_H_INCLUDED
#define UR_RTDE_DRIVER_ROBOT_STATE_H_INCLUDED

#include <mutex>
#include <condition_variable>

#include "ur_rtde_driver/primary/primary_package.h"
#include "ur_rtde_driver/primary/primary_header.h"

namespace ur_driver
{
namespace primary_interface
{
struct version_message
{
  uint64_t timestamp;
  int8_t source;
  int8_t robot_message_type;
  int8_t project_name_size;
  char project_name[15];
  uint8_t major_version;
  uint8_t minor_version;
  int svn_revision;
  char build_date[25];
};

struct masterboard_data
{
  int digitalInputBits;
  int digitalOutputBits;
  char analogInputRange0;
  char analogInputRange1;
  double analogInput0;
  double analogInput1;
  char analogOutputDomain0;
  char analogOutputDomain1;
  double analogOutput0;
  double analogOutput1;
  float masterBoardTemperature;
  float robotVoltage48V;
  float robotCurrent;
  float masterIOCurrent;
  unsigned char safetyMode;
  unsigned char masterOnOffState;
  char euromap67InterfaceInstalled;
  int euromapInputBits;
  int euromapOutputBits;
  float euromapVoltage;
  float euromapCurrent;
};

struct robot_mode_data
{
  uint64_t timestamp;
  bool isRobotConnected;
  bool isRealRobotEnabled;
  bool isPowerOnRobot;
  bool isEmergencyStopped;
  bool isProtectiveStopped;
  bool isProgramRunning;
  bool isProgramPaused;
  unsigned char robotMode;
  unsigned char controlMode;
  double targetSpeedFraction;
  double speedScaling;
};

class RobotState : PrimaryPackage
{
private:
  robot_state_type robot_state_;
  /*
      version_message version_msg_;
    masterboard_data mb_data_;
    robot_mode_data robot_mode_;
      std::recursive_mutex val_lock_; // Locks the variables while unpack parses data;
    std::condition_variable* pMsg_cond_; //Signals that new vars are available
    bool new_data_available_; //to avoid spurious wakes
    unsigned char robot_mode_running_;
  double ntohd(uint64_t nf); */

public:
  RobotState() = default;
  virtual ~RobotState() = default;
};

}  // namespace primary_interface
}  // namespace ur_driver

#endif /* UR_RTDE_DRIVER_ROBOT_STATE_H_INCLUDED */