/*
 * Copyright 2017, 2018 Simon Rasmussen (refactor)
 *
 * Copyright 2015, 2016 Thomas Timm Andersen (original version)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <inttypes.h>
#include <cstddef>
#include "ur_modern_driver/bin_parser.h"
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/pipeline.h"

enum class package_type : uint8_t
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

enum class message_type : uint8_t
{
  ROBOT_STATE = 16,
  ROBOT_MESSAGE = 20,
  PROGRAM_STATE_MESSAGE = 25
};

class URStatePacketConsumer;

class StatePacket
{
public:
  StatePacket()
  {
  }
  virtual ~StatePacket()
  {
  }
  virtual bool parseWith(BinParser& bp) = 0;
  virtual bool consumeWith(URStatePacketConsumer& consumer) = 0;
};
