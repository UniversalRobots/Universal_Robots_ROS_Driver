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

#include "ur_modern_driver/ur/robot_mode.h"
#include "ur_modern_driver/ur/consumer.h"

bool SharedRobotModeData::parseWith(BinParser& bp)
{
  bp.parse(timestamp);
  bp.parse(physical_robot_connected);
  bp.parse(real_robot_enabled);
  bp.parse(robot_power_on);
  bp.parse(emergency_stopped);
  bp.parse(protective_stopped);
  bp.parse(program_running);
  bp.parse(program_paused);
  return true;
}

bool RobotModeData_V1_X::parseWith(BinParser& bp)
{
  if (!bp.checkSize<RobotModeData_V1_X>())
    return false;

  SharedRobotModeData::parseWith(bp);

  bp.parse(robot_mode);
  bp.parse(speed_fraction);

  return true;
}

bool RobotModeData_V3_0__1::parseWith(BinParser& bp)
{
  if (!bp.checkSize<RobotModeData_V3_0__1>())
    return false;

  SharedRobotModeData::parseWith(bp);

  bp.parse(robot_mode);
  bp.parse(control_mode);
  bp.parse(target_speed_fraction);
  bp.parse(speed_scaling);

  return true;
}

bool RobotModeData_V3_2::parseWith(BinParser& bp)
{
  if (!bp.checkSize<RobotModeData_V3_2>())
    return false;

  RobotModeData_V3_0__1::parseWith(bp);

  bp.parse(target_speed_fraction_limit);

  return true;
}

bool RobotModeData_V3_5::parseWith(BinParser& bp)
{
  if (!bp.checkSize<RobotModeData_V3_5>())
    return false;

  RobotModeData_V3_2::parseWith(bp);

  bp.parse(unknown_internal_use);

  return true;
}

bool RobotModeData_V1_X::consumeWith(URStatePacketConsumer& consumer)
{
  return consumer.consume(*this);
}
bool RobotModeData_V3_0__1::consumeWith(URStatePacketConsumer& consumer)
{
  return consumer.consume(*this);
}
bool RobotModeData_V3_2::consumeWith(URStatePacketConsumer& consumer)
{
  return consumer.consume(*this);
}
bool RobotModeData_V3_5::consumeWith(URStatePacketConsumer& consumer)
{
  return consumer.consume(*this);
}
