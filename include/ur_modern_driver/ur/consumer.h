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

#include "ur_modern_driver/pipeline.h"
#include "ur_modern_driver/ur/master_board.h"
#include "ur_modern_driver/ur/messages.h"
#include "ur_modern_driver/ur/robot_mode.h"
#include "ur_modern_driver/ur/rt_state.h"
#include "ur_modern_driver/ur/state.h"

class URRTPacketConsumer : public IConsumer<RTPacket>
{
public:
  virtual bool consume(shared_ptr<RTPacket> packet)
  {
    return packet->consumeWith(*this);
  }

  virtual bool consume(RTState_V1_6__7& state) = 0;
  virtual bool consume(RTState_V1_8& state) = 0;
  virtual bool consume(RTState_V3_0__1& state) = 0;
  virtual bool consume(RTState_V3_2__3& state) = 0;
};

class URStatePacketConsumer : public IConsumer<StatePacket>
{
public:
  virtual bool consume(shared_ptr<StatePacket> packet)
  {
    return packet->consumeWith(*this);
  }

  virtual bool consume(MasterBoardData_V1_X& data) = 0;
  virtual bool consume(MasterBoardData_V3_0__1& data) = 0;
  virtual bool consume(MasterBoardData_V3_2& data) = 0;

  virtual bool consume(RobotModeData_V1_X& data) = 0;
  virtual bool consume(RobotModeData_V3_0__1& data) = 0;
  virtual bool consume(RobotModeData_V3_2& data) = 0;
};

class URMessagePacketConsumer : public IConsumer<MessagePacket>
{
public:
  virtual bool consume(shared_ptr<MessagePacket> packet)
  {
    return packet->consumeWith(*this);
  }

  virtual bool consume(VersionMessage& message) = 0;
};