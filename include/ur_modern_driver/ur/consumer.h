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