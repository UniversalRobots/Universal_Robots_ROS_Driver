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
