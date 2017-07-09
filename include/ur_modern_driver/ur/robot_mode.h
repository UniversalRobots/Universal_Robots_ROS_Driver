#pragma once

#include <inttypes.h>
#include <cstddef>
#include "ur_modern_driver/bin_parser.h"
#include "ur_modern_driver/types.h"
#include "ur_modern_driver/ur/state.h"

class SharedRobotModeData
{
public:
  virtual bool parseWith(BinParser& bp);

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

enum class robot_mode_V1_X : uint8_t
{
  ROBOT_RUNNING_MODE = 0,
  ROBOT_FREEDRIVE_MODE = 1,
  ROBOT_READY_MODE = 2,
  ROBOT_INITIALIZING_MODE = 3,
  ROBOT_SECURITY_STOPPED_MODE = 4,
  ROBOT_EMERGENCY_STOPPED_MODE = 5,
  ROBOT_FATAL_ERROR_MODE = 6,
  ROBOT_NO_POWER_MODE = 7,
  ROBOT_NOT_CONNECTED_MODE = 8,
  ROBOT_SHUTDOWN_MODE = 9,
  ROBOT_SAFEGUARD_STOP_MODE = 10
};

class RobotModeData_V1_X : public SharedRobotModeData, public StatePacket
{
public:
  virtual bool parseWith(BinParser& bp);
  virtual bool consumeWith(URStatePacketConsumer& consumer);

  robot_mode_V1_X robot_mode;
  double speed_fraction;

  static const size_t SIZE = SharedRobotModeData::SIZE + sizeof(uint8_t) + sizeof(robot_mode_V1_X) + sizeof(double);

  static_assert(RobotModeData_V1_X::SIZE == 24, "RobotModeData_V1_X has missmatched size");
};

enum class robot_mode_V3_X : uint8_t
{
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

enum class robot_control_mode_V3_X : uint8_t
{
  POSITION = 0,
  TEACH = 1,
  FORCE = 2,
  TORQUE = 3
};

class RobotModeData_V3_0__1 : public SharedRobotModeData, public StatePacket
{
public:
  virtual bool parseWith(BinParser& bp);
  virtual bool consumeWith(URStatePacketConsumer& consumer);

  robot_mode_V3_X robot_mode;
  robot_control_mode_V3_X control_mode;

  double target_speed_fraction;
  double speed_scaling;

  static const size_t SIZE = SharedRobotModeData::SIZE + sizeof(uint8_t) + sizeof(robot_mode_V3_X) +
                             sizeof(robot_control_mode_V3_X) + sizeof(double) + sizeof(double);

  static_assert(RobotModeData_V3_0__1::SIZE == 33, "RobotModeData_V3_0__1 has missmatched size");
};

class RobotModeData_V3_2 : public RobotModeData_V3_0__1
{
public:
  virtual bool parseWith(BinParser& bp);
  virtual bool consumeWith(URStatePacketConsumer& consumer);

  double target_speed_fraction_limit;

  static const size_t SIZE = RobotModeData_V3_0__1::SIZE + sizeof(double);

  static_assert(RobotModeData_V3_2::SIZE == 41, "RobotModeData_V3_2 has missmatched size");
};