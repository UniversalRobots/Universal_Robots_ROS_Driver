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