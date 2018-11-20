#include "ur_modern_driver/ros/mb_publisher.h"

inline void appendAnalog(std::vector<ur_msgs::Analog>& vec, double val, uint8_t pin)
{
  ur_msgs::Analog ana;
  ana.pin = pin;
  ana.state = val;
  vec.push_back(ana);
}

void MBPublisher::publish(ur_msgs::IOStates& io_msg, SharedMasterBoardData& data)
{
  appendAnalog(io_msg.analog_in_states, data.analog_input0, 0);
  appendAnalog(io_msg.analog_in_states, data.analog_input1, 1);
  appendAnalog(io_msg.analog_out_states, data.analog_output0, 0);
  appendAnalog(io_msg.analog_out_states, data.analog_output1, 1);

  io_pub_.publish(io_msg);
}

void MBPublisher::publishRobotStatus(industrial_msgs::RobotStatus& status, const SharedRobotModeData& data) const
{
  // note that this is true as soon as the drives are powered,
  // even if the breakes are still closed
  // which is in slight contrast to the comments in the
  // message definition
  status.drives_powered.val = data.robot_power_on;

  status.e_stopped.val = data.emergency_stopped;

  // I found no way to reliably get information if the robot is moving
  // data.programm_running would be true when using this driver to move the robot
  // but it would also be true when another programm is running that might not
  // in fact contain any movement commands
  status.in_motion.val = industrial_msgs::TriState::UNKNOWN;

  // the error code, if any, is not transmitted by this protocol
  // it can and should be fetched seperately
  status.error_code = 0;

  // note that e-stop is handled by a seperate variable
  status.in_error.val = data.protective_stopped;

  status_pub_.publish(status);
}

void MBPublisher::publishRobotStatus(const RobotModeData_V1_X& data) const
{
  industrial_msgs::RobotStatus msg;

  if (data.robot_mode == robot_mode_V1_X::ROBOT_FREEDRIVE_MODE)
    msg.mode.val = industrial_msgs::RobotMode::MANUAL;
  else
    msg.mode.val = industrial_msgs::RobotMode::AUTO;

  // todo: verify that this correct, there is also ROBOT_READY_MODE
  msg.motion_possible.val = (data.robot_mode == robot_mode_V1_X::ROBOT_RUNNING_MODE) ? industrial_msgs::TriState::ON :
                                                                                       industrial_msgs::TriState::OFF;

  publishRobotStatus(msg, data);
}

void MBPublisher::publishRobotStatus(const RobotModeData_V3_0__1& data) const
{
  industrial_msgs::RobotStatus msg;

  msg.motion_possible.val =
      (data.robot_mode == robot_mode_V3_X::RUNNING) ? industrial_msgs::TriState::ON : industrial_msgs::TriState::OFF;

  if (data.control_mode == robot_control_mode_V3_X::TEACH)
    msg.mode.val = industrial_msgs::RobotMode::MANUAL;
  else
    msg.mode.val = industrial_msgs::RobotMode::AUTO;

  publishRobotStatus(msg, data);
}

bool MBPublisher::consume(MasterBoardData_V1_X& data)
{
  ur_msgs::IOStates io_msg;
  appendDigital(io_msg.digital_in_states, data.digital_input_bits);
  appendDigital(io_msg.digital_out_states, data.digital_output_bits);
  publish(io_msg, data);
  return true;
}
bool MBPublisher::consume(MasterBoardData_V3_0__1& data)
{
  ur_msgs::IOStates io_msg;
  appendDigital(io_msg.digital_in_states, data.digital_input_bits);
  appendDigital(io_msg.digital_out_states, data.digital_output_bits);
  publish(io_msg, data);
  return true;
}
bool MBPublisher::consume(MasterBoardData_V3_2& data)
{
  consume(static_cast<MasterBoardData_V3_0__1&>(data));
  return true;
}

bool MBPublisher::consume(RobotModeData_V1_X& data)
{
  publishRobotStatus(data);
  return true;
}
bool MBPublisher::consume(RobotModeData_V3_0__1& data)
{
  publishRobotStatus(data);
  return true;
}
bool MBPublisher::consume(RobotModeData_V3_2& data)
{
  publishRobotStatus(data);
  return true;
}
