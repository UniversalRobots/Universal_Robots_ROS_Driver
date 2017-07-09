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
  return true;
}
bool MBPublisher::consume(RobotModeData_V3_0__1& data)
{
  return true;
}
bool MBPublisher::consume(RobotModeData_V3_2& data)
{
  return true;
}