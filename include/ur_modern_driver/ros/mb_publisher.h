#pragma once

#include <industrial_msgs/RobotStatus.h>
#include <ros/ros.h>
#include <ur_msgs/Analog.h>
#include <ur_msgs/Digital.h>
#include <ur_msgs/IOStates.h>

#include "ur_modern_driver/ur/consumer.h"

using namespace ros;

class MBPublisher : public URStatePacketConsumer
{
private:
  NodeHandle nh_;
  Publisher io_pub_;
  Publisher status_pub_;

  template <size_t N>
  inline void appendDigital(std::vector<ur_msgs::Digital>& vec, std::bitset<N> bits)
  {
    for (size_t i = 0; i < N; i++)
    {
      ur_msgs::Digital digi;
      digi.pin = static_cast<uint8_t>(i);
      digi.state = bits.test(i);
      vec.push_back(digi);
    }
  }

  void publish(ur_msgs::IOStates& io_msg, SharedMasterBoardData& data);
  void publishRobotStatus(industrial_msgs::RobotStatus& status, const SharedRobotModeData& data) const;
  void publishRobotStatus(const RobotModeData_V1_X& data) const;
  void publishRobotStatus(const RobotModeData_V3_0__1& data) const;

public:
  MBPublisher()
    : io_pub_(nh_.advertise<ur_msgs::IOStates>("ur_driver/io_states", 1))
    , status_pub_(nh_.advertise<industrial_msgs::RobotStatus>("ur_driver/robot_status", 1))
  {
  }

  virtual bool consume(MasterBoardData_V1_X& data);
  virtual bool consume(MasterBoardData_V3_0__1& data);
  virtual bool consume(MasterBoardData_V3_2& data);

  virtual bool consume(RobotModeData_V1_X& data);
  virtual bool consume(RobotModeData_V3_0__1& data);
  virtual bool consume(RobotModeData_V3_2& data);

  virtual void setupConsumer()
  {
  }
  virtual void teardownConsumer()
  {
  }
  virtual void stopConsumer()
  {
  }
};
