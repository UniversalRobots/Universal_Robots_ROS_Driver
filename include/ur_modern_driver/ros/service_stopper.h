#pragma once
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/ur/consumer.h"

enum class RobotState
{
  Running,
  Error,
  EmergencyStopped,
  ProtectiveStopped
};

enum class ActivationMode
{
  Never,
  Always,
  OnStartup
};

class Service
{
public:
  virtual void onRobotStateChange(RobotState state) = 0;
};

class ServiceStopper : public URStatePacketConsumer
{
private:
  ros::NodeHandle nh_;
  ros::ServiceServer enable_service_;
  std::vector<Service*> services_;
  RobotState last_state_;
  ActivationMode activation_mode_;

  void notify_all(RobotState state);
  bool handle(SharedRobotModeData& data, bool error);
  bool enableCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);

public:
  ServiceStopper(std::vector<Service*> services);

  virtual bool consume(RobotModeData_V1_X& data)
  {
    return handle(data, data.robot_mode != robot_mode_V1_X::ROBOT_RUNNING_MODE);
  }
  virtual bool consume(RobotModeData_V3_0__1& data)
  {
    return handle(data, data.robot_mode != robot_mode_V3_X::RUNNING);
  }
  virtual bool consume(RobotModeData_V3_2& data)
  {
    return handle(data, data.robot_mode != robot_mode_V3_X::RUNNING);
  }

  // unused
  virtual bool consume(MasterBoardData_V1_X& data)
  {
    return true;
  }
  virtual bool consume(MasterBoardData_V3_0__1& data)
  {
    return true;
  }
  virtual bool consume(MasterBoardData_V3_2& data)
  {
    return true;
  }
};
