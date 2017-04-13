#pragma once
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/pipeline.h"
#include "ur_modern_driver/ur/rt_state.h"
#include "ur_modern_driver/ur/commander.h"
#include "ur_modern_driver/ros/robot_hardware.h"

class ROSController : public URRTPacketConsumer
{
private:
  ros::NodeHandle nh_;
  ros::Time lastUpdate_;
  RobotHardware robot_;
  controller_manager::ControllerManager controller_;

public:
  ROSController(URCommander& commander,  std::vector<std::string>& joint_names)
    : robot_(commander, joint_names)
    , controller_(&robot_, nh_)
  {
  }

  virtual void setupConsumer()
  {
    lastUpdate_ = ros::Time::now();
  }

  bool handle(RTShared& state)
  {
    auto time = ros::Time::now();
    auto diff = time - lastUpdate_;
    lastUpdate_ = time;

    robot_.read(state);
    controller_.update(time, diff);
    robot_.write();
    //todo: return result of write
    return true;
  }


  virtual bool consume(RTState_V1_6__7& state)
  {
    return handle(state);
  }
  virtual bool consume(RTState_V1_8& state)
  {
    return handle(state);
  }
  virtual bool consume(RTState_V3_0__1& state)
  {
    return handle(state);
  }
  virtual bool consume(RTState_V3_2__3& state)
  {
    return handle(state);
  }
};