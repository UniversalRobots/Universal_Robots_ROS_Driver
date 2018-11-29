#pragma once

#include <ros/ros.h>
#include <string>

#include "std_msgs/String.h"
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/ros/service_stopper.h"
#include "ur_modern_driver/ur/commander.h"

class URScriptHandler : public Service
{
private:
  ros::NodeHandle nh_;
  URCommander &commander_;
  ros::Subscriber urscript_sub_;
  RobotState state_;

public:
  URScriptHandler(URCommander &commander);
  void urscriptInterface(const std_msgs::String::ConstPtr &msg);
  void onRobotStateChange(RobotState state);
};
