#include "ur_modern_driver/ros/urscript_handler.h"
#include "ur_modern_driver/log.h"

URScriptHandler::URScriptHandler(URCommander& commander) : commander_(commander), state_(RobotState::Error)
{
  LOG_INFO("Initializing ur_driver/URScript subscriber");
  urscript_sub_ = nh_.subscribe("ur_driver/URScript", 1, &URScriptHandler::urscriptInterface, this);
  LOG_INFO("The ur_driver/URScript initialized");
}

void URScriptHandler::urscriptInterface(const std_msgs::String::ConstPtr& msg)
{
  LOG_INFO("Message received");
  std::string str(msg->data);
  if (str.back() != '\n')
    str.append("\n");

  switch (state_)
  {
    case RobotState::Running:
      if (!commander_.uploadProg(str))
      {
        LOG_ERROR("Program upload failed!");
      }
      break;
    case RobotState::EmergencyStopped:
      LOG_ERROR("Robot is emergency stopped");
      break;
    case RobotState::ProtectiveStopped:
      LOG_ERROR("Robot is protective stopped");
      break;
    case RobotState::Error:
      LOG_ERROR("Robot is not ready, check robot_mode");
      break;
    default:
      LOG_ERROR("Robot is in undefined state");
      break;
  }
}

void URScriptHandler::onRobotStateChange(RobotState state)
{
  state_ = state;
}
