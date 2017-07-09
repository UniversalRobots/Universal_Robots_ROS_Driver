#include "ur_modern_driver/ros/service_stopper.h"

ServiceStopper::ServiceStopper(std::vector<Service*> services)
  : enable_service_(nh_.advertiseService("ur_driver/robot_enable", &ServiceStopper::enableCallback, this))
  , services_(services)
  , last_state_(RobotState::Error)
{
  // enable_all();
}

bool ServiceStopper::enableCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
  notify_all(RobotState::Running);
  return true;
}

void ServiceStopper::notify_all(RobotState state)
{
  if (last_state_ == state)
    return;

  for (auto const service : services_)
  {
    service->onRobotStateChange(state);
  }

  last_state_ = state;
}

bool ServiceStopper::handle(SharedRobotModeData& data, bool error)
{
  if (data.emergency_stopped)
  {
    notify_all(RobotState::EmergencyStopped);
  }
  else if (data.protective_stopped)
  {
    notify_all(RobotState::ProtectiveStopped);
  }
  else if (error)
  {
    notify_all(RobotState::Error);
  }

  return true;
}