#include "ur_modern_driver/ros/service_stopper.h"

ServiceStopper::ServiceStopper(std::vector<Service*> services)
  : enable_service_(nh_.advertiseService("ur_driver/robot_enable", &ServiceStopper::enableCallback, this))
  , services_(services)
  , last_state_(RobotState::Error)
  , activation_mode_(ActivationMode::Never)
{
  std::string mode;
  ros::param::param("~require_activation", mode, std::string("Never"));
  if (mode == "Always")
  {
    activation_mode_ = ActivationMode::Always;
  }
  else if (mode == "OnStartup")
  {
    activation_mode_ = ActivationMode::OnStartup;
  }
  else
  {
    if (mode != "Never")
    {
      LOG_WARN("Found invalid value for param require_activation: '%s'\nShould be one of Never, OnStartup, Always", mode.c_str());
      mode = "Never";
    }
    notify_all(RobotState::Running);
  }

  LOG_INFO("Service 'ur_driver/robot_enable' activation mode: %s", mode.c_str());
}

bool ServiceStopper::enableCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp)
{
  // After the startup call OnStartup and Never behave the same
  if (activation_mode_ == ActivationMode::OnStartup)
    activation_mode_ = ActivationMode::Never;
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
  else if (activation_mode_ == ActivationMode::Never)
  {
    // No error encountered, the user requested automatic reactivation
    notify_all(RobotState::Running);
  }

  return true;
}
