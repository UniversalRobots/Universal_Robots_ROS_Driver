#include "ur_modern_driver/ros/controller.h"

ROSController::ROSController(URCommander &commander, std::vector<std::string> &joint_names, double max_vel_change)
  : controller_(this, nh_)
  , joint_interface_(joint_names)
  , wrench_interface_()
  , position_interface_(commander, joint_interface_, joint_names)
  , velocity_interface_(commander, joint_interface_, joint_names, max_vel_change)
{
  registerInterface(&joint_interface_);
  registerInterface(&wrench_interface_);
  registerControllereInterface(&position_interface_);
  registerControllereInterface(&velocity_interface_);
}

void ROSController::setupConsumer()
{
  lastUpdate_ = ros::Time::now();
}

void ROSController::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list, const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  if (active_interface_ != nullptr && stop_list.size() > 0)
  {
    LOG_INFO("Stopping active interface");
    active_interface_->stop();
    active_interface_ = nullptr;
  }

  for (auto const& ci : start_list)
  {
    auto ait = available_interfaces_.find(ci.hardware_interface);

    if (ait == available_interfaces_.end())
      continue;

    auto new_interface = ait->second;

    LOG_INFO("Starting %s", ci.hardware_interface.c_str());
    active_interface_ = new_interface;
    new_interface->start();

    return;
  }

  LOG_WARN("Failed to start interface!");
}

bool ROSController::write()
{
  if (active_interface_ == nullptr)
    return true;

  return active_interface_->write();
}

void ROSController::read(RTShared& packet)
{
  joint_interface_.update(packet);
  wrench_interface_.update(packet);
}


bool ROSController::update(RTShared& state)
{
  auto time = ros::Time::now();
  auto diff = time - lastUpdate_;
  lastUpdate_ = time;

  read(state);
  controller_.update(time, diff);

  //emergency stop and such should not kill the pipeline
  //but still prevent writes
  if(!service_enabled_)
    return true;

  return write();
}

void ROSController::onRobotStateChange(RobotState state)
{
  service_enabled_ = (state == RobotState::Running);
}