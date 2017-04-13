#include "ur_modern_driver/ros/robot_hardware.h"

/*
bool RobotHardware::canSwitch(const std::list<ControllerInfo>& start_list,
                              const std::list<ControllerInfo>& stop_list) const
{

  bool running = active_interface_ != nullptr;
  size_t start_size = start_list.size();
  size_t stop_size = stop_list.size();


  for (auto const& ci : stop_list)
  {
    auto it = interfaces_.find(ci.hardware_interface);
    if(it == interfaces_.end() || it->second != active_interface_)
      return false;
  }
  
  for (auto const& ci : start_list)
  {
    auto it = interfaces_.find(ci.hardware_interface);
    //we can only start a controller that's already running if we stop it first
    if(it == interfaces_.end() || (it->second == active_interface_ && stop_size == 0))
      return false;
  }

  return true;
}
*/

void RobotHardware::doSwitch(const std::list<ControllerInfo>& start_list,
                             const std::list<ControllerInfo>& stop_list)
{
  if(active_interface_ != nullptr && stop_list.size() > 0)
  {
    active_interface_->stop();
    active_interface_ = nullptr;
  }

  for(auto const& ci : start_list)
  {
    auto it = interfaces_.find(ci.hardware_interface);
    if(it == interfaces_.end())
      continue;
    
    //we can only switch to one of the available interfaces
    if(std::find(available_interfaces_.begin(), available_interfaces_.end(), it->second) == available_interfaces_.end())
      continue;

    auto new_interface = static_cast<HardwareInterface*>(it->second);

    if(new_interface == nullptr)
      continue;
    
    LOG_INFO("Starting %s", ci.hardware_interface.c_str());
    active_interface_ = new_interface;
    new_interface->start();
    
    return;
  }
}

void RobotHardware::write()
{
  if(active_interface_ == nullptr)
    return;
  
  active_interface_->write();
}

void RobotHardware::read(RTShared& packet)
{
  joint_interface_.update(packet);
  wrench_interface_.update(packet);
}