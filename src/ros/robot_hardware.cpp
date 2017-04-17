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
