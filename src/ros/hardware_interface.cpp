// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Mauch mauch@fzi.de
 * \date    2019-04-11
 *
 */
//----------------------------------------------------------------------

#include "ur_rtde_driver/ros/hardware_interface.h"

namespace ur_driver
{
HardwareInterface::HardwareInterface()
  : joint_position_command_{ 0, 0, 0, 0, 0, 0 }
  , joint_positions_{ 0, 0, 0, 0, 0, 0 }
  , joint_velocities_{ 0, 0, 0, 0, 0, 0 }
  , joint_efforts_{ 0, 0, 0, 0, 0, 0 }
  , joint_names_(6)
  , position_controller_running_(false)

{
}

bool HardwareInterface ::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  joint_velocities_ = { 0, 0, 0, 0, 0, 0 };
  joint_efforts_ = { 0, 0, 0, 0, 0, 0 };
  std::string ROBOT_IP = robot_hw_nh.param<std::string>("robot_ip", "192.168.56.101");

  ROS_INFO_STREAM("Initializing urdriver");
  ur_driver_.reset(new UrDriver(ROBOT_IP));

  if (!root_nh.getParam("hardware_interface/joints", joint_names_))
  {
    ROS_ERROR_STREAM("Cannot find required parameter " << root_nh.resolveName("hardware_interface/joints")
                                                       << " on the parameter server.");
    throw std::runtime_error("Cannot find required parameter "
                             "'controller_joint_names' on the parameter server.");
  }

  // Create ros_control interfaces
  for (std::size_t i = 0; i < joint_positions_.size(); ++i)
  {
    // Create joint state interface for all joints
    js_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_positions_[i],
                                                                      &joint_velocities_[i], &joint_efforts_[i]));

    // Create joint position control interface
    pj_interface_.registerHandle(
        hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]));
  }

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&pj_interface_);

  ROS_INFO_STREAM_NAMED("hardware_interface", "Loaded ur_rtde_driver hardware_interface");

  return true;
}

void HardwareInterface ::read(const ros::Time& time, const ros::Duration& period)
{
  std::unique_ptr<rtde_interface::DataPackage> data_pkg = ur_driver_->getDataPackage();
  if (data_pkg)
  {
    if (!data_pkg->getData("actual_q", joint_positions_))
    {
      // This throwing should never happen unless misconfigured
      throw std::runtime_error("Did not find joint position in data sent from robot. This should not happen!");
    }
    if (!data_pkg->getData("actual_qd", joint_velocities_))
    {
      // This throwing should never happen unless misconfigured
      throw std::runtime_error("Did not find joint velocity in data sent from robot. This should not happen!");
    }
  }
  else
  {
    ROS_ERROR("Could not get fresh data package from robot");
  }
}

void HardwareInterface ::write(const ros::Time& time, const ros::Duration& period)
{
  if (position_controller_running_)
  {
    ur_driver_->writeJointCommand(joint_position_command_);
  }
}

bool HardwareInterface ::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                       const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  // TODO: Implement
  return true;
}

void HardwareInterface ::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                  const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  position_controller_running_ = false;
  for (auto& controller_it : start_list)
  {
    for (auto& resource_it : controller_it.claimed_resources)
    {
      if (resource_it.hardware_interface == "hardware_interface::PositionJointInterface")
      {
        position_controller_running_ = true;
      }
    }
  }
}

uint32_t HardwareInterface ::getControlFrequency() const
{
  if (ur_driver_ != nullptr)
  {
    return ur_driver_->getControlFrequency();
  }
  // TODO: Do this nicer than throwing an exception
  throw std::runtime_error("ur_driver is not yet initialized");
}
}  // namespace ur_driver
