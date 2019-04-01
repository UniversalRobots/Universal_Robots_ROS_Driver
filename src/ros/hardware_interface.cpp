/*
 * Copyright 2017, 2018 Simon Rasmussen (refactor)
 *
 * Copyright 2015, 2016 Thomas Timm Andersen (original version)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ur_modern_driver/ros/hardware_interface.h"
#include "ur_modern_driver/log.h"

const std::string JointInterface::INTERFACE_NAME = "hardware_interface::JointStateInterface";
JointInterface::JointInterface(std::vector<std::string> &joint_names)
{
  for (size_t i = 0; i < 6; i++)
  {
    registerHandle(hardware_interface::JointStateHandle(joint_names[i], &positions_[i], &velocities_[i], &efforts_[i]));
  }
}

void JointInterface::update(RTShared &packet)
{
  positions_ = packet.q_actual;
  velocities_ = packet.qd_actual;
  efforts_ = packet.i_actual;
}

const std::string WrenchInterface::INTERFACE_NAME = "hardware_interface::ForceTorqueSensorInterface";
WrenchInterface::WrenchInterface(std::string tcp_link)
{
  registerHandle(hardware_interface::ForceTorqueSensorHandle("wrench", tcp_link, tcp_.begin(), tcp_.begin() + 3));
}

void WrenchInterface::update(RTShared &packet)
{
  tcp_ = packet.tcp_force;
}

const std::string VelocityInterface::INTERFACE_NAME = "hardware_interface::VelocityJointInterface";
VelocityInterface::VelocityInterface(URCommander &commander, hardware_interface::JointStateInterface &js_interface,
                                     std::vector<std::string> &joint_names, double max_vel_change)
  : commander_(commander), max_vel_change_(max_vel_change), prev_velocity_cmd_({ 0, 0, 0, 0, 0, 0 })
{
  for (size_t i = 0; i < 6; i++)
  {
    registerHandle(JointHandle(js_interface.getHandle(joint_names[i]), &velocity_cmd_[i]));
  }
}

bool VelocityInterface::write()
{
  for (size_t i = 0; i < 6; i++)
  {
    // clamp value to ±max_vel_change
    double prev = prev_velocity_cmd_[i];
    double lo = prev - max_vel_change_;
    double hi = prev + max_vel_change_;
    prev_velocity_cmd_[i] = std::max(lo, std::min(velocity_cmd_[i], hi));
  }
  return commander_.speedj(prev_velocity_cmd_, max_vel_change_);
}

void VelocityInterface::reset()
{
  for (auto &val : prev_velocity_cmd_)
  {
    val = 0;
  }
}

const std::string PositionInterface::INTERFACE_NAME = "hardware_interface::PositionJointInterface";
PositionInterface::PositionInterface(TrajectoryFollower &follower,
                                     hardware_interface::JointStateInterface &js_interface,
                                     std::vector<std::string> &joint_names)
  : follower_(follower)
{
  for (size_t i = 0; i < 6; i++)
  {
    registerHandle(JointHandle(js_interface.getHandle(joint_names[i]), &position_cmd_[i]));
  }
}

bool PositionInterface::write()
{
  return follower_.execute(position_cmd_);
}

void PositionInterface::start()
{
  follower_.start();
}

void PositionInterface::stop()
{
  follower_.stop();
}
