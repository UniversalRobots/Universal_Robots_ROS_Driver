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
#include "ur_rtde_driver/ur/tool_communication.h"
#include <ur_rtde_driver/exceptions.h>

#include <Eigen/Geometry>

namespace ur_driver
{
HardwareInterface::HardwareInterface()
  : joint_position_command_({ 0, 0, 0, 0, 0, 0 })
  , joint_positions_{ { 0, 0, 0, 0, 0, 0 } }
  , joint_velocities_{ { 0, 0, 0, 0, 0, 0 } }
  , joint_efforts_{ { 0, 0, 0, 0, 0, 0 } }
  , joint_names_(6)
  , runtime_state_(static_cast<uint32_t>(rtde_interface::RUNTIME_STATE::STOPPED))
  , position_controller_running_(false)
  , pausing_state_(PausingState::RUNNING)
  , pausing_ramp_up_increment_(0.01)
{
}

bool HardwareInterface ::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  joint_velocities_ = { { 0, 0, 0, 0, 0, 0 } };
  joint_efforts_ = { { 0, 0, 0, 0, 0, 0 } };
  std::string robot_ip = robot_hw_nh.param<std::string>("robot_ip", "192.168.56.101");
  std::string script_filename;
  std::string recipe_filename;
  if (!robot_hw_nh.getParam("script_file", script_filename))
  {
    ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("script_file") << " not given.");
    return false;
  }

  if (!robot_hw_nh.getParam("recipe_file", recipe_filename))
  {
    ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("recipe_file") << " not given.");
    return false;
  }

  tcp_link_ = robot_hw_nh.param<std::string>("tcp_link", "tool0");
  program_state_pub_ = robot_hw_nh.advertise<std_msgs::Bool>("robot_program_running", 10, true);

  bool use_tool_communication = robot_hw_nh.param<bool>("use_tool_communication", "false");
  std::unique_ptr<ToolCommSetup> tool_comm_setup;
  if (use_tool_communication)
  {
    tool_comm_setup.reset(new ToolCommSetup());

    using ToolVoltageT = std::underlying_type<ToolVoltage>::type;
    ToolVoltageT tool_voltage;
    if (!robot_hw_nh.getParam("tool_voltage", tool_voltage))
    {
      ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("tool_voltage") << " not given.");
      return false;
    }
    tool_comm_setup->setToolVoltage(static_cast<ToolVoltage>(tool_voltage));

    using ParityT = std::underlying_type<Parity>::type;
    ParityT parity;
    if (!robot_hw_nh.getParam("tool_parity", parity))
    {
      ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("tool_parity") << " not given.");
      return false;
    }

    int baud_rate;
    if (!robot_hw_nh.getParam("tool_baud_rate", baud_rate))
    {
      ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("tool_baud_rate") << " not given.");
      return false;
    }
    tool_comm_setup->setBaudRate(baud_rate);

    int stop_bits;
    if (!robot_hw_nh.getParam("tool_stop_bits", stop_bits))
    {
      ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("tool_stop_bits") << " not given.");
      return false;
    }
    tool_comm_setup->setStopBits(stop_bits);

    int rx_idle_chars;
    if (!robot_hw_nh.getParam("tool_rx_idle_chars", rx_idle_chars))
    {
      ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("tool_rx_idle_chars") << " not given.");
      return false;
    }
    tool_comm_setup->setRxIdleChars(rx_idle_chars);
    tool_comm_setup->setParity(static_cast<Parity>(parity));

    int tx_idle_chars;
    if (!robot_hw_nh.getParam("tool_tx_idle_chars", tx_idle_chars))
    {
      ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("tool_tx_idle_chars") << " not given.");
      return false;
    }
    tool_comm_setup->setTxIdleChars(tx_idle_chars);
  }

  ROS_INFO_STREAM("Initializing urdriver");
  try
  {
    ur_driver_.reset(new UrDriver(robot_ip, script_filename, recipe_filename,
                                  std::bind(&HardwareInterface::handleRobotProgramState, this, std::placeholders::_1),
                                  std::move(tool_comm_setup)));
  }
  catch (ur_driver::ToolCommNotAvailable& e)
  {
    ROS_FATAL_STREAM(e.what() << " See parameter '" << robot_hw_nh.resolveName("use_tool_communication") << "'.");
    return false;
  }
  catch (ur_driver::UrException& e)
  {
    ROS_FATAL_STREAM(e.what());
    return false;
  }

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
    ROS_INFO_STREAM("Registing handles for joint " << joint_names_[i]);
    // Create joint state interface for all joints
    js_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_positions_[i],
                                                                      &joint_velocities_[i], &joint_efforts_[i]));

    // Create joint position control interface
    // pj_interface_.registerHandle(
    // hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]));
    spj_interface_.registerHandle(ur_controllers::ScaledJointHandle(
        js_interface_.getHandle(joint_names_[i]), &joint_position_command_[i], &speed_scaling_combined_));
  }

  speedsc_interface_.registerHandle(
      ur_controllers::SpeedScalingHandle("speed_scaling_factor", &speed_scaling_combined_));

  fts_interface_.registerHandle(hardware_interface::ForceTorqueSensorHandle(
      "wrench", tcp_link_, fts_measurements_.begin(), fts_measurements_.begin() + 3));

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&spj_interface_);
  // registerInterface(&pj_interface_);
  registerInterface(&speedsc_interface_);
  registerInterface(&fts_interface_);

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
    if (!data_pkg->getData("target_speed_fraction", target_speed_fraction_))
    {
      // This throwing should never happen unless misconfigured
      throw std::runtime_error("Did not find target_speed_fraction_ in data sent from robot. This should not happen!");
    }
    if (!data_pkg->getData("speed_scaling", speed_scaling_))
    {
      // This throwing should never happen unless misconfigured
      throw std::runtime_error("Did not find speed_scaling in data sent from robot. This should not happen!");
    }
    if (!data_pkg->getData("runtime_state", runtime_state_))
    {
      // This throwing should never happen unless misconfigured
      throw std::runtime_error("Did not find speed_scaling in data sent from robot. This should not happen!");
    }
    if (!data_pkg->getData("actual_TCP_force", fts_measurements_))
    {
      // This throwing should never happen unless misconfigured
      throw std::runtime_error("Did not find actual_TCP_force in data sent from robot. This should not happen!");
    }
    if (!data_pkg->getData("actual_TCP_pose", tcp_pose_))
    {
      // This throwing should never happen unless misconfigured
      throw std::runtime_error("Did not find actual_TCP_pose in data sent from robot. This should not happen!");
    }

    // Transform fts measurements to tool frame
    transform_force_torque();

    // pausing state follows runtime state when pausing
    if (runtime_state_ == static_cast<uint32_t>(rtde_interface::RUNTIME_STATE::PAUSED))
    {
      pausing_state_ = PausingState::PAUSED;
    }
    // When the robot resumed program execution and pausing state was PAUSED, we enter RAMPUP
    else if (runtime_state_ == static_cast<uint32_t>(rtde_interface::RUNTIME_STATE::PLAYING) &&
             pausing_state_ == PausingState::PAUSED)
    {
      speed_scaling_combined_ = 0.0;
      pausing_state_ = PausingState::RAMPUP;
    }

    if (pausing_state_ == PausingState::RAMPUP)
    {
      ROS_INFO_STREAM("Ramping up speed scaling");
      double speed_scaling_ramp = speed_scaling_combined_ + pausing_ramp_up_increment_;
      speed_scaling_combined_ = std::min(speed_scaling_ramp, speed_scaling_ * target_speed_fraction_);

      if (speed_scaling_ramp > speed_scaling_ * target_speed_fraction_)
      {
        pausing_state_ = PausingState::RUNNING;
      }
    }
    else if (runtime_state_ == static_cast<uint32_t>(rtde_interface::RUNTIME_STATE::RESUMING))
    {
      // We have to keep speed scaling on ROS side at 0 during RESUMING to prevent controllers from
      // continuing to interpolate
      speed_scaling_combined_ = 0.0;
    }
    else
    {
      // Normal case
      speed_scaling_combined_ = speed_scaling_ * target_speed_fraction_;
    }
  }
  else
  {
    ROS_ERROR("Could not get fresh data package from robot");
  }
}

void HardwareInterface ::write(const ros::Time& time, const ros::Duration& period)
{
  if (position_controller_running_ &&
      (runtime_state_ == static_cast<uint32_t>(rtde_interface::RUNTIME_STATE::PLAYING) ||
       runtime_state_ == static_cast<uint32_t>(rtde_interface::RUNTIME_STATE::PAUSING)))
  {
    // ROS_INFO_STREAM("Writing command: " << joint_position_command_);
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
      if (resource_it.hardware_interface == "ur_controllers::ScaledPositionJointInterface")
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

void HardwareInterface ::transform_force_torque()
{
  double tcp_angle = std::sqrt(std::pow(tcp_pose_[3], 2) + std::pow(tcp_pose_[4], 2) + std::pow(tcp_pose_[5], 2));

  Eigen::Vector3d rotation_vec;
  rotation_vec = Eigen::Vector3d(tcp_pose_[3], tcp_pose_[4], tcp_pose_[5]).normalized();
  Eigen::AngleAxisd rotation;
  if (tcp_angle < 1e-16)
  {
    rotation = Eigen::AngleAxisd::Identity();
  }
  else
  {
    rotation = Eigen::AngleAxisd(tcp_angle, rotation_vec);
  }

  Eigen::Vector3d tcp_force = Eigen::Vector3d(fts_measurements_[0], fts_measurements_[1], fts_measurements_[2]);
  Eigen::Vector3d tcp_torque = Eigen::Vector3d(fts_measurements_[3], fts_measurements_[4], fts_measurements_[5]);

  tcp_force = rotation.inverse() * tcp_force;
  tcp_torque = rotation.inverse() * tcp_torque;

  fts_measurements_ = { tcp_force[0], tcp_force[1], tcp_force[2], tcp_torque[0], tcp_torque[1], tcp_torque[2] };
}

bool HardwareInterface ::isRobotProgramRunning() const
{
  return robot_program_running_;
}

void HardwareInterface ::handleRobotProgramState(bool program_running)
{
  robot_program_running_ = program_running;
  std_msgs::Bool msg;
  msg.data = robot_program_running_;
  program_state_pub_.publish(msg);
}

}  // namespace ur_driver
