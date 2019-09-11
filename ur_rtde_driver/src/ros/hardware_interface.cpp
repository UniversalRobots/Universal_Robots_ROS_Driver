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
  , standard_analog_input_{ { 0, 0 } }
  , standard_analog_output_{ { 0, 0 } }
  , joint_names_(6)
  , runtime_state_(static_cast<uint32_t>(rtde_interface::RUNTIME_STATE::STOPPED))
  , position_controller_running_(false)
  , pausing_state_(PausingState::RUNNING)
  , pausing_ramp_up_increment_(0.01)
  , controllers_initialized_(false)
{
}

bool HardwareInterface ::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  joint_velocities_ = { { 0, 0, 0, 0, 0, 0 } };
  joint_efforts_ = { { 0, 0, 0, 0, 0, 0 } };
  robot_ip_ = robot_hw_nh.param<std::string>("robot_ip", "192.168.56.101");
  std::string script_filename;
  std::string output_recipe_filename;
  std::string input_recipe_filename;
  if (!robot_hw_nh.getParam("script_file", script_filename))
  {
    ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("script_file") << " not given.");
    return false;
  }

  if (!robot_hw_nh.getParam("output_recipe_file", output_recipe_filename))
  {
    ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("output_recipe_file") << " not given.");
    return false;
  }

  if (!robot_hw_nh.getParam("input_recipe_file", input_recipe_filename))
  {
    ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("input_recipe_file") << " not given.");
    return false;
  }
  std::string tf_prefix = robot_hw_nh.param<std::string>("tf_prefix", "");

  program_state_pub_ = robot_hw_nh.advertise<std_msgs::Bool>("robot_program_running", 10, true);
  tcp_transform_.header.frame_id = "base";
  tcp_transform_.child_frame_id = "tool0_controller";

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

  std::string calibration_checksum = robot_hw_nh.param<std::string>("kinematics/hash", "");
  ROS_INFO_STREAM("Initializing urdriver");
  try
  {
    ur_driver_.reset(new UrDriver(robot_ip_, script_filename, output_recipe_filename, input_recipe_filename,
                                  std::bind(&HardwareInterface::handleRobotProgramState, this, std::placeholders::_1),
                                  std::move(tool_comm_setup), calibration_checksum));
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
  command_sub_ = robot_hw_nh.subscribe("script_command", 1, &HardwareInterface::commandCallback, this);

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
    ROS_DEBUG_STREAM("Registing handles for joint " << joint_names_[i]);
    // Create joint state interface for all joints
    js_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_positions_[i],
                                                                      &joint_velocities_[i], &joint_efforts_[i]));

    // Create joint position control interface
    pj_interface_.registerHandle(
        hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]));
    spj_interface_.registerHandle(ur_controllers::ScaledJointHandle(
        js_interface_.getHandle(joint_names_[i]), &joint_position_command_[i], &speed_scaling_combined_));
  }

  speedsc_interface_.registerHandle(
      ur_controllers::SpeedScalingHandle("speed_scaling_factor", &speed_scaling_combined_));

  fts_interface_.registerHandle(hardware_interface::ForceTorqueSensorHandle(
      "wrench", "tool0_controller", fts_measurements_.begin(), fts_measurements_.begin() + 3));

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&spj_interface_);
  registerInterface(&pj_interface_);
  registerInterface(&speedsc_interface_);
  registerInterface(&fts_interface_);

  tcp_pose_pub_.reset(new realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>(root_nh, "/tf", 100));
  io_pub_.reset(new realtime_tools::RealtimePublisher<ur_msgs::IOStates>(robot_hw_nh, "io_states", 1));
  io_pub_->msg_.digital_in_states.resize(actual_dig_in_bits_.size());
  io_pub_->msg_.digital_out_states.resize(actual_dig_out_bits_.size());
  io_pub_->msg_.analog_in_states.resize(standard_analog_input_.size());
  io_pub_->msg_.analog_out_states.resize(standard_analog_output_.size());
  for (size_t i = 0; i < actual_dig_in_bits_.size(); ++i)
  {
    io_pub_->msg_.digital_in_states[i].pin = i;
  }
  for (size_t i = 0; i < actual_dig_out_bits_.size(); ++i)
  {
    io_pub_->msg_.digital_out_states[i].pin = i;
  }
  for (size_t i = 0; i < standard_analog_input_.size(); ++i)
  {
    io_pub_->msg_.analog_in_states[i].pin = i;
  }
  for (size_t i = 0; i < standard_analog_output_.size(); ++i)
  {
    io_pub_->msg_.analog_out_states[i].pin = i;
  }
  tool_data_pub_.reset(new realtime_tools::RealtimePublisher<ur_msgs::ToolDataMsg>(robot_hw_nh, "tool_data", 1));

  set_speed_slider_srv_ = robot_hw_nh.advertiseService("set_speed_slider", &HardwareInterface::setSpeedSlider, this);
  set_io_srv_ = robot_hw_nh.advertiseService("set_io", &HardwareInterface::setIO, this);

  deactivate_srv_ = robot_hw_nh.advertiseService("hand_back_control", &HardwareInterface::stopControl, this);

  ROS_INFO_STREAM_NAMED("hardware_interface", "Loaded ur_rtde_driver hardware_interface");

  return true;
}

template <typename T>
void HardwareInterface::readData(const std::unique_ptr<rtde_interface::DataPackage>& data_pkg,
                                 const std::string& var_name, T& data)
{
  if (!data_pkg->getData(var_name, data))
  {
    // This throwing should never happen unless misconfigured
    std::string error_msg = "Did not find '" + var_name + "' in data sent from robot. This should not happen!";
    throw std::runtime_error(error_msg);
  }
}

template <typename T, size_t N>
void HardwareInterface::readBitsetData(const std::unique_ptr<rtde_interface::DataPackage>& data_pkg,
                                       const std::string& var_name, std::bitset<N>& data)
{
  if (!data_pkg->getData<T, N>(var_name, data))
  {
    // This throwing should never happen unless misconfigured
    std::string error_msg = "Did not find '" + var_name + "' in data sent from robot. This should not happen!";
    throw std::runtime_error(error_msg);
  }
}

void HardwareInterface ::read(const ros::Time& time, const ros::Duration& period)
{
  std::unique_ptr<rtde_interface::DataPackage> data_pkg = ur_driver_->getDataPackage();
  if (data_pkg)
  {
    readData(data_pkg, "actual_q", joint_positions_);
    readData(data_pkg, "actual_qd", joint_velocities_);
    readData(data_pkg, "target_speed_fraction", target_speed_fraction_);
    readData(data_pkg, "speed_scaling", speed_scaling_);
    readData(data_pkg, "runtime_state", runtime_state_);
    readData(data_pkg, "actual_TCP_force", fts_measurements_);
    readData(data_pkg, "actual_TCP_pose", tcp_pose_);
    readData(data_pkg, "standard_analog_input0", standard_analog_input_[0]);
    readData(data_pkg, "standard_analog_input1", standard_analog_input_[1]);
    readData(data_pkg, "standard_analog_output0", standard_analog_output_[0]);
    readData(data_pkg, "standard_analog_output1", standard_analog_output_[1]);
    readData(data_pkg, "tool_mode", tool_mode_);
    readData(data_pkg, "tool_analog_input0", tool_analog_input_[0]);
    readData(data_pkg, "tool_analog_input1", tool_analog_input_[1]);
    readData(data_pkg, "tool_output_voltage", tool_output_voltage_);
    readData(data_pkg, "tool_output_current", tool_output_current_);
    readData(data_pkg, "tool_temperature", tool_temperature_);
    readBitsetData<uint64_t>(data_pkg, "actual_digital_input_bits", actual_dig_in_bits_);
    readBitsetData<uint64_t>(data_pkg, "actual_digital_output_bits", actual_dig_out_bits_);
    readBitsetData<uint32_t>(data_pkg, "analog_io_types", analog_io_types_);
    readBitsetData<uint32_t>(data_pkg, "tool_analog_input_types", tool_analog_input_types_);

    publishIOData();
    publishToolData();

    // Transform fts measurements to tool frame
    extractToolPose(time);
    transformForceTorque();
    publishPose();

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
  if ((runtime_state_ == static_cast<uint32_t>(rtde_interface::RUNTIME_STATE::PLAYING) ||
       runtime_state_ == static_cast<uint32_t>(rtde_interface::RUNTIME_STATE::PAUSING)) &&
      robot_program_running_)
  {
    if (position_controller_running_)
    {
      ur_driver_->writeJointCommand(joint_position_command_);
    }
    else if (robot_program_running_)
    {
      ur_driver_->writeKeepalive();
    }
    else
    {
      ur_driver_->stopControl();
    }
  }
}

bool HardwareInterface ::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                       const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  bool ret_val = true;
  if (controllers_initialized_ && !isRobotProgramRunning() && !start_list.empty())
  {
    for (auto& controller : start_list)
    {
      if (!controller.claimed_resources.empty())
      {
        ROS_ERROR_STREAM("Robot control is currently inactive. Starting controllers that claim resources is currently "
                         "not possible. Not starting controller '"
                         << controller.name << "'");
        ret_val = false;
      }
    }
  }

  controllers_initialized_ = true;
  return ret_val;
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

void HardwareInterface ::transformForceTorque()
{
  tcp_force_.setValue(fts_measurements_[0], fts_measurements_[1], fts_measurements_[2]);
  tcp_torque_.setValue(fts_measurements_[3], fts_measurements_[4], fts_measurements_[5]);

  tf2::Quaternion rotation_quat;
  tf2::fromMsg(tcp_transform_.transform.rotation, rotation_quat);
  tcp_force_ = tf2::quatRotate(rotation_quat.inverse(), tcp_force_);
  tcp_torque_ = tf2::quatRotate(rotation_quat.inverse(), tcp_torque_);

  fts_measurements_ = { tcp_force_.x(),  tcp_force_.y(),  tcp_force_.z(),
                        tcp_torque_.x(), tcp_torque_.y(), tcp_torque_.z() };
}

bool HardwareInterface ::isRobotProgramRunning() const
{
  return robot_program_running_;
}

void HardwareInterface ::handleRobotProgramState(bool program_running)
{
  if (robot_program_running_ == false && program_running)
  {
    controller_reset_necessary_ = true;
  }
  robot_program_running_ = program_running;
  std_msgs::Bool msg;
  msg.data = robot_program_running_;
  program_state_pub_.publish(msg);
}

bool HardwareInterface ::shouldResetControllers()
{
  if (controller_reset_necessary_)
  {
    controller_reset_necessary_ = false;
    return true;
  }
  else
  {
    return false;
  }
}

void HardwareInterface ::extractToolPose(const ros::Time& timestamp)
{
  double tcp_angle = std::sqrt(std::pow(tcp_pose_[3], 2) + std::pow(tcp_pose_[4], 2) + std::pow(tcp_pose_[5], 2));

  tf2::Vector3 rotation_vec(tcp_pose_[3], tcp_pose_[4], tcp_pose_[5]);
  tf2::Quaternion rotation;
  if (tcp_angle > 1e-16)
  {
    rotation.setRotation(rotation_vec.normalized(), tcp_angle);
  }
  tcp_transform_.header.stamp = timestamp;
  tcp_transform_.transform.translation.x = tcp_pose_[0];
  tcp_transform_.transform.translation.y = tcp_pose_[1];
  tcp_transform_.transform.translation.z = tcp_pose_[2];

  tcp_transform_.transform.rotation = tf2::toMsg(rotation);
}

void HardwareInterface ::publishPose()
{
  if (tcp_pose_pub_)
  {
    if (tcp_pose_pub_->trylock())
    {
      tcp_pose_pub_->msg_.transforms.clear();
      tcp_pose_pub_->msg_.transforms.push_back(tcp_transform_);
      tcp_pose_pub_->unlockAndPublish();
    }
  }
}

void HardwareInterface::publishIOData()
{
  if (io_pub_)
  {
    if (io_pub_->trylock())
    {
      for (size_t i = 0; i < actual_dig_in_bits_.size(); ++i)
      {
        io_pub_->msg_.digital_in_states[i].state = actual_dig_in_bits_[i];
      }
      for (size_t i = 0; i < actual_dig_out_bits_.size(); ++i)
      {
        io_pub_->msg_.digital_out_states[i].state = actual_dig_out_bits_[i];
      }
      for (size_t i = 0; i < standard_analog_input_.size(); ++i)
      {
        io_pub_->msg_.analog_in_states[i].state = standard_analog_input_[i];
      }
      for (size_t i = 0; i < standard_analog_output_.size(); ++i)
      {
        io_pub_->msg_.analog_out_states[i].state = standard_analog_output_[i];
      }
      // TODO: Handle analog domain
      io_pub_->unlockAndPublish();
    }
  }
}

void HardwareInterface::publishToolData()
{
  if (tool_data_pub_)
  {
    if (tool_data_pub_->trylock())
    {
      tool_data_pub_->msg_.tool_mode = tool_mode_;
      tool_data_pub_->msg_.analog_input_range2 = tool_analog_input_types_[0];
      tool_data_pub_->msg_.analog_input_range3 = tool_analog_input_types_[1];
      tool_data_pub_->msg_.analog_input2 = tool_analog_input_[0];
      tool_data_pub_->msg_.analog_input2 = tool_analog_input_[1];
      tool_data_pub_->msg_.tool_output_voltage = tool_output_voltage_;
      tool_data_pub_->msg_.tool_current = tool_output_current_;
      tool_data_pub_->msg_.tool_temperature = tool_temperature_;
      tool_data_pub_->unlockAndPublish();
    }
  }
}

bool HardwareInterface::stopControl(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
  if (isRobotProgramRunning())
  {
    robot_program_running_ = false;
    res.success = true;
    res.message = "Deactivated control";
  }
  else
  {
    res.success = true;
    res.message = "No control active. Nothing to do.";
  }
  return true;
}

bool HardwareInterface::setSpeedSlider(ur_rtde_msgs::SetSpeedSliderRequest& req,
                                       ur_rtde_msgs::SetSpeedSliderResponse& res)
{
  if (req.data >= 0.01 && req.data <= 1.0 && ur_driver_ != nullptr)
  {
    res.success = ur_driver_->getRTDEWriter().sendSpeedSlider(req.data);
  }
  else
  {
    res.success = false;
  }
  return true;
}

bool HardwareInterface::setIO(ur_msgs::SetIORequest& req, ur_msgs::SetIOResponse& res)
{
  if (req.fun == req.FUN_SET_DIGITAL_OUT && ur_driver_ != nullptr)
  {
    if (req.pin <= 7)
    {
      res.success = ur_driver_->getRTDEWriter().sendStandardDigitalOutput(req.pin, req.state);
    }
    else if (req.pin <= 15)
    {
      res.success = ur_driver_->getRTDEWriter().sendConfigurableDigitalOutput(req.pin - 8, req.state);
    }
    else
    {
      res.success = ur_driver_->getRTDEWriter().sendToolDigitalOutput(req.pin - 16, req.state);
    }
  }
  else if (req.fun == req.FUN_SET_ANALOG_OUT && ur_driver_ != nullptr)
  {
    res.success = ur_driver_->getRTDEWriter().sendStandardAnalogOuput(req.pin, req.state);
  }
  else
  {
    res.success = false;
  }

  return true;
}

void HardwareInterface::commandCallback(const std_msgs::StringConstPtr& msg)
{
  std::string str = msg->data;
  if (str.back() != '\n')
  {
    str.append("\n");
  }

  if (ur_driver_ == nullptr)
  {
    throw std::runtime_error("Trying to use the ur_driver_ member before it is initialized. This should not happen, "
                             "please contact the package maintainer.");
  }

  if (ur_driver_->sendScript(str))
  {
    ROS_DEBUG_STREAM("Sent script to robot");
  }
  else
  {
    ROS_ERROR_STREAM("Error sending script to robot");
  }
}
}  // namespace ur_driver
