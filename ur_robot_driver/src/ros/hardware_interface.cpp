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
 * \author  Felix Exner exner@fzi.de
 * \date    2019-04-11
 *
 */
//----------------------------------------------------------------------
#include <pluginlib/class_list_macros.hpp>
#include "ur_robot_driver/ros/hardware_interface.h"
#include "ur_robot_driver/ur/tool_communication.h"
#include <ur_robot_driver/exceptions.h>

#include <Eigen/Geometry>

using industrial_robot_status_interface::RobotMode;
using industrial_robot_status_interface::TriState;
using namespace ur_driver::rtde_interface;

namespace ur_driver
{
// bitset mask is applied to robot safety status bits in order to determine 'in_error' state
static const std::bitset<11> in_error_bitset_(1 << toUnderlying(UrRtdeSafetyStatusBits::IS_PROTECTIVE_STOPPED) |
                                              1 << toUnderlying(UrRtdeSafetyStatusBits::IS_ROBOT_EMERGENCY_STOPPED) |
                                              1 << toUnderlying(UrRtdeSafetyStatusBits::IS_EMERGENCY_STOPPED) |
                                              1 << toUnderlying(UrRtdeSafetyStatusBits::IS_VIOLATION) |
                                              1 << toUnderlying(UrRtdeSafetyStatusBits::IS_FAULT) |
                                              1 << toUnderlying(UrRtdeSafetyStatusBits::IS_STOPPED_DUE_TO_SAFETY));

HardwareInterface::HardwareInterface()
  : joint_position_command_({ 0, 0, 0, 0, 0, 0 })
  , joint_velocity_command_({ 0, 0, 0, 0, 0, 0 })
  , joint_positions_{ { 0, 0, 0, 0, 0, 0 } }
  , joint_velocities_{ { 0, 0, 0, 0, 0, 0 } }
  , joint_efforts_{ { 0, 0, 0, 0, 0, 0 } }
  , standard_analog_input_{ { 0, 0 } }
  , standard_analog_output_{ { 0, 0 } }
  , joint_names_(6)
  , safety_mode_(ur_dashboard_msgs::SafetyMode::NORMAL)
  , runtime_state_(static_cast<uint32_t>(rtde_interface::RUNTIME_STATE::STOPPED))
  , position_controller_running_(false)
  , velocity_controller_running_(false)
  , pausing_state_(PausingState::RUNNING)
  , pausing_ramp_up_increment_(0.01)
  , controllers_initialized_(false)
{
}

bool HardwareInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  joint_velocities_ = { { 0, 0, 0, 0, 0, 0 } };
  joint_efforts_ = { { 0, 0, 0, 0, 0, 0 } };
  std::string script_filename;
  std::string output_recipe_filename;
  std::string input_recipe_filename;

  // The robot's IP address.
  if (!robot_hw_nh.getParam("robot_ip", robot_ip_))
  {
    ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("robot_ip_") << " not given.");
    return false;
  }

  // Port that will be opened to communicate between the driver and the robot controller.
  int reverse_port = robot_hw_nh.param("reverse_port", 50001);

  // The driver will offer an interface to receive the program's URScript on this port.
  int script_sender_port = robot_hw_nh.param("script_sender_port", 50002);

  // When the robot's URDF is being loaded with a prefix, we need to know it here, as well, in order
  // to publish correct frame names for frames reported by the robot directly.
  robot_hw_nh.param<std::string>("tf_prefix", tf_prefix_, "");

  // Path to the urscript code that will be sent to the robot.
  if (!robot_hw_nh.getParam("script_file", script_filename))
  {
    ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("script_file") << " not given.");
    return false;
  }

  // Path to the file containing the recipe used for requesting RTDE outputs.
  if (!robot_hw_nh.getParam("output_recipe_file", output_recipe_filename))
  {
    ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("output_recipe_file") << " not given.");
    return false;
  }

  // Path to the file containing the recipe used for requesting RTDE inputs.
  if (!robot_hw_nh.getParam("input_recipe_file", input_recipe_filename))
  {
    ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("input_recipe_file") << " not given.");
    return false;
  }

  bool headless_mode;
  // Start robot in headless mode. This does not require the 'External Control' URCap to be running
  // on the robot, but this will send the URScript to the robot directly. On e-Series robots this
  // requires the robot to run in 'remote-control' mode.
  if (!robot_hw_nh.getParam("headless_mode", headless_mode))
  {
    ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("headless_mode") << " not given.");
    return false;
  }

  // Enables non_blocking_read mode. Should only be used with combined_robot_hw. Disables error generated when read
  // returns without any data, sets the read timeout to zero, and synchronises read/write operations. Enabling this when
  // not used with combined_robot_hw can suppress important errors and affect real-time performance.
  robot_hw_nh.param("non_blocking_read", non_blocking_read_, false);

  // Specify gain for servoing to position in joint space.
  // A higher gain can sharpen the trajectory.
  int servoj_gain = robot_hw_nh.param("servoj_gain", 2000);
  if ((servoj_gain > 2000) || (servoj_gain < 100))
  {
    ROS_ERROR_STREAM("servoj_gain is " << servoj_gain << ", must be in range [100, 2000]");
    return false;
  }

  // Specify lookahead time for servoing to position in joint space.
  // A longer lookahead time can smooth the trajectory.
  double servoj_lookahead_time = robot_hw_nh.param("servoj_lookahead_time", 0.03);
  if ((servoj_lookahead_time > 0.2) || (servoj_lookahead_time < 0.03))
  {
    ROS_ERROR_STREAM("servoj_lookahead_time is " << servoj_lookahead_time << ", must be in range [0.03, 0.2]");
    return false;
  }

  // Whenever the runtime state of the "External Control" program node in the UR-program changes, a
  // message gets published here. So this is equivalent to the information whether the robot accepts
  // commands from ROS side.
  program_state_pub_ = robot_hw_nh.advertise<std_msgs::Bool>("robot_program_running", 10, true);
  tcp_transform_.header.frame_id = tf_prefix_ + "base";
  tcp_transform_.child_frame_id = tf_prefix_ + "tool0_controller";

  // Should the tool's RS485 interface be forwarded to the ROS machine? This is only available on
  // e-Series models. Setting this parameter to TRUE requires multiple other parameters to be set,as
  // well.
  bool use_tool_communication = robot_hw_nh.param<bool>("use_tool_communication", "false");
  std::unique_ptr<ToolCommSetup> tool_comm_setup;
  if (use_tool_communication)
  {
    tool_comm_setup.reset(new ToolCommSetup());

    using ToolVoltageT = std::underlying_type<ToolVoltage>::type;
    ToolVoltageT tool_voltage;
    // Tool voltage that will be set as soon as the UR-Program on the robot is started. Note: This
    // parameter is only evaluated, when the parameter "use_tool_communication" is set to TRUE.
    // Then, this parameter is required.
    if (!robot_hw_nh.getParam("tool_voltage", tool_voltage))
    {
      ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("tool_voltage") << " not given.");
      return false;
    }
    tool_comm_setup->setToolVoltage(static_cast<ToolVoltage>(tool_voltage));

    using ParityT = std::underlying_type<Parity>::type;
    ParityT parity;
    // Parity used for tool communication. Will be set as soon as the UR-Program on the robot is
    // started. Can be 0 (None), 1 (odd) and 2 (even).
    //
    // Note: This parameter is only evaluated, when the parameter "use_tool_communication"
    // is set to TRUE.  Then, this parameter is required.
    if (!robot_hw_nh.getParam("tool_parity", parity))
    {
      ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("tool_parity") << " not given.");
      return false;
    }

    int baud_rate;
    // Baud rate used for tool communication. Will be set as soon as the UR-Program on the robot is
    // started. See UR documentation for valid baud rates.
    //
    // Note: This parameter is only evaluated, when the parameter "use_tool_communication"
    // is set to TRUE.  Then, this parameter is required.
    if (!robot_hw_nh.getParam("tool_baud_rate", baud_rate))
    {
      ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("tool_baud_rate") << " not given.");
      return false;
    }
    tool_comm_setup->setBaudRate(baud_rate);

    int stop_bits;
    // Number of stop bits used for tool communication. Will be set as soon as the UR-Program on the robot is
    // started. Can be 1 or 2.
    //
    // Note: This parameter is only evaluated, when the parameter "use_tool_communication"
    // is set to TRUE.  Then, this parameter is required.
    if (!robot_hw_nh.getParam("tool_stop_bits", stop_bits))
    {
      ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("tool_stop_bits") << " not given.");
      return false;
    }
    tool_comm_setup->setStopBits(stop_bits);

    int rx_idle_chars;
    // Number of idle chars for the RX unit used for tool communication. Will be set as soon as the UR-Program on the
    // robot is started. Valid values: min=1.0, max=40.0
    //
    // Note: This parameter is only evaluated, when the parameter "use_tool_communication"
    // is set to TRUE.  Then, this parameter is required.
    if (!robot_hw_nh.getParam("tool_rx_idle_chars", rx_idle_chars))
    {
      ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("tool_rx_idle_chars") << " not given.");
      return false;
    }
    tool_comm_setup->setRxIdleChars(rx_idle_chars);
    tool_comm_setup->setParity(static_cast<Parity>(parity));

    int tx_idle_chars;
    // Number of idle chars for the TX unit used for tool communication. Will be set as soon as the UR-Program on the
    // robot is started. Valid values: min=0.0, max=40.0
    //
    // Note: This parameter is only evaluated, when the parameter "use_tool_communication"
    // is set to TRUE.  Then, this parameter is required.
    if (!robot_hw_nh.getParam("tool_tx_idle_chars", tx_idle_chars))
    {
      ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("tool_tx_idle_chars") << " not given.");
      return false;
    }
    tool_comm_setup->setTxIdleChars(tx_idle_chars);
  }

  // Hash of the calibration reported by the robot. This is used for validating the robot
  // description is using the correct calibration. If the robot's calibration doesn't match this
  // hash, an error will be printed. You can use the robot as usual, however Cartesian poses of the
  // endeffector might be inaccurate. See the "ur_calibration" package on help how to generate your
  // own hash matching your actual robot.
  std::string calibration_checksum = robot_hw_nh.param<std::string>("kinematics/hash", "");
  ROS_INFO_STREAM("Initializing dashboard client");
  ros::NodeHandle dashboard_nh(robot_hw_nh, "dashboard");
  dashboard_client_.reset(new DashboardClientROS(dashboard_nh, robot_ip_));
  ROS_INFO_STREAM("Initializing urdriver");
  try
  {
    ur_driver_.reset(new UrDriver(robot_ip_, script_filename, output_recipe_filename, input_recipe_filename,
                                  std::bind(&HardwareInterface::handleRobotProgramState, this, std::placeholders::_1),
                                  headless_mode, std::move(tool_comm_setup), calibration_checksum,
                                  (uint32_t)reverse_port, (uint32_t)script_sender_port, servoj_gain,
                                  servoj_lookahead_time, non_blocking_read_));
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

  // Send arbitrary script commands to this topic. Note: On e-Series the robot has to be in
  // remote-control mode.
  //
  // Sending scripts to this will stop program execution unless wrapped in a secondary program:
  //
  // sec myProgram():
  //   set_digital_out(0, True)
  // end
  command_sub_ = robot_hw_nh.subscribe("script_command", 1, &HardwareInterface::commandCallback, this);

  // Names of the joints. Usually, this is given in the controller config file.
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
    ROS_DEBUG_STREAM("Registering handles for joint " << joint_names_[i]);
    // Create joint state interface for all joints
    js_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_positions_[i],
                                                                      &joint_velocities_[i], &joint_efforts_[i]));

    // Create joint position control interface
    pj_interface_.registerHandle(
        hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]));
    vj_interface_.registerHandle(
        hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[i]), &joint_velocity_command_[i]));
    spj_interface_.registerHandle(ur_controllers::ScaledJointHandle(
        js_interface_.getHandle(joint_names_[i]), &joint_position_command_[i], &speed_scaling_combined_));
    svj_interface_.registerHandle(ur_controllers::ScaledJointHandle(
        js_interface_.getHandle(joint_names_[i]), &joint_velocity_command_[i], &speed_scaling_combined_));
  }

  speedsc_interface_.registerHandle(
      ur_controllers::SpeedScalingHandle("speed_scaling_factor", &speed_scaling_combined_));

  fts_interface_.registerHandle(hardware_interface::ForceTorqueSensorHandle(
      "wrench", tf_prefix_ + "tool0_controller", fts_measurements_.begin(), fts_measurements_.begin() + 3));

  robot_status_interface_.registerHandle(industrial_robot_status_interface::IndustrialRobotStatusHandle(
      "industrial_robot_status_handle", robot_status_resource_));

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&spj_interface_);
  registerInterface(&pj_interface_);
  registerInterface(&vj_interface_);
  registerInterface(&svj_interface_);
  registerInterface(&speedsc_interface_);
  registerInterface(&fts_interface_);
  registerInterface(&robot_status_interface_);

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

  robot_mode_pub_.reset(
      new realtime_tools::RealtimePublisher<ur_dashboard_msgs::RobotMode>(robot_hw_nh, "robot_mode", 1, true));
  safety_mode_pub_.reset(
      new realtime_tools::RealtimePublisher<ur_dashboard_msgs::SafetyMode>(robot_hw_nh, "safety_mode", 1, true));

  // Set the speed slider fraction used by the robot's execution. Values should be between 0 and 1.
  // Only set this smaller than 1 if you are using the scaled controllers (as by default) or you know what you're
  // doing. Using this with other controllers might lead to unexpected behaviors.
  set_speed_slider_srv_ = robot_hw_nh.advertiseService("set_speed_slider", &HardwareInterface::setSpeedSlider, this);

  // Service to set any of the robot's IOs
  set_io_srv_ = robot_hw_nh.advertiseService("set_io", &HardwareInterface::setIO, this);

  if (headless_mode)
  {
    // When in headless mode, this sends the URScript program to the robot for execution. Use this
    // after the program has been interrupted, e.g. by a protective- or EM-stop.
    resend_robot_program_srv_ =
        robot_hw_nh.advertiseService("resend_robot_program", &HardwareInterface::resendRobotProgram, this);
  }

  // Calling this service will make the "External Control" program node on the UR-Program return.
  deactivate_srv_ = robot_hw_nh.advertiseService("hand_back_control", &HardwareInterface::stopControl, this);

  // Calling this service will zero the robot's ftsensor. Note: On e-Series robots this will only
  // work when the robot is in remote-control mode.
  tare_sensor_srv_ = robot_hw_nh.advertiseService("zero_ftsensor", &HardwareInterface::zeroFTSensor, this);

  ur_driver_->startRTDECommunication();
  ROS_INFO_STREAM_NAMED("hardware_interface", "Loaded ur_robot_driver hardware_interface");

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

void HardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
  // set defaults
  robot_status_resource_.mode = RobotMode::UNKNOWN;
  robot_status_resource_.e_stopped = TriState::UNKNOWN;
  robot_status_resource_.drives_powered = TriState::UNKNOWN;
  robot_status_resource_.motion_possible = TriState::UNKNOWN;
  robot_status_resource_.in_motion = TriState::UNKNOWN;
  robot_status_resource_.in_error = TriState::UNKNOWN;
  robot_status_resource_.error_code = 0;

  std::unique_ptr<rtde_interface::DataPackage> data_pkg = ur_driver_->getDataPackage();
  if (data_pkg)
  {
    packet_read_ = true;
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
    readData(data_pkg, "robot_mode", robot_mode_);
    readData(data_pkg, "safety_mode", safety_mode_);
    readBitsetData<uint32_t>(data_pkg, "robot_status_bits", robot_status_bits_);
    readBitsetData<uint32_t>(data_pkg, "safety_status_bits", safety_status_bits_);
    readData(data_pkg, "actual_current", joint_efforts_);
    readBitsetData<uint64_t>(data_pkg, "actual_digital_input_bits", actual_dig_in_bits_);
    readBitsetData<uint64_t>(data_pkg, "actual_digital_output_bits", actual_dig_out_bits_);
    readBitsetData<uint32_t>(data_pkg, "analog_io_types", analog_io_types_);
    readBitsetData<uint32_t>(data_pkg, "tool_analog_input_types", tool_analog_input_types_);

    extractRobotStatus();

    publishIOData();
    publishToolData();

    // Transform fts measurements to tool frame
    extractToolPose(time);
    transformForceTorque();
    publishPose();
    publishRobotAndSafetyMode();

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
    // If reading from RTDE fails, this means that we lost RTDE connection (or that our connection
    // is not reliable). If we ignore this, the joint_state_controller will continue to publish old
    // data (e.g. if we unplug the cable from the robot).  However, this also means that any
    // trajectory execution will be aborted from time to time if the connection to the robot isn't
    // reliable.
    // TODO: This doesn't seem too bad currently, but we have to keep this in mind, when we
    // implement trajectory execution strategies that require a less reliable network connection.
    controller_reset_necessary_ = true;
    if (!non_blocking_read_)
    {
      ROS_ERROR("Could not get fresh data package from robot");
    }
  }
}

void HardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
  if ((runtime_state_ == static_cast<uint32_t>(rtde_interface::RUNTIME_STATE::PLAYING) ||
       runtime_state_ == static_cast<uint32_t>(rtde_interface::RUNTIME_STATE::PAUSING)) &&
      robot_program_running_ && (!non_blocking_read_ || packet_read_))
  {
    if (position_controller_running_)
    {
      ur_driver_->writeJointCommand(joint_position_command_, comm::ControlMode::MODE_SERVOJ);
    }
    else if (velocity_controller_running_)
    {
      ur_driver_->writeJointCommand(joint_velocity_command_, comm::ControlMode::MODE_SPEEDJ);
    }
    else
    {
      ur_driver_->writeKeepalive();
    }
    packet_read_ = false;
  }
}

bool HardwareInterface::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
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

void HardwareInterface::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                 const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  for (auto& controller_it : stop_list)
  {
    for (auto& resource_it : controller_it.claimed_resources)
    {
      if (checkControllerClaims(resource_it.resources))
      {
        if (resource_it.hardware_interface == "ur_controllers::ScaledPositionJointInterface")
        {
          position_controller_running_ = false;
        }
        if (resource_it.hardware_interface == "hardware_interface::PositionJointInterface")
        {
          position_controller_running_ = false;
        }
        if (resource_it.hardware_interface == "ur_controllers::ScaledVelocityJointInterface")
        {
          velocity_controller_running_ = false;
        }
        if (resource_it.hardware_interface == "hardware_interface::VelocityJointInterface")
        {
          velocity_controller_running_ = false;
        }
      }
    }
  }
  for (auto& controller_it : start_list)
  {
    for (auto& resource_it : controller_it.claimed_resources)
    {
      if (checkControllerClaims(resource_it.resources))
      {
        if (resource_it.hardware_interface == "ur_controllers::ScaledPositionJointInterface")
        {
          position_controller_running_ = true;
        }
        if (resource_it.hardware_interface == "hardware_interface::PositionJointInterface")
        {
          position_controller_running_ = true;
        }
        if (resource_it.hardware_interface == "ur_controllers::ScaledVelocityJointInterface")
        {
          velocity_controller_running_ = true;
        }
        if (resource_it.hardware_interface == "hardware_interface::VelocityJointInterface")
        {
          velocity_controller_running_ = true;
        }
      }
    }
  }
}

uint32_t HardwareInterface::getControlFrequency() const
{
  if (ur_driver_ != nullptr)
  {
    return ur_driver_->getControlFrequency();
  }
  throw std::runtime_error("ur_driver is not yet initialized");
}

void HardwareInterface::transformForceTorque()
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

bool HardwareInterface::isRobotProgramRunning() const
{
  return robot_program_running_;
}

void HardwareInterface::handleRobotProgramState(bool program_running)
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

bool HardwareInterface::shouldResetControllers()
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

void HardwareInterface::extractToolPose(const ros::Time& timestamp)
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

void HardwareInterface::publishPose()
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

void HardwareInterface::extractRobotStatus()
{
  using namespace rtde_interface;

  robot_status_resource_.mode = robot_status_bits_[toUnderlying(UrRtdeRobotStatusBits::IS_TEACH_BUTTON_PRESSED)] ?
                                    RobotMode::MANUAL :
                                    RobotMode::AUTO;

  robot_status_resource_.e_stopped = safety_status_bits_[toUnderlying(UrRtdeSafetyStatusBits::IS_EMERGENCY_STOPPED)] ?
                                         TriState::TRUE :
                                         TriState::FALSE;

  // Note that this is true as soon as the drives are powered,
  // even if the brakes are still closed
  // which is in slight contrast to the comments in the
  // message definition
  robot_status_resource_.drives_powered =
      robot_status_bits_[toUnderlying(UrRtdeRobotStatusBits::IS_POWER_ON)] ? TriState::TRUE : TriState::FALSE;

  // I found no way to reliably get information if the robot is moving
  robot_status_resource_.in_motion = TriState::UNKNOWN;

  if ((safety_status_bits_ & in_error_bitset_).any())
  {
    robot_status_resource_.in_error = TriState::TRUE;
  }
  else
  {
    robot_status_resource_.in_error = TriState::FALSE;
  }

  // Motion is not possible if controller is either in error or in safeguard stop.
  // TODO: Check status of robot program "external control" here as well
  if (robot_status_resource_.in_error == TriState::TRUE ||
      safety_status_bits_[toUnderlying(UrRtdeSafetyStatusBits::IS_SAFEGUARD_STOPPED)])
  {
    robot_status_resource_.motion_possible = TriState::FALSE;
  }
  else if (robot_mode_ == ur_dashboard_msgs::RobotMode::RUNNING)

  {
    robot_status_resource_.motion_possible = TriState::TRUE;
  }
  else
  {
    robot_status_resource_.motion_possible = TriState::FALSE;
  }

  // the error code, if any, is not transmitted by this protocol
  // it can and should be fetched separately
  robot_status_resource_.error_code = 0;
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
        io_pub_->msg_.analog_in_states[i].domain = analog_io_types_[i];
      }
      for (size_t i = 0; i < standard_analog_output_.size(); ++i)
      {
        io_pub_->msg_.analog_out_states[i].state = standard_analog_output_[i];
        io_pub_->msg_.analog_out_states[i].domain = analog_io_types_[i + 2];
      }
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
      tool_data_pub_->msg_.analog_input3 = tool_analog_input_[1];
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

bool HardwareInterface::setSpeedSlider(ur_msgs::SetSpeedSliderFractionRequest& req,
                                       ur_msgs::SetSpeedSliderFractionResponse& res)
{
  if (req.speed_slider_fraction >= 0.01 && req.speed_slider_fraction <= 1.0 && ur_driver_ != nullptr)
  {
    res.success = ur_driver_->getRTDEWriter().sendSpeedSlider(req.speed_slider_fraction);
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
    res.success = ur_driver_->getRTDEWriter().sendStandardAnalogOutput(req.pin, req.state);
  }
  else
  {
    LOG_ERROR("Cannot execute function %u. This is not (yet) supported.", req.fun);
    res.success = false;
  }

  return true;
}

bool HardwareInterface::resendRobotProgram(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
  res.success = ur_driver_->sendRobotProgram();
  if (res.success)
  {
    res.message = "Successfully resent robot program";
  }
  else
  {
    res.message = "Could not resend robot program";
  }

  return true;
}

bool HardwareInterface::zeroFTSensor(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
  if (ur_driver_->getVersion().major < 5)
  {
    std::stringstream ss;
    ss << "Zeroing the Force-Torque sensor is only available for e-Series robots (Major version >= 5). This robot's "
          "version is "
       << ur_driver_->getVersion();
    ROS_ERROR_STREAM(ss.str());
    res.message = ss.str();
    res.success = false;
  }
  else
  {
    res.success = this->ur_driver_->sendScript(R"(sec tareSensor():
  zero_ftsensor()
end
)");
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

void HardwareInterface::publishRobotAndSafetyMode()
{
  if (robot_mode_pub_)
  {
    if (robot_mode_pub_->msg_.mode != robot_mode_)
    {
      if (robot_mode_pub_->trylock())
      {
        robot_mode_pub_->msg_.mode = robot_mode_;
        robot_mode_pub_->unlockAndPublish();
      }
    }
  }
  if (safety_mode_pub_)
  {
    if (safety_mode_pub_->msg_.mode != safety_mode_)
    {
      if (safety_mode_pub_->trylock())
      {
        safety_mode_pub_->msg_.mode = safety_mode_;
        safety_mode_pub_->unlockAndPublish();
      }
    }
  }
}

bool HardwareInterface::checkControllerClaims(const std::set<std::string>& claimed_resources)
{
  for (const std::string& it : joint_names_)
  {
    for (const std::string& jt : claimed_resources)
    {
      if (it == jt)
      {
        return true;
      }
    }
  }
  return false;
}
}  // namespace ur_driver

PLUGINLIB_EXPORT_CLASS(ur_driver::HardwareInterface, hardware_interface::RobotHW)
