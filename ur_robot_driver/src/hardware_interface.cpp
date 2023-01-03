// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
// Created on behalf of Universal Robots A/S
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
#include <ur_client_library/control/trajectory_point_interface.h>
#include <ur_robot_driver/hardware_interface.h>
#include <ur_client_library/ur/tool_communication.h>
#include <ur_client_library/exceptions.h>

#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <cartesian_control_msgs/FollowCartesianTrajectoryAction.h>

#include <Eigen/Geometry>
#include <stdexcept>

using industrial_robot_status_interface::RobotMode;
using industrial_robot_status_interface::TriState;
// using namespace urcl::rtde_interface;
namespace rtde = urcl::rtde_interface;

namespace ur_driver
{
// bitset mask is applied to robot safety status bits in order to determine 'in_error' state
static const std::bitset<11>
    in_error_bitset_(1 << urcl::toUnderlying(rtde::UrRtdeSafetyStatusBits::IS_PROTECTIVE_STOPPED) |
                     1 << urcl::toUnderlying(rtde::UrRtdeSafetyStatusBits::IS_ROBOT_EMERGENCY_STOPPED) |
                     1 << urcl::toUnderlying(rtde::UrRtdeSafetyStatusBits::IS_EMERGENCY_STOPPED) |
                     1 << urcl::toUnderlying(rtde::UrRtdeSafetyStatusBits::IS_VIOLATION) |
                     1 << urcl::toUnderlying(rtde::UrRtdeSafetyStatusBits::IS_FAULT) |
                     1 << urcl::toUnderlying(rtde::UrRtdeSafetyStatusBits::IS_STOPPED_DUE_TO_SAFETY));

HardwareInterface::HardwareInterface()
  : joint_position_command_({ 0, 0, 0, 0, 0, 0 })
  , joint_velocity_command_({ 0, 0, 0, 0, 0, 0 })
  , cartesian_velocity_command_({ 0, 0, 0, 0, 0, 0 })
  , cartesian_pose_command_({ 0, 0, 0, 0, 0, 0 })
  , joint_positions_{ { 0, 0, 0, 0, 0, 0 } }
  , joint_velocities_{ { 0, 0, 0, 0, 0, 0 } }
  , joint_efforts_{ { 0, 0, 0, 0, 0, 0 } }
  , standard_analog_input_{ { 0, 0 } }
  , standard_analog_output_{ { 0, 0 } }
  , joint_names_(6)
  , safety_mode_(ur_dashboard_msgs::SafetyMode::NORMAL)
  , runtime_state_(static_cast<uint32_t>(rtde::RUNTIME_STATE::STOPPED))
  , position_controller_running_(false)
  , velocity_controller_running_(false)
  , joint_forward_controller_running_(false)
  , cartesian_forward_controller_running_(false)
  , twist_controller_running_(false)
  , pose_controller_running_(false)
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
  std::string wrench_frame_id;
  std::string speed_scaling_id;
  std::string output_recipe_filename;
  std::string input_recipe_filename;

  // The robot's IP address.
  if (!robot_hw_nh.getParam("robot_ip", robot_ip_))
  {
    ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("robot_ip") << " not given.");
    return false;
  }

  // IP that will be used for the robot controller to communicate back to the driver.
  std::string reverse_ip = robot_hw_nh.param<std::string>("reverse_ip", "");

  // Port that will be opened to communicate between the driver and the robot controller.
  int reverse_port = robot_hw_nh.param("reverse_port", 50001);

  // The driver will offer an interface to receive the program's URScript on this port.
  int script_sender_port = robot_hw_nh.param("script_sender_port", 50002);

  // Port that will be opened to send trajectory points from the driver to the robot
  int trajectory_port = robot_hw_nh.param("trajectory_port", 50003);

  // Port that will be opened to forward script commands from the driver to the robot
  int script_command_port = robot_hw_nh.param("script_command_port", 50004);

  // When the robot's URDF is being loaded with a prefix, we need to know it here, as well, in order
  // to publish correct frame names for frames reported by the robot directly.
  robot_hw_nh.param<std::string>("tf_prefix", tf_prefix_, "");

  // Optional parameter to change the id of the wrench frame
  robot_hw_nh.param<std::string>("wrench_frame_id", wrench_frame_id, "wrench");

  // Optional parameter to change the id of the speed scaling topic
  robot_hw_nh.param<std::string>("speed_scaling_id", speed_scaling_id, "speed_scaling_factor");

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
  std::unique_ptr<urcl::ToolCommSetup> tool_comm_setup;
  if (use_tool_communication)
  {
    tool_comm_setup.reset(new urcl::ToolCommSetup());

    using ToolVoltageT = std::underlying_type<urcl::ToolVoltage>::type;
    ToolVoltageT tool_voltage;
    // Tool voltage that will be set as soon as the UR-Program on the robot is started. Note: This
    // parameter is only evaluated, when the parameter "use_tool_communication" is set to TRUE.
    // Then, this parameter is required.
    if (!robot_hw_nh.getParam("tool_voltage", tool_voltage))
    {
      ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("tool_voltage") << " not given.");
      return false;
    }
    tool_comm_setup->setToolVoltage(static_cast<urcl::ToolVoltage>(tool_voltage));

    using ParityT = std::underlying_type<urcl::Parity>::type;
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
    tool_comm_setup->setParity(static_cast<urcl::Parity>(parity));

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
    ur_driver_.reset(new urcl::UrDriver(
        robot_ip_, script_filename, output_recipe_filename, input_recipe_filename,
        std::bind(&HardwareInterface::handleRobotProgramState, this, std::placeholders::_1), headless_mode,
        std::move(tool_comm_setup), (uint32_t)reverse_port, (uint32_t)script_sender_port, servoj_gain,
        servoj_lookahead_time, non_blocking_read_, reverse_ip, trajectory_port, script_command_port));
  }
  catch (urcl::ToolCommNotAvailable& e)
  {
    ROS_FATAL_STREAM(e.what() << " See parameter '" << robot_hw_nh.resolveName("use_tool_communication") << "'.");
    return false;
  }
  catch (urcl::UrException& e)
  {
    ROS_FATAL_STREAM(e.what() << std::endl
                              << "Please note that the minimum software version required is 3.12.0 for CB3 robots and "
                                 "5.5.1 for e-Series robots. The error above could be related to a non-supported "
                                 "polyscope version. Please update your robot's software accordingly.");
    return false;
  }
  URCL_LOG_INFO("Checking if calibration data matches connected robot.");
  if (ur_driver_->checkCalibration(calibration_checksum))
  {
    ROS_INFO_STREAM("Calibration checked successfully.");
  }
  else
  {
    ROS_ERROR_STREAM("The calibration parameters of the connected robot don't match the ones from the given kinematics "
                     "config file. Please be aware that this can lead to critical inaccuracies of tcp positions. Use "
                     "the ur_calibration tool to extract the correct calibration from the robot and pass that into the "
                     "description. See "
                     "[https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#extract-calibration-information] "
                     "for details.");
  }
  ur_driver_->registerTrajectoryDoneCallback(
      std::bind(&HardwareInterface::passthroughTrajectoryDoneCb, this, std::placeholders::_1));

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
  if (!robot_hw_nh.getParam("joints", joint_names_))
  {
    ROS_ERROR_STREAM("Cannot find required parameter " << robot_hw_nh.resolveName("joints")
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
    spj_interface_.registerHandle(scaled_controllers::ScaledJointHandle(
        js_interface_.getHandle(joint_names_[i]), &joint_position_command_[i], &speed_scaling_combined_));
    svj_interface_.registerHandle(scaled_controllers::ScaledJointHandle(
        js_interface_.getHandle(joint_names_[i]), &joint_velocity_command_[i], &speed_scaling_combined_));
  }

  speedsc_interface_.registerHandle(scaled_controllers::SpeedScalingHandle(speed_scaling_id, &speed_scaling_combined_));

  fts_interface_.registerHandle(hardware_interface::ForceTorqueSensorHandle(
      wrench_frame_id, tf_prefix_ + "tool0_controller", fts_measurements_.begin(), fts_measurements_.begin() + 3));

  robot_status_interface_.registerHandle(industrial_robot_status_interface::IndustrialRobotStatusHandle(
      "industrial_robot_status_handle", robot_status_resource_));

  // Register callbacks for trajectory passthrough
  jnt_traj_interface_.registerGoalCallback(
      std::bind(&HardwareInterface::startJointInterpolation, this, std::placeholders::_1));
  jnt_traj_interface_.registerCancelCallback(std::bind(&HardwareInterface::cancelInterpolation, this));
  cart_traj_interface_.registerGoalCallback(
      std::bind(&HardwareInterface::startCartesianInterpolation, this, std::placeholders::_1));
  cart_traj_interface_.registerCancelCallback(std::bind(&HardwareInterface::cancelInterpolation, this));

  ros_controllers_cartesian::CartesianStateHandle handle(tf_prefix_ + "base", tf_prefix_ + "tool0_controller",
                                                         &cart_pose_, &cart_twist_, &cart_accel_, &cart_jerk_);
  cart_interface_.registerHandle(handle);
  twist_interface_.registerHandle(ros_controllers_cartesian::TwistCommandHandle(
      cart_interface_.getHandle(tf_prefix_ + "tool0_controller"), &twist_command_));
  twist_interface_.getHandle(tf_prefix_ + "tool0_controller");
  pose_interface_.registerHandle(ros_controllers_cartesian::PoseCommandHandle(
      cart_interface_.getHandle(tf_prefix_ + "tool0_controller"), &pose_command_));
  pose_interface_.getHandle(tf_prefix_ + "tool0_controller");

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&spj_interface_);
  registerInterface(&pj_interface_);
  registerInterface(&vj_interface_);
  registerInterface(&svj_interface_);
  registerInterface(&speedsc_interface_);
  registerInterface(&fts_interface_);
  registerInterface(&robot_status_interface_);
  registerInterface(&jnt_traj_interface_);
  registerInterface(&cart_traj_interface_);
  registerInterface(&twist_interface_);
  registerInterface(&pose_interface_);

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

  // Calling this service will zero the robot's ftsensor (only available for e-Series).
  tare_sensor_srv_ = robot_hw_nh.advertiseService("zero_ftsensor", &HardwareInterface::zeroFTSensor, this);

  // Setup the mounted payload through a ROS service
  set_payload_srv_ = robot_hw_nh.advertiseService("set_payload", &HardwareInterface::setPayload, this);

  ur_driver_->startRTDECommunication();
  ROS_INFO_STREAM_NAMED("hardware_interface", "Loaded ur_robot_driver hardware_interface");

  return true;
}

template <typename T>
void HardwareInterface::readData(const std::unique_ptr<rtde::DataPackage>& data_pkg, const std::string& var_name,
                                 T& data)
{
  if (!data_pkg->getData(var_name, data))
  {
    // This throwing should never happen unless misconfigured
    std::string error_msg = "Did not find '" + var_name + "' in data sent from robot. This should not happen!";
    throw std::runtime_error(error_msg);
  }
}

template <typename T, size_t N>
void HardwareInterface::readBitsetData(const std::unique_ptr<rtde::DataPackage>& data_pkg, const std::string& var_name,
                                       std::bitset<N>& data)
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

  std::unique_ptr<rtde::DataPackage> data_pkg = ur_driver_->getDataPackage();
  if (data_pkg)
  {
    packet_read_ = true;
    readData(data_pkg, "actual_q", joint_positions_);
    readData(data_pkg, "actual_qd", joint_velocities_);
    readData(data_pkg, "target_q", target_joint_positions_);
    readData(data_pkg, "target_qd", target_joint_velocities_);
    readData(data_pkg, "target_speed_fraction", target_speed_fraction_);
    readData(data_pkg, "speed_scaling", speed_scaling_);
    readData(data_pkg, "runtime_state", runtime_state_);
    readData(data_pkg, "actual_TCP_force", fts_measurements_);
    readData(data_pkg, "actual_TCP_pose", tcp_pose_);
    readData(data_pkg, "actual_TCP_speed", tcp_speed_);
    readData(data_pkg, "target_TCP_pose", target_tcp_pose_);
    readData(data_pkg, "target_TCP_speed", target_tcp_speed_);
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
    readData(data_pkg, "tcp_offset", tcp_offset_);

    cart_pose_.position.x = tcp_pose_[0];
    cart_pose_.position.y = tcp_pose_[1];
    cart_pose_.position.z = tcp_pose_[2];

    // UR robots operate in axis angle representation

    tcp_vec_ = KDL::Vector(tcp_pose_[3], tcp_pose_[4], tcp_pose_[5]);

    tcp_angle_ = tcp_vec_.Normalize();

    tcp_pose_rot_ = KDL::Rotation::Rot(tcp_vec_, tcp_angle_);
    tcp_pose_rot_.GetQuaternion(cart_pose_.orientation.x, cart_pose_.orientation.y, cart_pose_.orientation.z,
                                cart_pose_.orientation.w);

    cart_twist_.linear.x = tcp_speed_[0];
    cart_twist_.linear.y = tcp_speed_[1];
    cart_twist_.linear.z = tcp_speed_[2];
    cart_twist_.angular.x = tcp_speed_[3];
    cart_twist_.angular.y = tcp_speed_[4];
    cart_twist_.angular.z = tcp_speed_[5];

    extractRobotStatus();

    publishIOData();
    publishToolData();

    // Transform fts measurements to tool frame
    extractToolPose(time);
    transformForceTorque();
    publishPose();
    publishRobotAndSafetyMode();

    // Action feedback for joint trajectory forwarding
    if (joint_forward_controller_running_)
    {
      control_msgs::FollowJointTrajectoryFeedback feedback = control_msgs::FollowJointTrajectoryFeedback();
      for (size_t i = 0; i < 6; i++)
      {
        feedback.desired.positions.push_back(target_joint_positions_[i]);
        feedback.desired.velocities.push_back(target_joint_velocities_[i]);
        feedback.actual.positions.push_back(joint_positions_[i]);
        feedback.actual.velocities.push_back(joint_velocities_[i]);
        feedback.error.positions.push_back(std::abs(joint_positions_[i] - target_joint_positions_[i]));
        feedback.error.velocities.push_back(std::abs(joint_velocities_[i] - target_joint_velocities_[i]));
      }
      jnt_traj_interface_.setFeedback(feedback);
    }

    // Action feedback for cartesian trajectory forwarding
    if (cartesian_forward_controller_running_)
    {
      cartesian_control_msgs::FollowCartesianTrajectoryFeedback feedback =
          cartesian_control_msgs::FollowCartesianTrajectoryFeedback();

      target_cart_pose_.position.x = target_tcp_pose_[0];
      target_cart_pose_.position.y = target_tcp_pose_[1];
      target_cart_pose_.position.z = target_tcp_pose_[2];

      tcp_vec_ = KDL::Vector(target_tcp_pose_[3], target_tcp_pose_[4], target_tcp_pose_[5]);
      tcp_angle_ = tcp_vec_.Normalize();

      target_tcp_pose_rot_ = KDL::Rotation::Rot(tcp_vec_, tcp_angle_);
      target_tcp_pose_rot_.GetQuaternion(target_cart_pose_.orientation.x, target_cart_pose_.orientation.y,
                                         target_cart_pose_.orientation.z, target_cart_pose_.orientation.w);

      target_cart_twist_.linear.x = target_tcp_speed_[0];
      target_cart_twist_.linear.y = target_tcp_speed_[1];
      target_cart_twist_.linear.z = target_tcp_speed_[2];
      target_cart_twist_.angular.x = target_tcp_speed_[3];
      target_cart_twist_.angular.y = target_tcp_speed_[4];
      target_cart_twist_.angular.z = target_tcp_speed_[5];

      error_cart_pose_.position.x = std::abs(cart_pose_.position.x - target_cart_pose_.position.x);
      error_cart_pose_.position.y = std::abs(cart_pose_.position.y - target_cart_pose_.position.y);
      error_cart_pose_.position.z = std::abs(cart_pose_.position.z - target_cart_pose_.position.z);

      error_cart_twist_.linear.x = std::abs(cart_twist_.linear.x - target_cart_twist_.linear.x);
      error_cart_twist_.linear.y = std::abs(cart_twist_.linear.y - target_cart_twist_.linear.y);
      error_cart_twist_.linear.z = std::abs(cart_twist_.linear.z - target_cart_twist_.linear.z);
      error_cart_twist_.angular.x = std::abs(cart_twist_.angular.x - target_cart_twist_.angular.x);
      error_cart_twist_.angular.y = std::abs(cart_twist_.angular.y - target_cart_twist_.angular.y);
      error_cart_twist_.angular.z = std::abs(cart_twist_.angular.z - target_cart_twist_.angular.z);

      feedback.desired.pose = target_cart_pose_;
      feedback.desired.twist = target_cart_twist_;
      feedback.actual.pose = cart_pose_;
      feedback.actual.twist = cart_twist_;
      cart_traj_interface_.setFeedback(feedback);
    }

    // pausing state follows runtime state when pausing
    if (runtime_state_ == static_cast<uint32_t>(rtde::RUNTIME_STATE::PAUSED))
    {
      pausing_state_ = PausingState::PAUSED;
    }
    // When the robot resumed program execution and pausing state was PAUSED, we enter RAMPUP
    else if (runtime_state_ == static_cast<uint32_t>(rtde::RUNTIME_STATE::PLAYING) &&
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
    else if (runtime_state_ == static_cast<uint32_t>(rtde::RUNTIME_STATE::RESUMING))
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
  if ((runtime_state_ == static_cast<uint32_t>(rtde::RUNTIME_STATE::PLAYING) ||
       runtime_state_ == static_cast<uint32_t>(rtde::RUNTIME_STATE::PAUSING)) &&
      robot_program_running_ && (!non_blocking_read_ || packet_read_))
  {
    if (position_controller_running_)
    {
      ur_driver_->writeJointCommand(joint_position_command_, urcl::comm::ControlMode::MODE_SERVOJ);
    }
    else if (velocity_controller_running_)
    {
      ur_driver_->writeJointCommand(joint_velocity_command_, urcl::comm::ControlMode::MODE_SPEEDJ);
    }
    else if (joint_forward_controller_running_)
    {
      ur_driver_->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_NOOP);
    }
    else if (cartesian_forward_controller_running_)
    {
      ur_driver_->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_NOOP);
    }
    else if (twist_controller_running_)
    {
      cartesian_velocity_command_[0] = twist_command_.linear.x;
      cartesian_velocity_command_[1] = twist_command_.linear.y;
      cartesian_velocity_command_[2] = twist_command_.linear.z;
      cartesian_velocity_command_[3] = twist_command_.angular.x;
      cartesian_velocity_command_[4] = twist_command_.angular.y;
      cartesian_velocity_command_[5] = twist_command_.angular.z;
      ur_driver_->writeJointCommand(cartesian_velocity_command_, urcl::comm::ControlMode::MODE_SPEEDL);
    }
    else if (pose_controller_running_)
    {
      cartesian_pose_command_[0] = pose_command_.position.x;
      cartesian_pose_command_[1] = pose_command_.position.y;
      cartesian_pose_command_[2] = pose_command_.position.z;

      KDL::Rotation rot = KDL::Rotation::Quaternion(pose_command_.orientation.x, pose_command_.orientation.y,
                                                    pose_command_.orientation.z, pose_command_.orientation.w);
      cartesian_pose_command_[3] = rot.GetRot().x();
      cartesian_pose_command_[4] = rot.GetRot().y();
      cartesian_pose_command_[5] = rot.GetRot().z();

      ur_driver_->writeJointCommand(cartesian_pose_command_, urcl::comm::ControlMode::MODE_POSE);
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
        if (resource_it.hardware_interface == "scaled_controllers::ScaledPositionJointInterface")
        {
          position_controller_running_ = false;
        }
        if (resource_it.hardware_interface == "hardware_interface::PositionJointInterface")
        {
          position_controller_running_ = false;
        }
        if (resource_it.hardware_interface == "scaled_controllers::ScaledVelocityJointInterface")
        {
          velocity_controller_running_ = false;
        }
        if (resource_it.hardware_interface == "hardware_interface::VelocityJointInterface")
        {
          velocity_controller_running_ = false;
        }
        if (resource_it.hardware_interface == "hardware_interface::TrajectoryInterface<control_msgs::"
                                              "FollowJointTrajectoryGoal_<std::allocator<void> >, "
                                              "control_msgs::FollowJointTrajectoryFeedback_<std::allocator<void> > >")
        {
          joint_forward_controller_running_ = false;
        }
        if (resource_it.hardware_interface == "hardware_interface::TrajectoryInterface<cartesian_control_msgs::"
                                              "FollowCartesianTrajectoryGoal_<std::allocator<void> >, "
                                              "cartesian_control_msgs::FollowCartesianTrajectoryFeedback_<std::"
                                              "allocator<void> > >")
        {
          cartesian_forward_controller_running_ = false;
        }
        if (resource_it.hardware_interface == "ros_controllers_cartesian::TwistCommandInterface")
        {
          twist_controller_running_ = false;
        }
        if (resource_it.hardware_interface == "ros_controllers_cartesian::PoseCommandInterface")
        {
          pose_controller_running_ = false;
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
        if (resource_it.hardware_interface == "scaled_controllers::ScaledPositionJointInterface")
        {
          position_controller_running_ = true;
        }
        if (resource_it.hardware_interface == "hardware_interface::PositionJointInterface")
        {
          position_controller_running_ = true;
        }
        if (resource_it.hardware_interface == "scaled_controllers::ScaledVelocityJointInterface")
        {
          velocity_controller_running_ = true;
        }
        if (resource_it.hardware_interface == "hardware_interface::VelocityJointInterface")
        {
          velocity_controller_running_ = true;
        }
        if (resource_it.hardware_interface == "hardware_interface::TrajectoryInterface<control_msgs::"
                                              "FollowJointTrajectoryGoal_<std::allocator<void> >, "
                                              "control_msgs::FollowJointTrajectoryFeedback_<std::allocator<void> > >")
        {
          joint_forward_controller_running_ = true;
        }
        if (resource_it.hardware_interface == "hardware_interface::TrajectoryInterface<cartesian_control_msgs::"
                                              "FollowCartesianTrajectoryGoal_<std::allocator<void> >, "
                                              "cartesian_control_msgs::FollowCartesianTrajectoryFeedback_<std::"
                                              "allocator<void> > >")
        {
          cartesian_forward_controller_running_ = true;
        }
        if (resource_it.hardware_interface == "ros_controllers_cartesian::TwistCommandInterface")
        {
          twist_controller_running_ = true;
        }
        if (resource_it.hardware_interface == "ros_controllers_cartesian::PoseCommandInterface")
        {
          pose_controller_running_ = true;
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
  KDL::Wrench ft(KDL::Vector(fts_measurements_[0], fts_measurements_[1], fts_measurements_[2]),
                 KDL::Vector(fts_measurements_[3], fts_measurements_[4], fts_measurements_[5]));
  if (ur_driver_->getVersion().major >= 5)  // e-Series
  {
    // Setup necessary frames
    KDL::Vector vec = KDL::Vector(tcp_offset_[3], tcp_offset_[4], tcp_offset_[5]);
    double angle = vec.Normalize();
    KDL::Rotation rotation = KDL::Rotation::Rot(vec, angle);
    KDL::Frame flange_to_tcp = KDL::Frame(rotation, KDL::Vector(tcp_offset_[0], tcp_offset_[1], tcp_offset_[2]));

    vec = KDL::Vector(target_tcp_pose_[3], target_tcp_pose_[4], target_tcp_pose_[5]);
    angle = vec.Normalize();
    rotation = KDL::Rotation::Rot(vec, angle);
    KDL::Frame base_to_tcp =
        KDL::Frame(rotation, KDL::Vector(target_tcp_pose_[0], target_tcp_pose_[1], target_tcp_pose_[2]));

    // Calculate transformation from base to flange, see calculation details below
    // `base_to_tcp = base_to_flange*flange_to_tcp -> base_to_flange = base_to_tcp * inv(flange_to_tcp)`
    KDL::Frame base_to_flange = base_to_tcp * flange_to_tcp.Inverse();

    // rotate f/t sensor output back to the flange frame
    ft = base_to_flange.M.Inverse() * ft;

    // Transform the wrench to the tcp frame
    ft = flange_to_tcp * ft;
  }
  else  // CB3
  {
    KDL::Vector vec = KDL::Vector(target_tcp_pose_[3], target_tcp_pose_[4], target_tcp_pose_[5]);
    double angle = vec.Normalize();
    KDL::Rotation base_to_tcp_rot = KDL::Rotation::Rot(vec, angle);

    // rotate f/t sensor output back to the tcp frame
    ft = base_to_tcp_rot.Inverse() * ft;
  }
  fts_measurements_ = { ft[0], ft[1], ft[2], ft[3], ft[4], ft[5] };
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
  else
  {
    rotation.setValue(0.0, 0.0, 0.0, 1.0);  // default Quaternion is 0,0,0,0 which is invalid
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
  robot_status_resource_.mode =
      robot_status_bits_[urcl::toUnderlying(rtde::UrRtdeRobotStatusBits::IS_TEACH_BUTTON_PRESSED)] ? RobotMode::MANUAL :
                                                                                                     RobotMode::AUTO;

  robot_status_resource_.e_stopped =
      safety_status_bits_[urcl::toUnderlying(rtde::UrRtdeSafetyStatusBits::IS_EMERGENCY_STOPPED)] ? TriState::TRUE :
                                                                                                    TriState::FALSE;

  // Note that this is true as soon as the drives are powered,
  // even if the brakes are still closed
  // which is in slight contrast to the comments in the
  // message definition
  robot_status_resource_.drives_powered =
      robot_status_bits_[urcl::toUnderlying(rtde::UrRtdeRobotStatusBits::IS_POWER_ON)] ? TriState::TRUE :
                                                                                         TriState::FALSE;

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
      safety_status_bits_[urcl::toUnderlying(rtde::UrRtdeSafetyStatusBits::IS_SAFEGUARD_STOPPED)])
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
  else if (req.fun == req.FUN_SET_TOOL_VOLTAGE && ur_driver_ != nullptr)
  {
    res.success = ur_driver_->setToolVoltage(static_cast<urcl::ToolVoltage>(req.state));
  }
  else
  {
    ROS_ERROR("Cannot execute function %u. This is not (yet) supported.", req.fun);
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
    res.success = this->ur_driver_->zeroFTSensor();
  }
  return true;
}

bool HardwareInterface::setPayload(ur_msgs::SetPayloadRequest& req, ur_msgs::SetPayloadResponse& res)
{
  urcl::vector3d_t cog;
  cog[0] = req.center_of_gravity.x;
  cog[1] = req.center_of_gravity.y;
  cog[2] = req.center_of_gravity.z;
  res.success = this->ur_driver_->setPayload(req.mass, cog);
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
  for (const std::string& jt : claimed_resources)
  {
    if ("tool0_controller" == jt)
    {
      return true;
    }
  }
  return false;
}

void HardwareInterface::startJointInterpolation(const hardware_interface::JointTrajectory& trajectory)
{
  size_t point_number = trajectory.trajectory.points.size();
  ROS_DEBUG("Starting joint-based trajectory forward");
  ur_driver_->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_START, point_number);
  double last_time = 0.0;
  for (size_t i = 0; i < point_number; i++)
  {
    trajectory_msgs::JointTrajectoryPoint point = trajectory.trajectory.points[i];
    urcl::vector6d_t p;
    p[0] = point.positions[0];
    p[1] = point.positions[1];
    p[2] = point.positions[2];
    p[3] = point.positions[3];
    p[4] = point.positions[4];
    p[5] = point.positions[5];
    double next_time = point.time_from_start.toSec();
    ur_driver_->writeTrajectoryPoint(p, false, next_time - last_time);
    last_time = next_time;
  }
  ROS_DEBUG("Finished Sending Trajectory");
}

void HardwareInterface::startCartesianInterpolation(const hardware_interface::CartesianTrajectory& trajectory)
{
  size_t point_number = trajectory.trajectory.points.size();
  ROS_DEBUG("Starting cartesian trajectory forward");
  ur_driver_->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_START, point_number);
  double last_time = 0.0;
  for (size_t i = 0; i < point_number; i++)
  {
    cartesian_control_msgs::CartesianTrajectoryPoint point = trajectory.trajectory.points[i];
    urcl::vector6d_t p;
    p[0] = point.pose.position.x;
    p[1] = point.pose.position.y;
    p[2] = point.pose.position.z;

    KDL::Rotation rot = KDL::Rotation::Quaternion(point.pose.orientation.x, point.pose.orientation.y,
                                                  point.pose.orientation.z, point.pose.orientation.w);

    // UR robots use axis angle representation.
    p[3] = rot.GetRot().x();
    p[4] = rot.GetRot().y();
    p[5] = rot.GetRot().z();
    double next_time = point.time_from_start.toSec();
    ur_driver_->writeTrajectoryPoint(p, true, next_time - last_time);
    last_time = next_time;
  }
  ROS_DEBUG("Finished Sending Trajectory");
}

void HardwareInterface::cancelInterpolation()
{
  ROS_DEBUG("Cancelling Trajectory");
  ur_driver_->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_CANCEL);
}

void HardwareInterface::passthroughTrajectoryDoneCb(urcl::control::TrajectoryResult result)
{
  hardware_interface::ExecutionState final_state;
  switch (result)
  {
    case urcl::control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS:
    {
      final_state = hardware_interface::ExecutionState::SUCCESS;
      ROS_INFO_STREAM("Forwarded trajectory finished successful.");
      break;
    }
    case urcl::control::TrajectoryResult::TRAJECTORY_RESULT_CANCELED:
    {
      final_state = hardware_interface::ExecutionState::PREEMPTED;
      ROS_INFO_STREAM("Forwarded trajectory execution preempted by user.");
      break;
    }
    case urcl::control::TrajectoryResult::TRAJECTORY_RESULT_FAILURE:
    {
      final_state = hardware_interface::ExecutionState::ABORTED;
      ROS_INFO_STREAM("Forwarded trajectory execution failed.");
      break;
    }
    default:
    {
      std::stringstream ss;
      ss << "Unknown trajectory result: " << urcl::toUnderlying(result);
      throw(std::invalid_argument(ss.str()));
    }
  }

  if (joint_forward_controller_running_)
  {
    jnt_traj_interface_.setDone(final_state);
  }
  else if (cartesian_forward_controller_running_)
  {
    cart_traj_interface_.setDone(final_state);
  }
  else
  {
    ROS_ERROR_STREAM("Received forwarded trajectory result with no forwarding controller running.");
  }
}
}  // namespace ur_driver

PLUGINLIB_EXPORT_CLASS(ur_driver::HardwareInterface, hardware_interface::RobotHW)
