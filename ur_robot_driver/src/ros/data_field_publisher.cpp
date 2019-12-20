#include "ur_robot_driver/ros/data_field_publisher.h"
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
 * \author  Tristan Schnell schnell@fzi.de
 * \date    2019-10-30
 *
 */
//----------------------------------------------------------------------

#include "ur_robot_driver/exceptions.h"
#include "ur_robot_driver/ros/geometry_data_publishers.h"
#include "ur_robot_driver/ros/sensor_data_publishers.h"
#include "ur_robot_driver/ros/robot_state_publishers.h"
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <ur_rtde_msgs/JointPosition.h>
#include <ur_rtde_msgs/JointVelocity.h>
#include <ur_rtde_msgs/JointAcceleration.h>
#include <ur_rtde_msgs/JointCurrents.h>
#include <ur_rtde_msgs/JointTorques.h>
#include <ur_rtde_msgs/JointVoltages.h>
#include <ur_rtde_msgs/JointTemperature.h>
#include <ur_rtde_msgs/ToolMode.h>
#include <ur_rtde_msgs/SafetyStatus.h>
#include <ur_dashboard_msgs/RobotMode.h>
#include <ur_dashboard_msgs/SafetyMode.h>

namespace ur_driver
{
namespace rtde_interface
{
using _bool_publisher = DirectDataPublisher<bool, std_msgs::Bool>;
using _uint8_publisher = DirectDataPublisher<uint8_t, std_msgs::UInt8>;
using _uint32_publisher = DirectDataPublisher<uint32_t, std_msgs::UInt32>;
using _uint64_publisher = DirectDataPublisher<uint64_t, std_msgs::UInt64>;
using _int32_publisher = DirectDataPublisher<int32_t, std_msgs::Int32>;
using _double_publisher = DirectDataPublisher<double, std_msgs::Float64>;
using _string_publisher = DirectDataPublisher<std::string, std_msgs::String>;

std::unique_ptr<DataFieldPublisher> DataFieldPublisher::createFromString(const std::string& data_field_identifier,
                                                                         ros::NodeHandle& nh)
{
  if (data_field_identifier == "timestamp")
  {
    return std::unique_ptr<DataFieldPublisher>(new DurationPublisher(data_field_identifier, nh));
  }
  else if (data_field_identifier == "target_q")
  {
    return std::unique_ptr<DataFieldPublisher>(
        new ArrayDataPublisher<double, ur_rtde_msgs::JointPosition, 6>(data_field_identifier, nh));
  }
  else if (data_field_identifier == "target_qd")
  {
    return std::unique_ptr<DataFieldPublisher>(
        new ArrayDataPublisher<double, ur_rtde_msgs::JointVelocity, 6>(data_field_identifier, nh));
  }
  else if (data_field_identifier == "target_qdd")
  {
    return std::unique_ptr<DataFieldPublisher>(
        new ArrayDataPublisher<double, ur_rtde_msgs::JointAcceleration, 6>(data_field_identifier, nh));
  }
  else if (data_field_identifier == "target_current")
  {
    return std::unique_ptr<DataFieldPublisher>(
        new ArrayDataPublisher<double, ur_rtde_msgs::JointCurrents, 6>(data_field_identifier, nh));
  }
  else if (data_field_identifier == "target_movement")
  {
    return std::unique_ptr<DataFieldPublisher>(
        new ArrayDataPublisher<double, ur_rtde_msgs::JointTorques, 6>(data_field_identifier, nh));
  }
  else if (data_field_identifier == "actual_q")
  {
    return std::unique_ptr<DataFieldPublisher>(
        new ArrayDataPublisher<double, ur_rtde_msgs::JointPosition, 6>(data_field_identifier, nh));
  }
  else if (data_field_identifier == "actual_qd")
  {
    return std::unique_ptr<DataFieldPublisher>(
        new ArrayDataPublisher<double, ur_rtde_msgs::JointVelocity, 6>(data_field_identifier, nh));
  }
  else if (data_field_identifier == "actual_current")
  {
    return std::unique_ptr<DataFieldPublisher>(
        new ArrayDataPublisher<double, ur_rtde_msgs::JointCurrents, 6>(data_field_identifier, nh));
  }
  else if (data_field_identifier == "joint_control_output")
  {
    return std::unique_ptr<DataFieldPublisher>(
        new ArrayDataPublisher<double, ur_rtde_msgs::JointCurrents, 6>(data_field_identifier, nh));
  }
  else if (data_field_identifier == "actual_TCP_pose")
  {
    return std::unique_ptr<DataFieldPublisher>(new PosePublisher(data_field_identifier, nh));
  }
  else if (data_field_identifier == "actual_TCP_speed")
  {
    return std::unique_ptr<DataFieldPublisher>(new TwistPublisher(data_field_identifier, nh));
  }
  else if (data_field_identifier == "target_TCP_pose")
  {
    return std::unique_ptr<DataFieldPublisher>(new PosePublisher(data_field_identifier, nh));
  }
  else if (data_field_identifier == "target_TCP_speed")
  {
    return std::unique_ptr<DataFieldPublisher>(new TwistPublisher(data_field_identifier, nh));
  }
  else if (data_field_identifier == "joint_temperatures")
  {
    // could use sensor_msgs/Temperature, but would then be the only data field that is actively
    // connected to the joint_names of the robot, as they are needed as reference frames
    return std::unique_ptr<DataFieldPublisher>(
        new ArrayDataPublisher<double, ur_rtde_msgs::JointTemperature, 6>(data_field_identifier, nh));
  }
  else if (data_field_identifier == "actual_execution_time")
  {
    // High possibility that actual execution time isn't given in seconds and scaling has to be
    // added, but nothing found in RTDE documentation
    return std::unique_ptr<DataFieldPublisher>(new DurationPublisher(data_field_identifier, nh));
  }
  else if (data_field_identifier == "robot_mode")
  {
    return std::unique_ptr<DataFieldPublisher>(
        new ModePublisher<int32_t, ur_dashboard_msgs::RobotMode>(data_field_identifier, nh));
  }
  else if (data_field_identifier == "joint_mode")
  {
    return std::unique_ptr<DataFieldPublisher>(new JointModePublisher(data_field_identifier, nh));
  }
  else if (data_field_identifier == "safety_mode")
  {
    return std::unique_ptr<DataFieldPublisher>(
        new ModePublisher<int32_t, ur_dashboard_msgs::SafetyMode>(data_field_identifier, nh));
  }
  else if (data_field_identifier == "safety_status")
  {
    return std::unique_ptr<DataFieldPublisher>(
        new StatusPublisher<int32_t, ur_rtde_msgs::SafetyStatus>(data_field_identifier, nh));
  }
  else if (data_field_identifier == "actual_tool_accelerometer")
  {
    return std::unique_ptr<DataFieldPublisher>(new Vector3Publisher(data_field_identifier, nh));
  }
  else if (data_field_identifier == "actual_joint_voltage")
  {
    return std::unique_ptr<DataFieldPublisher>(
        new ArrayDataPublisher<double, ur_rtde_msgs::JointVoltages, 6>(data_field_identifier, nh));
    //} else if (data_field_identifier == "runtime_state") {
    // to be added once documentation for runtime_state output is found
  }
  else if (data_field_identifier == "elbow_position")
  {
    return std::unique_ptr<DataFieldPublisher>(new Vector3Publisher(data_field_identifier, nh));
  }
  else if (data_field_identifier == "elbow_velocity")
  {
    return std::unique_ptr<DataFieldPublisher>(new Vector3Publisher(data_field_identifier, nh));
  }
  else if (data_field_identifier == "robot_status_bits")
  {
    return std::unique_ptr<DataFieldPublisher>(new RobotStatusBitsPublisher(data_field_identifier, nh));
  }
  else if (data_field_identifier == "safety_status_bits")
  {
    return std::unique_ptr<DataFieldPublisher>(new SafetyStatusBitsPublisher(data_field_identifier, nh));
  }
  else if (data_field_identifier == "euromap67_input_bits")
  {
    return std::unique_ptr<DataFieldPublisher>(new BitRegisterArrayPublisher(data_field_identifier, nh, 0));
  }
  else if (data_field_identifier == "euromap67_output_bits")
  {
    return std::unique_ptr<DataFieldPublisher>(new BitRegisterArrayPublisher(data_field_identifier, nh, 0));
  }
  else if (data_field_identifier == "output_bit_registers0_to_31")
  {
    return std::unique_ptr<DataFieldPublisher>(new BitRegisterArrayPublisher(data_field_identifier, nh, 0));
  }
  else if (data_field_identifier == "output_bit_registers32_to_63")
  {
    return std::unique_ptr<DataFieldPublisher>(new BitRegisterArrayPublisher(data_field_identifier, nh, 32));
  }
  else if (data_field_identifier == "input_bit_registers0_to_31")
  {
    return std::unique_ptr<DataFieldPublisher>(new BitRegisterArrayPublisher(data_field_identifier, nh, 0));
  }
  else if (data_field_identifier == "input_bit_registers32_to_63")
  {
    return std::unique_ptr<DataFieldPublisher>(new BitRegisterArrayPublisher(data_field_identifier, nh, 32));
  }
  else if (data_field_identifier == "tool_output_mode")
  {
    return std::unique_ptr<DataFieldPublisher>(
        new ModePublisher<uint8_t, ur_rtde_msgs::ToolMode>(data_field_identifier, nh));
  }
  else if (data_field_identifier == "tool_digital_output0_mode")
  {
    return std::unique_ptr<DataFieldPublisher>(
        new ModePublisher<uint8_t, ur_rtde_msgs::ToolMode>(data_field_identifier, nh));
  }
  else if (data_field_identifier == "tool_digital_output1_mode")
  {
    return std::unique_ptr<DataFieldPublisher>(
        new ModePublisher<uint8_t, ur_rtde_msgs::ToolMode>(data_field_identifier, nh));
  }
  else if (data_field_identifier == "input_bit_registers0_to_31")
  {
    return std::unique_ptr<DataFieldPublisher>(new BitRegisterArrayPublisher(data_field_identifier, nh, 0));
  }
  else if (data_field_identifier == "input_bit_registers32_to_63")
  {
    return std::unique_ptr<DataFieldPublisher>(new BitRegisterArrayPublisher(data_field_identifier, nh, 32));
  }
  else if (DataPackage::isType<bool>(data_field_identifier))
  {
    return std::unique_ptr<DataFieldPublisher>(new _bool_publisher(data_field_identifier, nh));
  }
  else if (DataPackage::isType<uint8_t>(data_field_identifier))
  {
    return std::unique_ptr<DataFieldPublisher>(new _uint8_publisher(data_field_identifier, nh));
  }
  else if (DataPackage::isType<uint32_t>(data_field_identifier))
  {
    return std::unique_ptr<DataFieldPublisher>(new _uint32_publisher(data_field_identifier, nh));
  }
  else if (DataPackage::isType<uint64_t>(data_field_identifier))
  {
    return std::unique_ptr<DataFieldPublisher>(new _uint64_publisher(data_field_identifier, nh));
  }
  else if (DataPackage::isType<int32_t>(data_field_identifier))
  {
    return std::unique_ptr<DataFieldPublisher>(new _int32_publisher(data_field_identifier, nh));
  }
  else if (DataPackage::isType<double_t>(data_field_identifier))
  {
    return std::unique_ptr<DataFieldPublisher>(new _double_publisher(data_field_identifier, nh));
  }
  else
  {
    throw UrException("No supported publisher for RTDE data field " + data_field_identifier);
  }
}
}  // namespace rtde_interface
}  // namespace ur_driver
