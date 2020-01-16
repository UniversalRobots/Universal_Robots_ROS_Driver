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
 * \date    2019-04-10
 *
 */
//----------------------------------------------------------------------

#include "ur_robot_driver/rtde/data_package.h"
namespace ur_driver
{
namespace rtde_interface
{
std::unordered_map<std::string, DataPackage::_rtde_type_variant> DataPackage::g_type_list{
  { "timestamp", double() },
  { "target_q", vector6d_t() },
  { "target_qd", vector6d_t() },
  { "target_qdd", vector6d_t() },
  { "target_current", vector6d_t() },
  { "target_moment", vector6d_t() },
  { "actual_q", vector6d_t() },
  { "actual_qd", vector6d_t() },
  { "actual_qdd", vector6d_t() },
  { "actual_current", vector6d_t() },
  { "actual_moment", vector6d_t() },
  { "joint_control_output", vector6d_t() },
  { "actual_TCP_pose", vector6d_t() },
  { "actual_TCP_speed", vector6d_t() },
  { "actual_TCP_force", vector6d_t() },
  { "target_TCP_pose", vector6d_t() },
  { "target_TCP_speed", vector6d_t() },
  { "actual_digital_input_bits", uint64_t() },
  { "joint_temperatures", vector6d_t() },
  { "actual_execution_time", double() },
  { "robot_mode", int32_t() },
  { "joint_mode", vector6int32_t() },
  { "safety_mode", int32_t() },
  { "actual_tool_accelerometer", vector3d_t() },
  { "speed_scaling", double() },
  { "target_speed_fraction", double() },
  { "actual_momentum", double() },
  { "actual_main_voltage", double() },
  { "actual_robot_voltage", double() },
  { "actual_robot_current", double() },
  { "actual_joint_voltage", vector6d_t() },
  { "actual_digital_output_bits", uint64_t() },
  { "runtime_state", uint32_t() },
  { "elbow_position", vector3d_t() },
  { "elbow_velocity", vector3d_t() },
  { "robot_status_bits", uint32_t() },
  { "safety_status_bits", uint32_t() },
  { "analog_io_types", uint32_t() },
  { "standard_analog_input0", double() },
  { "standard_analog_input1", double() },
  { "standard_analog_output0", double() },
  { "standard_analog_output1", double() },
  { "io_current", double() },
  { "euromap67_input_bits", uint32_t() },
  { "euromap67_output_bits", uint32_t() },
  { "euromap67_24V_voltage", double() },
  { "euromap67_24V_current", double() },
  { "tool_mode", uint32_t() },
  { "tool_analog_input_types", uint32_t() },
  { "tool_analog_input0", double() },
  { "tool_analog_input1", double() },
  { "tool_output_voltage", int32_t() },
  { "tool_output_current", double() },
  { "tool_temperature", double() },
  { "tool_force_scalar", double() },
  { "output_bit_registers0_to_31", uint32_t() },
  { "output_bit_registers32_to_63", uint32_t() },
  { "output_int_register_0", int32_t() },
  { "output_int_register_1", int32_t() },
  { "output_int_register_2", int32_t() },
  { "output_int_register_3", int32_t() },
  { "output_int_register_4", int32_t() },
  { "output_int_register_5", int32_t() },
  { "output_int_register_6", int32_t() },
  { "output_int_register_7", int32_t() },
  { "output_int_register_8", int32_t() },
  { "output_int_register_9", int32_t() },
  { "output_int_register_10", int32_t() },
  { "output_int_register_11", int32_t() },
  { "output_int_register_12", int32_t() },
  { "output_int_register_13", int32_t() },
  { "output_int_register_14", int32_t() },
  { "output_int_register_15", int32_t() },
  { "output_int_register_16", int32_t() },
  { "output_int_register_17", int32_t() },
  { "output_int_register_18", int32_t() },
  { "output_int_register_19", int32_t() },
  { "output_int_register_20", int32_t() },
  { "output_int_register_21", int32_t() },
  { "output_int_register_22", int32_t() },
  { "output_int_register_23", int32_t() },
  { "output_double_register_0", double() },
  { "output_double_register_1", double() },
  { "output_double_register_2", double() },
  { "output_double_register_3", double() },
  { "output_double_register_4", double() },
  { "output_double_register_5", double() },
  { "output_double_register_6", double() },
  { "output_double_register_7", double() },
  { "output_double_register_8", double() },
  { "output_double_register_9", double() },
  { "output_double_register_10", double() },
  { "output_double_register_11", double() },
  { "output_double_register_12", double() },
  { "output_double_register_13", double() },
  { "output_double_register_14", double() },
  { "output_double_register_15", double() },
  { "output_double_register_16", double() },
  { "output_double_register_17", double() },
  { "output_double_register_18", double() },
  { "output_double_register_19", double() },
  { "output_double_register_20", double() },
  { "output_double_register_21", double() },
  { "output_double_register_22", double() },
  { "output_double_register_23", double() },
  { "input_bit_registers0_to_31", uint32_t() },
  { "input_bit_registers32_to_63", uint32_t() },
  { "input_bit_register_0", bool() },
  { "input_bit_register_1", bool() },
  { "input_bit_register_2", bool() },
  { "input_bit_register_3", bool() },
  { "input_bit_register_4", bool() },
  { "input_bit_register_5", bool() },
  { "input_bit_register_6", bool() },
  { "input_bit_register_7", bool() },
  { "input_bit_register_8", bool() },
  { "input_bit_register_9", bool() },
  { "input_bit_register_10", bool() },
  { "input_bit_register_11", bool() },
  { "input_bit_register_12", bool() },
  { "input_bit_register_13", bool() },
  { "input_bit_register_14", bool() },
  { "input_bit_register_15", bool() },
  { "input_bit_register_16", bool() },
  { "input_bit_register_17", bool() },
  { "input_bit_register_18", bool() },
  { "input_bit_register_19", bool() },
  { "input_bit_register_20", bool() },
  { "input_bit_register_21", bool() },
  { "input_bit_register_22", bool() },
  { "input_bit_register_23", bool() },
  { "input_int_register_0", int32_t() },
  { "input_int_register_1", int32_t() },
  { "input_int_register_2", int32_t() },
  { "input_int_register_3", int32_t() },
  { "input_int_register_4", int32_t() },
  { "input_int_register_5", int32_t() },
  { "input_int_register_6", int32_t() },
  { "input_int_register_7", int32_t() },
  { "input_int_register_8", int32_t() },
  { "input_int_register_9", int32_t() },
  { "input_int_register_10", int32_t() },
  { "input_int_register_11", int32_t() },
  { "input_int_register_12", int32_t() },
  { "input_int_register_13", int32_t() },
  { "input_int_register_14", int32_t() },
  { "input_int_register_15", int32_t() },
  { "input_int_register_16", int32_t() },
  { "input_int_register_17", int32_t() },
  { "input_int_register_18", int32_t() },
  { "input_int_register_19", int32_t() },
  { "input_int_register_20", int32_t() },
  { "input_int_register_21", int32_t() },
  { "input_int_register_22", int32_t() },
  { "input_int_register_23", int32_t() },
  { "input_double_register_0", double() },
  { "input_double_register_1", double() },
  { "input_double_register_2", double() },
  { "input_double_register_3", double() },
  { "input_double_register_4", double() },
  { "input_double_register_5", double() },
  { "input_double_register_6", double() },
  { "input_double_register_7", double() },
  { "input_double_register_8", double() },
  { "input_double_register_9", double() },
  { "input_double_register_10", double() },
  { "input_double_register_11", double() },
  { "input_double_register_12", double() },
  { "input_double_register_13", double() },
  { "input_double_register_14", double() },
  { "input_double_register_15", double() },
  { "input_double_register_16", double() },
  { "input_double_register_17", double() },
  { "input_double_register_18", double() },
  { "input_double_register_19", double() },
  { "input_double_register_20", double() },
  { "input_double_register_21", double() },
  { "input_double_register_22", double() },
  { "input_double_register_23", double() },
  { "speed_slider_mask", uint32_t() },
  { "speed_slider_fraction", double() },
  { "standard_digital_output_mask", uint8_t() },
  { "standard_digital_output", uint8_t() },
  { "configurable_digital_output_mask", uint8_t() },
  { "configurable_digital_output", uint8_t() },
  { "tool_digital_output_mask", uint8_t() },
  { "tool_digital_output", uint8_t() },
  { "standard_analog_output_mask", uint8_t() },
  { "standard_analog_output_type", uint8_t() },
  { "standard_analog_output_0", double() },
  { "standard_analog_output_1", double() },
};

void rtde_interface::DataPackage::initEmpty()
{
  for (auto& item : recipe_)
  {
    if (g_type_list.find(item) != g_type_list.end())
    {
      _rtde_type_variant entry = g_type_list[item];
      data_[item] = entry;
    }
  }
}

bool rtde_interface::DataPackage::parseWith(comm::BinParser& bp)
{
  bp.parse(recipe_id_);
  for (auto& item : recipe_)
  {
    if (g_type_list.find(item) != g_type_list.end())
    {
      _rtde_type_variant entry = g_type_list[item];
      auto bound_visitor = std::bind(ParseVisitor(), std::placeholders::_1, bp);
      boost::apply_visitor(bound_visitor, entry);
      data_[item] = entry;
    }
    else
    {
      return false;
    }
  }
  return true;
}

std::string rtde_interface::DataPackage::toString() const
{
  std::stringstream ss;
  for (auto& item : data_)
  {
    ss << item.first << ": ";
    ss << boost::apply_visitor(StringVisitor{}, item.second) << std::endl;
  }
  return ss.str();
}

size_t rtde_interface::DataPackage::serializePackage(uint8_t* buffer)
{
  uint16_t payload_size = sizeof(recipe_id_);

  for (auto& item : data_)
  {
    payload_size += boost::apply_visitor(SizeVisitor{}, item.second);
  }
  size_t size = 0;
  size += PackageHeader::serializeHeader(buffer, PackageType::RTDE_DATA_PACKAGE, payload_size);
  size += comm::PackageSerializer::serialize(buffer + size, recipe_id_);
  for (auto& item : recipe_)
  {
    auto bound_visitor = std::bind(SerializeVisitor(), std::placeholders::_1, buffer + size);
    size += boost::apply_visitor(bound_visitor, data_[item]);
  }

  return size;
}
}  // namespace rtde_interface
}  // namespace ur_driver
