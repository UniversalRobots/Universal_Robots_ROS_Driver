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
 * \date    2019-04-10
 *
 */
//----------------------------------------------------------------------

#include "ur_rtde_driver/rtde/data_package.h"
namespace ur_driver
{
namespace rtde_interface
{
std::unordered_map<std::string, DataPackage::_rtde_type_variant> DataPackage::type_list_{
  { "timestamp_", double() },
  { "target_q_", vector6d_t() },
  { "target_qd_", vector6d_t() },
  { "target_qdd_", vector6d_t() },
  { "target_current_", vector6d_t() },
  { "target_moment_", vector6d_t() },
  { "actual_q_", vector6d_t() },
  { "actual_qd_", vector6d_t() },
  { "actual_qdd_", vector6d_t() },
  { "actual_current_", vector6d_t() },
  { "actual_moment_", vector6d_t() },
  { "joint_control_output_", vector6d_t() },
  { "actual_TCP_pose_", vector6d_t() },
  { "actual_TCP_speed_", vector6d_t() },
  { "actual_TCP_force_", vector6d_t() },
  { "target_TCP_pose_", vector6d_t() },
  { "target_TCP_speed_", vector6d_t() },
  { "actual_digital_input_bits_", uint64_t() },
  { "joint_temperatures_", vector6d_t() },
  { "actual_execution_time_", double() },
  { "robot_mode_", int32_t() },
  { "joint_mode", vector6int32_t() },
  { "safety_mode_", int32_t() },
  { "actual_tool_accelerometer", vector3d_t() },
  { "speed_scaling_", double() },
  { "target_speed_fraction_", double() },
  { "actual_momentum_", double() },
  { "actial_main_voltage_", double() },
  { "actual_robot_voltage_", double() },
  { "actual_robot_current_", double() },
  { "actual_joint_voltage_", vector6d_t() },
  { "actual_digital_output_bits_", uint64_t() },
  { "runtime_state_", uint32_t() },
  { "elbow_position_", vector3d_t() },
  { "elbow_velocity_", vector3d_t() },
  { "robot_status_bits_", uint32_t() },
  { "safety_status_bits_", uint32_t() },
  { "analog_io_types_", uint32_t() },
  { "standard_analog_input0_", double() },
  { "standard_analog_input1_", double() },
  { "standard_analog_output0_", double() },
  { "standard_analog_output1_", double() },
  { "io_current_", double() },
  { "euromap67_input_bits_", uint32_t() },
  { "euromap67_output_bits_", uint32_t() },
  { "euromap67_24V_voltage_", double() },
  { "euromap67_24V_current_", double() },
  { "tool_mode_", uint32_t() },
  { "tool_analog_input_types_", uint32_t() },
  { "tool_analog_input0_", double() },
  { "tool_analog_input1_", double() },
  { "tool_output_voltage_", int32_t() },
  { "tool_output_current_", double() },
  { "tool_temperature_", double() },
  { "tool_force_scalar_", double() },
  { "output_bit_registers0_to_31_", uint32_t() },
  { "output_bit_registers32_to_63_", uint32_t() },
  { "output_int_register_0_", int32_t() },
  { "output_int_register_1_", int32_t() },
  { "output_int_register_2_", int32_t() },
  { "output_int_register_3_", int32_t() },
  { "output_int_register_4_", int32_t() },
  { "output_int_register_5_", int32_t() },
  { "output_int_register_6_", int32_t() },
  { "output_int_register_7_", int32_t() },
  { "output_int_register_8_", int32_t() },
  { "output_int_register_9_", int32_t() },
  { "output_int_register_10_", int32_t() },
  { "output_int_register_11_", int32_t() },
  { "output_int_register_12_", int32_t() },
  { "output_int_register_13_", int32_t() },
  { "output_int_register_14_", int32_t() },
  { "output_int_register_15_", int32_t() },
  { "output_int_register_16_", int32_t() },
  { "output_int_register_17_", int32_t() },
  { "output_int_register_18_", int32_t() },
  { "output_int_register_19_", int32_t() },
  { "output_int_register_20_", int32_t() },
  { "output_int_register_21_", int32_t() },
  { "output_int_register_22_", int32_t() },
  { "output_int_register_23_", int32_t() },
  { "output_double_register_0_", double() },
  { "output_double_register_1_", double() },
  { "output_double_register_2_", double() },
  { "output_double_register_3_", double() },
  { "output_double_register_4_", double() },
  { "output_double_register_5_", double() },
  { "output_double_register_6_", double() },
  { "output_double_register_7_", double() },
  { "output_double_register_8_", double() },
  { "output_double_register_9_", double() },
  { "output_double_register_10_", double() },
  { "output_double_register_11_", double() },
  { "output_double_register_12_", double() },
  { "output_double_register_13_", double() },
  { "output_double_register_14_", double() },
  { "output_double_register_15_", double() },
  { "output_double_register_16_", double() },
  { "output_double_register_17_", double() },
  { "output_double_register_18_", double() },
  { "output_double_register_19_", double() },
  { "output_double_register_20_", double() },
  { "output_double_register_21_", double() },
  { "output_double_register_22_", double() },
  { "output_double_register_23_", double() },
  { "input_bit_registers0_to_31_", uint32_t() },
  { "input_bit_registers32_to_63_", uint32_t() },
  { "input_int_register_0_", int32_t() },
  { "input_int_register_1_", int32_t() },
  { "input_int_register_2_", int32_t() },
  { "input_int_register_3_", int32_t() },
  { "input_int_register_4_", int32_t() },
  { "input_int_register_5_", int32_t() },
  { "input_int_register_6_", int32_t() },
  { "input_int_register_7_", int32_t() },
  { "input_int_register_8_", int32_t() },
  { "input_int_register_9_", int32_t() },
  { "input_int_register_10_", int32_t() },
  { "input_int_register_11_", int32_t() },
  { "input_int_register_12_", int32_t() },
  { "input_int_register_13_", int32_t() },
  { "input_int_register_14_", int32_t() },
  { "input_int_register_15_", int32_t() },
  { "input_int_register_16_", int32_t() },
  { "input_int_register_17_", int32_t() },
  { "input_int_register_18_", int32_t() },
  { "input_int_register_19_", int32_t() },
  { "input_int_register_20_", int32_t() },
  { "input_int_register_21_", int32_t() },
  { "input_int_register_22_", int32_t() },
  { "input_int_register_23_", int32_t() },
  { "input_double_register_0_", double() },
  { "input_double_register_1_", double() },
  { "input_double_register_2_", double() },
  { "input_double_register_3_", double() },
  { "input_double_register_4_", double() },
  { "input_double_register_5_", double() },
  { "input_double_register_6_", double() },
  { "input_double_register_7_", double() },
  { "input_double_register_8_", double() },
  { "input_double_register_9_", double() },
  { "input_double_register_10_", double() },
  { "input_double_register_11_", double() },
  { "input_double_register_12_", double() },
  { "input_double_register_13_", double() },
  { "input_double_register_14_", double() },
  { "input_double_register_15_", double() },
  { "input_double_register_16_", double() },
  { "input_double_register_17_", double() },
  { "input_double_register_18_", double() },
  { "input_double_register_19_", double() },
  { "input_double_register_20_", double() },
  { "input_double_register_21_", double() },
  { "input_double_register_22_", double() },
  { "input_double_register_23_", double() }
};

bool rtde_interface::DataPackage ::parseWith(comm::BinParser& bp)
{
  for (auto& item : recipe_)
  {
    if (type_list_.find(item) != type_list_.end())
    {
      _rtde_type_variant entry = type_list_[item];
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
  vector3d_t vec1;
  vector6uint32_t vec2;
  ss << vec1 << vec2;
  for (auto& item : data_)
  {
    ss << item.first << ": ";
    ss << boost::apply_visitor(StringVisitor{}, item.second);
  }
  return ss.str();
}
}  // namespace rtde_interface
}  // namespace ur_driver
