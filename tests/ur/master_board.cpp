/*
 * Copyright 2017, 2018 Simon Rasmussen (refactor)
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

#include "ur_modern_driver/ur/master_board.h"
#include <gtest/gtest.h>
#include "ur_modern_driver/bin_parser.h"
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/test/random_data.h"
#include "ur_modern_driver/test/utils.h"
#include "ur_modern_driver/types.h"

TEST(MasterBoardData_V1_X, testRandomDataParsing)
{
  RandomDataTest rdt(71);
  rdt.set<uint8_t>(1, 58);  // sets euromap67_interface_installed to true
  BinParser bp = rdt.getParser();
  MasterBoardData_V1_X state;

  ASSERT_TRUE(state.parseWith(bp)) << "parse() returned false";

  ASSERT_EQ((rdt.getNext<uint16_t, 10>()), state.digital_input_bits);
  ASSERT_EQ((rdt.getNext<uint16_t, 10>()), state.digital_output_bits);
  ASSERT_EQ(rdt.getNext<int8_t>(), state.analog_input_range0);
  ASSERT_EQ(rdt.getNext<int8_t>(), state.analog_input_range1);
  ASSERT_EQ(rdt.getNext<double>(), state.analog_input0);
  ASSERT_EQ(rdt.getNext<double>(), state.analog_input1);
  ASSERT_EQ(rdt.getNext<int8_t>(), state.analog_output_domain0);
  ASSERT_EQ(rdt.getNext<int8_t>(), state.analog_output_domain1);
  ASSERT_EQ(rdt.getNext<double>(), state.analog_output0);
  ASSERT_EQ(rdt.getNext<double>(), state.analog_output1);
  ASSERT_EQ(rdt.getNext<float>(), state.master_board_temperature);
  ASSERT_EQ(rdt.getNext<float>(), state.robot_voltage_48V);
  ASSERT_EQ(rdt.getNext<float>(), state.robot_current);
  ASSERT_EQ(rdt.getNext<float>(), state.master_IO_current);
  ASSERT_EQ(rdt.getNext<uint8_t>(), state.master_safety_state);
  ASSERT_EQ(rdt.getNext<bool>(), state.master_on_off_state);
  ASSERT_EQ(rdt.getNext<bool>(), state.euromap67_interface_installed);
  ASSERT_EQ(rdt.getNext<int32_t>(), state.euromap_input_bits);
  ASSERT_EQ(rdt.getNext<int32_t>(), state.euromap_output_bits);
  ASSERT_EQ(rdt.getNext<int16_t>(), state.euromap_voltage);
  ASSERT_EQ(rdt.getNext<int16_t>(), state.euromap_current);

  ASSERT_TRUE(bp.empty()) << "Did not consume all data";
}

TEST(MasterBoardData_V3_0__1, testRandomDataParsing)
{
  RandomDataTest rdt(83);
  rdt.set<uint8_t>(1, 62);  // sets euromap67_interface_installed to true
  BinParser bp = rdt.getParser();
  MasterBoardData_V3_0__1 state;
  ASSERT_TRUE(state.parseWith(bp)) << "parse() returned false";

  ASSERT_EQ((rdt.getNext<uint32_t, 18>()), state.digital_input_bits);
  ASSERT_EQ((rdt.getNext<uint32_t, 18>()), state.digital_output_bits);
  ASSERT_EQ(rdt.getNext<int8_t>(), state.analog_input_range0);
  ASSERT_EQ(rdt.getNext<int8_t>(), state.analog_input_range1);
  ASSERT_EQ(rdt.getNext<double>(), state.analog_input0);
  ASSERT_EQ(rdt.getNext<double>(), state.analog_input1);
  ASSERT_EQ(rdt.getNext<int8_t>(), state.analog_output_domain0);
  ASSERT_EQ(rdt.getNext<int8_t>(), state.analog_output_domain1);
  ASSERT_EQ(rdt.getNext<double>(), state.analog_output0);
  ASSERT_EQ(rdt.getNext<double>(), state.analog_output1);
  ASSERT_EQ(rdt.getNext<float>(), state.master_board_temperature);
  ASSERT_EQ(rdt.getNext<float>(), state.robot_voltage_48V);
  ASSERT_EQ(rdt.getNext<float>(), state.robot_current);
  ASSERT_EQ(rdt.getNext<float>(), state.master_IO_current);
  ASSERT_EQ(rdt.getNext<uint8_t>(), state.safety_mode);
  ASSERT_EQ(rdt.getNext<bool>(), state.in_reduced_mode);
  ASSERT_EQ(rdt.getNext<bool>(), state.euromap67_interface_installed);
  ASSERT_EQ(rdt.getNext<int32_t>(), state.euromap_input_bits);
  ASSERT_EQ(rdt.getNext<int32_t>(), state.euromap_output_bits);
  ASSERT_EQ(rdt.getNext<float>(), state.euromap_voltage);
  ASSERT_EQ(rdt.getNext<float>(), state.euromap_current);

  rdt.skip(sizeof(uint32_t));

  ASSERT_TRUE(bp.empty()) << "Did not consume all data";
}

TEST(MasterBoardData_V3_2, testRandomDataParsing)
{
  RandomDataTest rdt(85);
  rdt.set<uint8_t>(1, 62);  // sets euromap67_interface_installed to true
  BinParser bp = rdt.getParser();
  MasterBoardData_V3_2 state;
  ASSERT_TRUE(state.parseWith(bp)) << "parse() returned false";

  ASSERT_EQ((rdt.getNext<uint32_t, 18>()), state.digital_input_bits);
  ASSERT_EQ((rdt.getNext<uint32_t, 18>()), state.digital_output_bits);
  ASSERT_EQ(rdt.getNext<int8_t>(), state.analog_input_range0);
  ASSERT_EQ(rdt.getNext<int8_t>(), state.analog_input_range1);
  ASSERT_EQ(rdt.getNext<double>(), state.analog_input0);
  ASSERT_EQ(rdt.getNext<double>(), state.analog_input1);
  ASSERT_EQ(rdt.getNext<int8_t>(), state.analog_output_domain0);
  ASSERT_EQ(rdt.getNext<int8_t>(), state.analog_output_domain1);
  ASSERT_EQ(rdt.getNext<double>(), state.analog_output0);
  ASSERT_EQ(rdt.getNext<double>(), state.analog_output1);
  ASSERT_EQ(rdt.getNext<float>(), state.master_board_temperature);
  ASSERT_EQ(rdt.getNext<float>(), state.robot_voltage_48V);
  ASSERT_EQ(rdt.getNext<float>(), state.robot_current);
  ASSERT_EQ(rdt.getNext<float>(), state.master_IO_current);
  ASSERT_EQ(rdt.getNext<uint8_t>(), state.safety_mode);
  ASSERT_EQ(rdt.getNext<bool>(), state.in_reduced_mode);
  ASSERT_EQ(rdt.getNext<bool>(), state.euromap67_interface_installed);
  ASSERT_EQ(rdt.getNext<int32_t>(), state.euromap_input_bits);
  ASSERT_EQ(rdt.getNext<int32_t>(), state.euromap_output_bits);
  ASSERT_EQ(rdt.getNext<float>(), state.euromap_voltage);
  ASSERT_EQ(rdt.getNext<float>(), state.euromap_current);
  rdt.skip(sizeof(uint32_t));
  ASSERT_EQ(rdt.getNext<uint8_t>(), state.operational_mode_selector_input);
  ASSERT_EQ(rdt.getNext<uint8_t>(), state.three_position_enabling_device_input);

  ASSERT_TRUE(bp.empty()) << "Did not consume all data";
}

TEST(MasterBoardData_V1_X, testTooSmallBuffer)
{
  RandomDataTest rdt(10);
  BinParser bp = rdt.getParser();
  MasterBoardData_V1_X state;
  EXPECT_FALSE(state.parseWith(bp)) << "parse() should fail when buffer not big enough";
}

TEST(MasterBoardData_V3_0__1, testTooSmallBuffer)
{
  RandomDataTest rdt(10);
  BinParser bp = rdt.getParser();
  MasterBoardData_V3_0__1 state;
  EXPECT_FALSE(state.parseWith(bp)) << "parse() should fail when buffer not big enough";
}

TEST(MasterBoardData_V3_2, testTooSmallBuffer)
{
  RandomDataTest rdt(10);
  BinParser bp = rdt.getParser();
  MasterBoardData_V3_2 state;
  EXPECT_FALSE(state.parseWith(bp)) << "parse() should fail when buffer not big enough";
}
