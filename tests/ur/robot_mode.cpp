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

#include "ur_modern_driver/ur/robot_mode.h"
#include <gtest/gtest.h>
#include "ur_modern_driver/bin_parser.h"
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/test/random_data.h"
#include "ur_modern_driver/test/utils.h"
#include "ur_modern_driver/types.h"

TEST(RobotModeData_V1_X, testRandomDataParsing)
{
  RandomDataTest rdt(24);
  BinParser bp = rdt.getParser();
  RobotModeData_V1_X state;
  ASSERT_TRUE(state.parseWith(bp)) << "parse() returned false";

  ASSERT_EQ(rdt.getNext<uint64_t>(), state.timestamp);
  ASSERT_EQ(rdt.getNext<bool>(), state.physical_robot_connected);
  ASSERT_EQ(rdt.getNext<bool>(), state.real_robot_enabled);
  ASSERT_EQ(rdt.getNext<bool>(), state.robot_power_on);
  ASSERT_EQ(rdt.getNext<bool>(), state.emergency_stopped);
  ASSERT_EQ(rdt.getNext<bool>(), state.protective_stopped);
  ASSERT_EQ(rdt.getNext<bool>(), state.program_running);
  ASSERT_EQ(rdt.getNext<bool>(), state.program_paused);
  ASSERT_EQ(rdt.getNext<robot_mode_V1_X>(), state.robot_mode);
  ASSERT_EQ(rdt.getNext<double>(), state.speed_fraction);

  ASSERT_TRUE(bp.empty()) << "Did not consume all data";
}

TEST(RobotModeData_V3_0__1, testRandomDataParsing)
{
  RandomDataTest rdt(33);
  BinParser bp = rdt.getParser();
  RobotModeData_V3_0__1 state;
  ASSERT_TRUE(state.parseWith(bp)) << "parse() returned false";

  ASSERT_EQ(rdt.getNext<uint64_t>(), state.timestamp);
  ASSERT_EQ(rdt.getNext<bool>(), state.physical_robot_connected);
  ASSERT_EQ(rdt.getNext<bool>(), state.real_robot_enabled);
  ASSERT_EQ(rdt.getNext<bool>(), state.robot_power_on);
  ASSERT_EQ(rdt.getNext<bool>(), state.emergency_stopped);
  ASSERT_EQ(rdt.getNext<bool>(), state.protective_stopped);
  ASSERT_EQ(rdt.getNext<bool>(), state.program_running);
  ASSERT_EQ(rdt.getNext<bool>(), state.program_paused);
  ASSERT_EQ(rdt.getNext<robot_mode_V3_X>(), state.robot_mode);
  ASSERT_EQ(rdt.getNext<robot_control_mode_V3_X>(), state.control_mode);
  ASSERT_EQ(rdt.getNext<double>(), state.target_speed_fraction);
  ASSERT_EQ(rdt.getNext<double>(), state.speed_scaling);

  ASSERT_TRUE(bp.empty()) << "Did not consume all data";
}

TEST(RobotModeData_V3_2, testRandomDataParsing)
{
  RandomDataTest rdt(41);
  BinParser bp = rdt.getParser();
  RobotModeData_V3_2 state;
  ASSERT_TRUE(state.parseWith(bp)) << "parse() returned false";

  ASSERT_EQ(rdt.getNext<uint64_t>(), state.timestamp);
  ASSERT_EQ(rdt.getNext<bool>(), state.physical_robot_connected);
  ASSERT_EQ(rdt.getNext<bool>(), state.real_robot_enabled);
  ASSERT_EQ(rdt.getNext<bool>(), state.robot_power_on);
  ASSERT_EQ(rdt.getNext<bool>(), state.emergency_stopped);
  ASSERT_EQ(rdt.getNext<bool>(), state.protective_stopped);
  ASSERT_EQ(rdt.getNext<bool>(), state.program_running);
  ASSERT_EQ(rdt.getNext<bool>(), state.program_paused);
  ASSERT_EQ(rdt.getNext<robot_mode_V3_X>(), state.robot_mode);
  ASSERT_EQ(rdt.getNext<robot_control_mode_V3_X>(), state.control_mode);
  ASSERT_EQ(rdt.getNext<double>(), state.target_speed_fraction);
  ASSERT_EQ(rdt.getNext<double>(), state.speed_scaling);
  ASSERT_EQ(rdt.getNext<double>(), state.target_speed_fraction_limit);

  ASSERT_TRUE(bp.empty()) << "Did not consume all data";
}

TEST(RobotModeData_V1_X, testTooSmallBuffer)
{
  RandomDataTest rdt(10);
  BinParser bp = rdt.getParser();
  RobotModeData_V1_X state;
  EXPECT_FALSE(state.parseWith(bp)) << "parse() should fail when buffer not big enough";
}

TEST(RobotModeData_V3_0__1, testTooSmallBuffer)
{
  RandomDataTest rdt(10);
  BinParser bp = rdt.getParser();
  RobotModeData_V3_0__1 state;
  EXPECT_FALSE(state.parseWith(bp)) << "parse() should fail when buffer not big enough";
}

TEST(RobotModeData_V3_2, testTooSmallBuffer)
{
  RandomDataTest rdt(10);
  BinParser bp = rdt.getParser();
  RobotModeData_V3_2 state;
  EXPECT_FALSE(state.parseWith(bp)) << "parse() should fail when buffer not big enough";
}
