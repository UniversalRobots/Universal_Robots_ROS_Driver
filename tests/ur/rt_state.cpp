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

#include "ur_modern_driver/ur/rt_state.h"
#include <gtest/gtest.h>
#include "ur_modern_driver/bin_parser.h"
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/test/random_data.h"
#include "ur_modern_driver/test/utils.h"
#include "ur_modern_driver/types.h"

TEST(RTState_V1_6__7, testRandomDataParsing)
{
  RandomDataTest rdt(764);
  BinParser bp = rdt.getParser(true);
  RTState_V1_6__7 state;
  EXPECT_TRUE(state.parseWith(bp)) << "parse() returned false";

  ASSERT_EQ(rdt.getNext<double>(), state.time);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.q_target);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.qd_target);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.qdd_target);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.i_target);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.m_target);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.q_actual);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.qd_actual);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.i_actual);
  ASSERT_EQ(rdt.getNext<double3_t>(), state.tool_accelerometer_values);
  rdt.skip(sizeof(double) * 15);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.tcp_force);
  ASSERT_EQ(rdt.getNext<cartesian_coord_t>(), state.tool_vector_actual);
  ASSERT_EQ(rdt.getNext<cartesian_coord_t>(), state.tcp_speed_actual);
  ASSERT_EQ(rdt.getNext<uint64_t>(), state.digital_inputs);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.motor_temperatures);
  ASSERT_EQ(rdt.getNext<double>(), state.controller_time);
  rdt.skip(sizeof(double));  // skip unused value
  ASSERT_EQ(rdt.getNext<double>(), state.robot_mode);

  EXPECT_TRUE(bp.empty()) << "Did not consume all data";
}

TEST(RTState_V1_8, testRandomDataParsing)
{
  RandomDataTest rdt(812);
  BinParser bp = rdt.getParser(true);
  RTState_V1_8 state;
  EXPECT_TRUE(state.parseWith(bp)) << "parse() returned false";

  ASSERT_EQ(rdt.getNext<double>(), state.time);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.q_target);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.qd_target);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.qdd_target);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.i_target);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.m_target);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.q_actual);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.qd_actual);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.i_actual);
  ASSERT_EQ(rdt.getNext<double3_t>(), state.tool_accelerometer_values);
  rdt.skip(sizeof(double) * 15);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.tcp_force);
  ASSERT_EQ(rdt.getNext<cartesian_coord_t>(), state.tool_vector_actual);
  ASSERT_EQ(rdt.getNext<cartesian_coord_t>(), state.tcp_speed_actual);
  ASSERT_EQ(rdt.getNext<uint64_t>(), state.digital_inputs);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.motor_temperatures);
  ASSERT_EQ(rdt.getNext<double>(), state.controller_time);
  rdt.skip(sizeof(double));  // skip unused value
  ASSERT_EQ(rdt.getNext<double>(), state.robot_mode);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.joint_modes);

  EXPECT_TRUE(bp.empty()) << "Did not consume all data";
}

TEST(RTState_V3_0__1, testRandomDataParsing)
{
  RandomDataTest rdt(1044);
  BinParser bp = rdt.getParser(true);
  RTState_V3_0__1 state;
  EXPECT_TRUE(state.parseWith(bp)) << "parse() returned false";

  ASSERT_EQ(rdt.getNext<double>(), state.time);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.q_target);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.qd_target);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.qdd_target);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.i_target);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.m_target);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.q_actual);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.qd_actual);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.i_actual);

  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.i_control);
  ASSERT_EQ(rdt.getNext<cartesian_coord_t>(), state.tool_vector_actual);
  ASSERT_EQ(rdt.getNext<cartesian_coord_t>(), state.tcp_speed_actual);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.tcp_force);
  ASSERT_EQ(rdt.getNext<cartesian_coord_t>(), state.tool_vector_target);
  ASSERT_EQ(rdt.getNext<cartesian_coord_t>(), state.tcp_speed_target);

  ASSERT_EQ(rdt.getNext<uint64_t>(), state.digital_inputs);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.motor_temperatures);
  ASSERT_EQ(rdt.getNext<double>(), state.controller_time);
  rdt.skip(sizeof(double));  // skip unused value
  ASSERT_EQ(rdt.getNext<double>(), state.robot_mode);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.joint_modes);
  ASSERT_EQ(rdt.getNext<double>(), state.safety_mode);
  rdt.skip(sizeof(double) * 6);
  ASSERT_EQ(rdt.getNext<double3_t>(), state.tool_accelerometer_values);
  rdt.skip(sizeof(double) * 6);
  ASSERT_EQ(rdt.getNext<double>(), state.speed_scaling);
  ASSERT_EQ(rdt.getNext<double>(), state.linear_momentum_norm);
  rdt.skip(sizeof(double) * 2);
  ASSERT_EQ(rdt.getNext<double>(), state.v_main);
  ASSERT_EQ(rdt.getNext<double>(), state.v_robot);
  ASSERT_EQ(rdt.getNext<double>(), state.i_robot);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.v_actual);

  EXPECT_TRUE(bp.empty()) << "Did not consume all data";
}

TEST(RTState_V3_2__3, testRandomDataParsing)
{
  RandomDataTest rdt(1060);
  BinParser bp = rdt.getParser(true);
  RTState_V3_2__3 state;
  EXPECT_TRUE(state.parseWith(bp)) << "parse() returned false";

  ASSERT_EQ(rdt.getNext<double>(), state.time);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.q_target);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.qd_target);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.qdd_target);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.i_target);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.m_target);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.q_actual);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.qd_actual);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.i_actual);

  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.i_control);
  ASSERT_EQ(rdt.getNext<cartesian_coord_t>(), state.tool_vector_actual);
  ASSERT_EQ(rdt.getNext<cartesian_coord_t>(), state.tcp_speed_actual);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.tcp_force);
  ASSERT_EQ(rdt.getNext<cartesian_coord_t>(), state.tool_vector_target);
  ASSERT_EQ(rdt.getNext<cartesian_coord_t>(), state.tcp_speed_target);

  ASSERT_EQ(rdt.getNext<uint64_t>(), state.digital_inputs);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.motor_temperatures);
  ASSERT_EQ(rdt.getNext<double>(), state.controller_time);
  rdt.skip(sizeof(double));  // skip unused value
  ASSERT_EQ(rdt.getNext<double>(), state.robot_mode);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.joint_modes);
  ASSERT_EQ(rdt.getNext<double>(), state.safety_mode);
  rdt.skip(sizeof(double) * 6);
  ASSERT_EQ(rdt.getNext<double3_t>(), state.tool_accelerometer_values);
  rdt.skip(sizeof(double) * 6);
  ASSERT_EQ(rdt.getNext<double>(), state.speed_scaling);
  ASSERT_EQ(rdt.getNext<double>(), state.linear_momentum_norm);
  rdt.skip(sizeof(double) * 2);
  ASSERT_EQ(rdt.getNext<double>(), state.v_main);
  ASSERT_EQ(rdt.getNext<double>(), state.v_robot);
  ASSERT_EQ(rdt.getNext<double>(), state.i_robot);
  ASSERT_DOUBLE_ARRAY_EQ(rdt.getNext<double>(), state.v_actual);
  ASSERT_EQ(rdt.getNext<uint64_t>(), state.digital_outputs);
  ASSERT_EQ(rdt.getNext<double>(), state.program_state);

  EXPECT_TRUE(bp.empty()) << "did not consume all data";
}

TEST(RTState_V1_6__7, testTooSmallBuffer)
{
  RandomDataTest rdt(10);
  BinParser bp = rdt.getParser(true);
  RTState_V1_6__7 state;
  EXPECT_FALSE(state.parseWith(bp)) << "parse() should fail when buffer not big enough";
}

TEST(RTState_V1_8, testTooSmallBuffer)
{
  RandomDataTest rdt(10);
  BinParser bp = rdt.getParser(true);
  RTState_V1_8 state;
  EXPECT_FALSE(state.parseWith(bp)) << "parse() should fail when buffer not big enough";
}

TEST(RTState_V3_0__1, testTooSmallBuffer)
{
  RandomDataTest rdt(10);
  BinParser bp = rdt.getParser(true);
  RTState_V3_0__1 state;
  EXPECT_FALSE(state.parseWith(bp)) << "parse() should fail when buffer not big enough";
}

TEST(RTState_V3_2__3, testTooSmallBuffer)
{
  RandomDataTest rdt(10);
  BinParser bp = rdt.getParser(true);
  RTState_V3_2__3 state;
  EXPECT_FALSE(state.parseWith(bp)) << "parse() should fail when buffer not big enough";
}