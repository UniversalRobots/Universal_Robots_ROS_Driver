/*
 * Copyright 2017, 2018 Simon Rasmussen (refactor)
 *
 * Copyright 2015, 2016 Thomas Timm Andersen (original version)
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
#include "ur_modern_driver/ur/consumer.h"

void RTShared::parse_shared1(BinParser& bp)
{
  bp.parse(time);
  bp.parse(q_target);
  bp.parse(qd_target);
  bp.parse(qdd_target);
  bp.parse(i_target);
  bp.parse(m_target);
  bp.parse(q_actual);
  bp.parse(qd_actual);
  bp.parse(i_actual);
}

void RTShared::parse_shared2(BinParser& bp)
{
  bp.parse(digital_inputs);
  bp.parse(motor_temperatures);
  bp.parse(controller_time);
  bp.consume(sizeof(double));  // Unused "Test value" field
  bp.parse(robot_mode);
}

bool RTState_V1_6__7::parseWith(BinParser& bp)
{
  if (!bp.checkSize<RTState_V1_6__7>())
    return false;

  parse_shared1(bp);

  bp.parse(tool_accelerometer_values);
  bp.consume(sizeof(double) * 15);
  bp.parse(tcp_force);
  bp.parse(tool_vector_actual);
  bp.parse(tcp_speed_actual);

  parse_shared2(bp);

  return true;
}

bool RTState_V1_8::parseWith(BinParser& bp)
{
  if (!bp.checkSize<RTState_V1_8>())
    return false;

  RTState_V1_6__7::parseWith(bp);

  bp.parse(joint_modes);

  return true;
}

bool RTState_V3_0__1::parseWith(BinParser& bp)
{
  if (!bp.checkSize<RTState_V3_0__1>())
    return false;

  parse_shared1(bp);

  bp.parse(i_control);
  bp.parse(tool_vector_actual);
  bp.parse(tcp_speed_actual);
  bp.parse(tcp_force);
  bp.parse(tool_vector_target);
  bp.parse(tcp_speed_target);

  parse_shared2(bp);

  bp.parse(joint_modes);
  bp.parse(safety_mode);
  bp.consume(sizeof(double[6]));  // skip undocumented
  bp.parse(tool_accelerometer_values);
  bp.consume(sizeof(double[6]));  // skip undocumented
  bp.parse(speed_scaling);
  bp.parse(linear_momentum_norm);
  bp.consume(sizeof(double));  // skip undocumented
  bp.consume(sizeof(double));  // skip undocumented
  bp.parse(v_main);
  bp.parse(v_robot);
  bp.parse(i_robot);
  bp.parse(v_actual);

  return true;
}

bool RTState_V3_2__3::parseWith(BinParser& bp)
{
  if (!bp.checkSize<RTState_V3_2__3>())
    return false;

  RTState_V3_0__1::parseWith(bp);

  bp.parse(digital_outputs);
  bp.parse(program_state);

  return true;
}

bool RTState_V1_6__7::consumeWith(URRTPacketConsumer& consumer)
{
  return consumer.consume(*this);
}
bool RTState_V1_8::consumeWith(URRTPacketConsumer& consumer)
{
  return consumer.consume(*this);
}
bool RTState_V3_0__1::consumeWith(URRTPacketConsumer& consumer)
{
  return consumer.consume(*this);
}
bool RTState_V3_2__3::consumeWith(URRTPacketConsumer& consumer)
{
  return consumer.consume(*this);
}