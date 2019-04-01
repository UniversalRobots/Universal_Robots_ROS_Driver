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

#include "ur_modern_driver/ur/master_board.h"
#include "ur_modern_driver/ur/consumer.h"

bool SharedMasterBoardData::parseWith(BinParser& bp)
{
  bp.parse(analog_input_range0);
  bp.parse(analog_input_range1);
  bp.parse(analog_input0);
  bp.parse(analog_input1);
  bp.parse(analog_output_domain0);
  bp.parse(analog_output_domain1);
  bp.parse(analog_output0);
  bp.parse(analog_output1);
  bp.parse(master_board_temperature);
  bp.parse(robot_voltage_48V);
  bp.parse(robot_current);
  bp.parse(master_IO_current);
  return true;
}

bool MasterBoardData_V1_X::parseWith(BinParser& bp)
{
  if (!bp.checkSize<MasterBoardData_V1_X>())
    return false;

  bp.parse<uint16_t, 10>(digital_input_bits);
  bp.parse<uint16_t, 10>(digital_output_bits);

  SharedMasterBoardData::parseWith(bp);

  bp.parse(master_safety_state);
  bp.parse(master_on_off_state);
  bp.parse(euromap67_interface_installed);

  if (euromap67_interface_installed)
  {
    if (!bp.checkSize(MasterBoardData_V1_X::EURO_SIZE))
      return false;

    bp.parse(euromap_input_bits);
    bp.parse(euromap_output_bits);
    bp.parse(euromap_voltage);
    bp.parse(euromap_current);
  }

  return true;
}

bool MasterBoardData_V3_0__1::parseWith(BinParser& bp)
{
  if (!bp.checkSize<MasterBoardData_V3_0__1>())
    return false;

  bp.parse<uint32_t, 18>(digital_input_bits);
  bp.parse<uint32_t, 18>(digital_output_bits);

  SharedMasterBoardData::parseWith(bp);

  bp.parse(safety_mode);
  bp.parse(in_reduced_mode);
  bp.parse(euromap67_interface_installed);

  if (euromap67_interface_installed)
  {
    if (!bp.checkSize(MasterBoardData_V3_0__1::EURO_SIZE))
      return false;

    bp.parse(euromap_input_bits);
    bp.parse(euromap_output_bits);
    bp.parse(euromap_voltage);
    bp.parse(euromap_current);
  }

  bp.consume(sizeof(uint32_t));

  return true;
}

bool MasterBoardData_V3_2::parseWith(BinParser& bp)
{
  if (!bp.checkSize<MasterBoardData_V3_2>())
    return false;

  MasterBoardData_V3_0__1::parseWith(bp);

  bp.parse(operational_mode_selector_input);
  bp.parse(three_position_enabling_device_input);

  return true;
}

bool MasterBoardData_V1_X::consumeWith(URStatePacketConsumer& consumer)
{
  return consumer.consume(*this);
}
bool MasterBoardData_V3_0__1::consumeWith(URStatePacketConsumer& consumer)
{
  return consumer.consume(*this);
}
bool MasterBoardData_V3_2::consumeWith(URStatePacketConsumer& consumer)
{
  return consumer.consume(*this);
}