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

#pragma once

#include <inttypes.h>
#include <bitset>
#include <cstddef>
#include "ur_modern_driver/bin_parser.h"
#include "ur_modern_driver/types.h"
#include "ur_modern_driver/ur/state.h"

class SharedMasterBoardData
{
public:
  virtual bool parseWith(BinParser& bp);

  int8_t analog_input_range0;
  int8_t analog_input_range1;
  double analog_input0;
  double analog_input1;
  int8_t analog_output_domain0;
  int8_t analog_output_domain1;
  double analog_output0;
  double analog_output1;
  float master_board_temperature;
  float robot_voltage_48V;
  float robot_current;
  float master_IO_current;

  bool euromap67_interface_installed;

  // euromap fields are dynamic so don't include in SIZE
  int32_t euromap_input_bits;
  int32_t euromap_output_bits;

  static const size_t SIZE = sizeof(double) * 4 + sizeof(int8_t) * 4 + sizeof(float) * 4 + sizeof(uint8_t);

  static const size_t EURO_SIZE = sizeof(int32_t) * 2;
};

class MasterBoardData_V1_X : public SharedMasterBoardData, public StatePacket
{
public:
  virtual bool parseWith(BinParser& bp);
  virtual bool consumeWith(URStatePacketConsumer& consumer);

  std::bitset<10> digital_input_bits;
  std::bitset<10> digital_output_bits;

  uint8_t master_safety_state;
  bool master_on_off_state;

  // euromap fields are dynamic so don't include in SIZE
  int16_t euromap_voltage;
  int16_t euromap_current;

  static const size_t SIZE = SharedMasterBoardData::SIZE + sizeof(int16_t) * 2 + sizeof(uint8_t) * 2;

  static const size_t EURO_SIZE = SharedMasterBoardData::EURO_SIZE + sizeof(int16_t) * 2;
};

class MasterBoardData_V3_0__1 : public SharedMasterBoardData, public StatePacket
{
public:
  virtual bool parseWith(BinParser& bp);
  virtual bool consumeWith(URStatePacketConsumer& consumer);

  std::bitset<18> digital_input_bits;
  std::bitset<18> digital_output_bits;

  uint8_t safety_mode;
  bool in_reduced_mode;

  // euromap fields are dynamic so don't include in SIZE
  float euromap_voltage;
  float euromap_current;

  static const size_t SIZE = SharedMasterBoardData::SIZE + sizeof(int32_t) * 2 + sizeof(uint8_t) * 2 +
                             sizeof(uint32_t);  // UR internal field we skip

  static const size_t EURO_SIZE = SharedMasterBoardData::EURO_SIZE + sizeof(float) * 2;
};

class MasterBoardData_V3_2 : public MasterBoardData_V3_0__1
{
public:
  virtual bool parseWith(BinParser& bp);
  virtual bool consumeWith(URStatePacketConsumer& consumer);

  uint8_t operational_mode_selector_input;
  uint8_t three_position_enabling_device_input;

  static const size_t SIZE = MasterBoardData_V3_0__1::SIZE + sizeof(uint8_t) * 2;
};
