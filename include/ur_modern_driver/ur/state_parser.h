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
#include <vector>
#include "ur_modern_driver/bin_parser.h"
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/pipeline.h"
#include "ur_modern_driver/ur/master_board.h"
#include "ur_modern_driver/ur/parser.h"
#include "ur_modern_driver/ur/robot_mode.h"
#include "ur_modern_driver/ur/state.h"

template <typename RMD, typename MBD>
class URStateParser : public URParser<StatePacket>
{
private:
  StatePacket* from_type(package_type type)
  {
    switch (type)
    {
      case package_type::ROBOT_MODE_DATA:
        return new RMD;
      case package_type::MASTERBOARD_DATA:
        return new MBD;
      default:
        return nullptr;
    }
  }

public:
  bool parse(BinParser& bp, std::vector<std::unique_ptr<StatePacket>>& results)
  {
    int32_t packet_size;
    message_type type;
    bp.parse(packet_size);
    bp.parse(type);

    if (type != message_type::ROBOT_STATE)
    {
      // quietly ignore the intial version message
      if (type != message_type::ROBOT_MESSAGE)
      {
        LOG_WARN("Invalid state message type recieved: %u", static_cast<uint8_t>(type));
      }

      bp.consume();
      return true;
    }

    while (!bp.empty())
    {
      if (!bp.checkSize(sizeof(uint32_t)))
      {
        LOG_ERROR("Failed to read sub-package length, there's likely a parsing error");
        return false;
      }
      uint32_t sub_size = bp.peek<uint32_t>();
      if (!bp.checkSize(static_cast<size_t>(sub_size)))
      {
        LOG_WARN("Invalid sub-package size of %" PRIu32 " received!", sub_size);
        return false;
      }

      // deconstruction of a sub parser will increment the position of the parent parser
      BinParser sbp(bp, sub_size);
      sbp.consume(sizeof(sub_size));
      package_type type;
      sbp.parse(type);

      std::unique_ptr<StatePacket> packet(from_type(type));

      if (packet == nullptr)
      {
        sbp.consume();
        continue;
      }

      if (!packet->parseWith(sbp))
      {
        LOG_ERROR("Sub-package parsing of type %d failed!", static_cast<int>(type));
        return false;
      }

      results.push_back(std::move(packet));

      if (!sbp.empty())
      {
        LOG_ERROR("Sub-package of type %d was not parsed completely!", static_cast<int>(type));
        sbp.debug();
        return false;
      }
    }

    return true;
  }
};

typedef URStateParser<RobotModeData_V1_X, MasterBoardData_V1_X> URStateParser_V1_X;
typedef URStateParser<RobotModeData_V3_0__1, MasterBoardData_V3_0__1> URStateParser_V3_0__1;
typedef URStateParser<RobotModeData_V3_2, MasterBoardData_V3_2> URStateParser_V3_2;
typedef URStateParser<RobotModeData_V3_5, MasterBoardData_V3_2> URStateParser_V3_5;
