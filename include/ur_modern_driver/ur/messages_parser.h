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
#include "ur_modern_driver/ur/messages.h"
#include "ur_modern_driver/ur/parser.h"

class URMessageParser : public URParser<MessagePacket>
{
public:
  bool parse(BinParser& bp, std::vector<unique_ptr<MessagePacket>>& results)
  {
    int32_t packet_size;
    message_type type;
    bp.parse(packet_size);
    bp.parse(type);

    if (type != message_type::ROBOT_MESSAGE)
    {
      LOG_WARN("Invalid message type recieved: %u", static_cast<uint8_t>(type));
      return false;
    }

    uint64_t timestamp;
    uint8_t source;
    robot_message_type message_type;

    bp.parse(timestamp);
    bp.parse(source);
    bp.parse(message_type);

    std::unique_ptr<MessagePacket> result;
    bool parsed = false;

    switch (message_type)
    {
      case robot_message_type::ROBOT_MESSAGE_VERSION:
      {
        VersionMessage* vm = new VersionMessage(timestamp, source);
        parsed = vm->parseWith(bp);
        result.reset(vm);
        break;
      }

      default:
        return false;
    }

    results.push_back(std::move(result));
    return parsed;
  }
};