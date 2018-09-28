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