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
      LOG_WARN("Invalid message type recieved: %u", static_cast<uint8_t>(type));
      return false;
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
        LOG_INFO("Skipping sub-packet of type %d", type);
        continue;
      }

      if (!packet->parseWith(sbp))
      {
        LOG_ERROR("Sub-package parsing of type %d failed!", type);
        return false;
      }

      results.push_back(std::move(packet));

      if (!sbp.empty())
      {
        LOG_ERROR("Sub-package was not parsed completely!");
        return false;
      }
    }

    return true;
  }
};

typedef URStateParser<RobotModeData_V1_X, MasterBoardData_V1_X> URStateParser_V1_X;
typedef URStateParser<RobotModeData_V3_0__1, MasterBoardData_V3_0__1> URStateParser_V3_0__1;
typedef URStateParser<RobotModeData_V3_2, MasterBoardData_V3_2> URStateParser_V3_2;