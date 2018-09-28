#pragma once
#include <vector>
#include "ur_modern_driver/bin_parser.h"
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/pipeline.h"
#include "ur_modern_driver/ur/parser.h"
#include "ur_modern_driver/ur/rt_state.h"

template <typename T>
class URRTStateParser : public URParser<RTPacket>
{
public:
  bool parse(BinParser& bp, std::vector<std::unique_ptr<RTPacket>>& results)
  {
    int32_t packet_size = bp.peek<int32_t>();

    if (!bp.checkSize(packet_size))
    {
      LOG_ERROR("Buffer len shorter than expected packet length");
      return false;
    }

    bp.parse(packet_size);  // consumes the peeked data

    std::unique_ptr<RTPacket> packet(new T);
    if (!packet->parseWith(bp))
      return false;

    results.push_back(std::move(packet));

    return true;
  }
};

typedef URRTStateParser<RTState_V1_6__7> URRTStateParser_V1_6__7;
typedef URRTStateParser<RTState_V1_8> URRTStateParser_V1_8;
typedef URRTStateParser<RTState_V3_0__1> URRTStateParser_V3_0__1;
typedef URRTStateParser<RTState_V3_2__3> URRTStateParser_V3_2__3;