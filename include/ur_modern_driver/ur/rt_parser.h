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