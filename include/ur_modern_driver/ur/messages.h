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
#include <cstddef>
#include "ur_modern_driver/bin_parser.h"
#include "ur_modern_driver/pipeline.h"

enum class robot_message_type : uint8_t
{
  ROBOT_MESSAGE_TEXT = 0,
  ROBOT_MESSAGE_PROGRAM_LABEL = 1,
  PROGRAM_STATE_MESSAGE_VARIABLE_UPDATE = 2,
  ROBOT_MESSAGE_VERSION = 3,
  ROBOT_MESSAGE_SAFETY_MODE = 5,
  ROBOT_MESSAGE_ERROR_CODE = 6,
  ROBOT_MESSAGE_KEY = 7,
  ROBOT_MESSAGE_REQUEST_VALUE = 9,
  ROBOT_MESSAGE_RUNTIME_EXCEPTION = 10
};

class URMessagePacketConsumer;

class MessagePacket
{
public:
  MessagePacket(uint64_t timestamp, uint8_t source) : timestamp(timestamp), source(source)
  {
  }
  virtual bool parseWith(BinParser& bp) = 0;
  virtual bool consumeWith(URMessagePacketConsumer& consumer) = 0;

  uint64_t timestamp;
  uint8_t source;
};

class VersionMessage : public MessagePacket
{
public:
  VersionMessage(uint64_t timestamp, uint8_t source) : MessagePacket(timestamp, source)
  {
  }

  virtual bool parseWith(BinParser& bp);
  virtual bool consumeWith(URMessagePacketConsumer& consumer);

  std::string project_name;
  uint8_t major_version;
  uint8_t minor_version;
  int32_t svn_version;
  std::string build_date;
};