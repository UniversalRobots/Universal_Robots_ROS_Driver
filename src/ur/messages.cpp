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

#include "ur_modern_driver/ur/messages.h"
#include "ur_modern_driver/ur/consumer.h"

bool VersionMessage::parseWith(BinParser& bp)
{
  bp.parse(project_name);
  bp.parse(major_version);
  bp.parse(minor_version);
  bp.parse(svn_version);
  bp.consume(sizeof(uint32_t));  // undocumented field??
  bp.parse_remainder(build_date);

  return true;  // not really possible to check dynamic size packets
}

bool VersionMessage::consumeWith(URMessagePacketConsumer& consumer)
{
  return consumer.consume(*this);
}