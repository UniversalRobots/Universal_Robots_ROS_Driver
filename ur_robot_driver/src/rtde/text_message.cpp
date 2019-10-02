// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Tristan Schnell schnell@fzi.de
 * \date    2019-04-09
 *
 */
//----------------------------------------------------------------------

#include "ur_robot_driver/rtde/text_message.h"

namespace ur_driver
{
namespace rtde_interface
{
bool TextMessage::parseWith(comm::BinParser& bp)
{
  if (protocol_version_ == 2)
  {
    bp.parse(message_length_);
    bp.parse(message_, message_length_);
    bp.parse(source_length_);
    bp.parse(source_, source_length_);
    bp.parse(warning_level_);
  }
  else if (protocol_version_ == 1)
  {
    bp.parse(message_type_);
    bp.parseRemainder(message_);
  }

  return true;
}
std::string TextMessage::toString() const
{
  std::stringstream ss;
  ss << "message: " << message_ << std::endl;
  ss << "source: " << source_ << std::endl;
  ss << "warning level: " << static_cast<int>(warning_level_);

  return ss.str();
}
}  // namespace rtde_interface
}  // namespace ur_driver
