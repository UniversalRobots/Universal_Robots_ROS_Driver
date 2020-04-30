// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik (ur_robot_driver)
// Copyright 2017, 2018 Simon Rasmussen (refactor)
//
// Copyright 2015, 2016 Thomas Timm Andersen (original version)
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
 * \author  Felix Exner exner@fzi.de
 * \date    2020-04-30
 *
 */
//----------------------------------------------------------------------

#include "ur_robot_driver/log.h"
#include "ur_robot_driver/primary/robot_message/error_code_message.h"

namespace ur_driver
{
namespace primary_interface
{
bool ErrorCodeMessage::parseWith(comm::BinParser& bp)
{
  bp.parse(message_code_);
  bp.parse(message_argument_);
  bp.parse(report_level_);
  bp.parse(data_type_);
  bp.parse(data_);
  bp.parseRemainder(text_);

  return true;  // not really possible to check dynamic size packets
}

std::string ErrorCodeMessage::toString() const
{
  std::stringstream ss;
  ss << "Message code: " << message_code_ << std::endl;
  ss << "Message argument: " << message_argument_ << std::endl;
  ss << "Report level: " << report_level_ << std::endl;
  ss << "Datatype: " << static_cast<int>(data_type_) << std::endl;
  ss << "Data: " << data_ << std::endl;
  ss << "Text: " << text_;
  return ss.str();
}
}  // namespace primary_interface
}  // namespace ur_driver
