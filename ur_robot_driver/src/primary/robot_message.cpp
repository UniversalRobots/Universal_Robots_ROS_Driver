// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik (ur_robot_driver)
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
 * \date    2019-04-09
 *
 */
//----------------------------------------------------------------------
#include "ur_robot_driver/primary/robot_message.h"
#include "ur_robot_driver/primary/abstract_primary_consumer.h"

namespace ur_driver
{
namespace primary_interface
{
bool RobotMessage::parseWith(comm::BinParser& bp)
{
  return true;
}

bool RobotMessage::consumeWith(AbstractPrimaryConsumer& consumer)
{
  return consumer.consume(*this);
}

std::string RobotMessage::toString() const
{
  std::stringstream ss;
  ss << "timestamp: " << timestamp_ << std::endl;
  ss << "source: " << static_cast<int>(source_) << std::endl;
  ss << "message_type: " << static_cast<int>(message_type_) << std::endl;

  return ss.str();
}

}  // namespace primary_interface
}  // namespace ur_driver
