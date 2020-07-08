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
 * \date    2019-04-08
 *
 */
//----------------------------------------------------------------------

#include "ur_robot_driver/log.h"
#include "ur_robot_driver/primary/robot_message/version_message.h"
#include "ur_robot_driver/primary/abstract_primary_consumer.h"

namespace ur_driver
{
namespace primary_interface
{
bool VersionMessage::parseWith(comm::BinParser& bp)
{
  bp.parse(project_name_length_);
  bp.parse(project_name_, project_name_length_);
  bp.parse(major_version_);
  bp.parse(minor_version_);
  bp.parse(svn_version_);
  bp.parse(build_number_);
  bp.parseRemainder(build_date_);

  return true;  // not really possible to check dynamic size packets
}

bool VersionMessage ::consumeWith(AbstractPrimaryConsumer& consumer)
{
  return consumer.consume(*this);
}

std::string VersionMessage::toString() const
{
  std::stringstream ss;
  ss << "project name: " << project_name_ << std::endl;
  ss << "version: " << unsigned(major_version_) << "." << unsigned(minor_version_) << "." << svn_version_ << std::endl;
  ss << "build date: " << build_date_;

  return ss.str();
}
}  // namespace primary_interface
}  // namespace ur_driver
