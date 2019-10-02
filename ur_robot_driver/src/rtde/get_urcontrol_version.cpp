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

#include "ur_robot_driver/rtde/get_urcontrol_version.h"

namespace ur_driver
{
namespace rtde_interface
{
bool GetUrcontrolVersion::parseWith(comm::BinParser& bp)
{
  bp.parse(version_information_.major);
  bp.parse(version_information_.minor);
  bp.parse(version_information_.bugfix);
  bp.parse(version_information_.build);

  return true;
}
std::string GetUrcontrolVersion::toString() const
{
  std::stringstream ss;
  ss << "version: " << major_ << ".";
  ss << minor_ << "." << bugfix_ << "." << build_;

  return ss.str();
}

size_t GetUrcontrolVersionRequest::generateSerializedRequest(uint8_t* buffer)
{
  return PackageHeader::serializeHeader(buffer, PackageType::RTDE_GET_URCONTROL_VERSION, PAYLOAD_SIZE);
}
}  // namespace rtde_interface
}  // namespace ur_driver
