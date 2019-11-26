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
 * \author  Felix Exner exner@fzi.de
 * \date    2019-04-09
 *
 */
//----------------------------------------------------------------------

#include "ur_robot_driver/primary/primary_package.h"

namespace ur_driver
{
namespace primary_interface
{
bool PrimaryPackage::parseWith(comm::BinParser& bp)
{
  bp.rawData(buffer_, buffer_length_);
  return true;
}

std::string PrimaryPackage::toString() const
{
  std::stringstream ss;
  ss << "Raw byte stream: ";
  for (size_t i = 0; i < buffer_length_; ++i)
  {
    uint8_t* buf = buffer_.get();
    ss << std::hex << static_cast<int>(buf[i]) << " ";
  }
  ss << std::endl;
  return ss.str();
}

}  // namespace primary_interface
}  // namespace ur_driver
