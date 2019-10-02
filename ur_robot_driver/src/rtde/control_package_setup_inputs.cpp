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

#include "ur_robot_driver/rtde/control_package_setup_inputs.h"

namespace ur_driver
{
namespace rtde_interface
{
bool ControlPackageSetupInputs::parseWith(comm::BinParser& bp)
{
  bp.parse(input_recipe_id_);
  bp.parseRemainder(variable_types_);

  return true;
}
std::string ControlPackageSetupInputs::toString() const
{
  std::stringstream ss;
  ss << "input recipe id: " << static_cast<int>(input_recipe_id_) << std::endl;
  ss << "variable types: " << variable_types_;

  return ss.str();
}

size_t ControlPackageSetupInputsRequest::generateSerializedRequest(uint8_t* buffer,
                                                                   std::vector<std::string> variable_names)
{
  if (variable_names.size() == 0)
  {
    return 0;
  }
  std::string variables;
  for (const auto& piece : variable_names)
    variables += (piece + ",");
  variables.pop_back();
  uint16_t payload_size = variables.size();

  size_t size = 0;
  size += PackageHeader::serializeHeader(buffer, PACKAGE_TYPE, payload_size);
  size += comm::PackageSerializer::serialize(buffer + size, variables);

  return size;
}
}  // namespace rtde_interface
}  // namespace ur_driver
