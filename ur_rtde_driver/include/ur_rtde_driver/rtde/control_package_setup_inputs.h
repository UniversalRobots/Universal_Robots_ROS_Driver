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

#ifndef UR_RTDE_DRIVER_CONTROL_PACKAGE_SETUP_INPUTS_H_INCLUDED
#define UR_RTDE_DRIVER_CONTROL_PACKAGE_SETUP_INPUTS_H_INCLUDED

#include "ur_rtde_driver/rtde/rtde_package.h"

namespace ur_driver
{
namespace rtde_interface
{
class ControlPackageSetupInputs : public RTDEPackage
{
public:
  ControlPackageSetupInputs() : RTDEPackage(PackageType::RTDE_CONTROL_PACKAGE_SETUP_INPUTS)
  {
  }
  virtual ~ControlPackageSetupInputs() = default;

  virtual bool parseWith(comm::BinParser& bp);
  virtual std::string toString() const;

  uint8_t input_recipe_id_;
  std::string variable_types_;
};

class ControlPackageSetupInputsRequest : public RTDEPackage
{
public:
  ControlPackageSetupInputsRequest() : RTDEPackage(PackageType::RTDE_CONTROL_PACKAGE_SETUP_INPUTS)
  {
  }
  virtual ~ControlPackageSetupInputsRequest() = default;

  static size_t generateSerializedRequest(uint8_t* buffer, std::vector<std::string> variable_names);

  std::string variable_names_;

private:
  static const PackageType PACKAGE_TYPE = PackageType::RTDE_CONTROL_PACKAGE_SETUP_INPUTS;
};

}  // namespace rtde_interface
}  // namespace ur_driver

#endif  // UR_RTDE_DRIVER_CONTROL_PACKAGE_SETUP_INPUTS_H_INCLUDED
