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
 * \date    2019-04-17
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CONTROLLERS_SPEED_SCALING_INTERFACE_H_INCLUDED
#define UR_CONTROLLERS_SPEED_SCALING_INTERFACE_H_INCLUDED

#include <hardware_interface/internal/hardware_resource_manager.h>

namespace ur_controllers
{
class SpeedScalingHandle
{
public:
  SpeedScalingHandle() : name_(""), scaling_factor_(0){};
  SpeedScalingHandle(const std::string& name, const double* scaling_factor)
    : name_(name), scaling_factor_(scaling_factor){};
  virtual ~SpeedScalingHandle() = default;

  std::string getName() const
  {
    return name_;
  }

  const double* getScalingFactor() const
  {
    return scaling_factor_;
  }

private:
  std::string name_;
  const double* scaling_factor_;
};
/** \brief Hardware interface to support reading the speed scaling factor. */
class SpeedScalingInterface : public hardware_interface::HardwareResourceManager<SpeedScalingHandle>
{
};
}  // namespace ur_controllers

#endif  // ifndef UR_CONTROLLERS_SPEED_SCALING_INTERFACE_H_INCLUDED
