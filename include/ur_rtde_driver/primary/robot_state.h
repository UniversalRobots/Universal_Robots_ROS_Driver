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
 * \author  Lea Steffen steffen@fzi.de
 * \date    2019-04-01
 *
 */
//----------------------------------------------------------------------

#ifndef UR_RTDE_DRIVER_ROBOT_STATE_H_INCLUDED
#define UR_RTDE_DRIVER_ROBOT_STATE_H_INCLUDED

#include <mutex>
#include <condition_variable>

#include "ur_rtde_driver/primary/primary_package.h"
#include "ur_rtde_driver/primary/primary_header.h"

namespace ur_driver
{
namespace primary_interface
{
/*!
 * \brief Abstract class for a RobotState msg. This will never be instanciated, but the underlying
 * data packages will be used directly.
 */
class RobotState : public PrimaryPackage
{
public:
  RobotState() = default;
  virtual ~RobotState() = default;

private:
};

}  // namespace primary_interface
}  // namespace ur_driver

#endif /* UR_RTDE_DRIVER_ROBOT_STATE_H_INCLUDED */
